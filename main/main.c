#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_continuous.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Tag for logging
static const char *TAG = "ESP-SCOPE";

// WiFi configuration from Kconfig
/* See untracked wifi-credentials.h */
#ifndef ESP_WIFI_SSID
#define ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#endif

#define ESP_MAXIMUM_RETRY 5

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

// Embedded index.html
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t index_js_start[] asm("_binary_index_js_start");
extern const uint8_t index_js_end[] asm("_binary_index_js_end");

// Forward declarations
static void wifi_init_sta(void);
static void start_webserver(void);

// ADC Configuration
#define ADC_UNIT ADC_UNIT_1
#define _ADC_UNIT_STR(unit) #unit
#define ADC_UNIT_STR(unit) _ADC_UNIT_STR(unit)
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_BIT_WIDTH ADC_BITWIDTH_12

#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type2.data)

#define ADC_READ_LEN 512
#define ADC_MAX_STORE_BUF_SIZE 1024

static adc_continuous_handle_t adc_handle = NULL;
static TaskHandle_t s_task_handle;
static RingbufHandle_t s_ringbuf_handle;

// Single client support for simplicity, or use a list for multiple
static int s_ws_client_fd = -1;

// Global configuration state
static volatile bool s_reconfig_needed = false;
static uint32_t s_sample_rate = 20000;
static adc_atten_t s_atten = ADC_ATTEN_DB_12;
static adc_bitwidth_t s_bit_width = ADC_BIT_WIDTH;
static uint16_t s_test_hz = 100;

// Forward declarations
static void wifi_init_sta(void);
static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num,
                                adc_continuous_handle_t *out_handle);

/*
 * Task to read from ADC Continuous driver
 */
static void adc_read_task(void *arg) {
  esp_err_t ret;
  uint32_t ret_num = 0;
  uint8_t result[ADC_READ_LEN] = {0};
  memset(result, 0xcc, ADC_READ_LEN);

  s_task_handle = xTaskGetCurrentTaskHandle();

  // ADC Init (Moved from app_main)
  // TODO: Make this configurable or find a good default pin.
  // For ESP32C6 ADC1 Channel 0 is usually GPIO 0. Let's use Channel 0 for now.
  adc_channel_t channel[1] = {ADC_CHANNEL_0};
  continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t),
                      &adc_handle);
  ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

  while (1) {
    if (s_reconfig_needed) {
      ESP_LOGI(TAG, "Reconfiguring ADC...");
      if (adc_handle) {
        ESP_LOGI(TAG, "Stopping ADC...");
        ret = adc_continuous_stop(adc_handle);
        if (ret != ESP_OK) {
          ESP_LOGE(TAG, "adc_continuous_stop failed: %s", esp_err_to_name(ret));
        }
        ESP_LOGI(TAG, "Deinitializing ADC...");
        ret = adc_continuous_deinit(adc_handle);
        if (ret != ESP_OK) {
          ESP_LOGE(TAG, "adc_continuous_deinit failed: %s",
                   esp_err_to_name(ret));
        }
        adc_handle = NULL;
      }

      // Small delay to ensure hardware state clears
      vTaskDelay(pdMS_TO_TICKS(20));

      // Update global defaults for next init
      // Note: In a robust app, we should pass these to init function
      // For now, we rely on the global s_sample_rate etc being read by init
      adc_channel_t channel[1] = {ADC_CHANNEL_0};
      continuous_adc_init(channel, 1, &adc_handle);

      ESP_LOGI(TAG, "Starting ADC...");
      ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
      ESP_LOGI(TAG, "ADC Reconfigured and Restarted");
      s_reconfig_needed = false;
    }

    ret = adc_continuous_read(adc_handle, result, ADC_READ_LEN, &ret_num, 0);
    if (ret == ESP_OK) {
      // ESP_LOGI(TAG, "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
      for (int i = 0; i < ret_num; i += sizeof(adc_digi_output_data_t)) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
        uint32_t chan_num = ADC_GET_CHANNEL(p);
        uint32_t data = ADC_GET_DATA(p);
        /* Check the channel number validation, the data is invalid if the
         * channel num exceed the maximum channel */
        if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT)) {
          // ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIu32,
          // ADC_UNIT_STR(ADC_UNIT), chan_num, data);
          // ADC_UNIT_STR(ADC_UNIT), chan_num, data);
          // Send data to RingBuffer
          // We send raw value or processed? Sending raw 12-bit value is
          // smaller. Let's send the 32-bit `adc_digi_output_data_t` itself or
          // just the value. Sending just value (uint16_t) saves space.
          uint16_t val = (uint16_t)data;
          xRingbufferSend(s_ringbuf_handle, &val, sizeof(val), 0);

        } else {
          ESP_LOGW(TAG, "Invalid data [%" PRIu32 "_%" PRIu32 "]", chan_num,
                   data);
        }
      }
      /**
       * Because printing is slow, potentially delay here or yield if we fill
       * buffers too fast. But for continuous mode we usually just want to drain
       * the buffer. For now, just a small yield to prevent watchdog if we spin
       * tight.
       */
      vTaskDelay(1);
    } else if (ret == ESP_ERR_TIMEOUT) {
      // We try to read `ADC_READ_LEN` until API returns timeout, which means
      // there's no available data
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num,
                                adc_continuous_handle_t *out_handle) {
  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = 1024,
      .conv_frame_size = ADC_READ_LEN,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, out_handle));

  adc_continuous_config_t dig_cfg = {
      .sample_freq_hz = s_sample_rate,
      .conv_mode = ADC_CONV_MODE,
      .format = ADC_OUTPUT_TYPE,
  };

  adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
  dig_cfg.pattern_num = channel_num;

  for (int i = 0; i < channel_num; i++) {
    adc_pattern[i].atten = s_atten;
    adc_pattern[i].channel = channel[i] & 0x7;
    adc_pattern[i].unit = ADC_UNIT;
    adc_pattern[i].bit_width = s_bit_width;

    ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
    ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i,
             adc_pattern[i].channel);
    ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
  }
  dig_cfg.adc_pattern = adc_pattern;
  ESP_ERROR_CHECK(adc_continuous_config(*out_handle, &dig_cfg));
}
static bool ledc_inited = false;
static void start_test_signal(uint32_t hz) {
  if (ledc_inited) {
    ESP_LOGI(TAG, "De-init test signal");
    gpio_reset_pin(GPIO_NUM_1);

    // Stop the PWM signal on channel 0
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);

    // Reset the timer configuration
    ledc_timer_rst(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);

    // (Optional) Uninstall fade functionality if used
    ledc_fade_func_uninstall();
  }
  ESP_LOGI(TAG, "Starting test signal at %u Hz", hz);
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_14_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = hz,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_channel_config_t ledc_channel = {
    .gpio_num = 1, // GPIO data pin 1
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 1 << (ledc_timer.duty_resolution - 1), // 512, // 50% duty cycle (1024 / 2 for 10-bit resolution)
    .hpoint = 0
  };

  // Initialize the PWM
  ledc_timer_config(&ledc_timer);
  ledc_channel_config(&ledc_channel);
  // Start the PWM signal
  ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, ledc_channel.duty); // 50% duty cycle
  ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
  ledc_inited = true;
}

void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  /* Board-specific WiFi init (if any) */
    // Seeed XIAO ESP32C6: Configure GPIO 3 and GPIO 14 as outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 3) | (1ULL << 14),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Set GPIO 3 and GPIO 14 to low
    gpio_set_level(3, 0);
    gpio_set_level(14, 0);
  /* end board-specific WiFi init (if any) */

  wifi_init_sta();

  start_test_signal(100);

  // Create RingBuffer (e.g. 8KB)
  s_ringbuf_handle = xRingbufferCreate(8 * 1024, RINGBUF_TYPE_BYTEBUF);
  if (s_ringbuf_handle == NULL) {
    ESP_LOGE(TAG, "Failed to create ring buffer");
  }

  xTaskCreate(adc_read_task, "adc_read_task", 4 * 1024, NULL, 5, NULL);

  // Wait for WiFi connection
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s", ESP_WIFI_SSID);
    start_webserver();
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s", ESP_WIFI_SSID);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t* netif = esp_netif_create_default_wifi_sta();
  esp_netif_set_hostname(netif, "esp-scope");  // Set hostname for the STA interface

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = ESP_WIFI_SSID,
              .password = ESP_WIFI_PASS,
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");
}

/*
 * Web Server Configuration
 */
static httpd_handle_t s_server = NULL;

/*
 * WebSocket Handler
 */
static esp_err_t ws_handler(httpd_req_t *req) {
  if (req->method == HTTP_GET) {
    // Handshake
    return ESP_OK;
  }

  httpd_ws_frame_t ws_pkt;
  uint8_t *buf = NULL;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;

  // Get frame len
  esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK)
    return ret;

  if (ws_pkt.len) {
    buf = calloc(1, ws_pkt.len + 1);
    if (buf == NULL)
      return ESP_ERR_NO_MEM;
    ws_pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
      free(buf);
      return ret;
    }

    // Check for "hello"
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char *)ws_pkt.payload, "hello") == 0) {
      ESP_LOGI(TAG, "New WS client connected, fd=%d", httpd_req_to_sockfd(req));
      s_ws_client_fd = httpd_req_to_sockfd(req);
    }
    free(buf);
  }
  return ESP_OK;
}

/*
 * Control Params Handler (POST /params)
 */
static esp_err_t params_handler(httpd_req_t *req) {
  char buf[256];
  int ret, remaining = req->content_len;
  if (remaining >= sizeof(buf)) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  ret = httpd_req_recv(req, buf, remaining);
  if (ret <= 0)
    return ESP_FAIL;
  buf[ret] = '\0';

  cJSON *root = cJSON_Parse(buf);
  if (root) {
    cJSON *sample_rate = cJSON_GetObjectItem(root, "sample_rate");
    if (sample_rate && s_sample_rate != sample_rate->valueint) {
      s_reconfig_needed = true;
      s_sample_rate = sample_rate->valueint;
    }
    cJSON *atten = cJSON_GetObjectItem(root, "atten");
    if (atten && s_atten != (adc_atten_t)atten->valueint) {
      s_reconfig_needed = true;
      s_atten = (adc_atten_t)atten->valueint;
    }
    cJSON *bit_width = cJSON_GetObjectItem(root, "bit_width");
    if (bit_width && s_bit_width != (adc_bitwidth_t)bit_width->valueint) {
      s_reconfig_needed = true;
      s_bit_width = (adc_bitwidth_t)bit_width->valueint;
    }
    cJSON *test_hz = cJSON_GetObjectItem(root, "test_hz");
    if (test_hz) {
      if (s_test_hz != (adc_bitwidth_t)test_hz->valueint) {
        s_test_hz = (adc_bitwidth_t)test_hz->valueint;
        start_test_signal(s_test_hz);
      }
    }

    ESP_LOGI(TAG, "Config Request: Rate=%lu, Atten=%d, Width=%d, TestHz=%u, s_reconfig_needed=%d", s_sample_rate,
             s_atten, s_bit_width, s_test_hz,
            s_reconfig_needed);

    cJSON_Delete(root);
  }

  httpd_resp_send(req, "OK", 2);
  return ESP_OK;
}

static const httpd_uri_t uri_ws = {.uri = "/signal",
                                   .method = HTTP_GET,
                                   .handler = ws_handler,
                                   .user_ctx = NULL,
                                   .is_websocket = true};

static const httpd_uri_t uri_params = {.uri = "/params",
                                       .method = HTTP_POST,
                                       .handler = params_handler,
                                       .user_ctx = NULL};

/*
 * Task to pull from RingBuffer and send to WS
 */
static void ws_sender_task(void *arg) {
  size_t item_size;

  while (1) {
    // Receive item from ring buffer
    // usage: void *xRingbufferReceive(RingbufHandle_t xRingbuffer, size_t
    // *pxItemSize, TickType_t xTicksToWait); But we sent `val` using
    // `xRingbufferSend`. RingBuffer bytebuf mode sends stream. We used
    // send/recv. Wait, we used `xRingbufferSend`. In bytebuf mode, receive
    // returns a pointer to the buffer.

    uint16_t *data = (uint16_t *)xRingbufferReceive(
        s_ringbuf_handle, &item_size, pdMS_TO_TICKS(10));

    if (data != NULL) {
      if (s_ws_client_fd != -1) {
        // Create WebSocket frame for binary data
        httpd_ws_frame_t ws_frame = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_BINARY,
            .payload = (uint8_t *)data,
            .len = item_size
        };

        // Send binary frame
        esp_err_t ret = httpd_ws_send_frame_async(s_server, s_ws_client_fd, &ws_frame);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "WS Send failed, invalidating FD");
            s_ws_client_fd = -1;
        }
      }
      vRingbufferReturnItem(s_ringbuf_handle, (void *)data);
    } else {
      // No data
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}

/* Handler for serving index.html */
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
  uint32_t len = index_html_end - index_html_start;
  // Workaround for some build systems adding null byte
  while (len && index_js_start[len-1] == 0) len--;
  httpd_resp_send(req, (const char *)index_html_start, len);
  return ESP_OK;
}

static const httpd_uri_t uri_index = {
    .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL};

/* Handler for serving index.js */
static esp_err_t index_js_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/javascript");
  httpd_resp_set_hdr(req, "Content-Type", "text/javascript; charset=utf-8");
  uint32_t len = index_js_end - index_js_start;
  // Workaround for some build systems adding null byte
  while (len && index_js_start[len-1] == 0) len--;
  httpd_resp_send(req, (const char *)index_js_start, len);
  return ESP_OK;
}

static const httpd_uri_t uri_index_js = {
    .uri = "/index.js", .method = HTTP_GET, .handler = index_js_handler, .user_ctx = NULL};

static void start_webserver(void) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;

  ESP_LOGI(TAG, "Starting webserver on port: '%d'", config.server_port);
  if (httpd_start(&s_server, &config) == ESP_OK) {
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(s_server, &uri_index);
    httpd_register_uri_handler(s_server, &uri_index_js);
    httpd_register_uri_handler(s_server, &uri_ws);
    httpd_register_uri_handler(s_server, &uri_params);

    // Start sender task
    xTaskCreate(ws_sender_task, "ws_sender", 4096, NULL, 5, NULL);

  } else {
    ESP_LOGI(TAG, "Error starting server!");
  }
}
