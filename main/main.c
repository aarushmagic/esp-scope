#include <inttypes.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include "wifi_manager.h"

// DAC Support Checks
#if SOC_DAC_SUPPORTED
// Try to use legacy header for simplicity if available, or COS (New)
// For IDF 5.x, dac_oneshot is preferred.
#include "driver/dac_cosine.h"
#include "driver/dac_oneshot.h"
#include "mdns.h"


#endif

// Tag for logging
static const char *TAG = "ESP-SCOPE";

// Embedded index.html
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t index_js_start[] asm("_binary_index_js_start");
extern const uint8_t index_js_end[] asm("_binary_index_js_end");

// Forward declarations
static void start_webserver(void);

// ADC Configuration
#define ADC_UNIT ADC_UNIT_1
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH ADC_BITWIDTH_12

#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_DATA(p_data) ((p_data)->type1.data)

// Calibration Handle
static adc_cali_handle_t s_adc_cali_handle = NULL;
static int s_cali_v_max_mv = 0; // Cached max voltage (for 4095 raw)

static void init_adc_calibration() {
  ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
  adc_cali_line_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT,
      .atten = ADC_ATTEN,
      .bitwidth = ADC_BIT_WIDTH,
  };
  esp_err_t ret =
      adc_cali_create_scheme_line_fitting(&cali_config, &s_adc_cali_handle);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Calibration Success");

    // Calculate Vmax (Theoretical voltage at 4095 raw)
    // Note: bitwidth 12 means max raw is 4095.
    int voltage = 0;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(s_adc_cali_handle, 4095, &voltage));
    s_cali_v_max_mv = voltage;
    ESP_LOGI(TAG, "Calibrated Vmax @ 4095 = %d mV", voltage);

  } else if (ret == ESP_ERR_NOT_SUPPORTED || !ret) {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else {
    ESP_LOGE(TAG, "Invalid arg or no memory");
  }
}

// Function Generator Definitions
typedef enum {
  FUNC_OFF = 0,
  FUNC_SQUARE = 1,
  FUNC_SINE = 2,
  FUNC_TRIANGLE = 3
} func_type_t;

// Global Function Gen State
static func_type_t s_func_type = FUNC_SQUARE;
static uint32_t s_func_freq = 100;
static uint8_t s_func_amp = 100; // 0-100%

#if SOC_DAC_SUPPORTED
static dac_oneshot_handle_t s_dac_handle = NULL;
static dac_cosine_handle_t s_cosine_handle = NULL;
static esp_timer_handle_t s_dac_timer = NULL;
// Sine LUT removed for Hardware Cosine
#endif

/*
 * Web Server Configuration
 */
static httpd_handle_t s_server = NULL;
static bool is_ap = false;
#define ADC_READ_LEN 4096

static adc_continuous_handle_t adc_handle = NULL;

// Single client support for simplicity, or use a list for multiple
static int s_ws_client_fd = -1;

// Global configuration state
static volatile bool s_reconfig_needed = false;
// Function generator globals
static uint32_t s_phase_inc = 0;

static uint32_t s_sample_rate = 20000;
static adc_atten_t s_atten = ADC_ATTEN_DB_11;
static adc_bitwidth_t s_bit_width = ADC_BIT_WIDTH;
// static uint16_t s_test_hz = 100; // Removed/Unused

// Forward declarations
static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num,
                                adc_continuous_handle_t *out_handle);
static esp_err_t ws_handler(httpd_req_t *req);

// Helper to calculate optimal buffer size (approx 50ms latency, max 4096,
// aligned to 4)
static uint32_t get_optimal_buffer_size(uint32_t sample_rate) {
  uint32_t bytes_per_sec = sample_rate * sizeof(adc_digi_output_data_t);
  uint32_t target_size = bytes_per_sec / 50; // 20ms (50Hz)

  // Clamp to min/max
  if (target_size < 128)
    target_size = 128;
  if (target_size > ADC_READ_LEN)
    target_size = ADC_READ_LEN;

  // Align to 4 bytes
  return (target_size + 3) & ~3;
}

/*
 * Task to read from ADC Continuous driver
 */
static void adc_read_task(void *arg) {
  esp_err_t ret;
  uint32_t ret_num = 0;
  uint8_t result[ADC_READ_LEN] = {0};
  memset(result, 0xcc, ADC_READ_LEN);

  // ADC Init (Moved from app_main)
  // TODO: Make this configurable or find a good default pin.
  // For ESP32C6 ADC1 Channel 0 is usually GPIO 0. Let's use Channel 0 for now.
  adc_channel_t channel[1] = {ADC_CHANNEL_0};

  // Init Calibration before ADC
  init_adc_calibration(); // INIT HERE

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

    ret = adc_continuous_read(adc_handle, result,
                              get_optimal_buffer_size(s_sample_rate), &ret_num,
                              0);
    if (ret == ESP_OK) {
      // ESP_LOGI(TAG, "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);

      // OPTIMIZED BATCH SENDING
      // We have `ret_num` bytes of data in `result`.
      // It contains `adc_digi_output_data_t` structs (4 bytes each).
      // We want to extract just the data (12-16 bits) to save bandwidth?
      // The original code was: `uint16_t val = (uint16_t)data;` and sent that.
      // So we have 1/2 the size.

      if (s_ws_client_fd != -1) {
        // Allocate a small temp buffer on stack or static to avoid malloc in
        // loop ret_num is up to ADC_READ_LEN (1024). 1024 / 4 = 256 samples.
        // 256 * 2 bytes = 512 bytes output. Stack safe.
        uint16_t out_buf[ADC_READ_LEN / sizeof(adc_digi_output_data_t)];
        int out_idx = 0;

        for (int i = 0; i < ret_num; i += sizeof(adc_digi_output_data_t)) {
          adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
          uint32_t val = ADC_GET_DATA(p);

          // Apply Calibration (Raw -> Voltage mV)
          int voltage = 0;
          if (s_adc_cali_handle) {
            ESP_ERROR_CHECK(
                adc_cali_raw_to_voltage(s_adc_cali_handle, val, &voltage));
          } else {
            // Fallback Linear
            voltage = val * 3300 / 4096;
          }

          // Fix Ground Offset: Trace calibration often has intercept > 0
          if (val == 0)
            voltage = 0;

          out_buf[out_idx++] = (uint16_t)voltage;
        }

        if (out_idx > 0) {
          httpd_ws_frame_t ws_frame = {.final = true,
                                       .fragmented = false,
                                       .type = HTTPD_WS_TYPE_BINARY,
                                       .payload = (uint8_t *)out_buf,
                                       .len = out_idx * sizeof(uint16_t)};

          // Non-blocking send (best effort)
          esp_err_t ret_ws =
              httpd_ws_send_frame_async(s_server, s_ws_client_fd, &ws_frame);
          if (ret_ws != ESP_OK) {
            ESP_LOGW(TAG, "dropped: %s", esp_err_to_name(ret_ws));

            // Invalidate FD if it's no longer valid (client disconnected)
            if (ret_ws == ESP_ERR_INVALID_ARG || ret_ws == ESP_FAIL) {
              s_ws_client_fd = -1;
            }
          }
        }
      }

      /**
       * Yield check
       */
      taskYIELD(); /* Explicit yield to let WiFi stack run if needed, though
                      send_async should handle it */
    } else if (ret == ESP_ERR_TIMEOUT) {
      // We try to read `ADC_READ_LEN` until API returns timeout, which means
      // there's no available data
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num,
                                adc_continuous_handle_t *out_handle) {
  uint32_t frame_size = get_optimal_buffer_size(s_sample_rate);
  ESP_LOGI(TAG, "Dynamic Buffer Size: %lu bytes", frame_size);

  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = 16384,
      .conv_frame_size = frame_size,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, out_handle));

  // Update the global read length used by the task (hacky but simple for now)
  // Ideally return it, but our init function signature is fixed.
  // We can rely on get_optimal_buffer_size(s_sample_rate) being consistent.

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
// Helper to init Sine LUT

#if SOC_DAC_SUPPORTED
#if SOC_DAC_SUPPORTED
static void IRAM_ATTR dac_timer_callback(void *arg) {
  static uint32_t phase_acc = 0;

  // We pass s_phase_inc via arg
  phase_acc += *((uint32_t *)arg);

  uint8_t val = 0;

  if (s_func_type == FUNC_TRIANGLE) {
    // Triangle: 0..255..0
    // Top 8 bits (0-255) represents 0-360 deg.
    uint8_t p = (phase_acc >> 24) & 0xFF; // 0-255
    if (p < 128) {
      val = p * 2; // 0 -> 0, 127 -> 254
    } else {
      val = (255 - p) * 2; // 128 -> 254, 255 -> 0
    }
  }

  // Scale by amplitude
  // center is 128.
  // val = 128 + (val - 128) * (amp / 100)
  int16_t scale_val = (int16_t)val;
  scale_val = 128 + ((scale_val - 128) * s_func_amp) / 100;
  if (scale_val < 0)
    scale_val = 0;
  if (scale_val > 255)
    scale_val = 255;

  dac_oneshot_output_voltage(s_dac_handle, (uint8_t)scale_val);
}
#endif
#endif
// (Moved to top)
static void update_func_gen() {
  ESP_LOGI(TAG, "Updating Func Gen: Type=%d Freq=%lu Amp=%lu", s_func_type,
           s_func_freq, s_func_amp);

  // 1. Stop Everything First
  static bool ledc_active = false;
  if (ledc_active) {
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_active = false;
    // Reset GPIO 25 to default state after LEDC stops
    gpio_reset_pin(GPIO_NUM_25);
  }

#if SOC_DAC_SUPPORTED
  if (s_dac_timer) {
    esp_timer_stop(s_dac_timer);
    esp_timer_delete(s_dac_timer);
    s_dac_timer = NULL;
  }
  if (s_dac_handle) {
    dac_oneshot_del_channel(s_dac_handle);
    s_dac_handle = NULL;
  }
  if (s_cosine_handle) {
    dac_cosine_stop(s_cosine_handle);
    dac_cosine_del_channel(s_cosine_handle);
    s_cosine_handle = NULL;
  }
  // Ensure pin is reset from DAC mode if we switch to LEDC or Off?
  // dac_oneshot_del should free GPIO.
#endif

  if (s_func_type == FUNC_OFF) {
    ESP_LOGI(TAG, "FuncGen OFF");
    return;
  }

  if (s_func_type == FUNC_SQUARE) {
    // Init LEDC (PWM)
    ESP_LOGI(TAG, "Starting Square Wave (LEDC) at %" PRIu32 " Hz", s_func_freq);

    // Prepare config
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = (uint32_t)(s_func_freq *
                              0.82f), // Corrected for Scope Timebase Mismatch
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = 25, // GPIO 25
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = (1023 * s_func_amp) / 200, // 50% duty * Amp factor?
        // Usually Square wave amplitude isn't changeable with PWM duty (just
        // width). But we can't change voltage level of digital pin easily
        // without DAC. So for PWM, we just fix Duty at 50%? Or let Amplitude
        // control configured cycle? "Amplitude" on digital pin is always 3.3V.
        // We'll just stick to 50% Duty.
        .hpoint = 0};
    ledc_channel.duty = 512; // Fixed 50%

    ledc_channel_config(&ledc_channel);
    ledc_active = true;
  }
#if SOC_DAC_SUPPORTED
  else if (s_func_type == FUNC_SINE) {
    // Hardware Cosine Generator (Min Freq ~130Hz)
    // RTC Clock is uncalibrated. Applying correction factor.
    // User reports 2.09ms (478Hz) at 1.02x. Target 500Hz.
    // New Factor: 1.07x.
    uint32_t corrected_freq = (uint32_t)(s_func_freq * 1.07f);

    if (corrected_freq < 130) {
      corrected_freq = 130;
      ESP_LOGW(TAG, "Clamped Sine Freq to 130Hz");
    }
    ESP_LOGI(TAG,
             "Starting Hardware Cosine at %" PRIu32 " Hz (Target %" PRIu32 ")",
             corrected_freq, s_func_freq);

    dac_cosine_config_t cos_cfg = {.chan_id = DAC_CHAN_0,
                                   .freq_hz = corrected_freq,
                                   .clk_src = DAC_COSINE_CLK_SRC_DEFAULT,
                                   .offset = 0,
                                   .phase = DAC_COSINE_PHASE_0,
                                   .atten = DAC_COSINE_ATTEN_DEFAULT,
                                   .flags.force_set_freq = true};

    // Map Amplitude to Attenuation (Roughly)
    if (s_func_amp > 75)
      cos_cfg.atten = DAC_COSINE_ATTEN_DB_0;
    else if (s_func_amp > 50)
      cos_cfg.atten = DAC_COSINE_ATTEN_DB_6;
    else if (s_func_amp > 25)
      cos_cfg.atten = DAC_COSINE_ATTEN_DB_12;
    else
      cos_cfg.atten = DAC_COSINE_ATTEN_DB_18;

    ESP_ERROR_CHECK(dac_cosine_new_channel(&cos_cfg, &s_cosine_handle));
    ESP_ERROR_CHECK(dac_cosine_start(s_cosine_handle));
  } else if (s_func_type == FUNC_TRIANGLE) {
    ESP_LOGI(TAG, "Starting DAC Triangle at %" PRIu32 " Hz", s_func_freq);
    // Init DAC OneShot
    dac_oneshot_config_t dac_cfg = {
        .chan_id = DAC_CHAN_0,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac_cfg, &s_dac_handle));

    // Calculate Phase Increment
    // Relax timer to 20kHz (50us) to avoid CPU choke
    // Apply 0.82x factor to match Square/Scope Timebase
    uint64_t val = (uint64_t)((float)s_func_freq * 0.82f) * 4294967296ULL;
    s_phase_inc = (uint32_t)(val / 20000);

    esp_timer_create_args_t timer_args = {
        .callback = &dac_timer_callback, .arg = &s_phase_inc, .name = "dac_cw"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_dac_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_dac_timer, 50)); // 50us = 20kHz
  }
#else
  else if (s_func_type == FUNC_SINE || s_func_type == FUNC_TRIANGLE) {
    ESP_LOGW(TAG,
             "DAC Not Supported on this chip! Cannot generate Sine/Triangle.");
  }
#endif
}

// Keep old function signature but redirect
static void start_test_signal(uint32_t hz) {
  s_func_type = FUNC_SQUARE;
  s_func_freq = hz;
  s_func_amp = 100; // Default
  update_func_gen();
}

static void show_status_led() {
#ifdef CONFIG_BSP_CONFIG_GPIO
  gpio_config_t rst_conf = {.pin_bit_mask = (1ULL << CONFIG_BSP_CONFIG_GPIO),
                            .mode = GPIO_MODE_INPUT,
                            .pull_up_en = GPIO_PULLUP_ENABLE,
                            .pull_down_en = GPIO_PULLDOWN_DISABLE,
                            .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&rst_conf);
#endif

  int64_t reset_pressed_time = 0;
  while (true) {
    int64_t now = esp_timer_get_time() / 1000;
#ifdef CONFIG_LED_BUILTIN
    if (is_ap) {
      vTaskDelay(pdMS_TO_TICKS(500));
      gpio_set_level(CONFIG_LED_BUILTIN, 1);
      vTaskDelay(pdMS_TO_TICKS(500));
    } else {
      if (is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(CONFIG_LED_BUILTIN, 1);
      }
      vTaskDelay(pdMS_TO_TICKS(s_ws_client_fd == -1 ? 900 : 200));
    }
    gpio_set_level(CONFIG_LED_BUILTIN, 0);
#else
    vTaskDelay(pdMS_TO_TICKS(1000));
#endif

// Check Reset Pin
#ifdef CONFIG_BSP_CONFIG_GPIO
    if (!is_ap && gpio_get_level(CONFIG_BSP_CONFIG_GPIO) == 0) {
      if (now - reset_pressed_time > 1000) {
        ESP_LOGW(TAG, "Reset to WiFi-AP mode triggered via GPIO %d",
                 CONFIG_BSP_CONFIG_GPIO);
        wifi_manager_erase_config();
        esp_restart();
      } else {
        reset_pressed_time = now;
      }
    } else {
      reset_pressed_time = 0;
    }
#endif
  }
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

#ifdef CONFIG_LED_BUILTIN
  gpio_config_t led_io_conf = {.pin_bit_mask = (1ULL << CONFIG_LED_BUILTIN),
                               .mode = GPIO_MODE_OUTPUT,
                               .pull_up_en = GPIO_PULLUP_DISABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&led_io_conf);
  gpio_set_level(CONFIG_LED_BUILTIN, 0);
#endif

  // #ifdef CONFIG_BOARD_SPECIFIC_INIT
  // #include CONFIG_BOARD_SPECIFIC_INIT
  // #endif

  is_ap = wifi_manager_init_wifi();

  start_test_signal(100);
  xTaskCreate(adc_read_task, "adc_read_task", 8192 + ADC_READ_LEN, NULL, 5,
              NULL);

  // Wait for WiFi connection (Not strict blocking anymore, manager handles it)
  // But let's keep it to print status
  // Note: s_wifi_event_group is local to this file, we removed it in place of
  // manager's. Proper way: expose event group from manager or just dont block
  // here. For now, let's just proceed. The webserver will start and wait for
  // connections.

  start_webserver();
  show_status_led();
}

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

    // Func Gen Params
    cJSON *func_type = cJSON_GetObjectItem(root, "func_type");
    cJSON *func_freq = cJSON_GetObjectItem(root, "func_freq");
    cJSON *func_amp = cJSON_GetObjectItem(root, "func_amp");

    bool func_update = false;
    if (func_type) {
      s_func_type = (func_type_t)func_type->valueint;
      func_update = true;
    }
    if (func_freq) {
      s_func_freq = func_freq->valueint;
      func_update = true;
    }
    if (func_amp) {
      s_func_amp = func_amp->valueint;
      func_update = true;
    }

    if (func_update) {
      update_func_gen();
    }

    ESP_LOGI(
        TAG,
        "Config Request: Rate=%lu, Atten=%d, Width=%d, FGType=%d, Freq=%lu",
        s_sample_rate, s_atten, s_bit_width, s_func_type, s_func_freq);

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

/* Handler for serving index.html */
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
  uint32_t len = index_html_end - index_html_start;
  // Workaround for some build systems adding null byte
  while (len && index_js_start[len - 1] == 0)
    len--;
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
  while (len && index_js_start[len - 1] == 0)
    len--;
  httpd_resp_send(req, (const char *)index_js_start, len);
  return ESP_OK;
}

static const httpd_uri_t uri_index_js = {.uri = "/index.js",
                                         .method = HTTP_GET,
                                         .handler = index_js_handler,
                                         .user_ctx = NULL};

static const char *bye =
    "<head></head><body style='font-family: -apple-system, BlinkMacSystemFont, "
    "&quot;Segoe UI&quot;, Roboto, Helvetica, Arial, sans-serif;\n  "
    "background: #1a1a1a;\n  color: #e0e0e0;'><h1>Bye!</h1>Press \"reset\" on "
    "your esp-scope to start it up again</body>";

static esp_err_t power_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
  httpd_resp_send(req, bye, HTTPD_RESP_USE_STRLEN);

  // delay to ensure network is flushed
  vTaskDelay(pdMS_TO_TICKS(200));

  esp_deep_sleep_start();
  return ESP_OK;
}

static const httpd_uri_t uri_power = {.uri = "/poweroff",
                                      .method = HTTP_GET,
                                      .handler = power_handler,
                                      .user_ctx = NULL};

/* Error handler for 404 - Redirects to captive portal */
static esp_err_t http_404_error_handler(httpd_req_t *req,
                                        httpd_err_code_t err) {
  /* Set status 302 Redirect */
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", "/");
  httpd_resp_send(req, NULL, 0); // No body needed
  return ESP_OK;
}

static void start_mdns_service() {
  esp_err_t err = mdns_init();
  if (err) {
    ESP_LOGE(TAG, "MDNS Init failed: %d", err);
    return;
  }
  ESP_ERROR_CHECK(mdns_hostname_set("esp-scope"));
  ESP_ERROR_CHECK(mdns_instance_name_set("ESP32 Oscilloscope"));
  ESP_LOGI(TAG, "MDNS Init Complete. Hostname: esp-scope");

  // Add service for HTTP
  mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
}

static void start_webserver(void) {
  // Init mDNS
  start_mdns_service();

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;

  ESP_LOGI(TAG, "Starting webserver on port: '%d'", config.server_port);
  if (httpd_start(&s_server, &config) == ESP_OK) {
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(s_server, &uri_index);
    httpd_register_uri_handler(s_server, &uri_index_js);
    httpd_register_uri_handler(s_server, &uri_ws);
    httpd_register_uri_handler(s_server, &uri_params);

    // Register WiFi Manager endpoints
    wifi_manager_register_uri(s_server);
    httpd_register_uri_handler(s_server, &uri_power);

    // Register 404 handler for Captive Portal redirection
    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND,
                               http_404_error_handler);
  } else {
    ESP_LOGI(TAG, "Error starting server!");
  }
}
