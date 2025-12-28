/// <reference lib="dom" />

// Calibration Globals
let g_cal_data = { cal_sq: 0.82, cal_sin: 1.20, cal_tri: 0.82, cal_m: 1.0, cal_c: 0.0, lut: [] };
let scope_points = []; // {meas, act}

function interpolateLut(raw) {
  if (!g_cal_data.lut || g_cal_data.lut.length < 17) return raw * 3.3 / 4096; // Fallback
  let idx = Math.floor(raw / 256);
  if (idx >= 16) return g_cal_data.lut[16] / 1000.0;
  let val1 = g_cal_data.lut[idx];
  let val2 = g_cal_data.lut[idx + 1];
  let frac = (raw % 256) / 256.0;
  return (val1 + (val2 - val1) * frac) / 1000.0;
}

// Fetch calibration on load
fetch('/calibration').then(r => r.json()).then(d => {
  if (d.cal_sq !== undefined) g_cal_data = d;
  if (d.lut) g_cal_data.lut = d.lut;
  console.log("Cal Data:", g_cal_data);
  initCalUI();
}).catch(e => {
  console.log("Cal fetch error", e);
  initCalUI();
});

/*
 * ==========================================
 * 1. Constants & Configuration
 * ==========================================
 */

// Approximate full-scale voltages for ESP32C6/ESP32 ADC attenuations
// 0dB: ~950mV, 2.5dB: ~1250mV, 6dB: ~1750mV, 11dB: ~3100mV+ (use 3.3V)
/** @type {number[]} */
const ATTEN_TO_MAX_V = [0.95, 1.25, 1.75, 3.3];

// Data buffer size
/** @type {number} */
const countPoints = 4000;

/**
 * @typedef {Object} ActiveConfig
 * @property {number} desiredRate - Desired sample rate in Hz
 * @property {number} sample_rate - Actual hardware sample rate
 * @property {number} atten - Attenuation setting index
 * @property {number} bit_width - Bit width (e.g. 12)
 * @property {number} test_hz - Test signal frequency for simulation
 * @property {number} trigger - Trigger level (-1-4097)
 * @property {boolean} invert - Whether trigger logic is inverted
 * @property {number} func_type - 0:Off, 1:Square, 2:Sine, 3:Triangle
 * @property {number} func_freq - Function generator frequency
 * @property {number} func_amp - Function generator amplitude (0-100)
 */

/** @type {ActiveConfig} */
let activeConfig = {
  desiredRate: 20000,
  sample_rate: 20000,
  atten: 3, // 11dB Default
  bit_width: 12,
  test_hz: 100,
  trigger: 2048,
  invert: false,
  func_type: 1, // Default Square
  func_freq: 100,
  func_amp: 100
};

/**
 * @typedef {Object} LowRateState
 * @property {number} accMin - Accumulated minimum value
 * @property {number} accMax - Accumulated maximum value
 * @property {number} accSum - Accumulated sum for average
 * @property {number} accCount - Count of samples in accumulator
 * @property {number} progress - Fractional progress counter
 * @property {number} targetCount - Target samples per output point
 */

/** @type {LowRateState} */
let lowRateState = {
  accMin: 10.0,
  accMax: 0,
  accSum: 0,
  accCount: 0,
  progress: 0.0,
  targetCount: 1.0
};


/*
 * ==========================================
 * 2. Global UI Elements & State
 * ==========================================
 */

/** @type {HTMLCanvasElement} */ const canvas = /** @type {HTMLCanvasElement} */ (document.getElementById('adcChart'));
/** @type {CanvasRenderingContext2D} */ const ctx = canvas.getContext('2d');
/** @type {HTMLElement} */ const statusEl = document.getElementById('status');
/** @type {HTMLElement} */ const triggerStatusEl = document.getElementById('trigger-status');
/** @type {HTMLElement} */ const deltaPanel = document.getElementById('deltaPanel');
/** @type {HTMLInputElement & { invert?: boolean, downValue?: string }} */ const triggerLevel = /** @type {HTMLInputElement & { invert?: boolean, downValue?: string }} */ (document.getElementById('triggerLevel'));

// Config Elements
/** @type {HTMLElement} */ const reconnectBtn = document.getElementById('reconnectBtn');
/** @type {HTMLSelectElement} */ const sampleRateSelect = /** @type {HTMLSelectElement} */ (document.getElementById('sampleRate'));
/** @type {HTMLSelectElement} */ const bitWidthSelect = /** @type {HTMLSelectElement} */ (document.getElementById('bitWidth'));
/** @type {HTMLSelectElement} */ const attenSelect = /** @type {HTMLSelectElement} */ (document.getElementById('atten'));
/** @type {HTMLButtonElement} */ const resetBtn = /** @type {HTMLButtonElement} */ (document.getElementById('resetBtn'));
/** @type {HTMLButtonElement} */ const powerOffBtn = /** @type {HTMLButtonElement} */ (document.getElementById('powerOff'));

// Func Gen Elements
/** @type {HTMLSelectElement} */ const fgTypeSelect = /** @type {HTMLSelectElement} */ (document.getElementById('fgType'));
/** @type {HTMLInputElement} */ const fgFreqInput = /** @type {HTMLInputElement} */ (document.getElementById('fgFreq'));
/** @type {HTMLInputElement} */ const fgAmpInput = /** @type {HTMLInputElement} */ (document.getElementById('fgAmp'));
/** @type {HTMLButtonElement} */ const fgUpdateBtn = /** @type {HTMLButtonElement} */ (document.getElementById('fgUpdate'));

// Tab Elements
const tabBtns = document.querySelectorAll('.tab-btn');
const tabContents = document.querySelectorAll('.tab-content');

tabBtns.forEach(btn => {
  btn.addEventListener('click', () => {
    // Deactivate all
    tabBtns.forEach(b => b.classList.remove('active'));
    tabContents.forEach(c => c.classList.remove('active'));

    // Activate clicked
    btn.classList.add('active');
    const tabName = btn.getAttribute('data-tab');
    document.getElementById(`tab-${tabName}`).classList.add('active');
  });
});

// Auto-update Function Generator
let paramDebounce;
const debouncedSetParams = () => {
  clearTimeout(paramDebounce);
  paramDebounce = setTimeout(setParams, 400);
};

[fgTypeSelect, fgFreqInput, fgAmpInput].forEach(inp => {
  if (!inp) return;
  inp.addEventListener('change', setParams); // Immediate commit
  if (inp !== fgTypeSelect) inp.addEventListener('input', debouncedSetParams); // Slider drag
});

// Wifi Elements
/** @type {HTMLElement} */ const wifiModal = document.getElementById('wifiModal');
/** @type {HTMLButtonElement} */ const wifiBtn = /** @type {HTMLButtonElement} */ (document.getElementById('wifiBtn'));
/** @type {HTMLElement} */ const closeWifiBtn = document.getElementById('closeWifi');
/** @type {HTMLButtonElement} */ const saveWifiBtn = /** @type {HTMLButtonElement} */ (document.getElementById('saveWifi'));
/** @type {HTMLInputElement} */ const wifiSsidInput = /** @type {HTMLInputElement} */ (document.getElementById('wifiSsid'));
/** @type {HTMLInputElement} */ const wifiPassInput = /** @type {HTMLInputElement} */ (document.getElementById('wifiPass'));

/**
 * @typedef {Object} DownsampledPoint
 * @property {number} min
 * @property {number} max
 * @property {number} avg
 */

/** @type {Array<number|DownsampledPoint>} */
let dataBuffer = new Array(countPoints).fill(0);

/** @type {{x: number|null, y: number|null}} */
let lastMousePosition = { x: null, y: null };

/** @type {boolean} */
let isFrozen = false;

/** @type {{t: number, v: number}|null} */
let referencePosition = null; // Store the reference position for deltas

/**
 * @typedef {Object} ViewTransform
 * @property {number} scale
 * @property {number} offsetX
 * @property {number} offsetY
 */

/** @type {ViewTransform} */
let viewTransform = {
  scale: 1,
  offsetX: 0,
  offsetY: 0
};

// WebSocket vars
/** @type {WebSocket} */
let ws;
/** @type {number} */
let reconnectTimeout;


/*
 * ==========================================
 * 3. Coordinate System & Math Utils
 * ==========================================
 */

/**
 * Helper to get total time (width of buffer in ms)
 * @returns {number} Total time in milliseconds
 */
function getTotalTimeMs() {
  let msPerPoint;
  // If we are in low-rate mode (peak detect), we emit 1 point (object)
  // for every 'targetCount' samples of the hardware rate (which is 1kHz).
  if (activeConfig.desiredRate < 1000) {
    msPerPoint = lowRateState.targetCount;
  } else {
    // Normal mode: 1 point = 1 sample
    msPerPoint = 1000.0 / activeConfig.desiredRate;
  }

  // The total time displayed is simply the time-per-pixel * number-of-pixels
  // assuming 1 pixel = 1 point at scale 1.0.
  return msPerPoint * canvas.width;
}

/**
 * Helper to get max voltage
 * @returns {number} Max voltage stored in ATTEN_TO_MAX_V or 3.3
 */
function getMaxVoltage() {
  return ATTEN_TO_MAX_V[activeConfig.atten] || 3.3;
}

/**
 * Convert X pixel coordinate to Time (ms)
 * @param {number} px - Pixel X coordinate
 * @returns {number} Time in milliseconds
 */
function XtoTime(px) {
  const totalTime = getTotalTimeMs();
  if (canvas.width === 0) return 0;
  return ((px - viewTransform.offsetX) / viewTransform.scale) * (totalTime / canvas.width);
}

/**
 * Convert Time (ms) to X pixel coordinate
 * @param {number} t - Time in milliseconds
 * @returns {number} Pixel X coordinate
 */
function TimeToX(t) {
  const totalTime = getTotalTimeMs();
  if (totalTime === 0 || canvas.width === 0) return 0;
  const xp = (t / totalTime) * canvas.width;
  return xp * viewTransform.scale + viewTransform.offsetX;
}

/**
 * Convert Y pixel coordinate to Voltage
 * @param {number} py - Pixel Y coordinate
 * @returns {number} Voltage in volts
 */
function YtoVolts(py) {
  const maxV = getMaxVoltage();
  if (canvas.height === 0) return 0;
  const yp = (py - viewTransform.offsetY) / viewTransform.scale;
  return maxV * (1 - yp / canvas.height);
}

/**
 * Convert Voltage to Y pixel coordinate
 * @param {number} v - Voltage in volts
 * @returns {number} Pixel Y coordinate
 */
function VoltsToY(v) {
  const maxV = getMaxVoltage();
  const yp = canvas.height * (1 - v / maxV);
  return yp * viewTransform.scale + viewTransform.offsetY;
}

/**
 * Nice Number Generator for Grid
 * @param {number} range - Range of values
 * @param {boolean} round - Whether to round to nice fraction
 * @returns {number} Nice number interval
 */
function niceNum(range, round) {
  const exponent = Math.floor(Math.log10(range));
  const fraction = range / Math.pow(10, exponent);
  let niceFraction;
  if (round) {
    if (fraction < 1.5) niceFraction = 1;
    else if (fraction < 3) niceFraction = 2;
    else if (fraction < 7) niceFraction = 5;
    else niceFraction = 10;
  } else {
    if (fraction <= 1) niceFraction = 1;
    else if (fraction <= 2) niceFraction = 2;
    else if (fraction <= 5) niceFraction = 5;
    else niceFraction = 10;
  }
  return niceFraction * Math.pow(10, exponent);
}

/**
 * Calculate nice tick marks for axis
 * @param {number} min - Minimum value
 * @param {number} max - Maximum value
 * @param {number} maxTicks - Maximum number of ticks
 * @returns {number[]} Array of tick values
 */
function calculateNiceTicks(min, max, maxTicks) {
  const range = niceNum(max - min, false);
  const tickSpacing = niceNum(range / (maxTicks - 1), true);
  const niceMin = Math.floor(min / tickSpacing) * tickSpacing;
  const niceMax = Math.ceil(max / tickSpacing) * tickSpacing;
  const ticks = [];
  for (let t = niceMin; t <= niceMax + 0.00001; t += tickSpacing) {
    ticks.push(t);
  }
  return ticks;
}


/*
 * ==========================================
 * 4. Data Processing
 * ==========================================
 */

/**
 * Process incoming WebSocket data
 * @param {Uint16Array} newData - Array of ADC samples
 */
function processData(newData) {
  if (isFrozen) {
    return; // Skip updating the buffer when frozen
  }

  // Recalculate target count based on ratio
  // If virtual rate < 1000, we forced hardware to 1000
  if (activeConfig.desiredRate < 1000) {
    lowRateState.targetCount = activeConfig.sample_rate / activeConfig.desiredRate;
  } else {
    lowRateState.targetCount = 1;
  }

  // Convert incoming mV (integer) to Volts (float)
  const voltsData = new Float32Array(newData.length);
  // Store RAW volts (0-4095). Calibration applied at draw time.
  for (let i = 0; i < newData.length; i++) {
    voltsData[i] = newData[i];
  }

  if (lowRateState.targetCount <= 1) {
    // Passthrough mode
    pushToBuffer(Array.from(voltsData));
  } else {
    // Accumulation mode (Peak Detect) with Fractional Resampling
    /** @type {DownsampledPoint[]} */
    let pointsToPush = [];

    // We add 1.0 "samples worth" of progress for each input sample.
    for (const val of voltsData) {
      if (val < lowRateState.accMin) lowRateState.accMin = val;
      if (val > lowRateState.accMax) lowRateState.accMax = val;
      lowRateState.accSum += val;
      lowRateState.accCount++;

      lowRateState.progress += 1.0;

      if (lowRateState.progress >= lowRateState.targetCount) {
        // Push object with min/max/avg
        const avg = lowRateState.accCount > 0 ? (lowRateState.accSum / lowRateState.accCount) : val;
        pointsToPush.push({
          min: lowRateState.accMin,
          max: lowRateState.accMax,
          avg: avg
        });

        // Reset Min/Max/Sum for next window
        lowRateState.accMin = 10.0; // Safe high value (Volts)
        lowRateState.accMax = 0;
        lowRateState.accSum = 0;
        lowRateState.accCount = 0;

        // Subtract one full window's worth of progress
        lowRateState.progress -= lowRateState.targetCount;
      }
    }
    if (pointsToPush.length > 0) {
      pushToBuffer(pointsToPush);
    }
  }
}

/**
 * Push new items to the rolling data buffer
 * @param {Array<number|DownsampledPoint>} newItems - New items to add
 */
function pushToBuffer(newItems) {
  if (newItems.length >= countPoints) {
    dataBuffer = Array.from(newItems.slice(-countPoints));
  } else {
    dataBuffer.splice(0, newItems.length);
    dataBuffer.push(...newItems);
  }
}


/*
 * ==========================================
 * 5. Visualization & Rendering
 * ==========================================
 */

/**
 * Draw the grid and axis labels
 * @param {number} w - Canvas width
 * @param {number} h - Canvas height
 */
function drawGrid(w, h) {
  ctx.strokeStyle = '#333';
  ctx.lineWidth = 1;
  ctx.fillStyle = '#fff';
  ctx.font = '15px monospace';

  // Helper to draw text with lozenge background
  const drawLabel = (text, x, y, align) => {
    ctx.save();
    const paddingX = 6;
    const paddingY = 3;
    const fontSize = 13;

    // Set font to measure correctly
    ctx.font = `${fontSize}px monospace`;
    const metrics = ctx.measureText(text);
    const textWidth = metrics.width;

    // Calculate box position
    const boxHeight = fontSize + paddingY * 2;
    const boxWidth = textWidth + paddingX * 2;

    let boxX;
    if (align === 'left') boxX = x;
    else if (align === 'center') boxX = x - textWidth / 2;
    else if (align === 'right') boxX = x - textWidth;

    // Adjust for padding and visual centering
    boxX -= paddingX;
    const boxY = (y - 4) - boxHeight / 2;

    // Draw semi-transparent lozenge
    ctx.fillStyle = 'rgba(255, 255, 255, 0.55)';
    ctx.beginPath();
    ctx.roundRect(boxX, boxY, boxWidth, boxHeight, 8);
    ctx.fill();

    // Draw text
    ctx.fillStyle = 'black';
    ctx.textAlign = align;
    ctx.fillText(text, x, y);
    ctx.restore();
  };

  // Determine Visible Voltage Range
  const minV = YtoVolts(h);
  const maxV = YtoVolts(0);

  // Calculate handy ticks in the visible range
  const ticks = calculateNiceTicks(minV, maxV, 8);

  for (let val of ticks) {
    const y = VoltsToY(val);

    // Skip if out of bounds (with a bit of margin)
    if (y < -20 || y > h + 20) continue;

    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();

    drawLabel(val.toFixed(2) + 'V', 5, y + 4, 'left');
  }

  // Determine Visible Time Range
  const minT = XtoTime(0);
  const maxT = XtoTime(w);

  const tTicks = calculateNiceTicks(minT, maxT, 8);

  for (let t of tTicks) {
    const x = TimeToX(t);

    if (x < -50 || x > w + 50) continue;

    let timeStr;
    if (Math.abs(t) >= 1000) {
      timeStr = (t / 1000).toFixed(2) + 's';
    } else {
      timeStr = t.toFixed(1) + 'ms';
    }

    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();

    drawLabel(timeStr, x, h - 5, 'center');
  }
}

/**
 * Draw crosshairs at specific position
 * @param {number} x - X coordinate
 * @param {number} y - Y coordinate
 * @param {string} color - Color string
 */
function drawCrosshairs(x, y, color) {
  ctx.setLineDash([5, 5]);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;

  // Draw vertical line
  ctx.beginPath();
  ctx.moveTo(x, 0);
  ctx.lineTo(x, canvas.height);
  ctx.stroke();

  // Draw horizontal line
  ctx.beginPath();
  ctx.moveTo(0, y);
  ctx.lineTo(canvas.width, y);
  ctx.stroke();

  ctx.setLineDash([]);
}

/**
 * Main draw loop
 */
function draw() {
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const maxV = getMaxVoltage(); // Full Scale Voltage



  // Calibration Factors
  const calToggle = document.getElementById('calToggle');
  const useCal = calToggle && calToggle.value === 'cal';
  const hasUserCal = (Math.abs(g_cal_data.cal_m - 1.0) > 0.001 || Math.abs(g_cal_data.cal_c) > 0.001);

  const applyCal = (raw) => {
    if (!useCal) {
      // Raw Mode: Linear Map 0..4095 -> 0..MaxV
      return (raw / 4095.0) * maxV;
    }
    // Calibrated Mode
    if (hasUserCal) {
      // User Cal: Replaces Factory. Formula: Linear(Raw) * m + c.
      let lin = (raw / 4095.0) * maxV;
      return lin * g_cal_data.cal_m + g_cal_data.cal_c;
    } else {
      // Factory Cal: Use LUT (Interpolated)
      return interpolateLut(raw);
    }
  };

  // Trigger values
  let drawIdx = dataBuffer.length - w;
  if (drawIdx < 0)
    drawIdx = 0;
  else {
    // Slider is 0-4096. Map to 0-MaxV.
    // User reports "Down Increases Voltage", implying inversion.
    // Slider is 0-4096. Max is Top.
    // Slider is 0-4096. Max is Top (visually).
    // User reports Up = Lower Voltage. Thus Up = Low Value.
    // Invert so Up = High Voltage.
    const triggerRaw = parseInt(triggerLevel.value) || 2048;
    const triggerThreshold = 4096 - triggerRaw;

    // Helper to extract value for trigger (handles numbers and avg objects)
    const getVal = (i) => {
      const v = dataBuffer[i];
      return (typeof v === 'object' && v !== null) ? v.avg : v;
    };

    if (triggerLevel.invert) {
      while (drawIdx >= 0) {
        // Look for falling edge
        if (getVal(drawIdx) < triggerThreshold && getVal(drawIdx + 1) > triggerThreshold) {
          break;
        }
        drawIdx -= 1;
      }
    } else {
      while (drawIdx >= 0) {
        // Look for rising edge
        if (getVal(drawIdx) > triggerThreshold && getVal(drawIdx + 1) < triggerThreshold) {
          break;
        }
        drawIdx -= 1;
      }
    }

    const triggerDisplayV = applyCal(triggerThreshold);
    const triggerVolts = triggerDisplayV.toFixed(2) + "V";
    const triggerDir = triggerLevel.invert ? '&#x1F809;' : '&#x1F80B;';
    if (drawIdx < 0) {
      drawIdx = dataBuffer.length - w;
      triggerStatusEl.innerHTML = `<span style="color: #c22727;">${triggerVolts} ${triggerDir} No trigger</span>`;
    } else {
      triggerStatusEl.innerHTML = `<span style="color: #4ade80;">${triggerVolts} ${triggerDir} Triggered</span>`;
    }
  }

  // Pass 1: Draw Min/Max ranges for downsampled data
  ctx.lineWidth = 1;
  ctx.strokeStyle = '#2b7044'; // Dark green
  ctx.beginPath();
  for (let i = 0; i < w; i++) {
    const val = dataBuffer[i + drawIdx];
    if (typeof val === 'object' && val !== null) {
      const sx = i * viewTransform.scale + viewTransform.offsetX;
      const rawYMin = h - (applyCal(val.min) / maxV * h);
      const rawYMax = h - (applyCal(val.max) / maxV * h);

      const screenYMin = rawYMin * viewTransform.scale + viewTransform.offsetY;
      const screenYMax = rawYMax * viewTransform.scale + viewTransform.offsetY;

      ctx.moveTo(sx, screenYMin);
      ctx.lineTo(sx, screenYMax);
    }
  }
  ctx.stroke();

  // Pass 2: Draw Main Trace (Avg or raw value)
  ctx.lineWidth = 2;
  ctx.strokeStyle = '#4ade80'; // Bright green
  ctx.beginPath();

  for (let i = 0; i < w; i++) {
    const val = dataBuffer[i + drawIdx];
    const sx = i * viewTransform.scale + viewTransform.offsetX;
    /** @type {number} */ let rawVal;

    if (typeof val === 'object' && val !== null) {
      rawVal = val.avg;
    } else {
      rawVal = val;
    }

    const yp = h - (applyCal(rawVal) / maxV * h);
    const sy = yp * viewTransform.scale + viewTransform.offsetY;

    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  }

  ctx.stroke();

  // Draw Background Grid
  drawGrid(w, h);

  // Draw Crosshairs if mouse is over the canvas
  if (lastMousePosition.x !== null && lastMousePosition.y !== null) {
    drawCrosshairs(lastMousePosition.x, lastMousePosition.y, '#4ade80');
  }

  // Draw reference crosshairs and deltas if frozen
  if (isFrozen && referencePosition) {
    // Convert World Reference to Screen
    const refX = TimeToX(referencePosition.t);
    const refY = VoltsToY(referencePosition.v);
    drawCrosshairs(refX, refY, '#eab308');
  }
}

/**
 * Loop the animation
 */
function animationLoop() {
  if (!isFrozen) {
    draw();
  }
  requestAnimationFrame(animationLoop);
}


/*
 * ==========================================
 * 6. User Interaction
 * ==========================================
 */

/**
 * Update background color of trigger level slider
 */
function triggerColor() {
  activeConfig.trigger = parseInt(triggerLevel.value);
  const value = (activeConfig.trigger - parseInt(triggerLevel.min)) / (parseInt(triggerLevel.max) - parseInt(triggerLevel.min)) * 100;
  triggerLevel.style.background = triggerLevel.invert
    ? `linear-gradient(to bottom, #00000070 0%, #ff020270  ${value}%, #1302ff70 ${value}%, #00000070 100%)`
    : `linear-gradient(to bottom, #00000070 0%, #1302ff70 ${value}%, #ff020270  ${value}%, #00000070  100%)`
}

/**
 * Update Mouse info
 * @param {MouseEvent | {offsetX: number, offsetY: number, pageX: number, pageY: number}} event
 */
function updateInfo(event) {
  // Use raw coordinates or event helpers
  const voltage = YtoVolts(event.offsetY);
  const timeOffset = XtoTime(event.offsetX);
  const calToggle = document.getElementById('calToggle');
  const calMode = (calToggle && calToggle.value === 'cal') ? " (Cal)" : "";
  let info = `<div>${voltage.toFixed(3)}V${calMode}, ${timeOffset.toFixed(2)}ms</div>`;

  // Store the last mouse position
  lastMousePosition.x = event.offsetX;
  lastMousePosition.y = event.offsetY;

  // Update delta panel position and content if frozen
  if (isFrozen && referencePosition) {
    const deltaVoltage = Math.abs(referencePosition.v - voltage);
    const deltaTime = Math.abs(referencePosition.t - timeOffset);

    // avoid divide by zero
    const freq = deltaTime > 0.00001 ? (1000 / deltaTime).toFixed(2) : '---';
    info += `<div style='color: yellow'>ΔV ${deltaVoltage.toFixed(3)}V, ΔT ${deltaTime.toFixed(2)}ms (${freq} Hz)</div>`;
  }

  deltaPanel.style.left = `${event.pageX + 10}px`;
  deltaPanel.style.top = `${event.pageY + 10}px`;
  deltaPanel.innerHTML = info;

  // Force redraw when frozen
  if (isFrozen) {
    draw();
  }
}

// Trigger Level Events
triggerLevel.addEventListener('input', triggerColor);
triggerLevel.addEventListener('mousedown', function () {
  this.downValue = this.value;
});
triggerLevel.addEventListener('mouseup', function () {
  if (this.downValue == this.value) {
    this.invert = !this.invert;
    activeConfig.invert = this.invert;
    triggerColor();
    this.dispatchEvent(new Event("change"));
  }
  delete this.downValue;
});

// Canvas Interaction Events
canvas.addEventListener('click', (event) => {
  isFrozen = !isFrozen;
  if (isFrozen) {
    // Set reference position in World Coordinates
    referencePosition = {
      t: XtoTime(event.offsetX),
      v: YtoVolts(event.offsetY)
    };
  }
});

canvas.addEventListener('mousemove', updateInfo);
canvas.addEventListener('mouseenter', () => deltaPanel.style.display = 'block');
canvas.addEventListener('mouseleave', () => deltaPanel.style.display = 'none');

canvas.addEventListener('wheel', function (e) {
  e.preventDefault();
  const zoomFactor = 1.1;
  const direction = e.deltaY < 0 ? 1 : -1;
  const factor = direction > 0 ? zoomFactor : 1 / zoomFactor;

  let newScale = viewTransform.scale * factor;

  if (newScale < 1.001) {
    // Snap to 100% and reset position
    newScale = 1.0;
    viewTransform.offsetX = 0;
    viewTransform.offsetY = 0;
  } else if (newScale > 50) {
    return; // Max limit
  } else {
    // Zoom centered on mouse
    const mx = e.offsetX;
    const my = e.offsetY;

    viewTransform.offsetX = mx - (mx - viewTransform.offsetX) * factor;
    viewTransform.offsetY = my - (my - viewTransform.offsetY) * factor;
  }

  viewTransform.scale = newScale;
  if (newScale < 1.001) {
    statusEl.textContent = 'Connected via WebSocket';
  } else {
    statusEl.textContent = 'Scaled to ' + newScale.toFixed(2) + 'x';
  }

  draw();
  if (isFrozen) {
    updateInfo({ offsetX: e.offsetX, offsetY: e.offsetY, pageX: e.pageX, pageY: e.pageY });
  }
});


/*
 * ==========================================
 * 7. Network & Configuration Management
 * ==========================================
 */

/**
 * Connect to WebSocket
 */
function connect() {
  loadStoredConfig();

  clearTimeout(reconnectTimeout);
  if (reconnectBtn) reconnectBtn.style.display = 'none';

  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/signal`;
  // For local testing without ESP hardware, uncomment next line:
  // const wsUrl = 'ws://localhost:8080/signal';

  ws = new WebSocket(wsUrl);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => {
    statusEl.textContent = 'Connected via WebSocket';
    statusEl.style.color = '#4ade80';
    if (ws.readyState === WebSocket.OPEN) ws.send("hello");
  };

  ws.onclose = () => {
    scheduleReconnect();
  };

  ws.onmessage = (event) => {
    try {
      const arr = new Uint16Array(event.data);
      if (arr?.length) {
        processData(arr);
      }
    } catch (e) {
      console.error('Parse error:', e);
    }
  };
}

/**
 * Schedule a reconnect attempt
 */
function scheduleReconnect() {
  statusEl.textContent = 'Disconnected. Retrying in 2s...';
  statusEl.style.color = '#ef4444';
  reconnectTimeout = window.setTimeout(connect, 2000); // Use window.setTimeout explicit
}

// Checked code logic. No change yet.
function setParams() {
  const desiredRate = parseInt(sampleRateSelect.value);
  const hardwareRate = desiredRate < 1000 ? 1000 : desiredRate;

  const payload = {
    sample_rate: hardwareRate,
    bit_width: parseInt(bitWidthSelect.value),
    atten: parseInt(attenSelect.value),
    // test_hz: parseInt(testHzSelect.value) // Deprecated
    func_type: parseInt(fgTypeSelect.value),
    func_freq: parseInt(fgFreqInput.value),
    func_amp: parseInt(fgAmpInput.value)
  };

  fetch('/params', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload)
  }).then(res => {
    if (res.ok) {
      lowRateState.accMin = 4096;
      lowRateState.accMax = 0;
      lowRateState.accSum = 0;
      lowRateState.accCount = 0;

      // Update active config
      activeConfig = { ...payload, desiredRate, trigger: parseInt(triggerLevel.value) || 2048, invert: triggerLevel.invert };
      // Refresh LUT for new attenuation
      fetch('/calibration').then(r => r.json()).then(d => { if (d.lut) g_cal_data.lut = d.lut; });

      // Save to localStorage
      localStorage.setItem('esp32_adc_config', JSON.stringify(activeConfig));
    } else {
      alert('Error updating configuration');
    }
  }).catch(err => alert('Network error: ' + err));
}

/**
 * Load configuration from LocalStorage
 */
function loadStoredConfig() {
  const stored = localStorage.getItem('esp32_adc_config');
  if (stored) {
    try {
      const cfg = JSON.parse(stored);
      if (cfg.desiredRate) sampleRateSelect.value = cfg.desiredRate;
      if (cfg.bit_width) bitWidthSelect.value = cfg.bit_width;
      if (cfg.atten !== undefined) attenSelect.value = cfg.atten;

      // Func Gen restore
      if (cfg.func_type) fgTypeSelect.value = cfg.func_type;
      if (cfg.func_freq) fgFreqInput.value = cfg.func_freq;
      if (cfg.func_amp) fgAmpInput.value = cfg.func_amp;

      if (cfg.invert) triggerLevel.invert = Boolean(cfg.invert);
      if (cfg.trigger) triggerLevel.value = String(cfg.trigger);
      triggerColor();
      setParams();
    } catch (e) {
      console.error("Failed to load config", e);
    }
  }
}

// Config Listeners
if (reconnectBtn) reconnectBtn.addEventListener('click', connect);
// Standard inputs auto-update on change
[sampleRateSelect, bitWidthSelect, attenSelect].forEach(input => {
  if (input) input.addEventListener('change', setParams)
});

// Func Gen inputs
[fgTypeSelect, fgFreqInput, fgAmpInput].forEach(input => {
  if (input) input.addEventListener('change', () => {
    // Optional: auto-update or wait for button
    // Let's wait for button for Freq/Amp to avoid spam, but Type can be instant?
    // For consistency with Scope controls which are instant, maybe instant?
    // But I added a "Set" button. Let's rely on "Set" button for FuncGen to be safe.
    // Or better: Auto update on 'change' (commit), and 'input' (slider drag) is ignored until drop?
    // Let's just use the "Set" button event I added earlier: `fgUpdateBtn.addEventListener...`
    // So we don't need to add them here.
  });
});

triggerLevel.addEventListener('change', () => localStorage.setItem('esp32_adc_config', JSON.stringify(activeConfig)));
if (resetBtn) resetBtn.addEventListener('click', () => {
  localStorage.clear();
  window.location.reload();
});
if (powerOffBtn) powerOffBtn.addEventListener('click', () => window.location.href = "/poweroff");

// ==========================================
// 8. Advanced Calibration UI Logic
// ==========================================
function initCalUI() {
  // Toggle Listener
  const calToggle = document.getElementById('calToggle');
  if (calToggle) calToggle.addEventListener('change', () => {
    // Redraw immediately?
    draw();
  });

  // --- Scope Calibration ---
  const calScopeBtn = document.getElementById('calScopeBtn');
  const scopeModal = document.getElementById('scopeCalModal');
  const addPointBtn = document.getElementById('addCalPoint');
  const saveScopeBtn = document.getElementById('saveScopeCal');
  const closeScopeBtn = document.getElementById('closeScopeCal');
  const pointsList = document.getElementById('calPointsList');

  // Helper to render points safer
  const renderPoints = () => {
    if (!pointsList) return;
    pointsList.innerHTML = '';
    scope_points.forEach((p, i) => {
      const row = document.createElement('div');
      row.style.cssText = "display:flex; gap:5px; margin-bottom:5px;";

      const inMeas = document.createElement('input');
      inMeas.type = 'number'; inMeas.placeholder = 'Meas(V)'; inMeas.step = '0.01'; inMeas.value = p.meas;
      inMeas.oninput = (e) => scope_points[i].meas = parseFloat(e.target.value);

      const inAct = document.createElement('input');
      inAct.type = 'number'; inAct.placeholder = 'Actual(V)'; inAct.step = '0.01'; inAct.value = p.act;
      inAct.oninput = (e) => scope_points[i].act = parseFloat(e.target.value);

      const btnDel = document.createElement('button');
      btnDel.textContent = 'X';
      btnDel.style.cssText = "background:#cc3333; padding:2px 8px;";
      btnDel.onclick = () => { scope_points.splice(i, 1); renderPoints(); };

      row.appendChild(inMeas);
      row.appendChild(inAct);
      row.appendChild(btnDel);
      pointsList.appendChild(row);
    });
  };

  if (calScopeBtn) calScopeBtn.addEventListener('click', () => {
    scope_points = []; // Start fresh? Or load existing? 
    // Loading existing points is hard because we only stored m,c. We dont allow editing points again, just creating new regression.
    // So Start Fresh is correct.
    renderPoints();
    scopeModal.style.display = 'flex';
  });

  if (closeScopeBtn) closeScopeBtn.addEventListener('click', () => scopeModal.style.display = 'none');

  if (addPointBtn) addPointBtn.addEventListener('click', () => {
    if (scope_points.length >= 4) return;
    scope_points.push({ meas: 0, act: 0 });
    renderPoints();
  });

  if (saveScopeBtn) saveScopeBtn.addEventListener('click', () => {
    if (scope_points.length < 2) { alert("Need at least 2 points"); return; }
    // Linear Regression
    let n = scope_points.length;
    let sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    for (let p of scope_points) {
      if (isNaN(p.meas) || isNaN(p.act)) { alert("Invalid values"); return; }
      sumX += p.meas;
      sumY += p.act;
      sumXY += p.meas * p.act;
      sumXX += p.meas * p.meas;
    }
    // Denominator
    let denom = (n * sumXX - sumX * sumX);
    if (Math.abs(denom) < 0.0001) { alert("Points align vertically? Cannot calc slope."); return; }

    let m = (n * sumXY - sumX * sumY) / denom;
    let c = (sumY - m * sumX) / n;

    g_cal_data.cal_m = m;
    g_cal_data.cal_c = c;

    // Post to Server
    fetch('/calibration', {
      method: 'POST',
      body: JSON.stringify({ cal_m: m, cal_c: c })
    }).then(r => {
      if (r.ok) { scopeModal.style.display = 'none'; alert(`Saved! m=${m.toFixed(4)}, c=${c.toFixed(4)}`); }
    });
  });

  // --- Func Gen Calibration ---
  const calFuncBtn = document.getElementById('calFuncBtn');
  const funcModal = document.getElementById('funcCalModal');
  const saveFuncBtn = document.getElementById('saveFuncCal');
  const closeFuncBtn = document.getElementById('closeFuncCal');

  if (calFuncBtn) calFuncBtn.addEventListener('click', () => {
    document.getElementById('calTargetHz').value = activeConfig.func_freq;
    funcModal.style.display = 'flex';
  });
  if (closeFuncBtn) closeFuncBtn.addEventListener('click', () => funcModal.style.display = 'none');

  if (saveFuncBtn) saveFuncBtn.addEventListener('click', () => {
    const target = parseFloat(document.getElementById('calTargetHz').value);
    const actual = parseFloat(document.getElementById('calActualHz').value);
    if (!target || !actual) return;

    const ratio = target / actual;
    const type = parseInt(fgTypeSelect.value);
    let key = '';
    let currentFactor = 1.0;

    if (type === 1) { key = 'cal_sq'; currentFactor = g_cal_data.cal_sq; }
    else if (type === 2) { key = 'cal_sin'; currentFactor = g_cal_data.cal_sin; }
    else if (type === 3) { key = 'cal_tri'; currentFactor = g_cal_data.cal_tri; }
    else { alert("Select a wave type first"); return; }

    const newFactor = currentFactor * ratio;
    g_cal_data[key] = newFactor;

    let payload = {};
    payload[key] = newFactor;

    fetch('/calibration', { method: 'POST', body: JSON.stringify(payload) })
      .then(r => {
        if (r.ok) { funcModal.style.display = 'none'; alert(`Updated Factor: ${currentFactor.toFixed(3)} -> ${newFactor.toFixed(3)}`); setParams(); }
      });
  });

  // --- Reset Handlers ---
  const handleReset = () => {
    if (confirm("Reset FIRMWARE calibration to defaults?")) {
      fetch('/calibration', { method: 'POST', body: JSON.stringify({ reset: true }) })
        .then(r => {
          localStorage.clear();
          window.location.reload();
        });
    }
  };
  const fgReset = document.getElementById('fgResetBtn');
  if (fgReset) fgReset.addEventListener('click', handleReset);

  // New Menu Buttons
  const fgWifi = document.getElementById('fgWifiBtn');
  if (fgWifi) fgWifi.addEventListener('click', () => document.getElementById('wifiModal').style.display = 'flex');
  const fgPower = document.getElementById('fgPowerBtn');
  if (fgPower) fgPower.addEventListener('click', () => window.location.href = "/poweroff");
}

function setupWifiListeners() {
  if (wifiBtn) wifiBtn.onclick = () => {
    wifiModal.style.display = "flex";
    wifiSsidInput.focus();
  };
  if (closeWifiBtn) closeWifiBtn.onclick = () => wifiModal.style.display = "none";
  if (saveWifiBtn) saveWifiBtn.onclick = () => {
    const ssid = wifiSsidInput.value;
    const pass = wifiPassInput.value;

    if (!ssid) {
      alert("SSID is required");
      return;
    }

    saveWifiBtn.innerText = "Saving...";
    fetch('/api/save_wifi', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ssid, password: pass })
    })
      .then(res => res.text())
      .then(text => {
        alert(text);
        window.location.reload();
      })
      .catch(err => {
        alert("Error: " + err);
        saveWifiBtn.innerText = "Save";
      });
  };
}


/*
 * ==========================================
 * 8. Initialization
 * ==========================================
 */

/**
 * Resize canvas to fit container
 */
function resize() {
  canvas.width = canvas.offsetWidth;
  canvas.height = canvas.offsetHeight;
}

window.addEventListener('resize', resize);

// Startup sequence
resize();
setupWifiListeners();
loadStoredConfig();
connect();
requestAnimationFrame(animationLoop);
