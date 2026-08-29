// FIRMWARE ESP32-S3-CAM - ANDORRA 2 UART ONLY
// Detecta color (0=nada, 1=verde, 2=rojo) + centerX + area
// UART Serial1 (RX=3, TX=10) 115200 8N1
// Protocolo: Main envía "D\n" → CAM responde "color,cx,area\n"

#include "esp_camera.h"

// ========== DEFINES DE CONFIGURACIÓN ==========

// Pines cámara (ESP32-S3 DevKit CAM estándar OV2640)
#define CAM_PIN_D0       11   // Y2
#define CAM_PIN_D1       9    // Y3
#define CAM_PIN_D2       8    // Y4
#define CAM_PIN_D3       10   // Y5
#define CAM_PIN_D4       12   // Y6
#define CAM_PIN_D5       18   // Y7
#define CAM_PIN_D6       17   // Y8
#define CAM_PIN_D7       16   // Y9
#define CAM_PIN_VSYNC    6
#define CAM_PIN_HREF     7
#define CAM_PIN_PCLK     13
#define CAM_PIN_XCLK     15
#define CAM_PIN_SIOD     4
#define CAM_PIN_SIOC     5

// Cámara
#define FRAME_WIDTH      320
#define FRAME_HEIGHT     240
#define CAM_MOUNT_FLIP   1        // 1 = espeja CX (cámara 180°)
#define JPEG_QUALITY     8
#define FB_COUNT         1

// HSV ROJO
#define RED_HUE_MIN1     0
#define RED_HUE_MAX1     30
#define RED_HUE_MIN2     310
#define RED_HUE_MAX2     360
#define RED_SAT_MIN      31
#define RED_VAL_MIN      57

// HSV VERDE (principal)
#define GRN_HUE_MIN      60
#define GRN_HUE_MAX      180
#define GRN_SAT_MIN      21
#define GRN_VAL_MIN      48
// Verde pálido/lejano
#define GRN2_HUE_MIN     85
#define GRN2_HUE_MAX     165
#define GRN2_SAT_MIN     30
#define GRN2_VAL_MIN     35
#define GRN2_DIFF_R      15
#define GRN2_DIFF_B      15

// Detección
#define STEP_X           4
#define STEP_Y           4
#define MIN_BLOB_PX      40

// Filtro temporal 2-de-3
#define TEMP_WINDOW      3
#define TEMP_MIN_HITS    2

// UART
#define UART_BAUD        115200
#define UART_RX_PIN      3
#define UART_TX_PIN      47

// Cámara sensor
#define CAM_XCLK_FREQ    20000000
#define CAM_BRIGHTNESS   2
#define CAM_CONTRAST     2
#define CAM_SATURATION   1
#define CAM_SHARPNESS    2
#define CAM_AEC_VALUE    1200
#define CAM_AGC_GAIN     64
#define CAM_GAIN_CEILING GAINCEILING_8X

// Protocolo
#define UART_CMD         "D"
#define PROTO_NONE       0
#define PROTO_GREEN      1
#define PROTO_RED        2

// ========== ESTRUCTURAS ==========

typedef struct {
  uint8_t color;    // 0=nada, 1=verde, 2=rojo
  int centerX;
  int totalArea;
} DetectionResult;

DetectionResult lastResult = {PROTO_NONE, 0, 0};

// Estado filtro temporal 2-de-3
static uint8_t gHist[3] = {0, 0, 0};
static uint8_t gIdx = 0;
static uint8_t rHist[3] = {0, 0, 0};
static uint8_t rIdx = 0;

// Configuración cámara
static camera_config_t camera_config = {
  .pin_pwdn       = -1,
  .pin_reset      = -1,
  .pin_xclk       = CAM_PIN_XCLK,
  .pin_sscb_sda   = CAM_PIN_SIOD,
  .pin_sscb_scl   = CAM_PIN_SIOC,
  .pin_d7         = CAM_PIN_D7,
  .pin_d6         = CAM_PIN_D6,
  .pin_d5         = CAM_PIN_D5,
  .pin_d4         = CAM_PIN_D4,
  .pin_d3         = CAM_PIN_D3,
  .pin_d2         = CAM_PIN_D2,
  .pin_d1         = CAM_PIN_D1,
  .pin_d0         = CAM_PIN_D0,
  .pin_vsync      = CAM_PIN_VSYNC,
  .pin_href       = CAM_PIN_HREF,
  .pin_pclk       = CAM_PIN_PCLK,
  .xclk_freq_hz   = CAM_XCLK_FREQ,
  .ledc_timer     = LEDC_TIMER_0,
  .ledc_channel   = LEDC_CHANNEL_0,
  .pixel_format   = PIXFORMAT_JPEG,
  .frame_size     = FRAMESIZE_QVGA,
  .jpeg_quality   = JPEG_QUALITY,
  .fb_count       = FB_COUNT
};

// ========== HSV HELPERS ==========

void rgbToHsv(int r, int g, int b, int &h, int &s, int &v) {
  int maxC = max(r, max(g, b));
  int minC = min(r, min(g, b));
  v = maxC;
  s = (maxC == 0) ? 0 : ((maxC - minC) * 255 / maxC);
  if (maxC == minC) { h = 0; return; }
  if (maxC == r) {
    h = ((g - b) * 60 / (maxC - minC) + 360) % 360;
  } else if (maxC == g) {
    h = ((b - r) * 60 / (maxC - minC) + 120);
  } else {
    h = ((r - g) * 60 / (maxC - minC) + 240);
  }
}

bool isRed(int r, int g, int b) {
  int h, s, v;
  rgbToHsv(r, g, b, h, s, v);
  return ((h < RED_HUE_MAX1 || h > RED_HUE_MIN2) && s > RED_SAT_MIN && v > RED_VAL_MIN && r >= g && r >= b);
}

bool isGreen(int r, int g, int b) {
  int h, s, v;
  rgbToHsv(r, g, b, h, s, v);
  // Verde principal
  if (h > GRN_HUE_MIN && h < GRN_HUE_MAX && s > GRN_SAT_MIN && v > GRN_VAL_MIN && g >= r && g >= b) return true;
  // Verde pálido/lejano
  if (h > GRN2_HUE_MIN && h < GRN2_HUE_MAX && s > GRN2_SAT_MIN && v > GRN2_VAL_MIN &&
      (g - r) >= GRN2_DIFF_R && (g - b) >= GRN2_DIFF_B) return true;
  return false;
}

// ========== DETECCIÓN DE COLORES ==========

void detectColors() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  size_t rgbLen = FRAME_WIDTH * FRAME_HEIGHT * 3;
  uint8_t *rgb = (uint8_t *)malloc(rgbLen);
  if (!rgb) { esp_camera_fb_return(fb); return; }

  bool decOk = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb);
  esp_camera_fb_return(fb);
  if (!decOk) { free(rgb); return; }

  int rCount = 0, gCount = 0;
  int rCenterX = 0, gCenterX = 0;

  for (int y = 0; y < FRAME_HEIGHT; y += STEP_Y) {
    for (int x = 0; x < FRAME_WIDTH; x += STEP_X) {
      int idx = (y * FRAME_WIDTH + x) * 3;
      int r = rgb[idx];
      int g = rgb[idx + 1];
      int b = rgb[idx + 2];
      if (isRed(r, g, b))   { rCenterX += x; rCount++; }
      if (isGreen(r, g, b)) { gCenterX += x; gCount++; }
    }
  }
  free(rgb);

  bool rValid = rCount >= MIN_BLOB_PX;
  bool gValid = gCount >= MIN_BLOB_PX;

  // Decide color
  if (!rValid && !gValid) {
    lastResult.color = PROTO_NONE;
    lastResult.centerX = 0;
    lastResult.totalArea = 0;
  } else if (rValid && !gValid) {
    lastResult.color = PROTO_RED;
    lastResult.centerX = rCenterX / rCount;
    lastResult.totalArea = rCount;
  } else if (!rValid && gValid) {
    lastResult.color = PROTO_GREEN;
    lastResult.centerX = gCenterX / gCount;
    lastResult.totalArea = gCount;
  } else {
    // Ambos válidos: gana el de mayor área
    if (rCount >= gCount) {
      lastResult.color = PROTO_RED;
      lastResult.centerX = rCenterX / rCount;
      lastResult.totalArea = rCount;
    } else {
      lastResult.color = PROTO_GREEN;
      lastResult.centerX = gCenterX / gCount;
      lastResult.totalArea = gCount;
    }
  }

  // Flip horizontal si cámara invertida
  if (CAM_MOUNT_FLIP && lastResult.color != PROTO_NONE) {
    lastResult.centerX = FRAME_WIDTH - 1 - lastResult.centerX;
  }

  // Filtro temporal 2-de-3 (ventana deslizante)
  gHist[gIdx] = (lastResult.color == PROTO_GREEN);
  gIdx = (gIdx + 1) % TEMP_WINDOW;
  rHist[rIdx] = (lastResult.color == PROTO_RED);
  rIdx = (rIdx + 1) % TEMP_WINDOW;

  if (lastResult.color == PROTO_GREEN && (gHist[0] + gHist[1] + gHist[2]) < TEMP_MIN_HITS) {
    lastResult.color = PROTO_NONE;
    lastResult.centerX = 0;
    lastResult.totalArea = 0;
  }
  if (lastResult.color == PROTO_RED && (rHist[0] + rHist[1] + rHist[2]) < TEMP_MIN_HITS) {
    lastResult.color = PROTO_NONE;
    lastResult.centerX = 0;
    lastResult.totalArea = 0;
  }

}

// ========== CALIBRACIÓN ==========
// Envía "CAL" por Serial o Serial1 para ver rangos HSV de la escena actual

void calibrateColors() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("[CAL] ERROR: no se pudo capturar frame"); return; }

  size_t rgbLen = FRAME_WIDTH * FRAME_HEIGHT * 3;
  uint8_t *rgb = (uint8_t *)malloc(rgbLen);
  if (!rgb) { esp_camera_fb_return(fb); Serial.println("[CAL] ERROR: malloc falló"); return; }

  bool decOk = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb);
  esp_camera_fb_return(fb);
  if (!decOk) { free(rgb); Serial.println("[CAL] ERROR: decodificación falló"); return; }

  // Rangos para TODOS los pixels
  int allHMin = 999, allHMax = -1;
  int allSMin = 999, allSMax = -1;
  int allVMin = 999, allVMax = -1;
  int allCount = 0;

  // Rangos para píxeles VERDES (sin filtro, solo HSV)
  int grnHMin = 999, grnHMax = -1;
  int grnSMin = 999, grnSMax = -1;
  int grnVMin = 999, grnVMax = -1;
  int grnCount = 0;

  // Rangos para píxeles ROJOS (sin filtro, solo HSV)
  int redHMin = 999, redHMax = -1;
  int redSMin = 999, redSMax = -1;
  int redVMin = 999, redVMax = -1;
  int redCount = 0;

  for (int y = 0; y < FRAME_HEIGHT; y += STEP_Y) {
    for (int x = 0; x < FRAME_WIDTH; x += STEP_X) {
      int idx = (y * FRAME_WIDTH + x) * 3;
      int r = rgb[idx];
      int g = rgb[idx + 1];
      int b = rgb[idx + 2];

      int h, s, v;
      rgbToHsv(r, g, b, h, s, v);

      // Todos
      if (h < allHMin) allHMin = h; if (h > allHMax) allHMax = h;
      if (s < allSMin) allSMin = s; if (s > allSMax) allSMax = s;
      if (v < allVMin) allVMin = v; if (v > allVMax) allVMax = v;
      allCount++;

      // Verde: tono entre 60-180 y verde dominante
      if (h >= 60 && h <= 180 && g >= r && g >= b && s > 20 && v > 20) {
        if (h < grnHMin) grnHMin = h; if (h > grnHMax) grnHMax = h;
        if (s < grnSMin) grnSMin = s; if (s > grnSMax) grnSMax = s;
        if (v < grnVMin) grnVMin = v; if (v > grnVMax) grnVMax = v;
        grnCount++;
      }

      // Rojo: tono cerca de 0 o 360 + rojo dominante
      if ((h <= 30 || h >= 330) && r >= g && r >= b && s > 30 && v > 30) {
        if (h < redHMin) redHMin = h; if (h > redHMax) redHMax = h;
        if (s < redSMin) redSMin = s; if (s > redSMax) redSMax = s;
        if (v < redVMin) redVMin = v; if (v > redVMax) redVMax = v;
        redCount++;
      }
    }
  }
  free(rgb);

  Serial.println("========================================");
  Serial.println("[CAL] RESULTADOS DE CALIBRACIÓN");
  Serial.println("========================================");
  Serial.printf("[CAL] TOTAL pixels: %d\n", allCount);
  Serial.printf("[CAL] TODOS  -> H:%d-%d  S:%d-%d  V:%d-%d\n",
                allHMin, allHMax, allSMin, allSMax, allVMin, allVMax);
  Serial.println("----------------------------------------");

  if (grnCount > 0) {
    Serial.printf("[CAL] VERDE  -> H:%d-%d  S:%d-%d  V:%d-%d  (%d px)\n",
                  grnHMin, grnHMax, grnSMin, grnSMax, grnVMin, grnVMax, grnCount);
    Serial.printf("[CAL]   >> Copia estos valores a: GRN_HUE_MIN=%d  GRN_HUE_MAX=%d\n", grnHMin, grnHMax);
    Serial.printf("[CAL]   >> GRN_SAT_MIN=%d  GRN_VAL_MIN=%d\n", grnSMin, grnVMin);
  } else {
    Serial.println("[CAL] VERDE  -> NO detectado (apunta algo verde y vuelve a enviar CAL)");
  }

  Serial.println("----------------------------------------");

  if (redCount > 0) {
    Serial.printf("[CAL] ROJO   -> H:%d-%d  S:%d-%d  V:%d-%d  (%d px)\n",
                  redHMin, redHMax, redSMin, redSMax, redVMin, redVMax, redCount);
    Serial.printf("[CAL]   >> Copia estos valores a: RED_SAT_MIN=%d  RED_VAL_MIN=%d\n", redSMin, redVMin);
  } else {
    Serial.println("[CAL] ROJO   -> NO detectado (apunta algo rojo y vuelve a enviar CAL)");
  }

  Serial.println("========================================");
}

// ========== RESPUESTA UART ==========

void respondToD() {
  detectColors();
  if (lastResult.totalArea > MIN_BLOB_PX) {
    if(lastResult.color==1 || lastResult.color==0)
    {
      Serial1.printf("%d,%d,%d\n", lastResult.color, lastResult.centerX, lastResult.totalArea);
      Serial.printf("%d,%d,%d\n", lastResult.color, lastResult.centerX, lastResult.totalArea);
    }
    
  }
}

// ========== INICIALIZACIÓN CÁMARA ==========

void initCamera() {
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    while (true) { delay(100); }
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_quality(s, JPEG_QUALITY);
  s->set_brightness(s, CAM_BRIGHTNESS);
  s->set_contrast(s, CAM_CONTRAST);
  s->set_saturation(s, CAM_SATURATION);
  s->set_sharpness(s, CAM_SHARPNESS);
  s->set_whitebal(s, 1);
  s->set_aec2(s, 1);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, CAM_AGC_GAIN);
  s->set_gainceiling(s, CAM_GAIN_CEILING);
  s->set_exposure_ctrl(s, 1);
  s->set_aec_value(s, CAM_AEC_VALUE);
  if (CAM_MOUNT_FLIP) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  }
}

// ========== SETUP / LOOP ==========

void setup() {
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  initCamera();
}

void loop() {
  // Comandos desde Serial1 (UART externa)
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    if (cmd == UART_CMD) respondToD();
    if (cmd == "CAL") calibrateColors();
  }
  // Comandos desde Serial (USB - Monitor Serial)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "CAL") calibrateColors();
  }
  delay(50);
}