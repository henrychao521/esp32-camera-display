/*
 * camera_display.ino
 * ------------------------------------------------------------------
 * 迷你相機:XIAO ESP32-S3 Sense (OV2640) → 2.4" ILI9341 SPI TFT
 *   相機模式 :即時取景。點按搖桿=拍照;長按=開功能表
 *   功能表   :搖桿上下選、點按進入、長按回相機
 *     - Filter  濾鏡  :左右切特效、上下調亮度、點按返回
 *     - Gallery 相簿  :左右翻看 SD 照片、點按返回
 *     - Video   錄影  :點按開始/停止錄 MJPEG(.avi)、長按返回
 *     - Back    回相機
 *
 * 函式庫:
 *   - GFX Library for Arduino (Arduino_GFX)
 *   - TJpg_Decoder (Bodmer)   ← 相簿/錄影預覽解碼 JPEG,需另裝
 *   - esp_camera / SD 內建於 ESP32 核心(3.x)
 * 開發板:XIAO_ESP32S3 / PSRAM: OPI PSRAM(必開) / USB CDC On Boot: Enabled
 * SD 卡 :FAT32(勿用 exFAT)
 * ------------------------------------------------------------------
 */

#include "esp_camera.h"
#include "img_converters.h"
#include <Arduino_GFX_Library.h>
#include <TJpg_Decoder.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "camera_pins.h"

// ============ 顯示器 + SD 共用 SPI(GPIO 編號)============
#define TFT_CS    2    // D1  → 顯示器 CS
#define TFT_RST   1    // D0  → 顯示器 RES
#define TFT_DC    4    // D3  → 顯示器 DC
#define TFT_SCK   7    // D8  → 顯示器 SCL(與 SD 共用)
#define TFT_MOSI  9    // D10 → 顯示器 SDA(與 SD 共用)
#define TFT_MISO  8    // D9  → SD 卡 MISO(顯示器不接)
#define SD_CS     21   // 內建 SD 卡 CS(內部腳位)

// ============ 搖桿 ============
#define JOY_X     3    // D2 (GPIO3) → VRx(類比)
#define JOY_Y     5    // D4 (GPIO5) → VRy(類比)
#define JOY_SW    6    // D5 (GPIO6) → 按鈕(低電位有效)

// ============ 顏色(RGB565)============
#define COL_BLACK 0x0000
#define COL_WHITE 0xFFFF
#define COL_RED   0xF800
#define COL_GREEN 0x07E0
#define COL_BLUE  0x051D
#define COL_GREY  0x8410

// 顯示器用 Arduino_HWSPI(共享匯流排,才能和 SD 卡共用同一組 SPI)
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, TFT_RST, 1 /* 橫向 */, false);

// ============ 狀態 ============
enum Mode { LIVE, MENU, FILTER, GALLERY, VIDEO };
Mode mode = LIVE;

enum { DIR_NONE, DIR_UP, DIR_DOWN, DIR_LEFT, DIR_RIGHT };

bool cameraOK = false, sdOK = false;
int  photoIndex = 0;
int  menuSel = 0;
const char *menuItems[] = {"Filter", "Gallery", "Video", "Back to Camera"};
const int   MENU_N = 4;

// 濾鏡
int curEffect = 0, curBright = 0;
const char *effectNames[] = {"Normal", "Negative", "B&W", "Red", "Green", "Blue", "Sepia"};

// 相簿
#define GAL_MAX 120
char galList[GAL_MAX][16];
int  galCount = 0, galIdx = 0;

// 錄影(AVI/MJPEG)
File aviFile;
bool recording = false;
uint32_t aviFrames = 0, aviMovi = 0, aviStartMs = 0;
char aviPath[16];

// ------------------------------------------------------------------
// 相機
void applySensor() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  s->set_vflip(s, 1);            // 上下翻(視安裝方向調 0/1)
  s->set_hmirror(s, 0);          // 左右鏡像(視需要調 0/1)
  s->set_special_effect(s, curEffect);
  s->set_brightness(s, curBright);
}

bool initCamera(pixformat_t fmt) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;   config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;   config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;         // ★ XIAO Sense 必須用 10MHz,20MHz 會產生損毀影格(EV-VSYNC-OVF)
  config.frame_size   = FRAMESIZE_QVGA;   // 320x240
  config.pixel_format = fmt;              // RGB565(取景) 或 JPEG(錄影)
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { Serial.printf("相機初始化失敗:0x%x\n", err); return false; }
  applySensor();
  return true;
}

bool setCamFormat(pixformat_t fmt) {
  esp_camera_deinit();
  return initCamera(fmt);
}

// ------------------------------------------------------------------
// 小工具
void centerText(const char *msg, uint16_t color = COL_WHITE) {
  gfx->setTextSize(2);
  gfx->setTextColor(color);
  gfx->setCursor(20, gfx->height() / 2 - 8);
  gfx->print(msg);
}

void toast(const char *msg, uint16_t color) {
  gfx->fillRect(0, 0, gfx->width(), 26, COL_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(color);
  gfx->setCursor(6, 5);
  gfx->print(msg);
}

// TJpg 解碼輸出到螢幕
bool jpgDraw(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  gfx->draw16bitRGBBitmap(x, y, bitmap, w, h);
  return true;
}

// ------------------------------------------------------------------
// 輸入:按鈕(0=無 / 1=點按 / 2=長按)
int pollButton() {
  static bool pressed = false, longFired = false;
  static unsigned long t0 = 0;
  bool low = (digitalRead(JOY_SW) == LOW);
  unsigned long now = millis();
  if (low && !pressed) { pressed = true; longFired = false; t0 = now; return 0; }
  if (low && pressed && !longFired && (now - t0 >= 800)) { longFired = true; return 2; }
  if (!low && pressed) { pressed = false; if (!longFired && (now - t0 < 800)) return 1; }
  return 0;
}

// 輸入:搖桿方向(需回中才會再觸發一次)
int pollJoyDir() {
  static bool armed = true;
  int x = analogRead(JOY_X), y = analogRead(JOY_Y);
  int dir = DIR_NONE;
  if (y < 900) dir = DIR_UP;           // 若上下相反,對調 UP/DOWN 門檻
  else if (y > 3200) dir = DIR_DOWN;
  else if (x < 900) dir = DIR_LEFT;    // 若左右相反,對調 LEFT/RIGHT
  else if (x > 3200) dir = DIR_RIGHT;
  if (dir == DIR_NONE) { armed = true; return DIR_NONE; }
  if (!armed) return DIR_NONE;
  armed = false;
  return dir;
}

// ------------------------------------------------------------------
// 拍照(RGB565 → JPEG → SD)
int nextIndex(const char *fmt) {
  int i = 0; char path[16];
  while (i < 99999) { snprintf(path, sizeof(path), fmt, i); if (!SD.exists(path)) break; i++; }
  return i;
}

// 拍照:暫時切到相機「原生 JPEG」輸出再存檔(顏色正確,和錄影同一條路,
// 不經 RGB565→JPEG 轉換,徹底避開位元組順序問題)
void savePhoto() {
  if (!sdOK) { toast("No SD", COL_RED); delay(600); return; }
  toast("Saving...", COL_WHITE);
  if (!setCamFormat(PIXFORMAT_JPEG)) {
    toast("Cam switch fail", COL_RED); delay(600);
    setCamFormat(PIXFORMAT_RGB565); return;
  }
  camera_fb_t *fb = NULL;
  for (int i = 0; i < 4; i++) {           // 丟掉前幾張,讓曝光/白平衡穩定
    if (fb) esp_camera_fb_return(fb);
    fb = esp_camera_fb_get();
    delay(40);
  }
  bool ok = false;
  if (fb) {
    char path[16]; snprintf(path, sizeof(path), "/IMG%05d.JPG", photoIndex);
    File f = SD.open(path, FILE_WRITE);
    if (f) { f.write(fb->buf, fb->len); f.close(); ok = true; photoIndex++; }
    esp_camera_fb_return(fb);
  }
  setCamFormat(PIXFORMAT_RGB565);         // 切回即時取景
  toast(ok ? "Saved" : "Save fail", ok ? COL_GREEN : COL_RED);
  delay(600);
}

// ------------------------------------------------------------------
// 功能表
void drawMenu() {
  gfx->fillScreen(COL_BLACK);
  gfx->setTextSize(2); gfx->setTextColor(COL_WHITE);
  gfx->setCursor(10, 8); gfx->print("MENU");
  gfx->setTextSize(1); gfx->setTextColor(COL_GREY);
  gfx->setCursor(120, 14); gfx->print("up/down + press");
  for (int i = 0; i < MENU_N; i++) {
    int y = 44 + i * 42;
    if (i == menuSel) { gfx->fillRect(6, y - 5, gfx->width() - 12, 34, COL_BLUE); gfx->setTextColor(COL_WHITE); }
    else gfx->setTextColor(COL_GREY);
    gfx->setTextSize(2); gfx->setCursor(16, y); gfx->print(menuItems[i]);
  }
}

// ------------------------------------------------------------------
// 相簿
void buildGalleryList() {
  galCount = 0;
  File root = SD.open("/");
  if (!root) return;
  File f = root.openNextFile();
  while (f && galCount < GAL_MAX) {
    if (!f.isDirectory()) {
      const char *n = f.name();
      int L = strlen(n);
      if (L >= 4 && (strcasecmp(n + L - 4, ".JPG") == 0)) {
        if (n[0] != '/') { galList[galCount][0] = '/'; strncpy(galList[galCount] + 1, n, 14); }
        else strncpy(galList[galCount], n, 15);
        galList[galCount][15] = 0;
        galCount++;
      }
    }
    f = root.openNextFile();
  }
  root.close();
}

void showPhoto(int idx) {
  gfx->fillScreen(COL_BLACK);
  if (galCount == 0) { centerText("No photos", COL_GREY); return; }
  File f = SD.open(galList[idx]);
  if (!f) { centerText("Open fail", COL_RED); return; }
  size_t sz = f.size();
  uint8_t *buf = (uint8_t *)ps_malloc(sz);
  if (!buf) buf = (uint8_t *)malloc(sz);
  if (!buf) { f.close(); centerText("Mem fail", COL_RED); return; }
  size_t got = 0;                          // 讀滿整個檔(共享 SPI 下單次 read 可能不完整)
  while (got < sz) { int r = f.read(buf + got, sz - got); if (r <= 0) break; got += r; }
  f.close();
  TJpgDec.drawJpg(0, 0, buf, got);
  free(buf);
  gfx->fillRect(0, 0, 96, 20, COL_BLACK);
  gfx->setTextSize(2); gfx->setTextColor(COL_WHITE);
  gfx->setCursor(4, 3); gfx->printf("%d/%d", idx + 1, galCount);
}

// ------------------------------------------------------------------
// AVI / MJPEG 寫檔
void wr32(File &f, uint32_t v) { f.write((uint8_t)v); f.write((uint8_t)(v >> 8)); f.write((uint8_t)(v >> 16)); f.write((uint8_t)(v >> 24)); }
void wr16(File &f, uint16_t v) { f.write((uint8_t)v); f.write((uint8_t)(v >> 8)); }
void seekWr32(File &f, uint32_t pos, uint32_t v) { f.seek(pos); wr32(f, v); }

void writeAviHeader(File &f) {
  f.write((const uint8_t *)"RIFF", 4); wr32(f, 0); f.write((const uint8_t *)"AVI ", 4);
  f.write((const uint8_t *)"LIST", 4); wr32(f, 192); f.write((const uint8_t *)"hdrl", 4);
  f.write((const uint8_t *)"avih", 4); wr32(f, 56);
  wr32(f, 0); wr32(f, 0); wr32(f, 0); wr32(f, 0);   // usecPerFrame(@32) / maxBps / pad / flags
  wr32(f, 0); wr32(f, 0); wr32(f, 1); wr32(f, 0);   // totalFrames(@48) / initial / streams / bufsize
  wr32(f, 320); wr32(f, 240);                       // width / height
  wr32(f, 0); wr32(f, 0); wr32(f, 0); wr32(f, 0);   // reserved[4]
  f.write((const uint8_t *)"LIST", 4); wr32(f, 116); f.write((const uint8_t *)"strl", 4);
  f.write((const uint8_t *)"strh", 4); wr32(f, 56);
  f.write((const uint8_t *)"vids", 4); f.write((const uint8_t *)"MJPG", 4);
  wr32(f, 0); wr16(f, 0); wr16(f, 0); wr32(f, 0);   // flags / prio / lang / initialFrames
  wr32(f, 1); wr32(f, 0); wr32(f, 0); wr32(f, 0);   // scale(@128) / rate(@132) / start / length(@140)
  wr32(f, 0); wr32(f, 0); wr32(f, 0);               // bufsize / quality / sampleSize
  wr16(f, 0); wr16(f, 0); wr16(f, 320); wr16(f, 240); // rcFrame
  f.write((const uint8_t *)"strf", 4); wr32(f, 40);
  wr32(f, 40); wr32(f, 320); wr32(f, 240);
  wr16(f, 1); wr16(f, 24); f.write((const uint8_t *)"MJPG", 4);
  wr32(f, 320 * 240 * 3); wr32(f, 0); wr32(f, 0); wr32(f, 0); wr32(f, 0);
  f.write((const uint8_t *)"LIST", 4); wr32(f, 0); f.write((const uint8_t *)"movi", 4); // moviSize(@216)
}

bool aviStart() {
  snprintf(aviPath, sizeof(aviPath), "/VID%05d.AVI", nextIndex("/VID%05d.AVI"));
  aviFile = SD.open(aviPath, FILE_WRITE);
  if (!aviFile) return false;
  writeAviHeader(aviFile);
  aviFrames = 0; aviMovi = 0; aviStartMs = millis();
  return true;
}

void aviAddFrame(uint8_t *buf, size_t len) {
  aviFile.write((const uint8_t *)"00dc", 4); wr32(aviFile, len);
  aviFile.write(buf, len);
  uint32_t w = 8 + len;
  if (len & 1) { aviFile.write((uint8_t)0); w++; }   // 補到偶數
  aviMovi += w; aviFrames++;
}

void aviEnd() {
  uint32_t el = millis() - aviStartMs; if (el < 1) el = 1;
  float fps = aviFrames * 1000.0f / el; if (fps < 1) fps = 1;
  uint32_t fileSize = 224 + aviMovi;
  seekWr32(aviFile, 4,   fileSize - 8);                    // RIFF size
  seekWr32(aviFile, 32,  (uint32_t)(1000000.0f / fps));    // usecPerFrame
  seekWr32(aviFile, 48,  aviFrames);                       // totalFrames
  seekWr32(aviFile, 132, (uint32_t)(fps + 0.5f));          // rate (scale=1)
  seekWr32(aviFile, 140, aviFrames);                       // stream length
  seekWr32(aviFile, 216, fileSize - 220);                  // movi size
  aviFile.close();
  Serial.printf("錄影完成 %s  %u 幀  %.1f fps\n", aviPath, aviFrames, fps);
}

// ------------------------------------------------------------------
void enterSelected() {
  switch (menuSel) {
    case 0: mode = FILTER; break;                // 相機已是 RGB565
    case 1: buildGalleryList(); galIdx = 0; mode = GALLERY; showPhoto(0); break;
    case 2:
      if (setCamFormat(PIXFORMAT_JPEG)) { recording = false; mode = VIDEO; gfx->fillScreen(COL_BLACK); }
      else { drawMenu(); toast("Cam switch fail", COL_RED); delay(800); }
      break;
    case 3: mode = LIVE; break;
  }
}

void livePreview() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { delay(20); return; }
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)fb->buf, fb->width, fb->height);
  esp_camera_fb_return(fb);
}

// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);

  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI);      // 共享匯流排

  if (!gfx->begin()) Serial.println("顯示器初始化失敗");
  gfx->fillScreen(COL_BLACK);
  gfx->setTextColor(COL_WHITE); gfx->setTextSize(2);
  gfx->setCursor(10, 10); gfx->println("Booting...");

  // TJpg 解碼器
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(false);     // Arduino_GFX + draw16bitRGBBitmap 要用 false(true 會紅藍互換)
  TJpgDec.setCallback(jpgDraw);

  sdOK = SD.begin(SD_CS);
  if (sdOK) { photoIndex = nextIndex("/IMG%05d.JPG"); Serial.printf("SD 就緒,下一張 %d\n", photoIndex); }
  else Serial.println("SD 掛載失敗:檢查卡片/格式 FAT32");

  cameraOK = initCamera(PIXFORMAT_RGB565);
  if (!cameraOK) {
    gfx->fillScreen(COL_RED);
    gfx->setCursor(10, 10); gfx->println("Camera FAIL");
    gfx->setTextSize(1); gfx->setCursor(10, 40);
    gfx->println("PSRAM=OPI & board=XIAO_ESP32S3");
  }
}

// ------------------------------------------------------------------
void loop() {
  if (!cameraOK) { delay(500); return; }
  int btn = pollButton();
  int dir = pollJoyDir();

  switch (mode) {
    case LIVE:
      livePreview();
      if (btn == 1) { if (sdOK) savePhoto(); else { toast("No SD", COL_RED); delay(600); } }
      else if (btn == 2) { menuSel = 0; mode = MENU; drawMenu(); }
      break;

    case MENU:
      if (dir == DIR_UP)   { menuSel = (menuSel + MENU_N - 1) % MENU_N; drawMenu(); }
      if (dir == DIR_DOWN) { menuSel = (menuSel + 1) % MENU_N; drawMenu(); }
      if (btn == 1) enterSelected();
      else if (btn == 2) mode = LIVE;
      break;

    case FILTER:
      livePreview();
      if (dir == DIR_LEFT)  { curEffect = (curEffect + 6) % 7; applySensor(); }
      if (dir == DIR_RIGHT) { curEffect = (curEffect + 1) % 7; applySensor(); }
      if (dir == DIR_UP)    { if (curBright < 2)  { curBright++; applySensor(); } }
      if (dir == DIR_DOWN)  { if (curBright > -2) { curBright--; applySensor(); } }
      {
        int h = 28, y = gfx->height() - h;
        gfx->fillRect(0, y, gfx->width(), h, COL_BLACK);
        gfx->setTextSize(2); gfx->setTextColor(COL_WHITE);
        gfx->setCursor(4, y + 6); gfx->printf("%s  Bright:%d", effectNames[curEffect], curBright);
      }
      if (btn == 1) { mode = MENU; drawMenu(); }
      break;

    case GALLERY:
      if (galCount > 0) {
        if (dir == DIR_LEFT)  { galIdx = (galIdx + galCount - 1) % galCount; showPhoto(galIdx); }
        if (dir == DIR_RIGHT) { galIdx = (galIdx + 1) % galCount; showPhoto(galIdx); }
      }
      if (btn == 1) { mode = MENU; drawMenu(); }
      break;

    case VIDEO: {
      if (btn == 2) {                        // 長按=返回
        if (recording) { aviEnd(); recording = false; }
        setCamFormat(PIXFORMAT_RGB565);
        mode = MENU; drawMenu(); break;
      }
      if (btn == 1) {                        // 點按=開始/停止
        if (!recording) { if (sdOK && aviStart()) { recording = true; gfx->fillScreen(COL_BLACK); } }
        else { aviEnd(); recording = false; gfx->fillScreen(COL_BLACK); }
      }
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) { delay(10); break; }
      if (recording) {
        aviAddFrame(fb->buf, fb->len);
        gfx->fillRect(0, 0, gfx->width(), 30, COL_BLACK);
        gfx->fillCircle(14, 15, 7, COL_RED);
        gfx->setTextSize(2); gfx->setTextColor(COL_WHITE); gfx->setCursor(30, 8);
        gfx->printf("REC %u  %us", aviFrames, (millis() - aviStartMs) / 1000);
      } else {
        TJpgDec.drawJpg(0, 0, fb->buf, fb->len);   // 預覽
        gfx->fillRect(0, 0, gfx->width(), 20, COL_BLACK);
        gfx->setTextSize(1); gfx->setTextColor(COL_GREY);
        gfx->setCursor(4, 6); gfx->print("VIDEO  press=REC  hold=back");
      }
      esp_camera_fb_return(fb);
      break;
    }
  }
}
