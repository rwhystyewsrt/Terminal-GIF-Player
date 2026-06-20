/*
 * stm32_gif_player.ino
 * STM32F103RET6 + ST7735S 2.4" TFT SPI 240x320 GIF播放器
 * 通过USB CDC接收PC端预解码的GIF帧, 存入Flash, 循环播放
 *
 * 硬件SPI1引脚 (连续PA4-PA10, 方便接线):
 *   PA4  - CS    (Chip Select)
 *   PA5  - SCK   (SPI Clock)
 *   PA6  - MISO  (SPI Data In)
 *   PA7  - MOSI  (SPI Data Out)
 *   PA8  - LED   (Backlight PWM)
 *   PA9  - DC    (Data/Command)
 *   PA10 - RESET (Display Reset)
 *
 * 编译:
 *   1. 安装STM32duino: https://github.com/stm32duino/Arduino_Core_STM32
 *   2. 板子: Generic STM32F103RE
 *   3. USB support: CDC (generic 'Serial' supersede U(S)ART)
 *   4. 本目录下 build_opt.h 自动增大USB CDC缓冲区
 */

#include <SPI.h>
#include <Arduino.h>

// ==================== 引脚定义 ====================
#define TFT_CS    PA4
#define TFT_SCK   PA5
#define TFT_MISO  PA6
#define TFT_MOSI  PA7
#define TFT_LED   PA8
#define TFT_DC    PA9
#define TFT_RST   PA10

// ==================== ST7735S 命令 ====================
#define ST7735_SWRESET 0x01
#define ST7735_SLPOUT  0x11
#define ST7735_NORON   0x13
#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36
#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// ==================== 协议常量 ====================
#define MAGIC_B0 0xDE
#define MAGIC_B1 0xAD
#define MAGIC_B2 0xBE
#define MAGIC_B3 0xEF

#define CMD_FRAME_INFO   0x01
#define CMD_FRAME_DATA   0x02
#define CMD_START_PLAY   0x03
#define CMD_STOP         0x04
#define CMD_ERASE_FLASH  0x05

#define RESP_ACK   0x01
#define RESP_NACK  0x02
#define RESP_READY 0x03

// ==================== Flash 存储配置 ====================
// STM32F103RET6: 512KB Flash, 2KB/page
// 代码区: 0x08000000 - 0x0800FFFF (64KB, pages 0-31)
// 帧存储: 0x08010000 - 0x0807FFFF (448KB, pages 32-255)
#define FLASH_PAGE_SIZE        2048
#define FLASH_FRAME_BASE_ADDR  0x08010000
#define FLASH_FRAME_MAX_SIZE   (448UL * 1024UL)

// ==================== 全局变量 ====================
static uint16_t g_frameCount     = 0;
static uint16_t g_frameWidth     = 0;
static uint16_t g_frameHeight    = 0;
static uint32_t g_frameDataSize  = 0;
static uint32_t g_frameDelayMs   = 50;
static uint32_t g_frameFlashAddr = FLASH_FRAME_BASE_ADDR;
static uint32_t g_frameOffsets[256];
static bool     g_playing           = false;
static bool     g_frameInfoReceived  = false;

// ==================== 协议响应 ====================

static void sendResponse(uint8_t code, const uint8_t* payload = NULL, uint16_t len = 0) {
  uint8_t buf[7];
  buf[0] = MAGIC_B0;
  buf[1] = MAGIC_B1;
  buf[2] = MAGIC_B2;
  buf[3] = MAGIC_B3;
  buf[4] = code;
  buf[5] = (uint8_t)(len & 0xFF);
  buf[6] = (uint8_t)((len >> 8) & 0xFF);
  Serial.write(buf, 7);
  if (payload && len > 0) {
    Serial.write(payload, len);
  }
  Serial.flush();
}

static void sendReady() { sendResponse(RESP_READY); }
static void sendAck()   { sendResponse(RESP_ACK); }
static void sendNack()  { sendResponse(RESP_NACK); }

// ==================== ST7735S 驱动 (硬件SPI) ====================

static void tftWriteCmd(uint8_t cmd) {
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPI.transfer(cmd);
  digitalWrite(TFT_CS, HIGH);
}

static void tftWriteData(uint8_t data) {
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, HIGH);
  SPI.transfer(data);
  digitalWrite(TFT_CS, HIGH);
}

static void tftWriteData16(uint16_t data) {
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, HIGH);
  SPI.transfer16(data);
  digitalWrite(TFT_CS, HIGH);
}

static void tftWriteCmdDataBytes(uint8_t cmd, const uint8_t* data, uint16_t len) {
  digitalWrite(TFT_CS, LOW);

  noInterrupts();
  digitalWrite(TFT_DC, LOW);
  SPI.transfer(cmd);

  if (data && len > 0) {
    digitalWrite(TFT_DC, HIGH);
    for (uint16_t i = 0; i < len; i++) {
      SPI.transfer(data[i]);
    }
  }
  interrupts();

  digitalWrite(TFT_CS, HIGH);
}

static void tftSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  // CS 全程拉低，连续发送 CASET/RASET/RAMWR
  // 注意：调用者需要在写完后自己拉高 CS
  digitalWrite(TFT_CS, LOW);

  noInterrupts();
  digitalWrite(TFT_DC, LOW);
  SPI.transfer(ST7735_CASET);
  digitalWrite(TFT_DC, HIGH);
  SPI.transfer(0x00); SPI.transfer(x0);
  SPI.transfer(0x00); SPI.transfer(x1);

  digitalWrite(TFT_DC, LOW);
  SPI.transfer(ST7735_RASET);
  digitalWrite(TFT_DC, HIGH);
  SPI.transfer(0x00); SPI.transfer(y0);
  SPI.transfer(0x00); SPI.transfer(y1);

  digitalWrite(TFT_DC, LOW);
  SPI.transfer(ST7735_RAMWR);
  digitalWrite(TFT_DC, HIGH);
  interrupts();
}

static void tftFillScreen(uint16_t color) {
  if (g_frameWidth == 0 || g_frameHeight == 0) return;
  tftSetAddrWindow(0, 0, g_frameWidth - 1, g_frameHeight - 1);

  uint32_t total = (uint32_t)g_frameWidth * g_frameHeight;
  noInterrupts();
  for (uint32_t i = 0; i < total; i++) {
    SPI.transfer16(color);
  }
  interrupts();
  digitalWrite(TFT_CS, HIGH);
}

static void tftFillScreenDirect(uint16_t color, uint16_t w, uint16_t h) {
  tftSetAddrWindow(0, 0, w - 1, h - 1);
  uint32_t total = (uint32_t)w * h;
  noInterrupts();
  for (uint32_t i = 0; i < total; i++) {
    SPI.transfer16(color);
  }
  interrupts();
  digitalWrite(TFT_CS, HIGH);
}

static void tftInit() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  pinMode(TFT_LED, OUTPUT);

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_LED, LOW);

  // 硬件复位
  digitalWrite(TFT_RST, HIGH);
  delay(10);
  digitalWrite(TFT_RST, LOW);
  delay(10);
  digitalWrite(TFT_RST, HIGH);
  delay(150);

  // 软件复位
  tftWriteCmd(ST7735_SWRESET);
  delay(150);

  // 退出睡眠
  tftWriteCmd(ST7735_SLPOUT);
  delay(120);

  // 帧速率控制
  {
    uint8_t frm1[] = { 0x01, 0x2C, 0x2D };
    tftWriteCmdDataBytes(ST7735_FRMCTR1, frm1, 3);
  }
  {
    uint8_t frm2[] = { 0x01, 0x2C, 0x2D };
    tftWriteCmdDataBytes(ST7735_FRMCTR2, frm2, 3);
  }
  {
    uint8_t frm3[] = { 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D };
    tftWriteCmdDataBytes(ST7735_FRMCTR3, frm3, 6);
  }

  // 反转控制
  {
    uint8_t inv = 0x07;
    tftWriteCmdDataBytes(ST7735_INVCTR, &inv, 1);
  }

  // 电源控制
  {
    uint8_t pw1[] = { 0xA2, 0x02, 0x84 };
    tftWriteCmdDataBytes(ST7735_PWCTR1, pw1, 3);
  }
  {
    uint8_t pw2 = 0xC5;
    tftWriteCmdDataBytes(ST7735_PWCTR2, &pw2, 1);
  }
  {
    uint8_t pw3[] = { 0x0A, 0x00 };
    tftWriteCmdDataBytes(ST7735_PWCTR3, pw3, 2);
  }
  {
    uint8_t pw4[] = { 0x8A, 0x2A };
    tftWriteCmdDataBytes(ST7735_PWCTR4, pw4, 2);
  }
  {
    uint8_t pw5[] = { 0x8A, 0xEE };
    tftWriteCmdDataBytes(ST7735_PWCTR5, pw5, 2);
  }

  {
    uint8_t vm = 0x0E;
    tftWriteCmdDataBytes(ST7735_VMCTR1, &vm, 1);
  }

  // 显示不反转
  tftWriteCmd(ST7735_INVOFF);

  // 16-bit RGB565
  {
    uint8_t cm = 0x05;
    tftWriteCmdDataBytes(ST7735_COLMOD, &cm, 1);
  }

  // MADCTL: MX=1, MY=1, MV=0, BGR=0
  {
    uint8_t mad = 0xC0;
    tftWriteCmdDataBytes(ST7735_MADCTL, &mad, 1);
  }

  // Gamma
  {
    uint8_t gp[] = { 0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
                     0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10 };
    tftWriteCmdDataBytes(ST7735_GMCTRP1, gp, 16);
  }
  {
    uint8_t gn[] = { 0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                     0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10 };
    tftWriteCmdDataBytes(ST7735_GMCTRN1, gn, 16);
  }

  tftWriteCmd(ST7735_NORON);
  delay(10);

  tftWriteCmd(ST7735_DISPON);
  delay(100);

  digitalWrite(TFT_LED, HIGH);

  // 诊断：纯色测试，判断闪烁是硬件/初始化问题还是播放数据问题
  tftFillScreenDirect(0xF800, 128, 128); // 红
  delay(2000);
  tftFillScreenDirect(0x07E0, 128, 128); // 绿
  delay(2000);
  tftFillScreenDirect(0x001F, 128, 128); // 蓝
  delay(2000);

  // 清屏并显示等待提示
  tftFillScreenDirect(0x0000, 128, 128);
  drawString(16, 60, "Waiting for data", 0xFFFF);
}

// 简易 5x7 字体 'A'-'Z', 'a'-'z', '0'-'9', 空格, '-'
static const uint8_t font5x7[][5] = {
  {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
  {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
  {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
  {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
  {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
  {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
  {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
  {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
  {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
  {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
  {0x00, 0x00, 0x00, 0x00, 0x00}, // sp
  {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
  {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
  {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
  {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
  {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
  {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
  {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
  {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
  {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
  {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
  {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
  {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
  {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
  {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
  {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
  {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
  {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
  {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
  {0x46, 0x49, 0x49, 0x49, 0x31}, // S
  {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
  {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
  {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
  {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
  {0x63, 0x14, 0x08, 0x14, 0x63}, // X
  {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
  {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
  {0x00, 0x00, 0x5F, 0x00, 0x00}, // -
  {0x20, 0x54, 0x54, 0x54, 0x78}, // a
  {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
  {0x38, 0x44, 0x44, 0x44, 0x20}, // c
  {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
  {0x38, 0x54, 0x54, 0x54, 0x18}, // e
  {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
  {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
  {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
  {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
  {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
  {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
  {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
  {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
  {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
  {0x38, 0x44, 0x44, 0x44, 0x38}, // o
  {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
  {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
  {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
  {0x48, 0x54, 0x54, 0x54, 0x20}, // s
  {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
  {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
  {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
  {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
  {0x44, 0x28, 0x10, 0x28, 0x44}, // x
  {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
  {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
};

static int fontIndex(char c) {
  if (c == ' ') return 10;
  if (c == '-') return 36;
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'Z') return 11 + (c - 'A');
  if (c >= 'a' && c <= 'z') return 37 + (c - 'a');
  return 10; // space for unknown
}

static void drawChar(uint16_t x, uint16_t y, char c, uint16_t color) {
  int idx = fontIndex(c);
  tftSetAddrWindow(x, y, x + 4, y + 6);
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, HIGH);
  noInterrupts();
  for (int8_t row = 0; row < 7; row++) {
    for (int8_t col = 0; col < 5; col++) {
      bool on = (font5x7[idx][col] >> row) & 1;
      SPI.transfer16(on ? color : 0x0000);
    }
  }
  interrupts();
  digitalWrite(TFT_CS, HIGH);
}

static void drawString(uint16_t x, uint16_t y, const char* s, uint16_t color) {
  while (*s) {
    drawChar(x, y, *s++, color);
    x += 6;
  }
}

// ==================== Flash 操作 ====================

static bool flashErasePages(uint32_t startAddr, uint32_t size) {
  if (startAddr % FLASH_PAGE_SIZE != 0) return false;

  uint32_t numPages = (size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
  uint32_t endAddr  = startAddr + numPages * FLASH_PAGE_SIZE;
  if (endAddr > 0x08080000UL) return false;

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef eraseInit;
  eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
  eraseInit.PageAddress = startAddr;
  eraseInit.NbPages     = numPages;

  uint32_t pageError = 0;
  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &pageError);

  HAL_FLASH_Lock();
  return (status == HAL_OK);
}

// ==================== 协议处理 ====================

static bool waitForMagic() {
  uint32_t start = millis();
  uint8_t  state = 0;

  while (state < 4) {
    if (millis() - start > 5000) return false;
    if (Serial.available() > 0) {
      uint8_t b = Serial.read();
      if (state == 0 && b == MAGIC_B0) state = 1;
      else if (state == 1 && b == MAGIC_B1) state = 2;
      else if (state == 2 && b == MAGIC_B2) state = 3;
      else if (state == 3 && b == MAGIC_B3) { return true; }
      else state = (b == MAGIC_B0) ? 1 : 0;
      start = millis();
    }
  }
  return false;
}

static bool readPacket(uint8_t& cmd, uint8_t*& payload, uint16_t& payloadLen) {
  if (!waitForMagic()) return false;

  // 读命令
  uint32_t start = millis();
  while (Serial.available() < 1) {
    if (millis() - start > 2000) return false;
  }
  cmd = Serial.read();

  // 读payload长度 (2 bytes LE)
  while (Serial.available() < 2) {
    if (millis() - start > 2000) return false;
  }
  uint8_t lo = Serial.read();
  uint8_t hi = Serial.read();
  payloadLen = ((uint16_t)hi << 8) | lo;

  // 读payload
  if (payloadLen > 0) {
    payload = (uint8_t*)malloc(payloadLen);
    if (!payload) return false;

    uint16_t bytesRead = 0;
    start = millis();
    while (bytesRead < payloadLen) {
      if (millis() - start > 5000) {
        free(payload);
        payload = NULL;
        return false;
      }
      int avail = (int)Serial.available();
      if (avail > 0) {
        uint16_t toRead = (uint16_t)avail;
        if (toRead > payloadLen - bytesRead) toRead = payloadLen - bytesRead;
        size_t r = Serial.readBytes(payload + bytesRead, toRead);
        bytesRead += (uint16_t)r;
        start = millis();
      }
    }
  } else {
    payload = NULL;
  }

  return true;
}

static void handleFrameInfo(uint8_t* payload, uint16_t len) {
  if (len < 14) { sendNack(); return; }

  // struct.pack('<HHHHI'): [0:2]=count, [2:4]=width, [4:6]=height, [6:10]=dataSize, [10:14]=delay
  g_frameCount    = ((uint16_t)payload[1] << 8) | payload[0];
  g_frameWidth    = ((uint16_t)payload[3] << 8) | payload[2];
  g_frameHeight   = ((uint16_t)payload[5] << 8) | payload[4];
  g_frameDataSize = ((uint32_t)payload[6] << 0)  | ((uint32_t)payload[7] << 8)
                  | ((uint32_t)payload[8] << 16)  | ((uint32_t)payload[9] << 24);
  g_frameDelayMs  = ((uint32_t)payload[10] << 0)  | ((uint32_t)payload[11] << 8)
                  | ((uint32_t)payload[12] << 16)  | ((uint32_t)payload[13] << 24);

  g_frameInfoReceived = true;
  g_frameFlashAddr = FLASH_FRAME_BASE_ADDR;

  tftFillScreenDirect(0x0000, g_frameWidth, g_frameHeight);

  sendAck();
  sendReady();
}

static void handleEraseFlash(uint8_t* payload, uint16_t len) {
  if (len < 4) { sendNack(); return; }

  uint32_t totalSize = ((uint32_t)payload[0] << 0)  | ((uint32_t)payload[1] << 8)
                     | ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);

  if (totalSize > FLASH_FRAME_MAX_SIZE) { sendNack(); return; }

  uint32_t eraseSize = totalSize;
  if (eraseSize % FLASH_PAGE_SIZE != 0) {
    eraseSize = ((eraseSize / FLASH_PAGE_SIZE) + 1) * FLASH_PAGE_SIZE;
  }

  bool ok = flashErasePages(FLASH_FRAME_BASE_ADDR, eraseSize);
  if (ok) {
    g_frameFlashAddr = FLASH_FRAME_BASE_ADDR;
    sendAck();
    sendReady();
  } else {
    sendNack();
  }
}

static void handleFrameData(uint8_t* payload, uint16_t len) {
  if (len < 6) { sendNack(); return; }

  uint16_t frameIdx = ((uint16_t)payload[1] << 8) | payload[0];
  uint32_t dataSize = ((uint32_t)payload[2] << 0)  | ((uint32_t)payload[3] << 8)
                    | ((uint32_t)payload[4] << 16)  | ((uint32_t)payload[5] << 24);

  if (frameIdx >= 256) { sendNack(); return; }

  // 记录帧偏移
  g_frameOffsets[frameIdx] = g_frameFlashAddr;

  // 接收原始帧数据并直接写入Flash
  uint8_t  buf[512];
  uint32_t remaining = dataSize;
  uint32_t flashAddr = g_frameFlashAddr;

  uint32_t start = millis();

  HAL_FLASH_Unlock();

  while (remaining > 0) {
    if (millis() - start > 15000) {
      HAL_FLASH_Lock();
      sendNack();
      return;
    }

    int avail = (int)Serial.available();
    if (avail > 0) {
      uint32_t toRead = (uint32_t)avail;
      if (toRead > remaining) toRead = remaining;
      if (toRead > sizeof(buf)) toRead = sizeof(buf);

      size_t actual = Serial.readBytes(buf, toRead);
      if (actual == 0) continue;

      // 逐半字写入Flash (Python发送MSB优先, STM32存为little-endian)
      for (uint32_t i = 0; i + 1 < actual; i += 2) {
        uint16_t hw = ((uint16_t)buf[i] << 8) | buf[i + 1];
        HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                                  flashAddr + i, (uint64_t)hw);
        if (st != HAL_OK) {
          HAL_FLASH_Lock();
          sendNack();
          return;
        }
      }

      flashAddr += (uint32_t)actual;
      remaining -= (uint32_t)actual;
      start = millis();
    }
  }

  HAL_FLASH_Lock();

  g_frameFlashAddr = flashAddr;
  sendAck();
  sendReady();
}

static void handleStartPlay() {
  if (!g_frameInfoReceived || g_frameCount == 0) {
    sendNack();
    return;
  }
  sendAck();
  g_playing = true;
}

static void handleStop() {
  g_playing = false;
  tftFillScreen(0x0000);
  sendAck();
}

// ==================== 帧播放 ====================

static void playFrames() {
  if (!g_playing || g_frameCount == 0) return;

  static uint16_t currentFrame  = 0;
  static uint32_t lastFrameTime = 0;
  static bool     busy          = false;

  if (busy) return;
  busy = true;

  uint32_t now = millis();
  if (now - lastFrameTime < g_frameDelayMs) { busy = false; return; }
  lastFrameTime = now;

  uint32_t flashAddr = g_frameOffsets[currentFrame];

  // 设置地址窗口（包含 RAMWR 命令，CS 已拉低，DC 已置高）
  tftSetAddrWindow(0, 0, g_frameWidth - 1, g_frameHeight - 1);

  // 从Flash直接读取RGB565数据并发送到硬件SPI
  uint32_t totalPixels = (uint32_t)g_frameWidth * g_frameHeight;
  const uint16_t* flashData = (const uint16_t*)flashAddr;

  // 分块关中断：每256个像素关一次，避免USB CDC长时间被阻塞
  const uint32_t CHUNK = 256;
  for (uint32_t offset = 0; offset < totalPixels; offset += CHUNK) {
    uint32_t end = offset + CHUNK;
    if (end > totalPixels) end = totalPixels;
    noInterrupts();
    for (uint32_t i = offset; i < end; i++) {
      SPI.transfer16(flashData[i]);
    }
    interrupts();
  }

  digitalWrite(TFT_CS, HIGH);

  currentFrame++;
  if (currentFrame >= g_frameCount) {
    currentFrame = 0;
  }
  busy = false;
}

// ==================== 初始化 ====================

void setup() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_LED, LOW);

  // 硬件SPI1 (PA5/PA6/PA7是默认SPI1引脚)
  SPI.setMOSI(TFT_MOSI);
  SPI.setMISO(TFT_MISO);
  SPI.setSCLK(TFT_SCK);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // 9MHz (72/8)
  SPI.setBitOrder(MSBFIRST);

  tftInit();

  // 初始化 USB CDC（纯 CDC，波特率可任意）
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 5000)) {
    delay(10);
  }

  delay(500);
  Serial.flush();

  // 等待 PC 端枚举完成后再发准备就绪信号
  for (int i = 0; i < 10; i++) {
    if (Serial) break;
    delay(100);
  }

  // 发送多次 READY，方便 PC 端在任意时间连接后都能同步
  for (int i = 0; i < 10; i++) {
    sendReady();
    delay(100);
  }
}

void loop() {
  uint8_t  cmd;
  uint8_t* payload    = NULL;
  uint16_t payloadLen = 0;

  if (readPacket(cmd, payload, payloadLen)) {
    switch (cmd) {
      case CMD_FRAME_INFO:   handleFrameInfo(payload, payloadLen);   break;
      case CMD_FRAME_DATA:   handleFrameData(payload, payloadLen);   break;
      case CMD_START_PLAY:   handleStartPlay();                      break;
      case CMD_STOP:         handleStop();                           break;
      case CMD_ERASE_FLASH:  handleEraseFlash(payload, payloadLen);  break;
      default:               sendNack();                             break;
    }
    if (payload) { free(payload); payload = NULL; }
  }

  playFrames();
}