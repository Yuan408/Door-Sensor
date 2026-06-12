/*
 * Arduino CAN 远端传感器节点
 * ============================
 * 功能：监听 CAN 总线请求帧 (ID=0x101)，读取 SHT30 温湿度并通过 CAN 回复。
 *
 * 硬件接线：
 *   Arduino   ->   MCP2515
 *   ------        --------
 *   D10 (SS)      CS
 *   D13 (SCK)     SCK
 *   D11 (MOSI)    SI
 *   D12 (MISO)    SO
 *   5V             VCC
 *   GND            GND
 *
 *   Arduino   ->   SHT30
 *   ------        -----
 *   A5 (SCL)      SCL
 *   A4 (SDA)      SDA
 *   3.3V           VCC
 *   GND            GND
 *
 * 依赖库（Arduino Library Manager 安装）：
 *   - Adafruit SHT31 Library  (by Adafruit)
 *   - CAN_BUS_Shield          (by Seeed-Studio)
 */

#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <mcp_can.h>
#include <SPI.h>

/* ----- SHT30 ----- */
Adafruit_SHT31 sht = Adafruit_SHT31();

/* ----- MCP2515 ----- */
#define CAN_CS_PIN  10
MCP_CAN CAN(CAN_CS_PIN);

/* ----- CAN 协议定义 ----- */
#define CAN_REQ_ID   0x101   /* ESP32 主控发送的请求帧 ID */
#define CAN_RESP_ID  0x201   /* 本节点回复的传感器数据帧 ID */

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    /* ---- 初始化 I2C/SHT30 ---- */
    Wire.begin();
    if (!sht.begin(0x44)) {
        Serial.println(F("SHT30 not found! Check wiring."));
        while (1) { delay(1000); }
    }
    Serial.println(F("SHT30 ready"));

    /* ---- 初始化 MCP2515 (125 kbps) ---- */
    if (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println(F("MCP2515 init failed! Check wiring."));
        while (1) { delay(1000); }
    }
    CAN.setMode(MCP_NORMAL);
    Serial.println(F("MCP2515 ready (125 kbps, normal mode)"));
}

static void send_sensor_frame(void)
{
    float t = sht.readTemperature();
    float h = sht.readHumidity();

    if (isnan(t)) t = 0.0f;
    if (isnan(h)) h = 0.0f;

    int16_t temp_scaled = (int16_t)(t * 10.0f);
    int16_t humi_scaled = (int16_t)(h * 10.0f);

    uint8_t txBuf[4] = {
        (uint8_t)((temp_scaled >> 8) & 0xFF),
        (uint8_t)(temp_scaled & 0xFF),
        (uint8_t)((humi_scaled >> 8) & 0xFF),
        (uint8_t)(humi_scaled & 0xFF)
    };

    uint8_t send_ret = CAN.sendMsgBuf(CAN_RESP_ID, 0, 4, txBuf);

    Serial.print(F("Sent(ret="));
    Serial.print(send_ret);
    Serial.print(F(", err="));
    Serial.print(CAN.getError(), HEX);
    Serial.print(F(", txerr="));
    Serial.print(CAN.errorCountTX());
    Serial.print(F("): T="));
    Serial.print(t);
    Serial.print(F(" C, H="));
    Serial.print(h);
    Serial.println(F(" %"));
}

void loop()
{
    static unsigned long last_send_ms = 0;
    unsigned long rxId;
    uint8_t len = 0;
    uint8_t rxBuf[8];

    if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
        if (rxId == CAN_REQ_ID) {
            send_sensor_frame();
            last_send_ms = millis();
        } else {
            Serial.print(F("RX ID=0x"));
            Serial.print(rxId, HEX);
            Serial.print(F(" LEN="));
            Serial.println(len);
        }
    }

    if (millis() - last_send_ms >= 1000) {
        send_sensor_frame();
        last_send_ms = millis();
    }
}
