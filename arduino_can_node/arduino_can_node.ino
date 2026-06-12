/*
 * Arduino CAN 节点 —— 全新引脚软件 SPI
 */
#include <Wire.h>
#include <Adafruit_SHT31.h>

Adafruit_SHT31 sht = Adafruit_SHT31();

/* 全新软件 SPI 引脚（不用 D10-D13） */
#define CSPIN   9
#define MOSIPIN 7
#define MISOPIN 6
#define SCKPIN  5

#define CANSTAT  0x0E
#define CANCTRL  0x0F
#define CNF3     0x28
#define CNF2     0x29
#define CNF1     0x2A
#define CANINTE  0x2B
#define CANINTF  0x2C
#define EFLG     0x2D
#define TEC      0x1C
#define REC      0x1D
#define RXB0CTRL 0x60
#define RXB1CTRL 0x70
#define TXB0CTRL 0x30

#define CMD_RESET     0xC0
#define CMD_READ      0x03
#define CMD_WRITE     0x02
#define CMD_BIT_MODIFY 0x05

#define MODE_CONFIG  0x80
#define MODE_NORMAL  0x00
#define MODE_MASK    0xE0

#define CFG1 0x43
#define CFG2 0xAE
#define CFG3 0x85

static uint8_t xfer(uint8_t tx) {
    uint8_t rx = 0;
    for (int i = 0; i < 8; i++) {
        digitalWrite(MOSIPIN, tx & 0x80 ? HIGH : LOW);
        tx <<= 1;
        digitalWrite(SCKPIN, HIGH);
        delayMicroseconds(1);
        rx = (rx << 1) | (digitalRead(MISOPIN) ? 1 : 0);
        digitalWrite(SCKPIN, LOW);
        delayMicroseconds(1);
    }
    return rx;
}

static void sw_write(uint8_t addr, uint8_t val) {
    digitalWrite(CSPIN, LOW);
    xfer(CMD_WRITE); xfer(addr); xfer(val);
    digitalWrite(CSPIN, HIGH);
}

static uint8_t sw_read(uint8_t addr) {
    digitalWrite(CSPIN, LOW);
    xfer(CMD_READ); xfer(addr);
    uint8_t v = xfer(0x00);
    digitalWrite(CSPIN, HIGH);
    return v;
}

static void sw_bitmod(uint8_t addr, uint8_t mask, uint8_t val) {
    digitalWrite(CSPIN, LOW);
    xfer(CMD_BIT_MODIFY); xfer(addr); xfer(mask); xfer(val);
    digitalWrite(CSPIN, HIGH);
}

static void mcp_reset() {
    digitalWrite(CSPIN, LOW);
    xfer(CMD_RESET);
    digitalWrite(CSPIN, HIGH);
    delay(10);
}

static bool set_mode(uint8_t mode) {
    sw_bitmod(CANCTRL, MODE_MASK, mode);
    for (int i = 0; i < 200; i++) {
        if ((sw_read(CANSTAT) & MODE_MASK) == mode) return true;
        delay(1);
    }
    return false;
}

void setup() {
    Serial.begin(115200);

    pinMode(CSPIN, OUTPUT);    digitalWrite(CSPIN, HIGH);
    pinMode(MOSIPIN, OUTPUT);  digitalWrite(MOSIPIN, LOW);
    pinMode(MISOPIN, INPUT);   /* 先不用 pull-up */
    pinMode(SCKPIN, OUTPUT);   digitalWrite(SCKPIN, LOW);

    delay(500);

    /* 先测 MISO 空闲电平（CS=HIGH 时应为高阻，pull-up 会读到 HIGH） */
    pinMode(MISOPIN, INPUT_PULLUP);
    delay(1);
    int miso_idle = digitalRead(MISOPIN);
    pinMode(MISOPIN, INPUT);
    Serial.print(F("MISO idle (CS=HIGH, PU): "));
    Serial.println(miso_idle);

    /* SHT30 */
    Wire.begin();
    if (!sht.begin(0x44)) {
        Serial.println(F("SHT30 not found!"));
        while (1) { delay(1000); }
    }
    Serial.println(F("SHT30 ready"));

    /* MCP2515 复位 */
    mcp_reset();
    delay(50);

    /* 复位后直接读 CANSTAT，不切模式 */
    uint8_t stat = sw_read(CANSTAT);
    Serial.print(F("CANSTAT after reset: 0x"));
    Serial.println(stat, HEX);

    /* SPI 测试 */
    sw_write(CANINTE, 0xA5);
    uint8_t test = sw_read(CANINTE);
    Serial.print(F("SPI test CANINTE: wrote=0xA5 read=0x"));
    Serial.println(test, HEX);
    if (test != 0xA5) {
        Serial.println(F("FAIL: SPI readback mismatch"));
        while (1) { delay(1000); }
    }

    /* 进配置 */
    if (!set_mode(MODE_CONFIG)) {
        Serial.println(F("FAIL: config mode"));
        while (1) { delay(1000); }
    }
    Serial.println(F("Config mode OK"));

    /* 写 CNF */
    sw_write(CNF1, CFG1);
    sw_write(CNF2, CFG2);
    sw_write(CNF3, CFG3);
    delay(20);

    uint8_t c1 = sw_read(CNF1), c2 = sw_read(CNF2), c3 = sw_read(CNF3);
    Serial.print(F("CNF: ")); Serial.print(c1,HEX); Serial.print(' '); Serial.print(c2,HEX); Serial.print(' '); Serial.println(c3,HEX);
    if (c1 != CFG1 || c2 != CFG2 || c3 != CFG3) {
        Serial.println(F("FAIL: CNF mismatch"));
        while (1) { delay(1000); }
    }

    sw_write(RXB0CTRL, 0x60);
    sw_write(RXB1CTRL, 0x60);
    sw_write(CANINTE, 0x00);

    if (!set_mode(MODE_NORMAL)) {
        Serial.println(F("FAIL: normal mode"));
        while (1) { delay(1000); }
    }
    Serial.println(F("MCP2515 ready (SW SPI, 125k normal)"));
}

void loop() {
    static unsigned long last_s = 0;

    if (millis() - last_s >= 1000) {
        float t = sht.readTemperature();
        float h = sht.readHumidity();
        if (isnan(t)) t = 0.0f; if (isnan(h)) h = 0.0f;
        int16_t ts = (int16_t)(t*10.0f), hs = (int16_t)(h*10.0f);
        uint8_t tx[4] = {(uint8_t)(ts>>8),(uint8_t)ts,(uint8_t)(hs>>8),(uint8_t)hs};

        sw_bitmod(TXB0CTRL, 0x08, 0x00);
        sw_write(0x31,0x40); sw_write(0x32,0x20);
        sw_write(0x33,0); sw_write(0x34,0);
        sw_write(0x35,4);
        for(int i=0;i<4;i++) sw_write(0x36+i,tx[i]);
        digitalWrite(CSPIN,LOW); xfer(0x81); digitalWrite(CSPIN,HIGH);

        uint8_t intf = sw_read(CANINTF);
        uint8_t eflg = sw_read(EFLG);
        uint8_t tec  = sw_read(TEC);
        uint8_t rec  = sw_read(REC);

        Serial.print(F("Sent T=")); Serial.print(t);
        Serial.print(F(" H=")); Serial.print(h);
        Serial.print(F(" | INTF=0x")); Serial.print(intf,HEX);
        Serial.print(F(" EFLG=0x")); Serial.print(eflg,HEX);
        Serial.print(F(" TEC=")); Serial.print(tec);
        Serial.print(F(" REC=")); Serial.println(rec);
        last_s = millis();
    }

    uint8_t intf = sw_read(CANINTF);
    if (intf & 0x01) {
        uint32_t id = ((uint32_t)sw_read(0x61)<<3) | (sw_read(0x62)>>5);
        uint8_t dlc = sw_read(0x65)&0x0F; if(dlc>8)dlc=8;
        Serial.print(F("RX ID=0x")); Serial.print(id,HEX);
        Serial.print(F(" LEN=")); Serial.print(dlc);
        for(int i=0;i<dlc;i++) { Serial.print(' '); Serial.print(sw_read(0x66+i),HEX); }
        Serial.println();
        sw_bitmod(CANINTF,0x01,0x00);
    }
    if (intf & 0x02) {
        uint32_t id = ((uint32_t)sw_read(0x71)<<3) | (sw_read(0x72)>>5);
        uint8_t dlc = sw_read(0x75)&0x0F; if(dlc>8)dlc=8;
        Serial.print(F("RX(1) ID=0x")); Serial.print(id,HEX);
        Serial.print(F(" LEN=")); Serial.print(dlc);
        for(int i=0;i<dlc;i++) { Serial.print(' '); Serial.print(sw_read(0x76+i),HEX); }
        Serial.println();
        sw_bitmod(CANINTF,0x02,0x00);
    }
    delay(10);
}
