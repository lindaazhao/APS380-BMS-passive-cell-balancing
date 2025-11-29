#include <Wire.h>

// === CONSTANTS ===
#define VREF 5.0
#define R_DIV 5.08
#define ADC_MAX_COUNTS 1023.0
#define I_SENSE_V_OFFSET 2.5
#define I_SENSE_V_PER_A 0.045
#define T_SENSE_ADDR 0x48
#define T_SENSE_REG 0x00
#define PACK_CAP 2.5
#define I_INTERVAL 5   // seconds

// === PIN ASSIGNMENT – YOU MUST MAP THESE ===
// PIC pins have been replaced by placeholder Arduino pins
const int PIN_C1 = A0;
const int PIN_C2 = A1;
const int PIN_C3 = A2;
const int PIN_C4 = A3;

const int PIN_I_SENSE = A4;
const int PIN_BALANCE_ENABLE = 2;

const int PIN_BAL1 = 6;
const int PIN_BAL2 = 7;
const int PIN_BAL3 = 8;
const int PIN_BAL4 = 9;

// === DATA STRUCTURES ===
struct Voltages {
    uint16_t c1_mv;
    uint16_t c2_mv;
    uint16_t c3_mv;
    uint16_t c4_mv;
};

Voltages cellVoltages;
double packCurrent;
int16_t tempC_int;
int16_t tempC_frac;
double SoC = 1.0;
int adcFET = 0;
int balanceFET = 0;
bool balanceEnable = false;

// ==========================================================
// INITIALIZATION
// ==========================================================
void initialize() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(PIN_BAL1, OUTPUT);
    pinMode(PIN_BAL2, OUTPUT);
    pinMode(PIN_BAL3, OUTPUT);
    pinMode(PIN_BAL4, OUTPUT);
    pinMode(PIN_BALANCE_ENABLE, INPUT);

    // Serial.println("C1=n/a mV  C2=n/a mV  C3=n/a mV  C4=n/a mV  I=n/a A  T=n/a C  SoC=n/a");
}

double adcVoltageDecode(int raw) {
    double v_adc  = raw * (VREF / ADC_MAX_COUNTS);
    double v_cell = v_adc * R_DIV;
    return v_cell;
}

// ==========================================================
// VOLTAGE MEASUREMENT
// ==========================================================
void adcTapVoltages() {
    int c1_raw = analogRead(PIN_C1);
    int c2_raw = analogRead(PIN_C2);
    int c3_raw = analogRead(PIN_C3);
    int c4_raw = analogRead(PIN_C4);

    // Serial.print("Cell1="); Serial.print(c1_raw);
    // Serial.print("\nCell2="); Serial.print(c2_raw);
    // Serial.print("\nCell3="); Serial.print(c3_raw);
    // Serial.print("\nCell4="); Serial.print(c4_raw);
    // Serial.print("\n");

    double c1_v = adcVoltageDecode(c1_raw);
    double c2_v = adcVoltageDecode(c2_raw);
    double c3_v = adcVoltageDecode(c3_raw);
    double c4_v = adcVoltageDecode(c4_raw);

    // Serial.print("V_Cell1="); Serial.print(c1_v);
    // Serial.print("\nV_Cell2="); Serial.print(c2_v);
    // Serial.print("\nV_Cell3="); Serial.print(c3_v);
    // Serial.print("\nV_Cell4="); Serial.print(c4_v);
    // Serial.print("\n");

    c4_v = c4_v - c3_v;
    c3_v = c3_v - c2_v;
    c2_v = c2_v - c1_v;

    // Convert to mV; +0.5 for correct rounding during int conversion
    cellVoltages.c1_mv = (uint16_t)(c1_v * 1000.0 + 0.5); 
    cellVoltages.c2_mv = (uint16_t)(c2_v * 1000.0 + 0.5);
    cellVoltages.c3_mv = (uint16_t)(c3_v * 1000.0 + 0.5);
    cellVoltages.c4_mv = (uint16_t)(c4_v * 1000.0 + 0.5);
}

// ==========================================================
// CURRENT SENSE
// ==========================================================
void adcCurrentSense() {
    int raw = analogRead(PIN_I_SENSE);
    double v = (raw / ADC_MAX_COUNTS) * VREF;
    double v_delta = v - I_SENSE_V_OFFSET;
    packCurrent = v_delta / I_SENSE_V_PER_A;
}

// ==========================================================
// I2C TEMP SENSOR (LM75/TMP102-like)
// ==========================================================
int16_t tempDecode() {
    Wire.beginTransmission(T_SENSE_ADDR);
    Wire.write(T_SENSE_REG);
    if (Wire.endTransmission(false) != 0)
        return 0;

    Wire.requestFrom(T_SENSE_ADDR, 2);
    if (Wire.available() < 2)
        return 0;

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();

    int16_t raw = ((int16_t)msb << 8) | lsb;
    raw >>= 4;

    if (raw & 0x0800)
        raw |= 0xF000;

    return raw;
}

void i2cTempSense() {
    int16_t counts = tempDecode();
    double tC = counts * 0.0625;
    int16_t t_x10 = (int16_t)(tC * 10.0);

    tempC_int = t_x10 / 10;
    tempC_frac = abs(t_x10 % 10);
}

// ==========================================================
// COULOMB COUNTING
// ==========================================================
void coulombCount() {
    double delta_Ah = packCurrent * ((double)I_INTERVAL / 3600.0);
    SoC -= delta_Ah / PACK_CAP;

    if (SoC > 1.0) SoC = 1.0;
    if (SoC < 0.0) SoC = 0.0;
}

// ==========================================================
// BALANCING
// ==========================================================
void balanceHandler() {
    if (balanceFET == 0) {
        int maxVal = -100;
        int maxCell = 0;

        if (cellVoltages.c1_mv > maxVal) { maxVal = cellVoltages.c1_mv; maxCell = 1; }
        if (cellVoltages.c2_mv > maxVal) { maxVal = cellVoltages.c2_mv; maxCell = 2; }
        if (cellVoltages.c3_mv > maxVal) { maxVal = cellVoltages.c3_mv; maxCell = 3; }
        if (cellVoltages.c4_mv > maxVal) { maxVal = cellVoltages.c4_mv; maxCell = 4; }

        balanceFET = maxCell;

        if (maxCell == 1) digitalWrite(PIN_BAL1, HIGH);
        if (maxCell == 2) digitalWrite(PIN_BAL2, HIGH);
        if (maxCell == 3) digitalWrite(PIN_BAL3, HIGH);
        if (maxCell == 4) digitalWrite(PIN_BAL4, HIGH);
    }
    else {
        int minVal = 100000;
        int minCell = 0;

        if (cellVoltages.c1_mv < minVal) { minVal = cellVoltages.c1_mv; minCell = 1; }
        if (cellVoltages.c2_mv < minVal) { minVal = cellVoltages.c2_mv; minCell = 2; }
        if (cellVoltages.c3_mv < minVal) { minVal = cellVoltages.c3_mv; minCell = 3; }
        if (cellVoltages.c4_mv < minVal) { minVal = cellVoltages.c4_mv; minCell = 4; }

        if (balanceFET == 1 && minCell == 1) digitalWrite(PIN_BAL1, LOW);
        if (balanceFET == 2 && minCell == 2) digitalWrite(PIN_BAL2, LOW);
        if (balanceFET == 3 && minCell == 3) digitalWrite(PIN_BAL3, LOW);
        if (balanceFET == 4 && minCell == 4) digitalWrite(PIN_BAL4, LOW);

        balanceFET = 0;
        balanceEnable = false;
    }
}

void balanceDriver() {
    balanceEnable = digitalRead(PIN_BALANCE_ENABLE);
    if (balanceEnable) balanceHandler();
}

// ==========================================================
// SERIAL OUTPUT
// ==========================================================
void update() {
    int soc_pct = (int)(SoC * 100.0 + 0.5);

    // Serial.print("V_Cell1="); Serial.print(cellVoltages.c1_mv); Serial.print(",");
    // Serial.print("V_Cell2="); Serial.print(cellVoltages.c2_mv); Serial.print(",");
    // Serial.print("V_Cell3="); Serial.print(cellVoltages.c3_mv); Serial.print(",");
    // Serial.print("V_Cell4="); Serial.print(cellVoltages.c4_mv); Serial.print(",");
    // Serial.print("I_pack=");  Serial.print(packCurrent, 3);     Serial.print(",");
    // Serial.print("T_pack=");  Serial.print(tempC_int);          Serial.print(".");
    // Serial.print(tempC_frac);         Serial.print(",");
    // Serial.print("SOC=");     Serial.print(soc_pct);            Serial.print(",");
    // Serial.print(adcFET);             Serial.print(",");
    // Serial.println(balanceFET);

    Serial.print(cellVoltages.c1_mv); Serial.print(",");
    Serial.print(cellVoltages.c2_mv); Serial.print(",");
    Serial.print(cellVoltages.c3_mv); Serial.print(",");
    Serial.print(cellVoltages.c4_mv); Serial.print(",");
    Serial.print(packCurrent, 3);     Serial.print(",");
    Serial.print(tempC_int);          Serial.print(".");
    Serial.print(tempC_frac);         Serial.print(",");
    Serial.print(soc_pct);            Serial.print(",");
    Serial.print(adcFET);             Serial.print(",");
    Serial.println(balanceFET);
}

// ==========================================================
// MAIN LOOP
// ==========================================================
unsigned long last_ms = 0;

void setup() {
    initialize();
}

void loop() {
    unsigned long now = millis();

    // After 5 seconds, run cycle
    if (now - last_ms >= 5000) {
        last_ms = now;

        adcTapVoltages();
        adcCurrentSense();
        i2cTempSense();
        coulombCount();
        balanceDriver();
        update();
    }
}
