#include <Wire.h>

// === CONSTANTS ===
#define VREF 5.0
#define R_DIV 5.08
#define ADC_MAX_COUNTS 1023.0
#define I_SENSE_V_OFFSET 2.5
#define I_SENSE_V_PER_A 0.045
#define T_SENSE_ADDR 0x48
#define T_SENSE_REG 0x00
#define PACK_CAP 2.5    // Ah
#define I_INTERVAL 5    // seconds
const double BALANCE_START_SOC_TH = 0.05;
const double BALANCE_STOP_SOC_TH = 0.02;

const double C1_CAP = 2472.0;
const double C2_CAP = 2474.0;
const double C3_CAP = 2240.0;
const double C4_CAP = 2290.0;

const double C1_CUR = 2271.0;
const double C2_CUR = 2275.0;
const double C3_CUR = 2233.0;
const double C4_CUR = 2275.0;

const double I_discharge = 3;

bool balanceActive[4] = {false, false, false, false};


// === PIN ASSIGNMENT ===
// Cell voltage ADCs
const int PIN_C1 = A0;
const int PIN_C2 = A1;
const int PIN_C3 = A2;
const int PIN_C4 = A3;

const int PIN_I_SENSE = A4;

const int PIN_BALANCE_ENABLE = 2;

// Balance enable GPIOs
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

struct cellSOC {
    double c1_soc;
    double c2_soc;
    double c3_soc;
    double c4_soc;
};

Voltages cellVoltages;
cellSOC cell_SOCs;
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

    pinMode(PIN_BALANCE_ENABLE, INPUT);

    pinMode(PIN_BAL1, OUTPUT);
    pinMode(PIN_BAL2, OUTPUT);
    pinMode(PIN_BAL3, OUTPUT);
    pinMode(PIN_BAL4, OUTPUT);

    cell_SOCs.c1_soc = C1_CUR / C1_CAP;
    cell_SOCs.c2_soc = C2_CUR / C2_CAP;
    cell_SOCs.c3_soc = C3_CUR / C3_CAP;
    cell_SOCs.c4_soc = C4_CUR / C4_CAP;
    // Serial.println("C1=n/a mV  C2=n/a mV  C3=n/a mV  C4=n/a mV  I=n/a A  T=n/a C  SoC=n/a");
}

// ==========================================================
// VOLTAGE MEASUREMENT
// ==========================================================
double adcVoltageDecode(int raw) {
    double v_adc  = raw * (VREF / ADC_MAX_COUNTS);
    double v_cell = v_adc * R_DIV;
    return v_cell;
}

void adcTapVoltages() {
    int c1_raw = analogRead(PIN_C1);
    int c2_raw = analogRead(PIN_C2);
    int c3_raw = analogRead(PIN_C3);
    int c4_raw = analogRead(PIN_C4);

    double c1_v = adcVoltageDecode(c1_raw);
    double c2_v = adcVoltageDecode(c2_raw);
    double c3_v = adcVoltageDecode(c3_raw);
    double c4_v = adcVoltageDecode(c4_raw);

    // Serial.print("V_Cell1="); Serial.print(c1_v);
    // Serial.print("\nV_Cell2="); Serial.print(c2_v);
    // Serial.print("\nV_Cell3="); Serial.print(c3_v);
    // Serial.print("\nV_Cell4="); Serial.print(c4_v);
    // Serial.print("\n");

    // Get individual cell voltages (not absolute)
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
    double v = raw * (VREF / ADC_MAX_COUNTS);
    // Serial.print("Current_sense_v = "); Serial.println(v);
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
// void coulombCount() {
//     // Define positive current as discharging
//     double delta_mAh = 1000 * I_discharge * ((double) I_INTERVAL / 3600.0);
//     // double delta_mAh = 10;
//     Serial.print("delta Ah ="); Serial.println(delta_mAh);
 
//     cell_SOCs.c1_soc -= delta_mAh / C1_CAP;
//     cell_SOCs.c2_soc -= delta_mAh / C2_CAP;
//     cell_SOCs.c3_soc -= delta_mAh / C3_CAP;
//     cell_SOCs.c4_soc -= delta_mAh / C4_CAP;
//     // SoC -= delta_Ah / PACK_CAP;

//     // if (SoC > 1.0) SoC = 1.0;
//     // if (SoC < 0.0) SoC = 0.0;
// }

void coulombCountCell(int cell_idx) {
    // Define positive current as discharging
    double delta_mAh = 1000 * I_discharge * ((double) I_INTERVAL / 3600.0);
    // double delta_mAh = 10;
    // Serial.print("delta Ah="); Serial.println(delta_mAh);
 
    if (cell_idx == 0) {
        cell_SOCs.c1_soc -= delta_mAh / C1_CAP;
    } else if (cell_idx == 1) {
        cell_SOCs.c2_soc -= delta_mAh / C2_CAP;
    } else if (cell_idx == 2) {
        cell_SOCs.c3_soc -= delta_mAh / C3_CAP;
    } else {
        cell_SOCs.c4_soc -= delta_mAh / C4_CAP;
    }
    
    // SoC -= delta_Ah / PACK_CAP;

    // if (SoC > 1.0) SoC = 1.0;
    // if (SoC < 0.0) SoC = 0.0;
}

// ==========================================================
// BALANCING
// ==========================================================
void balanceHandler() {
    // Find lowest cell voltage
    double minSOC = cell_SOCs.c1_soc;
    if (cell_SOCs.c2_soc < minSOC) minSOC = cell_SOCs.c2_soc;
    if (cell_SOCs.c3_soc < minSOC) minSOC = cell_SOCs.c3_soc;
    if (cell_SOCs.c4_soc < minSOC) minSOC = cell_SOCs.c4_soc;

    // Connstruct array from cell voltages
    double cellValues[4] = {
        cell_SOCs.c1_soc,
        cell_SOCs.c2_soc,
        cell_SOCs.c3_soc,
        cell_SOCs.c4_soc
    };

    // Update balance state for each cell
    for (int i = 0; i < 4; i++) {
        double cv = cellValues[i];

        // Turn ON if above start threshold and currently off
        if (!balanceActive[i] && cv > minSOC + BALANCE_START_SOC_TH) {
            balanceActive[i] = true;
            // Serial.print("Balancing cell #"); Serial.print(i+1); Serial.print("\n");
        }

        // Turn OFF if cell has dropped close to min value
        if (balanceActive[i] && cv <= minSOC + BALANCE_STOP_SOC_TH) {
            balanceActive[i] = false;
            // Serial.print("Stopping balancing for cell #"); Serial.print(i+1); Serial.print("\n");
        }

        if (balanceActive[i]) {
            coulombCountCell(i);
        }
    }

    // Apply balancing outputs
    digitalWrite(PIN_BAL1, balanceActive[0] ? HIGH : LOW);
    digitalWrite(PIN_BAL2, balanceActive[1] ? HIGH : LOW);
    digitalWrite(PIN_BAL3, balanceActive[2] ? HIGH : LOW);
    digitalWrite(PIN_BAL4, balanceActive[3] ? HIGH : LOW);
}

void balanceDriver() {
    balanceEnable = digitalRead(PIN_BALANCE_ENABLE);

    if (balanceEnable) {
        balanceHandler();
        // Serial.println("Cell balancing enabled");
    } else {
        // Ensure all FETs are OFF if balancing disabled
        // Serial.println("Cell balancing disabled");
        for (int i = 0; i < 4; i++) balanceActive[i] = false;

        digitalWrite(PIN_BAL1, LOW);
        digitalWrite(PIN_BAL2, LOW);
        digitalWrite(PIN_BAL3, LOW);
        digitalWrite(PIN_BAL4, LOW);
    }
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

    // Print for UI display
    Serial.print(cellVoltages.c1_mv); Serial.print(",");
    Serial.print(cellVoltages.c1_mv); Serial.print(",");
    Serial.print(cellVoltages.c1_mv); Serial.print(",");
    Serial.print(cellVoltages.c1_mv); Serial.print(",");
    Serial.print(packCurrent, 3);     Serial.print(",");
    Serial.print(tempC_int);          Serial.print(".");
    Serial.print(tempC_frac);         Serial.print(",");
    Serial.print(soc_pct);            Serial.print(",");
    Serial.print(adcFET);             Serial.print(",");
    Serial.println(balanceFET);     Serial.print(",");




    Serial.print(cell_SOCs.c1_soc); Serial.print(",");
    Serial.print(cell_SOCs.c2_soc); Serial.print(",");
    Serial.print(cell_SOCs.c3_soc); Serial.print(",");
    Serial.print(cell_SOCs.c4_soc); Serial.print(",");
    // Serial.println(balanceActive[0] == true ? 1: 0);
    // Serial.println(balanceActive[1] == true ? 1: 0);
    // Serial.println(balanceActive[2] == true ? 1: 0);
    // Serial.println(balanceActive[3] == true ? 1: 0);
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
        // coulombCount();
        balanceDriver();
        update();
    }
}
