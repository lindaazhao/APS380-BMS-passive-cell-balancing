/*
 * File:   main.c
 * Author: winst
 *
 * Created on November 22, 2025, 7:17 PM
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/adc/adc.h"
#include "mcc_generated_files/uart/eusart1.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/i2c_host/mssp1.h"
#include <xc.h>

#define VREF 5.0
#define R_DIV 5.163265306
#define ADC_MAX_COUNTS 1023.0
#define I_SENSE_V_OFFSET 2.5
#define I_SENSE_V_PER_A 0.045
#define T_SENSE_ADDR 0x48
#define T_SENSE_REG 0x00
#define PACK_CAP 2.5
#define I_INTERVAL 5

typedef struct Voltages{
    uint16_t c1_mv;
    uint16_t c2_mv;
    uint16_t c3_mv;
    uint16_t c4_mv;
} Voltages;

typedef struct FETs{
    unsigned int adcFET;
    int balanceFET1;
    int balanceFET2;
    int balanceFET3;
    int balanceFET4;
} FETs;

void initialize();
void adcGATESHigh();
void adcGATESLow();
double adcVoltageDecode(adc_result_t raw);
void adcTapVoltages();
void adcCurrentSense();
static bool I2C1_WriteRead_Blocking(uint8_t addr, uint8_t *wbuf, size_t wlen, uint8_t *rbuf, size_t rlen);
static int16_t tempDecode();
void i2cTempSense();
static void coulombCount();
void update();

Voltages cellVoltages;
double packCurrent;
int16_t tempC_int;
int16_t tempC_frac;
static double SoC;
FETs FET;

void initialize()
{
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    FET.adcFET = 0;
    SoC = 1.0
    printf("C1=n/a mV  C2=n/a mV  C3=n/a mV  C4=n/a mV  I=n/a A  T=n/a C  SoC=n/a\r\n");
}

void adcGATESHigh()
{
    IO_RA2_SetHigh();
    IO_RA7_SetHigh();
    IO_RB0_SetHigh();
    IO_RB2_SetHigh();
    FET.adcFET = 1;
}

void adcGATESLow()
{
    IO_RA2_SetLow();
    IO_RA7_SetLow();
    IO_RB0_SetLow();
    IO_RB2_SetLow();
    FET.adcFET = 0;
}

double adcVoltageDecode(adc_result_t raw)
{
    double v_adc  = (raw / ADC_MAX_COUNTS) * VREF;
    double v_cell = v_adc * R_DIV;
    return v_cell;
}

void adcTapVoltages()
{
    adc_result_t c1_raw = ADC_ChannelSelectAndConvert(IO_RA3);
    adc_result_t c2_raw = ADC_ChannelSelectAndConvert(IO_RA6);
    adc_result_t c3_raw = ADC_ChannelSelectAndConvert(IO_RB1);
    adc_result_t c4_raw = ADC_ChannelSelectAndConvert(IO_RB3);
    double c1_v = adcVoltageDecode(c1_raw);
    double c2_v = adcVoltageDecode(c2_raw);
    double c3_v = adcVoltageDecode(c3_raw);
    double c4_v = adcVoltageDecode(c4_raw);
    c4_v = c4_v - c3_v;
    c3_v = c3_v - c2_v;
    c2_v = c2_v - c1_v;
    cellVoltages.c1_mv = (uint16_t)(c1_v * 1000.0 + 0.5);
    cellVoltages.c2_mv = (uint16_t)(c2_v * 1000.0 + 0.5);
    cellVoltages.c3_mv = (uint16_t)(c3_v * 1000.0 + 0.5);
    cellVoltages.c4_mv = (uint16_t)(c4_v * 1000.0 + 0.5);
}

void adcCurrentSense()
{
    adc_result_t raw = ADC_ChannelSelectAndConvert(IO_RA1);
    double v = (raw / ADC_MAX_COUNTS) * VREF;
    //Clamp check?
    double v_delta = v - I_SENSE_V_OFFSET;
    packCurrent = v_delta / I_SENSE_V_PER_A;
}

static bool I2C1_WriteRead_Blocking(uint8_t addr, uint8_t *wbuf, size_t wlen, uint8_t *rbuf, size_t rlen)
{
    if (!I2C1_WriteRead((uint16_t)addr, wbuf, wlen, rbuf, rlen)){
        return false;
    }
    while (I2C1_IsBusy()){
        //
    }
    i2c_host_error_t err = I2C1_ErrorGet();
    return (err == I2C_ERROR_NONE);
}

static int16_t tempDecode()
{
    uint8_t wbuf[1];
    uint8_t rbuf[2];

    wbuf[0] = T_SENSE_REG;
    bool ok = I2C1_WriteRead_Blocking(T_SENSE_ADDR, wbuf, 1, rbuf, 2);
    if (!ok){
        return 0;
    }
    int16_t raw16 = ((int16_t)rbuf[0] << 8) | rbuf[1];
    raw16 >>= 4;
    if (raw16 & 0x0800){
        raw16 |= 0xF000;
    }
    return raw16;
}

void i2cTempSense()
{
    int16_t counts = tempDecode();
    double tC = (double)counts * 0.0625;
    int16_t tempCx10 = (int16_t)(tC * 10.0);
    tempC_int = tempCx10 / 10;
    tempC_frac = tempCx10 % 10;
    if (tempC_frac < 0){tempC_frac = -tempC_frac;}
}

static void coulombCount()
{
    double delta_Ah = packCurrent * ((double)I_INTERVAL / 3600.0);

    SoC -= delta_Ah / PACK_CAP;

    if (SoC > 1.0){SoC = 1.0;}
    if (SoC < 0.0){SoC = 0.0;}
}

void update(){
    printf("C1=%u mV  C2=%u mV  C3=%u mV  C4=%u mV  I=%.2f A  T=%d.%d C  SoC=%d%%\r\n",
            cellVoltages.c1_mv,
            cellVoltages.c2_mv,
            cellVoltages.c3_mv,
            cellVoltages.c4_mv,
            packCurrent,
            tempC_int,
            tempC_frac,
            (int)(SoC * 100.0 + 0.5));
}

void balanceCells(){
    uint16_t max = -100;
    int maxCell = 0;
    if (cellVoltages.c1_mv > max){maxCell = 1;}
    if (cellVoltages.c2_mv > max){maxCell = 2;}
    if (cellVoltages.c3_mv > max){maxCell = 3;}
    if (cellVoltages.c4_mv > max){maxCell = 4;}
    if (maxCell == 1){}
}

/*IO_RC0_SetLow();
IO_RC1_SetLow();
IO_RC2_SetLow();
IO_RC5_SetLow();
IO_RC0_SetHigh();
IO_RC1_SetHigh();
IO_RC2_SetHigh();
IO_RC5_SetHigh();*/

void main(void)
{    
    unsigned int ms_counter = 0;

    while(1)
    {
        if (PIR0bits.TMR0IF)
        {
            PIR0bits.TMR0IF = 0;
            ms_counter++;

            if (ms_counter >= 4000 && FET.adcFET == 0)
            {
                adcGATESHigh();
            }
            
            if (ms_counter >= 5000)
            {
                ms_counter = 0;

                adcTapVoltages();

                adcCurrentSense();

                i2cTempSense();

                coulombCount();

                update();
                
                adcGATESLow();
            }
        }
    }
}
