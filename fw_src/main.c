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

unsigned int adcFETState = 0;

typedef struct Voltages{
    uint16_t c1_mv;
    uint16_t c2_mv;
    uint16_t c3_mv;
    uint16_t c4_mv;
} Voltages;

double adcVoltageDecode(adc_result_t raw)
{
    double v_adc  = (raw / ADC_MAX_COUNTS) * VREF;
    double v_cell = v_adc * R_DIV;
    return v_cell;
}

double adcCurrentDecode(adc_result_t raw)
{
    double v = (raw / ADC_MAX_COUNTS) * VREF;
    //Clamp check?
    double v_delta = v - I_SENSE_V_OFFSET;
    double i = v_delta / I_SENSE_V_PER_A;
    return i;
}

Voltages adcTapVoltages()
{
    Voltages cellVoltages;
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
    return cellVoltages;
}

double adcCurrentSense()
{
    adc_result_t i_raw = ADC_ChannelSelectAndConvert(IO_RA1);
    return adcCurrentDecode(i_raw);
}

void adcGATESHigh(){
    IO_RA2_SetHigh();
    IO_RA7_SetHigh();
    IO_RB0_SetHigh();
    IO_RB2_SetHigh();
    adcFETState = 1;
}

void adcGATESLow(){
    IO_RA2_SetLow();
    IO_RA7_SetLow();
    IO_RB0_SetLow();
    IO_RB2_SetLow();
    adcFETState = 0;
}

static int16_t tempDecode(){
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

static double tempCelsius(){
    int16_t counts = tempDecode();
    return (double)counts * 0.0625;
}

static bool I2C1_WriteRead_Blocking(uint8_t addr, uint8_t *wbuf, size_t wlen, uint8_t *rbuf, size_t rlen) //?
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

void main(void)
{    
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();

    unsigned int ms_counter = 0;
    adcFETState = 0;

    while(1){
        if (PIR0bits.TMR0IF){
            PIR0bits.TMR0IF = 0;
            ms_counter++;

            if (ms_counter >= 4000 && adcFETState == 0)
            {
                adcGATESHigh();
            }
            
            if (ms_counter >= 5000)
            {
                ms_counter = 0;

                Voltages cellVoltages = adcTapVoltages();
                double packCurrent = adcCurrentSense();

                printf("C1=%u mV  C2=%u mV  C3=%u mV  C4=%u mV  I=%d mA\r\n",
                        cellVoltages.c1_mv, cellVoltages.c2_mv,
                        cellVoltages.c3_mv, cellVoltages.c4_mv,
                        packCurrent);
                
                adcGATESLow();
            }
        }
    }
}
