#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 4000000

// ---------------------- ADC FUNCTIONS ----------------------
void ADC_Init(void)
{
    // Configure analog pins
    AD1PCFGbits.PCFG2 = 0;   // AN2 -> RA2
    AD1PCFGbits.PCFG3 = 0;   // AN3 -> RA3
    AD1PCFGbits.PCFG5 = 0;   // AN5 -> RB3
    AD1PCFGbits.PCFG6 = 0;   // AN6 -> RB4
    AD1PCFGbits.PCFG7 = 0;   // AN7 -> RB5

    // ADC configuration
    AD1CON1bits.FORM = 0;    // Integer 16-bit
    AD1CON1bits.SSRC = 7;    // Auto-convert
    AD1CON1bits.ASAM = 0;    // Manual sampling

    AD1CON2 = 0;
    AD1CON3bits.ADRC = 0;    // ADC clock from system
    AD1CON3bits.SAMC = 10;   // Auto sample time
    AD1CON3bits.ADCS = 5;    // ADC conversion clock

    AD1CON1bits.ON = 1;      // Turn on ADC
}

uint16_t ADC_Read(uint8_t channel)
{
    AD1CHSbits.CH0SA = channel;   // Select channel
    AD1CON1bits.SAMP = 1;         // Start sampling
    for(volatile int i=0;i<1000;i++); // ~short delay for sampling
    AD1CON1bits.SAMP = 0;         // Start conversion
    while (!AD1CON1bits.DONE);    // Wait for conversion
    return ADC1BUF0;
}

// ---------------------- UART PLACEHOLDER -------------------
void UART_Send(uint16_t v1, uint16_t v2, uint16_t v3, uint16_t v4,
               uint16_t current, uint16_t temperature, uint8_t soc)
{
    // TODO: implement UART transmit
}

// ---------------------- MAIN -------------------------------
void main(void)
{
    // GPIO setup
    TRISAbits.TRISA0 = 0; // Passive balancers
    TRISAbits.TRISA1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC5 = 0;

    TRISAbits.TRISA2 = 1; // ADC inputs
    TRISAbits.TRISA3 = 1;
    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1; // Current sensor

    TRISAbits.TRISA7 = 0; // Voltage taps
    TRISAbits.TRISA6 = 0;
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;

    ADC_Init();

    uint8_t soc = 100;
    const uint16_t capacity_mAh = 2500;
    const float efficiency = 0.95;

    while(1)
    {
        // Enable voltage taps
        LATAbits.LATA7 = 1;
        LATAbits.LATA6 = 1;
        LATCbits.LATC0 = 1;
        LATCbits.LATC1 = 1;

        for(volatile int i=0;i<5000;i++); // short delay (~10ms)

        // Read voltages
        uint16_t v1 = ADC_Read(2);
        uint16_t v2 = ADC_Read(3);
        uint16_t v3 = ADC_Read(5);
        uint16_t v4 = ADC_Read(6);

        // Disable voltage taps
        LATAbits.LATA7 = 0;
        LATAbits.LATA6 = 0;
        LATCbits.LATC0 = 0;
        LATCbits.LATC1 = 0;

        for(volatile int i=0;i<25000;i++); // delay (~50ms)

        // Read current sensor
        current = ADC_Read(7);

        // Read temperature sensor
        uint16_t temperature = ADC_Read(1); // Example: AN1

        // Simple SOC update placeholder (real calculation requires current in mA & time)
        soc = soc + (uint8_t)((current * efficiency) / capacity_mAh);

        // If the soc of any cell is too different from the others, engage the balancers
        uint16_t max_v = v1;
        if(v2 > max_v) max_v = v2;
        if(v3 > max_v) max_v = v3;
        if(v4 > max_v) max_v = v4;
        uint16_t min_v = v1;
        if(v2 < min_v) min_v = v2;
        if(v3 < min_v) min_v = v3;
        if(v4 < min_v) min_v = v4;
        uint16_t delta_v = max_v - min_v;
        const uint16_t balance_threshold = 50; // Example threshold
        if(delta_v > balance_threshold)
        {
            // Engage balancers for cells above average voltage
            uint16_t avg_v = (v1 + v2 + v3 + v4) / 4;
            LATAbits.LATA0 = (v1 > avg_v) ? 1 : 0;
            LATAbits.LATA1 = (v2 > avg_v) ? 1 : 0;
            LATCbits.LATC2 = (v3 > avg_v) ? 1 : 0;
            LATCbits.LATC5 = (v4 > avg_v) ? 1 : 0;
        }
        else
        {
            // Disengage all balancers
            LATAbits.LATA0 = 0;
            LATAbits.LATA1 = 0;
            LATCbits.LATC2 = 0;
            LATCbits.LATC5 = 0;
        }

        // Send data via UART
        UART_Send(v1, v2, v3, v4, current, temperature, soc);
    }
}
