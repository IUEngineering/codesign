#define F_CPU 32000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stddef.h>

#include "clock.h"
#include "sine.h"

#include "serialF0.h"

#define VCC         3.30
#define DAC_MAX     4096
#define AMPLITUDE   ((double)DAC_MAX / 4.6)
#define OFFSET      (double)DAC_MAX/(VCC)
#define SAMPLES     DAC_MAX


void initTimer(void);
void initDAC(void);

volatile int16_t i = 0;		

int main(void) {

    init_clock();
    init_stream(F_CPU, 152000);
    
    initTimer();
    initDAC();

    PMIC.CTRL |= PMIC_LOLVLEN_bm;
    sei();

    for (uint32_t i = 0; i < SAMPLES; i++) {
        lookupTable[i] = (uint16_t)(OFFSET + AMPLITUDE * sin(2.0f * M_PI * i / SAMPLES));
    }
    printf("\n};");
    
    while (1 {
		
	});
}

ISR(TCD0_OVF_vect) {
    i += 41; 
    
    if (i > SAMPLES - 1){
        i -= SAMPLES;
    }
    
    DACB.CH0DATA =  lookupTable[i];
    DACB.CH1DATA =  lookupTable[(SAMPLES-1-i)];
}


uint8_t readCalibrationByte(uint8_t index) {
    uint8_t result;

    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    result = pgm_read_byte(index);
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;

    return result;
}

void initDAC(void) {
    DACB.CH1GAINCAL   = readCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, DACB1GAINCAL) );
	DACB.CH1OFFSETCAL = readCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, DACB1OFFCAL) );
    DACB.CH0GAINCAL   = readCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, DACB0GAINCAL));
    DACB.CH0OFFSETCAL = readCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, DACB0OFFCAL));
    DACB.CTRLB = DAC_CHSEL_DUAL_gc | DAC_CH0TRIG_bm | DAC_CH1TRIG_bm;
    DACB.CTRLC = DAC_REFSEL_AVCC_gc;
    DACB.CTRLA = DAC_CH0EN_bm | DAC_CH1EN_bm | DAC_ENABLE_bm;
    
    DACB.EVCTRL = DAC_EVSEL_0_gc;
};

void initTimer(void){
    
    PORTD.DIRSET  = PIN1_bm;

    TCD0.CTRLB    = TC_WGMODE_SINGLESLOPE_gc;
    TCD0.CTRLA    = TC_CLKSEL_DIV1_gc;
    TCD0.INTCTRLA = TC_OVFINTLVL_LO_gc; 
    EVSYS.CH0MUX  = EVSYS_CHMUX_TCD0_OVF_gc;
    
    TCD0.PER      = 320; 
}