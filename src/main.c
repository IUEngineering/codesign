/*
 * Codesign DDS.c
 *
 * Created: 21-4-2021 19:06:08
 * Author : Melvin
 */ 

#define F_CPU				32000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stddef.h>
#include <avr/pgmspace.h>

#include <stdio.h>
#include <util/delay.h>


#include "clock.h"
#include "serialF0.h"

#define DAC_MAX				4096
#define SINE_TABLE_MAX		DAC_MAX-1
#define AMPLITUDE_1V		(((double)DAC_MAX/VCC)/1.09)
#define VCC                 3.30
#define DAC_REV				VCC
#define DC_OFFSET           (double)DAC_MAX/(VCC)
							
#define FREQUENCY			1000
#define SAMPLE_FREQUENCY    40000
#define PHASE_ACCUMULATOR   //round((double)(FREQUENCY/SAMPLE_FREQUENCY) * UINT16_MAX)// doet niet

// Define Delta-Sigma modulation parameters
#define DELTA_SIGMA_THRESHOLD 0.5 // Threshold for Delta-Sigma modulation
#define DELTA_SIGMA_BITS 2 // Number of bits for Delta-Sigma output

int16_t delta_sigma_accumulator = 0; // Use signed integer for multi-level Delta-Sigma modulation
uint8_t delta_sigma_output = 0; // Store multi-bit Delta-Sigma output

							
#define _USE_MATH_DEFINES	

void InitTimer(void);
void InitDAC(void);

int16_t sinusTabel[DAC_MAX];

int main(void)
{
	PORTF.DIRSET = PIN0_bm; // Debug led
	PORTC.DIRSET = PIN0_bm; // Debug led

	init_clock();
	
// 	init_stream(F_CPU, 115200);
	PORTF.OUTSET = PIN0_bm;
	for (uint16_t i = 0; i < DAC_MAX; i++){ // misschien groterre resolutie? Dus 1/4e sinus maken en dan steeds een kwart inladen
		sinusTabel[i] = (int16_t) round(AMPLITUDE_1V * sin(2 * M_PI * i / DAC_MAX));
	};
	
	PORTF.OUTCLR = PIN0_bm;

	InitTimer();
	InitDAC();

	PMIC.CTRL     |= PMIC_LOLVLEN_bm;
	sei();
		
    while (1) {
		asm ("nop");
    }
};

ISR(TCD0_OVF_vect){
	static uint16_t phase = 0;
	PORTD.OUTTGL = PIN0_bm;
	
	//Phase accumulator
	phase += 1639; //107374182;//

	// Phase to amplitude
	uint16_t DDS_out = sinusTabel[(phase>>4)];
	
	int16_t delta_sigma_input = (DDS_out >>12) - 1; // Scale DDS output to range [-0.5, 0.5]
	delta_sigma_accumulator += (int16_t)(delta_sigma_input * (1 << (DELTA_SIGMA_BITS - 1))); // Scale input to match Delta-Sigma output range
	delta_sigma_output = delta_sigma_accumulator >= 0 ? (1 << (DELTA_SIGMA_BITS - 1)) : 0; // Set MSB of Delta-Sigma output based on accumulator sign
	delta_sigma_accumulator -= delta_sigma_output;

	// Scale Delta-Sigma output to DAC output range
	uint16_t output = DDS_out + delta_sigma_output * (SINE_TABLE_MAX / (1 << (DELTA_SIGMA_BITS - 1)));
	
 	DACB.CH1DATA = DC_OFFSET + DDS_out;//output;//sinusTabel[(phase>>4)];
 	DACB.CH0DATA = DC_OFFSET - DDS_out;//output;//sinusTabel[(phase>>4)];
};

uint8_t readCalibrationByte(uint8_t index)
{
	uint8_t result;

	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return result;
};

void InitDAC(void){
	
	DACB.CH0GAINCAL   = readCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, DACB0GAINCAL));
	DACB.CH0OFFSETCAL = readCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, DACB0OFFCAL));
	DACB.CH1GAINCAL   = readCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, DACB1GAINCAL));
	DACB.CH1OFFSETCAL = readCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, DACB1OFFCAL));
	 
	DACB.CTRLB = DAC_CHSEL_DUAL_gc | DAC_CH0TRIG_bm | DAC_CH1TRIG_bm;
	DACB.CTRLC = DAC_REFSEL_AVCC_gc;
	DACB.CTRLA = DAC_CH0EN_bm | DAC_CH1EN_bm | DAC_ENABLE_bm;
	
	DACB.EVCTRL = DAC_EVSEL_0_gc;
};

void InitTimer(void){ // @40khz
	
	PORTD.DIRSET  = PIN0_bm; // Debug pin

	TCD0.CTRLB    = TC_WGMODE_SINGLESLOPE_gc;
	TCD0.CTRLA    = TC_CLKSEL_DIV1_gc;
	TCD0.INTCTRLA = TC_OVFINTLVL_LO_gc;
	EVSYS.CH0MUX  = EVSYS_CHMUX_TCD0_OVF_gc;
	
	TCD0.PER      = 800;//640;
};