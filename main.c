#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>


#define ADC_PIN0			0
#define ADC_PIN1			1

#define	LED_PIN0			PB0
#define	LED_PIN1			PB1

uint16_t adc_read(uint8_t adcx);
int main ()
{
    /**
	 * We will be using OCR1A as our PWM output which is the
	 * same pin as PB1.
	 */
	DDRB |= _BV(DDB1);
	DDRB |= _BV(DDB2);
 	
 	/* Enable the ADC */
	ADCSRA |= _BV(ADEN);
	/**
	 * There are quite a number of PWM modes available but for the
	 * sake of simplicity we'll just use the 8-bit Fast PWM mode.
	 * This is done by setting the WGM10 and WGM12 bits.  We 
	 * Setting COM1A1 tells the microcontroller to set the 
	 * output of the OCR1A pin low when the timer's counter reaches
	 * a compare value (which will be explained below).  CS10 being
	 * set simply turns the timer on without a prescaler (so at full
	 * speed).  The timer is used to determine when the PWM pin should be
	 * on and when it should be off.
	 */
	TCCR1A |= _BV(COM1A1) | _BV(WGM11) ;
	TCCR1A &= ~_BV(WGM10) ;
	TCCR1B |= _BV(CS10) | _BV(WGM12) | _BV(WGM13);// | _BV(WGM13);
	int top = 0x3F;
	ICR1 = top;
	//TCCR1B &= ~(_BV(WGM11) | _BV(WGM10) );
	//TCCR1A |= _BV(COM2A1) | _BV(WGM10);
	//TCCR2B |= _BV(CS20) | _BV(WGM12);

	/**
	 *  This loop is used to change the value in the OCR1A register.
	 *  What that means is we're telling the timer waveform generator
	 *  the point when it should change the state of the PWM pin.
	 *  The way we configured it (with _BV(COM1A1) above) tells the
	 *  generator to have the pin be on when the timer is at zero and then
	 *  to turn it off once it reaches the value in the OCR1A register.
	 *
	 *  Given that we are using an 8-bit mode the timer will reset to zero
	 *  after it reaches 0xff, so we have 255 ticks of the timer until it
	 *  resets.  The value stored in OCR1A is the point within those 255
	 *  ticks of the timer when the output pin should be turned off
	 *  (remember, it starts on).
	 *
	 *  Effectively this means that the ratio of pwm / 255 is the percentage
	 *  of time that the pin will be high.  Given this it isn't too hard
	 *  to see what when the pwm value is at 0x00 the LED will be off
	 *  and when it is 0xff the LED will be at its brightest.
	 */
	/*uint8_t pwm = 0x00;
	bool up = true;
	for(;;) {
 
		OCR1A = pwm;
 
		pwm += up ? 1 : -1;
		if (pwm == 0xff)
			up = false;
		else if (pwm == 0x00)
			up = true;
 
		_delay_ms(10);
	}*/
	float DC = 0;
	int OC = top*DC/100;	
	float V_ref = 5;
	
	int OCR_max = 0.90*top;
	int OCR_min = 0.04*top;
	OCR1A = OC;
	_delay_ms(1000);
	while(1)
	{
		float kp = 0.57;
		float ki = 0.5;
		float kd = .9;
		float del_ocp=0;
		float del_oci=0;
		float del_ocd=0;
		float Vbat_th = 1.92;
		
		float err=0;
		float int_err=0;
		float der_err=0;
		float temp = 0;
		int del = 1;
		float v_dvd1 = 0.283;
		float v_dvd2 = 0.5;
		uint16_t Vout_ADC = adc_read(ADC_PIN0);
		float Vout_volts = Vout_ADC*1.8/0x3FF;
		float Vout_Scaled = Vout_volts/v_dvd1;

		uint16_t Vbat_ADC = adc_read(ADC_PIN1);
		float Vbat_volts = Vbat_ADC*1.8/0x3FF;
		float Vbat_Scaled = Vbat_volts/v_dvd2;
		
		temp = err;
		err = V_ref-Vout_Scaled;
		der_err = (err-temp)/(del/1000);
		int_err += err*del/1000.0;

		if (Vbat_Scaled<Vbat_th){
			//OCR1A = 0.0;
			SMCR &= ~(_BV(SM2)|_BV(SM0));
			SMCR |= _BV(SM1);
			//Vbat_th = 2.25;
			//break;
		}
		
		
		del_ocp = err*kp;
		del_oci = int_err*ki;
		del_ocd = der_err*kd;
		OC+=(del_ocp+del_oci);	
	
		
		if (OC <OCR_min){
			OCR1A =OCR_min;
		}
		else if (OC>OCR_max)
		{
			OCR1A = OCR_max;
		}
		else{
			OCR1A = OC;
		}
		
		
		_delay_ms(del);
	}
	
}
 
uint16_t adc_read(uint8_t adcx) {
	/* adcx is the analog pin we want to use.  ADMUX's first few bits are
	 * the binary representations of the numbers of the pins so we can
	 * just 'OR' the pin's number with ADMUX to select that pin.
	 * We first zero the four bits by setting ADMUX equal to its higher
	 * four bits. */
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;
 
	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);
 
	/* This is an idle loop that just wait around until the conversion
	 * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	 * set above, to see if it is still set.  This bit is automatically
	 * reset (zeroed) when the conversion is ready so if we do this in
	 * a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );
 
	/* Finally, we return the converted value to the calling function. */
	return ADC;
}
