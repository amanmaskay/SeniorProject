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
	 *
	 * Setup PB1 for PWM mode.
	 */
	TCCR1A |= _BV(COM1A1) | _BV(WGM11) ;
	TCCR1A &= ~_BV(WGM10) ;
	TCCR1B |= _BV(CS10) | _BV(WGM12) | _BV(WGM13);// | _BV(WGM13);
	int top = 0x3F;
	ICR1 = top;
	
	//Initialize some variables
	float DC = 0;
	int OC = top*DC/100;	
	float V_ref = 4.92;
	
	int OCR_max = 0.90*top;
	int OCR_min = 0.04*top;
	OCR1A = OC;
	_delay_ms(50);
	
	float kp = 0.7;//2.6;//0.8;//2.8;//0.85;
	float ki = 8;//300.0;//8;//10;//0.5;
	float kd = 0.0008;//0.00003;//0.003;//8;//.9;

	float err=5;
	float int_err=0;
	float der_err=0;

	float del_ocp=0.0;
	float del_oci=0.0;
	float del_ocd=0.0;
	float Vbat_th = 0.0;
	
	
	float temp = 0;
	float del = 1;	
	float v_dvd1 = 270/950.0;
	float v_dvd2 = 0.5;
	
	// main loop
	while(1){	

	uint16_t Vbat_ADC;
		//sample battery voltage
		Vbat_ADC = adc_read(ADC_PIN1);
		float Vbat_volts = Vbat_ADC*1.8/0x3FF;
		float Vbat_Scaled = Vbat_volts/v_dvd2;

		//Sample Output voltage
		uint16_t Vout_ADC = adc_read(ADC_PIN0);
		float Vout_volts = Vout_ADC*1.8/0x3FF;
		float Vout_Scaled = Vout_volts/v_dvd1;
		
		//calculate error, its intgral and derivative
		temp = err;
		err = V_ref-Vout_Scaled;
		der_err = (err-temp)*1000.0/(del);
		int_err += err*del/1000.0;
		
		//if battery voltage below threshold shutoff output and exit maine loop
		if ((Vbat_Scaled<Vbat_th)&&((err<0.25))){
			_delay_ms(1000);
			OCR1A = 0.0;
			return 0;
			
		}
		Vbat_th = 1.96;
		
		// calculate required duty ratio
		del_ocp = err*kp;
		del_oci = int_err*ki;
		del_ocd = der_err*kd;
		OC+=(del_ocp+del_oci+del_ocd);	
	
		
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
	
	//Double sample to avoid garbage values
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;
 	ADCSRA |= _BV(ADSC);
 	while ( (ADCSRA & _BV(ADSC)) );
	/* Finally, we return the converted value to the calling function. */
	return ADC;
}
