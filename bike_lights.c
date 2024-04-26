#include <avr/io.h>

#define F_CPU 8000000UL

#include <avr/interrupt.h>
#include <util/delay.h>

//PB0 - SRCLK   PB3 - Brake
//PB1 - RCLK    PB4 - Signal Direction
//PB2 - SER     PB5 - Reset

#define signalflag   7
#define brake        3
#define left_bit     2
#define right_bit    1
#define flash_bit    0
#define SERCLKpin    PB0
#define REGCLKpin    PB1
#define SERpin       PB2
#define brakepin     PB3
#define signalpin    PB4
#define SER_1        PORTB |= (1 << SERpin)
#define SER_0        PORTB &= ~(1 << SERpin)
#define SERCLK_1     PORTB|= (1 << SERCLKpin)
#define SERCLK_0     PORTB &= ~(1 << SERCLKpin)
#define REGCLK_1     PORTB|= (1 << REGCLKpin)
#define REGCLK_0     PORTB &= ~(1 << REGCLKpin)
#define ledSpeed 250

volatile uint8_t shiftregister = 0b00000000;  //Output of shift register

// The lower bits are used to determine if the LEDs are on or off
// while they blink. The 2 highest bits are switch upon interrupt
// for clock pulses.

volatile uint16_t overflows = 0;
volatile int state = 0;
volatile uint16_t brake_value;
volatile uint16_t signal_value;
volatile uint8_t signal_state = 0;

void brake_signal();
void turn_signal();
void init_timer();
void init_ADC();
void init_interrupts();
void SR(uint8_t SER);
uint16_t read_brakes();
uint16_t read_signalswitch();

int main() {
  init_interrupts();
  init_timer();
  init_ADC();
  init_ports();

  while (1) 
  {
    turn_signal();
    brake_signal();
    SR(shiftregister);
  }
}

void brake_signal() 
{
  if(read_brakes() <= 588)
    shiftregister |= (3 << 6) | (3 << 0);
  else
    shiftregister &= ~(3 << 6) & ~(3 << 0);
}

void turn_signal() 
{
   if(read_signalswitch() > 552)
     signal_state = 2;
   else if(read_signalswitch() >= 471)
     signal_state = 1;
   else if(read_signalswitch() < 471)
     signal_state = 0;
}

void SR(uint8_t SER) 
{
  REGCLK_0;
  SERCLK_0;
  
  for(int i = 0; i < 8; i++)
  {
    _delay_ms(1);
    
    if(SER & (1 << i))
      SER_1;
    else
      SER_0;
    
    _delay_ms(1);
    SERCLK_1;

    _delay_ms(1);
    SERCLK_0;
  }
  
  _delay_ms(1);
  REGCLK_1;
}

void init_ports() 
{
  DDRB |= (1 << SERCLKpin) | (1 << REGCLKpin) | (1 << SERpin);
}

void init_timer() 
{
  TCCR1 |= (1 << CS12) | (1 << CS11);  //prescale of 32
}

void init_ADC() 
{
  ADCSRA |= (1 << ADEN);                  //enable ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1);  //prescale of 64 50kHz<(8MHz/64)<250kHz
}

void init_interrupts() 
{
  TIMSK |= (1 << TOIE1);  //Timer/counter 1 overflow interrupt enable

  sei();  //enable interrupts
}

uint16_t read_brakes() 
{
  uint16_t brake_result;

  ADMUX |= (1 << MUX1) | (1 << MUX0);  //measure from brakepin PB3
  ADCSRA |= (1 << ADSC);               //start conversion

  while (ADCSRA & (1 << ADSC)) 
  {
    continue;
  }

  brake_result = ADC;
  return brake_result;
}

uint16_t read_signalswitch() 
{
  uint16_t signalswitch_result;

  ADMUX |= (1 << MUX1);  //measure from brakepin PB4
  ADMUX &= ~(1 << MUX0);
  ADCSRA |= (1 << ADSC);  //start conversion

  while (ADCSRA & (1 << ADSC)) 
  {
    continue;
  }

  signalswitch_result = ADC;
  return signalswitch_result;
}

ISR(TIMER1_OVF_vect) 
{

  // interrupt service routine for timer1 overflow
  // 1 interrupt is approx. 1ms
  
  if(signal_state == 1) //if function_state left flag set
  {
    switch(overflows)
  	{
    	case 250:
    		shiftregister &= ~(15 << 2);
    		break;
    	case 350:
      		shiftregister &= ~(15 << 2);
    		shiftregister |= (8 << 2);
    		break;
    	case 450:
      		shiftregister &= ~(15 << 2);
    		shiftregister |= (12 << 2);
    		break;
    	case 550:
      		shiftregister &= ~(15 << 2);
    		shiftregister |= (14 << 2);
    		break;
    	case 650:
      		shiftregister &= ~(15 << 2);
    		shiftregister |= (15 << 2);
    		break;
    }
  }
  else if(signal_state == 2) //if function_state right flag set
  {
    switch(overflows)
  	{
    	case 250:
    		shiftregister &= ~(15 << 2);
    		break;
    	case 350:
      		shiftregister &= ~(15 << 2);
    		shiftregister |= (1 << 2);
    		break;
    	case 450:
      		shiftregister &= ~(15 << 2);
    		shiftregister |= (3 << 2);
    		break;
    	case 550:
      		shiftregister &= ~(15 << 2);
    		shiftregister |= (7 << 2);
    		break;
    	case 650:
      		shiftregister &= ~(15 << 2);
    		shiftregister |= (15 << 2);
    		break;
    }
  }
  else
  {
    shiftregister &= ~(15 << 2);
    overflows = 0;
  }
  
  if(overflows > 650)
	overflows = 0;
  
  overflows++;
   
}
