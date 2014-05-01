/*  Created by Francis Wheatley and Brian Fallon
    Class: EC450
    Date: 4/29/14
    Short Description: This program uses an infared detector connected to the
    ADC port to control the motor speed of a helicoptor
*/

#include <msp430.h>
#include "msp430g2553.h"

/* Constant Parameters */
#define LONGFLASH 180

// bitmask for the state light
#define LIGHT 0x10

//lights that help to indicate the status of commands
#define SPEEDUP 0x01
#define OFF 0x02
#define SLOWDOWN 0x01

// and for input buttons
#define BUTTONS 0x0E
#define FLIGHTBUTTON 0x02
#define PROXIMITYBUTTON 0x04
#define ABORTBUTTON 0x08

// for the adc port
#define ADC_INPUT_BIT_MASK 0x20
#define ADC_INCH INCH_5

asm(" .length 10000");
asm(" .width 132");

/* Global Variables */
unsigned int currentValue;
unsigned int state;
unsigned int flash_counter = LONGFLASH; // down counter to next flash
unsigned int startingValue;
unsigned int ad_count = 0;
unsigned int bounce_count = 0;
unsigned int wait = 1;
unsigned int calib_low_val = 0;
unsigned int calib_high_val = 0;

/* =============== procedures and interrupt handlers =============== */

/*
* Initializers
*/
void init_adc(void);

void init_gpio(){
// led pins as outputs with led's off
P1OUT &= ~(LIGHT + SLOWDOWN); // lower pins
P1DIR |= (LIGHT + SLOWDOWN);  // output pins

P1DIR &= ~BUTTONS ;     // input pin
P1OUT |= BUTTONS;	   // pullup on input pin
P1REN |=BUTTONS;	 // enable the pullup on the buttons
P1IE |= BUTTONS;	    // enable interrupts on button pin

P2OUT &= ~(SPEEDUP + OFF);
P2DIR |= (SPEEDUP + OFF);


}


void reset_button_input_state(){
state=0;	 // reset state variable
P1IES |= BUTTONS; //  set to interrupt on rising going edge
P1IFG &= ~BUTTONS; // clear interrupt flag
}

void init_wdt(){ // setup WDT
  WDTCTL =(WDTPW + // (bits 15-8) password
                   // bit 7=0 => watchdog timer on
                   // bit 6=0 => NMI on rising edge (not used here)
                   // bit 5=0 => RST/NMI pin does a reset (not used here)
           WDTTMSEL + // (bit 4) select interval timer mode
           WDTCNTCL +  // (bit 3) clear watchdog timer counter
            0 // bit 2=0 => SMCLK is the source
            +1 // bits 1-0 = 01 => source/8K
            );
  IE1 |= WDTIE;	 // enable the WDT interrupt (in the system interrupt register IE1)
}


void main(void){


  BCSCTL1 = CALBC1_8MHZ;	// set 8Mhz calibration for clock
  DCOCTL  = CALDCO_8MHZ;

  TACCTL1 = OUTMOD_7;                  // TA0CCR1 reset/set -- high voltage below count and
                                          // low voltage when past

  TACTL = TASSEL_2 + MC_1;             // Timer A control set to SMCLK clock TASSEL_2, 1MHz
                                          // and count up mode MC_1

  P1DIR |= BIT6;                        // Green LED for output

  P1SEL |= BIT6;                        // Green LED Pulse width modulation

  TACCR0 = 100;                       // PWM period

  TACCR1 = 90;                          // PWM duty cycle, time cycle on vs. off, on 90/100 initially

  init_wdt();
  init_gpio();
  init_adc();
  reset_button_input_state();
_bis_SR_register(GIE+LPM0_bits);  // enable interrupts and also turn the CPU off!
}


// ==== GPIO Input Interrupt Handler =======

interrupt void button_handler(){

  // buttons with debouncing
  if (!wait)
  {
    if (P1IFG & FLIGHTBUTTON) // if a button is pushed twice then it aborts
    {
      if (state == 1 || state == 2) // abort
      {
        TACCR1 -= 3;
      }
      else
      {
        P1OUT ^= LIGHT;
        state = 1; // next state for button interrupt
        calib_low_val = ADC10MEM;
        startingValue = calib_low_val;
      }
      wait = 1;
    }

    else if (P1IFG & PROXIMITYBUTTON) // if a button is pushed twice then it aborts
    {
      if (state == 1 || state == 2) // abort
      {
        TACCR1 += 1;
      }
      else if (calib_low_val != 0)
      {
        state = 2;
        calib_high_val = ADC10MEM;
      }
      else
      {
       state = 0;
      }
     wait = 1;

    }

    else if (P1IFG & ABORTBUTTON)
    {
      state = 0;
      wait = 1;
    }

    else // in the event that something stange does on and the other conditions are not met
    {
      state = 0;
      wait = 1;
    }

    if (state == 0) // if the state is zero, turn the LED off
    {
      TACCR1 = 90;
      P1OUT &= ~(LIGHT + SLOWDOWN);
      P2OUT &= ~(SPEEDUP + OFF);
    }

    P1IFG &= ~BUTTONS; // reset the gpio interrupt flag for the button pin!
  }
}
ISR_VECTOR(button_handler, ".int02")



// the initialization of the adc port
void init_adc()
{
  ADC10CTL1= ADC_INCH	//input channel 4
 			  +SHS_0 //use ADC10SC bit to trigger sampling
 			  +ADC10DIV_4 // ADC10 clock/5
 			  +ADC10SSEL_0 // Clock Source=ADC10OSC
 			  +CONSEQ_0; // single channel, single conversion
 			  ;
  ADC10AE0=ADC_INPUT_BIT_MASK; // enable A4 analog input
  ADC10CTL0= SREF_0	//reference voltages are Vss and Vcc
 	          +ADC10SHT_3 //64 ADC10 Clocks for sample and hold time (slowest)
 	          +ADC10ON	//turn on ADC10
 	          +ENC		//enable (but not yet start) conversions
 	          +ADC10IE  //enable interrupts
 	          ;
}

// the adc_handler controls the speed of the rotor based on the reading from
// the value in ADC10MEM
void interrupt adc_handler()
{
  // read in the current value from memory
  currentValue = ADC10MEM;

  // the current value should never go below the calibrated starting value
  if (currentValue < startingValue)
  {
     currentValue = startingValue;
  }

  // the calibrated high value gets set to the highest value seen by the port
  /*if (calib_high_val < ADC10MEM)
  {
    calib_high_val = ADC10MEM;
  }*/

  if (currentValue > calib_high_val)
  {
	  currentValue = calib_high_val;
  }


  // helps to regulate how quickly the speeds can change
  ad_count++;
  ad_count %= 100;


  // the logic for adjusting the PWM the is output to the controller
  if (ad_count == 0 && state == 2)
  {
    if (currentValue - startingValue < (.05*(calib_high_val - calib_low_val))) // turn off the rotors
    {
      TACCR1++;
      P2OUT |= OFF;
      P1OUT |= SLOWDOWN;
    }
    else if (currentValue - startingValue >= (.05*(calib_high_val - calib_low_val)) && currentValue - startingValue <= (.60*(calib_high_val - calib_low_val))) // slow down the rotors
    {
      TACCR1++;
      P1OUT ^= SLOWDOWN;
      P2OUT &= ~(SPEEDUP);
      P2OUT &= ~(OFF);
    }
    else if (currentValue - startingValue > (.85*(calib_high_val - calib_low_val))) // speed up the rotors
    {
      TACCR1--;
      P2OUT ^= SPEEDUP;
      P1OUT &= ~(SLOWDOWN);
      P2OUT &= ~(OFF);
    }
    else // hold the current speed and turn off lights
    {
      P2OUT ^= OFF;
      P1OUT &= ~(SLOWDOWN);
      P2OUT &= ~(SPEEDUP);
    }

    // prevents the motor from being slowed down too much or sped up too much
    if (TACCR1 > 90)
    {
      TACCR1 = 90;
    }
    if (TACCR1 < 25)
    {
      TACCR1 = 25;
    }

  }

}
ISR_VECTOR(adc_handler, ".int05")


// watchdog timer handler
interrupt void WDT_interval_handler()
{

  ADC10CTL0 |= ADC10SC;  // trigger a conversion

  // the debouncing for the buttons
  if (wait)
  {
    bounce_count++;
    if (bounce_count == 100)
    {
      wait = 0;
      bounce_count = 0;
    }
  }

  if (state == 0)
  {
        //do nothing but wait
  }

  else if (state == 1)
  {
        // do the flight sequence
        // we had spoken about implementing an automated flight sequence
        // this would be a future improvement that we are intested in working
        // to implement for the future
  }

  else if (state == 2) // this will be the proximity sensor mode
  {
    if (--flash_counter == 0) // is it time to flash?
    {
      P1OUT ^= LIGHT; // toggle one or more led's
      flash_counter=LONGFLASH;
    }

  }

  else // the fall through in the event that something strange happens
  {
    state = 0;
  }


}
// DECLARE function WDT_interval_handler as handler for interrupt 10
// using a macro defined in the msp430g2553.h include file
ISR_VECTOR(WDT_interval_handler, ".int10")
