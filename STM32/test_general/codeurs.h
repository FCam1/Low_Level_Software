#ifndef _MODULE_CODEUR_H
#define _MODULE_CODEUR_H

/*---------------Based on Example : HardTimerAsEncoder.ino-------------

 * Hardware Timer as an Encoder interface. 
 * 
 * The STM32 Timers have the possibility of being used as an encoder interface. 
 * This can be both a quadrature encoder (mode 3) or pulse encoder with a signal to give direction (modes 1 and 2). 
 * The default behavior is for quadrature encoder. 
 * 
 * To avoid overflowing the encoder (which may or may not happen (although with 16 bits, it's likely), the following code 
 * will interrupt every time the number of pulses that each revolution gives to increment/decrement a variable (ints). 
 * 
 * This means that the total number of pulses given by the encoder will be (ints * PPR) + Timer.getCount()
 * 
 * Attached is also a bit of code to simulate a quadrature encoder. 
 * To test this library, make the connections as below: 
 * 

 * COUNTING DIRECTION: 
 * 0 means that it is upcounting, meaning that Channel A is leading Channel B
 * 
 * EDGE COUNTING: 
 * 
 * mode 1 - only counts pulses on channel B
 * mode 2 - only counts pulses on Channel A
 * mode 3 - counts on both channels. 
 * 
*/

#include "variables.h"

//Pulses per revolution
#define PPR   2048

void setupCodeurs() {

//define the Timer channels as inputs. (5V Tolerant)
///////////////////////Timer1////////////////////////////////
  pinMode(PA8, INPUT_PULLUP);  //channel A
  pinMode(PA9, INPUT_PULLUP);  //channel B
/////////////////////////////////////////////////////////////////

///////////////////////Timer2////////////////////////////////
//Remap
//No remap or =AFIO_REMAP_TIM2_PARTIAL_2
//afio_remap(AFIO_REMAP_TIM2_PARTIAL_2); //for FT on Timer 2
//afio_remap(AFIO_REMAP_TIM2_FULL); //for FT on Timer 2
  afio_remap(AFIO_REMAP_TIM2_PARTIAL_1); //for FT on Timer 2
  disableDebugPorts();// Ports partagés
  pinMode(PA15, INPUT_PULLUP);  //channel A
  pinMode(PB3, INPUT_PULLUP);  //channel B
//  pinMode(PA0, INPUT_PULLUP);  //channel A
//  pinMode(PA1, INPUT_PULLUP);  //channel B
/////////////////////////////////////////////////////////////////

///////////////////////Timer4////////////////////////////////
  pinMode(PB6, INPUT_PULLUP);  //channel A
  pinMode(PB7, INPUT_PULLUP);  //channel B
/////////////////////////////////////////////////////////////////

///////////////////////Timer8////////////////////////////////
  pinMode(PC6, INPUT_PULLUP);  //channel A
  pinMode(PC7, INPUT_PULLUP);  //channel B
/////////////////////////////////////////////////////////////////


//configure Timer as encoder
  Timer1.setMode(0, TIMER_ENCODER); //set mode, the channel is not used when in this mode. 
  Timer1.pause(); //stop... 
  Timer1.setPrescaleFactor(4); //normal for encoder to have the lowest or no prescaler. 
  Timer1.setOverflow(PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  Timer1.setCount(0);          //reset the counter. 
  Timer1.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or Timer_SMCR_SMS_ENCODER1 or Timer_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  //Timer1.attachInterrupt(0, func1); //channel doesn't mean much here either.  
  Timer1.resume();                 //start the encoder... 

  Timer2.setMode(0, TIMER_ENCODER); //set mode, the channel is not used when in this mode. 
  Timer2.pause(); //stop... 
  Timer2.setPrescaleFactor(4); //normal for encoder to have the lowest or no prescaler. 
  Timer2.setOverflow(PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  Timer2.setCount(0);          //reset the counter. 
  Timer2.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or Timer_SMCR_SMS_ENCODER1 or Timer_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  //Timer2.attachInterrupt(0, func2); //channel doesn't mean much here either.  
  Timer2.resume();                 //start the encoder... 

  Timer4.setMode(0, TIMER_ENCODER); //set mode, the channel is not used when in this mode. 
  Timer4.pause(); //stop... 
  Timer4.setPrescaleFactor(4); //normal for encoder to have the lowest or no prescaler. 
  Timer4.setOverflow(PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  Timer4.setCount(0);          //reset the counter. 
  Timer4.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or Timer_SMCR_SMS_ENCODER1 or Timer_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  //Timer4.attachInterrupt(0, func3); //channel doesn't mean much here either.  
  Timer4.resume();                 //start the encoder... 

  Timer8.setMode(0, TIMER_ENCODER); //set mode, the channel is not used when in this mode. 
  Timer8.pause(); //stop... 
  Timer8.setPrescaleFactor(4); //normal for encoder to have the lowest or no prescaler. 
  Timer8.setOverflow(PPR);    //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  Timer8.setCount(0);          //reset the counter. 
  Timer8.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or Timer_SMCR_SMS_ENCODER1 or Timer_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  //Timer8.attachInterrupt(0, func4); //channel doesn't mean much here either.  
  Timer8.resume();                 //start the encoder... 

  
}

int readCodeurs (){

//     Serial.println(Timer1.getDirection());

    // rbuffer.rCodRMot=Timer1.getCount();
     //rbuffer.rCodRHip=Timer2.getCount();

    // rbuffer.rCodLMot=Timer4.getCount();
    // rbuffer.rCodLHip=Timer8.getCount();
}

#endif
