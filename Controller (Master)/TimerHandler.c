/* !\TimeHandler.c
 *
 * This file initializes the Timers and the ADC to be used in the Interrupt 
 * Service Routine.
 *
 *
 * Table of Contents:
 * 1. Constant Definitions
 * 2. Initialize Variables
 * 3. TimerInit() - Initializes Timer0 and Timer1 for use
 * 4. ADC0Init() - Initializes the ADC0 pin for sampling
 * 5. Timer0ADCIntHandler - Interrupt Service Routine (Timer 0) for ADC
 * 6. Timer1KeyIntHandler - Interrupt Service Routine (Timer 1) for keys
 *
 * 3.8.15 James Goin, Andrew Townsend, Raymond Mui
 *
 */



/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "hw_types.h"
#include "sysctl.h"
#include "hw_ints.h"
#include "adc.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "sysctl.h"
#include "lmi_timer.h"
#include "lm3s8962.h" // Imports drivers for the stellaris board
#include "gpio.h"
#include "rit128x96x4.h"
#include "keypad.h"
#include "interrupt.h"
#include <stdint.h>
#include "common.h"

#define timerINTERRUPT_FREQUENCY		( 4096 )
#define samplesPerAverage       		( 4096UL )
/* The highest available interrupt priority. */
#define timerHIGHEST_PRIORITY			( 0 )

/* Misc defines. */
#define timerMAX_32BIT_VALUE			( 0xffffffffUL )

uint16_t instantDist[4]; // array to store instant distant values
uint8_t debounceCount; //debounce counter
uint32_t sumDist[4]; // array to store the accumulated instant values
extern uint8_t instMutex; //mutex used to protect instant distant data

/*!
 * Timer0Init()
 *
 * Description: This function sets and enables Timer0 
 * to be used for the Interrupt Service Routine.
 * 
 * Operation: First enable the peripheral for the timer. Configure the
 * 32-bit periodic mode. Set timer 0 interrupt to the highest priority. Disable
 * interrupt to ensure interrupts do not start until the scheduler is running. 
 * Set the frequency of the timer 0 interrupt and then enable the timer 0 to begin.
 * 
 * 
 * Return Values: None
 */
void Timer0Init(void) {
  uint32_t ulFrequency;
   
	//Timer zero is used to generate the interrupts, and timer 1 is used
	//to measure the jitter. 
	SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER0 );
        
        TimerConfigure( TIMER0_BASE, TIMER_CFG_32_BIT_PER );
	
	//Set the timer interrupt to be above the kernel - highest. 
	IntPrioritySet( INT_TIMER0A, timerHIGHEST_PRIORITY );
	
	//Ensure interrupts do not start until the scheduler is running. 
	portDISABLE_INTERRUPTS();
	
	// The rate at which the timer will interrupt. 
	ulFrequency = configCPU_CLOCK_HZ / timerINTERRUPT_FREQUENCY;
	
        TimerLoadSet( TIMER0_BASE, TIMER_A, ulFrequency );
        
        IntEnable( INT_TIMER0A );
        
        TimerIntEnable( TIMER0_BASE, TIMER_TIMA_TIMEOUT );

	//Enable both timer. 	
        TimerEnable( TIMER0_BASE, TIMER_A );
}

/*!
 * Timer1Init()
 *
 * Description: This function sets and enables Timer1 
 * to be used for the Interrupt Service Routine.
 * 
 * Operation: First enable the peripheral for the timer. Configure the
 * 32-bit periodic mode. Set timer 1 interrupt to the highest priority. Disable
 * interrupt to ensure interrupts do not start until the scheduler is running. 
 * Set the frequency of the timer 1 interrupt and then enable the timer 1 to begin.
 * 
 * 
 * Return Values: None
 */
void Timer1Init(void) {
  uint32_t ulFrequency1;
   
	//Timer zero is used to generate the interrupts, and timer 1 is used
	//to measure the jitter. 
	SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER1 );
        
        TimerConfigure( TIMER1_BASE, TIMER_CFG_32_BIT_PER );
	
	//Set the timer interrupt to be above the kernel - highest. 
	IntPrioritySet( INT_TIMER1A, timerHIGHEST_PRIORITY );
	
	//Ensure interrupts do not start until the scheduler is running. 
	portDISABLE_INTERRUPTS();
	
	// The rate at which the timer will interrupt. 
	ulFrequency1 = configCPU_CLOCK_HZ / 10000;
	
        TimerLoadSet( TIMER1_BASE, TIMER_A, ulFrequency1 );
        
        IntEnable( INT_TIMER1A );
        
        TimerIntEnable( TIMER1_BASE, TIMER_TIMA_TIMEOUT );

	//Enable both timer. 	
        TimerEnable( TIMER1_BASE, TIMER_A );
}

/*!
 * ADC0Init()
 *
 * Description: Initializes the ADC0 pin for sampling
 * 
 * 
 * Operation: First enable the ADC0 pin of the RCGC0 register. Select appropriate
 * sample sequence to read. Select the end of a sequence. Enable Active sequence
 * samples. Trigger ADC converion.
 * 
 * 
 * Return Values: None
 */
void ADC0Init() {
  // Enable ADC0 pin of the RCGC0 register
  SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC;
  
  delay(100);
  
  //Select appropriate sample sequence to read
  ADC_SSMUX0_R = 0;
  ADC_SSMUX1_R = 1;
  ADC_SSMUX2_R = 2;
  ADC_SSMUX3_R = 3;
  
  // Select end of sample sequence
  ADC_SSCTL0_R = ADC_SSCTL0_END0;
  ADC_SSCTL1_R = ADC_SSCTL1_END0;
  ADC_SSCTL2_R = ADC_SSCTL2_END0;
  ADC_SSCTL3_R = ADC_SSCTL3_END0;
 
  // Enable Active Sequence Sample
  ADC0_ACTSS_R = ADC_ACTSS_ASEN0 |
                 ADC_ACTSS_ASEN1 |
                 ADC_ACTSS_ASEN2 |
                 ADC_ACTSS_ASEN3;
  
  //Trigger ADC conversion      
  ADCProcessorTrigger(ADC_BASE, 0);

}

/*!
 * Timer0ADCIntHandler()
 *
 * Description: Timer 0, ADC Interrupt
 * 
 * 
 * Operation: Clear Timer Interrupt. Trigger the ADC conversion. Read the value from
 * the Sample Sequencer FIFO register and shift the 10-bit value to the left making it
 * a 16-bit value and store the appropriate value in the local version of the instant distance
 * array. Use the local copy and set it to the global copy of instant distant and protect the 
 * shared data with a mutex. Accumulate the local instant distant values in the sum distant
 * array for 4096 times and then restarts the summing.
 * 
 * Return Values: None
 */
void Timer0ADCIntHandler(void) {
    static uint32_t samples = 0; // sample counter
    static uint16_t instLocal[4]; // local copy of instant distances read from ADC

    TimerIntClear( TIMER0_BASE, TIMER_TIMA_TIMEOUT ); //clear timer interrupt

    //Trigger ADC conversion
    ADCProcessorTrigger(ADC_BASE, 0);
    ADCProcessorTrigger(ADC_BASE, 1);
    ADCProcessorTrigger(ADC_BASE, 2);
    ADCProcessorTrigger(ADC_BASE, 3);
    instLocal[ADC0] =  ADC0_SSFIFO0_R << 6;  
    instLocal[ADC1] =  ADC0_SSFIFO1_R << 6;  
    instLocal[ADC2] =  ADC0_SSFIFO2_R << 6;  
    instLocal[ADC3] =  ADC0_SSFIFO3_R << 6;
    
    // shared variable will be updated only if 
    // instantDist isn't being modified/read
    // elsewhere
    if(myMutexTake(&instMutex)) {
        // copies local value to global value
        instantDist[ADC0] =  instLocal[ADC0];  
        instantDist[ADC1] =  instLocal[ADC1];  
        instantDist[ADC2] =  instLocal[ADC2];  
        instantDist[ADC3] =  instLocal[ADC3];
        myMutexGive(&instMutex);    
    }
    // takes samples and sums until samplePerAverage (4096)
    if( samples < samplesPerAverage) {
        samples++;
        // sums the local ADC values
        sumDist[ADC0] += instLocal[ADC0];
        sumDist[ADC1] += instLocal[ADC1];
        sumDist[ADC2] += instLocal[ADC2];
        sumDist[ADC3] += instLocal[ADC3];
    } else {
        samples = 0; // restarts samples
        
        // sends values to queue and handles error if queue is full
        if(xQueueSendFromISR(xSumQueue, &sumDist, NULL) ==  errQUEUE_FULL) {
            RIT128x96x4StringDraw("Sum Queue is full", 0, 20, 15); //draws the myData characters
            while(1);
        }
        //restarts summing
        sumDist[0] = 0;
        sumDist[1] = 0;
        sumDist[2] = 0;
        sumDist[3] = 0;
    }
}

/*!
 * Timer1KeyIntHandler()
 *
 * Description: Timer 1, Key Interrupt
 * 
 * 
 * Operation: Clears the timer interrupt and then gets the key value. If a key has
 * been press we check if the key has been debounced. If not we set the pastKey
 * to getkey(). Then we check if getkey() and pastKey are equal. If so we check
 * if the debounce count has exceeded 256 and increment accordingly. Otherwise we
 * reset debounce count.
 * 
 *
 * Return Values: None
 */
void Timer1KeyIntHandler( void ) {
    TimerIntClear( TIMER1_BASE, TIMER_TIMA_TIMEOUT ); //clear timer interrupt
    static uint8_t pastKey; // stores past key value
    // checks if a key has been pressed
    if (getkey()) {
        //checks if start of debounce
        if(debounceCount == 0) {
            pastKey = getkey();
        }
        // checks if the get key is the same as the last key press
        if(getkey() == pastKey) {
            if(debounceCount < 0xFF) // checks if count is less than 256
                debounceCount++; // increment debounce count
        } else {
            debounceCount = 0; //reset debouncing
        }     
    } else {
        debounceCount = 0; // reset debouncing
    } 
}


