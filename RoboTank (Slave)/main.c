/* !\main.c
 *
 * Holds the startup procedures and central tasks.
 * SLAVE VERSION
 * 
 * Table of Contents:
 * 1. Includes, Prototypes, and Constant Definitions
 * 2. main()  - runs initialization procedure and starts scheduler
 * 3. prvSetupHardware()  - initializes various hardware components
 * 4. vSensorToOLEDTask()    - draws the average distances to the OLED
 * 5. vADCTask()  - calculates and sends the average value
 * 6. vKeyTask() - gets keys from the UART bluetooth connection
 *
 * 3.8.15 James Goin, Andrew Townsend, Raymond Mui
 *
 * RTOS code taken from FreeRTOS at www.freertos.org
 * FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
 * All rights reserved
 *
 */ 
 
 
 
 
 
/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware library includes. */
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "sysctl.h"
#include "grlib.h"
#include "rit128x96x4.h"
#include "osram128x64x4.h"
#include "formike128x128x16.h"

/* Demo app includes. */
#include "flash.h"
#include "lcd_message.h"
#include "bitmap.h"


// Lab group Headers
#include "TimerHandler.h"
#include "keypad.h"
#include "common.h"
#include "pwm.h"
#include "interrupt.h"
#include "lm3s8962.h"
#include "uart.h"


#define ulSSI_FREQUENCY						( 3500000UL )

// Frequency at which the keys are checked for new input
#define keyHz                               ( configTICK_RATE_HZ / 100 )
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )
// Initializes hardware
static void prvSetupHardware( void );

// Hook functions that can get called by the kernel.
void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName );
void vApplicationTickHook( void );

// Task Prototypes

static void vKeyTask(void *pvParameters);
static void vADCTask(void *pvParameters);
static void vSensorToOLEDTask(void *pvParameters);
extern void vMenu( void *pvParameters );

// SubFunction Prototypes
uint16_t getConvertedDistance(uint16_t val);
uint8_t instMutex = 1; // Creates mutex cuz FreeRTOS sucks

// Contains the most recent instant distance sensor data calculated in the ADC
// and is protected using a myMutex
extern uint16_t instantDist[4]; 
uint16_t avgDist[4];
extern void MotorInit(void); 


/*!
 * main()
 *
 * Description: This function initializes the various hardware and data
 * protection as well as creating the tasks and starting the scheduler.
 *
 * Operation: First the queues and semaphores are initialized to ensure that
 * they are not referenced in an interrupt before being created. Then the 
 * hardware is initialized using prvSetupHardware and the tasks are created.
 * The scheduler is then started and the program should never pass that point
 * unless there us an error
 * 
 * Return Values: 0
 */
  int main(void)
{
    // Initializes queues and semaphores for protecting shared variables
    xSumQueue  = xQueueCreate(1 , 4*sizeof(uint32_t));
    xInstDistQ = xQueueCreate(1 , 4*sizeof(uint32_t));
    xAvgDistQ  = xQueueCreate(1 , 4*sizeof(uint32_t));
    xKeyQueue  = xQueueCreate(1 , sizeof(uint8_t));

    OLEDMutex = xSemaphoreCreateMutex();

    // Initializes the hardware.
    prvSetupHardware();
    UARTInit();    
    bluetoothInit();
    
    // Creates the tasks to be ran by the scheduler
    xTaskCreate(vKeyTask,           "keys", 1024, NULL, 100, &KeyHandle );
    xTaskCreate(vSensorToOLEDTask,  "STOT", 1024, NULL, 100, &STOTHandle);
    xTaskCreate(vADCTask,           "ADCT", 1024, NULL, 100, &ADCHandle ); 
    xTaskCreate(vMenu,              "menu", 1024, NULL, 100, &menuHandle);
 
	// Start the scheduler.
	vTaskStartScheduler();

    // Will only get here if there was insufficient memory to create the idle
    // task. 
	return 0;
}


/*!
 * prvSetupHardware()
 *
 * Description: This function initializes all of the hardware needed for the 
 * system to run. This includes the system clock, needed GPIO ports, PWMs, ADCs,
 * OLED display, and timers.
 *
 * Operation: In order for the other hardware to be initialized, first the
 * system clock is set and the various GPIOs are enabled. In order to support
 * error feedback, the OLED is next to be initialized. Afterwards, the various
 * hardware systems are initialized with timers last to prevent interrupts being
 * called before relevant hardware is ready.
 * 
 * Return Values: None.
 */
void prvSetupHardware( void )
{
    // Sets the system clock
	SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN 
    | SYSCTL_XTAL_8MHZ );
    
    // Enables the GPIO ports
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOF | SYSCTL_PERIPH_GPIOE 
    | SYSCTL_PERIPH_GPIOG | SYSCTL_PERIPH_GPIOB | SYSCTL_PERIPH_GPIOD);
    
    // Initializes the OLED display
    RIT128x96x4Init(1000000);
    
    // Initializes the
    MotorInit();    // PWMs and GPIO ports needed by the motor controller 
    ADC0Init();     // ADCs to collect the sensor data from all 4 ADC inputs
    Timer0Init();   // timer0 and its interrupt
}

void vSensorToOLEDTask(void *pvParameters) {
	static uint16_t measuredDist; // Holds the converted distance values
	static char val[7]; // Holds the string to be written

	while(1){ 
        // Gets the average distance from the front sensor
		measuredDist = getConvertedDistance(avgADC0); 
        
        // Given that measuredDist < 1000, converts measuredDist to decimal
        // ASCII characters and adds " mm" to show that it is in millimeters
		val[0] = (char)(measuredDist / 100 % 10 + offsetZero);
		val[1] = (char)(measuredDist / 10 % 10 + offsetZero);
		val[2] = (char)(measuredDist % 10 + offsetZero);
		val[3] = ' ';
		val[4] = 'm';
		val[5] = 'm';
		val[6] = '\0'; // end char
		char * string = val; // gets the pointer
        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // takes OLED control
        RIT128x96x4StringDraw(string, 0, 61, 15); // draws to OLED
        xSemaphoreGive(OLEDMutex); // gives OLED control
		
        
        // Gets the average distance from the back sensor
		measuredDist = getConvertedDistance(avgADC1);
        
        // Given that measuredDist < 1000, converts measuredDist to decimal
        // ASCII characters and adds " mm" to show that it is in millimeters
		val[0] = (char)(measuredDist / 100 % 10 + offsetZero);
		val[1] = (char)(measuredDist / 10 % 10 + offsetZero);
		val[2] = (char)(measuredDist % 10 + offsetZero);
		val[3] = ' ';
		val[4] = 'm';
		val[5] = 'm';
		val[6] = '\0'; // end char
		string = val; // gets the pointer
        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // takes OLED control
        RIT128x96x4StringDraw(val, 0, 69, 15); // draws to OLED
        xSemaphoreGive(OLEDMutex); // gives OLED control
		
        // Gets the average distance from the left sensor
		measuredDist = getConvertedDistance(avgADC2);
        
        // Given that measuredDist < 1000, converts measuredDist to decimal
        // ASCII characters and adds " mm" to show that it is in millimeters
		val[0] = (char)(measuredDist / 100 % 10 + offsetZero);
		val[1] = (char)(measuredDist / 10 % 10 + offsetZero);
		val[2] = (char)(measuredDist % 10 + offsetZero);
		val[3] = ' ';
		val[4] = 'm';
		val[5] = 'm';
		val[6] = '\0'; // end char
		string = val; // gets the pointer
        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // takes OLED control
        RIT128x96x4StringDraw(val, 0, 77, 15); // draws to OLED
        xSemaphoreGive(OLEDMutex); // gives OLED control
		
        // Gets the average distance from the right sensor
		measuredDist = getConvertedDistance(avgADC3);
        
        // Given that measuredDist < 1000, converts measuredDist to decimal
        // ASCII characters and adds " mm" to show that it is in millimeters
		val[0] = (char)(measuredDist / 100 % 10 + offsetZero);
		val[1] = (char)(measuredDist / 10 % 10 + offsetZero);
		val[2] = (char)(measuredDist % 10 + offsetZero);
		val[3] = ' ';
		val[4] = 'm';
		val[5] = 'm';
		val[6] = '\0'; // end char
		string = val; // gets the pointer
        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // takes OLED control
        RIT128x96x4StringDraw(val, 0, 85, 15); // draws to OLED
        xSemaphoreGive(OLEDMutex); // gives OLED control
		
		vTaskSuspend(STOTHandle); // suspends this task
	}
}


/*!
 * vADCTask()
 *
 * Description: This task calculates the average values from the ADC sensors 
 * once a second. If a value is not sent within 1200 ms, an error is drawn and
 * it enters an infinite loop.
 *
 * Operation: Waits for 1200 ms to receive a sum of 4096 distance values from 
 * timer interrupt in the xSumQueue. When it receives it, the sums are stored in
 * a buffer and the averages are then calculated and sent to the average
 * distance queue, xAvgDistQ. Control is then passed to the sensor to OLED task 
 * and the task waits for new input from the queue. If it does not receive
 * any values within the limit, priority is set to the highest and an error is
 * drawn to the OLED before entering an infinite loop.
 * 
 * Return Values: None.
 */
void vADCTask(void *pvParameters) {
static uint32_t queueBuf[4]; // holds the value 
    while(1) {
        // waits 1200 ms for a new Sum from the timer ints and stores it in 
        // queueBuf
        if ( xQueueReceive(xSumQueue, &queueBuf, 1200)) {
            // calculates averages
            avgDist[0] = queueBuf[0] / 4096; 
            avgDist[1] = queueBuf[1] / 4096;
            avgDist[2] = queueBuf[2] / 4096;
            avgDist[3] = queueBuf[3] / 4096;
            // sends average to STOT using queue
            xQueueOverwrite(xAvgDistQ, &avgDist);
        } else { // if the Sum takes longer than 1200 ms
            vTaskPrioritySet(ADCHandle, 200); // sets task to highest priority
            // draws description of error to OLED
            RIT128x96x4StringDraw("ADCInt not responding", 0, 20, 15);
            while(1); // enters infinite loop
        }
            vTaskResume(STOTHandle); // resumes the sensor to OLED task
	}
}

/*!
 * vKeyTask()
 *
 * Description: This task checks for key input sent via bluetooth from the
 * master.
 *
 * Operation: Gets the value in the UART data register and masks it for just the
 * key presses and then overwrites the key queue so that it can be used elsewhere.
 * 
 * Return Values: None.
 */

void vKeyTask(void *pvParameters) {
    static TickType_t xLastWakeTime; // stores start tick
    unsigned long data;
    static uint8_t keyOut; // sent key press

	while(1) {
          
          // get data from UART data register
          // (get char sent over bluetooth)
          data = UART_InChar();

          // masks the received char to match the key set-up
          keyOut = (uint8_t) data & 0x1F;
     
         
                                                                                                                                                                                                                                                                                                                                                                                                     
          // Overwrites the data currently in the queue
          xQueueOverwrite(xKeyQueue, &keyOut); 
        
          // Wait for the next cycle.
          vTaskDelayUntil( &xLastWakeTime, keyHz);
	}
}    



////////////////////////////////////////////////////////////////////////////////

// All code below here is from FreeRTOS

////////////////////////////////////////////////////////////////////////////////

void vApplicationTickHook( void )
{
static xOLEDMessage xMessage = { "PASS" };
static unsigned long ulTicksSinceLastDisplay = 0;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Called from every tick interrupt.  Have enough ticks passed to make it
	time to perform our health status check again? */
	ulTicksSinceLastDisplay++;
	if( ulTicksSinceLastDisplay >= mainCHECK_DELAY )
	{
		ulTicksSinceLastDisplay = 0;

		configASSERT( strcmp( ( const char * ) xMessage.pcMessage, "PASS" ) == 0 );

		/* Send the message to the OLED gatekeeper for display. */
		xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR( xOLEDQueue, &xMessage, &xHigherPriorityTaskWoken );
	}
}

void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName )
{
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}

void vAssertCalled( const char *pcFile, unsigned long ulLine )
{
volatile unsigned long ulSetTo1InDebuggerToExit = 0;

	taskENTER_CRITICAL();
	{
		while( ulSetTo1InDebuggerToExit == 1 )
		{
			/* Nothing do do here.  Set the loop variable to a non zero value in
			the debugger to step out of this function to the point that caused
			the assertion. */
			( void ) pcFile;
			( void ) ulLine;
		}
	}
	taskEXIT_CRITICAL();
}
