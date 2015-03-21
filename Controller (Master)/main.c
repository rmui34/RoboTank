/* !\main.c
 *
 * Holds the startup procedures and central tasks. 
 * MASTER VERSION
 * 
 * Table of Contents:
 * 1. Includes, Prototypes, and Constant Definitions
 * 2. main()  - runs initialization procedure and starts scheduler
 * 3. prvSetupHardware()  - initializes various hardware components
 * 4. vKeyTask() - evaluates keypad 100 times per second and sends to bluetooth
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


#define ulSSI_FREQUENCY		( 3500000UL )

// Frequency at which the keys are checked for new input
#define keyHz                   ( configTICK_RATE_HZ / 100 )
#define mainCHECK_DELAY		( ( TickType_t ) 5000 / portTICK_PERIOD_MS )
// Initializes hardware


// Hook functions that can get called by the kernel.
void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName );
void vApplicationTickHook( void );

// Global Variables
uint8_t instMutex = 1; // Creates mutex cuz FreeRTOS sucks

// Prototypes
static void prvSetupHardware( void );
static void vKeyTask(void *pvParameters);

// External Functions
extern void vMenu(void *pvParameters);
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
    // Initializes a queue and semaphore for protecting shared variables
    xKeyQueue = xQueueCreate(1 , sizeof(uint8_t));
    OLEDMutex = xSemaphoreCreateMutex();

    // Initializes the hardware.
    prvSetupHardware();
    UARTInit();    
    bluetoothInit();

      
    // Creates the tasks to be ran by the scheduler
	xTaskCreate(vKeyTask, "keys", 1024, NULL, 100, &KeyHandle );
	xTaskCreate(vMenu,    "menu", 1024, NULL, 100, &menuHandle); 
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
 * system to run. This includes the system clock, needed GPIO ports, PWMs,
 * OLED display, and timer.
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
    MotorInit();    // Used to mimic the slaves motor settings
    key_init();     // GPIOs used by the keypad
    Timer1Init();   // timer1 and its interrupt
}

/*!
 * vKeyTask()
 *
 * Description: This task checks for valid debounced keypad input 10 times per 
 * second and stores it (or overwrites the current value) in the xKeyQueue.
 *
 * Operation: Gets the current tick count and then disables the debouncing
 * interrupt to protect the debounce count, saves it locally, and then renables
 * debouncing interrupt. If the debounce count was below required amount, both
 * keyOut and the past value are zeroed. Otherwise, if the contPress variable is
 * true keyOut is set to the current keyVal gotten via getkey(), else it is set
 * to keyVal only if it is different than the last keyVal. keyOut then
 * overwrites the current value in the xKeyQueue to be used elsewhere. The task
 * then delays for 100 ticks since it was measured at the beginning. Sends keys
 * values out via Bluetooth SPP.
 * Return Values: None.
 */
void vKeyTask(void *pvParameters) {
    static TickType_t xLastWakeTime; // stores start tick
    static uint8_t keyOut; // sent key press
    static uint8_t keyVal;
    static uint8_t pastKeyOut; // last sent key press
    static uint8_t debounceLocal; // local debounce count

	while(1) {
          
          xLastWakeTime = xTaskGetTickCount(); // stores the current OS systick
          IntDisable( INT_TIMER1A ); // disables debounce int
          debounceLocal = debounceCount; // stores debounce value
          IntEnable( INT_TIMER1A ); // enables debounce int
          keyVal = getkey(); // gets current key press
          if(debounceLocal < debounceTest) { // tests if press isn't valid
            keyOut = 0; // sets to 0 if invalid
          } else 
            keyOut = keyVal; // else sets as key press

          UART_OutChar(keyOut); // sends via bluetooth
        
          pastKeyOut = keyOut; // saves the previously sent value
        
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
