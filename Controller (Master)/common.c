/* !\common.c
 *
 * This file initializes the Timers and the ADC to be used in the Interrupt 
 * Service Routine.
 *
 *
 * Table of Contents:
 * 1. Function Handles
 * 2. Initialize Variables
 * 3. distanceTable[256] - Lookup table to convert ADC readings to millimeters
 * 4. delay() - creates a time delay without RTOS running 
 * 5. LookupDistanceTable() - Takes ADC value and returns millimeter value
 * 6. getConvertedDistance() - This function gets the converted  millimeter distance for instant and  
 *                          average values of the ADC readings. 
 * 7. myMutexTake() -This function tells if a shared variable is being used or not 
 *                   and if it is not changes the value of the passed pointer to show that the
 *                   shared variable that the pointer is supposed to represent is being used.
 * 8. myMutexGive() -This function gives the mutex back and returns if the give was
 *                   successful (it was taken).
 *
 * 3.8.15 James Goin, Andrew Townsend, Raymond Mui
 *
 */



/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "common.h"
#include "TimerHandler.h"


// Addresses that point to the queues
QueueHandle_t xOLEDQueue;
QueueHandle_t xSumQueue;
QueueHandle_t xKeyQueue;
QueueHandle_t xInstDistQ;
QueueHandle_t xAvgDistQ;
QueueHandle_t xbToothQ;

// Address of the semaphore
SemaphoreHandle_t OLEDMutex;

// Handles that point to the tasks in order to suspend and resume tasks
TaskHandle_t KeyHandle;
TaskHandle_t STOTHandle;
TaskHandle_t ADCHandle;
TaskHandle_t ADCConvHandle;
TaskHandle_t menuHandle;
TaskHandle_t bToothHandle;



// Conversion table for SHARP distance sensors to millimeters (256 indices)
static const uint16_t distanceTable[256] = {
    800, 800, 800, 800, 800, 800, 800, 800, 800, 800,
    800, 800, 800, 800, 800, 800, 800, 800, 800, 800, 
	800, 800, 800, 800, 800, 800, 800, 800, 800, 800,
	800, 800, 800, 800, 779, 756, 732, 706,	681, 656, 
	632, 610, 591, 574, 560, 547, 536, 526, 517, 509, 
	500, 491, 482, 472, 463, 453, 443, 433, 424, 415, 
	406, 397, 390, 382, 375, 368, 362, 356, 350, 345, 
	340, 334, 329, 324, 319, 314, 309, 304, 299, 293, 
	287, 282, 276, 270, 264, 259, 253, 248, 243, 238,
	234, 230, 226, 223, 220, 217, 215, 213,	211, 210, 
	208, 207, 206, 205, 204, 203, 203, 202, 201, 200, 
	199, 198, 197, 196, 195, 194, 192, 191, 189, 188, 
	186, 184, 183, 181, 179, 177, 175, 173,	171, 169, 
	168, 166, 164, 162, 160, 158, 156, 154, 153, 151, 
	149, 147, 146, 144, 143, 141, 140, 138, 137, 135, 
	134, 133, 132, 130, 129, 128, 127, 126, 125, 124, 
	123, 122, 121, 120, 119, 118, 117, 117, 116, 115, 
	114, 114, 113, 112, 111, 111, 110, 109, 109, 108, 
	108, 107, 107, 106, 105, 105, 104, 104, 103, 103, 
	102, 102, 101, 101, 100, 100,  99,  99,  98,  98,
	 97,  97,  96,  95,  95,  94,  94,  93,  93,  92,
	 92,  91,  91,	90,  90,  89,  89,  88,	 88,  87,
	 87,  86,  85,  85,  84,  84,  83,  83,	 82,  82,
	 81,  81,  80,	80,  79,  79,  78,  78,	 77,  77, 
	 76,  76,  75,  75,  74,  74,  74,  73,	 73,  72,
	 72,  71,  71,	70,  70,  70,	
};

/*!
 * delay()
 *
 * Description: This function creates a delay without needing the RTOS running.
 *
 * Operation: This function runs through a for loop incrementing d times to
 * stall for time.
 * 
 * Return Values: None
 */
void delay(uint16_t d) {
    for(int i = 0; i < d; i++); // increments to take time
}

/*!
 * LookupDistanceTable()
 *
 * Description: This function gets the millimeter conversion of the value from
 * ADC that is sent to it.
 *
 * Operation: By shifting the inputted 16 bit value 8 bits to the right, the 
 * value is converted to an 8 bit value and then used as an index for the
 * conversion table and returned
 * 
 * Return Values: uint16_t value in mm representing the conversion of the input.
 */
uint16_t LookupDistanceTable(uint16_t distance_sensor) {
	return distanceTable[distance_sensor >> 8]; // gets index of top 8 bits
}

/*!
 * getConvertedDistance()
 *
 * Description: This function gets the converted  millimeter distance for instant and  
 * average values of the ADC readings.
 * Operation: Holds the average values from the queue. Checks if the input is valid, then
 * checks whether the input is requesting an instant or average conversion. Returns  appropriate
 * lookup distance value.
 *
 * Return Values: 0
 */
uint16_t getConvertedDistance(uint16_t val){
    // holds the averages from the queue
    static uint16_t convQBuff[4] = {0,0,0,0};
    uint16_t instVal = 0; // local version of the instant value 
    if(val < 8) { // checks if input is valid
    
        if(val < 4) { // if less than 4 it gets the instant conversions
        
            myMutexTake(&instMutex); // takes the mutex
            instVal = instantDist[val]; // saves instantDist locally
            myMutexGive(&instMutex); // gives mutex 
            
            // returns the instant value converted to milimeters
            return LookupDistanceTable(instVal); 
            
        } else { // (3<val<8) gets the average value for val-4
        
            // gets averages without removing it from the queue
            xQueuePeek(xAvgDistQ, &convQBuff, 10); 
            
            // returns the average value converted to milimeters
            return LookupDistanceTable(convQBuff[val - 4]);
        }
    }
    return 0;
}

/*!
 * myMutexTake()
 *
 * Description: This function tells if a shared variable is being used or not 
 * and if it is not changes the value of the passed pointer to show that the
 * shared variable that the pointer is supposed to represent is being used.
 *
 * Operation: Stores the value in the pointer locally before decrementing the
 * value of the pointer if it is greater than 0. It then returns if the pointer
 * passed was an taken mutex
 * 
 * Return Values: Returns a 1 if the pointer points to a value greater than 0 
 * and 0 otherwise.
 */ 
uint8_t myMutexTake(uint8_t* usesCount) {
    uint8_t available = *usesCount; // stores the value from the pointer
    if(*usesCount > 0) // if the mutex isn't taken
        *usesCount--;  // take it
    return available; // returns true if take was successful
}

/*!
 * myMutexGive()
 *
 * Description: This function gives the mutex back and returns if the give was
 * successful (it was taken).
 *
 * Operation: Increments the count and return 1 if the pointer pointed to a
 * value of 0, otherwise return 0.
 * 
 * Return Values: Returns a 1 if the pointer points to a value of 0 and return a
 * 0 otherwise.
 */ 
uint8_t myMutexGive(uint8_t* usesCount) {
    if (*usesCount == 0) { // if taken
        *usesCount++; // give it
        return 1; // return successful
    } else // if not taken
        return 0; // return failed
}