/* !\common.h
 *
 * This file allows for simple and safe integration of the common.c file by
 * allowing for all externally usable functions and variables to be called
 * by whatever file includes this header and puts the common.c file in the 
 * project directory. 
 *  
 * Table of Contents:
 * 1. Function Externs
 * 2. Variable Externs 
 * 3. Defines
 *
 * 3.8.15 James Goin, Andrew Townsend, Raymond Mui
 *
 */

#ifndef __COMMON_H__
#define __COMMON_H__

/*
 *
 * Function Externs
 * Check inside common.c for an in-depth description of each function.
 *
 */
extern void delay(uint16_t d);
extern uint16_t LookupDistanceTable(uint16_t distance_sensor);
extern uint16_t getConvertedDistance(uint16_t val);
extern uint8_t myMutexTake(uint8_t* usesCount);
extern uint8_t myMutexGive(uint8_t* usesCount);

/*
 *
 * Variable Externs
 *
 */

// Addresses that point to the queues
extern QueueHandle_t xOLEDQueue;
extern QueueHandle_t xSumQueue;
extern QueueHandle_t xKeyQueue;
extern QueueHandle_t xInstDistQ;
extern QueueHandle_t xAvgDistQ;
extern QueueHandle_t xbToothQ;

// Address of the semaphore
extern SemaphoreHandle_t OLEDMutex;

// Handles that point to the tasks in order to suspend and resume tasks
extern TaskHandle_t KeyHandle;
extern TaskHandle_t STOTHandle;
extern TaskHandle_t ADCHandle;
extern TaskHandle_t ADCConvHandle;
extern TaskHandle_t menuHandle;
extern TaskHandle_t bToothHandle;


extern uint8_t instMutex;
/*
 *
 * Defines
 *
 */

#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3


#define instADC0 0
#define instADC1 1
#define instADC2 2
#define instADC3 3
#define avgADC0 4
#define avgADC1 5
#define avgADC2 6
#define avgADC3 7

#define offsetZero 48 // used for ASCII char to make value 0.


#endif //__COMMON_H__