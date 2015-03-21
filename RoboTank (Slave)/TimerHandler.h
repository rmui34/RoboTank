/* !\TimerHandler.h
 *
 * This file allows for simple and safe integration of the TimeHandler.c file by
 * allowing for all externally usable functions and variables to be called
 * by whatever file includes this header and puts the TimeHandler.c file in the 
 * project directory. 
 *  
 * Table of Contents:
 * 1. Variable Externs 
 * 2. Function Externs
 *
 * 3.8.15 James Goin, Andrew Townsend, Raymond Mui
 *
 */

#ifndef __TIMERHANDLER_H__
#define __TIMERHANDLER_H__


/*
 *
 * Variable Externs
 *
 */

extern uint16_t instantDist[4]; // array to store instant distant values
extern uint8_t debounceCount; // debounce counter
extern uint32_t sumDist[4];// array to store the accumulated instant values



/*
 *
 * Function Externs
 * Check inside TimerHandler.c for an in-depth description of each function.
 *
 */

extern void Timer0Init(void);
extern void Timer1Init(void);
extern void ADC0Init();
extern void Timer0ADCIntHandler(void);
extern void Timer1KeyIntHandler(void);


#endif //__TIMERHANDLER_H__