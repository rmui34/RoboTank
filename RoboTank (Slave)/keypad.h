/* !\keypad.h
 *
 * This file allows for simple and safe integration of the keypad.c file by
 * allowing for all externally usable functions and variables to be called
 * by whatever file includes this header and puts the keypad.c file in the 
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

#ifndef __KEYPAD_H__
#define __KEYPAD_H__
/*
 *
 * Function Externs
 * Check inside keypad.c for an in-depth description of each function.
 *
 */

extern void key_init();      // initializes Ports to enable funtionality of keys.
extern uint8_t getkey();         // indicates which key was pressed.

/*
 *
 * Variable Externs
 *
 */

extern uint8_t debounceCount; // used to test debouncing
/*
 *
 * Defines
 *
 */

#define debounceTest 50 // tests for bouncing 100 times

// Assigned values to the keys on the keypad so we can differentiate the keys.
#define up      0x1 
#define down    0x2
#define left    0x4
#define right   0x8
#define select  0x10 // fifth bit (this is hex)

#endif //__KEYPAD_H__