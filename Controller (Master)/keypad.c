/* !\keypad.c
 *
 * Implementation of the keypad on Stellaris Board. The keys on the
 * keypad are up, down, left, right and select.
 *
 *
 * Table of Contents:
 * 1. Constant Definitions
 * 2. key_init()  - initializes Ports to enable funtionality of keys.
 * 3. getkey()    - indicates which key was pressed.
 *
 * 3.8.15 James Goin, Andrew Townsend, Raymond Mui
 *
 */

#include <stdint.h>
#include "hw_types.h"           // defines various data types
#include "sysctl.h"       // imports system control script
#include "lm3s8962.h"           // imports drivers for the stellaris board 
#include "keypad.h"


/*!
 * key_init()
 *
 * This function initializes the GPIO Ports to enable the 
 * functionality of the keys.
 * DATA SHEET REFERENCE             
 * Up Key : Port E, Pin 0
 * Down Key : Port E, Pin 1
 * Left Key : Port E, Pin 2
 * Right Key: Port E, Pin 3
 * Select Key: Port F, Pin 1
 * 
 * Due to the orientation of the Stellaris board and the LCD
 * display
 * Each GPIO Port has eight pins (zero indexed). The bits are  
 * zero indexed so Pin 0 is represented by the 
 * Least Significant Bit.
 * Changing these bits to high will enable the Ports.
 */
void key_init() {
  
/////////////////////PORT F//////////////////////////////////////
    
    //Set the direction register of the pin controlling LED1 to be  
    //output. 
    GPIO_PORTF_DIR_R &= ~0x02;
    // Disable alternate functionality of all pins using the
    // GPIO Alternate Function Select Register
    GPIO_PORTF_AFSEL_R &= 0xF0;
    
    // Enable pin 0 (LED) and pin1(Select) in the 
    // GPIO Digital Enable Register
    GPIO_PORTF_DEN_R |= 0x02;
    

    // Enable pull up resistor so it the voltage across the  
    // key can be measured.     
    GPIO_PORTF_PUR_R |= 0x02;
    
////////////////////PORT E///////////////////////////////////////    
     
    //Set the direction of the pins controlling Port E to be 
    //input
    GPIO_PORTE_DIR_R &= 0xF0;
  
    // Disable alternate functionality of pin using the GPIO 
    // Alternate Function Select Register
    GPIO_PORTE_AFSEL_R &= 0xFE;
    
    // Enable the pin in the GPIO Digital Enable Register
    // Need to enable  pins 0,1,2,3 to high. 
    GPIO_PORTE_DEN_R |= 0x0F; 
   
    // Enable pull up resistor so it the voltage across the 
    // switch can be measured
    GPIO_PORTE_PUR_R |= 0x0F;
}

/*!
 * getkey()
 *
 * Description: This function checks what key was pressed and
 * returns the value of that key. The integers values are 
 * assigned as:
 *
 * If no key is pressed the function will return 0.
 *
 *  
 * Up Key : Port E, Pin 0
 * Down Key : Port E, Pin 1
 * Left Key : Port E, Pin 2
 * Right Key: Port E, Pin 3
 * Select Key: Port F, Pin 1
 *
 * 5 bit: SRLDU
 * Active high
 * 
 * Operation: This function reads the data from each port
 * and is masked with a value that will isolate the bits 
 * representing the pins. The isolated bit(s) are then compared
 * with the value of that checks if that data pin changes to a 0
 * when the key is pressed.
 * 
 * Return Values: returns an uint8_t of the value of that specific   
 * key. See description for assigned int values.
 */
uint8_t getkey() {
    // shifts and masks the Select key to be the 5th bit
    uint8_t a = ((0x2 & (~GPIO_PORTF_DATA_R)) << 3); 
    // returns the keys pressed in the format 000SRLDU
    return ( a | (0xF & (~GPIO_PORTE_DATA_R)));
}

  





