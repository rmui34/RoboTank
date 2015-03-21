/* !\UART.c
 *
 * Implementation of the UART Port on Stellaris Board. Initializations for the port
 * and functions that manage data received and sent between bluetooth modems.
 *
 * Table of Contents:                                                                                     
 * 1. UARTInit()  - initializes Ports to enable funtionality of keys. (Taken from online)
 * 2. UART_InChar()   -Reads in a single character. (Taken from online)
 * 3. UART_OutChar(unsigned char data) - Sends out a single character. (Taken from online)
 * 4. UART_OutString(char *pt) - Sends out a string. (Taken from online)
 * 5. UART_InString(char *bufPt, unsigned short max) - Reads in a received string. (Taken from online)
 * 6. bluetoothInit() - Sets up the bluetooth modem to be either a master or a slave. 
 * 
 * Originally Created by Daniel Valvano May 30, 2014
 * Modified by (UT EE345L) students Charlie Gough && Matt Hawk
 * Modified by (UT EE345M) students Agustinus Darmawan && Mingjie Qiu
 * Modified on 3.13.15  by James Goin, Andrew Townsend, Raymond Mui (UW EE472)
 *
 */

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

//------------UART_Init------------
// Initialize the UART for 115,200 baud rate (assuming 50 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UARTInit(void) {
  
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0 *Put in Hardware setup
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A * Put in Hardware setup
  UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART (STEP 1)
  UART0_IBRD_R = 27;                    // IBRD = int(50,000,000 / (16 * 115,200)) = int(27.1267)
  UART0_FBRD_R = 8;                     // FBRD = int(0.1267 * 64 + 0.5) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_CTL_R |= UART_CTL_UARTEN;       // enable UART
  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1-0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1-0 
}





//------------UART_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
unsigned char UART_InChar(void){
    while((UART0_FR_R&UART_FR_RXFE) == 0){
        return((unsigned char)(UART0_DR_R&0xFF));
    }
    
    return 0;
}
//------------UART_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART_OutChar(unsigned char data){
    while((UART0_FR_R&UART_FR_TXFF) != 0);
    UART0_DR_R = data;
    
}


//------------UART_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART_OutString(char *pt){
  while(*pt){
    UART_OutChar(*pt);
    pt++;
  }
}

//------------UART_InString------------
// Accepts ASCII characters from the serial port
//    and adds them to a string until <enter> is typed
//    or until max length of the string is reached.
// It echoes each character as it is inputted.
// If a backspace is inputted, the string is modified
//    and the backspace is echoed
// terminates the string with a null character
// uses busy-waiting synchronization on RDRF
// Input: pointer to empty buffer, size of buffer
// Output: Null terminated string
// -- Modified by Agustinus Darmawan + Mingjie Qiu --
void UART_InString(char *bufPt, unsigned short max) {
int length=0;
char character;
  character = UART_InChar();
  while(character != '/'){
    if(character == 8){
      if(length){
        bufPt--;
        length--;
        UART_OutChar((char) 8);
      }
    }
    else if(length < max && character != '\0'){
      *bufPt = character;
      bufPt++;
      length++;
      UART_OutChar(character);
    }
    character = UART_InChar();
  }
  *bufPt = 0;
}


/*! 
 * bluetoothInit()
 *
 * Description: Sends out appropriate string commands to set up the bluetooth modem
 * to a slave setup for the modem on the RoboTank and a master for the modem on 
 * the controller.
 *
 * Operation: For Master setup - Sends modem into command mode, disable authentication   
 * when pairing, make auto connect master, set to connect to specific Mac Address of
 * modem that one wants to pair. Restart modem. Exit command mode.
 * For Slave setup- Send modem to command mode. Set to slave. Restart mode. 
 * Exit command mode.
 *
 * Bluetooth Command Definitions:
 * 
 * $$$: enter command mode
 * \r :signifies end of a command 
 * ---: exit command mode
 * SA,0: Disable authentification when pairing
 * SM,0: Slave Mode
 * SM,3: AutoConnect Master Mode
 * SR,<MacAdress>: Store Remote Address
 */
void bluetoothInit(void) {
    // UnBrick
    //UART_OutString("$$$\rS~,0\rR,1\r---\r");
    
    //Master Setup
    UART_OutString("$$$\r"); // enter command mode
    UART_OutString("SA,0\rSM,3\r"); // disable authentification, set to Master Mode 
    UART_OutString("SR,0006666D9A91\rR,1\r"); // set the remote address, restart
    UART_OutString("---\r"); // exit command mode
     
    //Slave Setup
    //UART_OutString("$$$\r"); // enter command mode
    //UART_OutString("SM,0\rR,1\r"); // set slave mode, restart
    //UART_OutString("---\r"); // exit command mode
}
