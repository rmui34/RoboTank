/* !\motor.c
 *
 * Implementation of the Toshiba TB6612FNG Motor Driver IC, buzzer and the corresponding
 * menu scripts for motor control on the tank. Writes interactive menu to the display and 
 * displays information to OLED. 
 *
 * Table of Contents:
 * 1. Constant Definitions
 * 2. MotorInit() - Initializes all need peripherials to use two DC motors and buzzer including PWM 
 * 3. motorControl() - Sets the PWM pulse width, state, direction of both motors 
 * 4. Directions() - Draws a string written to the OLED at the specified position and waits for input
 * 5. Options() - Draws column of options for the user to choose
 * 6. vMenu() - This task controls the motors and draws the current status of the motor controller 
 *
 * 3.5.15 James Goin, Andrew Townsend, Raymond Mui
 *
 */


// Include necissary files 


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lcd_message.h"
#include "semphr.h"

/* Library includes. */
#include "hw_types.h"
#include "sysctl.h"
#include "hw_ints.h"
#include "adc.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "interrupt.h"
#include "sysctl.h"
#include "lmi_timer.h"
#include "hw_timer.h"
#include "lm3s8962.h" // Imports drivers for the stellaris board
#include "pwm.h"
#include "gpio.h"
#include "rit128x96x4.h"
#include "keypad.h"
#include <stdint.h>
#include "common.h"



/*!

Control Layout for Motor

AIN for left, BIN for right
IN1 = 0 IN2 = 1 counter-clockwise (reverse) 
IN1 = 1 IN2 = 0 clockwise (forward)
IN1 = 0 IN2 = 0 high impedance state
IN1 = 1 IN2 = 1 short-circuit brake mode
PWM1 Right 
PMM2 Left 
// motorControl(AIN1, AIN2, BIN1 BIN2);

AIN1 = PORTB PIN 0 
AIN2 = PORTB PIN 1 
BIN1 = PORTB PIN 2 
BIN2 = PORTB PIN 3   
*/ 

#define  hImpedence  0x00 // represents high impedence mode for the motor
#define  reverse  0x1 // represents reverse mode for the motor
#define  forward  0x2 // represents forward mode for the motor
#define  brake  0x3 // represents brake mode for the motor 
#define  standby 0x80 // represents standby mode for the motor

#define  leftMotor  0x0  // Output for left motors shift
#define  rightMotor  0x2  // Output for right motors shift

#define  highSpeed  300 // High speed value for width of a pulse 
#define  lowSpeed 250 // Low speed value for the width of a pulse 

#define bow 0 // front
#define port 1 // left
#define starboard 2 // right
#define stern 3 // back

#define manual 1 // represents manual mode 
#define semi 2 // represents semi automatic mode 


// Extern needed functions and variables  
extern SemaphoreHandle_t OLEDMutex;
extern uint16_t getConvertedDistance(uint16_t sensor);
extern QueueHandle_t xKeyQueue;

/*!
 * MotorInit()
 *
 * Description: Initializes the board to use two dc motors and a buzzer, powered 
 * by pulse width modulation. 
 *
 * Operation: First enables PWM on the system control, GPIO ports for PWM 1-3,                         
 * enables to up and down, sets periods of each PWM, enables the PWMs, and enables
 * 5 GPIO output pins for digital output in order to control the motor controller. 
 * 
 * Return Values: 0
 */
void MotorInit() {
    
    // Enables PWM peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    
    // Enables GPIO port B where PWM2 and PMW3 is located
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    // Enables GPIO Port G where PWM1 (for buzzer sound)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    
    // PWM2 and PWM3 are on GPIO port B, pin 0 and pin 1
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // PWM1 is on pin1
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
    
    // Sets PWM generator 1 (PWM2-3) to count up and down and no sync
	PWMGenConfigure(PWM_BASE, PWM_GEN_1,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    // Sets PWM generator 0 (PWM1) to count up and down and no sync
    PWMGenConfigure(PWM_BASE, PWM_GEN_0,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    uint32_t ulPeriod = SysCtlClockGet() / 800; // 800 Hz
    
    // Sets the hz of the PWM
	PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, 333); // 24.024kHz
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod); // 800Hz
    
    // Sets the width of the positive section of the square wave
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, 166);
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, 300);
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 330);    
    
    // Enables the PWMs
	PWMGenEnable(PWM_BASE, PWM_GEN_1);
	PWMGenEnable(PWM_BASE, PWM_GEN_0);
    
    // Enable the pin in the GPIO Digital Enable Register
    // Need to enable  pins 0, 1, 2, 3 and 4.
    // Pin 2, 3, 4 ,5 ,7
    GPIO_PORTD_DEN_R |= 0xBC; // Enable     
    GPIO_PORTD_AFSEL_R &= 0x43; // Disable alternate functionality      
    GPIO_PORTD_DIR_R |= 0xBC; // Set to outputs  
    

    
    

}

/*!
 * motorControl()
 *
 * Description: Sets the PWM pulse width, state, direction of both motors using
 * the motor controller's corresponding GPIO ports and the PWM generator.
 *
 * Operation: Checks if the command inputs are both less than 4 and if so
 * configures the motor controller's GPIO pins to match the inputs. If not, the
 * standby pin is turned off resulting in standby mode for both motors. In all
 * cases, the PWM pulses are set to the passed width and the generator is
 * updated.
 * 
 * Return Values: None
 */
void motorControl (int leftMotorCommand, int rightMotorCommand, int leftMotorSpeed, int rightMotorSpeed) {
    
    // checks for valid input
    if( leftMotorCommand <= 3 && rightMotorCommand <= 3) {  
        // set the left and right motors to the inputs specified.
        GPIO_PORTD_DATA_R = standby | ((leftMotorCommand << 2)
        | (rightMotorCommand << 4)); 

    } else { // invalid input results in 
        GPIO_PORTD_DATA_R = ~standby; // standby mode
    }
    // set the new pulse width
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, rightMotorSpeed);
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, leftMotorSpeed);
    
    // re-enable the generator to update PWMs
    PWMGenEnable(PWM_BASE, PWM_GEN_1);
} 
  

/*!
 * Directions()
 *
 * Description: Draws a string, written, to the OLED at the specified position and waits
 * for select to be pressed and released
 *
 * Operation: Takes the OLEDMutex and draws the given string to the passed
 * position and then gives the OLEDMutex back. It then waits for the key press
 * to equal to select and then waits for select to be unpressed in order to
 * prevent going through all the menus in one press.
 * 
 * Return Values: None
 */
void Directions(char* written, uint8_t xVal, uint8_t yVal) {
    static uint8_t key = 0; // saves the key press from the xKeyQueue
    xSemaphoreTake(OLEDMutex, portMAX_DELAY); // takes OLEDMutex
    RIT128x96x4StringDraw(written, xVal, yVal, 15 ); // draws to OLED
    xSemaphoreGive(OLEDMutex); // gives OLEDMutex
    while(key != select) // when only select is pressed
        xQueueReceive(xKeyQueue, &key, portMAX_DELAY); // gets key press
    while(key & select) // when select is unpressed
        xQueueReceive(xKeyQueue, &key, portMAX_DELAY); // gets key press

}

/*!
 * Options()
 *
 * Description: Draws the string, written, at the specified x and y position 
 * on the OLED display and then draws a column of options below it. It then 
 * allows an option to be selected using the up, down, and select keys and
 * returns the index of the selection.
 *
 * Operation: All writes to the OLED are protected with the OLEDMutex. First the
 * directions string, written, is drawn to the given position on the OLED and 
 * then each of the option strings are subsequently drawn below it in order.
 * A cursor is then placed next to the first option and it waits for a key
 * press. Once it gets a key press, it erases the cursor and then draws a new 
 * cursor above if the key press was up, below if the key press was down, and 
 * redraws it to the same position otherwise. In the case that the key press 
 * resulted in going off either end of the list, the cursor will wrap around.
 * Once a certain option is selected with hte select button, it waits for the 
 * select button to be unpressed before returning the index of the option
 * selected.
 * 
 * Return Values: Returns the index in the opt array of the menu that was 
 * selected.
 */
uint8_t Options( char* written, char** opt, uint8_t opts, 
                    uint8_t xVal, uint8_t yVal) {
    uint8_t sel = 1; // current menu selection
    uint8_t prev = 0; // previous key press
    uint8_t key; // saves the key press from the xKeyQueue
    key = 0;
    xSemaphoreTake(OLEDMutex, portMAX_DELAY); // takes OLEDMutex
    RIT128x96x4StringDraw(written, xVal, yVal, 15 ); // draws to OLED
    xSemaphoreGive(OLEDMutex); // gives OLEDMutex
    for(int i = 0; i < opts; i++) {     
        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // takes OLEDMutex
        RIT128x96x4StringDraw(opt[i], xVal + 10, yVal + 10 + i*10, 15); // draws
        xSemaphoreGive(OLEDMutex); // gives OLEDMutex
    }
    while(key != select) { // until selected
        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // takes OLEDMutex
        // draws cursor to appropriate position for current selection
        RIT128x96x4StringDraw(">", xVal, yVal + sel*10, 15); 
        xSemaphoreGive(OLEDMutex); // gives OLEDMutex
        xQueueReceive(xKeyQueue, &key, portMAX_DELAY); // waits for key press
        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // takes OLEDMutex
        RIT128x96x4StringDraw(" ", xVal, yVal + sel*10, 15); // erases cursor
        xSemaphoreGive(OLEDMutex); // gives OLEDMutex
        if(key != prev) { // if the key changed
            if (key == up ) // if only up is pressed
                sel--; // decrement selection and move up the screen
            else if (key == down) // if only down is pressed
                sel++; // increment selection and move down the screen
        }  
        prev = key; // update previous key press
        
        // wrap selection
        if(sel == 0) // if too low
            sel = opts; // set to highest
        else if (sel == (opts + 1)) // if too high
            sel = 1; // set to lowest

    }
    
    while(key & select) // wait for select to be unpressed
        xQueueReceive(xKeyQueue, &key, portMAX_DELAY); // wait for key press
    return sel; // return menu selection index
}

/*!
 * vMenu() 
 *
 * Description: This task controls the motors and draws the current status of
 * the motor controller to the OLED display. It can select between 3 modes.
 * In manual mode, the current key presses control the directions and speed of 
 * the motors. It also detects obstacles and blocks user inputs that would 
 * result in colliding with them. A back-up alert was also added that changes 
 * frequency with impending danger. The next mode is semi-autonomous. It allows
 * the user tp specify the direction they wish to drive and the time they to go
 * in that direction. The final mode is an automated drive mode that follows a
 * wall on the left side.
 *
 * Operation: The menu space on the OLED is first cleared and the menu is
 * selected using the options function. Currently a selection of semi is ignored
 * and the task repeats from the top. In the case that manual mode is selected,
 * Every 10 ms or new key press whichever takes longer, the keys are used to
 * choose the current setting of the motors' direction and speed as well as the 
 * state of the buzzer and the direction drawn to the OLED. The allowed inputs
 * are left, forward left, forward, forward right, right, reverse right,
 * reverse, and reverse left. All other inputs will result in standby. Also,
 * if there is an obstacle in front or behind when a forward or reverse input is
 * given respectively, the motors will be set to standby. After the motors are 
 * set to the given input, the PWMs are enabled (after the first run through 
 * this step is unneccesary). In order to return to the top menu select and all
 * 4 directional keys must be pressed.  In semi-autonomous mode, the user uses
 * the up and down keys to select the appropriate direction and seconds they
 * want to go. Once a value is selected an FSM is used to properly set the 
 * motors to match. If there is an obstacle in front or behind, it will stop the
 * current command and wait for the time to end before asking for a new input.
 * The autonomous mode follows a wall on the left side by going straight if 
 * there is a wall and turning left if it is too far and turning right if there
 * is an obstacle in front of the tank.
 * 
 * 
 * Return Values: None
 */
void vMenu(void *pvParameters) {
    static uint8_t key; // current key press
    static uint8_t menuSel; // records the current menu selection
    // used for semi
    static char* dirs[] = {"S  ", "L  ", "FL ", "F  ", "FR ", "R  ", "DL ", "D  ", "DR " "X "};
    static uint8_t bowObs; // 1 if there is an obstacle in front
    static uint8_t sternObs; // 1 if there is an obsatcle behind
  
    motorControl(forward, forward, highSpeed, highSpeed);
    //while(1) {
    //}
    
    while(1) {

        // Initially set the motor to standby
        motorControl(standby, standby, lowSpeed, highSpeed);
        PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, 0); // turn off the buzzer 
        xQueueReceive(xKeyQueue, &key, portMAX_DELAY); // wait for key
        
        // erase direction space
        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
        RIT128x96x4Clear();
        xSemaphoreGive(OLEDMutex); // gives OLEDMutex
        
        // Setup menu selection
        Directions  ("Prepare to be amazed.", 0, 15);
        char* dTypes[]= {"Manual", "Semi-Autonomous", "Autonomous"}; // menu options
        // select a menu
        menuSel = Options("Select Control Mode. ", dTypes, 3, 0, 15);

        // erase direction space
        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
        RIT128x96x4StringDraw("                     ", 0, 15, 15);
        RIT128x96x4StringDraw("                     ", 0, 25, 15);
        RIT128x96x4StringDraw("                     ", 0, 35, 15);
        RIT128x96x4StringDraw("                     ", 0, 45, 15);        
        if(menuSel == 1)
          RIT128x96x4StringDraw("Manual", 0, 45, 15);
        else if( menuSel ==2){
          RIT128x96x4StringDraw("Semi", 0, 45, 15);
        }else{
          RIT128x96x4StringDraw("Auto", 0, 45, 15);
        }
        
        xSemaphoreGive(OLEDMutex); // gives OLEDMutex
        
        
        xQueueReceive(xKeyQueue, &key, portMAX_DELAY); // wait for key
        
        
        while(key != 0x1F) { // leave menu selection by pressing all 5 keys
            if(menuSel == manual) { // manual mode
                xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                RIT128x96x4StringDraw("Manual", 0, 15, 15 ); // draw to OLED
                xSemaphoreGive(OLEDMutex); // gives OLEDMutex
                xQueueReceive(xKeyQueue, &key, portMAX_DELAY); // wait for key
                
                // Slave side only obstacle checks
                bowObs = 0; 
                sternObs = 0;
                
                // setup the buzzer frequency based on the average rear distance
                uint32_t ulPeriod = SysCtlClockGet() / 800;
                PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);
                PWMGenEnable(PWM_BASE, PWM_GEN_0);
                
                // choose direction based on key press
                // (each selection only runs through one case)
                switch(key) {
                    case left :
                        // Right motor forward full, Left motor reverse full
                        motorControl(reverse, forward, highSpeed, highSpeed);
                        PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, 0); // buzz off
                        // draw direction
                        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                        RIT128x96x4StringDraw("Left          ", 0, 25, 15 );
                        xSemaphoreGive(OLEDMutex); // gives OLEDMutex
                        break;
                    case right :
                        // Left motor forward full, Right motor reverse full 
                        motorControl(forward, reverse, highSpeed, highSpeed);
                        PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, 0); // buzz off
                        // draw direction
                        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                        RIT128x96x4StringDraw("Right         ", 0, 25, 15 );
                        xSemaphoreGive(OLEDMutex); // gives OLEDMutex
                        break;
                    case up :
                        if(!bowObs) {
                            // Both motors forward full speed 
                            motorControl(forward, forward, highSpeed, highSpeed);
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Forward       ", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex
                        } else {
                            // Not allowed movement with obstacles
                            motorControl(standby, standby, lowSpeed, lowSpeed);
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Bow Obstacle  ", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex
                        }
                        PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, 0); // buzz off
                        break;
                    case (up | left) :
                        if(!bowObs) {
                            // Both motors forward full speed 
                            motorControl(forward, forward, lowSpeed, highSpeed);
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Forward Left  ", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex
                        } else {
                            // Not allowed movement with obstacles
                            motorControl(standby, standby, lowSpeed, lowSpeed);
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Bow Obstacle  ", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex 
                        }
                        PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, 0); // buzz off  
                        break;
                    case (up | right) :
                        if(!bowObs) {
                            // Left motor full, Right motor slow forward 
                            motorControl(forward, forward, highSpeed, lowSpeed); 
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Forward Right ", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex 
                        } else {
                            // Not allowed movement with obstacles
                            motorControl(standby, standby, lowSpeed, lowSpeed);
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Bow Obstacle  ", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex 
                        }
                        PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, 0); // buzz off
                        break;
                    case down :
                        if(!sternObs) {
                            // Both motors reverse full speed 
                            motorControl(reverse, reverse, highSpeed, highSpeed);
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Reverse       ", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex  
                        } else {
                            // Not allowed movement with obstacles
                            motorControl(standby, standby, lowSpeed, lowSpeed);
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Stern Obstacle", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex 
                        }
                        PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, 1); // buzz on
                        break;
                    case (down | left) :
                        if(!sternObs) {
                            // Left motor full, Right motor slow, reverse 
                            motorControl(reverse, reverse, highSpeed, lowSpeed); 
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Reverse Left  ", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex  
                        } else {
                            // Not allowed movement with obstacles
                            motorControl(standby, standby, lowSpeed, lowSpeed);
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Stern Obstacle", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex 
                        }
                        PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, 1); // buzz on 
                        break;
                    case (down | right) :
                        if(!sternObs) {
                            // Right motor full, Left motor slow, reverse  
                            motorControl(reverse, reverse, lowSpeed, highSpeed);
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Reverse Right ", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex 
                        } else {
                            // Not allowed movement with obstacles
                            motorControl(standby, standby, lowSpeed, lowSpeed);
                            // draw direction
                            xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                            RIT128x96x4StringDraw("Stern Obstacle", 0, 25, 15 );
                            xSemaphoreGive(OLEDMutex); // gives OLEDMutex 
                        }
                        PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, 1); // buzz on
                        break;
                    default :
                        // All other cases are standby mode
                        motorControl(standby, standby, lowSpeed, lowSpeed);
                        // draw direction
                        xSemaphoreTake(OLEDMutex, portMAX_DELAY); // take OLEDMutex
                        RIT128x96x4StringDraw("Standby       ", 0, 25, 15 );
                        xSemaphoreGive(OLEDMutex); // gives OLEDMutex
                        PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, 0); // buzz on 
                }
                // turns on the PWMs
                PWMOutputState(PWM_BASE, PWM_OUT_3_BIT, 1);
                PWMOutputState(PWM_BASE, PWM_OUT_2_BIT, 1);
                vTaskDelay(10); // waits for 10 ms to spread the processor
            
            //static char* dirs[] = {"S", "L", "FL", "F", "FR", "R", "DL", "D", "DR" "X"};
            } else if(menuSel == semi) { // this is the semi-autonomous code.
              xQueueReceive(xKeyQueue, &key, portMAX_DELAY);
              while( key != 0x1F) {
                uint8_t deciS = 0;
                uint8_t direction = 1;
                uint8_t type = 1;
                char decimal[3];
                xSemaphoreTake(OLEDMutex, portMAX_DELAY);
                RIT128x96x4StringDraw("Semi", 0, 15, 15 );
                RIT128x96x4StringDraw("Direction F", 10, 23, 15);
                RIT128x96x4StringDraw("Duration  0.0", 10, 31, 15);
                xSemaphoreGive(OLEDMutex);
                xQueueReceive(xKeyQueue, &key, portMAX_DELAY);
                while(!(key & select)) {
                    vTaskDelay(100); // waits for 100 ms to spread the processor
                    xQueueReceive(xKeyQueue, &key, portMAX_DELAY); // gets key
                    if(type == 1) { // if controlling direction
                      
                        if(key == left) // decrement on left
                            direction--;
                        else if(key == right) // increment on right
                            direction++;
                        
                        // reset if out of bounds
                        if(direction == 0)
                            direction = 8;
                        else if(direction == 9)
                            direction = 1;
                        
                        // draw the current direction to the OLED
                        xSemaphoreTake(OLEDMutex, portMAX_DELAY);
                        RIT128x96x4StringDraw(dirs[direction], 70, 23, 15);
                        xSemaphoreGive(OLEDMutex);
    
                    } else if(type == 2) { // if controlling time
                      
                        if(key == left) // decrement on left
                            deciS--;
                        else if(key == right) // increment on right
                            deciS++;
                        
                        // reset if out of bounds
                        if(deciS == 0)
                            deciS = 50;
                        else if(deciS == 51)
                            deciS = 0;
                        
                        // draw the time to the OLED
                        decimal[0] = deciS / 10 + offsetZero;
                        decimal[1] = '.';
                        decimal[2] = deciS % 10 + offsetZero;
                        xSemaphoreTake(OLEDMutex, portMAX_DELAY);
                        RIT128x96x4StringDraw(decimal, 70, 31, 15);
                        xSemaphoreGive(OLEDMutex);
                    }
                    
                    // Toggle the active selection using up and down
                    if(key == up)
                        type--;
                    else if(key == down)
                        type++;
                    if(type == 3)
                        type = 0;
                    else if(type == 0)
                        type = 2;
                }
                
                // FSM for drive states. Note that the values for direction
                // match up with the index of the string comments in the dirs
                // array.
                if(direction == 1) // L
                  motorControl(reverse, forward, highSpeed, highSpeed);
                else if(direction == 2) // FL
                  motorControl(forward, forward, lowSpeed, highSpeed);
                else if(direction == 3) // F
                  motorControl(forward, forward, highSpeed, highSpeed);
                else if(direction == 4) // FR
                  motorControl(forward, forward, highSpeed, lowSpeed);
                else if(direction == 5) // R
                  motorControl(forward, reverse, highSpeed, highSpeed);
                else if(direction == 6) // DL
                  motorControl(reverse, reverse, lowSpeed, highSpeed);
                else if(direction == 7) // D
                  motorControl(reverse, reverse, highSpeed, highSpeed);
                else if(direction == 8) // DR
                  motorControl(reverse, reverse, highSpeed, lowSpeed);
                else // S
                  motorControl(standby, standby, lowSpeed, lowSpeed);
                
                // Update status
                xSemaphoreTake(OLEDMutex, portMAX_DELAY);
                RIT128x96x4StringDraw("Going   ", 70, 39, 15);
                xSemaphoreGive(OLEDMutex);
                
                // Turn on PWMs
                PWMOutputState(PWM_BASE, PWM_OUT_3_BIT, 1);
                PWMOutputState(PWM_BASE, PWM_OUT_2_BIT, 1);
                
                //Continue till time is up
                while(deciS > 0) {
                  vTaskDelay(100); // 1 decisecond
                  deciS--;
                  if(bowObs || sternObs) // if there is an obstacle
                    motorControl(standby, standby, lowSpeed, lowSpeed); // stop
                }
                
                // Update status
                xSemaphoreTake(OLEDMutex, portMAX_DELAY);
                RIT128x96x4StringDraw("Stopped", 70, 39, 15);
                xSemaphoreGive(OLEDMutex);
                
                // Stop the motors
                motorControl(standby, standby, lowSpeed, lowSpeed);

              }                
            }else { // autonomous mode is entirely controlled on the slave side
                while(key != up) { // check for exit autonomous
                  vTaskDelay(20); // spread the processor
                }
            
            }
        }
    }
}
    
