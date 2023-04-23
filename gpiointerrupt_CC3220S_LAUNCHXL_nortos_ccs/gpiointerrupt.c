/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== Project Title: gpiointerrupt.c ========
 */

/*
 *  ======== Includes / Headers ========
 */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/*
 *  ======== END: Includes / Headers ========
 */



/*
 *  ======== BEGIN: Global Variables ========
 */

/* Type Def.s */
//UART Display object for outputting Thermostat information
#define DISPLAY(x) UART_write(uart, &output, x);

//Task object
typedef struct task{
    int state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*TickFx)(int);
} task;

//GPIO
char L_Button = 0; //Left Button Flag Global Var.
char R_Button = 0; //Right Button Flag Global Var.

//UART
char output[64]; //UART Global Var.
int bytesToSend; //UART Global Var.

//TIMER
char TimerFlag = 0; //Timer Global Var.
int32_t seconds = 0;

//I2C: Temperature Variables
int16_t setPoint; //Thermostat user set temperature
int16_t temperature; //Temperature reading from sensor
int8_t heat; //Heater on global var.

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
                { 0x48, 0x0000, "11X" },
                { 0x49, 0x0000, "116" },
                { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1]; //Transmit Buffer
uint8_t rxBuffer[2]; //Receive Buffer

I2C_Transaction i2cTransaction;

// Driver Handles
I2C_Handle i2c;
UART_Handle uart;
Timer_Handle timer0;

//Task Scheduler Global Variables
task tasks[3]; //task objects

//task timing variables
char tasksNum = 3;
uint32_t taskPeriodGCD = 100000; // IN Microseconds (100ms)
uint32_t periodCheckB_Flag = 200000; // IN Microseconds (200ms)
uint32_t periodCheckTemp = 500000; // IN Microseconds (500ms)
uint32_t periodOutputDisplay = 1000000; // IN Microseconds (1000ms)

/*
 *  ======== END: Global Variables ========
 */



/*
 *  ======== BEGIN: Callbacks ========
 */

/* GPIO Callback */
//LEFT Button = CONFIG_GPIO_BUTTON_0
void gpioButtonFxn0(uint_least8_t index){
    //Button Flag - setPoint
    L_Button = 1; // SET: Left Button Flag
}

//RIGHT Button = CONFIG_GPIO_BUTTON_1
void gpioButtonFxn1(uint_least8_t index){
    //Button Flag + setPoint
    R_Button = 1; // SET: Right Button Flag

}

/* Timer Callback */
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1; //Raise TimerFlag
}

/*
 *  ======== END: Callbacks ========
 */



/*
 *  ======== BEGIN: Driver Init ========
 */

/* I2C Init */
// Make sure you call initUART() before calling this function.
void initI2C(void) {

    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */

    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    for (i=0; i<3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));

        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }

        DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if(found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r",
                         sensors[i].id, i2cTransaction.slaveAddress));
    }
    else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}

/* UART Init */
void initUART(void) {
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

/* Timer Init */
void initTimer(void) {
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/*
 *  ======== END: Driver Init ========
 */



/*
 *  ======== BEGIN: Driver Functions ========
 */

/* I2C: Read Temperature Function */
int16_t readTemp(void) {
    //int j;
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {

        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    }
    else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temperature;
}

/* DISPLAY Temperature and Settings*/
int outputDisplay(O){
        //<%02d, %02d, %d, %04d>, temperature, setPoint, heatOn, seconds
        DISPLAY( snprintf(output, 64, "<%02d, %02d, %d, %04d>\n\r", temperature, setPoint, heat, seconds));
        seconds += 1;
        return 0;
}

/* GPIO Load Function */
void GPIO_Load(){

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }
}

/*
 *  ======== END: Driver Functions ========
 */



/*
 *  ======== BEGIN: State Machine ========
 */

/* Button State Machine */
enum B_STATES {
    B_SM_INIT,      //Init State of State Machine
    B_SM_RAISED,    //Button flag raised
    B_SM_IDLE,      //Idle state
} B_State;

int buttonFlags(B_State){
    //Transitions
    switch(B_State){
        case B_SM_INIT:
            //Initial Load, Set to Idle
            B_State = B_SM_IDLE;
            break;

        case B_SM_IDLE:
            if (L_Button || R_Button){ //Check if a button flag has been raised
                B_State = B_SM_RAISED; //Switch state to raised
            }
            break;

        case B_SM_RAISED:
            if (!(L_Button || R_Button)){ //Checks if more than one button was pressed
                B_State = B_SM_IDLE; //Set to idle to clear flags
            }
            else if (L_Button || R_Button){ //If only one has been raised pass state to actions
                B_State = B_SM_RAISED;
            }
            break;

        default:
            B_State = B_SM_IDLE; // set to idle in case of error
            break;
    }

    //Actions
    switch(B_State){
        case B_SM_IDLE:
            L_Button = 0; //ensure flags are cleared
            R_Button = 0; //ensure flags are cleared
            break;

        case B_SM_RAISED:
            if (L_Button){
                setPoint -= 1; //Decrease user set temperature
                L_Button = 0; //Lower Button Flag
            }
            if (R_Button){
                setPoint += 1; //Increase user set temperature
                R_Button = 0; //Lower Button Flag
            }
            break;

        default:
            B_State = B_SM_IDLE; //set to idle
            break;
    }
    return B_State;
}

/* LED State Machine */
enum LED_STATE{
    LED_SM_INIT,
    LED_SM_ON,
    LED_SM_OFF,
    LED_SM_IDLE
} L_State;

int LED_State(L_State){
    //Transitions
    switch(L_State){
        case LED_SM_INIT:
            temperature = readTemp(); //Gather Temperature
            setPoint = readTemp(); //Set default user temperature to current temperature
            heat = 0;
            L_State = LED_SM_IDLE; //Set to Idle state
            break;

        case LED_SM_ON:
            //Compare temperature to user set temperature(setPoint)
            if (temperature < setPoint){ //IF less than
                  L_State = LED_SM_ON;  //Change State to ON
              }
              else if (temperature >= setPoint){    //IF greater than or equal to
                  L_State = LED_SM_IDLE;            //Change State to Idle
              }
              break;

        case LED_SM_OFF:
            //Compare temperature to user set temperature(setPoint)
            if (temperature >= setPoint){       //IF greater than or equal to
                L_State = LED_SM_OFF;           //Change State to OFF
            }
            else if (temperature < setPoint){   //IF less than
                L_State = LED_SM_IDLE;          //Change State to Idle
            }
            break;

        case LED_SM_IDLE:
            //Change to LED on if last measured temperature is lower than user set temperature
            if (temperature < setPoint){
                L_State = LED_SM_ON;
            }
            //Change to LED off if last measured temperature is greater than or equal to user set temperature
            if (temperature >= setPoint){
                L_State = LED_SM_OFF;
            }
            break;

        default:
            L_State = LED_SM_IDLE; //Default IDLE in case of ERROR
            break;
    }

    //Actions
    switch(L_State){
        case LED_SM_ON:
            //Turn on LED and SET heat to 1 (True)
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            heat = 1;
            break;

        case LED_SM_OFF:
            //Turn off LED and SET heat to 0 (False)
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            heat = 0;
            break;

        case LED_SM_IDLE:
            temperature = readTemp(); //IDLE check temperature
            break;

        default:
            L_State = LED_SM_IDLE; //Default to IDLE state
            break;
    }
    return L_State;
}

/*
 *  ======== END: State Machine ========
 */



/*
 *  ======== BEGIN: Task Scheduler ========
 */

/* Task Loop Function */
void taskScheduler() {
  unsigned char i;                                          //Temp Variable for iterating through tasks
  for (i = 0; i < tasksNum; ++i) {                          //For loop to loop through tasks
     if ( tasks[i].elapsedTime >= tasks[i].period ) {       //Check timing intervals
        tasks[i].state = tasks[i].TickFx(tasks[i].state);   //update tasks
        tasks[i].elapsedTime = 0;                           //reset elapsed time
     }
     tasks[i].elapsedTime += taskPeriodGCD;                 //increment with GCD of all tasks
  }
}

/* Create Tasks Function */
void loadTasks(){
    unsigned char i = 0;

    //Every 200ms Check button flags
    tasks[i].state = B_SM_INIT;                 //Set default state
    tasks[i].period = periodCheckB_Flag;        //Match to period
    tasks[i].elapsedTime = tasks[i].period;     //Match with starting period to trigger immediately
    tasks[i].TickFx = &buttonFlags;             //Call function when triggered
    ++i;                                        //iterate

    //Every 500ms read temperature, update LED
    tasks[i].state = LED_SM_INIT;               //Set default state
    tasks[i].period = periodCheckTemp;          //Match to period
    tasks[i].elapsedTime = tasks[i].period;     //Match with starting period to trigger immediately
    tasks[i].TickFx = &LED_State;               //Call function when triggered
    ++i;                                        //iterate

    //Every 1000ms Output to UART
    tasks[i].state = 1; //Set default state
    tasks[i].period = periodOutputDisplay;      //Match to period
    tasks[i].elapsedTime = tasks[i].period;     //Match with starting period to trigger immediately
    tasks[i].TickFx = &outputDisplay;           //Call function when triggered
}

/*
 *  ======== END: Task Scheduler ========
 */



/***********************************************************************/



/*
 *  ======== BEGIN: mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    initUART();
    initI2C();
    initTimer();
    GPIO_init();

    /* Load Configs */
    GPIO_Load(); //Moved to de-clutter Main
    loadTasks(); //Tasks loaded using a loading function for scalability

    /* Main Loop */
    while(1){
        /*Every 200ms check the button flags
        * Every 500ms read the temperature and update the LED
        * Every second output the following to the UART
        * All functionality is controlled through the task
        * scheduler funtion.
        */
        taskScheduler();

        //Call Timer in nested loop
        while(!TimerFlag){}
        TimerFlag = 0; //Lower TimerFlag

    }

}
/*
 *  ======== END: mainThread ========
 */



