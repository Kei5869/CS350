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
 *  ======== uartecho.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"


/*LED State Machine States*/
enum LED_STATES {
    LED_SM_START,   //INIT State for SM.
    LED_SM_OFF,     //Off State for SM.
    LED_SM_ON,      //On State for SM.
    LED_UART_OREAD, //"O" is Read State for SM.
    LED_UART_FREAD, //"F" is Read State for SM.
    LED_SM_IDLE     //IDLE State for SM. Allows Reset If Bad Input.
    } state;        //state variable.

// LED State Machine Function
void LED_SM(input) {

    //SM Transitions
    switch(input){
        // "O" or "o" Checking
        case 'O': //Allow Fall Through For Case Checking
        case 'o':
            state = LED_UART_OREAD; // Mark Letter Has Been Read
            break;

        // "N" or "n" Checking
        case 'N': //Allow Fall Through For Case Checking
        case 'n':
            if(state == LED_UART_OREAD){ //Check if "O" has been read
                state = LED_SM_ON;
            }
            break;

        // "F" or "f" Checking
        case 'F': //Allow Fall Through For Case Checking
        case 'f':
            if(state == LED_UART_OREAD){
                state = LED_UART_FREAD;
            }
            else if(state == LED_UART_FREAD){
                state = LED_SM_OFF;
            }
            break;

        //Default Case. Should only be reached on bad input.
        default:
            state = LED_SM_IDLE;//Rest SM if Bad Input.
            break;
    }
    //END Transitions

    // SM Actions
    switch(state){

        case LED_SM_START: //Init Start value is ON.
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            break;

        case LED_SM_ON: //Action to turn ON LED.
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            break;

        case LED_SM_OFF: //Action to turn OFF LED.
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            break;

        //Default Case. Should only be reached on IDLE state due to bad input.
        default:
            //Maintains Current LED status.
            break;
    }
    //END Actions

}
//END SM Function

//BEGIN Main
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    char        input;
    const char  echoPrompt[] = "Echoing characters:\r\n";
    UART_Handle uart;
    UART_Params uartParams;



    /* Call driver init functions */
    GPIO_init();
    UART_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    /* Turn on user LED to indicate successful initialization */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    UART_write(uart, echoPrompt, sizeof(echoPrompt));

    /* Loop forever echoing */
    while (1) {
        UART_read(uart, &input, 1);
        LED_SM(input); //Call SM with input data.
        UART_write(uart, &input, 1);

    }
}
//END Main
