/*
 * ======== gpiointerrupt.c ========
 * CC1310: Test GPIO DIO4
 */

#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* DriverLib Header files */
#include <ti/devices/cc13x0/driverlib/gpio.h>
#include <ti/devices/cc13x0/driverlib/ioc.h>
#include <ti/devices/cc13x0/driverlib/prcm.h>

/* Board Header */
#include "Board.h"

/* Debug LED on DIO4 */
#define LED_PIN 4

/* Global for debug */
volatile int g_gpio_init_done = 0;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Initialize Board */
    Board_init();

    /* Enable GPIO clock */
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
    while (!PRCMLoadGet()) {}

    /* Configure DIO4 as GPIO output */
    IOCPinTypeGpioOutput(LED_PIN);

    g_gpio_init_done = 1;  /* Flag for debug */

    /* Toggle LED every 500ms - test */
    while (1) {
        /* HIGH */
        GPIO_writeDio(LED_PIN, 1);
        usleep(500000);  /* 500ms */

        /* LOW */
        GPIO_writeDio(LED_PIN, 0);
        usleep(500000);  /* 500ms */
    }

    return (NULL);
}
