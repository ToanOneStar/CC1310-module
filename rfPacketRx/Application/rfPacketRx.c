/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <stdio.h>      /* snprintf() */
#include <string.h>     /* strlen(), memcpy() */

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include <ti/devices/cc13x0/driverlib/ioc.h>
#include <ti/devices/cc13x0/driverlib/prcm.h>
#include <ti/devices/cc13x0/driverlib/uart.h>
#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>  /* SysCtrlClockGet() */

/* Board Header files */
#include "../Config/Board.h"

/* Application Header files */
#include "../Drivers/RF/RFQueue.h"
#include "../smartrf_settings/smartrf_settings.h"

/***** Defines *****/

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             30 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */

/* UART Configuration - send angles to STM32 */
#define UART_TX_IOID    IOID_4    /* DIO4 = UART TX (goes to STM32 RX) */
#define UART_RX_IOID    IOID_2    /* DIO2 = UART RX (goes to STM32 TX) */
#define UART_BAUD       9600      /* 9600: works reliably with most clock speeds */



/***** Prototypes *****/
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static int uart_init(void);
static void uart_send_str(const char* str);
static void uart_send_angles(float roll, float pitch, float yaw);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* ======= DEBUG VARIABLES (watch in CCS/RTX debugger) ======= */
volatile uint32_t g_db_rf_packet_ok   = 0; /* So lan RF packet OK (len >= 14) */
volatile uint32_t g_db_rf_total      = 0; /* Tong so RF packet nhan duoc */
volatile uint32_t g_db_uart_tx_calls  = 0; /* So lan goi UART TX */
volatile uint32_t g_db_uart_null     = 0; /* So lan uartHandle == NULL */
volatile uint32_t g_db_uart_len      = 0; /* Do dai du lieu UART gui di */
volatile int32_t  g_db_last_roll     = 0; /* Gia tri roll cuoi (x1000) */
volatile int32_t  g_db_last_pitch    = 0; /* Gia tri pitch cuoi (x1000) */
volatile int32_t  g_db_last_yaw      = 0; /* Gia tri yaw cuoi (x1000) */
volatile int32_t  g_db_last_len      = 0; /* Do dai packet cuoi nhan */

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)]
                                                  __attribute__((aligned(4)));
#else
#error This compiler is not supported.
#endif

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;


static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */

/***** UART Functions (DriverLib direct, no RTOS driver needed) *****/

/*
 * Initialize UART on DIO4 (TX) / DIO2 (RX) @ 9600 8N1
 *
 * Dùng UARTConfigSetExpClk() với SysCtrlClockGet() để TI DriverLib
 * tự tính IBRD/FBRD đúng từ clock thực (không hardcode giá trị).
 *
 * IOCPinTypeUart(base, RX_pin, TX_pin, CTS, RTS) — đúng thứ tự TI API:
 *   - Tham số 2 = RX pin → UART_RX_IOID = DIO2
 *   - Tham số 3 = TX pin → UART_TX_IOID = DIO4
 * Lỗi cũ: UART_TX_IOID đặt ở vị trí RX → DIO4 được config làm input,
 *          DIO2 được config làm output TX → ngược dây!
 */
static volatile uint32_t g_db_uart_clock = 0; /* clock CC1310 doc boi SysCtrlClockGet() */

static int uart_init(void) {
    /* Step 1: Enable UART power domain and clock */
    PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON) {}
    PRCMPeripheralRunEnable(PRCM_PERIPH_UART0);
    PRCMLoadSet();
    while (!PRCMLoadGet()) {}

    /* Step 2: Disable UART truoc khi cau hinh (bat buoc theo TI spec) */
    UARTDisable(UART0_BASE);

    /* Step 3: Lay clock thuc de debug va tinh baud dung
     * Xem g_db_uart_clock trong debugger: 48000000 = 48MHz */
    g_db_uart_clock = SysCtrlClockGet();

    /* Step 4: Config IO pins – CHU Y thu tu tham so TI API:
     *   IOCPinTypeUart(base, RX_ioid, TX_ioid, CTS_ioid, RTS_ioid)
     *   RX = DIO2 (nhan tu STM32 TX)
     *   TX = DIO4 (gui toi STM32 RX) */
    IOCPinTypeUart(UART0_BASE,
                   UART_RX_IOID,   /* DIO2 = UART RX (thu 2: RX) */
                   UART_TX_IOID,   /* DIO4 = UART TX (thu 3: TX) */
                   IOID_UNUSED, IOID_UNUSED);

    /* Step 5: Cau hinh baud 9600, 8N1 – TI tu tinh IBRD/FBRD tu clock thuc */
    UARTConfigSetExpClk(UART0_BASE,
                        g_db_uart_clock,  /* clock thuc (VD: 48000000) */
                        9600,
                        (UART_CONFIG_WLEN_8 |
                         UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    /* Step 6: Bat UART */
    UARTEnable(UART0_BASE);

    /* Step 7: Delay de UART on dinh */
    volatile uint32_t delay = 10000; while (delay--) {}

    g_db_uart_null = 1;  /* mark: init da duoc goi */
    return 0;
}

/*
 * Send string via UART (DriverLib blocking — UARTCharPut waits for ready)
 */
static void uart_send_str(const char* str) {
    while (*str) {
        UARTCharPut(UART0_BASE, (uint8_t)*str);
        str++;
    }
}

/*
 * Send raw byte via UART direct register (absolute fallback)
 */
static void uart_putc(uint8_t c) {
    /* Wait until TX holding register is empty */
    while (!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFE)) {}
    HWREG(UART0_BASE + UART_O_DR) = c;
}

/* Received angle data for debugging */
static volatile struct {
    float roll;
    float pitch;
    float yaw;
    uint16_t seq;
} g_rx_angles;

/* Bytes to float conversion */
static void bytes_to_float(uint8_t* buf, float* val) {
    union {
        float f;
        uint8_t b[4];
    } u;
    u.b[0] = buf[0];
    u.b[1] = buf[1];
    u.b[2] = buf[2];
    u.b[3] = buf[3];
    *val = u.f;
}

/*
 * UART send angles to STM32.
 *
 * Format: "R:<int> P:<int> Y:<int>\n"  (gia tri nhan 1000, la so nguyen)
 * Vi du: roll=0.427 → "R:427 P:-2584 Y:-125013\n"
 *
 * Ly do dung %d thay %f:
 *   STM32 arm-none-eabi-gcc khong ho tro sscanf("%f") mac dinh,
 *   can them "-u _scanf_float" vao linker. Dung %d luon hoat dong.
 *   STM32 nhan %d roi chia 1000.0f de co lai float.
 */
static void uart_send_angles(float roll, float pitch, float yaw) {
    g_db_uart_tx_calls++;

    /* Store scaled angles for debug watch window (x1000) */
    g_db_last_roll  = (int32_t)(roll  * 1000.0f);
    g_db_last_pitch = (int32_t)(pitch * 1000.0f);
    g_db_last_yaw   = (int32_t)(yaw   * 1000.0f);

    char buf[48];
    /* Gui so nguyen x1000: "R:427 P:-2584 Y:-125013\n" */
    int len = snprintf(buf, sizeof(buf), "R:%d P:%d Y:%d\n",
                       (int)g_db_last_roll,
                       (int)g_db_last_pitch,
                       (int)g_db_last_yaw);
    g_db_uart_len = len;

    uart_send_str(buf);
}

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE
};

/***** Function definitions *****/

void *mainThread(void *arg0)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }

    /* Initialize UART to send angles to STM32 */
    if (uart_init() != 0)
    {
        while(1); /* UART init failed */
    }

    /* TEST: gui chuoi co dinh truoc khi bat dau RF — kiem tra UART TX don thuan
     * Dung uart_putc truc tiep qua register DR de dam bao byte duoc gui */
    const char *test = "R:1.00 P:2.00 Y:3.00\n";
    while (*test) {
        uart_putc((uint8_t)*test);
        test++;
    }

    /* TEST 2: gui 10 byte 'X' de kiem tra STM32 nhan duoc gi khong */
    {
        int ii;
        for (ii = 0; ii < 10; ii++) {
            uart_putc('X');
        }
    }
    uart_putc('\n');

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }

    /* Modify CMD_PROP_RX command for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Enter RX mode and stay forever in RX */
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                               RF_PriorityNormal, &callback,
                                               RF_EventRxEntryDone);

    switch(terminationReason)
    {
        case RF_EventLastCmdDone:
            // A stand-alone radio operation command or the last radio
            // operation command in a chain finished.
            break;
        case RF_EventCmdCancelled:
            // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
            break;
        case RF_EventCmdAborted:
            // Abrupt command termination caused by RF_cancelCmd() or
            // RF_flushCmd().
            break;
        case RF_EventCmdStopped:
            // Graceful command termination caused by RF_cancelCmd() or
            // RF_flushCmd().
            break;
        default:
            // Uncaught error event
            while(1);
    }

    uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropRx)->status;
    switch(cmdStatus)
    {
        case PROP_DONE_OK:
            // Packet received with CRC OK
            break;
        case PROP_DONE_RXERR:
            // Packet received with CRC error
            break;
        case PROP_DONE_RXTIMEOUT:
            // Observed end trigger while in sync search
            break;
        case PROP_DONE_BREAK:
            // Observed end trigger while receiving packet when the command is
            // configured with endType set to 1
            break;
        case PROP_DONE_ENDED:
            // Received packet after having observed the end trigger; if the
            // command is configured with endType set to 0, the end trigger
            // will not terminate an ongoing reception
            break;
        case PROP_DONE_STOPPED:
            // received CMD_STOP after command started and, if sync found,
            // packet is received
            break;
        case PROP_DONE_ABORT:
            // Received CMD_ABORT after command started
            break;
        case PROP_ERROR_RXBUF:
            // No RX buffer large enough for the received data available at
            // the start of a packet
            break;
        case PROP_ERROR_RXFULL:
            // Out of RX buffer space during reception in a partial read
            break;
        case PROP_ERROR_PAR:
            // Observed illegal parameter
            break;
        case PROP_ERROR_NO_SETUP:
            // Command sent without setting up the radio in a supported
            // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
            break;
        case PROP_ERROR_NO_FS:
            // Command sent without the synthesizer being programmed
            break;
        case PROP_ERROR_RXOVF:
            // RX overflow observed during operation
            break;
        default:
            // Uncaught error event - these could come from the
            // pool of states defined in rf_mailbox.h
            while(1);
    }

    while(1);
}

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {
        /* Toggle pin to indicate RX */
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED2,
                           !PIN_getOutputValue(Board_PIN_LED2));

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(packet, packetDataPointer, (packetLength + 1));

        g_db_rf_total++;       /* dem tong so packet nhan */
        g_db_last_len = packetLength; /* luu do dai packet cuoi */

        /* Parse angle data: roll[0-3], pitch[4-7], yaw[8-11], seq[12-13] */
        if (packetLength >= 14) {
            g_db_rf_packet_ok++;  /* count valid RF packets */

            bytes_to_float(&packet[0], (float*)&g_rx_angles.roll);
            bytes_to_float(&packet[4], (float*)&g_rx_angles.pitch);
            bytes_to_float(&packet[8], (float*)&g_rx_angles.yaw);
            g_rx_angles.seq = (uint16_t)(packet[12] << 8) | packet[13];

            /* Forward angles to STM32 via UART */
            uart_send_angles(g_rx_angles.roll, g_rx_angles.pitch, g_rx_angles.yaw);
        }

        RFQueue_nextEntry();
    }
}
