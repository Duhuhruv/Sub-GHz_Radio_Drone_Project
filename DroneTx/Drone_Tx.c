/*******************************************************************************
 *  Drone_Tx.c
 *
 *  - Defines DeviceFamily so TI-Drivers headers resolve without SysConfig.
 *  - Opens UART2 (CONFIG_UART2_0) on DIO2 (RX) / DIO3 (TX) for debug prints (via USB-CDC).
 *  - Optionally opens a second UART2 (CONFIG_UART2_1) on header pins for external
 *    "Luckfox" JPEG capture (detects EOI 0xFF 0xD9).
 *  - Cycles through one Flash image (image3_jpg) plus the UART slot on each
 *    CMD_IMG_REQUEST —or, if SEND_FLASH_ALWAYS==0, sends flash only once then
 *    only UART images thereafter.
 ******************************************************************************/

#define DeviceFamily_CC13X2_CC26X2
#include <ti/devices/DeviceFamily.h>

/* Toggle behavior: 1 = flash+UART cycle each time; 0 = flash once, then only UART */
#define SEND_FLASH_ALWAYS  0

/* Standard C headers */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>      // for flashSent flag

/* TI Drivers */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/UART2.h>
#include "ti_drivers_config.h"

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* RF Queue & PHY */
#include "RFQueue.h"
#include <ti_radio_config.h>

/* Flash-resident JPEG */
#include "image3.h"   // extern const unsigned char image3_jpg[]; extern const unsigned int image3_jpg_len;

/* Command codes */
#define CMD_IMG_REQUEST   0x01
#define CMD_IMG_BEGIN     0x10
#define CMD_IMG_DATA      0x02
#define CMD_IMG_END       0x11

/* RF payload sizing */
#define PAYLOAD_LENGTH    220
#define DATA_CHUNK_SIZE   (PAYLOAD_LENGTH - 3)

/* Only one flash image */
#define NUM_FLASH_IMAGES  1

/* RFQueue RX buffering */
#define NUM_DATA_ENTRIES   2
#define NUM_APPENDED_BYTES 2
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(rxDataEntryBuffer,4)
static uint8_t rxDataEntryBuffer[
    RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                   PAYLOAD_LENGTH,
                                   NUM_APPENDED_BYTES)
];
#else
static uint8_t rxDataEntryBuffer[
    RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                   PAYLOAD_LENGTH,
                                   NUM_APPENDED_BYTES)
] __attribute__((aligned(4)));
#endif
static dataQueue_t        dataQueue;
static rfc_propRxOutput_t rxStatistics;

/* Flash image table */
static const unsigned char *flashImages[NUM_FLASH_IMAGES] = {
    image3_jpg
};
static unsigned int flashImageLens[NUM_FLASH_IMAGES];

/* UART2 JPEG buffer (34 KB) */
#define UART2_BUFFER_SIZE 34000
static uint8_t   uart2StaticBuf[UART2_BUFFER_SIZE];
static uint8_t  *uart2Buf     = uart2StaticBuf;
static uint32_t  uart2Len     = 0;
static volatile uint8_t uart2Ready = 0;

/* Current TX image pointers */
static const unsigned char *imageData = NULL;
static uint32_t            imageSize  = 0;
static uint16_t            totalPackets = 0;

/* RF driver objects */
static RF_Object rfObject;
static RF_Handle rfHandle;
static uint8_t   txPacket[PAYLOAD_LENGTH];

/* RTOS sync and UART2 handles */
static SemaphoreHandle_t requestSemaphore = NULL;
static SemaphoreHandle_t txDoneSemaphore  = NULL;
static SemaphoreHandle_t uart2Mutex       = NULL;
static UART2_Handle      uart2Debug       = NULL;  // CONFIG_UART2_0
static UART2_Handle      uart2Rx          = NULL;  // CONFIG_UART2_1

/* Prevent double-handling of a request */
static volatile uint8_t requestReceived = 0;

/* Track whether we've sent flash once (for SEND_FLASH_ALWAYS==0) */
#if (SEND_FLASH_ALWAYS == 0)
static bool flashSent = false;
#endif

/*==============================================================================
 * debugPrint(fmt, ...) — thread-safe printf over UART2 (debug UART)
 *==============================================================================*/
static void debugPrint(const char *fmt, ...)
{
    char buf[128];
    size_t written;
    va_list ap;
    if (xSemaphoreTake(uart2Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        UART2_write(uart2Debug, buf, strlen(buf), &written);
        xSemaphoreGive(uart2Mutex);
    }
}

/*==============================================================================
 * requestRxCallback — RF callback, fires on CMD_IMG_REQUEST reception
 *==============================================================================*/
static void requestRxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    BaseType_t woken = pdFALSE;
    if (requestReceived) return;
    if (e & RF_EventRxEntryDone) {
        rfc_dataEntryGeneral_t *entry = RFQueue_getDataEntry();
        if (entry) {
            uint8_t len = *((uint8_t *)&entry->data);
            uint8_t *pkt = (uint8_t*)(&entry->data + 1);
            if (len && len <= PAYLOAD_LENGTH && pkt[0] == CMD_IMG_REQUEST) {
                requestReceived = 1;
                GPIO_toggle(CONFIG_GPIO_RLED);
                xSemaphoreGiveFromISR(requestSemaphore, &woken);
                portYIELD_FROM_ISR(woken);
            }
            RFQueue_nextEntry();
        }
    }
}

/*==============================================================================
 * UART2ReceiveTask — captures a JPEG via UART2_1, detects 0xFF 0xD9 EOI
 *==============================================================================*/
static void UART2ReceiveTask(void *arg)
{
    size_t br;
    uint8_t c, prev = 0;
    uint32_t idx = 0;
    debugPrint("Drone: UART2ReceiveTask started\r\n");
    for (;;) {
        UART2_readTimeout(uart2Rx, &c, 1, &br, 10);
        if (br == 1) {
            uart2Buf[idx++] = c;
            if (prev == 0xFF && c == 0xD9) {
                uart2Len   = idx;
                uart2Ready = 1;
                debugPrint("Drone: UART2 slot ready (%lu bytes)\r\n",
                           (unsigned long)uart2Len);
                idx = prev = 0;
                continue;
            }
            prev = c;
            if (idx >= UART2_BUFFER_SIZE) { idx = prev = 0; }
        }
    }
}

/*==============================================================================
 * imageTransmitTask — sends BEGIN→DATA→END packets over RF
 *==============================================================================*/
static void imageTransmitTask(void *param)
{
    debugPrint("Drone: TX slot size=%lu bytes, pkts=%u\r\n",
               (unsigned long)imageSize, totalPackets);

    /* CMD_IMG_BEGIN */
    memset(txPacket, 0, PAYLOAD_LENGTH);
    txPacket[0] = CMD_IMG_BEGIN;
    RF_cmdPropTx_2gfsk1mbps868_0.pPkt   = txPacket;
    RF_cmdPropTx_2gfsk1mbps868_0.pktLen = 1;
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx_2gfsk1mbps868_0,
              RF_PriorityNormal, NULL, RF_EventLastCmdDone);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* CMD_IMG_DATA */
    for (uint16_t seq = 0; seq < totalPackets; seq++) {
        uint32_t off = seq * DATA_CHUNK_SIZE;
        uint32_t sz  = (off + DATA_CHUNK_SIZE > imageSize)
                       ? (imageSize - off)
                       : DATA_CHUNK_SIZE;
        memset(txPacket, 0, PAYLOAD_LENGTH);
        txPacket[0] = CMD_IMG_DATA;
        txPacket[1] = (uint8_t)seq;
        txPacket[2] = (uint8_t)totalPackets;
        memcpy(&txPacket[3], &imageData[off], sz);

        RF_cmdPropTx_2gfsk1mbps868_0.pPkt   = txPacket;
        RF_cmdPropTx_2gfsk1mbps868_0.pktLen = 3 + sz;
        RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx_2gfsk1mbps868_0,
                  RF_PriorityNormal, NULL, RF_EventLastCmdDone);
        GPIO_toggle(CONFIG_GPIO_RLED);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    /* CMD_IMG_END */
    memset(txPacket, 0, PAYLOAD_LENGTH);
    txPacket[0] = CMD_IMG_END;
    RF_cmdPropTx_2gfsk1mbps868_0.pPkt   = txPacket;
    RF_cmdPropTx_2gfsk1mbps868_0.pktLen = 1;
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx_2gfsk1mbps868_0,
              RF_PriorityNormal, NULL, RF_EventLastCmdDone);

    GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
    xSemaphoreGive(txDoneSemaphore);
    vTaskDelete(NULL);
}

/*==============================================================================
 * mainThread() — initializes UARTs, RF, then serves image requests
 *==============================================================================*/
void *mainThread(void *arg0)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);
    rfParams.nInactivityTimeout = 0;

    /* Create semaphores & mutex */
    uart2Mutex       = xSemaphoreCreateMutex();
    requestSemaphore = xSemaphoreCreateBinary();
    txDoneSemaphore  = xSemaphoreCreateBinary();
    if (!uart2Mutex || !requestSemaphore || !txDoneSemaphore) {
        while (1);
    }

    /* Open debug UART */
    UART2_Params uparams;
    UART2_Params_init(&uparams);
    uparams.baudRate  = 115200;
    uparams.readMode  = UART2_Mode_BLOCKING;
    uparams.writeMode = UART2_Mode_BLOCKING;
    uart2Debug = UART2_open(CONFIG_UART2_0, &uparams);
    if (!uart2Debug) {
        while (1);
    }
    debugPrint("Drone: Debug UART opened\r\n");

    /* Open image-data UART if present */
#ifdef CONFIG_UART2_1
    uart2Rx = UART2_open(CONFIG_UART2_1, &uparams);
    if (uart2Rx) {
        debugPrint("Drone: Data UART opened for image RX\r\n");
        xTaskCreate(UART2ReceiveTask, "UART2 Rx", 1024, NULL, 1, NULL);
    } else {
        debugPrint("Warning: UART2_1 open failed\r\n");
    }
#else
    uart2Rx = NULL;
#endif

    /* LEDs off */
    GPIO_setConfig(CONFIG_GPIO_RLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_GLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

    debugPrint("Drone: System start → flash+UART slot mode=%d\r\n", SEND_FLASH_ALWAYS);

    /* RFQueue define */
    if (RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            PAYLOAD_LENGTH + NUM_APPENDED_BYTES)) {
        debugPrint("Drone: RFQueue define failed\r\n");
        while (1);
    }

    /* Open RF & set frequency */
    rfHandle = RF_open(&rfObject,
                       &RF_prop_2gfsk1mbps868_0,
                       (RF_RadioSetup*)&RF_cmdPropRadioDivSetup_2gfsk1mbps868_0,
                       &rfParams);
    if (!rfHandle) {
        debugPrint("Drone: RF_open failed\r\n");
        while (1);
    }
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs_2gfsk1mbps868_0,
              RF_PriorityNormal, NULL, 0);
    debugPrint("Drone: RF opened, freq set.\r\n");

    /* Configure RX command */
    RF_cmdPropRx_2gfsk1mbps868_0.pQueue                = &dataQueue;
    RF_cmdPropRx_2gfsk1mbps868_0.pOutput               = (uint8_t*)&rxStatistics;
    RF_cmdPropRx_2gfsk1mbps868_0.rxConf.bAutoFlushCrcErr  = 1;
    RF_cmdPropRx_2gfsk1mbps868_0.rxConf.bAutoFlushIgnored = 1;
    RF_cmdPropRx_2gfsk1mbps868_0.pktConf.bRepeatOk        = 1;
    RF_cmdPropRx_2gfsk1mbps868_0.pktConf.bRepeatNok       = 1;
    RF_cmdPropRx_2gfsk1mbps868_0.maxPktLen               = PAYLOAD_LENGTH;
    RF_cmdPropRx_2gfsk1mbps868_0.endTrigger.triggerType  = TRIG_NEVER;

    /* Initialize flash length */
    flashImageLens[0] = image3_jpg_len;

    /* Main loop */
    int cycleIdx = -1;
    int totalSlots = NUM_FLASH_IMAGES + (uart2Rx ? 1 : 0);

    for (;;) {
        requestReceived = 0;
        debugPrint("Drone: Posting RX command...\r\n");
        RF_postCmd(rfHandle,
                   (RF_Op*)&RF_cmdPropRx_2gfsk1mbps868_0,
                   RF_PriorityNormal,
                   requestRxCallback,
                   RF_EventRxEntryDone | RF_EventLastCmdDone);

        /* Decide which slot to send */
#if (SEND_FLASH_ALWAYS == 1)
        /* Cycle flash + UART as before */
        do {
            cycleIdx = (cycleIdx + 1) % totalSlots;
        } while (cycleIdx == NUM_FLASH_IMAGES && !uart2Ready);
#else
        /* Flash once, then only UART thereafter */
        if (!flashSent) {
            cycleIdx   = 0;          // flash slot
            flashSent = true;
        } else {
            cycleIdx = NUM_FLASH_IMAGES;  // UART slot only
        }
#endif

        /* Choose data pointer */
        if (cycleIdx < NUM_FLASH_IMAGES) {
            imageData = flashImages[cycleIdx];
            imageSize = flashImageLens[cycleIdx];
        } else {
            imageData  = uart2Buf;
            imageSize  = uart2Len;
            uart2Ready = 0;  // clear ready flag
        }
        totalPackets = (uint16_t)((imageSize + DATA_CHUNK_SIZE - 1) / DATA_CHUNK_SIZE);

        debugPrint("Drone: slot=%d size=%lu pkts=%u\r\n",
                   cycleIdx,
                   (unsigned long)imageSize,
                   totalPackets);

        /* Wait for request */
        xSemaphoreTake(requestSemaphore, portMAX_DELAY);
        debugPrint("Drone: Received CMD_IMG_REQUEST\r\n");

        /* Transmit */
        RF_flushCmd(rfHandle, RF_CMDHANDLE_FLUSH_ALL, 0);
        debugPrint("Drone: sending slot %d\r\n", cycleIdx);
        xTaskCreate(imageTransmitTask, "ImgTxTask", 2048, NULL, 2, NULL);

        /* Wait done */
        xSemaphoreTake(txDoneSemaphore, portMAX_DELAY);
        debugPrint("Drone: slot %d done.\r\n", cycleIdx);
    }
}
