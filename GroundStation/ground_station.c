/*
 *  ======== groundStation.c ========
 *
 *  Sends CMD_IMG_REQUEST (0x01) to the drone, then listens for:
 *   - CMD_IMG_BEGIN (0x10)   -> Start accumulating JPEG bytes
 *   - CMD_IMG_DATA  (0x02)   -> Append chunk to our buffer
 *   - CMD_IMG_END   (0x11)   -> Stop; we have the entire JPEG
 *
 *  Once CMD_IMG_END is received, we print the entire collected image in
 *  hexadecimal, wrapped between:
 *    ===BEGIN_JPEG===
 *    (hex bytes...)
 *    ===END_JPEG===
 *
 *  we parse that from the serial log to reconstruct the image.
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include "ti_drivers_config.h"

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>

/* RF Queue API */
#include "RFQueue.h"

/* Radio config (custom PHY) */
#include <ti_radio_config.h>

/* Command codes */
#define CMD_IMG_REQUEST  0x01
#define CMD_IMG_DATA     0x02
#define CMD_IMG_BEGIN    0x10
#define CMD_IMG_END      0x11

/*
 *  For demonstration, up to 32 KB
 */
#define MAX_IMAGE_SIZE   32768

static uint8_t  imageBuffer[MAX_IMAGE_SIZE];
static uint32_t imageIndex     = 0;
static uint32_t totalBytes     = 0;
static uint8_t  inJpegTransfer = 0;   // Are we within BEGIN..END?
static volatile uint8_t  imageReceived = 0;   // Flag indicating a full image was received

/* Each data packet can have up to 217 bytes of image data. */
#define PAYLOAD_LENGTH   220  /* Use 220 for throughput; must match drone. Adjust for testing (e.g., 30). */

/* Global variables */
static RF_Object  rfObject;
static RF_Handle  rfHandle;
static uint8_t    txPacket[PAYLOAD_LENGTH];

/* RX queue definitions */
#define NUM_DATA_ENTRIES   2
#define NUM_APPENDED_BYTES 2

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(rxDataEntryBuffer, 4)
static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 PAYLOAD_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=4
static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 PAYLOAD_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 PAYLOAD_LENGTH,
                                                                 NUM_APPENDED_BYTES)]
                                                                 __attribute__((aligned(4)));
#else
#error This compiler is not supported
#endif

static dataQueue_t        dataQueue;
static rfc_propRxOutput_t rxStatistics;

/* UART for debug printing */
static UART2_Handle uart2;

static void debugPrint(const char *fmt, ...)
{
    char buffer[128];
    size_t bytesWritten;
    va_list args;

    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    UART2_write(uart2, buffer, strlen(buffer), &bytesWritten);
}

/*
 *  ======== imgRxCallback ========
 *  Processes RX packets for CMD_IMG_BEGIN, CMD_IMG_DATA, CMD_IMG_END.
 */
static void imgRxCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {
        rfc_dataEntryGeneral_t *currentDataEntry = RFQueue_getDataEntry();
        if (!currentDataEntry)
            return;

        uint8_t packetLen = *((uint8_t *)&currentDataEntry->data);
        uint8_t *pktData  = (uint8_t *)(&currentDataEntry->data + 1);

        if (packetLen > 0 && packetLen <= PAYLOAD_LENGTH)
        {
            uint8_t cmd = pktData[0];

            switch(cmd)
            {
            case CMD_IMG_BEGIN:
                debugPrint("Ground: RX CMD_IMG_BEGIN\r\n");
                inJpegTransfer = 1;
                imageIndex     = 0;
                totalBytes     = 0;
                GPIO_toggle(CONFIG_GPIO_RLED); // LED feedback
                break;

            case CMD_IMG_DATA:
                if (inJpegTransfer)
                {
                    uint8_t seq       = pktData[1];
                    /* totalPkts = pktData[2];  // for info, if needed */
                    uint8_t dataLen   = packetLen - 3;  // minus header

                    // Append data to imageBuffer if we have space
                    if ((imageIndex + dataLen) <= MAX_IMAGE_SIZE)
                    {
                        memcpy(&imageBuffer[imageIndex], &pktData[3], dataLen);
                        imageIndex  += dataLen;
                        totalBytes  += dataLen;
                        GPIO_toggle(CONFIG_GPIO_RLED);

                        debugPrint("Ground: RX CMD_IMG_DATA seq=%u, dataLen=%u\r\n",
                                   seq, dataLen);
                    }
                    else
                    {
                        debugPrint("Ground: Buffer overflow, dropping data!\r\n");
                    }
                }
                else
                {
                    debugPrint("Ground: CMD_IMG_DATA but not inJpegTransfer\r\n");
                }
                break;

            case CMD_IMG_END:
                if (inJpegTransfer)
                {
                    inJpegTransfer = 0;
                    GPIO_toggle(CONFIG_GPIO_GLED);

                    //debugPrint("Ground: Total bytes received = %u\r\n", totalBytes);

                    /*
                     *  Now we print the entire JPEG in hex, bracketed by
                     *  "===BEGIN_JPEG===" and "===END_JPEG===" 
                     */
                    debugPrint("===BEGIN_JPEG===\r\n");
                    for (uint32_t i = 0; i < totalBytes; i++)
                    {
                        // Print without spaces/newlines for easier parsing
                        debugPrint("%02X", imageBuffer[i]);
                    }
                    debugPrint("\r\n===END_JPEG===\r\n");

                    imageReceived = 1;
                }
                else
                {
                    debugPrint("Ground: CMD_IMG_END but not inJpegTransfer\r\n");
                }
                break;

            default:
                debugPrint("Ground: Unknown cmd=0x%02X\r\n", cmd);
                break;
            }
        }
        else
        {
            debugPrint("Ground: Invalid packetLen=%u\r\n", packetLen);
        }

        RFQueue_nextEntry();
    }
    else if (e & RF_EventLastCmdDone)
    {
        debugPrint("Ground: RX EventLastCmdDone\r\n");
    }
    else
    {
        debugPrint("Ground: RX error/event e=0x%X\r\n", e);
        GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
    }
}

/*
 *  ======== mainThread ========
 *  Sends CMD_IMG_REQUEST, then enters RX mode to collect the JPEG.
 */
void *mainThread(void *arg0)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Open UART2 for debug prints */
    UART2_Params uart2Params;
    UART2_Params_init(&uart2Params);
    uart2 = UART2_open(CONFIG_UART2_0, &uart2Params);
    if (!uart2)
    {
        while(1); // failed
    }
    debugPrint("Ground: UART opened.\r\n");

    /* Setup LEDs */
    GPIO_setConfig(CONFIG_GPIO_RLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_GLED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_OFF);

    debugPrint("Ground: mainThread starting.\r\n");

    /* Configure RX data queue */
    if (RFQueue_defineQueue(&dataQueue,
                             rxDataEntryBuffer,
                             sizeof(rxDataEntryBuffer),
                             NUM_DATA_ENTRIES,
                             PAYLOAD_LENGTH + NUM_APPENDED_BYTES))
    {
        debugPrint("Ground: RFQueue_defineQueue failed.\r\n");
        GPIO_write(CONFIG_GPIO_RLED, CONFIG_GPIO_LED_ON);
        GPIO_write(CONFIG_GPIO_GLED, CONFIG_GPIO_LED_ON);
        while(1);
    }

    /* Open RF driver (custom PHY commands) */
    rfHandle = RF_open(&rfObject,
                       &RF_prop_2gfsk1mbps868_0,
                       (RF_RadioSetup*)&RF_cmdPropRadioDivSetup_2gfsk1mbps868_0,
                       &rfParams);

    if (!rfHandle)
    {
        debugPrint("Ground: RF_open failed.\r\n");
        while(1);
    }
    debugPrint("Ground: RF opened.\r\n");

    /* Set frequency */
    RF_runCmd(rfHandle,
              (RF_Op*)&RF_cmdFs_2gfsk1mbps868_0,
              RF_PriorityNormal,
              NULL,
              0);
    debugPrint("Ground: Frequency set.\r\n");

    /* Optional: Wait for user input to proceed */
    debugPrint("Ground: Press Enter to send CMD_IMG_REQUEST...\r\n");
    {
        char c;
        size_t br;
        UART2_read(uart2, &c, 1, &br);
    }

    /* --- Send CMD_IMG_REQUEST --- */
    txPacket[0] = CMD_IMG_REQUEST;
    RF_cmdPropTx_2gfsk1mbps868_0.pPkt   = txPacket;
    RF_cmdPropTx_2gfsk1mbps868_0.pktLen = 1;
    RF_cmdPropTx_2gfsk1mbps868_0.startTrigger.triggerType = TRIG_NOW;

    debugPrint("Ground: Sending CMD_IMG_REQUEST...\r\n");
    RF_runCmd(rfHandle,
              (RF_Op*)&RF_cmdPropTx_2gfsk1mbps868_0,
              RF_PriorityNormal,
              NULL,
              0);

    /* --- Switch to RX mode to receive the image (BEGIN, DATA, END) --- */
    RF_cmdPropRx_2gfsk1mbps868_0.pQueue                    = &dataQueue;
    RF_cmdPropRx_2gfsk1mbps868_0.rxConf.bAutoFlushIgnored = 1;
    RF_cmdPropRx_2gfsk1mbps868_0.rxConf.bAutoFlushCrcErr  = 1;
    RF_cmdPropRx_2gfsk1mbps868_0.maxPktLen                = PAYLOAD_LENGTH;
    RF_cmdPropRx_2gfsk1mbps868_0.startTrigger.triggerType  = TRIG_NOW;
    RF_cmdPropRx_2gfsk1mbps868_0.pktConf.bRepeatOk        = 1;
    RF_cmdPropRx_2gfsk1mbps868_0.pktConf.bRepeatNok       = 1;
    RF_cmdPropRx_2gfsk1mbps868_0.pOutput = (uint8_t *)&rxStatistics;

    debugPrint("Ground: Posting RX command. Awaiting image...\r\n");
    RF_postCmd(rfHandle,
               (RF_Op*)&RF_cmdPropRx_2gfsk1mbps868_0,
               RF_PriorityNormal,
               imgRxCallback,
               (RF_EventRxEntryDone | RF_EventLastCmdDone));

    /*
     *  The callback handles received image packets.
     *  After each image is printed, the main loop sends another request after a 1-second pause.
     */
    while (1)
    {
        if (imageReceived)
        {
            imageReceived = 0;
            RF_flushCmd(rfHandle, RF_CMDHANDLE_FLUSH_ALL, 0);
            debugPrint("Ground: Image received. Requesting next image...\r\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            txPacket[0] = CMD_IMG_REQUEST;
            debugPrint("Ground: Sending CMD_IMG_REQUEST...\r\n");
            RF_runCmd(rfHandle,
                      (RF_Op*)&RF_cmdPropTx_2gfsk1mbps868_0,
                      RF_PriorityNormal,
                      NULL,
                      0);
            debugPrint("Ground: Posting RX command. Awaiting image...\r\n");
            RF_postCmd(rfHandle,
                       (RF_Op*)&RF_cmdPropRx_2gfsk1mbps868_0,
                       RF_PriorityNormal,
                       imgRxCallback,
                       (RF_EventRxEntryDone | RF_EventLastCmdDone));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
