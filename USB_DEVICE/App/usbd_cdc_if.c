#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* Command received from Docklight ('0', '1', '2').
   Written here (USB interrupt), consumed in main.c main loop. */
volatile uint8_t cdcCommand = 0;
volatile uint8_t cdcCommandReady = 0;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS
};

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);

    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)pbuf;
    (void)length;

    switch (cmd)
    {
        case CDC_SEND_ENCAPSULATED_COMMAND:
        case CDC_GET_ENCAPSULATED_RESPONSE:
        case CDC_SET_COMM_FEATURE:
        case CDC_GET_COMM_FEATURE:
        case CDC_CLEAR_COMM_FEATURE:
        case CDC_SET_LINE_CODING:
        case CDC_GET_LINE_CODING:
        case CDC_SET_CONTROL_LINE_STATE:
        case CDC_SEND_BREAK:
        default:
            break;
    }

    return USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    uint32_t i;

    if (Buf != NULL && Len != NULL && *Len > 0)
    {
        for (i = 0; i < *Len; i++)
        {
            uint8_t c = Buf[i];

            /* Ignore line endings / spaces that Docklight or terminal may append */
            if (c == '\r' || c == '\n' || c == ' ')
                continue;

            /* Just store it; main.c validates it and prints the reply */
            cdcCommand = c;
            cdcCommandReady = 1;
        }
    }

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    return USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    USBD_CDC_HandleTypeDef *hcdc;

    if (hUsbDeviceFS.pClassData == NULL || Buf == NULL || Len == 0)
    {
        return USBD_FAIL;
    }

    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (hcdc->TxState != 0)
    {
        return USBD_BUSY;
    }

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);

    return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

static int8_t CDC_TransmitCplt_FS(
    uint8_t *Buf,
    uint32_t *Len,
    uint8_t epnum)
{
    (void)Buf;
    (void)Len;
    (void)epnum;

    return USBD_OK;
}