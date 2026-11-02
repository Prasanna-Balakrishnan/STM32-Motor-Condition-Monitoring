#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/* =============================================================================
 * Pin Mapping for STM32F411CEU6 Black Pill:
 *   MPU6050 (Software I2C): PB8 SCL, PB9 SDA
 *   L298N: PA0 ENA (TIM2_CH1 PWM), PB12 IN1, PB13 IN2
 *   LCD 16x2 (4-bit, RW to GND): PA1 RS, PA2 E, PA3 D4, PA4 D5, PA6 D6, PA8 D7
 *   PA5 Green LED (Healthy)     PB14 Healthy LED 2 (Healthy)
 *   PA7 Buzzer (Abnormal)       PC13 Red LED, active LOW (Abnormal)
 *   PB15 Motor OFF / low vibration LED
 *   PB0 ADC1_IN8
 *   Bluetooth USART1: PB6 TX, PB7 RX
 *   USB CDC: PA11 / PA12
 *
 * UART / USB / Bluetooth commands:
 *   '0' -> motor OFF
 *   '1' -> motor 20% power
 *   '2' -> motor 25% power
 * =============================================================================
 */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Shared with usbd_cdc_if.c (received characters are used as motor commands) */
extern volatile uint8_t cdcCommand;
extern volatile uint8_t cdcCommandReady;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Error_Handler(void);

static void Delay_us_Init(void);
static void Delay_us(uint32_t us);

void USB_Send(const char *message);

void BT_Init(void);
void BT_Send(const char *message);

void SCL_High(void);
void SCL_Low(void);
void SDA_High(void);
void SDA_Low(void);
void SDA_Output(void);
void SDA_Input(void);
void I2C_Delay(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_BusRecover(void);
uint8_t I2C_WriteByte(uint8_t data);
uint8_t I2C_ReadByte(uint8_t ack);

uint8_t MPU_FindAddress(void);
uint8_t MPU_Write(uint8_t reg, uint8_t data);
uint8_t MPU_Read(uint8_t reg, uint8_t *data, uint8_t length);
uint8_t MPU_Init(void);
uint8_t MPU_ReadValues(void);
static uint8_t MPU_ReadValuesRetry(void);
static void MPU_Reconfigure(void);
static uint8_t Calculate_RMS(void);

void Motor_Init(void);
void Motor_PWM_Init(void);
void Motor_Set_Speed(uint32_t duty);
void Motor_PWM_Start(void);
void Motor_PWM_Stop(void);
void Motor_Forward(uint32_t duty);
void Motor_Stop(void);

void ADC1_Init(void);
uint16_t ADC1_Read(void);

static void Pin_Write(GPIO_TypeDef *port, uint8_t pin, uint8_t level);
static void Indicators_Init(void);
static uint8_t Classify_RMS(uint32_t total_x10000);
static const char *Mode_Label(uint8_t mode);
static void Apply_Mode(uint8_t mode);
static void Print_Mode_Change(uint8_t mode);

static void LCD_Pins_Config(void);
static void LCD_Pulse_E(void);
static void LCD_Write_Nibble(uint8_t nibble);
static void LCD_Send(uint8_t value, uint8_t rs);
static void LCD_Command(uint8_t cmd);
static void LCD_Data(uint8_t data);
static void LCD_Print_Row(uint8_t row, const char *text);
static void LCD_Reset_Sequence(void);
static void LCD_Init(void);
static void LCD_Show_Status(uint8_t mode, uint32_t total_x10000);
static void LCD_Show_Text(const char *row0, const char *row1);

#define SCL_PIN 8
#define SDA_PIN 9

#define RED_LED_PIN       13   /* PC13, active LOW */
#define GREEN_LED_PIN     5    /* PA5  */
#define BUZZER_PIN        7    /* PA7  */
#define HEALTHY_LED2_PIN  14   /* PB14 */
#define OFF_LED_PIN       15   /* PB15 */

#define MOTOR_IN1_PIN     12
#define MOTOR_IN2_PIN     13

#define ADC_POT_PIN       0

#define LCD_RS_PIN        1
#define LCD_E_PIN         2
#define LCD_D4_PIN        3
#define LCD_D5_PIN        4
#define LCD_D6_PIN        6
#define LCD_D7_PIN        8

#define LCD_REFRESH_MS    2000U

#define BT_BAUD           9600U
#define BT_TX_BUF_SIZE    1024U

/* Modes (decided ONLY by the measured TOTAL_RMS) */
#define MODE_OFF          0
#define MODE_HEALTHY      1
#define MODE_ABNORMAL     2

/* =============================================================================
 * RMS THRESHOLDS (in g x 10000, i.e. 1.0000 g = 10000)
 *   TOTAL_RMS <= 0.1000 g           -> MOTOR OFF / low vibration
 *   0.1001 g <= TOTAL_RMS <= 1.15 g  -> HEALTHY
 *   TOTAL_RMS >  1.15 g             -> ABNORMAL
 * =============================================================================
 */
#define RMS_OFF_MAX_X10000        1000U    /* 0.1000 g */
#define RMS_HEALTHY_MAX_X10000    11500U   /* 1.1500 g */

/* Motor runs at a constant PWM at start; UART commands can change it */
#define DUTY_RUN          25U
#define DUTY_20           20U
#define DUTY_25           25U

#define RMS_SAMPLES        100
#define MIN_VALID_SAMPLES  70
#define SAMPLE_DELAY       5
#define ACCEL_SCALE        16384.0f
#define I2C_RETRIES        3

#define MPU_WHO_AM_I      0x75
#define MPU_PWR_MGMT_1    0x6B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_XOUT_H  0x3B

uint8_t mpuAddress = 0;
uint8_t mpuData[6];
uint8_t whoAmI = 0;
static uint8_t mpuPresent = 0;

int16_t Accel_X;
int16_t Accel_Y;
int16_t Accel_Z;

char usbBuffer[256];
static char usbTxCopy[256];

static uint8_t currentMode = MODE_OFF;
static uint32_t currentDuty = 0;

/* Result of the last RMS block (g x 10000) */
static uint32_t rmsX = 0, rmsY = 0, rmsZ = 0, rmsTotal = 0;
static uint16_t rmsValid = 0;

/* =============================================================================
 * BLUETOOTH (USART1, interrupt driven TX ring buffer)
 * =============================================================================
 */

static uint8_t btTxBuf[BT_TX_BUF_SIZE];
static volatile uint16_t btHead = 0;
static volatile uint16_t btTail = 0;

void BT_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOB->MODER   &= ~((3U << 12) | (3U << 14));
    GPIOB->MODER   |=  ((2U << 12) | (2U << 14));
    GPIOB->OTYPER  &= ~((1U << 6) | (1U << 7));
    GPIOB->OSPEEDR &= ~((3U << 12) | (3U << 14));
    GPIOB->OSPEEDR |=  ((1U << 12) | (1U << 14));
    GPIOB->PUPDR   &= ~((3U << 12) | (3U << 14));
    GPIOB->PUPDR   |=  (1U << 14);
    GPIOB->AFR[0]  &= ~((15U << 24) | (15U << 28));
    GPIOB->AFR[0]  |=  ((7U << 24) | (7U << 28));

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = (HAL_RCC_GetPCLK2Freq() + (BT_BAUD / 2U)) / BT_BAUD;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
}

void BT_Send(const char *message)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    while (*message)
    {
        uint16_t next = (uint16_t)((btHead + 1U) % BT_TX_BUF_SIZE);

        if (next == btTail)
            break;

        btTxBuf[btHead] = (uint8_t)*message++;
        btHead = next;
    }

    USART1->CR1 |= USART_CR1_TXEIE;

    __set_PRIMASK(primask);
}

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;

    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))
    {
        uint8_t c = (uint8_t)USART1->DR;

        if ((sr & USART_SR_RXNE) &&
            !(sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)))
        {
            if (c > ' ' && c < 0x7F)
            {
                cdcCommand = c;
                cdcCommandReady = 1;   /* handled in main loop: '0', '1', '2' */
            }
        }
    }

    if ((USART1->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE))
    {
        if (btTail != btHead)
        {
            USART1->DR = btTxBuf[btTail];
            btTail = (uint16_t)((btTail + 1U) % BT_TX_BUF_SIZE);
        }
        else
        {
            USART1->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* =============================================================================
 * MICROSECOND DELAY (DWT)
 * =============================================================================
 */

static void Delay_us_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

/* =============================================================================
 * USB CDC TRANSMISSION (also forwards to Bluetooth)
 * =============================================================================
 */

void USB_Send(const char *message)
{
    uint32_t start = HAL_GetTick();
    size_t len = strlen(message);

    if (len == 0)
        return;

    BT_Send(message);

    if (len > sizeof(usbTxCopy) - 1)
        len = sizeof(usbTxCopy) - 1;

    while (1)
    {
        USBD_CDC_HandleTypeDef *hcdc;

        if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
            return;

        hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

        if (hcdc == NULL)
            return;

        if (hcdc->TxState == 0)
        {
            memcpy(usbTxCopy, message, len);

            if (CDC_Transmit_FS((uint8_t *)usbTxCopy, (uint16_t)len) == USBD_OK)
                return;
        }

        if ((HAL_GetTick() - start) > 25U)
            return;

        HAL_Delay(1);
    }
}

/* =============================================================================
 * SOFTWARE I2C (~125 kHz)
 * =============================================================================
 */

void SCL_High(void) { GPIOB->BSRR = (1U << SCL_PIN); }
void SCL_Low(void)  { GPIOB->BSRR = (1U << (SCL_PIN + 16)); }
void SDA_High(void) { GPIOB->BSRR = (1U << SDA_PIN); }
void SDA_Low(void)  { GPIOB->BSRR = (1U << (SDA_PIN + 16)); }

void SDA_Output(void)
{
    GPIOB->MODER &= ~(3U << (SDA_PIN * 2));
    GPIOB->MODER |=  (1U << (SDA_PIN * 2));
}

void SDA_Input(void)
{
    GPIOB->MODER &= ~(3U << (SDA_PIN * 2));
}

void I2C_Delay(void)
{
    Delay_us(4);
}

void I2C_Start(void)
{
    SDA_Output();
    SDA_High();
    SCL_High();
    I2C_Delay();
    SDA_Low();
    I2C_Delay();
    SCL_Low();
    I2C_Delay();
}

void I2C_Stop(void)
{
    SDA_Output();
    SDA_Low();
    SCL_Low();
    I2C_Delay();
    SCL_High();
    I2C_Delay();
    SDA_High();
    I2C_Delay();
}

void I2C_BusRecover(void)
{
    SDA_Input();
    SCL_High();
    I2C_Delay();

    for (uint8_t i = 0; i < 9; i++)
    {
        if (GPIOB->IDR & (1U << SDA_PIN))
            break;

        SCL_Low();
        I2C_Delay();
        SCL_High();
        I2C_Delay();
    }

    I2C_Start();
    I2C_Stop();
}

uint8_t I2C_WriteByte(uint8_t data)
{
    SDA_Output();

    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & 0x80)
            SDA_High();
        else
            SDA_Low();

        I2C_Delay();
        SCL_High();
        I2C_Delay();
        SCL_Low();
        I2C_Delay();

        data <<= 1;
    }

    SDA_Input();
    I2C_Delay();
    SCL_High();
    I2C_Delay();

    uint8_t ack = ((GPIOB->IDR & (1U << SDA_PIN)) == 0);

    SCL_Low();
    I2C_Delay();
    SDA_Output();

    return ack;
}

uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t data = 0;

    SDA_Input();

    for (uint8_t i = 0; i < 8; i++)
    {
        data <<= 1;
        SCL_High();
        I2C_Delay();

        if (GPIOB->IDR & (1U << SDA_PIN))
            data |= 1U;

        SCL_Low();
        I2C_Delay();
    }

    SDA_Output();

    if (ack)
        SDA_Low();
    else
        SDA_High();

    I2C_Delay();
    SCL_High();
    I2C_Delay();
    SCL_Low();
    I2C_Delay();
    SDA_Input();

    return data;
}

/* =============================================================================
 * MPU6050
 * =============================================================================
 */

uint8_t MPU_Write(uint8_t reg, uint8_t data)
{
    I2C_Start();

    if (!I2C_WriteByte(mpuAddress << 1))  { I2C_Stop(); return 0; }
    if (!I2C_WriteByte(reg))              { I2C_Stop(); return 0; }
    if (!I2C_WriteByte(data))             { I2C_Stop(); return 0; }

    I2C_Stop();
    return 1;
}

uint8_t MPU_Read(uint8_t reg, uint8_t *data, uint8_t length)
{
    if (length == 0 || data == NULL)
        return 0;

    I2C_Start();

    if (!I2C_WriteByte(mpuAddress << 1))  { I2C_Stop(); return 0; }
    if (!I2C_WriteByte(reg))              { I2C_Stop(); return 0; }

    I2C_Start();

    if (!I2C_WriteByte((mpuAddress << 1) | 1U)) { I2C_Stop(); return 0; }

    for (uint8_t i = 0; i < length; i++)
        data[i] = I2C_ReadByte(i < (length - 1));

    I2C_Stop();
    return 1;
}

uint8_t MPU_FindAddress(void)
{
    uint8_t addresses[] = {0x68, 0x69, 0x70};

    for (uint8_t i = 0; i < sizeof(addresses); i++)
    {
        I2C_Start();
        uint8_t ack = I2C_WriteByte(addresses[i] << 1);
        I2C_Stop();

        if (ack)
        {
            mpuAddress = addresses[i];

            snprintf(usbBuffer, sizeof(usbBuffer),
                     "I2C device found at 0x%02X\r\n", mpuAddress);
            USB_Send(usbBuffer);

            return 1;
        }

        HAL_Delay(5);
    }

    return 0;
}

uint8_t MPU_Init(void)
{
    uint8_t ok = 0;

    for (uint8_t attempt = 0; attempt < I2C_RETRIES; attempt++)
    {
        if (MPU_Read(MPU_WHO_AM_I, &whoAmI, 1))
        {
            ok = 1;
            break;
        }

        I2C_BusRecover();
        HAL_Delay(5);
    }

    if (!ok)
    {
        USB_Send("ERROR: WHO_AM_I read failed\r\n");
        return 0;
    }

    snprintf(usbBuffer, sizeof(usbBuffer), "WHO_AM_I = 0x%02X\r\n", whoAmI);
    USB_Send(usbBuffer);

    if (!MPU_Write(MPU_PWR_MGMT_1, 0x00))
        return 0;

    HAL_Delay(50);

    if (!MPU_Write(MPU_ACCEL_CONFIG, 0x00))
        return 0;

    if (!MPU_Write(MPU_GYRO_CONFIG, 0x00))
        return 0;

    USB_Send("MPU initialization complete\r\n");
    return 1;
}

static void MPU_Reconfigure(void)
{
    MPU_Write(MPU_PWR_MGMT_1, 0x00);
    HAL_Delay(10);
    MPU_Write(MPU_ACCEL_CONFIG, 0x00);
    MPU_Write(MPU_GYRO_CONFIG, 0x00);
}

uint8_t MPU_ReadValues(void)
{
    uint8_t allFF = 1;
    uint8_t all00 = 1;

    if (!MPU_Read(MPU_ACCEL_XOUT_H, mpuData, 6))
        return 0;

    for (uint8_t i = 0; i < 6; i++)
    {
        if (mpuData[i] != 0xFF) allFF = 0;
        if (mpuData[i] != 0x00) all00 = 0;
    }

    if (allFF || all00)
        return 0;

    Accel_X = (int16_t)(((uint16_t)mpuData[0] << 8) | mpuData[1]);
    Accel_Y = (int16_t)(((uint16_t)mpuData[2] << 8) | mpuData[3]);
    Accel_Z = (int16_t)(((uint16_t)mpuData[4] << 8) | mpuData[5]);

    return 1;
}

static uint8_t MPU_ReadValuesRetry(void)
{
    for (uint8_t attempt = 0; attempt < I2C_RETRIES; attempt++)
    {
        if (MPU_ReadValues())
            return 1;

        I2C_BusRecover();

        if (attempt == 1)
            MPU_Reconfigure();
    }

    return 0;
}

/* =============================================================================
 * ADC (PB0 = ADC1 CH8)
 * =============================================================================
 */

void ADC1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    GPIOB->MODER |= (3U << (ADC_POT_PIN * 2));
    GPIOB->PUPDR &= ~(3U << (ADC_POT_PIN * 2));

    ADC->CCR &= ~ADC_CCR_ADCPRE;
    ADC->CCR |= ADC_CCR_ADCPRE_0;

    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;

    ADC1->SMPR2 &= ~(7U << (8 * 3));
    ADC1->SMPR2 |=  (4U << (8 * 3));

    ADC1->SQR1 = 0;
    ADC1->SQR3 = 8;
}

uint16_t ADC1_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    uint32_t timeout = 20000;
    while (!(ADC1->SR & ADC_SR_EOC))
    {
        if (--timeout == 0)
            return 0;
    }
    return (uint16_t)ADC1->DR;
}

/* =============================================================================
 * INDICATORS
 * =============================================================================
 */

static void Pin_Write(GPIO_TypeDef *port, uint8_t pin, uint8_t level)
{
    if (level)
        port->BSRR = (1U << pin);
    else
        port->BSRR = (1U << (pin + 16));
}

static void Indicators_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIOB->MODER &= ~((3U << (HEALTHY_LED2_PIN * 2)) | (3U << (OFF_LED_PIN * 2)));
    GPIOB->MODER |=  ((1U << (HEALTHY_LED2_PIN * 2)) | (1U << (OFF_LED_PIN * 2)));

    GPIOB->OTYPER  &= ~((1U << HEALTHY_LED2_PIN) | (1U << OFF_LED_PIN));
    GPIOB->OSPEEDR &= ~((3U << (HEALTHY_LED2_PIN * 2)) | (3U << (OFF_LED_PIN * 2)));
    GPIOB->PUPDR   &= ~((3U << (HEALTHY_LED2_PIN * 2)) | (3U << (OFF_LED_PIN * 2)));

    Pin_Write(GPIOB, HEALTHY_LED2_PIN, 0);
    Pin_Write(GPIOB, OFF_LED_PIN, 0);
}

/* =============================================================================
 * LCD (HD44780, 4-bit, write-only)
 * =============================================================================
 */

static void LCD_Pins_Config(void)
{
    const uint8_t pins[] = {LCD_RS_PIN, LCD_E_PIN, LCD_D4_PIN,
                            LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    for (uint8_t i = 0; i < sizeof(pins); i++)
    {
        uint8_t p = pins[i];

        Pin_Write(GPIOA, p, 0);

        GPIOA->MODER   &= ~(3U << (p * 2));
        GPIOA->MODER   |=  (1U << (p * 2));
        GPIOA->OTYPER  &= ~(1U << p);
        GPIOA->OSPEEDR &= ~(3U << (p * 2));
        GPIOA->PUPDR   &= ~(3U << (p * 2));

        if (p < 8)
            GPIOA->AFR[0] &= ~(15U << (p * 4));
        else
            GPIOA->AFR[1] &= ~(15U << ((p - 8) * 4));
    }
}

static void LCD_Pulse_E(void)
{
    Pin_Write(GPIOA, LCD_E_PIN, 1);
    Delay_us(5);
    Pin_Write(GPIOA, LCD_E_PIN, 0);
    Delay_us(5);
}

static void LCD_Write_Nibble(uint8_t nibble)
{
    uint32_t set = 0;
    uint32_t rst = 0;

    if (nibble & 0x01U) set |= (1U << LCD_D4_PIN); else rst |= (1U << LCD_D4_PIN);
    if (nibble & 0x02U) set |= (1U << LCD_D5_PIN); else rst |= (1U << LCD_D5_PIN);
    if (nibble & 0x04U) set |= (1U << LCD_D6_PIN); else rst |= (1U << LCD_D6_PIN);
    if (nibble & 0x08U) set |= (1U << LCD_D7_PIN); else rst |= (1U << LCD_D7_PIN);

    GPIOA->BSRR = set | (rst << 16);
    Delay_us(2);
    LCD_Pulse_E();
}

static void LCD_Send(uint8_t value, uint8_t rs)
{
    Pin_Write(GPIOA, LCD_RS_PIN, rs);
    Delay_us(2);
    LCD_Write_Nibble(value >> 4);
    LCD_Write_Nibble(value & 0x0FU);
    Delay_us(60);
}

static void LCD_Command(uint8_t cmd)
{
    LCD_Send(cmd, 0);

    if (cmd == 0x01 || cmd == 0x02)
        HAL_Delay(3);
}

static void LCD_Data(uint8_t data)
{
    LCD_Send(data, 1);
}

static void LCD_Print_Row(uint8_t row, const char *text)
{
    uint8_t count = 0;

    LCD_Command((row == 0) ? 0x80 : 0xC0);

    while (*text && count < 16)
    {
        LCD_Data((uint8_t)*text++);
        count++;
    }

    while (count < 16)
    {
        LCD_Data(' ');
        count++;
    }
}

static void LCD_Reset_Sequence(void)
{
    Pin_Write(GPIOA, LCD_RS_PIN, 0);
    Pin_Write(GPIOA, LCD_E_PIN, 0);
    Delay_us(10);

    LCD_Write_Nibble(0x03);
    HAL_Delay(5);
    LCD_Write_Nibble(0x03);
    HAL_Delay(2);
    LCD_Write_Nibble(0x03);
    HAL_Delay(2);
    LCD_Write_Nibble(0x02);
    HAL_Delay(2);

    LCD_Command(0x28);
    LCD_Command(0x0C);
    LCD_Command(0x06);
}

static void LCD_Init(void)
{
    LCD_Pins_Config();
    HAL_Delay(100);
    LCD_Reset_Sequence();
    LCD_Command(0x01);
}

static void LCD_Show_Text(const char *row0, const char *row1)
{
    LCD_Reset_Sequence();
    LCD_Print_Row(0, row0);
    LCD_Print_Row(1, row1);
}

/* Row 0 = mode (from RMS), row 1 = the RMS value that caused it */
static void LCD_Show_Status(uint8_t mode, uint32_t total_x10000)
{
    char row1[20];
    const char *row0;

    switch (mode)
    {
        case MODE_HEALTHY:  row0 = "MOTOR HEALTHY";  break;
        case MODE_ABNORMAL: row0 = "MOTOR ABNORMAL"; break;
        case MODE_OFF:
        default:            row0 = "MOTOR OFF";      break;
    }

    snprintf(row1, sizeof(row1), "RMS: %lu.%04lu g",
             (unsigned long)(total_x10000 / 10000U),
             (unsigned long)(total_x10000 % 10000U));

    LCD_Show_Text(row0, row1);
}

/* =============================================================================
 * MODE FROM RMS (the single place where the decision is made)
 * =============================================================================
 */

static uint8_t Classify_RMS(uint32_t total_x10000)
{
    if (total_x10000 > RMS_HEALTHY_MAX_X10000)
        return MODE_ABNORMAL;               /* > 1.15 g          */

    if (total_x10000 > RMS_OFF_MAX_X10000)
        return MODE_HEALTHY;                /* 0.1001 .. 1.15 g   */

    return MODE_OFF;                        /* 0 .. 0.1000 g     */
}

static const char *Mode_Label(uint8_t mode)
{
    if (mode == MODE_HEALTHY)  return "MOTOR_HEALTHY";
    if (mode == MODE_ABNORMAL) return "MOTOR_ABNORMAL";
    return "MOTOR_OFF";
}

/* LEDs + buzzer only (motor power is set by the UART commands) */
static void Apply_Mode(uint8_t mode)
{
    switch (mode)
    {
        case MODE_HEALTHY:
            Pin_Write(GPIOB, OFF_LED_PIN, 0);
            Pin_Write(GPIOA, GREEN_LED_PIN, 1);
            Pin_Write(GPIOB, HEALTHY_LED2_PIN, 1);
            Pin_Write(GPIOC, RED_LED_PIN, 1);      /* OFF (active low) */
            Pin_Write(GPIOA, BUZZER_PIN, 0);
            break;

        case MODE_ABNORMAL:
            Pin_Write(GPIOB, OFF_LED_PIN, 0);
            Pin_Write(GPIOA, GREEN_LED_PIN, 0);
            Pin_Write(GPIOB, HEALTHY_LED2_PIN, 0);
            Pin_Write(GPIOC, RED_LED_PIN, 0);      /* ON (active low)  */
            Pin_Write(GPIOA, BUZZER_PIN, 1);
            break;

        case MODE_OFF:
        default:
            Pin_Write(GPIOB, OFF_LED_PIN, 1);
            Pin_Write(GPIOA, GREEN_LED_PIN, 0);
            Pin_Write(GPIOB, HEALTHY_LED2_PIN, 0);
            Pin_Write(GPIOC, RED_LED_PIN, 1);
            Pin_Write(GPIOA, BUZZER_PIN, 0);
            mode = MODE_OFF;
            break;
    }

    currentMode = mode;
}

static void Print_Mode_Change(uint8_t mode)
{
    switch (mode)
    {
        case MODE_HEALTHY:
            USB_Send("\r\n>>> HEALTHY (RMS 0.1001-1.15 g) | LEDs PA5 + PB14 ON\r\n");
            break;

        case MODE_ABNORMAL:
            USB_Send("\r\n>>> ABNORMAL (RMS > 1.15 g) | PC13 LED ON + BUZZER ON\r\n");
            break;

        case MODE_OFF:
        default:
            USB_Send("\r\n>>> MOTOR OFF / LOW VIBRATION (RMS <= 0.1 g) | LED PB15 ON\r\n");
            break;
    }
}

/* =============================================================================
 * MOTOR (L298N)
 * =============================================================================
 */

void Motor_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIOB->MODER &= ~((3U << 24) | (3U << 26));
    GPIOB->MODER |=  ((1U << 24) | (1U << 26));

    GPIOB->OTYPER  &= ~((1U << 12) | (1U << 13));
    GPIOB->OSPEEDR &= ~((3U << 24) | (3U << 26));
    GPIOB->PUPDR   &= ~((3U << 24) | (3U << 26));

    GPIOB->BSRR = (1U << (12 + 16));
    GPIOB->BSRR = (1U << (13 + 16));
}

void Motor_PWM_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    GPIOA->MODER &= ~(3U << 0);
    GPIOA->MODER |=  (2U << 0);
    GPIOA->AFR[0] &= ~(15U << 0);
    GPIOA->AFR[0] |=  (1U << 0);
    GPIOA->OTYPER &= ~(1U << 0);
    GPIOA->PUPDR &= ~(3U << 0);

    TIM2->CR1 = 0;
    TIM2->PSC = 95;
    TIM2->ARR = 999;
    TIM2->CCR1 = 0;

    TIM2->CCMR1 &= ~(7U << 4);
    TIM2->CCMR1 |=  (6U << 4);
    TIM2->CCMR1 |=  (1U << 3);
    TIM2->CCER |= (1U << 0);
    TIM2->EGR = (1U << 0);
    TIM2->CR1 |= (1U << 7);
}

void Motor_Set_Speed(uint32_t duty)
{
    if (duty > 100U)
        duty = 100U;

    TIM2->CCR1 = (duty * (TIM2->ARR + 1U)) / 100U;
}

void Motor_PWM_Start(void)
{
    TIM2->CR1 |= (1U << 0);
}

void Motor_PWM_Stop(void)
{
    TIM2->CCR1 = 0;
    TIM2->EGR = (1U << 0);
}

void Motor_Forward(uint32_t duty)
{
    GPIOB->BSRR = (1U << 12);
    GPIOB->BSRR = (1U << (13 + 16));

    Motor_Set_Speed(duty);
    Motor_PWM_Start();
}

void Motor_Stop(void)
{
    Motor_PWM_Stop();

    GPIOB->BSRR = (1U << (12 + 16));
    GPIOB->BSRR = (1U << (13 + 16));
}

/* =============================================================================
 * RMS CALCULATION (AC RMS, gravity/DC removed)
 * Returns 1 and fills rmsX/rmsY/rmsZ/rmsTotal (g x 10000) on success,
 * returns 0 if the sensor was unstable. It does NOT change any output:
 * main() uses the result to update everything at the same moment.
 * =============================================================================
 */

static uint8_t Calculate_RMS(void)
{
    int64_t sx = 0, sy = 0, sz = 0;
    int64_t sxx = 0, syy = 0, szz = 0;
    uint16_t valid = 0;
    uint16_t bad = 0;

    for (int i = 0; i < RMS_SAMPLES; i++)
    {
        if (!MPU_ReadValuesRetry())
        {
            bad++;

            if (bad > (RMS_SAMPLES - MIN_VALID_SAMPLES))
                break;

            HAL_Delay(SAMPLE_DELAY);
            continue;
        }

        valid++;

        sx += Accel_X;
        sy += Accel_Y;
        sz += Accel_Z;

        sxx += (int64_t)Accel_X * Accel_X;
        syy += (int64_t)Accel_Y * Accel_Y;
        szz += (int64_t)Accel_Z * Accel_Z;

        HAL_Delay(SAMPLE_DELAY);
    }

    if (valid < MIN_VALID_SAMPLES)
    {
        snprintf(usbBuffer, sizeof(usbBuffer),
                 "ERROR: Sensor unstable - only %u/%u valid samples (check I2C wiring, MPU power, motor noise)\r\n",
                 (unsigned int)valid, (unsigned int)RMS_SAMPLES);
        USB_Send(usbBuffer);

        I2C_BusRecover();
        MPU_Reconfigure();
        return 0;
    }

    int64_t n = (int64_t)valid;

    int64_t vx = n * sxx - sx * sx;
    int64_t vy = n * syy - sy * sy;
    int64_t vz = n * szz - sz * sz;

    if (vx < 0) vx = 0;
    if (vy < 0) vy = 0;
    if (vz < 0) vz = 0;

    float nn = (float)n * (float)n;

    float rx = sqrtf((float)vx / nn) / ACCEL_SCALE;
    float ry = sqrtf((float)vy / nn) / ACCEL_SCALE;
    float rz = sqrtf((float)vz / nn) / ACCEL_SCALE;

    float total = sqrtf(rx * rx + ry * ry + rz * rz);

    rmsX     = (uint32_t)(rx    * 10000.0f + 0.5f);
    rmsY     = (uint32_t)(ry    * 10000.0f + 0.5f);
    rmsZ     = (uint32_t)(rz    * 10000.0f + 0.5f);
    rmsTotal = (uint32_t)(total * 10000.0f + 0.5f);
    rmsValid = valid;

    return 1;
}

/* =============================================================================
 * MAIN
 * =============================================================================
 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    SystemCoreClockUpdate();
    Delay_us_Init();
    MX_GPIO_Init();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PB8 / PB9: software I2C (open-drain + pull-up) */
    GPIOB->MODER &= ~((3U << 16) | (3U << 18));
    GPIOB->MODER |=  (1U << 16) | (1U << 18);
    GPIOB->OTYPER |= (1U << 8) | (1U << 9);
    GPIOB->PUPDR &= ~((3U << 16) | (3U << 18));
    GPIOB->PUPDR |=  (1U << 16) | (1U << 18);

    SCL_High();
    SDA_High();

    Motor_Init();
    Motor_PWM_Init();
    Indicators_Init();
    ADC1_Init();
    BT_Init();

    /* PA5 green LED, PA7 buzzer */
    GPIOA->MODER &= ~((3U << 10) | (3U << 14));
    GPIOA->MODER |=  (1U << 10) | (1U << 14);

    /* PC13 red LED */
    GPIOC->MODER &= ~(3U << 26);
    GPIOC->MODER |=  (1U << 26);

    LCD_Init();

    /* Start: indicators in "low vibration" state until the first RMS result */
    Apply_Mode(MODE_OFF);
    LCD_Show_Text("STARTING...", "");

    MX_USB_DEVICE_Init();
    HAL_Delay(2000);

    /* USB start-up can touch GPIOA: re-configure the LCD */
    LCD_Init();
    LCD_Show_Text("STARTING...", "");

    /* Motor starts OFF; send '1' (20%) or '2' (25%) over UART to run it */
    Motor_Stop();
    currentDuty = 0;

    USB_Send("\r\n========================================\r\n");
    USB_Send("    STM32F411 RMS Motor Monitor         \r\n");
    USB_Send("========================================\r\n");
    USB_Send("Automatic mode from TOTAL_RMS:\r\n");
    USB_Send("  0 - 0.1 g      -> MOTOR OFF / LOW\r\n");
    USB_Send("  0.1001 - 1.15 g -> HEALTHY\r\n");
    USB_Send("  > 1.15 g       -> ABNORMAL\r\n");
    USB_Send("UART commands: 0 = motor OFF, 1 = 20%, 2 = 25%\r\n");

    if (MPU_FindAddress() && MPU_Init())
    {
        mpuPresent = 1;
        USB_Send("MPU6050: OK (Address found and initialized)\r\n");
    }
    else
    {
        mpuPresent = 0;
        USB_Send("WARNING: MPU6050 not detected or init failed.\r\n");
        USB_Send("         Cannot classify motor without RMS.\r\n");
        LCD_Show_Text("NO SENSOR", "CHECK MPU6050");
    }

    USB_Send("\r\nSystem ready.\r\n");

    uint32_t lastUsbPrint = 0;
    uint32_t lastLcdRefresh = HAL_GetTick();
    uint8_t firstResult = 1;

    while (1)
    {
        /* UART command: '0' = motor OFF, '1' = 20% power, '2' = 25% power */
        if (cdcCommandReady)
        {
            uint8_t cmd = cdcCommand;
            cdcCommandReady = 0;

            if (cmd == '0')
            {
                Motor_Stop();
                currentDuty = 0;
                USB_Send("\r\n>>> CMD 0: MOTOR OFF\r\n");
            }
            else if (cmd == '1')
            {
                Motor_Forward(DUTY_20);
                currentDuty = DUTY_20;
                USB_Send("\r\n>>> CMD 1: MOTOR 20% POWER\r\n");
            }
            else if (cmd == '2')
            {
                Motor_Forward(DUTY_25);
                currentDuty = DUTY_25;
                USB_Send("\r\n>>> CMD 2: MOTOR 25% POWER\r\n");
            }
            /* any other character is ignored */
        }

        if (mpuPresent)
        {
            if (Calculate_RMS())
            {
                /* ONE decision, then everything is updated together */
                uint8_t newMode = Classify_RMS(rmsTotal);

                if (newMode != currentMode || firstResult)
                {
                    Apply_Mode(newMode);          /* LEDs + buzzer */
                    Print_Mode_Change(newMode);   /* USB + Bluetooth */
                    firstResult = 0;
                }

                LCD_Show_Status(currentMode, rmsTotal);   /* LCD */
                lastLcdRefresh = HAL_GetTick();

                snprintf(usbBuffer, sizeof(usbBuffer),
                         "%s | PWM: %lu%% | RMS_X: %lu.%04lu g | RMS_Y: %lu.%04lu g | RMS_Z: %lu.%04lu g | TOTAL_RMS: %lu.%04lu g | OK: %3u/%u\r\n",
                         Mode_Label(currentMode), (unsigned long)currentDuty,
                         (unsigned long)(rmsX / 10000U), (unsigned long)(rmsX % 10000U),
                         (unsigned long)(rmsY / 10000U), (unsigned long)(rmsY % 10000U),
                         (unsigned long)(rmsZ / 10000U), (unsigned long)(rmsZ % 10000U),
                         (unsigned long)(rmsTotal / 10000U), (unsigned long)(rmsTotal % 10000U),
                         (unsigned int)rmsValid, (unsigned int)RMS_SAMPLES);
                USB_Send(usbBuffer);
            }
            else
            {
                /* Sensor unstable: keep last mode, tell the user on the LCD */
                LCD_Show_Text("SENSOR ERROR", "CHECK WIRING");
                lastLcdRefresh = HAL_GetTick();
            }
        }
        else
        {
            if ((HAL_GetTick() - lastLcdRefresh) >= LCD_REFRESH_MS)
            {
                lastLcdRefresh = HAL_GetTick();
                LCD_Show_Text("NO SENSOR", "CHECK MPU6050");
            }

            if (HAL_GetTick() - lastUsbPrint >= 1000)
            {
                lastUsbPrint = HAL_GetTick();
                snprintf(usbBuffer, sizeof(usbBuffer),
                         "PWM: %lu%% | MPU6050: DISCONNECTED\r\n",
                         (unsigned long)currentDuty);
                USB_Send(usbBuffer);
            }
            HAL_Delay(50);
        }
    }
}

/* =============================================================================
 * CLOCK CONFIGURATION
 * =============================================================================
 */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif