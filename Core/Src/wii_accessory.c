#include "main.h"
#include "wii_accessory.h"
#include "wii_accessory_config.h"
#include "string.h"

uint8_t wii_accessory_init_done = 0;
uint8_t data_format = 0xFF;
uint8_t device_type = 0xFF;

#ifdef WII_ACCESSORY_PORT
#ifdef WII_ACCESSORY_SCL_PIN
#ifdef WII_ACCESSORY_SDA_PIN

static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin,
        GPIO_PinState state, uint32_t timeout) {
    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = 1;

    for (; (state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);) // Wait until flag is set
            {
        if (timeout != HAL_MAX_DELAY) // Check for the timeout
        {
            if ((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout))
                ret = 0;
        }

        HAL_Delay(1);
    }
    return ret;
}

static void i2c_clear_busy_flag_erratum(I2C_HandleTypeDef *hi2c,
        GPIO_TypeDef *i2c_port, uint16_t scl_pin, uint16_t sda_pin) {
    // 2.13.7 I2C analog filter may provide wrong value, locking BUSY. STM32F10xx8 STM32F10xxB Errata sheet
    const int timeout = 100;

    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    // 1. Clear PE bit.
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_PE);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_I2C_DeInit(hi2c);
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = scl_pin;
    HAL_GPIO_Init(i2c_port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = sda_pin; // SDA
    HAL_GPIO_Init(i2c_port, &GPIO_InitStructure);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(i2c_port, scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(i2c_port, sda_pin, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(i2c_port, scl_pin, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(i2c_port, sda_pin, GPIO_PIN_SET, timeout);

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(i2c_port, sda_pin, GPIO_PIN_RESET);

    // 5. Check SDA Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(i2c_port, sda_pin, GPIO_PIN_RESET, timeout);

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(i2c_port, scl_pin, GPIO_PIN_RESET);

    // 7. Check SCL Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(i2c_port, scl_pin, GPIO_PIN_RESET, timeout);

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(i2c_port, scl_pin, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(i2c_port, scl_pin, GPIO_PIN_SET, timeout);

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(i2c_port, sda_pin, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(i2c_port, sda_pin, GPIO_PIN_SET, timeout);

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;

    GPIO_InitStructure.Pin = scl_pin;
    HAL_GPIO_Init(i2c_port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = sda_pin;
    HAL_GPIO_Init(i2c_port, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    HAL_Delay(1);

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    HAL_Delay(1);

    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
    HAL_Delay(1);

    // Call initialization function.
    HAL_I2C_Init(hi2c);
}

#endif
#endif
#endif

static uint8_t wii_accessory_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
        uint8_t offset, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    uint8_t errcode;
    errcode = HAL_I2C_Master_Transmit(hi2c, DevAddress, (void*) &offset, 1,
            Timeout);
    if (errcode != HAL_OK)
        return errcode;
    HAL_Delay(1);
    return HAL_I2C_Master_Receive(hi2c, DevAddress, pData, Size,
    WII_ACCESSORY_TIMEOUT);
}

static uint8_t wii_accessory_init(I2C_HandleTypeDef *hi2c) {
    uint8_t init_data[] = { 0xf0, 0x55, 0xfb, 0x00, 0xfe, 3 };
    uint8_t errcode;

    errcode = HAL_I2C_Master_Transmit(hi2c, WII_ACCESSORY_ADDRESS,
            (unsigned char*) &init_data[0], 2,
            WII_ACCESSORY_TIMEOUT);
#ifdef WII_ACCESSORY_PORT
#ifdef WII_ACCESSORY_SCL_PIN
#ifdef WII_ACCESSORY_SDA_PIN
    // 2.13.7 I2C analog filter may provide wrong value, locking BUSY. STM32F10xx8 STM32F10xxB Errata sheet
    if (errcode == HAL_BUSY) {
        i2c_clear_busy_flag_erratum(hi2c, WII_ACCESSORY_PORT,
        WII_ACCESSORY_SCL_PIN,
        WII_ACCESSORY_SDA_PIN);
    }
#endif
#endif
#endif
    if (errcode != HAL_OK)
        return errcode;
    HAL_Delay(1);
    errcode = HAL_I2C_Master_Transmit(hi2c, WII_ACCESSORY_ADDRESS,
            (unsigned char*) &init_data[2], 2,
            WII_ACCESSORY_TIMEOUT);
    if (errcode != HAL_OK)
        return errcode;
    HAL_Delay(1);
    errcode = HAL_I2C_Master_Transmit(hi2c, WII_ACCESSORY_ADDRESS,
            (unsigned char*) &init_data[4], 2,
            WII_ACCESSORY_TIMEOUT);
    return errcode;
}

static uint8_t get_bit(uint8_t data, uint8_t bitnum) {
    return (data >> bitnum) & 1;
}

static int8_t limit_int8(int16_t value) {
    if (value > 127)
        return 127;
    else if (value < -128)
        return -128;
    else
        return value;
}

uint8_t wii_accessory_poll(I2C_HandleTypeDef *hi2c,
        Wii_Accessory_Data *wii_accessory_data) {
    const int READ_LEN = 21;
    const uint8_t CONFIG_OFFSET = 0xFA;
    uint8_t i;
    uint8_t errcode;
    uint8_t config_data[6];
    uint8_t accessory_data[READ_LEN];
    // Clean up structure
    memset(wii_accessory_data, 0, sizeof(Wii_Accessory_Data));

#ifdef WII_ACCESSORY_DETECT_PIN
    // Wait for high level on DTCT pin
    if (!HAL_GPIO_ReadPin(WII_ACCESSORY_PORT, WII_ACCESSORY_DETECT_PIN)) {
        wii_accessory_init_done = 0;
        return WII_ACCESSORY_NOT_CONNECTED;
    }
#endif

    if (!wii_accessory_init_done) {
    	// Accessory is not initialized yet, need to initialize
        errcode = wii_accessory_init(hi2c);
        if (errcode != HAL_OK)
            return WII_ACCESSORY_NOT_CONNECTED;
        HAL_Delay(1);
        // Need to read back current data format and device type
        errcode = wii_accessory_read(hi2c, WII_ACCESSORY_ADDRESS, CONFIG_OFFSET,
                (void*) config_data, 6,
                WII_ACCESSORY_TIMEOUT);
        if (errcode != HAL_OK)
            return WII_ACCESSORY_POLL_ERROR;
        // Store this data
        data_format = config_data[4];
        device_type = config_data[5];
        wii_accessory_init_done = 1;
        HAL_Delay(1);
    }

    // Fill structure with info about controller
    wii_accessory_data->data_format = data_format;
    wii_accessory_data->device_type = device_type;

    // Lets read controller data
    errcode = wii_accessory_read(hi2c, WII_ACCESSORY_ADDRESS, 0,
            (void*) accessory_data, READ_LEN,
            WII_ACCESSORY_TIMEOUT);
    if (errcode != HAL_OK) {
        wii_accessory_init_done = 0;
        return WII_ACCESSORY_POLL_ERROR;
    }

    // Let's check that data is not corrupted
    accessory_data[18] = 0;	// for ultra shitty pro controller clones
    for (i = 10; i < READ_LEN; i++)
        if ((accessory_data[i] != 0) && (accessory_data[i] != 0xFF))
            return WII_ACCESSORY_INVALID_DATA;

    // Data decoding based on current data format
    switch (wii_accessory_data->data_format) {
    case 0:
        wii_accessory_data->jx = limit_int8(accessory_data[0] - 0x80);
        wii_accessory_data->jy = limit_int8(0x7fl - accessory_data[1]);
        wii_accessory_data->acc_x = accessory_data[2] - 0x80;
        wii_accessory_data->acc_y = accessory_data[3] - 0x80;
        wii_accessory_data->acc_z = accessory_data[4] - 0x80;
        wii_accessory_data->button_a = !get_bit(accessory_data[5], 1);
        wii_accessory_data->button_b = !get_bit(accessory_data[5], 0);
        break;
    case 1:
        wii_accessory_data->jx = limit_int8(
                ((accessory_data[0] & 0x3f) - 0x20) * 4);
        wii_accessory_data->rx = limit_int8(
                (((accessory_data[2] >> 7) | ((accessory_data[1] & 0xC0) >> 5)
                        | ((accessory_data[0] & 0xC0) >> 3)) - 0x10) * 8);
        wii_accessory_data->jy = limit_int8(
                ((accessory_data[1] & 0x3f) - 0x20) * -4);
        wii_accessory_data->ry = limit_int8(
                ((accessory_data[2] & 0x1f) - 0x10) * -8);
        wii_accessory_data->tl = ((accessory_data[3] >> 5)
                | ((accessory_data[2] & 0x60) >> 2)) * 8;
        wii_accessory_data->tr = (accessory_data[3] & 0x1f) * 8;
        wii_accessory_data->button_r = !get_bit(accessory_data[4], D_BTN_R);
        wii_accessory_data->button_start = !get_bit(accessory_data[4],
        										D_BTN_START);
        wii_accessory_data->button_home = !get_bit(accessory_data[4],
        										D_BTN_HOME);
        wii_accessory_data->button_select = !get_bit(accessory_data[4],
        										D_BTN_SELECT);
        wii_accessory_data->button_l = !get_bit(accessory_data[4], D_BTN_L);
        wii_accessory_data->dpad_down = !get_bit(accessory_data[4], D_BTN_DOWN);
        wii_accessory_data->dpad_right = !get_bit(accessory_data[4],
        										D_BTN_RIGHT);
        wii_accessory_data->dpad_up = !get_bit(accessory_data[5], D_BTN_UP);
        wii_accessory_data->dpad_left = !get_bit(accessory_data[5], D_BTN_LEFT);
        wii_accessory_data->button_zr = !get_bit(accessory_data[5], D_BTN_ZR);
        wii_accessory_data->button_x = !get_bit(accessory_data[5], D_BTN_X);
        wii_accessory_data->button_y = !get_bit(accessory_data[5], D_BTN_Y);
        wii_accessory_data->button_a = !get_bit(accessory_data[5], D_BTN_A);
        wii_accessory_data->button_b = !get_bit(accessory_data[5], D_BTN_B);
        wii_accessory_data->button_zl = !get_bit(accessory_data[5], D_BTN_ZL);
        break;
    case 2:
        wii_accessory_data->jx = limit_int8(accessory_data[0] - 0x80);
        wii_accessory_data->rx = limit_int8(accessory_data[1] - 0x80);
        wii_accessory_data->jy = limit_int8(0x7fl - accessory_data[2]);
        wii_accessory_data->ry = limit_int8(0x7fl - accessory_data[3]);
        wii_accessory_data->wtf = accessory_data[4];
        wii_accessory_data->tl = accessory_data[5];
        wii_accessory_data->tr = accessory_data[6];
        wii_accessory_data->button_r = !get_bit(accessory_data[7], D_BTN_R);
        wii_accessory_data->button_start = !get_bit(accessory_data[7],
        										D_BTN_START);
        wii_accessory_data->button_home = !get_bit(accessory_data[7],
        										D_BTN_HOME);
        wii_accessory_data->button_select = !get_bit(accessory_data[7],
        										D_BTN_SELECT);
        wii_accessory_data->button_l = !get_bit(accessory_data[7], D_BTN_L);
        wii_accessory_data->dpad_down = !get_bit(accessory_data[7], D_BTN_DOWN);
        wii_accessory_data->dpad_right = !get_bit(accessory_data[7],
        										D_BTN_RIGHT);
        wii_accessory_data->dpad_up = !get_bit(accessory_data[8], D_BTN_UP);
        wii_accessory_data->dpad_left = !get_bit(accessory_data[8], D_BTN_LEFT);
        wii_accessory_data->button_zr = !get_bit(accessory_data[8], D_BTN_ZR);
        wii_accessory_data->button_x = !get_bit(accessory_data[8], D_BTN_X);
        wii_accessory_data->button_y = !get_bit(accessory_data[8], D_BTN_Y);
        wii_accessory_data->button_a = !get_bit(accessory_data[8], D_BTN_A);
        wii_accessory_data->button_b = !get_bit(accessory_data[8], D_BTN_B);
        wii_accessory_data->button_zl = !get_bit(accessory_data[8], D_BTN_ZL);
        break;
    case 3:
        wii_accessory_data->jx = limit_int8(accessory_data[0] - 0x80);
        wii_accessory_data->rx = limit_int8(accessory_data[1] - 0x80);
        wii_accessory_data->jy = limit_int8(0x7fl - accessory_data[2]);
        wii_accessory_data->ry = limit_int8(0x7fl - accessory_data[3]);
        wii_accessory_data->tl = accessory_data[4];
        wii_accessory_data->tr = accessory_data[5];
        wii_accessory_data->button_r = !get_bit(accessory_data[6], D_BTN_R);
        wii_accessory_data->button_start = !get_bit(accessory_data[6],
        										D_BTN_START);
        wii_accessory_data->button_home = !get_bit(accessory_data[6],
        										D_BTN_HOME);
        wii_accessory_data->button_select = !get_bit(accessory_data[6],
        										D_BTN_SELECT);
        wii_accessory_data->button_l = !get_bit(accessory_data[6], D_BTN_L);
        wii_accessory_data->dpad_down = !get_bit(accessory_data[6], D_BTN_DOWN);
        wii_accessory_data->dpad_right = !get_bit(accessory_data[6],
        										D_BTN_RIGHT);
        wii_accessory_data->dpad_up = !get_bit(accessory_data[7], D_BTN_UP);
        wii_accessory_data->dpad_left = !get_bit(accessory_data[7], D_BTN_LEFT);
        wii_accessory_data->button_zr = !get_bit(accessory_data[7], D_BTN_ZR);
        wii_accessory_data->button_x = !get_bit(accessory_data[7], D_BTN_X);
        wii_accessory_data->button_y = !get_bit(accessory_data[7], D_BTN_Y);
        wii_accessory_data->button_a = !get_bit(accessory_data[7], D_BTN_A);
        wii_accessory_data->button_b = !get_bit(accessory_data[7], D_BTN_B);
        wii_accessory_data->button_zl = !get_bit(accessory_data[7], D_BTN_ZL);
        break;
    default:
        return WII_ACCESSORY_UNKNOWN_FORMAT;
    }

    return WII_ACCESSORY_OK;
}
