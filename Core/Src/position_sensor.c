#include "position_sensor.h"
#include "delay.h"     // IWYU pragma: export
#include "gd32f30x.h"  // IWYU pragma: export
#include "systick.h"   // IWYU pragma: export

#ifdef Resolver_Position
// AD2S1210 Status Flags, need to be monitored though CCP
uint8_t               Resolver_Fault  = 0;
ErrStatus             AD2S1210_Config = ERROR;
ErrStatus             AD2S1210_Ready  = ERROR;
static inline void    SPI_Init(void);
static inline void    AD2S1210_Init(void);
static inline uint8_t spi_send_receive_byte(uint32_t spi_periph,
                                            uint8_t  byte);
#endif
#ifdef Encoder_Position
static inline void Encoder_Init(void);
#endif

void Position_Sensor_Init(void) {
#ifdef Resolver_Position
    SPI_Init();
    AD2S1210_Init();
#endif
#ifdef Encoder_Position
    Encoder_Init();
#endif
}

#ifdef Resolver_Position
void ReadPositionSensor(uint16_t* position_data) {
    if (AD2S1210_Ready == SUCCESS) {
        gpio_bit_reset(SAMPLEPORT, SAMPLEPin);  // Reset SAMPLE
        delay_us(1);
        gpio_bit_reset(WRPORT, WRPin);  // Reset WR

        uint8_t Hbyte = spi_send_receive_byte(SPI2, 0x00);
        uint8_t Lbyte = spi_send_receive_byte(SPI2, 0x00);
        uint8_t Fault = spi_send_receive_byte(SPI2, 0x00);

        *position_data
            = (Hbyte << 8) | Lbyte;  // Combine high and low byte
        Resolver_Fault = Fault;
        gpio_bit_set(WRPORT, WRPin);          // Set WR
        gpio_bit_set(SAMPLEPORT, SAMPLEPin);  // Set SAMPLE
    } else {
        *position_data = 0;     // If not ready, set position data to 0
        Resolver_Fault = 0xFF;  // All fault if not ready
    }
}
#endif
#ifdef Encoder_Position
void ReadPositionSensor(uint16_t* position_data) {
    *position_data = TIMER_CNT(TIMER3);
}
#endif
#ifdef Resolver_Position
static inline void SPI_Init(void) {
    spi_parameter_struct spi_init_struct;
    /* SPI2 use CK_APB1(60M) */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI2);
    rcu_periph_clock_enable(RCU_AF);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP,
                          ENABLE);  //! Must release JTAG pin !//

    /* 配置 SPI2_SCK(PB3), SPI2_MOSI(PB5) 为推挽输出 */
    gpio_init(GPIOB,
              GPIO_MODE_AF_PP,
              GPIO_OSPEED_50MHZ,
              GPIO_PIN_3 | GPIO_PIN_5);
    /* 配置 SPI2_MISO(PB4) 为浮空输入 */
    gpio_init(
        GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

    /* 软件 NSS，不使用硬件 NSS 引脚 */
    spi_init_struct.trans_mode  = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.frame_size  = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase
        = SPI_CK_PL_LOW_PH_2EDGE;             // CKPH=1 CKPL=0
    spi_init_struct.nss      = SPI_NSS_SOFT;  // 软件 NSS
    spi_init_struct.prescale = SPI_PSC_4;
    spi_init_struct.endian   = SPI_ENDIAN_MSB;
    spi_init(SPI2, &spi_init_struct);

    /* 启用 SPI2 */
    spi_enable(SPI2);
}

static inline void AD2S1210_Init(void) {
    gpio_init(A0PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, A0Pin);
    gpio_init(A1PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, A1Pin);
    gpio_init(RESETPORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, RESETPin);
    gpio_init(WRPORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, WRPin);
    gpio_init(
        SAMPLEPORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SAMPLEPin);

    gpio_bit_set(A0PORT, A0Pin);
    gpio_bit_set(A1PORT, A1Pin);
    gpio_bit_set(WRPORT, WRPin);
    gpio_bit_set(SAMPLEPORT, SAMPLEPin);
    gpio_bit_set(RESETPORT, RESETPin);

    //< Power up>//
    delay_ms(100);

    //< Reset >//
    gpio_bit_reset(RESETPORT, RESETPin);
    delay_ms(500);
    gpio_bit_set(RESETPORT, RESETPin);

    //< Enter configuration mode >//
    gpio_bit_set(A0PORT, A0Pin);
    gpio_bit_set(A1PORT, A1Pin);

    delay_us(5);

    //- Write Excitation -//
    gpio_bit_reset(WRPORT,
                   WRPin);  // WR must goes low before sending data
    delay_us(5);
    spi_send_receive_byte(SPI2, EXCITE_REG);  // Send EXCITE_REG
    gpio_bit_set(WRPORT,
                 WRPin);  // WR must goes high after sending data

    delay_us(5);

    gpio_bit_reset(WRPORT, WRPin);  // Reset WR
    delay_us(5);
    spi_send_receive_byte(
        SPI2, Excitation_Frequency);  // Send Data Excitation frequency
    gpio_bit_set(WRPORT, WRPin);      // Set WR

    delay_us(5);

    //- Read back while writing Control Register -//
    gpio_bit_reset(WRPORT, WRPin);  // Reset WR
    delay_us(5);

    if (Excitation_Frequency
        == spi_send_receive_byte(SPI2, CONTROL_REG)) {
        AD2S1210_Config = SUCCESS;
    } else {
        AD2S1210_Config &= ERROR;
    }
    gpio_bit_set(WRPORT, WRPin);  // Set WR

    delay_us(5);

    gpio_bit_reset(WRPORT, WRPin);  // Reset WR
    delay_us(5);
    spi_send_receive_byte(
        SPI2, Control_Register_Data);  // Send Data RES and EnRES
    gpio_bit_set(WRPORT, WRPin);       // Set WR

    delay_us(5);

    // # Use the ERROR Register to read back Data and exit #//
    gpio_bit_reset(WRPORT, WRPin);  // Reset WR
    delay_us(5);
    if (Control_Register_Data
        == spi_send_receive_byte(SPI2, ERROR_REG)) {
        AD2S1210_Config &= SUCCESS;
    } else {
        AD2S1210_Config &= ERROR;
    }
    gpio_bit_set(WRPORT, WRPin);  // Set WR

    delay_us(5);

    //< Exit configuration mode >//
    gpio_bit_reset(A0PORT, A0Pin);
    gpio_bit_reset(A1PORT, A1Pin);

    if (AD2S1210_Config == SUCCESS) {
        AD2S1210_Ready = SUCCESS;
    }
}

static inline uint8_t spi_send_receive_byte(uint32_t spi_periph,
                                            uint8_t  byte) {
    /* 等待发送缓冲区空 */
    while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE))
        ;

    /* 发送字节 */
    spi_i2s_data_transmit(spi_periph, byte);

    /* 等待接收缓冲区非空 */
    while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE))
        ;

    /* 返回接收到的字节 */
    return spi_i2s_data_receive(spi_periph);
}
#endif

static inline void Encoder_Init(void) {
    timer_parameter_struct    timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;

    /* enable the key clock */
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_TIMER3);
    rcu_periph_clock_enable(RCU_AF);

    gpio_init(GPIOD,
              GPIO_MODE_IPU,
              GPIO_OSPEED_50MHZ,
              GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14);
    gpio_pin_remap_config(GPIO_TIMER3_REMAP, ENABLE);

    timer_deinit(TIMER3);

    /* TIMER configuration */
    timer_initpara.prescaler         = 1 - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 4000 - 1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    /* TIMER3 CH0,1 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x05;  // 0x05

    timer_input_capture_config(TIMER3, TIMER_CH_0, &timer_icinitpara);
    timer_input_capture_config(TIMER3, TIMER_CH_1, &timer_icinitpara);
    timer_input_capture_config(TIMER3, TIMER_CH_2, &timer_icinitpara);

    /* TIMER_ENCODER_MODE2 */
    timer_quadrature_decoder_mode_config(TIMER3,
                                         TIMER_ENCODER_MODE2,
                                         TIMER_IC_POLARITY_RISING,
                                         TIMER_IC_POLARITY_RISING);
    timer_slave_mode_select(TIMER3, TIMER_ENCODER_MODE2);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);

    timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_CH2);
    timer_interrupt_enable(TIMER3, TIMER_INT_CH2);

    timer_enable(TIMER3);
}
