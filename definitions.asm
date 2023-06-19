.EQU F_CPU			= 8		; 8 Mhz   
 
.DEF TEMP_REG_A                 = r16
.DEF TEMP_REG_B                 = r17
.DEF DELAY_16_r			= r19
.DEF DELAY_8_r			= r20
.DEF DISP_NUM_L                 = r24		; LSB числа которое сейчас на индикаторе
.DEF DISP_NUM_H                 = r25		; MSB числа которое сейчас на индикаторе

.EQU DIGIT_1_PIN                = PD2		; Пин разряда индикатора 1
.EQU DIGIT_2_PIN                = PD3		; Пин разряда индикатора 2
.EQU DIGIT_3_PIN                = PD4		; Пин разряда индикатора 3
.EQU DIGIT_4_PIN                = PD5		; Пин разряда индикатора 4

.EQU LED_ERR_PIN		= PD6		; Светодиод который говорит о том, что МК в состоянии ошибки 

.EQU OW_LINE			= PB1		; Пин шины 1-Wire
.EQU OW_DDR			= DDRB
.EQU OW_PIN			= PINB
.DEF OW_CMD_r			= r8
.DEF OWFR			= r23		; Флаги состояния для 1-Wire интерфейса
.EQU OWPRF                      = 0		; 1-Wire Флаг присутствия
.EQU OWSB                       = 1		; Флаг передачи одного бита

; **** КОМАНДЫ ДАТЧИКА *****************************************
.EQU DS18B20_CMD_CONVERTTEMP    = 0x44
.EQU DS18B20_CMD_RSCRATCHPAD    = 0xbe
.EQU DS18B20_CMD_WSCRATCHPAD    = 0x4e
.EQU DS18B20_CMD_CPYSCRATCHPAD  = 0x48
.EQU DS18B20_CMD_RECEEPROM      = 0xb8
.EQU DS18B20_CMD_RPWRSUPPLY     = 0xb4
.EQU DS18B20_CMD_SEARCHROM      = 0xf0
.EQU DS18B20_CMD_READROM        = 0x33
.EQU DS18B20_CMD_MATCHROM       = 0x55
.EQU DS18B20_CMD_SKIPROM        = 0xcc
.EQU DS18B20_CMD_ALARMSEARCH    = 0xec

.EQU USI_LATCH_PIN              = PB0		; ST_CP на 74HC595
.EQU USI_DO_PIN                 = PB6		; DS на 74HC595
.EQU USI_CLK_PIN                = PB7		; SH_CP на 74HC595

.EQU SW_PORT                    = PORTB
.EQU SW_PIN                     = PINB
.EQU SW_PLUS_PIN                = PB2		; Кнопка "Минус"
.EQU SW_MINUS_PIN               = PB3		; Кнопка "Плюс"
.EQU SW_SET_PIN                 = PB4		; Кнопка "Установить"
    
.EQU UART_RX_PIN		= PD0
.EQU UART_TX_PIN		= PD1
    

.EQU MCU_STATE_DEFAULT          = 0x00		; Режим измерения температуры и сравнения с заданными параметрами
.EQU MCU_STATE_PROGRAM          = 0x01		; Режим настройки параметров в EEPROM
.EQU MCU_STATE_ERROR            = 0x02		; Режим ошибки, например когда не подключен датчик