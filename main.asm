;
; Project name: thermostat
; Description: Electronic thermostat on AVR Microcontroller
; Source code: https://github.com/sergeyyarkov/attiny2313a_thermostat
; Device: ATtiny2313A
; Device Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf
; Assembler: AVR macro assembler 2.2.7
; Clock frequency: 8 MHz External Crystal Oscillator
; Fuses: lfuse: 0xCF, hfuse: 0x9F, efuse: 0xFF, lock: 0xFF
;
; Written by Sergey Yarkov 22.01.2023

.LIST

.INCLUDE "definitions.asm"
.INCLUDE "macros.asm"


//<editor-fold defaultstate="collapsed" desc="Сегмент данных">
; **** СЕГМЕНТ ДАННЫХ ********************************************
.DSEG
.ORG SRAM_START

MCU_STATE:      .BYTE 1                     ; Текущее состояние МК
SRAM_TEMP_1:    .BYTE 2                     ; Хранения временного 16-бит числа в ячейке
DIGITS:         .BYTE 4                     ; Ячейки, где хранятся символы, для вывода на индикатор
CURRENT_DIGIT:  .BYTE 1                     ; Номер разряда индикатора, который сейчас горит
SW_FLAGS:       .BYTE 1                     ; Состояние кнопок
TEMP_L:		.BYTE 1			    ; Младший байт температуры
TEMP_H:		.BYTE 1			    ; Старший байт температуры
TEMP_F:		.BYTE 1			    ; Дробная часть
SETTING_TEMP_H:	.BYTE 1			    ; Уставка температуры (старший байт)
SETTING_TEMP_L:	.BYTE 1			    ; Уставка температуры (младший байт)
SETTING_HYST:	.BYTE 1			    ; Гистерезис: отклонение от уставки
SETTING_MODE:	.BYTE 1			    ; Режим работы: '1' - нагрев; '0' - 'охлаждение'
DEVICE_FAMILY_CODE: .BYTE 1		    ; Должно быть 0x10 для DS18B20
SRAM_TEMP_2:	.BYTE 1
SRAM_TEMP_3:	.BYTE 1
;//</editor-fold>


//<editor-fold defaultstate="collapsed" desc="Сегмент кода">
; **** СЕГМЕНТ КОДА **********************************************
.CSEG

//<editor-fold defaultstate="collapsed" desc="Вектора">
.ORG 0x00     
    rjmp 	RESET_vect
    
.ORG 0x000B
;    rjmp  PCINT0_vect
    reti

.ORG 0x000D   
    rjmp	TIMER0_COMPA_vect
//</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Прерывание: изменение состояния пина">
PCINT0_vect:
    push    r16
    push    r17
    push    r18
    push    r19
    in	    r16, SREG
    
;    display_off
    
;    in	    r17, SW_PIN
;    clr	    r18
;    ldi	    r18, (1<<SW_PLUS_PIN) | (1<<SW_MINUS_PIN)
;    and	    r17, r18
;    cp	    r17, r18
;    breq    _PCINT0_vect_end
    sbic    SW_PIN, SW_SET_PIN
    rjmp    _PCINT0_vect_end
    
    lds	    r18, MCU_STATE
    cpi	    r18, MCU_STATE_DEFAULT
    breq    _INTO_PROGRAM_STATE
    rjmp    _PCINT0_vect_end
    
_INTO_PROGRAM_STATE:
    ldi	    r17, MCU_STATE_PROGRAM
    sts	    MCU_STATE, r17
    
_PCINT0_vect_end:
    out	    SREG, r16
    pop	    r19
    pop	    r18
    pop	    r17
    pop	    r16
    reti//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Прерывание: динамическая индикация">
; **** ДИНАМИЧЕСКАЯ ИНДИКАЦИЯ ************************************
TIMER0_COMPA_vect:
    push      r20
    push      r21
    push      r16
    push      r17
    in r21, SREG

    lds       r20,  CURRENT_DIGIT
    cpi       r20,  5
    brge      _CLR_CURRENT_DIGIT          ; сброс активного разряда если >= 5
    rjmp      _indicate_1

_CLR_CURRENT_DIGIT:                   ; сброс текущего активного разряда в ноль
    clr     r20
    sts     CURRENT_DIGIT, r20

_indicate_1:
    cpi       r20, 0
    brne      _indicate_2

    cbi       PORTD, DIGIT_2_PIN
    cbi       PORTD, DIGIT_3_PIN
    cbi       PORTD, DIGIT_4_PIN
    brts      PC+2
    rjmp      PC+3
    ldi	      TEMP_REG_A, 11 ; // -
    rjmp      PC+3
    lds	      TEMP_REG_A, DIGITS+2
    tst	      TEMP_REG_A
    breq      PC+3
    sbi       PORTD, DIGIT_1_PIN
    rjmp      PC+2
    cbi       PORTD, DIGIT_1_PIN
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

_indicate_2:
    cpi       r20, 1
    brne      _indicate_3

    cbi       PORTD, DIGIT_1_PIN
    cbi       PORTD, DIGIT_3_PIN
    cbi       PORTD, DIGIT_4_PIN
    lds       TEMP_REG_A, DIGITS+1
    lds	      r19, DIGITS+2
    or	      TEMP_REG_A, r19
    breq      PC+3
    sbi	      PORTD, DIGIT_2_PIN
    rjmp      PC+2
    cbi       PORTD, DIGIT_2_PIN
    lds       TEMP_REG_A, DIGITS+1
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

_indicate_3:
    cpi       r20, 2
    brne      _indicate_4

    cbi       PORTD, DIGIT_1_PIN
    cbi       PORTD, DIGIT_2_PIN
    cbi       PORTD, DIGIT_4_PIN
    sbi       PORTD, DIGIT_3_PIN
    lds       TEMP_REG_A, DIGITS
    rcall     DISPLAY_DECODER
    ; зажигаем точку
    mov	      r16, r0
    cbr	      r16, (1<<7)
    mov	      r0, r16
    rcall     USI_TRANSMIT

_indicate_4:
    cpi       r20, 3
    brne      _indicate_exit

    cbi       PORTD, DIGIT_1_PIN
    cbi       PORTD, DIGIT_2_PIN
    cbi       PORTD, DIGIT_3_PIN
    sbi       PORTD, DIGIT_4_PIN
    lds       TEMP_REG_A, TEMP_F
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

_indicate_exit:
    inc       r20
    sts       CURRENT_DIGIT, r20
    out SREG, r21
    
    pop	r17
    pop r16
    pop r21
    pop r20
    reti
;//</editor-fold>

; **** СТАРТ ПРОГРАММЫ *******************************************
RESET_vect:
    ldi       TEMP_REG_A, LOW(RAMEND)
    out       SPL, TEMP_REG_A

//<editor-fold defaultstate="collapsed" desc="Инициализация МК (настрока портов и переферии)">
; **** ПРОЦЕСС ИНИЦИАЛИЗАЦИИ МК **********************************
MCU_INIT:
    ; **** ИНИЦИАЛИЗАЦИЯ ПИНОВ *************************************
    outi      r16, DDRD, (1<<LED_ERR_PIN) | (1<<DIGIT_1_PIN) | (1<<DIGIT_2_PIN) | (1<<DIGIT_3_PIN) | (1<<DIGIT_4_PIN) | (1<<UART_TX_PIN) | (1<<RELAY_PIN)
    outi      r16, DDRB, (1<<USI_CLK_PIN) | (1<<USI_DO_PIN) | (1<<USI_LATCH_PIN) | (0<<SW_PLUS_PIN) | (0<<SW_MINUS_PIN) | (0<<SW_SET_PIN) | (1<<BUZZER_PIN)
    outi      r16, PORTB, (1<<SW_PLUS_PIN) | (1<<SW_MINUS_PIN) | (1<<SW_SET_PIN)

    ; **** ИНИЦИАЛИЗАЦИЯ ТАЙМЕРА 0 **********************************
    outi      r16, TCCR0A, (1<<WGM01)             ; режим CTC Compare A
    outi      r16, TCCR0B, (1<<CS02) | (1<<CS00)  ; 1024 делитель
    outi      r16, OCR0A, 25                      ; число для сравнения. (60Hz)

    ; **** ПРЕРЫВАНИЕ ПО ИЗМЕНЕНИЮ СОСТОЯНИЯ ПИНОВ ******************
    outi      r16, GIMSK, (1<<PCIE0)
    outi      r16, PCMSK0, (1<<PCINT2) | (1<<PCINT3) | (1<<PCINT4)          ; для кнопок
  
    ; **** ИНИЦИАЛИЗАЦИЯ USART **************************************
       outi      r16, UBRRL, LOW(51)		    ; 9600 БОД
       outi      r16, UBRRH, HIGH(51)		    ; 9600 БОД
       outi      r16, UCSRB, (1<<TXEN)		    ; Включение передачии
       outi      r16, UCSRC, (1<<UCSZ1) | (1<<UCSZ0)   ; Асинхронный режим, 8 бит фрейм, 1 стоповый бит
    
    ; **** ИНИЦИАЛИЗАЦИЯ ДАННЫХ В ОЗУ ******************************
    clr     r1
    sts     CURRENT_DIGIT,  r1

    ldi     r16, 0x00
    sts     MCU_STATE,      r16	    ; переводим МК сразу в режим измерения температуры
    
    ; настройка параметров
    ldi	    r16, DEFAULT_SETTING_TEMP_L
    sts	    SETTING_TEMP_L, r16			; уставка LOW
    
    ldi	    r16, DEFAULT_SETTING_TEMP_H
    sts	    SETTING_TEMP_H, r16			; уставка HIGH
    
    ldi	    r16, DEFAULT_SETTING_HYST
    sts	    SETTING_HYST, r16			; гистерезис
    
    ldi	    r16, DEFAULT_SETTING_MODE
    sts	    SETTING_MODE, r16			; режим работы
    
    clr	    r16
    sts	    TEMP_L, r16
    sts	    TEMP_H, r16
    ldi	    r16, 0xf0
    sts	    TEMP_F, r16
        
    ; **** СТАРТУЕМ ************************************************
    
    rcall   RD_F_CODE		; проверяем что датчик есть на шине
    
    lds	    r16, DEVICE_FAMILY_CODE
    cpi	    r16, 0x28		; код семейства для DS18B20 = 0x28
    brne    ERR_FAMILY_CODE
    rjmp    START_PROGRAM
    
ERR_FAMILY_CODE:
    ldi	    r17, MCU_STATE_ERROR
    sts	    MCU_STATE, r17
    rcall   BEEP_LONG
    rjmp    PC+2
START_PROGRAM:
    rcall   BEEP_SHORT
    display_load 0
    
;    sei
//</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Главный цикл">
; **** ГЛАВНЫЙ ЦИКЛ **********************************************
LOOP:
  ; rcall       DISPLAY_UPD_DIGITS
    lds         r16, MCU_STATE		    ; получаем текущее состояние МК
    
_STATE_DEFAULT:
    cpi		r16, MCU_STATE_DEFAULT
    brne	_STATE_PROGRAM
    cbi		LED_ERR_PORT, LED_ERR_PIN
    rcall	DISPLAY_UPD_DIGITS
    rcall	TEMP_UPD		    ; обновление данных о температуре
    outi	r16, TIMSK, (1<<OCIE0A)	    ; вкл. индикатор
    rcall	TEMP_COMPARSION		    ; логика термостата
    rcall	TEMP_SEND_UART		    ; отпрака данных в UART
_STATE_PROGRAM:
    cpi		r16, MCU_STATE_PROGRAM
    brne	_STATE_ERROR
    cbi		LED_ERR_PORT, LED_ERR_PIN
    rcall	DISPLAY_UPD_DIGITS
    rcall	SW_CHECK_PROCESS
_STATE_ERROR:
    cpi		r16, MCU_STATE_ERROR
    brne	LOOP
    sbi		LED_ERR_PORT, LED_ERR_PIN
    rjmp      LOOP
//</editor-fold>


; **** ПОДПРОГРАММЫ **********************************************
.INCLUDE "div16u.asm"
.INCLUDE "div8u.asm"
.INCLUDE "mpy16u.asm"
    
//<editor-fold defaultstate="collapsed" desc="Подпрограмма: отправка данных в UART">
TEMP_SEND_UART:
    push    r16
    ; целая часть
    lds	    r16, TEMP_L
    mov	    r5, r16
    rcall   UART_WR_BYTE
    
    ; дробная часть
    lds	    r16, TEMP_F
    mov	    r5, r16
    rcall   UART_WR_BYTE
    
    ; состояние реле
    in	    r16, PIND
    andi    r16, (1<<RELAY_PIN)
    mov	    r5, r16
    rcall   UART_WR_BYTE
    
    ldi	    r16, 0x04 ; EOT (End Of Transmission)
    mov	    r5, r16
    rcall   UART_WR_BYTE
    
    pop	    r16
    ret
//</editor-fold>
 
//<editor-fold defaultstate="collapsed" desc="Подпрограмма: отправка байта в UART из регистра r5">
UART_WR_BYTE:
    sbis    UCSRA, UDRE
    rjmp    UART_WR_BYTE
    out	    UDR, r5
    ret//</editor-fold>
   
//<editor-fold defaultstate="collapsed" desc="Подпрограмма: сравнение температуры с уставкой">
TEMP_COMPARSION:
    push    r16
    push    r17
    push    r18
    push    r19
    push    r20
    push    r21
;    
;    lds	    dd8u, SETTING_HYST
;    ldi	    dv8u, 10
;    rcall   div8u
;    
;    ; определение нижнего и верхнего порого срабатывания
;    lds	    r19, SETTING_INT
;    mov	    r20, r19
;    sub	    r19, dres8u			    ; MIN = SET - HYST_INT
;    add	    r20, dres8u			    ; MAX = SET + HYST_INT
; 
    .DEF    temp_r_l = r16
    .DEF    temp_r_h = r17
    
    .DEF    min_r_trhd_l = r11
    .DEF    min_r_trhd_h = r10
    
    .DEF    max_r_trhd_l = r13
    .DEF    max_r_trhd_h = r12
    ; умножаем температуру на 10
    lds	    mc16uL, TEMP_L		    // r16
    lds	    mc16uH, TEMP_H		    // r17
    ldi	    mp16uL, LOW(10)
    ldi	    mp16uH, HIGH(10)
    rcall   mpy16u
    mov	    temp_r_l, m16u0			    ; L
    mov	    temp_r_h, m16u1			    ; H
    ; добавляем дробную часть
    lds	    r18, TEMP_F
    clr	    r19
    add	    temp_r_l, r18
    adc	    temp_r_h, r19
    sts	    SRAM_TEMP_2, temp_r_h
    sts	    SRAM_TEMP_3, temp_r_l
    
    ; определяем нижний и верхний пороги
    lds	    r20, SETTING_TEMP_L
    lds	    r21, SETTING_TEMP_H
    lds	    r22, SETTING_HYST
    push    r20
    push    r21
    clr	    r23
    sub	    r20, r22
    sbc	    r21, r23
    mov	    min_r_trhd_l, r20
    mov	    min_r_trhd_h, r21
    pop	    r21
    pop	    r20
    clr	    r23
    add	    r20, r22
    adc	    r21, r23
    mov	    max_r_trhd_l, r20
    mov	    max_r_trhd_h, r21
    
    ; определяем знак температуры
    brts    _NEGATE_TEMP
    rjmp    _COMPARE

_NEGATE_TEMP:
    com	    temp_r_l
    com	    temp_r_h
    subi    temp_r_l, low(-1)
    sbci    temp_r_h, high(-1)
    
_COMPARE:
    ; проверяем нижний порог
    cp	    min_r_trhd_l, temp_r_l
    cpc	    min_r_trhd_h, temp_r_h
    brge    _MIN_THRESHOLD		    ; TEMP <= MIN
    ; проверяем верхний порог
    cp	    temp_r_l, max_r_trhd_l
    cpc	    temp_r_h, max_r_trhd_h
    brge    _MAX_THRESHOLD		    ; TEMP >= TOP
    rjmp    _COMPARSION_EXIT
_MIN_THRESHOLD:
    ; определяем режим работы
    lds	    r20, SETTING_MODE
    tst	    r20
    brne    PC+3
    relay_off
    rjmp    PC+2
    relay_on
    rjmp    _COMPARSION_EXIT
_MAX_THRESHOLD:
    lds	    r20, SETTING_MODE
    tst	    r20
    brne    PC+3
    relay_on
    rjmp    PC+2
    relay_off
_COMPARSION_EXIT:
    pop	    r21
    pop	    r20
    pop	    r19
    pop	    r18
    pop	    r17
    pop	    r16
    ret//</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Подпрограмма: обновляем данные о температуре">
TEMP_UPD:
    push    r17
    rcall   TEMP_CONV
    DELAY24 800000
    rcall   TEMP_RD
    
    ; обновляем данные в ячейках
    lds	    r17, TEMP_L
    mov	    DISP_NUM_L, r17
    lds	    r17, TEMP_H
    mov	    DISP_NUM_H, r17
    pop	    r17
    ret
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подрограмма: чтение кода семейства датчика">
RD_F_CODE:
    push	r16
    
    rcall	OW_PRESENCE
    ldi		r16, DS18B20_CMD_READROM
    mov		OW_CMD_r, r16
    rcall	OW_SEND_BYTE
        
    ldi		XL, LOW(DEVICE_FAMILY_CODE)
    ldi		XH, HIGH(DEVICE_FAMILY_CODE)
    rcall	OW_RD_BYTE
    
    pop		r16
    ret//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подпрограмма: опрос температуры и чтение">
TEMP_RD:
    push	r16
    push	r17
    push	r18
    push	r19
    push	r20
;    cli
    
    rcall	OW_PRESENCE
    sbrs	OWFR, OWPRF
    rjmp	_TEMP_RD_EXIT
    
    ldi		r16, DS18B20_CMD_SKIPROM
    mov		OW_CMD_r, r16
    rcall	OW_SEND_BYTE
    
    ldi		r16, DS18B20_CMD_RSCRATCHPAD
    mov		OW_CMD_r, r16
    rcall	OW_SEND_BYTE
    
    ldi		XL, LOW(TEMP_L)
    ldi		XH, HIGH(TEMP_L)
    rcall	OW_RD_BYTE
    
    ldi		XL, LOW(TEMP_H)
    ldi		XH, HIGH(TEMP_H)
    rcall	OW_RD_BYTE
    
    lds		r17, TEMP_L
    lds		r18, TEMP_H
    
    push	r18
    cbr		r18, (1<<0) | (1<<1) | (1<<2)	; убираем ненужные биты на время (интересуют последних битов в TEMP_H)
    cpi		r18, 0xf8			; проверяется является ли число минусовой
    breq	_TEMP_RD_TO_UNSIGNED		; если да то конвертируем в беззнаковое число
    clt
    rjmp	_TEMP_RD_CONTINUE		; если нет то преобразуем значение АЦП в температуру (делим на 16)

_TEMP_RD_TO_UNSIGNED:
    pop	    r18
    set
    com	    r17
    com	    r18
    subi    r17, low(-1)
    sbci    r18, high(-1)
;    ldi	    r19, 1
;    add	    r17, r19
;    ldi	    r19, 0
;    adc	    r18, r19
    rjmp    PC+2
    
_TEMP_RD_CONTINUE:			    ; Температура = Число с АЦП / 16 (сдвиг вправо 4)
    pop		r18
    lsr r18
    ror r17
    lsr r18
    ror r17
    lsr r18
    ror r17
    lsr r18
    ror r17
_TEMP_RD_END: 
    lds	    r19, TEMP_L
    brtc    PC+2
    neg	    r19				    ; переводим в беззнаковое число если минус
    
    ; далее происходит такое для получения дробной части: 
    ; ((n<<3) + (n<<1))>>4 или (n*10)/16
    mov	    r20, r19
    andi    r19, 0x0f
    mov	    r20, r19
    lsl	    r19
    lsl	    r19
    lsl	    r19
    
    lsl	    r20
    
    add	    r19, r20
    
    lsr	    r19
    lsr	    r19
    lsr	    r19
    lsr	    r19
    
    ; записываем данные
    sts		TEMP_L, r17
    sts		TEMP_H, r18
    sts		TEMP_F, r19
    
;    sei
_TEMP_RD_EXIT:
    pop		r20
    pop		r19
    pop		r18
    pop		r17
    pop		r16
    ret
;    
TEMP_CONV:
    push	r16
;    cli
    
    rcall	OW_PRESENCE
    sbrs	OWFR, OWPRF
    rjmp	_TEMP_CONV_EXIT
    
    ldi		r16, DS18B20_CMD_SKIPROM
    mov		OW_CMD_r, r16
    rcall	OW_SEND_BYTE
    
    ldi		r16, DS18B20_CMD_CONVERTTEMP
    mov		OW_CMD_r, r16
    rcall	OW_SEND_BYTE
    
;    sei
_TEMP_CONV_EXIT:
    pop		r16
    ret
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подпрограмма: отправка байта в сдвиговый регистр">
; **** ОТПРАВКА БАЙТА В СДВИГОВЫЙ РЕГИСТР *************************
USI_TRANSMIT:
    push      r16
    out       USIDR, r0            ; Байт для отправки всегда находится в регистре r0. Помещаем данные в регистр USIDR.

  ; Enable USI Overflow Interrupt Flag (will be 0 if transfer is not compeleted)
    ldi       TEMP_REG_A, (1<<USIOIF)      
    out       USISR, TEMP_REG_A
  
  ; Load settings of USI into temp register
  ; This will setup USI to Three-wire mode, Software clock strobe (USITC) 
  ; with External, positive edge and toggle USCK
  ;
  ; USIWM0 <--------------> USI Wire Mode
  ; USICS1 <--------------> USI Clock Source Select
  ; USICLK <--------------> USI Clock Strobe
  ; USITC  <--------------> USI Toggle Clock (Enable clock generation)      
    ldi       TEMP_REG_A, (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC)
  
_USI_TRANSMIT_LOOP:             ; Execute loop when USIOIF is 0
    out       USICR, TEMP_REG_A   ; Load settings from temp register into USI Control Register
    sbis      USISR, USIOIF       ; If transfer is comleted then move out of loop
    rjmp      _USI_TRANSMIT_LOOP

  ; Send pulse into LATCH pin. 
  ; This will copy byte from 74hc595 shift register into 74hc595 storage register
    sbi      PORTB, USI_LATCH_PIN
    cbi      PORTB, USI_LATCH_PIN
    pop      r16
    ret//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подпрограмма: опрос кнопок">
SW_CHECK_PROCESS:
    push    r16
    rcall   DEBOUNCE_SW
    sbis    SW_PIN, SW_PLUS_PIN
    adiw    DISP_NUM_L, 1

    sbis    SW_PIN, SW_MINUS_PIN
    sbiw    DISP_NUM_L, 1
    
    sbis    SW_PIN, SW_SET_PIN
    rjmp    _INTO_DEFAULT_STATE
    rjmp    _SW_CHECK_PROCESS_END
    
_INTO_DEFAULT_STATE:
    ldi	    r16, MCU_STATE_DEFAULT
    sts	    MCU_STATE, r16
_SW_CHECK_PROCESS_END:
    pop	    r16
    ret
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Реализация интерфеса 1-Wire">
//<editor-fold defaultstate="collapsed" desc="1-Wire: оспрос присутствия">
; **** ОПРОС ПРИСУТСТВИЯ УСТРОЙСТВА ******************************
OW_PRESENCE:
    cli
    ow_pull
    DELAY16	480
    ow_release
    DELAY16	70
    rcall	OW_CHECK_PRESENCE
    DELAY16	410
    sei
    ret
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="1-Wire: проверка наличия устройства">
; **** ПРОВЕРКА НАЛИЧИЯ УСТРОЙСТВА *******************************
; Если подчиненное устройство притянет шину, то устанавливаем флаг OWPRF в единицу в регистре флагов
;
OW_CHECK_PRESENCE:
    sbis    OW_PIN, OW_LINE
    sbr	    OWFR, (1<<OWPRF)
    rjmp    _EXIT
    sbic    OW_PIN, OW_LINE
    cbr	    OWFR, (1<<OWPRF)
_EXIT:
    ret
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="1-Wire: отправка байта">
OW_SEND_BYTE:
    push    r16
    push    r17    
    cli
    mov	    r16, OW_CMD_r
    ldi	    r17, 8
_OW_SEND_BYTE_LOOP:
    lsr	    r16
    brcc    _OW_SEND_0
    brcs    _OW_SEND_1
_OW_SEND_0:
    ow_pull
    DELAY16 60
    ow_release
    DELAY16 10
    rjmp    _OW_SEND_BYTE_END
_OW_SEND_1:
    ow_pull
    DELAY16 6
    ow_release
    DELAY16 64
_OW_SEND_BYTE_END:
    dec	    r17
    brne    _OW_SEND_BYTE_LOOP
    sei
    pop	    r17
    pop	    r16
    ret
//</editor-fold>
        
//<editor-fold defaultstate="collapsed" desc="1-Wire: чтение байта">
OW_RD_BYTE:
    push    r16
    push    r17
    cli
    clr	    r16
    ldi	    r17, 8
_OW_RD_BYTE_LP:
    lsr	    r16
    ow_pull
    DELAY16	6
    ow_release
    DELAY16	9
    sbic    OW_PIN, OW_LINE
    sbr	    r16, (1<<7)
    DELAY16	55
    dec	    r17
    brne    _OW_RD_BYTE_LP
    st	    X, r16
    sei
    pop	    r17
    pop	    r16
    ret
//</editor-fold>
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подпрограмма: обновление ячеек в SRAM для индикатора">
; **** ПОЛУЧЕНИЕ ЦИФР ИЗ 16-ТИ БИТНОГО ЧИСЛА *********************
; Описание: Перемещает цифры числа в соответствующие ячейки памяти в SRAM
;           путем деления этого числа несколько раз
DISPLAY_UPD_DIGITS:
    cli
    push  r21
    ldi   r21,    4                     ; (4 раза делим) т.к индикатор четырех разрядный
  
  ; инициализация указателя (за каждый проход цикла будет инкрементироваться)
    ldi   XL, LOW(DIGITS)
    ldi   XH, HIGH(DIGITS)

    .equ  dividend      = SRAM_TEMP_1   ; число которе будем делить
    .equ  divisor       = 10            ; на что делим

  ; загружаем число которое хотим поделить в адрес SRAM делимого
    sts   dividend,     DISP_NUM_L
    sts   dividend+1,   DISP_NUM_H

  ; четыре раза производим деление для получения остатков
DIV_LOOP:
    ; заполняем нужные регистры
    lds   dd16uL, dividend
    lds   dd16uH, dividend+1
    ldi   dv16uL, LOW(divisor)
    ldi   dv16uH, HIGH(divisor)
    
    rcall div16u                      ; делим

    st   X+,    drem16uL              ; сохраняем остаток в ячейку по указателю и увеличиваем его

    ; обновляем делимое
    sts  dividend,   dres16uL
    sts  dividend+1, dres16uH

    dec   r21                         ; декрементируем счетчик цикла
    brne  DIV_LOOP                    ; делим еще раз если не 0
    pop   r21
    sei
    ret
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подпрограмма: получение символа для индикатора">
; **** ЗАГРУЖАЕТ НУЖНЫЙ АДРЕС СИМВОЛА В R0 ***********************
DISPLAY_DECODER:
    push     r16
    push     r17
    ldi	     ZL, LOW(2*DISPLAY_SYMBOLS)
    ldi	     ZH, HIGH(2*DISPLAY_SYMBOLS)

    clr      TEMP_REG_B
    add      ZL, TEMP_REG_A
    adc      ZH, TEMP_REG_B

    lpm      r0, Z
    pop      r17
    pop      r16
    ret//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подпрограмма: короткий писк">
BEEP_SHORT:
    sbi	    BUZZER_PORT, BUZZER_PIN
    rcall   DELAY
    cbi	    BUZZER_PORT, BUZZER_PIN
    ret
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подпрограмма: длинный писк">
BEEP_LONG:
    sbi	    BUZZER_PORT, BUZZER_PIN
    rcall   DELAY
    rcall   DELAY
    rcall   DELAY
    rcall   DELAY
    cbi	    BUZZER_PORT, BUZZER_PIN
    ret//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подпрограмма: устранение дребезга кнопки">
DEBOUNCE_SW:
    push    r16
    push    r17
    push    r18

    ldi	    r18, 10
_loop_2:
    ldi     r17, 255
_loop_0:
    ldi     r16, 255
_dec_0:
    dec     r16
    brne    _dec_0
_loop_1:
    dec     r17
    brne    _loop_0
_loop_3:
    dec	    r18
    brne    _loop_2

    pop	    r18
    pop	    r17
    pop	    r16
    ret//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подпрограмма: задержка (16бит макс число)">
DELAY_LOOP_16:
    subi DELAY_8_r, 1
    sbci DELAY_16_r, 0
    brcc DELAY_LOOP_16
    ret
//</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Подпрограмма: задержка (24бит макс число)">
DELAY_LOOP_24:
    subi DELAY_8_r, 1
    sbci DELAY_16_r, 0
    sbci DELAY_24_r, 0
    brcc DELAY_LOOP_24
    ret
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Подпрограмма: задержка">
DELAY:
    push    r16
    push    r17
    push    r18

    ldi	    r18, 6
_DELAY_0:
    ldi     r16, 255
_DELAY_1:
    ldi     r17, 255   
_DELAY_2:
    dec     r17         
    brne    _DELAY_2    

    dec     r16
    brne    _DELAY_1    
    dec	    r18
    brne    _DELAY_0

    pop	    r18
    pop     r17
    pop     r16
    ret  //</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Символы для индикатора">
DISPLAY_SYMBOLS:
      ; HGFEDCBA    HGFEDCBA
    .DB 0b11000000, 0b11111001          ; 0, 1
    .DB 0b10100100, 0b10110000          ; 2, 3
    .DB 0b10011001, 0b10010010          ; 4, 5
    .DB 0b10000010, 0b11111000          ; 6, 7
    .DB 0b10000000, 0b10010000          ; 8, 9
    .DB 0b10011100, 0b10111111          ; °, -//</editor-fold>

//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Сегмент EEPROM">
; **** СЕГМЕНТ EEPROM ********************************************
; .ESEG
; INFO:       .DB "AVR Thermostat. Written by Sergey Yarkov 22.01.2023"
;</editor-fold>
