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
HYSTERESIS:     .BYTE 1                     ; Отклонение температуры от уставки
TEMP_L:		.BYTE 1
TEMP_H:		.BYTE 1
;//</editor-fold>


//<editor-fold defaultstate="collapsed" desc="Сегмент кода">
; **** СЕГМЕНТ КОДА **********************************************
.CSEG

//<editor-fold defaultstate="collapsed" desc="Вектора">
.ORG 0x00     
    rjmp 	RESET_vect

.ORG 0x000D   
    rjmp	TIMER0_COMPA_vect
//</editor-fold>


//<editor-fold defaultstate="collapsed" desc="Прерывание: Динамическая индикация">
; **** ДИНАМИЧЕСКАЯ ИНДИКАЦИЯ ************************************
TIMER0_COMPA_vect:
    push      r20
    push      r21
    push      r16
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
    sbi       PORTD, DIGIT_1_PIN
    lds       TEMP_REG_A, DIGITS+3
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

_indicate_2:
    cpi       r20, 1
    brne      _indicate_3

    cbi       PORTD, DIGIT_1_PIN
    cbi       PORTD, DIGIT_3_PIN
    cbi       PORTD, DIGIT_4_PIN
    sbi       PORTD, DIGIT_2_PIN
    lds       TEMP_REG_A, DIGITS+2
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

_indicate_3:
    cpi       r20, 2
    brne      _indicate_4

    cbi       PORTD, DIGIT_1_PIN
    cbi       PORTD, DIGIT_2_PIN
    cbi       PORTD, DIGIT_4_PIN
    sbi       PORTD, DIGIT_3_PIN
    lds       TEMP_REG_A, DIGITS+1
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

_indicate_4:
    cpi       r20, 3
    brne      _indicate_exit

    cbi       PORTD, DIGIT_1_PIN
    cbi       PORTD, DIGIT_2_PIN
    cbi       PORTD, DIGIT_3_PIN
    sbi       PORTD, DIGIT_4_PIN
    lds       TEMP_REG_A, DIGITS
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

_indicate_exit:
    inc       r20
    sts       CURRENT_DIGIT, r20
    out SREG, r21
    
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
  outi      r16, DDRD, (1<<LED_ERR_PIN) | (1<<DIGIT_1_PIN) | (1<<DIGIT_2_PIN) | (1<<DIGIT_3_PIN) | (1<<DIGIT_4_PIN) | (0<<UART_RX_PIN) | (1<<UART_TX_PIN)
  outi      r16, DDRB, (1<<USI_CLK_PIN) | (1<<USI_DO_PIN) | (1<<USI_LATCH_PIN) | (0<<SW_PLUS_PIN) | (0<<SW_MINUS_PIN) | (0<<SW_SET_PIN)
  outi      r16, PORTB, (1<<SW_PLUS_PIN) | (1<<SW_MINUS_PIN) | (1<<SW_SET_PIN)
  
  ; **** ИНИЦИАЛИЗАЦИЯ ТАЙМЕРА 0 **********************************
  outi      r16, TCCR0A, (1<<WGM01)             ; режим CTC Compare A
  outi      r16, TCCR0B, (1<<CS02) | (1<<CS00)  ; 1024 делитель
  outi      r16, OCR0A, 25                      ; число для сравнения. (60Hz)
  outi      r16, TIMSK, (1<<OCIE0A)             ; включение прерывания по совпадению

  ; **** ИНИЦИАЛИЗАЦИЯ USART *************************************
  ; outi      r16, UBRRL, LOW(3)                 ; 9600 БОД
  ; outi      r16, UBRRH, HIGH(3)                ; 9600 БОД
  ; outi      r16, UCSRB, (1<<RXEN) | (1<<TXEN)   ; Включение приема и передачии
  ; outi      r16, UCSRC, (1<<UCSZ1) | (1<<UCSZ0) ; Асинхронный режим, 8 бит фрейм, 1 стоповый бит

    clr       r1
    sts       CURRENT_DIGIT,  r1

    ldi       r16, 0x00
    sts       MCU_STATE,      r16	    ; переводим МК сразу в режим измерения температуры
    
    clr	    r16
    sts	    TEMP_L, r16
    sts	    TEMP_H, r16

    display_load 0			    ; загружаем число, которое нужно показать на индикатор
            
    lds		r16, TEMP_L
    mov		DISP_NUM_L, r16
    ldi		DISP_NUM_H, 0
    
    
;    sei
//</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Главный цикл">
; **** ГЛАВНЫЙ ЦИКЛ **********************************************
LOOP:
  ; rcall       DISPLAY_UPD_DIGITS
    lds         r16, MCU_STATE           ; получаем текущее состояние МК

_STATE_DEFAULT:
    cpi		r16, MCU_STATE_DEFAULT
    brne	_STATE_PROGRAM
    rcall	DISPLAY_UPD_DIGITS
    sbrc	OWFR, OWPRF
    sbi		PORTD, PD6
    sbrs	OWFR, OWPRF
    cbi		PORTD, PD6
    
    rcall	TEMP_CONV
    rcall	DEBOUNCE_SW
    rcall	DEBOUNCE_SW
    rcall	DEBOUNCE_SW
    rcall	DEBOUNCE_SW
    rcall	TEMP_RD
    lds		r17, TEMP_L
    mov		DISP_NUM_L, r17
    lds		r17, TEMP_H
    mov		DISP_NUM_H, r17

_STATE_PROGRAM:
    cpi       r16, MCU_STATE_PROGRAM
    brne      _STATE_ERROR
    rcall     DISPLAY_UPD_DIGITS
    rcall     SW_CHECK_PROCESS
_STATE_ERROR:
    cpi       r16, MCU_STATE_ERROR
    brne      LOOP
    ldi r17, 11
    sts DIGITS, r17
    sts DIGITS+1, r17
    sts DIGITS+2, r17
    sts DIGITS+3, r17
    nop

    rjmp      LOOP
//</editor-fold>


; **** ПОДПРОГРАММЫ **********************************************
.INCLUDE "div16u.asm"
    
DELAY_LOOP_16:
    subi DELAY_8_r, 1
    sbci DELAY_16_r, 0
    brcc DELAY_LOOP_16
    ret
    
//<editor-fold defaultstate="collapsed" desc="Подпрограмма: чтение и опрос температуры">
TEMP_RD:
    push	r16
    push	r17
    push	r18
    cli
    
    rcall	OW_PRESENCE
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
    
    lsr r18
    ror r17
    lsr r18
    ror r17
    lsr r18
    ror r17
    lsr r18
    ror r17
    
    sts		TEMP_L, r17
    sts		TEMP_H, r18

    
    sei
    pop		r18
    pop		r17
    pop		r16
    ret
;    
TEMP_CONV:
    push	r16
    cli
    
    rcall	OW_PRESENCE
    ldi		r16, DS18B20_CMD_SKIPROM
    mov		OW_CMD_r, r16
    rcall	OW_SEND_BYTE
    
    ldi		r16, DS18B20_CMD_CONVERTTEMP
    mov		OW_CMD_r, r16
    rcall	OW_SEND_BYTE
    
    sei
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
    rcall       DEBOUNCE_SW
    sbis        SW_PIN, SW_PLUS_PIN
    adiw        DISP_NUM_L, 1

    sbis        SW_PIN, SW_MINUS_PIN
    sbiw        DISP_NUM_L, 1
    ret//</editor-fold>


//<editor-fold defaultstate="collapsed" desc="Реализация интерфеса 1-Wire">
//<editor-fold defaultstate="collapsed" desc="1-Wire: оспрос присутствия">
; **** ОПРОС ПРИСУТСТВИЯ УСТРОЙСТВА ******************************
OW_PRESENCE:
    ow_pull
    DELAY16	480
    ow_release
    DELAY16	70
    rcall	OW_CHECK_PRESENCE
    DELAY16	410
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
    pop	    r17
    pop	    r16
    ret
//</editor-fold>
        
//<editor-fold defaultstate="collapsed" desc="1-Wire: чтение байта">
OW_RD_BYTE:
    push    r16
    push    r17
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


//<editor-fold defaultstate="collapsed" desc="Подпрограмма: задержка">
DELAY:
    push    r16
    push    r17

    ldi     r16, 255
_DELAY_1:
    ldi     r17, 255   
_DELAY_2:
    dec     r17         
    nop                 
    nop                
    nop                 
    brne    _DELAY_2    

    dec     r16
    brne    _DELAY_1    

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
