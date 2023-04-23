;
; Project name: thermostat
; Description: Electronic thermostat on AVR Microcontroller
; Source code: https://github.com/sergeyyarkov/attiny2313a_thermostat
; Device: ATtiny2313A
; Device Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf
; Package: SOIC-20W_7.5x12.8mm_P1.27mm
; Assembler: AVR macro assembler 2.2.7
; Clock frequency: 8 MHz External Crystal Oscillator
; Fuses: lfuse: 0xCF, hfuse: 0x9F, efuse: 0xFF, lock: 0xFF
;
; Written by Sergey Yarkov 22.01.2023

.INCLUDE "tn2313Adef.inc"
.LIST

.DEF TEMP_REG_A               = r16
.DEF TEMP_REG_B               = r17
.DEF DISP_NUM_L               = r24         ; LSB числа которое сейчас на индикаторе
.DEF DISP_NUM_H               = r25         ; MSB числа которое сейчас на индикаторе

.EQU DIGIT_1_PIN              = PD2         ; Пин разряда индикатора 1
.EQU DIGIT_2_PIN              = PD3         ; Пин разряда индикатора 2
.EQU DIGIT_3_PIN              = PD4         ; Пин разряда индикатора 3
.EQU DIGIT_4_PIN              = PD5         ; Пин разряда индикатора 4

.EQU LED_POWER_PIN            = PD6         ; Светодиод, который говорит о том что питание подано

.EQU USI_LATCH_PIN            = PB0         ; ST_CP на 74HC595
.EQU USI_DO_PIN               = PB6         ; DS на 74HC595
.EQU USI_CLK_PIN              = PB7         ; SH_CP на 74HC595

.EQU SW_PLUS_PIN              = PB2         ; Кнопка "Минус"
.EQU SW_MINUS_PIN             = PB3         ; Кнопка "Плюс"
.EQU SW_SET_PIN               = PB4         ; Кнопка "Установить"

.EQU MCU_STATE_DEFAULT        = 0x01        ; Режим измерения температуры и сравнения с заданными параметрами
.EQU MCU_STATE_PROGRAM        = 0x02        ; Режим настройки параметров в EEPROM
.EQU MCU_STATE_ERROR          = 0x03        ; Режим ошибки, например когда не подключен датчик

; **** МАКРОСЫ **************************************************
.MACRO outi
  ldi       @0, @2
  out       @1, @0
.ENDMACRO

.MACRO display_load
  ldi   DISP_NUM_L,    LOW(@0)
  ldi   DISP_NUM_H,    HIGH(@0)
.ENDMACRO

; **** СЕГМЕНТ ДАННЫХ ********************************************
.DSEG
.ORG SRAM_START

MCU_STATE:      .BYTE 1                     ; Текущее состояние МК
SRAM_TEMP_1:    .BYTE 2                     ; Хранения временного 16-бит числа в ячейке
DIGITS:         .BYTE 4                     ; Ячейки, где хранятся символы, для вывода на индикатор
CURRENT_DIGIT:  .BYTE 1                     ; Номер разряда индикатора, который сейчас горит

; **** СЕГМЕНТ КОДА **********************************************
.CSEG

.ORG 0x00     
  rjmp 	RESET_vect

.ORG 0x000B
  rjmp  PCINT0_vect

.ORG 0x000D   
  rjmp  TIMER0_COMPA_vect


; **** ДИНАМИЧЕСКАЯ ИНДИКАЦИЯ ************************************
TIMER0_COMPA_vect:
  push      r20
  push      r21
  in r21, SREG
  lds       r20,  CURRENT_DIGIT
  cpi       r20,  5
  brge      reset_digit_idx
  rjmp      _indicate_1

  reset_digit_idx:
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
    pop r21
    pop r20
reti

; **** ОБРАБОТКА КНОПОК ******************************************
PCINT0_vect:
  adiw        DISP_NUM_L, 1
reti

; **** СТАРТ ПРОГРАММЫ *******************************************
RESET_vect:
  ldi       TEMP_REG_A, LOW(RAMEND)
  out       SPL, TEMP_REG_A

; **** ПРОЦЕСС ИНИЦИАЛИЗАЦИИ МК **********************************
MCU_INIT:
  ; **** ИНИЦИАЛИЗАЦИЯ ПИНОВ *************************************
  outi      r16, DDRD, (1<<LED_POWER_PIN) | (1<<DIGIT_1_PIN) | (1<<DIGIT_2_PIN) | (1<<DIGIT_3_PIN) | (1<<DIGIT_4_PIN)
  outi      r16, DDRB, (1<<USI_CLK_PIN) | (1<<USI_DO_PIN) | (1<<USI_LATCH_PIN) | (0<<SW_PLUS_PIN) | (0<<SW_MINUS_PIN) | (0<<SW_SET_PIN)
  outi      r16, PORTB, (1<<SW_PLUS_PIN) | (1<<SW_MINUS_PIN) | (1<<SW_SET_PIN)

  ; **** ИНИЦИАЛИЗАЦИЯ ПРЕРЫВАНИЙ ********************************
  outi      r16, GIMSK, (1<<PCIE0)
  outi      r16, PCMSK0, (1<<PCINT2) | (1<<PCINT3) | (1<<PCINT4)          ; прерывание для кнопок на нажатие
  
  ; **** ИНИЦИАЛИЗАЦИЯ ТАЙМЕРОВ **********************************
  outi      r16, TCCR0A, (1<<WGM01)             ; режим CTC Compare A
  outi      r16, TCCR0B, (1<<CS02) | (1<<CS00)  ; 1024 делитель
  outi      r16, OCR0A, 25                      ; число для сравнения. (60Hz)
  outi      r16, TIMSK, (1<<OCIE0A)             ; включение прерывания по совпадению

  clr       r1
  sts   CURRENT_DIGIT, r1

  display_load 0                       ; загружаем число, которое нужно показать на индикатор
  sei

; **** ГЛАВНЫЙ ЦИКЛ **********************************************
LOOP:
  rcall       DISPLAY_UPD_DIGITS          ; получаем из этого числа цифры путем деления и распределяем их по ячейкам в SRAM
  rcall       TOGGLE_POWER_LED
  rcall       DELAY
  rcall       DELAY
  rcall       DELAY
  rcall       DELAY
  rcall       DELAY
  rcall       DELAY
  
  rjmp      LOOP

; **** ПОДПРОГРАММЫ **********************************************
.INCLUDE "div16u.asm"
.INCLUDE "usi.asm"

; **** ПОЛУЧЕНИЕ ЦИФР ИЗ 16-ТИ БИТНОГО ЧИСЛА *********************
; Описание: Перемещает цифры числа в соответствующие ячейки памяти в SRAM
;           путем деления этого числа несколько раз
DISPLAY_UPD_DIGITS:
  ldi   r21,    4                     ; максимум 4 цифры, т.к индикатор четырех разрядный
  ldi   XL, LOW(DIGITS)
  ldi   XH, HIGH(DIGITS)

  .equ  dividend      = SRAM_TEMP_1
  .equ  divisor       = 10

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

    st   X+,    drem16uL              ; сохраняем остаток в ячейку по указателю

    ; обновляем делимое
    sts  dividend,   dres16uL
    sts  dividend+1, dres16uH

    dec   r21                         ; декрементируем счетчик цикла
    brne  DIV_LOOP                    ; делим еще раз если не 0
ret

; **** ЗАГРУЖАЕТ НУЖНЫЙ АДРЕС СИМВОЛА В R0 ***********************
DISPLAY_DECODER:
  ldi	      ZL, LOW(2*DISPLAY_SYMBOLS)
	ldi	      ZH, HIGH(2*DISPLAY_SYMBOLS)

  clr       TEMP_REG_B
  add       ZL, TEMP_REG_A
  adc       ZH, TEMP_REG_B

  lpm       r0, Z
ret

DELAY:
  ; cli                          
  ; push r24
  ; push r25

  ; .equ c1 = 50000
  ; ldi r24, LOW(c1)
  ; ldi r25, HIGH(c1)
  ; _d_loop:
  ; sbiw r24, 1
  ; brne _d_loop

  ; pop r25
  ; pop r24
  ; sei

  push      r16
  push      r17
  ldi       r16, 255
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

  pop       r17
  pop       r16
ret                    

TOGGLE_POWER_LED:
  push      r16
  push      r17
  ldi       r16,   (1<<LED_POWER_PIN)
  in        r17,   PORTD
  eor       r17,   r16
  out       PORTD, r17
  pop       r17
  pop       r16
ret

DISPLAY_SYMBOLS:
      ; HGFEDCBA    HGFEDCBA
  .DB 0b11000000, 0b11111001          ; 0, 1
  .DB 0b10100100, 0b10110000          ; 2, 3
  .DB 0b10011001, 0b10010010          ; 4, 5
  .DB 0b10000010, 0b11111000          ; 6, 7
  .DB 0b10000000, 0b10010000          ; 8, 9
  .DB 0b10011100                      ; °

; **** СЕГМЕНТ EEPROM ********************************************
; .ESEG
; INFO:       .DB "AVR Thermostat. Written by Sergey Yarkov 22.01.2023"