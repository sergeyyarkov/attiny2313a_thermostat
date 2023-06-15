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

.DEF TEMP_REG_A                 = r16
.DEF TEMP_REG_B                 = r17
.DEF DISP_NUM_L                 = r24         ; LSB числа которое сейчас на индикаторе
.DEF DISP_NUM_H                 = r25         ; MSB числа которое сейчас на индикаторе

.EQU DIGIT_1_PIN                = PD2         ; Пин разряда индикатора 1
.EQU DIGIT_2_PIN                = PD3         ; Пин разряда индикатора 2
.EQU DIGIT_3_PIN                = PD4         ; Пин разряда индикатора 3
.EQU DIGIT_4_PIN                = PD5         ; Пин разряда индикатора 4

.EQU LED_POWER_PIN              = PD6         ; Светодиод, который говорит о том что питание подано

.EQU ONE_WIRE_LINE              = PB1         ; Пин шины 1-Wire
.EQU ONE_WIRE_DDR               = DDRB
.EQU ONE_WIRE_PIN               = PINB
.DEF ONE_WIRE_FLAGS             = r23         ; Флаги состояния для 1-Wire интерфейса
.EQU OWPRF                      = 0           ; 1-Wire Флаг присутствия
.EQU OWSB                       = 1           ; Флаг передачи одного бита

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

.EQU USI_LATCH_PIN              = PB0         ; ST_CP на 74HC595
.EQU USI_DO_PIN                 = PB6         ; DS на 74HC595
.EQU USI_CLK_PIN                = PB7         ; SH_CP на 74HC595

.EQU SW_PORT                    = PORTB
.EQU SW_PIN                     = PINB
.EQU SW_PLUS_PIN                = PB2         ; Кнопка "Минус"
.EQU SW_MINUS_PIN               = PB3         ; Кнопка "Плюс"
.EQU SW_SET_PIN                 = PB4         ; Кнопка "Установить"

.EQU MCU_STATE_DEFAULT          = 0x01        ; Режим измерения температуры и сравнения с заданными параметрами
.EQU MCU_STATE_PROGRAM          = 0x02        ; Режим настройки параметров в EEPROM
.EQU MCU_STATE_ERROR            = 0x03        ; Режим ошибки, например когда не подключен датчик

; **** МАКРОСЫ **************************************************
.MACRO outi
  ldi       @0, @2
  out       @1, @0
.ENDMACRO

.MACRO display_load
  ldi   DISP_NUM_L,    LOW(@0)
  ldi   DISP_NUM_H,    HIGH(@0)
.ENDMACRO

.MACRO _1_wire_pull
  sbi       ONE_WIRE_DDR, ONE_WIRE_LINE
.ENDMACRO

.MACRO _1_wire_release
  cbi       ONE_WIRE_DDR, ONE_WIRE_LINE
.ENDMACRO

; **** СЕГМЕНТ ДАННЫХ ********************************************
.DSEG
.ORG SRAM_START

MCU_STATE:      .BYTE 1                     ; Текущее состояние МК
SRAM_TEMP_1:    .BYTE 2                     ; Хранения временного 16-бит числа в ячейке
DIGITS:         .BYTE 4                     ; Ячейки, где хранятся символы, для вывода на индикатор
CURRENT_DIGIT:  .BYTE 1                     ; Номер разряда индикатора, который сейчас горит
PC_HISTORY:     .BYTE 1                     ; История изменений состояния кнопок (необходимо для прерываня по изменению состояния пина)

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

; **** ОБРАБОТКА КНОПОК ******************************************
PCINT0_vect:
  push 		r16
  push 		r17
	push		r18
	push		r19

  in 			r16, 	SREG
	
	in 			r17,	SW_PIN
	lds			r18,	PC_HISTORY
	
	eor			r17,	r18
	in			r19,  SW_PIN
	sts			PC_HISTORY, r19

	_SW_PLUS_CHECK:
	sbrs		r17, SW_PLUS_PIN
	rjmp		_SW_EXIT
	rjmp		_SW_PLUS_HANDLER

	_SW_PLUS_HANDLER:
		; rcall				DEBOUNCE_SW
		sbis				SW_PIN,		SW_PLUS_PIN
		adiw        DISP_NUM_L, 1
		;exit


  ; sbis PINB, SW_PLUS_PIN
  ; rjmp _sw_plus

  ; sbis PINB, SW_MINUS_PIN
  ; rjmp _sw_minus

  ; sbis PINB, SW_SET_PIN
  ; rjmp _sw_set

  ; rjmp _sw_exit
  ; _sw_plus:
  ;   adiw        DISP_NUM_L, 1
  ;   rjmp        _sw_exit
  ; _sw_minus:
  ;   adiw        DISP_NUM_L, 10
  ;   rjmp        _sw_exit
  ; _sw_set:
  ;   adiw        DISP_NUM_L, 63
  ; _sw_exit:

	_SW_EXIT:
  out 		SREG, r16
	pop 		r19
	pop			r18
	pop			r17
  pop 		r16
reti

; **** СТАРТ ПРОГРАММЫ *******************************************
RESET_vect:
  ldi       TEMP_REG_A, LOW(RAMEND)
  out       SPL, TEMP_REG_A

; **** ПРОЦЕСС ИНИЦИАЛИЗАЦИИ МК **********************************
MCU_INIT:
  ; **** ИНИЦИАЛИЗАЦИЯ ПИНОВ *************************************
  outi      r16, DDRD, (1<<LED_POWER_PIN) | (1<<DIGIT_1_PIN) | (1<<DIGIT_2_PIN) | (1<<DIGIT_3_PIN) | (1<<DIGIT_4_PIN) | (0<<PD0) | (1<<PD1)
  outi      r16, DDRB, (1<<USI_CLK_PIN) | (1<<USI_DO_PIN) | (1<<USI_LATCH_PIN) | (0<<SW_PLUS_PIN) | (0<<SW_MINUS_PIN) | (0<<SW_SET_PIN)
  outi      r16, PORTB, (1<<SW_PLUS_PIN) | (1<<SW_MINUS_PIN) | (1<<SW_SET_PIN)

  ; **** ИНИЦИАЛИЗАЦИЯ ПРЕРЫВАНИЙ ********************************
  outi      r16, GIMSK, (1<<PCIE0)
  outi      r16, PCMSK0, (1<<PCINT2) | (1<<PCINT3) | (1<<PCINT4)          ; прерывание для кнопок на нажатие
  
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

  display_load 0                       ; загружаем число, которое нужно показать на индикатор
  ; rcall _1_WIRE_DETECT_PRESENCE        ; проверяем наличие датчика на шине путем выполнения процедуры сброса
  sei

; **** ГЛАВНЫЙ ЦИКЛ **********************************************
LOOP:
  rcall       DISPLAY_UPD_DIGITS       ; получаем из этого числа цифры путем деления и распределяем их по ячейкам в SRAM
  
  ; usart_t:
  ; sbis        UCSRA, UDRE
  ; rjmp        usart_t
  ; ldi         r16, 'B'
  ; out         UDR, r16

  ; usart_r:
  ;   sbis UCSRA, RXC
  ;   rjmp usart_r
  ; in DISP_NUM_L, UDR

  ; sbrc ONE_WIRE_FLAGS, OWPRF
  ; sbi PORTD, PD6

  ; sbrs ONE_WIRE_FLAGS, OWPRF
  ; cbi PORTD, PD6

	sbi PORTD, PD6

  rcall       DEBOUNCE_SW

	cbi PORTD, PD6

	rcall       DEBOUNCE_SW

  rjmp      LOOP

; **** ПОДПРОГРАММЫ **********************************************
.INCLUDE "div16u.asm"
.INCLUDE "usi.asm"

; **** ОПРОС ПРИСУТСТВИЯ УСТРОЙСТВА ******************************
_1_WIRE_DETECT_PRESENCE:
  cli                                   ; Шаг 1. выключаем глобальные прерывания

  ; 62 тика
  outi      r16, TCNT1L, 0
  outi      r16, OCR1AL, 60           ; 480 мкс
  outi      r16, OCR1BL, 69           ; 72 мкс

  ; предделитель 64
  outi      r16, TCCR1B, (1<<CS11) | (1<<CS10)

  _1_wire_pull                          ; Шаг 2. притягиваем шину

  S1:
      in r16, TIFR
      sbrs r16, OCF1A                   ; Шаг 3. ждем минимум 480мкс
      rjmp  S1                          
      _1_wire_release                   ; Шаг 4. отпускаем шину
  S2: in r16, TIFR
      sbrs r16, OCF1B                   ; Шаг 5. ждем +- 70мкс
      rjmp S2
      rcall _1_WIRE_CHECK_PRESENCE      ; Шаг 6. проверяем есть ли устройство на шине, и если есть то устанавливаем флаг
  
  outi      r16, TCNT1L, 0
  outi      r16, OCR1AL, 52         ; 416 мкс
  S3: 
      in r16, TIFR
      sbrs r16, OCF1A
      rjmp S3  
      clr r16                           
      out TCCR1B, r16
      ser r16
      out TIFR, r16
  ; s3: in r16, TIFR
      ; sbrs r16, TOV1
      ; rjmp s3
      ; rcall TOV1_occured
      ; ser r16
      ; out TIFR, r16
  sei
ret

; **** ПРОВЕРКА НАЛИЧИЯ УСТРОЙСТВА *******************************
; Если подчиненное устройство притянет шину, то устанавливаем флаг OWPRF в единицу в регистре флагов
;
_1_WIRE_CHECK_PRESENCE:
  sbis ONE_WIRE_PIN, ONE_WIRE_LINE
  sbr ONE_WIRE_FLAGS, (1<<OWPRF)
ret

_1_WIRE_SEND_BYTE:
  push r17
  ldi r17, DS18B20_CMD_READROM
  ldi r16, 8
  _s_l:
    ; rol r16
    ; brcc
  pop r17
ret

.DEF TRANSMIT_BIT = r9
_1_WIRE_SEND_BIT:
  push r16
  cli
  sbr ONE_WIRE_FLAGS, (1<<OWSB)
  outi      r16, TCNT1L, 0
  outi      r16, OCR1AL, 1            ; 8 мкс
  outi      r16, OCR1BL, 9           ; 64 мкс
  
  _1_wire_pull
  SB_1:
    in r16, TIFR
    sbrs r16, OCF1A
    rjmp SB_1
    sbrc TRANSMIT_BIT, 0
    _1_wire_release
  SB_2:
    in r16, TIFR
    sbrs r16, OCF1B
    rjmp SB_2

  clr r16                           
  out TCCR1B, r16
  cbr ONE_WIRE_FLAGS, (1<<OWSB)
  sei
  pop r16
ret

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

; **** ЗАГРУЖАЕТ НУЖНЫЙ АДРЕС СИМВОЛА В R0 ***********************
DISPLAY_DECODER:
  push      r16
  push      r17
  ldi	      ZL, LOW(2*DISPLAY_SYMBOLS)
	ldi	      ZH, HIGH(2*DISPLAY_SYMBOLS)

  clr       TEMP_REG_B
  add       ZL, TEMP_REG_A
  adc       ZH, TEMP_REG_B

  lpm       r0, Z
  pop       r17
  pop       r16
ret

DEBOUNCE_SW:
  push    r16
  push    r17
	push		r18

	ldi			r18, 5
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
		dec			r18
		brne		_loop_2

	pop r18
  pop r17
  pop r16
ret

DELAY:
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