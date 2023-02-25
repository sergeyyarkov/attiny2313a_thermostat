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

.DEF TEMP_REG_A       = r16
.DEF TEMP_REG_B       = r17

;========================================;
;                LABELS                  ;
;========================================;

.EQU DIGIT_1_PIN              = PD2
.EQU DIGIT_2_PIN              = PD3
.EQU DIGIT_3_PIN              = PD4
.EQU DIGIT_4_PIN              = PD5

.EQU LED_POWER_PIN            = PD6

.EQU USI_LATCH_PIN            = PB0         ; ST_CP on 74HC595
.EQU USI_DO_PIN               = PB6         ; DS on 74HC595
.EQU USI_CLK_PIN              = PB7         ; SH_CP on 74HC595

.EQU MCU_STATE_DEFAULT        = 0x01        ; Temperature measurement and threshold comparison
.EQU MCU_STATE_PROGRAM        = 0x02        ; Write settings of hysteresis into EEPROM
.EQU MCU_STATE_ERROR          = 0x03        ; Cannot read data from temp sensor or something else

;========================================;
;              DATA SEGMENT              ;
;========================================;
.DSEG
.ORG SRAM_START

MCU_STATE:     .BYTE 1
DISPLAY_DIGIT: .BYTE 4

;========================================;
;              CODE SEGMENT              ;
;========================================;

.CSEG
.ORG 0x00

;========================================;
;                VECTORS                 ;
;========================================;

rjmp 	RESET_vect			       ; Program start at RESET vector
;reti                         ; External Interrupt Request 0 / inactive
;reti		                      ; External Interrupt Request 1 / inactive
;reti                         ; Timer/Counter1 Capture Event / inactive
;reti		                      ; Timer/Counter1 Compare Match A / inactive
;reti                         ; Timer/Counter1 Overflow / inactive
;reti                         ; Timer/Counter0 Overflow / inactive
;reti                         ; USART0, Rx Complete / inactive
;reti                         ; USART0 Data Register Empty / inactive
;reti						              ; USART0, Tx Complete / inactive
;reti                         ; Analog Comparator / inactive
;reti	                        ; Pin Change Interrupt Request 0/ inactive
;reti                         ; Timer/Counter1 Compare Match B / inactive
;reti                         ; Timer/Counter0 Compare Match A / inactive
;reti                         ; Timer/Counter0 Compare Match B / inactive
;reti                         ; USI Start Condition/ inactive
;reti                         ; USI Overflow / inactive
;reti                         ; EEPROM Ready/ inactive
;reti                         ; Watchdog Timer Overflow / inactive
;reti                         ; Pin Change Interrupt Request 1 / inactive
;reti                         ; Pin Change Interrupt Request 2 / inactive

RESET_vect:
  ;========================================;
  ;        INITIALIZE STACK POINTER        ;
  ;========================================;
  ldi       TEMP_REG_A, LOW(RAMEND)
  out       SPL, TEMP_REG_A

MCU_INIT:
  ;=======================================================;
  ;               INITIALIZE PORTS                        ;
  ;                                                       ;
  ;  POWER LED          <------------->   OUT PD6         ;
  ;    - This LED is used to indicate that device         ;
  ;       is working correctly                            ;
  ;  7SEG DIGIT 1       <------------->   OUT PD2         ;
  ;  7SEG DIGIT 2       <------------->   OUT PD3         ;
  ;  7SEG DIGIT 3       <------------->   OUT PD4         ;
  ;  7SEG DIGIT 4       <------------->   OUT PD5         ;
  ;                                                       ;
  ;  USI CLOCK PIN      <------------->   OUT PB0         ;
  ;  USI DATA OUT PIN   <------------->   OUT PB6         ;
  ;  USI LATCH PIN      <------------->   OUT PB7         ;
  ;=======================================================;
  ldi       r16, (1<<LED_POWER_PIN) | (1<<DIGIT_1_PIN) | (1<<DIGIT_2_PIN) | (1<<DIGIT_3_PIN) | (1<<DIGIT_4_PIN)
  out       DDRD, r16
  ldi       r16, (1<<USI_CLK_PIN) | (1<<USI_DO_PIN) | (1<<USI_LATCH_PIN)
  out       DDRB, r16
  ; sbi       PORTD, PD2
  ; sbi       PORTD, PD3
  ; sbi       PORTD, PD4
  ; sbi       PORTD, PD5

  ldi       TEMP_REG_A, 4
  sts       DISPLAY_DIGIT, TEMP_REG_A
  
  ldi       TEMP_REG_A, 3
  sts       DISPLAY_DIGIT+1, TEMP_REG_A

  ldi       TEMP_REG_A, 2
  sts       DISPLAY_DIGIT+2, TEMP_REG_A

  ldi       TEMP_REG_A, 1
  sts       DISPLAY_DIGIT+3, TEMP_REG_A

  clr       r1
  ldi	      ZL,LOW(2*DISPLAY_SYMBOLS)
	ldi	      ZH,HIGH(2*DISPLAY_SYMBOLS)

  rjmp      LOOP

;=START================================================================================================;
; Transmit byte into 74HC595 using USI
;======================================================================================================;
USI_TRANSMIT:
  out       USIDR, r0            ; Move byte from temp register to USI Data Register

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
ret
;=END==================================================================================================;

; .MACRO DISPLAY_SEND
;   ldi       TEMP_REG_A, @0
;   mov       r0, TEMP_REG_A
;   rcall     USI_TRANSMIT
; .ENDMACRO

;========================================;
;            MAIN PROGRAM LOOP           ;
;========================================;

LOOP:
  rcall     TOGGLE_POWER_LED
  rcall     DISPLAY_INDICATE
  ; rcall     USI_TRANSMIT

  ; ldi       TEMP_REG_B, 10
  ; cp        r1, TEMP_REG_B
  ; brge      RESET_POINTER
  ; rjmp      DISPLAY_SEND

  ; RESET_POINTER:
  ;   clr     r1
  ;   ldi	    ZL,LOW(2*DISPLAY_SYMBOLS)
	;   ldi	    ZH,HIGH(2*DISPLAY_SYMBOLS)

  ; DISPLAY_SEND:
  ;   lpm
  ;   adiw    Z, 1
  ;   inc     r1
  ;   rcall   USI_TRANSMIT

  ; rcall     DELAY
  ; rcall     DELAY
  ; rcall     DELAY
  ; rcall     DELAY

  rjmp      LOOP

DISPLAY_INDICATE:
  sbi       PORTD, DIGIT_1_PIN
  rcall     DELAY
  cbi       PORTD, DIGIT_1_PIN

  sbi       PORTD, DIGIT_2_PIN
  rcall     DELAY
  cbi       PORTD, DIGIT_2_PIN

  sbi       PORTD, DIGIT_3_PIN
  rcall     DELAY
  cbi       PORTD, DIGIT_3_PIN

  sbi       PORTD, DIGIT_4_PIN
  rcall     DELAY
  cbi       PORTD, DIGIT_4_PIN
ret

DELAY:
  push      r16
  push      r17
  cli
  ldi       r16, 100
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
  sei

  pop       r17
  pop       r16
ret                    

TOGGLE_POWER_LED:
  ldi       r16,   (1<<LED_POWER_PIN)
  in        r17,   PORTD
  eor       r17,   r16
  out       PORTD, r17
ret

DISPLAY_SYMBOLS:
      ; HGFEDCBA    HGFEDCBA
  .DB 0b11000000, 0b11111001          ; 0, 1
  .DB 0b10100100, 0b10110000          ; 2, 3
  .DB 0b10011001, 0b10010010          ; 4, 5
  .DB 0b10000010, 0b11111000          ; 6, 7
  .DB 0b10000000, 0b10010000          ; 8, 9

;========================================;
;             EEPROM SEGMENT             ;
;========================================;

.ESEG
INFO:       .DB "AVR Thermostat. Written by Sergey Yarkov 22.01.2023"