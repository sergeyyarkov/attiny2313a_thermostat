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
.DEF COUNTER          = r20

;========================================;
;                LABELS                  ;
;========================================;

.EQU LED_POWER_PORT   = PD6
.EQU USI_LATCH_PIN    = PB0   ; ST_CP on 74HC595
.EQU USI_DO_PIN       = PB6   ; DS on 74HC595
.EQU USI_CLK_PIN      = PB7   ; SH_CP on 74HC595

;========================================;
;              CODE SEGMENT              ;
;========================================;

.CSEG
.ORG 0x00

;========================================;
;                VECTORS                 ;
;========================================;

rjmp 	RESET_vect			      ; Program start at RESET vector
;reti                        ; External Interrupt Request 0 / inactive
;reti		                    ; External Interrupt Request 1 / inactive
;reti                        ; Timer/Counter1 Capture Event / inactive
;reti		                    ; Timer/Counter1 Compare Match A / inactive
;reti                        ; Timer/Counter1 Overflow / inactive
;reti                        ; Timer/Counter0 Overflow / inactive
;reti                        ; USART0, Rx Complete / inactive
;reti                        ; USART0 Data Register Empty / inactive
;reti						            ; USART0, Tx Complete / inactive
;reti                        ; Analog Comparator / inactive
;reti	                      ; Pin Change Interrupt Request 0/ inactive
;reti                        ; Timer/Counter1 Compare Match B / inactive
;reti                        ; Timer/Counter0 Compare Match A / inactive
;reti                        ; Timer/Counter0 Compare Match B / inactive
;reti                        ; USI Start Condition/ inactive
;reti                        ; USI Overflow / inactive
;reti                        ; EEPROM Ready/ inactive
;reti                        ; Watchdog Timer Overflow / inactive
;reti                        ; Pin Change Interrupt Request 1 / inactive
;reti                        ; Pin Change Interrupt Request 2 / inactive

RESET_vect:
  ;========================================;
  ;        INITIALIZE STACK POINTER        ;
  ;========================================;
  ldi       TEMP_REG_A, low(RAMEND)
  out       SPL, TEMP_REG_A

MCU_INIT:
  ;=======================================================
  ;  POWER LED    <------------>   PD6                   ;
  ;    - This LED is used to indicate that device        ;
  ;       is working correctly                           ;
  ;                                                      ;
  ;=======================================================
  rcall     INIT_PORTS
  ldi       COUNTER, 0b00000001
  rjmp      LOOP

INIT_PORTS:
  ldi       r16, (1<<LED_POWER_PORT)
  out       DDRD, r16
  ldi       r16, (1<<USI_CLK_PIN) | (1<<USI_DO_PIN) | (1<<USI_LATCH_PIN)
  out       DDRB, r16
ret

;========================================;
;          SEND BYTE TO 74HC595          ;
;========================================;

USI_TRANSMIT:
  mov       r19, COUNTER
  out       USIDR, r19            ; Move byte into USI Data Register

  ; Enable USI Overflow Interrupt Flag (will be 0 if transfer is not compeleted)
  ldi       r19, (1<<USIOIF)      
  out       USISR, r19
  
  ; Load settings of USI into temp register
  ; This will setup USI to Three-wire mode, Software clock strobe (USITC) 
  ; with External, positive edge and toggle USCK
  ;
  ; USIWM0 <--------------> USI Wire Mode
  ; USICS1 <--------------> USI Clock Source Select
  ; USICLK <--------------> USI Clock Strobe
  ; USITC  <--------------> USI Toggle Clock (Enable clock generation)      
  ldi       TEMP_REG_A, (1<<USIWM0) | (1<<USICS1) | (1<<USICLK) | (1<<USITC)
  
  _USI_TRANSMIT_LOOP:
    out       USICR, TEMP_REG_A   ; Load settings from temp register into USI Control Register
    sbis      USISR, USIOIF       ; If transfer is comleted then move out of loop
    rjmp      _USI_TRANSMIT_LOOP

  ; Send pulse into LATCH pin. 
  ; This will copy byte from 74hc595 shift register into 74hc595 storage register
  sbi      PORTB, USI_LATCH_PIN
  cbi      PORTB, USI_LATCH_PIN
ret

SUBPROGRAM_LED_SHIFTING:
  rcall     USI_TRANSMIT
  rol       COUNTER
ret

;========================================;
;            MAIN PROGRAM LOOP           ;
;========================================;

LOOP:
  rcall     TOGGLE_POWER_LED
  rcall     SUBPROGRAM_LED_SHIFTING
  rcall     DELAY
  rcall     DELAY
  rjmp      LOOP

DELAY:
  push      r16
  push      r17
  cli
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
  sei

  pop       r17
  pop       r16
ret                    

TOGGLE_POWER_LED:
  ldi       r16,  (1<<LED_POWER_PORT)
  in        r17,  PORTD
  eor       r17,  r16
  out       PORTD, r17
ret


;========================================;
;             EEPROM SEGMENT             ;
;========================================;

.ESEG
INFO:       .DB "AVR Thermostat. Written by Sergey Yarkov 22.01.2023"