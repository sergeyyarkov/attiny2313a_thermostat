;
; Project name: thermostat
; Description: Electronic thermostat on AVR Microcontroller
; Source code: https://github.com/sergeyyarkov/attiny2313a_thermostat
; Device: ATtiny2313A
; Device Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf
; Package: SOIC-20W_7.5x12.8mm_P1.27mm
; Assembler: AVR macro assembler 2.2.7
; Clock frequency: 12 MHz External Crystal Oscillator
; Fuses: lfuse: 0x4F, hfuse: 0x9F, efuse: 0xFF, lock: 0xFF
;
; Written by Sergey Yarkov 22.01.2023

.INCLUDE "tn2313Adef.inc"
.LIST

.DEF tmp_r_a = r16

;========================================;
;                LABELS                  ;
;========================================;

.EQU LED_POWER_PORT = PD6


;========================================;
;              CODE SEGMENT              ;
;========================================;

.CSEG
.ORG 0x00

;========================================;
;                VECTORS                 ;
;========================================;

rjmp 	RESET_vect			      ; Program start at RESET vector
reti                        ; External Interrupt Request 0 / inactive
reti		                    ; External Interrupt Request 1 / inactive
reti                        ; Timer/Counter1 Capture Event / inactive
reti		                    ; Timer/Counter1 Compare Match A / inactive
reti                        ; Timer/Counter1 Overflow / inactive
reti                        ; Timer/Counter0 Overflow / inactive
reti                        ; USART0, Rx Complete / inactive
reti                        ; USART0 Data Register Empty / inactive
reti						            ; USART0, Tx Complete / inactive
reti                        ; Analog Comparator / inactive
reti	                      ; Pin Change Interrupt Request 0/ inactive
reti                        ; Timer/Counter1 Compare Match B / inactive
reti                        ; Timer/Counter0 Compare Match A / inactive
reti                        ; Timer/Counter0 Compare Match B / inactive
reti                        ; USI Start Condition/ inactive
reti                        ; USI Overflow / inactive
reti                        ; EEPROM Ready/ inactive
reti                        ; Watchdog Timer Overflow / inactive
reti                        ; Pin Change Interrupt Request 1 / inactive
reti                        ; Pin Change Interrupt Request 2 / inactive

RESET_vect:
  ;
  ; INIT STACK POINTER
  ldi tmp_r_a, low(RAMEND)
  out SPL, tmp_r_a

MCU_INIT:
  rcall INIT_PORTS
  rjmp LOOP

INIT_PORTS:
  ldi r16, (1<<LED_POWER_PORT)
  out DDRD, r16
  out PORTD, r16
ret

;========================================;
;            MAIN PROGRAM LOOP           ;
;========================================;

LOOP:
  ldi r16, (1<<LED_POWER_PORT)
  out PORTD, r16

  rcall DELAY
  rcall DELAY

  ldi r16, (0<<LED_POWER_PORT)
  out PORTD, r16

  rcall DELAY
  rcall DELAY

  rjmp LOOP

DELAY:
  ldi     r16, 255     ; 1 clock cycle
  _DEL_1:
    ldi   r17, 255    ; 1 clock cycle
  _DEL_2:
    dec   r17         ; 1 clock cycle
    nop               ; 1 clock cycle
    brne  _DEL_2      ; 2 clock cycle when jumping / 1 when continue 

    dec   r16         ; 1 clock cycle
    brne  _DEL_1      ; 2 clock cycle when jumping / 1 when continue 
ret

INFO: .DB "AVR Thermostat. Written by Sergey Yarkov 22.01.2023"

;========================================;
;             EEPROM SEGMENT             ;
;========================================;

.ESEG