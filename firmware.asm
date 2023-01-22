;
; Project name: thermostat
; Description: Electronic thermostat on AVR Microcontroller
; Source code: https://github.com/sergeyyarkov/attiny2313a_thermostat
; Device: ATtiny2313A
; Device Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf
; Package: SOIC-20W_7.5x12.8mm_P1.27mm
; Assembler: AVR macro assembler 2.2.7
; Clock frequency: 12 MHz External Crystal Oscillator
; Fuses: lfuse: , hfuse: , efuse: , lock:
;
; Written by Sergey Yarkov 22.01.2023

.INCLUDE "tn2313Adef.inc"
.LIST

.DEF tmp_r_a = r16

.DSEG

.CSEG
.ORG 0x00

; MCU VECTORS
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
  ldi r16, (1<<PD6)
  out DDRD, r16
  out PORTD, r16
ret

;
; MAIN PROGRAM LOOP
LOOP:
  nop
  rjmp LOOP

INFO: .DB "AVR Thermostat. Written by Sergey Yarkov 22.01.2023"

.ESEG