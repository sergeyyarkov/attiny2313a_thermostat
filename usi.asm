; **** ОТПРАВКА БАЙТА В СДВИГОВЫЙ РЕГИСТР *************************
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