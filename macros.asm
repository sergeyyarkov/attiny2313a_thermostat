.MACRO outi
    ldi     @0, @2
    out     @1, @0
.ENDMACRO

.MACRO display_load

    ldi	    DISP_NUM_L,    LOW(@0)
    ldi	    DISP_NUM_H,    HIGH(@0)
.ENDMACRO

.MACRO ow_pull
    sbi     OW_DDR, OW_LINE
.ENDMACRO

.MACRO ow_release
    cbi     OW_DDR, OW_LINE
.ENDMACRO
    
.MACRO relay_on
    sbi	    RELAY_PORT, RELAY_PIN
.ENDMACRO
    
.MACRO relay_off
    cbi	    RELAY_PORT, RELAY_PIN
.ENDMACRO
        
.MACRO DELAY16
    ldi    DELAY_16_r, HIGH(@0*F_CPU/4-2)
    ldi    DELAY_8_r, LOW(@0*F_CPU/4-2)
    rcall  DELAY_LOOP_16
.ENDMACRO
    
.MACRO DELAY24
    ldi    DELAY_24_r, BYTE3(@0*F_CPU/5-3)
    ldi    DELAY_16_r, HIGH(@0*F_CPU/5-3)
    ldi    DELAY_8_r, LOW(@0*F_CPU/5-3)
    rcall  DELAY_LOOP_24
.ENDMACRO