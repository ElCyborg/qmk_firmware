
# QUANTUM_LIB_SRC += uart.c
UART_DRIVER_REQUIRED = yes
MCU_LDSCRIPT = STM32F103xB
DEFERRED_EXEC_ENABLE = yes

# Enter lower-power sleep mode when on the ChibiOS idle thread
OPT_DEFS += -DCORTEX_ENABLE_WFI_IDLE=TRUE
#OLED_ENABLE = yes
#OLED_TRANSPORT = spi




SRC +=   analog.c \
		 adc.c \
		 rgblight_custom.c\
		 rtc.c\
		 smart_ble.c
#		 //
#		 disable_swd.c
  DEBOUNCE_TYPE = sym_defer_g   