NEMA_REL_DIR := Lib/NemaGFX

C_SOURCES_NEMA += Src/nema_hal_freertos.c

C_INCLUDES_NEMA += -I$(NEMA_REL_DIR)/include

C_SOURCES += $(C_SOURCES_NEMA)
C_INCLUDES += $(C_INCLUDES_NEMA)

LIBDIR += -L$(NEMA_REL_DIR)/lib/core/cortex_m55/gcc
LIBS += -lnemagfx-float-abi-hard
LIBS_IAR += $(NEMA_REL_DIR)/lib/core/cortex_m55/IAR9.x/nemagfx_core_release.a
