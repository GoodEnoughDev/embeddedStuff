PROJECT = usartTest
BUILD_DIR = bin

SHARED_DIR = ../my-common-code
SHARED_DIR += ../my-common-code/freertos/include
SHARED_DIR += ../my-common-code/freertos/portable/GCC/ARM_CM4F

CFILES = my-project.c
CFILES += api.c
CFILES += printf.c
CFILES += freertos/tasks.c
CFILES += freertos/queue.c
CFILES += freertos/list.c
CFILES += port.c
CFILES += freertos/portable/MemMang/heap_2.c
AFILES += api-asm.S

# TODO - you will need to edit these two lines!
DEVICE=stm32f446zet6
OOCD_FILE = board/st_nucleo_f4.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
$(info $$var is [${INCLUDES}])
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
