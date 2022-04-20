ifeq (y,$(strip $(IIO_EXAMPLE)))
TINYIIOD=y
CFLAGS += -DIIO_EXAMPLE=1
SRCS += $(PROJECT)/src/examples/iio_example/iio_example.c
INCS += $(PROJECT)/src/examples/iio_example/iio_example.h
endif

ifeq (y,$(strip $(IIO_TRIGGER_EXAMPLE)))
TINYIIOD=y
CFLAGS += -DIIO_TRIGGER_EXAMPLE=1
SRCS += $(PROJECT)/src/examples/iio_trigger_example/iio_trigger_example.c \
	$(DRIVERS)/adc/adc_demo/iio_adc_demo_trig.c

INCS += $(PROJECT)/src/examples/iio_trigger_example/iio_trigger_example.h
endif

ifeq (y,$(strip $(TINYIIOD)))
SRC_DIRS += $(NO-OS)/iio/iio_app

INCS += $(DRIVERS)/adc/adc_demo/iio_adc_demo.h \
	$(DRIVERS)/dac/dac_demo/iio_dac_demo.h

SRCS += $(DRIVERS)/adc/adc_demo/iio_adc_demo.c \
	$(DRIVERS)/dac/dac_demo/iio_dac_demo.c 

INCS += $(INCLUDE)/no_os_list.h \
	$(PLATFORM_DRIVERS)/uart_extra.h
endif
