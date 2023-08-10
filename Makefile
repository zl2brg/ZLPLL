BUILDDIR := build


CPU := atmega328p
CC 	:= avr-gcc
CPP := avr-gcc -E
ARCH_OPTS := -mmcu=$(CPU) -mcall-prologues -fno-jump-tables
OPTIMISE_OPTS := -Os -fno-if-conversion -finline-limit=10
COMPLIANCE_OPTS := -Wall -Wno-pointer-sign

DEBUG_OPTS := -gstabs $(SAVE_TEMPS)
CFLAGS  := $(ARCH_OPTS) -D F_CPU=8000000 $(OPTIMISE_OPTS) $(COMPLIANCE_OPTS) $(VERSION_DEFS) $(USER_DEFS)  -I '/usr/lib/avr/include'

LINKER    :=avr-gcc
LDFLAGS    = -mmcu=$(CPU) -D F_CPU=8000000 -Wl,--cref,-Map=zlpll.map,-u,vfprintf -lprintf_flt -lm

INCLUDES := $(patsubst %,-I %,$(INCLUDE))
SYSINCLUDES := $(patsubst %,-I %,$(SYSINCLUDE))

SOURCES += zlpll.c
SOURCES += adf5355.c
SOURCES += adf5355.c
SOURCES += encoder.c
SOURCES += i2c_slave.c
SOURCES += i2c.c
SOURCES += lcd.c
SOURCES += lcd_bargraph.c
SOURCES += menu.c
SOURCES += mgm.c
SOURCES += usart.c
SOURCES += twi.c

HEADERS += zlpll.h
HEADERS += adf5355.h
HEADERS += cw.h
HEADERS += encoder.h
HEADERS += i2c_slave.h
HEADERS += i2c.h
HEADERS += lcd.h
HEADERS += menu.h
HEADERS += mgm.h
HEADERS += usart.h
HEADERS += twi.h
HEADERS += config.h


OBJS := $(SOURCES:.c=.o) $(ASOURCE:.S=.o)


# a.out
a.out: $(OBJS)
	${LINKER} ${LDFLAGS} -o $@ ${filter-out %.a %.so, $^} ${LOADLIBES}
	avr-objcopy -O ihex a.out zlpll.hex
	avrdude -p m328p -c avrisp2 -e -U flash:w:zlpll.hex:i



all: $(HEADERS) $(SOURCES)

%.c.o:
	$(info DEPS="$(DEPS)")
	$(COMPILE) $(OPTIONS) -c -o $@ $<



.PHONY : clean
clean:
	@${RM} *.out $(SOURCES:.c=.i) $(SOURCES:.c=.s)
	@${RM} *.o
	@${RM} -rf $(BUILDDIR)
