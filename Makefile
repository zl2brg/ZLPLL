BUILDDIR := build

CPU := atmega328p
CC := avr-gcc
CPP := avr-gcc -E
ARCH_OPTS := -mmcu=$(CPU) -mcall-prologues -fno-jump-tables
OPTIMISE_OPTS := -Os -fno-if-conversion -finline-limit=10 -Wall -Wextra
COMPLIANCE_OPTS := -Wall -Wno-pointer-sign

DEBUG_OPTS := -gstabs $(SAVE_TEMPS)
CFLAGS := $(ARCH_OPTS) -mmcu=$(CPU) -D F_CPU=8000000 $(OPTIMISE_OPTS) $(COMPLIANCE_OPTS) $(VERSION_DEFS) $(USER_DEFS) -I '/usr/lib/avr/include'

LINKER := avr-gcc
LDFLAGS := -mmcu=$(CPU) -D F_CPU=8000000 -Wl,--cref,-Map=$(BUILDDIR)/zlpll.map,-u,vfprintf -lprintf_flt -lm

INCLUDES := $(patsubst %,-I %,$(INCLUDE))
SYSINCLUDES := $(patsubst %,-I %,$(SYSINCLUDE))

SOURCES := zlpll.c \
           adf5355.c \
           encoder.c \
           i2c_slave.c \
           i2c.c \
           lcd.c \
           lcd_bargraph.c \
           menu.c \
           mgm.c \
           usart.c \
           twi.c
    #       libdiv64.S \
    #       libmul64.S

HEADERS := zlpll.h \
           adf5355.h \
           cw.h \
           encoder.h \
           i2c_slave.h \
           i2c.h \
           lcd.h \
           menu.h \
           mgm.h \
           usart.h \
           twi.h \
           config.h

# Object files should be placed in the build directory
OBJS := $(patsubst %.c,$(BUILDDIR)/%.o,$(patsubst %.S,$(BUILDDIR)/%.o,$(SOURCES)))

# Default target
all: $(BUILDDIR)/a.out

# Create the build directory if it doesn't exist
$(BUILDDIR):
	mkdir -p $(BUILDDIR)

# Compile C source files
$(BUILDDIR)/%.o: %.c | $(BUILDDIR)
	$(info Compiling $<)
	$(CC) $(CFLAGS) -c -o $@ $<

# Compile assembly source files
$(BUILDDIR)/%.o: %.S | $(BUILDDIR)
	$(info Compiling $<)
	$(CC) $(CFLAGS) -c -o $@ $<

# Link object files to create the final executable
$(BUILDDIR)/a.out: $(OBJS) | $(BUILDDIR)
	$(info Linking $@)
	$(LINKER) $(LDFLAGS) -o $@ $(OBJS)
	avr-objcopy -O ihex $(BUILDDIR)/a.out $(BUILDDIR)/zlpll.hex
	avrdude -p m328p -c avrisp2 -e -U flash:w:$(BUILDDIR)/zlpll.hex:i
#	avrdude -p m328p -c dragon_isp -e -U flash:w:$(BUILDDIR)/zlpll.hex:i

.PHONY : clean
clean:
	@$(RM) $(BUILDDIR)/*.out $(BUILDDIR)/*.i $(BUILDDIR)/*.s
	@$(RM) $(BUILDDIR)/*.o
	@$(RM) -rf $(BUILDDIR)
