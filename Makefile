PROGRAMMER_PORT = usb
#PROGRAMMER_BAUD = 115200
PROGRAMMER = avrispmkII

# Target filename
#TARGET := current_dir_name

# Source files c, c++, assembly (.c .cpp .cc .S)
#SRC = first_of(main.* TARGET.*)
#SRC += additional_source.cpp

# Define known board. To list all known boards run 'make boards_list'
#BOARD := teensy2
# If you do not know the board or something is different
MCU := atmega328p
F_CPU := 1MHz
F_CLOCK := 1MHz
# run 'make info' to know these values


# List macro defines here (-D for gcc)
#DEFS = -DFOOBAR="foo bar" -DBAZ=baz

# Uncomment if you do not want to have yaal to setup cpu pre-scaler and
# do not want to use void loop(); nor void setup();
YAAL_NO_INIT = 1

# If you are planning to change cpu.clock (F_CPU) at runtime,
# you should set following option, so yaal methods will get F_CPU at runtime
#DEFS += -DYAAL_UNSTABLE_F_CPU
# NOTE: this could make your code slow and big

# Here happens all the magic
YAAL := vendor/yaal
include ./vendor/yaamake/makefile.ext

# Additions by megari
build_hex_ntp: DEFS += -DDEC_TEST_PATTERN=0
build_hex_ntp: build_hex

build_hex: clean build hex eep

build_debug: DEFS += -DDEBUG=1
build_debug: build_hex

build_debug_ntp: DEFS += -DDEBUG=1
build_debug_ntp: build_hex_ntp

build_debug2: DEFS += -DDEBUG=2
build_debug2: build_hex

build_debug2_ntp: DEFS += -DDEBUG=2
build_debug2_ntp: build_hex_ntp

build_no_autoreset: DEFS += -DAUTORESET=0
build_no_autoreset: build_hex

build_no_panic: DEFS += -DERROR_PANIC=0
build_no_panic: build_hex

# run 'make help' for information
