CC = gcc

CFLAGS = -Wall -Wextra -Werror=missing-prototypes -Wswitch -Wformat
CFLAGS += -Wchar-subscripts -Wparentheses  -Wtrigraphs -Wpointer-arith
CFLAGS += -Wmissing-declarations -Wredundant-decls  -Wundef -Wmain
CFLAGS += -Wreturn-type -Wmultichar  -Wunused -Wmissing-braces -Werror
CFLAGS += -Wno-missing-field-initializers

CEXTRA = -pthread -std=gnu99
SOURCES = tcu_com.c raw_capture.c

all: tcu_muxer

check: tcu_muxer_integration
	python3 tests/test_physical_uart_hup.py ./tcu_muxer_integration

tcu_muxer: $(SOURCES)
	$(CC) $(CFLAGS) $(CEXTRA) $(SOURCES) -o tcu_muxer

# Host-side integration tests cannot create UUCP locks in /var/lock. Compile
# the identical muxer with only its lock directory redirected to /tmp.
tcu_muxer_integration: $(SOURCES)
	$(CC) $(CFLAGS) $(CEXTRA) -DUUCP_DIR='"/tmp"' $(SOURCES) -o $@

clean:
	rm -f *.o
	rm -f tcu_muxer tcu_muxer_integration
