CC = gcc

CFLAGS = -Wall -Wextra -Werror=missing-prototypes -Wswitch -Wformat
CFLAGS += -Wchar-subscripts -Wparentheses  -Wtrigraphs -Wpointer-arith
CFLAGS += -Wmissing-declarations -Wredundant-decls  -Wundef -Wmain
CFLAGS += -Wreturn-type -Wmultichar  -Wunused -Wmissing-braces -Werror
CFLAGS += -Wno-missing-field-initializers

CEXTRA = -pthread -std=gnu99
SOURCES = tcu_com.c raw_capture.c rx_stall.c

all: tcu_muxer

check: tcu_muxer_integration tcu_muxer_rx_stall_test tcu_muxer_stall_fixture
	./tcu_muxer_rx_stall_test
	python3 tests/test_physical_uart_hup.py ./tcu_muxer_integration
	python3 tests/test_silent_rx_stall.py ./tcu_muxer_stall_fixture

tcu_muxer: $(SOURCES) rx_stall.h
	$(CC) $(CFLAGS) $(CEXTRA) $(SOURCES) -o tcu_muxer

# Host-side integration tests cannot create UUCP locks in /var/lock. Compile
# the identical muxer with only its lock directory redirected to /tmp.
tcu_muxer_integration: $(SOURCES) rx_stall.h
	$(CC) $(CFLAGS) $(CEXTRA) -DUUCP_DIR='"/tmp"' $(SOURCES) -o $@

tcu_muxer_rx_stall_test: tests/test_rx_stall.c rx_stall.c rx_stall.h
	$(CC) $(CFLAGS) $(CEXTRA) tests/test_rx_stall.c -o $@

tcu_muxer_stall_fixture: tests/rx_stall_fixture.c $(SOURCES) rx_stall.h
	$(CC) $(CFLAGS) $(CEXTRA) -DUUCP_DIR='"/tmp"' $(SOURCES) tests/rx_stall_fixture.c -Wl,--wrap=rx_stall_check -o $@

clean:
	rm -f *.o
	rm -f tcu_muxer tcu_muxer_integration tcu_muxer_rx_stall_test tcu_muxer_stall_fixture
