CC = gcc

CFLAGS = -Wall -Wextra -Werror=missing-prototypes -Wswitch -Wformat
CFLAGS += -Wchar-subscripts -Wparentheses  -Wtrigraphs -Wpointer-arith
CFLAGS += -Wmissing-declarations -Wredundant-decls  -Wundef -Wmain
CFLAGS += -Wreturn-type -Wmultichar  -Wunused -Wmissing-braces -Werror
CFLAGS += -Wno-missing-field-initializers

CEXTRA = -pthread -std=gnu99
SOURCES = tcu_com.c

all: tcu_muxer

tcu_muxer: tcu_com.c
	$(CC) $(CFLAGS) $(CEXTRA) tcu_com.c -o tcu_muxer

clean:
	rm -f *.o
	rm -f tcu_muxer
