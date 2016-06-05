# Makefile for blink_until_button example program.
# Run:
#  # make
# to assemble the PRU binary and build the C loader.

CC=gcc
CFLAGS=-c -Wall
LIBS=-lprussdrv

PASM=pasm
PASMFLAGS=-b

all: pru_pwm.bin pru_pwm

pru_pwm.bin: pru_pwm.p
	$(PASM) $(PASMFLAGS) pru_pwm.p

blink_until_button: pru_pwm.o
	$(CC) $(LIBS) $^ -o pru_pwm
	chmod +x pru_pwm

pru_pwm.o: pru_pwm.c
	gcc $(CFLAGS) pru_pwm.c

clean:
	rm pru_pwm.bin pru_pwm.o pru_pwm

.PHONY: all clean
