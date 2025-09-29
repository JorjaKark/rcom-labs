# Makefile for Lab 1 - Second Class
# Builds transmitter and receiver

CC = gcc
CFLAGS = -Wall -Wextra -O2

# Default target: build both programs
all: transmitter receiver

transmitter: transmitter.c
	$(CC) $(CFLAGS) -o $@ $<

receiver: receiver.c
	$(CC) $(CFLAGS) -o $@ $<

# Clean up build artifacts
clean:
	rm -f transmitter receiver *.o
