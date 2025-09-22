# RCOM Lab01 – Non-Canonical Serial Communication

This lab demonstrates how to configure and use a serial port in **non-canonical mode** (raw mode), following the protocol steps for connection establishment.

## Project Structure

lab01/
├── Makefile
├── read_noncanonical.c
├── write_noncanonical.c
└── .obj/               # Object files are stored here (created automatically)

- `read_noncanonical.c`: Program that opens the serial port and reads data.
- `write_noncanonical.c`: Program that opens the serial port and writes data.
- `.obj/`: Hidden directory created automatically to hold compiled object files.

## Compilation

To build both executables, simply run:

```bash
make

