# RCOM Lab01 – Data Link Protocol

## Objective
Exchange arrays of bytes between two computers using a serial cable and implement the **establishment phase** of the data link protocol.

## Project Structure
```
lab01/
├── Makefile
├── read_noncanonical.c
├── write_noncanonical.c
└── .obj/               # Object files are stored here (created automatically)
```

- `read_noncanonical.c`: Program that opens the serial port and reads data.  
- `write_noncanonical.c`: Program that opens the serial port and writes data.  
- `.obj/`: Hidden directory created automatically to hold compiled object files.  

## Compilation
To build both executables, simply run:
```bash
make
```

This will generate the binaries:
- `read_noncanonical`
- `write_noncanonical`

Object files (`.o`) will be placed inside the hidden `.obj/` directory.

To remove only the object files:
```bash
make clean
```

To remove both object files **and** the executables:
```bash
make fclean
```

To force a full rebuild (clean + compile again):
```bash
make re
```

## Usage
Run the programs with the serial port device as argument.  
For example, if your serial port is `/dev/ttyS1`:
```bash
./read_noncanonical /dev/ttyS1
./write_noncanonical /dev/ttyS1
```

Make sure you use the correct device name for your system (e.g., `/dev/ttyS0`, `/dev/ttyUSB0`, etc.).

## Notes
- These programs are example implementations and must be extended according to the lab protocol (e.g., connection establishment with SET/UA exchange, disconnection, data transfer).  
- You must have permission to access the serial device. If necessary, run with `sudo` or adjust your user’s group permissions.  
- Object files are always stored in the hidden `.obj` directory to keep the project clean.  
