# RCOM Lab01 – Data Link Protocol

## Objective
Exchange arrays of bytes between two computers using a serial cable and implement the **establishment phase** of the data link protocol.

In addition to running with **real serial ports**, you can also use the provided **virtual cable** (`cable.c`) to emulate a serial connection between two programs on the same machine.

---

## Project Structure
```
lab01/
├── Makefile
├── read_noncanonical.c
├── write_noncanonical.c
├── cable.c
└── .obj/               # Object files are stored here (created automatically)
```

- `read_noncanonical.c`: Program that opens the serial port and reads data.  
- `write_noncanonical.c`: Program that opens the serial port and writes data.  
- `cable.c`: Virtual serial cable emulator using `socat`. Creates two virtual serial ports so the reader and writer can communicate without real hardware.  
- `.obj/`: Hidden directory created automatically to hold compiled object files.  

---

## Compilation
To build all executables:
```bash
make
```

This will generate:
- `read_noncanonical`
- `write_noncanonical`
- `cable`

Object files (`.o`) will be placed inside `.obj/`.

To remove only object files:
```bash
make clean
```

To remove both object files **and** executables:
```bash
make fclean
```

To force a full rebuild:
```bash
make re
```

---

## Usage

### With Real Serial Ports
Run the programs with the serial port device as argument.  
Example:
```bash
./read_noncanonical /dev/ttyS1
./write_noncanonical /dev/ttyS1
```
Make sure you use the correct device name for your system (e.g., `/dev/ttyS0`, `/dev/ttyUSB0`, `/dev/tty.usbserial-XXXX`).

---

### With the Virtual Cable (recommended for testing on one machine)
1. Start the cable emulator:
   ```bash
   ./cable
   ```
   This will create two linked virtual serial ports (symlinks `ttyS10` and `ttyS11` in the current directory).

2. Run the programs, each attached to one end of the cable:
   ```bash
   ./read_noncanonical ./ttyS10
   ./write_noncanonical ./ttyS11
   ```
   Data written on one side will appear on the other.

3. The emulator accepts interactive commands in its terminal:
   - `on` / `off` — connect or disconnect the virtual cable  
   - `baud <rate>` — set baud rate (default 9600)  
   - `prop <delay>` — set propagation delay in microseconds  
   - `ber <value>` — introduce errors with given Bit Error Rate  
   - `log <file>` — save traffic to a log file  
   - `quit` — exit the emulator  

---

### Test Procedure with `screen`
You can verify that the virtual cable works without running your own programs:

1. **Keep the cable emulator running** in one terminal:
   ```bash
   ./cable
   ```
   (this must stay open).

2. **Check the symlinks created**:
   ```bash
   ls -l ttyS10 ttyS11
   ```
   Example output:
   ```
   ttyS10 -> /dev/ttys002
   ttyS11 -> /dev/ttys004
   ```
   In this case, the real devices are `/dev/ttys002` and `/dev/ttys004`.

3. **Open one end** of the cable in a second terminal:
   ```bash
   screen /dev/ttys002 9600
   ```

4. **Open the other end** in a third terminal:
   ```bash
   screen /dev/ttys004 9600
   ```

5. Type characters in one window — they should appear in the other.

6. To exit `screen`: press `Ctrl+A`, then `\`, then confirm with `y`.

**Note:** Always use the **real `/dev/ttysXXX` device** (the right-hand side of the symlink), since `screen` may not recognize the relative symlink `./ttyS10`.

---

## Result
<img width="539" height="73" alt="image" src="https://github.com/user-attachments/assets/97f52edf-f737-48fb-b013-c2808c62db2c" />

<img width="550" height="71" alt="image" src="https://github.com/user-attachments/assets/14ee7f7e-319d-4016-a0e2-e3f52ce7c3b2" />

---
## Notes
- Extend `read_noncanonical.c` and `write_noncanonical.c` to implement the full lab protocol (SET/UA exchange, disconnection, data transfer).  
- You must have permission to access the serial device files. If necessary, run with `sudo` or adjust group permissions.  
- The **virtual cable** allows you to test your protocol even if you do not have access to real serial hardware.  
