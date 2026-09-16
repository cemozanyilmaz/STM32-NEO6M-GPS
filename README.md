# STM32 NUCLEO-L476RG NEO-6M GPS Driver & Parser

This project is an STM32 HAL firmware project for reading a NEO-6M GPS module
with a NUCLEO-L476RG development board.

The NEO-6M library, driver, and NMEA parser in this project are written
manually from scratch for this firmware. The GPS module sends NMEA sentences
over UART. The custom driver receives those sentences on `USART1`, the parser
extracts the supported GPS data fields, and the main application prints a
formatted position and satellite report over `USART2`.

## Features

- STM32L476RG / NUCLEO-L476RG based project
- Self-written NEO-6M GPS library/driver for STM32 HAL
- Self-written NMEA parser implementation
- NEO-6M GPS communication over `USART1`
- Interrupt-based UART receive for incoming NMEA data
- Queued sentence processing outside the interrupt handler
- Parsed GPS status, time, date, position, speed, course, fix, dilution, and
  altitude values
- Parsed satellite PRN lists for satellites used in the fix
- Parsed GSV satellite-in-view data, including PRN, elevation, azimuth, and SNR
- Human-readable serial output over `USART2`

## Hardware

- NUCLEO-L476RG development board
- NEO-6M GPS module
- USB cable for board power and serial monitor access
- Jumper wires

## Default Configuration

| Item | Value |
| --- | --- |
| MCU | STM32L476RG |
| Board | NUCLEO-L476RG |
| GPS module | NEO-6M |
| GPS UART | `USART1` |
| GPS UART pins | `PA9` / `PA10` |
| GPS UART baud rate | `9600` |
| Serial output UART | `USART2` |
| Serial output pins | `PA2` / `PA3` |
| Serial output baud rate | `9600` |
| Main print interval | `3000 ms` |

## Wiring

Typical wiring for the current firmware configuration:

| NEO-6M Pin | NUCLEO-L476RG |
| --- | --- |
| VCC | 3.3 V or module-supported supply voltage |
| GND | GND |
| TX | `PA10` / `USART1_RX` |
| RX | `PA9` / `USART1_TX` |

Make sure the GPS module and STM32 board share a common ground. Many NEO-6M
breakout boards can accept 5 V input, but the UART signal level should be
compatible with the STM32 pins.

## Project Structure

```text
Core/
  Inc/
    neo6m.h                Self-written NEO-6M driver/parser API and data types
    usart.h                STM32 UART declarations
    i2c.h                  STM32 I2C declarations
    gpio.h                 STM32 GPIO declarations
    main.h                 Main application declarations
  Src/
    main.c                 Main application loop and UART output
    neo6m.c                Self-written NEO-6M driver and NMEA parser
    usart.c                USART1 and USART2 initialization
    i2c.c                  I2C initialization generated for the project
    gpio.c                 GPIO initialization
Drivers/                   STM32 HAL and CMSIS drivers
L476_NEO6M.ioc            STM32CubeMX project configuration
startup_stm32l476xx.s      STM32L476 startup file
```

## How It Works

1. `main.c` initializes HAL, the system clock, GPIO, `USART2`, `I2C1`, and
   `USART1`.
2. `NEO6M_Init(&huart1)` starts the custom NEO-6M driver and one-byte interrupt
   reception from the GPS.
3. `HAL_UART_RxCpltCallback()` calls `NEO6M_RxCallback()` whenever a byte is
   received on `USART1`.
4. `NEO6M_RxCallback()` collects bytes until a newline is received, then stores
   the complete NMEA sentence in an internal queue.
5. The main loop calls `NEO6M_Process()` to parse all queued NMEA sentences.
6. `NEO6M_GetData()` returns the latest parsed GPS data.
7. The main loop prints the latest GPS summary and satellite details over
   `USART2`, then waits 3 seconds before repeating.

Parsing is intentionally kept out of the UART interrupt handler. The interrupt
only collects complete sentences, while the main loop performs the heavier
string parsing.

## Supported NMEA Sentences

| Sentence | Parsed data |
| --- | --- |
| `$GPRMC` | Valid flag, UTC time, latitude, longitude, speed, course, date |
| `$GPGGA` | Fix quality, satellites used, HDOP, altitude |
| `$GPGSA` | Fix type, PRNs used for the fix, PDOP, HDOP, VDOP |
| `$GPGSV` | Satellites in view, PRN, elevation, azimuth, SNR |

Latitude and longitude are converted from NMEA `ddmm.mmmm` /
`dddmm.mmmm` format into decimal degrees. South and west coordinates are stored
as negative values.

## Serial Output

Open a serial monitor on the board's `USART2` virtual COM port with:

```text
Baud rate: 9600
Data bits: 8
Parity:    None
Stop bits: 1
```

Example output format:

```text
Valid: 1 | Time: 12:34:56 | Date: 16/09/26 | Lat: 41.123456 N | Lon: 29.123456 E | Speed: 0.12 | Course: 85.30 | Fix: 1 | Fix Type: 3 | Sat: 8 | View: 12 | PDOP: 1.20 | HDOP: 0.80 | VDOP: 0.90 | Alt: 102.45 m | Used PRN: 03 08 11 16 22 27 31 32
Satellite 01 | PRN: 03 | Elevation: 45 deg | Azimuth: 120 deg | SNR: 38 dB-Hz
Satellite 02 | PRN: 08 | Elevation: 62 deg | Azimuth: 210 deg | SNR: 41 dB-Hz
```

The exact values depend on GPS reception, antenna placement, satellite
visibility, and whether the module has a valid fix.

## Important Files

- `Core/Src/main.c`: initializes the board, processes GPS data, and prints the
  serial report.
- `Core/Inc/neo6m.h`: defines `NEO6M_Data`, satellite data structures, and the
  public custom GPS driver/parser API.
- `Core/Src/neo6m.c`: custom NEO-6M library/driver implementation. It receives
  NMEA sentences, queues them, parses supported sentence types, and stores the
  latest GPS data.
- `Core/Src/usart.c`: configures `USART1` for GPS input and `USART2` for serial
  output.

## Notes

- The current parser recognizes `GP` talker sentences such as `$GPRMC`,
  `$GPGGA`, `$GPGSA`, and `$GPGSV`.
- The NMEA sentence buffer length is `128` bytes.
- The internal complete-sentence queue holds up to `8` sentence slots.
- GSV satellite details are stored for up to `16` satellites.
- If the sentence queue fills faster than the main loop processes it, new
  complete sentences are discarded until space is available.
