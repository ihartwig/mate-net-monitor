# mate-net-monitor
Baseboard PCBA for MATE Net and Magnum Net power system monitoring with a Particle Boron.

## GPIO Pin Usage

| Pin | Function        | Usage |
|-----|-----------------|-------|
| 0   | SDA             | -- (expansion connector)
| 1   | SCL             | -- (expansion connector)
| 2   | D2 / SPI_SCK    | -- (Mag Net activity indicator low)
| 3   | D3 / SPI_MOSI   | -- (Mag Net xcvr enable low unpopulated)
| 4   | D4 / SPI_MISO   | -- (Mag Net UART TX?)
| 5   | D5              | -- (Mag Net UART RX?)
| 6   | D6              | IRQ from MAX3100 adpt
| 7   | D7              | --
| 8   | D8              | --
| 9   | D9 / UART1_TX   | UART from MAX3100 adpt
| 10  | D10 / UART1_RX  | UART to MAX3100 adpt
| 11  | MI              | SPI to MAX3100 adpt
| 12  | MO              | SPI to MAX3100 adpt
| 13  | SCK             | SPI to MAX3100 adpt
| 14  | A5 / SPI_SS     | SPI to MAX3100 adpt
| 15  | A4              | -- (expansion connector)
| 16  | A3              | -- (expansion connector)
| 17  | A2              | -- (expansion connector)
| 18  | A1              | -- (expansion connector)
| 19  | A0              | MATE Net activity indicator high
| 20  |                 | --

## Hardware

* [pcb](pcb) - main baseboard with connectors and interface circuits for MATE Net and Magnum Net
* [pcb-max3100-uart-adpt](pcb-max3100-uart-adpt) - extra max3100 UART port (capable of 9N1) for MATE Net

The `pcb` dc-dc power supply required some rework to achieve stable operation.

![mate-net-monitor-rework-dcdc](pcb/mate-net-monitor-rework-dcdc.png)

Additionally, some modifications during assembly are required compared to the as-designed files here. This primarily addresses pin assignment issues:

* Mag Net is attached to pins that do not have a UART; remap these on the Boron board as the internal UART can be used for this.
* Mate Net needs a 9-bit capable UART; stack the `pcb-max3100-uart-adpt` on the bottom side of the `pcb` mainboard to add this.
  * GND, 3V3, and /IRQ on the far side need to be wired to sources on the mainboard.
  * /SHDN needs to be pulled up (via 10k) to 3V3.

## Firmware

* [fw-mate-net-monitor](fw-mate-net-monitor) - WIP application firmware for reporting MATE inverter stats to particle cloud
* [fw-test](fw-test) - simple hardware test firmware for burn-in testing of interfaces

Inside the `fw-test` folder there is a MAX3100Serial9B library extended from [ewpa's MAX3100Serial](https://github.com/ewpa/MAX3100Serial) with some adaptations to Particle and significant new features:

* addition of software buffers and interrupt handling for continuous stream operation
* irq, sent, read, and overflow counters
* (WIP) 9-bit read and write configuration
