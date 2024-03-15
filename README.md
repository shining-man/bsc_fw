# Battery safety controller (BSC)
![bsc_dashboard](https://github.com/shining-man/bsc_fw/blob/main/doc/README_Dashboard.png?raw=true)

The Battery Safety Controller (BSC) is a freely configurable controller that can be used for a wide range of control and monitoring tasks for DIY batteries. The BSC can take over the central control of the Energy-Sorange system and can be used to implement a second safety level in addition to the BMS.

The BSC consists of two components. The BSC hardware and the corresponding software.

1. The [BSC Hardware](https://github.com/shining-man/bsc_hw), which is used as middleware between the BMS and the inverter. One advantage of the BSC hardware is its secure operation. The components and design are such that all physical interfaces are galvanically isolated.

2. The BSC software, which makes the Battery Safety Controller hardware a freely configurable controller that can be used for a wide range of control and monitoring tasks in DIY battery systems.

The BSC software can also be used independently of the BSC hardware on ESP32 DevKit modules. In this case, for example, the NEEY balancer data can be transmitted to MQTT.  

The BSC display can also be used for visualization.

## Use cases of the BSC
### 1. Monitoring of connected devices (serial, Bluetooth)
For this purpose, the BSC can send the data from the connected devices (BMS, balancer, temperature sensors) via MQTT to a broker to display them graphically with Grafana, for example, or to further process the data from an automation system (ioBroker, NodRed, Home Assistant, ...).

It is possible to use the VenusOS dbus driver ([dbus-bsc-can](https://github.com/shining-man/dbus-bsc-can)) to transfer the extended data (cell voltages, temperatures, ...) via the CAN bus to the Victron system.

### 2. Charge control
For this, the BSC takes the data from the connected devices to control the inverter (Victron, Solis, DEYE, ...) connected via the CAN bus. Here are several functions available to adapt the charge control to your own DIY battery.
- Cell voltage dependent throttling of the charge current
- Reduce charge current in case of cell drift
- SoC dependent charge current reduction
- Charge current cut-off: prevents continuous recharging when the battery is full
- Combining data from the individual physical battery packs into an overall virtual battery pack, with consideration of many parameters, such as can/does the battery pack charge/discharge at all.

### 3. Second safety level besides the BMS.
The BSC can monitor various configurable parameters on connected devices (serial, Bluetooth) to create a second level of security. <br>
Parameters that can be monitored include:
- Does the connected BMS respond regularly
- Cell voltages (min/max)
- Total voltage (min/max)
- Temperatures

This can be used, for example, to control the relay outputs to trigger a load break switch.

### 4. Temperature monitoring
Up to 64 OneWire temperature sensors (DS18B20) can be connected and monitored. There are different regulations for monitoring the temperature:
- Maximum value monitoring
- Maximum value monitoring with one sensor as reference value
- Differential value monitoring

All settings can be freely parameterized via a web interface. <br>
As an example a part of the Inverter Settings:<br>
![bsc_inverter_settings](https://github.com/shining-man/bsc_fw/blob/main/doc/README_inverter_settings.png?raw=true)

## Supported BMS (RS485)
- JBD (Jiabaida)
- JK
- Seplos [1]
- DALY
- Sylcin [1]
- Gobel [1]

[1]: With the Seplos and the Sylcin BMS, up to 8 devices can be connected to one serial port.

## Supported Inverter
- Victron
- Solis, DEYE, ...

## Supported Bluetooth devices
- NEEY (up to 7 pieces) [2]
- JK-BMS (Test phase)

[2]: Special function: With the NEEY Balancer, not only can the data be read, it can also be configured via the BSC.

## Features der BSC-Hardware
* 6x Relay outputs
* 4x Digital inputs (isolated)
* 3x Serial interface (isolated) [3] 
* Onewire (64x DS18B20)
* CAN (isolated)
* Bluetooth
* WLAN

[3]: With the [Serial Extension](https://github.com/shining-man/bsc_extension_serial) the BSC can be extended by 8 additional physical and galvanically isolated serial ports.

## First steps
A lot of further information about commissioning, supported devices, information about the WebUI can be found in the [Wiki](https://github.com/shining-man/bsc_fw/wiki).

### Use with an ESP32 DevKit
### GPIOs
Interface | RX | TX
-------- | -------- | --------
**Serial 0**   | GPIO 16   | GPIO 17
**Serial 1**   | GPIO 23   | GPIO 25
**Serial 2**   | GPIO 35   | GPIO 33
**CAN**   | GPIO 5   | GPIO 4

Further description can be found [here](https://github.com/shining-man/bsc_fw/wiki/BSC%E2%80%90Test-ohne-orig.-Hardware).

## More information
* [Discord Forum](https://discord.gg/WRUhegRPQn)
(The Discord is in German, but you can also write in English. 
We are also happy to welcome members from other countries.)
* [Wiki - supported devices](https://github.com/shining-man/bsc_fw/wiki/Supported-devices)
* [Wiki (German)](https://github.com/shining-man/bsc_fw/wiki)
* [Discussions](https://github.com/shining-man/bsc_fw/discussions)
* [PCB](https://github.com/shining-man/bsc_hw)
* [Stromlaufplan](https://github.com/shining-man/bsc_hw/blob/main/circuit.pdf?raw=true "Stromlaufplan")


# Status display
To show the status of the BSC and the connected BMSes there is a status display for the BSC.<br>
More details are in the Github project of the display.
[GitHub project Display](https://github.com/shining-man/bsc_display)

![Picture_BSC_Display_Home](https://github.com/shining-man/bsc_display/blob/main/img/Disp_Home.jpg?raw=true)

# VenusOS dbus driver
To display more data in VenusOS (Victron) like the cell voltages of the connected BMS or the temperatures of the OneWire sensors, there is a dbus driver. Here the data is not transmitted via WLAN to the VenusOS, but via the CAN bus. <br>
For more info see the github project: [dbus-bsc-can](https://github.com/shining-man/dbus-bsc-can)
