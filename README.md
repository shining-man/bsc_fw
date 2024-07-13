<H1 align="center">Battery safety controller (BSC)</H1>

<!-- SPONSORS_START -->
<p align="center">Herzlichen Dank an alle <b>Sponsoren</b>, die das Projekt unterstützen. Zum Sponsoring geht es <a href="https://github.com/sponsors/shining-man" target=_blank>hier</a> :smiley:</p>

<p align="center">
  <a href="https://bsc-shop.com" target="_blank"><img src="https://github.com/user-attachments/assets/2a044a67-b0f3-4285-8d8b-3e64ffd7539f" height="68"/></a>
</p>

<p align="center"><strong>Energie-Pionier</strong></p><p align="center">
  <a href="https://github.com/zippeliniot" target=_blank><img src="https://avatars.githubusercontent.com/u/75243184?v=4" height="58"/></a>
</p>

<p align="center"><strong>Solarfackel</strong></p><p align="center">
  <a href="https://github.com/SladeTheLeveller" target=_blank><img src="https://avatars.githubusercontent.com/u/127076644?v=4" height="58"/></a>
  <a href="https://github.com/stxShadow" target=_blank><img src="https://avatars.githubusercontent.com/u/48672214?v=4" height="58"/></a>
  <a href="https://github.com/a-wolter" target=_blank><img src="https://avatars.githubusercontent.com/u/13150781?v=4" height="58"/></a>
  <a href="https://github.com/tanelvakker" target=_blank><img src="https://avatars.githubusercontent.com/u/26738629?v=4" height="58"/></a>
  <a href="https://github.com/asdt1803" target=_blank><img src="https://avatars.githubusercontent.com/u/61160231?v=4" height="58"/></a>
</p>

<p align="center"><strong>Koffein-Komplize</strong></p><p align="center">
  <a href="https://github.com/D3R-ST3FAN" target=_blank><img src="https://avatars.githubusercontent.com/u/30196271?v=4" height="58"/></a>
  <a href="https://github.com/Ingog79" target=_blank><img src="https://avatars.githubusercontent.com/u/128917948?v=4" height="58"/></a>
  <a href="https://github.com/Rindi66" target=_blank><img src="https://avatars.githubusercontent.com/u/106440831?v=4" height="58"/></a>
  <a href="https://github.com/Ximerox" target=_blank><img src="https://avatars.githubusercontent.com/u/65176158?v=4" height="58"/></a>
  <a href="https://github.com/geolin" target=_blank><img src="https://avatars.githubusercontent.com/u/6317255?v=4" height="58"/></a>
  <a href="https://github.com/Ben33142" target=_blank><img src="https://avatars.githubusercontent.com/u/103312565?v=4" height="58"/></a>
  <a href="https://github.com/Diedaa" target=_blank><img src="https://avatars.githubusercontent.com/u/175444581?v=4" height="58"/></a>
</p>
<!-- SPONSORS_ENDE -->

<p align="center">
  <img src="https://github.com/shining-man/bsc_fw/blob/main/doc/README_Dashboard.png"/>
</p>

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
- Seplos, Seplos V3 [1]
- DALY
- Sylcin [1]
- Gobel [1]

[1]: With the Seplos and the Sylcin BMS, up to 8 devices can be connected to one serial port.

## Supported Inverter
- Victron
- Pylontech protocol (Solis, DEYE, ...)
- BYD protocol (only tested with SolarEdge RWS)

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

<img src="https://github.com/user-attachments/assets/8db86295-1d72-4389-9650-80bd504366ac" height="400"/>

# VenusOS dbus driver
To display more data in VenusOS (Victron) like the cell voltages of the connected BMS or the temperatures of the OneWire sensors, there is a dbus driver. Here the data is not transmitted via WLAN to the VenusOS, but via the CAN bus. <br>
For more info see the github project: [dbus-bsc-can](https://github.com/shining-man/dbus-bsc-can)
