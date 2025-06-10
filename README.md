<H1 align="center">Battery safety controller (BSC)</H1>
  
### Table of Contents
1. [More information](#more-information)
2. [Sponsors](#sponsors)
3. [Use Cases](#use-cases-of-the-bsc)
4. [Supported Devices](#supported-bms)
5. [Display](#status-display)

<a id="about-the-project"></a>
The Battery Safety Controller (BSC) is a freely configurable controller that can be used for a wide range of control and monitoring tasks for DIY batteries. The BSC can take over the central control of the Energy-Sorange system and can be used to implement a second safety level in addition to the BMS.

The BSC consists of two components. The BSC hardware and the corresponding software.

1. The [BSC Hardware](https://github.com/shining-man/bsc_hw), which is used as middleware between the BMS and the inverter. One advantage of the BSC hardware is its secure operation. The components and design are such that all physical interfaces are galvanically isolated.

2. The BSC software, which makes the Battery Safety Controller hardware a freely configurable controller that can be used for a wide range of control and monitoring tasks in DIY battery systems.  
   Battery Management Systems (BMS) are connected via the three available serial interfaces, while the inverter is integrated through the CAN bus.
   
An optional [status display](#status-display) is available as an extension, providing real-time information about the status of all connected devices.

The BSC software can also be used independently of the BSC hardware on ESP32 DevKit modules. However, due to stability, it is recommended to use the original BSC hardware.

The firmware is open source and available in German, English, and Dutch. You can download the latest public version [here](https://github.com/shining-man/bsc_fw/releases).  
In addition to the freely available version, there is an **Insider version** exclusively for sponsors. This version includes the newest features before they are gradually released to the public.  
You can learn more about sponsorship on the <a href="https://github.com/sponsors/shining-man" target=_blank>Github Sponsor page</a> and in the <a href="https://bsc-org.github.io/bsc/insider/" target=_blank>documentation</a>.  

The WebUI dashboard:
<span align="center">
  <img src="https://github.com/user-attachments/assets/3658c9a1-92b1-4fdf-947f-fa504d6a0532"/>
</span>

> The new dashboard is only available to [insiders](https://bsc-org.github.io/bsc/insider). All other users have a simple WebUI.

## More information
* [Discord Forum](https://discord.gg/WRUhegRPQn)  
The Discord is in German, but you can also write in English.
We are also happy to welcome members from other countries.
* [Documentation (German)](https://bsc-org.github.io/bsc/)  
  The documentation is in German, but it can be translated using your browser. Alternatively, you can view it via [Google Translate](https://bsc--org-github-io.translate.goog/bsc/?_x_tr_sl=de&_x_tr_tl=en&_x_tr_hl=de&_x_tr_pto=wapp).
    - [System overview](https://bsc-org.github.io/bsc/system_overview/)
    - [BMS connection examples](https://bsc-org.github.io/bsc/devices/bms/#anbindungs-beispiele)
    - Supported Devices: [BMS](https://bsc-org.github.io/bsc/devices/bms/), [Inverter](https://bsc-org.github.io/bsc/devices/wechselrichter/), [External Shunt](https://bsc-org.github.io/bsc/devices/externer_shunt/)
    - [First steps](https://bsc-org.github.io/bsc/first_steps/)
* [Discussions](https://github.com/shining-man/bsc_fw/discussions)
* [Layout BSC board](https://github.com/shining-man/bsc_hw), [Circuit diagram](https://github.com/shining-man/bsc_hw/blob/main/circuit.pdf?raw=true "Stromlaufplan")
* Sources of supply
  * [BSC Hardware (LILYGO)](https://de.aliexpress.com/item/1005007096164253.html)
  * [BSC Housings and accessories (BSC-Shop.com)](https://bsc-shop.com/)

## Sponsors
<p align="center">Many thanks to all the <b>sponsors</b> who support the project.<br>You can find out more about sponsoring on the <a href="https://github.com/sponsors/shining-man" target=_blank>Github Sponsor</a> page and in the <a href="https://bsc-org.github.io/bsc/insider/" target=_blank>documentation</a>.

<p align="center">
  <a href="https://bsc-shop.com" target="_blank"><img src="https://github.com/user-attachments/assets/2a044a67-b0f3-4285-8d8b-3e64ffd7539f" height="68"/></a>
</p>

<!-- SPONSORS_START -->
<p align="center"><strong>Gold</strong></p><p align="center">
<a href="https://github.com/zippeliniot" target=_blank><img src="https://avatars.githubusercontent.com/u/75243184?u=efd3a6a21adc165643f928b79e498191e90ebd43&v=4" height="58"/></a>
</p>
<p align="center"><strong>Bronze</strong></p><p align="center">
<a href="https://github.com/a-wolter" target=_blank><img src="https://avatars.githubusercontent.com/u/13150781?u=7e4d945122ebbe34a31bb917a7735c666b62040d&v=4" height="58"/></a>
<a href="https://github.com/SlapJackNpNp" target=_blank><img src="https://avatars.githubusercontent.com/u/26175386?v=4" height="58"/></a>
<a href="https://github.com/tanelvakker" target=_blank><img src="https://avatars.githubusercontent.com/u/26738629?u=ba2c63272337da5fa8a5eda7472995d0b15795d6&v=4" height="58"/></a>
<a href="https://github.com/D3R-ST3FAN" target=_blank><img src="https://avatars.githubusercontent.com/u/30196271?v=4" height="58"/></a>
<a href="https://github.com/stxShadow" target=_blank><img src="https://avatars.githubusercontent.com/u/48672214?u=3a9f9e495c4bc28faf9f7486348216a592550433&v=4" height="58"/></a>
<a href="https://github.com/steirerman" target=_blank><img src="https://avatars.githubusercontent.com/u/49448230?u=1cfb45fd834c8cd939d3ac4d3fb3c07ffb9c2a8c&v=4" height="58"/></a>
<a href="https://github.com/myhacsint" target=_blank><img src="https://avatars.githubusercontent.com/u/111045655?u=908af7bf6d8071bcb5707437983329b694732577&v=4" height="58"/></a>
<a href="https://github.com/SladeTheLeveller" target=_blank><img src="https://avatars.githubusercontent.com/u/127076644?u=96cfedf0bba5aa3e61fc6741d73fceba56563eb6&v=4" height="58"/></a>
<a href="https://github.com/Metaworld82" target=_blank><img src="https://avatars.githubusercontent.com/u/156550945?u=0f06f37bcaa23d70651a913adb6d277d34eaeaaa&v=4" height="58"/></a>
<a href="https://github.com/golfer-fossy" target=_blank><img src="https://avatars.githubusercontent.com/u/175492267?u=d029b983a51288f7038ef3460a11aba6522c5d12&v=4" height="58"/></a>
</p>
<p align="center"><strong>Supporter</strong></p><p align="center">
<a href="https://github.com/stlorenz" target=_blank><img src="https://avatars.githubusercontent.com/u/5159308?u=cebe9493a817115c63c2c23a00fdd0f5f256b225&v=4" height="58"/></a>
<a href="https://github.com/Emilleopold" target=_blank><img src="https://avatars.githubusercontent.com/u/6137560?v=4" height="58"/></a>
<a href="https://github.com/BjoernKellermann" target=_blank><img src="https://avatars.githubusercontent.com/u/6324993?u=3f31c6576e32d16fa2667da502677d0e4936e67a&v=4" height="58"/></a>
<a href="https://github.com/Timon321" target=_blank><img src="https://avatars.githubusercontent.com/u/7792233?u=5830023fdc7fd4e21aba8246d70f2255d2e50a4b&v=4" height="58"/></a>
<a href="https://github.com/PatrickGoettsch" target=_blank><img src="https://avatars.githubusercontent.com/u/8688158?u=5a0d7247dfdb3cafd36a49e41a7e68f862e32dd0&v=4" height="58"/></a>
<a href="https://github.com/HJoost" target=_blank><img src="https://avatars.githubusercontent.com/u/13537894?v=4" height="58"/></a>
<a href="https://github.com/mmkeule" target=_blank><img src="https://avatars.githubusercontent.com/u/13866729?v=4" height="58"/></a>
<a href="https://github.com/jgatringer" target=_blank><img src="https://avatars.githubusercontent.com/u/14805417?u=656f266a33aac2bf74030d8e0b46271dc219e01e&v=4" height="58"/></a>
<a href="https://github.com/vo5tr0" target=_blank><img src="https://avatars.githubusercontent.com/u/14971935?v=4" height="58"/></a>
<a href="https://github.com/JoergWie" target=_blank><img src="https://avatars.githubusercontent.com/u/20060594?u=e2b4660646488d6b18034efcef9f3dbfc5dd5dac&v=4" height="58"/></a>
<a href="https://github.com/Saharel001" target=_blank><img src="https://avatars.githubusercontent.com/u/24453343?v=4" height="58"/></a>
<a href="https://github.com/Vitalic66" target=_blank><img src="https://avatars.githubusercontent.com/u/26961054?u=a497667f7c97db8684b860299e3223fb0070c919&v=4" height="58"/></a>
<a href="https://github.com/mhproit" target=_blank><img src="https://avatars.githubusercontent.com/u/29717578?v=4" height="58"/></a>
<a href="https://github.com/dl6dbh" target=_blank><img src="https://avatars.githubusercontent.com/u/35333262?v=4" height="58"/></a>
<a href="https://github.com/oalbers58" target=_blank><img src="https://avatars.githubusercontent.com/u/45533193?v=4" height="58"/></a>
<a href="https://github.com/GadingeL" target=_blank><img src="https://avatars.githubusercontent.com/u/46995271?v=4" height="58"/></a>
<a href="https://github.com/michi1984" target=_blank><img src="https://avatars.githubusercontent.com/u/58638770?v=4" height="58"/></a>
<a href="https://github.com/pepe-el-pepe" target=_blank><img src="https://avatars.githubusercontent.com/u/61097309?u=217ffbae91308f3675269d8bae5ab11ddda7bfa7&v=4" height="58"/></a>
<a href="https://github.com/Ximerox" target=_blank><img src="https://avatars.githubusercontent.com/u/65176158?u=d1952cbd2fa94be247696e69ddd230cd9e950731&v=4" height="58"/></a>
<a href="https://github.com/martinarva" target=_blank><img src="https://avatars.githubusercontent.com/u/77044330?v=4" height="58"/></a>
<a href="https://github.com/2SK135" target=_blank><img src="https://avatars.githubusercontent.com/u/82417004?v=4" height="58"/></a>
<a href="https://github.com/StefanM60" target=_blank><img src="https://avatars.githubusercontent.com/u/91480362?v=4" height="58"/></a>
<a href="https://github.com/HSGithi" target=_blank><img src="https://avatars.githubusercontent.com/u/95976129?v=4" height="58"/></a>
<a href="https://github.com/Hitman79" target=_blank><img src="https://avatars.githubusercontent.com/u/101149246?u=ce5e049664fd4234c017239ce3ac5d1e0e7c8309&v=4" height="58"/></a>
<a href="https://github.com/SHendrik123" target=_blank><img src="https://avatars.githubusercontent.com/u/103681547?v=4" height="58"/></a>
<a href="https://github.com/ThomasPfeifer69" target=_blank><img src="https://avatars.githubusercontent.com/u/108874804?u=12a660b472f2d40620fa64074020274950abe298&v=4" height="58"/></a>
<a href="https://github.com/gartenjoe" target=_blank><img src="https://avatars.githubusercontent.com/u/113258562?u=9a17d545201068adae93ae632985056122975fad&v=4" height="58"/></a>
<a href="https://github.com/KrawallOPA" target=_blank><img src="https://avatars.githubusercontent.com/u/115902005?v=4" height="58"/></a>
<a href="https://github.com/motorradfeger" target=_blank><img src="https://avatars.githubusercontent.com/u/118117416?v=4" height="58"/></a>
<a href="https://github.com/papala24" target=_blank><img src="https://avatars.githubusercontent.com/u/127226933?v=4" height="58"/></a>
<a href="https://github.com/guntec1" target=_blank><img src="https://avatars.githubusercontent.com/u/129986859?v=4" height="58"/></a>
<a href="https://github.com/Lederweisz" target=_blank><img src="https://avatars.githubusercontent.com/u/149304049?v=4" height="58"/></a>
<a href="https://github.com/toptec01" target=_blank><img src="https://avatars.githubusercontent.com/u/156579129?v=4" height="58"/></a>
<a href="https://github.com/michaelkohrs1998" target=_blank><img src="https://avatars.githubusercontent.com/u/156584099?v=4" height="58"/></a>
<a href="https://github.com/DerAmbergauer" target=_blank><img src="https://avatars.githubusercontent.com/u/163426788?u=daafdf199950a86e168a9320c8f329b2049ddea9&v=4" height="58"/></a>
<a href="https://github.com/Diedaa" target=_blank><img src="https://avatars.githubusercontent.com/u/175444581?u=72327f606d0cb2d7de9341b208a492090e0a0272&v=4" height="58"/></a>
<a href="https://github.com/mmoe-milano" target=_blank><img src="https://avatars.githubusercontent.com/u/177930080?v=4" height="58"/></a>
<a href="https://github.com/Knofn" target=_blank><img src="https://avatars.githubusercontent.com/u/191380062?v=4" height="58"/></a>
<a href="https://github.com/erwinh-at" target=_blank><img src="https://avatars.githubusercontent.com/u/195420064?v=4" height="58"/></a>
</p>
<!-- SPONSORS_ENDE -->

## Use cases of the BSC
### 1. Monitoring of connected devices (serial, Bluetooth)
For this purpose, the BSC can send the data from the connected devices (BMS, balancer, temperature sensors) via MQTT to a broker to display them graphically with Grafana, for example, or to further process the data from an automation system (ioBroker, NodRed, Home Assistant, ...).

### 2. Charge control
For this, the BSC takes the data from the connected devices to control the inverter (Victron, Solis, DEYE, ...) connected via the CAN bus. Here are several functions available to adapt the charge control to your own DIY battery.
- Cell voltage dependent throttling of the charge current
- Reduce charge current in case of cell drift
- SoC dependent charge current reduction
- Charge current cut-off: prevents continuous recharging when the battery is full
- Combining data from the individual physical battery packs into an overall virtual battery pack, with consideration of many parameters, such as can/does the battery pack charge/discharge at all.

### 3. Second safety level besides the BMS
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

All settings can be freely parameterized via a web interface.

## Supported BMS
For more information see the documentation in the section [Supported BMS](https://bsc-org.github.io/bsc/devices/bms/).
- JBD (Jiabaida)
- JK
- JK Inverter [1]
- Seplos
- Seplos V3 [1]
- DALY
- Sylcin [1]
- PACE RN150
- PCAE PC200 [1]

[1]: With these BMS you can connect up to 16 to one serial port.

## Supported external Shunts
For more information see the documentation in the section [Supported external Shunts](https://bsc-org.github.io/bsc/devices/externer_shunt/).
- Victron SmartShunt

## Supported Inverter
For more information see the documentation in the section [Supported Inverter](https://bsc-org.github.io/bsc/devices/wechselrichter/).
- Victron
- Pylontech protocol (Solis, DEYE, ...)
- BYD protocol (only tested with SolarEdge RWS)

## Features der BSC-Hardware
* 6x Relay outputs
* 4x Digital inputs (isolated)
* 3x Serial interface (isolated) [3]
* Onewire (64x DS18B20)
* CAN (isolated)
* WLAN

[3]: With the [Serial Extension](https://github.com/shining-man/bsc_extension_serial) the BSC can be extended by 8 additional physical and galvanically isolated serial ports.

## First steps
A lot of further information about commissioning, supported devices, information about the WebUI can be found in the [Documentation](https://bsc-org.github.io/bsc/first_steps/).

### Use with an ESP32 DevKit
### GPIOs
Interface | RX | TX
-------- | -------- | --------
**Serial 0**   | GPIO 16   | GPIO 17
**Serial 1**   | GPIO 23   | GPIO 25
**Serial 2**   | GPIO 35   | GPIO 33
**CAN**   | GPIO 5   | GPIO 4

Further description can be found [here](https://bsc-org.github.io/bsc/BSC_ohne_orig_hardware/).

## Status display
To show the status of the BSC and the connected BMSes there is a status display for the BSC.<br>
More details are in the Github project of the display and in the documenatation.  
[GitHub project Display](https://github.com/shining-man/bsc_display)  
[Documentation - Display](https://bsc-org.github.io/bsc/hardware/#anschluss-an-das-bsc-mainboard)

<img src="https://github.com/user-attachments/assets/8db86295-1d72-4389-9650-80bd504366ac" height="400"/>

## VenusOS dbus driver
To display more data in VenusOS (Victron) like the cell voltages of the connected BMS or the temperatures of the OneWire sensors, there is a dbus driver. Here the data is not transmitted via WLAN to the VenusOS, but via the CAN bus. <br>
For more info see the github project: [dbus-bsc-can](https://github.com/shining-man/dbus-bsc-can)
