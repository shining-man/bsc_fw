
# Battery safety controller (BSC)
![bsc_pcb_3d](https://github.com/shining-man/bsc_hw/blob/main/img/bsc_3d.png?raw=true)

The Battery Safety Controller (BSC) is a configurable controller for monitoring and protecting DIY (LiFePo4) storage devices. It should be noted that the BSC is not a BMS (battery management system). I.e. the BSC itself does not measure e.g. cell voltages. These must be made available to it via the serial interface or via Bluetooth from e.g. a BMS or a balancer. These parameters can then be monitored by the BSC.

The BSC also has an onewire interface to which up to 64 temperature sensors (DS18B20) can be connected. These can then also be monitored.

In addition to the monitoring function, the BSC can also control an inverter connected via the CAN bus. This can be used, for example, to regulate the charging current. Either for charge control or, for example, to control the charge current to 0 in the event of danger.

If one of the monitored parameters exceeds/falls below the set limits, then a set relay output can be switched. This can then be used, for example, by a circuit breaker to disconnect the battery. 

The BSC can also be used to send the data of the BMS/balancer to a MQTT broker.

All settings can be freely parameterized via a web interface.

**German**<br>
Der Battery Safety Controller (BSC) ist ein konfigurierbarer Controller zum Überwachen und Schützen von DIY (LiFePo4) Speichern. Zu beachten ist, dass der BSC kein BMS (Batteriemanagementsystem) ist. D.h. der BSC misst selber z.B. keine Zellspannungen. Diese müssen ihm über die serielle Schnittstelle oder über Bluetooth von z.B. einem BMS oder einem Balancer zur Verfügung gestellt werden. Diese Parameter können dann vom BSC überwacht werden.

Der BSC verfügt auch über eine onewire Schnittstellen an der bis zu 64 Temperatursensoren (DS18B20) angeschlossen werden können. Diese können dann auch überwacht werden.

Zusätzlich zu der Überwachungsfunktion kann der BSC auch einen über den CAN-Bus angeschlossenen Wechselrichter steuern. Hierüber kann z.B. der Ladestrom geregelt werden. Entweder zur Laderegelung oder um z.B. im Gefahrenfall den Ladestrom auf 0 zu regeln.

Wenn einer der überwachten Parameter die eingestellten Grenzen über-/unterschreitet, dann kann z.B. ein eingestellter Relaisausgang geschaltet werden. Über dieses kann dann z.B. ein Leistungsschalter die Batterie trennen. 

Der BSC kann aber auch genutzt werden um die Daten des BMS/Balancer an einen MQTT-Broker zu senden.

Alle Einstellungen sind frei über eine Weboberfläche parametrierbar.

**Features**
* 6x relay outputs
* 4x digital inputs (isolated)
* 3x serial interface (isolated); RS232, RS485
* Onewire (DS18B20)
* CAN (isolated)
* Bluetooth
* WLAN
* I2C for extensions

**Supported Devices**
* [Wiki - supported devices](https://github.com/shining-man/bsc_fw/wiki/Supported-devices)

**Weitere Informationen**
* [Wiki (German)](https://github.com/shining-man/bsc_fw/wiki)
* [Discussions](https://github.com/shining-man/bsc_fw/discussions)
* [PCB](https://github.com/shining-man/bsc_hw)
* [Stromlaufplan](https://github.com/shining-man/bsc_hw/blob/main/circuit.pdf?raw=true "Stromlaufplan")

## Statusdisplay
Um den Status des BSC und den verbundenen BMSen anzuzeigen gibt es für den BSC ein Statusdisplay.<br>
Weitere Details dazu sind im Github Projekt des Displays.<br>
[GitHub-Projekt Display](https://github.com/shining-man/bsc_display)<br>
<br>
![Picture_BSC_Display_Home](https://github.com/shining-man/bsc_display/blob/main/img/Disp_Home.jpg?raw=true)<br>
