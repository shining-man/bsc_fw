# Battery safety controller (BSC)


![bsc_pcb_3d](https://github.com/shining-man/bsc_hw/blob/main/img/bsc_3d.png?raw=true)

Der Battery Safety Controller (BSC) ist ein konfigurierbarer Controller zum Überwachen und Schützen von DIY (LiFePo4) Speichern. Zu beachten ist, dass der BSC kein BMS (Batteriemanagementsystem) ist. D.h. der BSC misst selber z.B. keine Zellspannungen. Diese müssen ihm über die serielle Schnittstelle oder über Bluetooth von z.B. einem BMS oder einem Balancer zur Verfügung gestellt werden. Diese Parameter können dann vom BSC überwacht werden.

Der BSC verfügt auch über eine onewire Schnittstellen an der bis zu 64 Temperatursensoren (DS18B20) angeschlossen werden können. Diese können dann auch überwacht werden.

Zusätzlich zu der Überwachungsfunktion kann der BSC auch einen über den CAN-Bus angeschlossenen Wechselrichter steuern. Hierüber kann z.B. der Ladestrom geregelt werden. Entweder zur Laderegelung oder um z.B. im Gefahrenfall den Ladestrom auf 0 zu regeln.

Wenn einer der überwachten Parameter die eingestellten Grenzen über-/unterschreitet, dann kann z.B. ein eingestellter Relaisausgang geschaltet werden. Über dieses kann dann z.B. ein Leistungsschalter die Batterie trennen. 

Der BSC kann aber auch nur genutzt werden um die Daten des BMS/Balancer an einen MQTT-Broker zu senden.

Alle Einstellungen sind frei über eine Weboberfläche parametrierbar.


Features:
* 6x Relaisausgänge
* 1x Opto-Ausgang
* 4x Digitaleingänge 
* 3x Serielle Schnittstelle (RS232, RS485)
* Onewire (DS18B20)
* CAN 
* Bluetooth
* WLAN

Unterstütze Hardware:
* Bluetooth: NEEY Balancer 4A
* Serielle Schnittstelle (RS232, RS485): JBD BMS

[Stromlaufplan](https://github.com/shining-man/bsc_hw/blob/main/circuit.pdf?raw=true "Stromlaufplan")

Weitere Informationen: siehe [Wiki](https://github.com/shining-man/bsc_fw/wiki)

