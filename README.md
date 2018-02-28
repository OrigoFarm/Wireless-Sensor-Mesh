# Wireless-Sensor-Mesh


## Proof of Concept Hardware Design

hardware folder contains the wiring diagram of the electronics that was built for node stations.


## Node Server

Installation:

```
easy_install pip (to easy install python modules)

pip install influxdb (influxdb module for python)
pip install pyserial

```

run the script by
```
python mesh-server.py
```



Install [InfluxDB](https://docs.influxdata.com/influxdb/v1.4/introduction/installation/)

Install [Grafana](http://docs.grafana.org/installation/debian/)

## Node Firmware

This contain a sketch to be compiled and uploaded to the Arduino Pro Mini.

The firmware will communicate to the Meshbee module via serial. Meshbee needs to be in API Mode before uploading this sketch in order for the firmware to work.
In order to do this, there are two ways:

1. Connect the Meshbee separately to a PC using a TTL to USB cable.
2. Upload the passthrough sketch to send commands directly to the Meshbee via the Arduino.

Send the following commands to the Meshbee

```
+++ //AT mode
ATAJ1 //Auto join
ATRS //Search network

ATAP //API mode
```

Make sure Meshbee is updated to firmware v1004
Please see http://wiki.seeedstudio.com/wiki/Zig_Bee for more information.