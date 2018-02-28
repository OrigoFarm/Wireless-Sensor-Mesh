#!/usr/bin/env python
#
#   Created by Andre Rumantir, Origo, 2018
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#   
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#   
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
"""
This module was created to insert measurements into InfluxDB 
from wireless nodes consisting of rainfall sensor and temperature and/or humidity sensor.

The script listens to a serial interface, where the receiver is connected on the server,
every 5 seconds for incoming CSV formatted messages.

The incoming messages are formatted below
MAC_ADDR,TEMPERATURE,RAINFALL,HUMIDITIY

Taking each value, store in an InfluxDB object structure and sending the object to a running InfluxDB server.

Please see InfluxDB(https://docs.influxdata.com/influxdb/) for more information.
"""

import serial, time
from influxdb import InfluxDBClient

COM = "/dev/tty.usbserial-DN2B80I8"
BAUD_RATE = 115200
DB_NAME = "agtech"

ser = serial.Serial(COM,BAUD_RATE,timeout=1)

client = InfluxDBClient(host='127.0.0.1', database=DB_NAME, port=8086)

while True:
    if (ser.in_waiting != 0):
        measurement = ser.readline().strip()
        list = measurement.split(",")

        data = [{}]
        data[0]["measurement"] = "weather"
        data[0]["tags"] = { "id": list[0]}
        data[0]["fields"] = {
            "temp" : float(list[1]),
            "rainfall" : int(list[2]),
            "humid" : float(list[3])
        }

        client.write_points(data)

        print data
    else:
        time.sleep(5)