Connect an Amazon FreeRTOS device to Robonomics by MQTT
=======================================================

Here we demonstrate how a microcontroller running [Amazon's FreeRTOS](https://aws.amazon.com/freertos/) may connect to Robonomics Network by MQTT.
We use [ESP32 DevKitC](https://devices.amazonaws.com/detail/a3G0L00000AANtjUAH/ESP32-WROOM-32-DevKitC/) with FreeRTOS distribution and MQTT implementation provided by [Espressif IoT Development Framework](https://github.com/espressif/esp-idf) while Espressif is a vendor of the microcontroller used.
Also there is a [PMS-3003]() sensor for demonstration purposes.
Sensor measures presence of particulated matter in the air and one may use it to estimate air quality.
Air quality is not a topic of the article, you may find more about it at WHO's website: [Ambient (outdoor) air pollution](https://www.who.int/news-room/fact-sheets/detail/ambient-(outdoor)-air-quality-and-health).
A goal of the system is to publish sensor measurements in Airalab's Robonomics network.

Hardware setup
--------------
We connect PMS3003 TXD PIN5 to ESP32 DevKitC IO17 to transfer measurements by UART. Also both devices require power and common ground.
TODO: add wiring images/wiring.png

Data flow
---------
TODO: add diagram images/send.svg
In order to deliver sensor measurements to Robonomics network, on a firmware level our goal is to get data from a sensor by embedded communication protocol it supports (UART in our case) and pass it to AIRA instance by MQTT / TCP.
In our example we use AIRA cloud deployment available by public IP address and domain name assigned.
On AIRA instance we receive data to Mosquitto MQTT broker.
Once data received into a topic dedicated for our sensor, a MQTT client on the same AIRA instance gets this data and pass it by pipeline to Robonomics IO.
It publishes data in Robonomics Network by IPFS.
TODO: add diagram images/recv.svg
Now data available in Robonomics Network and we can read it with Robonomics IO utility.

Firmware
--------
We use [ESP-MQTT sample application with TCP transport](https://github.com/espressif/esp-idf/tree/master/examples/protocols/mqtt/tcp) as a basis.
We only modify `main/app_main.c` UART connection to the sensor and periodic MQTT publisher routine.

### Wi-Fi Configuration
In order to communicate with AIRA instance deployed in cloud, our microcontroller requires Internet connection.
We use ESP32's Wi-Fi for it.
Espressif provides utilities to configure on-board Wi-Fi.
You may find more on how to configure ESP32 development environment on your system in [Espressif's ESP-IDF Programming guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#installation-step-by-step).
In our example we use Ubuntu 20.04 GNU/Linux.
To configure Wi-Fi we go to project folder and run SDK configuration tool.
```console
cd freertos_mqtt_robonomics_example/firmware
idf.py menuconfig
```
Then we set Wi-Fi access point SSID and password in `Example Connection Configuration` section.
TODO: menuconfig screenshot

### MQTT endpoint configuration
There are two things to configure for MQTT.
First is MQTT broker address.
It is configurable with SDK configuration tool.
```console
cd freertos_mqtt_robonomics_example/firmware
idf.py menuconfig
```
Set `Broker URL` in `Example Configuration` section.

We also add NTP configuration in order to supply sensor measurements with timestamp.

Original resources used
-----------------------
* ESP32 DevKitC pinout from GoJimmy's blog https://gojimmypi.blogspot.com/2017/03/jtag-debugging-for-esp32.html
* PSM3003 data structure and decoder from OpenAirProject https://github.com/openairproject/sensor-esp32
Thank you all!
