EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L FreeRTOS_MQTT:PMS3003 U1
U 1 1 6045D9B7
P 4250 3350
F 0 "U1" H 4308 3965 50  0000 C CNN
F 1 "PMS3003" H 4308 3874 50  0000 C CNN
F 2 "" H 4700 3850 50  0001 C CNN
F 3 "" H 4700 3850 50  0001 C CNN
	1    4250 3350
	1    0    0    -1  
$EndComp
$Comp
L FreeRTOS_MQTT:ESP32-DEVKITC-32D U2
U 1 1 6045EEC4
P 6300 3900
F 0 "U2" H 6300 5067 50  0000 C CNN
F 1 "ESP32-DEVKITC-32D" H 6300 4976 50  0000 C CNN
F 2 "MODULE_ESP32-DEVKITC-32D" H 6300 3900 50  0001 L BNN
F 3 "" H 6300 3900 50  0001 L BNN
F 4 "Espressif Systems" H 6300 3900 50  0001 L BNN "MANUFACTURER"
F 5 "4" H 6300 3900 50  0001 L BNN "PARTREV"
	1    6300 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 3000 4650 3000
Wire Wire Line
	5500 4300 5400 4300
Wire Wire Line
	5400 4300 5400 3100
Wire Wire Line
	5400 3100 5100 3100
$Comp
L power:+5V #PWR?
U 1 1 604720A6
P 4650 3000
F 0 "#PWR?" H 4650 2850 50  0001 C CNN
F 1 "+5V" H 4665 3173 50  0000 C CNN
F 2 "" H 4650 3000 50  0001 C CNN
F 3 "" H 4650 3000 50  0001 C CNN
	1    4650 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3000 4800 3000
Wire Wire Line
	4800 3000 4800 3200
Wire Wire Line
	4550 3200 4800 3200
Wire Wire Line
	4550 3500 4800 3500
Wire Wire Line
	4800 3500 4800 3200
Connection ~ 4800 3200
Wire Wire Line
	7100 4000 7250 4000
Wire Wire Line
	7250 4000 7250 5050
Wire Wire Line
	7250 5050 5250 5050
Wire Wire Line
	5250 5050 5250 3400
Wire Wire Line
	5250 3400 4550 3400
$Comp
L power:GND #PWR?
U 1 1 604810DE
P 5100 3100
F 0 "#PWR?" H 5100 2850 50  0001 C CNN
F 1 "GND" H 5105 2927 50  0000 C CNN
F 2 "" H 5100 3100 50  0001 C CNN
F 3 "" H 5100 3100 50  0001 C CNN
	1    5100 3100
	1    0    0    -1  
$EndComp
Connection ~ 5100 3100
Wire Wire Line
	5100 3100 4550 3100
$EndSCHEMATC
