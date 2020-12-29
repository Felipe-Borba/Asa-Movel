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
L Connector_Generic:Conn_01x09 J1
U 1 1 5FE8A892
P 1800 2550
F 0 "J1" H 1880 2592 50  0000 L CNN
F 1 "Conn_01x09" H 1880 2501 50  0000 L CNN
F 2 "" H 1800 2550 50  0001 C CNN
F 3 "~" H 1800 2550 50  0001 C CNN
	1    1800 2550
	-1   0    0    1   
$EndComp
Text GLabel 2000 2950 2    50   Input ~ 0
12V
Text GLabel 2000 2850 2    50   Input ~ 0
GND
Text GLabel 3100 1450 2    50   Input ~ 0
GND
Text GLabel 800  1150 0    50   Input ~ 0
12V
Text GLabel 1100 1450 0    50   Input ~ 0
GND
Text GLabel 3100 1150 2    50   Input ~ 0
5V
Text GLabel 3600 1700 3    50   Input ~ 0
5V
Text GLabel 3700 1700 3    50   Input ~ 0
GND
Text GLabel 4950 3000 0    50   Input ~ 0
5V
Text GLabel 4950 3100 0    50   Input ~ 0
GND
Text GLabel 3900 1700 3    50   Input ~ 0
SDA
Text GLabel 4950 2700 0    50   Input ~ 0
SDA
Text GLabel 3800 1700 3    50   Input ~ 0
SCL
Text GLabel 4950 2600 0    50   Input ~ 0
SCL
Text GLabel 5850 2800 2    50   Input ~ 0
POTENCIOMETRO1
Text GLabel 5850 2700 2    50   Input ~ 0
POTENCIOMETRO2
Text GLabel 4950 2300 0    50   Input ~ 0
Manual_Automatico
Text GLabel 4950 2400 0    50   Input ~ 0
Botao_Asa
Text GLabel 4950 2200 0    50   Input ~ 0
Servo1
Text GLabel 4950 1700 0    50   Input ~ 0
Servo2
$Comp
L Custom_Library:LM2596-DC-DC-Step-Down-Module U1
U 1 1 5FE809A2
P 2100 1300
F 0 "U1" H 2100 1400 50  0000 C CNN
F 1 "LM2596-DC-DC-Step-Down-Module" H 2100 1300 50  0000 C CNN
F 2 "" H 2100 1300 50  0001 C CNN
F 3 "" H 2100 1300 50  0001 C CNN
	1    2100 1300
	1    0    0    -1  
$EndComp
$Comp
L Custom_Library:MPU6050-module U2
U 1 1 5FE812F1
P 3950 1250
F 0 "U2" H 3900 1350 50  0000 L CNN
F 1 "MPU6050-module" H 3600 1250 50  0000 L CNN
F 2 "" H 3950 1250 50  0001 C CNN
F 3 "" H 3950 1250 50  0001 C CNN
	1    3950 1250
	1    0    0    -1  
$EndComp
$Comp
L Custom_Library:STM32_Blue_Pill U3
U 1 1 5FE81C0E
P 5400 1200
F 0 "U3" H 5400 1300 50  0000 C CNN
F 1 "STM32_Blue_Pill" H 5400 1200 50  0000 C CNN
F 2 "" H 5400 1200 50  0001 C CNN
F 3 "" H 5400 1200 50  0001 C CNN
	1    5400 1200
	1    0    0    -1  
$EndComp
Text GLabel 4950 3200 0    50   Input ~ 0
3V3
Text GLabel 2000 2150 2    50   Input ~ 0
Servo1
Text GLabel 2000 2250 2    50   Input ~ 0
Servo2
Text GLabel 2000 2350 2    50   Input ~ 0
Botao_Asa
Text GLabel 2000 2450 2    50   Input ~ 0
POTENCIOMETRO1
Text GLabel 2000 2550 2    50   Input ~ 0
Manual_Automatico
Text GLabel 2000 2650 2    50   Input ~ 0
5V
Text GLabel 2000 2750 2    50   Input ~ 0
3V3
$Comp
L Diode:1N4007 D1
U 1 1 5FEA8DF5
P 950 1150
F 0 "D1" H 950 933 50  0000 C CNN
F 1 "1N4007" H 950 1024 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 950 975 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 950 1150 50  0001 C CNN
	1    950  1150
	-1   0    0    1   
$EndComp
NoConn ~ 4950 1300
NoConn ~ 4950 1400
NoConn ~ 4950 1500
NoConn ~ 4950 1600
NoConn ~ 4950 1800
NoConn ~ 4950 1900
NoConn ~ 4950 2000
NoConn ~ 4950 2100
NoConn ~ 4950 2500
NoConn ~ 4950 2800
NoConn ~ 4950 2900
NoConn ~ 5250 3550
NoConn ~ 5350 3550
NoConn ~ 5450 3550
NoConn ~ 5550 3550
NoConn ~ 5850 3200
NoConn ~ 5850 3100
NoConn ~ 5850 3000
NoConn ~ 5850 2900
NoConn ~ 5850 2600
NoConn ~ 5850 2500
NoConn ~ 5850 2400
NoConn ~ 5850 2300
NoConn ~ 5850 2200
NoConn ~ 5850 2100
NoConn ~ 5850 2000
NoConn ~ 5850 1900
NoConn ~ 5850 1800
NoConn ~ 5850 1700
NoConn ~ 5850 1600
NoConn ~ 5850 1500
NoConn ~ 5850 1400
NoConn ~ 5850 1300
NoConn ~ 4300 1700
NoConn ~ 4200 1700
NoConn ~ 4100 1700
NoConn ~ 4000 1700
$EndSCHEMATC
