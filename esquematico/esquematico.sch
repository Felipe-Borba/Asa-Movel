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
L MCU_Module:Maple_Mini A1
U 1 1 5FE82585
P 6850 2200
F 0 "A1" H 6850 2300 50  0000 C CNN
F 1 "Maple_Mini" H 6850 2200 50  0000 C CNN
F 2 "Module:Maple_Mini" H 6900 1150 50  0001 L CNN
F 3 "http://docs.leaflabs.com/static.leaflabs.com/pub/leaflabs/maple-docs/0.0.12/hardware/maple-mini.html" H 6900 200 50  0001 L CNN
	1    6850 2200
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Switching:LM2596S-5 U1
U 1 1 5FE84FB9
P 2150 1300
F 0 "U1" H 2150 1450 50  0000 C CNN
F 1 "LM2596S-5" H 2150 1350 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-5_TabPin3" H 2200 1050 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2596.pdf" H 2150 1300 50  0001 C CNN
	1    2150 1300
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Motion:MPU-6050 U2
U 1 1 5FE86F8E
P 4500 1600
F 0 "U2" H 4400 1500 50  0000 C CNN
F 1 "MPU-6050" H 4400 1600 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm" H 4500 800 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf" H 4500 1450 50  0001 C CNN
	1    4500 1600
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x09 J1
U 1 1 5FE8A892
P 1550 2800
F 0 "J1" H 1630 2842 50  0000 L CNN
F 1 "Conn_01x09" H 1630 2751 50  0000 L CNN
F 2 "" H 1550 2800 50  0001 C CNN
F 3 "~" H 1550 2800 50  0001 C CNN
	1    1550 2800
	-1   0    0    1   
$EndComp
Text GLabel 1750 3200 2    50   Input ~ 0
12V
Text GLabel 1750 3100 2    50   Input ~ 0
GND
Text GLabel 2150 1600 3    50   Input ~ 0
GND
Text GLabel 1550 1200 0    50   Input ~ 0
12V
Wire Wire Line
	1550 1200 1650 1200
Text GLabel 1650 1400 0    50   Input ~ 0
GND
Text GLabel 2650 1400 2    50   Input ~ 0
5V
Text GLabel 4500 800  1    50   Input ~ 0
5V
Wire Wire Line
	4600 900  4500 900 
Wire Wire Line
	4500 900  4500 800 
Wire Wire Line
	4400 900  4500 900 
Connection ~ 4500 900 
Text GLabel 4500 2300 3    50   Input ~ 0
GND
Text GLabel 6800 950  1    50   Input ~ 0
5V
Wire Wire Line
	6750 1100 6800 1100
Wire Wire Line
	6800 1100 6800 950 
Wire Wire Line
	6850 1100 6800 1100
Connection ~ 6800 1100
Text GLabel 6850 3300 3    50   Input ~ 0
GND
Text GLabel 3800 1300 0    50   Input ~ 0
SDA
Text GLabel 7650 1400 2    50   Input ~ 0
SDA
Text GLabel 3800 1400 0    50   Input ~ 0
SCL
Text GLabel 7650 1300 2    50   Input ~ 0
SCL
Text GLabel 6050 2400 0    50   Input ~ 0
POTENCIOMETRO1
Text GLabel 6050 2300 0    50   Input ~ 0
POTENCIOMETRO2
Text GLabel 7650 1700 2    50   Input ~ 0
Manual_Automatico
Text GLabel 7650 1600 2    50   Input ~ 0
Botao_Asa
Text GLabel 7650 1800 2    50   Input ~ 0
Servo1
Text GLabel 7650 2500 2    50   Input ~ 0
Servo2
$EndSCHEMATC
