EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
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
L RPi_Hat-rescue:RPi_GPIO J2
U 1 1 5516AE26
P 5150 2750
AR Path="/5516AE26" Ref="J2"  Part="1" 
AR Path="/5515D395/5516AE26" Ref="J2"  Part="1" 
F 0 "J2" H 5900 3000 60  0000 C CNN
F 1 "RPi_GPIO" H 5900 2900 60  0000 C CNN
F 2 "cidco:Pin_Header_Straight_2x20" H 5150 2750 60  0001 C CNN
F 3 "" H 5150 2750 60  0000 C CNN
	1    5150 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x07_Odd_Even J8
U 1 1 5E30C6E7
P 5750 1350
F 0 "J8" H 5800 1867 50  0000 C CNN
F 1 "Conn_02x07_Odd_Even" H 5800 1776 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x07_P2.54mm_Vertical_Lock" H 5750 1350 50  0001 C CNN
F 3 "~" H 5750 1350 50  0001 C CNN
	1    5750 1350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J1
U 1 1 5E30E896
P 1150 1150
F 0 "J1" H 1068 725 50  0000 C CNN
F 1 "Conn_01x05" H 1068 816 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B5B-EH-A_1x05_P2.50mm_Vertical" H 1150 1150 50  0001 C CNN
F 3 "~" H 1150 1150 50  0001 C CNN
	1    1150 1150
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J3
U 1 1 5E30F076
P 1150 1900
F 0 "J3" H 1068 1475 50  0000 C CNN
F 1 "Conn_01x05" H 1068 1566 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B5B-EH-A_1x05_P2.50mm_Vertical" H 1150 1900 50  0001 C CNN
F 3 "~" H 1150 1900 50  0001 C CNN
	1    1150 1900
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J4
U 1 1 5E30F88C
P 1150 2650
F 0 "J4" H 1068 2225 50  0000 C CNN
F 1 "Conn_01x05" H 1068 2316 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B5B-EH-A_1x05_P2.50mm_Vertical" H 1150 2650 50  0001 C CNN
F 3 "~" H 1150 2650 50  0001 C CNN
	1    1150 2650
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x07 J6
U 1 1 5E310AE4
P 1150 4450
F 0 "J6" H 1068 3925 50  0000 C CNN
F 1 "Conn_01x07" H 1068 4016 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B7B-EH-A_1x07_P2.50mm_Vertical" H 1150 4450 50  0001 C CNN
F 3 "~" H 1150 4450 50  0001 C CNN
	1    1150 4450
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J7
U 1 1 5E310EEB
P 1150 5300
F 0 "J7" H 1068 4875 50  0000 C CNN
F 1 "Conn_01x05" H 1068 4966 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B5B-EH-A_1x05_P2.50mm_Vertical" H 1150 5300 50  0001 C CNN
F 3 "~" H 1150 5300 50  0001 C CNN
	1    1150 5300
	-1   0    0    1   
$EndComp
Text GLabel 4950 2850 0    50   Input ~ 0
SDA
Text GLabel 4950 2950 0    50   Input ~ 0
SCL
Text GLabel 6850 3050 2    50   Input ~ 0
TX
Text GLabel 6850 3150 2    50   Input ~ 0
RX
Text GLabel 6850 3850 2    50   Input ~ 0
C0
Text GLabel 6850 3950 2    50   Input ~ 0
C1
Text GLabel 4950 3650 0    50   Input ~ 0
MOSI
Text GLabel 4950 3750 0    50   Input ~ 0
MISO
Text GLabel 4950 3850 0    50   Input ~ 0
SCLK
Text GLabel 4950 3050 0    50   Input ~ 0
PPS
Text GLabel 4950 3250 0    50   Input ~ 0
LEDR
Text GLabel 4950 3450 0    50   Input ~ 0
LEDB
Text GLabel 4950 3350 0    50   Input ~ 0
LEDG
Text GLabel 4950 2750 0    50   Input ~ 0
3.3V
Text GLabel 4950 3550 0    50   Input ~ 0
3.3V
Text GLabel 4950 3150 0    50   Input ~ 0
GND
Text GLabel 4950 3950 0    50   Input ~ 0
GND
Text GLabel 4950 4650 0    50   Input ~ 0
GND
Text GLabel 6850 2750 2    50   Input ~ 0
5V
Text GLabel 6850 2850 2    50   Input ~ 0
5V
Text GLabel 6850 2950 2    50   Input ~ 0
GND
Text GLabel 6850 3350 2    50   Input ~ 0
GND
Text GLabel 6850 3650 2    50   Input ~ 0
GND
Text GLabel 6850 4150 2    50   Input ~ 0
GND
Text GLabel 6850 4350 2    50   Input ~ 0
GND
Text GLabel 5550 1050 0    50   Input ~ 0
5V
Text GLabel 5550 1150 0    50   Input ~ 0
5V
Text GLabel 6050 1050 2    50   Input ~ 0
5V
Text GLabel 6050 1150 2    50   Input ~ 0
5V
Text GLabel 6050 1650 2    50   Input ~ 0
GND
Text GLabel 6050 1550 2    50   Input ~ 0
GND
Text GLabel 6050 1450 2    50   Input ~ 0
GND
Text GLabel 6050 1350 2    50   Input ~ 0
GND
Text GLabel 6050 1250 2    50   Input ~ 0
12V_ON
Text GLabel 5550 1250 0    50   Input ~ 0
24V_ON
Text GLabel 5550 1350 0    50   Input ~ 0
RPI_PWR_OK
Text GLabel 5550 1450 0    50   Input ~ 0
RPI_SHDWN
Text GLabel 5550 1550 0    50   Input ~ 0
SDA
Text GLabel 5550 1650 0    50   Input ~ 0
SCL
Text GLabel 4950 4450 0    50   Input ~ 0
12V_ON
Text GLabel 4950 4350 0    50   Input ~ 0
24V_ON
Text GLabel 4950 4250 0    50   Input ~ 0
RPI_PWR_OK
Text GLabel 4950 4150 0    50   Input ~ 0
RPI_SHDWN
Text GLabel 1350 950  2    50   Input ~ 0
3.3V
Text GLabel 1350 1700 2    50   Input ~ 0
3.3V
Text GLabel 1350 2450 2    50   Input ~ 0
3.3V
Text GLabel 1350 3200 2    50   Input ~ 0
3.3V
Text GLabel 1350 4150 2    50   Input ~ 0
3.3V
Text GLabel 1350 5100 2    50   Input ~ 0
3.3V
Text GLabel 1350 5500 2    50   Input ~ 0
GND
Text GLabel 1350 4750 2    50   Input ~ 0
GND
Text GLabel 1350 3800 2    50   Input ~ 0
GND
Text GLabel 1350 2850 2    50   Input ~ 0
GND
Text GLabel 1350 2100 2    50   Input ~ 0
GND
Text GLabel 1350 1350 2    50   Input ~ 0
GND
Text GLabel 1350 1250 2    50   Input ~ 0
PPS
Text GLabel 1350 2000 2    50   Input ~ 0
PPS
Text GLabel 1350 2750 2    50   Input ~ 0
PPS
Text GLabel 1350 3700 2    50   Input ~ 0
PPS
Text GLabel 1350 4650 2    50   Input ~ 0
PPS
Text GLabel 1350 5400 2    50   Input ~ 0
PPS
Text GLabel 1350 1050 2    50   Input ~ 0
SDA
Text GLabel 1350 1800 2    50   Input ~ 0
SDA
Text GLabel 1350 2550 2    50   Input ~ 0
SDA
Text GLabel 1350 1150 2    50   Input ~ 0
SCL
Text GLabel 1350 1900 2    50   Input ~ 0
SCL
Text GLabel 1350 2650 2    50   Input ~ 0
SCL
Text GLabel 1350 3300 2    50   Input ~ 0
MOSI
Text GLabel 1350 4250 2    50   Input ~ 0
MOSI
Text GLabel 1350 3400 2    50   Input ~ 0
MISO
Text GLabel 1350 4350 2    50   Input ~ 0
MISO
Text GLabel 1350 3500 2    50   Input ~ 0
SCLK
Text GLabel 1350 4450 2    50   Input ~ 0
SCLK
Text GLabel 1350 5200 2    50   Input ~ 0
TX
Text GLabel 1350 5300 2    50   Input ~ 0
RX
$Comp
L Connector_Generic:Conn_01x07 J5
U 1 1 5E340372
P 1150 3500
F 0 "J5" H 1068 2975 50  0000 C CNN
F 1 "Conn_01x07" H 1068 3066 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B7B-EH-A_1x07_P2.50mm_Vertical" H 1150 3500 50  0001 C CNN
F 3 "~" H 1150 3500 50  0001 C CNN
	1    1150 3500
	-1   0    0    1   
$EndComp
Text GLabel 1350 3600 2    50   Input ~ 0
C0
Text GLabel 1350 4550 2    50   Input ~ 0
C1
$Comp
L Connector_Generic:Conn_01x04 J9
U 1 1 5E35FFF9
P 10300 1750
F 0 "J9" H 10380 1742 50  0000 L CNN
F 1 "Conn_01x04" H 10380 1651 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B4B-EH-A_1x04_P2.50mm_Vertical" H 10300 1750 50  0001 C CNN
F 3 "~" H 10300 1750 50  0001 C CNN
	1    10300 1750
	1    0    0    -1  
$EndComp
$Comp
L device:R_US R1
U 1 1 5E36419D
P 8200 2100
F 0 "R1" H 8268 2146 50  0000 L CNN
F 1 "1k" H 8268 2055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8240 2090 50  0001 C CNN
F 3 "~" H 8200 2100 50  0001 C CNN
	1    8200 2100
	1    0    0    -1  
$EndComp
$Comp
L device:R_US R3
U 1 1 5E3651E9
P 9700 2300
F 0 "R3" H 9768 2346 50  0000 L CNN
F 1 "1k" H 9768 2255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9740 2290 50  0001 C CNN
F 3 "~" H 9700 2300 50  0001 C CNN
	1    9700 2300
	1    0    0    -1  
$EndComp
Text GLabel 10100 1650 0    50   Input ~ 0
5V
Wire Wire Line
	9250 1850 10100 1850
Wire Wire Line
	10000 1950 10100 1950
Wire Wire Line
	8500 1750 10100 1750
Text GLabel 8500 2150 2    50   Input ~ 0
GND
Text GLabel 9250 2250 2    50   Input ~ 0
GND
Text GLabel 10000 2350 2    50   Input ~ 0
GND
Text GLabel 8200 2250 2    50   Input ~ 0
GND
Text GLabel 8950 2350 2    50   Input ~ 0
GND
Text GLabel 9700 2450 2    50   Input ~ 0
GND
Text GLabel 7950 2500 0    50   Input ~ 0
LEDR
Text GLabel 8800 2650 0    50   Input ~ 0
LEDG
Text GLabel 9500 2750 0    50   Input ~ 0
LEDB
Wire Wire Line
	8200 1950 7950 1950
Wire Wire Line
	7950 1950 7950 2500
Wire Wire Line
	8950 2050 8800 2050
Wire Wire Line
	9700 2150 9500 2150
Wire Wire Line
	9500 2150 9500 2750
Wire Wire Line
	8800 2050 8800 2650
$Comp
L device:R_US R2
U 1 1 5E364C8E
P 8950 2200
F 0 "R2" H 9018 2246 50  0000 L CNN
F 1 "1k" H 9018 2155 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8990 2190 50  0001 C CNN
F 3 "~" H 8950 2200 50  0001 C CNN
	1    8950 2200
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7000 Q3
U 1 1 5E653316
P 9900 2150
F 0 "Q3" H 10104 2196 50  0000 L CNN
F 1 "2N7000" H 10104 2105 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 10100 2075 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 9900 2150 50  0001 L CNN
	1    9900 2150
	1    0    0    -1  
$EndComp
Connection ~ 9700 2150
$Comp
L Transistor_FET:2N7000 Q2
U 1 1 5E653BFB
P 9150 2050
F 0 "Q2" H 9354 2096 50  0000 L CNN
F 1 "2N7000" H 9354 2005 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 9350 1975 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 9150 2050 50  0001 L CNN
	1    9150 2050
	1    0    0    -1  
$EndComp
Connection ~ 8950 2050
$Comp
L Transistor_FET:2N7000 Q1
U 1 1 5E6542F3
P 8400 1950
F 0 "Q1" H 8604 1996 50  0000 L CNN
F 1 "2N7000" H 8604 1905 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 8600 1875 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 8400 1950 50  0001 L CNN
	1    8400 1950
	1    0    0    -1  
$EndComp
Connection ~ 8200 1950
$EndSCHEMATC
