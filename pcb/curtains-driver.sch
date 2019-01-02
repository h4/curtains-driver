EESchema Schematic File Version 4
LIBS:curtains-driver-cache
EELAYER 26 0
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
L ESP8266:ESP-12F U1
U 1 1 5C0D7AFE
P 7750 4650
F 0 "U1" H 7750 5415 50  0000 C CNN
F 1 "ESP-12F" H 7750 5324 50  0000 C CNN
F 2 "ESP8266:ESP-12E_SMD" H 7750 4650 50  0001 C CNN
F 3 "" H 7750 4650 50  0001 C CNN
	1    7750 4650
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack J1
U 1 1 5C0D7D27
P 5150 1750
F 0 "J1" H 5205 2075 50  0000 C CNN
F 1 "Barrel_Jack" H 5205 1984 50  0000 C CNN
F 2 "Connector_BarrelJack:BarrelJack_Horizontal_DS_201" H 5200 1710 50  0001 C CNN
F 3 "~" H 5200 1710 50  0001 C CNN
	1    5150 1750
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x05_Male J3
U 1 1 5C0D813B
P 10750 4550
F 0 "J3" H 10700 4700 50  0000 R CNN
F 1 "Motor" H 10700 4600 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B05B-XH-A_1x05_P2.50mm_Vertical" H 10750 4550 50  0001 C CNN
F 3 "~" H 10750 4550 50  0001 C CNN
	1    10750 4550
	-1   0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM1117-3.3 U2
U 1 1 5C0D8350
P 6650 1650
F 0 "U2" H 6650 1892 50  0000 C CNN
F 1 "LM1117-3.3" H 6650 1801 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 6650 1650 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm1117.pdf" H 6650 1650 50  0001 C CNN
	1    6650 1650
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Reed SW1
U 1 1 5C0D94BB
P 5200 5350
F 0 "SW1" V 5500 5300 50  0000 R CNN
F 1 "Reed_NC" V 5400 5300 50  0000 R CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 5200 5350 50  0001 C CNN
F 3 "" H 5200 5350 50  0001 C CNN
	1    5200 5350
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Reed SW2
U 1 1 5C0D9634
P 6100 5400
F 0 "SW2" V 6400 5200 50  0000 L CNN
F 1 "Reed_NC" V 6300 5050 50  0000 L CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 6100 5400 50  0001 C CNN
F 3 "" H 6100 5400 50  0001 C CNN
	1    6100 5400
	0    -1   -1   0   
$EndComp
$Comp
L power:Earth #PWR0101
U 1 1 5C0D9FAF
P 6650 2250
F 0 "#PWR0101" H 6650 2000 50  0001 C CNN
F 1 "Earth" H 6650 2100 50  0001 C CNN
F 2 "" H 6650 2250 50  0001 C CNN
F 3 "~" H 6650 2250 50  0001 C CNN
	1    6650 2250
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0102
U 1 1 5C0DA2F0
P 8700 5800
F 0 "#PWR0102" H 8700 5550 50  0001 C CNN
F 1 "Earth" H 8700 5650 50  0001 C CNN
F 2 "" H 8700 5800 50  0001 C CNN
F 3 "~" H 8700 5800 50  0001 C CNN
	1    8700 5800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 5C0DA488
P 5800 1300
F 0 "#PWR0103" H 5800 1150 50  0001 C CNN
F 1 "+5V" H 5815 1473 50  0000 C CNN
F 2 "" H 5800 1300 50  0001 C CNN
F 3 "" H 5800 1300 50  0001 C CNN
	1    5800 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 5C0DA5C0
P 7300 1300
F 0 "#PWR0104" H 7300 1150 50  0001 C CNN
F 1 "+3.3V" H 7315 1473 50  0000 C CNN
F 2 "" H 7300 1300 50  0001 C CNN
F 3 "" H 7300 1300 50  0001 C CNN
	1    7300 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 5050 8700 5050
Wire Wire Line
	5450 1650 5800 1650
Wire Wire Line
	5800 1300 5800 1650
Connection ~ 5800 1650
Wire Wire Line
	5800 1650 5950 1650
Wire Wire Line
	6950 1650 7300 1650
Wire Wire Line
	7300 1650 7300 1300
Wire Wire Line
	6650 1950 6650 2250
$Comp
L power:Earth #PWR0110
U 1 1 5C0E3332
P 5550 2250
F 0 "#PWR0110" H 5550 2000 50  0001 C CNN
F 1 "Earth" H 5550 2100 50  0001 C CNN
F 2 "" H 5550 2250 50  0001 C CNN
F 3 "~" H 5550 2250 50  0001 C CNN
	1    5550 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 1850 5550 1850
Wire Wire Line
	5550 1850 5550 2250
NoConn ~ 7500 5550
NoConn ~ 7600 5550
NoConn ~ 7700 5550
NoConn ~ 7800 5550
NoConn ~ 7900 5550
NoConn ~ 8000 5550
Wire Wire Line
	6100 4950 6100 5200
$Comp
L power:Earth #PWR0107
U 1 1 5C0E7D21
P 6100 5850
F 0 "#PWR0107" H 6100 5600 50  0001 C CNN
F 1 "Earth" H 6100 5700 50  0001 C CNN
F 2 "" H 6100 5850 50  0001 C CNN
F 3 "~" H 6100 5850 50  0001 C CNN
	1    6100 5850
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0108
U 1 1 5C0E7E42
P 5200 5850
F 0 "#PWR0108" H 5200 5600 50  0001 C CNN
F 1 "Earth" H 5200 5700 50  0001 C CNN
F 2 "" H 5200 5850 50  0001 C CNN
F 3 "~" H 5200 5850 50  0001 C CNN
	1    5200 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 5600 6100 5850
Wire Wire Line
	5200 5550 5200 5850
$Comp
L Device:C C2
U 1 1 5C0E960D
P 5950 2000
F 0 "C2" H 6065 2046 50  0000 L CNN
F 1 "10uF" H 6065 1955 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 5988 1850 50  0001 C CNN
F 3 "~" H 5950 2000 50  0001 C CNN
	1    5950 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C4
U 1 1 5C0E97AC
P 7300 2000
F 0 "C4" H 7418 2046 50  0000 L CNN
F 1 "100uF" H 7418 1955 50  0000 L CNN
F 2 "Capacitor_SMD:C_2816_7142Metric_Pad3.20x4.45mm_HandSolder" H 7338 1850 50  0001 C CNN
F 3 "~" H 7300 2000 50  0001 C CNN
	1    7300 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 1850 7300 1650
Connection ~ 7300 1650
Wire Wire Line
	5950 1850 5950 1650
Connection ~ 5950 1650
Wire Wire Line
	5950 1650 6350 1650
$Comp
L power:Earth #PWR0111
U 1 1 5C0EA8C8
P 5950 2250
F 0 "#PWR0111" H 5950 2000 50  0001 C CNN
F 1 "Earth" H 5950 2100 50  0001 C CNN
F 2 "" H 5950 2250 50  0001 C CNN
F 3 "~" H 5950 2250 50  0001 C CNN
	1    5950 2250
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0112
U 1 1 5C0EA9AF
P 7300 2250
F 0 "#PWR0112" H 7300 2000 50  0001 C CNN
F 1 "Earth" H 7300 2100 50  0001 C CNN
F 2 "" H 7300 2250 50  0001 C CNN
F 3 "~" H 7300 2250 50  0001 C CNN
	1    7300 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 2150 5950 2250
Wire Wire Line
	7300 2150 7300 2250
$Comp
L Device:C C3
U 1 1 5C14C311
P 6550 5400
F 0 "C3" H 6665 5446 50  0000 L CNN
F 1 "0.1uF" H 6665 5355 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 6588 5250 50  0001 C CNN
F 3 "~" H 6550 5400 50  0001 C CNN
	1    6550 5400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5C14C6AA
P 5650 5350
F 0 "C1" H 5765 5396 50  0000 L CNN
F 1 "0.1uF" H 5765 5305 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 5688 5200 50  0001 C CNN
F 3 "~" H 5650 5350 50  0001 C CNN
	1    5650 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 4950 6550 5250
$Comp
L power:Earth #PWR0114
U 1 1 5C1526DA
P 6550 5850
F 0 "#PWR0114" H 6550 5600 50  0001 C CNN
F 1 "Earth" H 6550 5700 50  0001 C CNN
F 2 "" H 6550 5850 50  0001 C CNN
F 3 "~" H 6550 5850 50  0001 C CNN
	1    6550 5850
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0115
U 1 1 5C152829
P 5650 5850
F 0 "#PWR0115" H 5650 5600 50  0001 C CNN
F 1 "Earth" H 5650 5700 50  0001 C CNN
F 2 "" H 5650 5850 50  0001 C CNN
F 3 "~" H 5650 5850 50  0001 C CNN
	1    5650 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 5500 5650 5850
Wire Wire Line
	6550 5550 6550 5850
$Comp
L power:+5V #PWR0116
U 1 1 5C1567EF
P 10400 3700
F 0 "#PWR0116" H 10400 3550 50  0001 C CNN
F 1 "+5V" H 10415 3873 50  0000 C CNN
F 2 "" H 10400 3700 50  0001 C CNN
F 3 "" H 10400 3700 50  0001 C CNN
	1    10400 3700
	1    0    0    -1  
$EndComp
$Comp
L Transistor_Array:ULN2003 U3
U 1 1 5C158F48
P 9850 4250
F 0 "U3" H 9850 4917 50  0000 C CNN
F 1 "ULN2003" H 9850 4826 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_5.3x10.2mm_P1.27mm" H 9900 3700 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 9950 4050 50  0001 C CNN
	1    9850 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 4350 10550 4350
Wire Wire Line
	10250 4450 10550 4450
Wire Wire Line
	10250 4550 10550 4550
Wire Wire Line
	10250 4650 10550 4650
Wire Wire Line
	10250 3850 10400 3850
Wire Wire Line
	10400 3850 10400 3700
Wire Wire Line
	10550 4750 10400 4750
Wire Wire Line
	10400 4750 10400 3850
NoConn ~ 6850 4650
NoConn ~ 6850 4750
Wire Wire Line
	8650 4950 8850 4950
$Comp
L power:Earth #PWR0118
U 1 1 5C1853A2
P 8850 5800
F 0 "#PWR0118" H 8850 5550 50  0001 C CNN
F 1 "Earth" H 8850 5650 50  0001 C CNN
F 2 "" H 8850 5800 50  0001 C CNN
F 3 "~" H 8850 5800 50  0001 C CNN
	1    8850 5800
	1    0    0    -1  
$EndComp
Text GLabel 6850 4450 0    50   Input ~ 0
ADC
Text GLabel 9650 2250 2    50   Input ~ 0
ADC
$Comp
L Switch:SW_SPST SW3
U 1 1 5C16A9FB
P 8650 2100
F 0 "SW3" H 8650 2335 50  0000 C CNN
F 1 "UP" H 8650 2244 50  0000 C CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 8650 2100 50  0001 C CNN
F 3 "" H 8650 2100 50  0001 C CNN
	1    8650 2100
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPST SW4
U 1 1 5C16AC01
P 8650 1600
F 0 "SW4" H 8650 1835 50  0000 C CNN
F 1 "DOWN" H 8650 1744 50  0000 C CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 8650 1600 50  0001 C CNN
F 3 "" H 8650 1600 50  0001 C CNN
	1    8650 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 5C16B88E
P 8950 1850
F 0 "R8" H 8750 1800 50  0000 L CNN
F 1 "56K" H 8700 1900 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8880 1850 50  0001 C CNN
F 3 "~" H 8950 1850 50  0001 C CNN
	1    8950 1850
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR0119
U 1 1 5C16D7D1
P 8300 1450
F 0 "#PWR0119" H 8300 1300 50  0001 C CNN
F 1 "+3.3V" H 8315 1623 50  0000 C CNN
F 2 "" H 8300 1450 50  0001 C CNN
F 3 "" H 8300 1450 50  0001 C CNN
	1    8300 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 1600 8950 1600
Wire Wire Line
	8850 2100 8950 2100
Wire Wire Line
	8450 1600 8300 1600
NoConn ~ 10250 4050
NoConn ~ 10250 4150
NoConn ~ 10250 4250
NoConn ~ 9450 4050
NoConn ~ 9450 4150
NoConn ~ 9450 4250
$Comp
L Switch:SW_SPST SW5
U 1 1 5C1C0250
P 9600 5150
F 0 "SW5" H 9500 5400 50  0000 L CNN
F 1 "FLASH" H 9500 5300 50  0000 L CNN
F 2 "Button_Switch_SMD:SW_SPST_B3U-1000P" H 9600 5150 50  0001 C CNN
F 3 "" H 9600 5150 50  0001 C CNN
	1    9600 5150
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR01
U 1 1 5C1C7538
P 9850 5850
F 0 "#PWR01" H 9850 5600 50  0001 C CNN
F 1 "Earth" H 9850 5700 50  0001 C CNN
F 2 "" H 9850 5850 50  0001 C CNN
F 3 "~" H 9850 5850 50  0001 C CNN
	1    9850 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 5C1E55F5
P 8950 2500
F 0 "R10" H 8850 2450 50  0000 R CNN
F 1 "10K" H 8700 2550 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8880 2500 50  0001 C CNN
F 3 "~" H 8950 2500 50  0001 C CNN
	1    8950 2500
	-1   0    0    1   
$EndComp
$Comp
L power:Earth #PWR02
U 1 1 5C1E61B0
P 8950 2750
F 0 "#PWR02" H 8950 2500 50  0001 C CNN
F 1 "Earth" H 8950 2600 50  0001 C CNN
F 2 "" H 8950 2750 50  0001 C CNN
F 3 "~" H 8950 2750 50  0001 C CNN
	1    8950 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 2250 8950 2350
Wire Wire Line
	8950 2650 8950 2750
Wire Wire Line
	8950 2250 9400 2250
Wire Wire Line
	5200 4850 5200 5150
Wire Wire Line
	5650 4850 5650 5200
$Comp
L power:+3.3V #PWR0109
U 1 1 5C0E1C36
P 5200 3850
F 0 "#PWR0109" H 5200 3700 50  0001 C CNN
F 1 "+3.3V" H 5215 4023 50  0000 C CNN
F 2 "" H 5200 3850 50  0001 C CNN
F 3 "" H 5200 3850 50  0001 C CNN
	1    5200 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 1450 8300 1600
Wire Wire Line
	8700 5050 8700 5800
Wire Wire Line
	8950 3800 8950 4350
Wire Wire Line
	9050 3800 9050 4450
$Comp
L Switch:SW_SPST SW6
U 1 1 5C1F546E
P 6250 3750
F 0 "SW6" H 6200 4000 50  0000 L CNN
F 1 "RESET" H 6200 3900 50  0000 L CNN
F 2 "Button_Switch_SMD:SW_SPST_B3U-1000P" H 6250 3750 50  0001 C CNN
F 3 "" H 6250 3750 50  0001 C CNN
	1    6250 3750
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0120
U 1 1 5C1FC64C
P 5850 3850
F 0 "#PWR0120" H 5850 3600 50  0001 C CNN
F 1 "Earth" H 5850 3700 50  0001 C CNN
F 2 "" H 5850 3850 50  0001 C CNN
F 3 "~" H 5850 3850 50  0001 C CNN
	1    5850 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4850 5650 4850
Connection ~ 5650 4850
Wire Wire Line
	6100 4950 6550 4950
Connection ~ 6550 4950
Wire Wire Line
	6550 4950 6850 4950
Wire Wire Line
	6500 4550 6850 4550
Wire Wire Line
	5200 3850 5200 4000
Connection ~ 5200 4400
Connection ~ 5200 4200
Wire Wire Line
	5200 4200 5200 4400
Connection ~ 5200 4000
Wire Wire Line
	5200 4000 5200 4200
Wire Wire Line
	6500 4550 6500 4200
Wire Wire Line
	6450 3750 6700 3750
Wire Wire Line
	6700 3750 6700 4000
Wire Wire Line
	6700 4350 6800 4350
Wire Wire Line
	6050 3750 5850 3750
Wire Wire Line
	5850 3750 5850 3850
Wire Wire Line
	6700 4350 6700 4000
Connection ~ 6700 4000
Text GLabel 6850 5050 0    50   Input ~ 0
VCC
Text GLabel 10400 2250 0    50   Input ~ 0
VCC
Wire Wire Line
	10400 2250 10600 2250
Wire Wire Line
	10600 2250 10600 2000
$Comp
L power:+3.3V #PWR03
U 1 1 5C2B25DB
P 10600 2000
F 0 "#PWR03" H 10600 1850 50  0001 C CNN
F 1 "+3.3V" H 10615 2173 50  0000 C CNN
F 2 "" H 10600 2000 50  0001 C CNN
F 3 "" H 10600 2000 50  0001 C CNN
	1    10600 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 4950 8850 5200
Wire Wire Line
	8650 4750 9350 4750
Wire Wire Line
	9100 5200 9100 4850
Wire Wire Line
	9100 4850 8650 4850
Wire Wire Line
	9350 5200 9350 5150
Wire Wire Line
	9100 5700 9350 5700
Wire Wire Line
	9350 5700 9500 5700
Connection ~ 9350 5700
Text GLabel 9500 5700 2    50   Input ~ 0
VCC
Connection ~ 10400 3850
Connection ~ 9350 5150
Wire Wire Line
	9350 5150 9350 4750
Wire Wire Line
	9800 5150 9850 5150
Wire Wire Line
	9850 5850 9850 5150
Wire Wire Line
	8650 4650 9450 4650
Wire Wire Line
	8650 4550 9450 4550
Wire Wire Line
	8650 4450 9050 4450
Wire Wire Line
	9050 4450 9450 4450
Connection ~ 9050 4450
Wire Wire Line
	8650 4350 8950 4350
Wire Wire Line
	8950 4350 9450 4350
Connection ~ 8950 4350
Wire Wire Line
	9850 5150 9850 4850
Connection ~ 9850 5150
Wire Wire Line
	6300 4850 6300 4400
Wire Wire Line
	5650 4850 6300 4850
Connection ~ 6300 4850
Wire Wire Line
	6300 4850 6850 4850
Wire Wire Line
	6100 4600 6100 4950
Connection ~ 6100 4950
$Comp
L Device:R R2
U 1 1 5C3A4413
P 5450 4200
F 0 "R2" V 5350 4150 50  0000 C CNN
F 1 "10K" V 5350 4300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5380 4200 50  0001 C CNN
F 3 "~" H 5450 4200 50  0001 C CNN
	1    5450 4200
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5C3A78D6
P 5450 4400
F 0 "R3" V 5350 4350 50  0000 C CNN
F 1 "10K" V 5350 4500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5380 4400 50  0001 C CNN
F 3 "~" H 5450 4400 50  0001 C CNN
	1    5450 4400
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5C3A7AB8
P 5450 4600
F 0 "R4" V 5350 4550 50  0000 C CNN
F 1 "10K" V 5350 4700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5380 4600 50  0001 C CNN
F 3 "~" H 5450 4600 50  0001 C CNN
	1    5450 4600
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 5C3A7C3C
P 5450 4000
F 0 "R1" V 5350 3950 50  0000 C CNN
F 1 "10K" V 5350 4100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5380 4000 50  0001 C CNN
F 3 "~" H 5450 4000 50  0001 C CNN
	1    5450 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 4200 6500 4200
Wire Wire Line
	5600 4400 6300 4400
Wire Wire Line
	5200 4400 5300 4400
Wire Wire Line
	5200 4200 5300 4200
Wire Wire Line
	5200 4000 5300 4000
Wire Wire Line
	5200 4400 5200 4600
Wire Wire Line
	5200 4600 5300 4600
Wire Wire Line
	5600 4600 6100 4600
Wire Wire Line
	5600 4000 6700 4000
$Comp
L Device:R R7
U 1 1 5C3F400D
P 9350 5350
F 0 "R7" H 9250 5300 50  0000 C CNN
F 1 "10K" H 9150 5400 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9280 5350 50  0001 C CNN
F 3 "~" H 9350 5350 50  0001 C CNN
	1    9350 5350
	-1   0    0    1   
$EndComp
$Comp
L Device:R R6
U 1 1 5C3F59B6
P 9100 5350
F 0 "R6" H 9000 5300 50  0000 C CNN
F 1 "10K" H 8900 5400 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9030 5350 50  0001 C CNN
F 3 "~" H 9100 5350 50  0001 C CNN
	1    9100 5350
	-1   0    0    1   
$EndComp
$Comp
L Device:R R5
U 1 1 5C3F5B4E
P 8850 5350
F 0 "R5" H 8750 5300 50  0000 C CNN
F 1 "10K" H 8650 5400 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8780 5350 50  0001 C CNN
F 3 "~" H 8850 5350 50  0001 C CNN
	1    8850 5350
	-1   0    0    1   
$EndComp
Wire Wire Line
	9350 5500 9350 5700
Wire Wire Line
	9100 5500 9100 5700
Wire Wire Line
	8850 5500 8850 5800
Wire Wire Line
	9400 5150 9350 5150
$Comp
L Device:C C5
U 1 1 5C433B1E
P 7000 5400
F 0 "C5" H 7115 5446 50  0000 L CNN
F 1 "0.1uF" H 7115 5355 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 7038 5250 50  0001 C CNN
F 3 "~" H 7000 5400 50  0001 C CNN
	1    7000 5400
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR04
U 1 1 5C4348DC
P 7000 5850
F 0 "#PWR04" H 7000 5600 50  0001 C CNN
F 1 "Earth" H 7000 5700 50  0001 C CNN
F 2 "" H 7000 5850 50  0001 C CNN
F 3 "~" H 7000 5850 50  0001 C CNN
	1    7000 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 5050 7000 5250
Wire Wire Line
	7000 5550 7000 5850
Wire Wire Line
	7000 5050 6850 5050
Wire Wire Line
	8300 1600 8300 2100
Wire Wire Line
	8300 2100 8450 2100
Connection ~ 8300 1600
Wire Wire Line
	8950 1700 8950 1600
Connection ~ 8950 2100
Wire Wire Line
	8950 2100 8950 2000
Connection ~ 8950 2250
Wire Wire Line
	8950 2100 8950 2250
$Comp
L Device:C C6
U 1 1 5C20E8C4
P 9400 2500
F 0 "C6" H 9515 2546 50  0000 L CNN
F 1 "0.1uF" H 9515 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 9438 2350 50  0001 C CNN
F 3 "~" H 9400 2500 50  0001 C CNN
	1    9400 2500
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0105
U 1 1 5C20EBA4
P 9400 2750
F 0 "#PWR0105" H 9400 2500 50  0001 C CNN
F 1 "Earth" H 9400 2600 50  0001 C CNN
F 2 "" H 9400 2750 50  0001 C CNN
F 3 "~" H 9400 2750 50  0001 C CNN
	1    9400 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 2350 9400 2250
Connection ~ 9400 2250
Wire Wire Line
	9400 2250 9650 2250
Wire Wire Line
	9400 2750 9400 2650
$Comp
L Connector:USB_B_Micro J2
U 1 1 5C242497
P 1450 3150
F 0 "J2" H 1505 3617 50  0000 C CNN
F 1 "USB_B_Micro" H 1505 3526 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex_47346-0001" H 1600 3100 50  0001 C CNN
F 3 "~" H 1600 3100 50  0001 C CNN
	1    1450 3150
	1    0    0    -1  
$EndComp
Text GLabel 8950 3800 1    50   Input ~ 0
TXD
Text GLabel 9050 3800 1    50   Input ~ 0
RXD
Text GLabel 3750 2950 2    50   Input ~ 0
TXD
Text GLabel 3750 2850 2    50   Input ~ 0
RXD
$Comp
L power:+5V #PWR06
U 1 1 5C24ED56
P 1900 2450
F 0 "#PWR06" H 1900 2300 50  0001 C CNN
F 1 "+5V" H 1915 2623 50  0000 C CNN
F 2 "" H 1900 2450 50  0001 C CNN
F 3 "" H 1900 2450 50  0001 C CNN
	1    1900 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 2450 1900 2550
Wire Wire Line
	1750 2950 1900 2950
$Comp
L power:Earth #PWR0106
U 1 1 5C270F18
P 1450 3650
F 0 "#PWR0106" H 1450 3400 50  0001 C CNN
F 1 "Earth" H 1450 3500 50  0001 C CNN
F 2 "" H 1450 3650 50  0001 C CNN
F 3 "~" H 1450 3650 50  0001 C CNN
	1    1450 3650
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0113
U 1 1 5C270FDD
P 1350 3650
F 0 "#PWR0113" H 1350 3400 50  0001 C CNN
F 1 "Earth" H 1350 3500 50  0001 C CNN
F 2 "" H 1350 3650 50  0001 C CNN
F 3 "~" H 1350 3650 50  0001 C CNN
	1    1350 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 3550 1450 3650
Wire Wire Line
	1350 3550 1350 3650
$Comp
L Interface_USB:CH340G U4
U 1 1 5C24F530
P 2900 3250
F 0 "U4" H 3050 2650 50  0000 C CNN
F 1 "CH340G" H 3150 2550 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 2950 2700 50  0001 L CNN
F 3 "http://www.datasheet5.com/pdf-local-2195953" H 2550 4050 50  0001 C CNN
	1    2900 3250
	1    0    0    -1  
$EndComp
NoConn ~ 2500 2950
$Comp
L power:Earth #PWR010
U 1 1 5C2A318F
P 2900 4100
F 0 "#PWR010" H 2900 3850 50  0001 C CNN
F 1 "Earth" H 2900 3950 50  0001 C CNN
F 2 "" H 2900 4100 50  0001 C CNN
F 3 "~" H 2900 4100 50  0001 C CNN
	1    2900 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 4100 2900 3850
Wire Wire Line
	2800 2650 2800 2350
Wire Wire Line
	2800 2250 2900 2250
Wire Wire Line
	2900 2250 2900 2350
$Comp
L power:+3.3V #PWR09
U 1 1 5C2ABA2D
P 2800 2200
F 0 "#PWR09" H 2800 2050 50  0001 C CNN
F 1 "+3.3V" H 2815 2373 50  0000 C CNN
F 2 "" H 2800 2200 50  0001 C CNN
F 3 "" H 2800 2200 50  0001 C CNN
	1    2800 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 2250 2800 2350
Connection ~ 2800 2250
$Comp
L Device:C_Small C8
U 1 1 5C2B0616
P 2350 2500
F 0 "C8" H 2465 2546 50  0000 L CNN
F 1 "10uF" H 2465 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 2388 2350 50  0001 C CNN
F 3 "~" H 2350 2500 50  0001 C CNN
	1    2350 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 2400 2350 2350
Wire Wire Line
	2350 2350 2800 2350
Connection ~ 2800 2350
Wire Wire Line
	2800 2250 2800 2200
$Comp
L power:Earth #PWR07
U 1 1 5C2C1603
P 2350 2650
F 0 "#PWR07" H 2350 2400 50  0001 C CNN
F 1 "Earth" H 2350 2500 50  0001 C CNN
F 2 "" H 2350 2650 50  0001 C CNN
F 3 "~" H 2350 2650 50  0001 C CNN
	1    2350 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 5C2C634A
P 4150 2500
F 0 "C10" H 4265 2546 50  0000 L CNN
F 1 "10uF" H 4265 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 4188 2350 50  0001 C CNN
F 3 "~" H 4150 2500 50  0001 C CNN
	1    4150 2500
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR011
U 1 1 5C2C666E
P 4150 2650
F 0 "#PWR011" H 4150 2400 50  0001 C CNN
F 1 "Earth" H 4150 2500 50  0001 C CNN
F 2 "" H 4150 2650 50  0001 C CNN
F 3 "~" H 4150 2650 50  0001 C CNN
	1    4150 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 2400 4150 2350
Wire Wire Line
	4150 2350 2900 2350
Connection ~ 2900 2350
Wire Wire Line
	2900 2350 2900 2650
Wire Wire Line
	1750 3150 2500 3150
Wire Wire Line
	1750 3250 2500 3250
$Comp
L Device:Crystal Y1
U 1 1 5C2E463A
P 2200 3650
F 0 "Y1" H 2200 3918 50  0000 C CNN
F 1 "HC-49" H 2200 3827 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_HC49-SD_HandSoldering" H 2200 3650 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/12mhz-hc-49sm" H 2200 3650 50  0001 C CNN
	1    2200 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 3650 2400 3650
Wire Wire Line
	2050 3650 1900 3650
Wire Wire Line
	1900 3650 1900 3450
Wire Wire Line
	1900 3450 2500 3450
$Comp
L Device:C_Small C7
U 1 1 5C2EF4D3
P 1900 3850
F 0 "C7" H 1992 3896 50  0000 L CNN
F 1 "22pF" H 1992 3805 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 1900 3850 50  0001 C CNN
F 3 "~" H 1900 3850 50  0001 C CNN
	1    1900 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5C2EFD2A
P 2400 3850
F 0 "C9" H 2492 3896 50  0000 L CNN
F 1 "22pF" H 2492 3805 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 2400 3850 50  0001 C CNN
F 3 "~" H 2400 3850 50  0001 C CNN
	1    2400 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 3750 1900 3650
Connection ~ 1900 3650
Wire Wire Line
	2400 3750 2400 3650
Connection ~ 2400 3650
Wire Wire Line
	2400 3650 2500 3650
$Comp
L power:Earth #PWR08
U 1 1 5C2FA416
P 2400 4100
F 0 "#PWR08" H 2400 3850 50  0001 C CNN
F 1 "Earth" H 2400 3950 50  0001 C CNN
F 2 "" H 2400 4100 50  0001 C CNN
F 3 "~" H 2400 4100 50  0001 C CNN
	1    2400 4100
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR05
U 1 1 5C2FA56B
P 1900 4100
F 0 "#PWR05" H 1900 3850 50  0001 C CNN
F 1 "Earth" H 1900 3950 50  0001 C CNN
F 2 "" H 1900 4100 50  0001 C CNN
F 3 "~" H 1900 4100 50  0001 C CNN
	1    1900 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 4100 2400 3950
Wire Wire Line
	1900 4100 1900 3950
Wire Wire Line
	2350 2600 2350 2650
Wire Wire Line
	4150 2600 4150 2650
NoConn ~ 1750 3350
$Comp
L Device:D_Schottky_Small D1
U 1 1 5C34B14C
P 1900 2650
F 0 "D1" V 1854 2718 50  0000 L CNN
F 1 "M7" V 1945 2718 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" V 1900 2650 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/yt-m7" V 1900 2650 50  0001 C CNN
	1    1900 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	1900 2750 1900 2950
NoConn ~ 3300 3150
NoConn ~ 3300 3250
NoConn ~ 3300 3350
NoConn ~ 3300 3450
Wire Wire Line
	6800 4350 6800 3750
Connection ~ 6800 4350
Wire Wire Line
	6800 4350 6850 4350
Text GLabel 6800 3750 1    50   Input ~ 0
RST
Wire Wire Line
	9350 4750 9350 3800
Connection ~ 9350 4750
Text GLabel 9350 3800 1    50   Input ~ 0
GPIO0
$Comp
L Transistor_BJT:S8050 Q2
U 1 1 5C3778E4
P 4350 4050
F 0 "Q2" H 4541 4004 50  0000 L CNN
F 1 "S8050" H 4541 4095 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4550 3975 50  0001 L CIN
F 3 "http://www.unisonic.com.tw/datasheet/S8050.pdf" H 4350 4050 50  0001 L CNN
	1    4350 4050
	1    0    0    1   
$EndComp
$Comp
L Transistor_BJT:S8050 Q1
U 1 1 5C37813C
P 4350 3400
F 0 "Q1" H 4541 3446 50  0000 L CNN
F 1 "S8050" H 4541 3355 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4550 3325 50  0001 L CIN
F 3 "http://www.unisonic.com.tw/datasheet/S8050.pdf" H 4350 3400 50  0001 L CNN
	1    4350 3400
	1    0    0    -1  
$EndComp
Text GLabel 4450 4400 3    50   Input ~ 0
GPIO0
Text GLabel 4450 3050 1    50   Input ~ 0
RST
Wire Wire Line
	4450 3200 4450 3050
Wire Wire Line
	4450 4400 4450 4250
$Comp
L Device:R R9
U 1 1 5C3AF7CE
P 3900 3400
F 0 "R9" V 3800 3350 50  0000 C CNN
F 1 "10K" V 3800 3500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3830 3400 50  0001 C CNN
F 3 "~" H 3900 3400 50  0001 C CNN
	1    3900 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5C3AFAD3
P 3900 4050
F 0 "R11" V 3800 4000 50  0000 C CNN
F 1 "10K" V 3800 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3830 4050 50  0001 C CNN
F 3 "~" H 3900 4050 50  0001 C CNN
	1    3900 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	3300 3650 3500 3650
Wire Wire Line
	3500 3650 3500 4050
Wire Wire Line
	3500 4050 3600 4050
Wire Wire Line
	4050 4050 4150 4050
Wire Wire Line
	3300 3550 3500 3550
Wire Wire Line
	3500 3550 3500 3400
Wire Wire Line
	3500 3400 3600 3400
Wire Wire Line
	4050 3400 4150 3400
Wire Wire Line
	4450 3600 4450 3700
Wire Wire Line
	4450 3700 3600 3800
Wire Wire Line
	3600 3800 3600 4050
Connection ~ 3600 4050
Wire Wire Line
	3600 4050 3750 4050
Wire Wire Line
	4450 3850 4450 3800
Wire Wire Line
	4450 3800 3600 3700
Wire Wire Line
	3600 3700 3600 3400
Connection ~ 3600 3400
Wire Wire Line
	3600 3400 3750 3400
$Comp
L Device:R R12
U 1 1 5C4281D2
P 3550 2850
F 0 "R12" V 3450 2750 50  0000 C CNN
F 1 "470" V 3450 2950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3480 2850 50  0001 C CNN
F 3 "~" H 3550 2850 50  0001 C CNN
	1    3550 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R R13
U 1 1 5C42886E
P 3550 2950
F 0 "R13" V 3650 2850 50  0000 C CNN
F 1 "470" V 3650 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3480 2950 50  0001 C CNN
F 3 "~" H 3550 2950 50  0001 C CNN
	1    3550 2950
	0    1    1    0   
$EndComp
Wire Wire Line
	3300 2850 3400 2850
Wire Wire Line
	3300 2950 3400 2950
Wire Wire Line
	3700 2850 3750 2850
Wire Wire Line
	3700 2950 3750 2950
$EndSCHEMATC
