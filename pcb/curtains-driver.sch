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
P 5700 4300
F 0 "U1" H 5700 5065 50  0000 C CNN
F 1 "ESP-12F" H 5700 4974 50  0000 C CNN
F 2 "ESP8266:ESP-12E_SMD" H 5700 4300 50  0001 C CNN
F 3 "" H 5700 4300 50  0001 C CNN
	1    5700 4300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack J1
U 1 1 5C0D7D27
P 3150 1650
F 0 "J1" H 3205 1975 50  0000 C CNN
F 1 "Barrel_Jack" H 3205 1884 50  0000 C CNN
F 2 "Connector_BarrelJack:BarrelJack_Horizontal_DS_201" H 3200 1610 50  0001 C CNN
F 3 "~" H 3200 1610 50  0001 C CNN
	1    3150 1650
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x05_Male J3
U 1 1 5C0D813B
P 9450 4200
F 0 "J3" H 9400 4350 50  0000 R CNN
F 1 "Motor" H 9400 4250 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B05B-XH-A_1x05_P2.50mm_Vertical" H 9450 4200 50  0001 C CNN
F 3 "~" H 9450 4200 50  0001 C CNN
	1    9450 4200
	-1   0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM1117-3.3 U2
U 1 1 5C0D8350
P 4650 1550
F 0 "U2" H 4650 1792 50  0000 C CNN
F 1 "LM1117-3.3" H 4650 1701 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 4650 1550 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm1117.pdf" H 4650 1550 50  0001 C CNN
	1    4650 1550
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Reed_Opener SW1
U 1 1 5C0D94BB
P 2850 5000
F 0 "SW1" V 3150 4950 50  0000 R CNN
F 1 "Reed_NC" V 3050 4950 50  0000 R CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 2850 5000 50  0001 C CNN
F 3 "" H 2850 5000 50  0001 C CNN
	1    2850 5000
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Reed_Opener SW2
U 1 1 5C0D9634
P 3750 5050
F 0 "SW2" V 4050 4850 50  0000 L CNN
F 1 "Reed_NC" V 3950 4700 50  0000 L CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 3750 5050 50  0001 C CNN
F 3 "" H 3750 5050 50  0001 C CNN
	1    3750 5050
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x04_Male J2
U 1 1 5C0D9A29
P 6750 3150
F 0 "J2" V 6750 3400 50  0000 R CNN
F 1 "UART" V 6850 3500 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6750 3150 50  0001 C CNN
F 3 "~" H 6750 3150 50  0001 C CNN
	1    6750 3150
	0    1    1    0   
$EndComp
$Comp
L power:Earth #PWR0101
U 1 1 5C0D9FAF
P 4650 2150
F 0 "#PWR0101" H 4650 1900 50  0001 C CNN
F 1 "Earth" H 4650 2000 50  0001 C CNN
F 2 "" H 4650 2150 50  0001 C CNN
F 3 "~" H 4650 2150 50  0001 C CNN
	1    4650 2150
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0102
U 1 1 5C0DA2F0
P 6750 5450
F 0 "#PWR0102" H 6750 5200 50  0001 C CNN
F 1 "Earth" H 6750 5300 50  0001 C CNN
F 2 "" H 6750 5450 50  0001 C CNN
F 3 "~" H 6750 5450 50  0001 C CNN
	1    6750 5450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 5C0DA488
P 3800 1200
F 0 "#PWR0103" H 3800 1050 50  0001 C CNN
F 1 "+5V" H 3815 1373 50  0000 C CNN
F 2 "" H 3800 1200 50  0001 C CNN
F 3 "" H 3800 1200 50  0001 C CNN
	1    3800 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 5C0DA5C0
P 5300 1200
F 0 "#PWR0104" H 5300 1050 50  0001 C CNN
F 1 "+3.3V" H 5315 1373 50  0000 C CNN
F 2 "" H 5300 1200 50  0001 C CNN
F 3 "" H 5300 1200 50  0001 C CNN
	1    5300 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0105
U 1 1 5C0DA7A1
P 6300 3250
F 0 "#PWR0105" H 6300 3100 50  0001 C CNN
F 1 "+3.3V" H 6315 3423 50  0000 C CNN
F 2 "" H 6300 3250 50  0001 C CNN
F 3 "" H 6300 3250 50  0001 C CNN
	1    6300 3250
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0106
U 1 1 5C0DA839
P 6850 3450
F 0 "#PWR0106" H 6850 3200 50  0001 C CNN
F 1 "Earth" H 6850 3300 50  0001 C CNN
F 2 "" H 6850 3450 50  0001 C CNN
F 3 "~" H 6850 3450 50  0001 C CNN
	1    6850 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 4000 6650 4000
Wire Wire Line
	6600 4700 6750 4700
Wire Wire Line
	3450 1550 3800 1550
Wire Wire Line
	3800 1200 3800 1550
Connection ~ 3800 1550
Wire Wire Line
	3800 1550 3950 1550
Wire Wire Line
	4950 1550 5300 1550
Wire Wire Line
	5300 1550 5300 1200
Wire Wire Line
	4650 1850 4650 2150
Wire Wire Line
	3750 4600 4200 4600
Wire Wire Line
	2850 4500 3300 4500
Wire Wire Line
	4800 4700 4300 4700
Wire Wire Line
	4300 4700 4300 3400
$Comp
L power:Earth #PWR0110
U 1 1 5C0E3332
P 3550 2150
F 0 "#PWR0110" H 3550 1900 50  0001 C CNN
F 1 "Earth" H 3550 2000 50  0001 C CNN
F 2 "" H 3550 2150 50  0001 C CNN
F 3 "~" H 3550 2150 50  0001 C CNN
	1    3550 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 1750 3550 1750
Wire Wire Line
	3550 1750 3550 2150
NoConn ~ 5450 5200
NoConn ~ 5550 5200
NoConn ~ 5650 5200
NoConn ~ 5750 5200
NoConn ~ 5850 5200
NoConn ~ 5950 5200
Wire Wire Line
	3750 4600 3750 4850
$Comp
L power:Earth #PWR0107
U 1 1 5C0E7D21
P 3750 5500
F 0 "#PWR0107" H 3750 5250 50  0001 C CNN
F 1 "Earth" H 3750 5350 50  0001 C CNN
F 2 "" H 3750 5500 50  0001 C CNN
F 3 "~" H 3750 5500 50  0001 C CNN
	1    3750 5500
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0108
U 1 1 5C0E7E42
P 2850 5500
F 0 "#PWR0108" H 2850 5250 50  0001 C CNN
F 1 "Earth" H 2850 5350 50  0001 C CNN
F 2 "" H 2850 5500 50  0001 C CNN
F 3 "~" H 2850 5500 50  0001 C CNN
	1    2850 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5250 3750 5500
Wire Wire Line
	2850 5200 2850 5500
$Comp
L Device:C C2
U 1 1 5C0E960D
P 3950 1900
F 0 "C2" H 4065 1946 50  0000 L CNN
F 1 "10uF" H 4065 1855 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3988 1750 50  0001 C CNN
F 3 "~" H 3950 1900 50  0001 C CNN
	1    3950 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C4
U 1 1 5C0E97AC
P 5300 1900
F 0 "C4" H 5418 1946 50  0000 L CNN
F 1 "100uF" H 5418 1855 50  0000 L CNN
F 2 "Capacitor_SMD:C_2816_7142Metric_Pad3.20x4.45mm_HandSolder" H 5338 1750 50  0001 C CNN
F 3 "~" H 5300 1900 50  0001 C CNN
	1    5300 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 1750 5300 1550
Connection ~ 5300 1550
Wire Wire Line
	3950 1750 3950 1550
Connection ~ 3950 1550
Wire Wire Line
	3950 1550 4350 1550
$Comp
L power:Earth #PWR0111
U 1 1 5C0EA8C8
P 3950 2150
F 0 "#PWR0111" H 3950 1900 50  0001 C CNN
F 1 "Earth" H 3950 2000 50  0001 C CNN
F 2 "" H 3950 2150 50  0001 C CNN
F 3 "~" H 3950 2150 50  0001 C CNN
	1    3950 2150
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0112
U 1 1 5C0EA9AF
P 5300 2150
F 0 "#PWR0112" H 5300 1900 50  0001 C CNN
F 1 "Earth" H 5300 2000 50  0001 C CNN
F 2 "" H 5300 2150 50  0001 C CNN
F 3 "~" H 5300 2150 50  0001 C CNN
	1    5300 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2050 3950 2150
Wire Wire Line
	5300 2050 5300 2150
$Comp
L Device:R R4
U 1 1 5C0ED0B4
P 4750 3750
F 0 "R4" H 4820 3796 50  0000 L CNN
F 1 "10K" H 4820 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4680 3750 50  0001 C CNN
F 3 "~" H 4750 3750 50  0001 C CNN
	1    4750 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5C0ED236
P 4450 3750
F 0 "R3" H 4520 3796 50  0000 L CNN
F 1 "10K" H 4520 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4380 3750 50  0001 C CNN
F 3 "~" H 4450 3750 50  0001 C CNN
	1    4450 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 3900 4750 4000
Wire Wire Line
	4750 4000 4800 4000
Wire Wire Line
	4750 3600 4750 3400
Wire Wire Line
	4750 3400 4450 3400
Connection ~ 4300 3400
Wire Wire Line
	4300 3400 4300 3250
Wire Wire Line
	4450 3600 4450 3400
Connection ~ 4450 3400
Wire Wire Line
	4450 3400 4300 3400
Wire Wire Line
	4450 3900 4450 4200
Wire Wire Line
	4450 4200 4800 4200
$Comp
L Device:R R5
U 1 1 5C0F1AE2
P 7250 3700
F 0 "R5" H 7320 3746 50  0000 L CNN
F 1 "10K" H 7320 3655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7180 3700 50  0001 C CNN
F 3 "~" H 7250 3700 50  0001 C CNN
	1    7250 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5C0F1E2B
P 7650 3700
F 0 "R6" H 7720 3746 50  0000 L CNN
F 1 "10K" H 7720 3655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7580 3700 50  0001 C CNN
F 3 "~" H 7650 3700 50  0001 C CNN
	1    7650 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 3850 7250 4400
Wire Wire Line
	7250 4400 6600 4400
Wire Wire Line
	7650 3850 7650 4500
Wire Wire Line
	7650 4500 6600 4500
$Comp
L power:+3.3V #PWR0113
U 1 1 5C0FD42C
P 7450 3300
F 0 "#PWR0113" H 7450 3150 50  0001 C CNN
F 1 "+3.3V" H 7465 3473 50  0000 C CNN
F 2 "" H 7450 3300 50  0001 C CNN
F 3 "" H 7450 3300 50  0001 C CNN
	1    7450 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 3550 7250 3400
Wire Wire Line
	7450 3400 7450 3300
Wire Wire Line
	7650 3550 7650 3400
Connection ~ 7450 3400
$Comp
L Device:R R1
U 1 1 5C144017
P 2850 3750
F 0 "R1" H 2920 3796 50  0000 L CNN
F 1 "10K" H 2920 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2780 3750 50  0001 C CNN
F 3 "~" H 2850 3750 50  0001 C CNN
	1    2850 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5C1441E3
P 3750 3750
F 0 "R2" H 3820 3796 50  0000 L CNN
F 1 "10K" H 3820 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3680 3750 50  0001 C CNN
F 3 "~" H 3750 3750 50  0001 C CNN
	1    3750 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 3600 3750 3400
Wire Wire Line
	3750 3400 4300 3400
Wire Wire Line
	2850 3600 2850 3400
Wire Wire Line
	2850 3400 3750 3400
Connection ~ 3750 3400
Wire Wire Line
	3750 3900 3750 4600
Connection ~ 3750 4600
Wire Wire Line
	2850 3900 2850 4500
Connection ~ 2850 4500
$Comp
L Device:C C3
U 1 1 5C14C311
P 4200 5050
F 0 "C3" H 4315 5096 50  0000 L CNN
F 1 "0.1uF" H 4315 5005 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 4238 4900 50  0001 C CNN
F 3 "~" H 4200 5050 50  0001 C CNN
	1    4200 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5C14C6AA
P 3300 5000
F 0 "C1" H 3415 5046 50  0000 L CNN
F 1 "0.1uF" H 3415 4955 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3338 4850 50  0001 C CNN
F 3 "~" H 3300 5000 50  0001 C CNN
	1    3300 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4600 4200 4900
Connection ~ 4200 4600
Wire Wire Line
	4200 4600 4800 4600
$Comp
L power:Earth #PWR0114
U 1 1 5C1526DA
P 4200 5500
F 0 "#PWR0114" H 4200 5250 50  0001 C CNN
F 1 "Earth" H 4200 5350 50  0001 C CNN
F 2 "" H 4200 5500 50  0001 C CNN
F 3 "~" H 4200 5500 50  0001 C CNN
	1    4200 5500
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0115
U 1 1 5C152829
P 3300 5500
F 0 "#PWR0115" H 3300 5250 50  0001 C CNN
F 1 "Earth" H 3300 5350 50  0001 C CNN
F 2 "" H 3300 5500 50  0001 C CNN
F 3 "~" H 3300 5500 50  0001 C CNN
	1    3300 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 5150 3300 5500
Wire Wire Line
	4200 5200 4200 5500
$Comp
L power:+5V #PWR0116
U 1 1 5C1567EF
P 9100 3250
F 0 "#PWR0116" H 9100 3100 50  0001 C CNN
F 1 "+5V" H 9115 3423 50  0000 C CNN
F 2 "" H 9100 3250 50  0001 C CNN
F 3 "" H 9100 3250 50  0001 C CNN
	1    9100 3250
	1    0    0    -1  
$EndComp
$Comp
L Transistor_Array:ULN2003 U3
U 1 1 5C158F48
P 8500 3900
F 0 "U3" H 8500 4567 50  0000 C CNN
F 1 "ULN2003" H 8500 4476 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_5.3x10.2mm_P1.27mm" H 8550 3350 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 8600 3700 50  0001 C CNN
	1    8500 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 4200 8100 4200
Wire Wire Line
	6600 4300 8100 4300
Wire Wire Line
	8900 4000 9250 4000
Wire Wire Line
	8900 4100 9250 4100
Wire Wire Line
	8900 4200 9250 4200
Wire Wire Line
	8900 4300 9250 4300
Wire Wire Line
	8900 3500 9100 3500
Wire Wire Line
	9100 3500 9100 3250
Wire Wire Line
	9250 4400 9100 4400
Wire Wire Line
	9100 4400 9100 3500
Connection ~ 9100 3500
NoConn ~ 4800 4300
NoConn ~ 4800 4400
$Comp
L power:Earth #PWR0117
U 1 1 5C17A9EA
P 8500 5450
F 0 "#PWR0117" H 8500 5200 50  0001 C CNN
F 1 "Earth" H 8500 5300 50  0001 C CNN
F 2 "" H 8500 5450 50  0001 C CNN
F 3 "~" H 8500 5450 50  0001 C CNN
	1    8500 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 4500 8500 5450
$Comp
L Device:R R7
U 1 1 5C17DF03
P 7050 5050
F 0 "R7" H 6850 5000 50  0000 L CNN
F 1 "10K" H 6800 5100 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6980 5050 50  0001 C CNN
F 3 "~" H 7050 5050 50  0001 C CNN
	1    7050 5050
	-1   0    0    1   
$EndComp
Wire Wire Line
	7050 4900 7050 4600
Wire Wire Line
	6600 4600 7050 4600
$Comp
L power:Earth #PWR0118
U 1 1 5C1853A2
P 7050 5450
F 0 "#PWR0118" H 7050 5200 50  0001 C CNN
F 1 "Earth" H 7050 5300 50  0001 C CNN
F 2 "" H 7050 5450 50  0001 C CNN
F 3 "~" H 7050 5450 50  0001 C CNN
	1    7050 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 5200 7050 5450
Text GLabel 4800 4100 0    50   Input ~ 0
ADC
Text GLabel 7750 1700 2    50   Input ~ 0
ADC
$Comp
L Switch:SW_SPST SW3
U 1 1 5C16A9FB
P 6650 1500
F 0 "SW3" H 6650 1735 50  0000 C CNN
F 1 "UP" H 6650 1644 50  0000 C CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 6650 1500 50  0001 C CNN
F 3 "" H 6650 1500 50  0001 C CNN
	1    6650 1500
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPST SW4
U 1 1 5C16AC01
P 6650 1950
F 0 "SW4" H 6650 2185 50  0000 C CNN
F 1 "DOWN" H 6650 2094 50  0000 C CNN
F 2 "TerminalBlock_TE-Connectivity:TerminalBlock_TE_282834-2_1x02_P2.54mm_Horizontal" H 6650 1950 50  0001 C CNN
F 3 "" H 6650 1950 50  0001 C CNN
	1    6650 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 5C16B88E
P 7100 1500
F 0 "R8" V 6900 1400 50  0000 L CNN
F 1 "5K1" V 7000 1400 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7030 1500 50  0001 C CNN
F 3 "~" H 7100 1500 50  0001 C CNN
	1    7100 1500
	0    1    1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 5C16D39D
P 7100 1950
F 0 "R9" V 6900 1850 50  0000 L CNN
F 1 "10K" V 7000 1850 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7030 1950 50  0001 C CNN
F 3 "~" H 7100 1950 50  0001 C CNN
	1    7100 1950
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0119
U 1 1 5C16D7D1
P 6300 1350
F 0 "#PWR0119" H 6300 1200 50  0001 C CNN
F 1 "+3.3V" H 6315 1523 50  0000 C CNN
F 2 "" H 6300 1350 50  0001 C CNN
F 3 "" H 6300 1350 50  0001 C CNN
	1    6300 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 1500 6950 1500
Wire Wire Line
	6850 1950 6950 1950
Wire Wire Line
	7350 1950 7350 1700
Wire Wire Line
	7350 1700 7600 1700
Connection ~ 7350 1700
Wire Wire Line
	7350 1700 7350 1500
Wire Wire Line
	6450 1500 6300 1500
Wire Wire Line
	6300 1950 6450 1950
Wire Wire Line
	7250 1950 7350 1950
Wire Wire Line
	7250 1500 7350 1500
NoConn ~ 8900 3700
NoConn ~ 8900 3800
NoConn ~ 8900 3900
NoConn ~ 8100 3700
NoConn ~ 8100 3800
NoConn ~ 8100 3900
$Comp
L Switch:SW_SPST SW5
U 1 1 5C1C0250
P 7850 5000
F 0 "SW5" V 7900 4750 50  0000 L CNN
F 1 "FLASH" V 7800 4700 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 7850 5000 50  0001 C CNN
F 3 "" H 7850 5000 50  0001 C CNN
	1    7850 5000
	0    -1   -1   0   
$EndComp
$Comp
L power:Earth #PWR01
U 1 1 5C1C7538
P 7850 5450
F 0 "#PWR01" H 7850 5200 50  0001 C CNN
F 1 "Earth" H 7850 5300 50  0001 C CNN
F 2 "" H 7850 5450 50  0001 C CNN
F 3 "~" H 7850 5450 50  0001 C CNN
	1    7850 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 5200 7850 5450
Wire Wire Line
	7850 4800 7850 4400
Wire Wire Line
	7850 4400 7250 4400
Connection ~ 7250 4400
$Comp
L Device:R R10
U 1 1 5C1E55F5
P 7600 2050
F 0 "R10" H 7500 2000 50  0000 R CNN
F 1 "10K" H 7350 2100 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7530 2050 50  0001 C CNN
F 3 "~" H 7600 2050 50  0001 C CNN
	1    7600 2050
	-1   0    0    1   
$EndComp
$Comp
L power:Earth #PWR02
U 1 1 5C1E61B0
P 7600 2350
F 0 "#PWR02" H 7600 2100 50  0001 C CNN
F 1 "Earth" H 7600 2200 50  0001 C CNN
F 2 "" H 7600 2350 50  0001 C CNN
F 3 "~" H 7600 2350 50  0001 C CNN
	1    7600 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 1700 7600 1900
Connection ~ 7600 1700
Wire Wire Line
	7600 2200 7600 2350
Wire Wire Line
	7600 1700 7750 1700
Wire Wire Line
	2850 4500 2850 4800
Wire Wire Line
	3300 4500 3300 4850
Connection ~ 3300 4500
Wire Wire Line
	3300 4500 4800 4500
$Comp
L power:+3.3V #PWR0109
U 1 1 5C0E1C36
P 4300 3250
F 0 "#PWR0109" H 4300 3100 50  0001 C CNN
F 1 "+3.3V" H 4315 3423 50  0000 C CNN
F 2 "" H 4300 3250 50  0001 C CNN
F 3 "" H 4300 3250 50  0001 C CNN
	1    4300 3250
	1    0    0    -1  
$EndComp
Connection ~ 6300 1500
Wire Wire Line
	6300 1500 6300 1950
Wire Wire Line
	6300 1350 6300 1500
Wire Wire Line
	6850 3350 6850 3450
Wire Wire Line
	6300 3250 6300 3500
Wire Wire Line
	6300 3500 6550 3500
Wire Wire Line
	6550 3500 6550 3350
Wire Wire Line
	6750 4700 6750 5450
Wire Wire Line
	6600 4100 6750 4100
Wire Wire Line
	6650 3350 6650 4000
Connection ~ 6650 4000
Wire Wire Line
	6650 4000 8100 4000
Wire Wire Line
	6750 3350 6750 4100
Connection ~ 6750 4100
Wire Wire Line
	6750 4100 8100 4100
Wire Wire Line
	7250 3400 7450 3400
Wire Wire Line
	7650 3400 7450 3400
$EndSCHEMATC
