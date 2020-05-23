EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "ESPCLOCK V3.0"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:CP C4
U 1 1 5E47ECC7
P 8900 2650
F 0 "C4" V 9155 2650 50  0000 C CNN
F 1 "0.47F" V 9064 2650 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P5.00mm" H 8938 2500 50  0001 C CNN
F 3 "~" H 8900 2650 50  0001 C CNN
	1    8900 2650
	0    -1   -1   0   
$EndComp
$Comp
L Diode:BAT85 D1
U 1 1 5E4801F5
P 7550 2650
F 0 "D1" H 7550 2434 50  0000 C CNN
F 1 "BAT85" H 7550 2525 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 7550 2475 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAT85.pdf" H 7550 2650 50  0001 C CNN
	1    7550 2650
	-1   0    0    1   
$EndComp
Wire Wire Line
	2350 3100 2900 3100
Wire Wire Line
	2350 3650 2900 3650
$Comp
L Device:C C1
U 1 1 5E493107
P 2900 3400
F 0 "C1" H 3015 3446 50  0000 L CNN
F 1 "1uF" H 3015 3355 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W2.0mm_P2.50mm" H 2938 3250 50  0001 C CNN
F 3 "~" H 2900 3400 50  0001 C CNN
	1    2900 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 3250 2900 3100
Connection ~ 2900 3100
Wire Wire Line
	2900 3100 3050 3100
Wire Wire Line
	2900 3550 2900 3650
Connection ~ 2900 3650
Connection ~ 3600 3650
$Comp
L Device:C C3
U 1 1 5E49889D
P 5200 2450
F 0 "C3" V 4948 2450 50  0000 C CNN
F 1 "10uF" V 5039 2450 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W2.0mm_P2.50mm" H 5238 2300 50  0001 C CNN
F 3 "~" H 5200 2450 50  0001 C CNN
	1    5200 2450
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5E499E8A
P 5200 1650
F 0 "SW1" H 5200 1935 50  0000 C CNN
F 1 "SW_Push" H 5200 1844 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm" H 5200 1850 50  0001 C CNN
F 3 "~" H 5200 1850 50  0001 C CNN
	1    5200 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5E49B778
P 6000 2150
F 0 "R4" H 5941 2104 50  0000 R CNN
F 1 "10K" H 5941 2195 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" H 6000 2150 50  0001 C CNN
F 3 "~" H 6000 2150 50  0001 C CNN
	1    6000 2150
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5E49D04C
P 4500 2150
F 0 "R1" H 4441 2104 50  0000 R CNN
F 1 "10K" H 4441 2195 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" H 4500 2150 50  0001 C CNN
F 3 "~" H 4500 2150 50  0001 C CNN
	1    4500 2150
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 2450 4500 2250
Wire Wire Line
	6000 2250 6000 2450
Connection ~ 6000 2450
Wire Wire Line
	4500 2050 4500 1900
Wire Wire Line
	4500 1900 6000 1900
Connection ~ 4500 1900
Wire Wire Line
	6000 1900 6000 2050
Wire Wire Line
	6000 2450 6350 2450
Wire Wire Line
	6350 2450 6350 1650
Wire Wire Line
	5400 1650 6350 1650
$Comp
L HT7333:HT7333-A L1
U 1 1 5E482659
P 3450 3100
F 0 "L1" H 3500 3497 60  0000 C CNN
F 1 "HT7833" H 3500 3391 60  0000 C CNN
F 2 "HT7333.pretty:ht7333-a" H 3450 3100 60  0001 C CNN
F 3 "" H 3450 3100 60  0000 C CNN
	1    3450 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5E4B0DD7
P 5100 4350
F 0 "R3" V 4900 4350 50  0000 C CNN
F 1 "4.7K" V 5000 4350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" H 5100 4350 50  0001 C CNN
F 3 "~" H 5100 4350 50  0001 C CNN
	1    5100 4350
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5E4B893F
P 4900 4350
F 0 "R2" V 5096 4350 50  0000 C CNN
F 1 "4.7k" V 5005 4350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" H 4900 4350 50  0001 C CNN
F 3 "~" H 4900 4350 50  0001 C CNN
	1    4900 4350
	-1   0    0    1   
$EndComp
Wire Wire Line
	8550 3100 8550 2650
Wire Wire Line
	9050 2650 9200 2650
$Comp
L Device:R_Small R5
U 1 1 5E4DAA29
P 6100 2850
F 0 "R5" H 6041 2804 50  0000 R CNN
F 1 "4.7K" H 6041 2895 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" H 6100 2850 50  0001 C CNN
F 3 "~" H 6100 2850 50  0001 C CNN
	1    6100 2850
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R6
U 1 1 5E4DB027
P 6400 2850
F 0 "R6" H 6341 2804 50  0000 R CNN
F 1 "4.7K" H 6341 2895 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" H 6400 2850 50  0001 C CNN
F 3 "~" H 6400 2850 50  0001 C CNN
	1    6400 2850
	-1   0    0    1   
$EndComp
Connection ~ 6100 2650
Wire Wire Line
	6100 2650 6400 2650
Wire Wire Line
	6400 2750 6400 2650
Connection ~ 6400 2650
$Comp
L Connector:Conn_01x02_Female J1
U 1 1 5E4EB2CE
P 7700 5850
F 0 "J1" V 7546 5662 50  0000 R CNN
F 1 "Quartz Clock" V 7637 5662 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical" H 7700 5850 50  0001 C CNN
F 3 "~" H 7700 5850 50  0001 C CNN
	1    7700 5850
	0    -1   1    0   
$EndComp
Wire Wire Line
	7700 4650 7700 3700
Wire Wire Line
	7700 3700 7950 3700
Wire Wire Line
	7800 3800 7950 3800
Wire Wire Line
	7600 3400 7950 3400
Connection ~ 8550 2650
Wire Wire Line
	8550 2650 8750 2650
Wire Wire Line
	8550 4300 8550 4450
Wire Wire Line
	8550 4450 9200 4450
Wire Wire Line
	9200 4450 9200 2650
Wire Wire Line
	7700 2650 7850 2650
$Comp
L Device:R_Small R7
U 1 1 5E4B0785
P 7850 3000
F 0 "R7" H 7791 2954 50  0000 R CNN
F 1 "10K" H 7791 3045 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" H 7850 3000 50  0001 C CNN
F 3 "~" H 7850 3000 50  0001 C CNN
	1    7850 3000
	-1   0    0    1   
$EndComp
Wire Wire Line
	7850 3500 7850 3100
Wire Wire Line
	7850 3500 7950 3500
Wire Wire Line
	7850 2900 7850 2650
Connection ~ 7850 2650
Wire Wire Line
	7850 2650 8550 2650
Wire Wire Line
	7800 3800 7800 4650
$Comp
L Diode:1N4148 D2
U 1 1 5E67CA51
P 7550 5000
F 0 "D2" H 7550 5216 50  0000 C CNN
F 1 "1N4148" H 7550 5125 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 7550 4825 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 7550 5000 50  0001 C CNN
	1    7550 5000
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D4
U 1 1 5E67FD9D
P 7950 5000
F 0 "D4" H 7950 5216 50  0000 C CNN
F 1 "1N4148" H 7950 5125 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 7950 4825 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 7950 5000 50  0001 C CNN
	1    7950 5000
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D3
U 1 1 5E682C31
P 7550 5350
F 0 "D3" H 7550 5566 50  0000 C CNN
F 1 "1N4148" H 7550 5475 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 7550 5175 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 7550 5350 50  0001 C CNN
	1    7550 5350
	-1   0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D5
U 1 1 5E6835E6
P 7950 5350
F 0 "D5" H 7950 5566 50  0000 C CNN
F 1 "1N4148" H 7950 5475 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 7950 5175 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 7950 5350 50  0001 C CNN
	1    7950 5350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7800 5550 7800 5650
Wire Wire Line
	7700 5550 7700 5650
Wire Wire Line
	7700 5000 7800 5000
Wire Wire Line
	7700 5350 7800 5350
Wire Wire Line
	7800 4650 8250 4650
Wire Wire Line
	8250 4650 8250 5000
Wire Wire Line
	8250 5550 7800 5550
Wire Wire Line
	7700 4650 7250 4650
Wire Wire Line
	7250 4650 7250 5000
Wire Wire Line
	7250 5550 7700 5550
Wire Wire Line
	8100 5000 8250 5000
Connection ~ 8250 5000
Wire Wire Line
	8250 5000 8250 5350
Wire Wire Line
	7400 5000 7250 5000
Connection ~ 7250 5000
Wire Wire Line
	8100 5350 8250 5350
Connection ~ 8250 5350
Wire Wire Line
	8250 5350 8250 5550
Wire Wire Line
	7400 5350 7250 5350
Wire Wire Line
	7250 5000 7250 5350
Connection ~ 7250 5350
Wire Wire Line
	7250 5350 7250 5550
Wire Wire Line
	2900 3650 3600 3650
$Comp
L Device:C C2
U 1 1 5E71E33D
P 4150 3300
F 0 "C2" H 4265 3346 50  0000 L CNN
F 1 "2.2uF" H 4265 3255 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W2.0mm_P2.50mm" H 4188 3150 50  0001 C CNN
F 3 "~" H 4150 3300 50  0001 C CNN
	1    4150 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 3400 3600 3650
Wire Wire Line
	4150 1900 4500 1900
Wire Wire Line
	3950 2950 4150 2950
Wire Wire Line
	4150 1900 4150 2950
Connection ~ 4150 2950
Wire Wire Line
	4150 3450 4150 3650
Wire Wire Line
	4150 2950 4150 3150
$Comp
L Connector:Conn_01x08_Female ESP07-GND1
U 1 1 5E786620
P 5100 3700
F 0 "ESP07-GND1" V 5150 3700 50  0000 C CNN
F 1 "Conn_01x08_Female" H 4992 4094 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 5100 3700 50  0001 C CNN
F 3 "~" H 5100 3700 50  0001 C CNN
	1    5100 3700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3600 3650 4150 3650
Wire Wire Line
	4800 2950 4800 3250
Text Label 4800 3200 1    50   ~ 0
VCC
Wire Wire Line
	4800 3900 4800 4150
Wire Wire Line
	4800 4150 4150 4150
Wire Wire Line
	4150 4150 4150 3650
Connection ~ 4150 3650
Text Label 4800 4100 1    50   ~ 0
GND
Wire Wire Line
	6400 2650 6750 2650
$Comp
L Connector:Conn_01x06_Female J5
U 1 1 5E6F4C16
P 6550 4200
F 0 "J5" V 6750 4150 50  0000 L CNN
F 1 "PCF8563" V 6650 4000 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 6550 4200 50  0001 C CNN
F 3 "~" H 6550 4200 50  0001 C CNN
	1    6550 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	6100 2650 6100 2750
Wire Wire Line
	4800 2950 4800 2650
Wire Wire Line
	4800 2650 6100 2650
Connection ~ 4800 2950
Wire Wire Line
	4900 3900 4900 4250
Wire Wire Line
	4900 4450 4900 4600
Wire Wire Line
	4900 4600 4800 4600
Wire Wire Line
	4800 4600 4800 4150
Connection ~ 4800 4150
Text Label 4900 4200 1    50   ~ 0
GPIO15
Wire Wire Line
	5100 3900 5100 4250
Text Label 5100 4150 1    50   ~ 0
GPIO0
$Comp
L MCU_Microchip_ATtiny:ATtiny85-20PU U3
U 1 1 5E479636
P 8550 3700
F 0 "U3" H 8500 3750 50  0000 R CNN
F 1 "ATtiny85-20PU" H 8700 3850 50  0000 R CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 8550 3700 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-2586-avr-8-bit-microcontroller-attiny25-attiny45-attiny85_datasheet.pdf" H 8550 3700 50  0001 C CNN
	1    8550 3700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4150 4450 4150 4150
Connection ~ 8550 4450
Connection ~ 4150 4150
Wire Wire Line
	6400 3150 6400 2950
Wire Wire Line
	6400 3150 6550 3150
Wire Wire Line
	6550 3150 6550 4000
Wire Wire Line
	7600 3150 6550 3150
Wire Wire Line
	7600 3150 7600 3400
Connection ~ 6550 3150
Wire Wire Line
	7850 3500 6350 3500
Wire Wire Line
	6350 3500 6350 4000
Connection ~ 7850 3500
Wire Wire Line
	6450 4000 6450 3250
Wire Wire Line
	6450 3250 6100 3250
Wire Wire Line
	6100 3250 6100 3050
Connection ~ 6100 3050
Wire Wire Line
	6100 3050 6100 2950
Wire Wire Line
	7950 3600 7500 3600
Wire Wire Line
	7500 3600 7500 3250
Wire Wire Line
	7500 3250 6450 3250
Connection ~ 6450 3250
$Comp
L Connector:Conn_01x02_Female J2
U 1 1 5E75021A
P 2150 3400
F 0 "J2" V 2250 3400 50  0000 R CNN
F 1 "Battery" V 2350 3500 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical" H 2150 3400 50  0001 C CNN
F 3 "~" H 2150 3400 50  0001 C CNN
	1    2150 3400
	-1   0    0    1   
$EndComp
Wire Wire Line
	2350 3400 2350 3650
Wire Wire Line
	6650 4000 6650 3750
Wire Wire Line
	6650 3750 6100 3750
Wire Wire Line
	6100 3750 6100 4600
Wire Wire Line
	6100 4600 4900 4600
Connection ~ 4900 4600
Wire Wire Line
	6750 4000 6750 2650
Connection ~ 6750 2650
Wire Wire Line
	6750 2650 7400 2650
Wire Wire Line
	1700 3650 2350 3650
Wire Wire Line
	1700 1650 5000 1650
Connection ~ 2350 3650
Connection ~ 6400 3150
$Comp
L Connector:Conn_01x08_Female ESP07-VCC1
U 1 1 5E787715
P 5100 3450
F 0 "ESP07-VCC1" V 5150 3200 50  0000 L CNN
F 1 "Conn_01x08_Female" H 5128 3335 50  0001 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 5100 3450 50  0001 C CNN
F 3 "~" H 5100 3450 50  0001 C CNN
	1    5100 3450
	0    -1   1    0   
$EndComp
Text Label 6750 3950 1    50   ~ 0
VCC
Text Label 6650 3950 1    50   ~ 0
GND
Text Label 6550 3950 1    50   ~ 0
SDA
Text Label 6450 3950 1    50   ~ 0
SCL
Wire Wire Line
	1700 1650 1700 3650
Text Label 6350 3950 1    50   ~ 0
COT
Wire Wire Line
	4150 2950 4600 2950
Wire Wire Line
	4600 2950 4600 4750
Wire Wire Line
	4600 4750 5100 4750
Connection ~ 4600 2950
Wire Wire Line
	4600 2950 4800 2950
Wire Wire Line
	5100 4450 5100 4750
Wire Wire Line
	4150 4450 8550 4450
Wire Wire Line
	2350 3100 2350 3300
Wire Wire Line
	5200 3900 5200 4150
Wire Wire Line
	5200 4150 5750 4150
Wire Wire Line
	5750 4150 5750 3150
Wire Wire Line
	5750 3150 6400 3150
Text Label 5200 4050 1    50   ~ 0
SDA
Wire Wire Line
	5300 3900 5300 4050
Wire Wire Line
	5300 4050 5650 4050
Wire Wire Line
	5650 4050 5650 3050
Wire Wire Line
	5650 3050 6100 3050
Text Label 5300 4050 1    50   ~ 0
SCL
Text Label 5000 3250 1    50   ~ 0
GPIO12
Text Label 5500 3250 1    50   ~ 0
RST
Wire Wire Line
	5350 2450 6000 2450
Wire Wire Line
	4500 2450 5050 2450
Wire Wire Line
	5500 3250 5500 3000
Wire Wire Line
	5500 2800 4500 2800
Wire Wire Line
	4500 2800 4500 2450
Connection ~ 4500 2450
Wire Wire Line
	6000 2900 6000 2450
Wire Wire Line
	5200 3250 5200 3000
Wire Wire Line
	5200 3000 5500 3000
Connection ~ 5500 3000
Wire Wire Line
	5500 3000 5500 2800
Text Label 5200 3250 1    50   ~ 0
GPIO16
Wire Wire Line
	5000 3250 5000 2900
Wire Wire Line
	5000 2900 6000 2900
$EndSCHEMATC