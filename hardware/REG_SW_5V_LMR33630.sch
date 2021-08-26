EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
Title "Switching Regulator (5V)"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Regulators:LMR33630ADDA U?
U 1 1 60FF8859
P 5450 3600
AR Path="/60FF8859" Ref="U?"  Part="1" 
AR Path="/60F4CB3B/60FF8859" Ref="U11"  Part="1" 
F 0 "U11" H 5450 4067 50  0000 C CNN
F 1 "LMR33630ADDA" H 5450 3976 50  0000 C CNN
F 2 "NANO:Texas_HSOP-8-1EP_3.9x4.9mm_P1.27mm_ThermalVias" H 5450 2800 50  0001 C CNN
F 3 "" H 5450 3500 50  0001 C CNN
F 4 "C841384" H 5450 3600 50  0001 C CNN "LCSC"
	1    5450 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60FF8866
P 3600 3850
AR Path="/60FF8866" Ref="#PWR?"  Part="1" 
AR Path="/60F4CB3B/60FF8866" Ref="#PWR027"  Part="1" 
F 0 "#PWR027" H 3600 3600 50  0001 C CNN
F 1 "GND" H 3605 3677 50  0000 C CNN
F 2 "" H 3600 3850 50  0001 C CNN
F 3 "" H 3600 3850 50  0001 C CNN
	1    3600 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C Cin?
U 1 1 60FF886D
P 4250 3550
AR Path="/60FF886D" Ref="Cin?"  Part="1" 
AR Path="/60F4CB3B/60FF886D" Ref="Cin1"  Part="1" 
F 0 "Cin1" V 4200 3700 50  0000 C CNN
F 1 "10uf" V 4300 3700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4288 3400 50  0001 C CNN
F 3 "" H 4250 3550 50  0001 C CNN
F 4 "C440198" H 4250 3550 50  0001 C CNN "LCSC"
	1    4250 3550
	-1   0    0    1   
$EndComp
$Comp
L Device:C Chf?
U 1 1 60FF8874
P 4000 3550
AR Path="/60FF8874" Ref="Chf?"  Part="1" 
AR Path="/60F4CB3B/60FF8874" Ref="Chf1"  Part="1" 
F 0 "Chf1" V 3950 3700 50  0000 C CNN
F 1 "220nf" V 4050 3750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4038 3400 50  0001 C CNN
F 3 "" H 4000 3550 50  0001 C CNN
F 4 "C5378" H 4000 3550 50  0001 C CNN "LCSC"
	1    4000 3550
	-1   0    0    1   
$EndComp
$Comp
L Device:C Cboot?
U 1 1 60FF8885
P 6000 3350
AR Path="/60FF8885" Ref="Cboot?"  Part="1" 
AR Path="/60F4CB3B/60FF8885" Ref="Cboot1"  Part="1" 
F 0 "Cboot1" H 6115 3396 50  0000 L CNN
F 1 "0.1uf" H 6115 3305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6038 3200 50  0001 C CNN
F 3 "" H 6000 3350 50  0001 C CNN
F 4 "C1525 " H 6000 3350 50  0001 C CNN "LCSC"
	1    6000 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3500 6000 3500
Wire Wire Line
	6000 3200 5850 3200
Wire Wire Line
	5850 3200 5850 3400
$Comp
L Device:L_Small L?
U 1 1 60FF888F
P 6500 3600
AR Path="/60FF888F" Ref="L?"  Part="1" 
AR Path="/60F4CB3B/60FF888F" Ref="L1"  Part="1" 
F 0 "L1" H 6548 3646 50  0000 L CNN
F 1 "8uH, 14m" H 6548 3555 50  0000 L CNN
F 2 "Inductor_SMD:L_Bourns_SRP7028A_7.3x6.6mm" H 6500 3600 50  0001 C CNN
F 3 "" H 6500 3600 50  0001 C CNN
F 4 "C408450" H 6500 3600 50  0001 C CNN "LCSC"
	1    6500 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 4000 5450 4000
$Comp
L power:VCC #PWR?
U 1 1 60FF889B
P 7350 3700
AR Path="/60FF889B" Ref="#PWR?"  Part="1" 
AR Path="/60F4CB3B/60FF889B" Ref="#PWR030"  Part="1" 
F 0 "#PWR030" H 7350 3550 50  0001 C CNN
F 1 "VCC" H 7367 3873 50  0000 C CNN
F 2 "" H 7350 3700 50  0001 C CNN
F 3 "" H 7350 3700 50  0001 C CNN
	1    7350 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C Cout?
U 1 1 60FF88A5
P 6750 3850
AR Path="/60FF88A5" Ref="Cout?"  Part="1" 
AR Path="/60F4CB3B/60FF88A5" Ref="Cout1"  Part="1" 
F 0 "Cout1" H 6650 3950 50  0000 L CNN
F 1 "22uF" H 6650 3750 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6788 3700 50  0001 C CNN
F 3 "" H 6750 3850 50  0001 C CNN
F 4 "C45783" H 6750 3850 50  0001 C CNN "LCSC"
	1    6750 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C Cout?
U 1 1 60FF88AC
P 6950 3850
AR Path="/60FF88AC" Ref="Cout?"  Part="1" 
AR Path="/60F4CB3B/60FF88AC" Ref="Cout2"  Part="1" 
F 0 "Cout2" H 6850 3950 50  0000 L CNN
F 1 "22uF" H 6850 3750 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6988 3700 50  0001 C CNN
F 3 "" H 6950 3850 50  0001 C CNN
F 4 "C45783" H 6950 3850 50  0001 C CNN "LCSC"
	1    6950 3850
	1    0    0    -1  
$EndComp
Connection ~ 6950 3700
Wire Wire Line
	6950 3700 6750 3700
$Comp
L Device:C Cout?
U 1 1 60FF88B5
P 7150 3850
AR Path="/60FF88B5" Ref="Cout?"  Part="1" 
AR Path="/60F4CB3B/60FF88B5" Ref="Cout3"  Part="1" 
F 0 "Cout3" H 7050 3950 50  0000 L CNN
F 1 "22uF" H 7050 3750 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7188 3700 50  0001 C CNN
F 3 "" H 7150 3850 50  0001 C CNN
F 4 "C45783" H 7150 3850 50  0001 C CNN "LCSC"
	1    7150 3850
	1    0    0    -1  
$EndComp
Connection ~ 7150 3700
Wire Wire Line
	7150 3700 6950 3700
$Comp
L Device:C Cout?
U 1 1 60FF88BE
P 7350 3850
AR Path="/60FF88BE" Ref="Cout?"  Part="1" 
AR Path="/60F4CB3B/60FF88BE" Ref="Cout4"  Part="1" 
F 0 "Cout4" H 7250 3950 50  0000 L CNN
F 1 "22uF" H 7250 3750 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7388 3700 50  0001 C CNN
F 3 "" H 7350 3850 50  0001 C CNN
F 4 "C45783" H 7350 3850 50  0001 C CNN "LCSC"
	1    7350 3850
	1    0    0    -1  
$EndComp
Connection ~ 7350 3700
Wire Wire Line
	7350 3700 7150 3700
$Comp
L Device:R Rfbt?
U 1 1 60FF88C8
P 6300 3700
AR Path="/60FF88C8" Ref="Rfbt?"  Part="1" 
AR Path="/60F4CB3B/60FF88C8" Ref="Rfbt1"  Part="1" 
F 0 "Rfbt1" V 6400 3700 50  0000 C CNN
F 1 "100k 1%" V 6200 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6230 3700 50  0001 C CNN
F 3 "" H 6300 3700 50  0001 C CNN
F 4 "C25741" H 6300 3700 50  0001 C CNN "LCSC"
	1    6300 3700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6150 3700 6000 3700
$Comp
L Device:R Rfbb?
U 1 1 60FF88D2
P 6000 3850
AR Path="/60FF88D2" Ref="Rfbb?"  Part="1" 
AR Path="/60F4CB3B/60FF88D2" Ref="Rfbb1"  Part="1" 
F 0 "Rfbb1" H 6050 3800 50  0000 L CNN
F 1 "24K9 1%" H 6050 3700 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5930 3850 50  0001 C CNN
F 3 "" H 6000 3850 50  0001 C CNN
F 4 "C25874" H 6000 3850 50  0001 C CNN "LCSC"
	1    6000 3850
	1    0    0    -1  
$EndComp
Connection ~ 6000 3700
Wire Wire Line
	6000 3700 5850 3700
$Comp
L power:GND #PWR?
U 1 1 60FF88DA
P 6000 4000
AR Path="/60FF88DA" Ref="#PWR?"  Part="1" 
AR Path="/60F4CB3B/60FF88DA" Ref="#PWR031"  Part="1" 
F 0 "#PWR031" H 6000 3750 50  0001 C CNN
F 1 "GND" H 6005 3827 50  0000 C CNN
F 2 "" H 6000 4000 50  0001 C CNN
F 3 "" H 6000 4000 50  0001 C CNN
	1    6000 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3600 5050 3400
Wire Wire Line
	3600 3400 3800 3400
$Comp
L Device:D_TVS D?
U 1 1 60FF88E5
P 3800 3550
AR Path="/60FF88E5" Ref="D?"  Part="1" 
AR Path="/60F4CB3B/60FF88E5" Ref="D20"  Part="1" 
F 0 "D20" V 4000 3650 50  0000 R CNN
F 1 "15V standoff" V 3550 3800 50  0000 R CNN
F 2 "Diode_SMD:D_SMB" H 3800 3550 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/MDD-Jiangsu-Yutai-Elec-SMBJ15CA_C113989.pdf" H 3800 3550 50  0001 C CNN
F 4 "C113989" V 3800 3550 50  0001 C CNN "LCSC"
	1    3800 3550
	0    -1   1    0   
$EndComp
Connection ~ 3800 3400
$Comp
L Device:CP1_Small C?
U 1 1 60FF88F5
P 4650 3500
AR Path="/60FF88F5" Ref="C?"  Part="1" 
AR Path="/60F4CB3B/60FF88F5" Ref="C17"  Part="1" 
F 0 "C17" V 4450 3500 50  0000 C CNN
F 1 "330uf" V 4550 3500 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P3.50mm" H 4650 3500 50  0001 C CNN
F 3 "" H 4650 3500 50  0001 C CNN
	1    4650 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 60FF88FE
P 4950 3800
AR Path="/60FF88FE" Ref="C?"  Part="1" 
AR Path="/60F4CB3B/60FF88FE" Ref="C18"  Part="1" 
F 0 "C18" V 4721 3800 50  0000 C CNN
F 1 "1 uF" V 4812 3800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4950 3800 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/lcsc/1810191216_Samsung-Electro-Mechanics-CL21B105KBFNNNE_C28323.pdf" H 4950 3800 50  0001 C CNN
F 4 "C28323" H 4950 3800 50  0001 C CNN "LCSC"
	1    4950 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	3800 3400 4000 3400
Wire Wire Line
	4000 3400 4250 3400
Connection ~ 4000 3400
Connection ~ 4250 3400
Wire Wire Line
	4250 3400 4650 3400
Wire Wire Line
	3600 3700 3800 3700
Wire Wire Line
	3600 3700 3600 3850
Connection ~ 3800 3700
Wire Wire Line
	3800 3700 4000 3700
Connection ~ 4000 3700
Wire Wire Line
	4000 3700 4250 3700
Wire Wire Line
	4250 3700 4650 3700
Wire Wire Line
	4650 3700 4650 3600
Connection ~ 4250 3700
Wire Wire Line
	5450 4000 4850 4000
Wire Wire Line
	4850 4000 4850 3800
Connection ~ 5450 4000
Wire Wire Line
	6450 3700 6500 3700
Connection ~ 6750 3700
Wire Wire Line
	5550 4000 6000 4000
Connection ~ 5550 4000
Connection ~ 6000 4000
Wire Wire Line
	6000 4000 6750 4000
Connection ~ 6750 4000
Wire Wire Line
	6750 4000 6950 4000
Connection ~ 6950 4000
Wire Wire Line
	6950 4000 7150 4000
Connection ~ 7150 4000
Wire Wire Line
	7150 4000 7350 4000
Wire Wire Line
	4650 3400 5050 3400
Connection ~ 4650 3400
Connection ~ 5050 3400
Text HLabel 7350 3700 2    50   Input ~ 0
VCC
Connection ~ 6500 3700
Wire Wire Line
	6500 3700 6750 3700
Wire Wire Line
	6500 3500 6000 3500
Connection ~ 6000 3500
Text HLabel 3600 3400 0    50   Input ~ 0
VIN
$EndSCHEMATC
