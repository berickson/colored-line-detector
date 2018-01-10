EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:teensy
LIBS:brian_erickson
LIBS:schematic-cache
EELAYER 25 0
EELAYER END
$Descr USLetter 11000 8500
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
Text GLabel 7000 4250 0    60   Input ~ 0
LMotor-
Text GLabel 7000 4750 0    60   Input ~ 0
LMotor+
Text GLabel 7000 4350 0    60   Input ~ 0
GND
Text GLabel 7000 4450 0    60   Input ~ 0
3V3
Text GLabel 7000 4550 0    60   Input ~ 0
LEncoderA
Text GLabel 7000 4650 0    60   Input ~ 0
LEncoderB
Text GLabel 8650 4250 0    60   Input ~ 0
RMotor-
Text GLabel 8650 4750 0    60   Input ~ 0
LMotor+
Text GLabel 8650 4350 0    60   Input ~ 0
GND
Text GLabel 8650 4450 0    60   Input ~ 0
3V3
Text GLabel 8650 4550 0    60   Input ~ 0
REncoderA
Text GLabel 8650 4650 0    60   Input ~ 0
REncoderB
Text GLabel 2500 6100 1    60   Input ~ 0
GND
Text GLabel 2600 6100 1    60   Input ~ 0
3V3
Text GLabel 2700 6100 1    60   Input ~ 0
OLED_SCK
Text GLabel 2800 6100 1    60   Input ~ 0
OLED_MOSI
Text GLabel 2900 6100 1    60   Input ~ 0
OLED_RESET
Text GLabel 3000 6100 1    60   Input ~ 0
OLED_DC
Text GLabel 3100 6100 1    60   Input ~ 0
OLED_CS
Text GLabel 5850 1050 0    60   Input ~ 0
GND
Text GLabel 9250 3550 0    60   Input ~ 0
BAT-
$Comp
L Teensy3.2_outer_pins U1
U 1 1 5A4FD2BC
P 4950 2300
F 0 "U1" H 4950 3100 60  0000 C CNN
F 1 "Teensy3.2_outer_pins" H 4950 1350 60  0000 C CNN
F 2 "" H 4950 2300 60  0000 C CNN
F 3 "" H 4950 2300 60  0000 C CNN
	1    4950 2300
	1    0    0    -1  
$EndComp
Text GLabel 3950 1700 0    60   Input ~ 0
GND
Text GLabel 5950 1700 2    60   Input ~ 0
5V
Text GLabel 5950 1900 2    60   Input ~ 0
3V3
$Comp
L ENCODER_MOTOR L_MTR1
U 1 1 5A4FD5AE
P 7350 4450
F 0 "L_MTR1" H 7350 4800 60  0000 C CNN
F 1 "ENCODER_MOTOR" H 7400 4000 60  0000 C CNN
F 2 "" H 7300 4050 60  0001 C CNN
F 3 "" H 7300 4050 60  0001 C CNN
	1    7350 4450
	1    0    0    -1  
$EndComp
$Comp
L ENCODER_MOTOR R_MTR1
U 1 1 5A4FD6E6
P 9000 4450
F 0 "R_MTR1" H 9000 4800 60  0000 C CNN
F 1 "ENCODER_MOTOR" H 9050 4000 60  0000 C CNN
F 2 "" H 8950 4050 60  0001 C CNN
F 3 "" H 8950 4050 60  0001 C CNN
	1    9000 4450
	1    0    0    -1  
$EndComp
$Comp
L OLED_SSD1331 OLED1
U 1 1 5A4FDAFF
P 2750 6450
F 0 "OLED1" H 2250 6300 60  0000 C CNN
F 1 "OLED_SSD1331" H 2800 5900 60  0000 C CNN
F 2 "" V 3100 6500 60  0001 C CNN
F 3 "" V 3100 6500 60  0001 C CNN
	1    2750 6450
	1    0    0    -1  
$EndComp
Text GLabel 5950 3000 2    60   Input ~ 0
OLED_CLK
Text GLabel 3950 2900 0    60   Input ~ 0
OLED_MOSI
Text GLabel 3950 2800 0    60   Input ~ 0
OLED_CS
Text GLabel 3950 2600 0    60   Input ~ 0
OLED_DC
Text GLabel 3950 2700 0    60   Input ~ 0
OLED_RESET
$Comp
L JEVOIS CAM1
U 1 1 5A4FDEE7
P 2250 2100
F 0 "CAM1" H 2250 2450 60  0000 C CNN
F 1 "JEVOIS" H 2250 1700 60  0000 C CNN
F 2 "" H 2100 2200 60  0001 C CNN
F 3 "" H 2100 2200 60  0001 C CNN
	1    2250 2100
	-1   0    0    -1  
$EndComp
Text GLabel 2900 2000 2    60   Input ~ 0
GND
Text GLabel 2900 1900 2    60   Input ~ 0
3V3
Text GLabel 1600 2200 0    60   Input ~ 0
5V
Text GLabel 1600 2300 0    60   Input ~ 0
GND
Text GLabel 2900 2200 2    60   Input ~ 0
CAM_TX
Text GLabel 2900 2100 2    60   Input ~ 0
CAM_RX
Text GLabel 3950 1800 0    60   Input ~ 0
CAM_TX
Text GLabel 3950 1900 0    60   Input ~ 0
CAM_RX
$Comp
L MOTOR_CONTROLLER_WANGDD22 MCC1
U 1 1 5A4FE9B8
P 2750 3750
F 0 "MCC1" H 2750 4300 60  0000 C CNN
F 1 "MOTOR_CONTROLLER_WANGDD22" H 2750 3250 60  0000 C CNN
F 2 "" H 2500 3550 60  0001 C CNN
F 3 "" H 2500 3550 60  0001 C CNN
	1    2750 3750
	1    0    0    -1  
$EndComp
Text GLabel 2100 3650 0    60   Input ~ 0
L_FWD
Text GLabel 2100 3750 0    60   Input ~ 0
L_REV
Text GLabel 2100 3900 0    60   Input ~ 0
R_FWD
Text GLabel 2100 4000 0    60   Input ~ 0
R_REV
Text GLabel 3500 3650 2    60   Input ~ 0
LMotor-
Text GLabel 3500 3550 2    60   Input ~ 0
LMotor+
Text GLabel 3500 3900 2    60   Input ~ 0
RMotor-
Text GLabel 3500 3800 2    60   Input ~ 0
LMotor+
Text GLabel 2100 3400 0    60   Input ~ 0
BAT+
Text GLabel 2100 3500 0    60   Input ~ 0
BAT-
Text GLabel 3950 2300 0    60   Input ~ 0
L_FWD
Text GLabel 3950 2400 0    60   Input ~ 0
L_REV
Text GLabel 5950 2100 2    60   Input ~ 0
R_FWD
Text GLabel 5950 2200 2    60   Input ~ 0
R_REV
Text GLabel 5950 2600 2    60   Input ~ 0
LEncoderA
Text GLabel 5950 2700 2    60   Input ~ 0
LEncoderB
Text GLabel 5950 2800 2    60   Input ~ 0
REncoderA
Text GLabel 5950 2900 2    60   Input ~ 0
REncoderB
NoConn ~ 5950 1800
$Comp
L 5V_UBEC PS1
U 1 1 5A4FF28B
P 6500 1050
F 0 "PS1" H 6500 1300 60  0000 C CNN
F 1 "5V_UBEC" H 6500 900 60  0000 C CNN
F 2 "" H 6500 1050 60  0001 C CNN
F 3 "" H 6500 1050 60  0001 C CNN
	1    6500 1050
	1    0    0    -1  
$EndComp
$Comp
L 3S_LIPO B1
U 1 1 5A4FF30E
P 9850 3350
F 0 "B1" H 9850 3850 60  0000 C CNN
F 1 "3S_LIPO" H 9850 2800 60  0000 C CNN
F 2 "" H 9850 3350 60  0001 C CNN
F 3 "" H 9850 3350 60  0001 C CNN
	1    9850 3350
	1    0    0    1   
$EndComp
Text GLabel 9250 3650 0    60   Input ~ 0
BAT+
Text GLabel 8200 1550 2    60   Input ~ 0
BAT+
Text GLabel 7100 1050 2    60   Input ~ 0
BAT-
Text GLabel 5850 950  0    60   Input ~ 0
5V
$Comp
L BALANCE_ALARM U2
U 1 1 5A4FFA0F
P 9500 1250
F 0 "U2" H 9500 1700 60  0000 C CNN
F 1 "BALANCE_ALARM" H 9500 1100 60  0000 C CNN
F 2 "" H 9500 1250 60  0001 C CNN
F 3 "" H 9500 1250 60  0001 C CNN
	1    9500 1250
	1    0    0    -1  
$EndComp
Text GLabel 9250 3050 0    60   Input ~ 0
BALANCE_GND
Text GLabel 9250 3150 0    60   Input ~ 0
BALANCE_1
Text GLabel 9250 3250 0    60   Input ~ 0
BALANCE_2
Text GLabel 9950 1800 3    60   Input ~ 0
BALANCE_GND
Text GLabel 9850 1800 3    60   Input ~ 0
BALANCE_1
Text GLabel 9750 1800 3    60   Input ~ 0
BALANCE_2
NoConn ~ 9650 1800
NoConn ~ 9550 1800
NoConn ~ 9450 1800
NoConn ~ 9350 1800
NoConn ~ 9250 1800
NoConn ~ 9150 1800
NoConn ~ 9050 1800
NoConn ~ 3950 2000
NoConn ~ 3950 2100
NoConn ~ 3950 2200
NoConn ~ 3950 2500
NoConn ~ 3950 3000
$Comp
L SW_DIP_x01 SW1
U 1 1 5A5114F1
P 7900 1550
F 0 "SW1" H 7900 1700 50  0000 C CNN
F 1 "SW_DIP_x01" H 7900 1400 50  0000 C CNN
F 2 "" H 7900 1550 50  0001 C CNN
F 3 "" H 7900 1550 50  0001 C CNN
	1    7900 1550
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5A511658
P 7450 1850
F 0 "R1" V 7530 1850 50  0000 C CNN
F 1 "400k" V 7450 1850 50  0000 C CNN
F 2 "" V 7380 1850 50  0001 C CNN
F 3 "" H 7450 1850 50  0001 C CNN
	1    7450 1850
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5A511723
P 7450 2200
F 0 "R2" V 7530 2200 50  0000 C CNN
F 1 "100k" V 7450 2200 50  0000 C CNN
F 2 "" V 7380 2200 50  0001 C CNN
F 3 "" H 7450 2200 50  0001 C CNN
	1    7450 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5A511792
P 7450 2350
F 0 "#PWR01" H 7450 2100 50  0001 C CNN
F 1 "GND" H 7450 2200 50  0000 C CNN
F 2 "" H 7450 2350 50  0001 C CNN
F 3 "" H 7450 2350 50  0001 C CNN
	1    7450 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 2000 7450 2050
Wire Wire Line
	7600 1550 7450 1550
Wire Wire Line
	7450 950  7450 1700
Wire Wire Line
	7450 2000 5950 2000
Wire Wire Line
	7100 950  7700 950 
Connection ~ 7450 1550
Connection ~ 7450 950 
Text Notes 6050 2450 0    60   ~ 0
<- future: gyro i2c, \n<- interrupt\n
NoConn ~ 5950 2400
NoConn ~ 5950 2500
NoConn ~ 5950 2300
$Comp
L PWR_FLAG #FLG02
U 1 1 5A51263B
P 7700 950
F 0 "#FLG02" H 7700 1025 50  0001 C CNN
F 1 "PWR_FLAG" H 7700 1100 50  0000 C CNN
F 2 "" H 7700 950 50  0001 C CNN
F 3 "" H 7700 950 50  0001 C CNN
	1    7700 950 
	1    0    0    -1  
$EndComp
Text Notes 1100 5950 0    60   ~ 0
GND: Power ground\nVCC: 2.8-5.5V power supply\nD0: CLK clock\nD1: MOSI data\nRST: Reset\nDC: Data / command\nCS: Chip-select signal
$EndSCHEMATC
