;===================================================================================================
	.title	A60
;
;	Copyright 2013 Akimitsu Sadoi - The LED Artist - http://theLEDart.com
;	The use and distribution of this code without explicit permission is prohibited.
;---------------------------------------------------------------------------------------------------
;	Version history
;	0.0	ported from A12 ver 1.1
;	0.1	animate_LED routine LED value update bug fix - occasional glitch in fades eliminated
;	0.2	PWM drive method changed to reduce color bleeding
;	0.3	New patterns added
;	0.4	PWM timing parameters corrected
;	0.4b	PWM clock source changed to system clock (Tcy) - 16 MHz
;	0.4c	random/shuffle mode in demo mode
;	0.4d	production program test mode - disable switch press and run test sequence once after EEPROM erase
;===================================================================================================

	.include "P24F08KM204.INC"
;	.include "P24F16KM204.INC"
	.list	b=4
	.ascii	"A60 Ver.0.4d Copyright 2013 Akimitsu Sadoi aki@theledart.com"

;===================================================================================================
;	device config bits
;
	config __FGS,	GWRP_OFF & GCP_ON	; General Segment Code Protect on
	config __FOSCSEL, FNOSC_LPRC & SOSCSRC_DIG & LPRCSEL_LP & IESO_OFF	; low power RC osc
	config __FOSC,	POSCMOD_NONE & OSCIOFNC_IO & SOSCSEL_SOSCLP & FCKSM_CSECMD
	config __FWDT,	WDTPS_PS1 & FWPSA_PR32 & FWDTEN_OFF & WINDIS_OFF	; WDT disabled
;	config __FPOR,	BOREN_BOR3 & PWRTEN_ON & BORV_V18 & MCLRE_OFF	; BOR always on, MCLR off
	config __FPOR,	BOREN_BOR3 & PWRTEN_ON & BORV_V18 & MCLRE_ON	; BOR always on, MCLR on
	config __FICD,	ICS_PGx2	; debug pins on PGx2

;===================================================================================================
;	constants
;
	.global __reset
	.global __CCT1Interrupt				; CCP1 ISR entry - PWM LED driver service

;---------------------------------------------------------------------------------------------------
;
	.equ	ALWAYS_DEMO_MODE, 0

	.equ	clock_speed,32				; either 8, 16 or 32 MHz
	.equ	osc_adjust,	0				; value to calibrate internal oscillator (-32 ~ +31)

	.equ	speed_norm,0x7FFF			; speed adjust center/normal value

	.equ	num_modes,(mode_tbl_end-mode_tbl)/2-1

	;--- auto advance/shutoff timer setting --------------------------
	.equ	auto_adv_time, 60			; auto advance time in seconds (max: 126)
	.equ	demo_tics, (auto_adv_time*512)

	;--- COL & ROW drive parameters ----------------------------------
	.equ	COL_POL, 1					; 0:active-low 1:active-high
	.equ	ROW_POL, 1					; 0:active-low 1:active-high
	
	.equ	num_COLs, 30				; number of COLs
	.equ	num_ROWs, 2					; number of RGB ROWs
	.equ	num_LEDs, num_COLs*num_ROWs	; number of LEDs

	;--- PWM parameters ----------------------------------------------

	.equ	PWM_TYPE, 1								; 1: raising edge stationary, 0: falling edge stationary
.if (ROW_POL)
	.equ	ccp1con1L_param, 0b1000000000000101		; module on, Tcy clock, no prescale, PWM mode
;	.equ	ccp1con1L_param, 0b1000010000000101		; module on, Fosc clock, no prescale, PWM mode
	.equ	ccp1con3H_param, 0b0000000000000000		; Steerable Single Output mode, active high output
.else
	.equ	ccp1con1L_param, 0b1000000000000101		; module on, Tcy clock, no prescale, PWM mode
	.equ	ccp1con3H_param, 0b0000000000110000		; Steerable Single Output mode, active low output
.endif
	.equ	max_duty, 0xFF							; duty cycle value for 100% duty (8 bit)
	.equ	port_delay,	16							; delay time (in Tcy) between timer INT and port set
	.equ	pr_value, 299							; PWM freq. = clock_speed*500/(pr_value+1) kHz
;	.equ	pr_value, 599							; PWM freq. = clock_speed*1000/(pr_value+1) kHz
													; LED ref. rate = clock_speed*1000000/763(pr_value+1) Hz

	;--- switch parameters -------------------------------------------
	.equ	debounce_time, 16						; (up to 16) x 2.048 mS
	.equ	debounce_bits, (1<<debounce_time-1)
	.equ	long_push_time, 250						; x 2.048 mS

	;--- port & pin mapping for I/O ----------------------------------

	; switch 1 is connected to RB1/CN5/AN3
	.equ	SW1_PORT, PORTB
	.equ	SW1_TRIS, TRISB
	.equ	SW1_ANS, ANSB
	.equ	SW1_BIT, 1
	.equ	SW1_CN, 5

	;--- LEDs --------------------------------------------------------
	.equ	LED_1_PORT,_RC_
	.equ	LED_1_PIN,2

	.equ	LED_2_PORT,_RC_
	.equ	LED_2_PIN,1

	.equ	LED_3_PORT,_RC_
	.equ	LED_3_PIN,0

	.equ	LED_4_PORT,_RB_
	.equ	LED_4_PIN,3

	.equ	LED_5_PORT,_RB_
	.equ	LED_5_PIN,2

	.equ	LED_6_PORT,_RB_
	.equ	LED_6_PIN,0

	.equ	LED_7_PORT,_RA_
	.equ	LED_7_PIN,1

	.equ	LED_8_PORT,_RA_
	.equ	LED_8_PIN,0

	.equ	LED_9_PORT,_RB_
	.equ	LED_9_PIN,15

	.equ	LED_10_PORT,_RB_
	.equ	LED_10_PIN,14

	.equ	LED_11_PORT,_RA_
	.equ	LED_11_PIN,11

	.equ	LED_12_PORT,_RA_
	.equ	LED_12_PIN,10

	.equ	LED_13_PORT,_RB_
	.equ	LED_13_PIN,12

	.equ	LED_14_PORT,_RB_
	.equ	LED_14_PIN,11

	.equ	LED_15_PORT,_RA_
	.equ	LED_15_PIN,7

	.equ	LED_16_PORT,_RC_
	.equ	LED_16_PIN,9

	.equ	LED_17_PORT,_RC_
	.equ	LED_17_PIN,8

	.equ	LED_18_PORT,_RC_
	.equ	LED_18_PIN,7

	.equ	LED_19_PORT,_RC_
	.equ	LED_19_PIN,6

	.equ	LED_20_PORT,_RB_
	.equ	LED_20_PIN,9

	.equ	LED_21_PORT,_RB_
	.equ	LED_21_PIN,7

	.equ	LED_22_PORT,_RC_
	.equ	LED_22_PIN,5

	.equ	LED_23_PORT,_RC_
	.equ	LED_23_PIN,4

	.equ	LED_24_PORT,_RC_
	.equ	LED_24_PIN,3

	.equ	LED_25_PORT,_RA_
	.equ	LED_25_PIN,9

	.equ	LED_26_PORT,_RA_
	.equ	LED_26_PIN,4

	.equ	LED_27_PORT,_RB_
	.equ	LED_27_PIN,4

	.equ	LED_28_PORT,_RA_
	.equ	LED_28_PIN,8

	.equ	LED_29_PORT,_RA_
	.equ	LED_29_PIN,3

	.equ	LED_30_PORT,_RA_
	.equ	LED_30_PIN,2

	.equ	PORTA_bits, 12				; highest port # used + 1
	.equ	PORTB_bits, 16
	.equ	PORTC_bits, 10

	;--- PWM R/G/B ports ---
	.equ	PWM_R1_LAT,LATB
	.equ	PWM_R1_PIN,5			; RB5
	.equ	PWM_R1_ST,OCEEN

	.equ	PWM_G1_LAT,LATB
	.equ	PWM_G1_PIN,6			; RB6
	.equ	PWM_G1_ST,OCFEN

	.equ	PWM_B1_LAT,LATB
	.equ	PWM_B1_PIN,10			; RB10
	.equ	PWM_B1_ST,OCCEN

	.equ	PWM_R2_LAT,LATB
	.equ	PWM_R2_PIN,13			; RB13
	.equ	PWM_R2_ST,OCDEN

	.equ	PWM_G2_LAT,LATA
	.equ	PWM_G2_PIN,6			; RA6
	.equ	PWM_G2_ST,OCAEN

	.equ	PWM_B2_LAT,LATB
	.equ	PWM_B2_PIN,8			; RB8
	.equ	PWM_B2_ST,OCBEN

	.equ	_RA_,0
	.equ	_RB_,(PORTA_bits*2)
	.equ	_RC_,((PORTA_bits+PORTB_bits)*2)

	.text
;--- LED data -> duty_buff offset lookup table ---
LED_pins:
	.byte	LED_1_PIN*2+LED_1_PORT		; LED 1
	.byte	LED_1_PIN*2+LED_1_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 2
	.byte	LED_2_PIN*2+LED_2_PORT		; LED 3
	.byte	LED_2_PIN*2+LED_2_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 4
	.byte	LED_3_PIN*2+LED_3_PORT		; LED 5
	.byte	LED_3_PIN*2+LED_3_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 6
	.byte	LED_4_PIN*2+LED_4_PORT		; LED 7
	.byte	LED_4_PIN*2+LED_4_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 8
	.byte	LED_5_PIN*2+LED_5_PORT		; LED 9
	.byte	LED_5_PIN*2+LED_5_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 10
	.byte	LED_6_PIN*2+LED_6_PORT		; LED 11
	.byte	LED_6_PIN*2+LED_6_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 12
	.byte	LED_7_PIN*2+LED_7_PORT		; LED 13
	.byte	LED_7_PIN*2+LED_7_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 14
	.byte	LED_8_PIN*2+LED_8_PORT		; LED 15
	.byte	LED_8_PIN*2+LED_8_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 16
	.byte	LED_9_PIN*2+LED_9_PORT		; LED 17
	.byte	LED_9_PIN*2+LED_9_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 18
	.byte	LED_10_PIN*2+LED_10_PORT	; LED 19
	.byte	LED_10_PIN*2+LED_10_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 20
	.byte	LED_11_PIN*2+LED_11_PORT	; LED 21
	.byte	LED_11_PIN*2+LED_11_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 22
	.byte	LED_12_PIN*2+LED_12_PORT	; LED 23
	.byte	LED_12_PIN*2+LED_12_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 24
	.byte	LED_13_PIN*2+LED_13_PORT	; LED 25
	.byte	LED_13_PIN*2+LED_13_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 26
	.byte	LED_14_PIN*2+LED_14_PORT	; LED 27
	.byte	LED_14_PIN*2+LED_14_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 28
	.byte	LED_15_PIN*2+LED_15_PORT	; LED 29
	.byte	LED_15_PIN*2+LED_15_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 30
	.byte	LED_16_PIN*2+LED_16_PORT	; LED 31
	.byte	LED_16_PIN*2+LED_16_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 32
	.byte	LED_17_PIN*2+LED_17_PORT	; LED 33
	.byte	LED_17_PIN*2+LED_17_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 34
	.byte	LED_18_PIN*2+LED_18_PORT	; LED 35
	.byte	LED_18_PIN*2+LED_18_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 36
	.byte	LED_19_PIN*2+LED_19_PORT	; LED 37
	.byte	LED_19_PIN*2+LED_19_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 38
	.byte	LED_20_PIN*2+LED_20_PORT	; LED 39
	.byte	LED_20_PIN*2+LED_20_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 40
	.byte	LED_21_PIN*2+LED_21_PORT	; LED 41
	.byte	LED_21_PIN*2+LED_21_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 42
	.byte	LED_22_PIN*2+LED_22_PORT	; LED 43
	.byte	LED_22_PIN*2+LED_22_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 44
	.byte	LED_23_PIN*2+LED_23_PORT	; LED 45
	.byte	LED_23_PIN*2+LED_23_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 46
	.byte	LED_24_PIN*2+LED_24_PORT	; LED 47
	.byte	LED_24_PIN*2+LED_24_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 48
	.byte	LED_25_PIN*2+LED_25_PORT	; LED 49
	.byte	LED_25_PIN*2+LED_25_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 50
	.byte	LED_26_PIN*2+LED_26_PORT	; LED 51
	.byte	LED_26_PIN*2+LED_26_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 52
	.byte	LED_27_PIN*2+LED_27_PORT	; LED 53
	.byte	LED_27_PIN*2+LED_27_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 54
	.byte	LED_28_PIN*2+LED_28_PORT	; LED 55
	.byte	LED_28_PIN*2+LED_28_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 56
	.byte	LED_29_PIN*2+LED_29_PORT	; LED 57
	.byte	LED_29_PIN*2+LED_29_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 58
	.byte	LED_30_PIN*2+LED_30_PORT	; LED 59
	.byte	LED_30_PIN*2+LED_30_PORT+(PORTA_bits+PORTB_bits+PORTC_bits)*2	; LED 60

;===================================================================================================
;	variables

	.bss

mode_num:	.space 2				; mode (movment) number

random_val:	.space 2

var_start:							; top of variables to initialize

auto_adv_count:	.space 2			; auto-advance/shut down timer counter

;--- global flags ----------------------
g_flags:	.space 2
	do_not_read = 0					; LED data being modified - do not display
	HSV_mode = 1					; HSV color mode
	PWM_clock = 2					; PWM update clock to time main loop
	demo_mode = 3					; demo mode
	ignore_switch = 4				; ignore switch - used for testing

;--- button data -----------------------
btn_data:							; button sample data
	button_flags = 0				; button flags
		btn_down = 0				; button is held down
		btn_long = 1				; long push
		btn_push = 2				; button has been pushed (long or short)
	samples = 2						; button status samples
	long_push = 4					; button long push counter

btnA_data:	.space 6

;--- animation parameters --------------
update_rate:	.space 2			; overall animation speed
update_cnt:		.space 2			; keeps track of animation update timing

anim_params:
	anim_data = 0					; start of LED data address
	anim_LEDs = 2					; number of LEDs used in this animation
	anim_delay = 4
	anim_duty_diff = 6
	anim_step_up = 8
	anim_step_down = 10
	anim_max_duty = 12
	anim_update_rate = 14
	anim_update = 16
	anim_release_waitH = 18			; duty_hold at high duty will be released after this wait
	anim_release_waitL = 20			; duty_hold at low duty will be released after this wait
	anim_flags = 22
		af_direction = 0			; 0=forward, 1=reverse
		af_duty_hold_rs = 1			; DUTY_HOLD_RS
	anim_preprocess = 24
	anim_release_cnt = 26			; DUTY_HOLD release counter for each LED

anim_params_H:
anim_params_R:	.space anim_release_cnt+2*num_LEDs
anim_params_S:
anim_params_G:	.space anim_release_cnt+2*num_LEDs
anim_params_V:
anim_params_B:	.space anim_release_cnt+2*num_LEDs

preprocess:		.space 2			; subroutine to call before animation process

;--- LED data - duty levels, and other parameters/flags ---
LED_data:							; brightness level of each LED (lower 8 bit)
	DUTY_DIR = 14					; PWN duty sweep up/down direction (1=down)
	DUTY_HOLD = 15
LED_data_R:		.space 2*num_LEDs
LED_data_G:		.space 2*num_LEDs
LED_data_B:		.space 2*num_LEDs

;--- HSV data buffer -------------------
LED_data_HSV:
LED_data_H:		.space 2*num_LEDs
LED_data_S:		.space 2*num_LEDs
LED_data_V:		.space 2*num_LEDs

;--- for LED refresh/interrupt ---------
RGB:			.space 2			; RGB channel counter
output_duty:	.space 2			; PWM duty level being output
pulse_duration: .space 2			; pulse duration for the PWM level

duty_buff:							; LED data buffers
duty_buff_R:
duty_buff_R1:	.space 2*(PORTA_bits+PORTB_bits+PORTC_bits)
duty_buff_R2:	.space 2*(PORTA_bits+PORTB_bits+PORTC_bits)
duty_buff_G:
duty_buff_G1:	.space 2*(PORTA_bits+PORTB_bits+PORTC_bits)
duty_buff_G2:	.space 2*(PORTA_bits+PORTB_bits+PORTC_bits)
duty_buff_B:
duty_buff_B1:	.space 2*(PORTA_bits+PORTB_bits+PORTC_bits)
duty_buff_B2:	.space 2*(PORTA_bits+PORTB_bits+PORTC_bits)

port_buff:		.space 6			; PORTA/PORTB/PORTC buffers

;--- RGB -> HSV conversion temporary variables ---
var_I:			.space 2
var_H:			.space 2
var_1:			.space 2
var_2:			.space 2
var_3:			.space 2

temp1:			.space 2

end_of_vars:							; end of variables area to initialize

;===================================================================================================
;	main code

	.text
__reset:
				;--- turn off all peripherals ---
				setm	PMD1
				setm	PMD2
				setm	PMD3
				setm	PMD4
				setm	PMD6
				setm	PMD8

				;--- check if cold or hot start ----------------------
				btss	RCON,#POR
				bra		hot_start
													; cold start - power has just been applied
				bclr	RCON,#POR
				bclr	RCON,#BOR
				bra		cold_start
hot_start:
				;--- check if brown-out reset ------------------------
				btss	RCON,#BOR
				bra		0f
				bclr	RCON,#BOR
				bra		power_down					; power down if BOR occurred
0:
cold_start:
				repeat	#250						; wait for the power supply to stablize
				clrwdt

				;--- configure interrupt, etc. -----------------------
				bset	INTCON1,#NSTDIS				; disable nested interrupts

				mov		#psvpage(duty_LUT),w0		; set up PSV
				mov		w0,PSVPAG
				bset	CORCON,#PSV					; enable PSV

				mov		#__SP_init,w15				; Initialize stack pointer and stack limit register
				mov		#__SPLIM_init,w0
				mov		w0,SPLIM
				nop									; A NOP here is required for proper SPLIM functionality

				;--- clear RAM ---------------------------------------
				mov		#var_start,w0				; make sure that the value is even number
				disi	#(end_of_vars-var_start)*2
				repeat	#(end_of_vars-var_start)/2-1
				clr		[w0++]						; or trap will result

				mov		#speed_norm,w0				; set update_rate to the midway point
				mov		w0,update_rate

				;--- set the system clock ----------------------------
				bclr	CLKDIV,#RCDIV0				; set FRC postscaler to 1 (8 MHz)
				mov		#osc_adjust,w0				; adjust internal oscillator
				mov		w0,OSCTUN

				;--- switch clock source and turn on PLL ---
				mov.b	#0x01,w0
				;--- OSCCONH (high byte) unlock sequence ---
				mov		#OSCCONH,w1
				mov		#0x78,w2
				mov		#0x9A,w3
				mov.b	w2,[w1]
				mov.b	w3,[w1]
				;--- set new oscillator selection ----------
				mov.b	WREG,OSCCONH
				;--- OSCCONL (low byte) unlock sequence ----
				mov		#OSCCONL,w1
				mov		#0x46,w2
				mov		#0x57,w3
				mov.b	w2,[w1]
				mov.b	w3,[w1]
				;--- start oscillator switch operation -----
				bset	OSCCON,#OSWEN

				;--- configure I/O ports -----------------------------
.if (COL_POL)
				clr		LATA						; clear all latches (LEDs are active-high)
				clr		LATB
.else
				setm	LATA						; set all latches high (LEDs are active-low)
				setm	LATB
.endif
.if (ROW_POL)
				bclr	PWM_R1_LAT,#PWM_R1_PIN		; set PWM pins low (inactive)
				bclr	PWM_G1_LAT,#PWM_G1_PIN
				bclr	PWM_B1_LAT,#PWM_B1_PIN
				bclr	PWM_R2_LAT,#PWM_R2_PIN
				bclr	PWM_G2_LAT,#PWM_G2_PIN
				bclr	PWM_B2_LAT,#PWM_B2_PIN
.else
				bset	PWM_R1_LAT,#PWM_R1_PIN		; set PWM pins high (inactive)
				bset	PWM_G1_LAT,#PWM_G1_PIN
				bset	PWM_B1_LAT,#PWM_B1_PIN
				bset	PWM_R2_LAT,#PWM_R2_PIN
				bset	PWM_G2_LAT,#PWM_G2_PIN
				bset	PWM_B2_LAT,#PWM_B2_PIN
.endif

				clr		TRISA						; set all ports output
				clr		TRISB
				clr		TRISC

				;--- configure PWM for LEDs --------------------------
				bclr	PMD2,#CCP1MD				; enable power for CCP1
				mov		#ccp1con1L_param,w0
				mov		w0,CCP1CON1L				; set up PWM mode
				mov		#ccp1con3H_param,w0
				mov		w0,CCP1CON3H				; 
													; --- configure timer ---
				mov		#pr_value,w0				; set the time period
				mov		w0,CCP1PRL
.if (!PWM_TYPE)
				mov		w0,CCP1RB					; set the falling edge timing same as timer period
.else
				mov		#port_delay,w0
				mov		w0,CCP1RA					; set the rising edge timing same as port_delay
.endif
				bset	IEC0,#CCT1IE				; enable CCP1 timer int for PWM

				;--- use TIMER1 as frame timebase --------------------
				bclr	PMD1,#T1MD					; enable power for TIMER1
				mov		#0x400*clock_speed-1,w0		; timer period = 2.048 mS
				mov		w0,PR1
				bset	T1CON,#TON					; start timer1

				;--- configure SW input ------------------------------
				bset	SW1_TRIS,#SW1_BIT			; set the switch port input
				bclr	SW1_ANS,#SW1_BIT			; digital mode
				bset	CNPU1,#SW1_CN				; enable the pull-ups

				;--- read EEPROM to restore previous mode ------------
				call	eeprom_read
				mov		#0xFFFF,w1
				cp		w0,w1
				bra		NZ,0f						; if (w0 == 0xFFFF) {
				bset	g_flags,#ignore_switch		;   ignore the switch and
				mov		#0x00, w0					;   run test sequence
				call	eeprom_write				;   update EEPROM
0:													; }
				mov		w0,mode_num

.if ALWAYS_DEMO_MODE
				;--- permanent demo mode ---
				bset	g_flags,#demo_mode			; set demo mode flag
.endif

				;--- check for botton held ---
				mov		#debounce_time*8,w0			; set up loop counter
				mov		w0,temp1					; using temp1
0:
				inc		random_val					; randomize the seed
				btss	IFS0,#T1IF					; wait for TIMER1 overflow
				bra		0b
				bclr	IFS0,#T1IF					; clear T1IF

				call	button_check				; check the button state
				cp0		temp1
				bra		Z,1f						; wait for debounce
				dec		temp1
				bra		0b
1:
				btsc	btnA_data,#btn_down			; is button up?
				bra		2f							; {
				clr.b	LED_data					;   turn off LED 1
				clr		btnA_data					;   clear the button data
				bra		3f							; } else
2:				btss	btnA_data,#btn_long			; is button long pushed?
				bra		0b							; {
				bset	g_flags,#demo_mode			;   turn on demo mode
				dec.b	LED_data_R+59*2				;   flash LED 60
				dec.b	LED_data_G					;   flash LED 1
				dec.b	LED_data_B+1*2				;   flash LED 2
				bra		0b							; }
3:

;---------------------------------------------------------------------------------------------------

startup:
				btss	g_flags,#demo_mode			; if demo mode on
				bra		0f							; {
				mov		#demo_tics,w0				;   initialize the counter
				mov		w0,auto_adv_count
0:													; }

				;--- reset anim params ---------------------
				mov		#anim_params,w0				; make sure that the value is even number
				disi	#(end_of_vars-(anim_params))*2
				repeat	#(end_of_vars-(anim_params))/2-1
				clr		[w0++]						; or trap will result

				bclr	g_flags,#do_not_read		; clear do_not_read flag
				bclr	g_flags,#HSV_mode			; clear HSV mode flag

				;--- set default anim_parameters -----------
				; set LED data pointers
				mov		#LED_data_R,w0
				mov		w0,anim_params_R+anim_data
				mov		#LED_data_G,w0
				mov		w0,anim_params_G+anim_data
				mov		#LED_data_B,w0
				mov		w0,anim_params_B+anim_data

				; set number of LEDs
				mov		#num_LEDs,w0
				mov		w0,anim_params_R+anim_LEDs
				mov		w0,anim_params_G+anim_LEDs
				mov		w0,anim_params_B+anim_LEDs

				; set max_duty
				mov		#max_duty,w0
				mov		w0,anim_params_R+anim_max_duty
				mov		w0,anim_params_G+anim_max_duty
				mov		w0,anim_params_B+anim_max_duty

				;--- keep mode_num within range ---
				mov		#num_modes,w0				; num_modes
				cp		mode_num					; if (mode_num >= num_modes)
				bra		LEU,1f
				clr		mode_num					;   reset the mode number
1:				mov		mode_num,w0
				bra		w0
mode_tbl:
				bra		test_01
				bra		mode_01
				bra		mode_01r
				bra		mode_02
				bra		mode_02r
				bra		mode_03
				bra		mode_03r
				bra		mode_04
				bra		mode_04r
				bra		mode_05
				bra		mode_05r
				bra		mode_06
				bra		mode_06r
				bra		mode_07
				bra		mode_07r
				bra		mode_08
				bra		test_02
				bra		test_03
;				bra		test_04
mode_tbl_end:

				;-------------------------------------------
				; full rainbow
mode_01r:
				bset	anim_params_R+anim_flags,#af_direction	; reverse direction - counter clockwise
				bset	anim_params_G+anim_flags,#af_direction	; reverse direction - counter clockwise
				bset	anim_params_B+anim_flags,#af_direction	; reverse direction - counter clockwise
mode_01:
				; set update_rate
				mov		#0xFFFF*50/100,w0
				mov		w0,anim_params_R+anim_update_rate
				mov		#0xFFFF*50/100,w0
				mov		w0,anim_params_G+anim_update_rate
				mov		#0xFFFF*50/100,w0
				mov		w0,anim_params_B+anim_update_rate

				; set step_up
				mov		#1,w0
				mov		w0,anim_params_R+anim_step_up
				mov		w0,anim_params_G+anim_step_up
				mov		w0,anim_params_B+anim_step_up
				; set step_down
				mov		w0,anim_params_R+anim_step_down
				mov		w0,anim_params_G+anim_step_down
				mov		w0,anim_params_B+anim_step_down

				; set duty_diff
				mov		#255*2/num_LEDs,w0
				mov		w0,anim_params_R+anim_duty_diff
				mov		w0,anim_params_G+anim_duty_diff
				mov		w0,anim_params_B+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data,w0
				disi	#num_LEDs*3
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				bclr	LED_data_R,#DUTY_HOLD
				bclr	LED_data_G+(2*num_LEDs/3),#DUTY_HOLD
				bclr	LED_data_B+(2*num_LEDs*2/3),#DUTY_HOLD

				bra		main

				;-------------------------------------------
				; shifting rainbow
mode_02r:
				bset	anim_params_R+anim_flags,#af_direction	; reverse direction - counter clockwise
				bset	anim_params_G+anim_flags,#af_direction	; reverse direction - counter clockwise
				bset	anim_params_B+anim_flags,#af_direction	; reverse direction - counter clockwise
mode_02:
				; set update_rate
				mov		#0xFFFF*40/100,w0
				mov		w0,anim_params_R+anim_update_rate
				mov		#0xFFFF*50/100,w0
				mov		w0,anim_params_G+anim_update_rate
				mov		#0xFFFF*625/1000,w0
				mov		w0,anim_params_B+anim_update_rate

				; set step_up
				mov		#1,w0
				mov		w0,anim_params_R+anim_step_up
				mov		w0,anim_params_G+anim_step_up
				mov		w0,anim_params_B+anim_step_up
				; set step_down
				mov		w0,anim_params_R+anim_step_down
				mov		w0,anim_params_G+anim_step_down
				mov		w0,anim_params_B+anim_step_down

				; set duty_diff
				mov		#255*3/num_LEDs+1,w0
				mov		w0,anim_params_R+anim_duty_diff
				mov		w0,anim_params_G+anim_duty_diff
				mov		w0,anim_params_B+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data,w0
				disi	#num_LEDs*3
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				; clear DUTY_HOLD bits
				bclr	LED_data_R,#DUTY_HOLD
				bclr	LED_data_G,#DUTY_HOLD
				bclr	LED_data_B,#DUTY_HOLD

				; set all DUTY_HOLD_RS bits
				bset	anim_params_R+anim_flags,#af_duty_hold_rs
				bset	anim_params_G+anim_flags,#af_duty_hold_rs
				bset	anim_params_B+anim_flags,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,anim_params_R+anim_release_waitH
				mov		w0,anim_params_G+anim_release_waitH
				mov		w0,anim_params_B+anim_release_waitH
				mov		#0,w0
				mov		w0,anim_params_R+anim_release_waitL
				mov		w0,anim_params_G+anim_release_waitL
				mov		w0,anim_params_B+anim_release_waitL

				bra		main

				;-------------------------------------------
				; rainbow arrows
mode_03r:
				bset	anim_params_R+anim_flags,#af_direction	; reverse direction - counter clockwise
				bset	anim_params_G+anim_flags,#af_direction	; reverse direction - counter clockwise
				bset	anim_params_B+anim_flags,#af_direction	; reverse direction - counter clockwise
mode_03:
				btg		anim_params_G+anim_flags,#af_direction	; green runs opposit direction
				; set update_rate
				mov		#0xFFFF*45/100,w0
				mov		w0,anim_params_R+anim_update_rate
				mov		#0xFFFF*495/1000,w0
				mov		w0,anim_params_G+anim_update_rate
				mov		#0xFFFF*545/1000,w0
				mov		w0,anim_params_B+anim_update_rate

				; set step_up
				mov		#128,w0
				mov		w0,anim_params_R+anim_step_up
				mov		w0,anim_params_G+anim_step_up
				mov		w0,anim_params_B+anim_step_up
				; set step_down
				mov		#1,w0
				mov		w0,anim_params_R+anim_step_down
				mov		w0,anim_params_G+anim_step_down
				mov		w0,anim_params_B+anim_step_down

				; set duty_diff
				mov		#6,w0
				mov		w0,anim_params_R+anim_duty_diff
				mov		w0,anim_params_G+anim_duty_diff
				mov		w0,anim_params_B+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data,w0
				disi	#num_LEDs*3
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				; clear DUTY_HOLD bits
				bclr	LED_data_R,#DUTY_HOLD
				bclr	LED_data_G,#DUTY_HOLD
				bclr	LED_data_B,#DUTY_HOLD

				; set all DUTY_HOLD_RS bits
				bset	anim_params_R+anim_flags,#af_duty_hold_rs
				bset	anim_params_G+anim_flags,#af_duty_hold_rs
				bset	anim_params_B+anim_flags,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,anim_params_R+anim_release_waitH
				mov		w0,anim_params_G+anim_release_waitH
				mov		w0,anim_params_B+anim_release_waitH
				mov		#0,w0
				mov		w0,anim_params_R+anim_release_waitL
				mov		w0,anim_params_G+anim_release_waitL
				mov		w0,anim_params_B+anim_release_waitL

				bra		main

				;-------------------------------------------
				; HSV mode gradation x2
mode_04r:
				bset	anim_params_V+anim_flags,#af_direction	; reverse direction - counter clockwise
mode_04:
				bset	g_flags,#HSV_mode			; HSV mode flag on

				; set random hue
				call	random_number				; get a random number -> w0
				mov		#max_duty*6,w1				; keep the value under max_duty*6
0:				cp		w0,w1
				bra		LTU,1f
				sub		w0,w1,w0
				bra		0b
1:				mov		#LED_data_H,w1
				repeat	#num_LEDs-1
				mov		w0,[w1++]

				; set max_duty*6 for H channel
				mov		#(max_duty+1)*6-1,w0
				mov		w0,anim_params_H+anim_max_duty

				; set LED data pointers
				mov		#LED_data_H,w0
				mov		w0,anim_params_H+anim_data
				mov		#LED_data_S,w0
				mov		w0,anim_params_S+anim_data
				mov		#LED_data_V,w0
				mov		w0,anim_params_V+anim_data

				; set initial delay
				mov		#0,w0
				mov		w0,anim_params_H+anim_delay
				mov		w0,anim_params_S+anim_delay
				mov		w0,anim_params_V+anim_delay

				; set update_rate
				mov		#0x3000,w0
				mov		w0,anim_params_H+anim_update_rate
				mov		#0xFFFF,w0
				mov		w0,anim_params_S+anim_update_rate
				mov		#0xFFFF*5/10,w0
				mov		w0,anim_params_V+anim_update_rate

				; set step_up
				mov		#1,w0
				mov		w0,anim_params_H+anim_step_up
				mov		#255,w0
				mov		w0,anim_params_S+anim_step_up
				mov		#4,w0
				mov		w0,anim_params_V+anim_step_up
				; set step_down
				mov		#max_duty*6,w0
				mov		w0,anim_params_H+anim_step_down
				mov		#0,w0
				mov		w0,anim_params_S+anim_step_down
				mov		#4,w0
				mov		w0,anim_params_V+anim_step_down

				; set duty_diff
				mov		#0,w0
				mov		w0,anim_params_H+anim_duty_diff
				mov		w0,anim_params_S+anim_duty_diff
				mov		#4*7,w0
				mov		w0,anim_params_V+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data_V,w0
				repeat	#num_LEDs-1
				bset	[w0++],#DUTY_HOLD
				; except two
				bclr	LED_data_V,#DUTY_HOLD
				bclr	LED_data_V+(num_LEDs/2*2),#DUTY_HOLD

				; set DUTY_HOLD_RS bits
				bset	anim_params_V+anim_flags,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,anim_params_V+anim_release_waitH
				mov		#0,w0
				mov		w0,anim_params_V+anim_release_waitL

				bra		main

				;-------------------------------------------
				; HSV mode chase
mode_05r:
				bset	anim_params_V+anim_flags,#af_direction	; reverse direction - counter clockwise
mode_05:
				bset	g_flags,#HSV_mode			; HSV mode flag on

				; set random hue
				call	random_number				; get a random number -> w0
				mov		#max_duty*6,w1				; keep the value under max_duty*6
0:				cp		w0,w1
				bra		LTU,1f
				sub		w0,w1,w0
				bra		0b
1:				mov		#LED_data_H,w1
				repeat	#num_LEDs-1
				mov		w0,[w1++]

				; set max_duty*6 for H channel
				mov		#(max_duty+1)*6-1,w0
				mov		w0,anim_params_H+anim_max_duty

				; set LED data pointers
				mov		#LED_data_H,w0
				mov		w0,anim_params_H+anim_data
				mov		#LED_data_S,w0
				mov		w0,anim_params_S+anim_data
				mov		#LED_data_V,w0
				mov		w0,anim_params_V+anim_data

				; set initial delay
				mov		#0,w0
				mov		w0,anim_params_H+anim_delay
				mov		w0,anim_params_S+anim_delay
				mov		w0,anim_params_V+anim_delay

				; set update_rate
				mov		#0x3000,w0
				mov		w0,anim_params_H+anim_update_rate
				mov		#0xFFFF,w0
				mov		w0,anim_params_S+anim_update_rate
				mov		#0xFFFF*6/10,w0
				mov		w0,anim_params_V+anim_update_rate

				; set step_up
				mov		#1,w0
				mov		w0,anim_params_H+anim_step_up
				mov		#255,w0
				mov		w0,anim_params_S+anim_step_up
				mov		#85,w0
				mov		w0,anim_params_V+anim_step_up
				; set step_down
				mov		#max_duty*6,w0
				mov		w0,anim_params_H+anim_step_down
				mov		#0,w0
				mov		w0,anim_params_S+anim_step_down
				mov		#1,w0
				mov		w0,anim_params_V+anim_step_down

				; set duty_diff
				mov		#0,w0
				mov		w0,anim_params_H+anim_duty_diff
				mov		#0,w0
				mov		w0,anim_params_S+anim_duty_diff
				mov		#256*2/3/num_LEDs,w0
				mov		w0,anim_params_V+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data_S,w0
				repeat	#num_LEDs*2-1
				bset	[w0++],#DUTY_HOLD

;				bclr	LED_data_H,#DUTY_HOLD
				bclr	LED_data_S,#DUTY_HOLD
				bclr	LED_data_V,#DUTY_HOLD

				; set DUTY_HOLD_RS bits
				bset	anim_params_V+anim_flags,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,anim_params_V+anim_release_waitH
				mov		#0,w0
				mov		w0,anim_params_V+anim_release_waitL

				bra		main

				;-------------------------------------------
				; rainbow gradation x2
mode_06r:
				bset	anim_params_H+anim_flags,#af_direction	; reverse direction - counter clockwise
				bset	anim_params_V+anim_flags,#af_direction	; reverse direction - counter clockwise
mode_06:
				bset	g_flags,#HSV_mode			; HSV mode flag on

				; set max_duty*6 for H channel
				mov		#(max_duty+1)*6-1,w0
				mov		w0,anim_params_H+anim_max_duty

				; set LED data pointers
				mov		#LED_data_H,w0
				mov		w0,anim_params_H+anim_data
				mov		#LED_data_S,w0
				mov		w0,anim_params_S+anim_data
				mov		#LED_data_V,w0
				mov		w0,anim_params_V+anim_data

				; set initial delay
				mov		#0,w0
				mov		w0,anim_params_H+anim_delay
				mov		w0,anim_params_S+anim_delay
				mov		w0,anim_params_V+anim_delay

				; set update_rate
				mov		#0xCFFF,w0
				mov		w0,anim_params_H+anim_update_rate
				mov		#0xFFFF,w0
				mov		w0,anim_params_S+anim_update_rate
				mov		#0xFFFF*5/10,w0
				mov		w0,anim_params_V+anim_update_rate

				; set step_up
				mov		#3,w0
				mov		w0,anim_params_H+anim_step_up
				mov		#255,w0
				mov		w0,anim_params_S+anim_step_up
				mov		#4,w0
				mov		w0,anim_params_V+anim_step_up
				; set step_down
				mov		#(max_duty+1)*6,w0
				mov		w0,anim_params_H+anim_step_down
				mov		#0,w0
				mov		w0,anim_params_S+anim_step_down
				mov		#4,w0
				mov		w0,anim_params_V+anim_step_down

				; set duty_diff
				mov		#(max_duty+1)*6/num_LEDs-1,w0
				mov		w0,anim_params_H+anim_duty_diff
				mov		#0,w0
				mov		w0,anim_params_S+anim_duty_diff
				mov		#4*7,w0
				mov		w0,anim_params_V+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data_HSV,w0
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				bclr	LED_data_H,#DUTY_HOLD
				bclr	LED_data_S,#DUTY_HOLD
				bclr	LED_data_V,#DUTY_HOLD
				bclr	LED_data_V+(num_LEDs/2*2),#DUTY_HOLD

				; set DUTY_HOLD_RS bits
				bset	anim_params_V+anim_flags,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,anim_params_V+anim_release_waitH
				mov		#0,w0
				mov		w0,anim_params_V+anim_release_waitL

				bra		main

				;-------------------------------------------
				; rainbow chase
mode_07r:
				bset	anim_params_H+anim_flags,#af_direction	; reverse direction - counter clockwise
				bset	anim_params_V+anim_flags,#af_direction	; reverse direction - counter clockwise
mode_07:
				bset	g_flags,#HSV_mode			; HSV mode flag on

				; set max_duty*6 for H channel
				mov		#(max_duty+1)*6-1,w0
				mov		w0,anim_params_H+anim_max_duty

				; set LED data pointers
				mov		#LED_data_H,w0
				mov		w0,anim_params_H+anim_data
				mov		#LED_data_S,w0
				mov		w0,anim_params_S+anim_data
				mov		#LED_data_V,w0
				mov		w0,anim_params_V+anim_data

				; set initial delay
				mov		#0,w0
				mov		w0,anim_params_H+anim_delay
				mov		w0,anim_params_S+anim_delay
				mov		w0,anim_params_V+anim_delay

				; set update_rate
				mov		#0xCFFF,w0
				mov		w0,anim_params_H+anim_update_rate
				mov		#0xFFFF,w0
				mov		w0,anim_params_S+anim_update_rate
				mov		#0xFFFF*5/10,w0
				mov		w0,anim_params_V+anim_update_rate

				; set step_up
				mov		#3,w0
				mov		w0,anim_params_H+anim_step_up
				mov		#255,w0
				mov		w0,anim_params_S+anim_step_up
				mov		#85,w0
				mov		w0,anim_params_V+anim_step_up
				; set step_down
				mov		#(max_duty+1)*6,w0
				mov		w0,anim_params_H+anim_step_down
				mov		#0,w0
				mov		w0,anim_params_S+anim_step_down
				mov		#1,w0
				mov		w0,anim_params_V+anim_step_down

				; set duty_diff
				mov		#(max_duty+1)*6/num_LEDs-1,w0
				mov		w0,anim_params_H+anim_duty_diff
				mov		#0,w0
				mov		w0,anim_params_S+anim_duty_diff
				mov		#256*2/3/num_LEDs,w0
				mov		w0,anim_params_V+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data_HSV,w0
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				bclr	LED_data_H,#DUTY_HOLD
				bclr	LED_data_S,#DUTY_HOLD
				bclr	LED_data_V,#DUTY_HOLD

				; set DUTY_HOLD_RS bits
				bset	anim_params_V+anim_flags,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,anim_params_V+anim_release_waitH
				mov		#0,w0
				mov		w0,anim_params_V+anim_release_waitL

				bra		main

				;-------------------------------------------
				; HSV mode - color change only
mode_08:
				bset	g_flags,#HSV_mode			; HSV mode flag on
.if 1
				; set random hue
				call	random_number				; get a random number -> w0
				mov		#max_duty*6,w1				; keep the value under max_duty*6
0:				cp		w0,w1
				bra		LTU,1f
				sub		w0,w1,w0
				bra		0b
1:				mov		#LED_data_H,w1
				repeat	#num_LEDs-1
				mov		w0,[w1++]
.endif
				; set max_duty*6 for H channel
				mov		#(max_duty+1)*6-1,w0
				mov		w0,anim_params_H+anim_max_duty

				; set LED data pointers
				mov		#LED_data_H,w0
				mov		w0,anim_params_H+anim_data
				mov		#LED_data_S,w0
				mov		w0,anim_params_S+anim_data
				mov		#LED_data_V,w0
				mov		w0,anim_params_V+anim_data

				; set initial delay
				mov		#0,w0
				mov		w0,anim_params_H+anim_delay
				mov		w0,anim_params_S+anim_delay
				mov		w0,anim_params_V+anim_delay

				; set update_rate
				mov		#0x5FFF,w0
				mov		w0,anim_params_H+anim_update_rate
				mov		#0xFFFF,w0
				mov		w0,anim_params_S+anim_update_rate
				mov		#0xFFFF,w0
				mov		w0,anim_params_V+anim_update_rate

				; set step_up
				mov		#1,w0
				mov		w0,anim_params_H+anim_step_up
				mov		#255,w0
				mov		w0,anim_params_S+anim_step_up
				mov		#1,w0
				mov		w0,anim_params_V+anim_step_up
				; set step_down
				mov		#256*6,w0
				mov		w0,anim_params_H+anim_step_down
				mov		#0,w0
				mov		w0,anim_params_S+anim_step_down
				mov		w0,anim_params_V+anim_step_down

				; set duty_diff
;				mov		#4,w0
;				mov		w0,anim_params_H+anim_duty_diff
;				mov		w0,anim_params_S+anim_duty_diff
;				mov		w0,anim_params_V+anim_duty_diff

				; set all DUTY_HOLD bits
;				mov		#LED_data_H,w0
;				repeat	#num_LEDs-1
;				bset	[w0++],#DUTY_HOLD

;				bclr	LED_data_H,#DUTY_HOLD

				bra		main


				;-------------------------------------------
				; test RGB - chase
test_01:
				; set initial delay
				mov		#0,w0
				mov		w0,anim_params_R+anim_delay
				mov		#256*2,w0
				mov		w0,anim_params_G+anim_delay
				mov		#256*4,w0
				mov		w0,anim_params_B+anim_delay

				; set update_rate -> max
				mov		#0xFFFF,w0
				mov		w0,anim_params_R+anim_update_rate
				mov		w0,anim_params_G+anim_update_rate
				mov		w0,anim_params_B+anim_update_rate

				; set step_up
				mov		#1,w0
				mov		w0,anim_params_R+anim_step_up
				mov		w0,anim_params_G+anim_step_up
				mov		w0,anim_params_B+anim_step_up
				; set step_down
				mov		w0,anim_params_R+anim_step_down
				mov		w0,anim_params_G+anim_step_down
				mov		w0,anim_params_B+anim_step_down

				; set duty_diff
				mov		#0xFF/num_LEDs,w0
				mov		w0,anim_params_R+anim_duty_diff
				mov		w0,anim_params_G+anim_duty_diff
				mov		w0,anim_params_B+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data,w0
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				bclr	LED_data_R,#DUTY_HOLD
				bclr	LED_data_G,#DUTY_HOLD
				bclr	LED_data_B,#DUTY_HOLD

				; set all DUTY_HOLD_RS bits
				bset	anim_params_R+anim_flags,#af_duty_hold_rs
				bset	anim_params_G+anim_flags,#af_duty_hold_rs
				bset	anim_params_B+anim_flags,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,anim_params_R+anim_release_waitH
				mov		w0,anim_params_G+anim_release_waitH
				mov		w0,anim_params_B+anim_release_waitH
				mov		#0xFF*4,w0
				mov		w0,anim_params_R+anim_release_waitL
				mov		w0,anim_params_G+anim_release_waitL
				mov		w0,anim_params_B+anim_release_waitL

				bra		main

				;-------------------------------------------
				; random twinkles
test_02:
				bset	g_flags,#HSV_mode			; HSV mode flag on

				mov		#handle(random_anim),w0		; set random_anim as preprocessor
				mov		w0,preprocess

				; set max_duty*6 for H channel
				mov		#(max_duty+1)*6-1,w0
				mov		w0,anim_params_H+anim_max_duty

				; set LED data pointers
				mov		#LED_data_H,w0
				mov		w0,anim_params_H+anim_data
				mov		#LED_data_S,w0
				mov		w0,anim_params_S+anim_data
				mov		#LED_data_V,w0
				mov		w0,anim_params_V+anim_data

				; set update_rate
				mov		#0xFFFF,w0
				mov		w0,anim_params_H+anim_update_rate
				mov		w0,anim_params_S+anim_update_rate
				mov		w0,anim_params_V+anim_update_rate

				; set step_up
				mov		#1,w0
				mov		w0,anim_params_H+anim_step_up
				mov		#0,w0
				mov		w0,anim_params_S+anim_step_up
				mov		#63,w0
				mov		w0,anim_params_V+anim_step_up
				; set step_down
				mov		#255*6,w0
				mov		w0,anim_params_H+anim_step_down
				mov		#0,w0
				mov		w0,anim_params_S+anim_step_down
				mov		#1,w0
				mov		w0,anim_params_V+anim_step_down

				; set duty_diff
				mov		#0x1,w0
				mov		w0,anim_params_H+anim_duty_diff
				mov		w0,anim_params_S+anim_duty_diff
				mov		#0x100,w0
				mov		w0,anim_params_V+anim_duty_diff

				; set DUTY_HOLD bits for V
				mov		#LED_data_V,w0
				repeat	#num_LEDs-1
				bset	[w0++],#DUTY_HOLD

				; set DUTY_HOLD_RS bits for V
				bset	anim_params_V+anim_flags,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,anim_params_V+anim_release_waitH
				mov		#0,w0
				mov		w0,anim_params_V+anim_release_waitL

				bra		main

				;-------------------------------------------
				; random twinkles 2
test_03:
				bset	g_flags,#HSV_mode			; HSV mode flag on

				mov		#handle(random_anim2),w0	; set random_anim as preprocessor
				mov		w0,preprocess

				; set max_duty*6 for H channel
				mov		#(max_duty+1)*6-1,w0
				mov		w0,anim_params_H+anim_max_duty

				; set LED data pointers
				mov		#LED_data_H,w0
				mov		w0,anim_params_H+anim_data
				mov		#LED_data_S,w0
				mov		w0,anim_params_S+anim_data
				mov		#LED_data_V,w0
				mov		w0,anim_params_V+anim_data

				; set update_rate
				mov		#0xFFFF,w0
				mov		w0,anim_params_H+anim_update_rate
				mov		w0,anim_params_V+anim_update_rate
				mov		#0xFFFF/2,w0
				mov		w0,anim_params_S+anim_update_rate

				; set initial delay
				mov		#0x0300,w0
				mov		w0,anim_params_S+anim_delay

				; set step_up
				mov		#2,w0
				mov		w0,anim_params_H+anim_step_up
				mov		#1,w0
				mov		w0,anim_params_S+anim_step_up
				mov		#63,w0
				mov		w0,anim_params_V+anim_step_up
				; set step_down
				mov		#255*6,w0
				mov		w0,anim_params_H+anim_step_down
				mov		#1,w0
				mov		w0,anim_params_S+anim_step_down
				mov		#1,w0
				mov		w0,anim_params_V+anim_step_down

				; set duty_diff
				mov		#255*6/num_LEDs+1,w0
				mov		w0,anim_params_H+anim_duty_diff
				mov		#1,w0
				mov		w0,anim_params_S+anim_duty_diff
				mov		#0x100,w0
				mov		w0,anim_params_V+anim_duty_diff

				; set DUTY_HOLD bits for H
				mov		#LED_data_H,w0
				repeat	#num_LEDs-2
				bset	[w0++],#DUTY_HOLD

				; set DUTY_HOLD bits for S
				mov		#LED_data_S,w0
				repeat	#num_LEDs-2
				bset	[w0++],#DUTY_HOLD

				; set DUTY_HOLD bits for V
				mov		#LED_data_V,w0
				repeat	#num_LEDs-1
				bset	[w0++],#DUTY_HOLD

				; set DUTY_HOLD_RS bits
				bset	anim_params_S+anim_flags,#af_duty_hold_rs
				bset	anim_params_V+anim_flags,#af_duty_hold_rs

				; set release_wait
				mov		#0x0300,w0
				mov		w0,anim_params_S+anim_release_waitH
				mov		w0,anim_params_S+anim_release_waitL
				mov		#1,w0
				mov		w0,anim_params_V+anim_release_waitH
				mov		#0,w0
				mov		w0,anim_params_V+anim_release_waitL

				bra		main

				;-------------------------------------------
				; random twinkles 3
test_04:
				bset	g_flags,#HSV_mode			; HSV mode flag on

				mov		#handle(random_anim3),w0	; set random_anim as preprocessor
				mov		w0,preprocess

				; set max_duty*6 for H channel
				mov		#(max_duty+1)*6-1,w0
				mov		w0,anim_params_H+anim_max_duty

				; set LED data pointers
				mov		#LED_data_H,w0
				mov		w0,anim_params_H+anim_data
				mov		#LED_data_S,w0
				mov		w0,anim_params_S+anim_data
				mov		#LED_data_V,w0
				mov		w0,anim_params_V+anim_data

				; set update_rate
				mov		#0xFFFF,w0
				mov		w0,anim_params_H+anim_update_rate
				mov		#0xFFFF,w0
				mov		w0,anim_params_S+anim_update_rate
				mov		#0xFFFF*4/10,w0
				mov		w0,anim_params_V+anim_update_rate

				; set step_up
				mov		#3,w0
				mov		w0,anim_params_H+anim_step_up
				mov		#255,w0
				mov		w0,anim_params_S+anim_step_up
				mov		#128,w0
				mov		w0,anim_params_V+anim_step_up
				; set step_down
				mov		#2,w0
				mov		w0,anim_params_H+anim_step_down
				mov		#0,w0
				mov		w0,anim_params_S+anim_step_down
				mov		#1,w0
				mov		w0,anim_params_V+anim_step_down

				; set duty_diff
				mov		#3,w0
				mov		w0,anim_params_V+anim_duty_diff

				; set DUTY_HOLD bits
				mov		#LED_data_V,w0
				repeat	#num_LEDs-1
				bset	[w0++],#DUTY_HOLD

				bclr	LED_data_V,#DUTY_HOLD

				; set DUTY_HOLD_RS bits
				bset	anim_params_V+anim_flags,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,anim_params_V+anim_release_waitH
				mov		#0,w0
				mov		w0,anim_params_V+anim_release_waitL

				bra		main

;---------------------------------------------------------------------

random_anim:	;-------------------------------------------

				mov		#LED_data_V,w3				; w3 -> LED_data_V
				mov		#LED_data_V+num_LEDs*2,w9	; w9 -> end of LED_data_V
0:													; do {
				call	random_number				; 	get a random number -> w0
				mov		#0xFF00,w1
				cp		w0,w1						; 	if (w0 > threshold)
				bra		LTU,1f
				cp0.b	[w3]						; 	and ([w3] == 0)
				bra		NZ,1f						; 	{
				bclr	[w3],#DUTY_HOLD				; 	  release duty hold

				mov		#0x00FF,w1					;     
				and		w0,w1,w0
				mul.uu	w0,#6,w0					
				mov		w0,[w3-num_LEDs*4]			;     H = 6* lower 8 bit of random number

				call	random_number				;     get another random number -> w0
				mov.b	w0,[w3-num_LEDs*2]			;     S = lower 8 bit of random number
1:													;   }
				inc2	w3,w3
				cp		w3,w9
				bra		NZ,0b						; } while (w3 != w9)
				return


random_anim2:	;-------------------------------------------

				mov		#LED_data_V,w3				; w3 -> LED_data_V
				mov		#LED_data_V+num_LEDs*2,w9	; w9 -> end of LED_data_V
0:													; do {
				call	random_number				; 	get a random number -> w0
				mov		#0x0000,w1
				cp		w0,w1						; 	if (w0 > threshold)
				bra		LTU,1f
				cp0.b	[w3]						; 	if ([w3] == 0)
				bra		NZ,1f						; 	{
				mov.b	w0,[w3]						; 	  random brightness
				bset	[w3],#DUTY_DIR				; 	  duty dir = down
				bclr	[w3],#DUTY_HOLD				; 	  release duty hold
1:													; 	}
				inc2	w3,w3
				cp		w3,w9
				bra		NZ,0b						; } while (w3 != w9)
				return


random_anim3:	;-------------------------------------------
				;	random hue

				mov		#LED_data_V,w3				; w3 -> LED_data_V
				mov		#LED_data_V+num_LEDs*2,w9	; w9 -> end of LED_data_V
0:													; do {
				cp0.b	[w3]						; 	if ([w3] == 0)
				bra		NZ,1f						; 	{
				call	random_number				; 	  get a random number -> w0
				mov		#0x05FF,w1					;     
				and		w0,w1,w0
				mov		w0,[w3-num_LEDs*4]			;     H = random number
1:													;   }
				inc2	w3,w3
				cp		w3,w9
				bra		NZ,0b						; } while (w3 != w9)
				return


;===================================================================================================

main:
				bset	IEC0,#CCT1IE				; enable CCP1 timer int for PWM

				;---------------------------------------------------------------
main_loop:		;--- main loop -------------------------------------------------
.if 1
				btss	IFS0,#T1IF					; wait for TIMER1 overflow
				bra		main_loop
				bclr	IFS0,#T1IF					; clear T1IF
.else
				btss	g_flags,#PWM_clock			; wait for PWM update
				bra		main_loop
				bclr	g_flags,#PWM_clock			; clear the flag
.endif
				btss	g_flags,#demo_mode			; check if demo mode on
				bra		0f							; {
				dec		auto_adv_count				;   decrement the timer/counter
				bra		NZ,0f						;   if timed out
				call	random_mode					;     shuffle mode
				bra		mode_change					;     change mode
0:													; }

				;--- button services ---------------------------------
				call	button_check				; check the button states

				btsc	btnA_data,#btn_long			; if button long pushed
				bra		power_down					;   power down

				btss	btnA_data,#btn_push			; if button pushed
				bra		0f							; {
				bclr	btnA_data,#btn_push			;   clear the button flag
mode_change:
				inc		mode_num					;   go to next mode
				bclr	IEC0,#CCT1IE				;   disable CCP1 int for PWM
1:				
				btss	IFS0,#CCT1IF				;   wait until CCP1 timer overflow
				bra		1b

				mov		mode_num,w0					;   write the new mode into EEPROM
				call	eeprom_write

				bra		startup
0:													; }

animate:		;--- animate LEDs ------------------------------------
				mov		update_rate,w0
				add		update_cnt
				bra		NC,main_loop				; skip update if update_cnt <= 0xFFFF

				cp0		preprocess					; if preprocess is set
				bra		Z,0f
				mov		preprocess,w0
				call	w0							;   do the preprocess
0:
				mov		#anim_params_R,w1			; w1 -> anim_params_R
				call	animate_LED

				mov		#anim_params_G,w1			; w1 -> anim_params_G
				call	animate_LED

				mov		#anim_params_B,w1			; w1 -> anim_params_B
				call	animate_LED
animate_done:
;				bset	g_flags,#do_not_read		; disable LED data read

				btsc	g_flags,#HSV_mode			; if HSV mode
				call	convert_HSV

;				bclr	g_flags,#do_not_read		; enable LED data read
				bra		main_loop

;===================================================================================================
;---------------------------------------------------------------------
;	change mode randomly (shuffle)
;
random_mode:
				call	random_number				; random number in w0
				mov		#num_modes+1,w2
				disi	#18
				repeat	#17
				div.u	w0,w2						; w1 = w0 % (num_modes+1)
				mov		w1,w0
				cp		mode_num					; is it the same with current mode?
				bra		Z,random_mode				;   then try again
				mov		w0,mode_num					; otherwise update mode_num

				return


;---------------------------------------------------------------------------------------------------
;	button check

button_check:
				btsc	g_flags,#ignore_switch
				return
				;--- sample all buttons ---
				mov		SW1_PORT,w0
				btst.c	w0,#SW1_BIT
				rlc		btnA_data+samples			; add new sample at LSB

				mov		#btnA_data,w1
				call	check_one_button			; process one switch at a time

				return

;-----------------------------------------------------------
check_one_button:
				;--- process the button data ---
				btsc	[w1],#btn_down		; is the button up or down?
				bra		button_down

button_up:		;--- test if the button is down ---
				mov		#debounce_bits,w0
				mov		[w1+samples],w2
				and		w0,w2,w2					; if all samples are low
				bra		NZ,0f
				bset	[w1],#btn_down				; button is now down
				mov		#long_push_time,w0			; set the long push counter
				mov		w0,[w1+long_push]
0:				return

button_down:	;--- test if the button is up ---
				mov		#debounce_bits,w0
				mov		[w1+samples],w2
				and		w0,w2,w2					; if all samples are low
				xor		w2,w0,w2					; if all samples are high
				bra		NZ,button_held
				bclr	[w1],#btn_down				; button is now up
				bset	[w1],#btn_push				; button has been pushed
				return

button_held:
				add		w1,#long_push,w2
				dec		[w2],[w2]					; decrement long push counter
				bra		Z,0f
				return
0:				bset	[w1],#btn_long				; button is now long pushed
				mov		#long_push_time,w0			; set the long push counter
				mov		w0,[w1+long_push]
				return

;---------------------------------------------------------------------------------------------------
;	animate LEDs
;
;	register usage: 
;		w5 = anim_release_cnt_X
;		w6 = anim_flags_X
;		w7 = max_duty - duty_diff_X
;		w8 = duty_diff_X
;		w14 = max_duty
;		w9,w10,w11,w12,w13

animate_LED:
				mov		[w1+#anim_update_rate],w0
				add		w1,#anim_update,w3
				add		w0,[w3],[w3]				; update_X += update_rate_X
				bra		C,0f						; skip update if update_X <= 0xFFFF
				return
0:
				add		w1,#anim_delay,w0
				cp0		[w0]						; check delay
				bra		Z,0f
				dec		[w0],[w0]
				return
0:
				mov		[w1+#anim_preprocess],w0	; if preprocess is set
				cp0		w0
				bra		Z,0f
				push	w1							; 	save w1
				call	w0							;   do the preprocess
				pop		w1							; 	restore w1
0:
				;--- prepare for LED loop ---
				mov		[w1+#anim_data],w9			; w9 -> LED_data_X
				mov		[w1+#anim_LEDs],w0
				dec		w0,w0
				sl		w0,w0						; w0 = (num_LEDs-1)*2
				add		w9,w0,w11					; w11 -> last LED_data_X

				mov		[w1+#anim_step_up],w12
				mov		[w1+#anim_step_down],w13

				mov		[w1+#anim_duty_diff],w8		; w8 = duty_diff_X
				mov		[w1+#anim_max_duty],w14		; w14 = max_duty
				sub		w14,w8,w7					; w7 = max_duty - duty_diff_X

				mov		[w1+#anim_flags],w6
				btsc	w6,#af_direction			; if (af_direction = 0) {
				bra		1f
				add		w9,#2,w10					;   w10 -> LED_data_X2
				bra		2f							; } else {
1:				mov		w11,w10						;   w10 -> last LED_data_X
2:													; }

				add		w1,#anim_release_cnt,w5		; w5 -> release_cnt_X

LED_loop:
				;--- LED x ---
				mov		#(0xFFFF-1<<DUTY_DIR-1<<DUTY_HOLD),w0
				and		w0,[w9],w0					; mask flag bits and copy LED_data_Xn -> w0		

				btss	[w9],#DUTY_HOLD				; if DUTY_HOLD set
				bra		1f							; {
				cp0		[w5]						;   if (release_cnt_X == 0)
				bra		Z,2f						;     continue to the next LED
				dec		[w5],[w5]					;   if (--release_cnt_X != 0)
				bra		NZ,2f						;     continue to the next LED
				bclr	[w9],#DUTY_HOLD				;   else clear DUTY_HOLD
1:													; }
				btsc	[w9],#DUTY_DIR				; duty up or down?
				bra		LED_rampDown
LED_rampUp:
				cpsne	w8,w0						; if LED_data_Xn = duty_diff_X
				bclr	[w10],#DUTY_HOLD			; release DUTY_HOLD of next LED

				add		w0,w12,w0					; LED_data_Xn += step_up
				cp		w0,w14						; 
				bra		LTU,2f						; if (LED_data_Xn >= max_duty) {
				mov		w14,w0						;   LED_data_Xn = max_duty
				btg		[w9],#DUTY_DIR				;   and toggle the dir flag
				btss	w6,#af_duty_hold_rs			;   if DUTY_HOLD_RS set {
				bra		2f							; 
				bset	[w9],#DUTY_HOLD				;     hold the fade
				mov		[w1+#anim_release_waitH],w3
				mov		w3,[w5]						;     reset anim_release_cnt_Xn
				bra		2f							;   }
													; }
LED_rampDown:
				cpsne	w7,w0						; if LED_data_Xn = (max_duty - duty_diff_X)
				bclr	[w10],#DUTY_HOLD			; release DUTY_HOLD of next LED

				sub		w0,w13,w0					; LED_data_Xn -= step_down
				bra		C,2f						; if (LED_data_Xn <= 0) {
				clr		w0							;   LED_data_Xn = 0
				btg		[w9],#DUTY_DIR				;   and toggle the dir flag
				btss	w6,#af_duty_hold_rs			;   if DUTY_HOLD_RS set {
				bra		2f
				bset	[w9],#DUTY_HOLD				;     hold the fade
				mov		[w1+#anim_release_waitL],w3
				mov		w3,[w5]						;     reset anim_release_cnt_Xn
													;   }
													; }
2:
				mov		#(1<<DUTY_DIR-1<<DUTY_HOLD),w3
				and		w3,[w9],w3					; keep only the flags
				ior		w0,w3,[w9]					; copy w0 && flags -> LED_data_Xn
				cpsne	w9,w11						; last one done?
				return								;   then return

				inc2	w5,w5						; else move on to next LED
				inc2	w9,w9
				inc2	w10,w10
				cpsgt	w10,w11						; if next LED > last LED
				bra		LED_loop					; {
				mov		[w1+#anim_data],w10			;   w10 -> LED_data_X
				bra		LED_loop					; }


;---------------------------------------------------------------------------------------------------
;	convert HSV data to RGB

convert_HSV:
				mov		#LED_data_H,w3				; w3 -> LED_data_H
				mov		#LED_data_R,w7				; w7 -> LED_data_R
				mov		#LED_data_R+(num_LEDs-1)*2,w9	; w9 -> end of LED_data_R

0:													; while (w7 != w9) {
				mov		#(0xFFFF-1<<DUTY_DIR-1<<DUTY_HOLD),w0
				mov		[w3],w4						;   H value -> w4
				and		w0,w4,w4					;   mask the anim flags

				mov		[w3+num_LEDs*2],w5			;   S value -> w5
				and		w0,w5,w5					;   mask the anim flags

				mov		[w3+num_LEDs*4],w6			;   V value -> w6
				and		w0,w6,w6					;   mask the anim flags

				call	HSV2RGB						;   do HSV -> RGB conversion
				cp		w7,w9
				bra		Z,1f
				inc2	w3,w3
				inc2	w7,w7
				bra		0b							; }
1:
				return


;-------------------------------------------------------------------------------
;	HSV -> RGB conversion
;	takes:		H (0~256*6-1): w4
;				S (0~255): w5
;				V (0~255): w6
;	returns:	R (0~255): [w7] (LED_data_Rx)
;				G (0~255): [w7+num_LEDs*2] (LED_data_Gx)
;				B (0~255): [w7+num_LEDs*4] (LED_data_Bx)

HSV2RGB:
				cp0		w5							; if (S == 0) {
				bra		NZ,0f
				mov		w6,[w7]						;   R = G = B = V
				mov		w6,[w7+num_LEDs*2]
				mov		w6,[w7+num_LEDs*4]
				return
0:													; } else {
				mov		w4,var_H					;   var_H = H
				lsr		w4,#8,w0					;   var_I = var_H / 256
				mov 	w0,var_I					;   var_I (0-5)
				sl		w0,#8,w0					;   var_H = var_H - var_I*256
				sub		var_H

													;   var_1 = V(256 - S)/256
				mov		#256,w0
				sub		w0,w5,w0					;   w0 = 256 - S
				mul.uu	w0,w6,w0					;   w1:w0 = V * w0
				lsr		w0,#8,w0					;   w0 = w0 / 256
				mov		w0,var_1

													;   var_2 = V(256 - S(var_H)/256)/256
				mov		var_H,w0
				mul.uu	w0,w5,w0					;   w0 = S * var_H
				lsr		w0,#8,w0					;   w0 = w0 / 256
				neg		w0,w0						;   w0 = 256 - w0
				add		#256,w0
				mul.uu	w0,w6,w0					;   w1:w0 = V * w0
				lsr		w0,#8,w0					;   w0 = w0 / 256
				mov		w0,var_2

													;   var_3 = V(256 - S(256 - var_H)/256)/256
				mov		var_H,w0
				neg		w0,w0						;   w0 = 256 - var_H
				add		#256,w0
				mul.uu	w0,w5,w0					;   w0 = S * w0
				lsr		w0,#8,w0					;   w0 = w0 / 256
				neg		w0,w0						;   w0 = 256 - w0
				add		#256,w0
				mul.uu	w0,w6,w0					;   w1:w0 = V * w0
				lsr		w0,#8,w0					;   w0 = w0 / 256
				mov		w0,var_3

				mov		var_I,w0					;   switch (var_I)
				bra		w0
				bra		var_I0
				bra		var_I1
				bra		var_I2
				bra		var_I3
				bra		var_I4
				bra		var_I5
var_I0:
				mov		w6,[w7]						; R = V
				mov		var_3,w0					; G = var_3
				mov		w0,[w7+num_LEDs*2]
				mov		var_1,w0					; B = var_1
				mov		w0,[w7+num_LEDs*4]
				return
var_I1:
				mov		var_2,w0					; R = var_2
				mov		w0,[w7]
				mov		w6,[w7+num_LEDs*2]			; G = V
				mov		var_1,w0					; B = var_1
				mov		w0,[w7+num_LEDs*4]
				return
var_I2:
				mov		var_1,w0					; R = var_1
				mov		w0,[w7]
				mov		w6,[w7+num_LEDs*2]			; G = V
				mov		var_3,w0					; B = var_3
				mov		w0,[w7+num_LEDs*4]
				return
var_I3:
				mov		var_1,w0					; R = var_1
				mov		w0,[w7]
				mov		var_2,w0					; G = var_2
				mov		w0,[w7+num_LEDs*2]
				mov		w6,[w7+num_LEDs*4]			; B = V
				return
var_I4:
				mov		var_3,w0					; R = var_3
				mov		w0,[w7]
				mov		var_1,w0					; G = var_1
				mov		w0,[w7+num_LEDs*2]
				mov		w6,[w7+num_LEDs*4]			; B = V
				return
var_I5:
				mov		w6,[w7]						; R = V
				mov		var_1,w0					; G = var_1
				mov		w0,[w7+num_LEDs*2]
				mov		var_2,w0					; B = var_2
				mov		w0,[w7+num_LEDs*4]
				return
													; } (endif)


;===================================================================================================
;	PWM update service
;
__CCT1Interrupt:
				push.s								; push w0 - w3 and status
				push.d	w4							; push w4,w5

				clr		CCP1CON2H					; clear PWM output

				;--- set PORTs -----------------------------
				mov		port_buff,w0				; load next PORTA data into W0
				mov		port_buff+2,w1				; load next PORTB data into W1
				mov		port_buff+4,w2				; load next PORTC data into W2

				mov		w0,LATA						; update PORTA LATCH
				mov		w1,LATB						; update PORTB LATCH
				mov		w2,LATC						; update PORTC LATCH

				;--- RGB channel switch ------------------------------
				mov		RGB,w0
				bra		w0
				bra		Blanking	; copy LED data
				bra		Ch_R1		; R1
				bra		Ch_G1		; G1
				bra		Ch_B1		; B1
				bra		Ch_R2		; R2
				bra		Ch_G2		; G2
				bra		Ch_B2		; B2

Blanking:		;--- Blanking period ---------------------------------
				bset	g_flags,#PWM_clock			; set PWM update flag

				clr		output_duty
				inc		output_duty					; output_duty = 1

				btsc	g_flags,#do_not_read		; if LED data being updated
				bra		next_duty					;   skip copying of LED data

				;--- sort & copy LED duty values to buffer -----------
				mov		#tblpage(LED_pins),w0		; prepare LED pin lookup
				mov		w0,TBLPAG
				clr		w0							; clear w0 to prepare for byte read

				;--- R data ---
				mov		#tbloffset(LED_pins),w5		; table address -> w5
				mov		#LED_data_R,w1				; w1 -> LED_data
				mov		#LED_data_R+(2*num_LEDs),w4	; w4 -> last LED_data (loop stopper)
				mov		#duty_buff_R,w2				; w2 -> duty_buff
0:
				tblrdl.b [w5++],w0					; read the LED pin destination
				mov		[w1++],[w2+w0]				; copy LED_data(w1) -> duty_buff_x
				cp		w1,w4						; finish if (w1 = w4)
				bra		NZ,0b

				;--- G data ---
				mov		#tbloffset(LED_pins),w5		; table address -> w5
;				mov		#LED_data_G,w1
				mov		#LED_data_G+(2*num_LEDs),w4	; address of the last LED_data -> w4
				mov		#duty_buff_G,w2
0:
				tblrdl.b [w5++],w0					; read the LED pin destination
				mov		[w1++],[w2+w0]				; LED_data(w1) -> duty_buff_x
				cp		w1,w4						; finish if (w1 = w4)
				bra		NZ,0b

				;--- B data ---
				mov		#tbloffset(LED_pins),w5		; table address -> w5
;				mov		#LED_data_B,w1
				mov		#LED_data_B+(2*num_LEDs),w4	; address of the last LED_data -> w4
				mov		#duty_buff_B,w2
0:
				tblrdl.b [w5++],w0					; read the LED pin destination
				mov		[w1++],[w2+w0]				; LED_data(w1) -> duty_buff_x
				cp		w1,w4						; finish if (w1 = w4)
				bra		NZ,0b

				bra		next_duty


Ch_R1:			;--- R channel ---------------------------------------
				;--- steer pulse to PWM-R ---
				bset	CCP1CON2H,#PWM_R1_ST

				;--- prepare PORT data for the next cycle --
				call	update_ports_G1
				bra		RGB_done

Ch_G1:			;--- G channel ---------------------------------------
				;--- steer pulse to PWM-G ---
				bset	CCP1CON2H,#PWM_G1_ST

				;--- prepare PORT data for the next cycle --
				call	update_ports_B1
				bra		RGB_done

Ch_B1:			;--- B channel ---------------------------------------
				;--- steer pulse to PWM-B ---
				bset	CCP1CON2H,#PWM_B1_ST

				;--- prepare PORT data for the next cycle --
				call	update_ports_R2
				bra		RGB_done

Ch_R2:			;--- R channel ---------------------------------------
				;--- steer pulse to PWM-R ---
				bset	CCP1CON2H,#PWM_R2_ST

				;--- prepare PORT data for the next cycle --
				call	update_ports_G2
				bra		RGB_done

Ch_G2:			;--- G channel ---------------------------------------
				;--- steer pulse to PWM-G ---
				bset	CCP1CON2H,#PWM_G2_ST

				;--- prepare PORT data for the next cycle --
				call	update_ports_B2
				bra		RGB_done

Ch_B2:			;--- B channel ---------------------------------------
				;--- steer pulse to PWM-B ---
				bset	CCP1CON2H,#PWM_B2_ST


				;--- move on to the next PWM level -------------------
				inc2	output_duty
				mov		#max_duty,w0
				cp		output_duty					; if (output_duty > max_duty)
				bra		LEU,next_duty				; {
				clr		port_buff					;   clear port buffers
				clr		port_buff+2
				clr		port_buff+4
				clr		RGB							;   go to blanking
				bra		RGB_done2
													; } else {
next_duty:		;--- move on to next duty level ----------------------
				;--- expand duty value via LUT ---
				mov		#psvoffset(duty_LUT),w3
				mov		#0xFFFE,w0					; mask the LSB for address offset
				and		output_duty,wreg
				mov		[w3+w0],w0					; lookup table
				;--- set next PWM duty ---
.if (PWM_TYPE)
				mov		w0,CCP1RB					; set the falling edge
.else
				mov		w0,CCP1RA					; set the rising edge
.endif
				;--- prepare PORT data for the next cycle --
				call	update_ports_R1

				clr		RGB							; RGB will be 1 so R-ch will be next
													; }

RGB_done:		;---  ------------------------------------------------
				inc		RGB							; next RGB channel
RGB_done2:
				pop.d	w4							; pop w4,w5
				pop.s
				bclr	IFS0,#CCT1IF				; clear CCP1 timer int flag
				retfie

;---------------------------------------------------------------------
;	update ports based on the LED duty values
;		w1 -> duty_buff_X
;		w2 -> port_buff
;
update_ports_R1:
				mov		#duty_buff_R1,w1			; point to duty buffer
				bra		1f
update_ports_R2:
				mov		#duty_buff_R2,w1			; point to duty buffer
				bra		1f
update_ports_G1:
				mov		#duty_buff_G1,w1			; point to duty buffer
				bra		1f
update_ports_G2:
				mov		#duty_buff_G2,w1			; point to duty buffer
				bra		1f
update_ports_B1:
				mov		#duty_buff_B1,w1			; point to duty buffer
				bra		1f
update_ports_B2:
				mov		#duty_buff_B2,w1			; point to duty buffer

1:
				mov		#port_buff,w2			; w2 -> port buffer
				mov		output_duty,w3			; current output_duty -> w3

.if (PORTA_bits > 0)
				mov		#(1<<(PORTA_bits-1)),w4	; set the stopper bit
				mov		w4,[w2]					; this exits the loop
0:
				mov		[w1++],w0				; LED_data -> w0
				cp.b	w0,w3					; compare LED_data:output_duty -> C
				rrc		[w2],[w2]				; C -> MSB of port data
				bra		NC,0b
.if (PORTA_bits < 16)
				repeat	#16-PORTA_bits-1
				lsr		[w2],[w2]
.endif
.if	(COL_POL)	; active-high drive
				inc2	w2,w2					; advance the pointer
.else			; active-low drive
				com		[w2],[w2++]				; invert the data
.endif
.endif

												; repeat on PORTB
				mov		#(1<<(PORTB_bits-1)),w4	; set the stopper bit
				mov		w4,[w2]					; this exits the loop
0:
				mov		[w1++],w0				; LED_data -> w0
				cp.b	w0,w3					; compare LED_data:output_duty -> C
				rrc		[w2],[w2]				; C -> MSB of port data
				bra		NC,0b
.if (PORTB_bits < 16)
				repeat	#16-PORTB_bits-1
				lsr		[w2],[w2]
.endif
.if (ROW_POL==0 && COL_POL==1)
				mov		#(1<<PWM_R_PIN+1<<PWM_G_PIN+1<<PWM_B_PIN),w0
				ior		w0,[w2],[w2]
.endif
.if	(COL_POL)	; active-high drive
				inc2	w2,w2					; advance the pointer
.else			; active-low drive
				com		[w2],[w2++]				; invert the data
.endif

.if (PORTC_bits > 0)
												; repeat on PORTC
				mov		#(1<<(PORTC_bits-1)),w4	; set the stopper bit
				mov		w4,[w2]					; this exits the loop
0:
				mov		[w1++],w0				; LED_data -> w0
				cp.b	w0,w3					; compare LED_data:output_duty -> C
				rrc		[w2],[w2]				; C -> MSB of port data
				bra		NC,0b
.if (PORTC_bits < 16)
				repeat	#16-PORTC_bits-1
				lsr		[w2],[w2]
.endif
.if	(!COL_POL)	; active-low drive
				com		[w2],[w2]				; invert the data
.endif
.endif
				return

;---------------------------------------------------------------------
;	duty expansion look up table
;
	.equ	max_pulse_length, pr_value-port_delay

duty_LUT:
			.equ	x, 1
			.rept	(max_duty)/2
				.equ	y, (x*max_pulse_length)/max_duty
.if (PWM_TYPE)
				.word	port_delay+y 		; CCPR1 value (16 bit)
.else
				.word	pr_value-y  		; CCPR1 value (16 bit)
.endif
				.equ	x, x+2
			.endr

.if 0	;--- show/hide pulse_duration values for evaluation ---
duty_LUT_eval:
			.equ	x, 1
			.rept	(max_duty)/2
				.equ	y, (x*max_pulse_length)/max_duty
				.word	y
				.equ	x, x+2
			.endr
			.word	max_pulse_length		; EOT mark
.endif
duty_LUT_end:

;---------------------------------------------------------------------------------------------------
;	EEPROM read/write
;
	.equ	eeprom_addr,0x7ffe00	; EEPROM address to keep the data

eeprom_write:	;--- write w0 into eeprom_addr ---
				bclr	PMD4,#EEMD					; enable power for EEPROM
				mov		#tblpage(eeprom_addr),w1	; top 8 bits of EEPROM address
				mov		w1,TBLPAG
				mov		#tbloffset(eeprom_addr),w1	; address -> w1
				tblwtl	w0,[w1]

				clr		NVMCON
				bset	NVMCON,#WREN				; set write enable bit
				bset	NVMCON,#NVMOP2

				disi	#7							; disable INITs
													; unlock sequence
				mov		#0x55,w1
				mov		w1,NVMKEY
				mov		#0xAA,w1
				mov		w1,NVMKEY
				bset	NVMCON,#WR					; initiate write oparation
				nop
				nop

0:				clrwdt
				btsc	NVMCON,#WR					; wait till WR clears
				bra		0b

				bset	PMD4,#EEMD					; disable power for EEPROM
				return


eeprom_read:	;--- read eeprom_addr into w0
				bclr	PMD4,#EEMD					; enable power for EEPROM
				mov		#tblpage(eeprom_addr),w0	; top 8 bits of EEPROM address
				mov		w0,TBLPAG
				mov		#tbloffset(eeprom_addr),w0	; address -> w0
				tblrdl	[w0],w0

				bset	PMD4,#EEMD					; disable power for EEPROM
				return

;---------------------------------------------------------------------------------------------------
;	random number generator
;		returns random number in w0 as well as in random_val

random_number:
				mov		#0xB400,w0					; seed can be other values as well
				bclr	SR,#C						; clear carry flag
				rrc		random_val					; random_val>>1 -> carry
				btsc	SR,#C						; if carry is set
				xor		random_val					;   random_val xor= w0
				btsc	SR,#Z						; if (random_val = 0)
				inc		random_val					; 	random_val = 1
				mov		random_val,w0
				return

;---------------------------------------------------------------------------------------------------
;	power down

power_down:
				bclr	RCON,#SWDTEN				; disable WDT
				clr		T1CON						; stop timer1

				clr		IEC0						; disable all interrupts
				clr		IEC1						; disable all interrupts

				clr		LATA						; all output pins low
				clr		LATB

				clr		CCP1CON1L					; stop PWM for LEDs

				bclr	IFS0,#CCT1IF				; clear CCP1 timer int flag

				;--- switch clock source to save power -----
				mov.b	#0b00000101,w0
				;--- OSCCONH (high byte) unlock sequence ---
				mov		#OSCCONH,w1
				mov		#0x78,w2
				mov		#0x9A,w3
				mov.b	w2,[w1]
				mov.b	w3,[w1]
				;--- set new oscillator selection ---
				mov.b	WREG,OSCCONH
				;--- OSCCONL (low byte) unlock sequence ---
				mov		#OSCCONL,w1
				mov		#0x46,w2
				mov		#0x57,w3
				mov.b	w2,[w1]
				mov.b	w3,[w1]
				;--- start oscillator switch operation ---
				bset	OSCCON,#OSWEN


				bset	SW1_TRIS,#SW1_BIT			; set the switch port input
				bclr	SW1_ANS,#SW1_BIT			; digital mode
				bset	CNPU1,#SW1_CN				; enable the pull-ups

0:
				btss	SW1_PORT,#SW1_BIT			; wait until the button up
				bra		0b

				repeat	#2047						; pause for a bit to avoid SW chatter
				nop

				clr		IFS0						; clear all int flags
				clr		IFS1						; clear all int flags
				bset	CNEN1,#SW1_CN				; enable CN interrupt on SW1
				bset	IEC1,#CNIE

0:
				pwrsav	#SLEEP_MODE					; go into sleep mode
				bra		0b							; back to sleep if WDT wakes up

;===================================================================================================
__DefaultInterrupt:
				reset

	.end
