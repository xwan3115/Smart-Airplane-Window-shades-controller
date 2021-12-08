/*
 * project.asm
 *
 *  Created: 3/11/2021 11:46:45 PM
 *   Author: group F14A 2 
; ***Usage: 

; ***Board settings: 
; 1. Connect LCD data pins D0-D7 to PORTF0-7.
; 2. Connect the four LCD control pins BE-RS to PORTA4-7.
; 3. Connect the LED pins LED1-4 to PORTB3-0
; 3. Connect the LED pins LED5-8 to PORTK8-11
; 4. Connect the pin PL3 to pin PG1. output of timer5 from L to G
; 5. Connect the pin PL4 to pin PG3. output of timer5 from L to G
; 6. Connect the pin D0(INT0) to PB0

; ***Registers list:
; r16 can sometimes be used as a temporary register, in the LCD section
; r17 (temp) as a temporary register
; r16-20, r24 used in the main loop for keypad input
; r21, r22 as window 1 and window 2 status register (value from 0 to 3)
; r23 as indicator to avoid double trigger from one PB0 key
; r24 used as a temporary counter for Timer0
; r25 as temporary register
; r26 as lock_central to lock the keypads input from passengers
; r27 as emergency lock, locking all keypad inputs
; r28 as emergency_led (be loaded with 0xFF)
; r29 as a flag to alternate between emergency mode for PB0 Interrupt0
 */ 

.include "m2560def.inc"

.def row    =r16			; current row number
.def col    =r17			; current column number
.def rmask  =r18			; mask for current row
.def cmask	=r19			; mask for current column
.def temp1	=r20			; later is used to store input from keypad at the convert step
.def indicator = r23		; to solve the double interrupt trigger on PB0 press
.def timer_count  =r24		;r24 is used in Timers, and was used in keypad, that why key press paused timer0
.def temp = r25				;temporary register
.def window1 = r21			;hold windows brightness value (from 0 to 3)
.def window2 = r22
.def lock_central = r26		;to lock keypad inputs from passenger (1 = lock, 0 = unlock)
.def lock_emergency = r27	;emergency hard lock status
.def flag = r29				;to alternate between emergency on/off
.def emergency_led = r28	;flip between 0xFF, 0x00

.equ PORTCDIR =0xF0			; use PortD for input/output from keypad: PF7-4, output, PF3-0, input
.equ INITCOLMASK = 0xEF		; scan from the leftmost column, the value to mask output
.equ INITROWMASK = 0x01		; scan from the bottom row
.equ ROWMASK  =0x0F			; low four bits are output from the keypad. This value mask the high 4 bits.

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4

.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

;############### Interrupt requests instructions vector #####################
.cseg
	jmp RESET		;0000 as reset?		
.org INT0addr
	jmp EXT_INT0	;jump to the interrupt handler for external interrupt 0
.org OVF0addr
	jmp Timer0OVF	;jump to the interrupt handler for Timer0 overflow

;############### LCD instruction, command, and data macros ###################
.macro do_lcd_command	;send instructions to LCD RS = 0, RW = 0
	ldi r16, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro
.macro do_lcd_data		;output to display numerical content of a register. RS = 1, RW = 0
	mov r16, @0			;copy content of register @0 to r16
	subi r16, -48		;Convert to ASCII value
	rcall lcd_data
	rcall lcd_wait
.endmacro
.macro do_lcd_value		;output to display numerical content of immediate value. RS = 1, RW = 0
	ldi r16, @0			;copy the value @0 to r16
	rcall lcd_data
	rcall lcd_wait
.endmacro

.macro lcd_set		;set a bit at portA LCD control to 1
	sbi PORTA, @0
.endmacro
.macro lcd_clr		;set a bit at portA LCD control to 0
	cbi PORTA, @0
.endmacro

;################### LCD write-at a position macros ##########################
.macro line1_display_value				;@0 value, not register, @1 position
	do_lcd_command 0b10000000|@1		;display position @1 from left   eg: line1_display_value 3 11   (1011 or 0000) = 1011
	do_lcd_value @0
.endmacro
.macro line2_display_value				;@0 value, not register, @1 position
	do_lcd_command 0b11000000|@1		;display position @1 from left, second line
	do_lcd_value @0
.endmacro
.macro line1_display_register			;@0 register, @1 position
	do_lcd_command 0b10000000|@1		;display position @1 from left
	do_lcd_data @0
.endmacro
.macro line2_display_register			;@0 register, @1 position
	do_lcd_command 0b11000000|@1		;display position @1 from left, second line
	do_lcd_data @0
;	do_lcd_command 0b00000100|(@2<<1)	;display direction @2 = 0: 0<<1 = 00. @2 = 1: 1<<1 = 10
.endmacro

;###################### Window up down macro #######################
.macro increase_cap_at_3	;@0: a register, typically window1, window2, @1: OCR5nL
	cpi lock_central, 1		;if lock_central = 0, continue
	breq end_increase		;if lock_central = 1, end, do nothing
	line1_display_value 'L', 0
	inc @0
	sbrc @0, 2		;if that value below 4, skip next line to exit 0b 0100 4 -> 3
	ldi @0, 3		;if that value reaches 4, make it 3 to cap it, then exit
	LED_update_loop_match_window_to_LED @0, @1
end_increase:	
.endmacro

.macro decrease_cap_at_0	;@0 is a register, typically window1, window2
	cpi lock_central, 1		;if lock_central = 0, continue
	breq end_decrease		;if lock_central = 1, end, do nothing
	line1_display_value 'L', 0
	dec @0
	sbrc @0, 7				;if that value not negative, skip next line
	ldi @0, 0				;if that value reaches negative, make it 0
	LED_update_loop_match_window_to_LED @0, @1
end_decrease:
.endmacro

;#################### Update LED signal to window1 and 2#################
.macro update_LED_window1
	sbic PING, 0                   ; skip if Pin0 of portG is 0 (PL4 signal, OCA)
	sbi PORTB, 0                   ; turn on the first LED
	sbic PING, 0                   ; skip if Pin0 of portG is 0
	sbi PORTB, 1                   ; turn on the second LED

	sbis PING, 0                   ; skip if Pin0 of portG is 1
	cbi PORTB, 0                   ; turn off the first LED
	sbis PING, 0                   ; skip if Pin0 of portG is 1
	cbi PORTB, 1                   ; turn off the second LED
.endmacro
.macro update_LED_window2
	sbic PING, 2                   ; skip if Pin2 of portG is 0 (PL3 signal, OCB)
	sbi PORTB, 2                   ; turn on the third LED
	sbic PING, 2                   ; skip if Pin2 of portG is 0
	sbi PORTB, 3                   ; turn on the fourth LED

	sbis PING, 2                   ; skip if Pin2 of portG is 1
	cbi PORTB, 2                   ; turn off the third LED
	sbis PING, 2                   ; skip if Pin2 of portG is 1
	cbi PORTB, 3                   ; turn off the fourth LED
.endmacro

.macro LED_update_loop_match_window_to_LED ;@0 is window1, window2 register, @1 OCR5nL
;this macro matches the value in window1,2 to its LED brightness value
	cpi @0, 0
	breq led_255
	cpi @0, 1
	breq led_100
	cpi @0, 2
	breq led_30
	cpi @0, 3
	breq led_0
led_255:
	ldi temp, 255			;low bit = 0xFF
	sts @1, temp		;OCRn5 is the PWM width value of timer5
	rjmp end_led_update
led_100:
	ldi temp, 100			;low bit = 0x46
	sts @1, temp		;OCRn5 is the PWM width value of timer5
	rjmp end_led_update
led_30:
	ldi temp, 30			;low bit = 0x1E
	sts @1, temp		;OCRn5 is the PWM width value of timer5
	rjmp end_led_update
led_0:
	ldi temp, 0				;low bit = 0x00
	sts @1, temp		;OCRn5 is the PWM width value of timer5
	rjmp end_led_update
end_led_update:
.endmacro

;############# LCD init macro ###################
.macro lcd_init
	ser r16						;1111 1111
	out DDRF, r16				;port F for LCD display 
	out DDRA, r16				;and port A output is LCD control
	clr r16						;0000 0000
	out PORTF, r16				;values at port F and A are 0
	out PORTA, r16

	;init the LCD
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001100 ; Cursor off, bar, no blink
.endmacro

;############# Delay macro for use in keypad debouncing ############
.macro delay_10ms_x	
	ldi r19, @0
loop_delay_1s:
	rcall sleep_5ms
	dec r19
	cpi r19, 0
	brne loop_delay_1s
.endmacro

;############# Interrupt and Timer subroutines ############
RESET:
;set up default Stack location
	ldi temp, low(RAMEND)		;stack at Ramend 0x21ff for RESET
	out SPL, temp
	ldi temp, high(RAMEND)
	out SPH, temp

;set up LEDs port B, port K for LEDs
	ser temp				;temp = 1111 1111
	out DDRB, temp			;port B is set output for 4 LEDs for 2 windows
	sts DDRK, temp			;port K is set output for 4 LEDs emergency alert

;set up port G as input to copy-paste PWM signal from PL3,PL4 to LED1-4 port B
	clr temp				;0000 0000
	out DDRG, temp          ;set port G to input mode
	ser temp				;1111 1111
	out PORTG, temp         ;output 1 to input port, to activate pull-ups to maintain hard TRUE value

;set up port L output OC5A, OC5B at PL4, PL3 pins (PWM from TIMER5)
	ldi temp, 0b00011000	;set PL3, PL4 as output ports
	sts DDRL, temp			;port l use 'sts', not 'out'

;set up TIMER5 (lecture week 7)
	ldi temp1,(1<<CS50)		;Prescale value of 0 (CS0=1, CS1=CS2=0)
	sts TCCR5B,temp1		;
	ldi temp1,(1<<WGM50)|(1<<COM5A1)|(1<<COM5B1);WGM50 = 0 COM5A1 = 7 COM5B1 = 5. 0b1010 0001.   1<<0 means 1 ?
	sts TCCR5A,temp1		;put 1010 0001 to TCCR5A. Phase Correct PWM, use both OC5A and OC5B

;set up initial value of PWM signal for TIMER5: 0, 30, 100, 255*
	clr temp				;high bit = 0
	sts OCR5AH, temp		;PL4 (Yellow wire) to PG3 (bit0)
	sts OCR5BH, temp		;PL3 (Red wire) to PG1 (bit2)
	ldi temp, 255			;low bit = 0xFF
	sts OCR5AL, temp		;0xFF means max brightness as default state
	sts OCR5BL, temp		;OCRn5 is the PWM width value of timer5

;set up LCD, print default two-window template
	lcd_init
	line1_display_value 'S', 0	
	line1_display_value ':', 1
	line1_display_value 'W', 9
	line1_display_value '1', 10
	line1_display_value 'W', 13
	line1_display_value '2', 14

	line2_display_value '0', 9		;default windows open = clear(0)
	line2_display_value '0', 13		;default windows open = clear(0)

;set up keypads
	ldi temp1, PORTCDIR			; columns are outputs, rows are inputs 1111 0000
	out	DDRC, temp1

;set up INT0, port D0
	ldi temp, (2<<ISC00)		;ISC00 == 0. 2 == 0b0010, 2 shift 0 is 0010,
	sts EICRA, temp				;set INT0 as falling edge sensed interrupt
	in temp,EIMSK				;load what in EIMSK to temp
	ori temp, (1<<INT0)			;INT0 = 0, 1<<0 = 0001, to enable INT0
	out EIMSK, temp				;save the EIMSK config register

;set up Timer0 to count 1 second (toggle by PB0)
	ldi temp, 0b00000000	;TCCR0A set to 0000 means normal mode, counter up to 255 then reset
	out TCCR0A, temp
	ldi temp, 0b00000100	;TTCR0B 0100 means CS00 = CS01 = 0; CS02 = 1. Means prescaler 256
	out TCCR0B, temp		;that means one full counter cycle is 4096 micro second. 244 interrupt ~ 1 second
	ldi temp, 0<<TOIE0		;TOIE0 means overflown config pin in TIMSK0 timer0 config register.
	sts TIMSK0, temp		;1<<0 == 0001. Set the bit 1 TOIE0 of TIMSK0 to 1, to enable overflown timer

;Enable I in SREG
	sei						;set global bit I in SREG

;Clear
	clr indicator 
	clr flag
	clr window1					;windows default is clear(0)
	clr window2
	ldi emergency_led, 0xFF		;1111 1111 initial flashing pattern
	clr lock_central			;clear all locks
	clr lock_emergency

	jmp main	;start the main keypad loop at main:

EXT_INT0:
	cpi indicator, 1		;if indicater == 1, accept, otherwise exit INT
	breq accept_interrupt		
	jmp exit_interrupt		;if indicater == 0, exit

accept_interrupt:
	cpi flag, 0					;default = 0. Like toggle.
	breq emergency_active		;flag = 0, activate Timer0, set flag to 1
	rjmp emergency_deactive		;flag = 1, deactivate Timer0, set flag to 0

emergency_active:
	;turn on the Timer0 to flash 4 LEDs
	ldi temp, 1<<TOIE0		;TOIE0 means overflown config pin in TIMSK0 timer0 config register. 
	sts TIMSK0, temp		;TOIE0 = 0, 1<<0 == 0001. ENABLE overflown timer

	;open all windows, set to clear(0), display on LCD
	ldi window1, 0			;open all windows, set to clear(0)
	ldi window2, 0			;open all windows, set to clear(0)
	line2_display_register window1, 9	;show window status(#0) on LCD
	line2_display_register window2, 13	;show window status(#0) on LCD
	line1_display_value '!', 0, 1
	line1_display_value '!', 1, 1
	line1_display_value '!', 2, 1
	line1_display_value ':', 3, 1

	;update the LED to max brightness
	ldi temp, 255			;OCRn5 is the PWM width value of timer5 to control LEDs
	sts OCR5BL, temp		;low bit = 0xFF = 255 max brightness
	sts OCR5AL, temp		;low bit = 0xFF = 255 max brightness

activate_end:
	ldi lock_emergency, 1	;hard lock all keys
	ldi flag, 1				;come here because flag ==0, set to 1 to alternate on next press (toggle)
	jmp return_interrupt	;reti to return

emergency_deactive:
	;turn off the Timer0 to stop flashing 4 LEDs
	ldi temp, 0<<TOIE0		;TOIE0 means overflown config pin in TIMSK0 timer0 config register. 
	sts TIMSK0, temp		;TOIE0 = 0, 0<<0 == 0000. DISABLER overflown timer
	
	;also clear the 4 emergency LEDs
	clr temp				;0000 0000
	sts PORTK, temp			;turn off the emergency LEDs 5-8
	
	;Emergency off, return the lock to its previous status, update LCD
	;Disable emergency does not involve window1, window2, and lock_central
	line1_display_value ':', 1		;print ':' after the L/C status
	line1_display_value 32, 2		;print nul to erase the 3rd '!' ASCII 32 is ' '
	line1_display_value 32, 3		;print nul to erase the  ':'
	cpi lock_central, 0		;if no local lock, we return to L mode
	breq print_L			;otherwise, if there is local lock, we return to C mode
;print_C:
	line1_display_value 'C', 0, 1
	rjmp deactivate_end
print_L:
	line1_display_value 'L', 0, 1	
	rjmp deactivate_end

deactivate_end:
	ldi lock_emergency, 0	;release the hard lock, keypad now works
	ldi flag, 0				;come here because flag ==1, set to 0 to alternate on next press (toggle)
	jmp return_interrupt	;reti to return

return_interrupt:
	clr indicator		;to avoid-back to-back double Interrupts from PB0
exit_interrupt:
	reti				;return from interrupt

Timer0OVF:
	;244 cycles = 1s. Auto run in background to flash LEDs				
	subi timer_count, -1			;increase temp counter by 1
	cpi timer_count, 122			;Check if timer_count = 122 (half second)
	brne NotSecond_Continue			;if not 122, continue to count

	;every half second, flip the LED pattern 
	com emergency_led			;1111 1111	
	sts PORTK, emergency_led   	;show LEDs portK LED5-8

	clr timer_count						;when done with counter, reset to 0 for another interation
	rjmp endif
NotSecond_Continue:		;nothing here, timer continue to self-loop
endif:
	reti				;done the timer0 function

;##########################Main program###############################
;the main will loop to get keypad inputs dealing with L and C controls
;PB0 is external INT0 interrupt, dealing with emergency control
;The PWM signal from TIMER5 also need looping feed to show LEDs,

main:
;duplicate the PWM signal from PL3, PL4 to port G, then copy to LEDs at portB
;	LED_update_loop_match_window_to_LED window1, OCR5AL	;match windows to LEDs
;	LED_update_loop_match_window_to_LED window2, OCR5BL
	update_LED_window1	;share PWM signal to LEDs at port B
	update_LED_window2	;share PWM signal to LEDs at port B

;set indicator to 1 to solve the PB0 double trigger problem
	cpi indicator, 1		;check if indicator == 1, 
	breq continue_main		;if it's 1, skip, continue
	delay_10ms_x 60
	ldi indicator, 1		;if indicator not 1, set it to 1

continue_main:
;main loop to look for key pressed for L and C controls
	ldi cmask, INITCOLMASK		; initial column mask 1110 1111
	clr	col						; initial column
colloop:
	cpi col, 4					; if col is 4, we restart at main, col = 0
	breq main_jump
	out	PORTC, cmask			; set column to mask value (one column off)
	ldi temp1, 0xFF
	rjmp delay

main_jump:
	jmp main

delay:
	dec temp1
	brne delay

	in	temp1, PINC				; read PORTC the keypad
	andi temp1, ROWMASK			; rowmask = 0000 1111
	cpi temp1, 0xF				; check if any rows are on 1111
	breq nextcol
								; if yes, find which row is on
	ldi rmask, INITROWMASK		; initialise row check 0000 0001 as initial mask
	clr	row						; initial row
rowloop:
	cpi row, 4					; if row reaches 4, next collum
	breq nextcol
	mov r25, temp1
	and r25, rmask			; check masked bit rmask can be 0001, 0010, 0100, 1000
	breq convert 				; if bit is clear, convert the bitcode
	inc row						; else move to the next row
	lsl rmask					; shift the mask to the next bit 0001 to 0010 to 0100 to 1000
	jmp rowloop

nextcol:
	lsl cmask					; else get new mask by shifting and 
	inc col						; increment column value
	jmp colloop					; and check the next column

convert:
	cpi col, 3					; if column is 3 we have a letter
	breq letters				
	cpi row, 3					; if row is 3 we have a symbol or 0
	breq symbols

	mov temp1, row				; otherwise we have a number in 1-9
	lsl temp1
	add temp1, row				; temp1 = row * 3
	add temp1, col				; add the column address to get the value
	subi temp1, -'1'			; add the value of character '0' -'1' = -48. --48 = +48. convert to ASCII
	jmp convert_end

letters:
	ldi temp1, 'A'
	add temp1, row				; increment the character 'A' by the row value, A -> B,C,D
	jmp convert_end

symbols:
	cpi col, 0					; check if we have a star
	breq star
	cpi col, 1					; or if we have zero
	breq zero					
	ldi temp1, '#'				; if not we have hash
	jmp convert_end
star:
	ldi temp1, '*'				; set to star
	jmp convert_end
zero:
	ldi temp1, '0'				; set to zero

convert_end:
	delay_10ms_x 60				;some delay to avoid double trigger

	cpi lock_emergency, 1		;if emergency lock is engage, 
	breq end_of_control			;exit from control, back to main loop

controls:
	cpi temp1, '1'				; if '1' increase value in window 1, and darken it
	breq win_1_up
	cpi temp1, '4'				; if '4' decrease value in window 1, and lighten it
	breq win_1_down
	cpi temp1, '2'				; if '2' increase value in window 2, and darken it
	breq win_2_up
	cpi temp1, '5'				; if '5' decrease value in window 2, and lighten it
	breq win_2_down
	cpi temp1, '3'				; if '3' both windows to 3, and close them dark
	breq central_up
	cpi temp1, '6'				; if '6' both windows to 0, and open them bright
	breq central_down
	cpi temp1, 'A'				; if 'A' is pressed, release the central lock
	breq dismiss_central

	jmp end_of_control			; any other input, go back to main

win_1_up:
	jmp win_1_up_procedure
win_1_down:
	jmp win_1_down_procedure
win_2_up:
	jmp win_2_up_procedure
win_2_down:
	jmp win_2_down_procedure
central_up:
	jmp central_up_procedure
central_down:
	jmp central_down_procedure
dismiss_central:
	clr lock_central			;set lock_central = 0, clear lock
	line1_display_value 'L', 0, 1	;updates LCD the lock status
	jmp end_of_control

end_of_control:
	jmp main				; restart main loop when done with controls

win_1_up_procedure:
	increase_cap_at_3 window1, OCR5AL		;increase value in window 1
	line2_display_register window1, 9, 1	;show that value to screen under W1
	jmp end_of_control
win_1_down_procedure:
	decrease_cap_at_0 window1, OCR5AL		;decrease value in window 1
	line2_display_register window1, 9, 1	;show that value to screen under W1
	jmp end_of_control
win_2_up_procedure:
	increase_cap_at_3 window2, OCR5BL		;increase value in window 1
	line2_display_register window2, 13, 1	;show that value to screen under W2
	jmp end_of_control
win_2_down_procedure:
	decrease_cap_at_0 window2, OCR5BL		;decrease value in window 1
	line2_display_register window2, 13, 1	;show that value to screen under W2
	jmp end_of_control

central_up_procedure:
	ldi window1, 3					;close all windows dark = 3, clear = 0
	ldi window2, 3					
	line2_display_register window1, 9, 1	;update windows status on LCD
	line2_display_register window2, 13, 1

	ldi lock_central, 1				;set lock_central = 1, disallow passenger to change window
	line1_display_value 'C', 0, 1	;update LCD the lock status

	ldi temp, 0				;OCRn5 is the PWM width value of timer5 to control LEDs
	sts OCR5BL, temp		;low bit = 0x00 complete dark
	sts OCR5AL, temp		;low bit = 0x00 complete dark
	jmp end_of_control
central_down_procedure:
	ldi window1, 0					;open all windows clear = 0, dark = 3
	ldi window2, 0					
	line2_display_register window1, 9, 1	;update windows status on LCD
	line2_display_register window2, 13, 1

	ldi lock_central, 1				;set lock_central = 1, disallow passenger to change window
	line1_display_value 'C', 0, 1	;update LCD the lock status

	ldi temp, 255			;OCRn5 is the PWM width value of timer5 to control LEDs
	sts OCR5BL, temp		;low bit = 0xFF max brightness
	sts OCR5AL, temp		;low bit = 0xFF max brightness
	jmp end_of_control
; ############Functions for LCD Init###############
lcd_command:
	push r16			;Prologue

	out PORTF, r16
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop

	pop r16				;Epilogue
	ret

lcd_data:
	push r16			;Prologue 
	out PORTF, r16
	lcd_set LCD_RS
	nop
	nop
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	lcd_clr LCD_RS
	pop r16				;Epilogue
	ret

lcd_wait:
	push r16			;Prologue
	clr r16				;0000 0000
	out DDRF, r16
	out PORTF, r16
	lcd_set LCD_RW		
lcd_wait_loop:
	nop
	lcd_set LCD_E		
	nop
	nop
    nop
	in r16, PINF			;
	lcd_clr LCD_E			;
	sbrc r16, 7				;
	rjmp lcd_wait_loop
	lcd_clr LCD_RW			;
	ser r16
	out DDRF, r16
	pop r16				;Epilogue
	ret

sleep_1ms:
	push r30
	push r31
	ldi r31, high(DELAY_1MS)
	ldi r30, low(DELAY_1MS)
delayloop_1ms:
	sbiw r31:r30, 1
	brne delayloop_1ms
	pop r31
	pop r30
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret

;##############end of functions for LCD init#############
