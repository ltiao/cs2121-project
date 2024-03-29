.include "m64def.inc"
.def temp=r16
.def row =r17
.def col =r18
.def mask =r19
.def temp2 =r20
.def counter = r21
.def counter2 = r22
.def counter3 = r23
.def del_hi=r30
.def del_lo=r31
.def data = r25

.equ RAND_A = 214013
.equ RAND_C = 2531011

.equ PORTCDIR = 0xF0
.equ INITCOLMASK = 0xEF
.equ INITROWMASK = 0x01
.equ ROWMASK = 0x0F
.equ MAX = 0xFF
.equ max_speed = 255
.equ min_speed = 99
;LCD protocol control bits
.equ LCD_RS = 3
.equ LCD_RW = 1
.equ LCD_E = 2
;LCD functions
.equ LCD_FUNC_SET = 0b00110000
.equ LCD_DISP_OFF = 0b00001000
.equ LCD_DISP_CLR = 0b00000001
.equ LCD_DISP_ON = 0b00001100
.equ LCD_ENTRY_SET = 0b00000100
.equ LCD_ADDR_SET = 0b10000000
;LCD function bits and constants
.equ LCD_BF = 7
.equ LCD_N = 3
.equ LCD_F = 2
.equ LCD_ID = 1
.equ LCD_S = 0
.equ LCD_C = 1
.equ LCD_B = 0
.equ LCD_LINE1 = 0
.equ LCD_LINE2 = 0x40
.equ LENGTH = 14
.equ hurdle = 15
.equ FREQ2=255

.dseg
bases:  .byte 1
level:	.byte 1
shots_fired: .byte 1
probability: .byte 1
score:	.byte 2
timestep:	.byte 1
game_started: .byte 1
shockwave: .byte 1
RAND:	.byte 4
player: .byte 14
cpu: 	.byte 14
state:	.byte 14

.cseg
;setting up the interrupt vector
jmp RESET
jmp Default ; IRQ0 Handler
jmp Default ; IRQ1 Handler
jmp Default ; IRQ2 Handler
jmp Default ; IRQ3 Handler
jmp EXT_INT4 ; IRQ4 Handler
jmp EXT_INT5 ; IRQ5 Handler
jmp Default ; IRQ6 Handler
jmp Default ; IRQ7 Handler
jmp Default ; Timer2 Compare Handler
jmp Default ; Timer2 Overflow Handler
jmp Default ; Timer1 Capture Handler
jmp Default ; Timer1 CompareA Handler
jmp Default ; Timer1 CompareB Handler
jmp Default ; Timer1 Overflow Handler
jmp Default ; Timer0 Compare Handler
jmp Timer0  ; Timer0 Overflow Handler

;.org 0x100
game_over_string: .db "Game Over!"
final_score_string: .db "Final Score:"
start_game_string1: .db "Press PB0 to    "
start_game_string2: .db "begin battle!   "


DEFAULT:
reti

RESET:
ldi temp, low(RAMEND)
out SPL, temp
ldi temp, high(RAMEND)
out SPH, temp

ldi counter,0            
ldi counter2,0
ldi counter3,0
;ldi cycles, 0

ldi temp, PORTCDIR ; columns are outputs, rows are inputs
out DDRC, temp
;ser temp
;out DDRC, temp ; Make PORTC all outputs
;out PORTC, temp ; Turn on all the LEDs

ldi temp,255
out DDRB,temp
;ldi speed, 255
;ldi temp, 0b11000000
;clr temp
ldi temp, 0b00000010
out PORTB, temp

ldi temp, 0b00000010     ; 
out TCCR0, temp          ; Prescaling value=8  ;256*8/7.3728( Frequency of the clock 7.3728MHz, for the overflow it should go for 256 times)
ldi temp, 1<<TOIE0       ; =278 microseconds
out TIMSK, temp          ; T/C0 interrupt enable

ldi r16, 1 << CS10 ;Start timer
out TCCR1B, r16
;ldi temp, 1<<TOIE1
;out TIMSK, temp

ldi temp, (1 << WGM21) | (1 << COM20) | (3 << CS20); CTC moded, toggle OC2, prescalar = 64
out TCCR2, temp
ser temp
ldi temp, FREQ2
out OCR2, temp

ldi temp, (2 << ISC40) | (2 << ISC50) ;setting the interrupts for falling edge
sts EICRA, temp                       ;storing them into EICRA 
in temp, EIMSK                        ;taking the values inside the EIMSK  
ori temp, (1<<INT4) | (1<<INT5)       ; oring the values with INT0 and INT1  
out EIMSK, temp                       ; enabling interrput0 and interrupt1

rcall lcd_init
rcall start_screen

ldi temp, 3
ldi XL, low(bases)
ldi XH, high(bases)
st X, temp

ldi temp, 1
ldi XL, low(level)
ldi XH, high(level)
st X, temp

ldi temp, 0
ldi XL, low(score)
ldi XH, high(score)
st X+, temp
st X, temp

ldi temp, 0
ldi XL, low(game_started)
ldi XH, high(game_started)
st X, temp

ldi temp, 0
ldi XL, low(shockwave)
ldi XH, high(shockwave)
st X, temp

ldi temp, 0
ldi XL, low(shots_fired)
ldi XH, high(shots_fired)
st X, temp

ldi temp, 80
;ldi temp, 50
ldi XL, low(probability)
ldi XH, high(probability)
st X, temp

ldi temp, 35
ldi XL, low(timestep)
ldi XH, high(timestep)
st X, temp

rcall initRandom

; Initialize Array
		ldi temp, ' '
		ldi XL, low(player)        ; point Y at the string
        ldi XH, high(player)       ; recall that we must multiply any Program code label address
 
 		ldi counter, LENGTH << 1 
		init_array: 
		st X+, temp
		;inc temp
		dec counter
		brne init_array

ldi data, -16

sei
jmp main

; Start Game
EXT_INT4:
push temp
in temp, SREG
push temp

rcall level_up

pop temp
out SREG, temp
pop temp
reti

; Next Level
EXT_INT5:
push temp
in temp, SREG
push temp
push r17
push r28
push r29

ldi YL, low(game_started)
ldi YH, high(game_started)
ld r17, Y
cpi r17, 1
breq already_begun
ldi r17, 1
st Y, r17
rjmp exit_int5
already_begun:
;rcall level_up

exit_int5:

pop r29
pop r28
pop r17
pop temp
out SREG, temp
pop temp
reti

Timer0:                  ; Prologue starts.
push r20
push r29                 ; Save all conflict registers in the prologue.
push r28
in r24, SREG
push r24                 ; Prologue ends.

/**** a counter for 3597 is needed to get one second-- Three counters are used in this example **************/                                          
                         ; 3597  (1 interrupt 278microseconds therefore 3597 interrupts needed for 1 sec)
;cpi counter, 97          ; counting for 97
cpi counter, 50
brne notsecond

ldi XL, low(timestep)
ldi XH, high(timestep)
ld r20, X

;cpi counter2, 35         ; counting for 35
cp counter2, r20
brne secondloop          ; jumPINC into count 100 

ldi XL, low(game_started)
ldi XH, high(game_started)
ld r20, X
cpi r20, 1
breq begin_game
rjmp dont_kill_base
begin_game:

ldi XL, low(bases)
ldi XH, high(bases)

ld r20, X
cpi r20, 0
brne continue_game
rcall game_over
rjmp end
continue_game:

;Start

subi data, -'0' ; Add 48 to data

; Update Array
;		ldi XL, low(array)        ; point Y at the string
;       ldi XH, high(array)       ; recall that we must multiply any Program code label address
                                        ; by 2 to get the correct location
;		;ldi temp, 48 
;
;		ldi counter, LENGTH
;		First: 
;		st X+, data
;		;inc temp
;		dec counter
;		brne First

;Right Shift Player Array
		ldi YL, low(player+13)        ; point Y at the string
		ldi YH, high(player+13)       ; recall that we must multiply any Program code label address

		ldi counter, LENGTH
		Second:
		ld temp, -Y
		std Y+1, temp
		dec counter
		brne Second

ldi XL, low(shockwave)
ldi XH, high(shockwave)
ld r20, X
cpi r20, 1
brge player_no_fire
rjmp shit1
player_no_fire:
;ldi r20, 0
dec r20
st X, r20
ldi data, ' '

rjmp shit1

notsecond:
rjmp notsecond1

secondloop:
rjmp secondloop1

shit1:

;Push element onto array
ldi XL, low(player)        ; point Y at the string
ldi XH, high(player) 
st X, data

;Left Shift Player Array
		ldi YL, low(cpu)
		ldi YH, high(cpu)

		ldd r20, Y+13 ;check if a weapon was fired on the last round

		ldi counter, LENGTH
		First:
		;adiw Y, 1
		;ld temp, Y
		;st -Y, temp
		
		adiw Y, 1
		ld temp, Y
		sbiw Y, 1
		st Y, temp
		;st X+, data
		adiw Y, 1
		
		dec counter
		brne First

; if a weapon was fired on the last round, do not fire.
cpi r20, '0'
brsh no_fire

ldi XL, low(probability)
ldi XH, high(probability)
ld r20, X

; otherwise, get a random number between 0 and 80.
random_loop:
rcall getRandom
cp temp, r20
brsh random_loop

; if the random number lies between 0 and 9 inclusive, fire. else don't fire. 
; thus, probability of firing a weapon on a timestep is 1/8
cpi temp, 10
brlo fire
no_fire:
ldi data, 32
rjmp push_array

fire:
ldi XL, low(shots_fired)
ldi XH, high(shots_fired)
ld r20, X
cpi r20, hurdle-1
;cpi r20, 3
brsh survived
inc r20
st X, r20
rjmp battle
survived:

rcall level_up

battle:

ldi data, '0'
add data, temp

push_array:

ldi XL, low(cpu+13)
ldi XH, high(cpu+13)
st X, data

rjmp shit

notsecond1:
rjmp notsecond2

secondloop1:
rjmp secondloop2

shit:
/*
ldi XL, low(player)
ldi XH, high(player)
ldi YL, low(state)
ldi YH, high(state)

ldi counter, LENGTH
compute_loop:
ld counter2, X
adiw X, LENGTH
ld counter3, X
sbiw X, LENGTH-1
ld r20, X
;adiw X, LENGTH-1
;sbiw X, LENGTH-1

cpi counter2, '0'
brsh out1

cpi counter3, '0'
brsh out2

;else
ldi temp, 32
st Y+, temp
rjmp cont

out1:
cpi counter3, '0'
brsh same_space

;cpi r20, '0'
;brsh adjacent

st Y+, counter2
rjmp cont

out2:
;ld r20, X
cpi r20, '0'
brsh adjacent
;st Y+, counter3
;rjmp cont
;kill:
;ldi r20, 32
st Y+, counter3
rjmp cont

adjacent:
cp counter3, r20
breq neutralize_all_adj
;ldi temp, 32
st Y+, counter3
;adiw X, 1
ldi temp, 32
st X, temp
;sbiw X, 1
rjmp cont

neutralize_all_adj:
ldi temp, 32
st Y+, temp
;sbiw X, 1
ldi temp, 32
st X, temp
adiw X, LENGTH-1
st X, temp
sbiw X, LENGTH-1
rjmp cont

same_space:
ldi temp, '*'
st Y+, temp

cp counter2, counter3
breq neutralize_all

sbiw X, 1
ldi temp, 32
st X, temp
adiw X, 1
rjmp cont

neutralize_all:
sbiw X, 1
ldi temp, 32
st X, temp
adiw X, LENGTH
st X, temp
sbiw X, LENGTH-1

cont:
dec counter
brne compute_loop
*/

ldi XL, low(player)
ldi XH, high(player)
ldi YL, low(state)
ldi YH, high(state)

ldi counter, LENGTH
compute_loop1:
ld counter2, X
adiw X, LENGTH
ld counter3, X
sbiw X, 1
ld r20, X
adiw X, 1
sbiw X, LENGTH-1

cpi counter2, '0'
brsh out1

cpi counter3, '0'
brsh out2

;else
ldi temp, 32
st Y+, temp
rjmp cont

out1:
cpi counter3, '0'
brsh same_space

cpi r20, '0'
brsh adjacent

st Y+, counter2
rjmp cont

rjmp skip_intermediate
compute_loop:
rjmp compute_loop1
skip_intermediate:

out2:
ld r20, X
cpi r20, '0'
brsh kill
st Y+, counter3
rjmp cont
kill:
ldi r20, 32
st Y+, r20
rjmp cont

adjacent:
ldi temp, 32
st Y+, temp
cp counter2, r20
breq neutralize_all_adj
sbiw X, 1
ldi temp, 32
st X, temp
adiw X, 1

rcall wrong_countermeasure

rjmp cont

neutralize_all_adj:
sbiw X, 1
ldi temp, 32
st X, temp
adiw X, LENGTH-1
st X, temp
sbiw X, LENGTH-2

rcall correct_countermeasure

rjmp cont

same_space:
ldi temp, '*'
st Y+, temp

cp counter2, counter3
breq neutralize_all

sbiw X, 1
ldi temp, 32
st X, temp
adiw X, 1

rcall wrong_countermeasure

rjmp cont

neutralize_all:
sbiw X, 1
ldi temp, 32
st X, temp
adiw X, LENGTH
st X, temp
sbiw X, LENGTH-1

rcall correct_countermeasure

cont:
dec counter
brne compute_loop

;Display Array
ldi counter, LENGTH               ; initialise counter 
;ldi XL, low(player)        ; point Y at the string
;ldi XH, high(player) 

ldi XL, low(state)
ldi XH, high(state)

rcall lcd_init

;rcall game_over

ldi data, 'D'
rcall lcd_wait_busy
rcall lcd_write_data

display_loop: 
        ld data, X+                    ; read a character from the string 
        rcall lcd_wait_busy
        rcall lcd_write_data            ; write the character to the screen
        dec counter                       ; decrement character counter
        brne display_loop                  ; loop again if there are more characters

ldi data, 'M'
rcall lcd_wait_busy
rcall lcd_write_data

        rcall lcd_wait_busy
        ldi data, LCD_ADDR_SET | LCD_LINE2
        rcall lcd_write_com                     ; move the insertion point to start of line 2

ldi data, 'B'
rcall lcd_wait_busy
rcall lcd_write_data

ldi XL, low(bases)
ldi XH, high(bases)
;ldi XL, low(game_started)
;ldi XH, high(game_started)

ld temp, X
rcall display_integer

ldi data, ' '
rcall lcd_wait_busy
rcall lcd_write_data
/*
ldi data, 'F'
rcall lcd_wait_busy
rcall lcd_write_data

ldi XL, low(shots_fired)
ldi XH, high(shots_fired)
ld temp, X
rcall display_integer

ldi data, ' '
rcall lcd_wait_busy
rcall lcd_write_data
*/
ldi data, 'L'
rcall lcd_wait_busy
rcall lcd_write_data

ldi XL, low(level)
ldi XH, high(level)
ld temp, X
rcall display_integer

ldi data, ' '
rcall lcd_wait_busy
rcall lcd_write_data

ldi data, 'S'
rcall lcd_wait_busy
rcall lcd_write_data

;rcall display_score

ldi XL, low(score)
ldi XH, high(score)
ld temp, X
rcall display_integer

ldi data, '0'
rcall lcd_wait_busy
rcall lcd_write_data
;ldi data, '0'
;add data, temp
;rcall lcd_wait_busy
;rcall lcd_write_data

/*
ldi XL, low(player)
ldi XH, high(player)

ldi data, 'D'
rcall lcd_wait_busy
rcall lcd_write_data

ldi counter, LENGTH

display_loop2:
        ld data, X+                    ; read a character from the string 
        rcall lcd_wait_busy
        rcall lcd_write_data            ; write the character to the screen
        dec counter                       ; decrement character counter
        brne display_loop2                  ; loop again if there are more characters

ldi data, 'M'
rcall lcd_wait_busy
rcall lcd_write_data
*/

;        ldi counter, LENGTH                       ; initialise counter 
;        ldi data, '1'                           ; initialise character to '1' 

;Display some other stuuf on the next line for testing
;display_loop2: 
;        rcall lcd_wait_busy
;        rcall lcd_write_data            ; write the character to the screen 
;        inc data                                        ; increment character
;        cpi data, '9'+1                         ; compare with first character > '9'
;        brlo skip                                       ; if character is now greater than '9'
;        ldi data, '0'                           ; change it back to '0' 
;skip:
;        dec counter                                       ; decrement character counter
;        brne display_loop2                         ; loop again if there are more characters

;ldi temp, 34
;rcall lcd_init
;rcall display_integer
;clr data

ldi XL, low(cpu)
ldi XH, high(cpu)
ld temp, X
cpi temp, '0'
brlo dont_kill_base
ldi XL, low(bases)
ldi XH, high(bases)
ld temp, X
dec temp
st X, temp
dont_kill_base:

ldi data, -16
ldi counter, 0
ldi counter2, 0
ldi counter3, 0

rjmp exit        ; go to exit

notsecond2: inc counter   ; if it is not a second, increment the counter
           rjmp exit

secondloop2: inc counter3 ; counting 100 for every 35 times := 35*100 := 3500
            cpi counter3,100 
            brne exit
	    inc counter2
	    ldi counter3,0                  
exit: 
pop r24                  ; Epilogue starts;
out SREG, r24            ; Restore all conflict registers from the stack.
pop r28
pop r29
pop r20
reti                     ; Return from the interrupt.

; main keeps scanning the keypad to find which key is pressed.
main:
ldi mask, INITCOLMASK ; initial column mask
clr col ; initial column
colloop:
out PORTC, mask ; set column to mask value
; (sets column 0 off)
ldi del_hi, high(255)
ldi del_lo, low(255)
rcall delay
in temp, PINC ; read PORTC
andi temp, ROWMASK ; read only the row bits
cpi temp, 0xF ; check if any rows are grounded
breq nextcol ; if not go to the next column
ldi mask, INITROWMASK ; initialise row check
clr row ; initial row
rowloop:
mov temp2, temp
and temp2, mask ; check masked bit
brne skipconv ; if the result is non-zero,
; we need to look again
rcall convert ; if bit is clear, convert the bitcode
jmp main ; and start again
skipconv:
inc row ; else move to the next row
lsl mask ; shift the mask to the next bit
jmp rowloop
nextcol:
cpi col, 3 ; check if we�re on the last column
breq main ; if so, no buttons were pushed,
; so start again.

sec ; else shift the column mask:
; We must set the carry bit
rol mask ; and then rotate left by a bit,
; shifting the carry into
; bit zero. We need this to make
; sure all the rows have
; pull-up resistors
inc col ; increment column value
jmp colloop ; and check the next column
; convert function converts the row and column given to a
; binary number and also outputs the value to PORTC.
; Inputs come from registers row and col and output is in
; temp.
convert:
cpi col, 3 ; if column is 3 we have a letter
breq letters
cpi row, 3 ; if row is 3 we have a symbol or 0
breq symbols
mov temp, row ; otherwise we have a number (1-9)
lsl temp ; temp = row * 2
add temp, row ; temp = row * 3
add temp, col ; add the column aDDRAss
; to get the offset from 1
inc temp ; add 1. Value of switch is
; row*3 + col + 1.
jmp convert_end
letters:
ldi temp, 0xA
add temp, row ; increment from 0xA by the row value
jmp convert_end
symbols:
cpi col, 0 ; check if we have a star
breq star
cpi col, 1 ; or if we have zero
breq zero
ldi temp, 0xF ; we'll output 0xF for hash
jmp convert_end
star:
ldi temp, 0xE ; we'll output 0xE for star
jmp convert_end
zero:
clr temp ; set to zero
convert_end:
mov data, temp
;rcall lcd_init
;rcall display_integer
;out PORTC, temp ; write value to PORTC
;rcall change_speed
;out PORTC, speed
ret ; return to caller

end:
	rjmp end

;Function lcd_write_com: Write a command to the LCD. The data reg stores the value to be written.
lcd_write_com:
push temp
out PORTD, data ; set the data port's value up
clr temp
out PORTA, temp ; RS = 0, RW = 0 for a command write
nop ; delay to meet timing (Set up time)
sbi PORTA, LCD_E ; turn on the enable pin
nop ; delay to meet timing (Enable pulse width)
nop
nop
cbi PORTA, LCD_E ; turn off the enable pin
nop ; delay to meet timing (Enable cycle time)
nop
nop
pop temp
ret
;Function lcd_write_data: Write a character to the LCD. The data reg stores the value to be written.
lcd_write_data:
push temp
push data
out PORTD, data ; set the data port's value up
ldi temp, 1 << LCD_RS
out PORTA, temp ; RS = 1, RW = 0 for a data write
nop ; delay to meet timing (Set up time)
sbi PORTA, LCD_E ; turn on the enable pin
nop ; delay to meet timing (Enable pulse width)
nop
nop
cbi PORTA, LCD_E ; turn off the enable pin
nop ; delay to meet timing (Enable cycle time)
nop
nop
pop data
pop temp
ret
;Function lcd_wait_busy: Read the LCD busy flag until it reads as not busy.
lcd_wait_busy:
push temp
clr temp
out DDRD, temp ; Make PORTD be an input port for now
out PORTD, temp
ldi temp, 1 << LCD_RW
out PORTA, temp ; RS = 0, RW = 1 for a command port read
busy_loop:
nop ; delay to meet timing (Set up time / Enable cycle time)
sbi PORTA, LCD_E ; turn on the enable pin
nop ; delay to meet timing (Data delay time)
nop
nop
in temp, PIND ; read value from LCD
cbi PORTA, LCD_E ; turn off the enable pin
sbrc temp, LCD_BF ; if the busy flag is set
rjmp busy_loop ; repeat command read
clr temp ; else
out PORTA, temp ; turn off read mode,
ser temp
out DDRD, temp ; make PORTD an output port again
pop temp
ret ; and return
; Function delay: Pass a number in registers r18:r19 to indicate how many microseconds
; must be delayed. Actual delay will be slightly greater (~1.08us*r18:r19).
; r18:r19 are altered in this function.
; Code is omitted
delay:
delay_loop:
subi del_lo, 1
sbci del_hi, 0
nop
nop
nop
nop
brne delay_loop
ret
;Function lcd_init Initialisation function for LCD.
lcd_init:
push temp
push data
ser temp
out DDRD, temp ; PORTD, the data port is usually all otuputs
out DDRA, temp ; PORTA, the control port is always all outputs
ldi del_lo, low(15000)
ldi del_hi, high(15000)
rcall delay ; delay for > 15ms
; Function set command with N = 1 and F = 0
ldi data, LCD_FUNC_SET | (1 << LCD_N)
rcall lcd_write_com ; 1st Function set command with 2 lines and 5*7 font
ldi del_lo, low(4100)
ldi del_hi, high(4100)
rcall delay ; delay for > 4.1ms
rcall lcd_write_com ; 2nd Function set command with 2 lines and 5*7 font
ldi del_lo, low(100)
ldi del_hi, high(100)
rcall delay ; delay for > 100us
rcall lcd_write_com ; 3rd Function set command with 2 lines and 5*7 font
rcall lcd_write_com ; Final Function set command with 2 lines and 5*7 font
rcall lcd_wait_busy ; Wait until the LCD is ready
ldi data, LCD_DISP_OFF
rcall lcd_write_com ; Turn Display off
rcall lcd_wait_busy ; Wait until the LCD is ready
ldi data, LCD_DISP_CLR
rcall lcd_write_com ; Clear Display
rcall lcd_wait_busy ; Wait until the LCD is ready
; Entry set command with I/D = 1 and S = 0
ldi data, LCD_ENTRY_SET | (1 << LCD_ID)
rcall lcd_write_com ; Set Entry mode: Increment = yes and Shift = no
rcall lcd_wait_busy ; Wait until the LCD is ready
; Display on command with C = 0 and B = 1
ldi data, LCD_DISP_ON | (1 << LCD_C)
rcall lcd_write_com ; Trun Display on with a cursor that doesn't blink
pop data
pop temp
ret

InitRandom:
push r16 ; save conflict register

in r16, TCNT1L ; Create random seed from time of timer 1
sts RAND,r16
sts RAND+2,r16
in r16,TCNT1H
sts RAND+1, r16
sts RAND+3, r16

pop r16 ; restore conflict register
ret

GetRandom:
push r0 ; save conflict registers
push r1
push r17
push r18
push r19
push r20
push r21
push r22

clr r22 ; remains zero throughout

ldi r16, low(RAND_C) ; set original value to be equal to C
ldi r17, BYTE2(RAND_C)
ldi r18, BYTE3(RAND_C)
ldi r19, BYTE4(RAND_C)

; calculate A*X + C where X is previous random number.  A is 3 bytes.
lds r20, RAND
ldi r21, low(RAND_A)
mul r20, r21 ; low byte of X * low byte of A
add r16, r0
adc r17, r1
adc r18, r22

ldi r21, byte2(RAND_A)
mul r20, r21  ; low byte of X * middle byte of A
add r17, r0
adc r18, r1
adc r19, r22

ldi r21, byte3(RAND_A)
mul r20, r21  ; low byte of X * high byte of A
add r18, r0
adc r19, r1

lds r20, RAND+1
ldi r21, low(RAND_A)
mul r20, r21  ; byte 2 of X * low byte of A
add r17, r0
adc r18, r1
adc r19, r22

ldi r21, byte2(RAND_A)
mul r20, r21  ; byte 2 of X * middle byte of A
add r18, r0
adc r19, r1

ldi r21, byte3(RAND_A)
mul r20, r21  ; byte 2 of X * high byte of A
add r19, r0

lds r20, RAND+2
ldi r21, low(RAND_A)
mul r20, r21  ; byte 3 of X * low byte of A
add r18, r0
adc r19, r1

ldi r21, byte2(RAND_A)
mul r20, r21  ; byte 2 of X * middle byte of A
add r19, r0

lds r20, RAND+3
ldi r21, low(RAND_A)	
mul r20, r21  ; byte 3 of X * low byte of A
add r19, r0

sts RAND, r16 ; store random number
sts RAND+1, r17
sts RAND+2, r18
sts RAND+3, r19

mov r16, r19  ; prepare result (bits 30-23 of random number X)
lsl r18
rol r16

pop r22 ; restore conflict registers
pop r21 
pop r20
pop r19
pop r18
pop r17
pop r1
pop r0
ret

display_integer:
push del_hi
push del_lo

clr del_lo
mov del_hi, r16

cpi del_hi, 100
brsh three_digit

cpi del_hi, 10
brsh two_digit

one_digit:
inc del_lo
dec del_hi
cpi del_hi, 0
breq output
rjmp one_digit

two_digit:
inc del_lo
subi del_hi, 10
cpi del_hi, 10
brlo output2
rjmp two_digit

three_digit:
inc del_lo
subi del_hi, 100
cpi del_hi, 100
brlo output3
rjmp three_digit

output3:
ldi data,'0'
add data, del_lo
rcall lcd_wait_busy
rcall lcd_write_data
clr del_lo
cpi del_hi, 10
brlo tens_digit_zero
rjmp two_digit

tens_digit_zero:
ldi data,'0'
rcall lcd_wait_busy
rcall lcd_write_data
rjmp one_digit

output2:
ldi data,'0'
add data, del_lo
rcall lcd_wait_busy
rcall lcd_write_data
clr del_lo
rjmp one_digit

output:
ldi data,'0'
add data, del_lo
rcall lcd_wait_busy
rcall lcd_write_data

pop del_lo
pop del_hi
ret

game_over:
push r30
push r31
push counter

;cli

rcall lcd_init

        ldi ZL, low(game_over_string << 1)        ; point Y at the string
        ldi ZH, high(game_over_string << 1)       ; recall that we must multiply any Program code label address
                                        ; by 2 to get the correct location
        ldi counter, 10               ; initialise counter 
game_over_loop: 
        lpm data, Z+                    ; read a character from the string 
        rcall lcd_wait_busy
        rcall lcd_write_data            ; write the character to the screen
        dec counter                       ; decrement character counter
        brne game_over_loop                  ; loop again if there are more characters

        rcall lcd_wait_busy
        ldi data, LCD_ADDR_SET | LCD_LINE2
        rcall lcd_write_com                     ; move the insertion point to start of line 2

        ldi ZL, low(final_score_string << 1)        ; point Y at the string
        ldi ZH, high(final_score_string << 1)       ; recall that we must multiply any Program code label address
                                        ; by 2 to get the correct location
        ldi counter, 12               ; initialise counter 
final_score_loop: 
        lpm data, Z+                    ; read a character from the string 
        rcall lcd_wait_busy
        rcall lcd_write_data            ; write the character to the screen
        dec counter                       ; decrement character counter
        brne final_score_loop                  ; loop again if there are more characters

ldi XL, low(score)
ldi XH, high(score)
ld temp, X
rcall display_integer

ldi data, '0'
rcall lcd_wait_busy
rcall lcd_write_data

rcall motor

pop counter
pop r31
pop r30
ret

start_screen:
push r30
push r31
push counter

;cli

rcall lcd_init

        ldi ZL, low(start_game_string1 << 1)        ; point Y at the string
        ldi ZH, high(start_game_string1 << 1)       ; recall that we must multiply any Program code label address
                                        ; by 2 to get the correct location
        ldi counter, 16               ; initialise counter 
start_screen_loop1: 
        lpm data, Z+                    ; read a character from the string 
        rcall lcd_wait_busy
        rcall lcd_write_data            ; write the character to the screen
        dec counter                       ; decrement character counter
        brne start_screen_loop1                  ; loop again if there are more characters

        rcall lcd_wait_busy
        ldi data, LCD_ADDR_SET | LCD_LINE2
        rcall lcd_write_com                     ; move the insertion point to start of line 2

        ldi ZL, low(start_game_string2 << 1)        ; point Y at the string
        ldi ZH, high(start_game_string2 << 1)       ; recall that we must multiply any Program code label address
                                        ; by 2 to get the correct location
        ldi counter, 16               ; initialise counter 
start_screen_loop2: 
        lpm data, Z+                    ; read a character from the string 
        rcall lcd_wait_busy
        rcall lcd_write_data            ; write the character to the screen
        dec counter                       ; decrement character counter
        brne start_screen_loop2                 ; loop again if there are more characters

pop counter
pop r31
pop r30
ret

correct_countermeasure:
push r28
push r29
push r20
push r16
push r17

;ldi r17, 10
; a silly compensation for my lack of ability to display 4-digit integers
ldi r17, 1

ldi YL, low(level)
ldi YH, high(level)
ld r20, Y

ldi YL, low(score)
ldi YH, high(score)
ld temp, Y
mul r20, r17
add temp, r0
brvs score_upperbound1
st Y+, temp
rjmp score_continue1 
score_upperbound1:
ldi temp, 255
st Y+, temp
score_continue1:

;st Y+, temp
ld temp, Y
adc temp, r1
st Y, temp

pop r17
pop r16
pop r20
pop r29
pop r28
ret

wrong_countermeasure:
push r28
push r29
push r20
push r16
push r17

ldi YL, low(shockwave)
ldi YH, high(shockwave)
ld r20, Y
ldi r20, 2
st Y, r20

;ldi r17, 10
; a silly compensation for my lack of ability to display 4-digit integers
ldi r17, 1

ldi YL, low(level)
ldi YH, high(level)
ld r20, Y

ldi YL, low(score)
ldi YH, high(score)
ld temp, Y
mul r20, r17
sub temp, r0

cpi temp, 1
brge decrease_score
ldi temp, 0
st Y, temp
rjmp score_continue
decrease_score:
st Y, temp

score_continue:

st Y+, temp
ld temp, Y
sbc temp, r1
st Y, temp


/*

ldi YL, low(level)
ldi YH, high(level)
ld r20, Y

ldi YL, low(score)
ldi YH, high(score)
ld temp, Y
sub temp, r20
st Y, temp
*/
pop r17
pop r16
pop r20
pop r29
pop r28
ret

level_up:
push r28
push r29
push r20
push r16
push r17

;Update Score

;ldi r17, 100
ldi r17, 10 ; a silly compensation for my lack of ability to display 4-digit integers

ldi YL, low(level)
ldi YH, high(level)
ld r20, Y
mul r20, r17

ldi YL, low(score)
ldi YH, high(score)
ld temp, Y
add temp, r0
;st Y+, temp
;cpi temp, 255
brvs score_upperbound2
st Y+, temp
rjmp score_continue2 
score_upperbound2:
ldi temp, 255
st Y+, temp

score_continue2:

ld temp, Y
adc temp, r1
st Y, temp

;Reset shots fired
ldi YL, low(shots_fired)
ldi YH, high(shots_fired)
ld r20, Y
ldi r20, 0
st Y, r20

;Rebuild bases
ldi YL, low(bases)
ldi YH, high(bases)
ld r20, Y
inc r20
cpi r20, 3
brsh bases_upper_bound
st Y, r20
rjmp bases_continue
bases_upper_bound:
ldi r20, 3
st Y, r20

bases_continue:

;Increase Level
ldi YL, low(level)
ldi YH, high(level)
ld r20, Y
inc r20
st Y, r20

ldi YL, low(timestep)
ldi YH, high(timestep)
ld r20, Y
subi r20, 3
cpi r20, 5
brge decrease_time
ldi r20, 2
st Y, r20
rjmp time_continue
decrease_time:
st Y, r20

time_continue:

ldi YL, low(probability)
ldi YH, high(probability)
ld r20, Y
subi r20, 15
cpi r20, 20
brsh increase_probability
ldi r20, 15
st Y, r20
rjmp prob_continue
increase_probability:
st Y, r20

prob_continue:

rcall sound

pop r17
pop r16
pop r20
pop r29
pop r28
ret

sound:
push temp
push r17
;ser temp

ldi temp, 0b00000000
out PORTB, temp

ldi r17, 3
sound_loop:
ldi del_hi, 0xFF;high(9000000)
ldi del_lo, 0xFF;low(9000000)
rcall delay
dec r17
brne sound_loop

ldi temp, FREQ2-50
out OCR2, temp

ldi r17, 3
sound_loop2:
ldi del_hi, 0xFF;high(9000000)
ldi del_lo, 0xFF;low(9000000)
rcall delay
dec r17
brne sound_loop2

ldi temp, FREQ2
out OCR2, temp

ldi r17, 3
sound_loop3:
ldi del_hi, 0xFF;high(9000000)
ldi del_lo, 0xFF;low(9000000)
rcall delay
dec r17
brne sound_loop3

ldi temp, 0b00000010
out PORTB, temp

;clr temp
;out PORTB, temp

pop r17
pop temp
ret

motor:
push temp
push r17
;ser temp

ldi temp, 0b00000011
out PORTB, temp

ldi r17, 30
motor_loop:
ldi del_hi, 0xFF;high(9000000)
ldi del_lo, 0xFF;low(9000000)
rcall delay
dec r17
brne motor_loop

ldi temp, 0b00000010
out PORTB, temp

pop r17
pop temp
ret
