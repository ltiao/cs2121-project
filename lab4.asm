.include "m64def.inc"
.def temp=r16
.def row =r17
.def col =r18
.def mask =r19
.def temp2 =r20
.def del_hi=r30
.def del_lo=r31
.def speed=r23
.def cycles = r21
.def counter = r22
.def counter2 = r26
.def counter3 = r27
.def temp3 = r14

.equ PORTADIR = 0xF0
.equ INITCOLMASK = 0xEF
.equ INITROWMASK = 0x01
.equ ROWMASK = 0x0F
.equ MAX = 0xFF
.equ max_speed = 255
.equ min_speed = 99

.cseg
jmp RESET
.org INT0addr ; INT0addr is the address of EXT_INT0
jmp EXT_INT0
jmp DEFAULT
jmp DEFAULT
jmp DEFAULT
jmp DEFAULT
jmp DEFAULT
jmp DEFAULT
jmp DEFAULT
jmp DEFAULT
jmp Timer2

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

ldi temp, PORTADIR ; columns are outputs, rows are inputs
out DDRA, temp
ser temp
out DDRC, temp ; Make PORTC all outputs
out PORTC, temp ; Turn on all the LEDs

ldi temp,0b00010000
out DDRB,temp
;ldi speed, 255

ldi speed, min_speed
out OCR0, speed

ldi temp, (1<< WGM00)|(1<<COM01)|(1<<CS00)     ; 
out TCCR0, temp

ldi temp, 1<<CS21
out TCCR2, temp
ldi temp, 1<<TOIE2
out TIMSK, temp

ldi temp, (2<<ISC00)
sts EICRA, temp
in temp, EIMSK
ori temp, (1<<INT0)
out EIMSK, temp

sei
jmp main


; interrupt place invoked by EXT interrupt0 when hole is at PB0
EXT_INT0:                  ; saving the temp value into the stack
push temp3
in temp3, SREG              ; inserting the SREG values into temp
push temp3                  ; saving the temp into stack
inc cycles
pop temp3                   ; taking out temp from stack which has SREG
out SREG, temp3             ; copy the values in temp into SREG
pop temp3
reti

Timer2:                  ; Prologue starts.
push r29                 ; Save all conflict registers in the prologue.
push r28
in r24, SREG
push r24                 ; Prologue ends.

/**** a counter for 3597 is needed to get one second-- Three counters are used in this example **************/                                          
                         ; 3597  (1 interrupt 278microseconds therefore 3597 interrupts needed for 1 sec)
cpi counter, 99          ; counting for 97
brne notsecond
 
cpi counter2, 8         ; counting for 35
brne secondloop          ; jumping into count 100 

ldi counter, 0
ldi counter2, 0
ldi counter3, 0

;lsr cycles
;lsr cycles
out PORTC, cycles
;out PORTC, speed
clr cycles
rjmp exit        ; go to exit

notsecond: inc counter   ; if it is not a second, increment the counter
           rjmp exit

secondloop: inc counter3 ; counting 100 for every 35 times := 35*100 := 3500
            cpi counter3,100 
            brne exit
	    inc counter2
	    ldi counter3,0                  
exit: 
pop r24                  ; Epilogue starts;
out SREG, r24            ; Restore all conflict registers from the stack.
pop r28
pop r29
reti                     ; Return from the interrupt.

; main keeps scanning the keypad to find which key is pressed.
main:
ldi mask, INITCOLMASK ; initial column mask
clr col ; initial column
colloop:
out PORTA, mask ; set column to mask value
; (sets column 0 off)
ldi del_hi, high(255)
ldi del_lo, low(255)
rcall delay
in temp, PINA ; read PORTA
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
cpi col, 3 ; check if we’re on the last column
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
;out PORTC, temp ; write value to PORTC
rcall change_speed
;out PORTC, speed
ret ; return to caller

change_speed:

; if the motor has been stopped, get it to 
; min speed before performing further operations
sbrs speed, 0
ldi speed, min_speed

cpi temp, 1
breq increase
cpi temp, 2
breq decrease
cpi temp, 3
breq stop

;resets the motor to spin at the speed of 20 rps
ldi speed, min_speed
rjmp update

increase:
cpi speed, max_speed
breq update
subi speed, -4
;inc speed
ldi del_hi, high(1000000)
ldi del_lo, low(1000000)
rcall delay
rjmp update

decrease:
cpi speed, min_speed
breq update
subi speed, 4
;dec speed
ldi del_hi, high(100000)
ldi del_lo, low(100000)
rcall delay
rjmp update

stop:
ldi speed, 0

update:
out OCR0, speed
ret
/*
change_speed:
cpi speed, max
brne increase
ldi speed, 0
out OCR0, speed
increase:
inc speed
;subi speed, -5
out OCR0, speed
ret
*/
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
