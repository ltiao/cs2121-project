.include "m64def.inc"
.def temp =r16
.def row =r17
.def col =r18
.def mask =r19
.def temp2 =r20
.def n = r21
.def sum = r22
.def temp3 = r23
.def data = r24
.def del_lo = r25
.def del_hi = r26
;Keypad constants
.equ PORTDDIR = 0xF0
.equ INITCOLMASK = 0xEF
.equ INITROWMASK = 0x01
.equ ROWMASK = 0x0F
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

.cseg
jmp RESET
RESET:
ldi temp, low(RAMEND)
out SPL, temp
ldi temp, high(RAMEND)
out SPH, temp
rcall lcd_init
ldi temp, PORTDDIR ; columns are outputs, rows are inputs
out DDRD, temp
ser temp
out DDRC, temp ; Make PORTC all outputs
out PORTC, temp ; Turn on all the LEDs
clr n
clr sum
; main keeps scanning the keypad to find which key is pressed.
main:
ldi mask, INITCOLMASK ; initial column mask
clr col ; initial column
colloop:
out PORTD, mask ; set column to mask value
; (sets column 0 off)
ldi temp, 0xFF ; implement a delay so the
; hardware can stabilize
delay0:
dec temp
brne delay0
in temp, PIND ; read PORTD
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
add temp, col ; add the column address
; to get the offset from 1
inc temp ; add 1. Value of switch is
; row*3 + col + 1.
ldi del_lo, 10
mul n, del_lo
mov n, r0
add n, temp

ldi del_lo, low(10000000)
ldi del_hi, high(10000000)
rcall delay ; delay for a long time
rcall delay ; delay for a long time

;oneSecondDelay
;oneSecondDelay
;oneSecondDelay

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

;ldi temp, 0xF ; we'll output 0xF for hash

add sum, n
clr n

clr del_lo
mov del_hi, sum

cpi del_hi, 100
brge three_digit

cpi del_hi, 10
brge two_digit

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
brlt output2
rjmp two_digit

three_digit:
inc del_lo
subi del_hi, 100
cpi del_hi, 100
brlt output3
rjmp three_digit

output3:
ldi data,'0'
add data, del_lo
rcall lcd_wait_busy
rcall lcd_write_data
;ldi del_lo, low(10000000)
;ldi del_hi, high(10000000)
;rcall delay ; delay for a long time
;rcall delay ; delay for a long time
clr del_lo
;mov del_hi, sum
rjmp two_digit

output2:
ldi data,'0'
add data, del_lo
rcall lcd_wait_busy
rcall lcd_write_data
;ldi del_lo, low(10000000)
;ldi del_hi, high(10000000)
;rcall delay ; delay for a long time
;rcall delay ; delay for a long time
clr del_lo
;mov del_hi, sum
rjmp one_digit

output:
ldi data,'0'
add data, del_lo
rcall lcd_wait_busy
rcall lcd_write_data

ldi del_lo, low(10000000)
ldi del_hi, high(10000000)
rcall delay ; delay for a long time
rcall delay ; delay for a long time

jmp convert_end

star:
ldi temp, 0xE ; we'll output 0xE for star

jmp convert_end

zero:
clr temp ; set to zero

convert_end:
out PORTC, sum
ldi del_lo, low(10000000)
ldi del_hi, high(10000000)
rcall delay ; delay for a long time
rcall delay ; delay for a long time

;oneSecondDelay
;oneSecondDelay
;oneSecondDelay
;out PORTC, sum ; write value to PORTC
ret ; return to caller

;Function lcd_write_com: Write a command to the LCD. The data reg stores the value to be written.
lcd_write_com:
out PORTB, data ; set the data port's value up
clr temp3
out PORTA, temp3 ; RS = 0, RW = 0 for a command write
nop ; delay to meet timing (Set up time)
sbi PORTA, LCD_E ; turn on the enable pin
nop ; delay to meet timing (Enable pulse width)
nop
nop
cbi PORTA, LCD_E ; turn off the enable pin
nop ; delay to meet timing (Enable cycle time)
nop
nop
ret
;Function lcd_write_data: Write a character to the LCD. The data reg stores the value to be written.
lcd_write_data:
out PORTB, data ; set the data port's value up
ldi temp3, 1 << LCD_RS
out PORTA, temp3 ; RS = 1, RW = 0 for a data write
nop ; delay to meet timing (Set up time)
sbi PORTA, LCD_E ; turn on the enable pin
nop ; delay to meet timing (Enable pulse width)
nop
nop
cbi PORTA, LCD_E ; turn off the enable pin
nop ; delay to meet timing (Enable cycle time)
nop
nop
ret
;Function lcd_wait_busy: Read the LCD busy flag until it reads as not busy.
lcd_wait_busy:
clr temp3
out DDRB, temp3 ; Make PORTB be an input port for now
out PORTB, temp3
ldi temp3, 1 << LCD_RW
out PORTA, temp3 ; RS = 0, RW = 1 for a command port read
busy_loop:
nop ; delay to meet timing (Set up time / Enable cycle time)
sbi PORTA, LCD_E ; turn on the enable pin
nop ; delay to meet timing (Data delay time)
nop
nop
in temp3, PINB ; read value from LCD
cbi PORTA, LCD_E ; turn off the enable pin
sbrc temp3, LCD_BF ; if the busy flag is set
rjmp busy_loop ; repeat command read
clr temp3 ; else
out PORTA, temp3 ; turn off read mode,
ser temp3
out DDRB, temp3 ; make PORTB an output port again
ret ; and return
; Function delay: Pass a number in registers r18:r19 to indicate how many microseconds
; must be delayed. Actual delay will be slightly greater (~1.08us*r18:r19).
; r18:r19 are altered in this function.
delay:
loop:
subi del_lo, 1
sbci del_hi, 0
nop
nop
nop
nop
brne loop
ret
;Function lcd_init Initialisation function for LCD.
lcd_init:
ser temp3
out DDRB, temp3 ; PORTB, the data port is usually all otuputs
out DDRA, temp3 ; PORTA, the control port is always all outputs
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
ret

