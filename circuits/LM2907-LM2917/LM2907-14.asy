Version 4
SymbolType BLOCK
RECTANGLE Normal 112 160 -111 -160
ARC Normal -19 -175 13 -144 -19 -160 13 -160
TEXT -1 -72 Center 0 LM2907-14
TEXT -104 94 Left 0 6 NC
TEXT -104 144 Left 0 7 NC
TEXT 38 -95 Left 0 NC 13
TEXT 37 -144 Left 0 NC 14
WINDOW 0 0 -127 Center 0
SYMATTR Prefix X
SYMATTR SpiceModel LM2907-14
SYMATTR Description Frequency to Voltage Converter
SYMATTR ModelFile LM29x7.sub
PIN -112 -144 LEFT 8
PINATTR PinName 1 +IN
PINATTR SpiceOrder 1
PIN 112 0 RIGHT 8
PINATTR PinName -IN 11
PINATTR SpiceOrder 2
PIN -112 -96 LEFT 8
PINATTR PinName 2 C1
PINATTR SpiceOrder 3
PIN -112 -48 LEFT 8
PINATTR PinName 3 C2_R1
PINATTR SpiceOrder 4
PIN -112 0 LEFT 8
PINATTR PinName 4 CP+
PINATTR SpiceOrder 5
PIN 112 48 RIGHT 8
PINATTR PinName CP- 10
PINATTR SpiceOrder 6
PIN -112 48 LEFT 8
PINATTR PinName 5 EM_OT
PINATTR SpiceOrder 7
PIN 112 144 RIGHT 8
PINATTR PinName COL_OT 8
PINATTR SpiceOrder 8
PIN 112 96 RIGHT 8
PINATTR PinName VCC 9
PINATTR SpiceOrder 9
PIN 112 -48 RIGHT 8
PINATTR PinName GND 12
PINATTR SpiceOrder 10