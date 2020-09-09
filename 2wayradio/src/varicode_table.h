//==========================================================================
// Name:            varicode_table.h
// Purpose:         Varicode look up table
// Created:         Nov 24, 2012
// Authors:         Clint Turner, KA7OEI,  Peter Martinez, G3PLX
//
// License:
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License version 2.1,
//  as published by the Free Software Foundation.  This program is
//  distributed in the hope that it will be useful, but WITHOUT ANY
//  WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
//  License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, see <http://www.gnu.org/licenses/>.
//
//==========================================================================

/* The following table defines the PKS31 varicode.  There are 128 entries,
corresponding to ASCII characters 0-127 with two bytes for each entry.  The bits
for the varicode are to be shifted out MSB-first for both bytes, with the first byte
in the table being the first one to be sent.

More than one zero in sequence signifies the end of the character (i.e.
two zeroes are the intercharacter sequence, so at least two zeroes should always be
sent before the next character is sent.

This file is constructed with information from the article "PSK31 Fundamentals"
by Peter Martinez, G3PLX by Clint Turner, KA7OEI
*/
#define b00000000 0x00
#define b00000001 0x01
#define b00000010 0x02
#define b00000011 0x03
#define b00000100 0x04
#define b00000101 0x05
#define b00000110 0x06
#define b00000111 0x07
#define b00001000 0x08
#define b00001001 0x09
#define b00001010 0x0A
#define b00001011 0xb
#define b00001100 0x0C
#define b00001101 0x0D
#define b00001110 0x0E
#define b00001111 0x0F

#define b00010000 0x10
#define b00010001 0x11
#define b00010010 0x12
#define b00010011 0x13
#define b00010100 0x14
#define b00010101 0x15
#define b00010110 0x16
#define b00010111 0x17
#define b00011000 0x18
#define b00011001 0x19
#define b00011010 0x1A
#define b00011011 0x1B
#define b00011100 0x1C
#define b00011101 0x1D
#define b00011110 0x1E
#define b00011111 0x1F

#define b00100000 0x20
#define b00100001 0x21
#define b00100010 0x22
#define b00100011 0x23
#define b00100100 0x24
#define b00100101 0x25
#define b00100110 0x26
#define b00100111 0x27
#define b00101000 0x28
#define b00101001 0x29
#define b00101010 0x2A
#define b00101011 0x2B
#define b00101100 0x2C
#define b00101101 0x2D
#define b00101110 0x2E
#define b00101111 0x2F

#define b00110000 0x30
#define b00110001 0x31
#define b00110010 0x32
#define b00110011 0x33
#define b00110100 0x34
#define b00110101 0x35
#define b00110110 0x36
#define b00110111 0x37
#define b00111000 0x38
#define b00111001 0x39
#define b00111010 0x3A
#define b00111011 0x3B
#define b00111100 0x3C
#define b00111101 0x3D
#define b00111110 0x3E
#define b00111111 0x3F

#define b01000000 0x40
#define b01000001 0x41
#define b01000010 0x42
#define b01000011 0x43
#define b01000100 0x44
#define b01000101 0x45
#define b01000110 0x46
#define b01000111 0x47
#define b01001000 0x48
#define b01001001 0x49
#define b01001010 0x4A
#define b01001011 0x4B
#define b01001100 0x4C
#define b01001101 0x4D
#define b01001110 0x4E
#define b01001111 0x4F

#define b01010000 0x50
#define b01010001 0x51
#define b01010010 0x52
#define b01010011 0x53
#define b01010100 0x54
#define b01010101 0x55
#define b01010110 0x56
#define b01010111 0x57
#define b01011000 0x58
#define b01011001 0x59
#define b01011010 0x5A
#define b01011011 0x5B
#define b01011100 0x5C
#define b01011101 0x5D
#define b01011110 0x5E
#define b01011111 0x5F

#define b01100000 0x60
#define b01100001 0x61
#define b01100010 0x62
#define b01100011 0x63
#define b01100100 0x64
#define b01100101 0x65
#define b01100110 0x66
#define b01100111 0x67
#define b01101000 0x68
#define b01101001 0x69
#define b01101010 0x6A
#define b01101011 0x6B
#define b01101100 0x6C
#define b01101101 0x6D
#define b01101110 0x6E
#define b01101111 0x6F

#define b01110000 0x70
#define b01110001 0x71
#define b01110010 0x72
#define b01110011 0x73
#define b01110100 0x74
#define b01110101 0x75
#define b01110110 0x76
#define b01110111 0x77
#define b01111000 0x78
#define b01111001 0x79
#define b01111010 0x7A
#define b01111011 0x7B
#define b01111100 0x7C
#define b01111101 0x7D
#define b01111110 0x7E
#define b01111111 0x7F

#define b10000000 0x80
#define b10000001 0x81
#define b10000010 0x82
#define b10000011 0x83
#define b10000100 0x84
#define b10000101 0x85
#define b10000110 0x86
#define b10000111 0x87
#define b10001000 0x88
#define b10001001 0x89
#define b10001010 0x8A
#define b10001011 0x8B
#define b10001100 0x8C
#define b10001101 0x8D
#define b10001110 0x8E
#define b10001111 0x8F

#define b10010000 0x90
#define b10010001 0x91
#define b10010010 0x92
#define b10010011 0x93
#define b10010100 0x94
#define b10010101 0x95
#define b10010110 0x96
#define b10010111 0x97
#define b10011000 0x98
#define b10011001 0x99
#define b10011010 0x9A
#define b10011011 0x9B
#define b10011100 0x9C
#define b10011101 0x9D
#define b10011110 0x9E
#define b10011111 0x9F

#define b10100000 0xA0
#define b10100001 0xA1
#define b10100010 0xA2
#define b10100011 0xA3
#define b10100100 0xA4
#define b10100101 0xA5
#define b10100110 0xA6
#define b10100111 0xA7
#define b10101000 0xA8
#define b10101001 0xA9
#define b10101010 0xAA
#define b10101011 0xAB
#define b10101100 0xAC
#define b10101101 0xAD
#define b10101110 0xAE
#define b10101111 0xAF

#define b10110000 0xB0
#define b10110001 0xB1
#define b10110010 0xB2
#define b10110011 0xB3
#define b10110100 0xB4
#define b10110101 0xB5
#define b10110110 0xB6
#define b10110111 0xB7
#define b10111000 0xB8
#define b10111001 0xB9
#define b10111010 0xBA
#define b10111011 0xBB
#define b10111100 0xBC
#define b10111101 0xBD
#define b10111110 0xBE
#define b10111111 0xBF

#define b11000000 0xC0
#define b11000001 0xC1
#define b11000010 0xC2
#define b11000011 0xC3
#define b11000100 0xC4
#define b11000101 0xC5
#define b11000110 0xC6
#define b11000111 0xC7
#define b11001000 0xC8
#define b11001001 0xC9
#define b11001010 0xCA
#define b11001011 0xCB
#define b11001100 0xCC
#define b11001101 0xCD
#define b11001110 0xCE
#define b11001111 0xCF

#define b11010000 0xD0
#define b11010001 0xD1
#define b11010010 0xD2
#define b11010011 0xD3
#define b11010100 0xD4
#define b11010101 0xD5
#define b11010110 0xD6
#define b11010111 0xD7
#define b11011000 0xD8
#define b11011001 0xD9
#define b11011010 0xDA
#define b11011011 0xDB
#define b11011100 0xDC
#define b11011101 0xDD
#define b11011110 0xDE
#define b11011111 0xDF

#define b11100000 0xE0
#define b11100001 0xE1
#define b11100010 0xE2
#define b11100011 0xE3
#define b11100100 0xE4
#define b11100101 0xE5
#define b11100110 0xE6
#define b11100111 0xE7
#define b11101000 0xE8
#define b11101001 0xE9
#define b11101010 0xEA
#define b11101011 0xEB
#define b11101100 0xEC
#define b11101101 0xED
#define b11101110 0xEE
#define b11101111 0xEF

#define b11110000 0xF0
#define b11110001 0xF1
#define b11110010 0xF2
#define b11110011 0xF3
#define b11110100 0xF4
#define b11110101 0xF5
#define b11110110 0xF6
#define b11110111 0xF7
#define b11111000 0xF8
#define b11111001 0xF9
#define b11111010 0xFA
#define b11111011 0xFB
#define b11111100 0xFC
#define b11111101 0xFD
#define b11111110 0xFE
#define b11111111 0xFF

unsigned char const varicode_table1[256] =	{
    b10101010,
    b11000000, // 0 NUL
    b10110110,
    b11000000, // 1 SOH
    b10111011,
    b01000000, // 2 STX
    b11011101,
    b11000000, // 3 ETX
    b10111010,
    b11000000, // 4 EOT
    b11010111,
    b11000000, // 5 ENQ
    b10111011,
    b11000000, // 6 ACK
    b10111111,
    b01000000, // 7 BEL
    b10111111,
    b11000000, // 8 BS
    b11101111,
    b00000000, // 9 HT
    b11101000,
    b00000000, // 10 LF
    b11011011,
    b11000000, // 11 VT
    b10110111,
    b01000000, // 12 FF
    b11111000,
    b00000000, // 13 CR
    b11011101,
    b01000000, // 14 SO
    b11101010,
    b11000000, // 15 SI
    b10111101,
    b11000000, // 16 DLE
    b10111101,
    b01000000, // 17 DC1
    b11101011,
    b01000000, // 18 DC2
    b11101011,
    b11000000, // 19 DC3
    b11010110,
    b11000000, // 20 DC4
    b11011010,
    b11000000, // 21 NAK
    b11011011,
    b01000000, // 22 SYN
    b11010101,
    b11000000, // 23 ETB
    b11011110,
    b11000000, // 24 CAN
    b11011111,
    b01000000, // 25 EM
    b11101101,
    b11000000, // 26 SUB
    b11010101,
    b01000000, // 27 ESC
    b11010111,
    b01000000, // 28 FS
    b11101110,
    b11000000, // 29 GS
    b10111110,
    b11000000, // 30 RS
    b11011111,
    b11000000, // 31 US
    b10000000,
    b00000000, // 32 SP
    b11111111,
    b10000000, // 33 !
    b10101111,
    b10000000, // 34 "
    b11111010,
    b10000000, // 35 #
    b11101101,
    b10000000, // 36 $
    b10110101,
    b01000000, // 37 %
    b10101110,
    b11000000, // 38 &
    b10111111,
    b10000000, // 39 '
    b11111011,
    b00000000, // 40 (
    b11110111,
    b00000000, // 41 )
    b10110111,
    b10000000, // 42 *
    b11101111,
    b10000000, // 43 +
    b11101010,
    b00000000, // 44 ,
    b11010100,
    b00000000, // 45 -
    b10101110,
    b00000000, // 46 .
    b11010111,
    b10000000, // 47 /
    b10110111,
    b00000000, // 48 0
    b10111101,
    b00000000, // 49 1
    b11101101,
    b00000000, // 50 2
    b11111111,
    b00000000, // 51 3
    b10111011,
    b10000000, // 52 4
    b10101101,
    b10000000, // 53 5
    b10110101,
    b10000000, // 54 6
    b11010110,
    b10000000, // 55 7
    b11010101,
    b10000000, // 56 8
    b11011011,
    b10000000, // 57 9
    b11110101,
    b00000000, // 58 :
    b11011110,
    b10000000, // 59 ;
    b11110110,
    b10000000, // 60 <
    b10101010,
    b00000000, // 61 =
    b11101011,
    b10000000, // 62 >
    b10101011,
    b11000000, // 63 ?
    b10101111,
    b01000000, // 64 @
    b11111010,
    b00000000, // 65 A
    b11101011,
    b00000000, // 66 B
    b10101101,
    b00000000, // 67 C
    b10110101,
    b00000000, // 68 D
    b11101110,
    b00000000, // 69 E
    b11011011,
    b00000000, // 70 F
    b11111101,
    b00000000, // 71 G
    b10101010,
    b10000000, // 72 H
    b11111110,
    b00000000, // 73 I
    b11111110,
    b10000000, // 74 J
    b10111110,
    b10000000, // 75 K
    b11010111,
    b00000000, // 76 L
    b10111011,
    b00000000, // 77 M
    b11011101,
    b00000000, // 78 N
    b10101011,
    b00000000, // 79 O
    b11010101,
    b00000000, // 80 P
    b11101110,
    b10000000, // 81 Q
    b10101111,
    b00000000, // 82 R
    b11011110,
    b00000000, // 83 S
    b11011010,
    b00000000, // 84 T
    b10101011,
    b10000000, // 85 U
    b11011010,
    b10000000, // 86 V
    b10101110,
    b10000000, // 87 W
    b10111010,
    b10000000, // 88 X
    b10111101,
    b10000000, // 89 Y
    b10101011,
    b01000000, // 90 Z
    b11111011,
    b10000000, // 91 [
    b11110111,
    b10000000, // 92 "\"
    b11111101,
    b10000000, // 93 ]
    b10101111,
    b11000000, // 94 ^
    b10110110,
    b10000000, // 95 _ (underline)
    b10110111,
    b11000000, // 96 `
    b10110000,
    b00000000, // 97 a
    b10111110,
    b00000000, // 98 b
    b10111100,
    b00000000, // 99 c
    b10110100,
    b00000000, // 100 d
    b11000000,
    b00000000, // 101 e
    b11110100,
    b00000000, // 102 f
    b10110110,
    b00000000, // 103 g
    b10101100,
    b00000000, // 104 h
    b11010000,
    b00000000, // 105 i
    b11110101,
    b10000000, // 106 j
    b10111111,
    b00000000, // 107 k
    b11011000,
    b00000000, // 108 l
    b11101100,
    b00000000, // 109 m
    b11110000,
    b00000000, // 110 n
    b11100000,
    b00000000, // 111 o
    b11111100,
    b00000000, // 112 p
    b11011111,
    b10000000, // 113 q
    b10101000,
    b00000000, // 114 r
    b10111000,
    b00000000, // 115 s
    b10100000,
    b00000000, // 116 t
    b11011100,
    b00000000, // 117 u
    b11110110,
    b00000000, // 118 v
    b11010110,
    b00000000, // 119 w
    b11011111,
    b00000000, // 120 x
    b10111010,
    b00000000, // 121 y
    b11101010,
    b10000000, // 122 z
    b10101101,
    b11000000, // 123 {
    b11011101,
    b10000000, // 124 |
    b10101101,
    b01000000, // 125 }
    b10110101,
    b11000000, // 126 ~
    b11101101,
    b01000000, // 127 (del)
};

// This code was used on FDMDV version 1, and is more compact that Code 1, but only covers a subset
// of the ASCII cahacter set

char const varicode_table2[] = {
' ' ,b11000000,
    13  ,b01000000, // CR, end of message
    '=' ,b10000000,
    '1' ,b11110000,
    '2' ,b01110000,
    '3' ,b10110000,
    '4' ,b11010000,
    '5' ,b01010000,
    '6' ,b10010000,
    '7' ,b11100000,
    '8' ,b01100000,
    '9' ,b10100000,
    'a' ,b11111100,
    'b' ,b01111100,
    'c' ,b10111100,
    'd' ,b11011100,
    'e' ,b01011100,
    'f' ,b10011100,
    'g' ,b11101100,
    'h' ,b01101100,
    'i' ,b10101100,
    'j' ,b11110100,
    'k' ,b01110100,
    'l' ,b10110100,
    'm' ,b11010100,
    'n' ,b01010100,
    'o' ,b10010100,
    'p' ,b11100100,
    'q' ,b01100100,
    'r' ,b10100100,
    's' ,b11111000,
    't' ,b01111000,
    'u' ,b10111000,
    'v' ,b11011000,
    'w' ,b01011000,
    'x' ,b10011000,
    'y' ,b11101000,
    'z' ,b01101000,
    '0' ,b10101000
};

