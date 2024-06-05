/**
@file       slcd.c

@brief      SLCD interface
*/

#include "MKL46Z4.h"
#include "lcd.h"



// Mask bit array for setting specific bits in registers.
const unsigned long int MASK_BIT[32] =
{
    0x00000001, 0x00000002, 0x00000004, 0x00000008,
    0x00000010, 0x00000020, 0x00000040, 0x00000080,
    0x00000100, 0x00000200, 0x00000400, 0x00000800,
    0x00001000, 0x00002000, 0x00004000, 0x00008000,
    0x00010000, 0x00020000, 0x00040000, 0x00080000,
    0x00100000, 0x00200000, 0x00400000, 0x00800000,
    0x01000000, 0x02000000, 0x04000000, 0x08000000,
    0x10000000, 0x20000000, 0x40000000, 0x80000000,
};

const unsigned char WF_ORDERING_TABLE[ ] =
{
    CHAR1a,     // LCD81 --- Pin:5   LCDnAddress=51
    CHAR1b,     // LCD82 --- Pin:6   LCDnAddress=52
    CHAR2a,     // LCD83 --- Pin:7   LCDnAddress=53
    CHAR2b,     // LCD84 --- Pin:8   LCDnAddress=54
    CHAR3a,     // LCD85 --- Pin:9   LCDnAddress=55
    CHAR3b,     // LCD86 --- Pin:10  LCDnAddress=56
    CHAR4a,     // LCD87 --- Pin:11  LCDnAddress=57
    CHAR4b,     // LCD88 --- Pin:12  LCDnAddress=58
    CHARCOM0,   // LCD77 --- Pin:1   LCDnAddress=4D
    CHARCOM1,   // LCD78 --- Pin:2   LCDnAddress=4E
    CHARCOM2,   // LCD79 --- Pin:3   LCDnAddress=4F
    CHARCOM3,   // LCD80 --- Pin:4   LCDnAddress=50

};

// ASCII to waveform codification table for 8x6 dot matrix.
const char ASCII_TO_WF_CODIFICATION_TABLE[] = {
    // Char = 0
    SEGD | SEGE | SEGF | !SEGG, SEGC | SEGB | SEGA,
    // Char = 1
    !SEGD | !SEGE | !SEGF | !SEGG, SEGC | SEGB | !SEGA,
    // Char = 2
    SEGD | SEGE | !SEGF | SEGG, !SEGC | SEGB | SEGA,
    // Char = 3
    SEGD | !SEGE | !SEGF | SEGG, SEGC | SEGB | SEGA,
    // Char = 4
    !SEGD | !SEGE | SEGF | SEGG, SEGC | SEGB | !SEGA,
    // Char = 5
    SEGD | !SEGE | SEGF | SEGG, SEGC | !SEGB | SEGA,
    // Char = 6
    SEGD | SEGE | SEGF | SEGG, SEGC | !SEGB | SEGA,
    // Char = 7
    !SEGD | !SEGE | !SEGF | !SEGG, SEGC | SEGB | SEGA,
    // Char = 8
    SEGD | SEGE | SEGF | SEGG, SEGC | SEGB | SEGA,
    // Char = 9
    SEGD | !SEGE | SEGF | SEGG, SEGC | SEGB | SEGA,
    // Char = : (colon)
    !SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | !SEGA,
    // Char = ; (semicolon)
    !SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | !SEGA,
    // Char = < (less than)
    !SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | !SEGA,
    // Char = = (equals)
    SEGD | !SEGE | !SEGF | SEGG, !SEGC | !SEGB | !SEGA,
    // Char = > (greater than)
    !SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | !SEGA,
    // Char = ? (question mark)
    !SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | SEGA,
    // Char = @ (at symbol)
    SEGD | SEGE | SEGF | SEGG, SEGC | SEGB | SEGA,
    // Char = A
    !SEGD | SEGE | SEGF | SEGG, SEGC | SEGB | SEGA,
    // Char = B
    SEGD | SEGE | SEGF | SEGG, SEGC | !SEGB | !SEGA,
    // Char = C
    SEGD | SEGE | SEGF | !SEGG, !SEGC | !SEGB | SEGA,
    // Char = D
    SEGD | SEGE | !SEGF | SEGG, SEGC | SEGB | SEGA,
    // Char = E
    SEGD | SEGE | SEGF | SEGG, !SEGC | !SEGB | SEGA,
    // Char = F
    !SEGD | SEGE | SEGF | SEGG, !SEGC | !SEGB | SEGA,
    // Char = G
    SEGD | SEGE | SEGF | SEGG, SEGC | !SEGB | SEGA,
    // Char = H
    !SEGD | SEGE | SEGF | SEGG, SEGC | SEGB | !SEGA,
    // Char = I
    !SEGD | !SEGE | !SEGF | !SEGG, SEGC | !SEGB | !SEGA,
    // Char = J
    SEGD | SEGE | !SEGF | !SEGG, SEGC | SEGB | !SEGA,
    // Char = K
    !SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | !SEGA,
    // Char = L
    SEGD | SEGE | SEGF | !SEGG, !SEGC | !SEGB | !SEGA,
    // Char = M
    !SEGD | SEGE | SEGF | !SEGG, SEGC | SEGB | !SEGA,
    // Char = N
    !SEGD | SEGE | !SEGF | SEGG, SEGC | !SEGB | !SEGA,
    // Char = O
    SEGD | SEGE | !SEGF | SEGG, SEGC | !SEGB | !SEGA,
    // Char = P
    !SEGD | SEGE | SEGF | SEGG, !SEGC | SEGB | SEGA,
    // Char = Q
    SEGD | !SEGE | SEGF | SEGG, SEGC | SEGB | SEGA,
    // Char = R
    !SEGD | SEGE | SEGF | SEGG, SEGC | SEGB | SEGA,
    // Char = S
    SEGD | !SEGE | SEGF | SEGG, SEGC | !SEGB | SEGA,
    // Char = T
    SEGD | SEGE | SEGF | SEGG, !SEGC | !SEGB | !SEGA,
    // Char = U
    SEGD | SEGE | SEGF | !SEGG, SEGC | SEGB | !SEGA,
    // Char = V
    !SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | !SEGA,
    // Char = W
    !SEGD | SEGE | SEGF | !SEGG, SEGC | SEGB | !SEGA,
    // Char = X
    !SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | !SEGA,
    // Char = Y
    !SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | !SEGA,
    // Char = Z
    SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | SEGA,
    // Char = [
    SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | SEGA,
    // Char = \
    SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | SEGA,
    // Char = ]
    SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | SEGA,
    // Char = ^
    SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | SEGA,
    // Char = _
    SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | SEGA,
    // Char = `
    SEGD | !SEGE | !SEGF | !SEGG, !SEGC | !SEGB | SEGA,
};

unsigned char bLCD_CharPosition;

// SLCD clock configuration
void SLCD_ConfigureClock(void) {
    // Enable IRCLK (Internal Reference Clock)
    MCG->C1 = MCG_C1_IRCLKEN_MASK | MCG_C1_IREFSTEN_MASK;
}

// SLCD pin setup
void SLCD_SetupPins(void) {
    // Enable clocks for SLCD module and relevant GPIO ports
    SIM->SCGC5 |= SIM_SCGC5_SLCD_MASK | SIM_SCGC5_PORTC_MASK;

    // Configure pins for LCD operation
    PORTC->PCR[20] = 0x00000000; // VLL2
    PORTC->PCR[21] = 0x00000000; // VLL1
    PORTC->PCR[22] = 0x00000000; // VCAP2
    PORTC->PCR[23] = 0x00000000; // VCAP1
}

// SLCD GCR register configuration
void SLCD_ConfigureGCR(void) {
    // Configuration of LCD GCR register
    LCD->GCR = (LCD_GCR_RVEN_MASK  |
                LCD_GCR_RVTRIM(8) |// CPSEL = 1     0 -- 8000 pf 1 -- 6000 pf  2 -- 4000 pf  3 -- 2000 pf
                LCD_GCR_CPSEL_MASK * _LCDCPSEL |
                LCD_GCR_LADJ(3) |// CPSEL = 1     0 -- 8000 pf 1 -- 6000 pf  2 -- 4000 pf  3 -- 2000 pf
                LCD_GCR_VSUPPLY_MASK |
                LCD_GCR_ALTDIV(0) | // CPSEL = 1     0 -- 8000 pf 1 -- 6000 pf  2 -- 4000 pf  3 -- 2000 pf
                LCD_GCR_SOURCE_MASK * _LCDCLKSOURCE |
                LCD_GCR_ALTSOURCE_MASK * _LCDALRCLKSOURCE |
                LCD_GCR_LCLK(1) |// CPSEL = 1     0 -- 8000 pf 1 -- 6000 pf  2 -- 4000 pf  3 -- 2000 pf
                LCD_GCR_DUTY(_LCDDUTY));
}

void SLCD_EnablePins(void)
{
    unsigned char 		i;
   	unsigned long int *p_pen;
   	unsigned char 		pen_offset;   // 0 or 1   
   	unsigned char 		pen_bit;      // 0 to 31

   	LCD->PEN[0]	 = 0x0;
   	LCD->PEN[1]  = 0x0;
   	LCD->BPEN[0] = 0x0;
   	LCD->BPEN[1] = 0x0;
   
   	p_pen = (unsigned long int *)&LCD->PEN[0];

    for (i=0;i<_LCDUSEDPINS;i++) 
    {
      	pen_offset = WF_ORDERING_TABLE[i]/32;
      	pen_bit    = WF_ORDERING_TABLE[i]%32;
      	p_pen[pen_offset] |= MASK_BIT[pen_bit];
      	
      	// Pin is a backplane
      	if (i>= _LCDFRONTPLANES)    
      	{
      	    // Enable  BPEN 
            p_pen[pen_offset+2] |= MASK_BIT[pen_bit];  
            // fill with 0x01, 0x02, etc 
            LCD->WF8B[(unsigned char)WF_ORDERING_TABLE[i]] = MASK_BIT[i - _LCDFRONTPLANES];
      	} 
    }
}


// Enable SLCD module
void SLCD_EnableModule(void) {
    // Enable LCD module
    LCD->GCR |= LCD_GCR_LCDEN_MASK;
}

// LCD initialization function
void LCD_Init(void) {
    SLCD_ConfigureClock();    // Configure clock
    SLCD_SetupPins();         // Configure pins
    SLCD_ConfigureGCR();      // Configure LCD GCR register
    SLCD_EnablePins();        // Enable pins
    SLCD_EnableModule();      // Enable LCD module
}


void LCD_Display(unsigned char *lbpMessage) {
    unsigned char lbSize = 0;
    bLCD_CharPosition = 0; // Home display

    unsigned char *lbpLCDWF;
    unsigned char lbCounter;
    unsigned short int arrayOffset;
    unsigned char position;
    unsigned char temp;
    unsigned char char_val;

    lbpLCDWF = (unsigned char *)&LCD->WF[0];

    while (lbSize < _CHARNUM && *lbpMessage) {
        // Adjust character value
        unsigned char lbValue = *lbpMessage;

        // Ensure the character is within the printable ASCII range
        if (lbValue >= 'a' && lbValue <= 'z') lbValue -= 32; // Uppercase
        if (lbValue < ASCII_TABLE_START || lbValue > ASCII_TABLE_END) lbValue = BLANK_CHARACTER;

        // Remove the offset to search in the ASCII table
        lbValue -= ASCII_TABLE_START;

        // Calculate the offset in the ASCII to waveform codification table
        arrayOffset = lbValue * _CHAR_SIZE;

        // Ensure bLCD position is within the valid limit
        lbCounter = 0; // Number of writings to complete one char
        while (lbCounter < _CHAR_SIZE && bLCD_CharPosition < _CHARNUM) {
            position = (bLCD_CharPosition) * _LCDTYPE + lbCounter;
            temp = 0;
            if (lbCounter == 1) {
                temp = lbpLCDWF[WF_ORDERING_TABLE[position]] & 0x01; // Bit 0 has the special symbol information
            }

            char_val = ASCII_TO_WF_CODIFICATION_TABLE[arrayOffset + lbCounter];

            lbpLCDWF[WF_ORDERING_TABLE[position]] = char_val | temp;

            lbCounter++;
        }

        bLCD_CharPosition++;
        lbpMessage++;
        lbSize++;
    }

    // Fill remaining characters with blanks if necessary
    while (lbSize++ < _CHARNUM) {
        // Ensure bLCD position is within the valid limit
        if (bLCD_CharPosition < _CHARNUM) {
            lbCounter = 0; // Number of writings to complete one char
            while (lbCounter < _CHAR_SIZE) {
                position = (bLCD_CharPosition) * _LCDTYPE + lbCounter;
                lbpLCDWF[WF_ORDERING_TABLE[position]] = BLANK_CHARACTER;
                lbCounter++;
            }

            bLCD_CharPosition++;
        }
    }
}

