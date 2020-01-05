/*********************************************************************************************
 * LED_MATRIX_CLOCK
 * MAIN file for LED Controller RS485 Version
 * 4-22-19: Works great with both 16x32 and 32x32 panels.
 * 4-28-19: SYS_FREQ 80000000, Color depth = 7 but MSB = 0 to prevent ghosting, timer rollover = 250
 *          Also, general cleanup - eliminated unused methods.
 * 5-4-19: 32x32 code makes sense now.
 * 5-5-19: Fully debugged matrix math for 32x32 and 16x32 panels.
 *          Added pot reading capability for color balancing.
 *          Atmel methods are up and running for multiple ICs.
 * 12-24-19: Add 7x9 font. Display time.
 * 12-25-19: Switched to four 16x32 panels. Got change interrupts working.
 * 12-26-19: 
 *********************************************************************************************/
#define SYS_FREQ 80000000
#define BOARD_ID (2 * 16)  // $$$$
#define ALL_BOARDS '0'

// #include "Defs.h"
#include "Defs32x32.h"
#include "Delay.h"
#include <plib.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "AT45DB641.h"
#include "ASCII_5x7.h"
#include "ASCII_7x9.h"
#include "math.h"



#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select


/** V A R I A B L E S ********************************************************/
/*
unsigned char USBpanelData[MAX_PANEL_DATASIZE];
union {
    unsigned char b[2];
    unsigned short integer;
} convert;
*/

unsigned char tick = false;


#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR 
unsigned char HOSTTxBuffer[MAXHOSTBUFFER];
unsigned char HOSTRxBuffer[MAXHOSTBUFFER];
unsigned char HOSTRxData[MAXHOSTBUFFER];
unsigned short HOSTTxLength;
unsigned short HOSTRxLength;

#define RS485uart UART5
#define RS485bits U5STAbits
#define RS485_VECTOR _UART_5_VECTOR 
unsigned char RS485TxBuffer[MAXBUFFER];
unsigned char RS485RxBuffer[MAXRS485BUFFER];
unsigned char RS485RxBufferCopy[MAXRS485BUFFER];
unsigned char RS485RxData[MAXRS485BUFFER];
unsigned short RS485TxLength;
unsigned short RS485RxLength;
unsigned char RS485BufferFull = false;

#define MAXPOTS 3

unsigned char DmaIntFlag = false;
unsigned char command = 0;
unsigned char arrPots[MAXPOTS];
unsigned char displayMode = TRUE;

// extern unsigned long colorWheel[MAXCOLORINDEX];
unsigned short matrixOutData[(NUMWRITES*(COLORDEPTH+1)*MAXLINE)];
unsigned short line = 0;
unsigned short dataOffset = 0;
unsigned char colorPlane = 0;
unsigned short latchHigh[1] = {LATCH_HIGH};
unsigned long Delaymultiplier = TIMER_ROLLOVER;

unsigned long redAdjust, greenAdjust, blueAdjust, brightAdjust;
extern BOOL CRCcheck(unsigned char *ptrPacket, short length);
extern UINT16 CRCcalculate(unsigned char *ptrPacket, short length, BOOL addCRCtoPacket);
unsigned long ConvertColorToLong (unsigned char byteColor);
void InitializeDMA();


unsigned long Timer1MilliSeconds = 0;
unsigned char Timer1Flag = false;

unsigned long matrix[MAXROW][MAXCOL];
unsigned char matrixStatus[MAXROW][MAXCOL];
unsigned char matrixColorIndex[MAXROW][MAXCOL];

long HOSTuartActualBaudrate, RS485uartActualBaudrate;

const unsigned long colorWheel[MAXCOLORINDEX] = {MAGENTA, PURPLE, CYAN, LIME, YELLOW, ORANGE, RED, GREEN, BLUE, DARKRED, LAVENDER, TEAL, TURQUOISE, GRAY, WHITE, BLACK};
const unsigned long mediumColorWheel[MAXCOLORINDEX] = {MEDIUM_MAGENTA, MEDIUM_PURPLE, MEDIUM_CYAN, MEDIUM_LIME, MEDIUM_YELLOW, MEDIUM_ORANGE, MEDIUM_RED, MEDIUM_GREEN, MEDIUM_BLUE, MEDIUM_DARKRED, MEDIUM_LAVENDER, MEDIUM_TEAL, MEDIUM_GRAY, MEDIUM_WHITE, MEDIUM_WHITE, MEDIUM_BLACK};
const unsigned long lightColorWheel[MAXCOLORINDEX] = {LIGHT_MAGENTA, LIGHT_PURPLE, LIGHT_CYAN, LIGHT_LIME, LIGHT_YELLOW, LIGHT_ORANGE, LIGHT_RED, LIGHT_GREEN, LIGHT_BLUE, LIGHT_DARKRED, LIGHT_LAVENDER, LIGHT_TEAL, LIGHT_GRAY, LIGHT_WHITE, LIGHT_WHITE, LIGHT_BLACK};

short hours = 0, minutes = 0, seconds = 0;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUARTstatus(void);
void UserInit(void);

void initMatrix(long color)
{
    int row, col;
    for (row = 0; row < MAXROW; row++)
        for (col = 0; col < MAXCOL; col++)
        {
            matrix[row][col] = color;
            matrixStatus[row][col] = 0x00;
        }
}
//unsigned short decodePacket(unsigned char *ptrPacket, unsigned char *ptrBitmap);
//unsigned short processPanelData(unsigned char *ptrReceivedData, unsigned short packetLength);
void loadMatrix(void);
void updateOutputBuffer(void);
void ConfigAd(void);
unsigned long getRGBColor(unsigned short crossfade, unsigned short potVal);
unsigned char updateOutputPanel(unsigned char panelNumber);
unsigned char updateOutputPanelPtr(unsigned char panelNumber, unsigned char *ptrPanelData);
unsigned char updateOutputMatrix();
//unsigned char uncompressData(unsigned char *ptrCompressedData, unsigned char *ptrPanelData);
short packetSize = 0;
unsigned short packetCounter = 0;
unsigned char BoardAlias = 0;
short GetRandom(long RangeLow, long RangeHigh);
int drawBall(short row, short col, short radius, long color, unsigned char QuadOnly);
long RoundUp(float fValue);
long changeColor(long *color);


void resetTimer(void) {
    line = 0;
    dataOffset = 0;
    colorPlane = 0;
    Delaymultiplier = TIMER_ROLLOVER;
    mT23ClearIntFlag();
    OpenTimer23(T23_ON | T23_SOURCE_INT | T23_PS_1_1, TIMER_ROLLOVER);
}

#define BARWIDTH 4

void matrixColorBars()
{
    short row, col, colorIndex, numColumnsEachColor = 0;
    unsigned long color;
    
    colorIndex = 0;
    for (col = 0; col < MAXCOL; col++)
    {
        if (colorIndex < MAXCOLORINDEX) color = colorWheel[colorIndex];
        else color = BLACK;           
        for (row = 0; row < MAXROW; row++)
        {        
            matrix[row][col] = color;
        }
        numColumnsEachColor++;
        if (numColumnsEachColor >= BARWIDTH)
        {
            numColumnsEachColor = 0;
            colorIndex++;
        }
    }
    updateOutputMatrix();
}

void ClearRxBuffer()
{
    int i;
    for (i = 0; i < MAXHOSTBUFFER; i++) HOSTRxBuffer[i] = '\0';
}

unsigned char DisplayASCII_5x7(unsigned char ASCIIchar, short centerRow, short centerCol, long color)
{
    int i, j, row, col;
    long pixelColor;
    if (ASCIIchar < 32 ) return false;
    if (ASCIIchar > (32 + MAXASCII)) return false;
    unsigned char mask;    
    for (i = 0; i < 7; i++)
    {
        row = centerRow + i - 3;            
        mask = 0x10; // Start with top pixel and work down    
        for (j = 0; j < 5; j++)
        {                      
            col = centerCol + j - 2;
            if (col >= MAXCOL) break;                                
            if (   (  (ASCII_5x7[ASCIIchar-32][i]) & mask) != 0x00) 
                pixelColor = color;
            else pixelColor = LIGHT_YELLOW;            
            matrix[row][col] = pixelColor;
            mask = mask >> 1;
        }
    }
    return true;    
}

unsigned char DisplayASCII_7x9(unsigned char ASCIIchar, short centerRow, short centerCol, long color, long backGround)
{
    int i, j, row, col;
    long pixelColor;
    if (ASCIIchar < 32 ) return false;
    if (ASCIIchar > (32 + MAXASCII)) return false;
    unsigned char mask;    
    for (i = 0; i < 9; i++)
    {
        row = centerRow + i - 4;            
        mask = 0x40; // Start with top pixel and work down    
        for (j = 0; j < 7; j++)
        {                      
            col = centerCol + j - 3;
            if (col >= MAXCOL) break;                                
            if (   (  (ASCII_7x9[ASCIIchar-32][i]) & mask) != 0x00) 
            {
                matrix[row][col] = color;
                matrixStatus[row][col] = 1;
            }
            else
            {
                matrix[row][col] = backGround;
                matrixStatus[row][col] = 0;
            }
            mask = mask >> 1;
        }
    }
    return true;    
}

unsigned char DisplayTime(short hours, short minutes, short centerRow, short centerCol, long textColor, long backgroundColor)
{
    short digit1, digit2, digit3, digit4, i, j;
    
    if (hours < 0 || hours > 12) return false;
    for (i = 0; i < MAXROW; i++)
        for (j = 0; j < MAXCOL; j++)
            matrixStatus[i][j] = 0x00;
    
    if (hours > 9)
    {
        DisplayASCII_7x9('1', centerRow, centerCol-11, textColor, backgroundColor);
        digit2 = hours - 10;
        DisplayASCII_7x9(digit2 + '0', centerRow, centerCol-5, textColor, backgroundColor);
        
        matrix[centerRow-1][centerCol-1] = textColor;
        matrixStatus[centerRow-1][centerCol-1] = 1;
                
        matrix[centerRow+2][centerCol-1] = textColor;
        matrixStatus[centerRow+2][centerCol-1] = 1;

        digit3 = minutes / 10;
        DisplayASCII_7x9(digit3 + '0', centerRow, centerCol+4, textColor, backgroundColor);
        digit4 = minutes - (digit3 * 10);
        DisplayASCII_7x9(digit4 + '0', centerRow, centerCol+12, textColor, backgroundColor);
    }
    else
    {
        digit1 = hours;
        DisplayASCII_7x9(digit1 + '0', centerRow, 8, textColor, backgroundColor);
        
        matrix[centerRow-1][12] = textColor;
        matrix[centerRow+2][12] = textColor;                          

        digit2 = minutes / 10;
        DisplayASCII_7x9(digit2 + '0', centerRow, 17, textColor, backgroundColor);
        digit3 = minutes - (digit2 * 10);
        DisplayASCII_7x9(digit3 + '0', centerRow, 24, textColor, backgroundColor);
    }
    return true;
}

enum MODE {
    STANDBY = 0,
    SET_TIME,
    RUN
};

void DrawFrame(long color)
{
    short i;
    for (i = 0; i < MAXCOL; i++)
    {
        matrix[0][i] = matrix[MAXROW-1][i] = color;
        matrix[1][i] = matrix[MAXROW-2][i] = color;
    }
    for (i = 0; i < MAXROW; i++)
    {
        matrix[i][0] = matrix[i][MAXCOL-1] = color;
        matrix[i][1] = matrix[i][MAXCOL-2] = color;
    }
    updateOutputMatrix();
}

short GetRandom(long RangeLow, long RangeHigh)
{
    float flRandom;    
    long range;
    long lngRandom;
    short shortRandom;
    
    range = RangeHigh - RangeLow + 1;
    if (range < 0) return 0;
    flRandom =  ((float)rand() / (float)RAND_MAX);
    lngRandom = (long) (flRandom * (float) range);
    
    shortRandom = (short) (lngRandom + RangeLow);
    if (shortRandom < RangeLow) shortRandom = (short) RangeLow;
    if (shortRandom > RangeHigh) shortRandom = (short) RangeHigh;
    
    return shortRandom;
}

int drawBall(short row, short col, short radius, long color, unsigned char QuadOnly)
{
    short X, Y, drawRadius, i;
    float fRadius, fX, fY;
    int minCol, minRow;
    
    if (QuadOnly) 
    {
        minCol = MAXCOL/2;
        minRow = MAXCOL/2;
    }
    else
    {
        minCol = 0;
        minRow = 0;
    }
    
    if (radius < 0) return false;    
    else if (radius == 0)
    {
        if (row >= MAXROW) row = MAXROW-1;
        if (col >= MAXCOL) col = MAXCOL-1;
        matrix[row][col] = color;
        return true;
    }     
    drawRadius = radius;
    do {
        for (Y = 0; Y <= radius; Y++)
        {
            fRadius = (float) drawRadius;
            fY = Y;
            fX = sqrt((fRadius * fRadius)-(fY*fY));            
            X = (short)(fX);
            for (i = 0; i <= X; i++)
            {                
                if ((row+Y < MAXROW) && (col+i < MAXCOL)) matrix[row+Y][col+i] = color;
                if ((row+Y < MAXROW) && (col-i >= minCol))matrix[row+Y][col-i] = color;
                if ((row-Y >= minRow) && (col+i < MAXCOL))matrix[row-Y][col+i] = color;
                if ((row-Y >=minRow) && (col-i >= minCol))matrix[row-Y][col-i] = color;
            }
        }
       drawRadius--;
    } while (drawRadius);
    // updateOutputMatrix();
    return true;
}

long RoundUp(float Value)
{
    float fValue;
    long intValue;
    float fractional;
    unsigned char negative = false;
    
    if (Value < 0)
    {
        negative = true;
        fValue = 0 - Value;
    }
    else fValue = Value;
    
    intValue = (long)(fValue);
    fractional = fValue - (float)(intValue);
    if (fractional > 0.5) intValue++;
    if (negative) return (0 - intValue);    
    else return intValue;
}

int mode = STANDBY;
unsigned char flashON = true;

enum DIRECTION
{
    RIGHT = 0,
    UPRIGHT,
    DOWNRIGHT,
    UPLEFT,
    LEFT,
    DOWNLEFT,
    UP,
    DOWN
};

void changeDirection(short *direction)
{
    short nextDirection;
    do {
        nextDirection = GetRandom(0, 7);
    } while(nextDirection == *direction);
    *direction = nextDirection;
}

long changeColor(long *color)
{
    long nextColor;
    
    static int colorIndex = 0;    
    
    colorIndex = GetRandom(0, (MAXCOLORINDEX*3)-1);
    
    if (colorIndex < MAXCOLORINDEX) nextColor = colorWheel[colorIndex];
    else if (colorIndex < MAXCOLORINDEX*2) nextColor = mediumColorWheel[colorIndex-MAXCOLORINDEX];
    else nextColor = lightColorWheel[colorIndex-(MAXCOLORINDEX*2)];
    *color = nextColor;    
    
    colorIndex++;   
    if (colorIndex >= MAXCOLORINDEX*3) colorIndex = 0;
    if (colorIndex < MAXCOLORINDEX) nextColor = colorWheel[colorIndex];
    else if (colorIndex < MAXCOLORINDEX*2) nextColor = mediumColorWheel[colorIndex-MAXCOLORINDEX];
    else nextColor = lightColorWheel[colorIndex-(MAXCOLORINDEX*2)];    
    
    return nextColor;
}


// HOST UART interrupt handler it is set at priority level 2
void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) 
{
    static unsigned short HOSTRxIndex = 0;
    static unsigned char TxIndex = 0;
    unsigned char ch;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = UARTGetDataByte(HOSTuart);
            if (ch != 0 && ch != '\n') {
                if (HOSTRxIndex < MAXHOSTBUFFER - 2)
                    HOSTRxBuffer[HOSTRxIndex++] = ch;
                if (ch == '\r') {
                    HOSTRxBuffer[HOSTRxIndex] = '\0';
                    HOSTRxLength = HOSTRxIndex;
                    HOSTRxIndex = 0;
                }
            }
            // UARTtimeout=UART_TIMEOUT;
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
        if (HOSTTxLength) {
            if (TxIndex < MAXHOSTBUFFER) {
                ch = HOSTTxBuffer[TxIndex++];
                if (TxIndex <= HOSTTxLength) {
                    while (!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte(HOSTuart, ch);
                } else {
                    while (!UARTTransmitterIsReady(HOSTuart));
                    HOSTTxLength = false;
                    TxIndex = 0;
                }
            } else {
                TxIndex = 0;
                HOSTTxLength = false;
                INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
            }
        } else INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    }
}


static void InitializeSystem(void) {
    SYSTEMConfigPerformance(80000000); // was 60000000
    UserInit();

    //variables to known states.
}//end InitializeSystem


void UserInit(void) {
    

    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    // SET UP PORT A
#ifdef REV3BOARD 
    PORTSetPinsDigitalOut(IOPORT_A, BIT_0);
#else
    PORTSetPinsDigitalOut(IOPORT_A, BIT_6 | BIT_7); // `B
#endif
    
    // SET UP PORT B
    PORTSetPinsDigitalOut(IOPORT_B, BIT_0 | BIT_1 | BIT_2 | BIT_8 | BIT_12 | BIT_15);    
    PORTSetBits(IOPORT_B, BIT_0 | BIT_1 | BIT_2 | BIT_4 | BIT_8 | BIT_12);
    RS485_CTRL = 0;
    ATMEL_WRITE_PROTECT = 1; // Allow Atmel memory writes
    
    // SET UP PORT C
#ifdef REV3BOARD
    PORTSetPinsDigitalOut(IOPORT_C, BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_13 | BIT_14 | BIT_4);
    PORTSetBits(IOPORT_C, BIT_1 | BIT_2 | BIT_3 | BIT_4);
#else
    PORTSetPinsDigitalOut(IOPORT_C, BIT_3 | BIT_4);
#endif        
    
    // SET UP PORT D
    PORTD = 0x0000; // Set matrix output enable high. All other outputs low.
#ifdef REV3BOARD
    OEpin = 1;
#else    
    OEpin = 0;
#endif    
    
    // SET UP PORT E
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3);    
    
    // SET UP PORT F
    PORTSetPinsDigitalOut(IOPORT_F, BIT_2); 
    TEST_OUT = 0;
    
    // SET UP PORT G
    PORTSetPinsDigitalOut(IOPORT_G, BIT_0);        
    

#ifdef PANEL32X32    
    PORTSetPinsDigitalOut(IOPORT_D, BIT_0 | BIT_1 | BIT_2 | BIT_3); // For 32x32 matrix
#else
    PORTSetPinsDigitalOut(IOPORT_D, BIT_0 | BIT_1 | BIT_2); // For 16x32 matrix
#endif    

    
#ifdef REV3BOARD
    LATC = LATCH_LOW;
#else    
    PORTA = LATCH_LOW;
#endif    
    

#define PMP_CONTROL	(PMP_ON | PMP_MUX_OFF | PMP_READ_WRITE_EN | PMP_WRITE_POL_HI) 

#define PMP_MODE        (PMP_IRQ_READ_WRITE | PMP_MODE_MASTER1 | PMP_DATA_BUS_16 | PMP_WAIT_BEG_1 | PMP_WAIT_MID_0 | PMP_WAIT_END_1)
#define PMP_PORT_PINS	PMP_PEN_ALL
#define PMP_INTERRUPT	PMP_INT_OFF

    // setup the PMP
    mPMPOpen(PMP_CONTROL, PMP_MODE, PMP_PORT_PINS, PMP_INTERRUPT);


    // CONFIGURE DMA CHANNEL 1 - SEND DATA
    DmaChnOpen(DMA_CHANNEL1, 3, DMA_OPEN_DEFAULT);

    // Set the transfer event control: what event is to start the DMA transfer
    DmaChnSetEventControl(DMA_CHANNEL1, DMA_EV_START_IRQ(_PMP_IRQ));

    // Set up transfer destination: PMDIN Parallel Port Data Input Register
    DmaChnSetTxfer(DMA_CHANNEL1, &matrixOutData[0], (void*) &PMDIN, NUMWRITES, 2, 2);

    // Enable the transfer done event flag:
    DmaChnSetEvEnableFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE);


    // Once we configured the DMA channel we can enable it
    DmaChnEnable(DMA_CHANNEL1);

    DmaChnForceTxfer(DMA_CHANNEL1);

    // CONFIGURE DMA CHANNEL 2 - LATCH DATA
    DmaChnOpen(DMA_CHANNEL2, 2, DMA_OPEN_DEFAULT);

    // Set the transfer event control: what event is to start the DMA transfer
    DmaChnSetEventControl(DMA_CHANNEL2, DMA_EV_START_IRQ(_DMA1_IRQ));

    // Set up transfer: source & destination, source & destination size, number of bytes trasnferred per event
#ifdef REV3BOARD
    DmaChnSetTxfer(DMA_CHANNEL2, latchHigh, (void*) &LATC, sizeof (latchHigh), 2, 2);
#else
    DmaChnSetTxfer(DMA_CHANNEL2, latchHigh, (void*) &LATA, sizeof (latchHigh), 2, 2);
#endif
    // Enable the transfer done event flag:
    DmaChnSetEvEnableFlags(DMA_CHANNEL2, DMA_EV_CELL_DONE);

    // Once we configured the DMA channel we can enable it
    DmaChnEnable(DMA_CHANNEL2);
    
    // Set up RS485 UART #5
    UARTConfigure(RS485uart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetLineControl(RS485uart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);

    RS485uartActualBaudrate = (long) UARTSetDataRate(RS485uart, SYS_FREQ, 921600); // Was 230400
    UARTEnable(RS485uart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure RS485 UART Interrupts
    INTEnable(INT_U5TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(RS485uart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(RS485uart), INT_PRIORITY_LEVEL_2);
    
    // Set up HOST UART #2
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);

    HOSTuartActualBaudrate = (long) UARTSetDataRate(HOSTuart, SYS_FREQ, 115200); // Was 230400
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);      
    
    // Set up Timer 1 for 1 kHz interrupts:
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_64, 1250);    
    
    // Set up Timer 2/3 as a single 32 bit timer with interrupt priority of 2
    // Use internal clock, 1:1 prescale
    // If Postscale = 1600 interrupts occur about every 33 uS
    // This yields a refresh rate of about 59 hz for entire display
    // The flicker seems pretty minimal at this rate
    // 1 / 33 us x 8 x 2^6 = 59
    // 8 lines x 32 columns x 6 panels x 6 bit resolution = 9216 writes to PORT D for each refresh!       
    ConfigIntTimer23(T2_INT_ON | T2_INT_PRIOR_2);
    OpenTimer23(T23_ON | T23_SOURCE_INT | T23_PS_1_1, TIMER_ROLLOVER);
    
    RS485_CTRL = 0;
    ATMEL_CS1 = 1;
    ATMEL_CS2 = 1;
    ATMEL_CS3 = 1;
    ATMEL_CS4 = 1;
        
    PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_2 | BIT_3);    
    mCNOpen(CN_ON, CN2_ENABLE | CN3_ENABLE | CN4_ENABLE | CN5_ENABLE, CN2_PULLUP_ENABLE | CN3_PULLUP_ENABLE | CN4_PULLUP_ENABLE| CN5_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);        

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();
    
    // ConfigAd();
}//end UserInit


unsigned char updateOutputMatrix() 
{
    unsigned short line, col;
    unsigned char PWMbit;
    unsigned short outDataIndex; 
    unsigned short outputData;
    unsigned long matrixData, REDmask, GREENmask, BLUEmask;

    outDataIndex = 0;

    for (line = 0; line < MAXLINE; line++) 
    {
        REDmask = RED_LSB;
        GREENmask = GREEN_LSB;
        BLUEmask = BLUE_LSB;
        for (PWMbit = 0; PWMbit <= COLORDEPTH; PWMbit++) 
        {
            for (col = 0; col < MAXCOL; col++) 
            {
                if (PWMbit < COLORDEPTH)
                {
                    outputData = 0x0000;
                    matrixData = matrix[line][col];                    
                    
                    if (REDmask & matrixData) outputData |= R1bit;
                    if (GREENmask & matrixData) outputData |= G1bit;
                    if (BLUEmask & matrixData) outputData |= B1bit;

                    matrixData = matrix[line + MAXLINE][col];

                    if (REDmask & matrixData) outputData |= R2bit;
                    if (GREENmask & matrixData) outputData |= G2bit;
                    if (BLUEmask & matrixData) outputData |= B2bit;

                    matrixData = matrix[line + (MAXLINE*2)][col];                    
                    
                    if (REDmask & matrixData) outputData |= M2R1bit;
                    if (GREENmask & matrixData) outputData |= M2G1bit;
                    if (BLUEmask & matrixData) outputData |= M2B1bit;
                    
                    matrixData = matrix[line + (MAXLINE*3)][col];                    

                    if (REDmask & matrixData) outputData |= M2R2bit;
                    if (GREENmask & matrixData) outputData |= M2G2bit;
                    if (BLUEmask & matrixData) outputData |= M2B2bit;

                    matrixOutData[outDataIndex] = outputData;
                    
                }
                outDataIndex++;
            }
            REDmask = REDmask << 1;
            GREENmask = GREENmask << 1;
            BLUEmask = BLUEmask << 1;
        }
    }
}

void __ISR(_TIMER_23_VECTOR, ipl2) Timer23Handler(void) 
{
    static unsigned char line = 0;
    static unsigned char colorPlane = 0;
    static unsigned long Delaymultiplier = TIMER_ROLLOVER;
    static unsigned short dataOffset = 0;
    static unsigned char testFlag = FALSE;

    mT23ClearIntFlag(); // Clear interrupt flag    

    if (colorPlane == 0 || colorPlane == 2) TEST_OUT = 1;
    else TEST_OUT = 0;
    
    WritePeriod23(Delaymultiplier);
    PORTD = line;
#ifdef REV3BOARD
    LATC = LATCH_LOW;
#else
    PORTA = LATCH_LOW;
#endif
    DmaChnSetTxfer(DMA_CHANNEL1, &matrixOutData[dataOffset], (void*) &PMDIN, NUMWRITES * 2, 2, 2);
    DmaChnEnable(DMA_CHANNEL1);
    DmaChnEnable(DMA_CHANNEL2);    
    dataOffset = dataOffset + NUMWRITES;
    Delaymultiplier = Delaymultiplier << 1;
    colorPlane++;
    if (colorPlane > COLORDEPTH) 
    {
        colorPlane = 0;
        Delaymultiplier = TIMER_ROLLOVER;
        line++;
        if (line >= MAXLINE) 
        {
            line = 0;
            dataOffset = 0;
        }
    }
    else if (colorPlane == COLORDEPTH)
        Delaymultiplier = TIMER_ROLLOVER;
}

void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void) 
{
    mT1ClearIntFlag(); // Clear interrupt flag 
    Timer1MilliSeconds++;
    Timer1Flag = true;    
}

void __ISR(_ADC_VECTOR, ipl6) AdcHandler(void) 
{
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        arrPots[i] = (unsigned char) (ReadADC10(offSet + i) / 4); // read the result of channel 0 conversion from the idle buffer

}



/******************************************************************************
 *	Change Notice Interrupt Service Routine
 *
 *   Note: Switch debouncing is not performed.
 *   Code comes here if SW2 (CN16) PORTD.RD7 is pressed or released.
 *   The user must read the IOPORT to clear the IO pin change notice mismatch
 *	condition first, then clear the change notice interrupt flag.
 ******************************************************************************/

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) 
{  
static short cycleCounter = 0;
unsigned short PortBread;
static unsigned short tickCounter = 0;
static short speedChangeCounter = 0;
static short nextSpeedChange = 10;
static short speed = 0;
    
    // Step #1 - always clear the mismatch condition first
    PortBread = PORTB;
        
    cycleCounter++;
    tickCounter++;
    if (tickCounter >= speed)
    {        
        tickCounter = 0;
        tick = true;
    }
    
    speedChangeCounter++;
    if (speedChangeCounter > nextSpeedChange)
    {
        speedChangeCounter = 0;
        speed = GetRandom(1, 10);
        nextSpeedChange = GetRandom(10, 20);
    }
    
    if (cycleCounter >= 60)
    {
        cycleCounter = 0;
        seconds++;
        if (seconds >= 60)
        {
            seconds = 0;
            minutes++;
            if (minutes >= 60)
            {
                minutes = 0;
                hours++;
                if (hours > 12)
                    hours = 1;
            }
        }
    }
    // TESTOUT = 1; 
    
    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();

        
}


int main(void) {
    unsigned long i, j, pixelColor = BLACK, color = RED, frameColor = BLACK, complementaryColor = BLACK;
    short colorIndex = 0, frameColorIndex;
    short previousColorIndex = -1, previousSeconds = -1;
    short row = MAXROW/2;
    short col = MAXCOL/2;
    short direction = RIGHT, previousDirection;
    short directionChangeCounter = 100;
    short radius = 1;
    short minRow = MAXROW/2, minCol = MAXCOL/2;
    
    hours = 12;
    minutes = 0;
    seconds = 0;
    
    for (i = 0; i < (NUMWRITES * COLORDEPTH * MAXLINE); i++) matrixOutData[i] = (unsigned short) BLACK;

    initMatrix(BLACK);
    updateOutputMatrix();
    InitializeSystem();
    DelayMs(100);
    
    printf("\rTesting fantastic patterns");
    
    // BUCKY
    color = RED;
    
    
    //DisplayTime(hours, minutes, MAXROW/2, MAXCOL/2, color, BLACK);       
    //updateOutputMatrix();       
    
    minRow = MAXROW/2;
    minCol = MAXCOL/2;
    
    while(1)
    {      
        if (tick)
        {
            tick = false;
            previousDirection = direction;
            if (direction == RIGHT) 
            {
                if (col < MAXCOL-1) col++;
                else changeDirection(&direction);
            }
            else if (direction == LEFT) 
            {
                if (col > minCol) col--;
                else changeDirection(&direction);            
            }
            else if (direction == UP) 
            {
                if (row > minRow) row--;
                else changeDirection(&direction);            
            }
            else if (direction == DOWN) 
            {
                if (row < MAXROW-1) row++;
                else changeDirection(&direction);            
            }        
            else if (direction == UPRIGHT) 
            {
                if (col < MAXCOL-1 && row > minRow) 
                {
                    col++;
                    row--;
                }
                else changeDirection(&direction);            
            }
            else if (direction == UPLEFT) 
            {
                if (col > minCol && row > minRow) 
                {
                    col--;
                    row--;
                }
                else changeDirection(&direction);            
            }
            else if (direction == DOWNRIGHT) 
            {
                if (col < MAXCOL-1 && row < MAXROW-1) 
                {
                    col++;
                    row++;
                }
                else changeDirection(&direction);                        
            }
            else if (direction == DOWNLEFT) 
            {
                if (col > minCol && row < MAXROW-1) 
                {
                    col--;
                    row++;
                }
                else changeDirection(&direction);                          
            }
            if (direction == previousDirection)
            {
                if (directionChangeCounter)
                    directionChangeCounter--;
                else
                {
                    directionChangeCounter = GetRandom(10, 30);
                    changeDirection(&direction);
                    complementaryColor = changeColor(&color);                    
                    radius = GetRandom(0,5);
                }
            }
            else if (direction != previousDirection) 
            {
                    complementaryColor = changeColor(&color);
                    radius = GetRandom(0,5);
            }
            if (radius <= 4)
            {
                drawBall(row, col, radius, color, true);
                for (i = 0; i < MAXROW/2; i++)
                {
                    for (j = 0; j < MAXCOL/2; j++)
                    {
                        pixelColor = matrix[MAXROW-i-1][MAXCOL-j-1];
                        matrix[i][j] = pixelColor;
                        matrix[i][MAXCOL-j-1] = pixelColor;
                        matrix[MAXROW-i-1][j] = pixelColor;
                    }
                }
                updateOutputMatrix();
            }
        }        
        DelayMs(1);
    }
    
    
    while(1)
    {
        if (previousSeconds != seconds)
        {                        
            do {
                colorIndex = GetRandom(0,15);
            } while (colorIndex == previousColorIndex);
            previousColorIndex = colorIndex;            
            color = colorWheel[colorIndex];            
            
            do {
                frameColorIndex = GetRandom(0,15);
            } while (frameColorIndex == colorIndex);
            frameColor = colorWheel[frameColorIndex];                        

            DisplayTime(hours, minutes, MAXROW/2, MAXCOL/2, color, BLACK);
            DrawFrame(frameColor); 
            
            previousSeconds = seconds;
            printf("\r%d, %d", colorIndex, frameColorIndex);
        }
        DelayMs(1);
    }
}//end main
