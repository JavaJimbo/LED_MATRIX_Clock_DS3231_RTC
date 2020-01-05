/* 
 * File:   Defs.h
 * Author: Jim
 *
 * Modified 3-25-16 for Rev 2 LED controller
 *  Modified 4-30-16 for Rev 3 LED controller
 * For DUAL MATRIX option and 32x32 panels
 * 6-7-16: Basic video works great with three serial ports.
 * 4-22-19: Modified for RS485 version
 * 
 */

#ifndef DEFS_H
#define	DEFS_H

#define CR 13
#define	STX '>'
#define	DLE '/'
#define	ETX CR

enum MODE {
    RUN = 0,
    SET_TIME    
};


#define MAXPACKETSIZE 1024
#define MAX_PANEL_DATASIZE (32 * 16 * 3 * 3)
#define NUMCHANNELS 1
// #define REV2BOARD 
#define REV3BOARD  // $$$$


#define UART_STANDBY 0
#define UART_INCOMING 1
#define UART_DONE 2
#define UART_PACKET_OK 3
#define UART_CRC_ERROR 4
#define UART_LENGTH_ERROR 5
#define UART_TIMEOUT_ERROR 7
#define UART_PACKETERROR 8
#define UART_COMMAND_ERROR 9

#define Delayms DelayMs

#define MAXDATASIZE 10000

//#define MATRIX_WIDTH 64
//#define MATRIX_HEIGHT 32

#define MATRIX_WIDTH 32
#define MATRIX_HEIGHT 16


#ifdef PANEL32X32
#define PANELROWS 32
#define COLORDEPTH 3 // was 5
#define MAXLINE 16   // Equal to number of rows / 2
#define TIMER_ROLLOVER 4000
#define MAXCOL (MATRIX_WIDTH/2)
#define MAXROW MATRIX_HEIGHT
#define NUMPANELS 2 // (PANELS_ACROSS * PANELS_STACKED)
#else
#define PANELROWS 16
#define COLORDEPTH 8
#define MAXLINE 8
#define TIMER_ROLLOVER 250
#define MAXCOL MATRIX_WIDTH
#define MAXROW MATRIX_HEIGHT
#define NUMPANELS 1
#endif

#define NUMWRITES MAXCOL
#define PANELCOLS 32
#define PANELSIZE (PANELROWS*PANELCOLS*NUMCHANNELS)
#define NUMPOTS 4

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define XBEEuart UART4
#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR
#define HOST_VECTOR _UART_2_VECTOR

#define RS485uart UART5

#define TEST_OUT LATFbits.LATF2 // LATBbits.LATB0
#define RS485_TX_OUT LATBbits.LATB12
#define RS485_TX_ENABLE LATBbits.LATB12
#define DisableRS485_TX() PORTClearBits(IOPORT_B, BIT_12) 
#define EnableRS485_TX() PORTSetBits(IOPORT_B, BIT_12) 
#define START 1

#define MAXBUFFER 256
#define MAXHOSTBUFFER 256
#define MAXRS485BUFFER 4096 // 2048

#define XBEE_SLEEP PORTBbits.RB15

#define RED_LSB (0x01<<(8-COLORDEPTH))
#define GREEN_LSB (RED_LSB << 8)
#define BLUE_LSB  (RED_LSB << 16)

#define OEbit 0x0020
#define LATbit 0x0040

#ifdef REV3BOARD
#define LATCH_LOW 0x0000
#define LATCH_HIGH 0x6000
#define OE_OFF() PORTClearBits(IOPORT_C, BIT_13)
#define OE_ON() PORTCSetBits(IOPORT_C, BIT_13)
#endif

#ifdef REV2BOARD
#define LATCH_LOW 0x80  // OE=1, LATCH=0
#define LATCH_HIGH 0x40  // OE=0, LATCH=1
#endif


#ifdef REV3BOARD
#define R1bit 0x0100
#define B1bit 0x0001
#define G1bit 0x0200
#define R2bit 0x0002
#define B2bit 0x0800
#define G2bit 0x0400
#define M2R1bit 0x0080
#define M2B1bit 0x0040
#define M2G1bit 0x0020
#define M2R2bit 0x0004
#define M2B2bit 0x0008
#define M2G2bit 0x0010
#endif

#ifdef REV2BOARD
#define R1bit 0x0001
#define B1bit 0x0002
#define G1bit 0x0010
#define R2bit 0x0004
#define B2bit 0x0008
#define G2bit 0x0020

#define M2R1bit 0x0040
#define M2B1bit 0x0080
#define M2G1bit 0x0400
#define M2R2bit 0x0100
#define M2B2bit 0x0200
#define M2G2bit 0x0800
#endif

#define EVEN_MASK ~(R1bit | B1bit | G1bit | R2bit | B2bit | G2bit)
#define ODD_MASK  ~(M2R1bit | M2B1bit | M2G1bit | M2R2bit | M2B2bit | M2G2bit)


#define MAGENTA 0x10007F
#define PURPLE 0x7F0050
#define CYAN 0x405000
#define LIME 0x007F40
#define YELLOW 0x00607F
#define ORANGE 0x00207F
#define RED 0x00007F
#define GREEN 0x007F00
#define BLUE 0x7F0000
#define PINK 0x10207F
#define LAVENDER 0x400030
#define TURQUOISE 0x307F00
#define WHITE 0x606060
#define GRAY 0x404040
#define DARKGRAY 0x202020
#define BLACK 0


#define MEDIUM_MAGENTA 0x040020
#define MEDIUM_PURPLE 0x200018
#define MEDIUM_CYAN 0x101000
#define MEDIUM_LIME 0x002010
#define MEDIUM_YELLOW 0x001820
#define MEDIUM_ORANGE 0x000820
#define MEDIUM_RED 0x000020
#define MEDIUM_GREEN 0x002000
#define MEDIUM_BLUE 0x200000
#define MEDIUM_PINK 0x040820
#define MEDIUM_LAVENDER 0x100008
#define MEDIUM_TURQUOISE 0x082000
#define MEDIUM_WHITE 0x202020 // was 808080
#define MEDIUM_GRAY 0x181818
#define MEDIUM_DARKGRAY 0x080808
#define MEDIUM_BLACK 0


#define LIGHT_MAGENTA 0x010008
#define LIGHT_PURPLE 0x080006
#define LIGHT_CYAN 0x040400
#define LIGHT_LIME 0x000804
#define LIGHT_YELLOW 0x000C10
#define LIGHT_ORANGE 0x000208
#define LIGHT_RED 0x000008
#define LIGHT_GREEN 0x000800
#define LIGHT_BLUE 0x080000
#define LIGHT_PINK 0x010208
#define LIGHT_LAVENDER 0x040002
#define LIGHT_TURQUOISE 0x021000
#define LIGHT_WHITE 0x080808 // was 808080
#define LIGHT_GRAY 0x060606
#define LIGHT_DARKGRAY 0x020202
#define LIGHT_BLACK 0


#define MAXCOLORINDEX 16


#define MAXRANDOM (RAND_MAX+1)
#define true	TRUE
#define false 	FALSE

#define RS485_CTRL LATBbits.LATB12

#ifdef REV3BOARD
#define OEpin   LATCbits.LATC13
#define LATCH   LATCbits.LATC14
#else
#define OEpin   PORTDbits.RD5
#endif


#endif