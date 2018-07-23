/* Generated by CodeDescriptor 1.5.0.0907 */
/*
* Project Name      -> SSD1306 GLCD Driver
* Version           -> 1.0.0.0414
* Author            -> Jan Homann @ Workstadion.: XXXJANIIIX3XXX
* Build Date        -> 14.04.2018 09:13:07
* Description       -> Treiber unterstützt das I2C Protokoll.
*
*
*
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "I2C.h"
#include "ssd1306.h"

font_t font;

uint8_t buff[3]			= "";
uint8_t ssd1306Error	= 0;

uint8_t errTmp			= 0;
uint8_t ssd1306ErrCnt[ALL_ERRORS];

uint8_t ssd1306Reads( uint8_t *buff , uint8_t leng )
{
	cli();
	
	if ( i2c_start( SSD1306_ADDR + I2C_WRITE ) )
	{
		ssd1306Error |= ( 1 << START_NOT_RDY );
		ssd1306ErrCnt[START_NOT_RDY]++;
		
		i2c_stop();
		return 1;
	}
	
	errTmp = i2c_write( buff[0] ); // Register Adresse
	if ( errTmp )
	{
		errTmp = 0;
		ssd1306Error |= ( 1 << TX_ADDR_REG );
		ssd1306ErrCnt[TX_ADDR_REG]++;
	}
	
	if ( i2c_rep_start( SSD1306_ADDR + I2C_READ ) )
	{
		ssd1306Error |= ( 1 << REP_START_NOT_RDY );
		ssd1306ErrCnt[REP_START_NOT_RDY]++;
		i2c_stop();
		sei();
		
		return 1;
	}
	
	for ( uint8_t i = 0 ; i < leng ; i++ )
	{
		if ( leng == 1 )
		{
			*buff = i2c_readNak();
			sei();
			return 0;
		}
		else
		{
			*buff++ = i2c_readAck();
		}
	}
	*buff = i2c_readNak();
	i2c_stop();
	
	sei();
	
	return 0;
}

uint8_t ssd1306Writes( uint8_t *buff , uint8_t leng )
{
	cli();
	
	if ( i2c_start( SSD1306_ADDR + I2C_WRITE ) )
	{
		ssd1306Error |= ( 1 << START_NOT_RDY );
		ssd1306ErrCnt[START_NOT_RDY]++;
		i2c_stop();
		sei();
		return 1;
	}

	for ( uint8_t i = 0 ; i < leng ; i++ )
	{
		if ( leng == 1 )
		{
			errTmp = i2c_write( *buff );
			if ( errTmp )
			{
				errTmp = 0;
				ssd1306Error |= ( 1 << TX_BYTE );
				ssd1306ErrCnt[TX_BYTE]++;
			}
			
			i2c_stop();
			sei();
			
			return 0;
		}
		else
		{
			errTmp = i2c_write( *buff++ );
			if ( errTmp )
			{
				errTmp = 0;
				ssd1306Error |= ( 1 << TX_BYTE );
				ssd1306ErrCnt[TX_BYTE]++;
			}
		}
	}
	i2c_stop();

	sei();
	
	return 0;
}


void ssd1306SendCmd(uint8_t c)
{
	buff[0] = SSD1306_MODE_CMD;
	buff[1] = c;
	
	ssd1306Writes( buff , 2 );
}

void ssd1306SendData( uint8_t data )
{
	buff[0] = SSD1306_MODE_DATA;
	buff[1] = data;
	
	ssd1306Writes( buff , 2 );
}

void glcdInit(void)
{
	static uint8_t buff[] =
	{
		0xAE,			// Display OFF (sleep mode)
		0x20, 
		
		0b00,			// Set Memory Addressing Mode
						// 00=Horizontal Addressing Mode; 01=Vertical Addressing Mode;
						// 10=Page Addressing Mode (RESET); 11=Invalid
						
		0xB0,			// Set Page Start Address for Page Addressing Mode, 0-7
		0xC8,			// Set COM Output Scan Direction
		0x00,			// --set low column address
		0x10,			// --set high column address
		0x40,			// --set start line address
		0x81, 255,		// Set contrast control register
		0xA1,			// Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
		0xA6,			// Set display mode. A6=Normal; A7=Inverse
		0xA8, 0x3F,		// Set multiplex ratio(1 to 64)
		0xA4,			// Output RAM to Display
						// 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
		0xD3, 0x00,		// Set display offset. 00 = no offset
		0xD5,			// --set display clock divide ratio/oscillator frequency
		0xF0,			// --set divide ratio
		0xD9, 0,		// Set pre-charge period
		0xDA, 0x12,		// Set com pins hardware configuration
		0xDB,			// --set vcomh
		0x20,			// 0x20,0.77xVcc
		0x8D, 0x04,		// Set DC-DC enable
		0xAF,			// Display ON in normal mode
	};

	for (uint8_t i = 0 ; i < sizeof(buff) ; i++ )
	{
		ssd1306SendCmd( buff[i] );
	}
}

void glcdGoto( uint8_t y , uint8_t x )
{
	ssd1306SendCmd( SSD1306_MODE_CMD );	
	ssd1306SendCmd( SSD1306_CMD_PAGE_START_ADDR + y );
	//ssd1306SendCmd( SSD1306_CMD_SET_COLUMN_ADDR );
	//ssd1306SendCmd( x );			
	//ssd1306SendCmd( 0x7F );		
	ssd1306SendCmd( ( ( x & 0xF0) >> 4 ) | 0x10 );
	ssd1306SendCmd( x & 0x0F );
}

void glcdClear( void )
{
	for ( uint8_t page = 0 ; page < 8 ; page++ )
	{
		glcdGoto( page , 0 );
		for (uint8_t column = 0 ; column < 128 ; column++ )
		{
			ssd1306SendData(0x00);
		}
	}
}

static inline font_t calcFontStart(uint8_t c, font_t font, const uint8_t __flash *fontPtr)
{
	#define OFFSET_SETTING_INFOS	8
	
	uint8_t charNum = 0;
	
	font.width = fontPtr[(c-font.fontPtr[4])+OFFSET_SETTING_INFOS]; // Breite des Zeichens
	
	font.indexNum = font.fontPtr[6]; // Offset (ab hier beginnen die Pixel Daten)
	
	if ( font.fontPtr[7] == 1 ) // Fonts mit fester Breite
	{
		for(	; charNum <= (c - font.fontPtr[4])-1 ; charNum++)
		{
			font.indexNum += (font.fontPtr[2] * ((font.fontPtr[3] / 8)+1)); // Font breite berechnen
		}
	}
	else if ( font.fontPtr[7] == 0)
	{
		for(	; charNum <= (c - font.fontPtr[4])-1 ; charNum++)
		{
			font.indexNum += (fontPtr[charNum + OFFSET_SETTING_INFOS] * 2 ); // Anstatt *2 schieben wir hier einfach
		}
	}
	
	return font;
}

void glcdSetFont(const uint8_t __flash *chooseFontPtr)
{
	font.fontPtr = chooseFontPtr;
}

/* ONLY FOR INTERNAL OPERATIONAL */
static inline uint8_t swapBits(uint8_t byte)
{
	uint8_t ret = 0;
	
	for ( uint8_t i = 0 ; i < 8 ; i++ )
	{
		ret >>= 1;
		ret |= byte & 0x80;
		byte <<= 1;
	}
	return ret;
}

void glcdPutc(char c, uint8_t y, uint8_t x)
{	
	uint16_t	index = 0;	
	uint8_t		page = 0;
	
	font = calcFontStart( c , font , font.fontPtr );	
	
	#define FONT_IS_FIXED			0x01
	#define FONT_IS_NOT_FIXED		(!(FONT_IS_FIXED))

	/*
	*	Ist wichtig für den Abstand der Zeichen!
	*/
	if (font.fontPtr[7] == FONT_IS_FIXED)
	{
		font.width = font.fontPtr[2];
	}
	
	glcdGoto( y / 8 , x );
	for ( page = 0 ; page < ((font.fontPtr[3] / 8 ) + 2 ) ; page++ ) // Berechne die Anzahl der benötigten Reihen
	{	
		/*
		*	Bei Fonts die höher als 8 Pixel sind, müssen die letzten Zeilen dementprechend behandelt werden.
		*	Es entsteht eine Lücke, weil das Font nicht kompatibel zu einem senkrecht zeichnenden Display ist
		*/
		if ( ( page == ( ( ( font.fontPtr[3] / 8 ) + 2 ) - 1 ) ) && font.fontPtr[3] > 8 )
		{
			for ( ; index < ( font.width * page ) ; index++)
			{
				ssd1306SendData( ( font.fontPtr[ font.indexNum+index ] ) << ( ( page * 8 ) - ( font.fontPtr[3] ) -1 )  );
			}
			break;
		}
		
 		for ( ; index < ( font.width * page ) ; index++ )
 		{	
 			ssd1306SendData( ( font.fontPtr[ font.indexNum + index ] ) );
 		}
		glcdGoto( (( y / 8 )  + page) , x );
	}
}

void glcdPuts(char *str, uint8_t y , uint8_t x)
{
	uint16_t space = 0;
	while (*str)
	{
		glcdPutc(*str++,y,space+x);
		space += (font.width) + 1; // nächste Schreibposition anhand der größe vom Zeichen summieren.
	}}

void glcdPrintImage(const uint8_t *image, uint16_t sizeofimage, uint8_t y , uint8_t x)
{
	uint16_t column = image[2];
	uint16_t page = 0;
	
	glcdGoto((y/8),x);
	for ( ; page < ((image[1] / 8)+2) ; page++)
	{
		for ( ; column < (uint16_t)((image[0] * page)+image[2]) && (column < sizeofimage) ; column++)
		{
			ssd1306SendData(((image[column])));
		}
		glcdGoto((y/8)+(page),x);
	}
}

