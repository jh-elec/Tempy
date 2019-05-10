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

#define SSD1306_ADDR							0x78


#define SSD1306_LCD_Width						128
#define SSD1306_LCD_HEIGHT						64

#define SSD1306_MODE_CMD						0x00
#define SSD1306_MODE_DATA						0x40

#define SSD1306_CMD_ADDR_MODE					0x20
#define SSD1306_CMD_SET_COLUMN_ADDR				0x21
#define SSD1306_CMD_SET_PAGE_ADDR				0x22
#define SSD1306_CMD_SET_START_LINE				0x40
#define SSD1306_CMD_SET_CONTRAST				0x81
#define SSD1306_CMD_SET_CHARGEPUMP				0x8D
#define SSD1306_CMD_SET_SEGMENT_REMAP			0xA1
#define SSD1306_CMD_DISPLAY_ALL_ON_RES			0xA4
#define SSD1306_CMD_NORMAL						0xA6
#define SSD1306_CMD_SET_MUX						0xA8
#define SSD1306_CMD_DISPLAY_OFF					0xAE
#define SSD1306_CMD_DISPLAY_ON					0xAF
#define SSD1306_CMD_SET_COM_SCAN_INC			0xC0
#define SSD1306_CMD_SET_COM_SCAN_DEC			0xC8
#define SSD1306_CMD_SET_OFFSET					0xD3
#define SSD1306_CMD_SET_CLK_DIV					0xD5
#define SSD1306_CMD_SET_PRECHARGE				0xD9
#define SSD1306_CMD_SET_COM_PINS				0xDA
#define SSD1306_CMD_SET_VCOM_DESELECT			0xDB
#define SSD1306_CMD_PAGE_START_ADDR				0xB0
#define SSD1306_CMD_COLUMN_LOW_ADDR				0x00
#define SSD1306_CMD_COLUMN_HIGH_ADDR			0x10
#define SSD1306_ADDR_MODE_HORIZ					0
#define SSD1306_ADDR_MODE_VERT					1
#define SSD1306_ADDR_MODE_PAGE					2

#define _FONT_LENGTH			0
#define _FONT_FIXED_WIDTH		2
#define _FONT_HEIGHT			3
#define _FONT_FIRST_CHAR		4
#define _FONT_CHAR_COUNT		5
#define _FONT_WIDTH_TABLE		6

#define IsFixedWidthFont( font ) (font[ _FONT_LENGTH ] == 0 && font[ _FONT_LENGTH + 1 ] == 0 )



typedef struct
{
	/*
	*	Nach dem Aufruf von (calcFontStart())
	*	steht dort der Anfang des gesuchten Zeichens drinn
	*/
	uint16_t uiIndex;
	
	/*
	*	Zeichenbreite
	*/
	uint16_t uiWidht;
	
	/*
	*	Zeichenhöhe 
	*/
	uint16_t uiHeight;
	
	/*
	*	Zeichenhöhe in Bytes
	*/
	uint8_t	uiHeightInBytes;
	
}Font_t;


enum ssd1306_errors
{
	/*
	*	Kommunikations Beginn fehlgeschlagen
	*/
	START_NOT_RDY,
	
	/*
	*	Erneute Kommunikationsanfrage fehlgeschlagen
	*/
	REP_START_NOT_RDY,
	
	/*
	*	Register Adresse konnte nicht erfolgreich übertragen werden
	*/	
	TX_ADDR_REG,
	
	/*
	*	Ein Datenbyte konnte nicht erfolgreich übertragen werden
	*/	
	TX_BYTE,
	
	ALL_ERRORS,
};

extern uint8_t ssd1306Error;
extern uint8_t ssd1306ErrCnt[ALL_ERRORS];

void Ssd1306SendCmd(uint8_t c);

void Ssd1306SendData( uint8_t data );


void Ssd1306Init(void);

void Ssd1306Goto( uint8_t y , uint8_t x );

void Ssd1306Clear( void );

void Ssd1306SetFont(const uint8_t __flash *ptrFnt);

void Ssd1306PutC(char c, uint8_t y, uint8_t x);

void Ssd1306PutS(char *str, uint8_t y , uint8_t x);

void Ssd1306PrintImage(const uint8_t *image, uint16_t sizeofimage, uint8_t y , uint8_t x);


#define __USE_NEW_FUNCTIONS__

#ifdef __USE_NEW_FUNCTIONS__

Font_t		GetFont				( uint8_t c );

void		Ssd1306ClearScreen	( void );

void		Ssd1306FillScreen	( void );

void		Ssd1306DrawPixel	( uint8_t y , uint8_t x );

void		Ssd1306ClearPixel	( uint8_t y , uint8_t x );

uint16_t	Ssd1306PutChar		( char c , uint8_t y , uint8_t x );

void		Ssd1306PutString	( char *str, uint8_t y , uint8_t x );

void		Ssd1306SendRam		( void );

#endif 