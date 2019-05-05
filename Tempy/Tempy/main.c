/*	Solar Lader
*	Ruhestrom.: ~10mA
*	Ladestrom.:	~80mA
*
*/



/*
 * Tempy.c
 *
 * Created: 24.04.2018 17:05:43
 * Author : Jan Homann
 */ 
#define F_CPU	8000000UL


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "Hardware Treiber/I2C.h"
#include "Hardware Treiber/ssd1306.h"
#include "Hardware Treiber/rx8564.h"
#include "Hardware Treiber/bmp180.h"
#include "Hardware Treiber/sht21.h"

#include "Fonts/Arial_14.h"
#include "Fonts/Arial_Black_16.h"
#include "Fonts/fixed_num_8x16.h"
#include "Fonts/fixed_num_15x31.h"
#include "Fonts/fixed_num_5x7.h"

#include "build_info.h"
#include "ttostr.h"
#include "error.h"

#define DDRx(PORTx)				( * ( &PORTx - 1 ) )

#define STATE_LED_PORT			PORTA
#define STATE_LED_bp			PA1

#define STATE_LED_ON			STATE_LED_PORT &= ~(1<<STATE_LED_bp);
#define STATE_LED_OFF			STATE_LED_PORT |=  (1<<STATE_LED_bp);
#define STATE_LED_TOGGLE		STATE_LED_PORT ^=  (1<<STATE_LED_bp);

#define ENC_PORT				PORTA
#define ENC_PIN					PINA
#define ENC_PHASE_A_bp			PA5
#define ENC_PHASE_B_bp			PA6
#define ENC_SWITCH_bp			PA7


#define MEASUREMENT_AVERAGE		5

#define CALC_MIN(MIN)			( MIN * 59 )
#define CALC_HOUR(HOUR)			CALC_MIN( HOUR * 59)
#define DISPLAY_AUTO_OFF		CALC_HOUR(3)

typedef struct  
{
	uint16_t mil;
	uint8_t sec;
	uint8_t min;
}systime_t;
volatile systime_t tim;

typedef struct  
{
	uint8_t keyPress;
	uint8_t keyState;
	uint8_t keyRpt;
}keySystem_t; volatile keySystem_t internalKey;

typedef struct
{
	uint8_t enter;
	uint8_t enterRpt;
}button_t; volatile button_t button;

typedef struct  
{
	enum
	{
		RTC_IS_RDY_TO_RD,
		RTC_IS_RDY_TO_SHOW_TIME,
		SENSORS_READY,
		AVERAGE_READY,
		AVERAGE_FIRST_READY,
		MIN_MAX_VALUES_READY,
	}readState;
	
	struct  
	{
		struct
		{
			uint32_t	pressAvrg;
			int32_t	tempAvrg;
			int32_t	temp;
			uint32_t	pressure;
		}bmp180;
		
		struct
		{
			int32_t		tempAvrg;
			uint32_t	humidityAvrg;
			
			int16_t		temp;
			uint32_t	humidity;
		}sht21;
	}processValue;
	
	uint8_t ready;

}readtime_t; volatile readtime_t readSens;

typedef struct
{
    /*
    *   Zähler Tabelle für das incrementieren vom enc
    */
    const int8_t table[16];
     
    /*
    *   Aktueller Z?hlerstand vom enc
    */
    int8_t       result;
     
    /*
    *   Letzter Z?hlerstand vom enc (darf nicht weiter benutzt werden!)
    */
    int8_t       last;
     
}enc_t;
volatile enc_t enc = 
{
    .table = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , -1 , 0 , 0 },   
};

enum
{
	BMP_180_TEMP,
	BMP_180_PRESS,
	
	SHT_21_TEMP,
	SHT_21_HUMIDITY,
	
	MIN_MAX_ENTRYS
};

typedef struct  
{
	char *nameOfSensor;
	char *nameOfValue;
	int16_t max;
	int16_t min;
}minMax_t;
minMax_t minMax[MIN_MAX_ENTRYS] =
{
	{
		.nameOfSensor	= "-BMP180",
		.nameOfValue	= "Temp.",
		.min			= 32767,
		.max			= -32767,
	},
	
	{
		.nameOfSensor	=	"-BMP180",
		.nameOfValue	= "Luftdr.",
		.min			= 32767,
		.max			= -32767,
	},	
	
	{
		.nameOfSensor	= "-SHT21",
		.nameOfValue	= "Temp.",
		.min			= 32767,
		.max			= -32767,
	},
	
	{
		.nameOfSensor	= "-SHT21",
		.nameOfValue	= "Feuch.",
		.min			= 32767,
		.max			= -32767,
	},
};

typedef struct  
{
	uint16_t	hours;
	uint8_t		minutes;
	uint8_t		seconds;
	
	uint16_t	dispAutoOff;
}operating_t; volatile operating_t operating; operating_t operatingEEP EEMEM;

typedef struct  
{
	uint8_t brightness;
} eep_t; eep_t eep EEMEM ; eep_t ram;

typedef struct  
{
	uint8_t menueTimeout	:1;
	uint8_t dispIsOff		:1;
}flag_t; volatile flag_t flag;

rx8564_t rtc;

uint8_t clearOld		= 0, 
		clearNew		= 1, 
		showNewValues	= 0, 
		showNewValues_	= 0;

char output[20]	= "";

uint8_t		tme[5] =				{ 24 , 59 , 59 };
	
uint16_t	humidity_ = 0;
int16_t		temp_ = 0;

uint32_t	pressRaw = 0;
int32_t		tempRaw = 0;

uint8_t secoundCmp = 0;
uint8_t eepInit EEMEM;

inline void u16Str				( uint16_t u16 , char *str )
{
	str[0] = ( ( u16 / 10000 ) % 10 ) + '0';
	str[1] = ( ( u16 / 1000 ) % 10 ) + '0';
	str[2] = ( ( u16 / 100 ) % 10 ) + '0';
	str[3] = ( ( u16 / 10 ) % 10 ) + '0';
	str[4] = ( u16 % 10 ) + '0';
	str[5] = '\0';
}

void checkMinMax				( volatile readtime_t *r , minMax_t *mm )
{
	/*	Luftdruckmessung "BMP180"
	*	Hier werden die "minimalen / maximalen" Werte ermittelt
	*/
	if ( (int32_t)(r->processValue.bmp180.pressure / 100) > mm[BMP_180_PRESS].max )
	{
		mm[BMP_180_PRESS].max = (int32_t)r->processValue.bmp180.pressure / 100;	
	}
	if ( (int32_t)(r->processValue.bmp180.pressure / 100) < mm[BMP_180_PRESS].min )
	{
		mm[BMP_180_PRESS].min = (int32_t)r->processValue.bmp180.pressure / 100;
	}
	
	/*	Temperaturmessung "BMP180"
	*	Hier werden die "minimalen / maximalen" Werte ermittelt
	*/
	if ( (int32_t)(r->processValue.bmp180.temp / 10) > mm[BMP_180_TEMP].max )
	{
		mm[BMP_180_TEMP].max = (int32_t)r->processValue.bmp180.temp / 10;
	}
	if ( (int32_t)(r->processValue.bmp180.temp / 10) < mm[BMP_180_TEMP].min )
	{
		mm[BMP_180_TEMP].min = (int32_t)r->processValue.bmp180.temp / 10;
	}
	
	/*	Feuchtigkeitsmessung "SHT21"
	*	Hier werden die "minimalen / maximalen" Werte ermittelt
	*/
	if ( (int32_t)(r->processValue.sht21.humidity ) > mm[SHT_21_HUMIDITY].max )
	{
		mm[SHT_21_HUMIDITY].max = (int32_t)r->processValue.sht21.humidity;
	}
	if ( (int32_t)(r->processValue.sht21.humidity ) < mm[SHT_21_HUMIDITY].min )
	{
		mm[SHT_21_HUMIDITY].min = (int32_t)r->processValue.sht21.humidity;
	}
	
	/*	Temperaturmessung "SHT21"
	*	Hier werden die "minimalen / maximalen" Werte ermittelt
	*/
	if ( (int32_t)(r->processValue.sht21.temp ) > mm[SHT_21_TEMP].max )
	{
		mm[SHT_21_TEMP].max = (int32_t)r->processValue.sht21.temp;
	}
	if ( (int32_t)(r->processValue.sht21.temp ) < mm[SHT_21_TEMP].min )
	{
		mm[SHT_21_TEMP].min = (int32_t)r->processValue.sht21.temp;
	}
}

uint8_t cnfgTime_				( uint8_t *buff )
{     
	uint8_t cmp = 0;
	
	enum
	{
		TIME_HOUR,
		TIME_MINUTES,
		TIME_SECOUNDS,
		
		TIME_MAX_ENTRYS	
	};
	
	enum error
	{
		SUCCESS,
		BUFFER_TO_SMALL,	
	};
	
    /*
    *      Werte müssen beim Funktionsaufruf mitgegeben werden..
    *      Anhand der ersten beiden Bytes von "buff" werden die
    *      maximal Größen von den Werten festgelegt
    */
    uint8_t maxValues[] = { 23 , 59 , 59 }; 
		
	rtcGetData( &rtc );
	buff[TIME_HOUR]		= bcdToDec(rtc.hour);
	buff[TIME_MINUTES]	= bcdToDec(rtc.minute);
	buff[TIME_SECOUNDS]	= bcdToDec(rtc.second);
	
	glcdSetFont( System5x7 );
	glcdClear();
	glcdPuts("-Uhrzeit stellen.." , 0 , 0 );
	glcdSetFont( fixednums15x31 );

	
	glcdPuts( dec_ttostr( buff[TIME_HOUR] , buff[TIME_MINUTES] , buff[TIME_SECOUNDS] ) , 25 , 0 );	
		
    for ( uint8_t i = 0 ; i < 3 && ( ! ( flag.menueTimeout ) )  ; i++ )
    {
		cmp = buff[TIME_HOUR + i];
		while( 1 )
		{
			cmp += enc.result;
			enc.result = 0;
			
			if ( button.enter )
			{
				cmp = 0;
				button.enter = 0;
				break;
			}
			
			if ( button.enterRpt )
			{
				button.enterRpt = 0;
				flag.menueTimeout = 1;
				break;
			}
			
			if ( cmp <= 0 )
			{
				cmp = 0;
			}
		
			if ( cmp >= maxValues[TIME_HOUR + i] )
			{
				cmp = 0;
				buff[TIME_HOUR + i] = 0;
			}
			else
			{
				buff[TIME_HOUR + i] = cmp;
			}
			switch(i)
			{
				case 0:
				{
					glcdPuts( dec_ttostr( buff[TIME_HOUR] , buff[TIME_MINUTES] , buff[TIME_SECOUNDS] ) , 25 , 0 );	
					_delay_ms( 100 );
					glcdPuts( dec_ttostr( 0xFF , buff[TIME_MINUTES] , buff[TIME_SECOUNDS] ) , 25 , 0 );		
					_delay_ms( 100 );
				}break;
					
				case 1:
				{
					glcdPuts( dec_ttostr( buff[TIME_HOUR] , buff[TIME_MINUTES] , buff[TIME_SECOUNDS] ) , 25 , 0 );
					_delay_ms( 100 );
					glcdPuts( dec_ttostr( buff[TIME_HOUR] , 0xFF , buff[TIME_SECOUNDS] ) , 25 , 0 );
					_delay_ms( 100 );				
				}break;
					
				case 2:
				{
					glcdPuts( dec_ttostr( buff[TIME_HOUR] , buff[TIME_MINUTES] , buff[TIME_SECOUNDS] ) , 25 , 0 );
					_delay_ms( 100 );
					glcdPuts( dec_ttostr( buff[TIME_HOUR] , buff[TIME_MINUTES] , 0xFF ) , 25 , 0 );
					_delay_ms( 100 );				
				}break;
			}
		}     
	}
	
	rtcSetTime( buff[TIME_HOUR] , buff[TIME_MINUTES] , buff[TIME_SECOUNDS] );
 
	glcdClear();
	
    return SUCCESS;
}

uint8_t menueTime				( void );

uint8_t reboot					( void );

uint8_t bmp180MinMax			( void );

uint8_t sht21MinMax				( void );

uint8_t menueExit				( void );

uint8_t operatingHours			( void );

uint8_t setBrightness			( void );

uint8_t showErrors				( void );

typedef struct					
{
	const char	*Name;
	uint8_t		( *fp )	( void );
	uint8_t		cursPos;
}menue_t;

menue_t menueStructMain []	=	
{
	#define MENUE_EXIT	0xFF
	
	/*
	*	Aktueller			Funktion hinter				Cursorposition
	*	Menüpunkt			dem aktuellen				nach dem verlassen
	*						Menüpunkt					der Funktion
	*
	*	Menüpunkt '0' wird nur für den Namen des Menüs verwendet!
	*/
	{"-Einstellungen"	,	NULL						,	0	},
	{"Uhrzeit"			,	menueTime					,	0	},
	{"BMP180"			,	bmp180MinMax				,	0	},
	{"SHT21"			,	sht21MinMax					,	0	},
	{"Hellig."			,	setBrightness				,	0	},
	{"Betrieb."			,	operatingHours				,	0	},
	{"Neustart"			,	reboot						,	0	},
	{"Error(s)"			,	showErrors					,	0	},
	{"Exit"				,	menueExit					,	0xFF},
};

uint8_t showMenue				(menue_t *m, enc_t *enc, size_t menueLen)
{
	glcdSetFont( System5x7 );
	
	int8_t  pageStart = 0;
	uint8_t menueEntry = 0 , y = 0 , retCursor = 0;
	static uint8_t encWasMoved = 0;
	
	flag.menueTimeout = 0;
	
	while( ! ( flag.menueTimeout ) )
	{
		if ( enc->result <= 1 )
		{
			enc->result = 1;
		}
		else if ( enc->result > ( menueLen - 1 ) )
		{
			enc->result = ( menueLen - 1 );
		}

		pageStart = enc->result - 3;
	
		if ( ( pageStart + 5 ) >= menueLen )
		{
			pageStart = menueLen - 5;
		}
	
		if ( pageStart < 0 )
		{
			pageStart = 0;
		}
		
		if ( encWasMoved != enc->result )
		{
			encWasMoved = enc->result;
			tim.mil = 0;
		}
		
		/*
		*	Menü Name
		*/
		glcdPuts( (char*)m[0].Name , 0 , 0 );	
	
		/*
		*	Softwareversion
		*/
		glcdPuts( buildVer() , 60 , 0 );
	
		for ( y = 2 ; y < 6 ; y++ )
		{
			menueEntry = pageStart + ( y - 1 );
			glcdGoto( y , 0 );
			for ( uint8_t i = 0 ; i < 60 ; i++ )
			{
				ssd1306SendData( 0x00 );
			}
			
			if ( menueEntry < menueLen )
			{
				if ( menueEntry == enc->result )
				{	
					retCursor = enc->result;
					glcdPuts( "->" , y * 8 , 0 );
					glcdPuts( (char*)m[menueEntry].Name , y * 8 , 10 );
				}
				else
				{
					glcdPuts( (char*)m[menueEntry].Name , y * 8 , 0 );
				}		
			}
		}
	
		if ( button.enter )
		{	
			button.enter = 0;
			button.enterRpt = 0;
			flag.menueTimeout = 0;	
				
			glcdClear();
									
			/*
			*	Funktion aufrufen die für diesen Menüpunkt
			*	hinterlegt ist.
			*/			
			if ( m[retCursor].fp != NULL)
			{
				enc->result = 0 ;
				if (m[retCursor].fp() == MENUE_EXIT )
				{
					glcdSetFont( System5x7 );
					flag.menueTimeout = 0;
					button.enter = 0;
					button.enterRpt = 0;
					break;
				}
			}	
			glcdSetFont( System5x7 );
			flag.menueTimeout = 0;
			button.enter = 0;
			button.enterRpt = 0;
			glcdClear();

			enc->result = m[retCursor].cursPos;						
		}		
	}	
	
	glcdClear();
	showNewValues = 0; showNewValues_ = 1;	
	return 0;
}

uint8_t menueTime				( void )
{
	cnfgTime_( tme );
	
	return 0;
}

uint8_t reboot					( void )
{
	glcdClear();
	
	glcdSetFont( Arial_Black_16 );
	glcdPuts( "Reboot.." , 25 , 0 );	
	
	_delay_ms(2500);
	
	void (*reboot)(void) = 0;
	reboot();

	return 0;	
}

void clearLine					( uint8_t posx )
{	
	glcdGoto( posx , 0);
	for ( uint8_t i = 0 ; i < 128 ; i++ )
	{
		ssd1306SendData( 0x00 );
	}
}



void refreshDisplay				( void )
{		
	static char tmp[8] = "";
	static uint8_t avg = 0;
	static uint8_t firstRead = 0;	
	
	if ( readSens.ready & 1<<SENSORS_READY )
	{
		sht21Read( &temp_ , &humidity_ );
		pressRaw = bmp180_get_uncomp_pressure();
		pressRaw = bmp180_get_pressure( pressRaw );
	
		tempRaw = bmp180_get_uncomp_temperature();
		tempRaw = bmp180_get_temperature( tempRaw );
				
		if ( avg++ == MEASUREMENT_AVERAGE )
		{	
			avg = 0;
 			readSens.processValue.bmp180.pressAvrg		/= (uint32_t)MEASUREMENT_AVERAGE;
 			readSens.processValue.bmp180.tempAvrg		/= (int32_t)MEASUREMENT_AVERAGE;
 			readSens.processValue.sht21.humidityAvrg	/= (uint32_t)MEASUREMENT_AVERAGE;
 			readSens.processValue.sht21.tempAvrg		/= (int32_t)MEASUREMENT_AVERAGE;

			readSens.processValue.bmp180.pressure		= (uint32_t)readSens.processValue.bmp180.pressAvrg;
			readSens.processValue.bmp180.temp			= (int32_t)readSens.processValue.bmp180.tempAvrg;
			readSens.processValue.sht21.humidity		= (uint16_t)readSens.processValue.sht21.humidityAvrg;
			readSens.processValue.sht21.temp			= (int16_t)readSens.processValue.sht21.tempAvrg;

			readSens.processValue.bmp180.pressAvrg		= 0;
			readSens.processValue.bmp180.tempAvrg		= 0;
			readSens.processValue.sht21.humidityAvrg	= 0;
			readSens.processValue.sht21.tempAvrg		= 0;
			
			/*
			*	Jede zweite Messung auswerten..
			*/
			if ( firstRead < 2 )
			{
				firstRead++;
			}
			else
			{
				checkMinMax( &readSens , minMax );
				showNewValues = 0; showNewValues_ = 1;	
				readSens.ready |= ( ( 1<<MIN_MAX_VALUES_READY ) | ( 1<<AVERAGE_READY ) | ( 1<<AVERAGE_FIRST_READY ) );
			}	
		}
		else
		{
			readSens.processValue.bmp180.pressAvrg		+= pressRaw;
			readSens.processValue.bmp180.tempAvrg		+= tempRaw;
			readSens.processValue.sht21.humidityAvrg	+= humidity_;
			readSens.processValue.sht21.tempAvrg		+= temp_;
		}
		
		readSens.ready &= ~( 1<<SENSORS_READY );		
	}

	if ( readSens.ready & 1<<AVERAGE_FIRST_READY )
	{
		if ( clearOld != clearNew )
		{
			clearOld = clearNew;
			clearLine(5);
			clearLine(6);
			clearLine(7);
		}
		
		if ( showNewValues != showNewValues_ )
		{			
			showNewValues = showNewValues_;
			glcdSetFont( System5x7 );
			
			if ( readSens.processValue.sht21.humidity > 99 )
			{
				readSens.processValue.sht21.humidity = 100;
			}
			
			itoa( readSens.processValue.bmp180.pressure / 100 , tmp , 10 );
			strcpy( output , "Luftdruck: " );
			strcat( output , tmp );
			strcat( output , " hpa");
			glcdPuts(output , 35 , 0 );
			
			itoa( readSens.processValue.sht21.humidity , tmp , 10 );
			strcpy( output , "Luftfeu. : ");
			strcat(output  , tmp );
			strcat(output  , " % ");
			glcdPuts(output, 40 , 0 );
			
			itoa( readSens.processValue.sht21.temp , tmp , 10 );
			strcpy(output , "Temp.[1] : ");
			strcat( output , tmp );
			strcat(output , " Cel. " );
			glcdPuts( output , 50 , 0 );
			
// 			itoa( (int16_t)readSens.processValue.bmp180.temp / 10 , tmp , 10 );
// 			strcpy(output , "Temp.[2] : ");
// 			strcat( output , tmp );
// 			strcat(output , " Cel. " );
// 			glcdPuts( output , 60 , 0 );

			strcpy(output , "Error(s) : ");
			strcat( output , errorGetById( &err , _ERROR_GLCD_I2C_ ) );
			glcdPuts( output , 60 , 0 );

		}
	}
	else
	{
		glcdSetFont( System5x7 );
		glcdPuts("Scanning now.." , 50 , 20 );
		clearOld = 0 ; clearNew = 1;
	}
}

uint8_t get_key_press           ( uint8_t key_mask )
{
	cli();
	key_mask &= internalKey.keyPress;                          // read key(s)
	internalKey.keyPress ^= key_mask;                          // clear key(s)
	sei();
	return key_mask;
}

uint8_t get_key_rpt             ( uint8_t key_mask )
{
	cli();
	key_mask &= internalKey.keyRpt;                            // read key(s)
	internalKey.keyRpt ^= key_mask;                            // clear key(s)
	sei();
	return key_mask;
}

uint8_t showMinMaxValues		( minMax_t *mm , uint8_t pos )
{
	char tmp[10] = "";
	
	/*
	*	Name des aktuellen Sensors
	*/
	strcpy	( output , mm->nameOfSensor );
	strcat	( output , " [max/min]");
	glcdPuts( output , 0 , 0);

	strcpy	( output , mm->nameOfValue );
	strcat	( output , "[max].: ");
	itoa	( mm->max , tmp , 10 );	
	strcat	( output , tmp );		
	glcdPuts( output , pos , 0 );
	
	strcpy	( output , mm->nameOfValue );	
	strcat	( output , "[min].: ");
	itoa	( mm->min , tmp , 10 );
	strcat	( output , tmp );
	glcdPuts( output , pos + 10 , 0 );
				
	return 0;
}

uint8_t bmp180MinMax			( void )
{
	if ( ! (readSens.ready & 1<<AVERAGE_FIRST_READY ) )
	{
		glcdPuts("Please try later.." , 30 , 0);
		_delay_ms(1500);
		return MENUE_EXIT;
	}
		
	while (1)
	{
		showMinMaxValues( &minMax[BMP_180_PRESS] , 20 );
		showMinMaxValues( &minMax[BMP_180_TEMP]  , 40 );	
		
		if ( button.enter || flag.menueTimeout )
		{
			break;
		}
	}
	
	glcdClear();
	
	return 0;
}

uint8_t sht21MinMax				( void )
{
	if ( ! (readSens.ready & 1<<AVERAGE_FIRST_READY ) )
	{
		glcdPuts("Please try later.." , 30 , 0);
		_delay_ms(1500);
		return MENUE_EXIT;
	}
	
	while (1)
	{
		showMinMaxValues( &minMax[SHT_21_HUMIDITY] , 20 );
		showMinMaxValues( &minMax[SHT_21_TEMP]  , 40 );	
		
		if ( button.enter || flag.menueTimeout )
		{
			break;
		}	
	}
	glcdClear();
	
	return 0;
}

uint8_t menueExit				( void )
{
	button.enter = 0;
	button.enterRpt = 0;
	return MENUE_EXIT;
}

uint8_t operatingHours			( void )
{
	glcdPuts( "-Betriebsstunden" , 0 , 0 );
	glcdSetFont( fixednums15x31 );
	
	while (1)
	{
		u16Str( operating.hours , output );
		glcdPuts( output , 30 , 20 );
		
		if ( button.enterRpt || flag.menueTimeout )
		{
			flag.menueTimeout = 0;
			break;
		}
		
		if ( button.enter )
		{
			static uint8_t enterWasTipped = 0;
			button.enter = 0;
			if ( ++enterWasTipped >= 10 )
			{
				glcdClear();
				glcdSetFont( System5x7 );
				glcdPuts("Zaehler geloescht!" , 30 , 0 );
				enterWasTipped = 0;
				eeprom_write_word( &operatingEEP.hours , 0 );
				operating.hours = 0;
				_delay_ms(500);
				break;
			}
		}
	}
	glcdClear();
	
	return 0;
}

uint8_t eepIsInit				( void )
{
	if ( eeprom_read_byte( &eepInit) == 0xAA )
	{
		return 0;
	}
	
	return 1;
}

void eepReload					( void )
{
	if ( eepIsInit() )
	{
		eeprom_write_word( &operatingEEP.hours , 0 );
		eeprom_write_byte( &eepInit , 0xAA );
	}
	
	operating.hours = eeprom_read_word( &operatingEEP.hours );
	ram.brightness = eeprom_read_byte( &eep.brightness );
}

void sht21ShowSerialNumber		( uint8_t *numb )
{
	glcdClear();
	glcdSetFont( System5x7 );
	char tmp[4] = "";
	for ( uint8_t i = 0 ; i < 8 ; i++ )
	{
		memset( (char*)output , 0 , sizeof(output) );
		strcpy( output , "Byte[");
		output[5] = i + '0';
		strcat( output , "] - ");
		itoa( numb[i] , tmp , 10 );
		strcat( output , tmp );
		glcdPuts( output , i*8 , 0 );
	}
	
	_delay_ms(15000);
	glcdClear();
}

uint8_t setBrightness			( void )
{
	glcdSetFont( System5x7 );
	strcpy( output , "-" );
	strcat( output , menueStructMain[4].Name );
	glcdPuts( output , 0 , 0 ); 
	
	char tmp[5] = "";
	uint8_t old = 0;
	volatile enc_t *encoder = &enc;
	uint8_t bright = ram.brightness;
	glcdSetFont( fixednums15x31 );
	
	while (1)
	{
 		bright += encoder->result;
 		encoder->result = 0;
		
		if ( old != bright)
		{
			old = bright;
			tim.sec = 0;
		}
		
		if ( bright >= 100 )
		{
			bright = 100;
		}
		if (bright <= 10)
		{
			bright = 10;
		}
		
		itoa( bright , tmp , 10 );
		strcpy( output , tmp );
		strcat( output , "   ");
		
		glcdPuts( output , 30 , 50 );
		
		ssd1306SendCmd( SSD1306_CMD_SET_VCOM_DESELECT );
		ssd1306SendCmd( bright );
		
		if ( button.enter || flag.menueTimeout )
		{
			flag.menueTimeout	= 0;
			button.enter		= 0;
			break;
		}
	}
	
 	ram.brightness = bright;
 	eeprom_update_byte( &eep.brightness , bright );
	
	return 0;
}
void updateLiveLed				( uint16_t *mil )
{
	#define CALC_LIVE(x) (x*1)
	
	if ( *mil < CALC_LIVE(2) )
	{
		STATE_LED_ON;
	}
	else if ( *mil > CALC_LIVE(2) && *mil < CALC_LIVE(4) )
	{
		STATE_LED_OFF;
	}
	else if ( *mil > CALC_LIVE(8) && *mil < CALC_LIVE(10) )
	{
		STATE_LED_ON;
	}
	else if ( *mil > CALC_LIVE(10) && *mil < CALC_LIVE(12) )
	{
		STATE_LED_OFF;
	}
	else if ( *mil > 500 )
	{
		*mil = 0;
	}
}

uint8_t showErrors				( void )
{
	#define OFFSET_NEW_LINE			8
		
	glcdSetFont( System5x7 );
	glcdPuts( "-Error(s)" , 0 , 0 );
	
	while ( !button.enter )
	{
 		glcdPuts( errorGetById( &err , _ERROR_GLCD_I2C_ )	, 24 , 0 );
  		glcdPuts( errorGetById( &err , _ERROR_RTC_I2C_  )	, 32 , 0 );
 		glcdPuts( errorGetById( &err , _ERROR_SHT21_I2C_ )	, 40 , 0 );
  		glcdPuts( errorGetById( &err , _ERROR_BMP180_I2C_)	, 48 , 0 );
	}	
	return 0;
}


int main(void)
{
    DDRx( STATE_LED_PORT )	|= 1<<STATE_LED_bp;
	STATE_LED_PORT			|= (1<<STATE_LED_bp);
	
	eepReload();
	i2c_init();
	glcdInit();
	bmp180_init( &bmp180 );	
	errorInit( &err );
	
	/*	Timer 1 ( 16 Bit ) 
	*	Wird auf CompareMatch eingestellt
	*	Auslöseintervall.: 10ms
	*/
	TCCR1B	= ( ( 1<<CS11 ) | ( 1<<CS10 ) | ( 1<<WGM12 ) ); // Prescaler : 64 
	TIMSK   = ( ( 1<<OCIE1A ) | ( 1<<OCIE2 ) );
	OCR1A	= 0x04E1;
	
	/*	Timer 2 
	*	Wird auf CompareMatch eingestellt
	*	Auslöseintervall.: 1ms
	*/
	TCCR2  |= ((1<<CS22) | (1<<WGM21)); // Prescaler : 64
	OCR2   = ((F_CPU / 64 / 1000 ) - 1 ); 
	
	/*	Interrupts
	*	Interrupts global freigeben
	*/
	sei();
	
	glcdClear();
	
	button.enterRpt = 0;	
	button.enter	= 0;

// 	uint8_t ser[8] = "";
// 	sht21GetSerialNumber ( ser );
// 	sht21ShowSerialNumber( ser );
		
	glcdSetFont( (const __flash uint8_t*)&Arial14 );
		
// 	while (1)
// 	{
// 		Ssd1306PutChar( 'X' , 10 , 10 );
// 		Ssd1306SendRam();
// 	}
	
	
		
    while (1) 
    {	
 		if ( button.enterRpt )
 		{
			button.enterRpt = 0;
			button.enter	= 0;
 			
			glcdClear();
			showMenue( menueStructMain , (enc_t*)&enc , sizeof(menueStructMain) / sizeof(menueStructMain[0]) );
 		}
		
		if ( operating.dispAutoOff > DISPLAY_AUTO_OFF )
		{
			operating.dispAutoOff = 0;
			flag.dispIsOff = 1;
			ssd1306SendCmd( SSD1306_CMD_DISPLAY_OFF );
		}
		
		if ( flag.dispIsOff && button.enter )
		{
			flag.dispIsOff = 0;
			button.enter = 0;
			ssd1306SendCmd( SSD1306_CMD_DISPLAY_ON );
		}
		
		if ( readSens.ready & 1 << RTC_IS_RDY_TO_RD )
		{
			rtcGetData( &rtc );
			readSens.ready &= ~( 1 << RTC_IS_RDY_TO_RD );
			readSens.ready |= ( 1 << RTC_IS_RDY_TO_SHOW_TIME );
		}
				
		if ( readSens.ready & ( 1 << RTC_IS_RDY_TO_SHOW_TIME ) )
		{					
			glcdSetFont( fixednums15x31 );
			glcdPuts( bcd_ttostr( rtc.hour , rtc.minute , rtc.second ) , 0 , 0 );
	 
			readSens.ready &= ~( 1 << RTC_IS_RDY_TO_SHOW_TIME );	
		}
			
		refreshDisplay();		
    }
}

/* Timer[1] -> Compare Match A
*	Wird ca. jede 10ms aufgerufen
*/
ISR( TIMER1_COMPA_vect )
{
	static uint16_t liveLed = 0;
	static uint8_t rtcRead = 0;
	static uint8_t sensorsRead = 0;
	
	liveLed++;
 	updateLiveLed( &liveLed );

	if ( tim.mil++ >= 1000 )
	{
		tim.mil = 0;
		flag.menueTimeout = 1;
	}

	if ( rtcRead++ > 10 )
	{		
		rtcRead = 0;
		readSens.ready |= (1<<RTC_IS_RDY_TO_RD);
	}
	 
	if ( secoundCmp != rtc.second )
	{
		secoundCmp = rtc.second;
		operating.dispAutoOff++;
		
		if ( sensorsRead++ > 5 )
		{
			sensorsRead = 0;
			readSens.ready |= (1<<SENSORS_READY);
		}
		
		if ( operating.seconds++ > 59 )
		{
			operating.seconds = 0;
			if ( operating.minutes++ > 59 )
			{
				operating.minutes = 0;
				operating.hours++;
				eeprom_update_word( &operatingEEP.hours , operating.hours );
			}
		}
	}
	
    static uint8_t ct0, ct1;
    static uint16_t rpt;
    uint8_t i;
    
    i =  internalKey.keyState ^ ~ENC_PIN;
    ct0 = ~( ct0 & i );
    ct1 = ct0 ^ (ct1 & i);
    i &= ct0 & ct1;
    internalKey.keyState ^= i;
    internalKey.keyPress |= internalKey.keyState & i;
    
    if( (internalKey.keyState & 1<<ENC_SWITCH_bp) == 0 )   // check repeat function
    rpt = 150;                               // start delay
    if(--rpt == 0 )
    {
		//rpt = 50;
	    internalKey.keyRpt |= internalKey.keyState & 1<<ENC_SWITCH_bp;
    }
    
    if ( get_key_rpt( 1<<ENC_SWITCH_bp ) )
    {
	    button.enterRpt = 1;
    }
    
    if ( get_key_press( 1<<ENC_SWITCH_bp ) )
    {
	    button.enter = 1;
    }	
}

/* Timer[2] -> Compare Match
*	Wird ca. jede 1ms aufgerufen
*/
ISR(TIMER2_COMP_vect)
{	
    /*
    *   Drehenc auswertung.
    */
    enc.last = ( ( enc.last << 2 ) & 0x0F );
    if (ENC_PIN & 1<<ENC_PHASE_B_bp)
    {
        enc.last |= 2;
    }
    if (ENC_PIN & 1<<ENC_PHASE_A_bp)
    {
        enc.last |= 1;
    }   
    enc.result += enc.table[enc.last]; 	
}