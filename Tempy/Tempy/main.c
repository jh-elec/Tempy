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
#include "Hardware Treiber/Bmp180.h"
#include "Hardware Treiber/Sht21.h"

#include "Fonts/Arial_14.h"
#include "Fonts/Arial_Black_16.h"
#include "Fonts/fixed_num_8x16.h"
#include "Fonts/fixed_num_15x31.h"
#include "Fonts/fixed_num_5x7.h"
#include "Fonts/Verdana24.h"
#include "Fonts/corsiva_12.h"
#include "Fonts/impact32.h"

#include "build_info.h"
#include "ttostr.h"

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
	
	uint16_t	Milisecound;
	uint8_t		Secound;
	uint8_t		Minute;
	
}SystemTime_t;
volatile SystemTime_t SystemTime;

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
		RTC_IS_RDY_TO_SHOW_SystemTimeE,
		SENSORS_Ready,
		AVERAGE_Ready,
		AVERAGE_FIRST_Ready,
		MIN_MAX_VALUES_Ready,
	}SensorReady;
	
	struct  
	{
		struct
		{
			uint32_t	pressAvrg;
			int32_t		tempAvrg;
			int32_t		temp;
			uint32_t	pressure;
		}Bmp180;
		
		struct
		{
			int32_t		tempAvrg;
			uint32_t	humidityAvrg;
			
			int16_t		temp;
			uint32_t	humidity;
		}Sht21;
	}processValue;
	
	uint8_t Ready;

}Sensor_t; volatile Sensor_t Sensor;

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
		.nameOfSensor	= "-Bmp180",
		.nameOfValue	= "Temp.",
		.min			= 32767,
		.max			= -32767,
	},
	
	{
		.nameOfSensor	=	"-Bmp180",
		.nameOfValue	= "Luftdr.",
		.min			= 32767,
		.max			= -32767,
	},	
	
	{
		.nameOfSensor	= "-Sht21",
		.nameOfValue	= "Temp.",
		.min			= 32767,
		.max			= -32767,
	},
	
	{
		.nameOfSensor	= "-Sht21",
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
	uint8_t menueSystemTimeout	:1;
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

void	clearLine				( uint8_t PosY_Page )
{
	for ( uint8_t ui = 0 ; ui < 128 ; ui++ )
	{
		Ssd1306ClearByte( ( PosY_Page * 8 ) , ui );
	}
}

void	checkMinMax				( volatile Sensor_t *Sensor , minMax_t *mm )
{
	/*	Luftdruckmessung "Bmp180"
	*	Hier werden die "minimalen / maximalen" Werte ermittelt
	*/
	if ( (int32_t)(Sensor->processValue.Bmp180.pressure / 100) > mm[BMP_180_PRESS].max )
	{
		mm[BMP_180_PRESS].max = (int32_t)Sensor->processValue.Bmp180.pressure / 100;	
	}
	if ( (int32_t)(Sensor->processValue.Bmp180.pressure / 100) < mm[BMP_180_PRESS].min )
	{
		mm[BMP_180_PRESS].min = (int32_t)Sensor->processValue.Bmp180.pressure / 100;
	}
	
	/*	Temperaturmessung "Bmp180"
	*	Hier werden die "minimalen / maximalen" Werte ermittelt
	*/
	if ( (int32_t)(Sensor->processValue.Bmp180.temp / 10) > mm[BMP_180_TEMP].max )
	{
		mm[BMP_180_TEMP].max = (int32_t)Sensor->processValue.Bmp180.temp / 10;
	}
	if ( (int32_t)(Sensor->processValue.Bmp180.temp / 10) < mm[BMP_180_TEMP].min )
	{
		mm[BMP_180_TEMP].min = (int32_t)Sensor->processValue.Bmp180.temp / 10;
	}
	
	/*	Feuchtigkeitsmessung "Sht21"
	*	Hier werden die "minimalen / maximalen" Werte ermittelt
	*/
	if ( (int32_t)(Sensor->processValue.Sht21.humidity ) > mm[SHT_21_HUMIDITY].max )
	{
		mm[SHT_21_HUMIDITY].max = (int32_t)Sensor->processValue.Sht21.humidity;
	}
	if ( (int32_t)(Sensor->processValue.Sht21.humidity ) < mm[SHT_21_HUMIDITY].min )
	{
		mm[SHT_21_HUMIDITY].min = (int32_t)Sensor->processValue.Sht21.humidity;
	}
	
	/*	Temperaturmessung "Sht21"
	*	Hier werden die "minimalen / maximalen" Werte ermittelt
	*/
	if ( (int32_t)(Sensor->processValue.Sht21.temp ) > mm[SHT_21_TEMP].max )
	{
		mm[SHT_21_TEMP].max = (int32_t)Sensor->processValue.Sht21.temp;
	}
	if ( (int32_t)(Sensor->processValue.Sht21.temp ) < mm[SHT_21_TEMP].min )
	{
		mm[SHT_21_TEMP].min = (int32_t)Sensor->processValue.Sht21.temp;
	}
}

uint8_t cnfgSystemTimee_		( uint8_t *buff )
{     
	uint8_t cmp = 0;
	
	enum
	{
		SystemTimeE_HOUR,
		SystemTimeE_MINUTES,
		SystemTimeE_SECOUNDS,
		
		SystemTimeE_MAX_ENTRYS	
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
	buff[SystemTimeE_HOUR]		= bcdToDec(rtc.hour);
	buff[SystemTimeE_MINUTES]	= bcdToDec(rtc.minute);
	buff[SystemTimeE_SECOUNDS]	= bcdToDec(rtc.second);
	
	Ssd1306SetFont( System5x7 );
	Ssd1306PutString("-Uhrzeit stellen.." , 0 , 0 );
	Ssd1306SetFont( fixednums15x31 );

	
	//Ssd1306PutString( dec_ttostr( buff[SystemTimeE_HOUR] , buff[SystemTimeE_MINUTES] , buff[SystemTimeE_SECOUNDS] ) , 25 , 0 );	
	Ssd1306PutString( dec_ttostr( buff[SystemTimeE_HOUR] , buff[SystemTimeE_MINUTES] , buff[SystemTimeE_SECOUNDS] ) , 25 , 0 );	
		
    for ( uint8_t i = 0 ; i < 3 && ( ! ( flag.menueSystemTimeout ) )  ; i++ )
    {
		cmp = buff[SystemTimeE_HOUR + i];
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
				flag.menueSystemTimeout = 1;
				break;
			}
			
			if ( cmp <= 0 )
			{
				cmp = 0;
			}
		
			if ( cmp >= maxValues[SystemTimeE_HOUR + i] )
			{
				cmp = 0;
				buff[SystemTimeE_HOUR + i] = 0;
			}
			else
			{
				buff[SystemTimeE_HOUR + i] = cmp;
			}
			
			cli();
			switch(i)
			{
				case 0:
				{
					Ssd1306PutString( dec_ttostr( buff[SystemTimeE_HOUR] , buff[SystemTimeE_MINUTES] , buff[SystemTimeE_SECOUNDS] ) , 25 , 0 );	
					Ssd1306SendRam();
					_delay_ms( 100 );		
					Ssd1306PutString( dec_ttostr( 0xFF , buff[SystemTimeE_MINUTES] , buff[SystemTimeE_SECOUNDS] ) , 25 , 0 );	
					Ssd1306SendRam();
					_delay_ms( 100 );
				}break;
					
				case 1:
				{
					Ssd1306PutString( dec_ttostr( buff[SystemTimeE_HOUR] , buff[SystemTimeE_MINUTES] , buff[SystemTimeE_SECOUNDS] ) , 25 , 0 );
					Ssd1306SendRam();
					_delay_ms( 100 );
					Ssd1306PutString( dec_ttostr( buff[SystemTimeE_HOUR] , 0xFF , buff[SystemTimeE_SECOUNDS] ) , 25 , 0 );
					Ssd1306SendRam();
					_delay_ms( 100 );				
				}break;
					
				case 2:
				{
					Ssd1306PutString( dec_ttostr( buff[SystemTimeE_HOUR] , buff[SystemTimeE_MINUTES] , buff[SystemTimeE_SECOUNDS] ) , 25 , 0 );
					Ssd1306SendRam();
					_delay_ms( 100 );
					Ssd1306PutString( dec_ttostr( buff[SystemTimeE_HOUR] , buff[SystemTimeE_MINUTES] , 0xFF ) , 25 , 0 );
					Ssd1306SendRam();
					_delay_ms( 100 );				
				}break;
			}
			sei();
		}     
	}
	
	rtcSetTime( buff[SystemTimeE_HOUR] , buff[SystemTimeE_MINUTES] , buff[SystemTimeE_SECOUNDS] );
 
    return SUCCESS;
}

uint8_t menueSystemTime			( void );

uint8_t reboot					( void );

uint8_t menueExit				( void );

uint8_t operatingHours			( void );


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
	{"Uhrzeit"			,	menueSystemTime				,	0	},
	{"Neustart"			,	reboot						,	0	},
	{"Exit"				,	menueExit					,	0xFF},
};

uint8_t showMenue				(menue_t *m, enc_t *enc, size_t menueLen)
{
	Ssd1306SetFont( System5x7 );
	
	int8_t  pageStart = 0;
	uint8_t menueEntry = 0 , y = 0 , retCursor = 0;
	static uint8_t encWasMoved = 0;
	
	flag.menueSystemTimeout = 0;
	
	while( ! ( flag.menueSystemTimeout ) )
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
			SystemTime.Milisecound = 0;
		}
		
		Ssd1306ClearScreen();
		
		/*
		*	Menü Name
		*/
		Ssd1306PutString( (char*)m[0].Name , 0 , 0 );	
	
	
		/*
		*	Softwareversion
		*/
		Ssd1306PutString( buildVer() , 50 , 0 );

		for ( y = 2 ; y < 6 ; y++ )
		{
			menueEntry = pageStart + ( y - 1 );
			
			if ( menueEntry < menueLen )
			{
				if ( menueEntry == enc->result )
				{	
					retCursor = enc->result;
					Ssd1306PutString( "->" , y * 8 , 0 );
					Ssd1306PutString( (char*)m[menueEntry].Name , y * 8  , 10 );
				}
				else
				{					
					Ssd1306PutString( (char*)m[menueEntry].Name , y * 8 , 0 );
				}		
			}
		}Ssd1306SendRam();
	
		if ( button.enter )
		{	
			button.enter = 0;
			button.enterRpt = 0;
			flag.menueSystemTimeout = 0;	
				
			Ssd1306ClearScreen();
			Ssd1306SendRam();
									
			/*
			*	Funktion aufrufen die für diesen Menüpunkt
			*	hinterlegt ist.
			*/			
			if ( m[retCursor].fp != NULL)
			{
				enc->result = 0 ;
				if (m[retCursor].fp() == MENUE_EXIT )
				{
					Ssd1306SetFont( System5x7 );
					flag.menueSystemTimeout = 0;
					button.enter = 0;
					button.enterRpt = 0;
					break;
				}
			}	
			Ssd1306SetFont( System5x7 );
			flag.menueSystemTimeout = 0;
			button.enter = 0;
			button.enterRpt = 0;
			Ssd1306ClearScreen();
			Ssd1306SendRam();

			enc->result = m[retCursor].cursPos;						
		}		
	}	
	
	Ssd1306ClearScreen();
	Ssd1306SendRam();
	showNewValues = 0; showNewValues_ = 1;	
	return 0;
}

uint8_t menueSystemTime			( void )
{
	cnfgSystemTimee_( tme );
	
	return 0;
}

uint8_t reboot					( void )
{
	Ssd1306ClearScreen();
	
	Ssd1306SetFont( Arial_Black_16 );
	Ssd1306PutString( "Reboot.." , 25 , 0 );	
	Ssd1306SendRam();
	
	_delay_ms(2500);
	
	void (*reboot)(void) = 0;
	reboot();

	return 0;	
}

void	refreshDisplay			( void )
{		
	static char tmp[8] = "";
	static uint8_t avg = 0;
	static uint8_t firstRead = 0;	
	
	if ( Sensor.Ready & 1<<SENSORS_Ready )
	{
 		Sht21Read( &temp_ , &humidity_ );
 		pressRaw = bmp180_get_uncomp_pressure();
 		pressRaw = bmp180_get_pressure( pressRaw );
 	
 		tempRaw = bmp180_get_uncomp_temperature();
 		tempRaw = bmp180_get_temperature( tempRaw );
				
		if ( avg++ == MEASUREMENT_AVERAGE )
		{	
			avg = 0;
 			Sensor.processValue.Bmp180.pressAvrg		/= (uint32_t)MEASUREMENT_AVERAGE;
 			Sensor.processValue.Bmp180.tempAvrg		/= (int32_t)MEASUREMENT_AVERAGE;
 			Sensor.processValue.Sht21.humidityAvrg	/= (uint32_t)MEASUREMENT_AVERAGE;
 			Sensor.processValue.Sht21.tempAvrg		/= (int32_t)MEASUREMENT_AVERAGE;

			Sensor.processValue.Bmp180.pressure		= (uint32_t)Sensor.processValue.Bmp180.pressAvrg;
			Sensor.processValue.Bmp180.temp			= (int32_t)Sensor.processValue.Bmp180.tempAvrg;
			Sensor.processValue.Sht21.humidity		= (uint16_t)Sensor.processValue.Sht21.humidityAvrg;
			Sensor.processValue.Sht21.temp			= (int16_t)Sensor.processValue.Sht21.tempAvrg;

			Sensor.processValue.Bmp180.pressAvrg		= 0;
			Sensor.processValue.Bmp180.tempAvrg		= 0;
			Sensor.processValue.Sht21.humidityAvrg	= 0;
			Sensor.processValue.Sht21.tempAvrg		= 0;
			
			/*
			*	Jede zweite Messung auswerten..
			*/
			if ( firstRead < 2 )
			{
				firstRead++;
			}
			else
			{
				checkMinMax( &Sensor , minMax );
				showNewValues = 0; showNewValues_ = 1;	
				Sensor.Ready |= ( ( 1<<MIN_MAX_VALUES_Ready ) | ( 1<<AVERAGE_Ready ) | ( 1<<AVERAGE_FIRST_Ready ) );
			}	
		}
		else
		{
			Sensor.processValue.Bmp180.pressAvrg		+= pressRaw;
			Sensor.processValue.Bmp180.tempAvrg		+= tempRaw;
			Sensor.processValue.Sht21.humidityAvrg	+= humidity_;
			Sensor.processValue.Sht21.tempAvrg		+= temp_;
		}
		
		Sensor.Ready &= ~( 1<<SENSORS_Ready );		
	}

	if ( Sensor.Ready & 1<<AVERAGE_FIRST_Ready )
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
			Ssd1306SetFont( System5x7 );
			
			if ( Sensor.processValue.Sht21.humidity > 99 )
			{
				Sensor.processValue.Sht21.humidity = 100;
			}
			
			itoa( Sensor.processValue.Bmp180.pressure / 100 , tmp , 10 );
			strcpy( output , "Luftdruck: " );
			strcat( output , tmp );
			strcat( output , " hpa");
			Ssd1306PutString( output , 35 , 0 );
			
			itoa( Sensor.processValue.Sht21.humidity , tmp , 10 );
			strcpy( output , "Luftfeu. : ");
			strcat(output  , tmp );
			strcat(output  , " % ");
			Ssd1306PutString( output, 45 , 0 );
			
			itoa( Sensor.processValue.Sht21.temp , tmp , 10 );
			strcpy(output , "Temp.[1] : ");
			strcat( output , tmp );
			strcat(output , " Cel. " );
			Ssd1306PutString( output , 55 , 0 );
			
			Ssd1306SendRam();
		}
	}
	else
	{
 		Ssd1306SetFont( System5x7 );
 		Ssd1306PutString( "Scanning now.." , 50 , 20 );
		Ssd1306SendRam();
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
	Ssd1306PutString( output , 0 , 0);

	strcpy	( output , mm->nameOfValue );
	strcat	( output , "[max].: ");
	itoa	( mm->max , tmp , 10 );	
	strcat	( output , tmp );		
	Ssd1306PutString( output , pos , 0 );
	
	strcpy	( output , mm->nameOfValue );	
	strcat	( output , "[min].: ");
	itoa	( mm->min , tmp , 10 );
	strcat	( output , tmp );
	Ssd1306PutString( output , pos + 10 , 0 );
				
	Ssd1306SendRam();
				
	return 0;
}

uint8_t menueExit				( void )
{
	button.enter	= 0;
	button.enterRpt = 0;
	
	return MENUE_EXIT;
}

uint8_t eepIsInit				( void )
{
	if ( eeprom_read_byte( &eepInit) == 0xAA )
	{
		return 0;
	}
	
	return 1;
}

void	eepReload				( void )
{
	if ( eepIsInit() )
	{
		eeprom_write_word( &operatingEEP.hours , 0 );
		eeprom_write_byte( &eepInit , 0xAA );
	}
	
	operating.hours = eeprom_read_word( &operatingEEP.hours );
	ram.brightness	= eeprom_read_byte( &eep.brightness );
}

void	updateLiveLed			( uint16_t *mil )
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


int main(void)
{
    DDRx( STATE_LED_PORT )	|= 1<<STATE_LED_bp;
	STATE_LED_PORT			|= (1<<STATE_LED_bp);
	
	eepReload();
	i2c_init();
	Ssd1306Init();
	bmp180_init( &bmp180 );	
	
	/*	SystemTimeer 1 ( 16 Bit ) 
	*	Wird auf CompareMatch eingestellt
	*	Auslöseintervall.: 100ms
	*/
	TCCR1B	= ( ( 1<<CS11 ) | ( 1<<CS10 ) | ( 1<<WGM12 ) ); // Prescaler : 64 
	TIMSK   = ( ( 1<<OCIE1A ) | ( 1<<OCIE2 ) );
	OCR1A	= ((F_CPU / 64 / 100 ) - 1 );
	
	/*	SystemTimeer 2 
	*	Wird auf CompareMatch eingestellt
	*	Auslöseintervall.: 1ms
	*/
	TCCR2  |= ((1<<CS22) | (1<<WGM21)); // Prescaler : 64
	OCR2   = 0x7C; 
	
	/*	Interrupts
	*	Interrupts global freigeben
	*/
	sei();
	
	Ssd1306ClearScreen();
	Ssd1306SendRam();
	
	button.enterRpt = 0;	
	button.enter	= 0;
			
			
    while (1) 
    {	
 		if ( button.enterRpt )
 		{
			button.enterRpt = 0;
			button.enter	= 0;
 			
			Ssd1306ClearScreen();
			Ssd1306SendRam();
			showMenue( menueStructMain , (enc_t*)&enc , sizeof(menueStructMain) / sizeof(menueStructMain[0]) );
 		}
		
		if ( operating.dispAutoOff > DISPLAY_AUTO_OFF )
		{
			operating.dispAutoOff = 0;
			flag.dispIsOff = 1;
			//Ssd1306DisplayState( DISPLAY_OFF );
		}
		
		if ( flag.dispIsOff && button.enter )
		{
			flag.dispIsOff = 0;
			button.enter = 0;
			//Ssd1306DisplayState( DISPLAY_ON );
		}
		
		if ( Sensor.Ready & 1 << RTC_IS_RDY_TO_RD )
		{
			rtcGetData( &rtc );
			Sensor.Ready &= ~( 1 << RTC_IS_RDY_TO_RD );
			Sensor.Ready |= ( 1 << RTC_IS_RDY_TO_SHOW_SystemTimeE );
		}
		
		static uint8_t SecoundOld = 0;
				
		if ( Sensor.Ready & ( 1 << RTC_IS_RDY_TO_SHOW_SystemTimeE ) )
		{					
			if ( SecoundOld != rtc.second )
			{
				SecoundOld = rtc.second;
				Ssd1306SetFont( fixednums15x31 );
				Ssd1306PutString( bcd_ttostr( rtc.hour , rtc.minute , rtc.second , TTOSTR_HH_MM ) , 0 , 22 );
				Ssd1306SendRam();
			}
			Sensor.Ready &= ~( 1 << RTC_IS_RDY_TO_SHOW_SystemTimeE );	
		}
			
		refreshDisplay();		
    }
}

/* SystemTimeer[1] -> Compare Match A
*	Wird ca. jede 10ms aufgerufen
*/
ISR( TIMER1_COMPA_vect )
{
	static uint16_t liveLed = 0;
	static uint8_t rtcRead = 0;
	static uint8_t sensorsRead = 0;
	
	liveLed++;
 	updateLiveLed( &liveLed );

	if ( SystemTime.Milisecound++ >= 1000 )
	{
		SystemTime.Milisecound = 0;
		flag.menueSystemTimeout = 1;
	}

	if ( rtcRead++ > 150 )
	{		
		rtcRead = 0;
		Sensor.Ready |= (1<<RTC_IS_RDY_TO_RD);
	}
	 
	if ( secoundCmp != rtc.second )
	{
		secoundCmp = rtc.second;
		operating.dispAutoOff++;
		
		if ( sensorsRead++ > 5 )
		{
			sensorsRead = 0;
			Sensor.Ready |= (1<<SENSORS_Ready);
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

/* SystemTimeer[2] -> Compare Match
*	Wird ca. jede 1ms aufgerufen
*/
ISR( TIMER2_COMP_vect )
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