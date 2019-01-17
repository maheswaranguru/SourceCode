//#include <Event.h>
//#include <Timer.h>

//#include <EveryTimer.h>
//#include <OneShotTimer.h>
//#include "Timer.h"

//#include <Arduino.h>

#include "LiquidCrystal_I2C.h"

#include "config.h"
#include "timer.h"

#define RELAY 9
#define BUZZER 10
#define START_UP_DELAY 3000

float voltage[] = { 4,6,8,10,12,14,16 };
float current[] = { 0.6, 2.6, 4.55, 6.57, 8.6, 10.6, 12.6 };

//uint16_t outCurrentAdcLUT[] = {0, 3, 7, 10, 14, 17, 20, 24, 27, 31, 34, 68, 102, 136, 170, 341, 512, 683, 853, 1024 };
//float outCurrentCalLUT[] = { 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 2, 3, 4, 5, 10, 15, 20, 25, 30 };

uint16_t outCurrentAdcLUT[] = {9, 43, 85, 128, 171, 213, 256, 299, 341, 384, 427, 469, 512, 555, 597, 640, 683, 725, 768, 811, 853, 896, 939, 981, 1024 };
float outCurrentCalLUT[] = { 0.2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24 };  

uint8_t sizeAdcLUT = sizeof(outCurrentAdcLUT)/ sizeof(uint16_t);
uint8_t sizeCalCurLUT = sizeof(outCurrentCalLUT)/ sizeof(float);

//***FUNCTION DECLARATION ***
void readVIvalues( void );
void calculateVIvalues( void );
void resetValues( void );
systemError_t checkForThreshold( void );
float findyValue( float x3 );

uint8_t readDipSwd( void );

void Init_LCD( void );
void timerISR( void );

//*** VARIABLE DECLARATION ***
adc_t adc;
data_t outputVolt, inputVolt, outputCurrent, inputCurrent;
unsigned int load = 0;

float power = 0;
unsigned char opLowValue = 0;
unsigned char opHighValue = 0;

bool oneSec_f = false;
systemState_t oldSysStat = NOT_READY;
systemState_t sysStat = NOT_READY;
systemError_t sysErrorStat = NO_ERROR;
uint8_t startUpTimer = 0;
uint8_t minVoltCnt = 0;
uint8_t maxVoltCnt =0;
uint8_t overLoadCnt = 0;

bool buzzerStatus = OFF;
uint8_t buzzCnt = 0;

uint8_t minVoltageLtd = DEFAULT_MIN_VOLT;
uint8_t maxVoltageLtd = DEFAULT_MAX_VOLT;

const unsigned int ADC_OUTPUT_VOLT = A1;
const unsigned int ADC_INPUT_VOLT  = A0;
const unsigned int ADC_OUTPUT_CURRENT  = A3;
const unsigned int ADC_INPUT_CURRENT = A4;

unsigned int looptime = 0;

// initialize the LCD library with the numbers of the interface pins
LiquidCrystal_I2C lcd(0x3F,16,2); // set the LCD address to 0x27 for a 16 chars and 2 line display

/****************************************************
*Name : setup
*Para1:  NA
*Return: NA
*Details:
*****************************************************/
void setup() 
{
  uint8_t  inputSWDStatus = 0;
  
 /* lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight
  Serial.begin(9600);*/
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, OFF);  
  pinMode(BUZZER, OUTPUT);

  Init_LCD();
  inputSWDStatus = readDipSwd();

  sysStat = NOT_READY;
  //delay( START_UP_DELAY );
  lcd.clear();
  
 // TimerInit();

}

/****************************************************
*Name : 
*Para1:
*Return:
*Details:
*****************************************************/
void loop() 
{
         
      readVIvalues();
      calculateVIvalues();

      sysErrorStat = checkForThreshold();

      switch( sysStat )
      {
      case NOT_READY :
                      digitalWrite(RELAY, OFF);
                      sysStat = IDEL;                                            
                      break;

      case IDEL    : 
                    sysStat = NORMAL; 
                    break;

      case NORMAL :
                  digitalWrite(RELAY, ON);
                  digitalWrite(BUZZER, OFF);                  
                  break;

      case MINIMUM_VOLT :
                  //minVoltCnt++;
                  digitalWrite(RELAY, OFF);
                  break;

      case MAXIMUM_VOLT :
                  //maxVoltCnt++;
                  digitalWrite(RELAY, OFF);
                  break;

      case OVER_LOAD :
                 overLoadCnt++;
                 if( SHUTDOWN_POWER < load )
                  {
                      digitalWrite(RELAY, OFF);
                      updateLcd();
                      while(1);
                  }
                  if( buzzCnt++ > 1)
                  {
                    buzzerStatus = !buzzerStatus;
                    digitalWrite(BUZZER, buzzerStatus);
                    buzzCnt = 0;
                  }
                  break;

      case NO_LOAD:
      default :
                digitalWrite(RELAY, OFF);
          break;
      }

    updateLcd();
    resetValues();
}

/****************************************************
*Name : Init_LCD
*Para1:
*Return:
*Details:
*****************************************************/
void Init_LCD( void )
{
    lcd.init(); //initialize the lcd
    lcd.backlight(); //open the backlight
    lcd.clear();
    lcd.print("--- WELCOME ---");
    lcd.setCursor(0,1);
    lcd.print("**** AiMac **** ");
    digitalWrite(BUZZER, ON);
    delay(500);
    digitalWrite(BUZZER, OFF);    
    delay(1000);

    lcd.clear();
    lcd.print("Initializing...");
    lcd.setCursor(0,1);
#ifdef PRODUCD_1K5VA
    lcd.print("*** SAFE 1K5 ***");
#endif
#ifdef PRODUCD_3KVA
    lcd.print("*** SAFE 3K0 ***");
#endif

    delay(1000);

}
/****************************************************
*Name : updateLcd
*Para1:
*Return:
*Details:
*****************************************************/
void updateLcd( void )
{
    if( oldSysStat != sysStat )
    {
        lcd.clear();
        oldSysStat = sysStat;
        minVoltCnt = 0;
        maxVoltCnt =0;
        overLoadCnt = 0;
        digitalWrite( BUZZER, OFF );
    }
    
    switch( sysStat )
    {
    case NORMAL :
        lcd.setCursor(0,0);
        lcd.print( "IPV:    " );
        lcd.setCursor(4,0);
        lcd.print( (int)inputVolt.realWorldValue, DEC );
        lcd.setCursor(8,0);
        lcd.print( " OPV:   " );
        lcd.setCursor(13,0);
        lcd.print( (int)outputVolt.realWorldValue, DEC );

        lcd.setCursor(0,1);
        lcd.print( "OPC:   " );
        lcd.setCursor(4,1);
        lcd.print( outputCurrent.realWorldValue);
        lcd.setCursor(8,1);
        lcd.print( " LOD:   " );
        lcd.setCursor(13,1);
        lcd.print( (int)load, DEC );
        lcd.setCursor(15,1);
        lcd.print( "%" );
        break;
        
    case MAXIMUM_VOLT :
        lcd.clear();
        lcd.print( "IPV:" );
        lcd.print( (int)inputVolt.realWorldValue, DEC );
        lcd.print( " OPV:" );
        lcd.print( (int)outputVolt.realWorldValue, DEC );

        lcd.setCursor(0,1);
        lcd.print( "HighVolt Detect" );
        break;
    case MINIMUM_VOLT :
        lcd.clear();
        lcd.print( "IPV:" );
        lcd.print( (int)inputVolt.realWorldValue, DEC );
        lcd.print( " OPV:" );
        lcd.print( (int)outputVolt.realWorldValue, DEC );

        lcd.setCursor(0,1);
        lcd.print( "LOW Volt Detect" );
        break;
        
    case OVER_LOAD :
        lcd.clear();
        lcd.print( "OPC:   " );
        lcd.setCursor(4,0);
        lcd.print( outputCurrent.realWorldValue);
        lcd.setCursor(8,0);
        lcd.print( " LOD:   " );
        lcd.setCursor(13,0);
        lcd.print( (int)load, DEC );
        lcd.setCursor(15,0);
        lcd.print( "%" );
        
        lcd.setCursor(0,1);
        lcd.print( "OVER LOAD Detect" );
        
        break;
    }   
}

/****************************************************
*Name : readVIvalues
*Para1:
*Return:
*Details:
*****************************************************/
void readVIvalues( void )
{
    
      for( looptime=0;looptime<=ADC_READ_SAMPLES; looptime++ )
      {
    
        //!< OUTPUTVOLTAGE READING
        adc.voltOutput =  analogRead(ADC_OUTPUT_VOLT); 
        if( adc.voltOutput > outputVolt.peakMax )
        {
            outputVolt.peakMax = adc.voltOutput;
        }
        if( adc.voltOutput < outputVolt.lowMin )
        {
            outputVolt.lowMin = adc.voltOutput;
        }

        //!< INPUTVOLTAGE READING
        adc.voltInput = analogRead(ADC_INPUT_VOLT);
        if( adc.voltInput > inputVolt.peakMax )
        {
            inputVolt.peakMax = adc.voltInput;
        }
        if( adc.voltInput < inputVolt.lowMin )
        {
            inputVolt.lowMin = adc.voltInput;
        }
    
        //!< OUTPUTCURRENT READING
        adc.currentOutput = analogRead(ADC_OUTPUT_CURRENT); 
        if( adc.currentOutput > outputCurrent.peakMax )
        {
            outputCurrent.peakMax = adc.currentOutput;
        }
        if( adc.currentOutput < outputCurrent.lowMin )
        {
            outputCurrent.lowMin = adc.currentOutput;
        }
        
    }
}

/****************************************************
*Name :- calculateVIvalues
*Para1:-
*Return:-
*Details:-
*****************************************************/
void calculateVIvalues( void )
{    

    outputVolt.meanValue = outputVolt.peakMax - outputVolt.lowMin;
    outputVolt.realWorldValue = outputVolt.meanValue * 0.449657;                   //  n=(311/1023)*m;0.30400782
    outputVolt.realWorldValue = (outputVolt.realWorldValue/1.390);   
    

    inputVolt.meanValue = inputVolt.peakMax - inputVolt.lowMin;
    inputVolt.realWorldValue = inputVolt.meanValue * 0.449657;                    //  n=(311/1023)*m;0.30400782
    inputVolt.realWorldValue = (inputVolt.realWorldValue/1.390);  

    outputCurrent.meanValue = outputCurrent.peakMax - outputCurrent.lowMin;
    outputCurrent.meanValue /= 1.390; 
    outputCurrent.realWorldValue = findyValue((outputCurrent.meanValue*2)); //  THERE IS ADJUSTMENT MADE TO MAKE CORRRECT i VALUE ... NEED TO FIND ROOTCAUSE.
    
    power = outputVolt.realWorldValue*outputCurrent.realWorldValue;
    load = (unsigned int )((power/TOTAL_POWER)*100);

  }

/********************************************************************************
Name        : findyValue
para 1      : N/A
return      : N/A
Discription : Find the linear equvation value( intermit value )
**********************************************************************************/
float findyValue( float x3 )
{
    uint8_t x1p = 0, x2p = 0;
    uint8_t loop = 0;
    float m = 0;
    float y3 = 0;

    x1p = 0;
    x2p = sizeAdcLUT - 1;
    for(loop = 0;loop<sizeAdcLUT-1;loop++)
    {
        if( x3 > outCurrentAdcLUT[x1p] )
            x1p = loop;
        if( x3 < outCurrentAdcLUT[x2p] )
            x2p = sizeAdcLUT-(loop+1);
    }
    if( x1p != 0 )
    {
        x1p--;
    }
    x2p++;

    m = ( ( outCurrentCalLUT[x2p] - outCurrentCalLUT[x1p] ) / ( outCurrentAdcLUT[x2p] - outCurrentAdcLUT[x1p] ) );
    y3 = ( ((m*(x3-outCurrentAdcLUT[x1p]))+ outCurrentCalLUT[x1p]) );

    lcd.setCursor(0,0);
    lcd.print( outCurrentAdcLUT[x1p] );
    lcd.setCursor(5,0);
    lcd.print( outCurrentAdcLUT[x2p] );

    return( y3 );
}

/****************************************************
*Name :- resetValues
*Para1:- N/A
*Return:-N/A
*Details:-  reset all parameters for new reading.
*****************************************************/
void resetValues( void )
{
    outputVolt.peakMax = CLEAR;
    outputVolt.lowMin = SET_WORD;
    inputVolt.peakMax = CLEAR;
    inputVolt.lowMin = SET_WORD;
    outputCurrent.peakMax = CLEAR;
    outputCurrent.lowMin = SET_WORD;
    inputCurrent.peakMax = CLEAR;
    inputCurrent.lowMin = SET_WORD;
    load = CLEAR;
}

/****************************************************
*Name :- checkForThreshold
*Para1:-  N/A
*Return:- Error Code
*Details:-  Check for any error.
*****************************************************/
systemError_t checkForThreshold( void )
{
    systemError_t lstatus = NO_ERROR;

    if(  MAXIMUM_VOLT != sysStat )
    {
     if( inputVolt.realWorldValue > maxVoltageLtd  )
      {
          sysStat = MAXIMUM_VOLT;
          lstatus = HIGHVOLTAGE;          
      }
    }else
    {
     if( inputVolt.realWorldValue < ( maxVoltageLtd - INPUTVOLT_THRESHOLD ) )
      {
          sysStat = NORMAL;
          lstatus = NO_ERROR;          
      }else
      {
            sysStat = MAXIMUM_VOLT;
            lstatus = HIGHVOLTAGE;          
      }
    }
    
    if ( MINIMUM_VOLT != sysStat )
    {
       if( inputVolt.realWorldValue < minVoltageLtd )
       {
          sysStat = MINIMUM_VOLT;
          lstatus = LOWVOLTAGE;
       }
    }else
    {
        if( inputVolt.realWorldValue > ( minVoltageLtd+ INPUTVOLT_THRESHOLD ) )
        {
          sysStat = NORMAL;
          lstatus = NO_ERROR;       
        }else
        {
          sysStat = MINIMUM_VOLT;
          lstatus = LOWVOLTAGE;            
        }        
    }    
      if( load > MAX_POWER) 
      {
           if( overLoadCnt++ > LOAD_STABLE_CNT )      // Wait for stable no of times to conclude 
           {
               sysStat = OVER_LOAD;
               lstatus = OVERLOAD;
               overLoadCnt = CLEAR;
           }
           if( OVER_LOAD == sysStat )
           {
               lstatus = OVERLOAD;
           }           
      }
      
      if( lstatus == NO_ERROR )
      {
          sysStat = NORMAL;
      }

    return( lstatus );

}
/****************************************************
*Name :- readDipSwd
*Para1:-  N/A
*Return:- DIP switch Values
*Details:- Read DIP switch and asign input voltafe throshold 
*****************************************************/
uint8_t readDipSwd( void )
{
    uint8_t returnValue = 0;
    
    if( !digitalRead(5) )
    {
        returnValue |= (1<<0);
    }
    if( !digitalRead(6) )
    {
        returnValue |= (1<<1);
    }
    if( !digitalRead(7) )
    {
        returnValue |= (1<<2);
    }
    if( !digitalRead(8) )
    {
        returnValue |= (1<<3);
    }
    
    switch( returnValue )
    {
        case 0x00 :
            minVoltageLtd = inVoltRange[0];
            maxVoltageLtd = inVoltRange[1];
            break;
        case 0x01 :
            minVoltageLtd = inVoltRange[2];
            maxVoltageLtd = inVoltRange[3];
            break;
        case 0x02 :
            minVoltageLtd = inVoltRange[4];
            maxVoltageLtd = inVoltRange[5];
            break;
        case 0x04 :
            minVoltageLtd = inVoltRange[6];
            maxVoltageLtd = inVoltRange[7];
            break;
        case 0x08 :
            minVoltageLtd = inVoltRange[8];
            maxVoltageLtd = inVoltRange[9];
            break;
        default :
        {
            lcd.clear();
            lcd.print("  WRONG INPUT  ");
            lcd.setCursor(0,1);
            lcd.print("VOLTAGE SELECT");
            while(1);          
        }          
        
    }
    return ( returnValue );
    
}
