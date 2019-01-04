//#include <Event.h>
//#include <Timer.h>

//#include <EveryTimer.h>
//#include <OneShotTimer.h>
//#include "Timer.h"

//#include <Arduino.h>

#include "LiquidCrystal_I2C.h"

#include "config.h"
#include "timer.h"

#define RELAY 12
#define START_UP_DELAY 3000

float voltage[] = { 4,6,8,10,12,14,16 };
float current[] = { 0.6, 2.6, 4.55, 6.57, 8.6, 10.6, 12.6 };

uint16_t outCurrentAdcLUT[] = {0, 3, 7, 10, 14, 17, 20, 24, 27, 31, 34, 68, 102, 136, 170, 341, 512, 683, 853, 1024 };
float outCurrentCalLUT[] = { 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 2, 3, 4, 5, 10, 15, 20, 25, 30 };
uint8_t sizeAdcLUT = sizeof(outCurrentAdcLUT)/ sizeof(uint16_t);
uint8_t sizeCalCurLUT = sizeof(outCurrentCalLUT)/ sizeof(float);

//***FUNCTION DECLARATION ***
void readVIvalues( void );
void calculateVIvalues( void );
void resetValues( void );
systemError_t checkForThreshold( void );
float findyValue( int x3 );

void Init_LCD( void );
void timerISR( void );

//*** VARIABLE DECLARATION ***
adc_t adc;
data_t outputVolt, inputVolt, outputCurrent, inputCurrent;
unsigned int load;

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

uint8_t minVoltageLtd = DEFAULT_MIN_VOLT;
uint8_t maxVoltageLtd = DEFAULT_MAX_VOLT;

const unsigned int ADC_OUTPUT_VOLT = A1;
const unsigned int ADC_INPUT_VOLT  = A0;
const unsigned int ADC_OUTPUT_CURRENT  = A3;
const unsigned int ADC_INPUT_CURRENT = A4;

unsigned int looptime = 0;

// initialize the LCD library with the numbers of the interface pins
LiquidCrystal_I2C lcd(0x27,16,2); // set the LCD address to 0x27 for a 16 chars and 2 line display

/****************************************************
*Name : setup
*Para1:  NA
*Return: NA
*Details:
*****************************************************/
void setup() 
{
  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight
  Serial.begin(9600);
  
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, OFF);

  sysStat = NOT_READY;
  delay( START_UP_DELAY );
  TimerInit();


  //digitalWrite(RELAY, HIGH);
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
                  if(STARTUP_CNT <= ++startUpTimer)
                  {
                    sysStat = NORMAL;
                  }
                  break;

      case NORMAL :
                  if( NO_ERROR == sysErrorStat )
                  {
                      digitalWrite(RELAY, ON);
                  }else if(  ( HIGHVOLTAGE == sysErrorStat ) || ( LOWVOLTAGE == sysErrorStat ) || ( OVERLOAD == sysErrorStat ) )
                  {
                      digitalWrite(RELAY, OFF);
                  }
                  break;

      case MINIMUM_VOLT :
                  minVoltCnt++;
                  break;

      case MAXIMUM_VOLT :
                      maxVoltCnt++;
                  break;

      case OVER_LOAD :
                      overLoadCnt++;
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
    lcd.clear();
    lcd.print("--- WELCOME ---");
    lcd.setCursor(0,1);
    lcd.print("**** AiMac **** ");
    delay(2000);

    lcd.clear();
    lcd.print("Initializing...");
    lcd.setCursor(0,1);
    lcd.print("Safe 1K5 ");

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
    }
    
    switch( sysStat )
    {
    case NORMAL :
        lcd.print( "IPV:" );
        lcd.print( (int)inputVolt.realWorldValue, DEC );
        lcd.print( " OPV:" );
        lcd.print( (int)outputVolt.realWorldValue, DEC );

        lcd.setCursor(0,1);
        lcd.print( "OPC:" );
        lcd.print( (int)outputCurrent.realWorldValue, DEC );
        lcd.print( " LOD:" );
        lcd.print( (int)load, DEC );
        lcd.setCursor(15,1);
        lcd.print( "%" );

    case HIGHVOLTAGE :

        lcd.print( "IPV:" );
        lcd.print( (int)inputVolt.realWorldValue, DEC );
        lcd.print( " OPV:" );
        lcd.print( (int)outputVolt.realWorldValue, DEC );

        lcd.setCursor(0,1);
        lcd.print( "HighVolt Detect" );
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
        adc.voltOutput = analogRead(ADC_OUTPUT_VOLT); 
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
    outputCurrent.realWorldValue = findyValue(outputCurrent.meanValue);
    power = outputVolt.realWorldValue*outputCurrent.realWorldValue;

    load = (power/TOTAL_POWER)*100;

  }

/********************************************************************************
Name        : findyValue
para 1      : N/A
return      : N/A
Discription : Find the linear equvation value( intermit value )
**********************************************************************************/
float findyValue( int x3 )
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
        if( x3 < voltage[x2p] )
            x2p = outCurrentAdcLUT-(loop+1);
    }
    if(x1p != 0)
    {
        x1p--;
    }
    x2p++;

    //outputCurrent.realWorldValue = x1p;
    // load = x2p;

    m = ( outCurrentCalLUT[x2p] - outCurrentCalLUT[x1p] )/ ( outCurrentAdcLUT[x2p] - outCurrentAdcLUT[x1p] );
    y3 = ((m*(x3-outCurrentAdcLUT[x1p]))+ outCurrentCalLUT[x1p]);

    return( y3 );
}

/****************************************************
*Name :- resetValues
*Para1:-
*Return:-
*Details:-
*****************************************************/
void resetValues( void )
{
    outputVolt.peakMax = 0;
    outputVolt.lowMin = 0x3FF;
    inputVolt.peakMax = 0;
    inputVolt.lowMin = 0x3FF;
    outputCurrent.peakMax = 0;
    outputCurrent.lowMin = 0x3FF;
    inputCurrent.peakMax = 0;
    inputCurrent.lowMin = 0x3FF;
    load = 0;
}

/****************************************************
*Name :- checkForThreshold
*Para1:-
*Return:-
*Details:-
*****************************************************/
systemError_t checkForThreshold( void )
{
    systemError_t lstatus = NO_ERROR;
    int power  = 0;

    if( MIN_REQ_VOLT >= inputVolt.realWorldValue )
    {
        lstatus |= LOWVOLTAGE;
    }

    if( MAX_REQ_VOLT< inputVolt.realWorldValue )
    {
        lstatus |= HIGHVOLTAGE;
    }

    if ( load >= MAX_POWER )
    {
        lstatus |= OVERLOAD;
    }

    return( lstatus );

}
