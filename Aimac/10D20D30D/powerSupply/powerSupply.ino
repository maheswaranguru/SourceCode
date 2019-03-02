/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
This is the source code for 10D, 20D, 30D.
By : Maheswaran Ggurusamy.
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

#include "config.h"

int16_t outCurrentAdcLUT[] = {1 ,9 ,17 ,26 ,34 ,43 ,51 ,60 ,68 ,77 ,86 ,94 ,103 ,111 ,120 ,128 ,137 ,146 ,154 ,163 ,171 ,180 ,188 ,197 ,205 };
float outCurrentCalLUT[] = { 0.1 ,1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 ,16 ,17 ,18 ,19 ,20 ,21 ,22 ,23 ,24 };


uint8_t sizeAdcLUT = sizeof(outCurrentAdcLUT)/ sizeof(uint16_t);
uint8_t sizeCalCurLUT = sizeof(outCurrentCalLUT)/ sizeof(float);

//***FUNCTION DECLARATION ***
void readVIvalues( void );
void calculateVIvalues( void );
void resetValues( void );
systemError_t checkForThreshold( void );
float findyValue( float x3 );
void updateRelay( bool status1, bool status2 );

//*** VARIABLE DECLARATION ***
adc_t adc;
data_t outputVolt, inputVolt, outputCurrent, inputCurrent;
unsigned int load = 0;

float power = 0;

bool oneSec_f = false;
systemState_t oldSysStat = NOT_READY;
systemState_t sysStat = NOT_READY;
systemError_t sysErrorStat = NO_ERROR;
uint8_t minVoltCnt = 0;
uint8_t maxVoltCnt = 0;
uint8_t overLoadCnt = 0;

bool buzzerStatus = OFF;
uint8_t buzzCnt = 0;

uint8_t minVoltageLtd = DEFAULT_MIN_VOLT;
uint8_t maxVoltageLtd = DEFAULT_MAX_VOLT;

const unsigned int ADC_INPUT_VOLT  = A0;
const unsigned int ADC_OUTPUT_CURRENT  = A3;

unsigned int looptime = 0;


/****************************************************
*Name : setup
*Para1:  NA
*Return: NA
*Details:
*****************************************************/
void setup() 
{

    pinMode(5, INPUT);
    pinMode(6, INPUT);
    pinMode(7, INPUT);
    pinMode(8, INPUT);

    pinMode(LED_POWER_OK, OUTPUT);
    pinMode(LED_OUT_LIMT, OUTPUT);
    pinMode(LED_OVERLOAD, OUTPUT);

    pinMode(RELAY1, OUTPUT);
    digitalWrite(RELAY1, OFF);  
    pinMode(BUZZER, OUTPUT);


    digitalWrite(LED_POWER_OK, OFF);
    digitalWrite(LED_OUT_LIMT, OFF);
    digitalWrite(LED_OVERLOAD, OFF);

  sysStat = NOT_READY;

}

/****************************************************
*Name : MAIN lOOP
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
                    digitalWrite(RELAY1, OFF);
                    sysStat = IDEL;                                            
                    break;

      case IDEL    : 
                    sysStat = NORMAL; 
                    break;

      case NORMAL :
                    digitalWrite(RELAY1, ON);
                    digitalWrite(BUZZER, OFF);                  
                    break;

      case MINIMUM_VOLT :
                    //minVoltCnt++;
                    digitalWrite(RELAY1, OFF);
                    break;

      case MAXIMUM_VOLT :
                    //maxVoltCnt++;
                    digitalWrite(RELAY1, OFF);
                    break;

      case OVER_LOAD :
                 overLoadCnt++;
                    if( SHUTDOWN_POWER < load )
                    {
                      digitalWrite(RELAY1, OFF);
                      updateDisplay();
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
                digitalWrite(RELAY1, OFF);
          break;
      }

    updateDisplay();
    resetValues();
}

/****************************************************
*Name : updateDisplay
*Para1:N/A
*Return:N/A
*Details: Update display for current state.
*****************************************************/
void updateDisplay( void )
{
    switch( sysStat )
    {
        case NORMAL :
                    digitalWrite(LED_POWER_OK, ON);
                    digitalWrite(LED_OUT_LIMT, ON);
                    digitalWrite(LED_OVERLOAD, OFF);                            
        break;                
        case MINIMUM_VOLT : 
        case NOT_READY :
        case MAXIMUM_VOLT:
                    digitalWrite(LED_POWER_OK, OFF);
                    digitalWrite(LED_OUT_LIMT, OFF);
                    digitalWrite(LED_OVERLOAD, OFF);                                    
        break;
        
        case OVER_LOAD :
                    digitalWrite(LED_POWER_OK, OFF);
                    digitalWrite(LED_OUT_LIMT, OFF);
                    digitalWrite(LED_OVERLOAD, ON);            
        break; 
    }    
}

/****************************************************
*Name : readVIvalues
*Para1:N/A
*Return:N/A
*Details: Read adc for no times . this should control by TIMER.
*****************************************************/
void readVIvalues( void )
{
    
      for( looptime=0;looptime<=ADC_READ_SAMPLES; looptime++ )
      {

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


    inputVolt.meanValue = inputVolt.peakMax - inputVolt.lowMin;
    inputVolt.realWorldValue = inputVolt.meanValue * 0.449657;                    //  n=(311/1023)*m;0.30400782
    inputVolt.realWorldValue = (inputVolt.realWorldValue/1.390);  

    outputCurrent.meanValue = outputCurrent.peakMax - outputCurrent.lowMin;
    outputCurrent.meanValue /= 1.390; 
    outputCurrent.realWorldValue = findyValue((outputCurrent.meanValue)); //  THERE IS ADJUSTMENT MADE TO MAKE CORRRECT i VALUE ... NEED TO FIND ROOTCAUSE.
    
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
*Name :- updateRelay
*Para1:-  first relay status.
*Para2:-  second relay status.
*Return:- N/A
*Details:-  Update Relay status.
*****************************************************/
void updateRelay( bool status1, bool status2 )
{
    digitalWrite(RELAY1, status1);
 
#if PRODUCT_VARIANT != 20 
    digitalWrite(RELAY2, status2);
#else 
(void) status2;
#endif

}
