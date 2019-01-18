#ifndef CONFIG_H_
#define CONFIG_H_

//#define PRODUCT_3KVA (0)      // CHOOSE WHICH VARIENT NEED TO BUILD !!!
#define PRODUCT_1K5VA  (1)

#ifndef ON
#define ON 1
#endif

#ifndef OFF
#define OFF 0
#endif

#ifndef CLEAR
#define CLEAR (0)
#endif

#ifndef SET
#define SET (1)
#endif

#ifndef SET_WORD
#define SET_WORD (0xFFFF)
#endif

#define DEFAULT_MIN_VOLT	200
#define DEFAULT_MAX_VOLT 	250
#define ADC_READ_SAMPLES	1000

#define RELAY 9
#define NANO_LED PB5

#define LCD_ADDRESS  0x3F

#define ADC_VOLT  A4
#define ADC_CURRENT A6

#define MIN_REQ_VOLT 200
#define MIN_THRESHOLD_VOLT 205

#define MAX_REQ_VOLT 250
#define MAX_THRESHOLD_VOLT 245

#define MIN_LOAD  500
#define MAX_LOAD  1500

#define STABLE_CNT  5
#define STARTUP_CNT  320

#ifdef PRODUCT_1K5VA
#define TOTAL_POWER 1426       // (  voltage * current == 230 * 6.2 )
#endif

#ifdef PRODUCT_3KVA
#define TOTAL_POWER 2990       // (  voltage * current == 230 * 13 )
#endif

#define MAX_POWER (90)        //(TOTAL_POWER*0.9)
#define SHUTDOWN_POWER (95)  //( TOTAL_POWER*0.95 )

#define LOAD_STABLE_CNT 3

#define INPUTVOLT_THRESHOLD  5
const uint8_t inVoltRange[] = { 220, 245, 220, 255, 200, 245, 190, 245, 180, 245 };



typedef struct
{
  unsigned int voltInput;
  unsigned int voltOutput;
  unsigned int currentOutput;
}adc_t;

typedef struct
{
    unsigned int peakMax;
    unsigned int lowMin;
    unsigned int meanValue;    
    float realWorldValue;    
}data_t;


typedef enum{
  NOT_READY = 0,
  IDEL,
  NORMAL,
  MINIMUM_VOLT,
  MAXIMUM_VOLT,
  OVER_LOAD,
  NO_LOAD
}systemState_t;


typedef enum{
	NO_ERROR = 0,
	HIGHVOLTAGE,
	LOWVOLTAGE,
	NOLOAD,
	OVERLOAD,
	ERROR_MAX
}systemError_t;

#endif
