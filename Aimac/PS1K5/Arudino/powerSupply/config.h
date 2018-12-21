#ifndef CONFIG_H_
#define CONFIG_H_

#ifndef ON
#define ON 1
#endif

#ifndef OFF
#define OFF 0
#endif

#define DEFAULT_MIN_VOLT	200
#define DEFAULT_MAX_VOLT 	250
#define ADC_READ_SAMPLES	1000

#define RELAY 6
#define NANO_LED PB5

#define ADC_VOLT  A4
#define ADC_CURRENT A6

#define MIN_REQ_VOLT 200
#define MIN_THRESHOLD_VOLT 205

#define MAX_REQ_VOLT 250
#define MIN_THRESHOLD_VOLT 245

#define MIN_LOAD  500
#define MAX_LOAD  1500

#define STABLE_CNT  5
#define STARTUP_CNT  320

#define TOTAL_POWER 1464
#define MAX_POWER (TOTAL_POWER*0.9)


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
