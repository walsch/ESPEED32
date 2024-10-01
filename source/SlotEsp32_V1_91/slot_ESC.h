#ifndef SLOT_ESC_H_
#define SLOT_ESC_H_

/*********************************************************************************************************************/
/*------------------------------------------------------Includes-----------------------------------------------------*/
/*********************************************************************************************************************/
#include <OneBitDisplay.h>
#include <AiEsp32RotaryEncoder.h>
#include <AiEsp32RotaryEncoderNumberSelector.h>

#include "half_bridge.h"
#include "HAL.h"

/*********************************************************************************************************************/
/*-------------------------------------------------------Macros------------------------------------------------------*/
/*********************************************************************************************************************/

#define MENU_ITEMS_COUNT    9   /* Number of items in the menu */
#define MENU_ACCELERATION   0   /* Encoder acceleration when in the main menu */
#define SEL_ACCELERATION    100 /* Encoder acceleration when selecting parameter value */
#define ITEM_NO_CALLBACK    0   /* For when a item has no callback */
#define ITEM_NO_VALUE       0   /* For when an item has no value to be displayed */
#define MAX_ITEMS           10

#define MIN_SPEED_DEFAULT         10  // [%] userMinSpeed default                  
#define BRAKE_DEFAULT             90  // [%]
#define DRAG_BRAKE_DEFAULT        50  // [%]  
#define ANTISPIN_DEFAULT          50  // ms
#define MAX_SPEED_DEFAULT         100 // 
#define THR_CRV_OUT_LEVEL_DEFAULT 50
#define PWM_FREQ_DEFAULT          20   //[100*Hz]
#define DRAG_BRAKE_TYPE_DEFAULT   DRAG_BRAKE_T_DEC

#define THR_CRV_IN_LEVEL_JOINT THROTTLE_NORMALIZED/2 // throttle curve will break at 50% of the trigger travel

#define MIN_SPEED_MAX_VALUE 90
#define DRAG_MAX_VALUE      100
#define FREQ_MAX_VALUE      5000
#define BRAKE_MAX_VALUE     100
#define THR_SETP_MAX_VALUE  90
#define THR_SETP_MIN_VALUE  10
#define ANTISPIN_MAX_VALUE  250
#define FREQ_MIN_VALUE      200
#define MAX_UINT16          32767

#define HEIGHT12x16 16 // height of 12x16 characters
#define HEIGHT8x8   8 // height of 12x16 characters
#define WIDTH8x8    8 // height of 12x16 characters
#define WIDTH12x16  12 // width of 12x16 characters

#define LOOPTIME_MAX_CHECK  100
#define TIMER_FREQ          1000000 /* frequency of the timer interrupt in Hz (1MHz)*/
#define ESC_PERIOD_US       500    /* Period of the ESC alarm in microseconds */

#define CAR_MAX_COUNT       10 // how many different car model setting can be stored
#define CAR_NAME_MAX_SIZE   5 /* 4 char + terminator \0 */

#define CAR_OPTION_SELECT   0
#define CAR_OPTION_RENAME   1
#define CAR_OPTION_SAVE     2 /* DEPRECATED - NOT USED */

#define RENAME_CAR_SELECT_OPTION_MODE 0
#define RENAME_CAR_SELECT_CHAR_MODE   1
#define RENAME_CAR_MIN_ASCII    33
#define RENAME_CAR_MAX_ASCII    122

#define DRAG_BRAKE_T_FULL     0
#define DRAG_BRAKE_T_DEC      1
/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

/* Enum definition for the Finite State Machine */
typedef enum FSM_state_enum
{ 
  SELF_CHECK, 
  INIT, 
  CALIBRATION,
  RUNNING, 
  FAULT
} FSMstate;

/* Enum definition for the Main Menu state machine */
typedef enum menu_state_enum
{
  ITEM_SELECTION,
  VALUE_SELECTION
} menuState;

/* Enum definition for what kind of value is going to be displayed next to the item name */
typedef enum menu_item_value_enum
{
  VALUE_TYPE_INTEGER,
  VALUE_TYPE_DECIMAL,
  VALUE_TYPE_STRING,
} itemValueType;
/* Comment on representation of decimal values in the menu: the value must be represented as integer, and it must be specified where the decimal point is placed:
   e.g.: PWM freq is a value that we want represented in kHz, ranging from 0.2 to 5.0. We are gonna create the menu item type with the following fields:
         type: VALUE_TYPE_DECIMAL;
         maxValue: 50;
         minValue: 2;
         decimalPoint: 1;
         meaning that internally the values ranges from 2 to 50. When selected with the encoder in the main menu the value can be increased 1 by 1.
         The value that is displayed will be divided by 10^decimalPoint, to show values that ranges from 0.2 to 5.0.
         The application must take into account that the stored value will range from 2 to 50 during the calculations. */


typedef struct {
  uint16_t inThrottle;
  uint16_t outSpeed;
} throttle_curve_set_point_type;


typedef struct {
  uint16_t minSpeed; 
  uint16_t brake;    
  uint16_t dragBrake;
  uint16_t dragType;
  uint16_t maxSpeed;
  throttle_curve_set_point_type throttleSetPoint; 
  uint16_t antiSpin;//[ms]
  char     carName[CAR_NAME_MAX_SIZE];// 5 letters plus end of line
  uint16_t carNumber; /* Simply to identify the position in the array, not to be changed */
  uint16_t freqPWM; /* motor PWM frequency */
}car_param_type;

/* Struct definition for variables stored in the EEPROM */
typedef struct {
  car_param_type carParam[CAR_MAX_COUNT]; 
  uint16_t  carSelNumber;// which car is 
  int16_t  minTrigRaw;
  int16_t  maxTrigRaw;
} EEPROM_stored_var_type;


/* ESC_var_type: struct that contains all the charger variables */
typedef struct {
  uint16_t  outSpdRequest;    // [%] Set speed (desired target)
  uint16_t  outSpdSetPerc;    // [%] Set speed (desired target)
  int16_t   trigRaw;
  uint16_t  trigNorm;   // [%] trigger position
  int16_t   trigDeriv;
  uint16_t  encoderPos; // [%] encodReadRaw
  uint16_t  Vin_mV;
} ESC_var_type;


/* Define a pointer to a void function that takes no arguments */
typedef void (*function_pointer_type)(void);


/* menu_item_type: struct that defines an item of the menu */
typedef struct {
  char name[10];                  /* Name of the item that is displayed in the menu */
  void *value;                    /* Pointer to the variable that contains the value of the item. When assign, must cast to (void *). It's a generic void pointer so that it can be used to print any type of value specified by the type field */
  menu_item_value_enum type;      /* What kind of variable is value pointing to. Only applies if value is not ITEM_NO_VALUE. STRING type of value must be 5 letters, for INTEGER type of values can be up to 4 digits, for DECIMAL type of values can be up to 3 digits */
  uint16_t maxValue;              /* Maximum possible value fo the item. Only applies if  type is VALUE_TYPE_INTEGER or VALUE_TYPE_DECIMAL */
  uint16_t minValue;              /* Minimum possible value fo the item. Only applies if  type is VALUE_TYPE_INTEGER or VALUE_TYPE_DECIMAL */
  char unit;                      /* Measurement unit of the item, that's gonna be displayed next to the value. Must be one character */
  uint8_t decimalPoint;           /* Indicates where is placed the decimal point. Only applies if  type is VALUE_TYPE_DECIMAL. Possible values are 1, and 2. */
  function_pointer_type callback; /* Pointer to a callback that is called when the item is clicked in the menu. If no callback, then set to ITEM_NO_CALLBACK */
} menu_item_type;


typedef struct {
  menu_item_type item[MAX_ITEMS];  /* Array of pointers to menu_item_type */
  uint16_t lines;                     /* How many lines displayed at a time */
} menu_type;


/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
uint16_t  addDeadBand(uint16_t val_pct, uint16_t deadBand_pct);
uint16_t  saturateParamValue(uint16_t paramValue, uint16_t minValue, uint16_t maxValue);
uint16_t  throttleCurve(uint16_t inputThrottle, EEPROM_stored_var_type storedVar);
void      throttleCalibration(uint16_t adcRaw);
uint16_t  throttleAntiSpin(uint16_t throtSet, EEPROM_stored_var_type storedVar);
void      showRunningDisplay(menu_state_enum currMenuState);
void      showScreenCalibration(int16_t adcRaw);


#endif