/*********************************************************************************************************************/
/*------------------------------------------------------Includes-----------------------------------------------------*/
/*********************************************************************************************************************/
#include "slot_ESC.h"

/*********************************************************************************************************************/
/*-------------------------------------------------------Macros------------------------------------------------------*/
/*********************************************************************************************************************/
#define BOOTLOADER_BUILD 1
//#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#define CONFIG_BOOTLOADER_LOG_LEVEL ESP_LOG_NONE

#define SW_MAJOR_VERSION 1
#define SW_MINOR_VERSION 99
/* Last modified: 15/10/2024 */
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

TaskHandle_t Task1;
TaskHandle_t Task2;

static StateMachine_enum g_currState = INIT;
uint16_t debug;

/* Display Backbuffer */
#ifdef USE_BACKBUFFER
static uint8_t ucBackBuffer[1024];
#else
static uint8_t *ucBackBuffer = NULL;
#endif

OBDISP g_obd;       /* Display global instance */

char msgStr[50];    /* Display string global instance */


uint16_t g_carSel;  /* global variable telling whch car model has been selected */

/* Rotary Encoder global instance */
AiEsp32RotaryEncoder g_rotaryEncoder = AiEsp32RotaryEncoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_BUTTON_PIN, ENCODER_VCC_PIN, ENCODER_STEPS);

static uint8_t g_encoderMainSelector = 1;           /* Value of main menu selector (indicates selected item) */
static uint8_t g_encoderSecondarySelector = 0;      /* Value of secondarty selector (indicates selected item's value) */
static uint16_t *g_encoderSelectedValuePtr = NULL;  /* Global instance of pointer to the value selected by the encoder */


/* EEPROM stored values (mainly calibration) global instance */
StoredVar_type g_storedVar;

/* ESC internal variables gloabl instance */
ESC_type g_escVar{
  .outputSpeed_pct = 0,
  .trigger_raw = 0,
  .trigger_norm = 0,
  .triggerDerivative = 0,
  .encoderPos = 1,
  .Vin_mV = 0
};

/* Main menu global instances */
Menu_type g_mainMenu{
  .lines = 3
};

/* Car menu global instances */
Menu_type g_carMenu{
  .lines = 3
};

/* Preferences global instance (for storing NVM data, replace EEPROM library) */
Preferences g_pref;

static uint32_t g_lastEncoderInteraction = 0;  /* tell how much time has passed since last time encoder whas rotated/pressed
                                                  so we can avoid keep printing the display menu and save CPU cycles */

/*********************************************************************************************************************/
/*--------------------------------------------- Function Declaration----------------------------------------*/
/*********************************************************************************************************************/
void IRAM_ATTR readEncoderISR();


/*********************************************************************************************************************/
/*---------------------------------------------Setup Function Implementations----------------------------------------*/
/*********************************************************************************************************************/

/* Setup */
void setup()
{

  /***** Pin and Serial Setup *****/
  HAL_PinSetup();

  /***** HalfBridge & HW Setup *****/
  HalfBridge_SetupFabio();

  /***** create a task that will be executed in the Task1code() and Task2code() funcitons, executed on core 0 and 1 *****/
  /* TASK1: slotESC state machine, managex OLED display and Encoder, low priority task */
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  /* TASK2: perform trigger reading, trigger conditioning, PWM output */
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    2,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
}


/*********************************************************************************************************************/
/*---------------------------------------------MAIN LOOP IN TASKS----------------------------------------------*/
/*********************************************************************************************************************/

/**
 * Task 1: slotESC state machine: manage OLED display and Encoder, low priority task
 */
void Task1code(void *pvParameters) 
{
  for (;;) 
  {
    StateMachine_enum prevState = g_currState;  /* Keep track of the state at the previous loop */
    static uint16_t prevFreqPWM = 0;            /* Keep track if the PWM freq has changed */
    static MenuState_enum menuState = ITEM_SELECTION;   /* State of the Main Menu */
    static uint8_t swMajVer, swMinVer;                  /* SW major and minor version stored in the eeprom */

    g_escVar.Vin_mV = HAL_ReadVoltageDivider(AN_VIN_DIV, RVIFBL, RVIFBH); /* Read VIN */

    if (g_currState != INIT) /* If the user params are already fetched from the EEPROM */
      {
        g_carSel = g_storedVar.selectedCarNumber;    /* Update global variable telling which car model is actually selected */
      }

    /* Task 1 state machine */
    switch (g_currState) {
      case INIT:

        g_pref.begin("stored_var", false); /* Open the "stored" namespace in read/write mode. If it doesn't exist, it creates it */
        
        if (g_pref.isKey("sw_maj_ver") && g_pref.isKey("sw_min_ver") && g_pref.isKey("user_param")) /* If all keys exists, then check their value */
        {
          /* Get the values of the sw version */
          swMajVer = g_pref.getUChar("sw_maj_ver");
          swMinVer = g_pref.getUChar("sw_min_ver");

          if ((swMajVer == SW_MAJOR_VERSION) && (swMinVer == SW_MINOR_VERSION)) /* If both keys are equal to the SW Version MACRO, then the stored param are already initialized */
          {
            g_pref.getBytes("user_param", &g_storedVar, sizeof(g_storedVar)); /* Get the value of the stored user_param */
            initMenuItems();                                                  /* init menu items with EEPROM stored variables */

            /* If button is pressed at startup, go to CALIBRATION state */
            if (digitalRead(ENCODER_BUTTON_PIN) == BUTTON_PRESSED) 
            {
              g_currState = CALIBRATION;      /* Go to CALIBRATION state */
              /* Reset Min and Max to the opposite side, in order to have effective calibration */
              g_storedVar.minTrigger_raw = MAX_INT16;
              g_storedVar.maxTrigger_raw = MIN_INT16;
              calibSound();             /* Play calibration sound */
              initDisplayAndEncoder();  /* init and clear OLED and Encoder */

              /* Wait until button is released, then go to CALIBRATION state */
              while (digitalRead(ENCODER_BUTTON_PIN) == BUTTON_PRESSED)
              {
                showScreenPreCalibration();
              }
              
              obdFill(&g_obd, OBD_WHITE, 1); /* Clear OLED */
            }
            else  /* If button is NOT pressed at startup, go to RUNNING state */
            {
              g_currState = WELCOME;                    /* Go to WELCOME state */
              g_carSel = g_storedVar.selectedCarNumber; /* now it is safe to address the proper car */
              initDisplayAndEncoder();  /* init and clear OLED and Encoder */
              onSound();                /* Play ON sound */
            }

            g_pref.end(); /* Close the namespace */
            break;        /* Break the switch case: if code reaches here, it means that the stored user param and sw versions are OK */
          }
        }

        /* If the code reaches here it means that:
        - the sw version keys are not present --> stored var are not initialized
        - the sw version stored are not up to date --> stored var are initialized but might be outdated

        Calibration values are NOT stored, go to CALIBRATION state */
        initDisplayAndEncoder();  /* init and clear OLED and Encoder */
                              
        g_pref.clear();           /* Clear all the keys in this namespace */
        

        /* Store the correct SW version */
        g_pref.putUChar("sw_maj_ver", SW_MAJOR_VERSION);
        g_pref.putUChar("sw_min_ver", SW_MINOR_VERSION);

        initStoredVariables();  /* Initialize stored variables with default values */

        /* Reset Min and Max to the opposite side, in order to have effective calibration */
        g_storedVar.minTrigger_raw = MAX_INT16;
        g_storedVar.maxTrigger_raw = MIN_INT16;
        calibSound();                   /* Play calibration sound */
        g_currState = CALIBRATION;      /* Go to CALIBRATION state */
        obdFill(&g_obd, OBD_WHITE, 1); /* Clear OLED */
        /* Press and release button to go to CALIBRATION state */
        while (!g_rotaryEncoder.isEncoderButtonClicked()) /* Loop until button is pressed */
        {
          showScreenNoEEPROM();
        }

        break;


      case CALIBRATION:
        /* Read Throttle */
        throttleCalibration(g_escVar.trigger_raw);    /* trigger raw is continuously read on task2 */
        showScreenCalibration(g_escVar.trigger_raw);  /* Show calibration screen */
        /* Exit calibration if button is presseded */
        if (g_rotaryEncoder.isEncoderButtonClicked())  /* exit calibration and save calibration data to EEPROM */
        {
          offSound();
          initMenuItems();  /* Init Menu Items */
          g_pref.putBytes("user_param", &g_storedVar, sizeof(g_storedVar)); /* Put the value of the stored user_param */
          g_pref.end();                                                     /* Close the namespace */
          HalfBridge_Enable();    /* Enable HalfBridge */
          g_currState = WELCOME;  /* Go to WELCOME state */
        }
        break;


      case WELCOME:
        /* This WELCOME state is only used to show the SW versions on the Screen, but the controller is actually fully operational:
            - Calibration is OKAY (either just done, or using stored values)
            - User param are OKAY (either just initialized to default, or using stored values)
            - Trigger is being read on Task 2

          The user trigger input is being elaborated and the correct speed (PWM) output is being produced for the whole duration of the WELCOME state */

        showScreenWelcome();    /* Show welcome screen */
        delay(1000);            /* Wait one second */
        g_currState = RUNNING;  /* Go to RUNNING state */
        break;


      case RUNNING: /* when the global variable State is in RUNNING the Task2 will elaborate the trigger to produce the PWM out */

        /* Change menu state if encoder button is clicked */
        if (g_rotaryEncoder.isEncoderButtonClicked()) 
        {
          menuState = rotary_onButtonClick(menuState);  /* This function is called if the encoder is pressed
                                                           Take the current menu state and returns the next menu state */
          g_lastEncoderInteraction = millis();          /* Update last encoder interaction */
        }

        /* Get encoder position if it was changed*/
        if (g_rotaryEncoder.encoderChanged()) 
        {
          g_escVar.encoderPos = g_rotaryEncoder.readEncoder();  /* Update the global ESC variable keeping track of the encoder value (position) */
          g_lastEncoderInteraction = millis();  /* Update last encoder interaction */
        }

        /* Change Encoder variable depending on menu state */
        if (menuState == ITEM_SELECTION) 
        {
          g_encoderMainSelector = g_escVar.encoderPos;  /* If in ITEM_SELECTION, update the encoder MainSelector with the encoder position */
        }
        if (menuState == VALUE_SELECTION) 
        {
          g_encoderSecondarySelector = g_escVar.encoderPos;         /* If in ITEM_SELECTION, update the encoder SecondaryEncoder with the encoder position */
          *g_encoderSelectedValuePtr = g_encoderSecondarySelector;  /* Also update the value of the selected parameter */
          //g_storedVar.carParam[g_carSel].throttleCurveVertex.curveSpeedDiff = saturateParamValue(g_storedVar.carParam[g_carSel].throttleCurveVertex.curveSpeedDiff, THROTTLE_CURVE_SPEED_DIFF_MIN_VALUE, THROTTLE_CURVE_SPEED_DIFF_MAX_VALUE); /* Regulate min value for throttle setpoint */
        }

        /* Show Main Menu display */
        printMainMenu(menuState);
        if (g_storedVar.carParam[g_carSel].freqPWM != prevFreqPWM)  /* if PWM freq parameter is changed, update motor PWM */
        {
          prevFreqPWM = g_storedVar.carParam[g_carSel].freqPWM;
          ledcDetach(HB_IN_PIN);
          ledcDetach(HB_INH_PIN);
          uint16_t freqTmp = g_storedVar.carParam[g_carSel].freqPWM * 100;
          ledcAttachChannel(HB_IN_PIN, freqTmp, THR_PWM_RES_BIT, THR_IN_PWM_CHAN);
          ledcAttachChannel(HB_INH_PIN, freqTmp, THR_PWM_RES_BIT, THR_INH_PWM_CHAN);
        }
        break;


      case FAULT:
        /* FAULT state, not currently used */
        break;


      default:
        /* Default case, do nothing */
        break;
    }

    if (g_currState != prevState) /* Every time FSM machine change state */
      obdFill(&g_obd, OBD_WHITE, 1);
  }
}

/**
 * Task 2 performs trigger reading, trigger conditioning and set output PWM
 */
void Task2code(void *pvParameters) {
  static uint16_t dragBrake_norm, positiveDerivative;                       /* Used to calculate drag brake                  */
  static unsigned long prevCallTime_uS = 0, deltaTime_uS, currCallTime_uS;  /* Used to keep track of time between executions */
  static unsigned long currTrigger_raw = 0, prevTrigger_raw = 0;            /* Used to keep track of current and previous trigger readings */
  
  HalfBridge_Enable();  /* TODO: verify if needed */

  for (;;) 
  {
    currCallTime_uS = micros();                         /* Get current time in uS */
    deltaTime_uS = currCallTime_uS - prevCallTime_uS;   /* Calculate delta time between current and previous execution */
    
    if (deltaTime_uS > ESC_PERIOD_US) /* This condition ensure that the following code is executed every ESC_PERIOD_US */
    {
      prevTrigger_raw = currTrigger_raw;
      currTrigger_raw = HAL_ReadTriggerRaw();  /* Read raw trigger value */
      g_escVar.trigger_raw = (prevTrigger_raw + currTrigger_raw) / 2;   /* Take the average between current and previous trigger readings --> attenuate disturbs */
      prevCallTime_uS = currCallTime_uS;                              /* update last call static memory */
      if (!(g_currState == CALIBRATION || g_currState == INIT))           /* Do not apply power if in calibration or before initialization (TODO: would be better to have also variables init) */
      {
        /* Throttle -> Speed pipeline , perform time dependent adjustment */
        g_escVar.trigger_norm = normalizeAndClamp(g_escVar.trigger_raw, g_storedVar.minTrigger_raw, g_storedVar.maxTrigger_raw, THROTTLE_NORMALIZED, THROTTLE_REV);  /* Get Raw trigger position and return throttle between 0 and THROTTLE_NORMALIZED */
        g_escVar.trigger_norm = addDeadBand(g_escVar.trigger_norm, 0, THROTTLE_NORMALIZED, THROTTLE_DEADBAND_NORM); /* Account for deadband */
        g_escVar.triggerDerivative = computeTriggerDerivativeGPT(g_escVar.trigger_norm);    /* Compute trigger derivative */
        g_escVar.outputSpeed_pct = throttleCurve(g_escVar.trigger_norm);                    /* Map trigger(throttle) to speed (duty) */
        g_escVar.outputSpeed_pct = throttleAntiSpin(g_escVar.outputSpeed_pct);              /* Define actual speed output (apply antispin) */

        if (g_escVar.outputSpeed_pct == 0)                                /* If the requested speed is 0 */
        {
          HalfBridge_SetPwmDrag(0, g_storedVar.carParam[g_carSel].brake);  /* Apply brake only (and speed to 0) in case speed set is 0 */
          dragBrake_norm = 0;                                           /* Set also dragBrake_norm to 0 just to look nicer on the display */
        }
        else                                                            /* If the requested speed is > 0 */
        {
          positiveDerivative = constrain(-g_escVar.triggerDerivative, 0, MAX_UINT16);   /* TODO: verify if the "-" before triggerDerivative is related to REV_THROTTLE */
          dragBrake_norm = normalizeAndClamp(positiveDerivative, 0, DERIVATIVE_MAX_ACTION_NORM, g_storedVar.carParam[g_carSel].dragBrake, 0);  /* Calculate dragBrake proportional to the derivative and the dragBrake parameter */
          HalfBridge_SetPwmDrag(g_escVar.outputSpeed_pct, dragBrake_norm);  /* Apply output speed (duty) and drag brake */
        }
      }
    }
  }
}


/* real loop are in the Tasks */
void loop() {}

/*********************************************************************************************************************/
/*---------------------------------------------Setup Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

/*
  Initialize OLED display and encoder.
  This is done separately from the HW init to shorten the startup time.
*/
void initDisplayAndEncoder() 
{
  uint16_t rc;
  /***** OLED Display Setup *****/
  rc = obdI2CInit(&g_obd, MY_OLED, OLED_ADDR, FLIP180, INVERT_DISP, USE_HW_I2C, SDA1_PIN, SCL1_PIN, RESET_PIN, 800000L);  // use standard I2C bus at 400Khz
  if (rc != OLED_NOT_FOUND) 
  {
    obdSetBackBuffer(&g_obd, ucBackBuffer);
    obdFill(&g_obd, OBD_WHITE, 1);
  } 
  else 
  {
    Serial.println("Error! Failed OLED Display initialization!");
  }

  /***** Encoder Setup *****/
  g_rotaryEncoder.begin();
  g_rotaryEncoder.setup(readEncoderISR);
  g_rotaryEncoder.setBoundaries(1, MENU_ITEMS_COUNT, false); /* minValue, maxValue, circleValues true|false (when max go to min and vice versa) */
  g_rotaryEncoder.setAcceleration(MENU_ACCELERATION);        /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
}


/* Rotary Encoder ISR */
void IRAM_ATTR readEncoderISR() {
  g_rotaryEncoder.readEncoder_ISR();
}


/**
 * Initialize the stored variables with default values.
   This is done for every CAR in the CarParam array.
 */
void initStoredVariables() {
  for (int i = 0; i < CAR_MAX_COUNT; i++) {
    g_storedVar.carParam[i].minSpeed = MIN_SPEED_DEFAULT;
    g_storedVar.carParam[i].brake = BRAKE_DEFAULT;
    g_storedVar.carParam[i].dragBrake = DRAG_BRAKE_DEFAULT;
    g_storedVar.carParam[i].maxSpeed = MAX_SPEED_DEFAULT;
    g_storedVar.carParam[i].throttleCurveVertex = { THROTTLE_CURVE_INPUT_THROTTLE_DEFAULT, THROTTLE_CURVE_SPEED_DIFF_DEFAULT };
    g_storedVar.carParam[i].antiSpin = ANTISPIN_DEFAULT;
    g_storedVar.carParam[i].freqPWM = PWM_FREQ_DEFAULT;
    g_storedVar.carParam[i].carNumber = i;
    sprintf(g_storedVar.carParam[i].carName, "CAR%1d", i);
  }
  g_storedVar.selectedCarNumber = 0;
  g_storedVar.minTrigger_raw = 0;
  g_storedVar.maxTrigger_raw = ACD_RESOLUTION_STEPS;
}


/**
 * Initialize the menu items with the value from the stored variables.
   Insert the items one by one in the main menu global instance.
   Then insert the items (cars) in the car menu.
   To add an item:
    - update the value of MENU_ITEMS_COUNT to match the amount of items in the menu.
    - update MAX_ITEMS so that it is greater or equal MENU_ITEMS_COUNT.
    - Add the item here, in the desired position (check the comment in slot_ESC.h MenuItem_type to know more).
 */
void initMenuItems() {
  int i = 0;
  /* Init menu items */
  sprintf(g_mainMenu.item[i].name, "SENSI");
  g_mainMenu.item[i].value = (void *)&g_storedVar.carParam[g_carSel].minSpeed;
  g_mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  g_mainMenu.item[i].unit = '%';
  g_mainMenu.item[i].maxValue = min(MIN_SPEED_MAX_VALUE, (int)g_storedVar.carParam[g_carSel].maxSpeed);
  g_mainMenu.item[i].minValue = 0;
  g_mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(g_mainMenu.item[++i].name, "BRAKE");
  g_mainMenu.item[i].value = (void *)&g_storedVar.carParam[g_carSel].brake;
  g_mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  g_mainMenu.item[i].unit = '%';
  g_mainMenu.item[i].maxValue = BRAKE_MAX_VALUE;
  g_mainMenu.item[i].minValue = 0;
  g_mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(g_mainMenu.item[++i].name, "ANTIS");
  g_mainMenu.item[i].value = (void *)&g_storedVar.carParam[g_carSel].antiSpin;
  g_mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  g_mainMenu.item[i].unit = 'm';
  g_mainMenu.item[i].maxValue = ANTISPIN_MAX_VALUE;
  g_mainMenu.item[i].minValue = 0;
  g_mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(g_mainMenu.item[++i].name, "CURVE");
  g_mainMenu.item[i].value = (void *)&g_storedVar.carParam[g_carSel].throttleCurveVertex.curveSpeedDiff;
  g_mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  g_mainMenu.item[i].unit = '%';
  g_mainMenu.item[i].maxValue = THROTTLE_CURVE_SPEED_DIFF_MAX_VALUE;
  g_mainMenu.item[i].minValue = THROTTLE_CURVE_SPEED_DIFF_MIN_VALUE;
  g_mainMenu.item[i].callback = &showCurveSelection;

  sprintf(g_mainMenu.item[++i].name, "DRAGB");
  g_mainMenu.item[i].value = (void *)&g_storedVar.carParam[g_carSel].dragBrake;
  g_mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  g_mainMenu.item[i].unit = '%';
  g_mainMenu.item[i].maxValue = DRAG_MAX_VALUE;
  g_mainMenu.item[i].minValue = 0;
  g_mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(g_mainMenu.item[++i].name, "PWM_F");
  g_mainMenu.item[i].value = (void *)&g_storedVar.carParam[g_carSel].freqPWM;
  g_mainMenu.item[i].type = VALUE_TYPE_DECIMAL;
  g_mainMenu.item[i].unit = 'k';
  g_mainMenu.item[i].maxValue = FREQ_MAX_VALUE / 100;
  g_mainMenu.item[i].minValue = FREQ_MIN_VALUE / 100;
  g_mainMenu.item[i].decimalPoint = 1;
  g_mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(g_mainMenu.item[++i].name, "LIMIT");
  g_mainMenu.item[i].value = (void *)&g_storedVar.carParam[g_carSel].maxSpeed;
  g_mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  g_mainMenu.item[i].unit = '%';
  g_mainMenu.item[i].maxValue = MAX_SPEED_DEFAULT;
  g_mainMenu.item[i].minValue = max(5, (int)g_storedVar.carParam[g_carSel].minSpeed + 5);
  g_mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(g_mainMenu.item[++i].name, "*CAR*");
  g_mainMenu.item[i].value = (void *)&g_storedVar.carParam[g_carSel].carName;
  g_mainMenu.item[i].type = VALUE_TYPE_STRING;
  g_mainMenu.item[i].maxValue = CAR_MAX_COUNT - 1;  // so menu will scroll in the array (CAR_MAX_COUNT long)
  g_mainMenu.item[i].minValue = 0;
  g_mainMenu.item[i].callback = &showSelectRenameCar;

  /* Init Car selection menu items */
  for (uint8_t j = 0; j < CAR_MAX_COUNT; j++)
  {
    sprintf(g_carMenu.item[j].name, (const char *)g_storedVar.carParam[j].carName);
    g_carMenu.item[j].value = (void *)&g_storedVar.carParam[j].carNumber;
  }
}


/**
 * Show the Welcome screen
 */
void showScreenWelcome() 
{
  obdWriteString(&g_obd, 0, 16, 12, (char *)"ESPEED32", FONT_12x16, OBD_BLACK, 1);
  sprintf(msgStr, "V%d.%02d", SW_MAJOR_VERSION, SW_MINOR_VERSION);  
  obdWriteString(&g_obd, 0, 34, 28, msgStr, FONT_12x16, OBD_WHITE, 1);
}


/**
 * Show the Pre-Calibration screen, when the button is pressed during startup, but not yet released
 */
void showScreenPreCalibration() 
{
  sprintf(msgStr, "ESPEED32 v%d.%02d", SW_MAJOR_VERSION, SW_MINOR_VERSION);  //print SW version
  obdWriteString(&g_obd, 0, 8, 0, msgStr, FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, (OLED_WIDTH / 2) - 48, 1 * HEIGHT8x8, (char *)"Release button", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, (OLED_WIDTH / 2) - 48, 2 * HEIGHT8x8, (char *)"to calibrate", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, (OLED_WIDTH / 2) - 48, 3 * HEIGHT8x8, (char *)"     OR    ", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, (OLED_WIDTH / 2) - 48, 4 * HEIGHT8x8, (char *)"remove power", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, (OLED_WIDTH / 2) - 48, 5 * HEIGHT8x8, (char *)"  to exit  ", FONT_8x8, OBD_BLACK, 1);
}


/**
 * Show the screen indicating that the stored variables are not present in the EEPROM
 */
void showScreenNoEEPROM() 
{
  sprintf(msgStr, "ESPEED32 v%d.%02d", SW_MAJOR_VERSION, SW_MINOR_VERSION);  //print SW version
  obdWriteString(&g_obd, 0, 0, 0, msgStr, FONT_8x8, OBD_WHITE, 1);
  obdWriteString(&g_obd, 0, (OLED_WIDTH / 2) - 64, 3 * HEIGHT8x8, (char *)"EEPROM NOT init!", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, (OLED_WIDTH / 2) - 48, 5 * HEIGHT8x8, (char *)"Press button", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, (OLED_WIDTH / 2) - 48, 6 * HEIGHT8x8, (char *)"to calibrate", FONT_8x8, OBD_BLACK, 1);
}


/**
 * Show the calibration screen. It displays the current trigger raw value, as well as the max and min encountered values.
 * 
 * @param adcRaw The raw trigger value read from the ADC
 */
void showScreenCalibration(int16_t adcRaw)
{
  sprintf(msgStr, "CALIBRATION");
  obdWriteString(&g_obd, 0, (OLED_WIDTH - 66) / 2, 0, msgStr, FONT_6x8, OBD_WHITE, 1);

  sprintf(msgStr, "press/releas throttle");
  obdWriteString(&g_obd, 0, 0, 8, msgStr, FONT_6x8, OBD_BLACK, 1);

  sprintf(msgStr, "Raw throttle %4d  ", adcRaw);
  obdWriteString(&g_obd, 0, 0, 24, msgStr, FONT_6x8, OBD_BLACK, 1);

  sprintf(msgStr, "Min throttle %4d   ", g_storedVar.minTrigger_raw);
  obdWriteString(&g_obd, 0, 0, 32, msgStr, FONT_6x8, OBD_BLACK, 1);

  sprintf(msgStr, "Max throttle %4d   ", g_storedVar.maxTrigger_raw);
  obdWriteString(&g_obd, 0, 0, 40, msgStr, FONT_6x8, OBD_BLACK, 1);

  sprintf(msgStr, " push when done ");
  obdWriteString(&g_obd, 0, 0, 56, msgStr, FONT_6x8, OBD_BLACK, 1);
}


/**
 * Print the main menu.
 * Takes care of printing the menu items and to scroll them according to the encoder position.
 * The current menu state is used to highlight decide whether to highlight only the item or also the value.
 * 
 * @param currMenuState The current menu state (ITEM_SELECTION or VALUE_SELECTION)
 */
void printMainMenu(MenuState_enum currMenuState) 
{
  static uint16_t tmp = 0;

  /* "Frame" indicates which items are currently displayed.
     It consist of a lower and upper bound: only the items within this boundaries are displayed.
     The difference between upper and lower bound is fixed to be g_mainMenu.lines
     It's important that the encoder boundaries matches the menu items (e.g., 8 items, encoder boundaries must be [1,8]) */
  static uint16_t frameUpper = 1;
  static uint16_t frameLower = g_mainMenu.lines;

  /* In encoder move out of frame, adjust frame */
  if (g_encoderMainSelector > frameLower) 
  {
    frameLower = g_encoderMainSelector;
    frameUpper = frameLower - g_mainMenu.lines + 1;
    obdFill(&g_obd, OBD_WHITE, 1);
  } 
  else if (g_encoderMainSelector < frameUpper) 
  {
    frameUpper = g_encoderMainSelector;
    frameLower = frameUpper + g_mainMenu.lines - 1;
    obdFill(&g_obd, OBD_WHITE, 1);
  }

  /* Print main menu only if there was an encoder interaction in the last 100 milliseconds */
  if (millis() - g_lastEncoderInteraction < 100) 
  {
    for (uint8_t i = 0; i < g_mainMenu.lines; i++)
    {
      /* Print item name */
      /* Item color: WHITE if item is selected, black otherwise */
      obdWriteString(&g_obd, 0, 0, i * HEIGHT12x16, g_mainMenu.item[frameUpper - 1 + i].name, FONT_12x16, (g_encoderMainSelector - frameUpper == i) ? OBD_WHITE : OBD_BLACK, 1);

      /* Only print value if value != ITEM_NO_VALUE */
      /* Value color: WHITE if corresponding item is selected AND menu state is VALUE_SELECTION, black otherwise */
      if (g_mainMenu.item[frameUpper - 1 + i].value != ITEM_NO_VALUE) 
      {
        /* if the value is a number, cast to *(unit16_t *), then print number and unit */
        if (g_mainMenu.item[frameUpper - 1 + i].type == VALUE_TYPE_INTEGER) 
        {
          /* value is a generic pointer to void, so first cast to uint16_t pointer, then take the pointed value */
          sprintf(msgStr, "%4d%c", *(uint16_t *)(g_mainMenu.item[frameUpper - 1 + i].value), g_mainMenu.item[frameUpper - 1 + i].unit);
          obdWriteString(&g_obd, 0, OLED_WIDTH - 60, i * HEIGHT12x16, msgStr, FONT_12x16, (((g_encoderMainSelector - frameUpper == i) && (currMenuState == VALUE_SELECTION)) ? OBD_WHITE : OBD_BLACK), 1);
        }
        /* If the value is a decimal, cast to *(unit16_t *), divide by 10^decimalPoint then print number and unit */
        else if (g_mainMenu.item[frameUpper - 1 + i].type == VALUE_TYPE_DECIMAL) 
        {
          /* value is a generic pointer to void, so first cast to uint16_t pointer, then take the pointed value */
          tmp = *(uint16_t *)(g_mainMenu.item[frameUpper - 1 + i].value);
          sprintf(msgStr, " %d.%01d%c", tmp / 10, (tmp % 10), g_mainMenu.item[frameUpper - 1 + i].unit);
          obdWriteString(&g_obd, 0, OLED_WIDTH - 60, i * HEIGHT12x16, msgStr, FONT_12x16, (((g_encoderMainSelector - frameUpper == i) && (currMenuState == VALUE_SELECTION)) ? OBD_WHITE : OBD_BLACK), 1);
        }
        /* If the value is a string, cast to (char *) then print the string */
        else if (g_mainMenu.item[frameUpper - 1 + i].type == VALUE_TYPE_STRING) 
        {
          /* value is a generic pointer to void, so cast to string pointer */
          sprintf(msgStr, "%s", (char *)(g_mainMenu.item[frameUpper - 1 + i].value));
          obdWriteString(&g_obd, 0, OLED_WIDTH - (4 * WIDTH12x16), i * HEIGHT12x16, msgStr, FONT_12x16, (((g_encoderMainSelector - frameUpper == i) && (currMenuState == VALUE_SELECTION)) ? OBD_WHITE : OBD_BLACK), 1);
        }
      }
    }

    initMenuItems();  /* update menu items with the storedVar that could have been changed in previous for cycle */
    /* Print LIMITER warning if LIMIT is any value other than 100% */
    if (g_storedVar.carParam[g_carSel].maxSpeed < MAX_SPEED_DEFAULT) 
    {
      obdWriteString(&g_obd, 0, WIDTH8x8, 3 * HEIGHT12x16, (char *)" - LIMITER - ", FONT_8x8, OBD_WHITE, 1);
    } 
    else 
    {
      obdWriteString(&g_obd, 0, WIDTH8x8, 3 * HEIGHT12x16, (char *)"             ", FONT_8x8, OBD_BLACK, 1);
    }
  }

  /* print analytic - statistic line */
  /* Current Output speed */
  sprintf(msgStr, "%3d%c", g_escVar.outputSpeed_pct, '%');  
  obdWriteString(&g_obd, 0, 0, 3 * HEIGHT12x16 + HEIGHT8x8, msgStr, FONT_8x8, (g_escVar.outputSpeed_pct == 100) ? OBD_WHITE : OBD_BLACK, 1);
  /* Trigger derivative */
  sprintf(msgStr, "%3d ", g_escVar.triggerDerivative);
  obdWriteString(&g_obd, 0, 4 * WIDTH8x8, 3 * HEIGHT12x16 + HEIGHT8x8, msgStr, FONT_6x8, OBD_BLACK, 1);
    /* Current voltaqge */
  sprintf(msgStr, " %d.%01dV ", g_escVar.Vin_mV / 1000, (g_escVar.Vin_mV % 1000) / 100);
  obdWriteString(&g_obd, 0, 7 * WIDTH8x8, 3 * HEIGHT12x16 + HEIGHT8x8, msgStr, FONT_6x8, OBD_BLACK, 1);

  /* Debug line, unused
  sprintf(msgStr, " %d   ", debug);
  obdWriteString(&g_obd, 0, 12 * WIDTH8x8, 3 * HEIGHT12x16 + HEIGHT8x8, msgStr, FONT_6x8, OBD_BLACK, 1);
  */
}


/**
 * Apply antispin calculation (according to antispin settings) to a requested output speed.
 * 
 * @param requestedSpeed [%] The requested outputSpeed at the end of the throttle -> speed pipeline
 * @return [%] The output speed closer to the requestedSpeed that respect the Antispin settings.
 */
uint16_t throttleAntiSpin(uint16_t requestedSpeed) 
{
  static uint32_t lastOutputSpeedx1000 = 0, maxDeltaSpeedx1000, outputSpeedX1000;   /* To keep track of output speeds, in a larger scale to increase granularity */
  static unsigned long prevCall_uS = 0, deltaTime_uS, currCall_uS;    /* To keep track of time between executions */
  uint32_t outputSpeed;     

  currCall_uS = micros();                     /* Get current time in uS */
  deltaTime_uS = currCall_uS - prevCall_uS;   /* Get delta time from last call of this function */
  prevCall_uS = currCall_uS;                  /* Update last call static memory */

  /* Bypass calculation if antiSpin is 0 (OFF) and just return requestedSpeed */
  if ((g_storedVar.carParam[g_carSel].antiSpin) == 0) 
  {
    outputSpeed = requestedSpeed;
  } 
  else 
  {
    maxDeltaSpeedx1000 = ((g_storedVar.carParam[g_carSel].maxSpeed - g_storedVar.carParam[g_carSel].minSpeed) * (deltaTime_uS)) / (g_storedVar.carParam[g_carSel].antiSpin);  /* deltaSpeed = ((minSPeed-MaxSpeed)* DeltaTime) / antiSpin (from min speed to max speed) */

    if ((uint32_t)requestedSpeed * 1000 <= lastOutputSpeedx1000)  /* If current requestSpeed is less than previous output speed (car braking/slowing) do it immediately */
    {
      outputSpeed = requestedSpeed;
      lastOutputSpeedx1000 = outputSpeed * 1000;
    } 
    else /* Requested speed is increasing (Car is accelerating) */
    {
      if (lastOutputSpeedx1000 < ((uint32_t)requestedSpeed * 1000 - maxDeltaSpeedx1000))  /* Check there is room to increase speed by a maxdeltaspeed */
      {
        outputSpeedX1000 = (lastOutputSpeedx1000 + maxDeltaSpeedx1000);
      } 
      else 
      {
        outputSpeedX1000 = requestedSpeed * 1000;
      }

      if (outputSpeedX1000 < g_storedVar.carParam[g_carSel].minSpeed * 1000)  /* check in order to start the ramp from minspeed (and not from 0) */
      {
        outputSpeedX1000 = g_storedVar.carParam[g_carSel].minSpeed * 1000;
      }

      lastOutputSpeedx1000 = outputSpeedX1000;  /* save latest outspeed, so next iteration of this function can have only the defined delta */
      outputSpeed = outputSpeedX1000 / 1000;
    }
  }

  return outputSpeed;
}

/**
 * Accounts for a deadband in an input value.
 * 
 * @param inputVal The input value
 * @param minVal The lower bound of the input value range
 * @param maxVal The upper bound of the input value range
 * @param deadBand The deadband as absolute value (not a percentage)
 * @return If the input value is less than the minVal + deadBand, it returns minVal.
           If the input value is more than the maxVal - deadBand, it returns the maxVal.
           Otherwise, it returns the inputVal, scaled in order to range from minVal to maxVal.
 */
uint16_t addDeadBand(uint16_t inputVal, uint16_t minVal, uint16_t maxVal, uint16_t deadBand) 
{
  uint16_t retVal = 0;

  /* If the inputVal is less than deadBand_pct, return 0 */
  if (inputVal < minVal + deadBand) 
  {
    retVal = 0;
  } 
  /* If the inputVal is more than maxValue - deadBand, return maxValue */
  else if (inputVal > maxVal - deadBand) 
  {
    retVal = maxVal;
  } 
  else 
  {
    /* retVal = (THROTTLE_NORMALIZED * (inputVal - deadBand)) / (THROTTLE_NORMALIZED - 2 * deadBand); TODO: verify what this is suppose to do */
    retVal = map(inputVal, deadBand, maxVal - deadBand, minVal, maxVal);  /* Scale the inputValue (which ranges from (minVal + deadBand) to (maxVal - deadBand))
                                                                             so that it ranges from minVal to maxVal */
  }

  return retVal;
}


/**
 * Apply the throttle curve (defined by the curve vertex) to an input throttle.
 * 
 * @param inputThrottle The input throttle, normalized (ranges from 0 to THROTTLE_NORMALIZED)
 * @return The corresponding output speed [%] (duty cycle) according to the throttle curve.
 */
uint16_t throttleCurve(uint16_t inputThrottle)
{
  uint16_t outputSpeed = 0;           /* The requested output speed (duty cycle) from 0% to 100% */
  uint32_t throttleCurveVertexSpeed;  /* The output speed when the throttle is at 50% (that is, the value of throttleCurveVertex.inputThrottle) */

  /* Calculate the output speed of the throttle curve vertex
     This is calculated as the curveSpeedDiff (from 10% to 90%) percentage of the difference between minSpeed and maxSpeed */
  throttleCurveVertexSpeed = g_storedVar.carParam[g_carSel].minSpeed + (((uint32_t)g_storedVar.carParam[g_carSel].maxSpeed - (uint32_t)g_storedVar.carParam[g_carSel].minSpeed) * ((uint32_t)g_storedVar.carParam[g_carSel].throttleCurveVertex.curveSpeedDiff) / 100);

  if (inputThrottle == 0)   /* If input throttle is 0 --> output speed is 0% */
  {
    outputSpeed = 0;
  } 
  else if (inputThrottle <= g_storedVar.carParam[g_carSel].throttleCurveVertex.inputThrottle) /* If the input throttle is less than the vertex point (fixed at 50%), than map the output speed from 0 to the throttleCurveVertexSpeed */
  {
    outputSpeed = map(inputThrottle, 0, g_storedVar.carParam[g_carSel].throttleCurveVertex.inputThrottle, g_storedVar.carParam[g_carSel].minSpeed, throttleCurveVertexSpeed);
  } 
  else  /* If the input throttle is more than the vertex point (fixed at 50%), than map the output speed from throttleCurveVertexSpeed to the maxSpeed*/
  {
    outputSpeed = map(inputThrottle, g_storedVar.carParam[g_carSel].throttleCurveVertex.inputThrottle, THROTTLE_NORMALIZED, throttleCurveVertexSpeed, g_storedVar.carParam[g_carSel].maxSpeed);
  }

  return outputSpeed;
}

/**
 * Call this when calibrating the throttle.
 * Check if the parameter adcRaw is bigger/smaller than the stored max/min values, and updates them accordingly.
 * @param inputThrottle The raw ADC value
 */
void throttleCalibration(int16_t adcRaw)
{
  if (g_storedVar.maxTrigger_raw < adcRaw)
    {
      g_storedVar.maxTrigger_raw = adcRaw;
    }
  if (g_storedVar.minTrigger_raw > adcRaw)
    {
      g_storedVar.minTrigger_raw = adcRaw;
    }
}


/**
 * Reacts to the rotary encoder being clicked.
 * If the menu is in ITEM_SELECTION, it must select the correct item (update the g_encoderSelectedValuePtr) or invoking the item's callback --> then return the next state VALUE_SELECTION
 * If the menu is in VALUE_SELECTION, it must return to ITEM_SELECTION
 * 
 * @param currMenuState The current menu state.
 * @return The next menu state.
 */
MenuState_enum rotary_onButtonClick(MenuState_enum currMenuState) 
{
  static unsigned long lastTimePressed = 0;
  static uint16_t selectedParamMaxValue = 100;
  static uint16_t selectedParamMinValue = 0;
  /* ignore multiple press in that are less than 200ms apart */
  if (millis() - lastTimePressed < 200)
  {
    return currMenuState;
  }
  
  lastTimePressed = millis();

  if (currMenuState == ITEM_SELECTION) /* If the current state is ITEM_SELECTION */
  {
    /* If an item has no callback, go in menu state VALUE_SELECTION */
    if (g_mainMenu.item[g_encoderMainSelector - 1].callback == ITEM_NO_CALLBACK) 
    {
      g_rotaryEncoder.setAcceleration(SEL_ACCELERATION); /* Set higher encoder acceleration so it doesn't require too many turns to make a big value change */
      
      
      g_encoderSelectedValuePtr = (uint16_t *)g_mainMenu.item[g_encoderMainSelector - 1].value;   /* Update the g_encoderSelectedValuePtr to point to the value of the selected item.
                                                                                                     value is a generic pointer to void, so cast to uint16_t pointer */
      selectedParamMaxValue = g_mainMenu.item[g_encoderMainSelector - 1].maxValue;                /* Set Max and Min boundaries according to the selected items max and min value */
      selectedParamMinValue = g_mainMenu.item[g_encoderMainSelector - 1].minValue;
      g_rotaryEncoder.setBoundaries(selectedParamMinValue, selectedParamMaxValue, false);
      g_rotaryEncoder.reset(*g_encoderSelectedValuePtr);  /* Reset the encoder to the current value of the selected item */
      g_escVar.encoderPos = *g_encoderSelectedValuePtr;   /* Set the encoderPos global variable to the current value of the selected item */
      return VALUE_SELECTION;                             /* Return the VALUE_SELECTION state */                                  
    }
    /* if an item has a callback, execute it, then return to ITEM SELECTION */
    else 
    {
      g_mainMenu.item[g_encoderMainSelector - 1].callback();  /* Invoke the selected item's callback */
      return ITEM_SELECTION;
    }
  } 
  else /* If the current state is VALUE_SELECTION */
  {
    g_rotaryEncoder.setAcceleration(MENU_ACCELERATION);         /* Set the encoder acceleration to MENU_ACCELERATION */
    g_rotaryEncoder.setBoundaries(1, MENU_ITEMS_COUNT, false);  /* Set the encoder boundaries to the menu boundaries */
    g_rotaryEncoder.reset(g_encoderMainSelector);               /* Reset the encoder value to g_encoderMainSelector, so that it doesn't change the selected item */
    g_escVar.encoderPos = g_encoderMainSelector;
    
    saveEEPROM(g_storedVar);  /* Save modified values to EEPROM */
    return ITEM_SELECTION;    /* Return the ITEM_SELECTION state */   
  }
}


/**
 * Show the Car selection menu.
 * Uses a "Frame" just like the main menu to display and scroll through the carMenu items.
 * Scrolling the Cars is actually changing the value of selectedCarNumber.
 */
void showCarSelection() {
  /* "Frame" indicates which items are currently displayed.
     It consist of a lower and upper bound: only the items within this boundaries are displayed.
     The difference between upper and lower bound is fixed to be g_carMenu.lines
     It's important that the encoder boundaries matches the menu items (e.g., 10 items (cars), encoder boundaries must be [0,9]) */
  static uint16_t frameUpper = 1;
  static uint16_t frameLower = g_carMenu.lines;

  /* Clear screen */
  obdFill(&g_obd, OBD_WHITE, 1);

  /* Set encoder to car selection parameter */
  g_rotaryEncoder.setAcceleration(MENU_ACCELERATION);
  g_rotaryEncoder.setBoundaries(0, CAR_MAX_COUNT - 1, false);
  g_rotaryEncoder.reset(g_storedVar.selectedCarNumber);

  /* Exit car selection when encoder is clicked */
  while (!g_rotaryEncoder.isEncoderButtonClicked()) 
  {
    /* Get encoder value if changed */
    g_storedVar.selectedCarNumber = g_rotaryEncoder.encoderChanged() ? g_rotaryEncoder.readEncoder() : g_storedVar.selectedCarNumber;
    
    /* If encoder move out of frame, adjust frame */
    if (g_storedVar.selectedCarNumber > frameLower) 
    {
      frameLower = g_storedVar.selectedCarNumber;
      frameUpper = frameLower - g_carMenu.lines + 1;
      obdFill(&g_obd, OBD_WHITE, 1);
    } 
    else if (g_storedVar.selectedCarNumber < frameUpper) 
    {
      frameUpper = g_storedVar.selectedCarNumber;
      frameLower = frameUpper + g_carMenu.lines - 1;
      obdFill(&g_obd, OBD_WHITE, 1);
    }

    /* Print car menu */
    for (uint8_t i = 0; i < g_carMenu.lines; i++) 
    {
      /* Print the item (car) name */
      obdWriteString(&g_obd, 0, 0, i * HEIGHT12x16, g_carMenu.item[frameUpper + i].name, FONT_12x16, (g_storedVar.selectedCarNumber - frameUpper == i) ? OBD_WHITE : OBD_BLACK, 1);
      if (g_carMenu.item[frameUpper + i].value != ITEM_NO_VALUE) 
      {
        /* value is a generic pointer to void, so first cast to uint16_t pointer, then take the pointed value */
        sprintf(msgStr, "%2d", *(uint16_t *)(g_carMenu.item[frameUpper + i].value));
        /* Print the item value (car number) */
        obdWriteString(&g_obd, 0, OLED_WIDTH - 24, i * HEIGHT12x16, msgStr, FONT_12x16, OBD_BLACK, 1);
      }
    }

    /* Print "-SELECT THE CAR-" on the bottom of the screen */
    obdWriteString(&g_obd, 0, 16, OLED_HEIGHT - HEIGHT8x8, (char *)"-SELECT THE CAR-", FONT_6x8, OBD_WHITE, 1);
  }

  /* Reset encoder and clear is done in caller routine */
  return;
}


/* Deprecated --> now save is done automatically upon parameter change 
void showSaveCar() {
  saveEEPROM(g_storedVar);
  
  g_rotaryEncoder.setAcceleration(MENU_ACCELERATION);
  g_rotaryEncoder.setBoundaries(1, MENU_ITEMS_COUNT, false);
  g_rotaryEncoder.reset(g_encoderMainSelector);
  g_escVar.encoderPos = g_encoderMainSelector;
  
  obdFill(&g_obd, OBD_WHITE, 1);
  return;
}
*/


/**
 * Show the options to Select or Rename the Car, called upon selecting the CAR item in the main menu
 */
void showSelectRenameCar() {

  /* Trigger reading stops, so stop the motor */
  /* Set trigRaw to max throttle if throttle is reversed, set to min throttle otherwise */
  /* g_escVar.trigger_raw = THROTTLE_REV ? g_storedVar.maxTrigger_raw : g_storedVar.minTrigger_raw; */ /*TODO: decide what to do now that trigger reading is done on task 2 */

  uint16_t selectedOption = 0;
  /* Clear screen */
  obdFill(&g_obd, OBD_WHITE, 1);

  /* Set encoder to selection parameter */
  g_rotaryEncoder.setAcceleration(MENU_ACCELERATION);
  g_rotaryEncoder.setBoundaries(0, 1, false); /* Boundaries are [0, 1] because there are only two options */
  g_rotaryEncoder.reset(selectedOption);

  /* Print the "SELECT AN OPTION" */
  obdWriteString(&g_obd, 0, 16, OLED_HEIGHT - HEIGHT8x8, (char *)"-PICK AN OPTION-", FONT_6x8, OBD_WHITE, 1);

  /* Exit car selection when encoder is clicked */
  while (!g_rotaryEncoder.isEncoderButtonClicked())
  {
    /* Get encoder value if changed */
    selectedOption = g_rotaryEncoder.encoderChanged() ? g_rotaryEncoder.readEncoder() : selectedOption;
    /* Print the two options */
    obdWriteString(&g_obd, 0, 0, 0 * HEIGHT12x16, (char *)"SELECT", FONT_12x16, (selectedOption == CAR_OPTION_SELECT) ? OBD_WHITE : OBD_BLACK, 1);
    obdWriteString(&g_obd, 0, 0, 1 * HEIGHT12x16, (char *)"RENAME", FONT_12x16, (selectedOption == CAR_OPTION_RENAME) ? OBD_WHITE : OBD_BLACK, 1);
  }

  /* If RENAME option was selected, go to renameCar routine */
  if (selectedOption == CAR_OPTION_RENAME) 
  {
    showRenameCar();
    saveEEPROM(g_storedVar);
  }
  /* If SELECT option was selected, go to showCarSelection routine */
  else if (selectedOption == CAR_OPTION_SELECT) 
  {
    showCarSelection();
    saveEEPROM(g_storedVar);
  }

  /* Reset encoder */
  g_rotaryEncoder.setAcceleration(MENU_ACCELERATION);
  g_rotaryEncoder.setBoundaries(1, MENU_ITEMS_COUNT, false);
  g_rotaryEncoder.reset(g_encoderMainSelector);
  g_escVar.encoderPos = g_encoderMainSelector;
  /* Clear screen */
  obdFill(&g_obd, OBD_WHITE, 1);

  return;
}


/**
 * Show the Rename car screen. Called by selecting RENAME option on the showSelectRenameCar screen.
 */
void showRenameCar() {

  uint16_t selectedChar = RENAME_CAR_MIN_ASCII; /* the selected char: starts from 33 (ASCII for !) to 122 (ASCII for z)*/
  uint16_t selectedOption = 0;                  /* The selected option, could be one of the char of the name (0 : CAR_NAME_MAX_SIZE - 2) or the confirm option (CAR_NAME_MAX_SIZE - 1)*/
                                                /* Remember that CAR_NAME_MAX_SIZE includes the last terminator char */
  char tmpName[CAR_NAME_MAX_SIZE];  /* Array of chars containing the temporary name */
  uint16_t mode = RENAME_CAR_SELECT_OPTION_MODE;  /* Initial mode. There are two mode:
                                                     - RENAME_CAR_SELECT_OPTION_MODE, when scrolling the encoder changes the selectedOption (pick which char to change, or the OK option)
                                                     - RENAME_CAR_SELECT_CHAR_MODE, when scrolling the encoder changes the selectedChar, that is the value of the selectedOption */
  sprintf(tmpName, "%s", g_storedVar.carParam[g_storedVar.selectedCarNumber].carName); /* Store the current carName in the temporary name */

  /* Clear screen */
  obdFill(&g_obd, OBD_WHITE, 1);

  /* Set encoder to selection parameter */
  g_rotaryEncoder.setAcceleration(MENU_ACCELERATION);
  g_rotaryEncoder.setBoundaries(0, CAR_NAME_MAX_SIZE - 1, false);
  g_rotaryEncoder.reset(selectedOption);

  /* Print "-RENAME THE CAR-"  and "-CLICK OK TO CONFIRM" */
  obdWriteString(&g_obd, 0, 16, 0, (char *)"-RENAME THE CAR-", FONT_6x8, OBD_WHITE, 1);
  obdWriteString(&g_obd, 0, 1, OLED_HEIGHT - HEIGHT8x8, (char *)"-CLICK OK TO CONFIRM-", FONT_6x8, OBD_WHITE, 1);

  /* Draw the right arrow */
  for (uint8_t j = 0; j < 8; j++) 
  {
    obdDrawLine(&g_obd, 80 + j, 16 + j, 80 + j, 30 - j, OBD_BLACK, 1);
  }
  obdDrawLine(&g_obd, 72, 22, 80, 22, OBD_BLACK, 1);
  obdDrawLine(&g_obd, 72, 23, 80, 23, OBD_BLACK, 1);
  obdDrawLine(&g_obd, 72, 24, 80, 24, OBD_BLACK, 1);

  /* Exit car renaming when encoder is clicked AND CONFIRM is selected */
  while (1) 
  {
    /* Get encoder value if changed */
    /* Change selectedOption if in RENAME_CAR_SELECT_OPTION_MODE */
    if (mode == RENAME_CAR_SELECT_OPTION_MODE) 
    {
      selectedOption = g_rotaryEncoder.encoderChanged() ? g_rotaryEncoder.readEncoder() : selectedOption;
    }
    /* Change selectedChar if in RENAME_CAR_SELECT_CHAR_MODE */
    if (mode == RENAME_CAR_SELECT_CHAR_MODE) 
    {
      selectedChar = g_rotaryEncoder.encoderChanged() ? g_rotaryEncoder.readEncoder() : selectedChar;
      tmpName[selectedOption] = (char)selectedChar; /* Change the value of the selected char in the temp name */

      /* Draw the upward and downward arrows on the selected char to indicate that it can be changed */
      for (uint8_t j = 0; j < 6; j++) 
      {
        obdDrawLine(&g_obd, 1 + j + (selectedOption * 12), 14 - j, 11 - j + (selectedOption * 12), 14 - j, OBD_BLACK, 1);
        obdDrawLine(&g_obd, 1 + j + (selectedOption * 12), 33 + j, 11 - j + (selectedOption * 12), 33 + j, OBD_BLACK, 1);
      }
    }

    /* Print each Char of the name, only the selected one is highlighted (WHITE color) */
    for (uint8_t i = 0; i < CAR_NAME_MAX_SIZE - 1; i++) 
    {
      sprintf(msgStr, "%c", tmpName[i]);
      obdWriteString(&g_obd, 0, 0 + (i * 12), 22, msgStr, FONT_12x16, (selectedOption == i) ? OBD_WHITE : OBD_BLACK, 1);
    }

    /* Print the confirm button */
    obdWriteString(&g_obd, 0, OLED_WIDTH - 24, 22, (char *)"OK", FONT_12x16, (selectedOption == CAR_NAME_MAX_SIZE - 1) ? OBD_WHITE : OBD_BLACK, 1);

    /* If encoder button is clicked */
    if (g_rotaryEncoder.isEncoderButtonClicked()) 
    {
      /* Exit renameCar routing when CONFIRM is selected */
      if (selectedOption == CAR_NAME_MAX_SIZE - 1) 
      {
        /* Change the name of the Car */
        sprintf(g_storedVar.carParam[g_storedVar.selectedCarNumber].carName, "%s", tmpName);
        /* Menu variables are initialized in main loop */
        return;
      }

      /* If in RENAME_CAR_SELECT_OPTION_MODE */
      if (mode == RENAME_CAR_SELECT_OPTION_MODE) {
        /* switch mode */
        mode = RENAME_CAR_SELECT_CHAR_MODE;
        /* Reset encode */
        g_rotaryEncoder.setBoundaries(RENAME_CAR_MIN_ASCII, RENAME_CAR_MAX_ASCII, false);
        g_rotaryEncoder.reset((uint16_t)tmpName[selectedOption]);
        selectedChar = (uint16_t)tmpName[selectedOption];
      }
      /* If in RENAME_CAR_SELECT_CHAR_MODE */
      else {
        /* switch mode */
        mode = RENAME_CAR_SELECT_OPTION_MODE;
        /* Reset encode */
        g_rotaryEncoder.setBoundaries(0, CAR_NAME_MAX_SIZE - 1, false);
        g_rotaryEncoder.reset(selectedOption);
        /* Cancel the upward and downward arrows (draw them black) */
        for (uint8_t j = 0; j < 6; j++) {
          obdDrawLine(&g_obd, 1 + j + (selectedOption * 12), 14 - j, 11 - j + (selectedOption * 12), 14 - j, OBD_WHITE, 1);
          obdDrawLine(&g_obd, 1 + j + (selectedOption * 12), 33 + j, 11 - j + (selectedOption * 12), 33 + j, OBD_WHITE, 1);
        }
      }
    }
  }
}


/**
 * Show the Throttle Curve selection screen. Shwon when the CURVE item is selected.
 * Draws the throttle curve graph as two segmented lines, that changes when the encoder is rotated (that is, the CURVE parameter is being changed).
 * The calculation for drawing the throttle curve are the same as the ones done in the throttleCurve function.
 * The CURVE parameter and the current trigger value are also displayed.
 */
void showCurveSelection()
{
  uint16_t throttleCurveVertexSpeed;
  uint16_t prevTrigger = g_escVar.outputSpeed_pct;
  uint16_t inputThrottle = (g_storedVar.carParam[g_carSel].throttleCurveVertex.inputThrottle * 100) / THROTTLE_NORMALIZED;  //Take inputThrottle (from 0 to THROTTLE NORMALIZED) and convert it in a 0% to 100% value
  
  /*The inputThrottle (X axis) (that ranges from 0 to THROTTLE_NORMALIZED) is converted to the 0-100 range, to simplify calculations.
    The output speed (Y axis) is already expressed in the 0-100 range.
    The origin OLED display screen, which is 128x64, is located on the top-left corner of the screen.
    The origin of the graph is therefore shifted to the pixel (25, 50). Given that the screen heigth is only 64, the y values are also scaled to range from 0 to 50 (they are divided by 2).
    The x pixel values increases from left to right, but the y pixel values increases from top to bottom, therefore every (x, y) value in the (throttle, speed) domain is converted in the pixel domain as:
    x_pixel = (x_throttle + 25);
    y_pixel = (50 - (y_speed / 2)); */
  /* Clear screen and draw x and y axis */
  obdFill(&g_obd, OBD_WHITE, 1);
  obdDrawLine(&g_obd, 25, 0, 25, 50, OBD_BLACK, 1);
  obdDrawLine(&g_obd, 25, 50, 125, 50, OBD_BLACK, 1);
  /* Write the 100%, 0%, 50% and MIN and MAXpoints labels */
  obdWriteString(&g_obd, 0, 0, 0, (char *)"100%", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, 0, 58, (char *)"  0%", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, 104, 58, (char *)"100%", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, 0, map(g_storedVar.carParam[g_carSel].minSpeed, 0, 100, 50, 8), (char *)"MIN", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, 28, 50 - (g_storedVar.carParam[g_carSel].maxSpeed / 2), (char *)"MAX", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&g_obd, 0, 64, 58, (char *)"50%", FONT_6x8, OBD_BLACK, 1);

  /* Draw axis ticks at 50%, MIN SPEED and MAX speed points*/
  obdSetPixel(&g_obd, 24, 50 - (g_storedVar.carParam[g_carSel].minSpeed / 2), OBD_BLACK, 1);
  obdSetPixel(&g_obd, 23, 50 - (g_storedVar.carParam[g_carSel].minSpeed / 2), OBD_BLACK, 1);
  obdSetPixel(&g_obd, 25 + inputThrottle, 51, OBD_BLACK, 1);
  obdSetPixel(&g_obd, 25 + inputThrottle, 52, OBD_BLACK, 1);
  obdSetPixel(&g_obd, 26, 50 - (g_storedVar.carParam[g_carSel].maxSpeed / 2), OBD_BLACK, 1);
  obdSetPixel(&g_obd, 27, 50 - (g_storedVar.carParam[g_carSel].maxSpeed / 2), OBD_BLACK, 1);

  /* Set encoder to curve parameters */
  g_rotaryEncoder.setAcceleration(SEL_ACCELERATION);
  g_rotaryEncoder.setBoundaries(THROTTLE_CURVE_SPEED_DIFF_MIN_VALUE, THROTTLE_CURVE_SPEED_DIFF_MAX_VALUE, false);
  g_rotaryEncoder.reset(g_storedVar.carParam[g_carSel].throttleCurveVertex.curveSpeedDiff);

  /* Calculate the output speed of the throttle curve vertex (as in throttleCurve() function)
     This is calculated as the curveSpeedDiff (from 10% to 90%) percentage of the difference between minSpeed and maxSpeed */
  throttleCurveVertexSpeed = g_storedVar.carParam[g_carSel].minSpeed + (((uint32_t)g_storedVar.carParam[g_carSel].maxSpeed - (uint32_t)g_storedVar.carParam[g_carSel].minSpeed) * ((uint32_t)g_storedVar.carParam[g_carSel].throttleCurveVertex.curveSpeedDiff) / 100);

  /* Draw Line from MIN SPEED, to middle point */
  obdDrawLine(&g_obd, 25, 50 - (g_storedVar.carParam[g_carSel].minSpeed / 2), 25 + inputThrottle, map(throttleCurveVertexSpeed, 0, 100, 50, 0), OBD_BLACK, 1);
  /* Draw Line from middle point to 100% */
  obdDrawLine(&g_obd, 25 + inputThrottle, map(throttleCurveVertexSpeed, 0, 100, 50, 0), 125, map(g_storedVar.carParam[g_carSel].maxSpeed, 0, 100, 50, 0), OBD_BLACK, 1);

  /* Write the CURVE value */
  sprintf(msgStr, "%3d%c", g_storedVar.carParam[g_carSel].throttleCurveVertex.curveSpeedDiff, '%');
  obdWriteString(&g_obd, 0, OLED_WIDTH - 48, 34, msgStr, FONT_12x16, OBD_BLACK, 1);

  /* Write the trigger value */
  sprintf(msgStr, "%3d%c", prevTrigger, '%');
  obdWriteString(&g_obd, 0, OLED_WIDTH - 32, 26, msgStr, FONT_8x8, OBD_BLACK, 1);

  /* Exit curve function when encoder is clicked */
  while (!g_rotaryEncoder.isEncoderButtonClicked())
  {
    /* Write the trigger value only if it changed */
    if (g_escVar.outputSpeed_pct != prevTrigger)
    {
      /* Update trigger */
      prevTrigger = g_escVar.outputSpeed_pct;

      /* Write the trigger value */
      sprintf(msgStr, "%3d%c", prevTrigger, '%');
      obdWriteString(&g_obd, 0, OLED_WIDTH - 32, 26, msgStr, FONT_8x8, OBD_BLACK, 1);
    }

    /* Get encoder position if it was changed and correct the new lines*/
    if (g_rotaryEncoder.encoderChanged())
    {
      /* Cancel the old lines (draw them in black) */
      obdDrawLine(&g_obd, 25, 50 - (g_storedVar.carParam[g_carSel].minSpeed / 2), 25 + inputThrottle, map(throttleCurveVertexSpeed, 0, 100, 50, 0), OBD_WHITE, 1);
      obdDrawLine(&g_obd, 25 + inputThrottle, map(throttleCurveVertexSpeed, 0, 100, 50, 0), 125, map(g_storedVar.carParam[g_carSel].maxSpeed, 0, 100, 50, 0), OBD_WHITE, 1);
      
      /* Update the speed coordinate of the vertex */
      g_storedVar.carParam[g_carSel].throttleCurveVertex.curveSpeedDiff = g_rotaryEncoder.readEncoder();
      sprintf(msgStr, "%3d%c", g_storedVar.carParam[g_carSel].throttleCurveVertex.curveSpeedDiff, '%');
      throttleCurveVertexSpeed = g_storedVar.carParam[g_carSel].minSpeed + ((uint32_t)g_storedVar.carParam[g_carSel].maxSpeed - (uint32_t)g_storedVar.carParam[g_carSel].minSpeed) * ((uint32_t)g_storedVar.carParam[g_carSel].throttleCurveVertex.curveSpeedDiff) / 100;
      obdWriteString(&g_obd, 0, OLED_WIDTH - 48, 34, msgStr, FONT_12x16, OBD_BLACK, 1);
      
      /* Draw the new lines */
      obdDrawLine(&g_obd, 25, 50 - (g_storedVar.carParam[g_carSel].minSpeed / 2), 25 + inputThrottle, map(throttleCurveVertexSpeed, 0, 100, 50, 0), OBD_BLACK, 1);
      obdDrawLine(&g_obd, 25 + inputThrottle, map(throttleCurveVertexSpeed, 0, 100, 50, 0), 125, map(g_storedVar.carParam[g_carSel].maxSpeed, 0, 100, 50, 0), OBD_BLACK, 1);
    }
    else
    {
      /* Service the watchdog, to prevent CPU reset */
      vTaskDelay(10);
    }
  }

  /* Reset encoder */
  g_rotaryEncoder.setAcceleration(MENU_ACCELERATION);
  g_rotaryEncoder.setBoundaries(1, MENU_ITEMS_COUNT, false);
  g_rotaryEncoder.reset(g_encoderMainSelector);
  g_escVar.encoderPos = g_encoderMainSelector;
  saveEEPROM(g_storedVar);          /* Save modified values to EEPROM */
  obdFill(&g_obd, OBD_WHITE, 1);    /* Clear screen */

  return;
}



/**
 * Take an ADC raw value where the max and min has been recorded and returns the value scaled
 * 
 * @param raw The raw ADC value
 * @param minIn The lowest reading of the ADC
 * @param maxIn The highest reading of the ADC
 * @param normalizedMax The new max value of the scale
 * @param isReversed Whether the throttle is reversed or not
 * @return The raw ADC value scaled from 0 to normalizedMax
 */
uint16_t normalizeAndClamp(uint16_t raw, uint16_t minIn, uint16_t maxIn, uint16_t normalizedMax, bool isReversed)
{
  uint16_t retVal = 0;  /* From 0 to normalizedMax */

  if (maxIn == minIn)  /* If maxIn == minIn avoid division by 0 */
  {
    retVal = 0;
  }
  else
  {
    raw = constrain(raw, minIn, maxIn); /* Make sure the raw value is constrained between minIn and maxIn */

    if (isReversed == true)  /* If throttle is reversed (it goes to low values when pressed) */
    {
      raw = abs(maxIn - raw);
    }
    else
    {
      raw = abs(raw - minIn);
    }

    retVal = ((uint32_t)raw * normalizedMax) / (maxIn - minIn); /* Scale the raw reading */
  }

  return retVal;
}


/**
 * Calculate the derivative of the trigger based on a rolling average of the previous trigger values.
 * 
 * @param currTrigger The current trigger value
 * @return The derivative of the trigger as difference between current trigger and rolling average of previous trigger values.
 */
int16_t computeTriggerDerivativeGPT(uint16_t currTrigger) {
  static uint16_t valueBuffer[TRIG_AVG_COUNT] = { 0 };  /* Buffer to store the last 100 values    */
  static uint8_t bufferIndex = 0;                       /* Index for the circular buffer          */
  static uint32_t sum = 0;                              /* Sum of the values in the buffer        */
  static uint8_t isFirstCall = 1;                       /* Flag to check if it's the first call   */
  uint16_t rollingAverage = 0;                          /* Average trigger calculated on the buffer values */
  int16_t derivative;

  if (isFirstCall) {
    /* Initialize the buffer and sum on the first call */
    for (uint8_t i = 0; i < TRIG_AVG_COUNT; i++)
    {
      valueBuffer[i] = currTrigger;
      sum += currTrigger;
    }
    isFirstCall = 0;
  } 
  else 
  {
    /* Update the sum by subtracting the old value and adding the new value */
    sum -= valueBuffer[bufferIndex];
    sum += currTrigger;

    /* Update the buffer with the current value */
    valueBuffer[bufferIndex] = currTrigger;

    /* Increment the buffer index and wrap around if necessary */
    bufferIndex = (bufferIndex + 1) % TRIG_AVG_COUNT;
  }

  /* Calculate the rolling average */
  rollingAverage = sum / TRIG_AVG_COUNT;

  /* Calculate the derivative as the difference between the current value and the rolling average */
  derivative = (int16_t)(currTrigger - rollingAverage);

  return derivative;
}


/**
 * Saturate an input value between a upper and lower bound
 * 
 * @param paramValue The input value to be saturated
 * @param minValue The lower bound
 * @param maxValue The upper bound
 * @return The saturated input value.
 */
uint16_t saturateParamValue(uint16_t paramValue, uint16_t minValue, uint16_t maxValue) 
{
  uint16_t retValue = paramValue;

  if (paramValue > maxValue) 
  {
    retValue = maxValue;
  } else if (paramValue < minValue) 
  {
    retValue = minValue;
  }

  return retValue;
}


void saveEEPROM(StoredVar_type toSave) {
  g_pref.begin("stored_var", false);                      /* Open the "stored" namespace in read/write mode */
  g_pref.putBytes("user_param", &toSave, sizeof(toSave)); /* Put the value of the stored user_param */
  g_pref.end();                                           /* Close the namespace */
}

/***********************  DEBUG -TEST  **************************************************/
//printf for Antispin debug
/***********************  DEBUG -TEST  **************************************************/
//printf for Antispin debug
// include following variables among the global variables
// debug printf array inside the ISR routine
/*#define MAX_LOG_COUNT 300 // log is used in the antispin func
bool log_overflow=false;
static uint8_t arrayIndex = 0;
static uint32_t arrayOutSpeed[MAX_LOG_COUNT];
static uint32_t arrayOutSpeed1000[MAX_LOG_COUNT];
static unsigned long arrayDeltaMicros[MAX_LOG_COUNT];
static unsigned long arrayMicros[MAX_LOG_COUNT];
static uint32_t arrayTrigPos[MAX_LOG_COUNT];
static uint32_t arrayMaxDeltaSx1000[MAX_LOG_COUNT];
static uint32_t arrayRampStart[MAX_LOG_COUNT];
uint16_t        throttleReadAnalog;
/*

  // DEBUG LOG, remove me- include this in the Antispin function
  if (throtSet !=0 && outSpeed !=100)
  {
      //arrayRampStart[arrayIndex]     = rampStartUs;
      arrayMaxDeltaSx1000[arrayIndex] = maxDeltaSpeedx1000;
      arrayDeltaMicros[arrayIndex]   = deltaTimeUs;
      arrayMicros[arrayIndex]        = thisCalluS;
      arrayTrigPos[arrayIndex]       = throtSet;
      arrayOutSpeed [arrayIndex]     = outSpeed;
      arrayOutSpeed1000 [arrayIndex] = outSpeedX1000;
      if (arrayIndex < MAX_LOG_COUNT)
        arrayIndex++;
      else
        log_overflow = true;
  }
  if (throtSet ==0)
    {
      arrayIndex = 0;
      log_overflow = false;
    }



      // DEBUG antistispin , include this in the main loop
      if (arrayIndex != 0 && escVar.outSpdSetPerc==100 )
      {
        if (log_overflow == true)
          Serial.print(" log array overflow");
        else
        {
          for (int i = 0 ; i< arrayIndex ; i++)
          {
            Serial.print(" index=");
            Serial.print(i);
            Serial.print(" trigPos=");
            Serial.print(arrayTrigPos [i]);
            Serial.print(" micro=");
            Serial.print(arrayMicros [i]);
            Serial.print(" deltaMicro=");
            Serial.print(arrayDeltaMicros [i]);
            Serial.print(" outspeed=");
            Serial.print(arrayOutSpeed [i]);
            Serial.print(" arrayMaxDeltaSx1000=");
            Serial.print(arrayMaxDeltaSx1000 [i]);
            Serial.print(" outspeed1000=");
            Serial.println(arrayOutSpeed1000 [i]);
            delay(1);
          }
        }
        arrayIndex =0;
      }  
int16_t computeTrigDerivFabio(uint16_t currTrigger)
{
  int16_t retVal; // using int instead of float because using float during ISR causes the core to panic 
  uint16_t currTime;
  static uint16_t prevTrigForAVG[TRIG_AVG_COUNT] = {1, 1, 2, 3, 4};
  int16_t deltaTrigger;
  int16_t avgPrevTriggers = 0;

  // calculate the average of the previous TRIG_AVG_COUNT triggrer values
  for (int i = 0; i < TRIG_AVG_COUNT; i++)
  {
    avgPrevTriggers += prevTrigForAVG[i];
  }
  avgPrevTriggers = avgPrevTriggers / TRIG_AVG_COUNT;

  // Calculate delta trigger and delta time 
  // We want that a pressed trigger produce a positive delta 
  deltaTrigger = currTrigger - avgPrevTriggers;
  // Compute derivative as dTrigger / dt (dt is always 1 ms)
  retVal = deltaTrigger;

  // shift previous trigger discaring the last one to move the average forward 
  for (int i = TRIG_AVG_COUNT; i > 0; i--)
  {
    prevTrigForAVG[i] = prevTrigForAVG[i-1];
  }
  prevTrigForAVG[0] = currTrigger;

  return retVal;
}  
*/