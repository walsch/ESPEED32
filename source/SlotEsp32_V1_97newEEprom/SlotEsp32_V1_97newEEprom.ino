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
#define SW_MINOR_VERSION 97
/* Last modified: 01/10/2024 */
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

TaskHandle_t Task1;
TaskHandle_t Task2;

static FSM_state_enum currState = INIT;
uint16_t debug;

/* Display Backbuffer */
#ifdef USE_BACKBUFFER
static uint8_t ucBackBuffer[1024];
#else
static uint8_t *ucBackBuffer = NULL;
#endif

/* Display global instance */
OBDISP obd;

/* Display string global instance */
char msgStr[50];

uint16_t gCarSel;  // global variable telling whch car model has been selected

/* Rotary Encoder global instance */
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_BUTTON_PIN, ENCODER_VCC_PIN, ENCODER_STEPS);

/* Value of main menu selector (indicates selected item) */
static uint8_t encoderMainSelector = 1;
/* Value of secondarty selector (indicates selected item's value) */
static uint8_t encoderSecondarySelector = 0;
/* Global instance of pointer to the value selected by the encoder */
static uint16_t *encoderSelectedValuePtr = NULL;

/* EEPROM stored values (mainly calibration) global instance */
EEPROM_stored_var_type storedVar;

/* ESC internal variables gloabl instance */
ESC_var_type escVar{
  .outSpdSetPerc = 0,  // [%] set speed
  .trigRaw = 0,
  .trigNorm = 0,  //  Normalized trigger position
  .trigDeriv = 0,
  .encoderPos = 1,  // [%] encodReadRaw
  .Vin_mV = 0
};

/* Menu global instances */
menu_type mainMenu{
  .lines = 3
};

menu_type carMenu{
  .lines = 3
};

/* Preferences global instance (for storing NVM data, replace EEPROM library) */

Preferences pref;

static uint32_t lastEncoderInteraction = 0;  // tell how long is the encoder untouched, so can avoid keep printing he display menu and save CPU cycles

/*********************************************************************************************************************/
/*--------------------------------------------- Function Declaration----------------------------------------*/
/*********************************************************************************************************************/
void IRAM_ATTR readEncoderISR();


/*********************************************************************************************************************/
/*---------------------------------------------Setup Function Implementations----------------------------------------*/
/*********************************************************************************************************************/

/* Setup */
void setup() {

  /***** Pin and Serial Setup *****/
  HAL_PinSetup();

  /***** HalfBridge & HW Setup *****/
  HalfBridge_SetupFabio();

  /***** create a task that will be executed in the Task1code() and Task2code() funcitons, executed on core 0 and 1 *****/
  /* Task1code:slotESC state machine, managex OLED display and Encoder, low priority task */
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  /* TASK2 : perform trigger reading, trigger conditioning, PWM output */
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

//Task1code: slotESC state machine: manage OLED display and Encoder, low priority task
void Task1code(void *pvParameters) {
  for (;;) {
    FSM_state_enum prevState = currState;  // variable used to track if state machine changed state from last loop
    static uint16_t prevFreqPWM = 0;
    static menu_state_enum menuState = ITEM_SELECTION;
    static uint8_t eepromTmp;
    static uint8_t swMajVer, swMinVer;

    escVar.Vin_mV = HAL_ReadVoltageDivider(AN_VIN_DIV, RVIFBL, RVIFBH); /* Read VIN */
    if (currState != INIT) // if variables are already fetched from the EEPROM
      gCarSel = storedVar.carSelNumber;    // update global variable telling which car model is actually selected

    switch (currState) {
     case INIT:
         pref.begin("stored_var", false); /* Open the "stored" namespace in read/write mode. If it doesn't exist, it creates it */
        if (pref.isKey("sw_maj_ver") && pref.isKey("sw_min_ver") && pref.isKey("user_param")) /* If all keys exists, then check their value */
        {
          /* Get the values of the sw version */
          swMajVer = pref.getUChar("sw_maj_ver");
          swMinVer = pref.getUChar("sw_min_ver");

          if ((swMajVer == SW_MAJOR_VERSION) && (swMinVer == SW_MINOR_VERSION)) /* If both keys are equal to the SW Version MACRO, then the stored param are already initialized */
          {

            pref.getBytes("user_param", &storedVar, sizeof(storedVar)); /* Get the value of the stored user_param */
            initMenuVariables();                                        // init menu variables with EEPROM stored variables
            /* If button is pressed at startup, go to CALIBRATION state */
            if (digitalRead(ENCODER_BUTTON_PIN) == BUTT_PRESSED) {
              currState = CALIBRATION;
              /* Reset Min and Max to the opposite side, in order to have effective calibration */
              storedVar.minTrigRaw = MAX_INT16;
              storedVar.maxTrigRaw = MIN_INT16;
              calibSound();
              initDisplayAndEncoder();  // init and clear OLED and Encoder
              /* Wait until button is released, then go to CALIBRATION state */
              while (digitalRead(ENCODER_BUTTON_PIN) == BUTT_PRESSED)
                showScreenPreCalButPres();
              obdFill(&obd, OBD_WHITE, 1); /* Clear OLED */
            }
            /* If button is NOT pressed at startup, go to RUNNING state */
            else  //Init completed!
            {
              currState = WELCOME;
              gCarSel = storedVar.carSelNumber; //now it is safe to address the proper car
              initDisplayAndEncoder();  // init and clear OLED and Encoder
              onSound();
            }
            pref.end(); /* Close the namespace */
            break;
          }

          initDisplayAndEncoder();  // init and clear OLED and Encoder
                                    /* Clear all the keys in this namespace */

          pref.clear();
          /* If the code reaches here it means that:
        - the sw version keys are not present --> stored var are not initialized
        - the sw version stored are not up to date --> stored var are initialized but must be corrected

      Calibration values are NOT stored, go to CALIBRATION state */

          pref.putUChar("sw_maj_ver", SW_MAJOR_VERSION);
          pref.putUChar("sw_min_ver", SW_MINOR_VERSION);

          initStoredVariables();  // initialize stored variables with default values
          /* Reset Min and Max to the opposite side, in order to have effective calibration */
          storedVar.minTrigRaw = MAX_INT16;
          storedVar.maxTrigRaw = MIN_INT16;
          calibSound();
          currState = CALIBRATION;
          obdFill(&obd, OBD_WHITE, 1); /* Clear OLED */
          /* Press and release button to go to CALIBRATION state */
          while (!rotaryEncoder.isEncoderButtonClicked()) /* Loop until button is pressed */
          {
            showScreenNoEEPROM();
          }
        }

        break;

      case CALIBRATION:
        /* Read Throttle */
        throttleCalibration(escVar.trigRaw);  // trig raw is continuously read on task2
        showScreenCalibration(escVar.trigRaw);
        /* Exit calibration if button is presseded */
        if (rotaryEncoder.isEncoderButtonClicked())  // exit calibration and save data to EEPROM (note that all ESC variables remain the same becasue already loaded at startup)
        {
          offSound();
          initMenuVariables();                                        // init menu variables
          pref.putBytes("user_param", &storedVar, sizeof(storedVar)); /* Put the value of the stored user_param */
          pref.end();                                                 /* Close the namespace */
          HalfBridge_Enable();
          currState = WELCOME;
        }
        break;

      case WELCOME:
        showScreenWelcome();
        delay(1000);
        currState = RUNNING;
        break;

      case RUNNING: /* when the global variable State is in RUNNING MODE the Task2 will elaborate the trigger to produce the PWM out */
        /* Change menu state if encoder button is clicked */
        if (rotaryEncoder.isEncoderButtonClicked()) {
          menuState = rotary_onButtonClick(menuState);
          lastEncoderInteraction = millis();
        }

        /* Get encoder position if it was changed*/
        if (rotaryEncoder.encoderChanged()) {
          escVar.encoderPos = rotaryEncoder.readEncoder();
          lastEncoderInteraction = millis();
        }

        /* Change Encoder variable depending on menu state */
        if (menuState == ITEM_SELECTION) {
          encoderMainSelector = escVar.encoderPos;
        }
        if (menuState == VALUE_SELECTION) {
          encoderSecondarySelector = escVar.encoderPos;
          *encoderSelectedValuePtr = encoderSecondarySelector;
          storedVar.carParam[gCarSel].throttleSetPoint.outSpeed = saturateParamValue(storedVar.carParam[gCarSel].throttleSetPoint.outSpeed, THR_SETP_MIN_VALUE, THR_SETP_MAX_VALUE); /* Regulate min value for throttle setpoint */
        }

        /* Show RUN display */
        printMainMenu(menuState);
        if (storedVar.carParam[gCarSel].freqPWM != prevFreqPWM)  // if PWM freq parameter is changed, update motor PWM
        {
          prevFreqPWM = storedVar.carParam[gCarSel].freqPWM;
          ledcDetach(HB_IN_PIN);
          ledcDetach(HB_INH_PIN);
          //uint16_t freqTmp = map(storedVar.carParam[gCarSel].freqPWM, 0, 100, FREQ_MIN_VALUE, FREQ_MAX_VALUE);
          uint16_t freqTmp = storedVar.carParam[gCarSel].freqPWM * 100;
          ledcAttachChannel(HB_IN_PIN, freqTmp, THR_PWM_RES_BIT, THR_IN_PWM_CHAN);
          ledcAttachChannel(HB_INH_PIN, freqTmp, THR_PWM_RES_BIT, THR_INH_PWM_CHAN);
        }
        break;

      case FAULT:
        break;

      default:
        break;
    }

    if (currState != prevState) /* Every time FSM machine change state */
      obdFill(&obd, OBD_WHITE, 1);
  }
}

/* TASK2 : trigger reading, trigger conditioning, PWM output
   Legend:
  - Perc, Pct: percent value between 0 and 100%
  - Norm: Normalized value between 0 and THROTTLE_NORMALIZED
  */
void Task2code(void *pvParameters) {
  static uint16_t tmpDragBrake, tmpTrigNorm, onlyPlusDeriv;
  static unsigned long lastCallUs, deltaTimeUs, thisCalluS;
  static unsigned long actualTrigRaw = 0, prevTrigRaw = 0;

 // HalfBridge_Enable();  //not really needed? it is only toggling the inhibit

  for (;;) {
    thisCalluS = micros();
    deltaTimeUs = thisCalluS - lastCallUs;  // delta time from last call of this function
    if (deltaTimeUs > ESC_PERIOD_US) {
      prevTrigRaw = actualTrigRaw;
      actualTrigRaw = HAL_ReadTriggerRaw();  //always read trigger (needed in calibration and other func)
      escVar.trigRaw = (prevTrigRaw + actualTrigRaw) / 2;
      //escVar.trigRaw = storedVar.maxTrigRaw ;// DEBUG remove me !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      lastCallUs = thisCalluS;                               // update last call static memory
      if (!(currState == CALIBRATION || currState == INIT))  // do not apply power if in calibration or before vari (TODO: would be better to have also variables init)
      {
        //Throttle -> Speed pipeline , perform time dependent adjustment
        tmpTrigNorm = normalizeAndClamp(escVar.trigRaw, storedVar.minTrigRaw, storedVar.maxTrigRaw, THROTTLE_NORMALIZED, THROTTLE_REV);  // Get Raw trigger position and return throttle between 0 and THROTTLE_NORMALIZED
        escVar.trigNorm = addDeadBand(tmpTrigNorm, THROTTLE_DEADBAND_NORM);                                                              // Account for Deadband percent
        escVar.trigDeriv = computetrigDerivGPT(escVar.trigNorm);
        escVar.outSpdSetPerc = throttleCurve(escVar.trigNorm);          // Map inputTrigPos to throttleCurve
        escVar.outSpdSetPerc = throttleAntiSpin(escVar.outSpdSetPerc);  // define actual throttle output , considering antispin

        if (escVar.outSpdSetPerc == 0) {                                // if the requested speed is 0
          HalfBridge_SetPwmDrag(0, storedVar.carParam[gCarSel].brake);  // apply brake only (and speed to 0) in case speed set is 0
          tmpDragBrake = 0;                                             //set also tmpDragBrake to 0 just to look nicer on the display
        } else {
          // make TMP dragbrake prpoportional to the derivative
          onlyPlusDeriv = constrain(-escVar.trigDeriv, 0, MAX_UINT16);
          tmpDragBrake = normalizeAndClamp(onlyPlusDeriv, 0, DERIVATE_MAX_ACTION_NORM, storedVar.carParam[gCarSel].dragBrake, 0);
          HalfBridge_SetPwmDrag(escVar.outSpdSetPerc, tmpDragBrake);  // apply output duty & drag brake
        }
      }
    }  // if (deltaTimeUs > ESC_PERIOD_US)
  }
}

// real loop are in the Tasks
void loop() {
}

/*********************************************************************************************************************/
/*---------------------------------------------Setup Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

// separate from intiHW becasue it takes some time, and we want wuick start
void initDisplayAndEncoder() {
  uint16_t rc;
  /***** OLED Display Setup *****/
  rc = obdI2CInit(&obd, MY_OLED, OLED_ADDR, FLIP180, INVERT_DISP, USE_HW_I2C, SDA1_PIN, SCL1_PIN, RESET_PIN, 800000L);  // use standard I2C bus at 400Khz
  if (rc != OLED_NOT_FOUND) {
    obdSetBackBuffer(&obd, ucBackBuffer);
    obdFill(&obd, OBD_WHITE, 1);
  } else {
    Serial.println("Error! Failed OLED Display initialization!");
  }

  /***** Encoder Setup *****/
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(1, MENU_ITEMS_COUNT, false); /* minValue, maxValue, circleValues true|false (when max go to min and vice versa) */
  rotaryEncoder.setAcceleration(MENU_ACCELERATION);        /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
}

/* Rotary Encoder ISR */
void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

void initStoredVariables() {
  for (int i = 0; i < CAR_MAX_COUNT; i++) {
    storedVar.carParam[i].minSpeed = MIN_SPEED_DEFAULT;  // [%] userMinSpeed default
    storedVar.carParam[i].brake = BRAKE_DEFAULT;
    storedVar.carParam[i].dragBrake = DRAG_BRAKE_DEFAULT;
    storedVar.carParam[i].maxSpeed = MAX_SPEED_DEFAULT;
    storedVar.carParam[i].throttleSetPoint = { THR_CRV_IN_LEVEL_JOINT, THR_CRV_OUT_LEVEL_DEFAULT };
    storedVar.carParam[i].antiSpin = ANTISPIN_DEFAULT;
    storedVar.carParam[i].freqPWM = PWM_FREQ_DEFAULT;
    storedVar.carParam[i].carNumber = i;
    sprintf(storedVar.carParam[i].carName, "CAR%1d", i);
  }
  storedVar.carSelNumber = 0;
  storedVar.minTrigRaw = 0;
  storedVar.maxTrigRaw = ACD_RESOLUTION_STEPS;
}


void initMenuVariables() {
  int i = 0;
  /* Init menu items */
  sprintf(mainMenu.item[i].name, "SENSI");
  mainMenu.item[i].value = (void *)&storedVar.carParam[gCarSel].minSpeed;
  mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  mainMenu.item[i].unit = '%';
  mainMenu.item[i].maxValue = min(MIN_SPEED_MAX_VALUE, (int)storedVar.carParam[gCarSel].maxSpeed);
  mainMenu.item[i].minValue = 0;
  mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(mainMenu.item[++i].name, "BRAKE");
  mainMenu.item[i].value = (void *)&storedVar.carParam[gCarSel].brake;
  mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  mainMenu.item[i].unit = '%';
  mainMenu.item[i].maxValue = BRAKE_MAX_VALUE;
  mainMenu.item[i].minValue = 0;
  mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(mainMenu.item[++i].name, "ANTIS");
  mainMenu.item[i].value = (void *)&storedVar.carParam[gCarSel].antiSpin;
  mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  mainMenu.item[i].unit = 'm';
  mainMenu.item[i].maxValue = ANTISPIN_MAX_VALUE;
  mainMenu.item[i].minValue = 0;
  mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(mainMenu.item[++i].name, "CURVE");
  mainMenu.item[i].value = (void *)&storedVar.carParam[gCarSel].throttleSetPoint.outSpeed;
  mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  mainMenu.item[i].unit = '%';
  mainMenu.item[i].maxValue = THR_SETP_MAX_VALUE;
  mainMenu.item[i].minValue = THR_SETP_MIN_VALUE;
  mainMenu.item[i].callback = &showCurveSelection;

  sprintf(mainMenu.item[++i].name, "DRAGB");
  mainMenu.item[i].value = (void *)&storedVar.carParam[gCarSel].dragBrake;
  mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  mainMenu.item[i].unit = '%';
  mainMenu.item[i].maxValue = DRAG_MAX_VALUE;
  mainMenu.item[i].minValue = 0;
  mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(mainMenu.item[++i].name, "PWM_F");
  mainMenu.item[i].value = (void *)&storedVar.carParam[gCarSel].freqPWM;
  mainMenu.item[i].type = VALUE_TYPE_DECIMAL;
  mainMenu.item[i].unit = 'k';
  mainMenu.item[i].maxValue = FREQ_MAX_VALUE / 100;
  mainMenu.item[i].minValue = FREQ_MIN_VALUE / 100;
  mainMenu.item[i].decimalPoint = 1;
  mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(mainMenu.item[++i].name, "LIMIT");
  mainMenu.item[i].value = (void *)&storedVar.carParam[gCarSel].maxSpeed;
  mainMenu.item[i].type = VALUE_TYPE_INTEGER;
  mainMenu.item[i].unit = '%';
  mainMenu.item[i].maxValue = MAX_SPEED_DEFAULT;
  mainMenu.item[i].minValue = max(5, (int)storedVar.carParam[gCarSel].minSpeed + 5);
  mainMenu.item[i].callback = ITEM_NO_CALLBACK;

  sprintf(mainMenu.item[++i].name, "*CAR*");
  mainMenu.item[i].value = (void *)&storedVar.carParam[gCarSel].carName;
  mainMenu.item[i].type = VALUE_TYPE_STRING;
  mainMenu.item[i].maxValue = CAR_MAX_COUNT - 1;  // so menu will scroll in the array (CAR_MAX_COUNT long)
  mainMenu.item[i].minValue = 0;
  mainMenu.item[i].callback = &showSelectRenameCar;

  /* Init Car selection menu variables */
  for (uint8_t j = 0; j < CAR_MAX_COUNT; j++) {
    sprintf(carMenu.item[j].name, (const char *)storedVar.carParam[j].carName);
    carMenu.item[j].value = (void *)&storedVar.carParam[j].carNumber;
    mainMenu.item[i].type = VALUE_TYPE_STRING;
    carMenu.item[i].unit = ' ';
    carMenu.item[i].maxValue = CAR_MAX_COUNT - 1;  // so menu will scroll in the array (CAR_MAX_COUNT long)
    carMenu.item[i].minValue = 0;
    carMenu.item[i].callback = ITEM_NO_CALLBACK;
  }
}

void showScreenWelcome() {
  obdWriteString(&obd, 0, 0, 12, (char *)"SlotEsp32", FONT_12x16, OBD_BLACK, 1);
  sprintf(msgStr, "V%d.%02d", SW_MAJOR_VERSION, SW_MINOR_VERSION);  //print SW version
  obdWriteString(&obd, 0, 0, 28, msgStr, FONT_12x16, OBD_WHITE, 1);
}


void showScreenPreCalButPres() {
  sprintf(msgStr, "   ESC v%d.%02d ", SW_MAJOR_VERSION, SW_MINOR_VERSION);  //print SW version
  obdWriteString(&obd, 0, 0, 0, msgStr, FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, (OLED_WIDTH / 2) - 48, 1 * HEIGHT8x8, (char *)"Release button", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, (OLED_WIDTH / 2) - 48, 2 * HEIGHT8x8, (char *)"to calibrate", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, (OLED_WIDTH / 2) - 48, 3 * HEIGHT8x8, (char *)"     OR    ", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, (OLED_WIDTH / 2) - 48, 4 * HEIGHT8x8, (char *)"remove power", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, (OLED_WIDTH / 2) - 48, 5 * HEIGHT8x8, (char *)"  to exit  ", FONT_8x8, OBD_BLACK, 1);
}


void showScreenNoEEPROM() {
  sprintf(msgStr, "SlotEsp32 v%d.%02d", SW_MAJOR_VERSION, SW_MINOR_VERSION);  //print SW version
  obdWriteString(&obd, 0, 0, 0, msgStr, FONT_8x8, OBD_WHITE, 1);
  obdWriteString(&obd, 0, (OLED_WIDTH / 2) - 64, 3 * HEIGHT8x8, (char *)"EEPROM NOT init!", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, (OLED_WIDTH / 2) - 48, 5 * HEIGHT8x8, (char *)"Press button", FONT_8x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, (OLED_WIDTH / 2) - 48, 6 * HEIGHT8x8, (char *)"to calibrate", FONT_8x8, OBD_BLACK, 1);
}


void showScreenCalibration(int16_t adcRaw) {
  sprintf(msgStr, "CALIBRATION");
  obdWriteString(&obd, 0, (OLED_WIDTH - 66) / 2, 0, msgStr, FONT_6x8, OBD_WHITE, 1);

  sprintf(msgStr, "press/releas throt");
  obdWriteString(&obd, 0, 0, 8, msgStr, FONT_6x8, OBD_BLACK, 1);

  sprintf(msgStr, "Raw throttle %4d  ", adcRaw);
  obdWriteString(&obd, 0, 0, 24, msgStr, FONT_6x8, OBD_BLACK, 1);

  sprintf(msgStr, "Min throttle %4d   ", storedVar.minTrigRaw);
  obdWriteString(&obd, 0, 0, 32, msgStr, FONT_6x8, OBD_BLACK, 1);

  sprintf(msgStr, "Max throttle %4d   ", storedVar.maxTrigRaw);
  obdWriteString(&obd, 0, 0, 40, msgStr, FONT_6x8, OBD_BLACK, 1);

  sprintf(msgStr, " push when done ");
  obdWriteString(&obd, 0, 0, 56, msgStr, FONT_6x8, OBD_BLACK, 1);
}

void printMainMenu(menu_state_enum currMenuState) {
  static uint16_t tmp = 0;
  ;

  /* "Frame" indicates which items are currently displayed */
  static uint16_t frameUpper = 1;
  static uint16_t frameLower = mainMenu.lines;

  /* In encoder move out of frame, adjust frame */
  if (encoderMainSelector > frameLower) {
    frameLower = encoderMainSelector;
    frameUpper = frameLower - mainMenu.lines + 1;
    obdFill(&obd, OBD_WHITE, 1);
  } else if (encoderMainSelector < frameUpper) {
    frameUpper = encoderMainSelector;
    frameLower = frameUpper + mainMenu.lines - 1;
    obdFill(&obd, OBD_WHITE, 1);
  }

  /* Print main menu only if there was an encoder interaction in the last 100 milliseconds */
  if (millis() - lastEncoderInteraction < 100) {
    for (uint8_t i = 0; i < mainMenu.lines; i++) {
      /* Print item name */
      obdWriteString(&obd, 0, 0, i * HEIGHT12x16, mainMenu.item[frameUpper - 1 + i].name, FONT_12x16, (encoderMainSelector - frameUpper == i) ? OBD_WHITE : OBD_BLACK, 1);

      /* Only print value if value != ITEM_NO_VALUE */
      if (mainMenu.item[frameUpper - 1 + i].value != ITEM_NO_VALUE) {
        /* if the value is a number, cast to *(unit16_t *), then print number and unit */
        if (mainMenu.item[frameUpper - 1 + i].type == VALUE_TYPE_INTEGER) {
          /* value is a generic pointer to void, so first cast to uint16_t pointer, then take the pointed value */
          sprintf(msgStr, "%4d%c", *(uint16_t *)(mainMenu.item[frameUpper - 1 + i].value), mainMenu.item[frameUpper - 1 + i].unit);
          obdWriteString(&obd, 0, OLED_WIDTH - 60, i * HEIGHT12x16, msgStr, FONT_12x16, (((encoderMainSelector - frameUpper == i) && (currMenuState == VALUE_SELECTION)) ? OBD_WHITE : OBD_BLACK), 1);
        }
        /* If the value is a decimal, cast to *(unit16_t *), divide by 10^decimalPoint then print number and unit */
        else if (mainMenu.item[frameUpper - 1 + i].type == VALUE_TYPE_DECIMAL) {
          /* value is a generic pointer to void, so first cast to uint16_t pointer, then take the pointed value */
          tmp = *(uint16_t *)(mainMenu.item[frameUpper - 1 + i].value);
          sprintf(msgStr, " %d.%01d%c", tmp / 10, (tmp % 10), mainMenu.item[frameUpper - 1 + i].unit);
          //sprintf(msgStr, "%#4.*%c", mainMenu.item[frameUpper - 1 + i].decimalPoint, (float)tmp/(pow(10, mainMenu.item[frameUpper - 1 + i].decimalPoint)), mainMenu.item[frameUpper - 1 + i].unit);

          obdWriteString(&obd, 0, OLED_WIDTH - 60, i * HEIGHT12x16, msgStr, FONT_12x16, (((encoderMainSelector - frameUpper == i) && (currMenuState == VALUE_SELECTION)) ? OBD_WHITE : OBD_BLACK), 1);
        }
        /* If the value is a string, cast to (char *) then print the string */
        else if (mainMenu.item[frameUpper - 1 + i].type == VALUE_TYPE_STRING) {
          /* value is a generic pointer to void, so cast to string pointer */
          sprintf(msgStr, "%s", (char *)(mainMenu.item[frameUpper - 1 + i].value));
          obdWriteString(&obd, 0, OLED_WIDTH - (4 * WIDTH12x16), i * HEIGHT12x16, msgStr, FONT_12x16, (((encoderMainSelector - frameUpper == i) && (currMenuState == VALUE_SELECTION)) ? OBD_WHITE : OBD_BLACK), 1);
        }
      }
    }

    initMenuVariables();  // update menu variables with the storedVar that could hav been changed in previous for cycle

    if (storedVar.carParam[gCarSel].maxSpeed < MAX_SPEED_DEFAULT) {
      obdWriteString(&obd, 0, WIDTH8x8, 3 * HEIGHT12x16, (char *)" - LIMITER - ", FONT_8x8, OBD_WHITE, 1);  // print "LIM" to show limited speed is available
    } else {
      obdWriteString(&obd, 0, WIDTH8x8, 3 * HEIGHT12x16, (char *)"             ", FONT_8x8, OBD_BLACK, 1);  // remove "LIM" to show limited speed is available
    }
  }

  /* print analytic - statistic line */
  sprintf(msgStr, "%3d%c", escVar.outSpdSetPerc, '%');
  obdWriteString(&obd, 0, 0, 3 * HEIGHT12x16 + HEIGHT8x8, msgStr, FONT_8x8, (escVar.outSpdSetPerc == 100) ? OBD_WHITE : OBD_BLACK, 1);
  sprintf(msgStr, "%3d ", escVar.trigDeriv);
  obdWriteString(&obd, 0, 4 * WIDTH8x8, 3 * HEIGHT12x16 + HEIGHT8x8, msgStr, FONT_6x8, OBD_BLACK, 1);
  sprintf(msgStr, " %d.%01dV ", escVar.Vin_mV / 1000, (escVar.Vin_mV % 1000) / 100);
  obdWriteString(&obd, 0, 7 * WIDTH8x8, 3 * HEIGHT12x16 + HEIGHT8x8, msgStr, FONT_6x8, OBD_BLACK, 1);  // with -1 as X means start when previous write stopped

  sprintf(msgStr, " %d   ", debug);
  obdWriteString(&obd, 0, 12 * WIDTH8x8, 3 * HEIGHT12x16 + HEIGHT8x8, msgStr, FONT_6x8, OBD_BLACK, 1);  // with -1 as X means start when previous write stopped
}

uint16_t throttleAntiSpin(uint16_t throtSet) {
  static uint32_t lastOutSpeedx1000 = 0, maxDeltaSpeedx1000;
  static unsigned long lastCallUs, deltaTimeUs, thisCalluS;
  uint32_t outSpeed, outSpeedX1000;

  thisCalluS = micros();
  deltaTimeUs = thisCalluS - lastCallUs;  // delta time from last call of this function
  lastCallUs = thisCalluS;                // update last call static memory

  /* Bypass calculation if antiSpin is 0 (OFF) and just return throtSet */
  if ((storedVar.carParam[gCarSel].antiSpin) == 0) {
    outSpeed = throtSet;
  } else {
    maxDeltaSpeedx1000 = ((storedVar.carParam[gCarSel].maxSpeed - storedVar.carParam[gCarSel].minSpeed) * (deltaTimeUs)) / (storedVar.carParam[gCarSel].antiSpin);  // deltaSpeed = (minSPeed-MaxSpeed)*(DeltaTime / antiSpin (from min speed to max speed) )

    if ((uint32_t)throtSet * 1000 <= lastOutSpeedx1000)  // when braking or slowering do it immediately
    {
      outSpeed = throtSet;
      lastOutSpeedx1000 = outSpeed * 1000;
    } else {
      if (lastOutSpeedx1000 < ((uint32_t)throtSet * 1000 - maxDeltaSpeedx1000))  // check there is room to increase speed by a maxdeltaspeed
      {
        outSpeedX1000 = (lastOutSpeedx1000 + maxDeltaSpeedx1000);
      } else {
        outSpeedX1000 = throtSet * 1000;
      }

      if (outSpeedX1000 < storedVar.carParam[gCarSel].minSpeed * 1000)  // check in order to start the ramp from minspeed (and not from 0)
        outSpeedX1000 = storedVar.carParam[gCarSel].minSpeed * 1000;

      lastOutSpeedx1000 = outSpeedX1000;  // save latest outspeed, so next iteratin of this function can have only the defined delta
      outSpeed = outSpeedX1000 / 1000;
    }
  }

  return outSpeed;
}


/*
 if the potentiometer is <deadband[%] or> 100%-deadBand
 then it returns respectively 0% or 100%
 if it is any value in the middle , retrn the value, but again scaled form 1% to 99%
*/
uint16_t addDeadBand(uint16_t val_pct, uint16_t deadBand_pct) {
  uint16_t retVal = 0;

  if (val_pct < deadBand_pct) {
    retVal = 0;
  } else if (val_pct > (THROTTLE_NORMALIZED - deadBand_pct)) {
    retVal = THROTTLE_NORMALIZED;
  } else {
    retVal = (THROTTLE_NORMALIZED * (val_pct - deadBand_pct)) / (THROTTLE_NORMALIZED - 2 * deadBand_pct);
  }

  return retVal;
}

/* takes an Input throttle from 0 to THROTTLE_NORMALIZED and it then it applies a min and max and a broken line curve
  Returns a 0% to 100% value */
uint16_t throttleCurve(uint16_t inputThrottle) {
  uint16_t outputSpeed = 0;
  uint32_t thrOutIntersection;  // curve for output level coordinate of the intersection (breakpoint)

  // calculate the thorttle output breakpoint (the input is fixed to 50%)
  thrOutIntersection = storedVar.carParam[gCarSel].minSpeed + ((uint32_t)storedVar.carParam[gCarSel].maxSpeed - (uint32_t)storedVar.carParam[gCarSel].minSpeed) * ((uint32_t)storedVar.carParam[gCarSel].throttleSetPoint.outSpeed) / 100;

  if (inputThrottle == 0) {
    outputSpeed = 0;
  } else if (inputThrottle <= THR_CRV_IN_LEVEL_JOINT) {
    outputSpeed = map(inputThrottle, 0, THR_CRV_IN_LEVEL_JOINT, storedVar.carParam[gCarSel].minSpeed, thrOutIntersection);
  } else {
    outputSpeed = map(inputThrottle, THR_CRV_IN_LEVEL_JOINT, THROTTLE_NORMALIZED, thrOutIntersection, storedVar.carParam[gCarSel].maxSpeed);
  }

  return outputSpeed;
}

/*
 call this when calibrating the thorsttle
 Check if the parameter adcRaw is bigger/smaller than the stored max/min values
*/
void throttleCalibration(int16_t adcRaw) {
  if (storedVar.maxTrigRaw < adcRaw)
    storedVar.maxTrigRaw = adcRaw;
  if (storedVar.minTrigRaw > adcRaw)
    storedVar.minTrigRaw = adcRaw;
}


menu_state_enum rotary_onButtonClick(menu_state_enum currMenuState) {
  static unsigned long lastTimePressed = 0;
  static uint16_t selectedParamMaxValue = 100;
  static uint16_t selectedParamMinValue = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 200) {
    return currMenuState;
  }
  lastTimePressed = millis();

  if (currMenuState == ITEM_SELECTION) {
    /* If an item has no callback, just go in menu state VALUE_SELECTION */
    if (mainMenu.item[encoderMainSelector - 1].callback == ITEM_NO_CALLBACK) {
      rotaryEncoder.setAcceleration(SEL_ACCELERATION); /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
      /* value is a generic pointer to void, so cast to uint16_t pointer */
      encoderSelectedValuePtr = (uint16_t *)mainMenu.item[encoderMainSelector - 1].value;
      selectedParamMaxValue = mainMenu.item[encoderMainSelector - 1].maxValue;
      selectedParamMinValue = mainMenu.item[encoderMainSelector - 1].minValue;
      rotaryEncoder.setBoundaries(selectedParamMinValue, selectedParamMaxValue, false);
      rotaryEncoder.reset(*encoderSelectedValuePtr);
      escVar.encoderPos = *encoderSelectedValuePtr;
      return VALUE_SELECTION;
    }
    /* if an item has a callback, execute it, then return to ITEM SELECTION */
    else {
      mainMenu.item[encoderMainSelector - 1].callback();
      return ITEM_SELECTION;
    }
  } else {
    rotaryEncoder.setAcceleration(MENU_ACCELERATION); /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
    rotaryEncoder.setBoundaries(1, MENU_ITEMS_COUNT, false);
    rotaryEncoder.reset(encoderMainSelector);
    escVar.encoderPos = encoderMainSelector;
    /* save modified values to EEPROM */
    saveEEPROM(storedVar); /* Save is only done on SAVE option */
    return ITEM_SELECTION;
  }
}

void showCarSelection() {
  /* "Frame" indicates which items are currently displayed */
  static uint16_t frameUpper = 1;
  static uint16_t frameLower = carMenu.lines;

  /* Clear screen */
  obdFill(&obd, OBD_WHITE, 1);

  /* Set encoder to car selection parameter */
  rotaryEncoder.setAcceleration(MENU_ACCELERATION); /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
  rotaryEncoder.setBoundaries(0, CAR_MAX_COUNT - 1, false);
  rotaryEncoder.reset(storedVar.carSelNumber);

  /* Exit car selection when encoder is clicked */
  while (!rotaryEncoder.isEncoderButtonClicked()) {
    /* Get encoder value if changed */
    storedVar.carSelNumber = rotaryEncoder.encoderChanged() ? rotaryEncoder.readEncoder() : storedVar.carSelNumber;
    /* In encoder move out of frame, adjust frame */
    if (storedVar.carSelNumber > frameLower) {
      frameLower = storedVar.carSelNumber;
      frameUpper = frameLower - carMenu.lines + 1;
      obdFill(&obd, OBD_WHITE, 1);
    } else if (storedVar.carSelNumber < frameUpper) {
      frameUpper = storedVar.carSelNumber;
      frameLower = frameUpper + carMenu.lines - 1;
      obdFill(&obd, OBD_WHITE, 1);
    }

    /* Print car menu */
    for (uint8_t i = 0; i < carMenu.lines; i++) {
      obdWriteString(&obd, 0, 0, i * HEIGHT12x16, carMenu.item[frameUpper + i].name, FONT_12x16, (storedVar.carSelNumber - frameUpper == i) ? OBD_WHITE : OBD_BLACK, 1);
      if (carMenu.item[frameUpper + i].value != ITEM_NO_VALUE) {
        /* value is a generic pointer to void, so first cast to uint16_t pointer, then take the pointed value */
        sprintf(msgStr, "%2d", *(uint16_t *)(carMenu.item[frameUpper + i].value));
        obdWriteString(&obd, 0, OLED_WIDTH - 24, i * HEIGHT12x16, msgStr, FONT_12x16, OBD_BLACK, 1);
      }
    }

    /* Print "-SELECT THE CAR-" on the bottom of the screen */
    obdWriteString(&obd, 0, 16, OLED_HEIGHT - HEIGHT8x8, (char *)"-SELECT THE CAR-", FONT_6x8, OBD_WHITE, 1);
  }

  /* Reset encoder and clear is done in caller routine */
  return;
}

void showSaveCar() {
  saveEEPROM(storedVar);
  /* Reset encoder */
  rotaryEncoder.setAcceleration(MENU_ACCELERATION); /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
  rotaryEncoder.setBoundaries(1, MENU_ITEMS_COUNT, false);
  rotaryEncoder.reset(encoderMainSelector);
  escVar.encoderPos = encoderMainSelector;
  /* Clear screen */
  obdFill(&obd, OBD_WHITE, 1);
  return;
}


void showSelectRenameCar() {

  /* Trigger reading stops, so stop the motor */
  /* Set trigRaw to max throttle if throttle is reversed, set to min throttle otherwise */
  escVar.trigRaw = THROTTLE_REV ? storedVar.maxTrigRaw : storedVar.minTrigRaw;

  uint16_t selectedOption = 0;
  /* Clear screen */
  obdFill(&obd, OBD_WHITE, 1);

  /* Set encoder to selection parameter */
  rotaryEncoder.setAcceleration(MENU_ACCELERATION); /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
  rotaryEncoder.setBoundaries(0, 1, false);
  rotaryEncoder.reset(selectedOption);

  /* Print the "SELECT AN OPTION" */
  obdWriteString(&obd, 0, 16, OLED_HEIGHT - HEIGHT8x8, (char *)"-PICK AN OPTION-", FONT_6x8, OBD_WHITE, 1);

  /* Exit car selection when encoder is clicked */
  while (!rotaryEncoder.isEncoderButtonClicked()) {
    /* Get encoder value if changed */
    selectedOption = rotaryEncoder.encoderChanged() ? rotaryEncoder.readEncoder() : selectedOption;
    /* Print the two options */
    obdWriteString(&obd, 0, 0, 0 * HEIGHT12x16, (char *)"SELECT", FONT_12x16, (selectedOption == CAR_OPTION_SELECT) ? OBD_WHITE : OBD_BLACK, 1);
    obdWriteString(&obd, 0, 0, 1 * HEIGHT12x16, (char *)"RENAME", FONT_12x16, (selectedOption == CAR_OPTION_RENAME) ? OBD_WHITE : OBD_BLACK, 1);
    //obdWriteString(&obd, 0, 0, 2 * HEIGHT12x16, (char *)"SAVE", FONT_12x16, (selectedOption == CAR_OPTION_SAVE) ? OBD_WHITE : OBD_BLACK, 1);
  }

  /* If RENAME option was selected, go to renameCar routine */
  if (selectedOption == CAR_OPTION_RENAME) {
    showRenameCar();
    saveEEPROM(storedVar);
  }
  /* If SELECT option was selected, go to showCarSelection routine */
  else if (selectedOption == CAR_OPTION_SELECT) {
    showCarSelection();
    saveEEPROM(storedVar);
  }

  /* Reset encoder */
  rotaryEncoder.setAcceleration(MENU_ACCELERATION); /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
  rotaryEncoder.setBoundaries(1, MENU_ITEMS_COUNT, false);
  rotaryEncoder.reset(encoderMainSelector);
  escVar.encoderPos = encoderMainSelector;
  /* Clear screen */
  obdFill(&obd, OBD_WHITE, 1);
  return;
}

void showRenameCar() {
  uint16_t selectedChar = RENAME_CAR_MIN_ASCII; /* the selected char: starts from 33 (ASCII for !) to 122 (ASCII for z)*/
  uint16_t selectedOption = 0;                  /* The selected option, could be one of the char of the name (0 : CAR_NAME_MAX_SIZE - 2) or the confirm option (CAR_NAME_MAX_SIZE - 1)*/
  /* Remember that CAR_NAME_MAX_SIZE is 6 because the last char is the terminator */
  char tmpName[CAR_NAME_MAX_SIZE];
  uint16_t mode = RENAME_CAR_SELECT_OPTION_MODE;
  sprintf(tmpName, "%s", storedVar.carParam[storedVar.carSelNumber].carName);

  /* Clear screen */
  obdFill(&obd, OBD_WHITE, 1);

  /* Set encoder to selection parameter */
  rotaryEncoder.setAcceleration(MENU_ACCELERATION); /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
  rotaryEncoder.setBoundaries(0, CAR_NAME_MAX_SIZE - 1, false);
  rotaryEncoder.reset(selectedOption);

  /* Print "-RENAME THE CAR-"  and "-CLICK OK TO CONFIRM" */
  obdWriteString(&obd, 0, 16, 0, (char *)"-RENAME THE CAR-", FONT_6x8, OBD_WHITE, 1);
  obdWriteString(&obd, 0, 1, OLED_HEIGHT - HEIGHT8x8, (char *)"-CLICK OK TO CONFIRM-", FONT_6x8, OBD_WHITE, 1);

  /* Draw the right arrow */
  for (uint8_t j = 0; j < 8; j++) {
    obdDrawLine(&obd, 80 + j, 16 + j, 80 + j, 30 - j, OBD_BLACK, 1);
  }
  obdDrawLine(&obd, 72, 22, 80, 22, OBD_BLACK, 1);
  obdDrawLine(&obd, 72, 23, 80, 23, OBD_BLACK, 1);
  obdDrawLine(&obd, 72, 24, 80, 24, OBD_BLACK, 1);

  /* Exit car renaming when encoder is clicked when CONFIRM is selected */
  while (1) {
    /* Get encoder value if changed */
    /* Change selectedOption if in RENAME_CAR_SELECT_OPTION_MODE */
    if (mode == RENAME_CAR_SELECT_OPTION_MODE) {
      selectedOption = rotaryEncoder.encoderChanged() ? rotaryEncoder.readEncoder() : selectedOption;
    }
    /* Change selectedChar if in RENAME_CAR_SELECT_CHAR_MODE */
    if (mode == RENAME_CAR_SELECT_CHAR_MODE) {
      selectedChar = rotaryEncoder.encoderChanged() ? rotaryEncoder.readEncoder() : selectedChar;
      tmpName[selectedOption] = (char)selectedChar;

      /* Draw the upward and downward arrows on the selected char to indicate that it can be changed */
      for (uint8_t j = 0; j < 6; j++) {
        obdDrawLine(&obd, 1 + j + (selectedOption * 12), 14 - j, 11 - j + (selectedOption * 12), 14 - j, OBD_BLACK, 1);
        obdDrawLine(&obd, 1 + j + (selectedOption * 12), 33 + j, 11 - j + (selectedOption * 12), 33 + j, OBD_BLACK, 1);
      }
    }

    /* Print each Char of the name, only the selected one is highlighted */
    for (uint8_t i = 0; i < CAR_NAME_MAX_SIZE - 1; i++) {
      sprintf(msgStr, "%c", tmpName[i]);
      obdWriteString(&obd, 0, 0 + (i * 12), 22, msgStr, FONT_12x16, (selectedOption == i) ? OBD_WHITE : OBD_BLACK, 1);
    }

    /* Print the confirm button */
    obdWriteString(&obd, 0, OLED_WIDTH - 24, 22, (char *)"OK", FONT_12x16, (selectedOption == CAR_NAME_MAX_SIZE - 1) ? OBD_WHITE : OBD_BLACK, 1);

    /* If encoder button is clicked */
    if (rotaryEncoder.isEncoderButtonClicked()) {
      /* Exit renameCar routing when CONFIRM is selected */
      if (selectedOption == CAR_NAME_MAX_SIZE - 1) {
        /* Change the name of the Car */
        sprintf(storedVar.carParam[storedVar.carSelNumber].carName, "%s", tmpName);
        /* Menu variables are initialized in main loop */
        return;
      }

      /* If in RENAME_CAR_SELECT_OPTION_MODE */
      if (mode == RENAME_CAR_SELECT_OPTION_MODE) {
        /* switch mode */
        mode = RENAME_CAR_SELECT_CHAR_MODE;
        /* Reset encode */
        rotaryEncoder.setBoundaries(RENAME_CAR_MIN_ASCII, RENAME_CAR_MAX_ASCII, false);
        rotaryEncoder.reset((uint16_t)tmpName[selectedOption]);
        selectedChar = (uint16_t)tmpName[selectedOption];
      }
      /* If in RENAME_CAR_SELECT_CHAR_MODE */
      else {
        /* switch mode */
        mode = RENAME_CAR_SELECT_OPTION_MODE;
        /* Reset encode */
        rotaryEncoder.setBoundaries(0, CAR_NAME_MAX_SIZE - 1, false);
        rotaryEncoder.reset(selectedOption);
        /* Cancel the upward and downward arrows (draw them black) */
        for (uint8_t j = 0; j < 6; j++) {
          obdDrawLine(&obd, 1 + j + (selectedOption * 12), 14 - j, 11 - j + (selectedOption * 12), 14 - j, OBD_WHITE, 1);
          obdDrawLine(&obd, 1 + j + (selectedOption * 12), 33 + j, 11 - j + (selectedOption * 12), 33 + j, OBD_WHITE, 1);
        }
      }
    }
  }
}

void showCurveSelection() {
  uint16_t thrOutIntersection;
  uint16_t prevTrigger = escVar.outSpdSetPerc;
  uint16_t inThrottlePerc = (storedVar.carParam[gCarSel].throttleSetPoint.inThrottle * 100) / THROTTLE_NORMALIZED;  //Take inThrottle (from 0 to THROTTLE NORMALIZED) and convert it in a 0% to 100% value
  /* Clear screen and draw x and y axis */
  obdFill(&obd, OBD_WHITE, 1);
  obdDrawLine(&obd, 25, 0, 25, 50, OBD_BLACK, 1);
  obdDrawLine(&obd, 25, 50, 125, 50, OBD_BLACK, 1);
  /* Write the 100%, 0%, 50% and MIN and MAXpoints labels */
  obdWriteString(&obd, 0, 0, 0, (char *)"100%", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, 0, 58, (char *)"  0%", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, 104, 58, (char *)"100%", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, 0, map(storedVar.carParam[gCarSel].minSpeed, 0, 100, 50, 8), (char *)"MIN", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, 28, 50 - (storedVar.carParam[gCarSel].maxSpeed / 2), (char *)"MAX", FONT_6x8, OBD_BLACK, 1);
  obdWriteString(&obd, 0, 64, 58, (char *)"50%", FONT_6x8, OBD_BLACK, 1);

  /* Draw axis ticks at 50%, MIN SPEED and MAX speed points*/
  obdSetPixel(&obd, 24, 50 - (storedVar.carParam[gCarSel].minSpeed / 2), OBD_BLACK, 1);
  obdSetPixel(&obd, 23, 50 - (storedVar.carParam[gCarSel].minSpeed / 2), OBD_BLACK, 1);
  obdSetPixel(&obd, 25 + inThrottlePerc, 51, OBD_BLACK, 1);
  obdSetPixel(&obd, 25 + inThrottlePerc, 52, OBD_BLACK, 1);
  obdSetPixel(&obd, 26, 50 - (storedVar.carParam[gCarSel].maxSpeed / 2), OBD_BLACK, 1);
  obdSetPixel(&obd, 27, 50 - (storedVar.carParam[gCarSel].maxSpeed / 2), OBD_BLACK, 1);

  /* Set encoder to curve parameters */
  rotaryEncoder.setAcceleration(SEL_ACCELERATION); /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
  rotaryEncoder.setBoundaries(THR_SETP_MIN_VALUE, THR_SETP_MAX_VALUE, false);
  rotaryEncoder.reset(storedVar.carParam[gCarSel].throttleSetPoint.outSpeed);

  /* Calculate intersection point (as in throttleCurve() function) */
  thrOutIntersection = storedVar.carParam[gCarSel].minSpeed + ((uint32_t)storedVar.carParam[gCarSel].maxSpeed - (uint32_t)storedVar.carParam[gCarSel].minSpeed) * ((uint32_t)storedVar.carParam[gCarSel].throttleSetPoint.outSpeed) / 100;

  /* Draw Line from MIN SPEED, to middle point */
  obdDrawLine(&obd, 25, 50 - (storedVar.carParam[gCarSel].minSpeed / 2), 25 + inThrottlePerc, map(thrOutIntersection, 0, 100, 50, 0), OBD_BLACK, 1);
  /* Draw Line from middle point to 100% */
  obdDrawLine(&obd, 25 + inThrottlePerc, map(thrOutIntersection, 0, 100, 50, 0), 125, map(storedVar.carParam[gCarSel].maxSpeed, 0, 100, 50, 0), OBD_BLACK, 1);

  /* Write the CURV value */
  sprintf(msgStr, "%3d%c", storedVar.carParam[gCarSel].throttleSetPoint.outSpeed, '%');
  obdWriteString(&obd, 0, OLED_WIDTH - 48, 34, msgStr, FONT_12x16, OBD_BLACK, 1);

  /* Write the trigger value */
  sprintf(msgStr, "%3d%c", prevTrigger, '%');
  obdWriteString(&obd, 0, OLED_WIDTH - 32, 26, msgStr, FONT_8x8, OBD_BLACK, 1);

  /* Exit curve function when encoder is clicked */
  while (!rotaryEncoder.isEncoderButtonClicked()) {

    /* Write the trigger value only if it changed */
    if (escVar.outSpdSetPerc != prevTrigger) {
      /* Update trigger */
      prevTrigger = escVar.outSpdSetPerc;

      /* Write the trigger value */
      sprintf(msgStr, "%3d%c", prevTrigger, '%');
      obdWriteString(&obd, 0, OLED_WIDTH - 32, 26, msgStr, FONT_8x8, OBD_BLACK, 1);
    }

    /* Get encoder position if it was changed and correct the new lines*/
    if (rotaryEncoder.encoderChanged()) {
      /* Cancel the old lines (draw them in black) */
      obdDrawLine(&obd, 25, 50 - (storedVar.carParam[gCarSel].minSpeed / 2), 25 + inThrottlePerc, map(thrOutIntersection, 0, 100, 50, 0), OBD_WHITE, 1);
      obdDrawLine(&obd, 25 + inThrottlePerc, map(thrOutIntersection, 0, 100, 50, 0), 125, map(storedVar.carParam[gCarSel].maxSpeed, 0, 100, 50, 0), OBD_WHITE, 1);
      /* Update the value of the setpoint */
      storedVar.carParam[gCarSel].throttleSetPoint.outSpeed = rotaryEncoder.readEncoder();
      sprintf(msgStr, "%3d%c", storedVar.carParam[gCarSel].throttleSetPoint.outSpeed, '%');
      thrOutIntersection = storedVar.carParam[gCarSel].minSpeed + ((uint32_t)storedVar.carParam[gCarSel].maxSpeed - (uint32_t)storedVar.carParam[gCarSel].minSpeed) * ((uint32_t)storedVar.carParam[gCarSel].throttleSetPoint.outSpeed) / 100;
      obdWriteString(&obd, 0, OLED_WIDTH - 48, 34, msgStr, FONT_12x16, OBD_BLACK, 1);
      /* Draw the new lines */
      obdDrawLine(&obd, 25, 50 - (storedVar.carParam[gCarSel].minSpeed / 2), 25 + inThrottlePerc, map(thrOutIntersection, 0, 100, 50, 0), OBD_BLACK, 1);
      obdDrawLine(&obd, 25 + inThrottlePerc, map(thrOutIntersection, 0, 100, 50, 0), 125, map(storedVar.carParam[gCarSel].maxSpeed, 0, 100, 50, 0), OBD_BLACK, 1);
    } else {
      /* Needed to service the watchdog, to prevent CPU reset */
      vTaskDelay(10);
    }
  }

  /* Reset encoder */
  rotaryEncoder.setAcceleration(MENU_ACCELERATION); /* Larger number = more accelearation; 0 or 1 means disabled acceleration */
  rotaryEncoder.setBoundaries(1, MENU_ITEMS_COUNT, false);
  rotaryEncoder.reset(encoderMainSelector);
  escVar.encoderPos = encoderMainSelector;
  /* save modified values to EEPROM */
  //saveEEPROM(storedVar); /* Save is only done on SAVE option */
  /* Clear screen */
  obdFill(&obd, OBD_WHITE, 1);

  return;
}

#define TRIG_AVG_TIME_ms 30
#define TRIG_AVG_COUNT (TRIG_AVG_TIME_ms * 1000 / (ESC_PERIOD_US))

// takes ADC values for a potentiometer, where the max and min has been recorded
// and returns from 0 to THROTTLE_NORMALIZED
uint16_t normalizeAndClamp(uint16_t raw, uint16_t minIn, uint16_t maxIn, uint16_t normalizedMax, bool reverse) {
  uint16_t pct = 0;  // o to THROTTLE_NORMALIZED

  if (maxIn == minIn)  // avoid division by 0
    return 0;

  raw = constrain(raw, minIn, maxIn);

  if (reverse == true)  // if throttle is reversed(it goes to low values when pressed)
    raw = abs(maxIn - raw);
  else
    raw = abs(raw - minIn);

  pct = ((uint32_t)raw * normalizedMax) / (maxIn - minIn);

  return pct;
}


int16_t computetrigDerivGPT(uint16_t current_value) {
  static uint16_t value_buffer[TRIG_AVG_COUNT] = { 0 };  // Buffer to store the last 100 values
  static uint8_t buffer_index = 0;                       // Index for the circular buffer
  static uint32_t sum = 0;                               // Sum of the values in the buffer
  static uint8_t is_first_call = 1;                      // Flag to check if it's the first call
  int16_t derivative;

  if (is_first_call) {
    // Initialize the buffer and sum on the first call
    for (uint8_t i = 0; i < TRIG_AVG_COUNT; i++) {
      value_buffer[i] = current_value;
      sum += current_value;
    }
    is_first_call = 0;
  } else {
    // Update the sum by subtracting the old value and adding the new value
    sum -= value_buffer[buffer_index];
    sum += current_value;

    // Update the buffer with the current value
    value_buffer[buffer_index] = current_value;

    // Increment the buffer index and wrap around if necessary
    buffer_index = (buffer_index + 1) % TRIG_AVG_COUNT;
  }

  // Calculate the rolling average
  uint16_t rolling_average = sum / TRIG_AVG_COUNT;

  // Calculate the derivative as the difference between the current value and the rolling average
  derivative = (int16_t)(current_value - rolling_average);

  return derivative;
}


uint16_t saturateParamValue(uint16_t paramValue, uint16_t minValue, uint16_t maxValue) {
  uint16_t retValue = paramValue;

  if (paramValue > maxValue) {
    retValue = maxValue;
  } else if (paramValue < minValue) {
    retValue = minValue;
  }

  return retValue;
}

void saveEEPROM(EEPROM_stored_var_type toSave) {
  //timerStop(timer);                         /* Stop the timer */
  pref.begin("stored_var", false);                      /* Open the "stored" namespace in read/write mode */
  pref.putBytes("user_param", &toSave, sizeof(toSave)); /* Put the value of the stored user_param */
  pref.end();
  //timerStart(timer);                        /* Resume the timer */
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