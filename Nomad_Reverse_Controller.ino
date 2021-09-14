
/*
Reverse Bucket Controller with a Potentiometer as position sensor

Externally the unit has 3 Iluminated Buttons
  IO            Buttons             Indicator
  Bucket DN     Reverse             In Reverse;- Bucket Fully Down
  Bucket UP     Forward             Going Forward;- Bucket UP
                Neutral             In Neutral; - Bucket in programmed intermediate position
*/
#define VERSION  "1.01k"
#include <EEPROM.h>


#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#define eeNeutralPositionPointer  0 //pointer stored here to currently used NeutralPosition value in memory
#define eeNeutralPositionStoreStart  1
#define eeNeutralPositionStoreEnd  2

#define DebugLCD true 

enum StateMachine_enum {UP=1, MOVE_TO_NEUTRAL, MOV_TO_UP, MOV_TO_DN, NEUTRAL, DOWN, LEARNING_TRANSITION, LEARNING};
enum Buttons_enum {NO_PUSHED=1, UP_PUSHED, DOWN_PUSHED, NEUTRAL_PUSHED, UP_DN_PUSHED};

void state_machine_run(uint8_t read_Buttons);
void solenoids_Up();
void solenoid_Move_To_Neutral();
void solenoid_Move_To_Up();
void solenoid_Move_To_Down();
void solenoid_Neutral();
void solenoid_Down();
uint8_t read_Buttons();

uint8_t state = MOV_TO_UP; // on startup move bucket to full UP position.
uint8_t statePrevious = MOV_TO_UP ; //varible to store previoue state while learning

void TurnOnForwardLED();

void bucket_Solenoid_UP();
void bucket_Solenoids_DOWN();
void bucket_Solenoids_OFF();


int bucketPositionPin = A3;        // potentiometer is connected to analog 4 pin
int reverseSolenoidPin = 4;        // Pin to drive bucket to Reverse (Down)position
int forwardSolenoidPin = 6;        // Pin to drive bucket to Forward (Up) position
int buttonReversePin = 10;  // input pin
int buttonForwardPin = 12;
int buttonNeutralPin = 11;
int ledForwardPin = 9;
int ledNeutralPin = 8;
int ledReversePin = 7;

static uint8_t toggleLED=0; // toggle the LED
static uint32_t ledtime= millis();
int AliveLED = 13;  // LED we toggle to show processor is running
int16_t aliveFlashTime = 250; // time Alive light toggles in ms
int16_t timeOutFlashTime = 100; // time Alive light toggles in ms if a movement has timmed out before completion


unsigned long learningButtonIgnoreTime; // storage for button debounce timer
int16_t learningStateWaitdelay = 2000; // when all buttons pushed wait this time before processing anymore buttons. 


int16_t bucketMoveTimeoutValue = 6000; // max time motor can stay on looking for Target bucket position.
static uint32_t bucketMoveTimer = millis();



int potBucketValue = 0;           // variable used to store the value coming from the sensor
int percentDown = 0;            // variable used to store the percentage value

int revPinDeb = 0;
int fwdPinDeb = 0;
int nwtPinDeb = 0;
int debounceVal = 3;

int posNeutralBucket = 49;
int posUpBucket = 10;
int posDownBucket = 99;
int testDelay = 200;

bool buttonForwardState = false;
bool buttonReverseState = false;
bool buttonNeutralState = false;
bool newStateFlag;
bool solenoidsMovementTimedOut = false;

bool buttFwdPrevState = false;
bool buttRevPrevState = false;
bool buttNeutPrevState = false;

bool bucketMovingUP = false;
bool bucketMovingDN = false;
bool newButtonState = false;
bool statusLEDflash = false;

int neutralBucketEeAddress = 0;
//int currentSystemState = 0;


void setup(){

  // pinMode(redPin, OUTPUT); // red LED is as an output
  // pinMode(greenPin, OUTPUT); // green LED is as an output
 Serial.begin(115200);   // Started serial communication at baud rate of 250000.

  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Nomad Jets");
  lcd.setCursor(1,1);
  lcd.print(VERSION);



 neutralBucketEeAddress = EEPROM.read(eeNeutralPositionPointer); // get where current neutral biucket position is stored.

  if ( (neutralBucketEeAddress < eeNeutralPositionStoreStart) && (neutralBucketEeAddress > eeNeutralPositionStoreEnd) ){  // check that the pointer for the storage location is valid we alternate between location 0 and 1
    neutralBucketEeAddress = eeNeutralPositionStoreStart;
    EEPROM.update(eeNeutralPositionPointer,neutralBucketEeAddress);
    delay(100);
  }

  posNeutralBucket = EEPROM.read(neutralBucketEeAddress); // get stored Neutral bucket position
  if (posNeutralBucket==0){  // saved value possibly invalid
    posNeutralBucket=49;
    EEPROM.update(neutralBucketEeAddress, posNeutralBucket);
    delay(100);
    }




 pinMode(buttonReversePin, INPUT_PULLUP);  // input pin
 pinMode(buttonForwardPin, INPUT_PULLUP);
 pinMode(buttonNeutralPin, INPUT_PULLUP);


 pinMode(reverseSolenoidPin, OUTPUT);        // Pin to drive bucket to Reverse position
 pinMode(forwardSolenoidPin, OUTPUT);        // Pin to drive bucket to Reverse position
 pinMode(ledForwardPin, OUTPUT);
 pinMode(ledNeutralPin, OUTPUT);
 pinMode(ledReversePin, OUTPUT); 
 pinMode(AliveLED, OUTPUT); // Alive Pin

 
 // Note: analog pins are automatically set as inputs

 test_Button_LEDs(); //flash Button LED's to  for startup self test

    lcd.setCursor(0,0);
    lcd.print("B:              ");

}
//  -------------------------------------

void loop(){

  updateToggleLEDAlive();   // LED Flash and timer toggle update
    
  //Serial.println(state);                // Printing current State 
                                         
  getBucketPosition();


    lcd.setCursor(2,0);
    lcd.print(percentDown);
    lcd.print(" :");
    lcd.print(potBucketValue);
    lcd.print("    ");

 
  //Serial.print("Gate Position is: ");   
  //Serial.println(percentDown);   

  //  Serial.print("   Current State is: ");   
  //  Serial.println(state);

                                 
  state_machine_run(read_Buttons());
  /*
  Serial.print("bFWD: ");  
  Serial.print(buttonForwardState);                              
  Serial.print("  bNEU: ");
  Serial.print(buttonNeutralState);    
  Serial.print("  bREV: ");
  Serial.println(buttonReverseState);    
*/
//  delay(100);
if(bucketMovingUP||bucketMovingDN){ // while any solenoid is ON
  if((millis()-bucketMoveTimer)>bucketMoveTimeoutValue){  //Check the timeout
    bucket_Solenoids_OFF();  // turn off solenoids
    lcd.setCursor(15,0);
    lcd.print("T");  // indicate output timmed OFF
    solenoidsMovementTimedOut = true ;
        
  }
}

}


void state_machine_run(uint8_t read_Buttons)
{
  newButtonState = false;  
  switch(state)
  {
    case UP:
    
//      Serial.print("State: UP ");
        
      solenoids_Up();
      
          
      break;
      
    case MOVE_TO_NEUTRAL:
//      Serial.print("State: MOV_DN_N ");
      solenoid_Move_To_Neutral();



      break;


        
    case MOV_TO_UP:
//      Serial.print("State: MOV_TO_UP ");
      solenoid_Move_To_Up();

     
      break;
     
    case MOV_TO_DN:
//      Serial.print("State: MOV_TO_DN ");
      solenoid_Move_To_Down();

      break;
 
    case NEUTRAL:
//      Serial.print("State: NEUTRAL ");
      
     
      solenoid_Neutral();

     
      break;
     
    case DOWN:
//      Serial.print("State: DOWN ");
 
          
        solenoid_Down();
      

     
      break;
     

    case LEARNING_TRANSITION:
//      Serial.print("State: LEARNING_TRANSITION ");
      solenoid_Neutral_Learning();


      break;
    

    case LEARNING:
//      Serial.print("State: LEARNING ");
     
      learningNeutralPos();
      ProcessLearnButtonsPressed();

     
      break;
  }
  
}  

// , , , , , , , 
//--------------------------------------------------------------------------------------------------- 

// Status LED flash timer 
// Clear edge flags - they should have been processed after last scan.

//--------------------------------------------------------------------------------------------------- 
void getBucketPosition()
{
  potBucketValue = analogRead(bucketPositionPin); // read the value from the potentiometer and assign the name potValue
  percentDown = map(potBucketValue, 5, 809, 0, 100); // convert potentiometer reading to a percentage
//  Serial.println(percentDown);                // Printing current BUCKET POSITION 
}

void solenoids_Up()
{
  TurnOnForwardLED();

  whichButtonsPressed();
}

void solenoid_Move_To_Neutral()
{
  FlashLED(ledNeutralPin);
  whichButtonsPressed();
  if (newButtonState==false){

    if (percentDown == posNeutralBucket){
      bucket_Solenoids_OFF();
      state = NEUTRAL;
      Serial.println("State: NEUTRAL ");
            
      lcd.setCursor(1,1);
      lcd.print("State: Neutral  ");
    

    
    }else if(((percentDown < posNeutralBucket) && !bucketMovingDN && !solenoidsMovementTimedOut) == true ){
      bucket_Solenoids_DOWN();

    }else if (((percentDown > posNeutralBucket) && !bucketMovingUP && !solenoidsMovementTimedOut) == true) {
      bucket_Solenoid_UP();
    }
  }  
}


void solenoid_Move_To_Up()
{

  FlashLED(ledForwardPin);

  //Serial.print("Gate Position is:- ");   
  //Serial.println(percentDown);
  whichButtonsPressed();
  if (newButtonState==false){   
    if (percentDown <= posUpBucket){
      Serial.print("Gate Position fully UP ");
      bucket_Solenoids_OFF(); // turn off all solenoids
      TurnOnForwardLED();//Indicate we can now go forward
      state = UP;
      Serial.println("State: UP ");
      lcd.setCursor(1,1);
      lcd.print("State: UP       ");

    }  
  }  
}

void solenoid_Move_To_Down()
{
  
  FlashLED(ledReversePin);
  whichButtonsPressed(); // check for button presses
  if (newButtonState==false){
    if(percentDown >= posDownBucket){
      bucket_Solenoids_OFF();
      TurnOnReverseLED();
      state = DOWN;
      Serial.println("State: DOWN ");
      lcd.setCursor(1,1);
      lcd.print("State: DOWN     ");

    }
  }

}




void solenoid_Neutral()
{
  TurnOnNeutralLED();
  whichButtonsPressed();
}

void solenoid_Down()
{
  TurnOnReverseLED();
  whichButtonsPressed();
}

void solenoid_Neutral_Learning()
{
  //FlashLED(ledForwardPin);
  //FlashLED(ledReversePin);
  FlashLED(ledNeutralPin);
  //FlashAllLED();

  if ( (millis()-learningButtonIgnoreTime) > learningStateWaitdelay) {
    TurnOffAllLEDs();
    state = LEARNING;
    Serial.println("State: LEARNING ");
    lcd.setCursor(1,1);
    lcd.print("Learning Neutral");
    
  }



}

void learningNeutralPos()
{

  FlashAllLED();
  //FlashLED(ledForwardPin);
  //FlashLED(ledReversePin);
  // whichButtonsPressed();
}

uint8_t read_Buttons()
{
bool thisPinScan = false;
  thisPinScan = !digitalRead(buttonForwardPin); // read pin and invert logic as pin is pulled High button pulls to ground
 // digitalWrite(ledForwardPin, thisPinScan);
//  Serial.println(thisPinScan);  
  if (buttFwdPrevState == thisPinScan){
    fwdPinDeb++;
//    Serial.println(fwdPinDeb);  
    if (fwdPinDeb==debounceVal){
      buttonForwardState = thisPinScan; //New State
      
      
    }else if (fwdPinDeb > debounceVal){
      fwdPinDeb=debounceVal; // stop the debounce counter overflowing 
      
    }
          
  } else {
    buttFwdPrevState = thisPinScan;
    fwdPinDeb = 0;
  }

   
thisPinScan = !digitalRead(buttonReversePin); // read pin and invert logic as pin is pulled High button pulls to ground
 // digitalWrite(ledReversePin, thisPinScan);
//  Serial.println(thisPinScan);  
  if (buttRevPrevState == thisPinScan){
    revPinDeb++;
   // Serial.println(fwdPinDeb);  
    if (revPinDeb==debounceVal){
      buttonReverseState = thisPinScan; //New State
      
      
    }else if (revPinDeb > debounceVal){
      revPinDeb=debounceVal; // stop the debounce counter overflowing 
      
    }
   
  } else {
    buttRevPrevState = thisPinScan;
    revPinDeb = 0;
  }



/*
   buttonNeutralPin

*/
thisPinScan = !digitalRead(buttonNeutralPin); // read pin and invert logic as pin is pulled High button pulls to ground
 // digitalWrite(ledReversePin, thisPinScan);
 // Serial.println(thisPinScan);  
  if (buttNeutPrevState == thisPinScan){
    nwtPinDeb++;
   // Serial.println(fwdPinDeb);  
    if (nwtPinDeb==debounceVal){
      buttonNeutralState = thisPinScan; //New State
      
      
    }else if (nwtPinDeb > debounceVal){
      nwtPinDeb=debounceVal; // stop the debounce counter overflowing 
      
    }
   
  } else {
    buttNeutPrevState = thisPinScan;
    nwtPinDeb = 0;
  }



}


void TurnOnForwardLED()
{
 
    digitalWrite(ledReversePin, LOW);
    digitalWrite(ledNeutralPin, LOW);
    digitalWrite(ledForwardPin, HIGH);
    Serial.println("Forward LED - ON");  
  
}

void TurnOnReverseLED()
{
 
    digitalWrite(ledReversePin, HIGH);
    digitalWrite(ledNeutralPin, LOW);
    digitalWrite(ledForwardPin, LOW);
    Serial.println("Reverse LED - ON");
  
}

void TurnOnNeutralLED()
{
  digitalWrite(ledReversePin, LOW);
  digitalWrite(ledNeutralPin, HIGH);
  digitalWrite(ledForwardPin, LOW);
  Serial.println("Neutral LED - ON");
    
}


void TurnOffAllLEDs()
{
  digitalWrite(ledReversePin,LOW);
  digitalWrite(ledNeutralPin, LOW);
  digitalWrite(ledForwardPin, LOW);
  Serial.println("ALL LED - OFF");
    
}

void TurnOnAllLEDs()
{
  digitalWrite(ledReversePin,HIGH);
  digitalWrite(ledNeutralPin, HIGH);
  digitalWrite(ledForwardPin, HIGH);
  Serial.println("ALL LED - ON");
    
}



void bucket_Solenoid_UP()
{
  bucketMoveTimer = millis();
  bucketMovingUP = true;
  bucketMovingDN = false;
  digitalWrite(reverseSolenoidPin, LOW);
  digitalWrite(forwardSolenoidPin, HIGH);
  Serial.println("\t ON Solenoids UP ");  
  lcd.setCursor(15,0);
  lcd.print(" ");  // Clear Output timeout indicator
  solenoidsMovementTimedOut = false ;
}
void bucket_Solenoids_DOWN()
{
  bucketMoveTimer = millis();
  bucketMovingDN = true;
  bucketMovingUP = false;
  digitalWrite(forwardSolenoidPin, LOW);
  digitalWrite(reverseSolenoidPin, HIGH);
  Serial.println("\t ON Solenoids DOWN ");    
  lcd.setCursor(15,0);
  lcd.print(" ");  // Clear Output timeout indicator
  solenoidsMovementTimedOut = false ;
}

void bucket_Solenoids_OFF()
{
  
  digitalWrite(forwardSolenoidPin, LOW);
  digitalWrite(reverseSolenoidPin, LOW);
  bucketMovingUP = false;
  bucketMovingDN = false;
  Serial.println("\t OFF Solenoids ALL ");
 

        

}

void whichButtonsPressed()
{
  if ((buttonForwardState && buttonNeutralState && buttonReverseState) == true){  // learning mode
    statePrevious = state ; // store the current state on entry to learning
    state =  LEARNING_TRANSITION;
    newButtonState=true;

    Serial.println("State: LEARNING_TRANSITION ");
    lcd.setCursor(1,1);
    lcd.print("Learning Trans  ");

    learningButtonIgnoreTime = millis();  //Store the current timer staten so we can ignore buttons
    TurnOffAllLEDs();
    bucket_Solenoids_OFF();
    
    
  }else if (((buttonNeutralState && !(buttonForwardState || buttonReverseState)) && !( (state == NEUTRAL)))== true){  //Neutral Button Pressed
  //}else if ((buttonNeutralState & (!(state == MOVE_TO_NEUTRAL) & !(state == NEUTRAL)))== true){
    state = MOVE_TO_NEUTRAL;
    newButtonState=true;
    Serial.println("State: MOV_TO_NEUTRAL pressed");
    lcd.setCursor(1,1);
    lcd.print("Moving-> Neutral");
    solenoidsMovementTimedOut = false ; //Button pressed so clear timeout and try again
    TurnOffAllLEDs();
    //bucket_Solenoids_OFF();  

  } else if ((buttonForwardState && !buttonReverseState && !(state == UP || state == MOV_TO_UP)) == true){   //FORWARD Button pressed
  //} else if ((buttonForwardState & !buttonNeutralState & !buttonReverseState) == true){
    state = MOV_TO_UP;
    newButtonState=true;
  //  if((percentDown > posUpBucket)&& solenoidsMovementTimedOut == false) ;{
    if(percentDown > posUpBucket) ;{
      bucket_Solenoid_UP();
    }
    Serial.println("State: MOV_TO_UP pressed ");
    lcd.setCursor(1,1);
    lcd.print("Moving-> UP     ");

    TurnOffAllLEDs();

  } else if (buttonReverseState && !((state == MOV_TO_DN) && (state== DOWN)) == true){   //REVERSE Button pressed
    state = MOV_TO_DN;
    newButtonState=true;
  //  if((percentDown < posDownBucket)&& solenoidsMovementTimedOut == false){
    if(percentDown < posDownBucket){
      
      bucket_Solenoids_DOWN();
    }
    Serial.println("State: MOV_TO_DN ");
    lcd.setCursor(1,1);
    lcd.print("Moving-> DOWN   ");

    TurnOffAllLEDs();
  }  
}

void ProcessLearnButtonsPressed()
{
  if ((buttonForwardState && buttonReverseState)== true){
    lcd.setCursor(1,1);
    lcd.print("Rtn 2 Prev State");
    state =  statePrevious; // restore the previous state ena exit
    newButtonState=true;
    Serial.print("Restoring Previous State before Learning ");
    Serial.println("State: ");
    Serial.println(state);
    bucket_Solenoids_OFF();
    TurnOffAllLEDs();  
    
    
  }else if (buttonNeutralState == true){ //Neutral Button Pressed in Learn mode so Save New position to EEprom
    bucket_Solenoids_OFF();
    TurnOffAllLEDs();  
    
    getBucketPosition(); // read current bucket position
    posNeutralBucket = percentDown ; //Save the New Neutral Bucket Position
    
    EEPROM.update(neutralBucketEeAddress, posNeutralBucket);
    delay(100);
    // nasty should check it has saved and retry if required.
  
    Serial.print("New Bucket Neutral Position: ");
    Serial.println(posNeutralBucket);
    state = NEUTRAL;
    lcd.setCursor(1,1);
    lcd.print("St:Neutral Saved");
    newButtonState=true;
    Serial.println("State: NEUTRAL ");
   
    

  } else if ((buttonForwardState && !bucketMovingUP) == true){  // While UP button is pressed turn on UP solenoid 
    
    lcd.setCursor(1,1);
    lcd.print("Moving UP       ");
    bucket_Solenoid_UP();


  } else if ((buttonReverseState && !bucketMovingDN) == true){  // While DN button is pressed turn on DOWN solenoid 
    lcd.setCursor(1,1);
    lcd.print("Moving DOWN     ");
    bucket_Solenoids_DOWN();


  } else if (!(buttonForwardState || buttonReverseState || buttonNeutralState) && (bucketMovingDN || bucketMovingUP) == true){
    bucket_Solenoids_OFF();
    lcd.setCursor(1,1);
    lcd.print("Learning Neutral");
  }

}



void updateToggleLEDAlive()
{ 
  int16_t activeFlashTime = aliveFlashTime ;
  if (solenoidsMovementTimedOut){  // so the LED flashes quicker if the movement timed out before reaching target position
    activeFlashTime = timeOutFlashTime ;
  }

  if ( (millis()-ledtime) > activeFlashTime) {
       ledtime = millis();

       toggleLED = ~toggleLED; // Invert
       statusLEDflash = true;
/*        Serial.print("Gate Position is: ");   
        Serial.print(percentDown);   
        Serial.print("   State: ");
        Serial.println(state);
        */

       if (toggleLED) digitalWrite(AliveLED,HIGH); else digitalWrite(AliveLED,LOW);
    }
}



void FlashLED(int LED_Pin)
{
  if(statusLEDflash == true){
    statusLEDflash = false;
    if (toggleLED) digitalWrite(LED_Pin,HIGH); else digitalWrite(LED_Pin,LOW);
        //Serial.println("flash Status LED");   

  }  
}

void FlashAllLED(){
    if(statusLEDflash == true){
      statusLEDflash = false;
      if (toggleLED){
        digitalWrite(ledForwardPin,HIGH);
        digitalWrite(ledNeutralPin,HIGH);
        digitalWrite(ledReversePin,HIGH); 
      }else{
        digitalWrite(ledForwardPin,LOW);
        digitalWrite(ledNeutralPin,LOW);
        digitalWrite(ledReversePin,LOW); 
        
      }
    }
}
  

void test_Button_LEDs()
{
TurnOnAllLEDs();
delay(testDelay);
TurnOffAllLEDs();
delay(testDelay);
TurnOnForwardLED();
delay(testDelay);
TurnOnNeutralLED();
delay(testDelay);
TurnOnReverseLED();
delay(testDelay);
TurnOffAllLEDs();
delay(testDelay);
TurnOnReverseLED();
delay(testDelay);
TurnOnNeutralLED();
delay(testDelay);
TurnOnForwardLED();
delay(testDelay);
TurnOffAllLEDs();
delay(testDelay);
TurnOnAllLEDs();
delay(testDelay);
TurnOffAllLEDs();
delay(testDelay);



bucket_Solenoid_UP();
delay(testDelay);
bucket_Solenoids_OFF();
delay(testDelay);
bucket_Solenoids_DOWN();
delay(testDelay);
bucket_Solenoids_OFF();
delay(testDelay);
bucket_Solenoid_UP();
delay(testDelay);
bucket_Solenoids_OFF();
delay(testDelay);
bucket_Solenoids_DOWN();

}
