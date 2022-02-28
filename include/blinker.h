/*
 *  by: MRO for UVSA GROUP
 *  Todos los derechos reservados
 *  Winnipeg, Manitoba. Canada. Julio 2020
 */

#define LED_ERROR 99
#define LED_OFF 0
#define LED_ON 1
#define LED_SHORT 2
#define LED_SHORT_DOUBLE 3
#define LED_EQUAL_SLOW 4
#define LED_EQUAL_FAST 5

class Flasher
{
  // Class Member Variables
  // These are initialized at startup
  int ledPin;      // the number of the LED pin
  unsigned long OnTime;     // milliseconds of on-time
  unsigned long OffTime;    // milliseconds of off-time

  // These maintain the current state
  int ledState;             		// ledState used to set the LED
  unsigned long previousMillis;  	// will store last time LED was updated
  int counter = 0; //contador para determinar el flahseo doble
  int mode = 0;

  // Constructor - creates a Flasher
  // and initializes the member variables and state
  public:
  Flasher(int pin)
  {
  ledPin = pin;
  pinMode(ledPin, OUTPUT);

  OnTime = 50;
  OffTime = 1000;

  ledState = LOW;
  previousMillis = 0;
  }

void Update_mode(int modes){
  mode = modes;
}

void CheckMode(){
  switch (mode) {
    case 0: //LED OFF
      OnTime = 0;
      break;

      case 1: //LED ON
        OffTime = 0;
        break;

      case 2: //Short flash
        OnTime = 50;
        OffTime = 1000;
        break;

      case 3: //Double short flash
        OnTime = 50;
        if (counter != 0)
          OffTime = 100;
        else
          OffTime = 1950;
        break;

      case 4: //equal on off fast
        OnTime = 300;
        OffTime = 300;
        break;

      case 5: //equal on off slow
        OnTime = 1000;
        OffTime = 1000;
        break;

      default: //error fast equal on-off
        OnTime = 100;
        OffTime = 100;
        break;
  }

}

  void Update()
  {
    // check to see if it's time to change the state of the LED
    unsigned long currentMillis = millis();
    CheckMode();
    if (OnTime == 0) ledState = LOW;
    else if (OffTime == 0) ledState = HIGH;
    else{
      if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
      {
        ledState = LOW;  // Turn it off
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(ledPin, ledState);  // Update the actual LED
        counter++;
        if (counter >= 2) counter = 0;
      }
      else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
      {
        ledState = HIGH;  // turn it on
        previousMillis = currentMillis;   // Remember the time
      }
    }
    digitalWrite(ledPin, ledState);	  // Update the actual LED
  }
};