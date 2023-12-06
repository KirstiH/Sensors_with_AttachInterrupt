/*

20.10.2023 Kirsti Härö

When button B2 is pressed, interrupt will occur and temperature and humidity will be measured with 
SHTC3 sensor and the information will be printed out. VL53L1X sensor is continuesly measuring distance
and if the distance is less than 200mm, the red turns on and ALARM will be printed in the monitor and 
red led turns on. By pressing B1 the alarm will be turned off with the red led and this will be printed
out in the monitor as well.

*/

#include <Arduino.h>
#include <Adafruit_SHTC3.h>
#include <Adafruit_VL53L1X.h>

#define IRQ_PIN 12
#define XSHUT_PIN 11
#define TRUE 1 //for controlling red led
#define FALSE 0 //for controlling red led

int ledRed = 7; // LED connected to digital pin 7
int Button1 = 4;    // pushbutton for red led
int Button2 = 3;    // pushbutton for SHTC3
volatile byte state_red = LOW; //turning led on and off
volatile byte state_sensor = LOW; //for measuring
volatile unsigned int led_off = FALSE; //for led

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void setup() {

  pinMode(ledRed, OUTPUT);  // sets the digital pin 7 as output
  pinMode(Button1, INPUT_PULLUP);    // sets the digital pin 4 as input
  attachInterrupt(digitalPinToInterrupt(Button1), blink_red, FALLING); //interrupt for red led
  attachInterrupt(digitalPinToInterrupt(Button2), use_sensor, FALLING); //interrupt for humidity and temperature sensor

  Serial.begin(9600); //activating serial monitor

  while (!Serial)
    delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  if (! shtc3.begin()) {
    Serial.println("Couldn't find SHTC3");
    while (1) delay(1);
  }
  //Serial.println("Found SHTC3 sensor");

  Wire.begin();
  if (! vl53.begin(0x52, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  //Serial.println(F("VL53L1X sensor OK!"));

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);

}

void loop() {

  sensors_event_t humidity, temp;
  int16_t distance;

  //start measuring temperature and humidity when the button is pressed
  if (state_sensor == HIGH) {
    shtc3.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

    //print out the sensor data
    Serial.print("Temperature is "); Serial.print(temp.temperature, 1); Serial.print(" °C");
    Serial.print(" and relative moisture is "); Serial.print(humidity.relative_humidity, 0); Serial.println("%");
    
    delay(1000);
  }

  //measuring distance when the led light is off
  //turn on the red light when the distance is under 200mm
  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1 ) {
      // something went wrong or the distance is too big!
      return;
    }

    //if the distance is less than 200mm and the led is off, make an alarm
    if (state_red == LOW && distance < 200) {
      state_red = HIGH;
      led_off = TRUE;
	    digitalWrite(ledRed, state_red);
      Serial.println("A L A R M !!!");
      delay(200);
    }
    // data is read out, time for another reading!
    vl53.clearInterrupt();
  }

  //turn off led by clicking the pushbutton
  if (led_off == TRUE && state_red == LOW) {
	  digitalWrite(ledRed, state_red);
    Serial.println("Alarm off");
    led_off = FALSE;
    delay(500);
  }

}

//interrupt for red led and informing alarm is off
void blink_red() {
  state_red = LOW;
  led_off = TRUE;
}

//interrupt for measuring temperature and humidity
void use_sensor() {
  state_sensor = HIGH;
}

