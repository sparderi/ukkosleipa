#include <LedControl.h>
#include <Servo.h>
#include <Stepper.h>

// Define number of steps per rotation:
const int stepsPerRevolution = 2048;

const int8_t g_mayhem_pin = 2; 
//volatile bool  b_doorbell_pressed = false;
//const int8_t g_led_pin = 13;
bool led_state[] = {0, 0, 0, 0, 0}; 
int ledPins[] = {A5 , A4, A3, A2 , A1};
volatile bool mayhem = 0;

const int numDevices = 1;      // number of MAX7219s used in this case 2
const long scrollDelay = 100;   // adjust scrolling speed
unsigned long bufferLong [50] = {0};  
LedControl lc=LedControl(5,12,7,numDevices);//DATA | CLK | CS/LOAD | number of matrices
const unsigned char scrollText[] PROGMEM ={"WAITING  FOR  MAYHEM !    "};


Servo myServo;  // create a servo object
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);
//int const potPin = A0; // analog pin used to connect the potentiometer
//int potVal;  // variable to read the value from the analog pin
const int8_t angle;   // variable to hold the angle for the servo motor



void setup() {
  // put your setup code here, to run once:
  pinMode(g_mayhem_pin, INPUT_PULLUP);
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, HIGH);
  digitalWrite(A5, HIGH);

  //digitalWrite(g_led_pin, LOW);
  attachInterrupt(digitalPinToInterrupt(g_mayhem_pin), haloo_interrupt, CHANGE);

  myServo.attach(6); // attaches the servo on pin 9 to the servo object
  myStepper.setSpeed(5);
  //Serial.begin(9600); // open a serial connection to your computer

  TCCR2A = 0;
  TCCR2B = 0;
  OCR2A = 256;
  TCCR2B |= (1 << WGM12) | (1 << CS10) | (1 << CS12);
  TIMSK2 |= (1 << OCIE1A);

  for (int x=0; x<numDevices; x++)
  {
    lc.shutdown(x,false);       //The MAX72XX is in power-saving mode on startup
    lc.setIntensity(x,8);       // Set the brightness to default value
    lc.clearDisplay(x);         // and clear the display
  }
  
}

ISR (TIMER2_COMPA_vect){
  static byte i;
  static byte loops;
  loops++;
    if (loops == 36){
    i = (5 == i) ? 0 : i;
    led_state[i] = !led_state[i];
    digitalWrite(ledPins[i], led_state[i]);
    i++;
    loops = 0;
    }
    
}


void haloo_interrupt() 
{
    mayhem = !mayhem;
}

void stepper_spinning()
{
  myStepper.step(stepsPerRevolution/16);
}

void servo_rolling()
{
  static bool servo_rolled;
  if (!servo_rolled){
    myServo.write(180);
  } else {
    myServo.write(40);
  }
  servo_rolled = !servo_rolled;
}

void loop() {
  if (!mayhem){
    scrollMessage(scrollText);   //scrollFont();
  } else{
  //scrollMessage(scrollText);   //scrollFont();
  servo_rolling();
  myStepper.step(stepsPerRevolution/8);
  }
}
