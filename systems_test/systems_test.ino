/* Includes */
#include <Servo.h> // For servo control
#include <math.h>  // for rounding & math

// Volatiles
int traffic_scale = 5;     // manually changed via ISR
int car_threshold = 3;      // changed by algorithm based on detections and traffic_scale
int num_cars_passed = 0;       // num LED cars passed since last light change

// Constants
const int yellow_light_delay = 2000;
const int red_to_green_delay = 1000;
const byte yellow = 0x00;   //TODO
const byte red = 0x00;
const byte green = 0x00; 

// Motors
Servo servoR;
Servo servoL;

void setup() {

  // Configure Pin modes for traffic LEDs
  pinMode(33, OUTPUT);    // highway red
  pinMode(35, OUTPUT);    // highway yellow
  pinMode(37, OUTPUT);    // highway green
  pinMode(49, OUTPUT);    // side_road red
  pinMode(51, OUTPUT);    // side_road yellow
  pinMode(53, OUTPUT);    // side_road green
  // Configure Pin modes for LED "strips"
  pinMode(22, OUTPUT);    // strip 1 #1
  pinMode(24, OUTPUT);    // strip 1 #2
  pinMode(26, OUTPUT);    // strip 1 #3
  pinMode(28, OUTPUT);    // strip 1 #4
  pinMode(23, OUTPUT);    // strip 2 #1
  pinMode(25, OUTPUT);    // strip 2 #2
  pinMode(27, OUTPUT);    // strip 2 #3
  pinMode(29, OUTPUT);    // strip 2 #4
  digitalWrite(22, LOW);
  digitalWrite(24, LOW);
  digitalWrite(26, LOW);
  digitalWrite(28, LOW);
  digitalWrite(23, LOW);
  digitalWrite(25, LOW);
  digitalWrite(27, LOW);
  digitalWrite(29, LOW);
  
  /* Open software serial port with baud rate = 9600. */
  Serial.begin(9600);
  
  // Traffic Flow Interrupts
  //attachInterrupt(digitalPinToInterrupt(pin), dec, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(pin), inc, CHANGE);

  // Attach servos to Digital Pins
  servoR.attach(10);    // servo_1
  servoL.attach(9);     // servo_2
  
  // set servo to initial positions
  servoR.write(30);
  servoL.write(143);
}

void loop(){
 traffic_lights(5);
}


 
// Utility Functions

bool traffic_lights(int num_cars){
  // if the number of cars is less than threshold, then we just leave the lights going the highway way
  if ((num_cars >= car_threshold) || (traffic_scale == 0)) {
    
    // highway turns yellow
    Serial.println("Side Light Yellow");
    digitalWrite(37, LOW);
    digitalWrite(35, HIGH);
    // stop the LED strips
    int old_traffic_scale = traffic_scale;
    traffic_scale = 0;
    delay(yellow_light_delay);
    Serial.println("Side Light Red");
    digitalWrite(35, LOW);
    digitalWrite(33, HIGH);
    // red light stays on
    delay(red_to_green_delay);
    // turn off side_road red light
    digitalWrite(49, LOW);
    digitalWrite(53, HIGH);
    move_gate(1);
    delay(5000);
    // side_road turns yellow
    digitalWrite(53, LOW);
    digitalWrite(51, HIGH);
    move_gate(0);
    delay(yellow_light_delay);    // yellow light delay
    // side_road light turns red
    digitalWrite(51, LOW);
    digitalWrite(49, HIGH);
    delay(red_to_green_delay);
    // highway turns green, side_road red stays on
    Serial.println("Side Light Green");
    digitalWrite(33, LOW);
    digitalWrite(37, HIGH);
    
    traffic_scale = old_traffic_scale;

    num_cars_passed = 0;
    return true;
  }
  return false;
}

void LED_pass(int num_cars){
  // num_cars is the number of cars to be passed in 1 second
  float interval = 2000 / num_cars;
  float q_time = 100;
  int led_num;
  
  for (int i=0; i<num_cars; i++){
    led_num = random(1,3);    // choose which LED to light up for this car
    // light up led strip designated by LED num, probably need another loop to byte address LEDs
    if (led_num == 1){
        digitalWrite(22, HIGH);
        delay(q_time);
        digitalWrite(22, LOW);
        digitalWrite(24, HIGH);
        delay(q_time);
        digitalWrite(24, LOW);
        digitalWrite(26, HIGH);
        delay(q_time);
        digitalWrite(26, LOW);
        digitalWrite(28, HIGH);
        delay(q_time);
        digitalWrite(28, LOW);
    }
    else {
        digitalWrite(23, HIGH);
        delay(q_time);
        digitalWrite(23, LOW);
        digitalWrite(25, HIGH);
        delay(q_time);
        digitalWrite(25, LOW);
        digitalWrite(27, HIGH);
        delay(q_time);
        digitalWrite(27, LOW);
        digitalWrite(29, HIGH);
        delay(q_time);
        digitalWrite(29, LOW);
    }
    delay(interval - q_time*4);
  }
}

void move_gate(int trig){
  // when trig = 1, open the gates, else, close the gates
  if (trig == 1) {
    // move the servo 90 degrees in open direction
    servoR.write(30);
    servoL.write(143);
  }
  else {
    // move the servo back to close (-90)
    servoR.write(145);
    servoL.write(33);
  }
}

// Interrupt Service Routines

void dec(){
  if (traffic_scale > 0) {
    traffic_scale --;
  }
}

void inc(){
  if (traffic_scale < 5) {
    traffic_scale ++;
  }
}
