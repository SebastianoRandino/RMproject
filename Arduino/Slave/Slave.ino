#include <Wire.h>

char gen_speed_str[6]; //convert float speed to string
float generator_speed = 1;
const int generator_pin = 2;
volatile unsigned long generator_dt = 2;
volatile unsigned long generator_last_time = 1;
volatile unsigned long generator_now = 2;
float pi = 2 * acos(0.0);
int t1;
int t2;
volatile int gen_pos = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin(8);                //  I2C bus with address #8
  Wire.onRequest(requestEvent); // register event
  pinMode(generator_pin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(generator_pin),Encoder,RISING);
}

void loop() {
  if (generator_dt > 0){
    if ((micros() - generator_last_time) < 1e5){
      generator_speed =  (2 * pi / 100) / (generator_dt * 1e-6); // [rad/s] (my encoder actually has 100 Cycles Per Revolution)
    }
    else  {
      generator_speed = 0;
    }
  }
  if (micros()%6){
  Serial.println(generator_speed);
  }
  dtostrf(generator_speed,6,2, gen_speed_str);

}

void requestEvent() {
  //t1 = micros();
  //dtostrf(generator_speed,6,2, String);
  Wire.write(gen_speed_str);
  // Serial.print("Required speed: ");
  // Serial.println(generator_speed);
  //t2 = micros();
  // Serial.print("Required speed: ");
  // Serial.println(t2-t1);
}

void Encoder() {
  gen_pos += 1;
  generator_now = micros();
  if ((gen_pos % 1)==0){
  if (generator_now > generator_last_time){
       generator_dt = (generator_now - generator_last_time)/1;
     }
  generator_last_time = generator_now;
}
}
