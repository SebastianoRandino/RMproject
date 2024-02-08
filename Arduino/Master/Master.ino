#include <Wire.h>

unsigned long myTime;

float generator_speed;  //generator speed

// serial communication
int ind_space;     //space index
String cmd;        //command passed as argument
String value_str;  //value passed as argument

// string variables (global)
String message;

int i=0;

const int relay_firstpin = 23;
const int nbr_bits = 11; //10 resistances connected

bool acquire = false;

int myTimeout = 3;

int freq = 10;

// convert int to binary bits
#define BIT(n,i) (n>>i&1)

void setup() {

  Wire.begin();        // join I2C bus (address optional for master)

  // serial communication
  Serial.begin (9600);
  Serial.setTimeout(myTimeout);

  //relays
  for (i=0; i<=nbr_bits; i++){
    pinMode (relay_firstpin+i, OUTPUT);
    digitalWrite(relay_firstpin+i, HIGH);
  }

}
 
void loop() {
  delayMicroseconds(50);
  //Send infos to serial port
  myTime = millis(); //ms from the start of the program

  if (acquire == true) {
    if (myTime%freq == 0) {
      String speedString = "";
      Wire.requestFrom(8, 6);    // request 6 bytes from slave device #8

      while (Wire.available()){ // slave may send less than requested
        char c = Wire.read();
        speedString = speedString + c;
      }
    generator_speed = speedString.toFloat();
    Serial.println(generator_speed);
  }
  }

  //Check if anything is available in the serial receive buffer
  if (Serial.available() > 0)
  {
    //Taking as an input a string in the format 'cmd + float' 
    //message = Serial.readString();
    //t3 = micros();
    //ind_space = message.indexOf(' ');
    // reading string, splitting in two `cmd + float`
    // cmd = message.substring(0, ind_space);
    // cmd.toLowerCase();
    // value_str = message.substring(ind_space+1, message.length()-1);

    //Reading the string as an array of bytes
    byte input[100];
    int i = 0;
    while (Serial.available() > 0 && i < 99) {
    input[i] = Serial.read();
    if (input[i] == ' ') {
      break;
    }
    i++;
    }
    input[i] = '\0';

    //Parsing the input array to get the command and value
    cmd = String((char*)input);
    cmd.trim();
    value_str = Serial.readStringUntil('\n');
    value_str.trim();

    if ((cmd == "r")){
      int relay_value = value_str.toInt();
      set_relays(relay_value);
      // Serial.print("Relay uptaded");
    }
    else if (cmd == "s") {
      acquire = true;
	  }
    else if (cmd == "t") {
      acquire = false;
	  }
    else if (cmd == "f") {
      freq = value_str.toInt();
	  }
    else
    {
      Serial.println("Command not recognized. Try again!");
    }
  }
}

void set_relays(int nbr_input){
  int   i = nbr_bits-1; //8 bits
  for (i=0; i<=(nbr_bits-1); i++){
    int curr_bit = BIT(nbr_input,(nbr_bits-1)-i);
    if (curr_bit == 1){
      digitalWrite(relay_firstpin+i, HIGH);
    }
    else if (curr_bit == 0){
      digitalWrite(relay_firstpin+i, LOW);
    }
    else {
    }
  }
}
