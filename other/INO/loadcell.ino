#include "HX711.h"

HX711 scale(2, 3);

float calibration_factor = 39500; // this calibration factor is adjusted according to my load cell
float units;
bool loops = true;

void setup() {
  Serial.begin(2000000);
  //Serial.setTimeout(10);

  scale.set_scale();
  scale.tare();  //Reset the scale to 0
  long zero_factor = scale.read_average(); //Get a baseline reading
}

void loop() {
  while(loops){
    read_port();
  }

  while(loops == false){
    scale.set_scale(calibration_factor); 
    units = scale.get_units(), 10;
    if (units < 0){
      units = 0.00;
    }
    Serial.println(units);
    read_port();
  }
}

void read_port(){
  if (Serial.available() != 0) {
      String car_ini = Serial.readString();
      //Serial.println(car_ini);
      //Serial.readString();
      if(car_ini == "s"){
        loops = false;
      }else{
        loops = true;
      }
  }
}
