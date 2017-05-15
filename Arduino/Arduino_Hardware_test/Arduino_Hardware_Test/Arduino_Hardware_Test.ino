unsigned long previousMillis = 0;
const long tick = 100; //t(ms)

void setup() {
  // put your setup code here, to run once:
    Serial.begin(230400); //115200; 500000
    Serial.println("Start");


  // TCRT5000 IR sensor. Floor detector.
  // http://www.sunrom.com/p/lineobstacle-sensor-tcrt5000  
  #define IR_INPUT_FLOOR  3
  pinMode(IR_INPUT_FLOOR, INPUT);
  //attachInterrupt(digitalPinToInterrupt(IR_INPUT_FLOOR), isr_ir_floor_detect, RISING); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;
  
}

// INTERRUPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////
void isr_ir_floor_detect(){  //Sensor output == LOW when detecting black surface 
    if (!digitalRead(IR_INPUT_FLOOR) ) {
        Serial.print("Black floor!\n");
    }
}
// INTERRUPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= tick) {
    previousMillis = currentMillis;
    Serial.write(0x0C); ////Form Feed char(0x0C): Page break on terminal. Select "Handle Form Feed Character" at CoolTerm preferences     

    // put your main code here, to run repeatedly:
    if(digitalRead(IR_INPUT_FLOOR)){
      Serial.print("White floor!\n");
    }else{
      Serial.print("Black floor!\n");
    }
    
  }//IF END  
}


