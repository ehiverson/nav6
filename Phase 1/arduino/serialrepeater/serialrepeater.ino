
void setup() {
  // put your setup code here, to run once:
pinMode(2,OUTPUT);
pinMode(3,OUTPUT);
digitalWrite(2,HIGH);
digitalWrite(3,LOW);
Serial.begin(115200);
Serial1.begin(115200);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  while (Serial1.available()>0){
    Serial.write(Serial1.read());
  }
  while (Serial.available()>0){
    Serial1.write(Serial.read());
  }
  
}

