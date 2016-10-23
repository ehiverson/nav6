uint8_t dataArr[1000];
int millistart;
int milliend;
elapsedMillis mi;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(57600);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  while (Serial.available()==0){}
  mi=0;
  millistart=mi;
  for (int i=0;i<1000;i++){
    dataArr[i]=Serial2.read();
    if (i==0)millistart=mi;
  }
  milliend=mi;
  Serial.println(milliend-millistart);
}
