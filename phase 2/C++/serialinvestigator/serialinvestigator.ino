#define ctspin 2
#define rtspin 3
#define BAUD 57600
#define EVBUFFERLENGTH 4096
#define TXBUFFERLENGTH 20480
#define RXBUFFERLENGTH  20480
int NEWSEREVENTTIME=1000;
volatile bool ctsrose=false;
volatile bool ctsfell=false;
volatile bool rtsrose=false;
volatile bool rtsfell=false;
elapsedMicros timer=0;
//Buffers
char txBuffer[TXBUFFERLENGTH];
char rxBuffer[RXBUFFERLENGTH];
uint32_t eventBuffer[EVBUFFERLENGTH]; //stored is sets of three. (timestamp,eventidentifier,index(if applicable))
//Eventidentifiers: 1 rx, 2 tx, 3 ctsrise, 4 ctsfall, 5 rtsrise, 6 rtsfall


void setup() {
  // put your setup code here, to run once:
  Serial2.begin(BAUD);
  Serial1.begin(BAUD);
  Serial.begin(115200);
  pinMode(ctspin,INPUT);
  pinMode(rtspin,INPUT);
  attachInterrupt(digitalPinToInterrupt(ctspin),ctsrise,RISING);
  attachInterrupt(digitalPinToInterrupt(ctspin),ctsfall,FALLING);
  attachInterrupt(digitalPinToInterrupt(rtspin),rtsrise,RISING);
  attachInterrupt(digitalPinToInterrupt(rtspin),rtsfall,FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  int txind=0;
  int rxind=0;
  int evind=0;
  int lasttx=0;
  int lastrx=0;
  int lastprint=0;
  while (true){
    while (Serial.available()==0){}
    String e=Serial.readStringUntil('/');
    if (e=="go"){break;}  
  }
  timer=0;
  while (true){ 
    if (timer-lastprint>1000000){
      lastprint=timer;
      Serial.print("evind ");
      Serial.print(evind);
      Serial.print(" txind ");
      Serial.print(txind);
      Serial.print(" rxind ");
      Serial.println(rxind);
    }
    if (ctsrose)
    {
      ctsrose=false;
      eventBuffer[evind]=timer;
      eventBuffer[evind+1]=3;
      evind+=2;
    }
    if (ctsfell)
    {
      ctsfell=false;
      eventBuffer[evind]=timer;
      eventBuffer[evind+1]=4;
      evind+=2;
    }
    if (rtsrose)
    {
      rtsrose=false;
      eventBuffer[evind]=timer;
      eventBuffer[evind+1]=5;
      evind+=2;
    }
    if (rtsfell)
    {
      rtsfell=false;
      eventBuffer[evind]=timer;
      eventBuffer[evind+1]=6;
      evind+=2;
    }
    if (Serial2.available())
    {
      
      if (timer-lasttx>NEWSEREVENTTIME){
        eventBuffer[evind]=timer;
        eventBuffer[evind+1]=2;
        eventBuffer[evind+2]=txind;
        evind+=3;
      }
      lasttx=timer;
      while (Serial2.available()>0)
      {
        txBuffer[txind]=Serial2.read();
        txind++;
      }
    }
    if (Serial1.available())
    {
      
      if (timer-lastrx>NEWSEREVENTTIME){
        eventBuffer[evind]=timer;
        eventBuffer[evind+1]=1;
        eventBuffer[evind+2]=rxind;
        evind+=3 ;
      }
      lastrx=timer;
      while (Serial1.available()>0)
      { 
        rxBuffer[rxind]=Serial1.read();
        rxind++;
      }
    }
    if ((txind>=TXBUFFERLENGTH-10) or (rxind>=RXBUFFERLENGTH-10) or (evind>=EVBUFFERLENGTH-10)){
      break;
    }
  }
  Serial.println("stats");
  Serial.println(evind);
  Serial.println(rxind);
  Serial.println(txind);
  Serial.println("evbuffer");
  for (int i=0;i<evind;i++){
    Serial.println(eventBuffer[i]);
    delay(1);
  }
  Serial.println("rxbuffer");

  for (int i=0;i<rxind;i++){
    Serial.print(rxBuffer[i]);
    delayMicroseconds(100);
  }
  Serial.println();
  Serial.println("txbuffer");
  for (int i=0;i<txind;i++){
    Serial.print(txBuffer[i]);
    delayMicroseconds(100);
  }
  Serial.println();
  Serial.println("stop");

}

void ctsrise()
{
  ctsrose=true;
  
}
void ctsfall ()
{
  ctsfell=true;
}
void rtsfall()
{
  rtsrose=true;
}
void rtsrise()
{
  rtsfell=true;
}
