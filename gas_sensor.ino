int button = 2;
int flag = 0;
int sound = 11;
const int interval=200;

void interrupt(){
   while(1){
   }
}



int gas_sensor= A0; 

void setup()    
{   
  Serial.begin(9600);
  pinMode(gas_sensor,INPUT); 

  pinMode(sound,OUTPUT);
  pinMode(2,INPUT);  
  digitalWrite(button,1);
}    
      
void loop()  
{    
  int val=analogRead(gas_sensor); 
  Serial.println(val); 
  // delay(100);
 if(val>300){
    attachInterrupt(0,interrupt, CHANGE);
    tone(sound,330,interval);
    delay(interval);
    tone(sound,311,interval);
    delay(interval);
    tone(sound,330,interval);
    delay(interval);
    tone(sound,311,interval);
    delay(interval);
    tone(sound,330,interval);
    delay(interval);
    tone(sound,247,interval);
    delay(interval);
    tone(sound,294,interval);
    delay(interval);
    tone(sound,262,interval);
    delay(interval);
    tone(sound,220,interval*4);
    delay(interval*3);
    tone(sound,262,interval);
    delay(interval);
    tone(sound,165,interval);
    delay(interval-1);
    tone(sound,220,interval);
    delay(interval);
    tone(sound,247,interval*4);
    delay(interval*3);
    tone(sound,165,interval);
    delay(interval);
    tone(sound,208,interval);
    delay(interval);
    tone(sound,247,interval);
    delay(interval);
    tone(sound,262,interval*4);
    delay(interval*5);
 
    tone(sound,330,interval);
    delay(interval);
    tone(sound,311,interval);
    delay(interval);
    tone(sound,330,interval);
    delay(interval);
    tone(sound,311,interval);
    delay(interval);
    tone(sound,330,interval);
    delay(interval);
    tone(sound,247,interval);
    delay(interval);
    tone(sound,294,interval);
    delay(interval);
    tone(sound,262,interval);
    delay(interval);
    tone(sound,220,interval*4);
    delay(interval*3);
    tone(sound,262,interval);
    delay(interval);
    tone(sound,165,interval);
    delay(interval-1);
    tone(sound,220,interval);
    delay(interval);
    tone(sound,247,interval*4);
    delay(interval*3);
    tone(sound,165,interval);
    delay(interval);
    tone(sound,262,interval);
    delay(interval);
    tone(sound,247,interval);
    delay(interval);
    tone(sound,220,interval*4);
    delay(interval*8);
 }
   

} 
