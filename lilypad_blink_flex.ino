


int finger1;
int finger2;

//int soundTrigger;
int fingerTrigger1 = 100;
int fingerTrigger2 = 100;
//int led = 13;

void setup () 

{
  Serial.begin(9600);
 // pinMode(A2, OUTPUT); 
  //pinMode(led, OUTPUT);
}

void loop() {

  /*byte motoControl;
  if(Serial.available()){
     motoControl = Serial.read();
    analogWrite(A2, motoControl);
    Serial.println(motoControl);
  }*/
  finger1 = analogRead(A0);
  finger2 = analogRead(A1);

  Serial.print(finger1);
  Serial.print(",");
  Serial.println(finger2);

  delay(200);


}
