int IN1=7;
int IN2=6;
int ENA=10;
int IN3=4;
int IN4=3;
int ENB=9;
void setup()
{
 pinMode(IN1,OUTPUT);
 pinMode(IN2,OUTPUT);
 pinMode(ENA,OUTPUT);
 pinMode(IN3,OUTPUT);
 pinMode(IN4,OUTPUT);
 pinMode(ENB,OUTPUT);
}
void loop()
{  
 analogWrite(ENA, 200);// motor speed  
 analogWrite(ENB, 200);// motor speed  
 digitalWrite(IN1,LOW);// rotate forward
 digitalWrite(IN2,HIGH);
 digitalWrite(IN3,LOW);// rotate forward
 digitalWrite(IN4,HIGH);
 delay(2000);  
 digitalWrite(IN1,HIGH);// rotate reverse
 digitalWrite(IN2,LOW);
 digitalWrite(IN3,HIGH);// rotate reverse
 digitalWrite(IN4,LOW);
 delay(2000);
}