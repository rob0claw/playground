#define RIGHT_ENCA 2
#define RIGHT_ENCB 4
#define LEFT_ENCA 3 
#define LEFT_ENCB 5

int IN1=7;
int IN2=8;
int ENA=6;
int IN3=10;
int IN4=11;
int ENB=9;

int PULSES_PER_REV = 364.;
//wheel circumference
float circumference = 20.1;//in cm

// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;

// One-second interval for measurements
int interval = 1000;
  
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

void setup() {
  Serial.begin(9600);

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENB,OUTPUT);
  // pinMode(left,INPUT);
  // pinmode(right,OUTPUT);
 
  // Set pin states of the encoder
  pinMode(RIGHT_ENCA , INPUT_PULLUP);
  pinMode(RIGHT_ENCB , INPUT);
  pinMode(LEFT_ENCA , INPUT_PULLUP);
  pinMode(LEFT_ENCB , INPUT);

 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA), left_wheel_pulse, RISING);
  delay(3000);
  move(100,70);
}

void loop() {
  currentMillis = millis();// Record the time
  if(PULSES_PER_REV*5<left_wheel_pulse_count) move(75,120);
  if(PULSES_PER_REV*11<left_wheel_pulse_count) move(0,0);

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    Serial.print("Right Pulses: ");
    Serial.println(right_wheel_pulse_count);
    Serial.print("Left Pulses: ");
    Serial.println(left_wheel_pulse_count);
    Serial.println("");
    //right_wheel_pulse_count = 0;
    //left_wheel_pulse_count = 0;
  }
}
void move(int right_speed, int left_speed){
  analogWrite(ENA, right_speed);// motor speed  
  analogWrite(ENB, left_speed);
  digitalWrite(IN1,LOW);// rotate forward
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  // delay(2000);  
  // digitalWrite(IN1,HIGH);// rotate reverse
  // digitalWrite(IN2,LOW);
  // digitalWrite(IN3,HIGH);
  // digitalWrite(IN4,LOW);
  // delay(2000);
}

// Increment the number of pulses by 1
void right_wheel_pulse() {
  // Read the value for the encoder for the right wheel
  int val = digitalRead(RIGHT_ENCB);
  right_wheel_pulse_count++;
}
void left_wheel_pulse() {
  // Read the value for the encoder for the right wheel
  int val = digitalRead(LEFT_ENCB);
  left_wheel_pulse_count++;
}