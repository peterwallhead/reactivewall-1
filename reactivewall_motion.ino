int rangeSensor = 8;
int distance;
int m1pos;
int m2pos;
unsigned long pulseduration=0;

// Assign motor shield pins that start and stop the motor.
int PWM1 = 5;
int PWM2 = 6;

// Assign motor shield pins that dictate motor direction.
int DIR1 = 4;
int DIR2 = 7;

int motorSpeed = 255;    

int potPinM1 = A0;
int potPinM2 = A1;

void setup()
{
  pinMode(rangeSensor, OUTPUT);
  Serial.begin(9600);
  
  // Set pin modes to output for all those used by the motor shield.
  int i;
  for(i=4; i<=7; i++) {
      pinMode(i, OUTPUT);
  }
  
  wallController(400,400);
  delay(3000);
  
}

void loop()
{
  // Get the raw measurement data from Ping)))
  measureDistance();
    
  if(distance < 60) {

    m1pos = map(distance,59,0,10,400);
    m2pos = map(distance,59,0,400,10);
    
    wallController(m1pos,m2pos);
    
  } else {
    wallController(400,400);
  }

  delay(100);
}

void measureDistance()
{
  // Set pin as output so we can send a pulse.
  pinMode(rangeSensor, OUTPUT);

  // Set output to LOW.
  digitalWrite(rangeSensor, LOW);
  delayMicroseconds(5);
  
  // Now send the 5uS pulse out to activate Ping)))
  digitalWrite(rangeSensor, HIGH);
  delayMicroseconds(5);
  digitalWrite(rangeSensor, LOW);
  
  // Now we need to change the digital pin 
  // to input to read the incoming pulse.
  pinMode(rangeSensor, INPUT);
  
  // Finally, measure the length of the incoming pulse.
  pulseduration=pulseIn(rangeSensor, HIGH);
  
  // Divide the pulse length by half.
  pulseduration=pulseduration/2;
  
  // Now convert to centimetres. We're metric here people...
  distance = int(pulseduration/29);
}

void wallController(int targetM1, int targetM2){
  int valM1 = analogRead(potPinM1);
  int valM2 = analogRead(potPinM2);

  analogWrite(PWM1, motorSpeed);
  analogWrite(PWM2, motorSpeed);
  
  while(abs(valM1 - targetM1) > 1){
      valM1 = analogRead(potPinM1);
      
      if(valM1 > targetM1){
          digitalWrite(DIR1, LOW);
      } else {
          digitalWrite(DIR1, HIGH);
      }      
  }

  analogWrite(PWM1, 0);

  while(abs(valM2 - targetM2) > 1){
    valM2 = analogRead(potPinM2);

    if(valM2 > targetM2){
        digitalWrite(DIR2, LOW);
    } else {
        digitalWrite(DIR2, HIGH);
    }
  }

  analogWrite(PWM2, 0);
}
