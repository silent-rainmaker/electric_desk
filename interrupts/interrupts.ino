#define UP 1
#define DOWN 2


const int hallPin1 = 2;     // hall sensor 1 connected to pin2
const int hallPin2 = 7;     // hall sensor 2 connected to pin4
const int ledPin =  13;      // the number of the LED pin
const int pwmPinUp =  9;
const int pwmPinDown =  10;
const int dirPin =  8;

// variables will change:
volatile int counter = 0;         // variable for reading the pushbutton status


void setup() {
  // initialize pins as an outputs:
  pinMode(ledPin, OUTPUT);
  pinMode(pwmPinUp, OUTPUT);
  pinMode(pwmPinDown, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  // initialize the pushbutton pin as an input:
  pinMode(hallPin1, INPUT);
  pinMode(hallPin2, INPUT);
  
  //pwm settings
  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00001010; // x8 fast pwm
  // Attach an interrupt to the ISR vector
  attachInterrupt(0, hall_ISR, FALLING);
  Serial.begin(9600);
  Serial.println("Siema");
}

void loop() {
  Serial.println(counter);
  delay(1000);
  blinkLed();
  
  
  runMotor(UP,127);
  delay(5000);
  Serial.println(counter);
  
  stopMotor();
  delay(1000);
  
  runMotor(DOWN,127);
  delay(5000);
  Serial.println(counter);
  
  
  stopMotor();
  delay(1000);
  
  
 
  
  
  
}

void hall_ISR() {
  if(digitalRead(hallPin2)==0) {
    counter++;
  }
    else {
      counter--;
    }
  
}

void blinkLed(){
  digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second
}

int runMotor(int dir, int pwm){
  if (pwm<0) pwm=0;
  if (pwm>255) pwm=255;
  
  if(dir==UP){
  digitalWrite(dirPin, HIGH);
  analogWrite(pwmPinUp, pwm);
  
  }
  else if (dir==DOWN) {
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPinDown, pwm);
  }
  else return 1;//input data error
    
  
 
  return 0;
}


void stopMotor(){
 runMotor(UP,0);
 runMotor(DOWN,0);
}
