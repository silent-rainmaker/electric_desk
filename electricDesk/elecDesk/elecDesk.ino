/*
TODO

[X] current sensing + 
[ ] status flag diagnosis
[X] moving by pushbutton
[ ] normalization

errors and malfunctions detection
[ ] -missing hall (only one,both)
[ ] -overcurrent
[ ] -moving too long
[ ] -duty cycle (thermal)
[ ] -stopping actual operation if button pressed

[ ] reseting
[ ] setting upper limit
[ ] setting lower limit
[ ] deleteing limits

[ ] saving positions to eeprom
[ ] restoring positions from eeprom

[ ] setting and removing detection collision sensitivity

[ ] connect and control enable pins 

sleep mode?
 
[ ] non sensitiviti for inrush current
[ ] separate calibartion for each channel current measurements

*/
#define UP 1
#define DOWN 2

//inputs
const int hallPin1 = 2;     // hall sensor 1 connected to pin2
const int hallPin2 = 7;     // hall sensor 2 connected to pin7
const int currentSensePin =  A0;
const int buttonPinUp=4;
const int buttonPinDown=5;

//outputs
const int ledPin =  13;      // the number of the LED pin
const int pwmPinUp =  9;
const int pwmPinDown =  10;
const int dirPin =  8;
const int debugPin =  12;

const int minPwm=64;
const int maxPwm=192;
const int maxPosition=5000;
const int minPosition=0;
const int slopeDistance=200;//hall ticks needed for smooth motor start - distance for pwn increase/decrease
const int adcArraySize=100;//size of the array used for averaging adc readouts

volatile int rawPosition = 0;         // variable for reading the pushbutton status
volatile int adcValue = 0;
volatile int adcCounter=0;
volatile int adcSum=0;
volatile int adcAvg=0;

int pwmArr[slopeDistance];
//int adcArray[adcArraySize]/*={100,50,100,50,100,50,100,50,100,50}*/;//can be removed, probably

int ledState = HIGH;        // the current state of the output pin
int buttonUpState;            // the current reading from the input pin
int lastButtonUpState = LOW;  // the previous reading from the input pin
int buttonDownState;            // the current reading from the input pin
int lastButtonDownState = LOW;  // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  //zeroingAvgCurrArr();
  
  // initialize pins as an outputs:
  pinMode(ledPin, OUTPUT);
  pinMode(pwmPinUp, OUTPUT);
  pinMode(pwmPinDown, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(debugPin, OUTPUT);
  
  // initialize pins as an inputs:
  pinMode(hallPin1, INPUT);
  pinMode(hallPin2, INPUT);
  
  pinMode(buttonPinUp, INPUT_PULLUP);
  pinMode(buttonPinDown, INPUT_PULLUP);
  
  // set initial LED state
  digitalWrite(ledPin, ledState);
  
  //pwm settings
  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00001010; // x8 fast pwm
  
  

  
  // Attach an interrupt to the ISR vector
  //attachInterrupt(0, hall_ISR, FALLING);
  // attachInterrupt(digitalPinToInterrupt(hallPin1), hall_ISR, FALLING);
 
  // Configure ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Set ADC prescaler to 128
  ADMUX |= (1 << REFS0);  // Set ADC reference voltage to AVCC
  ADCSRA |= (1 << ADIE);  // Enable ADC conversion complete interrupt
  ADCSRA |= (1 << ADEN);  // Enable ADC
  
  // Start first ADC conversion
  ADCSRA |= (1 << ADSC);
  
  Serial.begin(9600);
  Serial.println("Siema");
  
 
}

void loop() {
  calculatePwm(pwmArr,slopeDistance);
 
  int readingUp = digitalRead(buttonPinUp);
  int readingDown = digitalRead(buttonPinDown);
  
  if (readingUp != lastButtonUpState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  
  if (readingDown != lastButtonDownState) {// assuming there is no other button press while counting debounce time
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (readingUp != buttonUpState) {
      buttonUpState = readingUp;

      // only toggle the LED if the new button state is HIGH
      if (buttonUpState == HIGH) {
        ledState = LOW;
        stopMotor();
      }
      if (buttonUpState == LOW) {
        ledState = HIGH;
        runMotor(UP, 75);
      }
    }
  }
  
   if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (readingDown != buttonDownState) {
      buttonDownState = readingDown;

      // only toggle the LED if the new button state is HIGH
      if (buttonDownState == HIGH) {
        ledState = LOW;
        stopMotor();
      }
      if (buttonDownState == LOW) {
        ledState = HIGH;
        runMotor(DOWN, 75);
      }
    }
  }
  
  // set the LED:
  digitalWrite(ledPin, ledState);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonUpState = readingUp;
  lastButtonDownState = readingDown;
  
  
  //Serial.print("position ");
  //Serial.println(rawPosition);
  //delay(500);
  //blinkLed();
  
  /*
  Serial.println("start moving");
  digitalWrite(debugPin, LOW);
  digitalWrite(debugPin, HIGH);
  goToPosition(500);
  digitalWrite(debugPin, LOW);
  Serial.println("stop moving");
  
  Serial.println("max reached");
  
  delay(1000);
  goToPosition(0);
  Serial.println("min reached");
  */
  
  /*
  runMotor(DOWN, 175);//down because current read will not work
  delay(50);
  while(1) {
    Serial.print("ADC: ");
    Serial.println(adcAvg);
    
	}
	
    */
 
  
  
}

void hall_ISR() {
  if(digitalRead(hallPin2)==0) {
    rawPosition++;
  }
    else {
      rawPosition--;
    }
   //Serial.println(rawPosition);
}


ISR(ADC_vect) {// ADC conversion complete interrupt handler
  
  /*adcValue = ADC; // Read ADC value
  //Serial.print(adcValue);
  //Serial.println(" ");
  adcArray[adcCounter]=adcValue;
  adcCounter++;
  if (adcCounter==adcArraySize) {
	  adcCounter=0; 
	  
  }
  
    ADCSRA |= (1 << ADSC);// Start the next ADC conversion

    */
    //adcValue = ADC;
    
    
    adcSum+=ADC;
    adcCounter++;
    if (adcCounter==adcArraySize-1){
		adcAvg=adcSum/adcArraySize;
		adcSum=0;
		adcCounter=0;
	}
	
	
	ADCSRA |= (1 << ADSC);// Start the next ADC conversion
     
}

void blinkLed(){
  digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
  delay(100); 
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

int goToPosition(int destPosition){
  
  int setDistance=abs(destPosition-rawPosition);
  int actualDistance=0;
  //checking input data
  if (destPosition > maxPosition) destPosition=maxPosition;
  else if (destPosition < minPosition) destPosition=minPosition;
  
    if(destPosition>rawPosition){//going up
      while(rawPosition<destPosition){
        actualDistance=destPosition-rawPosition;
                
        runMotor(UP,momentaryPwm(actualDistance,setDistance) );
       //runMotor(UP,100 );
      
      
      }
    }
    else if (destPosition > maxPosition) destPosition=minPosition;
    
    else if(destPosition<rawPosition){//going down
      while(rawPosition>destPosition){
      actualDistance=rawPosition-destPosition;
          runMotor(DOWN,momentaryPwm(actualDistance,setDistance));
      }
    }
    
  stopMotor();
  return 0;
}

int momentaryPwm(int momentaryDistance, int initialDistance){
  if (initialDistance<2*slopeDistance) return minPwm;//case when the distance is to small to reach full speed and then slow down
  
  else if (initialDistance-momentaryDistance<slopeDistance ){//first slope - rising PWM
    if (  (initialDistance-momentaryDistance >=0) && (initialDistance-momentaryDistance <slopeDistance) )
    return pwmArr[initialDistance-momentaryDistance];
    
    else {
    Serial.println("momentaryPwm wrong range/increasing");
    while(1);
    }
  }
  
  //second phase - constat PWM
 // else if ( (initialDistance-momentaryDistance> (slopeDistance) ) && (momentaryDistance > slopeDistance) ){
    else if ( (momentaryDistance > slopeDistance-1) ){
    return maxPwm;
  }
  
  //third stage - PWM slowing down
  else if (momentaryDistance<slopeDistance){
   // if (  (momentaryDistance >0) && (initialDistance-momentaryDistance <50) )
    /*
    Serial.print(" momentary dist= ");
    Serial.println(momentaryDistance);
    */
    
    return pwmArr[momentaryDistance];
    
    /*
    else {
    Serial.print("momentaryPwm wrong range/slowing; momentary dist= ");
    Serial.println(momentaryDistance);
    Serial.print("momentaryPwm wrong range/slowing; initial dist= ");
    Serial.println(initialDistance);
    while(1) blinkLed();
    }
    */
    
  }
  else return -255;
}

int calculatePwm (int* arr, int arrSize){
 
   const float a=((float)maxPwm-(float)minPwm)/(float)slopeDistance;
   
     
  float tempPwm=0;
  for(int i=0; i<arrSize; i++){
    tempPwm=a*i+minPwm; 
    arr[i]= (int)tempPwm;
  }
  return 0;
}


/*
int zeroingAvgCurrArr(){
	for (int i=0;i<adcArraySize;i++){
	adcArray[i]=0;
	}
	
}
*/
