/*
TODO

[X] current sensing + 
[ ] status flag diagnosis
[X] moving by pushbutton
[X] normalization
[X] determine positions and current ranges
[x] convert impulses to cm
[ ] position drift
[ ] different speeds and slopes for upward and downward movement
[ ] pwm slope for operation from buttons
[ ] calibrate down current readings
[WIP] overcurrent protection when button pressed and in max/min position

errors and malfunctions detection
[ ] -missing hall (only one,both)
[ ] -overcurrent
[ ] -moving too long
[ ] -duty cycle (thermal)
[ ] -stopping actual operation if button pressed
[ ] uptime - when  motor movemend command is set but no feedback from hall nor adc
[ ] what if normalization starts in end position?

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
#define UP 2
#define DOWN 1

//inputs
const int hallPin1 = 2;     // hall sensor 1 connected to pin2
const int hallPin2 = 7;     // hall sensor 2 connected to pin7
const int currentSensePinUP =  A0;// Up direction
const int currentSensePinDown =  A1;// Up direction
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

const int slopeDistance=50;//hall ticks needed for smooth motor start - distance for pwn increase/decrease
const int maxAdcCurrentUpRaw=669;//4000 mA limit; y=a*x+b; a=5.93, b=34.02
const int maxAdcCurrentDownRaw=500;//3000 mA limit
const int adcArraySize=100;//size of the array used for averaging adc readouts

const int maxStoredPos=4;

volatile int rawPosition = 0;         // variable for reading the pushbutton status
volatile int adcValue = 0;
//volatile int adcValueDown = 0;
volatile int adcCounter=0;
//volatile int adcCounterDown=0;
volatile long int adcSum=0;
//volatile long int adcSumDown=0;
volatile int adcAvg=0;
//volatile int adcAvgDown=0;


int pwmArr[slopeDistance];
int storedPositions[maxStoredPos];


int minPosition=0;
int maxPosition=5000;
int startPosition=0;//start position to determine pwm for button operation

int ledState = HIGH;        // the current state of the output pin
int buttonUpState = HIGH;            // the current reading from the input pin
int lastButtonUpState = HIGH;  // the previous reading from the input pin
int buttonDownState=HIGH;            // the current reading from the input pin
int lastButtonDownState = HIGH;  // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  //zeroingAvgCurrArr();
  storedPositions[0]=87;
  storedPositions[1]=642;
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
  attachInterrupt(0, hall_ISR, FALLING);
  // attachInterrupt(digitalPinToInterrupt(hallPin1), hall_ISR, FALLING);
 
  // Configure ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Set ADC prescaler to 128
  ADMUX |= (1 << REFS0) | (1<<MUX0);  // Set ADC reference voltage to AVCC, select chanel 1
  ADCSRA |= (1 << ADIE);  // Enable ADC conversion complete interrupt
  ADCSRA |= (1 << ADEN);  // Enable ADC
  
  
  
  // Start first ADC conversion
  ADCSRA |= (1 << ADSC);
  
  Serial.begin(9600);
  Serial.println("Siema");
  
 
}

void loop() {
  calculatePwm(pwmArr,slopeDistance);
  Serial.print("Low normalization will start in 3s ");
  delay(1000);
  Serial.print("2s ");
  delay(1000);
  Serial.println("1s ");
  delay(1000);
  
  Serial.println("low normalization started");
  minPosition=normalizationLow(-3000);
  Serial.print("low normalization stopped, min. position: ");
  Serial.println(minPosition);
  
  delay(1000);
  
  goToPosition(110);
  /*
  goToStoredPos(0);
  delay(1000);
  goToStoredPos(1);
  delay(1000);
  goToStoredPos(0);
  delay(1000);
  */ 
 

  /*
  Serial.println("start of normalization");
  maxPosition=normalizationHigh(3000);
  
  Serial.print("stop of normalization, max. position: ");
  Serial.println(maxPosition);
  */ 
   
   
  delay(1000);
  
  
  //runMotor(DOWN, 175);//down because current read will not work
  delay(50);
  
   Serial.println("waiting for the button to be pressed");
  while(1) {
   
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
  
  if ((millis() - lastDebounceTime) > debounceDelay) {//go up
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (readingUp != buttonUpState) {
      buttonUpState = readingUp;
		startPosition=rawPosition;
      // only toggle the LED if the new button state is HIGH
      if (buttonUpState == HIGH) {
        ledState = LOW;
        stopMotor();
        Serial.println(rawPosition);
      }
      if (buttonUpState == LOW) {
        ledState = HIGH;
        runMotor(UP, maxPwm);
        Serial.println("moving up");
      }
    }
  }
  
   if ((millis() - lastDebounceTime) > debounceDelay) {//go down
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (readingDown != buttonDownState) {
      buttonDownState = readingDown;
	  //startPosition=rawPosition;

      // only toggle the LED if the new button state is HIGH
      if (buttonDownState == HIGH) {
        ledState = LOW;
        stopMotor();
        Serial.println(rawPosition);
      }
      if (buttonDownState == LOW) {
        ledState = HIGH;
       //if(adcAvg<maxAdcCurrentDownRaw)
       // runMotor(DOWN, 120);
		//runMotor(DOWN,momentaryPwm(-3000-rawPosition,-3000));
		//goToPosition(-5000);
        
		//else stopMotor();
        Serial.println("moving down");
      }
    }
  }
  
  if ( (buttonDownState == LOW) && (abs(rawPosition-200)>4 ) ) {
		int tempDistance=rawPosition-200;
         // runMotor(DOWN,momentaryPwm(tempDistance,200));
		runMotor(DOWN, 100);

	 //goToPosition(200);
	  
	  /*
	  runMotor(DOWN,momentaryPwm(rawPosition,-3000));
	  Serial.print("pwm: ");
	  Serial.println(momentaryPwm(rawPosition,0));
	  Serial.print("raw position: ");
	  Serial.println(rawPosition);
	  */
	  
	 
     
	  
	  
  }
  
  if (buttonDownState == LOW) {
	  stopMotor();
  }
   //actualDistance=rawPosition-destPosition;
   //       runMotor(DOWN,momentaryPwm(actualDistance,setDistance));
  
//if(adcAvg>maxAdcCurrentDownRaw) stopMotor();
  
  // set the LED:
  digitalWrite(ledPin, ledState);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonUpState = readingUp;
  lastButtonDownState = readingDown;
    
	}
	 
}

void hall_ISR() {
  if(digitalRead(hallPin2)==0) {
    rawPosition++;
  }
    else {
      rawPosition--;
    }
 // Serial.println(rawPosition);
 // Serial.print("; ");
  //Serial.println(adcAvg);
  //Serial.print(" | ");
 // Serial.println(adcAvgDown);
}

ISR(ADC_vect) {// ADC conversion complete interrupt handler
	
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
  
  if(dir==DOWN){
	ADMUX |= (1<<MUX0);
	digitalWrite(dirPin, HIGH);
	analogWrite(pwmPinUp, pwm);
  
  }
  else if (dir==UP) {
	  ADMUX&= ~(1<<MUX0);
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
      while(rawPosition<=destPosition /*&& adcAvg<maxAdcCurrentUpRaw*/ ){
        actualDistance=destPosition-rawPosition;
                
        runMotor(UP,momentaryPwm(actualDistance,setDistance) );
       // Serial.println(rawPosition);
      
      
      }
    }
    else if (destPosition > maxPosition) destPosition=minPosition;
    
    else if(destPosition<rawPosition){//going down
      while(rawPosition>=destPosition){
      actualDistance=rawPosition-destPosition;
          runMotor(DOWN,momentaryPwm(actualDistance,setDistance));
         // Serial.println(rawPosition);
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
    Serial.print("momentaryPwm wrong range/increasing, initial dist: ");
	Serial.println(initialDistance);
	Serial.print("momentary distance: ");
	Serial.println(momentaryDistance);
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

int normalizationLow(int destPosition){
	int setDistance=abs(destPosition-rawPosition);
     int actualDistance=0;
	
	 while(rawPosition>=destPosition && adcAvg<maxAdcCurrentDownRaw ){
        actualDistance=rawPosition-destPosition;
                
       //runMotor(DOWN,momentaryPwm(actualDistance,setDistance) );
        runMotor(DOWN,minPwm) ;     
      }
		
	stopMotor();
	Serial.print("-----------------minimum position: ");
	Serial.println(rawPosition);
	rawPosition=0;
	return rawPosition;
}

int normalizationHigh(int destPosition){
	 int setDistance=abs(destPosition-rawPosition);
     int actualDistance=0;
	
	 while(rawPosition<=destPosition && adcAvg<maxAdcCurrentUpRaw ){
        actualDistance=destPosition-rawPosition;
                
        runMotor(UP,momentaryPwm(actualDistance,setDistance) );
       // Serial.println(rawPosition);
      
      
      }
	
	
	/*
	while(adcAvg<maxAdcCurrentUpRaw  && latch==0){
		//runMotor(UP,128);
		goToPosition(100);
	}
	*/
	stopMotor();
	Serial.print("-----------------maximum position: ");
	Serial.println(rawPosition);
	return rawPosition;
	
}


int goToStoredPos(int storedPosNumb){
	goToPosition(storedPositions[storedPosNumb]);
	
}

