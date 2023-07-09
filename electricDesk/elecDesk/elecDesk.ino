/*
TODO

[X] current sensing + 
[ ] status flag diagnosis
[ ] moving by pushbutton
[ ] normalization

errors and malfunctions detection
[ ] -missing hall (only one,both)
[ ] -overcurrent
[ ] -moving too long
[ ] -duty cycle (thermal)

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


const int hallPin1 = 2;     // hall sensor 1 connected to pin2
const int hallPin2 = 7;     // hall sensor 2 connected to pin7
const int ledPin =  13;      // the number of the LED pin
const int pwmPinUp =  9;
const int pwmPinDown =  10;
const int dirPin =  8;
const int debugPin =  12;
const int currentSensePin =  A0;

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
int adcArray[adcArraySize]/*={100,50,100,50,100,50,100,50,100,50}*/;//can be removed, probably



void setup() {
  zeroingAvgCurrArr();
  
  // initialize pins as an outputs:
  pinMode(ledPin, OUTPUT);
  pinMode(pwmPinUp, OUTPUT);
  pinMode(pwmPinDown, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(debugPin, OUTPUT);
  
  // initialize pins as an input:
  pinMode(hallPin1, INPUT);
  pinMode(hallPin2, INPUT);
  
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
  Serial.print("position ");
  Serial.println(rawPosition);
  delay(500);
  blinkLed();
  
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
  runMotor(DOWN, 175);//down because current read will not work
  delay(50);
  while(1) {
    Serial.print("ADC: ");
    Serial.println(adcAvg);
    //Serial.println(adcValue);
    //delay(100);
    //blinkLed();
    
    /*
    for(int j=0;j<adcArraySize;j++){
		
		Serial.print(j);
		Serial.print(": ");
		Serial.print(adcArray[j]);
		Serial.print(", ");
		
		//avgCurrent+=adcArray[j];
		/*
		Serial.print("subsum  ");
		Serial.println(avgCurrent);
		
		}
		*/
	}
	
    
 
  
  
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

int getCurrentAvg(){
	//ADCSRA &= ~(1 << ADIE);//Disable adc int
	int avgCurrent=0;
	
	//Serial.println("-----------------------");
	for(int j=0;j<adcArraySize;j++){
		/*
		Serial.print(j);
		Serial.print(": ");
		Serial.println(adcArray[j]);
		*/
		avgCurrent+=adcArray[j];
		/*
		Serial.print("subsum  ");
		Serial.println(avgCurrent);
		*/
	}
	/*
	Serial.print("sum for avg: ");
	Serial.println(avgCurrent);
	Serial.println("-----------------------");
	*/
	return avgCurrent/adcArraySize;
	//ADCSRA |= (1 << ADIE);// Enable ADC interrupt
	//return -255;
}


int zeroingAvgCurrArr(){
	for (int i=0;i<adcArraySize;i++){
	adcArray[i]=0;
	}
	
}
