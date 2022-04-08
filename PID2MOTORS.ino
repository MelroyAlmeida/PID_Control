#include <util/atomic.h>
#include <math.h>
long distToPpr,angToPpr,a,b,x,y,count,call;

/*------------ CLASS ------------*/
class SimplePID{
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral;
   
  public:
    // Default initialization list
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
   
    // Set the parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // Evaluate the signal
    void evalu(int value, int target, float deltaT,int &pwr, int &dir){
       
      // error
      int e = target - value;
     
      float dedt = (e-eprev)/(deltaT);
      eintegral = eintegral + e*deltaT;
      float u = kp*e + kd*dedt + ki*eintegral;
   
      // motor power
      pwr = (int) fabs(u);
      if( pwr > umax ){
        pwr = umax;
      }
           
      // motor direction
      dir = 1;
      if(u<0){
        dir = -1;
      }
           
      // store previous error
      eprev = e;
    }
   
};

/*------------ GLOBALS AND DEFINITIONS ------------*/
// Define the motors
#define NMOTORS 2
#define M0 0
#define M1 1
const int enca[] = {2,3}; // encoder interrupt pins white
const int encb[] = {4,5}; // encoder signal pins green
const int pwm[] = {9,11}; // motor speed pins
const int in1[] = {8,12}; // motor in2 and in4
const int in2[] = {10,13};// motor in1 and in3


// Global variables
long prevT = 0;
int posPrev[] = {0,0};

// positions
volatile int posi[] = {0,0};

// PID classes
SimplePID pid[NMOTORS];


/*------------ FUNCTIONS ------------*/
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}

// targets
long target[] = {0,0};



/*------------ SETUP ------------*/
void setup() {
  Serial.begin(9600);
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT_PULLUP);
    pinMode(encb[k],INPUT_PULLUP);
    //motor pinmode not set
    pid[k].setParams(10,0.025,0,100);  
  }
  attachInterrupt(digitalPinToInterrupt(enca[M0]),readEncoder<M0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M1]),readEncoder<M1>,RISING);
 
}



/*------------ LOOP ------------*/
void loop() {
 
  if(Serial.available()>0)
  { count=0;
    call=0;
    String str =  Serial.readStringUntil('\n');
    int ind1=str.indexOf(',');
    String distance=str.substring(0,ind1);
    String angle=str.substring(ind1+1);
    int dist=distance.toInt();
    int ang=angle.toInt();
    float x=dist/47.1;
    distToPpr=long((360*x));
    float y=29.5*ang*3.14;
    float angToDist = (y*360)/180;
    angToPpr = (int) ( angToDist/47.1);
  }
call++;
if((distToPpr>0)&&(call==1))
  {
       a=a+distToPpr;
      target[0] = a;
      target[1] =-a ;
 
  }
  else
  {
    if((angToPpr>0)&& (call==1))
    {
       a=a+angToPpr;
      target[0]=a;//angToPpr;
     
    }
    else if((angToPpr<0)&& (call==1))
    {
       a=a+(-angToPpr);
     
      target[1]=-a;
    }
  }
 

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
   
 
 
 
  // Get the current position from the follower
  long pos[2];
 
  // Read the position in an atomic block to avoid a potential misread
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }

  // Loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir); // compute the position
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]); // signal the motor
  }

 /*for(int i = 0; i<2; i++){
    Serial.print(target[i]);
    Serial.print(" ");
  }
  for(int i = 0; i<2; i++){
    Serial.print(pos[i]);
    Serial.print(" ");
  }
  Serial.println();
  */
  if(distToPpr > 0)
  {
  if(((pos[0] >= target[0]-5)&&(pos[0] <= target[0]+5))&&(((-pos[1]) >= (-target[1])-5)&&((-pos[1]) <= (-target[1])+5)))
  { count++;
    if(count==1)
    {
      Serial.println("1\n");
    }
  }
  }
else
{
  if(angToPpr > 0)
  {
    if(((pos[0] >= target[0]-5)&&(pos[0] <= target[0]+5)))
    {
      count++;
    if(count >= 1)
    {
      Serial.println("1\n");
    }
     
      }
    }
  else if(angToPpr < 0)
  {
    if(((-pos[1] >= -target[1]-5)&&(-pos[1] <= -target[1]+5))){
      count++;
    if(count>=1)
    {  
      Serial.println("1\n");
    }
      }
    }  
}
  }

