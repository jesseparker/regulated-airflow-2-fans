#include <FastPID.h>
#define PRESSURESENSOR 5
#define FAN 3
//#define SETPOINT 55
#define RPM_DELAY 100

//float Kp=0.1, Ki=0.5, Kd=0, Hz=10;
float Kp=0.1, Ki=0.5, Kd=0.1, Hz=10;


int output_bits = 8;
bool output_signed = false;

double setPoint;
double rpms;
double outputVal;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

long int NbTopsFan;

void rpm ()
{ 
  NbTopsFan++;
}
 

void setup() {
  pinMode(PRESSURESENSOR, INPUT);
  pinMode(FAN, OUTPUT);
  //pinMode(SETPOINT, INPUT);

  Serial.begin(115200);
  
}
 
void loop ()
{
  NbTopsFan = 0;
  
//Enables interrupts
//sei();
 
//Wait 
//delay (RPM_DELAY);
 
//Disable interrupts
//cli();

rpms = analogRead(PRESSURESENSOR);

// Read the setpoint pot and map it to 0-200
setPoint = 45;

// Run the PID to figure out what the power should be
uint8_t outputVal = myPID.step(setPoint, rpms);

// Write the power value to the PWM output
analogWrite(FAN, outputVal);
 

// Serial info
Serial.print ("setpoint:");
Serial.print (setPoint, DEC);

Serial.print (" rpm:");
Serial.print (rpms, DEC);

Serial.print (" pwr:");
Serial.print (outputVal, DEC);
Serial.print ("\r\n");
} 
