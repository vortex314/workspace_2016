#include <Arduino.h>


const int led = PB1;
volatile int c;
void setup(void) {
   pinMode(led,OUTPUT);
   digitalWrite(led,1); // turn led off
   int a=sizeof(int);
   int b = sizeof(long int);
   c=a+b;
   Serial1.begin(115200);
   Serial1.println(" Start up ");
}
int counter=0;
void loop(void) {
   digitalWrite(led,0); // turn led on
   delay(100);
   digitalWrite(led,1); // turn led off
   delay(900);
   Serial1.println(" Start up "+counter++);
}
