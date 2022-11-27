#include "DigitalIn.h"
#include "DigitalOut.h"
#include "PinNameAliases.h"
#include "PinNames.h"
#include "PinNamesTypes.h"
#include "ThisThread.h"
#include <cstdio>
#include "mbed.h"

DigitalOut trigger(D9);
DigitalOut led(LED1);
DigitalIn  echo(D10);
Timer sonar;
 
int main(){
    float mesafe = 0;
    int correction = 0;
    DigitalIn button(BUTTON1);
    button.mode(PullUp);
    sonar.reset();
    // measure actual software polling timer delays
    // delay used later in time correction
    // start timer
    sonar.start();
    // min software polling delay to read echo pin
    while(echo==2){};
    // stop timer
    sonar.stop();
    // read timer
    correction = sonar.elapsed_time().count();
    printf("Approximate software overhead timer delay is %d uS\n",correction);

//Loop to read Sonar distance values, scale, and print
    while(1){
        if(button == false){led = 0;
            // trigger sonar to send a ping
            trigger = 1;
            sonar.reset();
            wait_us(10.0);
            trigger = 0;
            //wait for echo high
            while(echo==0){};
            //echo high, so start timer
            sonar.start();
            //wait for echo low
            while(echo==1){};
            //stop timer and read value
            sonar.stop();
            //subtract software overhead timer delay and scale to cm
            mesafe = (sonar.elapsed_time().count()-correction)/58.0;
            printf("%d cm \n",(int)mesafe);
            //wait so that any echo(s) return before sending another ping
            ThisThread::sleep_for(20ms);
        }
        else{
            led = 1;
        }
    }
}
