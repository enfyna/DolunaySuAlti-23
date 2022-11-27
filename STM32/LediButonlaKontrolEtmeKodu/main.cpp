#include "DigitalIn.h"
#include "PinNamesTypes.h"
#include "mbed.h"
#include <cstdio>
#include <ctime>

//STM'in üzerindeki ledi mavi buton ile açıp kapatma kodu
//

int main()
{
    DigitalOut led(LED1); //Led ayarla 
    DigitalIn button(BUTTON1); //Buton ayarla
    button.mode(PullUp); //Buton basıldığında 1 bırakıldığında 0 olması için ayarla
    int sure = 0; 

    while (true) {
        if(button == true){
            sure++;
        }
        else if (button == false) {
            if(sure>0){
                sure--;
            } 
        }
        if(sure>0){
        led = !led;
        printf("%d\n",sure);
        ThisThread::sleep_for(sure);
        }
        else if (sure == 0){
            led = true;
        }
    }
}
