#include "DigitalIn.h"
#include "PinNamesTypes.h"
#include "mbed.h"
#include <cstdio>
#include <ctime>

///////////////////////////////////////////////////////////
//STM'in üzerindeki ledi mavi buton ile açıp kapatma kodu//
///////////////////////////////////////////////////////////

int main()
{
    DigitalOut led(LED1); //Led ayarla 
    DigitalIn button(BUTTON1); //Buton ayarla
    button.mode(PullUp); //Buton basıldığında 1 bırakıldığında 0 olması için ayarla

    while (true) {
        if(button == true) //Buton basılı tutulduğunda ledi aç
            led = 1;
        else if (button == false) //Buton basılı değilse ledi kapat
            led = 0;
    }
}
