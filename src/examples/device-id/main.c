#include <stdio.h>
#include "pas-co2-serial-rpi.h"

int main()
{ 
    printf("CO2 example Device ID\n");
    Error_t ret;
    int16_t co2ppm;
    uint8_t prodID=0;
    uint8_t revID=0;
    
    ret = begin(true,false);        //First argument enables I2C and the second argument enables UART
    if(ret != XENSIV_PASCO2_OK)
    {   
        printf("CO2 intialization error\n");
        goto exitapp;
    }

    ret = getDeviceID(&prodID,&revID);
    
    printf("Product ID: %d\n",prodID);
    
    printf("Version ID: %d\n",revID);

exitapp:
    end(true,false);
    return 0;
}
