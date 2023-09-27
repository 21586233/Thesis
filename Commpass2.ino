
// Include Libraries
#include "Arduino.h"
#include "HMC5883L.h"


// Pin Definitions



// Global variables and defines

// object initialization
HMC5883L compass;


// define vars for testing menu
const int timeout = 90000;       //define timeout of 10 sec
char menuOption = 0;
long time0;


void setup() 
{
    Serial.begin(9600);
    while (!Serial) ; 
    Serial.println("start");
    
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For declination angle 4'36E (positive)
    compass.begin(-25,45);
    menuOption = menu();
    
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{
    
    
    if(menuOption == '1') {
    // HMC5883L - Magnetometer (Compass) - Test Code
    //Read heading value from compass
    float compassHeading = compass.getHeadingDeg();
    Serial.print(F("Heading: ")); Serial.print(compassHeading); Serial.println(F("[°]"));

    }
    
    if (millis() - time0 > timeout)
    {
        menuOption = menu();
    }
    
}



// Menu function for selecting the components to be tested
// Follow serial monitor for instrcutions
char menu()
{

    Serial.println(F("\nWhich component would you like to test?"));
    Serial.println(F("(1) HMC5883L - Magnetometer (Compass)"));
    Serial.println(F("(menu) send anything else or press on board reset button\n"));
    while (!Serial.available());

    // Read data from serial monitor if received
    while (Serial.available()) 
    {
        char c = Serial.read();
        if (isAlphaNumeric(c)) 
        {   
            
            if(c == '1') 
          Serial.println(F("Now Testing HMC5883L - Magnetometer (Compass)"));
            else
            {
                Serial.println(F("illegal input!"));
                return 0;
            }
            time0 = millis();
            return c;
        }
    }
}
