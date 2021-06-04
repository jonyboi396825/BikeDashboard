/**
 * This code is kind of messy but it works
 * 
 * "Borrowed code":
 * https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/FullExample/FullExample.ino
*/

// define pins + constants
#define RX 2
#define TX 3
#define LEDPIN 4
#define ERRPIN 6
#define BUTTON1 7
#define BUTTON2 8
#define POT A1
#define CHIPSELECT 10 // cs pin

// Width and height of LCD
#define WID 16 
#define HEI 2

#define SMDELAYTIME 1000 // total delay = SMDELAYTIME + 1 sec

// include
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
#include <Timezone.h>

// variables
LiquidCrystal_I2C lcd(0x27, WID, HEI);
SoftwareSerial ss(RX, TX);
TinyGPSPlus gps;

// for adjusting to my time zone
TimeChangeRule usPST = {"PST", First, Sun, Nov, 2, -480}; // pacific standard time (winter) UTC-8
TimeChangeRule usPDT = {"PDT", Second, Sun, Mar, 2, -420}; // pacific daylight time (summer) UTC-7
Timezone usPacific(usPDT, usPST); // time offset will not be constant bc of daylight savings
time_t pacific; // time object

byte unit; // 0 = mph, 1 = kph, 2 = m/s
const char units[3][4] = {"mph", "kph", "m/s"}; // stores string of unit at the corresponding index, takes less memory
byte curhr, curmin; // cur hour and min to get from GPS
byte trackingState; // 0 = not tracking, 1 = tracking, 2 = paused
unsigned long startMillis; // startMillis to track seconds

float speedmph, speedkph, speedmps; // stores speed
float latitude, longitude; // stores current position
int y; // stores year
byte mo, d, h, m, s; // stores date and time

bool state1, state2; // variables for 1st and 2nd button states respectively
bool prevs1, prevs2; // stores previous states of buttons
bool first = true; // makes sure to print "DISCONNECTED" only once when ss not available
bool first2; // for creating files
bool first3 = true; // for disconnected when pos not accurate
bool turnA, turnB; // decides if red LED should turn on (A is for position error, B is for file error) (true = on, false = off)

// setup
void setup(void){
    // serial 
    Serial.begin(9600);
    ss.begin(9600);

    // pinMode inputs
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);
    pinMode(POT, INPUT);

    // pinMode outputs
    pinMode(LEDPIN, OUTPUT);
    pinMode(ERRPIN, OUTPUT);

    // initialize LCD
    lcd.init();
    lcd.backlight();

    // initialize millis
    startMillis = millis();

    //initialize SD
    if (!SD.begin(CHIPSELECT)){
        turnB = true; 
    }
}

// loop
void loop(void){
    readButtons(); // read states of buttons
    
    first = true;

    // turn off LED if not tracking
    if (trackingState == 0){
        first3 = true;
        digitalWrite(LEDPIN, LOW);
        digitalWrite(ERRPIN, LOW); // turns off bc not tracking or writing anything
    }
    // if currently tracking, log file every 5 seconds and turn on LED
    if (trackingState == 1){
        digitalWrite(LEDPIN, HIGH);
        // if first time here, create the file
        if(first2){
            first2 = false;
          
            if (getCurTime()){
                // if time is valid then print to file the start date and time
                writeToFile(40);
            } 
        }

        if (getCurPos()){
            first3 = true;
            turnA = false; // turn off red LED if location is valid
            writeToFile(10);
        } else{
            if (first3){
                writeToFile(30); // write DISCONNECTED if not valid
                first3 = false;
            }
            turnA = true; // if location is not valid, turn on red LED
        }

        digitalWrite(ERRPIN, (turnA || turnB)); // turn on LED if position has error OR file has error 
    }
    // if currently paused, blink the LED every second
    if (trackingState == 2){
        first3 = true;
        if ((millis()%2000) < 1000){
            digitalWrite(LEDPIN, HIGH);
        } else{
            digitalWrite(LEDPIN, LOW);  
        } 
        digitalWrite(ERRPIN, (turnA || turnB)); // turns LED to state of file
    }

    // get current min and sec from GPS
    byte mult = 1; // used to determine which of speed or time has error
    if(!getCurTime()){
        mult *= 2;
    }

    // get cur speed from GPS
    if (!getCurSpeed()){
        mult *= 3;  
    } 

    // display those onto LCD
    switch(unit){
        case 0:
            updateDisplay(true, mult, speedmph);
            break;
        case 1:
            updateDisplay(true, mult, speedkph);
            break;
        case 2:
            updateDisplay(true, mult, speedmps);
            break;
    }

    smartDelay(SMDELAYTIME); // smart delay so GPS can gather info

    // if gps is not available
    while (!ss.available()){
        // first, try to reconnect
        startMillis = millis();
        while ((millis()-startMillis) < (SMDELAYTIME+1000)){
            // if it reconnects, then break if not continue
            if (ss.available()) break;
            readButtons();
        }

        // if that works, then break, if not continue
        if (ss.available()) break;

        Serial.println("disc");
        
        readButtons(); // read states of buttons
        if (first){
            lcd.clear();
            if (trackingState == 1){
                // write DISCONNECTED to file
                writeToFile(30);  
            }
            first = false;
        }
        updateDisplay(false, 0, 0);
    }
}

// function implementations

// reads button inputs, call this in every while loop to avoid having to hold buttons
void readButtons(void){
    short readIn = analogRead(POT);
    unit = map(readIn, 0, 1024, 0, 3); // update unit to match potentiometer state

    lcd.setCursor(WID-3, 1);
    lcd.print(units[unit]);
    
    // get digital input of buttons
    state1 = digitalRead(BUTTON1); // get 1st button state  
    state2 = digitalRead(BUTTON2); // get 2nd button state
  
    // if first button pressed, change the tracking state to start if stopped and to stop if started
    if (prevs1 == LOW and state1 == HIGH){
        trackingState = !trackingState;
        digitalWrite(LEDPIN, trackingState);

        // create a new file if just started tracking
        if (trackingState == 1){
            // make first2 true to write to file
            first2 = true;
        }
    }

    // if second button pressed, change the tracking state to paused if not paused and to resume if paused 
    if (prevs2 == LOW and state2 == HIGH and trackingState != 0){
        trackingState = !(trackingState-1) + 1;
        
        if (trackingState == 2){
            getCurPos();
            writeToFile(20);  
        } else{
            digitalWrite(LEDPIN, HIGH);  
        }
    }

    // change previous state
    prevs1 = state1; 
    prevs2 = state2; 
}

// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples at
// https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/FullExample/FullExample.ino  
static void smartDelay(uint16_t ms) // I am only delaying for at most 10000 ms
{
    unsigned long start = millis();
    do
    {
        // If data has come in from the GPS module
        while (ss.available()){
            gps.encode(ss.read()); // Send it to the encode function
        }
        readButtons();

        // if currently paused, blink the LED every second
        if (trackingState == 2){
            if ((millis()%2000) < 1000){
                digitalWrite(LEDPIN, HIGH);
            } else{
                digitalWrite(LEDPIN, LOW);  
            } 
        }
        
        // tinyGPS.encode(char) continues to "load" the tinGPS object with new
        // data coming in from the GPS module. As full NMEA strings begin to come in
        // the tinyGPS library will be able to start parsing them for pertinent info
    } while (millis() - start < ms);
}

// get current speed
bool getCurSpeed(void){
    if (ss.available() && gps.speed.isValid()){
        speedmph = (float) gps.speed.mph();
        speedkph = (float) gps.speed.kmph();
        speedmps = (float) gps.speed.mps();
        
//        Serial.print("sp ");
//        Serial.print(_mph);
//        Serial.print("mph ");
//        Serial.print(_kph);
//        Serial.print("kph ");
//        Serial.print(_mps);
//        Serial.println("m/s");

        return true;
    } 
    return false;
}

// get current time
bool getCurTime(void){
    if (ss.available() && gps.date.isValid() && gps.time.isValid()){
        byte __h, __m, __s, __mth, __d;
        int __y;
        __h = gps.time.hour();
        __m = gps.time.minute();
        __s = gps.time.second();
        __y = gps.date.year();
        __mth = gps.date.month();
        __d = gps.date.day();

        setTime(__h, __m, __s, __d, __mth, __y); // set time from time library
        time_t _t = now();
        pacific = usPacific.toLocal(_t); // update time if it was accurate

        h = (byte) hour(pacific);
        m = (byte) minute(pacific);
        s = (byte) second(pacific);
        y = year(pacific);
        mo = (byte) month(pacific);
        d = (byte) day(pacific);
        
//        Serial.print("t ");
//        Serial.print(y);
//        Serial.print("-");
//        Serial.print(mo);
//        Serial.print("-");
//        Serial.print(d);  
//        Serial.print(" ");
//        Serial.print(h);
//        Serial.print(":");
//        Serial.print(m); 
//        Serial.print(":");
//        Serial.println(s);
        
        return true;
    }
    return false; 
}

// get current position
bool getCurPos(void){
    if (ss.available() && gps.location.isValid()){
        latitude = gps.location.lat();
        longitude = gps.location.lng();  

//        Serial.print("pos ");
//        Serial.print(c.latitude, 6);
//        Serial.print(" ");
//        Serial.print(c.longitude, 6);
//        Serial.println(" ");

        return true;
    }
    return false;
} 
// writes cur position to file
void writeToFile(byte code){
    // codes: 10 = write cur pos, 20 = write pause, 30 = write disconnected, 40 = write date time
  
    // open file, only write to the file if the time is valid
    File f = SD.open("course.txt", FILE_WRITE);
    if (f) {
        turnB = false; // turn error LED off
        // if file is initialized, then write and close
        switch (code){
          case 10:
              f.print(latitude, 6);
              f.print(",");
              f.println(longitude, 6);
              break;
          case 20:
              f.println("PAUSE");  
              break;
          case 30:
              f.println("DISCONNECTED");
              break;
          case 40:
              f.print("T ");
              f.print(y);
              f.print("-");
              f.print(mo);
              f.print("-");
              f.print(d);
              f.print(" ");
              f.print(h);
              f.print(":");
              f.print(m);
              f.print(":");
              f.print(s);
              f.println();
              break;
        }
        f.close(); // close file
    } else{
        // otherwise print error
        turnB = true; // toggle error LED
        Serial.println("Error opening file");  
    }
}

// prints data onto LCD
void updateDisplay(bool _av, int _b, float _curspeed){
    // (hour, minute, code, speed) 
    // if _b == 3, curspeed has an error
    // if _b == 2, time has an error
    // if _b == 6, curspeed and time have errors
    // curspeed will be given in the wanted unit  

    // if not available, print "not available" and return
    if (!_av){
        lcd.setCursor(0, 0);
        lcd.print(" NOT AVAILABLE  ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
        return;
    }

    // print time onto screen only if time is accurate
    if (_b != 2 && _b != 6){
        // print date
        lcd.setCursor(0, 0);
        if (calcLen(mo) == 1){ // add 0 in front if month is single digit
            lcd.print('0');
            lcd.setCursor(1, 0);
            lcd.print(mo);
        } else{
            lcd.print(mo);  
        }

        // print dash
        lcd.setCursor(2, 0);
        lcd.print('-');
        lcd.setCursor(3, 0);
        
        if (calcLen(d) == 1){ // add 0 in front if day is single digit
            lcd.print('0');
            lcd.setCursor(4, 0);
            lcd.print(d);
        } else{
            lcd.print(d);  
        }

        // print spaces
        lcd.setCursor(5, 0);
        for (int i = 0; i < WID-10; i++){
            lcd.setCursor(i+5, 0);
            lcd.print(' ');
        }

        // print time
        lcd.setCursor(WID-5, 0);
        if (calcLen(h) == 1){ // add 0 in front if hour is single digit
            lcd.print('0');
            lcd.setCursor(WID-4, 0);
            lcd.print(h);  
        } else{
            lcd.print(h);  
        }

        // print time colon
        lcd.setCursor(WID-3, 0);
        lcd.print(':');
        lcd.setCursor(WID-2, 0);

        if (calcLen(m) == 1){ // add 0 in front if min is single digit
            lcd.print('0');
            lcd.setCursor(WID-1, 0);
            lcd.print(m);
        } else{
            lcd.print(m);  
        }
    } else{ // if not valid, print "NO INFO"
        lcd.setCursor(0, 0);
        lcd.print("  NO TIME INFO  ");
    }

    // if there is a speed error print onto LCD
    if (_b == 3 || _b == 6){
        lcd.setCursor(WID-8, 1);
        lcd.print("ERROR");
    } else{
        lcd.setCursor(WID-10, 1);
        lcd.print("      ");
        lcd.setCursor(WID-3-(calcLen((int) floor(_curspeed))), 1);
        lcd.print((int) floor(_curspeed));
    }
    lcd.setCursor(WID-3, 1);
    lcd.print(units[unit]);

}

// calculates number of digits of a given number
byte calcLen(int a){
    if (a == 0) return 1;
    
    byte len = 0;
    while (a > 0){
        a/=10;
        len++;  
    }
    return len;
}
