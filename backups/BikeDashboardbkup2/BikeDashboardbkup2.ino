/*
 * TODO:
 * - err LED pin
*/

// define pins + constants
#define RX 2
#define TX 3
#define LEDPIN 4
//#define NEOPIN 5
#define ERRPIN 6
#define BUTTON1 7
#define BUTTON2 8
#define BUTTON3 9
#define POT A1

#define NUM_LEDS 12
#define BRIGHTNESS 40 

#define WID 16
#define HEI 2

#define SMDELAYTIME 2000 // total delay = SMDELAYTIME + 1 sec

// include
#include <LiquidCrystal_I2C.h>
//#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
#include <Timezone.h>

// variables
LiquidCrystal_I2C lcd(0x27, WID, HEI);
//Adafruit_NeoPixel ring(NUM_LEDS, NEOPIN, NEO_GRB);
SoftwareSerial ss(RX, TX);
TinyGPSPlus gps;
File f;

// for adjusting to my time zone
// put in flash memory
TimeChangeRule usPST = {"PST", First, Sun, Nov, 2, -480}; // pacific standard time (winter) UTC-8
TimeChangeRule usPDT = {"PDT", Second, Sun, Mar, 2, -420}; // pacific daylight time (summer) UTC-7
Timezone usPacific(usPDT, usPST); // this will not be constant bc of daylight savings

byte unit; // 0 = mph, 1 = kph, 2 = m/s
bool neoPixMode; // shows mode of neopixel, 0 = speed, 1 = clock
String h, m, b, curspeed; // strings of hr, min, bat% and speed for lcd display
byte curhr, curmin; // cur hour and min to get from GPS
byte trackingState; // 0 = not tracking, 1 = tracking, 2 = paused
String fileName; // current file name
unsigned long startMillis; // startMillis to track seconds

double speedmph, speedkph, speedmps; // stores speed
// stores current position
struct CurrentPos{
    double latitude;
    double longitude;  
};

bool state1, state2, state3; // variables for 1st, 2nd, 3rd button states respectively
bool prevs1, prevs2, prevs3; // stores previous states of buttons
CurrentPos cur; // store current position
bool first = true; // makes sure to print "DISCONNECTED" only once
bool first2; // for creating files

int me = 0; // <-- test variable, remove

// setup
void setup(){
    // serial 
    Serial.begin(9600);
    ss.begin(9600);

    // pinMode inputs
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);
    pinMode(BUTTON3, INPUT);
    pinMode(POT, INPUT);

    // pinMode outputs
    pinMode(LEDPIN, OUTPUT);
    pinMode(ERRPIN, OUTPUT);
    
    // initialize NeoPixel ring
//    ring.begin();
//    ring.show();
//    ring.setBrightness(BRIGHTNESS);

    // initialize LCD
    lcd.init();
    lcd.backlight();

    // initialize millis
    startMillis = millis();
}

// loop
void loop(){
    readButtons(); // read states of buttons
    
    // set first to true
    first = true;
    
    // if first button pressed, change the tracking state to start if stopped and to stop if started
    if (prevs1 == LOW and state1 == HIGH){
        trackingState = !trackingState;
    }

    // if second button pressed, change the tracking state to paused if not paused and to resume if paused 
    if (prevs2 == LOW and state2 == HIGH and trackingState != 0){
        trackingState = !(trackingState-1) + 1;

        if (trackingState == 2){
            getCurPos(cur);
//            Serial.println("a");
            writeToFile(cur, 20);  
        }
    }

    // if third button pressed, change NeoPixel mode
    if (prevs3 == LOW and state3 == HIGH){
        neoPixMode = !neoPixMode;
    }

    // turn off LED if not tracking
    if (trackingState == 0){
        f.close();
        digitalWrite(LEDPIN, LOW);
        digitalWrite(ERRPIN, LOW); // turns off bc not tracking
    }
    // if currently tracking, log file every 5 seconds and turn on LED
    if (trackingState == 1){
        digitalWrite(LEDPIN, HIGH);
//        Serial.println("a" + String(ss.available()));
        // if first time here, create the file
        if(first2){
            first2 = false;
          
            byte _h, _m, _s, _mth, _d;
            int _y;
            Serial.println("new file");
            if (getCurTime(_h, _m, _s, _y, _mth, _d)){
                // if time is valid then set the filename to the start date and time
                fileName = "";
                fileName += String(_y) ;
                fileName += "-";
                fileName += String(_mth);
                fileName += "-";
                fileName += String(_d);
                fileName += " ";
                fileName += String(_h);
                fileName += ":";
                fileName += String(_m);
                fileName += ":";
                fileName += String(_s);
            } else {
//              digitalWrite(ERRPIN, HIGH);
                fileName = "INVALID";  
            }
        }

        if (getCurPos(cur)){
            digitalWrite(ERRPIN, LOW); // turn off red LED if location is valid
            writeToFile(cur, 10);
        } else{
            writeToFile(cur, 30); // write DISCONNECTED if not valid
            digitalWrite(ERRPIN, HIGH); // if location is not valid, turn on red LED
        }
    }
    // if currently paused, blink the LED every second
    if (trackingState == 2){
        if ((millis()%2000) < 1000){
            digitalWrite(LEDPIN, HIGH);
        } else{
            digitalWrite(LEDPIN, LOW);  
        } 
        digitalWrite(ERRPIN, LOW); // turns off bc not tracking
    }

    // get current min and sec from GPS and speed
    byte t1, t3, t4;
    int t2;
    byte mult = 1;
    if(!getCurTime(curhr, curmin, t1, t2, t3, t4)){
        mult *= 2;
    }

    if (!getCurSpeed(speedmph, speedkph, speedmps)){
        mult *= 3;  
    }

//    Serial.print("mult test ");
//    Serial.print(mult);
//    Serial.print(" ");
//    Serial.print(curhr);
//    Serial.print(" ");
//    Serial.println(curmin);

    // display those onto LCD
    switch(unit){
        case 0:
            updateDisplay(true, curhr, curmin, mult, speedmph);
            break;
        case 1:
            updateDisplay(true, curhr, curmin, mult, speedkph);
            break;
        case 2:
            updateDisplay(true, curhr, curmin, mult, speedmps);
            break;
    }

    smartDelay(SMDELAYTIME); // smart delay so GPS can gather info

    // if gps is not available
    while (!ss.available()){
        // first, try to reconnect
        startMillis = millis();
        while (millis()-startMillis < SMDELAYTIME){
            // if it reconnects, then break if not continue
            if (ss.available()) break;
            readButtons();
        }

        // if that works, then break, if not continue
        if (ss.available()) break;

//        digitalWrite(ERRPIN, HIGH);

        Serial.println("disc");
        
        readButtons(); // read states of buttons
        if (first){
            lcd.clear();
            if (trackingState == 1){
//                Serial.println("b");
                writeToFile(cur, 30);  
            }
            first = false;
        }
        updateDisplay(false, 0, 0, 0, 0.0);
    }
}

// function implementations

// reads button inputs, call this in every while loop to avoid having to hold buttons
void readButtons(){
    short readIn = analogRead(POT);
    unit = map(readIn, 0, 1024, 0, 3); // update unit to match potentiometer state
//    Serial.println(String(readIn) + " " + String(unit));

    String _unit = ""; // update unit on screen
    switch(unit){
        case 0:
            _unit = "mph";
            break;
        case 1:
            _unit = "kph";
            break;
        default: 
            _unit = "m/s";  
    }
    lcd.setCursor(13, 1);
    lcd.print(_unit);
    
    // get digital input of buttons
    state1 = digitalRead(BUTTON1); // get 1st button state  
    state2 = digitalRead(BUTTON2); // get 2nd button state
    state3 = digitalRead(BUTTON3); // get 3rd button state
  
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
            getCurPos(cur);
//            Serial.println("a");
            writeToFile(cur, 20);  
        }
    }

    // if third button pressed, change NeoPixel mode
    if (prevs3 == LOW and state3 == HIGH){
        neoPixMode = !neoPixMode;
    }

    // change previous state
    prevs1 = state1; 
    prevs2 = state2; 
    prevs3 = state3; 
}

// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples. 
// "Borrowed" from SparkFun at https://learn.sparkfun.com/tutorials/gps-logger-shield-hookup-guide/example-sketch-tinygps-serial-streaming
static void smartDelay(uint16_t ms) // I am only delaying for at most 10000 ms
{
    unsigned long start = millis();
    do
    {
        // If data has come in from the GPS module
        while (ss.available()){
//            Serial.println("asdjflkamsdf.asmdlkfajsdklfaskldfjas");
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
bool getCurSpeed(double& _mph, double& _kph, double& _mps){
    if (ss.available() && gps.speed.isValid()){
        _mph = gps.speed.mph();
        _kph = gps.speed.kmph();
        _mps = gps.speed.mps();
        
        Serial.print("sp ");
        Serial.print(_mph);
        Serial.print("mph ");
        Serial.print(_kph);
        Serial.print("kph ");
        Serial.print(_mps);
        Serial.println("m/s");

        return true;
    } 
    return false;
}

// get current time
bool getCurTime(byte& _h, byte& _m, byte& _s, int& _y, byte& _mth, byte& _d){
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
        time_t pacific = usPacific.toLocal(_t); // update time if it was accurate

        _h = hour(pacific);
        _m = minute(pacific);
        _s = second(pacific);
        _y = year(pacific);
        _mth = month(pacific);
        _d = day(pacific);
        
        Serial.print("t ");
        Serial.print(_y);
        Serial.print("-");
        Serial.print(_mth);
        Serial.print("-");
        Serial.print(_d);  
        Serial.print(" ");
        Serial.print(_h);
        Serial.print(":");
        Serial.print(_m); 
        Serial.print(":");
        Serial.println(_s);
        
        return true;
    }
    return false;
}

// get current position
bool getCurPos(CurrentPos& c){
//    Serial.println("b" + String(ss.available()) + " " + String(gps.location.isValid()));
    if (ss.available() && gps.location.isValid()){
        c.latitude = gps.location.lat();
        c.longitude = gps.location.lng();  

        Serial.print("pos ");
        Serial.print(c.latitude, 6);
        Serial.print(" ");
        Serial.print(c.longitude, 6);
        Serial.println(" ");

        return true;
    }
    return false;
} 

// writes cur position to file
void writeToFile(CurrentPos c, byte code){
    // codes: 10 = write cur pos, 20 = write pause, 30 = write disconnected
  
    // open file, only write to the file if the time is valid
    f = SD.open(fileName, FILE_WRITE);
    if (f && fileName != "INVALID") {
        // if file is initialized, then write and close
        if (code == 10){
            f.println(String(c.latitude) + "," + String(c.longitude));
        } 
        else if (code == 20){
            f.println("PAUSE");  
        }
        else if (code == 30){
            f.println("DISCONNECTED");
        }
        f.close();
    } else{
        // otherwise print error
        // TOGGLE RED LED
        Serial.print("Error opening file: ");  
        Serial.println(fileName); 
    }
}

// prints data onto LCD
void updateDisplay(bool _av, int _h, int _m, int _b, double _curspeed){
    // (hour, minute, code, speed) 
    // if _b == 3, curspeed has an error
    // if _b == 2, time has an error
    // if _b == 6, curspeed and time have errors
    // curspeed will be given in the wanted unit  

    // if not available, print "error" and return
    if (!_av){
        lcd.setCursor(0, 0);
        lcd.print(" NOT AVAILABLE  ");
        return;
    }

    // round _curspeed
    curspeed = roundToString(_curspeed, 2);

    // if the speed is 0 when floored, just display 0
    if (floor(_curspeed) == 0.0){
        curspeed = "0";  
    }
    
    // if you somehow go above 9999 mph, kph, or m/s
    if (curspeed.length() > 7 || _b == 3 || _b == 6){
        curspeed = "ERROR";
    }
    
    // add 0 if _h has only one digit
    if (_h/10 == 0){
        h = "0" + String(_h);
    } else{
        h = String(_h);
    }

    // add 0 if _m has only one digit
    if (_m/10 == 0){
        m = "0" + String(_m);  
    } else{
        m = String(_m);  
    }

    // print time onto screen only if time is accurate
    if (_b != 2 && _b != 6){
        lcd.setCursor(0, 0);
        lcd.print("Time: " + h + ":" + m + "     ");
    } else{
        lcd.setCursor(0, 0);
        lcd.print("Time: NO INFO   ");
    }

    // print speed onto screen
    String _buf = ""; // how many spaces to print since I am formatting the speed from the right
    for (int i = 0; i < WID-9-curspeed.length(); i++){
        _buf += " ";  
    }

    String _unit = ""; // for unit
    switch(unit){
        case 0:
            _unit = "mph";
            break;
        case 1:
            _unit = "kph";
            break;
        default: 
            _unit = "m/s";  
    }
    
    lcd.setCursor(0, 1);
    lcd.print("Speed:" + _buf + curspeed + _unit);
}

// rounds a double to n digits and returns a string
String roundToString(double _d, byte _n){ // _n will be at most 2 in this program
    String _s = String(_d);
    return _s.substring(0, _s.indexOf(".") + _n + 1);
}