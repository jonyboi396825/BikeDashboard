/*
 * TODO:
 * - test LED pin
 * - add error LED
*/

// define pins + constants
#define RX 2
#define TX 3
#define LEDPIN 4
#define NEOPIN 5
#define ERRPIN 6
#define BUTTON1 7
#define BUTTON2 8
#define BUTTON3 9
#define POT A1

#define NUM_LEDS 12
#define BRIGHTNESS 40 

#define WID 16
#define HEI 2

// include
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>

// variables
LiquidCrystal_I2C lcd(0x27, WID, HEI);
Adafruit_NeoPixel ring(NUM_LEDS, NEOPIN, NEO_GRB);
SoftwareSerial ss(RX, TX);
TinyGPSPlus gps;
File f;

byte unit; // 0 = mph, 1 = kph, 2 = m/s
bool neoPixMode; // shows mode of neopixel, 0 = speed, 1 = clock
String h, m, b, curspeed; // strings of hr, min, bat% and speed
byte trackingState; // 0 = not tracking, 1 = tracking, 2 = paused
String fileName; // current file name
long startMillis; // startMillis to track seconds

double speedmph, speedkph, speedmps; // stores speed
// stores current position
struct CurrentPos{
    double latitude;
    double longitude;  
};

bool prevs1, prevs2, prevs3; // stores previous states of buttons
bool first = true; // makes sure to print "DISCONNECTED" only once

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
    
    // initialize NeoPixel ring
    ring.begin();
    ring.show();
    ring.setBrightness(BRIGHTNESS);

    // initialize LCD
    lcd.init();
    lcd.backlight();

    // initialize millis
    startMillis = millis();
}

// loop
void loop(){
    // loop variables
    CurrentPos cur; // store current position
    bool state1, state2, state3; // variables for 1st, 2nd, 3rd button states respectively

    // keep running while gps is available
    while (ss.available()){
        // more local variables
        unit = map(analogRead(POT), 0, 1024, 0, 3); // update unit to match potentiometer state
        state1 = digitalRead(BUTTON1); // get 1st button state
        state2 = digitalRead(BUTTON2); // get 2nd button state
        state3 = digitalRead(BUTTON3); // get 3rd button state
        
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
                Serial.println("a");
                writeToFile(cur, 20);  
            }
        }
    
        // if third button pressed, change NeoPixel mode
        if (prevs3 == LOW and state3 == HIGH){
            neoPixMode = !neoPixMode;
        }

         // gps needs this to work
        if(gps.encode(ss.read())){
            // if not currently tracking, turn LED off
            if (trackingState == 0){
                digitalWrite(LEDPIN, LOW);
            }
            // if currently tracking, log file every 5 seconds and turn on LED
            if (trackingState == 1){
                digitalWrite(LEDPIN, HIGH);
                if (millis() - startMillis >= 5000){ 
                    getCurPos(cur);
                    Serial.println(String(cur.latitude) + " " + String(cur.longitude)); // makes sure if GPS works
                    writeToFile(cur, 10);
                    startMillis = millis();
                }  
            }
            // if currently paused, blink the LED every 1 second
            if (trackingState == 2){
                if ((millis()%2000) < 1000){
                    digitalWrite(LEDPIN, HIGH);
                } else{
                    digitalWrite(LEDPIN, LOW);  
                }
            }
    
            // change previous state
            prevs1 = state1; 
            prevs2 = state2; 
            prevs3 = state3; 
        }
    }

    // if gps is not available
    while (!ss.available()){
        state1 = digitalRead(BUTTON1); // get 1st button state
        state2 = digitalRead(BUTTON2); // get 2nd button state
        state3 = digitalRead(BUTTON3); // get 3rd button state
      
        if (first){
            lcd.clear();
            Serial.println("b");
            writeToFile(cur, 30);  
            first = false;
        }
        // testing code (remove later) vvv
//        updateDisplay(true, me++ % 24, me++ % 60, 123456, 121.21382713); 
        delay(500);
        // end testing code ^^^
        updateDisplay(false, 0, 0, 0, 0.0);
        
        // change previous state
        prevs1 = state1; 
        prevs2 = state2; 
        prevs3 = state3; 
    }

    
}

// function implementations

// get current time
void getCurTime(int& h, int& m){
    if (ss.available()){
        h = gps.time.hour();
        m = gps.time.minute();
    }
}

// get current position
void getCurPos(CurrentPos& c){
    if (ss.available()){
        c.latitude = gps.location.lat();
        c.longitude = gps.location.lng();  
    }
}

// writes cur position to file
void writeToFile(CurrentPos c, byte code){
    // codes: 10 = write cur pos, 20 = write pause, 30 = write disconnected
  
    // open file
    f = SD.open(fileName, FILE_WRITE);
    if (f) {
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
        Serial.println("Error opening file");  
    }
}

// prints data onto LCD
void updateDisplay(bool _av, int _h, int _m, int _b, double _curspeed){
    // (hour, minute, batt% (later), speed) 
    // curspeed will be given in the wanted unit  

    // if not available, print "error" and return
    if (!_av){
        lcd.setCursor(1, 0);
        lcd.print("NOT AVAILABLE");
        return;
    }

    // round _curspeed
    curspeed = roundToString(_curspeed, 2);

    // if you somehow go above 9999 mph, kph, or m/s
    if (curspeed.length() > 7){
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

    // print time onto screen
    lcd.setCursor(0, 0);
    lcd.print("Time: " + h + ":" + m + "     ");

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
String roundToString(double _d, int _n){
    String _s = String(_d);
    return _s.substring(0, _s.indexOf(".") + _n + 1);
}
