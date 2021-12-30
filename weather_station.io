#include <SoftwareSerial.h> 
#include <SparkFunESP8266WiFi.h>
#include <Wire.h> //I2C needed for sensors
#include <RTClib.h>
#include "SparkFunMPL3115A2.h" //Pressure sensor
#include "SparkFun_Si7021_Breakout_Library.h" //Humidity sensor 

//wifi SSID and password settings
const char mySSID[] = "enter_your_ssid";
const char myPSK[] = "enter_your_password";

//Wunderground station ID and password
String WU_pwsID = "ENTER_YOUR_WUNDERGROUND_ID";
String WU_pwsPASSWORD = "ENTER_YOUR_WUNDERGROUND_PASSWORD";

MPL3115A2 myPressure; //Create an instance of the pressure sensor
Weather myHumidity;//Create an instance of the humidity sensor
RTC_Millis rtc; //defines Real Time Clock

// declaring variables for pins on arduino board
// digital i/o pins
const byte WIND_SPEED = 3; //wind speed
const byte RAIN = 2; //rain gauge
const byte STAT_BLUE = 7; //blue LED
const byte STAT_GREEN = 8; //green LED

// analog i/o pins
const byte REFERENCE_3V3 = A3; // This is the 3.3 volt rail to power the light sensor
const byte LIGHT = A1; //light sensor
const byte WIND_DIR = A0; //wind direction

int winddir = 0;
float windspeed = 0;
volatile byte windclicks = 0;//1 click = 1.492 mph
long lastWindCheck = 0;

float baromin;

const char destServer[] = "rtupdate.wunderground.com";

const String httpRequest = "GET /weatherstation/updateweatherstation.php?ID="+WU_pwsID+"&PASSWORD="+WU_pwsPASSWORD+"&dateutc=now&tempf=63&baromin="+baromin+"&action=updateraw&realtime=1&rtfreq=60 HTTP/1.1\n"
                           "Host: rtupdate.wunderground.com\n"
                           "Connection: close\n\n";

DateTime now = rtc.now(); //DateTime is an object that 
int lastDay;
long millicounter; //counts milliseconds to seconds

//volatile because the rain gauge will change this value
volatile float dailyrainin = 0; //rain over past 24 hours

///////////////////////////////////

void raingauge() 
{
    static uint32_t
        lastInt = 0ul;
    uint32_t nowInt = micros();
    
    if( (nowInt - lastInt) >= 10000ul )//accounting for bounce in sensor readings
    {
        lastInt = nowInt;
        
        dailyrainin += 0.011; //rain gauge bucket is 0.011 inches
      
        if (now.day() != lastDay) 
        {
            dailyrainin = 0;
        }      
        lastDay = now.day();       
    }
}

///////////////////////////////////
///////////////////////////////////

void get_wind_clicks()
{
  static uint32_t
    lastInt = 0ul;
  uint32_t nowInt = micros();
    
  if( (nowInt - lastInt) >= 20000ul )//accounting for bounce in sensor readings
  {
    windclicks++;
  }
}

///////////////////////////////////
///////////////////////////////////

int get_wind_direction()
{
  unsigned int adc;

  adc = analogRead(WIND_DIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}

///////////////////////////////////

///////////////////////////////////
///////////////////////////////////
///////////////////////////////////
void setup() {
  Serial.begin(9600);
  serialTrigger(F("Press any key to begin."));

  initializeESP8266();

  connectESP8266();

  displayConnectInfo();

  //setting up pins as either input or output
  pinMode(STAT_BLUE, OUTPUT); //sets up blue LED
  pinMode(STAT_GREEN, OUTPUT); //sets up green LED
  pinMode(REFERENCE_3V3, INPUT); //power source -  3.3 volt rail- powers light sensor
  pinMode(LIGHT, INPUT); //sets up light sensor
  pinMode(RAIN, INPUT_PULLUP); //set up rain gauge
  pinMode(WIND_SPEED, INPUT_PULLUP); //setup wind speed anemometer
  
  //set up pressure sensor
  myPressure.begin();
  myPressure.setModeBarometer(); //measure pressure in Pascals
  myPressure.setOversampleRate(7); //sets the # of samples to be taken
  myPressure.enableEventFlags(); //enables the temp and pressure event flags
  
  //set up humidity sensor
  myHumidity.begin();
  millicounter = millis(); //built in arduino function, counts milliseconds to seconds
  
  //setup raingauge as interrupting hardware
  attachInterrupt(digitalPinToInterrupt(2), raingauge, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), get_wind_clicks, FALLING);
  
  // turn on interrupts
  interrupts();

  Serial.println("Weather station online");
}

///////////////////////////////////
///////////////////////////////////

void initializeESP8266()
{
  int test = esp8266.begin();
  if (test != true)
  {
    Serial.println(F("Error talking to ESP8266."));
    errorLoop(test);
  }
  Serial.println(F("ESP8266 Shield Present"));
}

///////////////////////////////////
///////////////////////////////////

void connectESP8266()
{
  // The ESP8266 can be set to one of three modes:
  //  1 - ESP8266_MODE_STA - Station only
  //  2 - ESP8266_MODE_AP - Access point only
  //  3 - ESP8266_MODE_STAAP - Station/AP combo
  // Use esp8266.getMode() to check which mode it's in:
  int retVal = esp8266.getMode();
  if (retVal != ESP8266_MODE_STA)
  { // If it's not in station mode.
    // Use esp8266.setMode([mode]) to set it to a specified
    // mode.
    retVal = esp8266.setMode(ESP8266_MODE_STA);
    if (retVal < 0)
    {
      Serial.println(F("Error setting mode."));
      errorLoop(retVal);
    }
  }
  Serial.println(F("Mode set to station"));

  // esp8266.status() indicates the ESP8266's WiFi connect
  // status.
  // A return value of 1 indicates the device is already
  // connected. 0 indicates disconnected. (Negative values
  // equate to communication errors.)
  retVal = esp8266.status();
  if (retVal <= 0)
  {
    Serial.print(F("Connecting to "));
    Serial.println(mySSID);
    // esp8266.connect([ssid], [psk]) connects the ESP8266
    // to a network.
    // On success the connect function returns a value >0
    // On fail, the function will either return:
    //  -1: TIMEOUT - The library has a set 30s timeout
    //  -3: FAIL - Couldn't connect to network.
    retVal = esp8266.connect(mySSID, myPSK);
    if (retVal < 0)
    {
      Serial.println(F("Error connecting"));
      errorLoop(retVal);
    }
  }
}

///////////////////////////////////
///////////////////////////////////

void displayConnectInfo()
{
  char connectedSSID[24];
  memset(connectedSSID, 0, 24);
  // esp8266.getAP() can be used to check which AP the
  // ESP8266 is connected to. It returns an error code.
  // The connected AP is returned by reference as a parameter.
  int retVal = esp8266.getAP(connectedSSID);
  if (retVal > 0)
  {
    Serial.print(F("Connected to: "));
    Serial.println(connectedSSID);
  }

  // esp8266.localIP returns an IPAddress variable with the
  // ESP8266's current local IP address.
  IPAddress myIP = esp8266.localIP();
  Serial.print(F("My IP: ")); Serial.println(myIP);
}

///////////////////////////////////
///////////////////////////////////

void clientDemo()
{
  // To use the ESP8266 as a TCP client, use the 
  // ESP8266Client class. First, create an object:
  ESP8266Client client;

  // ESP8266Client connect([server], [port]) is used to 
  // connect to a server (const char * or IPAddress) on
  // a specified port.
  // Returns: 1 on success, 2 on already connected,
  // negative on fail (-1=TIMEOUT, -3=FAIL).
  int retVal = client.connect(destServer, 80);
  if (retVal <= 0)
  {
    Serial.println(F("Failed to connect to server."));
    Serial.println(retVal);
    return;
  }

  // print and write can be used to send data to a connected
  // client connection.
  client.print(httpRequest);

  // available() will return the number of characters
  // currently in the receive buffer.
  while (client.available())
    Serial.write(client.read()); // read() gets the FIFO char
  
  // connected() is a boolean return value - 1 if the 
  // connection is active, 0 if it's closed.
  if (client.connected())
    client.stop(); // stop() closes a TCP connection.
}

///////////////////////////////////
///////////////////////////////////

void serialTrigger(String message)
{
  Serial.println();
  Serial.println(message);
  Serial.println();
  while (!Serial.available())
    ;
  while (Serial.available())
    Serial.read();
}

///////////////////////////////////
///////////////////////////////////

void errorLoop(int error)
{
  Serial.print(F("Error: ")); Serial.println(error);
  Serial.println(F("Looping forever."));
  for (;;)
    ;
}

///////////////////////////////////
///////////////////////////////////
///////////////////////////////////
///////////////////////////////////
void loop() {
  
  //wind speed calculations
  float deltaTime = millis() - lastWindCheck;
  deltaTime /= 1000.0; //Convert to seconds
  windclicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();
  windspeed *= 1.492;
    
  float tempf = myHumidity.getTempF();
  baromin = myPressure.readPressure();
  float humidity = myHumidity.getRH();
  float windspeedmph = (float)windclicks / deltaTime;
  winddir = get_wind_direction();
  


  //if sensor is dead
  if (humidity == 998){
    Serial.println("Humidity sensor not responding.");
      
    //re-initialize and try again
    myPressure.begin();
    myPressure.setModeBarometer(); 
    myPressure.setOversampleRate(7);
    myPressure.enableEventFlags();
    myHumidity.begin();
  }
  
  serialTrigger(F("Press any key to connect client."));
  clientDemo();
  Serial.println("Weather data uploaded");

}
///////////////////////////////////
///////////////////////////////////
///////////////////////////////////
