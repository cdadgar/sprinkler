/*
 * module is a esp-1
 * flash size set to 1M (128K SPIFFS)
 * 
 * to program, slide switch to point nearest the button, then press the button
 * (the port montor should show some giberish.  it if shows normal debug output, the esp is running, and not in programing mode.
 * 
 * slide switch farthest away from button to go into run mode.
 * 
 * 
 * also program the spiffs:
 * close the serial port monitor.  this fails if its open.
 * enter programming mode. select tools->sketch data upload
 * 
 * note:  must use the profilic usb to serial to get the spiffs upload to work.  its fails with the other converter
 * if the profilic port isn't working, trying using the old version of the driver 3.3.3.114
 * 
 * 
 * 
 * during upload, I sometimes get a few
 * LmacRxBlk:1
 * messages....they don't seem to cause a problem
 * 
 * after upload, you should see: (for a good reboot)
Update Success: 296592
Rebooting...

 ets Jan  8 2013,rst cause:2, boot mode:(3,6)

load 0x4010f000, len 1264, room 16 
tail 0
chksum 0x42
csum 0x42
@cp:0
ld


this also happens:
Update Success: 296592
Rebooting...

 ets Jan  8 2013,rst cause:2, boot mode:(1,7)


 ets Jan  8 2013,rst cause:4, boot mode:(1,7)

wdt reset

and the device hangs until power cycled

what does the boot mode refer to?  are the pins being pulled up/down correctly? (with resistors)
 */

/*
 * LiquidCrystal_I2C - https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library (git)

#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <TimeLib.h> 
//#include <Timezone.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// --------------------------------------------

#include <ESP8266WebServer.h>
#include <FS.h>

// --------------------------------------------

// wifi manager includes
#include <DNSServer.h>
#include <WiFiManager.h>

// --------------------------------------------

// aync library includes
#include <ESPAsyncTCP.h>
#include <ESPAsyncUDP.h>

// --------------------------------------------

// display includes
#include <LiquidCrystal_I2C.h>

// --------------------------------------------

#define WU_API_KEY  "76d6efe987659cd1" // sprinkler key
#define WU_LOCATION "34241"
// weather station in serenoa...but it doesn't provide us the fields
// we need for the zimmerman calculation
//#define WU_LOCATION "pws:MD0683"
#define WUNDERGROUND "api.wunderground.com"
const char WUNDERGROUND_REQ1[] = "GET /api/";
// getting the conditions and history in a singl query (the response is large)
const char WUNDERGROUND_REQ2[] = "/conditions/history/q/";
const char WUNDERGROUND_REQ3[] = ".json HTTP/1.1\r\n"
    "User-Agent: ESP8266/0.1\r\n"
    "Accept: */*\r\n"
    "Host: " WUNDERGROUND "\r\n"
    "Connection: close\r\n"
    "\r\n";

#define LOGGING_IP_ADDR "192.168.1.210"
#define LOGGING_IP_PORT 80
#define LOG_URL "http://cpd.us.to:90/sprinkler/"

#define TORRENT_IP_ADDR "192.168.1.210"
#define TORRENT_IP_PORT 9091

#define TEMP_ERROR -999

// --------------------------------------------


/*
note: had to modify LiquidCrystal_I2C.cpp and remove Wire.begin()
so that we can reconfigure the i2c pins
could have also used Wire.pins(sda,scl), but it has been deprecated

the 16x2 display needs to run on 5v so that the brightness is visible.
all the the other lines work at 3.3V
*/

#define SDA 0
#define SCL 2

// --------------------------------------------
// using the PCF8674P which has a fixed address of 100.
#define MUX1      0x20    // 00100000   PCF8574A, A0, A1, and A2 to GND
#define MUX2      0x21    // 00100001   PCF8574A, A1, and A2 to GND, A0 to VCC
// using the PCF8674AP which has a fixed address of 111.
#define MUX1_AP   0x38    // 00111000   PCF8574AP, A0, A1, and A2 to GND
#define MUX2_AP   0x39    // 00111001   PCF8574AP, A1, and A2 to GND, A0 to VCC

// the chip type config calue uses this to set the right chip
// chip type is either P or AP
#define MUX_ADD   0x18    // 00011000   add this to the address of a A chip to make it the address of a AP chip


#ifdef OLD
// p0 not used
#define UP     0x40
#define ENTER  0x20
#define DOWN   0x10

#define ZONE1  0x04
#define ZONE2  0x08
#define ZONE3  0x04
#define ZONE4  0x02
#define ZONE5  0x01
#define ZONE6  0x40
#define ZONE7  0x20
#define ZONE8  0x10
#define ZONE9  0x01
#define ZONE10 0x02

// the rain sesnor is normally closed, and opens up when it gets wet
#define RAIN_SENSOR_TRIGGERED HIGH
#define rainSensorMask  0x08
#define rainSensorAddr  MUX1

#define pumpMask  0x80
#define pumpAddr  MUX2

byte zoneMasks[] = { ZONE1, ZONE2, ZONE3, ZONE4, ZONE5, ZONE6, ZONE7, ZONE8, ZONE9, ZONE10 };
byte zoneAddrs[] = { MUX2,  MUX2,  MUX1,  MUX1,  MUX1,  MUX2,  MUX2,  MUX2,  MUX2,  MUX2   };
#else
#define UP     0x04  // P2  MUX1
#define ENTER  0x02  // P1  MUX1
#define DOWN   0x01  // P0  MUX1

#define ZONE1  0x20  // P5  MUX1
#define ZONE2  0x40  // P6  MUX1
#define ZONE3  0x80  // P7  MUX1
#define ZONE4  0x01  // P0  MUX2
#define ZONE5  0x02  // P1  MUX2
#define ZONE6  0x04  // P2  MUX2
#define ZONE7  0x08  // P3  MUX2
#define ZONE8  0x10  // P4  MUX2
#define ZONE9  0x20  // P5  MUX2
#define ZONE10 0x40  // P6  MUX2

// the rain sesnor is normally closed, and opens up when it gets wet
#define RAIN_SENSOR_TRIGGERED HIGH
#define rainSensorMask  0x08  // P3
#define rainSensorAddr  MUX1

#define pumpMask  0x10  // P4
#define pumpAddr  MUX1

byte zoneMasks[] = { ZONE1, ZONE2, ZONE3, ZONE4, ZONE5, ZONE6, ZONE7, ZONE8, ZONE9, ZONE10 };
byte zoneAddrs[] = { MUX1,  MUX1,  MUX1,  MUX2,  MUX2,  MUX2,  MUX2,  MUX2,  MUX2,  MUX2   };
#endif



const int buttonPins[] = {UP, ENTER, DOWN};
const int defaultButtonState = HIGH;

int numButtons;
int *buttonStates;
int *lastButtonStates;

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long *lastDebounceTimes;  // the last time the output pin was toggled
const long debounceDelay = 50;    // the debounce time

// --------------------------------------------

/*
 * mux1 usage:
 * 7 - n.c.
 * 6 - up button
 * 5 - enter button
 * 4 - down button
 * 3 - rain sensor
 * 2 - pump
 * 1 - zone 1
 * 0 - zone 2
 * int - n.c.
 * 
 * mux2 usage:
 * 7 - zone 3
 * 6 - zone 4
 * 5 - zone 5
 * 4 - zone 6
 * 3 - zone 7
 * 2 - zone 8
 * 1 - zone 9
 * 0 - zone 10
 * int - n.c.
 */

// --------------------------------------------

//US Eastern Time Zone (New York, Detroit)
//TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
//TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
//Timezone myTZ(myDST, mySTD);

const char *weekdayNames[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

// --------------------------------------------

#define MenuIdleTime                 10000

LiquidCrystal_I2C lcd(0x27,16,2);
bool isSetup = false;
bool isDisplayOn;
unsigned long lastButtonPressTime;
unsigned long lastMinutes;
unsigned long lastSeconds;

enum screenState { MAIN, MENU };
screenState screen;
int selectedMenuItem;

#define OFF    0
#define RUN    1
const char *modeNames[] = { "Off    ", "Running" };

bool isPumpOn;
bool isManual;
bool isRain;
unsigned int rainTime;
unsigned int rainBlackoutTime;

// actions
#define RAIN_SENSOR_ACTIVE    't'
#define RAIN_SENSOR_NORMAL    'n'
#define RAIN_BLACKOUT_OVER    'b'
#define PROGRAMS_RUN          'r'
#define PROGRAMS_OFF          'o'

#define ZONE_WATERING         'w'
#define ZONE_SKIPPED          's'
#define ZONE_MANUAL           'm'


#define NONE           -1
#define MANUAL_BUTTON   0
#define MODE_BUTTON     1

#define TIME_OFF 255
#define NUM_PROGRAMS 4
#define NUM_START_TIMES 3
#define NUM_ZONES 10

typedef struct {
  byte isEnabled;
  byte dayMask;
  byte startTime[NUM_START_TIMES];
  byte duration[NUM_ZONES];
} programType;

programType program[NUM_PROGRAMS];

typedef struct {
  char name[13];
  bool usesRainSensor;
  bool usesPump;
} zoneType;

zoneType zone[NUM_ZONES];

int runningProgram;
int runningZone;
int remainingZoneMinutes;
int remainingZoneSeconds;
bool isName;

ESP8266WebServer server(80);
File fsUploadFile;
bool isUploading;

WebSocketsServer webSocket = WebSocketsServer(81);
int webClient = -1;
int programClient = -1;
int zoneClient = -1;
int setupClient = -1;
int testClient = -1;

int testAddr = -1;
int testValue = -1;
unsigned int testHeap = 0;

bool isTimeSet = false;

bool isPromModified;
bool isMemoryReset = false;
//bool isMemoryReset = true;

typedef struct {
  byte mode;
  byte display_timeout;
  byte water_adjust;
  byte use_zimmerman;
  char wu_key[20];
  char wu_location[20];
  byte use_logging;
  char logging_ip_addr[17];
  int  logging_ip_port;
  char log_url[40];
  byte rainBlackoutMultiplier;
  byte use_torrent;
  char torrent_ip_addr[17];
  int  torrent_ip_port;
  byte torrent_delay;
  char chip_type[3];
} configType;

configType config;

int actual_water_adjust;


void send(byte, byte);
void setupDisplay(void);
void setupButtons(void);
void loadConfig(void);
void loadProgramConfig(void);
void loadZoneConfig(void);
bool setupWifi(void);
void setupTime(void);
void setupWebServer(void);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght);
void drawMainScreen(void);
void set(char *name, const char *value);
void saveZoneConfig(void);
void saveProgramConfig(void);
unsigned long sendNTPpacket(IPAddress& address);
void printTime(bool isCheckProgram, bool isDisplay, bool isTest);
void displayBacklight(bool);
void printWatering(bool);
void printModeState(bool);
void printRainState(bool);
void sendWeb(const char *command, const char *value);
void checkTimeMinutes(void);
void checkTimeSeconds(void);
void flashTime(unsigned long time);
void checkButtons(void);
void eraseTime(void);
void checkRemainingWatering(void);
int getRainSensor(void);
void startNextZone(void);
void doButton(int pin);
void doUp(void);
void doDown(void);
void doEnter(void);
void setSelectedMenuItem(int item);
void startPreviousZone(void);
void setSelectedProgramItem(int item);
void drawMenuScreen(void);
void doMenuEnter(void);
void doProgramEnter(void);
void drawMenuScreen(void);
void saveConfig(void);
void stopCurrentZone(void);
void stopProgram(void);
void startProgram(int index);
void startProgramAfterAdjust(void);
void drawProgramScreen(void);
void checkProgram(int day, int h, int m);
void update(int addr, byte data);
void rainChange(void);
void logRainState(void);
String getId(void);
void logAction(char action);
void logNames(void);
void logZone(int zone, int duration, char action, int percentage);
void requestZimmermanAdjust(void);
void requestZimmermanAdjustNow(void);
void startTorrents(String id);
void stopTorrents(String id);



void setup() {
  // start serial port
  Serial.begin(115200);
  Serial.print(F("\n\n"));

  Wire.begin(SDA,SCL);

  Serial.println(F("esp8266 sprinkler"));
  Serial.println(F("compiled:"));
  Serial.print( __DATE__);
  Serial.print(F(","));
  Serial.println( __TIME__);

  setupDisplay();
  setupButtons();

  if (!setupWifi())
    return;
    
  // must specify amount of eeprom to use (max is 4k?)
  EEPROM.begin(512);
  
  loadConfig();
  loadProgramConfig();
  loadZoneConfig();
  isMemoryReset = false;

  if (strcmp(config.chip_type,"P") == 0) {
    send(MUX1, 0xFF);
    send(MUX2, 0xFF);
  }
  else {
    send(MUX1+MUX_ADD, 0xFF);
    send(MUX2+MUX_ADD, 0xFF);
  }

  setupTime();
  setupWebServer();  

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
    
  lastMinutes = 0;
  lastSeconds = 0;

  isPumpOn = false;
  isManual = false;
  isRain = false;
  rainTime = 0;
  runningProgram = -1;
  runningZone = -1;
  remainingZoneMinutes = 0;
  remainingZoneSeconds = 0;
  isName = false;
  
  isUploading = false;

  isSetup = true;
  drawMainScreen();
}

void initZone(void) {
/*
zone      name
1         front rotor
2         front shrub
3         pool side
4         back front
5         back rear
6         driveway
7         pool drip
8         pool shrub
9         back shrub
10        side shrub

all zones use the rain sensor except for the pool drip
 */
  for (int j=0; j < NUM_ZONES; ++j) {
    zone[j].usesRainSensor = true;
    zone[j].usesPump = true;
  }
     
  set(zone[0].name, "Front Rotor");
  set(zone[1].name, "Front Shrub");
  set(zone[2].name, "Pool Side");
  set(zone[3].name, "Back Front");
  set(zone[4].name, "Back Rear");
  set(zone[5].name, "Driveway");
  set(zone[6].name, "Pool Drip");
  set(zone[7].name, "Pool Shrub");
  set(zone[8].name, "Back Shrub");
  set(zone[9].name, "Side Shrub");

  zone[6].usesRainSensor = false;

  zone[6].usesPump = false;
  zone[7].usesPump = false;
  zone[8].usesPump = false;
  zone[9].usesPump = false;
  
  for (int i=0; i < NUM_ZONES; ++i)
    Serial.printf("zone %d: %s %d %d\n", i, zone[i].name,
      zone[i].usesRainSensor, zone[i].usesPump);

  saveZoneConfig(); 
}

void initProgram(void) {
/*
sprinkler timer main
3:30am, tue and sat
zone      location            duration
1         front rotor         30
2         froint shrub        20
3         pool side           30
4         back front          35
5         back rear           35
6         driveway            10
8         pool shrub           5
9         back shrub           4
10        side shrub           0

2nd sprinkler timer
7am &7pm, 7 days a week
7         pool drip            7

each day is a bit: sun is lsb, sat is msb
sat and tue: B01000100

time starts at 12am = 0, and goes in 15 minute increments
3:30am: 14

number of minutes for each zone (1..12)
0 is off.  255 is max time
*/
  for (int i=0; i < NUM_PROGRAMS; ++i) {
    program[i].isEnabled = false;
    program[i].dayMask = B00000000;
    for (int j=0; j < NUM_START_TIMES; ++j)
      program[i].startTime[j] = TIME_OFF;
    for (int j=0; j < NUM_ZONES; ++j)
      program[i].duration[j] = 0;
  }
  
  program[0].isEnabled = true;
  program[0].dayMask = B01000100;
  program[0].startTime[0] = 14;
  program[0].duration[1] = 20;
  program[0].duration[2] = 30;
  program[0].duration[3] = 35;
  program[0].duration[4] = 35;
  program[0].duration[5] = 10;
  program[0].duration[7] = 5;
  program[0].duration[8] = 4;
  program[0].duration[9] = 0;

  program[1].isEnabled = true;
  program[1].dayMask = B11111111;
  program[1].startTime[0] = 7*4;       // 7am
  program[1].startTime[1] = (7+12)*4;  // 7pm
  program[1].duration[6] = 7;

  saveProgramConfig(); 
}

#define AP_NAME "Sprinkler"

void configModeCallback(WiFiManager *myWiFiManager) {
  // this callback gets called when the enter AP mode, and the users
  // need to connect to us in order to configure the wifi
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Join:"));
  lcd.print(AP_NAME);
  lcd.setCursor(0,1);
  lcd.print(F("Goto:"));
  lcd.print(WiFi.softAPIP());
}

bool setupWifi(void) {
  WiFi.hostname("sprinkler");
  
  WiFiManager wifiManager;
//  wifiManager.setDebugOutput(false);
  
  //reset settings - for testing
  //wifiManager.resetSettings();

  String ssid = WiFi.SSID();
  if (ssid.length() > 0) {
    Serial.print(F("Connecting to "));
    Serial.println(ssid);
    lcd.setCursor(0,0);
    lcd.print(F("Connecting to:"));
    lcd.setCursor(0,1);
    lcd.print(ssid);
  }
  
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  if(!wifiManager.autoConnect(AP_NAME)) {
    Serial.println(F("failed to connect and hit timeout"));
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 

  return true;
}


void setupTime(void) {
  Serial.println(F("Getting time"));

  AsyncUDP* udp = new AsyncUDP();

  // time.nist.gov NTP server
  // NTP requests are to port 123
  if (udp->connect(IPAddress(129,6,15,28), 123)) {
//    Serial.println("UDP connected");
    
    udp->onPacket([](void *arg, AsyncUDPPacket packet) {
//      Serial.println(F("received NTP packet"));
      byte *buf = packet.data();
      
      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:
    
      // convert four bytes starting at location 40 to a long integer
      unsigned long secsSince1900 =  (unsigned long)buf[40] << 24;
      secsSince1900 |= (unsigned long)buf[41] << 16;
      secsSince1900 |= (unsigned long)buf[42] << 8;
      secsSince1900 |= (unsigned long)buf[43];
      time_t utc = secsSince1900 - 2208988800UL;
    
    // cpd..hack until timezone is fixed
      utc -= 60 * 60 * 4;
      setTime(utc);
    
//      TimeChangeRule *tcr;
//      time_t local = myTZ.toLocal(utc, &tcr);
//      Serial.printf("\ntime zone %s\n", tcr->abbrev);
//    
//      setTime(local);
    
      // just print out the time
      printTime(false, false, true);
    
      isTimeSet = true;

      free(arg);
    }, udp);
    
//    Serial.println(F("sending NTP packet"));

    const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
    byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold outgoing packet

    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
    
    // all NTP fields have been given values, now
    // send a packet requesting a timestamp:
    udp->write(packetBuffer, NTP_PACKET_SIZE);
  }
  else {
    free(udp);
    Serial.println(F("\nWiFi:time failed...will retry in a minute"));
  }
}


void setupDisplay(void) {
  // initialize the lcd 
  lcd.init();
  lcd.clear();

  displayBacklight(true);
}

void setupButtons(void) {
  numButtons = sizeof(buttonPins)/sizeof(buttonPins[0]);
  buttonStates = new int[numButtons];
  lastButtonStates = new int[numButtons];
  lastDebounceTimes = new long[numButtons];

  for (int i=0; i < numButtons; ++i) {
    buttonStates[i] = defaultButtonState;
    lastButtonStates[i] = defaultButtonState;
    lastDebounceTimes[i] = 0;
  }
}

void drawMainScreen(void) {
  screen = MAIN;

  lcd.clear();
  
  printTime(true, true, false);
  printWatering(true);
  printModeState(true);
  printRainState(true);
}

void printRainState(bool isDisplay) {
  char msg[12];
  if (isRain) {
    int h = rainTime / 60;
    int m = rainTime % 60;
    sprintf(msg, "Rain %d:%02d", h, m);
  }
  else if (rainBlackoutTime > 0) {
    int h = rainBlackoutTime / 60;
    int m = rainBlackoutTime % 60;
    sprintf(msg, "Delay %d:%02d", h, m);
  }
  else
    sprintf(msg, "           ");

  if (isDisplay) {
    lcd.setCursor(0,1);
    lcd.print(msg);
  }
  
  if (webClient != -1) {
    sendWeb("rainState", msg);
  }
}

void printModeState(bool isDisplay) {
  const char *msg;
  if (isManual)
    msg = "Manual      ";
  else if (config.mode == OFF)
    msg = "Off         ";
  else if (runningProgram == -1)
    msg = "Running     ";
  else
    msg = "Watering    ";

  if (isDisplay) {
    lcd.setCursor(0,0);
    lcd.print(msg);
  }
  
  if (webClient != -1) {
    char buf[3];
    sprintf(buf, "%d", config.mode);    
    sendWeb("mode", buf);
    sendWeb("modeState", msg);
  }
}

void loop(void)
{
  if (isUploading) {
    server.handleClient();
    return;
  }
  
  if (!isSetup)
    return;
   
  unsigned long time = millis();

  if (screen == MAIN) {
    // turn off the display if idle
    if (time - lastButtonPressTime > (config.display_timeout*1000) ) {
      displayBacklight(false);
    }

    checkTimeMinutes();

    if (runningProgram != -1)
      checkTimeSeconds();

    // flash the time if its not set
    if (!isTimeSet)
      flashTime(time);
  }
  else {
    // go back to main screen if idle
    if (time - lastButtonPressTime > MenuIdleTime ) {
      drawMainScreen();
    }
  }

  checkButtons();

  webSocket.loop();
  server.handleClient();
}

#define TimeBetweenFlashes 500
unsigned long lastFlashTime = 0;
bool isFlash = false;

void flashTime(unsigned long time) {
  // the temperature sensor blocks the app from running while it is reading.
  // so, it may make the flash look off every time it is being checked
  if (time - lastFlashTime > TimeBetweenFlashes) {
    lastFlashTime = time;
    isFlash = !isFlash;
    if (isFlash)
      eraseTime();
    else
      printTime(false, true, false);
  }
}

void checkTimeSeconds(void) {
  int seconds = second();
  if (seconds == lastSeconds)
    return;

  lastSeconds = seconds;
  checkRemainingWatering();
  printWatering(true);
}

void checkTimeMinutes() {
  int minutes = minute();
  if (minutes == lastMinutes)
    return;

  // resync time at 3am every morning
  // this also catches daylight savings time changes which happen at 2am
  if (minutes == 0 && hour() == 3)
    isTimeSet = false;

  if (!isTimeSet)
    setupTime();
  
  lastMinutes = minutes;
  printTime(true, true, false);

  
  bool isRained = (getRainSensor() == RAIN_SENSOR_TRIGGERED);
//  Serial.printf("isRained %d\n", isRained);
  if (isRain != isRained) {
    isRain = isRained;
    rainChange();
    printRainState(true);
    logRainState();
  }
  else {
    if (isRain) {
      // still raining
      ++rainTime;
      printRainState(true);
    }
    else {
      // still not raining
      if (rainBlackoutTime > 0) {
        --rainBlackoutTime;
        if (rainBlackoutTime == 0)
          logAction(RAIN_BLACKOUT_OVER);
        printRainState(true);
      }
    }
  }
}

void rainChange(void) {
  if (isRain) {
    // just startd raining
    // reset the rain time counter
    rainTime = 0;
  }
  else {
    // just stopped raining
    // calculate the rain blackout timer
    rainBlackoutTime = rainTime * config.rainBlackoutMultiplier;
    printRainState(true);
  }
}

void checkRemainingWatering(void) {
  // called every second
  if (runningProgram == -1)
    return;

  if (--remainingZoneSeconds < 0) {
    remainingZoneSeconds = 59;
    --remainingZoneMinutes;
    if (remainingZoneMinutes < 0)
      startNextZone();
  }
}

int getRainSensor(void) {
  if (strcmp(config.chip_type,"P") == 0)
    Wire.requestFrom(rainSensorAddr, 1); 
  else
    Wire.requestFrom(rainSensorAddr+MUX_ADD, 1); 
  
  if (!Wire.available())
    return HIGH;
    
  byte value = Wire.read();
  int reading = ((value & rainSensorMask) == 0) ? LOW : HIGH;
  return reading;
}

void checkButtons(void) {
  if (testClient != -1 && testAddr != -1) {
    if (strcmp(config.chip_type,"P") == 0)
      Wire.requestFrom(testAddr, 1); 
    else
      Wire.requestFrom(testAddr+MUX_ADD, 1); 
    if (!Wire.available())
      return;
    
    unsigned int heap = ESP.getFreeHeap();
    byte value = Wire.read();
    if (value != testValue || heap != testHeap) {
      testValue = value;
      testHeap = heap;
      char json[128];
      sprintf(json, "{\"command\":\"read\",\"value\":\"%d\",\"heap\":\"%u\"}", value, heap);
      Serial.printf("sending %s\n", json);
      webSocket.sendTXT(testClient, json, strlen(json));
    }
  }

  if (strcmp(config.chip_type,"P") == 0)
    Wire.requestFrom(MUX1, 1); 
  else
    Wire.requestFrom(MUX1+MUX_ADD, 1); 
  if (!Wire.available())
    return;
    
  byte value = Wire.read();    
  for (int i=0; i < numButtons; ++i) {
    int reading = ((value & buttonPins[i]) == 0) ? LOW : HIGH;
    
    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonStates[i]) {
      // reset the debouncing timer
      lastDebounceTimes[i] = millis();
    }

    if ((millis() - lastDebounceTimes[i]) > debounceDelay) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != buttonStates[i]) {
        buttonStates[i] = reading;
        if (reading != defaultButtonState)
          doButton(buttonPins[i]);
      }
    }

    // save the reading.  Next time through the loop,
    // it'll be the lastButtonState:
    lastButtonStates[i] = reading;
  }
}

void doButton(int pin) {
  lastButtonPressTime = millis();
  
  if (!isDisplayOn) {
    // just turn on the display, ignore the button press
    displayBacklight(true);
    return;
  }
  
  switch (pin) {
    case UP:
      doUp();
      break;
    case DOWN:
      doDown();
      break;
    case ENTER:
      doEnter();
      break;
  }
}

void doUp(void) {
  switch(screen) {
    case MAIN:
      if (isManual)
        startNextZone();
      break;
    case MENU:
      setSelectedMenuItem(selectedMenuItem-1);
      break;
  }
}

void doDown(void) {
  switch(screen) {
    case MAIN:
      if (isManual)
        startPreviousZone();
      break;
    case MENU:
      setSelectedMenuItem(selectedMenuItem+1);
      break;
  }
}

void doEnter(void) {
  switch(screen) {
    case MAIN:
      drawMenuScreen();
      setSelectedMenuItem(MANUAL_BUTTON);
      break;
    case MENU:
      doMenuEnter();
      break;
  }
}

void modeChange(bool isLocal) {
  saveConfig();
  if (config.mode == OFF) {
    // stop program if its running
    if (runningProgram != -1 and !isManual) {
      stopCurrentZone();
      stopProgram();
    }
  }
  drawMainScreen();
  printModeState(true);

  if (config.mode == OFF)
    logAction(PROGRAMS_OFF);
  else
    logAction(PROGRAMS_RUN);
}

void sendWeb(const char *command, const char *value) {
  char json[128];
  sprintf(json, "{\"command\":\"%s\",\"value\":\"%s\"}", command, value);
  webSocket.sendTXT(webClient, json, strlen(json));
}

void startManual(void) {
  isManual = true;
  startProgram(0);
  drawMainScreen();
}

void doMenuEnter(void) {
  switch(selectedMenuItem) {
    case MANUAL_BUTTON:
      startManual();
      break;
    case MODE_BUTTON:
      if ( ++(config.mode) > RUN )
        config.mode = OFF;
      modeChange(true);
      break;
  }
}

int row[] = { 0, 0, 1, 1 };
int col[] = { 0, 15, 0, 15 };
byte chr[] = { 126, 127, 126, 127 };  // right and left arrows

void setSelectedMenuItem(int item) {
  if (selectedMenuItem != NONE) {
    lcd.setCursor(col[selectedMenuItem],row[selectedMenuItem]);
    lcd.print(F(" "));
  }
  
  if (item < MANUAL_BUTTON) item = MODE_BUTTON;
  else if (item > MODE_BUTTON ) item = MANUAL_BUTTON;
  
  selectedMenuItem = item;
    
  lcd.setCursor(col[selectedMenuItem],row[selectedMenuItem]);
  lcd.write(chr[selectedMenuItem]);
}

void drawMenuScreen(void) {
  screen = MENU;
  selectedMenuItem = NONE;

  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print(F("Manual"));
  lcd.setCursor(11,0);
  lcd.print(F("Mode"));

  // display our ip address
  lcd.setCursor(1,1);
  lcd.print(WiFi.localIP());
}

void send(byte addr, byte b) {
  if (strcmp(config.chip_type,"P") == 0)
    Wire.beginTransmission(addr);
  else
    Wire.beginTransmission(addr+MUX_ADD);
  Wire.write(b);
  Wire.endTransmission(); 
}

void displayBacklight(bool isOn) {
  isDisplayOn = isOn;
  if (isOn)
    lcd.backlight();
  else
    lcd.noBacklight();
}

void clearWatering(void) {
  lcd.setCursor(0,1);
  lcd.print(F("          "));

  if (webClient != -1) {
    sendWeb("zoneState", "");
    sendWeb("zoneName", "");
    sendWeb("water_adjust", "");
  }
}

void printWatering(bool isDisplay) {
  if (runningProgram == -1)
    return;
    
  char buf[11];
  sprintf(buf, "%d-%2d,%2d:%02d", runningProgram+1, runningZone+1,
    remainingZoneMinutes, remainingZoneSeconds);
  
  if (isDisplay) {
    lcd.setCursor(0,1);
    lcd.print(buf);

    // every few seconds, toggle the zone name on the display
    if (remainingZoneSeconds % 5 == 0) {
      isName = !isName;
      const char *msg;
      if (isName) {
        msg = zone[runningZone].name;
      }
      else if (isManual)
        msg = "Manual      ";
      else
        msg = "Watering    ";

      lcd.setCursor(0,0);
      lcd.print(msg);
    }
  }
  
  if (webClient != -1) {
    sendWeb("zoneState", buf);
    sendWeb("zoneName", zone[runningZone].name);
    if (actual_water_adjust != 100) {
      sprintf(buf, "%d%%", actual_water_adjust);
      sendWeb("water_adjust", buf);
    }
  }
}

void eraseTime(void) {
  lcd.setCursor(13,0);
  lcd.print(F("   "));
  lcd.setCursor(10,1);
  lcd.print(F("      "));
}

void printTime(bool isCheckProgram, bool isDisplay, bool isTest) {
  int dayOfWeek = weekday()-1;
  int hours = hour();
  int minutes = minute();

  const char *ampm = "a";
  int h = hours;
  if (h == 0)
    h = 12;
  else if (h == 12)
    ampm = "p";
  else if (h > 12) {
    h -= 12;
    ampm = "p";
  }
  char buf[7];
  sprintf(buf, "%2d:%02d%s", h, minutes, ampm); 
//  Serial.println(buf);

  if (isDisplay) {
    lcd.setCursor(13,0);
    lcd.print(weekdayNames[dayOfWeek]);
    lcd.setCursor(10,1);
    lcd.print(buf);
  }
   
  if (webClient != -1 || isTest) {
    char msg[6+1+4];
    sprintf(msg, "%s %s", buf, weekdayNames[dayOfWeek]); 
    if (webClient != -1)
      sendWeb("time", msg);
    if (isTest)
      Serial.printf("time is %s\n", msg);
  }

  if (isCheckProgram) {
    if (runningProgram == -1) {
      // see if the program needs to be started
      checkProgram(dayOfWeek, hours, minutes);
    }
  }
}

void pump(boolean isOn) {
  isPumpOn = isOn;
  byte data = 0xFF;
  if (isOn)
    data ^=  pumpMask;
  send(pumpAddr, data);  
  Serial.printf("pump, sending addr %d, value ", pumpAddr);
  Serial.println(data, BIN);
}

boolean programNeedsZimmerman(int index) {
  // if the running program contains a zone that uses the rain sensor,
  // then return true
  if (program[index].isEnabled) {
    for (int j=0; j < NUM_ZONES; ++j) {
      if (program[index].duration[j] > 0) {
        if (zone[j].usesRainSensor)
          return true;
      }
    }
  }
  return false;
}

int programToStart;

void startProgram(int index) {
  programToStart = index;
  
  // adjust watering time if needed
  if (config.use_zimmerman == 1 && programNeedsZimmerman(index)) {
    Serial.println(F("use zimmerman"));
    // should only need to make the wunderground call once when the program starts
    requestZimmermanAdjust();
//    Serial.println(F("TEST ZERO WATER ADJUST"));
//    actual_water_adjust = 0;
//    startProgramAfterAdjust();
  }
  else {
    Serial.println(F("use % adjust"));
    actual_water_adjust = config.water_adjust;
    startProgramAfterAdjust();
  }
}


void startProgramAfterAdjust(void) {
  runningProgram = programToStart;
  Serial.printf("starting program %d\n", (runningProgram+1));
  Serial.printf("adjust water %d\n", actual_water_adjust);
  runningZone = -1;
  startNextZone();
}


void stopProgram(void) {
  runningProgram = -1;
  runningZone = -1;
  pump(false);
  clearWatering();
  isManual = false;
  printModeState(true);
}

void startCurrentZone(void) {
  if (runningProgram != -1) {
    remainingZoneMinutes = program[runningProgram].duration[runningZone];
    remainingZoneSeconds = 0;

    if (actual_water_adjust != 100) {
      Serial.printf("from %d:%02d", remainingZoneMinutes, remainingZoneSeconds);
      int i = remainingZoneMinutes * 60 * actual_water_adjust / 100;
      remainingZoneMinutes = i / 60;
      remainingZoneSeconds = i % 60;
      Serial.printf(" to %d:%02d\n", remainingZoneMinutes, remainingZoneSeconds);
    }
    
    // skip if less than a minutes of runtime
    if (remainingZoneMinutes == 0) {
      Serial.printf("skipped due to time\n");
      logZone(runningZone, 0, ZONE_WATERING, actual_water_adjust);
      remainingZoneSeconds = 0;
      return;
    }

    // since we're only logging minutes
    int m = remainingZoneMinutes;
    if (remainingZoneSeconds >= 30)
      ++m;
    logZone(runningZone, m, ZONE_WATERING, actual_water_adjust);
  }
  else {
    // no program, so the zone must have been turned on manually
    logZone(runningZone, 0, ZONE_MANUAL, 100);
  }

  // turn on the pump if this zone needs it, or off if it doesn't
  // must call pump before turning our zone on, since the pump call turns off any other
  // zones on its mux
  pump(zone[runningZone].usesPump);
  
  // turn on the new zone
  byte data = 0xFF ^ zoneMasks[runningZone];
  // don't turn off the pump, if it needs to stay on
  if (isPumpOn && pumpAddr == zoneAddrs[runningZone])
    data ^= pumpMask;
  send(zoneAddrs[runningZone], data);
  Serial.printf("start zone %d, sending addr %d, value ", runningZone, zoneAddrs[runningZone]);
  Serial.println(data, BIN);

  isName = false;
  
  printWatering(true);
  printModeState(true);
  
  // turn on the display
  if (!isDisplayOn) {
    lastButtonPressTime = millis();
    displayBacklight(true);
  }
}

void stopCurrentZone(void) {
  // turn off the current zone
  if ( runningZone != -1 ) {
    byte data = 0xFF;
    // don't turn off the pump, if it needs to stay on
    if (isPumpOn && pumpAddr == zoneAddrs[runningZone])
      data ^= pumpMask;
    send(zoneAddrs[runningZone], data);
    remainingZoneMinutes = 0;
    remainingZoneSeconds = 0;
    isName = false;
  }
}

void startNextZone(void) {
  stopCurrentZone();

  bool isRained = (getRainSensor() == RAIN_SENSOR_TRIGGERED);
  if (isRain != isRained) {
    isRain = isRained;
    rainChange();
    printRainState(true);
    logRainState();
  }

  // locate the next zone with a non zero duration
  // and the rain sensor is ok with
  while (true) {
    if (++runningZone == NUM_ZONES) {
      // no more zones to run in this program
      if (isManual) {
        // use the next program
        runningZone = 0;
        if (++runningProgram == NUM_PROGRAMS) {
          // no more programs to run...stop the program
          stopProgram();
          return;
        }
        else
          Serial.printf("starting program %d\n", (runningProgram+1));
      }
      else {
        stopProgram();
        return;
      }
    }
    
    if (program[runningProgram].isEnabled && program[runningProgram].duration[runningZone] > 0) {
      if (zone[runningZone].usesRainSensor && (isRain || (rainBlackoutTime > 0))) {
        Serial.printf("skipping zone %d because of rain sensor\n", runningZone);
        logZone(runningZone, 0, ZONE_SKIPPED, 100);
        continue;
      }
      break;
    }
  }  

  startCurrentZone();
}

void startPreviousZone(void) {
  stopCurrentZone();

  bool isRained = (getRainSensor() == RAIN_SENSOR_TRIGGERED);
  if (isRain != isRained) {
    isRain = isRained;
    rainChange();
    printRainState(true);
    logRainState();
  }

  // locate the previous zone with a non zero duration
  // and the rain sensor is ok with
  while (true) {
    if (--runningZone == -1) {
      // no more zones to run in this program
      if (isManual) {
        // use the previous program
        runningZone = NUM_ZONES-1;
        if (--runningProgram == -1) {
          // no more programs to run...stop the program
          stopProgram();
          return;
        }
        else
          Serial.printf("starting program %d\n", (runningProgram+1));
      }
      else {
        stopProgram();
        return;
      }
    }
    
    if (program[runningProgram].isEnabled && program[runningProgram].duration[runningZone] > 0) {
      if (zone[runningZone].usesRainSensor && (isRain || (rainBlackoutTime > 0))) {
        Serial.printf("skipping zone %d because of rain sensor\n", runningZone);
        logZone(runningZone, 0, ZONE_SKIPPED, 100);
        continue;
      }
      break;
    }
  }  

  startCurrentZone();
}

void checkProgram(int day, int h, int m) {
  if (config.mode == OFF)
    return;
    
  // check each program
  int ctime = h*60+m;
  for (int i=0; i < NUM_PROGRAMS; ++i) {
    if ( !program[i].isEnabled )
      continue;
    
    // check day
    if (((1 << day) & program[i].dayMask) == 0)
      continue;
    
    // check each start time
    for (int j=0; j < NUM_START_TIMES; ++j) {
      if (program[i].startTime[j] == TIME_OFF)
        continue;
        
      int ptime = program[i].startTime[j]*15;
      if (ptime == ctime) {
        startProgram(i); 
        return;
      }
    }
  }
  
  // no programs were matches
}

#define MAGIC_NUM   0xAD

#define MAGIC_NUM_ADDRESS      0
#define CONFIG_ADDRESS         1
#define PROGRAM_ADDRESS        CONFIG_ADDRESS + sizeof(config)
#define ZONE_ADDRESS           PROGRAM_ADDRESS + sizeof(program)

void set(char *name, const char *value) {
  for (int i=strlen(value); i >= 0; --i)
    *(name++) = *(value++);
}

void loadConfig(void) {
  int magicNum = EEPROM.read(MAGIC_NUM_ADDRESS);
  if (magicNum != MAGIC_NUM) {
    Serial.println(F("invalid eeprom data"));
    isMemoryReset = true;
  }
  
  if (isMemoryReset) {
    // nothing saved in eeprom, use defaults
    Serial.println(F("using default config"));
    config.mode = OFF;
    config.display_timeout = 20;
    config.water_adjust = 100;
    config.use_zimmerman = 1;
    set(config.wu_key, WU_API_KEY);
    set(config.wu_location, WU_LOCATION);
    config.use_logging = 1;
    set(config.logging_ip_addr, LOGGING_IP_ADDR);
    config.logging_ip_port = LOGGING_IP_PORT;
    set(config.log_url, LOG_URL);
    config.rainBlackoutMultiplier = 3;
    config.use_torrent = 1;
    set(config.torrent_ip_addr, TORRENT_IP_ADDR);
    config.torrent_ip_port = TORRENT_IP_PORT;
    config.torrent_delay = 5;
    set(config.chip_type, "AP");

    saveConfig();
  }
  else {
    int addr = CONFIG_ADDRESS;
    byte *ptr = (byte *)&config;
    for (int i=0; i < sizeof(config); ++i, ++ptr)
      *ptr = EEPROM.read(addr++);
  }

  Serial.printf("mode %d\n", config.mode);
  Serial.printf("display_timeout %d\n", config.display_timeout);
  Serial.printf("water_adjust %d\n", config.water_adjust);
  Serial.printf("use_zimmerman %d\n", config.use_zimmerman);
  Serial.printf("wu_key %s\n", config.wu_key);
  Serial.printf("wu_location %s\n", config.wu_location);
  Serial.printf("use_logging %d\n", config.use_logging);
  Serial.printf("logging_ip_addr %s\n", config.logging_ip_addr);
  Serial.printf("logging_ip_port %d\n", config.logging_ip_port);
  Serial.printf("log_url %s\n", config.log_url);
  Serial.printf("rainBlackoutMultiplier %d\n", config.rainBlackoutMultiplier);
  Serial.printf("use_torrent %d\n", config.use_torrent);
  Serial.printf("torrent_ip_addr %s\n", config.torrent_ip_addr);
  Serial.printf("torrent_ip_port %d\n", config.torrent_ip_port);
  Serial.printf("torrent_delay %d\n", config.torrent_delay);
  Serial.printf("chip_type %s\n", config.chip_type);
}

void loadProgramConfig(void) {
  if (isMemoryReset) {
    // nothing saved in eeprom, use defaults
    Serial.printf("using default programs\n");
    initProgram();  
  }
  else {
    Serial.printf("loading programs from eeprom\n");
    int addr = PROGRAM_ADDRESS;
    byte *ptr = (byte *)&program;
    for (int i = 0; i < sizeof(program); ++i, ++ptr, ++addr)
      *ptr = EEPROM.read(addr);
  }
}

void loadZoneConfig(void) {
  if (isMemoryReset) {
    // nothing saved in eeprom, use defaults
    Serial.printf("using default zones\n");
    initZone();
  }
  else {
    Serial.printf("loading zones from eeprom\n");
    int addr = ZONE_ADDRESS;
    byte *ptr = (byte *)&zone;
    for (int i = 0; i < sizeof(zone); ++i, ++ptr, ++addr)
      *ptr = EEPROM.read(addr);
  }
//  for (int i=0; i < NUM_ZONES; ++i)
//    Serial.printf("zone %d: %s %d %d\n", i, zone[i].name,
//      zone[i].usesRainSensor, zone[i].usesPump);
}

void saveConfig(void) {
  isPromModified = false;
  update(MAGIC_NUM_ADDRESS, MAGIC_NUM);

  byte *ptr = (byte *)&config;
  int addr = CONFIG_ADDRESS;
  for (int j=0; j < sizeof(config); ++j, ++ptr)
    update(addr++, *ptr);
  
  if (isPromModified)
    EEPROM.commit();
}

void update(int addr, byte data) {
  if (EEPROM.read(addr) != data) {
    EEPROM.write(addr, data);
    isPromModified = true;
  }
}

void saveProgramConfig(void) {
  isPromModified = false;
  Serial.printf("saving programs to eeprom\n");
  int addr = PROGRAM_ADDRESS;
  byte *ptr = (byte *)&program;
  for (int i = 0; i < sizeof(program); ++i, ++ptr, ++addr)
    update(addr, *ptr);

  if (isPromModified)
    EEPROM.commit();
}

void saveZoneConfig(void) {
  isPromModified = false;
  Serial.printf("saving zones to eeprom\n");
  int addr = ZONE_ADDRESS;
  byte *ptr = (byte *)&zone;
  for (int i = 0; i < sizeof(zone); ++i, ++ptr, ++addr)
    update(addr, *ptr);

  if (isPromModified)
    EEPROM.commit();

  logNames();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      if (num == webClient)
        webClient = -1;
      else if (num == programClient)
        programClient = -1;
      else if (num == setupClient)
        setupClient = -1;
      else if (num == zoneClient)
        zoneClient = -1;
      else if (num == testClient) {
        testClient = -1;
        testAddr = -1;
        testValue = -1;
        testHeap = 0;
      }
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      
      if (strcmp((char *)payload,"/") == 0) {
        webClient = num;
      
        // send the current state
        printModeState(false);
        printRainState(false);
        printWatering(false);
        printTime(false, false, false);
        sendWeb("log_url", config.log_url);
      }
      else if (strcmp((char *)payload,"/program") == 0) {
        programClient = num;
        
        // send programs
        char json[265*2];
        strcpy(json, "{\"command\":\"program\",\"value\":[");
        for (int i=0; i < NUM_PROGRAMS; ++i) {
          sprintf(json+strlen(json), "%s[%d,%d,[", (i==0)?"":",", program[i].isEnabled, program[i].dayMask);
          for (int j=0; j < NUM_START_TIMES; ++j)
            sprintf(json+strlen(json), "%d%s", program[i].startTime[j], (j==NUM_START_TIMES-1)?"],[":",");
          for (int j=0; j < NUM_ZONES; ++j)
            sprintf(json+strlen(json), "%d%s", program[i].duration[j], (j==NUM_ZONES-1)?"]]":",");
        }
        strcpy(json+strlen(json), "],\"name\":[");
        for (int i=0; i < NUM_ZONES; ++i)
          sprintf(json+strlen(json), "\"%s\"%s", zone[i].name, (i==NUM_ZONES-1)?"":",");
        strcpy(json+strlen(json), "]}");
        //Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(programClient, json, strlen(json));
      }
      else if (strcmp((char *)payload,"/setup") == 0) {
        setupClient = num;

        char json[256];
        strcpy(json, "{");
        sprintf(json+strlen(json), "\"date\":\"%s\"", __DATE__);
        sprintf(json+strlen(json), ",\"time\":\"%s\"", __TIME__);
        sprintf(json+strlen(json), ",\"timeout\":\"%d\"", config.display_timeout);
        sprintf(json+strlen(json), ",\"water_adjust\":\"%d\"", config.water_adjust);
        sprintf(json+strlen(json), ",\"use_zimmerman\":\"%d\"", config.use_zimmerman);
        sprintf(json+strlen(json), ",\"key\":\"%s\"", config.wu_key);
        sprintf(json+strlen(json), ",\"location\":\"%s\"", config.wu_location);
        sprintf(json+strlen(json), ",\"use_logging\":\"%d\"", config.use_logging);
        sprintf(json+strlen(json), ",\"logging_ip_addr\":\"%s\"", config.logging_ip_addr);
        sprintf(json+strlen(json), ",\"logging_ip_port\":\"%d\"", config.logging_ip_port);
        sprintf(json+strlen(json), ",\"log_url\":\"%s\"", config.log_url);
        sprintf(json+strlen(json), ",\"rainBlackoutMultiplier\":\"%d\"", config.rainBlackoutMultiplier);        
        sprintf(json+strlen(json), ",\"use_torrent\":\"%d\"", config.use_torrent);
        sprintf(json+strlen(json), ",\"torrent_ip_addr\":\"%s\"", config.torrent_ip_addr);
        sprintf(json+strlen(json), ",\"torrent_ip_port\":\"%d\"", config.torrent_ip_port);
        sprintf(json+strlen(json), ",\"torrent_delay\":\"%d\"", config.torrent_delay);
        sprintf(json+strlen(json), ",\"chip_type\":\"%s\"", config.chip_type);
        strcpy(json+strlen(json), "}");
//        Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(setupClient, json, strlen(json));
      }
      else if (strcmp((char *)payload,"/test") == 0) {
        testClient = num;
        testAddr = MUX1;

        // send the addessses of the muxes we are using
        char json[128];
        sprintf(json, "{\"msg\":\"MUXes at %d and %d\",\"addr\":\"%d\"}", MUX1, MUX2, MUX1);
//        Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(testClient, json, strlen(json));
      }
      else if (strcmp((char *)payload,"/zone") == 0) {
        zoneClient = num;
        
        // send zones
        char json[265];
        strcpy(json, "{\"command\":\"zone\",\"value\":[");
        for (int i=0; i < NUM_ZONES; ++i)
          sprintf(json+strlen(json), "%s[\"%s\",%d,%d]", (i==0)?"":",",
            zone[i].name, zone[i].usesRainSensor, zone[i].usesPump);
        strcpy(json+strlen(json), "]}");
        //Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(zoneClient, json, strlen(json));
      }
      else {
        Serial.printf("unknown call %s\n", payload);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      
      if (num == webClient) {
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"manual",6) == 0) {
          startManual();
        }
        else if (strncmp(ptr,"next",4) == 0) {
          if (isManual)
            startNextZone();
        }
        else if (strncmp(ptr,"prev",4) == 0) {
          if (isManual)
            startPreviousZone();
        }        
        else if (strncmp(ptr,"mode",4) == 0) {
          target = "value";
          ptr = strstr(ptr, target) + strlen(target)+3;
          config.mode = strtol(ptr, &ptr, 10);
          modeChange(false);
        }        
      }
      else if (num == programClient) {
        Serial.printf("save programs\n");
        char *ptr = strchr((char *)payload, '[')+2;
        for (int i=0; i < NUM_PROGRAMS; ++i) {
          program[i].isEnabled = strtol(ptr, &ptr, 10);
          ptr += 1;

          program[i].dayMask = strtol(ptr, &ptr, 10);
          ptr += 2;

          for (int j=0; j < NUM_START_TIMES; ++j, ++ptr) {
            program[i].startTime[j] = strtol(ptr, &ptr, 10);
//            Serial.printf("startTime %d %d %d\n", i, j, program[i].startTime[j]);
          }
          ptr += 2;
          for (int j=0; j < NUM_ZONES; ++j, ++ptr) {
            program[i].duration[j] = strtol(ptr, &ptr, 10);
//            Serial.printf("duration %d %d %d\n", i, j, program[i].duration[j]);
          }
          ptr += 3;
        }      
        saveProgramConfig();
        drawMainScreen();
      }
      else if (num == setupClient) {
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"reboot",6) == 0) {
          ESP.restart();
        }
        else if (strncmp(ptr,"save",4) == 0) {
          Serial.printf("save setup\n");
          
          const char *target = "timeout";
          char *ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.display_timeout = strtol(ptr, &ptr, 10);
  
          target = "water_adjust";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.water_adjust = strtol(ptr, &ptr, 10);
  
          target = "use_zimmerman";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.use_zimmerman = strtol(ptr, &ptr, 10);
  
          target = "key";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          char *end = strchr(ptr, '\"');
          memcpy(config.wu_key, ptr, (end-ptr));
          config.wu_key[end-ptr] = '\0';
  
          target = "location";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.wu_location, ptr, (end-ptr));
          config.wu_location[end-ptr] = '\0';

          target = "use_logging";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.use_logging = strtol(ptr, &ptr, 10);
          
          target = "logging_ip_addr";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.logging_ip_addr, ptr, (end-ptr));
          config.logging_ip_addr[end-ptr] = '\0';

          target = "logging_ip_port";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.logging_ip_port = strtol(ptr, &ptr, 10);

          target = "log_url";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.log_url, ptr, (end-ptr));
          config.log_url[end-ptr] = '\0';

          target = "rainBlackoutMultiplier";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.rainBlackoutMultiplier = strtol(ptr, &ptr, 10);

          target = "use_torrent";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.use_torrent = strtol(ptr, &ptr, 10);
          
          target = "torrent_ip_addr";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.torrent_ip_addr, ptr, (end-ptr));
          config.torrent_ip_addr[end-ptr] = '\0';

          target = "torrent_ip_port";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.torrent_ip_port = strtol(ptr, &ptr, 10);

          target = "torrent_delay";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.torrent_delay = strtol(ptr, &ptr, 10);

          target = "chip_type";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.chip_type, ptr, (end-ptr));
          config.chip_type[end-ptr] = '\0';

    Serial.printf("display_timeout %d\n", config.display_timeout);
    Serial.printf("water_adjust %d\n", config.water_adjust);
    Serial.printf("use_zimmerman %d\n", config.use_zimmerman);
    Serial.printf("wu_key %s\n", config.wu_key);
    Serial.printf("wu_location %s\n", config.wu_location);
    Serial.printf("use_logging %d\n", config.use_logging);
    Serial.printf("logging_ip_addr %s\n", config.logging_ip_addr);
    Serial.printf("logging_ip_port %d\n", config.logging_ip_port);
    Serial.printf("log_url %s\n", config.log_url);
    Serial.printf("rainBlackoutMultiplier %d\n", config.rainBlackoutMultiplier);
    Serial.printf("use_torrent %d\n", config.use_torrent);
    Serial.printf("torrent_ip_addr %s\n", config.torrent_ip_addr);
    Serial.printf("torrent_ip_port %d\n", config.torrent_ip_port);
    Serial.printf("torrent_delay %d\n", config.torrent_delay);
    Serial.printf("chip_type %s\n", config.chip_type);
          saveConfig();
        }
      }
      else if (num == testClient) {
        Serial.printf("test %s\n", payload);
        const char *target = "addr";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        int addr = strtol(ptr, &ptr, 10);
        target = "value";
        ptr = strstr(ptr, target) + strlen(target)+3;
        int value = strtol(ptr, &ptr, 10);
        Serial.printf("%d %d\n", addr, value);
        send(addr, value);
        testAddr = addr;
      }
      else if (num == zoneClient) {
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"saveZone",8) == 0) {
          Serial.printf("save zones\n");
          char *ptr = strchr((char *)payload, '[')+3;
          for (int i=0; i < NUM_ZONES; ++i) {
            char *end = strchr(ptr, '\"');
            memcpy(zone[i].name, ptr, (end-ptr));
            zone[i].name[end-ptr] = '\0';
            zone[i].usesRainSensor = (strtol(end+2, &ptr, 10) == 1);
            zone[i].usesPump = (strtol(ptr+1, &ptr, 10) == 1);
            ptr += 4;
            Serial.printf("'%s' '%d' '%d'\n", zone[i].name, zone[i].usesRainSensor, zone[i].usesPump);
          }      
          saveZoneConfig();
        }
        else if (strncmp(ptr,"OnZone",6) == 0) {
          target = "value";
          ptr = strstr(ptr, target) + strlen(target)+3;
          int id = strtol(ptr, &ptr, 10);
          Serial.printf("on zone %d\n", id);
          runningZone = id;
          startCurrentZone();
        }
        else if (strncmp(ptr,"OffZone",7) == 0) {
          target = "value";
          ptr = strstr(ptr, target) + strlen(target)+3;
          int id = strtol(ptr, &ptr, 10);
          Serial.printf("off zone %d\n", id);
          // this actually stops everything, regardless of the zone
          stopCurrentZone();
          stopProgram();
        }
      }
      break;
  }
}



const char http_path_zone[]   = "/sprinkler/remote/insertZone.php";
const char http_path_action[] = "/sprinkler/remote/insertAction.php";
const char http_path_name[]   = "/sprinkler/remote/insertName.php";

typedef struct {
  int zone;
  int duration;
  char action[2];
  int percentage;
} logZoneType;


void logZone(int zone, int duration, char action, int percentage) {
  if (!config.use_logging)
    return;
    
  AsyncClient* aclient = new AsyncClient();

  logZoneType* data = new logZoneType;
  data->zone = zone;
  data->duration = duration;
  data->action[0] = action;
  data->action[1] = '\0';
  data->percentage = percentage;
  
  aclient->onConnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onConnect\n");
    
    logZoneType* data = (logZoneType*) obj;
    
    // after connecting, send the request
    // Make an HTTP GET request
    c->write( "GET ");
    c->write( http_path_zone );
    c->write("?zone=");
    c->write( String(data->zone+1).c_str() );
    c->write("&");
    c->write("duration=");
    c->write( String(data->duration).c_str() );
    c->write("&");
    c->write("action=");
    c->write( data->action );
    c->write("&");
    c->write("percentage=");
    c->write( String(data->percentage).c_str() );
    c->write( " HTTP/1.1\r\n");
    c->write("Host: ");
    c->write(config.logging_ip_addr);
    c->write( "\r\nContent-Type: application/x-www-form-urlencoded\r\n" );
    c->write("Connection: close\r\n\r\n");
    c->stop();

//    Serial.print("logged zone: ");
//    Serial.println(data->zone);
    free(obj);
  }, data);

  aclient->onDisconnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onDisconnect\n");
    free(c);
  }, NULL);

  aclient->onError([](void *obj, AsyncClient* c, int8_t err) {
    Serial.printf("logZone [A-TCP] onError: %s\n", c->errorToString(err));
  }, NULL);

  aclient->onData([](void *obj, AsyncClient* c, void *buf, size_t len) {
//    Serial.printf("[A-TCP] onData: %s\n", buf);
  }, NULL);

//  Serial.printf("connecting to %s:%d\n", config.logging_ip_addr, config.logging_ip_port);
//  unsigned long t = millis();
  if (!aclient->connect(config.logging_ip_addr, config.logging_ip_port)) {
//    Serial.printf("connect failed %d\n", (millis() - t));
    free(aclient);
    free(data);
  }
}

void logRainState(void) {
  if (isRain)
    logAction(RAIN_SENSOR_ACTIVE);
  else
    logAction(RAIN_SENSOR_NORMAL);
}


void logAction(char action) {
  if (!config.use_logging)
    return;

  char* str = new char[2];
  str[0] = action;
  str[1] = '\0';

  AsyncClient* aclient = new AsyncClient();

  aclient->onConnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onConnect\n");
    // after connecting, send the request
    // Make an HTTP GET request
    c->write( "GET ");
    c->write( http_path_action );
    c->write("?action=");
    char* str = (char*) obj;
    c->write( str );
    c->write( " HTTP/1.1\r\n");
    c->write("Host: ");
    c->write(config.logging_ip_addr);
    c->write( "\r\nContent-Type: application/x-www-form-urlencoded\r\n" );
    c->write("Connection: close\r\n\r\n");
    c->stop();

//    Serial.print("logged action: ");
//    Serial.println(str);
    free(str);
  }, str);

  aclient->onDisconnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onDisconnect\n");
    free(c);
  }, NULL);

  aclient->onError([](void *obj, AsyncClient* c, int8_t err) {
    Serial.printf("logAction [A-TCP] onError: %s\n", c->errorToString(err));
  }, NULL);

  aclient->onData([](void *obj, AsyncClient* c, void *buf, size_t len) {
//    Serial.printf("[A-TCP] onData: %s\n", buf);
  }, NULL);

//  Serial.printf("connecting to %s:%d\n", config.logging_ip_addr, config.logging_ip_port);
  if (!aclient->connect(config.logging_ip_addr, config.logging_ip_port)) {
    free(aclient);
    free(str);
  }
}


void logNames(void) {
  if (!config.use_logging)
    return;
    
  AsyncClient* aclient = new AsyncClient();

  aclient->onConnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onConnect\n");
    // after connecting, send the request
    // Make an HTTP GET request
    c->write( "GET ");
    c->write( http_path_name );

    for (int i=0; i < NUM_ZONES; ++i) {
      if (i == 0)
        c->write("?");
      else
        c->write("&");
      c->write(String(i+1).c_str());
      c->write("=");
      // the name may have spaces in it which need to be url encoded
      String str = zone[i].name;
      str.replace(" ", "%20");
      c->write( str.c_str() );
    }

    c->write( " HTTP/1.1\r\n");
    c->write("Host: ");
    c->write(config.logging_ip_addr);
    c->write( "\r\nContent-Type: application/x-www-form-urlencoded\r\n" );
    c->write("Connection: close\r\n\r\n");
    c->stop();

//    Serial.println("logged names");
  }, NULL);

  aclient->onDisconnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onDisconnect\n");
    free(c);
  }, NULL);

  aclient->onError([](void *obj, AsyncClient* c, int8_t err) {
    Serial.printf("logNames [A-TCP] onError: %s\n", c->errorToString(err));
  }, NULL);

  aclient->onData([](void *obj, AsyncClient* c, void *buf, size_t len) {
//    Serial.printf("[A-TCP] onData: %s\n", buf);
  }, NULL);

//  Serial.printf("connecting to %s:%d\n", config.logging_ip_addr, config.logging_ip_port);
  if (!aclient->connect(config.logging_ip_addr, config.logging_ip_port)) {
    free(aclient);
  }
}


#define DEFAULT_ADJUST 100.0

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

#define NUM_ZIMMERMAN_FIELDS 6
enum zimmerFields { precip_today_in, maxtempi, mintempi, maxhumidity, minhumidity, precipi };
const char *zimmerFieldNames[] = { "\"precip_today_in\":", "\"maxtempi\":", "\"mintempi\":", "\"maxhumidity\":", "\"minhumidity\":", "\"precipi\":" };
float zimmermanField[ NUM_ZIMMERMAN_FIELDS ];
int zimmermanNumGot;

// precipi is not unique in the response

void requestZimmermanAdjust(void) {
  actual_water_adjust = DEFAULT_ADJUST;

  // cpd...fix
//  String id = "";
//  if (config.use_torrent == 1) {
//    id = getId();
//    if (id.length() > 0) {
//      stopTorrents(id);
//
//      // wait a bit before for things to settle down
//      if (config.torrent_delay > 0)
//        delay(1000 * config.torrent_delay);
//    }
//  }
  
  requestZimmermanAdjustNow();

  // cpd...fix
//  if (config.use_torrent == 1) {
//    if (id.length() > 0)
//      startTorrents(id);
//  }
}


void requestZimmermanAdjustNow(void) {
  for (int i=0; i < NUM_ZIMMERMAN_FIELDS; ++i) {
    zimmermanField[i] = -1;
  }
  zimmermanNumGot = 0;
  
  AsyncClient* aclient = new AsyncClient();
      
  aclient->onConnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onConnect\n");
    // after connecting, send the request, and wait for a reply
    // Make an HTTP GET request
    c->write(WUNDERGROUND_REQ1);
    c->write(config.wu_key);
    c->write(WUNDERGROUND_REQ2);
    c->write(config.wu_location);
    c->write(WUNDERGROUND_REQ3);
  }, NULL);

  aclient->onDisconnect([](void *obj, AsyncClient* c) {
//    Serial.printf("[A-TCP] onDisconnect\n");
    free(c);
  }, NULL);

  aclient->onError([](void *obj, AsyncClient* c, int8_t err) {
//    Serial.printf("requestOutsideTemperature [A-TCP] onError: %s\n", c->errorToString(err));
  }, NULL);

  aclient->onData([](void *obj, AsyncClient* c, void *buf, size_t len) {
//    Serial.printf("[A-TCP] onData: %s\n", buf);

  // the response back is huge (30Kb), and we only need the data at the end
  // so skip lines until we get to the dailysummary
  // we need these parmeters from the conditions, so pull them out without
  // parsing the huge response

    // search for the fields in the response
    for (int i=0; i < NUM_ZIMMERMAN_FIELDS; ++i) {
      if (zimmermanField[i] == -1) {
        const char *target = zimmerFieldNames[i];
//        Serial.print("looking for: ");
//        Serial.println(target);
        char *ptr = strstr((char *)buf, target);
        if (ptr != NULL) {
          ptr += strlen(target)+1;
          zimmermanField[i] = strtod(ptr, &ptr);
          if (zimmermanField[i] == -9999.0)
            zimmermanField[i] = -1;
          else {
            Serial.print(target);
            Serial.println(zimmermanField[i]);
            ++zimmermanNumGot;
          }
        }
      }
    }

    if (zimmermanNumGot == NUM_ZIMMERMAN_FIELDS) {
      const int humidity = ( zimmermanField[maxhumidity] + zimmermanField[minhumidity] ) / 2;
      Serial.print(F("humidity: "));  Serial.println(humidity);
      const float precip = zimmermanField[precip_today_in] + zimmermanField[precipi];
      Serial.print(F("precip: "));  Serial.println(precip);
      int temp = ( ( zimmermanField[maxtempi] + zimmermanField[mintempi] ) / 2 ); 
      Serial.print(F("temp: "));  Serial.println(temp);
      int humidityFactor = ( 30 - humidity );
      Serial.print(F("humidityFactor: "));  Serial.println(humidityFactor);
      int tempFactor = ( ( temp - 70 ) * 4 );
      Serial.print(F("tempFactor: "));  Serial.println(tempFactor);
      int precipFactor = ( precip * -200 );  
      Serial.print(F("precipFactor: "));  Serial.println(precipFactor);
      
      int percent = 100 + humidityFactor + tempFactor + precipFactor; 
      int value = min(max(0,percent), 200);
      Serial.print(F("zimmerman value: "));  Serial.println(value);

      actual_water_adjust = value;
      startProgramAfterAdjust();
      c->close(true);  // close right now...no more onData
    }
  }, NULL);
  
//  Serial.printf("request temp\n");
  if (!aclient->connect(WUNDERGROUND, 80)) {
    free(aclient);
    startProgramAfterAdjust();
  }
}


void setUpOta(void) {
  server.on("/update", HTTP_POST, [](){
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
    ESP.restart();
  },[](){
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START){
      Serial.setDebugOutput(true);
      Serial.printf("Update filename: %s\n", upload.filename.c_str());
      uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
      if(!Update.begin(maxSketchSpace)){//start with max available size
        Update.printError(Serial);
      }
    } else if(upload.status == UPLOAD_FILE_WRITE){
      Serial.printf("uploaded: %d\n", upload.totalSize);
      if(Update.write(upload.buf, upload.currentSize) == upload.currentSize){
        // update the total percent complete on the web page
        // cpd
      }
      else {
        Update.printError(Serial);
      }
    } else if(upload.status == UPLOAD_FILE_END){
      if(Update.end(true)){ //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
      Serial.setDebugOutput(false);
    }
    yield();
  });
}

//format bytes
String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}

String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path){
  Serial.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload_edit(){
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    //Serial.print("handleFileUpload Data: "); Serial.println(upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
    Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
  }
}

void handleFileDelete(){
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.println("handleFileDelete: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate(){
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.println("handleFileCreate: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  if(!server.hasArg("dir")) {server.send(500, "text/plain", "BAD ARGS"); return;}
  
  String path = server.arg("dir");
  Serial.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }
  
  output += "]";
  server.send(200, "text/json", output);
}

void countRootFiles(void) {
  int num = 0;
  size_t totalSize = 0;
  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {
    ++num;
    String fileName = dir.fileName();
    size_t fileSize = dir.fileSize();
    totalSize += fileSize;
    Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
  }
  Serial.printf("FS File: serving %d files, size: %s from /\n", num, formatBytes(totalSize).c_str());
}

void setupWebServer(void) {
  SPIFFS.begin();

  countRootFiles();

  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  
  //load editor
  server.on("/edit", HTTP_GET, [](){
    if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
  });
  
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload_edit);

  setUpOta();

  //called when the url is not defined here
  //use it to load content from SPIFFS
  server.onNotFound([](){
    if(!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });

  server.begin();

  Serial.println("HTTP server started");
}

void command(String cmd, String id) {
  Serial.print(F("sending command "));
  Serial.println(cmd);

  Serial.print(F("Connecting to "));
  Serial.println(config.torrent_ip_addr);

  // Use WiFiClient class to create TCP connections
  WiFiClient httpclient;
  if (!httpclient.connect(config.torrent_ip_addr, config.torrent_ip_port)) {
    Serial.println(F("connection failed"));
    return;
  }

  Serial.println(F("connected"));
  String data = "{\"method\":\"" + cmd + "\"}";

  String s;
  s = "POST /transmission/rpc HTTP/1.1\n";
  s += "Content-Type: application/json-rpc\n";
  s += "X-Transmission-Session-Id: ";
  s += id;
  s += "\n";
  s += "Connection: close\n";
  s += "Content-Length: ";
  s += data.length();
  s += "\n\n";
  s += data;
  httpclient.print(s);

  Serial.println(F("waiting for reply"));
  while (httpclient.connected() || httpclient.available()) {
    String aLine = httpclient.readStringUntil('\n');
    Serial.println(aLine);
    delay(1);
  }
  httpclient.stop();
  Serial.println(F("done"));
}

String getId(void) {
  Serial.println(F("getting transmission id"));
  String id = "";

  // Open socket to WU server port 80
  Serial.print(F("Connecting to "));
  Serial.println(config.torrent_ip_addr);

  // Use WiFiClient class to create TCP connections
  WiFiClient httpclient;
  if (!httpclient.connect(config.torrent_ip_addr, config.torrent_ip_port)) {
    Serial.println(F("connection failed"));
    return id;
  }

  Serial.println(F("connected"));
  String data = "{\"method\":\"session-get\"}";

  String s;
  s = "POST /transmission/rpc HTTP/1.1\n";
  s += "Connection: close\n";
  s += "Content-Length: ";
  s += data.length();
  s += "\n\n";
  s += data;
  httpclient.print(s);

  String target = "X-Transmission-Session-Id: ";
  Serial.println(F("waiting for reply"));
  while (httpclient.connected() || httpclient.available()) {
    String aLine = httpclient.readStringUntil('\n');
//    Serial.println(aLine);
    if (aLine.startsWith(target)) {
      id = aLine.substring(target.length());
      Serial.print("ID is |");
      Serial.print(id);
      Serial.println("|");
      break;
    }
    delay(1);
  }
  httpclient.stop();
  Serial.println(F("done"));
  return id;
}

void startTorrents(String id) {
  command("torrent-start", id);
}

void stopTorrents(String id) {
  command("torrent-stop", id);
}



/*
todo:

CREATE DATABASE sprinkler;
create table action (
  ts timestamp DEFAULT CURRENT_TIMESTAMP,
  action char
);

create table zone (
  ts timestamp DEFAULT CURRENT_TIMESTAMP,
  zone int,
  duration int,
  action char,
  percentage int
);

create table name (
  zone int,
  name varchar(12),
  PRIMARY KEY (`zone`)
);


use forecast to adjust watering schedule
use weather underground api to get current weather...what forecast can I get?

adjust watering durations (% of total) depending on the season
i.e. less watering during the rainy season...more during the dry season?



zimmerman an torrents need to be rewritten using async
*/