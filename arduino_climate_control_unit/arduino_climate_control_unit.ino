#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>

enum PostDataTypes {
  Temperature,
  Preasure,
  Mode
};


#define FailString "FAIL"
#define OkString "OK"
#define OnString "ON"
#define OffString "OFF"
#define AutoString "AUTO"
#define RelayPin 8
#define ErrorCodeForSensorsData -500
#define EspRxPinOnArduino 11
#define EspTxPinOnArduino 10
#define SwitchDisplayButtonPin 14 //A0
#define SwitchModeButtonPin 15 //A0
String PostTemperatureCommand = "POST_TEMP";
String PostPreasureCommand = "POST_PREA";
String PostModeCommand = "POST_MODE";
String EndOftransmissionCommand = "EOT";

SoftwareSerial espSerial(EspRxPinOnArduino, EspTxPinOnArduino); // RX, TX

Adafruit_BMP280 bmp; // I2C // pin 3 - Serial clock out (SCLK) // pin 4 - Serial data out (DIN)// pin 5 - Data/Command select (D/C)// pin 6 - LCD chip select (CS)// pin 7 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(3, 4, 5, 6, 7);


bool _isBmpOk = false;
bool _isEspOk = false;
bool _isWiFiOk = false;
bool _isInitialized = false;
String _ipAddress = "N/A";
float _bmpPreasure = ErrorCodeForSensorsData;
float _bmpTemp = ErrorCodeForSensorsData;
bool _isLastSendDataFailed = true;
unsigned long _previousMillisMain = 0;
unsigned long _previousMillisBtn = 0;
unsigned long _previousMillisShowScreen = 0;
unsigned long _previousMillisCheckEsp = 0;
bool _isSwichDisplayButtonPressed = false;
bool _isSwichModeButtonPressed = false;
byte _screenNumber = 0; // 1 displayParameters (default), 2 display module states, 0 author about
byte _modeNumber = 0; //AUTO (default), ON, OFF
byte _timeSwitchDisplayButtonHolded = 0;


void(* resetFunc) (void) = 0;



const static unsigned char PROGMEM logoBmp[] =
{
  B00000000, B00011111, B11111000, B00000000, B00000111, B11111100, B00000000, B00000011, B11111111, B10000000, B00000000,
  B00000000, B01111111, B11111111, B00000000, B00111111, B11111111, B00000000, B00001111, B11111111, B11100000, B00000000,
  B00000001, B11111111, B11111111, B10000000, B11111111, B11111110, B00000000, B00111111, B11111111, B11111000, B00000000,
  B00000011, B11111111, B11111111, B11100001, B11111111, B11111000, B00000000, B11111111, B11111111, B11111100, B00000000,
  B00001111, B11111100, B00111111, B11100011, B11111111, B00110000, B00000001, B11111111, B10000011, B11111111, B00000000,
  B00001111, B11100000, B00000111, B11100111, B11110000, B00000000, B00000000, B11111100, B00000000, B01111111, B00000000,
  B00011111, B10000000, B00000001, B11001111, B11100000, B00000000, B00001110, B01110000, B00000000, B00011111, B10000000,
  B00111111, B00000000, B00000000, B10011111, B10000000, B00000000, B00111110, B01100000, B00000000, B00001111, B11000000,
  B00111110, B00000000, B00000000, B00011111, B00000000, B00000000, B01111111, B00000000, B00000000, B00000111, B11000000,
  B01111100, B00000000, B00000000, B00111111, B00000000, B00000001, B11111111, B00000000, B00000000, B00000011, B11100000,
  B01111100, B00000000, B00000000, B00111110, B00000000, B00000011, B11111111, B10000000, B00000000, B00000011, B11100000,
  B11111000, B00000000, B00010000, B00111100, B00000000, B00001111, B11111111, B10000000, B00000000, B00000001, B11110000,
  B11111000, B00000000, B01111000, B01111100, B00000000, B00111111, B11100111, B10000000, B00000000, B00000001, B11110000,
  B11111000, B00000000, B11111000, B01111100, B00000000, B01111111, B10000111, B11000000, B00000000, B00000001, B11110000,
  B11111000, B00000011, B11111100, B01111100, B00000001, B11111111, B00000111, B11000000, B00000000, B00000000, B11110000,
  B11110000, B00000111, B11111000, B01111100, B00000011, B11111100, B00000111, B11000000, B00000000, B00000000, B11110000,
  B11110000, B00011111, B11100000, B01111100, B00001111, B11111000, B00000111, B11000000, B00000000, B00000000, B11110000,
  B11111000, B01111111, B11000000, B01111100, B00011111, B11100000, B00000111, B11000000, B00000000, B00000001, B11110000,
  B11111000, B11111111, B00000000, B01111100, B01111111, B11000000, B00000111, B11000000, B00000000, B00000001, B11110000,
  B11111011, B11111110, B00000000, B01111100, B11111111, B00000000, B00000111, B10000000, B00000000, B00000001, B11110000,
  B11111111, B11111000, B00000000, B00111110, B11111100, B00000000, B00001111, B10000000, B00000000, B00000001, B11110000,
  B01111111, B11110000, B00000000, B00111110, B01111000, B00000000, B00001111, B10000000, B00000000, B00000011, B11100000,
  B01111111, B11000000, B00000000, B00111111, B01100000, B00000000, B00011111, B00000000, B00000000, B00000011, B11100000,
  B00111111, B00000000, B00000000, B00011111, B00000000, B00000000, B00111111, B00100000, B00000000, B00000111, B11000000,
  B00111111, B00000000, B00000000, B11001011, B10000000, B00000000, B01111110, B01110000, B00000000, B00001111, B11000000,
  B00011111, B11000000, B00000011, B11001100, B01000000, B00000000, B11111100, B01111000, B00000000, B00111111, B10000000,
  B00001111, B11110000, B00001111, B11100111, B11110000, B00000011, B11111100, B11111110, B00000000, B01111111, B00000000,
  B00000111, B11111111, B11111111, B11100011, B11111111, B00111111, B11111000, B01111111, B11111111, B11111110, B00000000,
  B00000011, B11111111, B11111111, B11000001, B11111111, B11111111, B11100000, B00111111, B11111111, B11111100, B00000000,
  B00000001, B11111111, B11111111, B00000000, B01111111, B11111111, B11000000, B00001111, B11111111, B11110000, B00000000,
  B00000000, B01111111, B11111110, B00000000, B00011111, B11111111, B00000000, B00000011, B11111111, B11100000, B00000000,
  B00000000, B00001111, B11110000, B00000000, B00000111, B11111000, B00000000, B00000000, B01111111, B00000000, B00000000
};


void setup() {
  Serial.begin(115200);
  initialize();
  displayModuleSatuses();
  delay(2000);
  _screenNumber = 1;
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - _previousMillisMain >= 1000) {

    _previousMillisMain = currentMillis;

    _bmpTemp = getBmpTemp();
    _bmpPreasure = getBmpPreasure();


    modeController();
  }

  if (currentMillis - _previousMillisBtn >= 100) {
    _previousMillisBtn = currentMillis;
    checkSwitchDisplayButton();
    checkSwitchModeButton();
    if (currentMillis - _previousMillisShowScreen >= 1000) {
      _previousMillisShowScreen = currentMillis;
      switch (_screenNumber) {
        case 0:
          displayAbout();
          break;
        case 1:
          displayParameters(_bmpTemp, _bmpPreasure, getMode());
          break;
        case 2:
          displayModuleSatuses();
          break;
      }
    }
  }

  if (currentMillis - _previousMillisCheckEsp >= 60000) {
    _previousMillisCheckEsp = currentMillis;
    displaySyncing();
    checkEsp();
    if (_isEspOk && _isWiFiOk && _ipAddress.length() && _ipAddress != "N/A") {
      postData(Temperature, String(_bmpTemp));
      postData(Preasure, String(_bmpPreasure));
      postData(Mode, getMode());
    }
  }

}

void initialize () {
  initButtons();
  initDisplay();
  initRelay();
  initBmp();
  initEsp();

  delay(1000);
  _isInitialized = true;
}
void initButtons() {
  digitalWrite(SwitchDisplayButtonPin, HIGH);
  digitalWrite(SwitchModeButtonPin, HIGH);
}

void initDisplay() {
  display.begin();              // Инициализация дисплея
  display.setContrast(20);      // Устанавливаем контраст
  display.clearDisplay();
  display.setTextColor(BLACK);  // Устанавливаем цвет текста
  display.setTextSize(1);       // Устанавливаем размер текста
  display.clearDisplay();       // Очищаем дисплей
  display.display();
  delay(1000);
  display.drawBitmap(0, 0, logoBmp, 84, 32, BLACK); // x, y, logo, w, h, color
  display.setCursor(0, 33);
  display.print("creative media");
  display.setCursor(15, 40);
  display.print("solutions");
  display.display();
}
void initRelay() {
  pinMode(RelayPin, OUTPUT);
  turnReleyOff();
}
void initBmp() {
  _isBmpOk = bmp.begin();
}
void initEsp() {
  espSerial.begin(9600); //default baud rate for current firmware
  delay(500);
  checkEsp();

}
void checkEsp() {
  _isEspOk = getEspState();
  if (_isEspOk) {
    _isWiFiOk = getWiFiStatus();
    if (_isWiFiOk) {
      _ipAddress = getIP();
    }
  }
}

void displayModuleSatuses() {  // screen 2
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.print("BMP280: ");
  if (!_isBmpOk) {
    display.println(FailString);
  }
  else {
    display.println(OkString);
  }

  display.print("Mode: ");
  display.println(getMode());
  display.println("WIFI: ");
  if (!_isEspOk) {
    display.println(FailString);
  }
  else {
    display.println(_ipAddress);
  }
  display.display();
}

void displayParameters (float bmpTemp, float bmpPreasure, bool releyState) { // screen 1 default
  if (!_isInitialized) {
    return;
  }
  else {
    display.clearDisplay();
    display.display();
    display.setCursor(0, 0);
    display.print("Here:");
    display.print(bmpTemp);
    display.println(" C");
    display.print("Here:");
    display.print(bmpPreasure);
    display.println(" mA");
    display.drawLine(0, 16, display.width(), 16, BLACK);
    display.setCursor(0, 18);
    display.print("There:");
    display.print(bmpTemp);
    display.println(" C");
    display.print("There:");
    display.print(bmpTemp);
    display.println(" %");
    display.drawLine(0, 34, display.width(), 34, BLACK);
    display.setCursor(0, 36);
    display.print("Mode: ");
    display.println(getMode());

    display.display();
  }
}

void displayAbout() { //screen 0
  if (!_isInitialized) {
    return;
  }
  else {
    display.clearDisplay();
    display.display();
    display.setCursor(0, 0);
    display.println("Author:Mykola");
    display.println("Pidopryhora");
    display.println("");
    display.println("88.genreal@");
    display.println("gmail.com");
    display.display();
  }
}

void displaySyncing() {
  if (!_isInitialized) {
    return;
  }
  else {
    display.clearDisplay();
    display.display();
    display.setCursor(20, 15);
    display.print("syncing");
    display.setCursor(10, 23);
    display.print("please wait");
    display.display();
  }
}

void modeController() {

  switch (_modeNumber) {
    case 1: //on
      turnReleyOn();
      return;
    case 2: //off
      turnReleyOff();
      return;
    default: //auto
      if (_bmpTemp > 28.00) {
        turnReleyOn();
      }
      if (_bmpTemp < 27.00) {
        turnReleyOff();
      }
      return;
  }
}

void turnReleyOn() {
  digitalWrite(RelayPin, LOW);
}

void turnReleyOff() {
  digitalWrite(RelayPin, HIGH);
}

float getBmpTemp() {
  if (_isBmpOk) {
    return bmp.readTemperature();
  }
  else {
    return ErrorCodeForSensorsData;
  }
}

float getBmpPreasure() {
  if (_isBmpOk) {
    return (bmp.readPressure() / 101325) * 1000;
  }
  else {
    return ErrorCodeForSensorsData;
  }
}

String getMode() {
  switch (_modeNumber) {
    case 1:
      return OnString;
    case 2:
      return OffString;
    default:
      return AutoString;
  }
}

bool getEspState () {
  byte attemptsCounter = 0;
  _ipAddress = "N/A";
  if (!espSerial) {
    _isEspOk = false;
    return _isEspOk;
  }
  else {
    while (attemptsCounter < 2) {
      attemptsCounter++;
      Serial.println("check if ESP up");
      espSerial.println("AT"); //this is not an AT command. ESP has custom firmware
      delay(1000);
      if (espSerial.available()) {
        if (espSerial.find(OkString)) {
          Serial.println(">Esp OK");
          _isEspOk = true;
          break;
        }
      }
      _isEspOk = false;
    }
  }
  return _isEspOk;
}
bool getWiFiStatus() {
  byte attemptsCounter = 0;
  byte waitFor = 0;
  if (!espSerial) {
    _isEspOk = false;
    return _isEspOk;
  }
  else {
    Serial.println("getting wifi status");
    while (attemptsCounter < 2) {
      attemptsCounter++;
      Serial.println("send WIFISTATUS");
      espSerial.println("WIFISTATUS");
      delay(500);
      while (!espSerial.available()) {
        if (waitFor > 5) {
          break;
        }
        Serial.println("wait");
        waitFor++;
        delay(500);
      }
      if (espSerial.available()) {
        if (espSerial.find("3")) {
          Serial.println(">WIFI OK");
          _isWiFiOk = true;
          break;
        }
        else {
          _isWiFiOk = false;
        }
      }
      else {
        _isWiFiOk = false;
      }
    }
  }
  return _isWiFiOk;
}
String getIP() {
  byte waitFor = 0;
  _ipAddress = "N/A";
  if (!espSerial) {
    _isEspOk = false;
    return _ipAddress;
  }
  else {
    Serial.println("getting IP");
    espSerial.println("IP");
    delay(500);
    while (!espSerial.available()) {
      if (waitFor > 7) {
        break;
      }
      Serial.println("wait for send response");
      waitFor++;
      delay(500);
    }
    String ipString = espSerial.readString();
    delay(500);
    Serial.println(ipString);
    String ip = ipString.substring(2, ipString.length());
    if (ip.length() < 7) {
      _ipAddress = "N/A";
    }
    else {
      _ipAddress = ip;
    }
    return  _ipAddress;
  }
}
void resetEspAndArduino() {
  byte attemptsCounter = 0;
  if (!espSerial) {
    _isEspOk = false;
    return;
  }
  else {
    Serial.println("reseting ESP");
    while (attemptsCounter < 5) {
      attemptsCounter++;
      espSerial.println("RST");
      delay(1000);
      if (espSerial.available()) {
        if (espSerial.find(OkString)) {
          Serial.println(">Esp OK");
          break;
        }
      }
    }
    resetFunc();
    delay(100);
    return;
  }
}

void checkSwitchDisplayButton() {
  if (_isSwichModeButtonPressed) {
    return;
  }
  if (digitalRead(SwitchDisplayButtonPin) == LOW ) {
    _timeSwitchDisplayButtonHolded++;
    if (!_isSwichDisplayButtonPressed) {
      _isSwichDisplayButtonPressed = true;
      if (_screenNumber < 2) {
        _screenNumber++;
      } else {
        _screenNumber = 0;
      }
    }
    Serial.print("display btn holded during ");
    Serial.println(_timeSwitchDisplayButtonHolded * 100);// delay for this method
    if (_timeSwitchDisplayButtonHolded >= 100) {
      resetEspAndArduino();
    }
  }
  if (digitalRead(SwitchDisplayButtonPin) == HIGH && _isSwichDisplayButtonPressed) {
    _isSwichDisplayButtonPressed = false;
    _timeSwitchDisplayButtonHolded = 0;
  }
}

void checkSwitchModeButton() {
  if (_isSwichDisplayButtonPressed) {
    return;
  }
  if (digitalRead(SwitchModeButtonPin) == LOW ) {
    if (!_isSwichModeButtonPressed) {
      _isSwichModeButtonPressed = true;
      if (_modeNumber < 2) {
        _modeNumber++;
      } else {
        _modeNumber = 0;
      }
    }
    Serial.println("mode btn holded during ");

  }
  if (digitalRead(SwitchModeButtonPin) == HIGH && _isSwichModeButtonPressed) {
    _isSwichModeButtonPressed = false;
  }
}

void postData(PostDataTypes type, String data) {

  byte waitFor = 0;
  if (_isWiFiOk) {
    Serial.print("post ");
    Serial.println(type);
    String command;
    switch (type) {
      case Temperature:
        command += PostTemperatureCommand;
        break;
      case Preasure:
        command += PostPreasureCommand;
        break;
      case Mode:
        command += PostModeCommand;
        break;
    }
    command += data;
    command += EndOftransmissionCommand;
    Serial.print("Sending ");
    Serial.println(command);
    espSerial.println(command);
    delay(500);
    while (!espSerial.available()) {
      if (waitFor > 7) {
        break;
      }
      Serial.println("wait for send response");
      waitFor++;
      delay(500);
    }
    if (espSerial.find(OkString)) {
      Serial.println(">Esp OK");
      _isWiFiOk = true;
    }
    else {
      Serial.println(espSerial.readString());
    }
  }
}


