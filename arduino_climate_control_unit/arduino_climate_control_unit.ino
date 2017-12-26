#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>


#define FailString "FAIL"
#define OkString "OK"
#define OnString "ON"
#define OffString "OFF"
#define RelayPin 8
#define ErrorCodeForSensorsData -500
#define EspRxPinOnArduino 11
#define EspTxPinOnArduino 10
#define ButtonPin 14 //A0
String PostTemperatureCommand = "POST_TEMP";
String PostPreasureCommand = "POST_PREA";
String PostModeCommand = "POST_MODE";
String EndOftransmissionCommand = "EOT";

SoftwareSerial espSerial(EspRxPinOnArduino, EspTxPinOnArduino); // RX, TX

Adafruit_BMP280 bmp; // I2C // pin 3 - Serial clock out (SCLK) // pin 4 - Serial data out (DIN)// pin 5 - Data/Command select (D/C)// pin 6 - LCD chip select (CS)// pin 7 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(3, 4, 5, 6, 7);

String _releyState = "OFF"; //false OFF, true ON
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
bool _isButtonPressed = false;
byte _screenNumber = 0;
int _timeButtonHolded = 0;

void(* resetFunc) (void) = 0;

enum PostDataTypes {
  Temperature,
  Preasure,
  Mode
};

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
    _releyState = getReleyState();

    modeController();
  }

  if (currentMillis - _previousMillisBtn >= 100) {
    _previousMillisBtn = currentMillis;
    checkButton();
    if (currentMillis - _previousMillisShowScreen >= 1000) {
      _previousMillisShowScreen = currentMillis;
      switch (_screenNumber) {
        case 0:
          displayModuleSatuses();
          break;
        default:
          displayParameters(_bmpTemp, _bmpPreasure, _releyState);
          break;
      }      
    }
  }

  if (currentMillis - _previousMillisCheckEsp >= 60000) {
    _previousMillisCheckEsp = currentMillis;
    checkEsp();
    if(_isEspOk && _isWiFiOk && _ipAddress.length() && _ipAddress != "N/A"){
      postData(Temperature, String(_bmpTemp));
      postData(Preasure, String(_bmpPreasure));
      postData(Mode, _releyState);
    }
  }

}

void initialize () {
  initButton();
  initDisplay();
  initRelay();
  initBmp();
  initEsp();

  delay(1000);
  _isInitialized = true;
}
void initButton() {
  digitalWrite(ButtonPin, HIGH);
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

void displayModuleSatuses() {  // screen 0
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
  display.println(getReleyState());
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
    display.println(getReleyState());   
  
    display.display();
  }
}

void modeController() {
  if (_bmpTemp == ErrorCodeForSensorsData) {
    return;
  }

  if (_bmpTemp > 28.00) {
    turnReleyOn();
  }
  if (_bmpTemp < 27.00) {
    turnReleyOff();
  }

}

void turnReleyOn () {
  digitalWrite(RelayPin, LOW);
  _releyState = "ON";
}

void turnReleyOff () {
  digitalWrite(RelayPin, HIGH);
  _releyState = "OFF";
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

String getReleyState() {
  if (_releyState) {
    return OnString;
  } else {
   return OffString;
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
      delay(1000);
      if (espSerial.available()) {
        if (espSerial.find("3")) {
          Serial.println(">WIFI OK");
          _isWiFiOk = true;
        }
        else {
          _isWiFiOk = false;
        }
      }
    }
  }
  return _isWiFiOk;
}
String getIP() {
  byte attemptsCounter = 0;
  _ipAddress = "N/A";
  if (!espSerial) {
    _isEspOk = false;
    return _ipAddress;
  }
  else {
    Serial.println("getting IP");
    while (attemptsCounter < 2) {
      attemptsCounter++;
      espSerial.println("IP");
      delay(1000);
      if (espSerial.available()) {
        String ipString = espSerial.readString();
        delay(500);
        String ip = ipString.substring(2, ipString.length());
        Serial.print(ip);
        return ip;
      }
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

void checkButton() {
  if (digitalRead(ButtonPin) == LOW ) {
    _timeButtonHolded++;
    if (!_isButtonPressed) {
      _isButtonPressed = true;
      if (_screenNumber < 1) {
        _screenNumber++;
      } else {
        _screenNumber = 0;
      }
    }
    Serial.print("btn holded during ");
    Serial.println(_timeButtonHolded * 100);// delay for this method
    if (_timeButtonHolded >= 100) {
      resetEspAndArduino();
    }
  }
  if (digitalRead(ButtonPin) == HIGH && _isButtonPressed) {
    _isButtonPressed = false;
    _timeButtonHolded = 0;
  }
}

void postData(PostDataTypes type, String data) {
  byte attemptsCounter = 0;
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
    espSerial.println(command);
    delay(1000);
    if (espSerial.available()) {
      if (espSerial.find(OkString)) {
        // do not wait response
        
        _isWiFiOk = true;
      }
      else {
        //_isWiFiOk = false;
      }
    }
  }
}


