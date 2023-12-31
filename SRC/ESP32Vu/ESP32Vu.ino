/*
@Leo Bros Studios
@Alexis Alexopoulos & Tarso Calixto
Code to measure audio analog input and convert to digital values.
Based on ESP32
Using 2 Resistors to create a voltage divider and capturing the entire waveform. Bias point at 1.65V
Reference voltage is 3.3V
Inspired by Thijs de Bont https://fittingmedia.wordpress.com/2013/12/18/arduino-vu-meter/
Adafruit example https://learn.adafruit.com/adafruit-microphone-amplifier-breakout/measuring-sound-levels */

#include "SPI.h"
#include "math.h"
#include "SdFat.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

// For the Adafruit screen, these are the pin outlets for Hardware SPI (FAST).
#define TFT_CS     22 //22
#define TFT_DC     21 //21
#define TFT_RST    14 //17

// Define SD card CS pin
#define SD_CS      5     // External SD card reader Chip Select

// Create an instance of the ILI9341 library
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Define SD
SdFat SD;

//analog input pins
int analogPinLeft = 2;
int analogPinRight = 4;

//Button definitions. Vars will not change
const int buttonPin = 32;     // the number of the pushbutton pin. Const, will not change

//Button definitions. Vars will change
// Variables will change:
volatile int buttonPushCounter = 0;   // counter for the number of button presses
volatile bool changed = false;        //boolean for color switch in loop

//Vars for position of drawing
int xPos = 160;                    //XPos of the needle start
int yPos = 120;                    //YPos of the needle start
int radius = 105;                  //size of the circle
int needlePercent = 105;           // Percentage of the circle for drawing lines. If value == radius then lines will drawn until end of circle
int vuLines = 30;                  //Ammount of lines to draw
int vuLinesVariance = 20;          //Value of hig big to draw lines. Circle end - vulinesvariance. Higer value is longer lines

//Define vars for Left channel
int audioPeakLeft;                 //holds the highest audio peak value for the Left channel
int audioLevelLeft;                //value of analogread for the Left channel, remapped between 0-4095
int vuLevelLeft;                   //logarithmic sum used to draw peak values Left channel
int previousVuLevelLeft;           //Used to calculate falltime Left channel
int LastvuLevelLeft;               //Used to save last value left channel

//Define vars for Right channel
int audioPeakRight;                //holds the highest audio peak value for the Right channel
int audioLevelRight;               //value of analogread for the Right channel, remapped between 0-4095
int vuLevelRight;                  //logarithmic sum used to draw peak values Right channel
int previousVuLevelRight;          //Used to calculate falltime Right channel
int LastvuLevelRight;              //Used to save last value right channel

//Vars for VU scale
int maxVuScaleValue = 58;          //Log bas 10 calculation maximum. value of 58
int minVuScaleValue = 0;           //Scale start at 0

//Part for timings
int vuSmoothDelay = 30;            //ms to measure the peak
long vuSmoothTimer;                //Time between Milis and vuSmoothDelay
long previousVuSmoothTimer;        //New value of Milis after the Milis loop has run
int falltimeLeft = 15;             //fall time threshold
int falltimeRight = 15;            //fall time threshold
int redline = 20;

//Define start and end radian
float gs_radLeft=3.665;            //start radian 210 degrees for left channel
float ge_radLeft=5.325;            //end radian 300 degrees for left channel
float gs_radRight=2.617;           //start radian 150 degrees for right channel
float ge_radRight=1.047;           //end radian 60 degrees for right channel

//Voltage divider
int minADC = 1900;                 //ingore readings above 1900 and belove 2150 (noise)
int maxADC = 2150;                 //ingore readings above 1900 and belove 2150 (noise)

//Default values
int valLeft = 0;                   //analogRead left channel
int valRight = 0;                  //analogRead right channel
int ValLeftMax = 0;                //used for diagnostics
int ValLeftMin = 4096;             //used for diagnostics
int ValRightMax = 0;               //used for diagnostics
int ValRightMin = 4096;            //used for diagnostics

//Debounce Interrupt
long debouncing_time = 200;         //Debouncing Time in Milliseconds for butten
volatile unsigned long last_micros; //Dbouncing time for buttong

//Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF
#define LIGHTBLUE 0x9E79
#define LIGHTRED 0xD801
#define LIGHTORANGE 0xF4A6
#define TEAL 0x2658

//TFT buffer pixel
#define BUFFPIXEL 20

//Diagnostics
//#define debug

void setup(){
  Serial.begin(115200);
  Serial.println();
  tft.begin();                    //Init TFT screen
  tft.setRotation(3);
  delay(100);

// Initialize the SD card reader
if (!SD.begin(SD_CS, SD_SCK_MHZ(25))) {
  tft.fillScreen(ILI9341_BLACK);  //Set background to black
  tft.setTextColor(RED, BLACK);
  tft.setCursor(110, 120);
  tft.setTextSize(2);
  tft.print("SD FAILED");
  Serial.println("SD card initialization failed!");
  //while (true);  // Halt the program if SD card initialization fails
}
else{
  //show boot image
  bootImg();
}



//define pinmode
pinMode(analogPinLeft, INPUT);
pinMode(analogPinRight, INPUT);
//Interrupt////////////////////////////////////////////////////////////////
pinMode(buttonPin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(buttonPin),changeColor,RISING);
delay(100);
///////////////////////////////////////////////////////////////////////////////
}

void loop(){
//part to run if interrupt is triggered
  if (changed){
      changed = false;
      buttonPushCounter++;
      buttonPushCounter = buttonPushCounter % 2;
  //Reset values
      audioPeakLeft = 0;                    //Reset audio peak meter
      audioPeakRight = 0;                   //Reset audio peak meter
      previousVuSmoothTimer = millis();     //Reset smoothing timer
      if (buttonPushCounter == 0){
        displayBMP("VU_Blue.bmp");
      }
      if (buttonPushCounter == 1){
        displayBMP("VU_Amber.bmp");
      }
  }
//part for the VU meter readings and drawing on the screen
  else{
    valLeft = analogRead(analogPinLeft);
    valRight = analogRead(analogPinRight);
    GetAudioPeakLeft(valLeft);                //captures audiopeak value for Left channel
    GetAudioPeakRight(valRight);              //captures audiopeak value for Right channel

    #ifdef debug
    //Start Diagnostic - Store lowest and highest value Left #################################
    if (valLeft > ValLeftMax)
    {
      ValLeftMax = valLeft;
      tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
      //screen.setFont(u8g2_font_profont11_mf); //7 pixels
      tft.setCursor(100, 100);
      tft.setTextSize(2);
      tft.print(ValLeftMax);          //audiopeak log10 *62
    }

    if (valLeft < ValLeftMin)
    {
      ValLeftMin = valLeft;
      tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
      //screen.setFont(u8g2_font_profont11_mf); //7 pixels
      tft.setCursor(100, 150);
      tft.setTextSize(2);
      tft.print(ValLeftMin);          //audiopeak log10 *62
    }

    //Store lowest and highest value Right
    if (valRight > ValRightMax)
    {
      ValRightMax = valRight;
      tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
      //screen.setFont(u8g2_font_profont11_mf); //7 pixels
      tft.setCursor(200, 100);
      tft.setTextSize(2);
      tft.print(ValRightMax);          //audiopeak log10 *62
    }


    if (valRight < ValRightMin)
    {
      ValRightMin = valRight;
      tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
      //screen.setFont(u8g2_font_profont11_mf); //7 pixels
      tft.setCursor(200, 150);
      tft.setTextSize(2);
      tft.print(ValRightMin);          //audiopeak log10 *62
    }
    //End Diagnostics ############################################################
#endif

    //Milis funtion for saving highest audiopeak in var ms. millis is current time ,previousVuSmoothTimer is the last time when the script block ran.
      //Part for the left channel
      vuSmoothTimer = millis() - previousVuSmoothTimer; //Check if the detection time has passed.
      if(vuSmoothTimer > vuSmoothDelay ) { //if the ms var exceeded
        vuLevelLeft = (log10(audioPeakLeft)*16); //Convert the audioPeak to logarithmic and set the VU scale for the Left channel. log10x2048 = ~ 3.3
        if (vuLevelLeft < previousVuLevelLeft ) {
          vuLevelLeft = previousVuLevelLeft - falltimeLeft;    //falltime for natural needle fall 
          if (vuLevelLeft < 0){
            vuLevelLeft = 0;
          }
        }
    //Part for the right channel
        vuLevelRight = (log10(audioPeakRight)*16); //Convert the audioPeak to logarithmic and set the VU scale for the right channel. log10x2048 = ~ 3.3
        if (vuLevelRight < previousVuLevelRight ) {
          vuLevelRight = previousVuLevelRight - falltimeRight;    //falltime for natural needle fall
          if (vuLevelRight < 0){
            vuLevelRight = 0;
          }
        }
    //Writing values to the screen
    //Left channel
        DrawPartialgauge(xPos,yPos,radius,needlePercent,vuLevelLeft,minVuScaleValue,maxVuScaleValue,gs_radLeft,ge_radLeft,vuLinesVariance,vuLines);
        LastvuLevelLeft = vuLevelLeft;

    //Right channel
        DrawPartialgauge(xPos,yPos,radius,needlePercent,vuLevelRight,minVuScaleValue,maxVuScaleValue,gs_radRight,ge_radRight,vuLinesVariance,vuLines);
        LastvuLevelRight = vuLevelRight;

    //Reset values
        previousVuLevelLeft = vuLevelLeft;        //set new vuLevel for Left channel
        previousVuLevelRight = vuLevelRight;        //set new vuLevel for Left channel
        audioPeakLeft = 0;                     //Reset audio peak meter after threshold ms have past
        audioPeakRight = 0;                     //Reset audio peak meter after threshold ms have past
        previousVuSmoothTimer = millis(); //Reset smoothing timer. Set to current time for calculation
      }
  }
}

//Function to measure input on Analog port
void GetAudioPeakLeft(int channel){
    audioLevelLeft = channel; //Read audio input
  //Part to map values. mapped 0 should not be in the first and last if. in this case zero is between 2000 and 3000
  if(audioLevelLeft < minADC){ //Audio is biased on 1.65 volts (half Vcc) by voltage divider. with no input analogread value ~ 2048
      audioLevelLeft = map(audioLevelLeft,0,2048,2048,0); //Convert downgoing signal to positive values.
  }
  //Convert upgoing signal to the same range as the downgoing signal.
  else if(audioLevelLeft >= maxADC && audioLevelLeft <= 4095){
    audioLevelLeft = map(audioLevelLeft,2048,4095,0,2048);
  }
  //Overload protection
  else if (audioLevelLeft > 4095){
    audioLevelLeft = 4095;
  }
  else { //for detection the highest value within the detection time.
    audioLevelLeft = 0;
  }
  if(audioLevelLeft > audioPeakLeft) {
    audioPeakLeft = audioLevelLeft; //Save the highest measured value to variable
  }
}

//Function to measure input on Analog port
void GetAudioPeakRight(int channel){
    audioLevelRight = channel; //Read audio input
  //Part to map values. mapped 0 should not be in the first and last if. in this case zero is between 2000 and 3000
  if(audioLevelRight < minADC) { //Audio is biased on 1.65 volts (half Vcc) by voltage divider. with no input analogread value ~ 2048
    audioLevelRight = map(audioLevelRight,0,2048,2048,0); //Convert downgoing signal to positive values.
  }
  else if(audioLevelRight >= maxADC && audioLevelRight <= 4095){ //Convert upgoing signal to the same range as the downgoing signal.
    audioLevelRight = map(audioLevelRight,2048,4095,0,2048);
  }
  //Overload protection
  else if (audioLevelRight > 4095){
    audioLevelRight = 4095;
  }
  else { //for detection the highest value within the detection time.
    audioLevelRight = 0;
  }
  if(audioLevelRight > audioPeakRight) {
    audioPeakRight = audioLevelRight; //Save the highest measured value to variable
  }
}

/*Function DrawPartialgauge
1 = byte X center position
2 = byte Y center position
3 = byte radius of the gauge
4 = byte needle percentage
5 = int v is the input value
6 = int minimum value
7 = int maximum value
8 = float start radinats
9 = float end radinats
10 = variance for the ring size, lengt of lines
11 = int lines for the ammount of lines (for lines/value calculation)
*/
void DrawPartialgauge(int x, int y, int r, int p, int v, int minVal, int maxVal, float s_rad, float e_rad, int variance, int lines) {
  int units = (maxVal - minVal);
  int rv;
  float factor =  ((float)(lines) / (units));
  float value = (factor * (v));
  float maxValue = (factor * (maxVal));
  float minValue = (factor * (minVal));
  int startx;
  int starty;
  int endx;
  int endy;
  int startxClear;
  int startyClear;
  int endxClear;
  int endyClear;
//Draw until red line
    for (int a= 0; a < maxVuScaleValue; a++) {
    //do the math
      rv = r-variance;
      int * array1 = calculateRing(x,y,rv,p,a,minValue,maxValue,s_rad,e_rad);
      startx = array1[0];
      starty = array1[1];
      rv = r;
      int * array2 = calculateRing(x,y,rv,p,a,minValue,maxValue,s_rad,e_rad);
      endx = array2[0];
      endy = array2[1];
    //drawing logic
      if (a >= value ){
          tft.drawLine(startx,starty,endx,endy, BLACK);
      }
      else if (a < redline){
        switch (buttonPushCounter){
          case 0:
            tft.drawLine(startx,starty,endx,endy, LIGHTBLUE);
            break;
          case 1:
            tft.drawLine(startx,starty,endx,endy, LIGHTORANGE);
        break;
        }
      }
      else{
        switch (buttonPushCounter)
          {
          case 0:
            tft.drawLine(startx,starty,endx,endy, RED);
            break;
          case 1:
            tft.drawLine(startx,starty,endx,endy, TEAL);
        break;
        }
      }
    }
}

/*Function calculate coordinates
1 = byte X center position
2 = byte Y center position
3 = byte radius of the gauge
4 = byte needle percentage
5 = int c is the input value
6 = int minimum value
7 = int maximum value
8 = float start radinats
9 = float end radinats
*/
//Function to calculate the enpoint x,y values
  int * calculateRing(int x, byte y, byte r, byte p, int v, int minVal, int maxVal, float s_rad, float e_rad) {
    int n=(r/100.00)*p; // calculate needle percent lenght
    float i=((v-minVal)*(e_rad-s_rad)/(maxVal-minVal)+s_rad);
    int xp = x+(sin(i) * n);
    int yp = y-(cos(i) * n);
    //TODO return xp en yp
    static int arr1[2];
    arr1[0] = xp;
    arr1[1] = yp;
    return arr1;
  }

void bootImg(){
  //showing splash screen
  displayBMP("UP_Blue.bmp");
  delay(3000);
  //Show VU screen
  displayBMP("VU_Blue.bmp");
}

void changeColor(){
  if((long)(micros() - last_micros) >= debouncing_time * 1000) {
      changed = true;
      last_micros = micros();
  }
}

void displayBMP(const char* filename) {
    File bmpFile = SD.open(filename);
    if (!bmpFile) {
        Serial.println("Error opening BMP file!");
        return;
    }

    // Skip BMP header (54 bytes)
    bmpFile.seek(54);

    uint16_t pixel;
    uint8_t b, g, r;

    for (int y = tft.height() - 1; y >= 0; y--) {
        for (int x = 0; x < tft.width(); x++) {
            b = bmpFile.read();
            g = bmpFile.read();
            r = bmpFile.read();
            
            // Convert color to 16-bit pixel
            pixel = tft.color565(r, g, b);
            
            tft.drawPixel(x, y, pixel);
        }
    }

    bmpFile.close();
}