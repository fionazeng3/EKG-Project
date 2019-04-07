// Joseph Shin 1143310, Fiona Zeng 1565731
// Lab 8
// This sketch contains our entire
// ECG HeartMonitor program.
// It begins with a prompt to connect to Bluetooth
// via a mobile device. Then, it attempts to stabilize
// the incoming signal. With the stabilization complete,
// it will then interpret the ECG signal. It displays
// BPM, QRS Interval, Bradycardia and Tachycardia.
// After 30 seconds, the program autostops. The user 
// can then scroll through the recorded data.

#include <stdint.h>
#include <kinetis.h>
#include "ILI9341_t3.h"
#include <XPT2046_Touchscreen.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#define TFT_DC 9
#define TFT_CS 10
#define CS_PIN  4

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);
XPT2046_Touchscreen ts(CS_PIN);      

// Heart rate BPM detection/drawing variables
double filteredSignal = 0;
double filteredSignal0 = 0;
double heartRateTimer = 0;
double QRSTimer = 0;
double adaptiveThresh = 0;
double RRPeriod = 0;
double heartRate = 0;
double QRSVal = 0;
unsigned long pixelTimer = 0;
unsigned long startTimer = 0;  
int adcValueCopy; 
int x, y = 0;
int xPrev;
int yPrev = 0;   
unsigned long pixelTime = 40 / (320 / 40); // 320/40 is the pixels per block. 40ms is the time per block.  

// Calibration phase variables
double calFiltSignal;          
int BeatCount = 0;                            
double calibratedThreshold;  
int threshCount = 0;          
unsigned long calibrationTimer = 0;   
int prevBeatCount = 0;
boolean stabilized = false;
volatile int stabilizedFlag = 0;

// Moving average variables
double heartRateArray[3];
int heartRateIndex = 0;
double QRSValArray[5];
int qrsValIndex = 0;
int beginQRS = 0;
int startDraw = 0;

// Scroll variables
int directionChosen = 0;
int lineCount = 0;
unsigned long bufferTimer = 0;
int scrollxPrev;
int scrollyPrev = 0;
int scrollX;
volatile int touchData[20] = {0};
volatile int readIndex;
int addIndex, leftCount, rightCount, touchPrev, currentDirection, scrolling;
volatile int myBuffer[3600]; 
volatile int bufferIndex;
volatile int adcValue;

// Touch start/stop variables
boolean previousTouchState = false;
int startCube = -1;

void setup() {
  Serial.begin(9600);

  bufferIndex = 0;
  adcValue = 0;

  // Screen Setup 
  tft.begin(); 
  tft.setRotation(3); 
  tft.fillScreen(ILI9341_WHITE);

  // Initialize PDB, ADC, and Bluetooth
  adcInit();
  pdbInit();
  heartRateMonitorBlueInit();
  
  // Draw start button
  tft.fillScreen(ILI9341_WHITE);
  tft.setCursor(265,145);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.fillCircle(280,150,20,ILI9341_RED);
  tft.print("Start");
  tft.setTextColor(ILI9341_BLACK);
  
  // Welcome screen
  tft.setCursor(0,0);
  tft.setTextSize(3);
  tft.println("    ECG PROGRAM");
  tft.println("");
  tft.setTextSize(2);
  tft.println("          Welcome");
  tft.println("     Please press start");
  tft.println("          to begin");
  tft.println("");
  tft.setTextSize(1);
  tft.println("            Joseph Shin, Fiona Zeng, CSE474");
  tft.println("                      Winter 2018");
  
  // Begin timer for drawing data
  pixelTimer = millis();
}

void loop() {
  // Manages touch start/stop states
  touchStartStop();

  // Stabilization/start/stop logic
  if (startCube == 1) {
    if (stabilizedFlag == 0) {
      stabilize();
    } 
    EKG();    
  } else {
    stabilizedFlag = 0;
  }
}

// A simple state machine to manage touched/not touched states
void touchStartStop() {
  boolean isTouched = false;
  if(ts.touched()) {
    TS_Point p = ts.getPoint();
    isTouched = (p.x >= 400 && p.x <=1100 && p.y>= 1300 && p.y <= 1800);
  }
  if (isTouched == true && previousTouchState == false) {
    startCube = startCube * -1;
  }
  previousTouchState = isTouched;
}


// Waits for the values to stabilize,
// or remain between two preset thresholds (0 and 4000)
// Then attempts to find an adaptive threshold
// for detecting R peaks
void stabilize() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_BLUE);
  tft.setCursor(0,0);
  tft.println("");
  tft.println("   STABILIZING");
  tft.println("");
  tft.setTextSize(2);
  tft.println("   Please keep fingers");
  tft.println("          still");
  delay(2000);
  
  int stableVal[5000];
  int adcValueCopy;
  boolean enoughStableVals = false;
  int stableValIndex = 0;

  // Making sure input values are stable
  while (stabilizedFlag == 0) {
    adcValueCopy = adcValue;
    stableVal[stableValIndex] = adcValueCopy;
    stableValIndex++;
    if (stableValIndex == 5000) {
      stableValIndex = 0;
      enoughStableVals = true;
    }
    if (enoughStableVals) {
      for (int i = 0; i < 5000; i++) {
        if (stableVal[i] > 0 && stableVal[i] < 4000) {
         stabilizedFlag = 1;
        } else {
         stabilizedFlag = 0;
        }
      }
    }
  }

  stabilizedFlag = 0;
  calibratedThreshold = 10000;
  BeatCount = 0;
  tft.setCursor(0,0);
  tft.fillScreen(ILI9341_WHITE);
  tft.println("");
  tft.println("    VALUES ARE STABLE");
  tft.println("");
  tft.println("FINDING ADAPTIVE THRESHOLD");
  delay(1000);
  
  // Begin finding adaptive threshold
  calibrationTimer = millis();
  while (stabilizedFlag == 0) {
    calFiltSignal = exponentialAverageCal(0.8, calFiltSignal, adcValue);
    
    if (calFiltSignal > calibratedThreshold) {
      RRPeriod = (millis() - heartRateTimer);
      if (RRPeriod > 300) { // If a normal beat is found
        heartRate =  60 / (RRPeriod / 1000);
        heartRateTimer = millis();
      
        if (heartRate > 160) { // If threshold is too low (detecting too many R peaks)
          BeatCount = 0;
          calibratedThreshold = 1.1 * calibratedThreshold;
        } else if (heartRate < 30) { // If threshold is too high (detecting too few R peaks)
          BeatCount = 0;
          calibratedThreshold = 0.85 * calibratedThreshold;
        } else {
          BeatCount++; // Beat is in normal range
        }
      }
    threshCount = 0;
    } else {
    threshCount++;
    }
    if (threshCount > 10000) { // If it is too long without a heartbeat, then last threshold hold attempt failed
    threshCount = 0;
    calibratedThreshold = 10000;
    }

    if (BeatCount  < prevBeatCount && (BeatCount != 0) && (prevBeatCount != 0)) {
    } else if (BeatCount > prevBeatCount) {
      tft.print("."); // print each heartbeat
    }
    prevBeatCount = BeatCount;
    
    // If the adaptive threshold is taking too long (20s)
    if ((millis() - calibrationTimer) >= 20000) {
      tft.fillScreen(ILI9341_WHITE);
      tft.setCursor(0,0);
      tft.println("ADAPTIVE THRESHOLD NOT ");
      tft.println("FOUND");
      tft.println("USING DEFAULT VALUE");
      delay(1000);
      adaptiveThresh = 10000; 
      stabilizedFlag = 1;
    }

    // If 5 consecutive beats are found
    if (BeatCount > 5) {
      stabilizedFlag = 1;
      tft.fillScreen(ILI9341_WHITE);
      tft.setCursor(0,0);
      tft.println("ADAPTIVE THRESHOLD FOUND");
      delay(1000);
      if (calibratedThreshold > 9000 && calibratedThreshold < 21000) {
        adaptiveThresh = calibratedThreshold;     
      } else {
        adaptiveThresh = 10000;
      }
      tft.println(calibratedThreshold);
      delay(2000);
    }
  }

  // Finish stabilization
  tft.setCursor(0,0);
  tft.fillScreen(ILI9341_WHITE);
  tft.println("STABILIZING");
  tft.println("COMPLETE");
  x = 0;
  xPrev = 0;
  delay(500);

  // Draw on-screen detection labels
  tft.fillScreen(ILI9341_WHITE);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setCursor(30, 130);
  tft.print("QRSI");
  
  // draw data for heart rate 
  tft.setCursor(90, 130);
  tft.print("HR");

  // draw data for BC
  tft.setCursor(140, 130); 
  tft.print("BC");

  // draw data for TC
  tft.setCursor(190, 130);
  tft.print("TC");
  tft.setCursor(20, 170);

  // drawing the stop sign
  tft.setCursor(265,145);
  tft.setTextSize(1);
  tft.fillCircle(280,150,20,ILI9341_RED);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Stop");
  tft.setTextColor(ILI9341_BLACK);
  
  // drawing labels
  tft.setCursor(60, 170);
  tft.print("ms");
  tft.setCursor(100, 170);
  tft.print("BPM");
  
  // Drawing borders
  tft.fillRect(30, 150, 40, 20, ILI9341_BLACK);
  tft.fillRect(80,150,40,20,ILI9341_BLACK);
  tft.fillRect(130,150,40,20,ILI9341_BLACK);
  tft.fillRect(180,150,40,20,ILI9341_BLACK);
  
  // Drawing scroll arrows
  tft.fillRect(2, 200, 316,236,ILI9341_GREEN);
  tft.drawLine(2,218, 316, 218, ILI9341_BLACK);
  tft.drawLine(2,218, 32, 200, ILI9341_BLACK);
  tft.drawLine(2,218, 32, 236, ILI9341_BLACK);
  tft.drawLine(316,218, 286, 200, ILI9341_BLACK);
  tft.drawLine(316,218, 286, 236, ILI9341_BLACK);
  
  stabilized = true;
  startTimer = millis();
  bufferIndex = 0;
} 

// Manages the EKG drawing
void EKG() {
  if (millis() - startTimer <= 30000) {
    startDraw = 0;
    adcValueCopy = adcValue;
    filteredSignal0 = exponentialAverage(0.8, filteredSignal0); 
    pixelTimer = millis() - pixelTimer;
    if (pixelTimer >= pixelTime) {
      drawData();
      pixelTimer = millis();
    }
    bufferTimer = millis();
  } else { // Start scroll logic after 30 s
    if (startDraw == 0) {
      tft.setCursor(265,145);
      tft.setTextSize(1);
      tft.fillCircle(280,150,20,ILI9341_RED);
      tft.setTextColor(ILI9341_WHITE);
      tft.print("Start");
      startDraw = 1;
    }
    
    currentDirection = touchPoint(); // Determine which direction to scroll
    if(currentDirection == -1 && directionChosen == 0) {
      bufferIndex -= 320;
      directionChosen = 1;
      lineCount = 0;
    } else if (currentDirection == 1 && directionChosen == 0) {
      bufferIndex += 320;
      directionChosen = 1;
      lineCount = 0;
    }
    if(bufferIndex < 0) { // If the beginning of the data is reached
      bufferIndex = 0;
    }
    if (bufferIndex > 3200) { // If the end of the data is reached
      bufferIndex = 3200 - 320;
    }
    if (directionChosen == 1) { // Scrolling logic
      bufferTimer = millis() - bufferTimer;
      if (bufferTimer >= pixelTime) {
        drawOldData(myBuffer[bufferIndex]);
        bufferTimer = millis();
        bufferIndex++;
        lineCount++;
      }
      if (lineCount == 320) { // Scroll page by page 
        directionChosen = 0;
        bufferIndex = bufferIndex - 320;
        tft.fillRect(270, 175, 60, 25, ILI9341_WHITE);
        tft.setCursor(270, 180);
        tft.setTextColor(ILI9341_BLACK);
        tft.setTextSize(1);
        tft.print("pg:");
        tft.setTextSize(2);
        tft.print(round(bufferIndex / 320));
      }    
    }
  }
}

// Determines which direction to scroll based
// on the touch input
int touchPoint() {
  leftCount = 0;
  rightCount = 0;
  if(ts.touched()) {
    TS_Point p = ts.getPoint();
    if(p.y <= 1200 && p.x != touchPrev) {
      touchData[addIndex] = p.x;
      addIndex++;
      touchPrev = p.x;
    }
    if(addIndex == 19) {
      int previous = touchData[0];
      for(int i = 1; i <= addIndex; i++) {
        if(touchData[i] > previous && touchData[i] >0) {
          leftCount++;
        } else if(touchData[i] < previous && touchData[i] >0) {
          rightCount++;
        }
        previous = touchData[i];
      }
      addIndex = 0;
    }
    if(leftCount > 15) {
      return -1;
    } else if(rightCount > 15) {
      return 1;
    }
    return 0;
  } else {
    return 0;
  }
}

// Draws the old data for scrolling
void drawOldData(int scrollY) {  
  drawGrid();
  tft.drawLine(scrollX, 0, scrollX, 120, ILI9341_WHITE); //20 to 240

  if (scrollY >= 120) { 
  } else {
    tft.drawLine(scrollxPrev + 1, scrollyPrev, scrollX + 1, scrollY, ILI9341_BLACK);
    tft.drawLine(scrollxPrev - 1, scrollyPrev, scrollX + 1, scrollY, ILI9341_BLACK);
    tft.drawLine(scrollxPrev, scrollyPrev, scrollX, scrollY, ILI9341_BLACK);
  }
  
  scrollxPrev = scrollX;
  scrollyPrev = scrollY;
  scrollX++;
  if (scrollX > 320) {
    scrollX = 0;
    scrollxPrev = 0;
  } 
}

// Draws the EKG Data
void drawData() {
  filteredSignal = exponentialAverage(0.8, filteredSignal); 
  y = map(filterloop(adcValueCopy), 0, 4095, 0, 110); 
  addToBuffer(y);

  if (filteredSignal < 1500  && filteredSignal > 200) { // QRS detection
      if (beginQRS == 1) {
        QRSTimer = millis() - QRSTimer;
        QRSVal = (int)QRSTimer;

        QRSValArray[qrsValIndex] = QRSVal;
        if (qrsValIndex == 4) {
          qrsValIndex = 0;
        } else {
          qrsValIndex++;
        }
        /*
        tft.drawLine(x, 100, x, 120, ILI9341_MAGENTA); // Indicates Q/S 
        tft.drawLine(x + 1, 100, x + 1, 120, ILI9341_MAGENTA);
        tft.drawLine(x - 1, 100, x - 1, 120, ILI9341_MAGENTA);
        */
      }
      beginQRS = 0;
      QRSTimer = millis();
  }

  if (filteredSignal > adaptiveThresh) { //R peak detection
    RRPeriod = (millis() - heartRateTimer);
    if (RRPeriod > 300) {
      beginQRS = 1;
      heartRate =  60 / (RRPeriod / 1000);
      heartRateTimer = millis();
      heartRateArray[heartRateIndex] = heartRate;
      if (heartRateIndex == 2) {
        heartRateIndex = 0;
      } else {
        heartRateIndex++;
      }
    }
    /*
    tft.drawLine(x, 100, x, 120, ILI9341_GREEN); // Indicates R peak
    tft.drawLine(x + 1, 100, x + 1, 120, ILI9341_GREEN);
    tft.drawLine(x - 1, 100, x - 1, 120, ILI9341_GREEN);
    */
  }

  drawGrid();
  tft.drawLine(x, 0, x, 120, ILI9341_WHITE); //20 to 240

  if (y >= 120) { 
  } else {
    tft.drawLine(xPrev + 1, yPrev, x + 1, y, ILI9341_BLACK);
    tft.drawLine(xPrev - 1, yPrev, x + 1, y, ILI9341_BLACK);
    tft.drawLine(xPrev, yPrev, x, y, ILI9341_BLACK);
  }
  
  xPrev = x;
  yPrev = y;
  x++;
  if (x > 320) {
    x = 0;
    xPrev = 0;
    textResults((int)movingAverage(heartRateArray, 3), (int) movingAverage(QRSValArray, 5));
    heartRateMonitorBlue();
  }
}

// Calculates the average of an array
int movingAverage(double valArray[], int n){
  int sum = 0;
  for (int i=0; i< n; i++)
  {
    sum = sum + valArray[i];
  }
  return sum / n; 
}

// Prints the results to the screen
void textResults(int hr, int qrs) {
  tft.fillRect(30, 150, 40, 20, ILI9341_BLACK);
  tft.fillRect(80,150,40,20,ILI9341_BLACK);
  tft.fillRect(130,150,40,20,ILI9341_BLACK);
  tft.fillRect(180,150,40,20,ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(31, 151);

  // draws QRS data
  if (qrs > 200 || qrs <= 0) {
    tft.print("---");
  } else if (qrs < 100) {
    tft.print(" ");
    tft.print(qrs);
  } else {
    tft.print(qrs);
  }
  
  // draws heart rate data
  tft.setCursor(81,151);
  if (hr > 200 || hr <= 0) {
    tft.print("---");
  } else {
    tft.print(hr);
  }
  tft.setCursor(131,151);

  // draws tachycardia/bradycardia data
  if (hr < 60 && hr > 0) {
    tft.setTextColor(ILI9341_RED);
    tft.print("YES");
  } else {
    tft.setTextColor(ILI9341_GREEN);
    tft.print("NO");
  }
  tft.setCursor(181,151);
  if (hr > 110) {
    tft.setTextColor(ILI9341_RED);
    tft.print("YES");
  } else {
    tft.setTextColor(ILI9341_GREEN);
    tft.print("NO");
  }
  
  // draws the stop button
  tft.setCursor(265,145);
  tft.setTextSize(1);
  tft.fillCircle(280,150,20,ILI9341_RED);
  tft.setTextColor(ILI9341_WHITE);
  if (startCube == 1) {
    tft.print("Stop");
  } 
}

// Draws a grid on the top half of the screen
void drawGrid() {
  for (int i = 0; i < 40; i++) {
    tft.drawLine(i * 8, 0, i * 8, 120, ILI9341_RED); 
  }
  for (int i = 0; i < 21; i++) { // 2 to 20
    tft.drawLine(0, i * 6, 320, i * 6, ILI9341_RED);
  }
}

// Adds values to a buffer
void addToBuffer(int val) {
    myBuffer[bufferIndex] = val;
    bufferIndex++;
    if (bufferIndex > 3600) {
     bufferIndex = 0;
    }
}

/* ADC triggered by the PDB
*/
#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

static const uint8_t channel2sc1a[] = {
  5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
  0, 19, 3, 21, 26, 22
};

/*
  ADC_CFG1_ADIV(2)         Divide ratio = 4 (F_BUS = 48 MHz => ADCK = 12 MHz)
  ADC_CFG1_MODE(2)         Single ended 10 bit mode
  ADC_CFG1_ADLSMP          Long sample time
*/
#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP)

/*
  ADC_CFG2_MUXSEL          Select channels ADxxb
  ADC_CFG2_ADLSTS(3)       Shortest long sample time
*/
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))

void adcInit() {
  ADC0_CFG1 = ADC_CONFIG1;
  ADC0_CFG2 = ADC_CONFIG2;
  // Voltage ref vcc, hardware trigger, DMA
  ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

  // Enable averaging, 4 samples
  ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);

  adcCalibrate();

  // Enable ADC interrupt, configure pin
  ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[0];
  NVIC_ENABLE_IRQ(IRQ_ADC0);

}

void adcCalibrate() {
  uint16_t sum;

  // Begin calibration
  ADC0_SC3 = ADC_SC3_CAL;
  // Wait for calibration
  while (ADC0_SC3 & ADC_SC3_CAL);

  // Plus side gain
  sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
  sum = (sum / 2) | 0x8000;
  ADC0_PG = sum;

  // Minus side gain (not used in single-ended mode)
  sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
  sum = (sum / 2) | 0x8000;
  ADC0_MG = sum;
}

/*
  PDB_SC_TRGSEL(15)        Select software trigger
  PDB_SC_PDBEN             PDB enable
  PDB_SC_PDBIE             Interrupt enable
  PDB_SC_CONT              Continuous mode
  PDB_SC_PRESCALER(7)      Prescaler = 128
  PDB_SC_MULT(1)           Prescaler multiplication factor = 10
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
  | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))

// 48 MHz / 128 / 10 / 1 Hz = 37500
#define PDB_PERIOD (F_BUS / 128 / 10 / 100)

void pdbInit() {
  // Enable PDB clock
  SIM_SCGC6 |= SIM_SCGC6_PDB;
  // Timer period
  PDB0_MOD = PDB_PERIOD;
  // Interrupt delay
  PDB0_IDLY = 0;
  // Enable pre-trigger
  PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
  // PDB0_CH0DLY0 = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  // Software trigger (reset and restart counter)
  PDB0_SC |= PDB_SC_SWTRIG;
  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_PDB);
}

void adc0_isr() {
  adcValue = ADC0_RA;
}

void pdb_isr() {
  // Clear interrupt flag
  PDB0_SC &= ~PDB_SC_PDBIF;
}


// Filters utilized

// Low pass filter 60hz
#define NZEROS 4
#define NPOLES 4
#define GAIN   1.209590303e+01

static float xv[NZEROS+1], yv[NPOLES+1];

double filterloop(int input)
  { for (;;)
      { xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; 
        xv[4] = input / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; 
        yv[4] =   (xv[0] + xv[4]) + 4 * (xv[1] + xv[3]) + 6 * xv[2]
                     + ( -0.0181370770 * yv[0]) + (  0.0329072181 * yv[1])
                     + ( -0.4937426526 * yv[2]) + (  0.1562105841 * yv[3]);
        return yv[4];
      }
  }

//Band pass filter 3-18hz
#define NZEROS_BPF 4
#define NPOLES_BPF 4
#define GAIN_BPF   3.562818274e+01

static float xv_BPF[NZEROS+1], yv_BPF[NPOLES+1];

double filterloopBPF()
  { for (;;)
      { xv_BPF[0] = xv_BPF[1]; xv_BPF[1] = xv_BPF[2]; xv_BPF[2] = xv_BPF[3]; xv_BPF[3] = xv_BPF[4]; 
        xv_BPF[4] = (double)adcValueCopy / GAIN_BPF;
        yv_BPF[0] = yv_BPF[1]; yv_BPF[1] = yv_BPF[2]; yv_BPF[2] = yv_BPF[3]; yv_BPF[3] = yv_BPF[4]; 
        yv_BPF[4] =   (xv_BPF[0] + xv_BPF[4]) - 2 * xv_BPF[2]
                     + ( -0.5869195081 * yv_BPF[0]) + (  2.6037234627 * yv_BPF[1])
                     + ( -4.4333768759 * yv_BPF[2]) + (  3.4156658390 * yv_BPF[3]);
        return yv_BPF[4];
      }
  }

//Band pass filter for calibration
#define NZEROS_BPFCal 4
#define NPOLES_BPFCal 4
#define GAIN_BPFCal   3.562818274e+01

static float xv_BPFCal[NZEROS+1], yv_BPFCal[NPOLES+1];

double filterloopBPFCal(int input)
  { for (;;)
      { xv_BPFCal[0] = xv_BPFCal[1]; xv_BPFCal[1] = xv_BPFCal[2]; xv_BPFCal[2] = xv_BPFCal[3]; xv_BPFCal[3] = xv_BPFCal[4]; 
        xv_BPFCal[4] = input / GAIN_BPFCal;
        yv_BPFCal[0] = yv_BPFCal[1]; yv_BPFCal[1] = yv_BPFCal[2]; yv_BPFCal[2] = yv_BPFCal[3]; yv_BPFCal[3] = yv_BPFCal[4]; 
        yv_BPFCal[4] =   (xv_BPFCal[0] + xv_BPFCal[4]) - 2 * xv_BPFCal[2]
                     + ( -0.5869195081 * yv_BPFCal[0]) + (  2.6037234627 * yv_BPFCal[1])
                     + ( -4.4333768759 * yv_BPFCal[2]) + (  3.4156658390 * yv_BPFCal[3]);
        return yv_BPFCal[4];
      }
  }

// Differentiator filter
double in_differentiator[4] = {0, 0, 0, 0};
double out_differentiator[4] = {0, 0, 0, 0}; 
double differentiator() {
  float num[3 + 1] = {2, 1, -1, -2};
  float denom[3 + 1] = {1, 0, 0, 0};
  
  in_differentiator[0] = (double) filterloopBPF(); 
  out_differentiator[0] = num[0]*in_differentiator[0] +  num[1]*in_differentiator[1] +  num[2]*in_differentiator[2]  +  num[3]*in_differentiator[3] -  denom[1]*out_differentiator[1] - denom[2]*out_differentiator[2] - denom[3]*out_differentiator[3]; 
  for (int i = 0; i < 4 - 1; i++) {
    in_differentiator[i+1] = in_differentiator[i]; 
  }

  for (int i = 0; i < 4 - 1; i++) {
    out_differentiator[i+1] = out_differentiator[i];
  }
  return  out_differentiator[0];
}

// Squaring filter
double squaring() {
  return (pow(differentiator(),2));
}

// Moving average filter
double exponentialAverage(float avgFactor, double prevOutput){
  double sqr = squaring();
  return squaring()*avgFactor + (1-avgFactor)*prevOutput;  
}

// Differentiator filter for calibration
double in_differentiatorCal[4] = {0, 0, 0, 0};
double out_differentiatorCal[4] = {0, 0, 0, 0}; 
double differentiatorCal(int input) {
  float numCal[3 + 1] = {2, 1, -1, -2};
  float denomCal[3 + 1] = {1, 0, 0, 0};
  
  in_differentiatorCal[0] = (double) filterloopBPFCal(input); 
  out_differentiatorCal[0] = numCal[0]*in_differentiatorCal[0] +  numCal[1]*in_differentiatorCal[1] +  numCal[2]*in_differentiatorCal[2]  +  numCal[3]*in_differentiatorCal[3] -  denomCal[1]*out_differentiatorCal[1] - denomCal[2]*out_differentiatorCal[2] - denomCal[3]*out_differentiatorCal[3]; 
  for (int i = 0; i < 4 - 1; i++) {
    in_differentiatorCal[i+1] = in_differentiatorCal[i]; 
  }

  for (int i = 0; i < 4 - 1; i++) {
    out_differentiatorCal[i+1] = out_differentiatorCal[i];
  }
  return  out_differentiatorCal[0];
}

// Squaring calibration filter 
double squaringCal(int input) {
  return (pow(differentiatorCal(input),2));
}

// Moving average calibration filter
double exponentialAverageCal(float avgFactor, double prevOutput, int input) {
  double sqr = squaringCal(input);
  return squaringCal(input)*avgFactor + (1-avgFactor)*prevOutput;  
}

// Bluetooth Code

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  while (1);
}

/* The service information */
int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

// Sets up the Bluetooth connection to the device
void heartRateMonitorBlueInit() {
  tft.begin();
  tft.setRotation(3);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.println("Please Connect to the" );
  tft.println("ECG (ZS ECG) through" );
  tft.println("the nRF Toolbox App" );
  
  boolean success;

  Serial.begin(115200);

  /* Initialise the module */
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }
  
  /* Disable command echo from Bluefruit */
  ble.echo(false);

  /* Print Bluefruit information */
  ble.info();

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME= ZS ECG")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);
  if (! success) {
    error(F("Could not add HRM service"));
  }

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
    if (! success) {
    error(F("Could not add HRM characteristic"));
  }

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);
    if (! success) {
    error(F("Could not add BSL characteristic"));
  }

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  ble.reset();
}

// Sends the BPM data to the phone application
void heartRateMonitorBlue() {
  int heart_rate = (int)movingAverage(heartRateArray, 3);

  ble.print( F("AT+GATTCHAR=") );
  ble.print( hrmMeasureCharId );
  ble.print( F(",00-") );
  ble.println(heart_rate, HEX);
}

 
