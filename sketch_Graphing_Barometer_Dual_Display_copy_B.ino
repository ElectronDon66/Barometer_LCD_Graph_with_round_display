// Converted sketch: GC9A01 (Arduino_GFX) + ILI9341_t3n (plot)
// Only the ILI display was converted to ILI9341_t3n (Option A)
// Needed to load mjs513 fonts to get display looking good
//Sketch is working , looks good
// DEW 12/12/25 Release 


#include <Arduino_GFX_Library.h>   // for GC9A01 round display
#include <Adafruit_BMP3XX.h>
#include <SPI.h>
#include <elapsedMillis.h>
#include <ILI9341_t3n.h>          // NEW: ILI display library
#include <font_Arial.h>           // custom fonts from github.com/mjs513 
#include "font_ArialBold.h"

// ---------------------------
// PIN DEFINITIONS
// ---------------------------
#define BMP_CS       8
#define TFT_CS       7
#define TFT_DC       9
#define TFT_RST      4
#define ANALOG_OFFSET_PIN A0

// ILI (rectangular plot)
#define ILI_CS    10  
#define ILI_DC     6
#define ILI_RST    5

// Colors (RGB565)
#define C_WHITE 0xFFFF
#define C_BLACK 0x0000
#define C_RED   0xF800
#define C_LTGREY 0xE71C
#define C_LTYELLOW 0xF7EE
#define C_BLUE  0x001F

// ---------------------------
// GLOBAL OBJECTS
// ---------------------------
// BMP390 + GC9A01 remain unchanged
Adafruit_BMP3XX bmp;   // SPI BMP390
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST, 0, true);
Arduino_Canvas_Indexed *canvas = new Arduino_Canvas_Indexed(240, 240, gfx);

// NEW: create ILI9341_t3n object (uses same hardware SPI bus)
ILI9341_t3n tft_ili = ILI9341_t3n(ILI_CS, ILI_DC, ILI_RST);

// ---------------------------
// TIMERS
// ---------------------------
elapsedMillis t_1sec = 0;
elapsedMillis t_0p1hr = 0;
elapsedMillis t_1hr = 0;

// ---------------------------
// GAUGE SETTINGS
// ---------------------------
float minInHg = 28.0;
float maxInHg = 32.0;
float lastAngle = -140;

// ---------- Plot buffer & settings -------
const int PLOT_POINTS = 100;             // 100 points over 10 hours (0.1 hr spacing)
float pressureBuffer[PLOT_POINTS];
int bufferIndex = 0;
bool bufferFilled = false;

// Plot area margins (on ILI 320x240)
const int PLOT_W = 300;
const int PLOT_H = 200;
const int PLOT_X = 10;    // left margin
const int PLOT_Y = 20;    // top margin

const float YSPAN = 0.25f; // +/-0.25 inHg

// Forward declarations
void initPlotBuffer(float initVal);
void samplePressureToBuffer();
float bufferAverage();
void drawPressurePlot();

// ---------------------------
// UTILITY FUNCTIONS
// ---------------------------
float pressureToAngle(float p) {
  float ratio = (p - minInHg) / (maxInHg - minInHg);
  ratio = constrain(ratio, 0, 1);
  return -140 + ratio * 280 - 90; // match dial rotation
}

float readPressureInHg() {
  if (!bmp.performReading()) return -1;
  float inHg = (bmp.pressure / 100.0) * 0.02953;

  int raw = analogRead(ANALOG_OFFSET_PIN);
  float offset = map(raw, 0, 4095, -1000, 1000) / 1000.0;

  return inHg + offset;
}

// ---------------------------
// GAUGE DRAWING (unchanged)
// ---------------------------
void drawGaugeFace() {
  canvas->fillScreen(C_WHITE);
  int cx = 120, cy = 120;
  int r = 110;

  // --- Draw shadow on the outer perimeter ---
  for (int s = 0; s < 5; s++) {            // 5 layers of shadow
    int shadowRadius = r + s;               // slightly larger than main radius
    canvas->drawCircle(cx, cy, shadowRadius, C_LTGREY);
  }

  float majorStep = 0.5;   // Major ticks every 0.5 inHg
  float minorStep = 0.1;   // Minor ticks every 0.1 inHg
  float tol = 0.01;        // Tolerance for floating-point comparison

  // Draw all ticks
  for (float value = minInHg; value <= maxInHg + 0.01; value += minorStep) {
    float angle = -140 + ((value - minInHg) / (maxInHg - minInHg)) * 280 - 90;
    float rad = angle * 0.0174533;

    // Determine if major or minor tick
    bool isMajor = (fmod(value, majorStep) < tol) || (majorStep - fmod(value, majorStep) < tol);
    int tickLength = isMajor ? 15 : 8;       // Major ticks longer

    int x1 = cx + cos(rad)*(r - tickLength);
    int y1 = cy + sin(rad)*(r - tickLength);
    int x2 = cx + cos(rad)*r;
    int y2 = cy + sin(rad)*r;

    canvas->drawLine(x1, y1, x2, y2, C_BLACK);

    // Draw number for major ticks
    if (isMajor) {
      char buf[10]; dtostrf(value, 4, 1, buf);

      // Offset numbers slightly beyond major tick
      int xt = cx + cos(rad)*(r - 30);  // 10 pixels beyond the tick line
      int yt = cy + sin(rad)*(r - 30);

      canvas->setCursor(xt - 12, yt - 6);
      canvas->setTextColor(C_BLACK);
      canvas->setTextSize(2);
      canvas->print(buf);
    }
  }

  canvas->fillCircle(cx, cy, 5, C_BLACK);
}

void drawNeedle(float angleDeg)
{
  int cx = 120, cy = 120;
  int length = 90;

  float rad = angleDeg * 0.0174533;

  // Needle tip
  int tipX = cx + cos(rad) * length;
  int tipY = cy + sin(rad) * length;

  // Needle base width
  int baseWidth = 6;

  // Calculate perpendicular for the base
  float perpRad = rad + 1.5708; // 90 degrees in radians
  int x1 = cx + cos(perpRad) * baseWidth;
  int y1 = cy + sin(perpRad) * baseWidth;
  int x2 = cx - cos(perpRad) * baseWidth;
  int y2 = cy - sin(perpRad) * baseWidth;

  // Draw filled triangle (needle)
  canvas->fillTriangle(x1, y1, x2, y2, tipX, tipY, C_RED);

  // Optional: draw thin center highlight
  int hx = cx + cos(rad) * length;
  int hy = cy + sin(rad) * length;
  canvas->drawLine(cx, cy, hx, hy, C_LTYELLOW); // subtle highlight

  // Draw circular hub over base
  canvas->fillCircle(cx, cy, 5, C_BLACK);
}

void drawDigitalReadout(float p) {
  char buf[16]; dtostrf(p, 6, 2, buf);
  canvas->fillRect(60, 140, 120, 30, C_WHITE);
  canvas->setTextColor(C_BLACK);
  canvas->setTextSize(3);
  canvas->setCursor(65, 145);
  canvas->print(buf);
}

void updateNeedleSmooth(float newAngle, float pressureInHg) {
  int steps = 12;
  float step = (newAngle - lastAngle)/steps;

  for (int i = 1; i <= steps; i++) {
    float a = lastAngle + step*i;
    drawGaugeFace();
    drawNeedle(a);
    drawDigitalReadout(pressureInHg);
    canvas->flush();
    delay(10); // small delay, less blocking
  }
  lastAngle = newAngle;
}

// ---------------------------
// PLOT BUFFER FUNCTIONS
// ---------------------------
void initPlotBuffer(float initVal) {
  for (int i = 0; i < PLOT_POINTS; ++i) pressureBuffer[i] = initVal;
  bufferIndex = 0;
  bufferFilled = true;
}

// Called every 0.1 hr (6 minutes) by timer B (t_0p1hr)
void samplePressureToBuffer() {
  float p = readPressureInHg();
  if (p <= 0) return; // ignore bad reads

  pressureBuffer[bufferIndex] = p;
  bufferIndex++;
  if (bufferIndex >= PLOT_POINTS) {
    bufferIndex = 0;
    bufferFilled = true;
  }
}

// Compute average of buffer
float bufferAverage() {
  float sum = 0;
  for (int i = 0; i < PLOT_POINTS; ++i) sum += pressureBuffer[i];
  return sum / (float)PLOT_POINTS;
}

// Map a relative pressure (p - avg) to pixel Y (centered)
int mapRelativeToY(float relative) {
  // relative in [-YSPAN, +YSPAN]
  if (relative > YSPAN) relative = YSPAN;
  if (relative < -YSPAN) relative = -YSPAN;
  // centerY at middle of plot area
  int centerY = PLOT_Y + PLOT_H / 2;
  // map +YSPAN -> PLOT_Y (top), -YSPAN -> PLOT_Y+PLOT_H (bottom)
  float frac = (relative + YSPAN) / (2.0f * YSPAN); // 0..1
  int y = PLOT_Y + (int)((1.0f - frac) * PLOT_H + 0.5f);
  return y;
}

// Draw axes, ticks, and the plot line on the ILI display (using ILI9341_t3n)
void drawPressurePlot() {
  // Use direct tft_ili drawing (RGB565 colors)
  tft_ili.fillScreen(C_LTGREY);

  // Draw plot border and background
  tft_ili.drawRect(PLOT_X - 1, PLOT_Y - 1, PLOT_W + 2, PLOT_H + 2, C_BLACK);
  tft_ili.fillRect(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, C_WHITE);

  // X spacing and center line
  float xSpacing = (float)PLOT_W / (PLOT_POINTS - 1);
  int centerY = PLOT_Y + PLOT_H / 2;
  tft_ili.drawLine(PLOT_X, centerY, PLOT_X + PLOT_W, centerY, C_LTGREY);

  // X-axis ticks and labels (-10 -> 0 hours)
  
  tft_ili.setFont(Arial_9_Bold);
  tft_ili.setTextSize(1);           // fractional size (if supported)
 
  tft_ili.setTextColor(C_BLACK);
  
  for (int hr = -10; hr <= 0; hr++) {
    int sampleIndex = (hr + 10) * 10;  // hr=-10 => 0, hr=0 => 100 -> clamp
    if (sampleIndex < 0) sampleIndex = 0;
    if (sampleIndex > PLOT_POINTS - 1) sampleIndex = PLOT_POINTS - 1;
    float xf = PLOT_X + sampleIndex * xSpacing;
    int xi = (int)(xf + 0.5f);

    // tick
    tft_ili.drawLine(xi, PLOT_Y + PLOT_H, xi, PLOT_Y + PLOT_H - 8, C_BLACK);

    // label (centered approximation)
    char lbl[8];
    itoa(hr, lbl, 10);
    // approximate centering; adjust if needed
    tft_ili.setCursor(xi - 8, PLOT_Y + PLOT_H + 2);
    tft_ili.print(lbl);
  }

  // Y axis ticks and labels at +0.25, +0.125, 0, -0.125, -0.25
  
  tft_ili.setFont(Arial_9);
  tft_ili.setTextSize(1);
  float ymarks[] = { YSPAN, YSPAN/2.0f, 0.0f, -YSPAN/2.0f, -YSPAN };
  for (int i = 0; i < 5; ++i) {
    int yy = mapRelativeToY(ymarks[i]);
    tft_ili.drawLine(PLOT_X - 6, yy, PLOT_X, yy, C_BLACK);

    char buf[12];
    dtostrf(ymarks[i], 5, 3, buf);
    tft_ili.setCursor(2, yy - 6);
    tft_ili.print(buf);
  }

  // Draw polyline of relative pressure = sample - avg
  float avg = bufferAverage();
  int idx = bufferIndex; // oldest sample
  int prevX = PLOT_X;
  int prevY = mapRelativeToY(pressureBuffer[idx] - avg);

  for (int i = 0; i < PLOT_POINTS; ++i) {
    float p = pressureBuffer[idx];
    float rel = p - avg;
    float xf = PLOT_X + i * xSpacing;
    int xi = (int)(xf + 0.5f);
    int yi = mapRelativeToY(rel);

    if (i > 0) {
      tft_ili.drawLine(prevX, prevY, xi, yi, C_RED); // trace color
    }
    prevX = xi; prevY = yi;

    idx++;
    if (idx >= PLOT_POINTS) idx = 0;
  }

  // Title
  tft_ili.setCursor(PLOT_X, 2);

  tft_ili.setFont(Arial_11_Bold);
  tft_ili.setTextSize(1);
  tft_ili.setTextColor(C_BLACK);
  tft_ili.print("                      Pressure  (in/Hg)  ");
}

// ---------------------------
// SETUP
// ---------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("Setup start");
  analogReadResolution(12);

  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, LOW);
  delay(50);
  digitalWrite(TFT_RST, HIGH);

  // Reset the ILI
  pinMode(ILI_RST, OUTPUT);
  digitalWrite(ILI_RST, LOW); delay(50); digitalWrite(ILI_RST, HIGH);

  // initialize GC9A01 (main round gauge)
  gfx->begin();
  canvas->begin();

  // initialize ILI9341_t3n (rectangular plot)
  tft_ili.begin();
  tft_ili.setRotation(1);
  tft_ili.invertDisplay(false); // fix for your panel variant

  Serial.println("Display initialized");

  // BMP init AFTER displays (prevents blocking issues)
  if (!bmp.begin_SPI(BMP_CS)) {
    Serial.println("BMP390 not found!");
    while(1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  drawGaugeFace();
  drawNeedle(lastAngle);
  drawDigitalReadout(minInHg);
  canvas->flush();

  Serial.println("Setup complete");
 float initial = readPressureInHg();
 int ti =0;
 while (ti< 4) {
  initial = readPressureInHg(); ti++;  // throw away first 4 readings for IIR filter 
 }

  
    

  // initialize and prefill buffer with a current reading
   initial = readPressureInHg();
  if (initial <= 0) initial = minInHg; // fallback
  initPlotBuffer(initial);
  
  
  // Print initial reading for trouble shooting 
  Serial.print("Initial reading");
  Serial.println(initial);
  
  drawPressurePlot();
}

// ---------------------------
// MAIN LOOP
// ---------------------------
void loop() {
  if (t_1sec >= 1000) {
    t_1sec = 0;
    float p = readPressureInHg();
    if (p > 0) {
      float angle = pressureToAngle(p);
      updateNeedleSmooth(angle, p);
      Serial.print("Pressure: "); Serial.println(p);
    }
  }

  // 0.1 hr timer -> sampling (6 minutes)
  if (t_0p1hr >= 360000) {
    t_0p1hr = 0;
    samplePressureToBuffer();
  }

  // 1 hr timer -> update plot display
  if (t_1hr >= 3600000) {
    t_1hr = 0;
    drawPressurePlot();
  }
}
