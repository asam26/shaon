#include <Arduino.h>
#include <Watchy.h>

// Pin definitions for Watchy v3 (from config.h)
#define DISPLAY_CS   33
#define DISPLAY_DC   34
#define DISPLAY_RES  35
#define DISPLAY_BUSY 36
#define EPD_MOSI     48
#define EPD_SCK      47

// GxEPD2 display instance using the correct v3 display (GDEY0154D67)
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(
  GxEPD2_154_D67(DISPLAY_CS, DISPLAY_DC, DISPLAY_RES, DISPLAY_BUSY)
);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("SHAON booting...");

  // Initialize SPI with Watchy v3 pins
  SPI.begin(EPD_SCK, -1, EPD_MOSI, DISPLAY_CS);

  // Initialize display
  display.init(115200, true, 2, false);
  display.setRotation(0);
  display.setFullWindow();

  // Draw "Hello" screen
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setFont();        // built-in font for now
    display.setTextSize(2);
    display.setCursor(52, 88);
    display.print("SHAON");
    display.setTextSize(1);
    display.setCursor(44, 112);
    display.print("Hello, World!");
  } while (display.nextPage());

  Serial.println("Display updated.");
}

void loop() {
  // nothing — e-ink holds its image with no power
  delay(60000);
}