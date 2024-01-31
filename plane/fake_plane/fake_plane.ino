#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define OLED display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Initialize OLED display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the display
  display.clearDisplay();
  display.display();
}

void loop() {
  // Read GPS coordinates
  float timeSec = millis() / (float)1000;

  Serial.print("{\"type\": \"gps\", \"altitude\": ");
  Serial.print(20.55 + sin(timeSec), 4);
  Serial.print(", \"latitude\": 50.67, \"longitude\": ");
  Serial.print(-120.325 + 0.005 * cos(timeSec + 0.5), 8);
  Serial.print(", \"bearing\": 10}\n");

  // Check for incoming data on serial
  checkSerial();

  // Delay for stability (adjust as needed)
  delay(50);

}

void checkSerial() {
  // Check if there is data available to read
  if (Serial.available() > 0) {
    // Read the incoming data
    String incomingData = Serial.readString();

    // Display the incoming data on OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Received:");
    display.println(incomingData);
    display.display();
  }
}
