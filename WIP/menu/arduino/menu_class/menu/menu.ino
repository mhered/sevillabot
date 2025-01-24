// 128x64 OLED Display menu controlled by a KY-40 encoder
// mhered 2024
// Vaguely inspired in three different tutorials by: 
// - upir (Menu design): https://youtu.be/HVHVkKt-ldc 
// - LaBuhardillaDelLoco (Adafruit GPX library for OLED): https://youtu.be/cWgWvNhWg-A 
// - Yvan  (Encoder input): https://www.brainy-bits.com/post/best-code-to-use-with-a-ky-040-rotary-encoder-let-s-find-out// see https://wokwi.com/projects/397015999090347009

#include "src/Encoder.h" // custom Encoder library

#include <Adafruit_GFX.h> // graphical library
#include <Adafruit_SSD1306.h> // SSD1306 controller library

// Custom fonts
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

// OLED display constants
#define SCREEN_ADDRESS 0x3C // alternatively try 0x3D
#define SCREEN_WIDTH  128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # if screen has 5th wire (or -1 otherwise)

// Declaration for an Encoder
// define pins
#define CLK_PIN 2 
#define DT_PIN  4
#define SW_PIN  3

// create instance of Encoder
Encoder menuEncoder(CLK_PIN, DT_PIN, SW_PIN); // Create an Encoder object for pins provided

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// For arduino UNO or NANO:       A4(SDA), A5(SCL)

// create instance of OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH,
                         SCREEN_HEIGHT,
                         &Wire,
                         OLED_RESET); // &Wire is pointer to I2C comms




// definition to avoid magic numbers
const int MENU_SCREEN = 0;
const int OPTION_SCREEN = 1;

const int NUM_ITEMS = 6; // number of items in the list
const int MAX_ITEM_LENGTH = 20; // maximum characters for the item name

char menu_items [NUM_ITEMS] [MAX_ITEM_LENGTH] = {  // array with item names
  { "1.Toxic" },
  { "2.Zombies" },
  { "3.Tracker" },
  { "4.Mines" },
  { "5.Navigator" },
  { "6.Maze" }
};

int item_selected = 0; // which item in the menu is selected

int current_screen = MENU_SCREEN;

// for 50Hz refresh rate -> refreshDelay = 20
long lastRefreshTime = 0;
int refreshDelay = 200;


// arrays below were generated from B/W images using image2cpp online tool
// ------------------ generated bitmaps from image2cpp ---------------------------------

// 'icon01', 14x14px
const unsigned char bitmap_icon01 [] PROGMEM = {
  0x40, 0x08, 0xc0, 0x1c, 0x60, 0x30, 0x37, 0xa0, 0x1f, 0xc0, 0x27, 0x20, 0x62, 0x30, 0x72, 0x70,
  0x3f, 0xe0, 0x0a, 0x80, 0x2f, 0xa0, 0x69, 0xd8, 0xcc, 0xcc, 0x47, 0x88
};
// 'icon02', 14x14px
const unsigned char bitmap_icon02 [] PROGMEM = {
  0x02, 0x00, 0x0f, 0x80, 0x12, 0x40, 0x22, 0x20, 0x40, 0x10, 0x40, 0x10, 0xf2, 0x78, 0x40, 0x10,
  0x40, 0x10, 0x22, 0x20, 0x12, 0x40, 0x0f, 0x80, 0x02, 0x00, 0x00, 0x00
};
// 'icon03', 14x14px
const unsigned char bitmap_icon03 [] PROGMEM = {
  0x00, 0x38, 0x00, 0x7c, 0x00, 0x6c, 0x00, 0x38, 0x00, 0x38, 0x00, 0x10, 0x06, 0xc0, 0x18, 0x00,
  0x60, 0x00, 0x3c, 0x00, 0x07, 0xc0, 0x00, 0xf8, 0x00, 0x3c, 0x00, 0x7c
};
// 'icon04', 14x14px
const unsigned char bitmap_icon04 [] PROGMEM = {
  0x01, 0x00, 0x01, 0x00, 0x21, 0x08, 0x13, 0x90, 0x0f, 0xe0, 0x09, 0xe0, 0x11, 0xf0, 0xf3, 0xfc,
  0x1f, 0xf0, 0x0f, 0xe0, 0x0f, 0xe0, 0x13, 0x90, 0x21, 0x08, 0x01, 0x00
};
// 'icon05', 14x14px
const unsigned char bitmap_icon05 [] PROGMEM = {
  0x0f, 0x80, 0x10, 0x40, 0x22, 0x20, 0x42, 0x10, 0x80, 0x88, 0x83, 0x0c, 0xb5, 0x6c, 0x86, 0x0c,
  0x88, 0x0c, 0x42, 0x1c, 0x22, 0x38, 0x10, 0x70, 0x0f, 0xe0, 0x07, 0xc0
};
// 'icon06', 14x14px
const unsigned char bitmap_icon06 [] PROGMEM = {
  0xff, 0xfc, 0xb0, 0x44, 0x87, 0x10, 0xfd, 0x7c, 0x81, 0x04, 0xbf, 0x7c, 0x80, 0x04, 0xf7, 0xf4,
  0x80, 0x04, 0xbf, 0xfc, 0x80, 0x04, 0xfb, 0xf4, 0x82, 0x04, 0xfe, 0xfc
};

// Array of all bitmaps for convenience.
const unsigned char* bitmap_icons[NUM_ITEMS] = {
  bitmap_icon01,
  bitmap_icon02,
  bitmap_icon03,
  bitmap_icon04,
  bitmap_icon05,
  bitmap_icon06
};

// 'scrollbar_background', 8x48px
const unsigned char bitmap_scrollbar_background [] PROGMEM = {
  0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 
  0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 
  0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10
};

// ------------------ end generated bitmaps from image2cpp ---------------------------------



void setup() {
  // initialize serial port at 9600 bauds
  Serial.begin(9600);

  // initialize screen
  // SSD1306_SWITCHCAPVCC to generate internally 3.3V to power the display 
  // or SSD1306_EXTERNALVCC to power it externally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Loop forever
  };



}
void loop() {
    int command = menuEncoder.getCommand();
  if (current_screen == MENU_SCREEN) { // MENU SCREEN

    // check encoder input for the menu screen

    switch (command) {
      case Encoder::CLICK:
        current_screen = 1; // menu items screen --> screenshots screen
        break;
      case Encoder::CLOCKWISE:
        item_selected = increment(item_selected, NUM_ITEMS);
         break;
      case Encoder::ANTICLOCKWISE:
        item_selected = decrement(item_selected, NUM_ITEMS);
        break;
      case Encoder::NONE:
        // Optionally do something when no command is detected
        break;
    }

    // refresh Menu screen
    refresh_menu();
  }
  else if (current_screen == OPTION_SCREEN) { // SELECTION SCREEN
    refresh_screen(item_selected);

    if (command == Encoder::CLICK)
      current_screen = MENU_SCREEN; // screenshots screen --> menu items screen 
  }
};

int increment(int item, int NUM_ITEMS) {
  int result = item + 1;
  if (result > NUM_ITEMS - 1) {
    result = 0;
  }
  return result;
}

int decrement(int item, int NUM_ITEMS) {
  int result = item - 1;
  if (result < 0 ) {
    result = NUM_ITEMS - 1;
  }
  return result;
}

void refresh_screen(int selection) {

  // refresh screen only at regular intervals
  if ((millis() - lastRefreshTime) > refreshDelay) {

    display.clearDisplay(); // clear screen

    display.setTextSize(2); // 1 - 7... (1: 4x8pix, 2: 8x16pix, ...)
    display.setFont();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20); // top left origin in pixels
    display.println(menu_items[item_selected]);

    display.display(); //refresh
    lastRefreshTime = millis();
  }
};

void refresh_menu() {

  // refresh screen only at regular intervals
  if ((millis() - lastRefreshTime) > refreshDelay) {

    // get previous and next items
    int item_sel_previous = decrement(item_selected, NUM_ITEMS);
    int item_sel_next = increment(item_selected, NUM_ITEMS);

    display.clearDisplay(); // clear screen

    display.fillRect(0, 0, 128, 16, SSD1306_WHITE);

    display.setTextSize(1); // 1 - 7... (1: 4x8pix, 2: 8x16pix, ...)
    display.setFont(&FreeSansBold9pt7b); //
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setCursor(21, 14); // for Custom fonts bottom left origin in pixels
    display.println("sevillabot");

    display.setTextColor(SSD1306_WHITE);
    display.setFont();

    display.setCursor(20, 19); // bottom left origin in pixels
    display.println(menu_items[item_sel_previous]);

    display.setCursor(20, 56); // bottom left origin in pixels
    display.println(menu_items[item_sel_next]);

    display.setFont(&FreeSans9pt7b);
    display.setCursor(20, 46); // bottom left origin in pixels
    display.println(menu_items[item_selected]);

    // draw icons
    display.drawBitmap(4, 16, bitmap_icons[item_sel_previous], 14, 14, SSD1306_WHITE);
    display.drawBitmap(4, 34, bitmap_icons[item_selected], 14, 14, SSD1306_WHITE);
    display.drawBitmap(4, 52, bitmap_icons[item_sel_next], 14, 14, SSD1306_WHITE);

    // draw scrollbar background
    display.drawBitmap(120, 16, bitmap_scrollbar_background, 8, 48, SSD1306_WHITE);

    // draw scrollbar handle
    display.fillRect(122, 15+48/NUM_ITEMS * item_selected, 5, 48/NUM_ITEMS, SSD1306_WHITE); 

    // draw selection box
    display.drawRoundRect(0, 31, 120, 20, 4, SSD1306_WHITE);

    display.display(); //refresh
    lastRefreshTime = millis();
  }
};
