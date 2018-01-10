#define pin_oled_sclk 13
#define pin_oled_mosi 11
#define pin_oled_cs   10
#define pin_oled_rst  9
#define pin_oled_dc   8


#include <SerialCommands.h> // see https://github.com/ppedro74/Arduino-SerialCommands



#include <Adafruit_GFX.h>
#include "Adafruit_SSD1331.h"
#include <SPI.h>

// Color definitions
#define BLACK           0x0000
//                        aaaaabbbbbbccccc
#define GRAY            0b0010000100000100
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

const int display_width = 96;
const int display_height = 64;




Adafruit_SSD1331 display = Adafruit_SSD1331(pin_oled_cs, pin_oled_dc, pin_oled_rst);

float p = 3.1415926;
const int t_delay=0;


// buffer must be big enough for command, arguments, terminiators and null
char serial_command_buffer_[100];
SerialCommands serial_commands(&Serial1, serial_command_buffer_, sizeof(serial_command_buffer_), (char *)"\r\n", (char *)" ");



void on_colors_sensed(SerialCommands * sender) {
  char * bottom = sender->Next();
  if(bottom == NULL) {
    // invalid colors
    return;
  }
  char * left = sender->Next();
  if(left== NULL) {
    // invalid colors
    return;
  }
  char * right = sender->Next();
  if(right== NULL) {
    // invalid colors
    return;
  }
  show_goalpost(bottom, left, right);
}

//This is the default handler, and gets called when no other command matches. 
void on_unrecognized(SerialCommands* sender, const char* cmd)
{
}


uint16_t sensor_color(char c) {
  switch(c) {
    case 'r':
      return RED;
    case 'g':
      return GREEN;
    case 'b':
      return BLUE;
    case 'k':
      return BLACK;
    default:
      return WHITE;
  }
}

void show_goalpost(const char * bottom, const char * left, const char * right) {
  char n_bottom = strlen(bottom);
  char n_left= strlen(left);
  char n_right = strlen(right);
  

  for(int i=0;i<n_bottom;++i) {
    const int sensor_size = 8;
    display.fillRect(10*i + 15,display_height-sensor_size,sensor_size,sensor_size,sensor_color(bottom[i]));
  }

  for(int i=0;i<n_left;++i) {
    const int sensor_width = 8;
    const int sensor_height = 3;
    const int gap = 1;
    display.fillRect(
      0, 
      display_height-sensor_height-i * (sensor_height + gap), 
      sensor_width, 
      sensor_height,
      sensor_color(left[i]));
  }

  for(int i=0;i<n_right;++i) {
    const int sensor_width = 8;
    const int sensor_height = 3;
    const int gap = 1;
    display.fillRect(
      display_width - sensor_width,
      display_height-sensor_height-i * (sensor_height + gap), 
      sensor_width, 
      sensor_height,
      sensor_color(right[i]));
  }

  
}


void pause() {
  delay(0);
}

SerialCommand cmd_colors_sensed("cld",on_colors_sensed);

void setup(void) {
  Serial1.begin(115200);
  display.begin();
  display.fillScreen(GRAY);
  serial_commands.SetDefaultHandler(on_unrecognized);
  serial_commands.AddCommand(&cmd_colors_sensed);
  
}

void drawTile(uint8_t x, uint8_t y) {
  display.fillRect(x+0,y+0,45,45,WHITE);
  display.fillRect(x+0,y+20,20,5,GREEN);
  display.fillRect(x+20,y+0,5,20,RED);
  display.fillRect(x+20,y+25,5,20,BLACK);
  display.fillRect(x+25,y+20,20,5,BLUE);
}

class LoopChecker {
  public:

    // execute run on the top of every loop
    void run() {
        last_loop_ms = loop_ms;
        loop_ms = millis();
        ++loop_count;
    }
    // returns true if loop time passes through n ms boundary
    bool every_n_ms(unsigned long ms) {
        return last_loop_ms && (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
    }
    unsigned long loop_ms = 0;
    unsigned long last_loop_ms = 0;
    unsigned long loop_count = 0;
};
LoopChecker loop_checker;


void loop() {
  loop_checker.run();
  serial_commands.ReadSerial();
  bool second_boundary = loop_checker.every_n_ms(1000);

/*
  if(second_boundary) {
    drawTile(0,0);
    drawTile(50,0);
  }
*/
  if(second_boundary) {
    Serial.println((String)"Loop count: " + loop_checker.loop_count);
  }
}

