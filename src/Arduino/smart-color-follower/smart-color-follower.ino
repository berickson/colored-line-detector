#include <SerialCommands.h> // see https://github.com/ppedro74/Arduino-SerialCommands

#define pin_oled_sclk 13
#define pin_oled_mosi 11
#define pin_oled_cs   10
#define pin_oled_rst  9
#define pin_oled_dc   12
#define pin_button_1_a 30
#define pin_button_1_b 32
const int pin_left_fwd = 5;
const int pin_left_rev = 6;

const int pin_right_fwd = 21;
const int pin_right_rev = 22;

const int pin_right_a = 17;
const int pin_right_b = 16;

const int pin_left_a = 15;
const int pin_left_b = 4;
const int pin_battery_voltage_divider = A9;

#define Bluetooth Serial3


int sign_of(int i) {
  if (i<0) return -1;
  return 1;
}

class QuadratureEncoder {
public:
  const int pin_a;
  const int pin_b;

  long a_ticks = 0;
  long b_ticks = 0;
  unsigned int a_us;
  unsigned int b_us;

  QuadratureEncoder(int pin_a, int pin_b) : pin_a(pin_a), pin_b(pin_b){
  }

  inline void on_a_changed() {
    a_us = micros();
    if(digitalReadFast(pin_a)==digitalReadFast(pin_b)) {
      --a_ticks;
    } else {
      ++a_ticks;
    }
  }

  inline void on_b_changed() {
    b_us = micros();
    if(digitalReadFast(pin_a)==digitalReadFast(pin_b)) {
      ++b_ticks;
    } else {
      --b_ticks;
    }
  }

  inline long get_ticks() {
    return a_ticks + b_ticks;
  }
};

class Speedometer {
public:
  QuadratureEncoder & encoder;
  float meters_per_tick = 0; // set by client
  int last_odo_a = 0;
  int last_odo_b = 0;
  unsigned int last_a_us = 0;
  unsigned int last_b_us = 0;
  unsigned int last_ab_us = 0;
  unsigned int last_clock_us = 0;
  volatile float velocity = 0;
  volatile float acceleration = 0;
  volatile float v_a;
  volatile float v_b;
  
  Speedometer(QuadratureEncoder & encoder, float meters_per_tick) : encoder(encoder), meters_per_tick(meters_per_tick) {}

  inline float get_velocity() {
    return velocity;
  }
  
  void update() {
    float last_v = velocity;

    auto a_us = encoder.a_us;
    auto b_us = encoder.b_us;
    auto odo_a = encoder.a_ticks;
    auto odo_b = encoder.a_ticks;
    auto clock_us = micros();
    
    if (a_us > last_a_us) {
      v_a =  (odo_a-last_odo_a)*2*meters_per_tick / (a_us - last_a_us) *1E6;
    } 
    if(b_us != last_b_us) {
      v_b =  (odo_b-last_odo_b)*2*meters_per_tick / (b_us - last_b_us) *1E6;
    }
    velocity = (v_a + v_b) / 2.;
  
  
    unsigned tick_us = std::max(a_us,b_us);
    unsigned last_tick_us = std::max(last_a_us, last_b_us);
    float elapsed_seconds = (tick_us - last_tick_us) / 1000000.;
    if (elapsed_seconds == 0) {
      // no tick this time, how long has it been?
      elapsed_seconds= ( clock_us - last_tick_us) / 1000000.;
      if (elapsed_seconds > 0.1){
        // it's been a long time, let's call velocity zero
        velocity = 0.0;
      } else {
        // we've had a tick recently, fastest possible speed is when a tick is about to happen
        // do nothing unless smaller than previously certain velocity
        float  max_possible = meters_per_tick / elapsed_seconds;
        if(max_possible < fabs(velocity)){
          if(velocity > 0)
            velocity = max_possible;
          else
            velocity = -max_possible;
        }
      }
    }
    if(elapsed_seconds > 0) {
      acceleration = (velocity - last_v) / elapsed_seconds;
    }
  
    last_odo_a = odo_a;
    last_odo_b = odo_b;
    last_a_us  = a_us;
    last_b_us  = b_us;
    last_clock_us = clock_us;
  }

  float get_odo_meters() {
    return (last_odo_a + last_odo_b) * meters_per_tick;
  }
};


QuadratureEncoder right_encoder(pin_right_a, pin_right_b);
Speedometer speedometer(right_encoder, 1./4520);

// unfortunatly, we need external callbacks
void on_right_a_changed () {
  right_encoder.on_a_changed();
}

void on_right_b_changed () {
  right_encoder.on_b_changed();
}



class Motor {
  public:
    const int pin_fwd;
    const int pin_rev;
    const int power_stall = (int) 145.;
    const int power_max = (int) 255;
    const int pwm_frequency = 50000;
    
    Motor(int pin_fwd, int pin_rev) : pin_fwd(pin_fwd), pin_rev(pin_rev){
    }

    void setup() {
      pinMode(pin_fwd, OUTPUT);
      pinMode(pin_rev, OUTPUT);
      analogWriteFrequency(pin_fwd, pwm_frequency);
      analogWriteFrequency(pin_rev, pwm_frequency);
      attachInterrupt(right_encoder.pin_a, on_right_a_changed, CHANGE);
      attachInterrupt(right_encoder.pin_b, on_right_b_changed, CHANGE);
    }

    void set_speed_raw(int speed) {
      Serial.println(speed);
      if(speed >= 0) {
        analogWrite(pin_fwd, speed);
        analogWrite(pin_rev, 0);
      } else {
        analogWrite(pin_fwd, 0);
        analogWrite(pin_rev, -speed);
      }
    }

    void set_speed_percent(int percent) {
      percent = constrain(percent, -100, 100);
      int power = abs(percent);
      int raw_power = ( percent == 0) ? 0 : map(power, 0, 100, power_stall, power_max);
      
      set_speed_raw(sign_of(percent) * raw_power);
      
    }
};

Motor motor_left(pin_left_fwd, pin_left_rev);
Motor motor_right(pin_right_fwd, pin_right_rev);


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

int speed_percent_sp = 50;

// button that is tied directly to two digital pins
// the only reason you would do this is if it is easier
// to mount to pins instead of mounting to ground or vcc
class TwoPinButton {
public:
  const int pin_a;
  const int pin_b;
  bool prev_pressed = false;
  
  TwoPinButton(int pin_a, int pin_b) : pin_a(pin_a), pin_b(pin_b) {
    pinMode(pin_a, OUTPUT);
    pinMode(pin_b, INPUT_PULLUP);
    digitalWrite(pin_a, 0);
  }

  bool is_pressed() {
    return digitalRead(pin_b)==0;
  }

  bool was_clicked() {
    bool pressed = is_pressed();
    bool clicked = false;
    if(is_pressed() && ! prev_pressed) {
      clicked = true;
    }
    prev_pressed = pressed;
    return clicked;
  }
  
};

TwoPinButton button1(pin_button_1_a, pin_button_1_b);

class VoltageDivider {
public:
  const int pin;
  float multiplier;

  // 3.338 was measured for vref
  VoltageDivider(int pin, float resistor1_k, float resistor2_k) : pin(pin), multiplier((resistor1_k + resistor2_k) / resistor2_k  * 3.338 / 1023 * 8 / 7.37) {
  }

  void setup() {
    Serial.begin(115200);
    while(!Serial){}
    Bluetooth.begin(9600);
    Serial.print("waiting for Bluetooth...");
    while(!Bluetooth){
      delay(1);
    }
    Serial.println("ready");
    
    pinMode(pin, INPUT);
  }

  float get_voltage() {
    const int sample_count = 100;
    float sum = 0;
    for(int i = 0; i < sample_count; i++) {
      sum += analogRead(pin) * multiplier;  
    }
    return sum / sample_count;
    
  }
 
};

VoltageDivider v_bat(pin_battery_voltage_divider, 463.1, 98.28);




Adafruit_SSD1331 display = Adafruit_SSD1331(pin_oled_cs, pin_oled_dc, pin_oled_rst);

float p = 3.1415926;
const int t_delay=0;


// buffer must be big enough for command, arguments, terminiators and null
char camera_command_buffer_[100];
SerialCommands camera_commands(&Serial1, camera_command_buffer_, sizeof(camera_command_buffer_), (char *)"\r\n", (char *)" ");
char bluetooth_command_buffer_[100];
SerialCommands bluetooth_commands(&Bluetooth, bluetooth_command_buffer_, sizeof(bluetooth_command_buffer_), (char *)"\r\n", (char *)" ");


void cmd_set_speed(SerialCommands * sender) {
  speed_percent_sp = atoi(sender->Next());
}

void cmd_left(SerialCommands * sender) {
  motor_left.set_speed_percent(-speed_percent_sp);
  motor_right.set_speed_percent(speed_percent_sp);
}

void cmd_right(SerialCommands * sender) {
  motor_left.set_speed_percent(speed_percent_sp);
  motor_right.set_speed_percent(-speed_percent_sp);
}

void cmd_go(SerialCommands * sender) {
  sender->GetSerial()->println("inside go");
  float d = atof(sender->Next());
  float destination_meters = d + speedometer.get_odo_meters();
  float velocity_max = atof(sender->Next());
  float throttle = 0;
  float velocity_k_p = 200.0;
  float velocity_k_d = 50;
  float last_error = 0;
  float last_position_error = 0;

  float position_k_p = 10;
  float position_k_d = 200;
  float accel_m_s2 = 3;

  motor_left.set_speed_percent(throttle);
  motor_right.set_speed_percent(throttle);

  sender->GetSerial()->println((String) "go " + d + " " + velocity_max);

  while(speedometer.get_odo_meters() < destination_meters) {
    delay(1);
    speedometer.update();
    loop();

    float position_error = destination_meters - speedometer.get_odo_meters();
    float velocity_sp = position_k_p * position_error + position_k_d * (position_error-last_position_error);
    //float velocity_sp = position_error > 0 ? sqrt(2 * position_error * accel_m_s2) : 0;
    velocity_sp = constrain(velocity_sp, -velocity_max, velocity_max);

    float error = velocity_sp - speedometer.velocity;
    float d_error = error-last_error;
    throttle += velocity_k_p * error + velocity_k_d * d_error;
    throttle = constrain(throttle, -100, 100);
    //output += (String)position_error + " v: " + speedometer.velocity + " v_sp: " + velocity_sp + "gas: " + throttle + "\r\n";
    motor_left.set_speed_percent(throttle);
    motor_right.set_speed_percent(throttle);
    last_error = error;
    last_position_error = position_error;
  }

  motor_left.set_speed_percent(0);
  motor_right.set_speed_percent(0);
  sender->GetSerial()->println((String)"final velocity: " + speedometer.get_velocity());
  sender->GetSerial()->println((String)"final distance: " + (destination_meters - speedometer.get_odo_meters()));
}



void cmd_fwd(SerialCommands * sender) {
  motor_left.set_speed_percent(speed_percent_sp);
  motor_right.set_speed_percent(speed_percent_sp);
}

void cmd_rev(SerialCommands * sender) {
  motor_left.set_speed_percent(-speed_percent_sp);
  motor_right.set_speed_percent(-speed_percent_sp);
}

void cmd_stop(SerialCommands * sender) {
  motor_left.set_speed_percent(0);
  motor_right.set_speed_percent(0);
}

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

SerialCommand cmd_colors_sensed("cld",on_colors_sensed);

SerialCommand cmd_go_sensed("go",cmd_go);
SerialCommand cmd_fwd_sensed("fwd",cmd_fwd);
SerialCommand cmd_rev_sensed("rev",cmd_rev);
SerialCommand cmd_left_sensed("left",cmd_left);
SerialCommand cmd_right_sensed("right",cmd_right);
SerialCommand cmd_stop_sensed("stop",cmd_stop);
SerialCommand cmd_set_speed_sensed("speed",cmd_set_speed);

void setup(void) {
  Serial1.begin(115200);
  display.begin();
  motor_left.setup();
  motor_right.setup();
  display.fillScreen(GRAY);
  camera_commands.SetDefaultHandler(on_unrecognized);
  camera_commands.AddCommand(&cmd_colors_sensed);

  Bluetooth.begin(9600);
  bluetooth_commands.AddCommand(&cmd_go_sensed);
  bluetooth_commands.AddCommand(&cmd_fwd_sensed);
  bluetooth_commands.AddCommand(&cmd_rev_sensed);
  bluetooth_commands.AddCommand(&cmd_left_sensed);
  bluetooth_commands.AddCommand(&cmd_right_sensed);
  bluetooth_commands.AddCommand(&cmd_stop_sensed);
  bluetooth_commands.AddCommand(&cmd_set_speed_sensed);
  
}

void drawTile(uint8_t x, uint8_t y) {
  display.fillRect(x+0,y+0,45,45,WHITE);
  display.fillRect(x+0,y+20,20,5,GREEN);
  display.fillRect(x+20,y+0,5,20,RED);
  display.fillRect(x+20,y+25,5,20,BLACK);
  display.fillRect(x+25,y+20,20,5,BLUE);
}

const char * color_order = "krgb";
int current_color_index = 0;

/*
class Tile {
  
  public:
  char color_forward = "?";
  char color_right = "?";
  char color_back = "?";
  char color_left = "?";
}
*/


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
  static int run_mode = 0;
  const int run_mode_count = 12;
  
  loop_checker.run();
  camera_commands.ReadSerial();
  bluetooth_commands.ReadSerial();
  //bool boundary_1_s = loop_checker.every_n_ms(1000);
  bool boundary_10_ms = loop_checker.every_n_ms(10);
  bool boundary_100_ms = loop_checker.every_n_ms(100);

  if(boundary_10_ms) {
    speedometer.update();
  }

  if(boundary_100_ms) {
    if(button1.was_clicked()) {
      run_mode = (run_mode+1)%run_mode_count;

      switch (run_mode) {
        case 0:
          motor_left.set_speed_percent(0);
          motor_right.set_speed_percent(0);
          break;
          
        case 1:
          motor_left.set_speed_percent(100);
          motor_right.set_speed_percent(100);
          break;
          
        case 2:
          motor_left.set_speed_percent(0);
          motor_right.set_speed_percent(0);
          break;
          
        case 3:
          motor_left.set_speed_percent(-100);
          motor_right.set_speed_percent(-100);
          break;
          
        case 4:
          motor_left.set_speed_percent(0);
          motor_right.set_speed_percent(0);
          break;

        case 5:
          motor_left.set_speed_percent(-100);
          motor_right.set_speed_percent(100);
          break;

        case 6:
          motor_left.set_speed_percent(0);
          motor_right.set_speed_percent(0);
          break;
          
        case 7:
          motor_left.set_speed_percent(100);
          motor_right.set_speed_percent(-100);
          break;
          
        case 8:
          motor_left.set_speed_percent(0);
          motor_right.set_speed_percent(0);
          break;
          
        case 9:
          motor_left.set_speed_percent(100);
          motor_right.set_speed_percent(50);
          break;
          
        case 10:
          motor_left.set_speed_percent(0);
          motor_right.set_speed_percent(0);
          break;
          
        case 11:
          motor_left.set_speed_percent(50);
          motor_right.set_speed_percent(100);
          break;
          
        default:
          motor_left.set_speed_percent(0);
          motor_right.set_speed_percent(0);
          break;
        
      }
    }
  }


  

/*
  if(second_boundary) {
    drawTile(0,0);
    drawTile(50,0);
  }
*/
  if(boundary_100_ms) {
    //display.fillScreen(GRAY);
    display.setCursor(10,10);
    String v_string = (String)v_bat.get_voltage()+"v " + speedometer.velocity + " "; // space helps drawing issues
    int16_t x1,y1;
    uint16_t h, w;
    display.getTextBounds((char *)v_string.c_str(), 10, 10,
      &x1, &y1, &w, &h);
    display.fillRect(x1,y1,w,h,GRAY);
    display.print(v_string);
    Serial.println((String)"Loop count: " + loop_checker.loop_count);
  }
}

