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

const int pin_right_a = 16;
const int pin_right_b = 17;

const int pin_left_a = 15;
const int pin_left_b = 14;
const int pin_battery_voltage_divider = A9;

bool g_stop = false;
float line_center = 0;
uint32_t line_center_us = 0;

#define bluetooth Serial3
#define jevois Serial1

float sign_of(float i) {
  if (i<0) return -1;
  return 1;
}

int sign_of(int i) {
  if (i<0) return -1;
  return 1;
}

struct Settings {
  float velocity_k_p = 3.0;
  float velocity_k_d = 100;
  float position_k_p = 20;
  float position_k_d = 400;
  float line_k_p = 0.03;
  float throttle_follow = 15;
};
Settings settings;

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
  long last_odo_a = 0;
  long last_odo_b = 0;
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
  
  void execute() {
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
Speedometer right_speedometer(right_encoder, 1./7132);
QuadratureEncoder left_encoder(pin_left_a, pin_left_b);
Speedometer left_speedometer(left_encoder, 1./7132);

// unfortunatly, we need external callbacks
void on_right_a_changed () {
  right_encoder.on_a_changed();
}

void on_right_b_changed () {
  right_encoder.on_b_changed();
}

void on_left_a_changed () {
  left_encoder.on_a_changed();
}

void on_left_b_changed () {
  left_encoder.on_b_changed();
}



class Motor {
  public:
    const int pin_fwd;
    const int pin_rev;
    const int power_stall = (int) 155.;
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
      attachInterrupt(left_encoder.pin_a, on_left_a_changed, CHANGE);
      attachInterrupt(left_encoder.pin_b, on_left_b_changed, CHANGE);
    }

    void set_speed_raw(int speed) {
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

float speed_m_s_sp = 0.25;


class Goalpost {
public:
  struct TopColor {
    char color = '.';
    uint16_t count = 0;
  };

  struct RgbCounts {
    uint16_t r=0;
    uint16_t g=0;
    uint16_t b=0;
    uint16_t k=0;
    uint16_t w=0;

    void print() {
      bluetooth.println((String)"r:"+r+ " g:" + g + " b:" + b + " k: " + k + " w:"+ w );
    }

    void reset() {
      *this = {0};
    }

    void add_reading(String colors) {
      uint16_t max_i = colors.length();
      for (int i = 0; i < max_i; ++i) {
        char c = colors[i];
        switch(c) {
          case 'r':
            ++r;
            break;
          case 'g':
            ++g;
            break;
          case 'b':
            ++b;
            break;
          case 'k':
            ++k;
            break;
          case '.':
            ++w;
            break;
          default:
            break;
        }
      }
    }

    TopColor top_color() {
      TopColor top;
      if(r > top.count) {
        top.color = 'r';
        top.count = r;
      }
      if(g > top.count) {
        top.color = 'g';
        top.count = g;
      }
      if(b > top.count) {
        top.color = 'b';
        top.count = b;
      }
      if(k > top.count) {
        top.color = 'k';
        top.count = k;
      }
      return top;
    }
  };

  void print_color_counts() {
    bluetooth.println("left:");
    left_counts.print();
    bluetooth.println("right:");
    right_counts.print();
  }

  RgbCounts left_counts, right_counts;

  String left;
  String right;
  String bottom;


  bool end_ahead() {
    uint16_t color_count = 0;
    uint16_t max_i = bottom.length();
    for (uint16_t i = 0; i < max_i; ++i) {
      if (bottom[i]!='.') {
        ++color_count;
      }
    }
    return color_count > 3 || color_count == 0;
  }

  bool intersection_ahead() {
    unsigned int max_distance = 10;
    bool left_ahead = false;
    bool right_ahead = false;
    unsigned int end_i = std::min(std::min(max_distance, left.length()),right.length());
    for (unsigned int i=0; i < end_i && left[i] != 0; i++) {
      if (left[i] != '.') {
        left_ahead = true;
        break;
      }
    }
    for (unsigned int i=0; i < end_i && right[i] != 0; i++) {
      if (right[i] != '.') {
        right_ahead = true;
        break;
      }
    }
    if(left_ahead && right_ahead) {
      return true;
    }
    return false;
  }

  char current_color() {
    RgbCounts bottom_counts;
    bottom_counts.add_reading(bottom);
    return bottom_counts.top_color().color;
  }

  enum Direction {dir_forward, dir_left, dir_right};

  
  Direction get_color_direction(char color_name) {
    if (current_color() == color_name) {
      return Direction::dir_forward;
    }
    if (left_counts.top_color().color == color_name) {
       return Direction::dir_left;
    } else if (right_counts.top_color().color == color_name) {
       return Direction::dir_right;
    }
    return Direction::dir_forward;

    // see what current color is
    

  }

  void reset_color_counts() {
    left_counts.reset();
    right_counts.reset();
  }

  void set_colors(const char * left, const char * right, const char * bottom) {
    this->left = left;
    this->right = right;
    this->bottom = bottom;

    left_counts.add_reading(this->left);
    right_counts.add_reading(this->right);
  }

} goalpost;


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


struct WheelPID {
  Speedometer & speedometer;
  unsigned long last_execute_us = 0;
  float last_velocity_error;
  float throttle;
  float last_position_error;
  float start_meters;
  float destination_meters;
  float velocity_max;
  WheelPID(Speedometer & speedometer) :
    speedometer(speedometer), 
    last_execute_us(0),
    last_velocity_error(NAN),
    throttle(0),
    last_position_error(NAN),
    start_meters(NAN),
    velocity_max(speed_m_s_sp) {
  }

  void set_goal(unsigned long us, float d, double velocity_max) {
    last_execute_us = us;
    start_meters = speedometer.get_odo_meters();
    destination_meters = start_meters + d;
    this->velocity_max = velocity_max;
    last_velocity_error = 0;
    last_position_error = d;
  }


  float execute(unsigned long us) {
    float elapsed_ms = (us - last_execute_us) / 1000.;
    float position_error = destination_meters - speedometer.get_odo_meters();
    float velocity_sp = settings.position_k_p * position_error;
    if (last_position_error != NAN) {
      velocity_sp += settings.position_k_d * (position_error-last_position_error);
    }
    velocity_sp = constrain(velocity_sp, -velocity_max, velocity_max);

    float velocity_error = velocity_sp - speedometer.velocity;
    float d_error = velocity_error - last_velocity_error;
    throttle += (settings.velocity_k_p * velocity_error + settings.velocity_k_d * d_error) * elapsed_ms;
    throttle = constrain(throttle, -100, 100);
    last_execute_us = us;
    last_position_error = position_error;
    last_velocity_error = velocity_error;
    return throttle;
  }

  bool done() {
    float p_error = fabs(destination_meters - speedometer.get_odo_meters());
    float v_error = fabs(speedometer.get_velocity());
    return p_error < 0.01 && v_error < 0.01;
  }
};
WheelPID left_wheel(left_speedometer);
WheelPID right_wheel(right_speedometer);

class Driver {
public:
  bool destination_pending = false;
  bool disable = false;

  float right_destination_meters = NAN;
  float left_destination_meters = NAN;
  float right_start_meters = NAN;
  float left_start_meters = NAN;
  float throttle = 0;

  float k_diff = settings.position_k_d * 15;
  float last_error = 0;
  float last_position_error = 0;
  float velocity_max = 0;
  float ratio_left;
  float ratio_right;

  float accel_m_s2 = 3;

  float fwd_rev = sign_of(ratio_right); // 1 for forward, -1 for reverse

  unsigned int last_execute_us;

  

public:
  void set_goal(float d, double velocity_max, float ratio_left = 1, float ratio_right = 1) {
    this->ratio_right = ratio_right;
    this->ratio_left = ratio_left;
    this->velocity_max = velocity_max;
    this->left_start_meters = left_speedometer.get_odo_meters();
    this->right_start_meters = right_speedometer.get_odo_meters();
    this->right_destination_meters = ratio_right * d + this->right_start_meters;
    this->left_destination_meters = ratio_left * d + this->left_start_meters;
    //throttle = 0;
    last_error = 0;
    last_position_error = 0;
    fwd_rev = sign_of(ratio_right); // 1 for forward, -1 for reverse
    destination_pending = true;
    g_stop = false;
    right_wheel.set_goal(micros(), ratio_right * d, velocity_max);
    left_wheel.set_goal(micros(), ratio_left * d, velocity_max);
  }

  void turn_degrees(float degrees, float velocity_max) {
    float meters_per_degree = 0.14/90.;
    float d = degrees * meters_per_degree;
    int ccw_cw = sign_of(d);
    set_goal(fabs(d), velocity_max, -ccw_cw, ccw_cw);
  }

  void stop() {
    destination_pending = false;
    throttle = 0;
  }


  void execute() {
    if(disable) {
      return;
    }
    if(g_stop) {
      destination_pending = false;
    }
    if(!destination_pending) {
      motor_left.set_speed_percent(0);
      motor_right.set_speed_percent(0);
      return;
    }


    // assumes speedometer is current
    unsigned long us = micros();
    float elapsed_ms = (us - last_execute_us) / 1000.;
    float position_error = right_destination_meters - right_speedometer.get_odo_meters();
    float velocity_sp = settings.position_k_p * position_error + settings.position_k_d * (position_error-last_position_error)/elapsed_ms;
    velocity_sp = constrain(velocity_sp, -velocity_max, velocity_max);

    float error = velocity_sp - right_speedometer.velocity;
    float d_error = error-last_error;
    throttle += fwd_rev * (settings.  velocity_k_p * error + settings.velocity_k_d * d_error) * elapsed_ms;
    throttle = constrain(throttle, -100, 100);

    // set a correction in the left motor based on how far off it is compared to the right motor
    float left_error = 0;
    {
      float right_travelled = right_speedometer.get_odo_meters() - right_start_meters;
      float left_travelled = left_speedometer.get_odo_meters() - left_start_meters;
      float left_expected = right_travelled * ratio_right / ratio_left;
      left_error = left_expected - left_travelled;
    }

    // motor_right.set_speed_percent(ratio_right * throttle - left_error * k_diff);
    //motor_right.set_speed_percent(ratio_right * throttle - left_error * k_diff);
    //motor_left.set_speed_percent(ratio_left * throttle + left_error * k_diff);
    
    motor_right.set_speed_percent(right_wheel.execute(us));
    motor_left.set_speed_percent( left_wheel.execute(us));
    //k_diff = 0;
    //motor_right.set_speed_percent(right_wheel.execute(us)- left_error * k_diff);
    //motor_left.set_speed_percent( left_wheel.execute(us) + left_error * k_diff);
    //motor_right.set_speed_percent(20);
    //motor_left.set_speed_percent(20);

    
    //motor_left.set_speed_percent(left_error * k_diff);

    last_error = error;
    last_position_error = position_error;
    last_execute_us = micros();
/*
    if (right_speedometer.get_odo_meters() * fwd_rev  < right_destination_meters * fwd_rev || fabs(right_speedometer.get_velocity()) > 0.02) {
      destination_pending = true;
    } else {
      destination_pending = false;
      bluetooth.println("goal");
    }
*/
    if (left_wheel.done() && right_wheel.done()) {
      destination_pending = false;
      bluetooth.println("goal");
    } else {
      destination_pending = true;
    }
  }
};

Driver driver;

class Planner {
public:
  enum PlannerState {
    idle,
    looking_for_intersection,
    driving_to_intersection,
    waiting_for_turn,
    waiting_for_end
  } state;

  Planner() {
    state = idle;
  }
  char next_color;

  String colors = "rbg";
  uint16_t next_color_index = 0;

  void go_intersection(String colors) {
    if(colors.length()==0) {
      return;
    }
    this->colors = colors;
    next_color = colors[0];
    next_color_index = 1;
    driver.set_goal(5.0, speed_m_s_sp);
    state = looking_for_intersection;
  }

  void execute() {
    if (g_stop || state == idle) {
      state = idle;
      driver.disable=false;
      if(button1.was_clicked()) {
        go_intersection("rbg");
      }
      return;
    }
    if (state==idle) {
      return;
    }
    if (button1.was_clicked()) {
      driver.stop();
      state = idle;
      g_stop = true;
    }

    if (state == looking_for_intersection) {
      // try to follow the line
      driver.disable = true;
      motor_left.set_speed_percent(settings.throttle_follow + settings.line_k_p * line_center);
      //  bluetooth.println(settings.throttle_follow + settings.line_k_p * line_center);
      motor_right.set_speed_percent(settings.throttle_follow - settings.line_k_p * line_center);

      //driver.set_goal(1.0, speed_m_s_sp, 1 + settings.line_k_p * line_center, 1 - settings.line_k_p * line_center);
      if(goalpost.intersection_ahead()) {
        goalpost.reset_color_counts();
        driver.disable = false;
        driver.set_goal(0.155, speed_m_s_sp);
        state = driving_to_intersection;
      }
    }
    if (state == driving_to_intersection) {
      if (!driver.destination_pending) {
        bluetooth.println((String) "planner looking for " + next_color );
        goalpost.print_color_counts();
        // turn to direction of next color
        Goalpost::Direction direction = goalpost.get_color_direction(next_color);
        if (direction==Goalpost::Direction::dir_left) {
          driver.turn_degrees(90, speed_m_s_sp);
        } else if (direction==Goalpost::Direction::dir_right) {
          driver.turn_degrees(-90, speed_m_s_sp);
        }
        state = waiting_for_turn;
      }
    }
    if (state == waiting_for_turn) {
      if (!driver.destination_pending) {
        if(next_color_index < colors.length()) {
          next_color = colors[next_color_index];
          ++next_color_index;
          driver.set_goal(5.0, speed_m_s_sp);
          state = looking_for_intersection;
        } else {
          driver.set_goal(5.0, speed_m_s_sp);
          state = waiting_for_end;
        }
      }
    }
    if (state == waiting_for_end) {
      if(goalpost.end_ahead()) {
          driver.stop();
          state = idle;
          bluetooth.println("planner done");
      }
    }
  }
};

Planner planner;


// buffer must be big enough for command, arguments, terminators and null
char camera_command_buffer_[100];
SerialCommands camera_commands(&jevois, camera_command_buffer_, sizeof(camera_command_buffer_), (char *)"\r\n", (char *)" ");
char bluetooth_command_buffer_[100];
SerialCommands bluetooth_commands(&bluetooth, bluetooth_command_buffer_, sizeof(bluetooth_command_buffer_), (char *)"\r\n", (char *)" ");

const float max_meters = 1;

void on_speed(SerialCommands * sender) {
  speed_m_s_sp = atof(sender->Next());
  bluetooth.println("ok");
}

void on_left(SerialCommands * sender) {
  driver.set_goal(max_meters, speed_m_s_sp, -1, 1);
  bluetooth.println("ok");
}

void on_right(SerialCommands * sender) {
  driver.set_goal(max_meters, speed_m_s_sp, 1, -1);
  bluetooth.println("ok");
}

void on_fwd(SerialCommands * sender) {
  driver.set_goal(max_meters, speed_m_s_sp, 1, 1);
  bluetooth.println("ok");
}

void on_rev(SerialCommands * sender) {
  driver.set_goal(max_meters, speed_m_s_sp, -1, -1);
  bluetooth.println("ok");
}

void on_stop(SerialCommands * sender) {
  driver.stop();
  g_stop = true;
  sender->GetSerial()->println("ok");
}

void on_turn(SerialCommands * sender) {
  // turns anticlockwise distance d
  sender->GetSerial()->println("inside turn");
  float degrees = atof(sender->Next());
  driver.turn_degrees(degrees, speed_m_s_sp);
  bluetooth.println("ok");
}

void on_help(SerialCommands * sender) {
  bluetooth.println("commands");
  bluetooth.println("--------");
  bluetooth.println("?");
  bluetooth.println("set");
  bluetooth.println("go");
  bluetooth.println("go_intersection");
  bluetooth.println("turn");
  bluetooth.println("fwd");
  bluetooth.println("rev");
  bluetooth.println("left");
  bluetooth.println("right");
  bluetooth.println("stop");
  bluetooth.println("speed");
  bluetooth.println();
  bluetooth.println("status");
  bluetooth.println("------");
  bluetooth.println((String)"voltage: "+v_bat.get_voltage());
}

// calibrates the white balance
void on_wb(SerialCommands * sender) {
  bluetooth.println("calibrating white balance");
  bluetooth.println("wait...");
  // set auto white balance
  jevois.println("");
  jevois.println("setcam autoexp 0");
  delay(10);
  jevois.println("setcam autowb 1");
  delay(5000);
  jevois.println("setcam autoexp 1");
  delay(10);
  jevois.println("setcam autowb 0");
  bluetooth.println("done");
}

void on_set(SerialCommands * sender) {
  String param = sender->Next();

  String param2 = sender->Next();
  if(param) {
    if(param2 == "") {
      bluetooth.println("invalid value");
      return;
    } else {
      double value = atof(param2.c_str());
      if (value < 0 || value == NAN) {
        bluetooth.println("invalid value");
        return;
      } else {
        if(param == "") {
          // do nothing if there is no first parameter
        } else if (param == "throttle_follow" || param == "tf") {
          settings.throttle_follow = value;
        } else if (param == "velocity_k_p" || param == "vkp") {
          settings.velocity_k_p = value;
        } else if (param == "velocity_k_d" || param == "vkd") {
          settings.velocity_k_d = value;
        } else if (param == "position_k_p" || param == "xkp") {
          settings.position_k_p = value;
        } else if (param == "position_k_d" || param == "xkd") {
          settings.position_k_d = value;
        } else if (param == "line_k_p" || param == "lkp") {
          settings.line_k_p = value;
        } else {
          bluetooth.println("invalid parameter " + param);
          return;
        }
      }
    }
  }

  bluetooth.println((String)"throttle_follow (tf): " + settings.throttle_follow);
  bluetooth.println((String)"velocity_k_p (vkp): " + settings.velocity_k_p);
  bluetooth.println((String)"velocity_k_d (vkd): " + settings.velocity_k_d);
  bluetooth.println((String)"position_k_p (xkp): " + settings.position_k_p);
  bluetooth.println((String)"position_k_d (xkd): " + settings.position_k_d);
  bluetooth.println((String)"line_k_p (lkp): " + settings.line_k_p);

  bluetooth.println("ok");
}


void on_go(SerialCommands * sender) {
  const char * param1 = sender->Next();
  float d = atof(param1);
  driver.set_goal(d, speed_m_s_sp);
  bluetooth.println("ok");
}

void on_go_intersection(SerialCommands * sender) {
  String color = sender->Next();
  planner.go_intersection(color);
  return;
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
  goalpost.set_colors(left, right, bottom);
  
  show_goalpost(bottom, left, right);
}

void on_line_sensed(SerialCommands * sender) {
  line_center = atof(sender->Next());
  line_center_us = micros();
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
SerialCommand cmd_line_sensed("lc",on_line_sensed);

SerialCommand cmd_help("?", on_help);
SerialCommand cmd_wb("wb", on_wb);
SerialCommand cmd_set("set", on_set);
SerialCommand cmd_go("go", on_go);
SerialCommand cmd_go_intersection("go_intersection", on_go_intersection);
SerialCommand cmd_turn("turn", on_turn);
SerialCommand cmd_fwd("fwd", on_fwd);
SerialCommand cmd_rev("rev", on_rev);
SerialCommand cmd_left("left", on_left);
SerialCommand cmd_right("right", on_right);
SerialCommand cmd_stop("stop", on_stop);
SerialCommand cmd_speed("speed", on_speed);

void setup(void) {
  jevois.begin(115200); // camera
  Serial.begin(115200);  // host
  //while(!Serial){}
  bluetooth.begin(9600);
  Serial.print("waiting for Bluetooth...");
  while(!bluetooth){
    delay(1);
  }
  Serial.println("ready");
  delay(1000);

  bluetooth.print("AT+BAUD8");
  delay(100);
  while(bluetooth.available()) {
    Serial.write(bluetooth.read());
  }
  bluetooth.end();
  bluetooth.begin(115200); // http://www.instructables.com/id/AT-command-mode-of-HC-05-Bluetooth-module/
  bluetooth.println("bluetooth connected at 115200");
  Serial.println("bluetooth connected at 115200");



  display.begin();
  motor_left.setup();
  motor_right.setup();
  display.fillScreen(GRAY);
  camera_commands.SetDefaultHandler(on_unrecognized);
  camera_commands.AddCommand(&cmd_colors_sensed);
  camera_commands.AddCommand(&cmd_line_sensed);

  bluetooth_commands.AddCommand(&cmd_help);
  bluetooth_commands.AddCommand(&cmd_set);
  bluetooth_commands.AddCommand(&cmd_wb);
  bluetooth_commands.AddCommand(&cmd_go);
  bluetooth_commands.AddCommand(&cmd_go_intersection);
  bluetooth_commands.AddCommand(&cmd_turn);
  bluetooth_commands.AddCommand(&cmd_fwd);
  bluetooth_commands.AddCommand(&cmd_rev);
  bluetooth_commands.AddCommand(&cmd_left);
  bluetooth_commands.AddCommand(&cmd_right);
  bluetooth_commands.AddCommand(&cmd_stop);
  bluetooth_commands.AddCommand(&cmd_speed);
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

    // execute on the top of every loop
    void execute() {
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
  loop_checker.execute();
  camera_commands.ReadSerial();
  bluetooth_commands.ReadSerial();

  if(loop_checker.every_n_ms(1)) {
    right_speedometer.execute();
    left_speedometer.execute();
    planner.execute();
    driver.execute();
  }

  if(loop_checker.every_n_ms(100)) {
    display.setCursor(10,10);
    //String v_string = (String)left_speedometer.velocity + " " + right_speedometer.velocity+ " "; // space helps drawing issues
    //String v_string = (String)left_speedometer.velocity + " " + right_speedometer.velocity+ " "; // space helps drawing issues
    //String v_string = (String)left_speedometer.get_odo_meters() + " " + right_speedometer.get_odo_meters()+ " "; // space helps drawing issues
    String v_string = (String)line_center + " ";
    int16_t x1,y1;
    uint16_t h, w;
    display.getTextBounds((char *)v_string.c_str(), 10, 10,
      &x1, &y1, &w, &h);
    display.fillRect(x1,y1,w,h,GRAY);
    display.print(v_string);
  }
}

