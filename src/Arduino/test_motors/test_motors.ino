const int pin_left_fwd = 5;
const int pin_left_rev = 6;

const int pin_right_fwd = 21;
const int pin_right_rev = 22;

const int pin_left_a = 15;
const int pin_left_b = 4;

int sign_of(int i) {
  if (i<0) return -1;
  return 1;
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
      int power = abs(percent);
      int raw_power = ( percent == 0) ? 0 : map(power, 0, 100, power_stall, power_max);
      
      set_speed_raw(sign_of(percent) * raw_power);
      
    }
};

class TriangleWave {
public:
  int v_min;
  int v_max;
  int delta;
  int v;

  TriangleWave(int v_min, int v_max, int delta) : v_min(v_min), v_max(v_max), delta(delta), v(-delta) {
  }

  int next() {
    if(v >= v_max) {
      delta = -abs(delta);
    }
    if(v <= v_min) {
      delta = abs(delta);
    }
    v += delta;
    return v;
  }
};

Motor motor_left(pin_left_fwd, pin_left_rev);
Motor motor_right(pin_right_fwd, pin_right_rev);

void setup() {
  motor_left.setup();
  motor_right.setup();
  Serial.begin(115200);
  delay(1000);
  
  // put your setup code here, to run once:

}


void loop() {
  static TriangleWave percent(-100, 100, 1);
  int dt = 10000;

  for(;;) {
    motor_left.set_speed_percent(percent.next());
    motor_right.set_speed_percent(percent.next());
    delayMicroseconds(dt);
  }
 
  Serial.flush();
}
