///==============================================================================///

/// #INCLUDES

#include "Wire.h" // For I2C
#include "LCD.h" // For LCD
#include "LiquidCrystal_I2C.h"

///==============================================================================///

/// #DEFINES

#define CAP_TOUCH_CALIBRATE_SAMPLES 50.00000
#define CAP_TOUCH_SAMPLES 10.00000

#define CAP_TOUCH_PIN 1

#define ISD 10

///==============================================================================///

///==============================================================================///

/// ENUMS

enum Colours {
    White = 0b000,
    Red = 0b001,
    Green = 0b010,
    Blue = 0b011,
    Black = 0b100
};

enum NavStates {
    Forward,
    Reverse,
    RotateRight,
    RotateLeft,
    Stop,
    MazeDone
};

enum SensorPosition {
    Left,
    Right,
    Unseen,
    Seen,
    CrossedLine
};

enum States {
  Idle,
  Calibrate,
  Maze,
  Sos
};

enum p_bytes {
  controlByte = 0,
  dat1 = 1,
  dat0 = 2,
  dec = 3
};


///==============================================================================///

///==============================================================================///

/// STRUCTS

struct NAVCON {
    enum NavStates state;
    enum NavStates prev;
    enum NavStates next;
    enum SensorPosition first_red;
    bool outside_sensor;
    enum SensorPosition first_sensor_side;
    enum Colours colour;
    enum Colours prev_colour;
    uint16_t reference_distance;
    uint8_t red_at_sensor;
    uint8_t blue_count;
    uint16_t AOI_correction;
};

struct MDPS {
    uint8_t level;
    uint16_t rotation;
    uint8_t left_wheel_speed;
    uint8_t right_wheel_speed;
    uint16_t distance;
};

struct SS {
    enum Colours sensor[5];
    uint8_t incidence;
};

struct Packet {
  byte bytes[4];
};

///==============================================================================///


///==============================================================================///

/// SYSTEM VARIABLES

struct SS* sensorSystem = NULL;
struct MDPS* motorSystem = NULL;
struct NAVCON* navcon = NULL;

enum States* state = NULL;

bool* touch = NULL;
bool* clap = NULL;

float* cap_touch_threshold;

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);

///==============================================================================///

///******************************************************************************///
///------------------------------------ISR--------------------------------------///
///==============================================================================///

void _ISR() {
  *touch = run_touch(*cap_touch_threshold);
  *clap = run_clap();
  display_critical_diagnostics();
}

///------------------------------>> END of ISR <<--------------------------------///
///******************************************************************************///

void setup() {
  // SYSTEM VARIABLE INIT
    
    // sensor system:

    sensorSystem = new struct SS;
    
    sensorSystem->sensor[0] = White;
    sensorSystem->sensor[1] = White;
    sensorSystem->sensor[2] = White;
    sensorSystem->sensor[3] = White;
    sensorSystem->sensor[4] = White;
    sensorSystem->incidence = 0;
    
    // motor system:

    motorSystem = new struct MDPS;
    
    motorSystem->distance = 0;
    motorSystem->left_wheel_speed = 0;
    motorSystem->right_wheel_speed = 0;
    motorSystem->level = 0;
    motorSystem->rotation = 0;
    
    // NAVCON: 

    navcon = new struct NAVCON;
    
    navcon->state = Forward;
    navcon->next = Forward;
    navcon->prev = Forward;
    navcon->red_at_sensor = 255;
    navcon->first_red = Unseen;
    navcon->prev_colour = White;
    navcon->colour = White;
    navcon->outside_sensor = false;
    navcon->reference_distance = 200;
    navcon->AOI_correction = 0;
    navcon->blue_count = 0;

    // state: 

    state = new States;
    *state = Idle;

    // cap touch:

    touch = new bool;
    *touch = false;
    cap_touch_threshold = new float;

    // clap:
    
    clap = new bool;
    *clap = false;
    
  // END SYSTEM VARIABLE INIT
  //==================================

  // ARDIUNO INIT

    // Serial comms:
    
    Serial.begin(19200);

    // LCD:

    lcd.begin (20,4); // 20 x 4 LCD module
    lcd.setBacklightPin(3,POSITIVE); // BL, BL_POL
    lcd.setBacklight(HIGH);
    lcd.setCursor(0,0);
    lcd.print("START INIT");
    // timer interrupt:

    #if USE_TIMER_1
    
    ITimer1.init();
    ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS, _ISR);

    #endif
    //display_critical_diagnostics();
    lcd.setCursor(0,0);
    lcd.print("END INIT    ");

    *cap_touch_threshold = 0;
  // END ARDUINO INIT
  //==================================

}

void loop() {
  //*cap_touch_threshold = calibrate_cap_touch();
  _ISR();
  
  switch (*state) {
      case Idle: {run_idle(state, navcon, *cap_touch_threshold);  break;}
      case Calibrate: {run_calibrate(state, *cap_touch_threshold);  break;}
      case Maze: {run_maze(state, motorSystem, sensorSystem, navcon, *cap_touch_threshold);  break;}
      case Sos: {run_sos(state);  break;}
  }

}

///******************************************************************************///
///---------------------------------NAVCON---------------------------------------///
///==============================================================================///

void green_encounter(struct NAVCON* navcon, uint8_t incidence, enum SensorPosition position) {
    navcon->prev_colour = Green;
    if (incidence <= 5) {
        navcon->state = Forward;
        return;
    }
    else if (incidence < 45) {
        navcon->AOI_correction = (uint16_t)incidence;
    }
    else {
        navcon->AOI_correction = 5;
    }

    navcon->prev = Forward;
    navcon->state = Stop;

    switch (position) {
        case Left:
            navcon->next = RotateLeft;
            break;
        case Right:
            navcon->next = RotateRight;
            break;
    }
}

void red_encounter(struct NAVCON* navcon, uint8_t incidence, enum SensorPosition position) {
    navcon->prev_colour = Red;
    if (incidence <= 5) {
        if (navcon->first_red == Unseen) {
            switch (position) {
                case Left:
                    navcon->red_at_sensor = 4;
                    break;
                case Right:
                    navcon->red_at_sensor = 0;
                    break;
            }

            navcon->first_red = Seen;
        }
    }
    else {
        green_encounter(navcon, incidence, position);
    }
}

void blue_encounter(struct NAVCON* navcon, uint8_t incidence, enum SensorPosition position) {
    if (incidence <= 5) {
        navcon->AOI_correction = 0;
    }
    else if (incidence < 45) {
        navcon->AOI_correction = (uint16_t)incidence;
    }
    else {
        navcon->AOI_correction = 5;
    }

    navcon->prev = Forward;
    navcon->state = Stop;  

    switch (position) {
        case Left:
            navcon->next = RotateRight;
            navcon->AOI_correction = 90 - navcon->AOI_correction;
            break;
        case Right:
            navcon->next = RotateRight;
            navcon->AOI_correction += 90;
            break;
    }
    
    if (navcon->prev_colour == Blue) {
        switch (position) {
            case Left:
                navcon->next = RotateRight;
                break;
            case Right:
                navcon->next = RotateRight;
                break;
        }
        navcon->AOI_correction = 180 + (uint16_t)incidence;
    }
    
    if (incidence >= 45) {
        navcon->AOI_correction = 5;
    }
    
    navcon->prev_colour = Blue;
}

void greater_than_45(struct NAVCON* navcon, enum SensorPosition side){
    switch (navcon->colour) {
        case Green:
            green_encounter(navcon, 50, side);
            break;
        case Red:
            red_encounter(navcon, 50, side);
            break;
        default:
            blue_encounter(navcon, 50, side);
            break;
    }
}

void less_than_45(struct NAVCON* navcon, enum SensorPosition side, uint8_t incidence) {
    switch (navcon->colour){
        case Green:
            green_encounter(navcon, incidence, side);
            break;
        case Red:
            red_encounter(navcon, incidence, side);
            break;
        default:
            blue_encounter(navcon, incidence, side);
            break;
    }
}

void run_navcon(struct MDPS* motor_system, struct SS* sensor_system, struct NAVCON* navcon) {
    switch (navcon->state) {
        case Forward:
          if (navcon->outside_sensor == false && (sensor_system->sensor[0] != White || sensor_system->sensor[4] != White) && sensor_system->incidence == 0) {
                navcon->outside_sensor = true;
                navcon->reference_distance = motor_system->distance;
                if (sensor_system->sensor[0] != White) {
                    navcon->colour = sensor_system->sensor[0];
                    navcon->first_sensor_side = Left;
                }
                else {
                    navcon->colour = sensor_system->sensor[4];
                    navcon->first_sensor_side = Right;
                }
            }

            if (navcon->outside_sensor == true) {
                if ((motor_system->distance - navcon->reference_distance) > ISD) {
                    greater_than_45(navcon, navcon->first_sensor_side);
                    navcon->colour = White;
                    navcon->outside_sensor = false;
                }
                else if (sensor_system->sensor[1] != White || sensor_system->sensor[3] != White){
                    enum SensorPosition side;
                    if (sensor_system->sensor[1] != White) {
                        navcon->colour = sensor_system->sensor[1];
                        side = Left;
                    }
                    else if (sensor_system->sensor[3] != White) {
                        navcon->colour = sensor_system->sensor[3];
                        side = Right;
                    }
                    less_than_45(navcon, side, sensor_system->incidence);
                    navcon->colour = White;
                    navcon->outside_sensor = false;
                }
                 
            }
            if (navcon->first_red == Seen) {
                int red_count = 0;
                
                for (int i = 0; i < 5; i++) {
                    if (sensor_system->sensor[i] == Red) {
                        red_count++;
                    }
                }
                
                if (red_count == 0) {
                    navcon->state = MazeDone;
                }
            } 
            break;
        case Reverse:
            // until some distance covered, keep reversing....
            if (motor_system->distance < 10) {
                return;
            }

            navcon->prev = Reverse;
            navcon->state = Stop;

            break;
        case RotateLeft:
            if (motor_system->rotation < navcon->AOI_correction) {
                // if the rotation is still in progress, then keep rotating
                return;
            }

            navcon->colour = White;
            navcon->state = Forward;

            break;
        case RotateRight:
            if (motor_system->rotation < navcon->AOI_correction) {
                // if the rotation is still in progress, then keep rotating
                return;
            }

            navcon->colour = White;
            navcon->state = Forward;

            break;
        case Stop:
            if (navcon->prev == Forward) {
                navcon->state = Reverse;
                return;
            }

            navcon->state = navcon->next;

            break;
        case MazeDone:
            navcon->AOI_correction = 360;
            navcon->state = RotateRight;

            break;
    }
}

///--------------------------->> END of NAVCON <<--------------------------------///
///******************************************************************************///



///******************************************************************************///
///-------------------------------CAP TOUCH--------------------------------------///
///==============================================================================///

float calibrate_cap_touch(void) {
    // calibrate the cap touch threshold
    float sum = 0;
    int count = 0;
    
    for (int i = 0; i < (int)CAP_TOUCH_CALIBRATE_SAMPLES; i++) {
        // set pin to output
        pinMode(CAP_TOUCH_PIN, OUTPUT);
        
        // set pin to HIGH and let cap charge
        digitalWrite(CAP_TOUCH_PIN, HIGH);
        delayMicroseconds(50);
        
        // set pin to input
        pinMode(CAP_TOUCH_PIN, INPUT);
        
        // while the cap still discharging count 
        while (digitalRead(CAP_TOUCH_PIN) == 1) {
            count += 1;
        }
        
        // add count to sum
        sum += count;

        // reset count to zero
        count = 0;
    }
    
    // return the average sum without touch
    return (sum / CAP_TOUCH_CALIBRATE_SAMPLES);
}

bool run_touch(float threshold) {
    return true;
    float sum = 0;
    int count = 0;
    
    for (int i = 0; i < (int)CAP_TOUCH_SAMPLES; i++) {
        // set pin to output
        pinMode(CAP_TOUCH_PIN, OUTPUT);
        
        // set pin to HIGH and let cap charge
        digitalWrite(CAP_TOUCH_PIN, HIGH);
        delayMicroseconds(50);
        
        // set pin to input
        pinMode(CAP_TOUCH_PIN, INPUT);
        
        // while the cap still discharging count 
        while (digitalRead(CAP_TOUCH_PIN) == 1) {
            count += 1;
        }
        
        // add count to sum
        sum += count;

        // reset count to zero
        count = 0;
    }
        
    //return (sum / CAP_TOUCH_SAMPLES) > (threshold + 20.0000);
}


///------------------------->> END of CAP TOUCH <<-------------------------------///
///******************************************************************************///



///******************************************************************************///
///--------------------------CRITICAL DIAGNOSTICS--------------------------------///
///==============================================================================///

void display_critical_diagnostics()
{
  String str_state = "";
  
  switch (*state) {
    case Idle: 
      str_state = "STATE:IDLE      ";
      break;
    case Calibrate: 
      str_state = "STATE:CALIBRATE ";
      break;
    case Maze:
      str_state = "STATE:MAZE      "; 
      break;
    case Sos:
      str_state = "STATE:SOS       "; 
      break;
  } 

  char sensor_colours[5] = {'W', 'W', 'W', 'W', 'W'};

  for (int i = 0; i < 5; i++) {
    switch (sensorSystem->sensor[i]) {
      case Red: 
        sensor_colours[i] = 'R';
        break;
      case Green: 
        sensor_colours[i] = 'G';
        break;
      case Blue: 
        sensor_colours[i] = 'B';
        break;
      case Black: 
        sensor_colours[i] = 'K';
        break;
    }
  }

  lcd.setCursor(0,0);
  lcd.print(str_state);
  lcd.setCursor(0,1);
  lcd.print("AOI:");
  lcd.print(sensorSystem->incidence);
  
  if (sensorSystem->incidence < 10) {
    lcd.print("  ");
  }
  else if (sensorSystem->incidence < 100) {
    lcd.print(" ");
  }
  
  lcd.print(" SENSOR:");
  
  for (int i = 0; i < 5; i++) {lcd.print(sensor_colours[i]);}
  
  lcd.setCursor(0,2);
  
  lcd.print("DIST:");
  lcd.print(motorSystem->distance);
  lcd.print("   ");
  
  lcd.setCursor(0,3);
  lcd.print("SPEED:");
  lcd.print(motorSystem->left_wheel_speed);
  
  if (motorSystem->left_wheel_speed < 10) {
    lcd.print(" ");
  }
  lcd.print("  -  ");
  if (motorSystem->right_wheel_speed < 10) {
    lcd.print(" ");
  }
  
  lcd.print(motorSystem->right_wheel_speed);
}

///-------------------->> END of CRITICAL DIAGNOSTICS <<-------------------------///
///******************************************************************************///



///******************************************************************************///
///----------------------------------CLAP----------------------------------------///
///==============================================================================///

bool run_clap() {
  int value = 0;
  //value = analogRead(A0);

  if (value > 165) {
    return true;
  }

  return false;
}

///---------------------------->> END of CLAP <<---------------------------------///
///******************************************************************************///




///******************************************************************************///
///-----------------------------------SCS----------------------------------------///
///==============================================================================///

void send_packet(struct Packet p) {
    for (int i = 0; i < 4; i++) {
        Serial.write(p.bytes[i]);
        Serial.flush();
    }
}

struct Packet receive_packet(void) {
    struct Packet p;

    delayMicroseconds(200);
    
    for (int i = 0; i < 4; i++) {
      while (Serial.available() < 1) {}
      p.bytes[i] = Serial.read();
    }
    
    return p;
}

void reset_packet(struct Packet* p) {
  p->bytes[controlByte] = 0;
  p->bytes[dat1] = 0;
  p->bytes[dat0] = 0;
  p->bytes[dec] = 0;
}

///----------------------------->> END of SCS <<---------------------------------///
///******************************************************************************///




///******************************************************************************///
///-----------------------------------STATE--------------------------------------///
///==============================================================================///

void run_idle(enum States* state, struct NAVCON* navcon, float cap_threshold) {
    // create packet for start instruction from the HUB
    struct Packet in_packet = {{1,1,1,1}};
    
    struct Packet packet_out;
    reset_packet(&packet_out);
    //display_critical_diagnostics();
    // wait for | 0 | 0 | 0 | 0 | from the HUB
    lcd.setCursor(0,0);
    lcd.print("WAITING FOR HUB");
    while (in_packet.bytes[controlByte] != 0x00) {in_packet = receive_packet();} 

    navcon->state = Forward;
    navcon->next = Forward;
    navcon->prev = Forward;
    navcon->red_at_sensor = 255;
    navcon->first_red = Unseen;
    navcon->prev_colour = White;
    navcon->colour = White;
    navcon->outside_sensor = false;
    navcon->reference_distance = 200;
    navcon->AOI_correction = 0;
    navcon->blue_count = 0;

    packet_out.bytes[controlByte] = 16;
    
    send_packet(packet_out);

    display_critical_diagnostics();
    
    while (run_touch(cap_threshold) == false) {}
    // now send that there was a touch :)
    packet_out.bytes[dat1] = 1;
    packet_out.bytes[dat0] = 10;
    send_packet(packet_out);   

    // wait for user to stop touching
    while (run_touch(cap_threshold) == true) {}
    
    // head to calibrate state!
    reset_packet(&packet_out);
    *state = Calibrate; 
}

void run_calibrate(enum States* state, float cap_threshold){
    struct Packet packet_in;
    struct Packet packet_out;
    
    reset_packet(&packet_out);
    reset_packet(&packet_in);
    
    // wait for initial packets from SS & MDPS
    while (packet_in.bytes[controlByte] != 113 || packet_out.bytes[dat1] != 1){
        packet_in = receive_packet();
        
        packet_out.bytes[dat1] = run_touch(cap_threshold);

        switch (packet_in.bytes[controlByte])
        {
        case 113:
            packet_out.bytes[controlByte] = 80;
            send_packet(packet_out);
            break;
        
        default:
            break;
        }

    }

    while (run_touch(cap_threshold) == false) { /* wait for touch to stop */ }
    
    *state = Maze;
}

void run_maze(enum States* state, struct MDPS* motorSystem, struct SS* sensorSystem, struct NAVCON* navcon, float cap_threshold){
    struct Packet packet_in;
    struct Packet packet_out;
    // ALWAYS RESET PACKETS AT BEGINNING OF FUNCTION
    // the programmer keeps the memory, and does not reset it (i.e. it does not create a new struct when the function runs again)
    reset_packet(&packet_in);
    reset_packet(&packet_out);

    packet_out.bytes[controlByte] = 145;

    if (*clap == true) {
        // oh nooo! SOS!
        packet_out.bytes[dat1] = 1;
        send_packet(packet_out);
        *clap = false;
        *state = Sos;
        return;
    }
    
    send_packet(packet_out);

    reset_packet(&packet_out);
    packet_out.bytes[controlByte] = 146;
    
    if (run_touch(cap_threshold) == true) {
        *state = Idle;
        return;
    }
    
    send_packet(packet_out);
    
    // execute NAVCON :D
    run_navcon(motorSystem, sensorSystem, navcon);
    
    // will be outputting NAVCON data
    packet_out.bytes[controlByte] = 147;
    
    switch (navcon-> state) {
        case Forward: {
            packet_out.bytes[dec] = 0;
            packet_out.bytes[dat0] = 10;
            packet_out.bytes[dat1] = 10;
            
            break;
        }
        case Reverse: {
            packet_out.bytes[dec] = 1;
            packet_out.bytes[dat0] = 10;
            packet_out.bytes[dat1] = 10;
            
            break;
        }
        case RotateLeft: {
            packet_out.bytes[dec] = 2;
            packet_out.bytes[dat1] = (uint8_t)((navcon->AOI_correction & 0xFF00) >> 8);
            packet_out.bytes[dat0] = (uint8_t)(navcon->AOI_correction & 0x00FF);
            
            break;
        }
        case RotateRight: {
            packet_out.bytes[dec] = 3;
            packet_out.bytes[dat1] = (uint8_t)((navcon->AOI_correction & 0xFF00) >> 8);
            packet_out.bytes[dat0] = (uint8_t)(navcon->AOI_correction & 0x00FF);
            
            break;
        }
        case Stop: {
            /* no need to change the packet. i.e. MARV must stop */
        }
        case MazeDone: {
            /* no need to change the packet. i.e. MARV must stop */
        }
    }

    // send NAVCON data
    send_packet(packet_out);

    // to hold all incoming MARV data (4 packets from MDPS & 2 from SS)
    struct Packet packets[6];
    
    //Serial.write(161);
    // wait for level data from MDPS
    while (packet_in.bytes[controlByte] != 161 && packet_in.bytes[controlByte] != 1) {
        packet_in = receive_packet();
    }
    
    packets[0] = packet_in;
    struct Packet packet = {{1,1,1,1}};
    if (packets[0].bytes[controlByte] == 1) {
        
        *state = Idle;
        //send_packet(packet_in);
        return;
    }
    
    //Serial.write(162);
    // wait for rotation data from MDPS
    while (packet_in.bytes[controlByte] != 162) {
        packet_in = receive_packet();
    }
    
    packets[1] = packet_in;
    //Serial.write(163);
    // wait for speed data from MDPS
    while (packet_in.bytes[controlByte] != 163) {
        packet_in = receive_packet();
    }
    
    packets[2] = packet_in;
    //Serial.write(164);
    // wait for distance data from MDPS
    while (packet_in.bytes[controlByte] != 164) {
        packet_in = receive_packet();
    }
    
    packets[3] = packet_in;
    //Serial.write(177);
    // wait for colour data from SS
    while (packet_in.bytes[controlByte] != 177) {
        packet_in = receive_packet();
    }
    
    packets[4] = packet_in;
    //Serial.write(178);
    // wait for incidence data from SS
    while (packet_in.bytes[controlByte] != 178) {
        packet_in = receive_packet();
    }
    
    packets[5] = packet_in;
    
    
    // save MDPS Level
    motorSystem->level = packets[0].bytes[dat1];

    // save MDPS Rotation
    motorSystem->rotation = (uint16_t)((packets[1].bytes[dat1] << 8) + (packets[1].bytes[dat0]));

    // save MDPS Speed
    motorSystem->right_wheel_speed = packets[2].bytes[dat1];
    motorSystem->left_wheel_speed = packets[2].bytes[dat0];
    
    // save MDPS Distance
    motorSystem->distance = (uint16_t)((packets[3].bytes[dat1] << 8) + (packets[3].bytes[dat0])); 

    // save SS Colours OR End-Of-Maze
    if (packets[1].bytes[controlByte] == 179) {
        // end-of-maze
        *state = Idle;
        return;
    }
    
    uint16_t colours = ((uint16_t)((packets[4].bytes[dat1] << 8) + packets[4].bytes[dat0]));
    
    // beautiful bit shifting for sensor colours
    sensorSystem->sensor[0] = (enum Colours)((colours & 0b0111000000000000) >> 12);
    sensorSystem->sensor[1] = (enum Colours)((colours & 0b0000111000000000) >> 9);
    sensorSystem->sensor[2] = (enum Colours)((colours & 0b0000000111000000) >> 6);
    sensorSystem->sensor[3] = (enum Colours)((colours & 0b0000000000111000) >> 3);
    sensorSystem->sensor[4] = (enum Colours)((colours & 0b0000000000000111) >> 0); // shift 0 here for clarity (doesn't actually do anything)
    
    
    // save SS Incidence
    sensorSystem->incidence = packets[5].bytes[dat1];
}

void run_sos(enum States* state){
    struct Packet packet = {{1,1,1,1}};
    send_packet(packet);
}

///----------------------------->> END of STATE <<-------------------------------///
///******************************************************************************///

/// END OF FILE
