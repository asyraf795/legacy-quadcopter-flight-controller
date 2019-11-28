#include <Wire.h>
#include "config.h"
#include <PIDCntrl.h>

int channel_receiver_pulse[9]; 
int esc_pulse[4];
unsigned long esc_timer[4], esc_ends_timer;
int channel_receiver_counter;
unsigned long channel_starts_timer, channel_ends_timer;
int throttle, battery_voltage;
int cal_counter;
unsigned long refresh_rate_timer;
bool start = false;
boolean auto_level = true;
float angular_motion[3];
float gravity_accel[3];
float angle_acc[2];
float angle[3];
float level_adjust[2];
float pid_output[TOTAL_PID]; 
static float mpu6050_value[7];

PIDCntrl pidRoll, pidYaw, pidPitch;

void setup() {
  
  start_fast_iic();

  port_init();

  led_status(1);

  setup_mpu6050();
  
  interrupt_init();

  rx_init();

  callibrate_mpu6050();
  
  battery_voltage = read_battery_voltage(1.2317f);

  pidRoll.SetParameters(1.2f, 0.03f, 14.5f, MAX);
  pidPitch.SetParameters(1.2f, 0.03f, 14.5f, MAX);
  pidYaw.SetParameters(4.0f, 0.02f, 0.0f, MAX);

  retrieve_mpu6050();
  
  refresh_rate_timer = micros();
  
  led_status(0);
  
}

void loop() {

  angular_motion[ROLL] = (angular_motion[ROLL] * 0.7) + (mpu6050_value[GX] * 0.3);   
  angular_motion[PITCH] = (angular_motion[PITCH] * 0.7) + (mpu6050_value[GY] * 0.3);  
  angular_motion[YAW] = (angular_motion[YAW] * 0.7) + (mpu6050_value[GZ] * 0.3); 

  gravity_accel[ROLL] = (gravity_accel[ROLL] * 0.7) + (mpu6050_value[AX] * 0.3);   
  gravity_accel[PITCH] = (gravity_accel[PITCH] * 0.7) + (mpu6050_value[AY] * 0.3);  
  gravity_accel[YAW] = (gravity_accel[YAW] * 0.7) + (mpu6050_value[AZ] * 0.3); 
  imu();

  level_adjust[ROLL] = angle[ROLL] * 15;    
  level_adjust[PITCH] = angle[PITCH] * 15; 

  if(channel_receiver_pulse[THREED] > 1500){
                                                              
    level_adjust[ROLL] = 0;  
    level_adjust[PITCH] = 0;   
                                          
  }

 if(start == false && channel_receiver_pulse[START] == ON && channel_receiver_pulse[THROTTLE] <= 1008) {    

    pidRoll.ResetMem();
    pidPitch.ResetMem();
    pidYaw.ResetMem();
    
    start = true;

    angle[ROLL] = angle_acc[ROLL];
    angle[PITCH] = angle_acc[PITCH];

  } else if (channel_receiver_pulse[START] == OFF && channel_receiver_pulse[THROTTLE] <= 1008) {
    
    start = false;

  }
  
  float setpoint = setpoint_calculation(channel_receiver_pulse[AILERONS], level_adjust[ROLL]);
  pid_output[ROLL] = pidRoll.Calculate(angular_motion[ROLL], setpoint);
  
  setpoint = setpoint_calculation(channel_receiver_pulse[ELEVATOR], level_adjust[PITCH]);
  pid_output[PITCH] = pidPitch.Calculate(angular_motion[PITCH], setpoint);

  setpoint = 0.0f;
  if(channel_receiver_pulse[THROTTLE] > 1048) {
      setpoint = setpoint_calculation(channel_receiver_pulse[RUDDER], 0.0f);
  }
  pid_output[YAW] = pidYaw.Calculate(angular_motion[YAW], setpoint);

  battery_voltage = battery_voltage * 0.92 + read_battery_voltage(0.09853f);
   
  if(battery_voltage < 1050 && battery_voltage > 600){
    
    led_status(1);
    
  } 
  if(start == true) {
    
    output_calculation();
    
  } else if (start == false) {

    esc_pulse[0] = 1000;                                                         
    esc_pulse[1] = 1000;                                                          
    esc_pulse[2] = 1000;                                                         
    esc_pulse[3] = 1000; 
    
  }

  while(micros() - refresh_rate_timer < 5000);
  refresh_rate_timer = micros();                                                    
  
  PORTD |= B11110000;         
  esc_timer[0] = esc_pulse[0] + refresh_rate_timer;                                    
  esc_timer[1] = esc_pulse[1] + refresh_rate_timer;                                    
  esc_timer[2] = esc_pulse[2] + refresh_rate_timer;                                     
  esc_timer[3] = esc_pulse[3] + refresh_rate_timer;                                    

  retrieve_mpu6050();

  while(PORTD >= 16){                                                     

    esc_ends_timer = micros();                                            
    if(esc_timer[0] <= esc_ends_timer)PORTD &= B11101111;                
    if(esc_timer[1] <= esc_ends_timer)PORTD &= B11011111;              
    if(esc_timer[2] <= esc_ends_timer)PORTD &= B10111111;               
    if(esc_timer[3] <= esc_ends_timer)PORTD &= B01111111;               

  }

}

void iic_connection() {
  
  Wire.begin();
  
}

void iic_update_rate(long cpu_frequency) {

  TWBR = ((cpu_frequency/400000L) -16)/2;
  
}

void start_fast_iic() {

  iic_connection();

  iic_update_rate(CPU_FREQUENCY);
  
}

void port_init() {

  DDRD |= B11110000;
  DDRB |= B00100000;  
  
}

void led_status(byte stat) {

  digitalWrite(LED_BUILTIN, stat);
 
}

void setup_mpu6050() {
    
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00); 
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x19);
  Wire.write(109);                                            
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                  
  Wire.write(0x18);                                                 
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10); 
  Wire.endTransmission();
  
}

void interrupt_init() {

  PCICR |= (1 << PCIE0);                                      
  PCMSK0 |= (1 << PCINT0); 

}

ISR(PCINT0_vect){
  
  if(PINB & B00000001) {
    
    channel_ends_timer = micros();
    channel_receiver_pulse[channel_receiver_counter] = channel_ends_timer - channel_starts_timer;
    channel_receiver_counter++; 
    
    if (channel_receiver_counter == 9 || channel_receiver_pulse[channel_receiver_counter-1] > 3000) {
      
      channel_receiver_counter = 0;    
      
    }
    
    channel_starts_timer = channel_ends_timer;

  }
  
}

void motor_arm() {
  
  PORTD |= B11110000;                                       
  delayMicroseconds(1000);                                 
  PORTD &= B00001111;  
   
}

void rx_init() {

    while(channel_receiver_pulse[THROTTLE] > 1008 || channel_receiver_pulse[THROTTLE] < 992 ){

      motor_arm();
    
      led_blink(60);
                          
  }
  
}

void led_blink(int count) {

  static int counter = 0;
  
  if(counter % count == 0) digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  counter++;
  
}

float read_battery_voltage(float filter) {

  return (analogRead(BATTERY_ANALOG_INPUT) + 65) * filter; 
  
}

void callibrate_mpu6050() {

  for(cal_counter = 0 ; cal_counter < 2065 ; cal_counter++) {

    led_blink(15);

    retrieve_mpu6050();
    
    motor_arm();
    
  }
  
}


void retrieve_mpu6050() {

  static long gyro_cal[3];
  static int samples = 0;
  int16_t mpu6050_raw[7];
  
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x3B);                                                    
  Wire.endTransmission(); 
                                                
  Wire.requestFrom(0x68, 14);                                          
  while(Wire.available() < 14);  
                                        
  mpu6050_raw[AX] = Wire.read()<<8|Wire.read();                                  
  mpu6050_raw[AY] = Wire.read()<<8|Wire.read();                                  
  mpu6050_raw[AZ] = Wire.read()<<8|Wire.read();    
                               
  mpu6050_raw[TEMPERATURE] = Wire.read()<<8|Wire.read();
                           
  mpu6050_raw[GX] = Wire.read()<<8|Wire.read();                                 
  mpu6050_raw[GY] = Wire.read()<<8|Wire.read();                                 
  mpu6050_raw[GZ] = Wire.read()<<8|Wire.read();     

  if(samples < 64) {
    
    samples++;
    
  } else if(samples < 2064) {
    
    gyro_cal[X] += mpu6050_raw[GX];
    gyro_cal[Y] += mpu6050_raw[GY];
    gyro_cal[Z] += mpu6050_raw[GZ];
    samples++;

  } else if(samples == 2064) {
    
    gyro_cal[X] /= 2000;
    gyro_cal[Y] /= 2000;
    gyro_cal[Z] /= 2000;
    samples++;
    
  } 

    mpu6050_value[7];


    mpu6050_value[AX] = (float)mpu6050_raw[AX] * RANGE_PER_DIGIT * EARTH_GRAVITY;
    mpu6050_value[AY] = (float)mpu6050_raw[AY] * RANGE_PER_DIGIT * EARTH_GRAVITY;;  
    mpu6050_value[AZ] = (float)-mpu6050_raw[AZ] * RANGE_PER_DIGIT * EARTH_GRAVITY;;

    mpu6050_value[TEMPERATURE] = (float)mpu6050_raw[TEMPERATURE] / 340.0f + 36.53f ;

    mpu6050_value[GX] = ((float)mpu6050_raw[GX] - gyro_cal[0]) / 16.4f;
    mpu6050_value[GY] = ((float)mpu6050_raw[GY] - gyro_cal[1]) / 16.4f;  
    mpu6050_value[GZ] = -((float)mpu6050_raw[GZ] - gyro_cal[2]) / 16.4f;
  
    mpu6050_value;
  
}

void imu() {
  
  angle_acc[ROLL] = atan2(gravity_accel[PITCH], sqrt(gravity_accel[ROLL]* gravity_accel[ROLL] + gravity_accel[YAW] * gravity_accel[YAW]));
  angle_acc[PITCH] = atan2(-gravity_accel[ROLL], sqrt(gravity_accel[PITCH]* gravity_accel[PITCH] + gravity_accel[YAW] * gravity_accel[YAW]));

  angle_acc[ROLL] *= (180 / M_PI); 
  angle_acc[PITCH] *= (180 / M_PI);

  angle_acc[ROLL] -= ACC_ANGLE_ROLL_OFFSET; 
  angle_acc[PITCH] -= ACC_ANGLE_PITCH_OFFSET;
  
  angle[ROLL] += (angular_motion[ROLL] * 0.005);
  angle[PITCH] += (angular_motion[PITCH]* 0.005);

  angle[ROLL] += angle[PITCH] * sin(angular_motion[YAW] * M_PI / 180 * 0.005);                  
  angle[PITCH] -= angle[ROLL] * sin(angular_motion[YAW] * M_PI / 180 * 0.005);                  

  angle[ROLL] = angle[ROLL] * 0.996 + angle_acc[ROLL] * 0.004;            
  angle[PITCH] = angle[PITCH] * 0.996 + angle_acc[PITCH] * 0.004;               
 
}


float setpoint_calculation(int channel_receiver_pulse, float level_adjust) {

  if(channel_receiver_pulse < 1000) {
    channel_receiver_pulse = 1000;
  } else if(channel_receiver_pulse > 2000) {
    channel_receiver_pulse = 2000;
  }
//  if(channel_receiver_pulse < 1492 || channel_receiver_pulse > 1508)
    float setpoint_calculated = ((float)channel_receiver_pulse - 1000.0f - level_adjust) * 328.0f / 1000.0f - 164.0f;
    if (setpoint_calculated > 164.0f) return 164.0f;
    else if (setpoint_calculated < -164.0f) return -164.0f;

    return setpoint_calculated;
//  return 0.0;

}

void output_calculation() {
  
  throttle = channel_receiver_pulse[THROTTLE];

  if (throttle > 1800) throttle = 1800; 
  esc_pulse[0] = throttle - pid_output[ROLL] - pid_output[PITCH] + pid_output[YAW];
  esc_pulse[1] = throttle - pid_output[ROLL] + pid_output[PITCH] - pid_output[YAW]; 
  esc_pulse[2] = throttle + pid_output[ROLL] + pid_output[PITCH] + pid_output[YAW]; 
  esc_pulse[3] = throttle + pid_output[ROLL] - pid_output[PITCH] - pid_output[YAW];     
    
  if (battery_voltage < 1240 && battery_voltage > 800){              
      
    esc_pulse[0] += esc_pulse[0] * ((1240 - battery_voltage)/(float)3500);           
    esc_pulse[1] += esc_pulse[1] * ((1240 - battery_voltage)/(float)3500);              
    esc_pulse[2] += esc_pulse[2] * ((1240 - battery_voltage)/(float)3500);              
    esc_pulse[3] += esc_pulse[3] * ((1240 - battery_voltage)/(float)3500);           
    
  } 

  for(int i = 0; i < 4; i++) {
    
    if(esc_pulse[i] < 1100) {

      esc_pulse[i] = 1100;
        
    } else if(esc_pulse[i] > 2000) {

      esc_pulse[i] = 2000;
    }
  }

}

