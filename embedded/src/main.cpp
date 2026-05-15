#include <Arduino.h>
#include "pinout_def.h"
#include "config.h"
#include "driver/adc.h"
#include "motor_control.h"
#include "pid_ctr.h"
#include "esp32-hal-timer.h"
#include "driver/mcpwm.h"
#include "driver/rmt.h"
#include "TelemetryData.h"
#include "GPSManager.h"
#include "SensorManager.h"
#include "RadioManager.h"

// ----------------------------------------------------
// TIMING & INTERRUPTS
// ----------------------------------------------------
// The Source of Truth
TelemetryData currentData;

// Timer Globals
hw_timer_t *timer = NULL;
volatile bool shouldLog = false;
unsigned long lastBroadcastTime = 0;

void IRAM_ATTR onTimer() {
    shouldLog = true;
}

// Run mode variables

STATE curr_state = IDLE;
float set_point = 0;
bool stop_flag = true;

// For encoder and PID controll

hw_timer_t *pid_timer = NULL;
volatile bool pid_flag = false;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile long enc_pulses = 0;
         long curr_pulses = 0;
         float dist = 0;
         float vlct = 0;
         float ctrl_sig = 0;
         float err = 0;
         bool  start = true;
mcpwm_config_t cfg ={
    .frequency = pwm_freq,
    .cmpr_a = 0,
    .cmpr_b = 0,
    .duty_mode = MCPWM_DUTY_MODE_0,
    .counter_mode = MCPWM_UP_DOWN_COUNTER
};

motor_pwm motor = {
    MCPWM_UNIT_0,
    MCPWM_TIMER_0,
    MCPWM0A,
    MCPWM0B,
    MCPWM_DUTY_MODE_0
};

PID PID_position(0.4,0.005,0);

// For Limit Switches

volatile unsigned long lastDebounceTimeUp = 0; 
volatile bool limitUpFlag = 0;
bool isPressed = 0;

// For input PWM
RingbufHandle_t rb = NULL;
volatile double fall_time = 0;
volatile double rise_time = 0;
volatile bool new_Pulse = false;
uint32_t RC_pwm_in= 0;


uint32_t getLowPulseWidth(rmt_item32_t* items, int count) {
  for (int i = 0; i < count; i++) {
    // Return first LOW found
    if (items[i].level0 == 0) return items[i].duration0;
    if (items[i].level1 == 0) return items[i].duration1;
  }
  return 0; // no LOW found
}

void IRAM_ATTR onLimitUp(){
  if(!limitUpFlag){
    lastDebounceTimeUp = millis();
    limitUpFlag = true;
  }
}

void IRAM_ATTR enc_phA() {
  bool B = GPIO.in & (1 << ph_B);
  enc_pulses += B ? 1 : -1;
}

/*
void IRAM_ATTR onTimer_PID(){
portENTER_CRITICAL_ISR(&timerMux);
  pid_flag = true;
portEXIT_CRITICAL_ISR(&timerMux);
}
*/

// ----------------------------------------------------
// TASK ON OTHER CORE
// ----------------------------------------------------

TaskHandle_t pidTaskHandle = NULL;
BaseType_t hpTaskWoken = pdFALSE;
void pidTask(void *param)
{
    for(;;){
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      dist = (enc_pulses)*dist_per_r/pls_per_r;
      //vlct = dist/dt * 1000; //velocity per second
      curr_pulses = enc_pulses;
      err = set_point - dist;
      if (curr_state == RUN || curr_state == MEASURE) {
          ctrl_sig = PID_position.update(set_point, dist);

          if (motor_is_armed()) {
              update_speed(motor, ctrl_sig);
          } else {
              motor_stop(motor);
          }
      }
      else if (curr_state == HOMING) {
          // HOMING steruje silnikiem ręcznie w loop(), PID nie przeszkadza.
          ctrl_sig = 0;
      }
      else {
          ctrl_sig = 0;
          motor_stop(motor);
      }
    }
}

void IRAM_ATTR onTimer_PID(){
  BaseType_t hpTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(pidTaskHandle, &hpTaskWoken);
  if(hpTaskWoken){
    portYIELD_FROM_ISR();
  }
}
// ----------------------------------------------------
// MAIN SETUP
// ----------------------------------------------------


void setup() {

    // Initialize PWM

    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_RX;
    config.channel = RMT_CHANNEL_0;
    config.gpio_num = (gpio_num_t)PWM_in;
    config.mem_block_num = 1;
    config.clk_div = 80;
    config.rx_config.filter_en = true;
    config.rx_config.filter_ticks_thresh = 100;
    config.rx_config.idle_threshold = 4000;
    rmt_config(&config);
    rmt_driver_install(config.channel, 1024, 0);
    rmt_get_ringbuf_handle(config.channel, &rb);
    rmt_rx_start(config.channel, true);

    // TIMER FOR PID THAT TRIGGERS EVERY dt ms

    xTaskCreatePinnedToCore(
      pidTask,
      "PID Task",
      4096,
      NULL,
      configMAX_PRIORITIES - 1,
      &pidTaskHandle,
      0
    );
    pid_timer = timerBegin(3, 80, true);
    timerAttachInterrupt(pid_timer, &onTimer_PID, true);
    timerAlarmWrite(pid_timer, dt * 1000, true);
    timerAlarmEnable(pid_timer);
    Serial.begin(115200);
   //DC Motor driver 

   //stare wywołania:
    //pinMode(R_en, OUTPUT);
    //pinMode(L_en, OUTPUT);
    //digitalWrite(R_en, HIGH);
    //digitalWrite(L_en, HIGH);

    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, R_s);
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, L_s);
    //mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0 , &cfg);

    //mcpwm_gpio_init(motor.unit, motor.sigA, R_s);
    //mcpwm_gpio_init(motor.unit, motor.sigB, L_s);
    //mcpwm_init(motor.unit, motor.timer, &cfg);
//nowe wywołanie:
    init_motor_safe(motor, cfg);

    //Encoder
   // ROZWIJANIE TO DODAWANIE
   // ZWIJANIE TO ODEJMOWANIE
   // l channel to zwijanie
   // r channel to rozwijanie
   // pinMode(PWM_in, INPUT);
   // attachInterrupt(digitalPinToInterrupt(PWM_in), input_width, CHANGE);

    pinMode(ph_A, INPUT);
    pinMode(ph_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ph_A), enc_phA, RISING);
    
    pinMode(L_up, INPUT_PULLUP);
    pinMode(L_dwn, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(L_up), onLimitUp, RISING);

    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);

    if(!(GPIO.in & (1 << L_up))){
          isPressed = 1;
    }
    
    initGPS();
    initSensor();
    initRadio();
    
    delay(1000);
}

void loop() {
/*
  if(new_Pulse){
    RC_pwm_in = rise_time - fall_time;
    new_Pulse = !new_Pulse;
  }
  */
 size_t rx_size = 0;
 rmt_item32_t *items =
      (rmt_item32_t *)xRingbufferReceive(rb, &rx_size, 0);

 if(items){
    int item_count = rx_size / sizeof(rmt_item32_t);
    uint32_t low_pulse = getLowPulseWidth(items, item_count);
    vRingbufferReturnItem(rb, (void *)items);
    RC_pwm_in = low_pulse;
 }


  if(limitUpFlag){
      if(millis() -  lastDebounceTimeUp > dbnc_time){
          if(GPIO.in & (1 << L_up)){
              isPressed = 0;
          }
          else{    
              isPressed = 1;
          }
        
          limitUpFlag = 0;
      }
    }
/*
  if(pid_flag){
      dist = (enc_pulses)*dist_per_r/pls_per_r;
      //vlct = dist/dt * 1000; //velocity per second
      curr_pulses = enc_pulses;
      err = set_point - dist;
      ctrl_sig = PID_position.update(set_point, dist);
      
      if(curr_state != START){
        update_speed(motor, ctrl_sig);
      }
      pid_flag = !pid_flag;
    }
*/
switch (curr_state) {

    case IDLE:
        motor_arm(motor, false);
        PID_position.reset();

        set_point = dist;
        ctrl_sig = 0;
        stop_flag = true;

        // start po komendzie z RC
        // wtedy dojście do górnej końcówki
        if (RC_pwm_in >= 1900 && RC_pwm_in < 2500) {
            motor_arm(motor, true);
            PID_position.reset();
            curr_state = HOMING;
        }

    break;

    case HOMING:
        // pozycja domowa - zwijak zwinięty
        if (isPressed == 0) {
            update_speed(motor, -70);
        }
        else {
            motor_stop(motor);
            motor_arm(motor, false);
            PID_position.reset();

            enc_pulses = 0;
            dist = 0;
            set_point = 0;
            ctrl_sig = 0;
            stop_flag = true;

            motor_arm(motor, true);
            PID_position.reset();

            curr_state = RUN;
        }

    break;

    case RUN:

        if (RC_pwm_in >= 700 && RC_pwm_in < 1000) {
            set_point = 2500;
            stop_flag = true;

            if (currentData.isWaterDetected) {
                curr_state = MEASURE;
            }
        }
        else if (RC_pwm_in >= 1200 && RC_pwm_in < 1700) {
            if (stop_flag) {
                set_point = dist;
                stop_flag = false;
                PID_position.reset();
            }
        }
        else if (RC_pwm_in >= 1900 && RC_pwm_in < 2500) {
            stop_flag = true;
            set_point = 0;

            if (isPressed) {
                enc_pulses = 0;
                dist = 0;
            }
        }

    break;

    case MEASURE:

        if (stop_flag) {
            set_point = dist;
            ctrl_sig = 0;
            stop_flag = false;
            PID_position.reset();
        }

        if (currentData.isMeasurementFinished) {
            set_point = dist;
        }

        if (RC_pwm_in >= 1900 && RC_pwm_in < 2500) {
            PID_position.reset();
            stop_flag = true;
            curr_state = HOMING;
        }

    break;

    default:
        motor_arm(motor, false);
        PID_position.reset();
        curr_state = IDLE;
    break;
}

    if(isPressed){
        if(GPIO.in & (1 << L_up))
          isPressed = !isPressed;

        if (ctrl_sig < 0) {
            motor_stop(motor);
            PID_position.reset();
            set_point = dist;
}
    }
  /*
    Serial.print(">pos:");
    Serial.println(dist);
    Serial.print(">Err:");
    Serial.println(err);
    Serial.print(">sf:");
    Serial.println(stop_flag);
    Serial.print(">Sp:");
    Serial.println(set_point);
      //ledcWrite(r_channel,0);
      //ledcWrite(l_channel, 500);
    */
    // ------------------------------------------------------------
    // SENSOR & TELEMETRY LOGIC
    // ------------------------------------------------------------
    
    // 1. Run the Sensor State Machine (Non-blocking)
    // CX-461 returns all parameters in one response: Request -> Wait -> Parse
    runSensorLogic(currentData);

    // 2. Broadcast Data / GPS Update (Triggered by Timer)
    if (millis() - lastBroadcastTime >= BROADCAST_INTERVAL_MS) {
        Serial.println("Water Detected: " + String(currentData.isWaterDetected));
        Serial.println("Measurement Finished: " + String(currentData.isMeasurementFinished));
        Serial.println("Current State: " + String(curr_state) + " | Set Point: " + String(set_point) + " | Distance: " + String(dist));
        updateGPS(currentData);
        broadcastData(currentData);
        lastBroadcastTime = millis();
    }

   delay(1);
}