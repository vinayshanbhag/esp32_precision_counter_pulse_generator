/* Frequency Counter (8-9 digit precision) using reciprocal counting method
   Vinay Shanbhag

   Components:
    - Wemos (LOLIN) S2 mini
    - Dual D flip flop 74LVT74 (or 74LS74 with TTL level shifter TXS0108E)
    - rotary encoder (KY 040) for setting gate time (1ms - 10s)
    - 16x2 LCD display

   Theory of operation:
   A gating signal is produced on GATING_SIGNAL_OUT pin on the ESP32 and is approximately 
   equal to the selected gate time (MIN_GATE_TIME x user selected gate_time_multiplier).
   The gating signal from the ESP32 is wired into the D-flip flops, to produce a pcnt control signal
   perfectly aligned with the input signal(f_in), such that counting is enabled from the first 
   rising edge of f_in to the first rising edge of f_in after the gate time.

                                        |‾‾‾‾‾‾‾|     |‾‾‾‾‾‾‾|
      GATING_SIGNAL_OUT (ESP32) >>------|D     Q|-----|D     Q|--|-->> GATING_SIGNAL_IN   (ESP32)
                           f_in >>---+--|>clk   |  +--|>clk   |  |-->> PCNT_INPUT_CTRL_IO (ESP32)
                                     |  |_______|  |  |_______|
                                     +-------------+---------------->> PCNT_INPUT_SIG_IO1 (ESP32)

   REFERENCE_SIGNAL_OUT (ESP32) >>---------------------------------->> PCNT_INPUT_SIG_IO0 (ESP32)                                                    

  The counter counts the pulses in f_in and an internal reference (20Mhz) during this time.
  The frequency of f_in is given by n_in/n_ref * f_ref where,
  n_in = number of pulses from input signal f_in during the gate time
  n_ref = number of pulses from reference signal during the gate time
  f_ref = internal reference frequency.

  Compared to a direct counter, this method gives 8 digit resolution at all frequencies in 1s gate time. 
  
  Internal reference frequency of approximately 20Mhz is generated using ESP32 RMT.      
  Exact value of this frequency must be measured with a calibrated meter and entered in reference variable.
  2 PCNT counters are used - 
  - counter 0 counts the internal reference connected to PCNT_INPUT_SIG_IO0
  - counter 1 counts the input signal connected to PCNT_INPUT_SIG_IO1
  Both counters are set to count up when GATING_SIGNAL_IN is high.
  When GATING_SIGNAL_IN is low, counting is paused, frequency is calculated and the counters are reset.
  The counters are 16bit hence cannot go beyond 32767. noverflow0 and noverflow1, count the overflows on both counters.
  The total count for a counter is given by noverflow*PCNT_H_LIM_VAL + count. 
  PCNT_H_LIM_VAL is set to 20000(doesn't seem to work well at higher values)

  Rotary encoder controls the gate time in discrete steps (1,5,10,50,100,500,1000,2000,5000,10000ms). Defaults to 1000ms
                            GND >>---------------------------------->> GND (ESP32)
                            +   >>---------------------------------->> 3V3 (ESP32)
                            SW  >>---------------------------------->> GPIO37 (ESP32)
                            DT  >>---------------------------------->> GPIO33 (ESP32) 
                            CLK >>---------------------------------->> GPIO35 (ESP32)    
*/
#include <stdio.h>
#include "driver/pcnt.h"
#include "soc/pcnt_reg.h"
#include "soc/pcnt_struct.h"
#include "esp_timer.h"
#include "AiEsp32RotaryEncoder.h"

#define PCNT_H_LIM_VAL            20000       // counter overflow limit. max 32767, but higher values don't seem to work reliably

#define PCNT_TEST_UNIT0           PCNT_UNIT_0
#define PCNT_INPUT_SIG_IO0        GPIO_NUM_18  /* Pulse Input GPIO  */
#define PCNT_INPUT_CTRL_IO        GPIO_NUM_3
#define PCNT_TEST_UNIT1           PCNT_UNIT_1
#define PCNT_INPUT_SIG_IO1        GPIO_NUM_9  /* Pulse Input GPIO  */
#define GATING_SIGNAL_IN          GPIO_NUM_5
#define GATING_SIGNAL_OUT         GPIO_NUM_7
#define REFERENCE_SIGNAL_OUT      GPIO_NUM_16
#define MIN_GATE_TIME             1000

#define ROTARY_ENCODER_A_PIN 35  // CLK
#define ROTARY_ENCODER_B_PIN 33  // DT  
#define ROTARY_ENCODER_BUTTON_PIN GPIO_NUM_37 // SW
#define ROTARY_ENCODER_VCC_PIN -1 /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */
#define ROTARY_ENCODER_STEPS 4

int gate_time_multiplier=1000;
int gt_multipliers[10] = {1,5,10,50,100,500,1000,2000,5000,10000};

//xQueueHandle pcnt_evt_queue;  /*A queue to handle pulse counter event*/

rmt_data_t bit_data[60];
rmt_obj_t* rmt_send = NULL;

typedef struct {
    int unit;        /*pulse counter unit*/
    uint32_t status; /*pulse counter internal status*/
} pcnt_evt_t;

typedef struct {
    uint32_t count0;
    uint32_t noverflow0;
    uint32_t count1;
    uint32_t noverflow1; 
} reading_t;

reading_t reading;

uint32_t noverflow0 = 0;
uint32_t noverflow1 = 0;
int16_t  count0     = 0;
int16_t  count1     = 0;
float    reference  = 19837298.2;//measured on hp8316B
int      precision  = 0;

bool last = false;
bool ready = false;

esp_timer_create_args_t timer_args;
esp_timer_handle_t timer;

// pcnt_evt_t evt;
// portBASE_TYPE res;

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
RTC_DATA_ATTR int encoderValue = 6;

static void timer_callback(void* arg);

/* interrupt handler called when the gating signal (from the D flip flops) falls
 */
void ARDUINO_ISR_ATTR isr() {
  // Pause counting
  pcnt_counter_pause(PCNT_TEST_UNIT0);
  pcnt_counter_pause(PCNT_TEST_UNIT1);
  // retrieve counter values
  pcnt_get_counter_value(PCNT_TEST_UNIT0, &count0);
  pcnt_get_counter_value(PCNT_TEST_UNIT1, &count1);
  // save count and overflow values
  reading.count0 = count0;
  reading.count1 = count1;
  reading.noverflow0 = noverflow0;
  reading.noverflow1 = noverflow1;
  
  // reset overflow variable to zero
  noverflow0=0;
  noverflow1=0;
  // clear the counters
  pcnt_counter_clear(PCNT_TEST_UNIT0);
  pcnt_counter_clear(PCNT_TEST_UNIT1);
  // resume (counting will resume when control signal is high)
  pcnt_counter_resume(PCNT_TEST_UNIT0);
  pcnt_counter_resume(PCNT_TEST_UNIT1);
  ready = true;
}

/* Start a 20Mhz reference signal at specified pin (doesn't seem to go beyond 20Mhz reliably)
 */
void start_ref_clock(gpio_num_t pin, uint32_t clk_mult, uint32_t cycles_on, uint32_t cycles_off){
  if ((rmt_send = rmtInit(pin, RMT_TX_MODE, RMT_MEM_64)) == NULL)
  {
      Serial.println("init sender failed\n");
  }
  float realTick = rmtSetTick(rmt_send, 12.5*clk_mult);
  // Send the data
  for (int i=0; i < 60; i++){
    bit_data[i].level0=1;
    bit_data[i].duration0=cycles_on;
    bit_data[i].level1=0;
    bit_data[i].duration1=cycles_off;
  }

  rmtLoop(rmt_send, bit_data, 60);
  Serial.printf("%0.0fMHz reference signal at pin %d\n", 1000/(12.5*clk_mult*(cycles_on+cycles_off)), pin);
}

/* Start a timer to create a gating signal at GATING_SIGNAL_OUT
   Period is set by MIN_GATE_TIME x gate_time_multiplier
 */
void start_gating_timer(){
  timer_args.callback = timer_callback;
  
  esp_timer_create(&timer_args, &timer);
  ESP_ERROR_CHECK(esp_timer_start_once(timer, MIN_GATE_TIME*gate_time_multiplier));
  gpio_pad_select_gpio(GATING_SIGNAL_OUT);                              
  gpio_set_direction(GATING_SIGNAL_OUT, GPIO_MODE_OUTPUT);
}

/* Timer callback for gating signal. Toggles the GATING_SIGNAL_OUT pin high/low
*/
static void timer_callback(void* arg)
{
    last = !last;
    gpio_set_level(GATING_SIGNAL_OUT, last);
    ESP_ERROR_CHECK(esp_timer_start_once(timer, MIN_GATE_TIME*gate_time_multiplier+random(500)));
}

/* Interrrupt handler for PCNT events. In this case only looking for event when the counter(s) overflow
   When there is an overflow increment the corresponding overflow variable.
*/
void IRAM_ATTR pcnt_intr_handler(void* arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    // pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;
    for(i = 0; i < PCNT_UNIT_MAX; i++) {
        if(intr_status & (BIT(i))) {
            // evt.unit = i;
            // evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            /*H LIM EVT*/
            if((i==0) && PCNT.status_unit[i].cnt_thr_h_lim_lat_un) { // looked up from header files, does not match the ESP documentation
                noverflow0++;
            }
            if((i==1) && PCNT.status_unit[i].cnt_thr_h_lim_lat_un) { // looked up from header files, does not match the ESP documentation
                noverflow1++;
            }
            // xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            // if(HPTaskAwoken == pdTRUE) {
            //     portYIELD_FROM_ISR();
            // }
        }
    }
}
/* Initialize a counter 
*/
static void init_counter(pcnt_unit_t unit, pcnt_channel_t channel, gpio_num_t signal_in, gpio_num_t control_in, void (*handler)(void *)){
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = signal_in;
  pcnt_config.ctrl_gpio_num = control_in;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = unit;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DIS;      //keep the counter value
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;      //when control signal is high,keep the primary counter mode
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
  pcnt_unit_config(&pcnt_config);
  pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_isr_register(handler, NULL, 0, NULL);
  pcnt_intr_enable(unit);
  pcnt_counter_resume(unit);
}

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

void setup()
{
    Serial.begin(115200);
    delay(3000);
    Serial.printf("Begin\n");
    /*Init PCNT event queue */
    // pcnt_evt_queue = xQueueCreate(50, sizeof(pcnt_evt_t));
    
    // Initialize rotary encoder
    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    bool circleValues = false;
    rotaryEncoder.setBoundaries(0, 9, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
    rotaryEncoder.setEncoderValue(encoderValue);
    encoderValue = rotaryEncoder.readEncoder();

    // Start reference clock
    start_ref_clock(REFERENCE_SIGNAL_OUT,1,1,3);   
     
    // Initialize reference counter
    init_counter(PCNT_TEST_UNIT0, PCNT_CHANNEL_0, PCNT_INPUT_SIG_IO0, PCNT_INPUT_CTRL_IO, pcnt_intr_handler);
    // Initialize input signal counter
    init_counter(PCNT_TEST_UNIT1, PCNT_CHANNEL_0, PCNT_INPUT_SIG_IO1, PCNT_INPUT_CTRL_IO, pcnt_intr_handler);

    // start gating timer
    start_gating_timer();

    // Configure gating signal input
    gpio_pad_select_gpio(GATING_SIGNAL_IN);                              
    gpio_set_direction(GATING_SIGNAL_IN, GPIO_MODE_INPUT);
    // Attach an interrupt that fires when gating signal falls 
    attachInterrupt(digitalPinToInterrupt(GATING_SIGNAL_IN), isr, FALLING);
}

void loop(){
  if (ready){
    int n_ref = ((reading.noverflow0*PCNT_H_LIM_VAL)+reading.count0);
    int n_sig = ((reading.noverflow1*PCNT_H_LIM_VAL)+reading.count1);

    float frequency = float(n_sig)/float(n_ref)*reference;
    int precision = int(log10(n_ref)+1) - int(log10(frequency)+1);
    precision = precision<0?0:precision;
    Serial.printf("Frequency: %0.*fHz, Gate Time: %0.3fs\n", precision,frequency, gate_time_multiplier*MIN_GATE_TIME/1000000.0);
    ready = false;
  }

  if (rotaryEncoder.encoderChanged()) // if encoder value changed
  {
    encoderValue = rotaryEncoder.readEncoder();
    gate_time_multiplier = gt_multipliers[encoderValue];
  }
  if (rotaryEncoder.isEncoderButtonClicked())
  {
    rotaryEncoder.setEncoderValue(6);
    encoderValue = rotaryEncoder.readEncoder();
    gate_time_multiplier = gt_multipliers[encoderValue];
  } 
}