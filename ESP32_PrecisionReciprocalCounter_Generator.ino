/* Frequency Counter and square wave generator. Reciprocal counter with 8-9 digit precision
   Vinay Shanbhag

   Components:
    - Wemos (LOLIN) S2 mini
    - Dual D flip flop 74LVT74 (or 74LS74 with TTL level shifter TXS0108E)
    - rotary encoder (KY 040) for setting square wave frequency
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
  
  Internal reference frequency of approximately 10Mhz is generated using ESP32 RMT.      
  Exact value of this frequency must be measured with a calibrated meter and entered in reference variable.
  2 PCNT counters are used - 
  - counter 0 counts the internal reference connected to PCNT_INPUT_SIG_IO0
  - counter 1 counts the input signal connected to PCNT_INPUT_SIG_IO1
  Both counters are set to count up when GATING_SIGNAL_IN is high.
  When GATING_SIGNAL_IN is low, counting is paused, frequency is calculated and the counters are reset.
  The counters are 16bit hence cannot go beyond 32767. noverflow0 and noverflow1, count the overflows on both counters.
  The total count for a counter is given by noverflow*PCNT_H_LIM_VAL + count. 
  PCNT_H_LIM_VAL is set to 20000(doesn't seem to work well at higher values)

  Rotary encoder controls the output frequency in discrete steps. 
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
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include "AiEsp32RotaryEncoder.h"

// signal generator params {clock multiplier, on/off duration, nominal freq, calibrated freq value} 
float params[348][4] = {{247, 32393, 5, 4.9999829},
 {204, 32684, 6, 6.0000004},
 {201, 28433, 7, 6.9999978},
 {153, 32684, 8, 8.0000005},
 {136, 32684, 9, 9.0000006},
 {203, 19707, 10, 10.000002},
 {120, 30307, 11, 10.999994},
 {136, 24513, 12, 12.000001},
 {141, 21825, 13, 12.999998},
 {145, 19707, 14, 14.000003},
 {97, 27495, 15, 14.999998},
 {102, 24513, 16, 16.000001},
 {96, 24513, 17, 17.000001},
 {68, 32684, 18, 18.000001},
 {129, 16322, 19, 18.999998},
 {103, 19420, 20, 20.000009},
 {145, 13138, 21, 21.000004},
 {60, 30307, 22, 21.999988},
 {71, 24498, 23, 22.999991},
 {68, 24513, 24, 24.000002},
 {117, 13677, 25, 24.999996},
 {173, 8894, 26, 26.000004},
 {47, 31525, 27, 26.999996},
 {81, 17639, 28, 27.999977},
 {70, 19707, 29, 29.000006},
 {203, 6569, 30, 30.000006},
 {54, 23898, 31, 30.999974},
 {51, 24513, 32, 32.000002},
 {40, 30307, 33, 32.999982},
 {48, 24513, 34, 34.000002},
 {58, 19707, 35, 35.000007},
 {34, 32684, 36, 36.000002},
 {143, 7561, 37, 36.99997},
 {129, 8161, 38, 37.999997},
 {45, 22795, 39, 38.999994},
 {103, 9710, 40, 40.000019},
 {49, 19913, 41, 41.000002},
 {145, 6569, 42, 42.000009},
 {38, 24483, 43, 42.999996},
 {30, 30307, 44, 43.999976},
 {39, 22795, 45, 44.999993},
 {71, 12249, 46, 45.999982},
 {27, 31525, 47, 46.999992},
 {34, 24513, 48, 48.000003},
 {41, 19913, 49, 49.000002},
 {103, 7768, 50, 50.000023},
 {24, 32684, 51, 51.000003},
 {173, 4447, 52, 52.000009},
 {32, 23588, 53, 52.999961},
 {31, 23898, 54, 53.999955},
 {24, 30307, 55, 54.999971},
 {42, 17009, 56, 56.000071},
 {43, 16322, 57, 56.999995},
 {35, 19707, 58, 58.000012},
 {35, 19373, 59, 58.999961},
 {22, 30307, 60, 59.999968},
 {133, 4931, 61, 61.000024},
 {27, 23898, 62, 61.999948},
 {31, 20484, 63, 62.999947},
 {53, 11794, 64, 63.999953},
 {27, 22795, 65, 64.999990},
 {20, 30307, 66, 65.999965},
 {21, 28433, 67, 66.999979},
 {24, 24513, 68, 68.000004},
 {71, 8166, 69, 68.999973},
 {29, 19707, 70, 70.000015},
 {18, 31303, 71, 70.999973},
 {17, 32684, 72, 72.000005},
 {19, 28843, 73, 72.999959},
 {18, 30034, 74, 73.999872},
 {47, 11349, 75, 74.999988},
 {167, 3152, 76, 76.000066},
 {49, 10603, 77, 77.000192},
 {59, 8693, 78, 78.000063},
 {187, 2708, 79, 78.999871},
 {103, 4855, 80, 80.000037},
 {217, 2276, 81, 80.999932},
 {29, 16823, 82, 82.000255},
 {163, 2957, 83, 82.999929},
 {27, 17639, 84, 83.999930},
 {25, 18826, 85, 84.999933},
 {19, 24483, 86, 85.999993},
 {35, 13138, 87, 87.000019},
 {15, 30307, 88, 87.999953},
 {17, 26441, 89, 88.999968},
 {109, 4078, 90, 90.000087},
 {16, 27476, 91, 91.000370},
 {20, 21742, 92, 91.999859},
 {18, 23898, 93, 92.999922},
 {211, 2017, 94, 94.000095},
 {17, 24771, 95, 95.000127},
 {17, 24513, 96, 96.000006},
 {13, 31725, 97, 96.999984},
 {24, 17009, 98, 98.000124},
 {13, 31084, 99, 99.000274},
 {103, 3884, 100, 100.00005},
 {12, 30307, 110, 109.99994},
 {11, 30307, 120, 119.99994},
 {107, 2876, 130, 130.00019},
 {23, 12424, 140, 139.99978},
 {10, 26670, 150, 150.00082},
 {23, 10871, 160, 159.99975},
 {25, 9413, 170, 169.99987},
 {109, 2039, 180, 180.00017},
 {7, 30079, 190, 190.00071},
 {103, 1942, 200, 200.00009},
 {29, 6569, 210, 210.00004},
 {6, 30307, 220, 219.99988},
 {7, 24848, 230, 229.99965},
 {8, 20836, 240, 240.00059},
 {17, 9413, 250, 249.99980},
 {107, 1438, 260, 260.00038},
 {9, 16463, 270, 270.00087},
 {23, 6212, 280, 279.99957},
 {7, 19707, 290, 290.00006},
 {5, 26670, 300, 300.00164},
 {5, 25810, 310, 309.99782},
 {6, 20836, 320, 320.00079},
 {4, 30307, 330, 329.99982},
 {7, 16809, 340, 339.99829},
 {11, 10391, 350, 349.99885},
 {6, 18521, 360, 359.99873},
 {7, 15446, 370, 370.00073},
 {6, 17546, 380, 380.00322},
 {67, 1531, 390, 390.00184},
 {103, 971, 400, 400.00019},
 {7, 13939, 410, 410.00296},
 {13, 7327, 420, 419.99789},
 {5, 18607, 430, 430.00181},
 {3, 30307, 440, 439.99976},
 {4, 22225, 450, 450.00246},
 {4, 21742, 460, 459.99929},
 {47, 1811, 470, 470.00269},
 {4, 20836, 480, 480.00118},
 {19, 4297, 490, 490.00182},
 {3, 26670, 500, 500.00273},
 {7, 11206, 510, 509.99743},
 {107, 719, 520, 520.00076},
 {11, 6862, 530, 529.9968},
 {4, 18521, 540, 539.99809},
 {7, 10391, 550, 549.99819},
 {23, 3106, 560, 559.99914},
 {5, 14037, 570, 569.99670},
 {5, 13795, 580, 579.99592},
 {5, 13561, 590, 590.00396},
 {3, 22225, 600, 600.00328},
 {11, 5962, 610, 610.00303},
 {5, 12905, 620, 619.99564},
 {2, 31750, 630, 630.00344},
 {3, 20836, 640, 640.00158},
 {2, 30773, 650, 650.00518},
 {2, 30307, 660, 659.99965},
 {3, 19903, 670, 670.00316},
 {2, 29416, 680, 679.99080},
 {37, 1567, 690, 689.99497},
 {2, 28575, 700, 700.00382},
 {5, 11269, 710, 710.00477},
 {3, 18521, 720, 719.99745},
 {2, 27401, 730, 729.99559},
 {7, 7723, 740, 740.00145},
 {3, 17780, 750, 750.00410},
 {3, 17546, 760, 760.00643},
 {5, 10391, 770, 769.99747},
 {7, 7327, 780, 779.99607},
 {5, 10128, 790, 789.99247},
 {79, 633, 800, 799.99237},
 {3, 16463, 810, 810.00260},
 {3, 16262, 820, 820.01432},
 {157, 307, 830, 830.00101},
 {3, 15875, 840, 840.00459},
 {5, 9413, 850, 849.99933},
 {2, 23259, 860, 859.99438},
 {7, 6569, 870, 870.00019},
 {2, 22730, 880, 880.00921},
 {2, 22475, 890, 889.99374},
 {2, 22225, 900, 900.00492},
 {2, 21981, 910, 909.99542},
 {2, 21742, 920, 919.99859},
 {2, 21508, 930, 930.00787},
 {11, 3869, 940, 939.99433},
 {9, 4679, 950, 949.99450},
 {2, 20836, 960, 960.00236},
 {2, 20621, 970, 970.01160},
 {2, 20411, 980, 979.99164},
 {17, 2377, 990, 990.00764},
 {3, 13335, 1000, 1000.0055},
 {2, 18184, 1100, 1100.0115},
 {2, 16669, 1200, 1199.9886},
 {1, 30773, 1300, 1300.0104},
 {1, 28575, 1400, 1400.0076},
 {3, 8890, 1500, 1500.0082},
 {1, 25003, 1600, 1600.0167},
 {1, 23532, 1700, 1700.0348},
 {1, 22225, 1800, 1800.0098},
 {1, 21055, 1900, 1900.0341},
 {83, 241, 2000, 1999.9609},
 {1, 19050, 2100, 2100.0115},
 {1, 18184, 2200, 2200.0230},
 {1, 17394, 2300, 2299.9436},
 {1, 16669, 2400, 2399.9771},
 {1, 16002, 2500, 2500.0137},
 {1, 15387, 2600, 2599.9362},
 {1, 14817, 2700, 2699.954},
 {1, 14288, 2800, 2799.9173},
 {1, 13795, 2900, 2899.9796},
 {3, 4445, 3000, 3000.0164},
 {1, 12905, 3100, 3099.9782},
 {1, 12502, 3200, 3199.9055},
 {27, 449, 3300, 3299.9438},
 {1, 11766, 3400, 3400.0696},
 {3, 3810, 3500, 3500.0191},
 {1, 11113, 3600, 3599.8577},
 {3, 3604, 3700, 3700.0757},
 {1, 10528, 3800, 3799.8878},
 {1, 10258, 3900, 3899.9043},
 {1, 10001, 4000, 4000.1218},
 {1, 9757, 4100, 4100.1556},
 {1, 9525, 4200, 4200.0229},
 {1, 9304, 4300, 4299.7870},
 {1, 9092, 4400, 4400.0460},
 {1, 8890, 4500, 4500.0246},
 {1, 8697, 4600, 4599.8872},
 {1, 8512, 4700, 4699.8612},
 {1, 8334, 4800, 4800.2422},
 {1, 8164, 4900, 4900.1983},
 {1, 8001, 5000, 5000.0273},
 {37, 212, 5100, 5100.1044},
 {7, 1099, 5200, 5200.2104},
 {1, 7548, 5300, 5300.1084},
 {1, 7408, 5400, 5400.2725},
 {1, 7274, 5500, 5499.7551},
 {1, 7144, 5600, 5599.8346},
 {11, 638, 5700, 5700.3731},
 {3, 2299, 5800, 5800.3797},
 {1, 6781, 5900, 5899.6046},
 {1, 6668, 6000, 5999.5829},
 {1, 6558, 6100, 6100.2163},
 {1, 6452, 6200, 6200.4368},
 {25, 254, 6300, 6300.0344},
 {1, 6251, 6400, 6399.811},
 {1, 6155, 6500, 6499.6293},
 {1, 6061, 6600, 6600.432},
 {1, 5971, 6700, 6699.9194},
 {1, 5883, 6800, 6800.1391},
 {1, 5798, 6900, 6899.8307},
 {3, 1905, 7000, 7000.0382},
 {1, 5635, 7100, 7099.4177},
 {1, 5556, 7200, 7200.3633},
 {1, 5480, 7300, 7300.2224},
 {3, 1802, 7400, 7400.1514},
 {1, 5334, 7500, 7500.041},
 {1, 5264, 7600, 7599.7756},
 {1, 5195, 7700, 7700.7158},
 {1, 5129, 7800, 7799.8086},
 {1, 5064, 7900, 7899.9247},
 {1, 5001, 8000, 7999.4438},
 {1, 4939, 8100, 8099.8620},
 {1, 4879, 8200, 8199.4709},
 {1, 4820, 8300, 8299.8379},
 {1, 4763, 8400, 8399.1641},
 {1, 4706, 8500, 8500.8964},
 {1, 4652, 8600, 8599.5741},
 {11, 418, 8700, 8700.5695},
 {1, 4546, 8800, 8800.0921},
 {5, 899, 8900, 8899.9374},
 {1, 4445, 9000, 9000.0492},
 {1, 4396, 9100, 9100.3682},
 {1, 4348, 9200, 9200.8322},
 {239, 18, 9300, 9299.2140},
 {1, 4256, 9400, 9399.7224},
 {1, 4211, 9500, 9500.1706},
 {1, 4167, 9600, 9600.4844},
 {1, 4124, 9700, 9700.5865},
 {1, 4082, 9800, 9800.3965},
 {1, 4041, 9900, 9899.8314},
 {1, 4001, 10000, 9998.8049},
 {1, 3637, 11000, 10999.510},
 {1, 3334, 12000, 11999.166},
 {17, 181, 13000, 13001.371},
 {1, 2858, 14000, 13997.627},
 {1, 2667, 15000, 15000.082},
 {1, 2500, 16000, 16002.087},
 {1, 2353, 17000, 17001.793},
 {1, 2223, 18000, 17996.050},
 {27, 78, 19000, 18995.830},
 {1, 2000, 20000, 20002.609},
 {3, 635, 21000, 21000.115},
 {3, 606, 22000, 22005.071},
 {1, 1739, 23000, 23004.726},
 {1, 1667, 24000, 23998.331},
 {1, 1600, 25000, 25003.262},
 {27, 57, 26000, 25994.294},
 {1, 1482, 27000, 26994.075},
 {1, 1429, 28000, 27995.254},
 {1, 1379, 29000, 29010.311},
 {1, 1334, 30000, 29988.919},
 {1, 1290, 31000, 31011.797},
 {1, 1250, 32000, 32004.175},
 {1, 1212, 33000, 33007.606},
 {107, 11, 34000, 33989.141},
 {1, 1143, 35000, 35000.191},
 {11, 101, 36000, 36008.298},
 {1, 1081, 37000, 37007.603},
 {27, 39, 38000, 37991.661},
 {27, 38, 39000, 38991.441},
 {1, 1000, 40000, 40005.219},
 {1, 976, 41000, 40988.953},
 {3, 310, 43000, 43016.364},
 {3, 303, 44000, 44010.141},
 {1, 889, 45000, 45000.246},
 {1, 870, 46000, 45983.01},
 {1, 851, 47000, 47009.658},
 {1, 800, 50000, 50006.523},
 {1, 769, 52000, 52022.391},
 {1, 755, 53000, 52987.044},
 {1, 741, 54000, 53988.149},
 {1, 702, 57000, 56987.491},
 {5, 138, 58000, 57978.578},
 {1, 678, 59000, 59004.747},
 {1, 667, 60000, 59977.839},
 {1, 656, 61000, 60983.565},
 {1, 645, 62000, 62023.595},
 {1, 635, 63000, 63000.344},
 {1, 625, 64000, 64008.35},
 {1, 606, 66000, 66015.212},
 {1, 597, 67000, 67010.416},
 {1, 580, 69000, 68974.515},
 {1, 548, 73000, 73002.224},
 {27, 19, 78000, 77982.882},
 {1, 500, 80000, 80010.437},
 {1, 494, 81000, 80982.224},
 {1, 488, 82000, 81977.907},
 {1, 482, 83000, 82998.379},
 {3, 155, 86000, 86032.728},
 {1, 460, 87000, 86967.866},
 {1, 435, 92000, 91966.020},
 {43, 10, 93000, 93035.392},
 {1, 421, 95000, 95024.272},
 {1, 404, 99000, 99022.818},
 {1, 400, 100000, 100013.05},
 {1, 200, 200000, 200026.09},
 {1, 100, 400000, 400052.19},
 {1, 80, 500000, 500065.23},
 {1, 50, 800000, 800104.37},
 {1, 40, 1000000, 1000130.5},
 {1, 20, 2000000, 2000260.9},
 {1, 10, 4000000, 4000521.9},
 {1, 8, 5000000, 5000652.3}};


LiquidCrystal_PCF8574 lcd(0x27);
byte custom[3][8] = {
{ B11100,// gate on
  B11000,
  B10000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000},
{ B00000,//out
  B00000,
  B00000,
  B01110,
  B01010,
  B11011,
  B00000,
  B00000},
{ B10100,//Hz
  B10111,
  B11101,
  B10110,
  B10111,
  B00000,
  B00000,
  B00000}};                  

#define PCNT_H_LIM_VAL            20000       // counter overflow limit. max 32767, but higher values don't seem to work reliably

#define PCNT_TEST_UNIT0           PCNT_UNIT_0 // counter 0 counts reference clock pulses
#define PCNT_INPUT_SIG_IO0        GPIO_NUM_39 // counter 0 input (tied to REFERENCE_SIGNAL_OUT pin)
#define PCNT_INPUT_CTRL_IO        GPIO_NUM_9  // counter input control (high => count, low => stop counting)
#define PCNT_TEST_UNIT1           PCNT_UNIT_1 // counter 1 counts input signal transitions
#define PCNT_INPUT_SIG_IO1        GPIO_NUM_3  // counter 1 input (input signal connected here. 3V3 level only)
#define GATING_SIGNAL_IN          GPIO_NUM_7  // gating signal input from D flipflop. edges are aligned with input signal
#define GATING_SIGNAL_OUT         GPIO_NUM_5  // raw gating signal ~1s
#define REFERENCE_SIGNAL_OUT      GPIO_NUM_40 // ~10Mhz reference signal generated using ESP32 RMT
#define MIN_GATE_TIME             1000

#define SDA_PIN GPIO_NUM_33 // default SDA. change here if needed
#define SCL_PIN GPIO_NUM_35 // default SCL. change here if needed

#define ROTARY_ENCODER_A_PIN      GPIO_NUM_13 // DT pin on rotary encoder
#define ROTARY_ENCODER_B_PIN      GPIO_NUM_10 // CLK pin on rotary encoder
#define ROTARY_ENCODER_BUTTON_PIN GPIO_NUM_14 // SW on rotary encoder (push) 
#define ROTARY_ENCODER_VCC_PIN    -1          // tied to 3V3
#define OUTPUT_PIN                GPIO_NUM_16 // Signal generator output
#define ROTARY_ENCODER_STEPS      4

RTC_DATA_ATTR int encoderValue = 185; // default to index 185 in the lookup - 1KHz 
RTC_DATA_ATTR int set_duty = 50; // default to 50 
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

int gate_time_multiplier=1000;

//xQueueHandle pcnt_evt_queue;  /*A queue to handle pulse counter event*/

rmt_data_t bit_data[2];
rmt_obj_t* rmt_send = NULL;

rmt_data_t freq_gen_bit_data[2];
rmt_obj_t* freq_gen_rmt_send = NULL;

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
float    reference  = 10001305.4; // Actual value measured on a calibrated HP5316B. measure the actual reference frequency on a calibrated meter and enter here.

bool last = false;
bool ready = false;

esp_timer_create_args_t timer_args;
esp_timer_handle_t timer;

// pcnt_evt_t evt;
// portBASE_TYPE res;

char display_line1[16]={'*',' ','i','n','i','t','i','a','l','i','z','i','n','g',' ','*'};
char display_line2[16]={'.','.','.','.','.','.','.','.','.','.','.','.','.','.','.','.'};

/* Create a formatted display string to be displayed on 1602 LCD
*/
void create_display_string(char* buf, float f, int duty=-1){
  char unit =' ';
  if (f>1000000){
    f /=1000000;
    unit = 'M';
  } else if (f > 1000){
    f/=1000;
    unit = 'K';
  } else {
    unit = ' ';
  }
  int precision = 8-int(log10(f)+1);
  precision = precision<0?0:precision;
  if (duty == -1){
    sprintf(buf, "%8.*f%c\03    ",precision,f,unit);
  } else {
    sprintf(buf, "%8.*f%c\03 %d%%",precision,f,unit,duty);  
  }
}

/* When rotary encoder push button is clicked. change the duty cycle and display on LCD
   Cycle through 10,20,30,40,50,60,70,80,90% duty cycle. Initial value is 50%
   Duty cycle remain at 50% if the paramters do not allow change in duty cycle
*/
void rotary_onButtonClick()
{ 
  encoderValue = rotaryEncoder.readEncoder(); //get encoder value
  
  float t_dur = params[encoderValue][1]; // get duration from lookup
  int duty = 50; // actual duty cycle
  Serial.print("button pressed ");
  if (t_dur < 18000 && t_dur > 40){ // if duty cycle can be changed, increment in steps of 10, between 10 and 90
    if (set_duty < 90){
      set_duty += 10;
    } else {
      set_duty = 10;
    }
    duty = set_duty;
  } else { // duty cycle cannot be applied reliably for this frequency, set to default 50%
    duty = 50;
  }

  float t_on = (t_dur/100)*2*duty; // compute on time
  float t_off = 2*t_dur - int(t_on); // off time = total time - on time
  bool stat = rmtDeinit(freq_gen_rmt_send);  // deinit rmt
  //Serial.printf("rmtDeinit: %d\n", stat);
  if ((freq_gen_rmt_send = rmtInit(OUTPUT_PIN, RMT_TX_MODE, RMT_MEM_64)) == NULL)
  {
      //Serial.println("init sender1 failed\n");
  }
  
  float realTick1 = rmtSetTick(freq_gen_rmt_send, 12.5*params[encoderValue][0]); // set clock per lookup
  // Send the data

  for (int i=0; i < 2; i++){ // set on and off times
    freq_gen_bit_data[i].level0=1;
    freq_gen_bit_data[i].duration0=int(t_on);
    freq_gen_bit_data[i].level1=0;
    freq_gen_bit_data[i].duration1=int(t_off);
  }
  Serial.printf("Index: %d, dur: %d, duty: %d, d0: %d, d1: %d, freq: %0.3f\n",encoderValue, int(t_dur), duty, int(t_on), int(t_off), params[encoderValue][3]);    
  rmtLoop(freq_gen_rmt_send, freq_gen_bit_data, 2);// send pulses
  //int d = 8-int(log10(params[encoderValue][3])+1);
  //d = d<0?0:d;
  //sprintf(display_line2, "%8.*f \03 %d%%",d,params[encoderValue][3],duty);
  create_display_string(display_line2, params[encoderValue][3], duty);
  lcd.setCursor(1, 1);      
  lcd.print(display_line2);
  lcd.setCursor(0, 1);   
  lcd.print("\02");
}


/* Rotaty encoder interrup service routine
*/
void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

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
#include <driver/rmt.h>

/* Start a reference signal at specified pin 
 */
void start_ref_clock(gpio_num_t pin, uint32_t clk_mult, uint32_t cycles_on, uint32_t cycles_off){  
  if ((rmt_send = rmtInit(pin, RMT_TX_MODE, RMT_MEM_64)) == NULL)
  {
      Serial.println("init sender failed\n");
  }
  float realTick = rmtSetTick(rmt_send, 12.5*clk_mult);
  // Send the data
  for (int i=0; i < 2; i++){
    bit_data[i].level0=1;
    bit_data[i].duration0=cycles_on;
    bit_data[i].level1=0;
    bit_data[i].duration1=cycles_off;
  }
  rmtLoop(rmt_send, bit_data, 2);
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
  pcnt_config.neg_mode = PCNT_COUNT_DIS;//keep the counter value      
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

void setup()
{
    Serial.begin(115200);
    //while (!Serial);
    delay(500);
    Serial.printf("Begin\n");
    Wire.begin(SDA_PIN,SCL_PIN);
    Wire.beginTransmission(0x27);
    int result = Wire.endTransmission();
    Serial.print("Result: ");
    Serial.print(result);

    if (result == 0) {
      Serial.println(": LCD found.");
      lcd.begin(16, 2);  // initialize the lcd
      for (int i=0; i < 3; i++){
        lcd.createChar(i+1, custom[i]);
      }
      lcd.setBacklight(100);
      lcd.clear();
      lcd.setCursor(0, 0);      
      lcd.print(display_line1);
      lcd.setCursor(0, 1);      
      lcd.print(display_line2);
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("calibrated ref:");
      lcd.setCursor(0, 1); 
      sprintf(display_line2, "%10.1fHz", reference);    
      lcd.print(display_line2);
      delay(5000);
      lcd.clear();
    } else {
      Serial.println(": LCD not found.");
    }
    /*Init PCNT event queue */
    // pcnt_evt_queue = xQueueCreate(50, sizeof(pcnt_evt_t));   

    // Start reference clock
    start_ref_clock(REFERENCE_SIGNAL_OUT,1,3,5);   
     
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

    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    //set boundaries and if values should cycle or not
    //in this example we will set possible values between 0 and 1000;
    bool circleValues = false;
    rotaryEncoder.setBoundaries(0, 347, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  
    rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
    //rotaryEncoder.setAcceleration(250); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration

    rotaryEncoder.setEncoderValue(encoderValue);// set encoder default to index 185 - 1KHz. If waking from sleep. set to last value before sleep
    encoderValue = rotaryEncoder.readEncoder();
    Serial.printf("setup - Index: %d, dur: %d, duty: %d, freq: %0.3f\n",encoderValue, int(params[encoderValue][1]), set_duty, params[encoderValue][3]);

}


void loop(){
  // if there is a new reading, display new frequency
  if (ready){
    int n_ref = ((reading.noverflow0*PCNT_H_LIM_VAL)+reading.count0);
    int n_sig = ((reading.noverflow1*PCNT_H_LIM_VAL)+reading.count1);

    float frequency = float(n_sig)/float(n_ref)*reference;
    int precision = 8 - int(log10(frequency)+1);//int(log10(n_ref)+1) - int(log10(frequency)+1);
    precision = precision<0?0:precision;
    if (frequency){
      //sprintf(display_line1, "%-8.*f \04", precision,frequency);
      create_display_string(display_line1,frequency);
      Serial.printf("Frequency: %0.*f Hz, Gate Time: %0.3fs\n", precision,frequency, gate_time_multiplier*MIN_GATE_TIME/1000000.0);
      //lcd.clear();
      lcd.setCursor(1, 0);      
      lcd.print(display_line1);
    }
    ready = false;
  }
  // flash the gate indicator on LCD to indicate counter is working
  bool gate_on = gpio_get_level(PCNT_INPUT_CTRL_IO);
  if (gate_on){
    lcd.setCursor(0, 0);   
    lcd.print("\01");
  } else {
    lcd.setCursor(0, 0);   
    lcd.print(" ");
  }
  // If rotary encoder changed, signal generator outputs new frequency and displays it on LCD 
  if (rotaryEncoder.encoderChanged()) // if encoder value changed
  {
    Serial.print("encoder changed ");
    encoderValue = rotaryEncoder.readEncoder(); // read encoder value
    float t_dur = params[encoderValue][1];
    int duty = 50;
    if (t_dur > 18000 || t_dur < 40){ // requested duty cycle cannot be applied reliably, default to 50
      duty = 50;
    } else {
      duty = set_duty;
    }
    float t_on = (t_dur/100)*2*duty;
    float t_off = 2*t_dur - int(t_on);    
    bool stat = rmtDeinit(freq_gen_rmt_send);
    //Serial.printf("rmtDeinit: %d\n", stat);
    if ((freq_gen_rmt_send = rmtInit(OUTPUT_PIN, RMT_TX_MODE, RMT_MEM_64)) == NULL)
    {
        //Serial.println("init sender1 failed\n");
    }
    float realTick1 = rmtSetTick(freq_gen_rmt_send, 12.5*params[encoderValue][0]);
    // Send the data
    for (int i=0; i < 2; i++){
      freq_gen_bit_data[i].level0=1;
      freq_gen_bit_data[i].duration0=int(t_on);
      freq_gen_bit_data[i].level1=0;
      freq_gen_bit_data[i].duration1=int(t_off);
    }
    Serial.printf("Index: %d, dur: %d, duty: %d, d0: %d, d1: %d, freq: %0.3f\n",encoderValue, int(t_dur), duty, int(t_on), int(t_off), params[encoderValue][3]);    
    rmtLoop(freq_gen_rmt_send, freq_gen_bit_data, 2);

    // int d = 8-int(log10(params[encoderValue][3])+1);
    // d = d<0?0:d;
    // sprintf(display_line2, "%8.*f \03 %d%%",d,params[encoderValue][3],duty);
    create_display_string(display_line2, params[encoderValue][3], duty);
    lcd.setCursor(1, 1);      
    lcd.print(display_line2);
    lcd.setCursor(0, 1);   
    lcd.print("\02");
  }
  // If rotary encoder was pushed. Update duty cycle (if possible) and display on LCD
  if (rotaryEncoder.isEncoderButtonClicked())
  {
    rotary_onButtonClick();
  } 

  delay(10);
}