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

// calibration parameters
float params[348][4] = {{247, 32393, 5, 4.999982945888294},
 {204, 32684, 6, 6.000000382276363},
 {201, 28433, 7, 6.999997821332161},
 {153, 32684, 8, 8.000000509701815},
 {136, 32684, 9, 9.000000573414543},
 {203, 19707, 10, 10.000002136932016},
 {120, 30307, 11, 10.999994101704063},
 {136, 24513, 12, 12.000000764552723},
 {141, 21825, 13, 12.999997903647294},
 {145, 19707, 14, 14.000002991704823},
 {97, 27495, 15, 14.999997581131492},
 {102, 24513, 16, 16.00000101940363},
 {96, 24513, 17, 17.000001083116356},
 {68, 32684, 18, 18.000001146829085},
 {129, 16322, 19, 18.99999836091365},
 {103, 19420, 20, 20.000009273215184},
 {145, 13138, 21, 21.000004487557234},
 {60, 30307, 22, 21.999988203408126},
 {71, 24498, 23, 22.999991116746184},
 {68, 24513, 24, 24.000001529105447},
 {117, 13677, 25, 24.999995968552486},
 {173, 8894, 26, 26.00000425619233},
 {47, 31525, 27, 26.999995646036684},
 {81, 17639, 28, 27.999976587263077},
 {70, 19707, 29, 29.00000619710285},
 {203, 6569, 30, 30.000006410796047},
 {54, 23898, 31, 30.99997407875555},
 {51, 24513, 32, 32.00000203880726},
 {40, 30307, 33, 32.99998230511219},
 {48, 24513, 34, 34.00000216623271},
 {58, 19707, 35, 35.00000747926205},
 {34, 32684, 36, 36.00000229365817},
 {143, 7561, 37, 36.99996998661831},
 {129, 8161, 38, 37.9999967218273},
 {45, 22795, 39, 38.99999371094188},
 {103, 9710, 40, 40.00001854643037},
 {49, 19913, 41, 41.000001587355406},
 {145, 6569, 42, 42.00000897511447},
 {38, 24483, 43, 42.99999629048878},
 {30, 30307, 44, 43.99997640681625},
 {39, 22795, 45, 44.99999274339447},
 {71, 12249, 46, 45.99998223349237},
 {27, 31525, 47, 46.99999242087868},
 {34, 24513, 48, 48.00000305821089},
 {41, 19913, 49, 49.000001897083294},
 {103, 7768, 50, 50.00002318303797},
 {24, 32684, 51, 51.00000324934907},
 {173, 4447, 52, 52.00000851238466},
 {32, 23588, 53, 52.99996098233398},
 {31, 23898, 54, 53.999954846864505},
 {24, 30307, 55, 54.99997050852031},
 {42, 17009, 56, 56.00007075923586},
 {43, 16322, 57, 56.999995082740945},
 {35, 19707, 58, 58.0000123942057},
 {35, 19373, 59, 58.999960989656294},
 {22, 30307, 60, 59.99996782747671},
 {133, 4931, 61, 61.000023708899214},
 {27, 23898, 62, 61.9999481575111},
 {31, 20484, 63, 62.999947321341914},
 {53, 11794, 64, 63.999952884327826},
 {27, 22795, 65, 64.99998951823646},
 {20, 30307, 66, 65.99996461022438},
 {21, 28433, 67, 66.99997914703638},
 {24, 24513, 68, 68.00000433246542},
 {71, 8166, 69, 68.99997335023855},
 {29, 19707, 70, 70.0000149585241},
 {18, 31303, 71, 70.99997257778169},
 {17, 32684, 72, 72.00000458731634},
 {19, 28843, 73, 72.9999590320034},
 {18, 30034, 74, 73.99987153234002},
 {47, 11349, 75, 74.99998790565746},
 {167, 3152, 76, 76.00006563429245},
 {49, 10603, 77, 77.0001916069988},
 {59, 8693, 78, 78.00006346201289},
 {187, 2708, 79, 78.99987075103557},
 {103, 4855, 80, 80.00003709286074},
 {217, 2276, 81, 80.99993227029677},
 {29, 16823, 82, 82.00025529261336},
 {163, 2957, 83, 82.9999285232326},
 {27, 17639, 84, 83.99992976178923},
 {25, 18826, 85, 84.99993317505876},
 {19, 24483, 86, 85.99999258097756},
 {35, 13138, 87, 87.00001859130853},
 {15, 30307, 88, 87.9999528136325},
 {17, 26441, 89, 88.9999678503781},
 {109, 4078, 90, 90.00008672366245},
 {16, 27476, 91, 91.00036975187756},
 {20, 21742, 92, 91.99985868098935},
 {18, 23898, 93, 92.99992223626666},
 {211, 2017, 94, 94.00009527744363},
 {17, 24771, 95, 95.00012716207853},
 {17, 24513, 96, 96.00000611642179},
 {13, 31725, 97, 96.99998435798365},
 {24, 17009, 98, 98.00012382866277},
 {13, 31084, 99, 99.00027357344715},
 {103, 3884, 100, 100.00004636607594},
 {12, 30307, 110, 109.99994101704063},
 {11, 30307, 120, 119.99993565495342},
 {107, 2876, 130, 130.00019025919113},
 {23, 12424, 140, 139.99978494933163},
 {10, 26670, 150, 150.0008194557233},
 {23, 10871, 160, 159.99975422780756},
 {25, 9413, 170, 169.99986635011751},
 {109, 2039, 180, 180.0001734473249},
 {7, 30079, 190, 190.00070551757233},
 {103, 1942, 200, 200.00009273215187},
 {29, 6569, 210, 210.0000448755723},
 {6, 30307, 220, 219.99988203408125},
 {7, 24848, 230, 229.99964670247334},
 {8, 20836, 240, 240.00059121737257},
 {17, 9413, 250, 249.99980345605516},
 {107, 1438, 260, 260.00038051838226},
 {9, 16463, 270, 270.00086759427813},
 {23, 6212, 280, 279.99956989866325},
 {7, 19707, 290, 290.00006197102846},
 {5, 26670, 300, 300.0016389114466},
 {5, 25810, 310, 309.9978190534011},
 {6, 20836, 320, 320.0007882898301},
 {4, 30307, 330, 329.9998230511219},
 {7, 16809, 340, 339.9982878971419},
 {11, 10391, 350, 349.9988499561806},
 {6, 18521, 360, 359.99872711013984},
 {7, 15446, 370, 370.00072648342984},
 {6, 17546, 380, 380.00321582166305},
 {67, 1531, 390, 390.00183812006014},
 {103, 971, 400, 400.00018546430374},
 {7, 13939, 410, 410.00295726114194},
 {13, 7327, 420, 419.9978850494105},
 {5, 18607, 430, 430.0018116713216},
 {3, 30307, 440, 439.9997640681625},
 {4, 22225, 450, 450.00245836716994},
 {4, 21742, 460, 459.9992934049467},
 {47, 1811, 470, 470.0026851139186},
 {4, 20836, 480, 480.00118243474515},
 {19, 4297, 490, 490.0018194926865},
 {3, 26670, 500, 500.00273151907766},
 {7, 11206, 510, 509.99743184571287},
 {107, 719, 520, 520.0007610367645},
 {11, 6862, 530, 529.9968012087836},
 {4, 18521, 540, 539.9980906652098},
 {7, 10391, 550, 549.9981927882839},
 {23, 3106, 560, 559.9991397973265},
 {5, 14037, 570, 569.9967022703057},
 {5, 13795, 580, 579.9959195192665},
 {5, 13561, 590, 590.0039606052858},
 {3, 22225, 600, 600.0032778228932},
 {11, 5962, 610, 610.0030274898814},
 {5, 12905, 620, 619.9956381068022},
 {2, 31750, 630, 630.0034417140379},
 {3, 20836, 640, 640.0015765796602},
 {2, 30773, 650, 650.0051757846392},
 {2, 30307, 660, 659.9996461022438},
 {3, 19903, 670, 670.0031577960006},
 {2, 29416, 680, 679.9907966555854},
 {37, 1567, 690, 689.9949731599614},
 {2, 28575, 700, 700.0038241267087},
 {5, 11269, 710, 710.0047661521237},
 {3, 18521, 720, 719.9974542202797},
 {2, 27401, 730, 729.9955941177586},
 {7, 7723, 740, 740.0014529668597},
 {3, 17780, 750, 750.0040972786164},
 {3, 17546, 760, 760.0064316433261},
 {5, 10391, 770, 769.9974699035974},
 {7, 7327, 780, 779.9960722346196},
 {5, 10128, 790, 789.9924673941827},
 {79, 633, 800, 799.9923720447418},
 {3, 16463, 810, 810.0026027828343},
 {3, 16262, 820, 820.0143186332432},
 {157, 307, 830, 830.001007258271},
 {3, 15875, 840, 840.0045889520504},
 {5, 9413, 850, 849.9993317505877},
 {2, 23259, 860, 859.9943795700891},
 {7, 6569, 870, 870.0001859130854},
 {2, 22730, 880, 880.0092069696744},
 {2, 22475, 890, 889.9937385726676},
 {2, 22225, 900, 900.0049167343399},
 {2, 21981, 910, 909.9954176070562},
 {2, 21742, 920, 919.9985868098934},
 {2, 21508, 930, 930.007870300386},
 {11, 3869, 940, 939.9943266721823},
 {9, 4679, 950, 949.9945037838429},
 {2, 20836, 960, 960.0023648694903},
 {2, 20621, 970, 970.0116034343971},
 {2, 20411, 980, 979.9916356092647},
 {17, 2377, 990, 990.007635646549},
 {3, 13335, 1000, 1000.0054630381553},
 {2, 18184, 1100, 1100.0115087120932},
 {2, 16669, 1200, 1199.9885580671128},
 {1, 30773, 1300, 1300.0103515692783},
 {1, 28575, 1400, 1400.0076482534173},
 {3, 8890, 1500, 1500.0081945572329},
 {1, 25003, 1600, 1600.0167399448626},
 {1, 23532, 1700, 1700.0347844994646},
 {1, 22225, 1800, 1800.0098334686797},
 {1, 21055, 1900, 1900.0341272306532},
 {83, 241, 2000, 1999.960933302075},
 {1, 19050, 2100, 2100.011472380126},
 {1, 18184, 2200, 2200.0230174241865},
 {1, 17394, 2300, 2299.9435753042085},
 {1, 16669, 2400, 2399.9771161342255},
 {1, 16002, 2500, 2500.0136575953884},
 {1, 15387, 2600, 2599.9362155612794},
 {1, 14817, 2700, 2699.9540088304925},
 {1, 14288, 2800, 2799.9173116490347},
 {1, 13795, 2900, 2899.9795975963325},
 {3, 4445, 3000, 3000.0163891144657},
 {1, 12905, 3100, 3099.9781905340105},
 {1, 12502, 3200, 3199.905499027468},
 {27, 449, 3300, 3299.943788570602},
 {1, 11766, 3400, 3400.0695689989293},
 {3, 3810, 3500, 3500.019120633543},
 {1, 11113, 3600, 3599.8576935878164},
 {3, 3604, 3700, 3700.075707440011},
 {1, 10528, 3800, 3799.8877800951186},
 {1, 10258, 3900, 3899.9043233419193},
 {1, 10001, 4000, 4000.1218426998703},
 {1, 9757, 4100, 4100.155636859834},
 {1, 9525, 4200, 4200.022944760252},
 {1, 9304, 4300, 4299.787032334631},
 {1, 9092, 4400, 4400.046034848373},
 {1, 8890, 4500, 4500.024583671699},
 {1, 8697, 4600, 4599.887150608417},
 {1, 8512, 4700, 4699.8612016965935},
 {1, 8334, 4800, 4800.2422064844495},
 {1, 8164, 4900, 4900.198254390177},
 {1, 8001, 5000, 5000.027315190777},
 {37, 212, 5100, 5100.104353498394},
 {7, 1099, 5200, 5200.210392414065},
 {1, 7548, 5300, 5300.108445792449},
 {1, 7408, 5400, 5400.272482295006},
 {1, 7274, 5500, 5499.755093324361},
 {1, 7144, 5600, 5599.834623298069},
 {11, 638, 5700, 5700.373118957167},
 {3, 2299, 5800, 5800.379664903784},
 {1, 6781, 5900, 5899.604564052707},
 {1, 6668, 6000, 5999.58286575306},
 {1, 6558, 6100, 6100.21630814904},
 {1, 6452, 6200, 6200.4368488594855},
 {25, 254, 6300, 6300.034417140378},
 {1, 6251, 6400, 6399.810998054936},
 {1, 6155, 6500, 6499.629333686662},
 {1, 6061, 6600, 6600.432032476721},
 {1, 5971, 6700, 6699.919368420936},
 {1, 5883, 6800, 6800.139137997859},
 {1, 5798, 6900, 6899.830725912626},
 {3, 1905, 7000, 7000.038241267086},
 {1, 5635, 7100, 7099.417666165289},
 {1, 5556, 7200, 7200.363309726674},
 {1, 5480, 7300, 7300.222362927264},
 {3, 1802, 7400, 7400.151414880022},
 {1, 5334, 7500, 7500.040972786164},
 {1, 5264, 7600, 7599.775560190237},
 {1, 5195, 7700, 7700.7157938097025},
 {1, 5129, 7800, 7799.808646683839},
 {1, 5064, 7900, 7899.924673941825},
 {1, 5001, 8000, 7999.443821004081},
 {1, 4939, 8100, 8099.862026491476},
 {1, 4879, 8200, 8199.47090568588},
 {1, 4820, 8300, 8299.83787320361},
 {1, 4763, 8400, 8399.164087516567},
 {1, 4706, 8500, 8500.896419218318},
 {1, 4652, 8600, 8599.574064669261},
 {11, 418, 8700, 8700.569497355677},
 {1, 4546, 8800, 8800.092069696746},
 {5, 899, 8900, 8899.937385726676},
 {1, 4445, 9000, 9000.049167343397},
 {1, 4396, 9100, 9100.368186724614},
 {1, 4348, 9200, 9200.832232944205},
 {239, 18, 9300, 9299.213981599583},
 {1, 4256, 9400, 9399.722403393187},
 {1, 4211, 9500, 9500.170636153267},
 {1, 4167, 9600, 9600.484412968899},
 {1, 4124, 9700, 9700.586457042047},
 {1, 4082, 9800, 9800.396508780354},
 {1, 4041, 9900, 9899.831365711805},
 {1, 4001, 10000, 9998.804935976357},
 {1, 3637, 11000, 10999.510186648722},
 {1, 3334, 12000, 11999.16573150612},
 {17, 181, 13000, 13001.370994098603},
 {1, 2858, 14000, 13997.62720393331},
 {1, 2667, 15000, 15000.081945572329},
 {1, 2500, 16000, 16002.087419536561},
 {1, 2353, 17000, 17001.792838436635},
 {1, 2223, 18000, 17996.049729573282},
 {27, 78, 19000, 18995.830270105133},
 {1, 2000, 20000, 20002.6092744207},
 {3, 635, 21000, 21000.11472380126},
 {3, 606, 22000, 22005.070708933665},
 {1, 1739, 23000, 23004.72602003531},
 {1, 1667, 24000, 23998.33146301224},
 {1, 1600, 25000, 25003.261593025876},
 {27, 57, 26000, 25994.294053828078},
 {1, 1482, 27000, 26994.07459435992},
 {1, 1429, 28000, 27995.25440786662},
 {1, 1379, 29000, 29010.31076783278},
 {1, 1334, 30000, 29988.9194519051},
 {1, 1290, 31000, 31011.79732468326},
 {1, 1250, 32000, 32004.174839073123},
 {1, 1212, 33000, 33007.6060634005},
 {107, 11, 34000, 33989.14065322125},
 {1, 1143, 35000, 35000.19120633544},
 {11, 101, 36000, 36008.29752370963},
 {1, 1081, 37000, 37007.60272788289},
 {27, 39, 38000, 37991.660540210265},
 {27, 38, 39000, 38991.441080742115},
 {1, 1000, 40000, 40005.2185488414},
 {1, 976, 41000, 40988.95343118996},
 {3, 310, 43000, 43016.36403101226},
 {3, 303, 44000, 44010.14141786733},
 {1, 889, 45000, 45000.24583671699},
 {1, 870, 46000, 45983.00982625449},
 {1, 851, 47000, 47009.65751920259},
 {1, 800, 50000, 50006.52318605175},
 {1, 769, 52000, 52022.39083074305},
 {1, 755, 53000, 52987.04443555153},
 {1, 741, 54000, 53988.14918871984},
 {1, 702, 57000, 56987.49081031539},
 {5, 138, 58000, 57978.57760701654},
 {1, 678, 59000, 59004.747122184956},
 {1, 667, 60000, 59977.8389038102},
 {1, 656, 61000, 60983.56486103873},
 {1, 645, 62000, 62023.59464936652},
 {1, 635, 63000, 63000.34417140378},
 {1, 625, 64000, 64008.349678146245},
 {1, 606, 66000, 66015.212126801},
 {1, 597, 67000, 67010.4163297176},
 {1, 580, 69000, 68974.51473938173},
 {1, 548, 73000, 73002.22362927263},
 {27, 19, 78000, 77982.88216148423},
 {1, 500, 80000, 80010.4370976828},
 {1, 494, 81000, 80982.22378307977},
 {1, 488, 82000, 81977.90686237992},
 {1, 482, 83000, 82998.37873203612},
 {3, 155, 86000, 86032.72806202451},
 {1, 460, 87000, 86967.86641052479},
 {1, 435, 92000, 91966.01965250899},
 {43, 10, 93000, 93035.39197404977},
 {1, 421, 95000, 95024.27208750928},
 {1, 404, 99000, 99022.81819020149},
 {1, 400, 100000, 100013.0463721035},
 {1, 200, 200000, 200026.092744207},
 {1, 100, 400000, 400052.185488414},
 {1, 80, 500000, 500065.2318605176},
 {1, 50, 800000, 800104.370976828},
 {1, 40, 1000000, 1000130.4637210352},
 {1, 20, 2000000, 2000260.9274420703},
 {1, 10, 4000000, 4000521.8548841407},
 {1, 8, 5000000, 5000652.318605175}};


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

rmt_data_t bit_data[60];
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
    sprintf(buf, "%8.*f%c\03",precision,f,unit);
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

  for (int i=0; i < 60; i++){ // set on and off times
    freq_gen_bit_data[i].level0=1;
    freq_gen_bit_data[i].duration0=int(t_on);
    freq_gen_bit_data[i].level1=0;
    freq_gen_bit_data[i].duration1=int(t_off);
  }
  Serial.printf("Index: %d, dur: %d, duty: %d, d0: %d, d1: %d, freq: %0.3f\n",encoderValue, int(t_dur), duty, int(t_on), int(t_off), params[encoderValue][3]);    
  rmtLoop(freq_gen_rmt_send, freq_gen_bit_data, 60);// send pulses
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

/* Start a reference signal at specified pin 
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