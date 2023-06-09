## Precision frequency counter and pulse generator using ESP32s2 (Wemos S2 mini)

### Overview

A frequency counter with 8-digit precision over the entire range (5Hz-10Mhz) for a 1s gate time. A variable duty cycle pulse generator (5Hz-5Mhz) using esp32 RMT.

### Theory of operation

A gating signal is produced on GATING_SIGNAL_OUT pin on the ESP32 and is approximately 
equal to the selected gate time (MIN_GATE_TIME x user selected gate_time_multiplier).
The gating signal from the ESP32 is wired into the D-flip flops, to produce a pcnt control signal
perfectly aligned with the input signal(f_in), such that counting is enabled from the first 
rising edge of f_in to the first rising edge of f_in after the gate time.

```
                                      |‾‾‾‾‾‾‾|     |‾‾‾‾‾‾‾|
    GATING_SIGNAL_OUT (ESP32) >>------|D     Q|-----|D     Q|--|-->> GATING_SIGNAL_IN   (ESP32)
                         f_in >>---+--|>clk   |  +--|>clk   |  |-->> PCNT_INPUT_CTRL_IO (ESP32)
                                   |  |_______|  |  |_______|
                                   +-------------+---------------->> PCNT_INPUT_SIG_IO1 (ESP32)

 REFERENCE_SIGNAL_OUT (ESP32) >>---------------------------------->> PCNT_INPUT_SIG_IO0 (ESP32)                                                    
```
The counter counts the pulses in $f_{in}$ and an internal reference (10Mhz) $f_{ref}$ during this time.
The frequency of $f_{in}$ is given by 

$f_{in} = \frac{n_{in}}{n_{ref}} * f_{ref}$

where,

$n_{in}$ = number of pulses from input signal f_in during the gate time

$n_{ref}$ = number of pulses from reference signal during the gate time

$f_{ref}$ = internal reference frequency


Compared to a direct counter, this method gives 8 digit resolution at all frequencies (less than the reference frequency) in 1s gate time. 
  
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


![Timing](/timing.png)

Synchronizing the gating signal with input, significantly reduces measurement error. 

In the example above,

the measured frequency without a synchronized gate is 

$\frac{counter_{1}}{counter_{0}}*f_{ref} = \frac{5}{10004615}*10001305.4 = 4.9983459Hz$

the measured frequency with a synchronized gate is 

$\frac{counter_{1}}{counter_{0}}*f_{ref} = \frac{5}{10001339}*10001305.4 = 4.9999832Hz$

where,

$counter_{1}$, are the input signal pulses counted by a PCNT counter 

$counter_{0}$, are the reference clock pulses counted by another PCNT counter in the same gating period

$f_{ref}$, is the reference clock frequency measured on an external calibrated meter 


The actual input frequency in this example, as measured on a calibrated meter is $4.9999823Hz$

### 


