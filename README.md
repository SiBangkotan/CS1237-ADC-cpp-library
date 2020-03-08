# CS1237-ADC-cpp-library
a library for ChipSea ADC CS1237


Written specifically for ESP8266 with some timing critical sections in assembly.

Was generated over Arduino IDE environment.





The ADC itself claimed 24 bit depth. This level of precision must be implemented with very good pcb design, power supply, and general EMI protection. Otherwise even the smallest noise will creep into measurement. So my test on this IC is invalid for so many interference I failed to keep away. Nevertheless precision to 18 bits was achived for seting with 1x PGA analog gain and 10 sps. Test with internal short on input amplifier trough Config bit, achieve less than 4 bit noise level. Maybe a better power supply will give better result.

Sampling rate maximum at 1280 sps. Able to capture 11th harmonics of 50Hz electricity supply. But it was primarily designed for body weight digital scale.

Raising programable analog gain setting on the IC front end raises digitalized noise considerably.




