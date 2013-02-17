directsynth
===

An experimental additive synthesizer using Teensy 3.0.
The Teensy board uses an Arm Cortex M4 clocked at 100MHz.  That should be fast enough for a small realtime additive synth.


Status
---

Experimental.  In progress.  Not for government work.  YMMV.


Background
---

This project is a simple additive synthesizer.

An additive synthesizer generates sequences of wave values -- in this case very pure sine waves -- and adds them together.
The sinewaves for a root note and a series of its harmonics (partials) can be calculated in real time, then summed (with appropriate phase and amplitude),
to produce rich and interesting sounds.  The interesting dimensions of this are

* How many partials can be calculated at once,
* The initial amplitude relationships between those partials,
* The way those amplitudes (and possibly frequencies) change over time.


Dependencies
---

* [Teensy3](http://www.pjrc.com/teensy/).
* [Arduino library for WM8731 stereo codec](https://github.com/hughpyle/machinesalem-arduino-libs/tree/master/WM8731).
* [Arduino library for I2S](https://github.com/hughpyle/teensy-i2s).
* [ARM math and CMSIS](http://forum.pjrc.com/threads/14845-T3-linking-libarm_cortexM3l_math-a).  The easiest way to obtain CMSIS is to install Arduino 1.5 and then look in hardware/arduino/sam/system/CMSIS for documentation and pre-built M4 libraries.  You'll need to have "arm_math.h" in the project or on your include path, and add "teensy3.build.additionalobject1=/your/path/to/libarm_cortexM4l_math.a" into boards.txt for the linker.


Core Oscillator
---

This project uses the [Minsky magic circle](http://cabezal.com/misc/minsky-circles.html) algorithm, which I like owing to its extreme stability.  The central loop of the oscillator is 

    x' = gain * x  +  delta * y
    y' = gain * y  -  delta * x'
    
where the gain value should be fixed at 1.0 (slightly smaller values will make the loop decay), and delta is the angular fraction that rotates the [x,y] vector.

The data layout used in this implementation is an array of (16-bit) word pairs a[]=[d][g][d][g]...
and another array of (16-bit) x, y values b[]=[x][y][x][y]...

I'm using Q15 fractional integer representation for the number values.  With Q15, the value ranges from 0x8000 (-32768 fractional, or -1.0) to 0x7FFF (1.0-epsilon).
The central loop is expressed in SIMD instructions, using [smuad](http://tech.munts.com/MCU/Frameworks/ARM/stm32f4/libs/STM32F4xx_DSP_StdPeriph_Lib_V1.0.1/Libraries/CMSIS/Documentation/CMSIS_CM4_SIMD.htm#__SMUAD) (dual 16-bit multiply and add) and [smusdx](http://tech.munts.com/MCU/Frameworks/ARM/stm32f4/libs/STM32F4xx_DSP_StdPeriph_Lib_V1.0.1/Libraries/CMSIS/Documentation/CMSIS_CM4_SIMD.htm#__SMUSDX) (dual 16-bit exchange, multiply and subtract) instructions:

    // x' = [0.5]*[x] + [delta]*[y], as Q30
    int32_t p = __SMUAD(a[i],b[i]);
    // b[hi,lo] = [y,x'].  Shift x by 15 (Q30 to Q15) and multiply by 2.
    int32_t q = __PKHTB(b[i],p,14);
    // y' = [0.5*y] - [delta*x'] as Q30
    p = __SMUSDX(a[i],q);
    // b[hi,lo] = [y',x'].  Take high-word of y (Q30 to Q15) and multiply by 2.
    b[i] = __PKHBT(q,p,2);
    
Including the register loads, this comes to about 7 machine instructions per iteration.
I'm very interested in suggestions to optimize this further.


Generating the Overtones
---

This project has two ways to generate a series of overtones.

For "string-like" sounds, the overtones have a harmonic relationship to the root.  The second harmonic is 2x the root frequency; third is at 3x; and so on.
Additionally, the higher harmonics can have a smll upward frequency skew, which has a couple different effects:  it mimics some natural effects,
and also makes the phase relationships within the note change over time.

A second method builds "drum-like" sounds.  The frequencies of the overtones are related by being zeros of the Bessel functions.
They're strangely inharmonic, but very familiar.  My goal is to make something that can sounds like a tabla or floor tom or kettledrum or glockenspiel.

I'm hoping that some interesting sounds can be made by combining these two methods together.  Perhaps the beginning of a sound can be "drumlike" and its decay
turn "stringlike" -- or, maybe a sustained sound can become increasingly "stringlike".

But already there are too many degrees of freedom.  The design now needs to *narrow* the set of choices, to have only two or three dimensions,
that can be smoothly controlled with sliders or other input mechanisms to create a wide and interesting palette.


Additional reading
---

* Smith, J.O; Cook, P.R (1999_ "The Second-Order Digital Waveguide Oscillator"
* Hodes, T; Hauser, J; Wawrzynek, J; Freed, A; Wessel, D (1999) "A Fixed-Point Recursive Digital Oscillator for Additive Synthesis of Audio"
* Legge, K.A; Fletcher, N.H (1989) "Nonlinearity, chaos, and the sound of shallow gongs"
* Fletcher, N.H (1993) "Nonlinear Dynamics and Chaos in Musical Instruments"
* Chaigne, A; Touze´, C; Thomas, O (2005) "Nonlinear vibrations and chaos in gongs and cymbals"

