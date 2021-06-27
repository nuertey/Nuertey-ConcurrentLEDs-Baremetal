# Nuertey-ConcurrentLEDs-Baremetal - Concurrent LEDs on Baremetal MCU

Illustrating how the ARM Mbed Baremetal profile can be used to control 
several LEDs concurrently (even though threading is not supported on baremetal).

Functionally, this application continuously blinks 3 10mm external LEDs
connected to various I/O ports at different frequencies, whilst employing
PWM output pins to vary the intensity of yet another 3 10mm LEDs according to:

[1] An on-the-fly calculated sawtooth waveform pattern,

[2] A precomputed triangular waveform pattern,

[3] And a precomputed sinusoidal waveform pattern.

Essentially, the values of said waveforms are scaled and used as the duty
cycle in driving, dimming, and brightening of the latter 3 10mm LEDs.

## Video/Pictures Of STM32F767 MCU, the Running Code and LEDs in Action (click on first picture for video link)

[![Watch the video](https://github.com/nuertey/RandomArtifacts/blob/master/20210627_092109.jpg)](https://github.com/nuertey/RandomArtifacts/blob/master/ConcurrentLEDs-Baremetal.mp4)

![alt text](https://github.com/nuertey/RandomArtifacts/blob/master/20210627_092115.jpg?raw=true)

## License
MIT License

Copyright (c) 2021 Nuertey Odzeyem

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
