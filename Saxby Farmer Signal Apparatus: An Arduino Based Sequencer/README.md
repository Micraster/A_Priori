John Saxby began his working life as a carpenter at Brighton Station. It was in this role that he developed the first semaphore signals and a system of interlocking for points. The London to Brighton mainline was the effectively the test bed for this system that later spread around the world after the Saxby & Farmer signalling company was established in Hassocks. Saxby designed the iconic raised signal box with a glazed control room, many of which are found on the railway network, including the signal box at Lewes Station.

This module is based on Hagiwo Euclidian (https://www.youtube.com/watch?v=lkoBfiq6KPY) and 6 Channel Gate Sequencer (https://www.youtube.com/watch?v=YszdC8YdFl0).

Like the original this version uses an Arduino Nano but the code is modified to make use of an SPI OLED which allows for a faster refresh rate allowing the module to also function as a basic oscilloscope.

The hardware incorporates an additional input channel to allow an external gate to trigger transistions between sequences and an additional tactile switch to allow the user to select between modes.

The module has 4 Modes 
1) A BPM indicator based on clock pulses as 16th Notes
2) A 6 Channel Sequencer based on the Hagiwo original but with heavily modified code to incorporate a chaining mode, genre presets and a probability function
3) A Euclidian Sequencer
4) Oscilloscope with variable timebase

