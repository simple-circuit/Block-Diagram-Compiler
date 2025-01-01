# Block-Diagram-Compiler
A simple text-based Block Diagram Compiler for Continuous System Simulation.<br/>
Learn to program an Analog Computer using this digital simulator.<br/>
Can run real-time simulations with ADC inputs and DAC outputs.<br/>
Start reading about it with the block_programming_pico.pdf document.<br/>

![](./NewFile5.png)

Coding requires simple three letter block functions followed by variable names

adc ch0 out<br/>
prt out<br/>
end<br/>
set ch0 0<br/>
set dt 0.01<br/>
run<br/>

The above example will read ADC0 every 10ms and print the result

An optional circuit board can be made using a CNC mill. Drill and profile<br/>
files are included for a 100mmx70mm board. The board enables<br/>
+-10V analog signals.<br/>

A serial terminal is required to enter and receive text data.<br/> 
Use simpleCRT.exe if you don't have a terminal program.<br/> 
simpleCRT can plot up to five variables.<br/>

Start with reading block_programming_pico.pdf.<br/> This document covers language basics, the schemtic diagram and several program examples.

Additional experiments with block programs and externl circuits are covered in these documents:

ADC linearity.pdf.....................Measure ADC linearity and reduce differential linearity error.<br/>
Adding Block Functions.pdf........Add your own block function, a step by step tutorial.<br/>
Experiments with Integrators.pdf....Build an integrator with few arithmetic blocks. Investigate severl applications.<br/>
Some Building Tips.pdf.............Tips on building the +-10V analog I/O board. (the Block program can run without it)<br/>
looking at dynamic range.pdf......Examine limitations on the ADC and PWM DAC signal range.<br/>
multiplexer_examples.pdf..........A look at some analog multipler applicatons.<br/>
oscillators.pdf...................Build lots of oscillators.<br/>
x-y plotting.pdf..................Component curve tracing and Lissajous figures.<br/>

If you are going to build this project, here's what the other folders and files are for:

Folder Block..................Contains symbols for drawing Block programs in LTspice<br/>
Folder Pico_block_compiler_m....Arduino Pico .ino sketch program<br/>
Folder experiments............Block program examples for the youtube video Build a four dollar analog computer<br/>
Drill_block.gcode.............CNC drill file for the analog I/O board<br/>
Profile_block.gcode...........CNC trace profile cut for the analog I/O board<br/>
Pico_block_compiler_m.uf2.....Drag and drop UF2 file for Block compiler<br/>
simpleCRT.exe.................Windows serial terminal with plotting. Written in FPC Lazarus.<br/>
spice_to_block.exe............Windows program for converting LTspice schematic nodes to a Block program. Also a Lazarus program.<br/> 
