# Building-a-simple-processor-Memory-hierarchy
Group mini project (Lab 05, 06) in Computer Architecture Course

## Pre requests
verilog compiler(Icarus Verilog) with GTKWave.

## Compile & run the verilog file

`iverilog -o fileName fileName.v`

Then it will create a binary file

`vvp fileName`

This will run the binary file and give outputs

## Run the GTKwaveData if it is created

`gtkwave GTKwaveFileName.vcd`

The GTKWave will open automatically
Then you can select components that want to show from the left tab

Instruction file "instr_mem.mem" will be needed to run the CPU properly.
It should be in the same directory with verilog file. If not, change the path in verilog file.
