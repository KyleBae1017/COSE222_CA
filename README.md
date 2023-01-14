# COSE222_CA

In this course, I learned about basic concept of computer architecture (single-cycle / pipelined CPU, hazards, Cache & TLB) and RISC-V instructions.\
The main assignment for this class was implementing additional instructions for single-cycle RISC-V CPU and pipelined RISC-V CPU capable of handling hazards.


## Assignment Details

I completed these assignments in this class.

- Implementing the sorting algorithm in RISC-V assembly language
- Add some RISC-V instructions to provided reference single-cycle RV32I CPU code and system
- Change the provided single-cycle RV32I CPU to pipelined CPU code
- Make the pipelined CPU code not to suffer with data hazard and control hazard (Structural hazard can't be occured in this sysyem because the data memory and instruction memory are seperated.)

*RV32I_HW_System.7z* is provided single-cycle RV32I CPU system by professor. This system can be operated on Altera DE0 board.\
With this system, I only changed two files, *basic_modules.v* and *rv32i_cpu.v* for assignments.\
*rv32i_cpu.v* is the main CPU code and *basic_modules.v* is the code for some useful modules for implementing CPU like ALU or multiplexer.
The uploaded two files are my final code after accomplishing the final assignment.
