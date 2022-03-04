# RISC-V_CORE
A 5 stage pipelined RISC-V core with a Hazard Control Unit
The 5 pipeline stages include Instruction Fetch, Decode, Execute, Memory Access and Write Back.
A pipeline register has been placed between every two consecutive stages, and this register allows the propagation of the instruction in the previous stage to the next stage in the pipeline, at the rising edge of the clock signal. 
A hazard control unit has also been implemented that detects and resolves the RAW data hazards, load data hazards and the control hazards. The interrupt controller flushes the Fetch, Decode and Execute pipeline stages at the positive edge of an interrupt signal and causes the Execution of interrupt service routine. 
After the interrupt service routine has been executed, normal flow of the application program continues.
