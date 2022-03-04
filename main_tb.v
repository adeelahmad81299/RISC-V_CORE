`timescale 1ns/1ps
 module main_tb;
   wire [31:0] gcd;
   
  reg clk,rst,start,interrupt_sig;
  reg [4:0] Num1;
  reg [4:0] Num2;
	
  
  main uut (gcd,rst,clk,
  Num1,Num2,
  start,interrupt_sig);
   
  initial begin
   
    $dumpfile("main_tb.vcd");
    $dumpvars(0,main_tb);
    Num1=5'd12;
    Num2=5'd4;
	interrupt_sig=0;
	#100;
	start=1;
	#48000
	interrupt_sig=1;
	#3000
	interrupt_sig=0;
    #2000000
     $finish;
  end
  always begin
         clk=0;
    #6000 clk=1;
    #6000;
  end
endmodule