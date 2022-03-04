module main(gcd,rst,clk,
Num1,Num2,
start,interrupt_sig);
//////////////////////////////// I/O /////////////////////////////
input [4:0] Num1;
input [4:0] Num2;
input rst,clk,start,interrupt_sig; 
output reg [31:0] gcd;
//////////////////////////// wire ///////////////////////////////////

wire [31:0] gcd_wire;

wire [31:0] pc;
wire [31:0] instruction;


wire [6:0] op;
wire [4:0] rd;
wire [4:0] rs1;
wire [4:0] rs2;
wire [2:0] func3;
wire [6:0] func7;
wire [31:0] imm32;
wire wr_reg;

wire [31:0] Port_A;
wire [31:0] Port_B;
wire [31:0] Din;

wire ALU_src;
wire lt_flag;
wire zero_flag;
wire [3:0] Function;

wire [31:0] mux_ALU_src_o;
wire [31:0] Result;

wire [31:0] Data_Out;
wire wr_mem;
wire read_mem;
wire start;

wire pc_source;
wire [4:0] rd_D;
wire [4:0] rd_E;
wire [4:0] rd_M;
wire [4:0] rd_W;

////////////////////////////////////----Wires for first pipeline reg----////////////////////////////////////////////////////////
wire [31:0] instruction_F;
wire [6:0] op_F;
wire [4:0] rd_F;
wire [4:0] rs1_F;
wire [4:0] rs2_F;
wire [2:0] func3_F;
wire [6:0] func7_F;
wire [31:0] imm32_F;

wire [31:0]  instruction_D;
wire [6:0] op_D;
wire [4:0] rs1_D;
wire [4:0] rs2_D;
wire [2:0] func3_D;
wire [6:0] func7_D;
wire [31:0] imm32_D;
////////////////////////////////////----Wires for second pipeline reg---//////////////////////////////////////////////////////


  wire [3:0] Function_D;
  wire ALU_src_D,Result_src_D,pc_source_D,wr_reg_D,wr_mem_D,read_mem_D;
  wire [31:0] Port_A_D, Port_B_D;
  
  
  wire [3:0] Function_E;
  wire ALU_src_E,Result_src_E,pc_source_E,wr_reg_E,wr_mem_E,read_mem_E;
  wire [31:0] Port_A_E, Port_B_E, imm32_E;
  wire [31:0]  instruction_E;
  wire [6:0] op_E;
  wire [4:0] rs1_E;
  wire [4:0] rs2_E;
  wire [2:0] func3_E;
  wire [6:0] func7_E;
////////////////////////////////////----Wires for third pipeline reg----///////////////////////////////////////////////////////

  
  
  wire [31:0] Result_E;
  
 
  wire [3:0] Function_M;
  wire ALU_src_M,Result_src_M,pc_source_M,wr_reg_M,wr_mem_M,read_mem_M;
  wire [31:0] Result_M,Port_B_M;
  wire [31:0]  instruction_M;
  wire [6:0] op_M;
  wire [4:0] rs1_M;
  wire [4:0] rs2_M;
  wire [2:0] func3_M;
  wire [6:0] func7_M;
  wire [31:0] imm32_M;
///////////////////////////////////----Wire for the fourth register----///////////////////////////////////////////////

wire Result_src_W,pc_source_W,wr_reg_W;
wire [31:0] Data_Out_W,Result_W,Data_Out_M;
wire [31:0]  instruction_W;
wire [6:0] op_W;
wire [4:0] rs1_W;
wire [4:0] rs2_W;
wire [2:0] func3_W;
wire [6:0] func7_W;
wire [31:0] imm32_W;

/////////////////////////////////----Wires for Hazard Unit-------///////////////////////////////////////////////////
wire stall_pctoF,stall_FtoD,flush_E,flush_D,flush_F;
wire [31:0] op_A_out,op_B_out,instruction_out;

////////////////////////////////-----wire for interrupt controller----------//////////////////////////////////////
wire [31:0] pc_F,pc_D,pc_E,jumpto_address;
wire flush_D_int,flush_E_int,flush_F_int,interrupt_ack,pc_source2;
wire handler_done;//from cu


// MODULE Instantiations	
interrupt_controller ic (.flush_E_int(flush_E_int),.flush_D_int(flush_D_int),.flush_F_int(flush_F_int),.jumpto_address(jumpto_address),.pc_source2(pc_source2),.interrupt_sig(interrupt_sig),.handler_done(handler_done),.interrupt_ack(interrupt_ack),.pc_E(pc_E),.clk(clk));
 
 
adress_generator a_g (.interrupt_ack(interrupt_ack),.pc(pc_F),.clk(clk),.instruction(instruction_F),.pc_source(pc_source),.pc_source2(pc_source2),.start(start),.stall_pctoF(stall_pctoF),.instruction_E(instruction_E),.jumpto_address(jumpto_address));
Instruction_Memory I_M (.pc(pc_F),.instruction(instruction_F)); 
Instruction_fetch I_f (.instruction_out(instruction_out),.op(op_F),.rd(rd_F),.rs1(rs1_F),.rs2(rs2_F),.func3(func3_F),.func7(func7_F),.imm32(imm32_F),.instruction(instruction_F),.rst(rst),.flush_F(flush_F),.flush_F_int(flush_F_int));

insmem_regfile r1 (.instruction_D(instruction_D),.pc_D(pc_D),.op_D(op_D),.rd_D(rd_D),.rs1_D(rs1_D),.rs2_D(rs2_D),.func3_D(func3_D),.func7_D(func7_D),.imm32_D(imm32_D),.instruction_F(instruction_out),.pc_F(pc_F),.op_F(op_F),.rd_F(rd_F),.rs1_F(rs1_F),.rs2_F(rs2_F),.func3_F(func3_F),.func7_F(func7_F),.imm32_F(imm32_F),.clk(clk),.stall_FtoD(stall_FtoD),.flush_D(flush_D),.flush_D_int(flush_D_int));////  PIPELINE REGISTER 1

register_file  r_f (.gcd(gcd_wire),.Port_A(Port_A_D),.Port_B(Port_B_D),.Din(Din),.Addr_A(rs1_D),.Addr_B(rs2_D),.Addr_Wr(rd_W), .wr_reg(wr_reg_W),.clk(clk));//Din ko check krna hai
control_unit c_u (.Function(Function_D),.handler_done(handler_done),.ALU_src(ALU_src_D),.Result_src(Result_src_D),.pc_source(pc_source),.wr_reg(wr_reg_D),.wr_mem(wr_mem_D),.read_mem(read_mem_D),.func3(func3_D),.func7(func7_D),.op(op_D),.zero_flag(zero_flag),.lt_flag(lt_flag),.op_E(op_E),.func3_E(func3_E));

Hazard_Cotrol_Unit HCU (
///Output-->stall and flush signals to PC and pipeline registers
.stall_pctoF(stall_pctoF),.stall_FtoD(stall_FtoD),.flush_E(flush_E),.flush_D(flush_D),.flush_F(flush_F),
///Output--> operands A and operands B of ALU which is in execution state
.op_A_out(op_A_out),.op_B_out(op_B_out),
///Input--> Destination registers of instructions in different pipeline stages
.rd_D(rd_D),.rd_E(rd_E),.rd_M(rd_M),.rd_W(rd_W),
///Input--> Source registers 1 and 2 from different pipeline stages
.rs1_D(rs1_D),.rs1_E(rs1_E),.rs1_M(rs1_M),.rs1_W(rs1_W),
.rs2_D(rs2_D),.rs2_E(rs2_E),.rs2_M(rs2_M),.rs2_W(rs2_W),
///Input--> Write registers flags of instructions in different pipeline stages 
.wr_reg_E(wr_reg_E),.wr_reg_M(wr_reg_M),.wr_reg_W(wr_reg_W),
///Input--> operands A and operands B of ALU which is in execution state
.op_A_in(Port_A_E),.op_B_in(mux_ALU_src_o),
///Input-->Final data that is written to register file during writeback stage
.Data_Out_W(Data_Out_W),
///Input-->ALU Result of the Instructions in different pipeline stages
.Result_M(Result_M),.Result_W(Result_W),
///Input-->Opcode of Instructions from different pipeline stages
.op_E(op_E),.op_M(op_M),.op_W(op_W),
///Input-->PC-source signal from CU.....
.pc_source(pc_source)
);

 regfile_muxALUsrc r2 (
//control-sig-input
.Function_D(Function_D),.ALU_src_D(ALU_src_D),.Result_src_D(Result_src_D),.wr_reg_D(wr_reg_D),.wr_mem_D(wr_mem_D),.read_mem_D(read_mem_D),
//control-sig-output
.Function_E(Function_E),.ALU_src_E(ALU_src_E),.Result_src_E(Result_src_E),.wr_reg_E(wr_reg_E),.wr_mem_E(wr_mem_E),.read_mem_E(read_mem_E),
//input coming from register file
.Port_A_D(Port_A_D), .Port_B_D(Port_B_D),
//output corresponding to the input coming from register file
.Port_A_E(Port_A_E), .Port_B_E(Port_B_E),
//input coming from instruction fetch
.imm32_D(imm32_D),
//output corresponding to input coming from instruction fetch
.imm32_E(imm32_E),
//input from instruction fetch
.rd_D(rd_D),
//output correspnding to input from instruction fetch
.rd_E(rd_E),
//input instruction from r2
.instruction_D(instruction_D),.pc_D(pc_D),.op_D(op_D),.rs1_D(rs1_D),.rs2_D(rs2_D),.func3_D(func3_D),.func7_D(func7_D),
//output instruction to r3
.instruction_E(instruction_E),.pc_E(pc_E),.op_E(op_E),.rs1_E(rs1_E),.rs2_E(rs2_E),.func3_E(func3_E),.func7_E(func7_E),
//clock signal
.clk(clk),.flush_E(flush_E),.flush_E_int(flush_E_int)
); //pipeline reg 2

mux_ALU_src m_A_s (.o(mux_ALU_src_o),.a(Port_B_E),.b(imm32_E),.sel(ALU_src_E));
ALU ALU_inst (.Result(Result_E),.alu_z(zero_flag),.lt_flag(lt_flag),.Op_A(op_A_out),.Op_B(op_B_out), .Function(Function_E));

 ALU_datamem r3 (
//control-sig-input
.Function_E(Function_E),.ALU_src_E(ALU_src_E),.Result_src_E(Result_src_E),.wr_reg_E(wr_reg_E),.wr_mem_E(wr_mem_E),.read_mem_E(read_mem_E),
//control-sig-output
.Function_M(Function_M),.ALU_src_M(ALU_src_M),.Result_src_M(Result_src_M),.wr_reg_M(wr_reg_M),.wr_mem_M(wr_mem_M),.read_mem_M(read_mem_M),
//input from ALU
.Result_E(Result_E),
//output corresponding to the input from ALU
.Result_M(Result_M),
//misc input
.Port_B_E(Port_B_E),
//misc outout
.Port_B_M(Port_B_M),
//input coming from r2
.imm32_E(imm32_E),
//output to r4
.imm32_M(imm32_M),
//input from r2
.rd_E(rd_E),
//output correspnding to input from r2
.rd_M(rd_M),
//input instruction from r3
.instruction_E(instruction_E),.op_E(op_E),.rs1_E(rs1_E),.rs2_E(rs2_E),.func3_E(func3_E),.func7_E(func7_E),
//output instruction to r4
.instruction_M(instruction_M),.op_M(op_M),.rs1_M(rs1_M),.rs2_M(rs2_M),.func3_M(func3_M),.func7_M(func7_M),
//clock
.clk(clk)
);//PIPELINE REGISTER-3

Data_Memory D_M (.Data_Out(Data_Out_M),.Data_In(Port_B_M),.D_Addr(Result_M),.wr_mem(wr_mem_M),.clk(clk),.read_mem(read_mem_M),
.Num1(Num1),.Num2(Num2)
);

datamem_muxresultsrc r4 (

//control-sig-input
.Result_src_M(Result_src_M),.wr_reg_M(wr_reg_M),
//control-sig-output
.Result_src_W(Result_src_W),.wr_reg_W(wr_reg_W),
//misc input
.Result_M(Result_M),
//misc output
.Result_W(Result_W),
//input from data memory
.Data_Out_M(Data_Out_M),
//output corresponding to input from data memory
.Data_Out_W(Data_Out_W),
//input coming from r2
.imm32_M(imm32_M),
//output to r4
.imm32_W(imm32_W),
//input from r3
.rd_M(rd_M),
//output correspnding to input from r3
.rd_W(rd_W),
//input instruction from r3
.instruction_M(instruction_M),.op_M(op_M),.rs1_M(rs1_M),.rs2_M(rs2_M),.func3_M(func3_M),.func7_M(func7_M),
//output instruction to r4
.instruction_W(instruction_W),.op_W(op_W),.rs1_W(rs1_W),.rs2_W(rs2_W),.func3_W(func3_W),.func7_W(func7_W),
//clock
.clk(clk)
);
 



mux_Result_src  m_R_s (.o(Din),.a(Result_W),.b(Data_Out_W),.sel(Result_src_W));

always @ (*)
  begin
    gcd<=gcd_wire;
  end
endmodule


/////////////////////////////////////////////MODULE DEFINITIONS///////////////////////////////////////////////////
module Instruction_Memory (pc,instruction);
	input [31:0] pc;
	output reg [31:0] instruction;
		
  always @ (pc)
   begin	
    case(pc)
       32'h00:instruction <= 32'h01400293;//to test interrupts  addi x5,x0,20
       32'h04:instruction <= 32'h00a00313;//                    addi x6,x0,10
       32'h08:instruction <= 32'h00a00393;//                    addi x7,x0,10
       32'h0C:instruction <= 32'h00a00413;//                    addi x8,x0,10
       32'h10:instruction <= 32'h00500493;//                    addi x9,x0,5
       
	   
       32'h40:instruction <= 32'h00500213;// addi x4,x0,5
       32'h44:instruction <= 32'h30200073;//mret
   
   
   
   
       





    //32'h00:instruction <= 32'h00902103;//ldw x2,0x9(x0)
	//32'h04:instruction <= 32'h00A02183;//ldw x2,0xA(x0)
    //32'h08:instruction <= 32'h00310C63;
    //32'h0C:instruction <= 32'h00314663;
    //32'h10:instruction <= 32'h40310133;
    //32'h14:instruction <= 32'hFE000AE3;
    //32'h18:instruction <= 32'h402181B3;
    //32'h1C:instruction <= 32'hFE0006E3;
    //32'h20:instruction <= 32'h00000063;
       
      
	 
	    default:instruction <= 32'h00000033;  //ADD  x0,x0,x0 
    endcase
   end
endmodule



//////////////////////////////////////////////////////////////////////////////////////////////


 module Instruction_fetch (instruction_out,op,rd,rs1,rs2,func3,func7,imm32,instruction,rst,flush_F,flush_F_int);
  
  output reg [6:0] op;
  output reg [4:0] rd;
  output reg [4:0] rs1;
  output reg [4:0] rs2;
  output reg [2:0] func3;
  output reg [6:0] func7;
  output reg [31:0] imm32,instruction_out;

  reg clr;
  
  input [31:0] instruction;
  input rst,flush_F,flush_F_int;
  initial begin
    clr=0;
  end
  
  always @ (*)
    begin
	  if((flush_F==1'b1) || (flush_F_int==1'b1))
	    begin
		    op <= 7'b0000001;
		    rd <= 5'b00000;
			func3 <= 3'b000;
			func7 <= 7'b0000000;
		    rs1 <= 5'b00000;
			rs2 <= 5'b00000;
			imm32 <= 32'd0;
			instruction_out=32'h00000013;
		    clr=~clr;
		end
	  else
	    begin
		  instruction_out<=instruction;
          case (instruction[6:0])
          
            7'b0010011 : // I-type instruction
            begin
              op <= instruction[6:0];
              rd <= instruction[11:7];
          	func3 <= instruction[14:12];
          	func7 <= 7'b0000000;
              rs1 <= instruction[19:15];
          	rs2 <= 5'b00000;
          	imm32 <= {{20{instruction[31]}} ,instruction [31:20]};
          	
          	
            end
            7'b0110011 : // R-type instruction
            begin
              op <= instruction[6:0];
          	rd <= instruction[11:7];
          	func3 <= instruction[14:12];
              rs1 <= instruction[19:15];
          	rs2 <= instruction[24:20];
          	func7 <= instruction[31:25];
          	imm32 <= 32'd0;
          	
            end			 
            7'b1100011 : // B-type instruction
            begin
              op <= instruction[6:0];
              func3 <= instruction[14:12];
              rs1 <= instruction[19:15];
          	rs2 <= instruction[24:20];
          	func7 <= 7'b0000000;
          	imm32 <= {{20{instruction[31]}}, instruction[7],  instruction[30:25], instruction[11:8], 1'b0};
          	rd <= 5'b00000;
            end	
            
            7'b0000011 : // LW-type instruction
            begin
              op <= instruction[6:0];
              rd <= instruction[11:7];
          	func3 <= instruction[14:12];
          	func7 <= 7'b0000000;
              rs1 <= instruction[19:15];
          	rs2 <= 5'b00000;
          	imm32 <= {{20{instruction[31]}} ,instruction [31:20]};
            end 
			
			7'b1110011 : // mret instruction
            begin
              op <= instruction[6:0];
              rd <= 5'd0;
          	func3 <= 5'd0;
          	func7 <= 7'b0000000;
              rs1 <= 5'd0;
          	rs2 <= 5'b00000;
          	imm32 <= 32'd0;  //func12 ignored
            end 
			
			
			
			
          endcase
	    end
	end
endmodule


/////////////////////////////////////////////////////////////////////////////////////////////

module register_file(gcd,Port_A, Port_B, Din, Addr_A, Addr_B, Addr_Wr, wr_reg,clk);
			output reg [31:0] Port_A, Port_B;			// Data to be provided from register to execute the instruction
			output reg [31:0] gcd;
			input [31:0] Din;						// Data to be loaded in the register
			input [4:0] Addr_A, Addr_B, Addr_Wr;	// Address (or number) of register to be written or to be read
			input wr_reg,clk;								// input wr flag input
			reg [31:0] Reg_File [31:0];				// Register file
			//reg [31:0] temp;
  initial begin
  Reg_File[0]=32'b0;
  end
  
  always @ (*)// (posedge clk)	
	begin
      Port_A = Reg_File[Addr_A];
      Port_B = Reg_File[Addr_B];
	  gcd = Reg_File[2];
	 // temp =Reg_File[Addr_B];
	end 
	
  always @ (negedge clk)
	begin
	  if(wr_reg==1)
	    Reg_File[Addr_Wr] <= Din;
		//gcd <= Reg_File[2];
		//gcd <= 32'd1;
	end
	
	
endmodule


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


module ALU(Result, alu_z,lt_flag,Op_A, Op_B, Function);
	output reg [31:0] Result;		// 32 bit Result
	output reg alu_z,lt_flag;				// Zero flag (1 if result is zero) lt_flag==1 if blt true
	input [31:0] Op_A, Op_B;	    // Two operands based on the type of instruction....Op_A=rs1
	input [3:0] Function;			// Function to be performed as per instructed by Control Unit
	
	
	initial begin
	alu_z=1'b0;
	
	end
	
	always @ ( Op_A or Op_B or Function)
	  begin
	    case (Function)
		  
		  
			4'b0000 :   //Add
			  begin
			  Result = Op_A+Op_B;
			  if(Result==0)
			    alu_z = 1;
			  else
			    alu_z =0;
			  end
		  
		    4'b0001 :   //SUB
			  begin
			  Result = Op_A-Op_B;
			  if(Result==0)
			    alu_z = 1;
			  else
			    alu_z =0;	
			  end
			  
			4'b0010 :   //MUL
			  begin
			  Result = Op_A*Op_B;
			  if(Result==0)
			    alu_z = 1;
			  else
			    alu_z =0;
			  end
			  
			4'b0011 :   //Divide
			  begin
			  Result = Op_A/Op_B;
			  if(Result==0)
			    alu_z = 1;
		      else
			    alu_z =0;
			  end 
			  
			4'b0100 :   //And
			  begin
			  Result = Op_A & Op_B;
			  if(Result==0)
			    alu_z = 1;
			  else
			    alu_z =0;	
			  end
			  
			4'b0101 :   //Or
			  begin
			  Result = Op_A | Op_B;
			  if(Result==0)
			    alu_z = 1;
			  else
			    alu_z =0;
			  end
			
			4'b0110 :   //LSL
			  begin
			  Result = Op_A << Op_B;
			  if(Result==0)
			    alu_z = 1;
			  else
			    alu_z =0;
			  end
			  
			  
			4'b0110 :   //LSR
			  begin
			  Result = Op_A >> Op_B;
			  if(Result==0)
			    alu_z = 1;
			  else
			    alu_z =0;
			  end
			  
			  
			4'b0111 :   //BLT
			  begin
			  if(Op_A<Op_B)
			    lt_flag = 1;
			  end
		      
		endcase

	  end 
endmodule




////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module control_unit(Function,handler_done,ALU_src,Result_src,pc_source,wr_reg,wr_mem,read_mem,func3,func7,op,zero_flag,lt_flag,op_E,func3_E);

  output reg wr_reg,wr_mem,read_mem;
  output  reg [3:0] Function;//ALU operation
  output reg ALU_src,Result_src,pc_source,handler_done;


  input [6:0] func7,op,op_E;
  input [2:0] func3,func3_E;
  input zero_flag,lt_flag;
  
  always @ (*)
    begin
	case ({op,func3,func7})
	  17'b01100110000000000:  //ADD
	    begin
		Function <=  4'b0000;
		wr_reg <= 1;
		ALU_src <=0;
		Result_src <=0;
		
		wr_mem <= 0;
		read_mem <= 0;
		end
		
		17'b01100110000100000: //SUB
	    begin
		Function <=  4'b0001;
		wr_reg <= 1;
		ALU_src <=0;
		Result_src <=0;
		
		wr_mem <= 0;
		read_mem <= 0;
		end
		
		17'b01100110000000001: // mul
	    begin
		Function <=  4'b0010;
		wr_reg <= 1;
		ALU_src <=0;
		Result_src <=0;
		 
		wr_mem <= 0;
		read_mem <= 0;
		end
		
		17'b01100111000000001: //divide
	    begin
		Function <=  4'b0011;
		wr_reg <= 1;
		ALU_src <=0;
		Result_src <=0;
		
		wr_mem <= 0;
		read_mem <= 0;
		end
		
		17'b01100111110000000: // and 
	    begin
		Function <=  4'b0100;
		wr_reg <= 1;
		ALU_src <=0;
		Result_src <=0;
		 
		wr_mem <= 0;
		read_mem <= 0;
		end
		
		17'b01100111100000000: // or
	    begin
		Function <=  4'b0101;
		wr_reg <= 1;
		ALU_src <=0;
		Result_src <=0;
		 
		wr_mem <= 0;
		read_mem <= 0;
		end
		
		17'b11000110000000000: // beq
		begin
		Function <=  4'b0001;
		wr_reg <= 0;
		ALU_src <=0;
		Result_src <=0;
		wr_mem <= 0;
		read_mem <= 0;
		end
		
		17'b11000111000000000: // blt
		begin
		Function <=  4'b0111;
		wr_reg <= 0;
		ALU_src <=0;
		Result_src <=0;
		wr_mem <= 0;
		read_mem <= 0;
		
		end
		
		17'b00100110000000000: //ADDI
		begin
		Function <=  4'b0000;
		wr_reg <= 1;
		ALU_src <=1;//Immediate value will be selected instead of rs2
		Result_src <=0;
		 
		wr_mem <= 0;
		read_mem <= 0;
		end
		
		17'b00000110100000000: // LW
		begin
		Function <=  4'b0000;
		wr_reg <= 1;
		ALU_src <=1;//Immediate value will be selected instead of rs2
		Result_src <=1;
		
		wr_mem <= 0;
		read_mem <= 1;
	
		end
		
	endcase	
	if(op_E==7'b1100011)
	 begin
	   if((func3_E==3'b000)&& (zero_flag==1))
		  pc_source <= 1; 
		else if((func3_E==3'b100)&& (lt_flag==1))
		  pc_source <= 1;
		else
		  pc_source <= 0;  
		
	 end
	
	else
	  pc_source <= 0; 	
	if(op==7'b1110011)
		handler_done=1;
	else
	    handler_done=0;
    end
endmodule



///////////////////////////////////////////////////////////////////////////////////////////////////////////////

module mux_ALU_src(output reg [31:0]o,		// 32 bit output
					input[31:0]a,b,		// 32 bit inputs
					input sel			// Selection Signal
			);
			
  always @ (*)
    begin
      if(sel==0)
	    o <= a;
	  if(sel==1)
	    o <= b;
	end
endmodule


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module mux_Result_src(output reg [31:0]o,		// 32 bit output
					input[31:0]a,b,		// 32 bit inputs
					input sel			// Selection Signal
			);
			
  always @ (*)
    begin
      if(sel==0)
	    o <= a;
	  if(sel==1)
	    o <= b;
	end
endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////
module Data_Memory(Data_Out,Data_In,D_Addr,wr_mem,clk,read_mem,
Num1,Num2
);
		
		output reg [31:0] Data_Out;
		input [31:0] Data_In;
		input [31:0] D_Addr;
		input wr_mem,clk,read_mem;
		input [4:0] Num1;
		input [4:0] Num2;
		reg [31:0] Mem [255:0];		// Data Memory
		reg [31:0] temp;
		
		initial begin
		Mem[8'd9] = 32'd4;
        Mem[8'd10]= 32'd8;	
		
		end
		
		always @ (posedge clk)
		  begin
		  if(wr_mem==1'b1) begin
		     Mem[D_Addr] <= Data_In;
		  end
		   Mem[32'd9]  <= {{27{1'b0}},Num1};
		   Mem[32'd10] <= {{27{1'b0}},Num2};
		 end
		always @ (D_Addr)
		  begin
		    if(read_mem==1'b1)
			  begin
			    temp <=Mem[D_Addr];
			    Data_Out <= Mem[D_Addr];
		      end
		  end
		
		

	
endmodule 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module adress_generator (interrupt_ack,pc,clk,instruction,pc_source,pc_source2,jumpto_address,start,stall_pctoF,instruction_E);	
  output reg  [31:0] pc; 
  output reg  interrupt_ack;
  
  input clk,pc_source,start,stall_pctoF,pc_source2;
  input [31:0] instruction,instruction_E,jumpto_address;
  reg   [31:0] immext;
  
  
  initial begin
    pc= 32'd0;
	interrupt_ack=0;
    end
	
  always @ (negedge clk)
  begin
    if(interrupt_ack==1'b1)
	  interrupt_ack=1'b0;
  end
	
	
	
  always @ (posedge clk)
	begin
	  if(start==1'b1)
	    begin
		  if(stall_pctoF==1'b0)
		    begin
              /* case(instruction[6:0])
              
              
            	7'b1100011:    // B-Type Instruction
            	  begin
            		if(pc_source== 1'b1)
            		  begin
            			immext = {{20{instruction[31]}}, instruction[7],  instruction[30:25], instruction[11:8], 1'b0}; 
            			pc = pc+ immext;
            		  end
            		else
            		  begin
            			pc = pc+4;
            		  end
            		  
            		  
            	  end	
            
            	7'b0010011:   // I-Type Instruction
            	  begin
            		pc = pc+4;
            		//$display("in");
            	  end
            	
            	7'b0110011:   // R-Type Instruction
            	  begin
            		pc= pc+4;
            	  end
            	7'b0000011://    I-Type-Load-Instruction  
            	  begin
            		pc= pc+4;
            	  end 
            	
            	
              endcase	 */
			  
			    if(pc_source2==1'b1)
				begin
				pc = jumpto_address;
				interrupt_ack =1'b1;
				
				end
			    else if(instruction_E[6:0]==7'b1100011)//branch
				  begin
				    if(pc_source== 1'b1)
            		  begin
            			immext = {{20{instruction_E[31]}}, instruction_E[7],  instruction_E[30:25], instruction_E[11:8], 1'b0}; 
            			pc = pc+ immext - 8;
            		  end
            		else
            		  begin
            			pc = pc+4;
            		
                      end
				  end
				else
            		  begin
            			pc = pc+4;
            		  end  
				  
				  
				  
			  
			  
			  
			  
			  
			  
			  
			  
            end
	    end
    end
endmodule
////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     ____            ____   ____                                              ____     // 
 //                    |    |  =====   |    | |                      |       =====    ||    |   |         //
 //                    |____|    |     |____| |                      |         |      | |   |   |         //
 //                    |         |     |      |____                  |         |      |  |  |   |____     //
 //                    |         |     |      |                      |         |      |   | |   |         //
 //                    |       =====   |      |____                  |_____  =====    |    ||   |____     //
 //					                                                                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////////////      

module insmem_regfile(instruction_D,pc_D,op_D,rd_D,rs1_D,rs2_D,func3_D,func7_D,imm32_D,instruction_F,pc_F,op_F,rd_F,rs1_F,rs2_F,func3_F,func7_F,imm32_F,clk,stall_FtoD,flush_D,flush_D_int);
  output reg [31:0] instruction_D,pc_D;
  output reg [6:0] op_D;
  output reg [4:0] rd_D;
  output reg [4:0] rs1_D;
  output reg [4:0] rs2_D;
  output reg [2:0] func3_D;
  output reg [6:0] func7_D;
  output reg [31:0] imm32_D;
  
  input [31:0] instruction_F,pc_F;
  input [6:0] op_F;
  input [4:0] rd_F;
  input [4:0] rs1_F;
  input [4:0] rs2_F;
  input [2:0] func3_F;
  input [6:0] func7_F;
  input [31:0] imm32_F;
  input clk,stall_FtoD,flush_D,flush_D_int;
  
  initial
    begin
	        op_D    <= 0;
		    rd_D    <= 0;
			func3_D <= 0;
			func7_D <= 0;
		    rs1_D   <= 0;
			rs2_D   <= 0;
			imm32_D <= 0;
		    instruction_D <= 0;
	
	
	end
    always @ (*)
    begin
	   if((flush_D==1'b1) || (flush_D_int==1'b1))
	    begin
		    op_D <= 7'b0000001;
		    rd_D <= 5'b00000;
			func3_D <= 3'b000;
			func7_D <= 7'b0000000;
		    rs1_D <= 5'b00000;
			rs2_D <= 5'b00000;
			imm32_D <= 32'd0;
		    instruction_D <= 32'h00000013;

		end 
	end
	
	
	
	
	
	
	
	
	
  always @ (posedge clk)
    begin
	  /* if(flush_D==1'b1)
	    begin
		    op_D <= 7'b0000001;
		    rd_D <= 5'b00000;
			func3_D <= 3'b000;
			func7_D <= 7'b0000000;
		    rs1_D <= 5'b00000;
			rs2_D <= 5'b00000;
			imm32_D <= 32'd0;
		    instruction_D <= 32'h00000013;

		end 
	  else*/
     	if(stall_FtoD==1'b0)
	    begin
          instruction_D <= instruction_F;
          op_D          <= op_F;
          rd_D          <= rd_F;
          rs1_D         <= rs1_F;
          rs2_D         <= rs2_F;
          func3_D       <= func3_F;
          func7_D       <= func7_F;
          imm32_D       <= imm32_F;
		  pc_D          <= pc_F;
        end
	  	
		
		
	end
endmodule



////////////////////////////////////////////////////////////////////////////////////////////////////////////


module regfile_muxALUsrc(
//control-sig-input
Function_D,ALU_src_D,Result_src_D,wr_reg_D,wr_mem_D,read_mem_D,
//control-sig-output
Function_E,ALU_src_E,Result_src_E,wr_reg_E,wr_mem_E,read_mem_E,
//input coming from register file
Port_A_D, Port_B_D,
//output corresponding to the input coming from register file
Port_A_E, Port_B_E,
//input coming from instruction fetch
imm32_D,
//output corresponding to input coming from instruction fetch
imm32_E,
//input from decode stage
rd_D,
//output to execute stage
rd_E,
//input instruction from decode stage
instruction_D,pc_D,op_D,rs1_D,rs2_D,func3_D,func7_D,
//output instruction to execute stage
instruction_E,pc_E,op_E,rs1_E,rs2_E,func3_E,func7_E,
//clock signal
clk,flush_E,flush_E_int
);
  input [3:0] Function_D;
  input ALU_src_D,Result_src_D,wr_reg_D,wr_mem_D,read_mem_D;
  input [31:0] Port_A_D, Port_B_D, imm32_D,instruction_D,pc_D;
  input [4:0] rd_D;
  input clk,flush_E,flush_E_int;
  
  input [6:0] op_D;
  input [4:0] rs1_D;
  input [4:0] rs2_D;
  input [2:0] func3_D;
  input [6:0] func7_D;
  
  
  
  output reg [3:0] Function_E;
  output reg ALU_src_E,Result_src_E,wr_reg_E,wr_mem_E,read_mem_E;
  output reg [31:0] Port_A_E, Port_B_E, imm32_E,instruction_E,pc_E;
  output reg [4:0] rd_E;
  
  output reg [6:0] op_E;
  output reg [4:0] rs1_E;
  output reg [4:0] rs2_E;
  output reg [2:0] func3_E;
  output reg [6:0] func7_E;
  
  
  initial
    begin
	  Function_E   <=       0;        
          ALU_src_E    <=   0;        
          Result_src_E <=   0;        
							
          wr_reg_E     <=   0;        
          wr_mem_E     <=   0;        
          read_mem_E   <=   0;        
          Port_A_E     <=   0;        
          Port_B_E     <=   0;        
          imm32_E      <=   0;
          rd_E         <=   0;
          instruction_E<=   0;	
          op_E         <=   0;
          rs1_E        <=   0;
          rs2_E        <=   0;
          func3_E      <=   0;
          func7_E      <=   0;  
	
	
	end
  
  
  always @ (*)
    begin
	  if(flush_E_int==1'b1)
	    begin
		    op_E <= 7'b0000001;
		    rd_E <= 5'b00000;
			func3_E <= 3'b000;
			func7_E <= 7'b0000000;
		    rs1_E <= 5'b00000;
			rs2_E <= 5'b00000;
			imm32_E <= 32'd0;
		    instruction_E <= 32'h00000013;
			Function_E   <=0;
			ALU_src_E    <=1'b1;
			Result_src_E <=1'b0;
			  
			wr_reg_E     <=1'b0;//----->//So my ins is not actually nop as wr_reg of nop is 0 here...nothing is written.
			wr_mem_E     <=1'b0;
			read_mem_E   <=1'b0;
			Port_A_E     <=32'd0;
			Port_B_E     <=32'hzzzzzzzz;
			
			
			
		end
	end
   
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  always @ (posedge clk)
    begin
	  if(flush_E==1'b1)
	    begin
		    op_E <= 7'b0000001;
		    rd_E <= 5'b00000;
			func3_E <= 3'b000;
			func7_E <= 7'b0000000;
		    rs1_E <= 5'b00000;
			rs2_E <= 5'b00000;
			imm32_E <= 32'd0;
		    instruction_E <= 32'h00000013;
			Function_E   <=0;
			ALU_src_E    <=1'b1;
			Result_src_E <=1'b0;
			  
			wr_reg_E     <=1'b0;//----->//So my ins is not actually nop as wr_reg of nop is 0 here...nothing is written.
			wr_mem_E     <=1'b0;
			read_mem_E   <=1'b0;
			Port_A_E     <=32'd0;
			Port_B_E     <=32'hzzzzzzzz;
			
			
			
		end
	
	
	
	
	
	  else
	    begin
          Function_E   <=   Function_D  ;        
          ALU_src_E    <=   ALU_src_D   ;        
          Result_src_E <=   Result_src_D;        
            
          wr_reg_E     <=   wr_reg_D    ;        
          wr_mem_E     <=   wr_mem_D    ;        
          read_mem_E   <=   read_mem_D  ;        
          Port_A_E     <=   Port_A_D    ;        
          Port_B_E     <=   Port_B_D    ;        
          imm32_E      <=   imm32_D     ;
          rd_E         <=   rd_D        ;
          instruction_E<=   instruction_D;	
          op_E         <=   op_D        ;
          rs1_E        <=   rs1_D       ;
          rs2_E        <=   rs2_D       ;
          func3_E      <=   func3_D     ;
          func7_E      <=   func7_D     ;
		  pc_E         <=   pc_D        ;
        end



	  
	end

endmodule


////////////////////////////////////////////////////////////////////////////////////////////////////////////////


module ALU_datamem(
//control-sig-input
Function_E,ALU_src_E,Result_src_E,wr_reg_E,wr_mem_E,read_mem_E,
//control-sig-output
Function_M,ALU_src_M,Result_src_M,wr_reg_M,wr_mem_M,read_mem_M,
//input from ALU
Result_E,
//output corresponding to the input from ALU
Result_M,
//misc input
Port_B_E,
//misc outout
Port_B_M,
//input from r2
rd_E,
//output correspnding to input from r2
rd_M,
//input instruction from r3
instruction_E,op_E,rs1_E,rs2_E,func3_E,func7_E,imm32_E,
//output instruction to r4
instruction_M,op_M,rs1_M,rs2_M,func3_M,func7_M,imm32_M,
//clock
clk
);
  input [3:0] Function_E;
  input ALU_src_E,Result_src_E,wr_reg_E,wr_mem_E,read_mem_E;
  input [31:0] Result_E,Port_B_E,instruction_E,imm32_E;
  input [4:0] rd_E;
  input clk;
  input [6:0] op_E;
  input [4:0] rs1_E;
  input [4:0] rs2_E;
  input [2:0] func3_E;
  input [6:0] func7_E;
  
  
  
  
  output reg [3:0] Function_M;
  output reg ALU_src_M,Result_src_M,wr_reg_M,wr_mem_M,read_mem_M;
  output reg [31:0] Result_M,Port_B_M,instruction_M,imm32_M;
  output reg [4:0] rd_M;
  output reg [6:0] op_M;
  output reg [4:0] rs1_M;
  output reg [4:0] rs2_M;
  output reg [2:0] func3_M;
  output reg [6:0] func7_M;
  
   initial
    begin
	  Function_M   <=   0;        
	  ALU_src_M    <=   0;        
	  Result_src_M <=   0;        
						
	  wr_reg_M     <=   0;        
	  wr_mem_M     <=   0;        
	  read_mem_M   <=   0;              
	  Port_B_M     <=   0;   ////------------>>>>>>>>CHECK LATER     
	  Result_M     <=   0; 
      rd_M         <=   0;	
      instruction_M<=   0;
      op_M         <=   0;
      rs1_M        <=   0;
      rs2_M        <=   0;
      func3_M      <=   0;
      func7_M      <=   0;
	  imm32_M      <=   0;
	end                 
  
  
  
   always @ (posedge clk)
    begin
	  Function_M   <=   Function_E  ;        
	  ALU_src_M    <=   ALU_src_E   ;        
	  Result_src_M <=   Result_src_E;        
	         
	  wr_reg_M     <=   wr_reg_E    ;        
	  wr_mem_M     <=   wr_mem_E    ;        
	  read_mem_M   <=   read_mem_E  ;              
	  Port_B_M     <=   Port_B_E    ;   ////------------>>>>>>>>CHECK LATER     
	  Result_M     <=   Result_E    ; 
      rd_M         <=   rd_E        ;	
      instruction_M<=   instruction_E;
      op_M         <=   op_E        ;
      rs1_M        <=   rs1_E       ;
      rs2_M        <=   rs2_E       ;
      func3_M      <=   func3_E     ;
      func7_M      <=   func7_E     ;
	  imm32_M      <=   imm32_E     ;
	end

endmodule


//////////////////////////////////////////////////////////////////////////////////////////////////
module datamem_muxresultsrc(

//control-sig-input
Result_src_M,wr_reg_M,
//control-sig-output
Result_src_W,wr_reg_W,
//misc input
Result_M,
//misc output
Result_W,
//input from data memory
Data_Out_M,
//output corresponding to input from data memory
Data_Out_W,
//input from r3
rd_M,
//output correspnding to input from r3
rd_W,
//input instruction from r3
instruction_M,op_M,rs1_M,rs2_M,func3_M,func7_M,imm32_M,
//output instruction to r4
instruction_W,op_W,rs1_W,rs2_W,func3_W,func7_W,imm32_W,
//clock
clk
);
  
  input Result_src_M,wr_reg_M;
  input [31:0] Data_Out_M,Result_M,instruction_M,imm32_M;
  input [4:0] rd_M;
  input clk;
  input [6:0] op_M;
  input [4:0] rs1_M;
  input [4:0] rs2_M;
  input [2:0] func3_M;
  input [6:0] func7_M;
  
  output reg Result_src_W,wr_reg_W;
  output reg [31:0] Data_Out_W,Result_W,instruction_W,imm32_W;
  output reg [4:0] rd_W;
  output reg [6:0] op_W;
  output reg [4:0] rs1_W;
  output reg [4:0] rs2_W;
  output reg [2:0] func3_W;
  output reg [6:0] func7_W;
  
  initial
    begin       
	  Result_src_W <=   0;        
						
	  wr_reg_W     <=   0;        
	  Data_Out_W   <=   0;
      Result_W     <=   0;
	  rd_W         <=   0;
	  instruction_W<=   0;
	  op_W         <=   0;
      rs1_W        <=   0;
      rs2_W        <=   0;
      func3_W      <=   0;
      func7_W      <=   0;
	  imm32_W      <=   0;
	end                 
   always @ (posedge clk)
    begin       
	  Result_src_W <=   Result_src_M;        
	        
	  wr_reg_W     <=   wr_reg_M    ;        
	  Data_Out_W   <=   Data_Out_M  ;
      Result_W     <=   Result_M    ;
	  rd_W         <=   rd_M        ;
	  instruction_W<=   instruction_M;
	  op_W         <=   op_M        ;
      rs1_W        <=   rs1_M       ;
      rs2_W        <=   rs2_M       ;
      func3_W      <=   func3_M     ;
      func7_W      <=   func7_M     ;
	  imm32_W      <=   imm32_M     ;
	end

endmodule

module Hazard_Cotrol_Unit(
///Output-->stall and flush signals to PC and pipeline registers
stall_pctoF,stall_FtoD,flush_E,flush_D,flush_F,
///Output--> operands A and operands B of ALU which is in execution state
op_A_out,op_B_out,
///Input--> Destination registers of instructions in different pipeline stages
rd_D,rd_E,rd_M,rd_W,
///Input--> Source registers 1 and 2 from different pipeline stages
rs1_D,rs1_E,rs1_M,rs1_W,
rs2_D,rs2_E,rs2_M,rs2_W,
///Input--> Write registers flags of instructions in different pipeline stages 
wr_reg_E,wr_reg_M,wr_reg_W,
///Input--> operands A and operands B of ALU which is in execution state
op_A_in,op_B_in,
///Input-->Final data that is written to register file during writeback stage if instr was ld
Data_Out_W,
///Input-->ALU Result of the Instructions in different pipeline stages
Result_M,Result_W,
///Input-->Opcode of Instructions from different pipeline stages
op_E,op_M,op_W,
///Input-->PC-source signal from CU.....
pc_source
);
  
  input [4:0] rd_D,rd_E,rd_M,rd_W,rs1_D,rs1_E,rs1_M,rs1_W,rs2_D,rs2_E,rs2_M,rs2_W; //registers
  input wr_reg_E,wr_reg_M,wr_reg_W,pc_source;//write reg flags
  input [31:0] op_A_in,op_B_in,Data_Out_W,Result_M,Result_W;//opA and opB of ALU
  input [6:0] op_E,op_M,op_W;
  
  output reg [31:0] op_A_out,op_B_out;
  output reg stall_pctoF,stall_FtoD,flush_E,flush_D,flush_F;
  reg c1,c2;
  initial begin
  
    stall_pctoF = 1'b0;
	stall_FtoD = 1'b0;
	flush_E = 1'b0;
    flush_D =1'b0;
	flush_F=1'b0;
	c1=0;
	c2=0;
  end
  
  
  
  
  always @ (*)
    begin
	//----------------------------------------->RAW DATA HAZARD(Forwarding)<----------------------------------------------\\
	
      
          /////////////////////////////1 instruction gap is present/////////////////////
          if(((wr_reg_W==1) && (rd_W!=5'b00000) && (rd_W==rs1_E)) && (!((wr_reg_M==1) && (rd_M!=5'b00000) && (rd_M==rs1_E))))
		    if(op_W==7'b0000011)
			  begin
			  op_A_out=Data_Out_W  ;// ld instruction
			  end
			else
			  begin
              op_A_out=Result_W;// others
              end
          if(((wr_reg_W==1) && (rd_W!=5'b00000) && (rd_W==rs2_E)) && (!((wr_reg_M==1) && (rd_M!=5'b00000) && (rd_M==rs2_E))))
            if(op_W==7'b0000011)
			  begin
			  op_B_out=Data_Out_W  ;// when there is ld in wb stage
			  end
			else
			  begin
              op_B_out=Result_W;// others
			  end
          
            ////////////////////////////Very Next Instruction //////////////////////////////
          if(((wr_reg_M==1) && (rd_M!=5'b00000) && (rd_M==rs1_E)) &&   (!((wr_reg_W==1) && (rd_W!=5'b00000) && (rd_W==rs1_E))))
            begin
			op_A_out=Result_M;
            
			end
          if(((wr_reg_M==1) && (rd_M!=5'b00000) && (rd_M==rs2_E)) &&    (!((wr_reg_W==1) && (rd_W!=5'b00000) && (rd_W==rs2_E))))
		    begin
            op_B_out=Result_M;
            end
            ////////////////////if source reg matches with destination reg of both of top 2 instructions///////////
          if(((wr_reg_M==1) && (rd_M!=5'b00000) && (rd_M==rs1_E)) &&   (((wr_reg_W==1) && (rd_W!=5'b00000) && (rd_W==rs1_E))))
            begin
			op_A_out=Result_M;
            c2=~c2;
            end
          if(((wr_reg_M==1) && (rd_M!=5'b00000) && (rd_M==rs2_E)) &&   (((wr_reg_W==1) && (rd_W!=5'b00000) && (rd_W==rs2_E))))
            op_B_out=Result_M;  
            
            
          if(!(((wr_reg_W==1) && (rd_W!=5'b00000) && (rd_W==rs1_E))      ||     ((wr_reg_M==1) && (rd_M!=5'b00000) && (rd_M==rs1_E))  ))
            begin 
            op_A_out=op_A_in;
          	//op_B_out=op_B_in;
			
            end
		 if(!(  ((wr_reg_W==1) && (rd_W!=5'b00000) && (rd_W==rs2_E))   ||   ((wr_reg_M==1) && (rd_M!=5'b00000) && (rd_M==rs2_E)) ))
            begin 
            //op_A_out=op_A_in;
          	op_B_out=op_B_in;
			
            end
         
	//----------------------------------------->LOAD-USE RAW DATA HAZARD<----------------------------------------------\\
	  
	  if((op_E==7'b0000011) || (op_E==7'b0000001))  //For stalling
	    begin
		  if(((wr_reg_E==1) && (rd_E!=5'b00000) && (rd_E==rs1_D)))
		    begin 
			  stall_pctoF=1;
			  stall_FtoD=1;
			  flush_E=1;//when this is 1 nop is inserted.
			end
			
			if(((wr_reg_E==1) && (rd_E!=5'b00000) && (rd_E==rs2_D)))
		      begin 
			    stall_pctoF=1'b1; //PC value not given to IM
			    stall_FtoD=1'b1;//Decode stage values not updated
			    flush_E=1'b1;//when this is 1 nop is inserted.E stage values cleared to zero 
			  end
			
			if(((wr_reg_E==1'b0) && (rd_E==5'b00000)))/// stall stopping condition
			  begin 
			    stall_pctoF=1'b0; //PC value not given to IM
			    stall_FtoD=1'b0;//Decode stage values not updated
			    flush_E=1'b0;//when this is 1 nop is inserted.E stage values cleared to zero 
			  end
			
		end
	//----------------------------------------->CONTROL HAZARD<----------------------------------------------\\  
	  //if((op_E==7'b1100011)||(op_E==7'b0000001))  //For Branch Hazard
	   // begin
		  if(pc_source==1)
            begin
			  flush_D=1'b1;
			  flush_F=1'b1;
            end			
		  else
			begin
			  flush_D=1'b0;
			  flush_F=1'b0;
			  c1=~c1;
            end	
			
			
	//	end
	
	
	
	
	
	
	
	end

endmodule



module interrupt_controller(flush_E_int,flush_D_int,flush_F_int,jumpto_address,pc_source2,interrupt_sig,handler_done,interrupt_ack,pc_E,clk);
  input interrupt_sig,handler_done,clk;//handler done will be set by control unit when mret is decoded.
  input interrupt_ack;//generated by address generator when jump to interrupt handler or return address has been made.It will reset all flags.
  input [31:0] pc_E;
  
  output reg flush_E_int,flush_D_int,flush_F_int,pc_source2;
  output reg [31:0] jumpto_address;
  
  reg [31:0] ret_address;
  
  
  initial begin
    flush_F_int = 0;
	flush_D_int = 0;
	flush_E_int = 0;
	pc_source2  = 0;
	jumpto_address=32'h40;
  end
  
  
  always @ (posedge interrupt_sig) //level triggered interrupt
  begin
    flush_F_int = 1;
	flush_D_int = 1;
	flush_E_int = 1;
	pc_source2 = 1;
	ret_address = pc_E;
  end
	always @ (*)
	  begin
	    if(interrupt_ack==1)
		  begin
		    flush_F_int <= 0;
	        flush_D_int <= 0;
	        flush_E_int <= 0;
	        pc_source2  <= 0;
		  end
		if(handler_done==1)
		  begin
		    pc_source2  <= 1;
			jumpto_address <= ret_address;
		  end
	end
endmodule