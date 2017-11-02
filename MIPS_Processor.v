/******************************************************************

* Description

*	This is the top-level of a MIPS processor that can execute the next set of instructions:

*		add

*		addi

*		sub

*		ori

*		or

*		bne

*		beq

*		and

*		nor

* This processor is written Verilog-HDL. Also, it is synthesizable into hardware.

* Parameter MEMORY_DEPTH configures the program memory to allocate the program to

* be execute. If the size of the program changes, thus, MEMORY_DEPTH must change.

* This processor was made for computer organization class at ITESO.

* Version:

*	1.0

* Author:

*	Dr. José Luis Pizano Escalante

* email:

*	luispizano@iteso.mx

* Date:

*	12/06/2016

******************************************************************/





module MIPS_Processor

#(

	parameter MEMORY_DEPTH = 256

)



(

	// Inputs

	input clk,

	input reset,

	input [7:0] PortIn,

	// Output

	output [31:0] ALUResultOut,

	output [31:0] PortOut

);

//******************************************************************/

//******************************************************************/

assign  PortOut = 0;



//******************************************************************/

//******************************************************************/

// Data types to connect modules
wire JR_wire;

wire Jal_wire;

wire Jump_wire;

wire MemRead_wire;

wire MemWrite_wire;

wire MemToReg_wire;

wire Lui_wire;

wire Branch;

wire BranchNE_wire;

wire BranchEQ_wire;

wire RegDst_wire;

wire NotZeroANDBrachNE;

wire ZeroANDBrachEQ;

wire ORForBranch;

wire ALUSrc_wire;

wire RegWrite_wire;

wire Zero_wire;

wire [2:0] ALUOp_wire;

wire [3:0] ALUOperation_wire;

wire [4:0] WriteRegister_wire;

wire [4:0] MuxJalOutput_wire;

wire [31:0] RegisterFile31_wire;

wire [31:0] BranchAddress_wire;

wire [31:0] MUX_BranchOutput_wire;

wire [31:0] MUX_PC_wire;

wire [31:0] PC_wire;

wire [31:0] Instruction_wire;

wire [31:0] ReadData1_wire;

wire [31:0] ReadData2_wire;

wire [31:0] InmmediateExtend_wire;

wire [31:0] ReadData2OrInmmediate_wire;

wire [31:0] ALUResult_wire;

wire [31:0] PC_4_wire;

wire [31:0] BranchAdderOutput_wire;

wire [31:0] InmmediateExtendAnded_wire;

wire [31:0] PCtoBranch_wire;

wire [31:0] LuiOutput_wire;

wire [31:0] MuxLuiOutput_wire;

wire [31:0] RAM_Addr_wire;

wire [31:0] RAM_ReadData_wire;

wire [31:0] RAMLUIALU_wire;

wire [31:0] JumpAddress_wire;

wire [31:0] NewPC_wire;
wire [31:0] DATA_PC4Output_wire;
wire [31:0] NewPCJROutput_wire;

integer ALUStatus;




/////////////// CONTROL UNIT ///////////////////////////

Control

ControlUnit

(

	.OP(Instruction_wire[31:26]),

	.RegDst(RegDst_wire),

	.BranchNE(BranchNE_wire),

	.BranchEQ(BranchEQ_wire),

	.ALUOp(ALUOp_wire),

	.ALUSrc(ALUSrc_wire),

	.RegWrite(RegWrite_wire),

	.Lui(Lui_wire),

	.MemtoReg(MemToReg_wire),

	.MemRead(MemRead_wire),

	.MemWrite(MemWrite_wire),

	.Jump(Jump_wire),
	
	.Jal(Jal_wire)
	
);
//////////// PROGRAM COUNTER //////////////////////////
 PC_Register

#(

	 .N(32)

	 

 )

 

 programCounter

(

	 .clk(clk),

	 .reset(reset),

	 .NewPC(NewPC_wire),

	 .PCValue(PC_wire)

);







ProgramMemory

#(

	.MEMORY_DEPTH(MEMORY_DEPTH)

)

ROMProgramMemory

(

	.Address({22'b0,PC_wire[9:0]}),

	.Instruction(Instruction_wire)

);

////////////////// ADDERS //////////////////////////////
Adder32bits

PC_Puls_4

(

	.Data0(PC_wire),

	.Data1(4),

	

	.Result(PC_4_wire)

);



Adder32bits

AdderForBranches

(

	.Data0(PC_4_wire),

	.Data1(BranchAddress_wire),

	

	.Result(BranchAdderOutput_wire)

);


//////////////// LEFT SHIFTER /////////////////////////////////
ShiftLeft2

BranchAddressShifter 

(   

	.DataInput(InmmediateExtend_wire),

   .DataOutput(BranchAddress_wire)



);





//////////// MULTIPLEXERS //////////////////////////////

Multiplexer2to1

#(

	.NBits(5)

)

MUX_ForRTypeAndIType

(

	.Selector(RegDst_wire),

	.MUX_Data0(Instruction_wire[20:16]),

	.MUX_Data1(Instruction_wire[15:11]),

	

	.MUX_Output(WriteRegister_wire)



);

Multiplexer2to1

#(

	.NBits(5)

)

MUX_JAL
(

	.Selector(Jal_wire),

	.MUX_Data0(WriteRegister_wire),

	.MUX_Data1(RegisterFile31_wire),

	.MUX_Output(MuxJalOutput_wire)
	
);


Multiplexer2to1

#(

	.NBits(32)

)

MUX_DATA_PC4

(

	.Selector(Jal_wire),

	.MUX_Data0(RAMLUIALU_wire),

	.MUX_Data1(PC_4_wire),

	.MUX_Output(DATA_PC4Output_wire)



);

Multiplexer2to1

#(

	.NBits(32)

)

MUX_Branch

(

	.Selector(Branch),

	.MUX_Data0(PC_4_wire),

	.MUX_Data1(BranchAdderOutput_wire),

	.MUX_Output(MUX_BranchOutput_wire)



);

Multiplexer2to1

#(

	.NBits(32)

)

MUX_JR

(

	.Selector(JR_wire),

	.MUX_Data0(NewPC_wire),

	.MUX_Data1(ReadData1_wire),

	.MUX_Output(NewPCJROutput_wire)



);

Multiplexer2to1

#(

	.NBits(32)

)

MUX_Jumps

(

	.Selector(Jump_wire),

	.MUX_Data0(MUX_BranchOutput_wire),

	.MUX_Data1(JumpAddress_wire),

	.MUX_Output(NewPC_wire)



);



Multiplexer2to1

#(

	.NBits(32)

)

MUX_Lui

(

	.Selector(Lui_wire),

	.MUX_Data0(ALUResult_wire),

	.MUX_Data1(LuiOutput_wire),

	

	.MUX_Output(MuxLuiOutput_wire)



);



Multiplexer2to1

#(

	.NBits(32)

)

MUX_RAM

(

	.Selector(MemToReg_wire),

	.MUX_Data0(MuxLuiOutput_wire),

	.MUX_Data1(RAM_ReadData_wire),

	

	.MUX_Output(RAMLUIALU_wire)


);





Multiplexer2to1

#(

	.NBits(32)

)

MUX_ForReadDataAndInmediate

(

	.Selector(ALUSrc_wire),

	.MUX_Data0(ReadData2_wire),

	.MUX_Data1(InmmediateExtend_wire),

	

	.MUX_Output(ReadData2OrInmmediate_wire)



);


//////////////// REGISTR FILE ////////////////////////////////////
RegisterFile

Register_File

(

	.clk(clk),

	.reset(reset),

	.RegWrite(RegWrite_wire),

	.WriteRegister(MuxJalOutput_wire),

	.ReadRegister1(Instruction_wire[25:21]),

	.ReadRegister2(Instruction_wire[20:16]),

	.WriteData(DATA_PC4Output_wire),

	.ReadData1(ReadData1_wire),

	.ReadData2(ReadData2_wire)



);

////////////// SIGN EXTEND /////////////////////////////////

SignExtend

SignExtendForConstants

(   

	.DataInput(Instruction_wire[15:0]),

   .SignExtendOutput(InmmediateExtend_wire)

);


///////////// LUI /////////////////////////////////
LuiModule

Lui

(   

	.DataInput(Instruction_wire[15:0]),

   .LuiOutput(LuiOutput_wire)

);


/////////////// ALU ALUCONTROL ////////////////////////////////////

ALUControl

ArithmeticLogicUnitControl

(

	.ALUOp(ALUOp_wire),

	.ALUFunction(Instruction_wire[5:0]),

	.ALUOperation(ALUOperation_wire)



);


ALU

ArithmeticLogicUnit 

(

	.ALUOperation(ALUOperation_wire),

	.A(ReadData1_wire),

	.B(ReadData2OrInmmediate_wire),

	.Zero(Zero_wire),

	.ALUResult(ALUResult_wire),

	.shamt(Instruction_wire[10:6])

);

//////////////////////// RAM ////////////////////////////

DataMemory 

#(	.DATA_WIDTH(32),

	.MEMORY_DEPTH(256)

)

RAM

(

	.WriteData(ReadData2_wire),

	.Address(RAM_Addr_wire),

	.MemWrite(MemWrite_wire),

	.MemRead(MemRead_wire),

	.clk(clk),

	.ReadData(RAM_ReadData_wire)

);

///////////////////////////////////////////////////////////////////////////

assign ALUResultOut = ALUResult_wire;



assign RAM_Addr_wire = {{24{1'b0}}, ALUResult_wire[9:2]};



assign Branch = ((BranchNE_wire & ~(Zero_wire))| (BranchEQ_wire & Zero_wire));



assign JumpAddress_wire = {{PC_4_wire[31:28],Instruction_wire[25:0], 2'b0}};


assign RegisterFile31_wire = 5'b11111;



endmodule
