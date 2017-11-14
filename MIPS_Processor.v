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

assign  PortOut = 0;

//******************************************************************/


// Data types to connect modules

reg  JR_wire;
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

//////////// IFID ////////////////////////////

wire [31:0] IFID_PC_4_wire;
wire [31:0] IFID_Instruction_wire;

//////////// IDEX ////////////////////////////

wire IDEX_RegDst_wire;
wire IDEX_ALUOp_wire; 
wire IDEX_ALUSrc_wire; 
wire IDEX_BranchEQ_wire; 
wire IDEX_BranchNE_wire; 		
wire IDEX_RegWrite_wire;
wire IDEX_MemRead_wire, 
wire IDEX_MemWrite_wire, 
wire IDEX_MemToReg_wirewire;


wire [31:0] IDEX_PC_4_wire;
wire [31:0] IDEX_ReadData1_wire;
wire [31:0] IDEX_ReadData2_wire;
wire [31:0] IDEX_ImmExtend_wire;
wire [31:0] IDEX_Instruction_wire;

//////////// X-MEN /////////////////////////////

wire [31:0]EXMEM_BranchAdderOutput_wire; 
wire [31:0]EXMEM_ALUResult_wire;  
wire [31:0]EXMEM_ReadData2_wire;
wire [4:0] EXMEM_WriteRegister_wire; 

wire EXMEM_Zero_wire;
wire EXMEM_regWrite;
wire EXMEM_brancheq; 
wire EXMEM_branchne; 
wire EXMEM_memtoreg; 
wire EXMEM_memwrite; 
wire EXMEM_memread;

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
	 .NewPC(NewPCJROutput_wire),
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

/////////////////// PIPE REGISTERS ///////////////////////

Register_pipe
#(
	.N(64)
)
 IFID
(
	
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput({PC_4_wire, Instruction_wire}),

	.DataOutput({IFID_PC_4_wire, IFID_Instruction_wire})
	
);

Register_pipe
#(
	.N(153)
)
 IDEX
(
	
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput({IFID_PC_4_wire, IFID_Instruction_wire[20:16], IFID_Instruction_wire[15:11],
					InmmediateExtend_wire, ReadData1_wire, ReadData2_wire, RegDst_wire, 
					ALUOp_wire, ALUSrc_wire, BranchEQ_wire, BranchNE_wire, RegWrite_wire, 
					MemRead_wire, MemWrite_wire, MemToReg_wirewire}),

	.DataOutput({IDEX_PC_4_wire, IDEX_Instruction_wire, IDEX_ImmExtend_wire, IDEX_ReadData1_wire, 
					 IDEX_ReadData2_wire, IDEX_RegDst_wire, IDEX_ALUOp_wire, IDEX_ALUSrc_wire, 
					 IDEX_BranchEQ_wire, IDEX_BranchNE_wire, IDEX_RegWrite_wire, 
					 IDEX_MemRead_wire, IDEX_MemWrite_wire, IDEX_MemToReg_wire})
	
);

Register_pipe
#(
	.N(135)
)
 EXMEM
(
	
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput({BranchAdderOutput_wire, ALUResult_wire, Zero_wire, IDEX_ReadData2_wire,
					WriteRegister_wire, IDEX_RegWrite_wire, IDEX_BranchEQ_wire, IDEX_BranchNE_wire, 
					IDEX_MemToReg_wire, IDEX_MemWrite_wire, IDEX_MemRead_wire}),

	.DataOutput({EXMEM_BranchAdderOutput_wire, EXMEM_ALUResult_wire, EXMEM_Zero_wire, EXMEM_ReadData2_wire,
					EXMEM_WriteRegister_wire, EXMEM_RegWrite_wire, EXMEM_BranchNE_wire, EXMEM_BranchEQ_wire, 
					EXMEM_MemToReg_wire, EXMEM_MemWrite_wire, EXMEM_MemRead_wire})
	
);

Register_pipe
#(
	.N(71)
)
 MEMWB
(
	
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput({RAM_ReadData_wire, EXMEM_ALUResult_wire,
					EXMEM_WriteRegister_wire, EXMEM_RegWrite_wire, EXMEM_MemToReg_wire}),

	.DataOutput({EXMEM_BranchAdderOutput_wire, EXMEM_ALUResult_wire, EXMEM_Zero_wire, EXMEM_ReadData2_wire,
					EXMEM_WriteRegister_wire, EXMEM_RegWrite_wire, EXMEM_BranchNE_wire, EXMEM_BranchEQ_wire, 
					EXMEM_MemToReg_wire, EXMEM_MemWrite_wire, EXMEM_MemRead_wire})
	
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

	.Data0(IDEX_PC_4_wire),
	.Data1(BranchAddress_wire),

	.Result(BranchAdderOutput_wire)

);

//////////////// LEFT SHIFTER /////////////////////////////////

ShiftLeft2

BranchAddressShifter 
(   
	.DataInput(IDEX_ImmExtend_wire),
	
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
	.MUX_Data0(IDEX_Instruction_wire[20:16]),
	.MUX_Data1(IDEX_Instruction_wire[15:11]),
	
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
	.MUX_Data1(EXMEM_BranchAdderOutput_wire),

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

	.MUX_Data0(IDEX_ReadData2_wire),
	.MUX_Data1(IDEX_ImmExtend_wire),

	.MUX_Output(ReadData2OrInmmediate_wire)
);

//////////////// REGISTER FILE ////////////////////////////////////

RegisterFile

Register_File
(
	.clk(clk),

	.reset(reset),

	.RegWrite(RegWrite_wire),
	.WriteRegister(MuxJalOutput_wire),
	.ReadRegister1(IFID_Instruction_wire[25:21]),
	.ReadRegister2(IFID_Instruction_wire[20:16]),
	.WriteData(DATA_PC4Output_wire),

	.ReadData1(ReadData1_wire),
	.ReadData2(ReadData2_wire)
);



////////////// SIGN EXTEND /////////////////////////////////

SignExtend

SignExtendForConstants
(   
	.DataInput(IFID_Instruction_wire[15:0]),

   .SignExtendOutput(InmmediateExtend_wire)
);

///////////// LUI /////////////////////////////////

LuiModule

Lui
(   
	.DataInput(Instruction_wire[15:0]),

   .LuiOutput(LuiOutput_wire)
);

/////////////// ALU / ALUCONTROL ////////////////////////////////////

ALUControl

ArithmeticLogicUnitControl
(
	.ALUOp(ALUOp_wire),
	.ALUFunction(IDEX_Instruction_wire[5:0]),
	.ALUOperation(ALUOperation_wire)
);

ALU

ArithmeticLogicUnit 
(
	.ALUOperation(ALUOperation_wire),
	.A(IDEX_ReadData1_wire),
	.B(ReadData2OrInmmediate_wire),

	.Zero(Zero_wire),

	.ALUResult(ALUResult_wire),

	.shamt(Instruction_wire[10:6])
);

//////////////////////// RAM ////////////////////////////

DataMemory 
#(	
	.DATA_WIDTH(32),
	.MEMORY_DEPTH(256)
)

RAM
(
	.WriteData(EXMEM_ReadData2_wire),
	.Address(RAM_Addr_wire),
	.MemWrite(EXMEM_MemWrite_wire),
	.MemRead(EXMEM_MemRead_wire),

	.clk(clk),

	.ReadData(RAM_ReadData_wire)
);

///////////////////////////////////////////////////////////////////////////

assign ALUResultOut = ALUResult_wire;

assign RAM_Addr_wire = {{24{1'b0}}, EXMEM_ALUResult_wire[9:2]};

assign Branch = ((EXMEM_BranchNE_wire & ~(EXMEM_Zero_wire))| (EXMEM_BranchEQ_wire & EXMEM_Zero_wire));

assign JumpAddress_wire = {{PC_4_wire[31:28],Instruction_wire[25:0], 2'b0}};

assign RegisterFile31_wire = 5'b11111;


always @(Instruction_wire)
begin
    if((Instruction_wire[31:26] ==5'b0) && (Instruction_wire[5:0] == 6'b001000))
		JR_wire = 1;
    else
	   JR_wire = 0;
end

endmodule