/******************************************************************
* Description
*	This is control unit for the MIPS processor. The control unit is 
*	in charge of generation of the control signals. Its only input 
*	corresponds to opcode from the instruction.
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	01/03/2014
******************************************************************/
module Control
(
	input [5:0]OP,
	
	
	output Jal,
	output Jump,
	output Lui,
	output RegDst,
	output BranchEQ,
	output BranchNE,
	output MemRead,
	output MemtoReg,
	output MemWrite,
	output ALUSrc,
	output RegWrite,
	output [2:0]ALUOp
);
localparam R_Type = 0;

localparam I_Type_ADDI = 6'h8;
localparam I_Type_ORI = 6'h0d;
localparam I_Type_LUI = 6'h0f;
localparam I_Type_BEQ = 6'h04;
localparam I_Type_BNE = 6'h05;
localparam I_Type_LW = 6'h23;
localparam I_Type_SW = 6'h2b;
localparam J_Type_Jump = 6'h02;
localparam J_Type_Jal = 6'h03;



reg [13:0] ControlValues;

always@(OP) begin
	casex(OP)
		R_Type:       	ControlValues = 14'b0_0_0_1_001_00_00_111;
		I_Type_ADDI: 	ControlValues = 14'b0_0_0_0_101_00_00_100;
		I_Type_ORI: 	ControlValues = 14'b0_0_0_0_101_00_00_101;
		I_Type_LUI: 	ControlValues = 14'b0_0_1_0_001_00_00_000;
		I_Type_BEQ: 	ControlValues = 14'b0_0_0_0_000_00_01_010;
		I_Type_BNE: 	ControlValues = 14'b0_0_0_0_000_00_10_010;
		I_Type_LW: 		ControlValues = 14'b0_0_0_0_111_10_00_100;//100 igual que addi para que sume
		I_Type_SW: 		ControlValues = 14'b0_0_0_0_100_01_00_100;
		
		J_Type_Jump: 	ControlValues = 14'b0_1_0_0_000_00_00_000;
		J_Type_Jal: 	ControlValues = 14'b1_1_0_0_001_00_00_000;
		
		default:
			ControlValues= 10'b0000000000;
		endcase
end	

assign Jal = ControlValues[13];	

assign Jump = ControlValues[12];	

assign Lui = ControlValues[11];	

assign RegDst = ControlValues[10];

assign ALUSrc = ControlValues[9];
assign MemtoReg = ControlValues[8];
assign RegWrite = ControlValues[7];

assign MemRead = ControlValues[6];
assign MemWrite = ControlValues[5];

assign BranchNE = ControlValues[4];
assign BranchEQ = ControlValues[3];

assign ALUOp = ControlValues[2:0];	

endmodule


