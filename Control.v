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
	
	output JR,
	output Jal,
	output Jump,
	output Lui,
	output RegDst,			//EX
	output BranchEQ,		//M
	output BranchNE,		//M
	output MemRead,		//M
	output MemtoReg,		//WB
	output MemWrite,		//M
	output ALUSrc,			//EX
	output RegWrite,		//WB
	output [2:0]ALUOp		//EX
);
localparam R_Type = 0;
//localparam R_Type_JR = 6'h08;

localparam I_Type_ADDI = 6'h8;
localparam I_Type_ORI =  6'h0d;
localparam I_Type_LUI =  6'h0f;
localparam I_Type_BEQ =  6'h04;
localparam I_Type_BNE =  6'h05;
localparam I_Type_LW =   6'h23;
localparam I_Type_SW =   6'h2b;

localparam J_Type_Jump = 6'h02;
localparam J_Type_Jal =  6'h03;



reg [14:0] ControlValues;

always@(OP) begin
	casex(OP)
		R_Type:       	ControlValues = 15'b0_0_0_0_1_001_00_00_111;
	//	R_Type_JR:     ControlValues = 15'b1_0_0_0_1_001_00_00_111;
		
		I_Type_ADDI: 	ControlValues = 15'b0_0_0_0_0_101_00_00_100;
		I_Type_ORI: 	ControlValues = 15'b0_0_0_0_0_101_00_00_101;
		I_Type_LUI: 	ControlValues = 15'b0_0_0_1_0_001_00_00_000;
		I_Type_BEQ: 	ControlValues = 15'b0_0_0_0_0_000_00_01_010;
		I_Type_BNE: 	ControlValues = 15'b0_0_0_0_0_000_00_10_010;
		I_Type_LW: 		ControlValues = 15'b0_0_0_0_0_111_10_00_100;//100 igual que addi para que sume
		I_Type_SW: 		ControlValues = 15'b0_0_0_0_0_100_01_00_100;
		
		J_Type_Jump: 	ControlValues = 15'b0_0_1_0_0_000_00_00_000;
		J_Type_Jal: 	ControlValues = 15'b0_1_1_0_0_001_00_00_000;
		
		default:
			ControlValues= 10'b0000000000;
		endcase
end	

assign JR = ControlValues[14];

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


