/******************************************************************
* Description
*	This is an 32-bit arithetic logic unit that can execute the next set of operations:
*		add
*		sub
*		or
*		and
*		nor
* This ALU is written by using behavioral description.
* Version:
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	01/03/2014
******************************************************************/

module ALU 
(
	input [3:0] ALUOperation,
	input [31:0] A,
	input [31:0] B,
	input	[4:0] shamt,
	output reg Zero,
	output reg [31:0]ALUResult
);

localparam AND = 	4'b0000;
localparam OR  = 	4'b0001;
localparam NOR = 	4'b0010;
localparam ADD = 	4'b0011;
localparam SUB = 	4'b0100;
localparam SLL = 	4'b1110;
localparam SRL = 	4'b1100;
localparam JR = 	4'b1101;
   
   always @ (A or B or ALUOperation or shamt)
     begin
		case (ALUOperation)
		
		AND:
		ALUResult = A & B;
		OR:
		ALUResult = A | B;
		NOR:
		ALUResult = ~(A|B);
		SLL:
		ALUResult = B<<shamt;
		SRL:
		ALUResult = B>>shamt;
			
		ADD: // add
		ALUResult=A + B;
		SUB: // sub
		ALUResult=A - B;
		
		JR:
		ALUResult={{27{1'b0}},{A[25:21]}};

		default:
			ALUResult= 0;
		endcase // case(control)
		Zero = (ALUResult==0) ? 1'b1 : 1'b0;
     end // always @ (A or B or control)
endmodule // ALU