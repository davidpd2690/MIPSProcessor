/******************************************************************
* Description
*	This is the data memory for the MIPS processor
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	01/03/2014
******************************************************************/

module DataMemory 
#(	parameter DATA_WIDTH=8,
	parameter MEMORY_DEPTH = 1024

)
(
	input [DATA_WIDTH-1:0] WriteData,
	input [DATA_WIDTH-1:0]  Address,
	input MemWrite,MemRead, clk,
	output  [DATA_WIDTH-1:0]  ReadData
);
	
	// Declare the RAM variable
	reg [DATA_WIDTH-1:0] ram[MEMORY_DEPTH-1:0];
	wire [DATA_WIDTH-1:0] ReadDataAux;
	
	/*initial
	begin
		$readmemh("C:/MIPSProjects/MIPSProcessor_ver1/Sources/text.dat", rom);
	end

	always @ (RealAddress)
	begin
		Instruction = rom[RealAddress];
	end*/

	always @ (posedge clk)
	begin
		// Write
		if (MemWrite)
			ram[Address] <= WriteData;
	end
	assign ReadDataAux = ram[Address];
  	assign ReadData = {DATA_WIDTH{MemRead}}& ReadDataAux;

endmodule
