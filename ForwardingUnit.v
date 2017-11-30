module ForwardingUnit

(
	input  [4:0]IDEX_RegisterRs,
	input  [4:0]IDEX_RegisterRt,
	
	input  EXMEM_RegWrite,
	input  [4:0]EXMEM_RegisterRd,
	
	input  MEMWB_RegWrite,
	input  [4:0]MEMWB_RegisterRd,
	
	output reg [1:0]ForwardA,
	output reg [1:0]ForwardB
);


always@(EXMEM_RegWrite or EXMEM_RegisterRd or IDEX_RegisterRs or MEMWB_RegWrite or MEMWB_RegisterRd)
begin

	if 
		((EXMEM_RegWrite) && 
		(EXMEM_RegisterRd !=0) && 
		(EXMEM_RegisterRd == IDEX_RegisterRs))
			ForwardA = 2'b10;
		
	else if 
		((MEMWB_RegWrite) &&
		(MEMWB_RegisterRd != 0) &&
		(EXMEM_RegisterRd != IDEX_RegisterRs) &&
		(MEMWB_RegisterRd == IDEX_RegisterRs))
			ForwardA = 2'b01;	
	else 
		ForwardA = 2'b00;

end

always@(EXMEM_RegWrite or EXMEM_RegisterRd or IDEX_RegisterRt or MEMWB_RegWrite or MEMWB_RegisterRd)
begin

	if 
		((EXMEM_RegWrite) && 
		(EXMEM_RegisterRd !=0) && 
		(EXMEM_RegisterRd == IDEX_RegisterRt))
			ForwardB = 2'b10;
		
	else if 
		((MEMWB_RegWrite) &&
		(MEMWB_RegisterRd != 0) &&
		(EXMEM_RegisterRd != IDEX_RegisterRt) &&
		(MEMWB_RegisterRd == IDEX_RegisterRt))
			ForwardB = 2'b01;	
	else 
		ForwardB = 2'b00;

end


endmodule