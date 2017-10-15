module LuiModule
(
	input[15:0] DataInput,
	output [31:0] LuiOutput

);

assign LuiOutput = {DataInput,16'b0};

endmodule 