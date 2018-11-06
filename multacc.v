module multacc(a, b, c, mult, acc);
	input[7:0] a; 
	input[7:0] b; 
	input[15:0] acc;
	
	output[15:0] mult; 
	output[15:0] c; 

	assign mult = a*b; 
	assign c = acc + mult;

endmodule