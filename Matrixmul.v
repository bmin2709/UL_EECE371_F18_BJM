module matmul(a,b,out);
	input [23:0] a, b;
	reg [47:0] acc;

	wire [15:0] cout, c;
	wire [7:0] a_temp [2:0];
	wire [7:0] b_temp [2:0];
	wire [15:0] acc_temp [2:0];
	
	output [15:0] out;

	assign {a_temp [2], a_temp [1], a_temp [0]} = a;
	assign {b_temp [2], b_temp [1], b_temp [0]} = b;
	assign {acc_temp [2], acc_temp [1], acc_temp [0]} = acc;

	multacc u0(a_temp[0], b_temp[0], acc_temp[0], mult, acc);
	multacc u1(a_temp[1], b_temp[1], acc_temp[1], mult, acc_temp[0]);
	multacc u2(a_temp[2], b_temp[2], acc_temp[2], mult, acc_temp[1]);
	
	assign out = acc_temp[2];		
endmodule