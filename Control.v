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
	input [5:0]func,
	
	output RegDst,
	output BranchEQ,
	output BranchNE,
	output Jump,
	output jr,
	output jal,
	output MemRead,
	output MemtoReg,
	output MemWrite,
	output ALUSrc,
	output RegWrite,
	output [2:0]ALUOp
);
localparam R_Type = 0;
localparam I_Type_ADDI	= 6'h08;
localparam I_Type_ORI 	= 6'h0d;

localparam I_Type_ANDI	= 6'h0c;
localparam I_Type_LUI	= 6'h0f;
localparam I_Type_LW		= 6'h23;
localparam I_Type_SW 	= 6'h2b;
localparam I_Type_BEQ 	= 6'h04;
localparam I_Type_BNE 	= 6'h05;
localparam J_Type_J		= 6'h02;
localparam J_Type_JAL	= 6'h03;

reg [13:0] ControlValues;

always@(OP or func) begin
	case(OP)
		R_Type: 
				case(func) 
				6'h8:
					ControlValues = 14'b0_000_00_00100_000;
				default
					ControlValues = 14'b1_001_00_00000_111;
				endcase
		I_Type_ADDI:		ControlValues = 14'b0_101_00_00000_100;
		I_Type_ORI:			ControlValues = 14'b0_101_00_00000_101;
		I_Type_ANDI:		ControlValues = 14'b0_101_00_00000_110;
		I_Type_LUI:			ControlValues = 14'b0_101_00_00000_001;
		I_Type_LW:			ControlValues = 14'b0_111_10_00000_100;
		I_Type_SW:			ControlValues = 14'b0_100_01_00000_100;
		I_Type_BEQ:			ControlValues = 14'b0_000_00_10000_011;
		I_Type_BNE:			ControlValues = 14'b0_000_00_01000_011;
		J_Type_J:			ControlValues = 14'b0_000_00_00010_000;
		J_Type_JAL:			ControlValues = 14'b0_001_00_00011_000;
		
		default:
			ControlValues= 14'b00000000000000;
		endcase
end	
	
assign RegDst   = ControlValues[13];
assign ALUSrc   = ControlValues[12];
assign MemtoReg = ControlValues[11];
assign RegWrite = ControlValues[10];
assign MemRead  = ControlValues[9];
assign MemWrite = ControlValues[8];
assign BranchEQ = ControlValues[7];
assign BranchNE = ControlValues[6];
assign jr		 = ControlValues[5];
assign Jump		 = ControlValues[4];
assign jal		 = ControlValues[3];
assign ALUOp    = ControlValues[2:0];

endmodule


