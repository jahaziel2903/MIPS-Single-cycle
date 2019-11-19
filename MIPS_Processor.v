/******************************************************************
* Description
*	This is the top-level of a MIPS processor that can execute the next set of instructions:
*		add
*		addi
*		sub
*		ori
*		or
*		and

*		nor
* This processor is written Verilog-HDL. Also, it is synthesizable into hardware.
* Parameter MEMORY_DEPTH configures the program memory to allocate the program to
* be execute. If the size of the program changes, thus, MEMORY_DEPTH must change.
* This processor was made for computer architecture class at ITESO.
* Version:
*	1.5
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	2/09/2018
******************************************************************/


module MIPS_Processor
#(
	parameter MEMORY_DEPTH = 64,
	parameter PC_INCREMENT = 4,
	parameter jump_start = 32'b1111_1111_1100_0000_0000_0000_0000_0000,
	parameter ra = 31
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
//******************************************************************/
assign  PortOut = 0;

//******************************************************************/
//******************************************************************/
// signals to connect modules
wire BranchEQ_wire;
wire BranchNE_wire;
wire branch_wire;
wire jump_wire;
wire sel_jr; //nuevo wire
wire sel_jal; //nuevo wire
wire PCSrc_wire;
wire RegDst_wire;
wire not_zero_and_brach_ne;
wire zero_and_brach_eq;
wire or_for_branch;
wire ALUSRC_wire;
wire reg_write_wire;
wire zero_wire;
wire wMemWrite;
wire wMemRead;
wire wMemtoReg;


// bUSES DE WIRES
wire [2:0]  aluop_wire;
wire [3:0]  alu_operation_wire;
wire [4:0]  register_wire;
wire [4:0]  write_register_wire;
wire [31:0] instruction_bus_wire;
wire [31:0] read_data_1_wire;
wire [31:0] read_data_2_wire;
wire [31:0] Inmmediate_extend_wire;
wire [31:0] read_data_2_orr_inmmediate_wire;
wire [31:0] alu_result_wire;
wire [31:0] pc_plus_4_wire;
wire [31:0] ADDER_ALURESULT_wire;
wire [31:0] inmmediate_extended_wire;
wire [31:0] Inmmediate_shifted_wire;
wire [31:0] pc_to_branchEQ_wire;
wire [31:0] PCBranchEQ_wire;
wire [31:0] PCBranchNE_wire;
wire [31:0] pc_or_branch_or_jump_wire;
wire [31:0] offset_Start;
wire [31:0] MUX_PC_wire;
wire [31:0] PC_write;
wire [31:0] pc_wire;
wire [31:0] writeData_wire;
wire [31:0] wReadData;
wire [31:0] wRamAluMux;
wire [27:0] jump_no_concatenado;


//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/

Control
ControlUnit
(
	.OP(instruction_bus_wire[31:26]),
	.func(instruction_bus_wire[5:0]), //se agrego la func para diferencial jr de las demas de tipo r 
	.RegDst(RegDst_wire),
	.BranchEQ(BranchEQ_wire),
	.BranchNE(BranchNE_wire),
	.Jump(jump_wire),
	.jal(sel_jal), //se agrego al nuevo wire
	.jr(sel_jr),	//se agrego al nuevo wire 
	.MemRead(wMemRead),
	.MemtoReg(wMemtoReg),
	.MemWrite(wMemWrite),
	.ALUOp(aluop_wire),
	.ALUSrc(ALUSRC_wire),
	.RegWrite(reg_write_wire)
);

PC_Register
ProgramCounter
(
	.clk(clk),
	.reset(reset),
	.NewPC(MUX_PC_wire),
	.PCValue(pc_wire)
);

ProgramMemory
#(
	.MEMORY_DEPTH(MEMORY_DEPTH)
)
ROMProgramMemory
(
	.Address(pc_wire),

	.Instruction(instruction_bus_wire)
);

Adder32bits
PC_Plus_4
(
	.Data0(pc_wire),
	.Data1(PC_INCREMENT),
	
	.Result(pc_plus_4_wire)
);


//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForRTypeAndIType
(
	.Selector(RegDst_wire),
	.MUX_Data0(instruction_bus_wire[20:16]),
	.MUX_Data1(instruction_bus_wire[15:11]),
	
	.MUX_Output(register_wire)

);



RegisterFile
Register_File
(
	.clk(clk),
	.reset(reset),
	.RegWrite(reg_write_wire),
	.WriteRegister(write_register_wire),
	.ReadRegister1(instruction_bus_wire[25:21]),
	.ReadRegister2(instruction_bus_wire[20:16]),
	.WriteData(writeData_wire), //viene del mux agregado de jal, para escribir direccion de $ra
	.ReadData1(read_data_1_wire),
	.ReadData2(read_data_2_wire)

);

SignExtend
SignExtendForConstants
(   
	.DataInput(instruction_bus_wire[15:0]),
   .SignExtendOutput(Inmmediate_extend_wire)
);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForReadDataAndInmediate
(
	.Selector(ALUSRC_wire),
	.MUX_Data0(read_data_2_wire),
	.MUX_Data1(Inmmediate_extend_wire),
	
	.MUX_Output(read_data_2_orr_inmmediate_wire)

);

ALUControl
ArithmeticLogicUnitControl
(
	.ALUOp(aluop_wire),
	.ALUFunction(instruction_bus_wire[5:0]),
	.ALUOperation(alu_operation_wire)

);

ALU
ArithmeticLogicUnit 
(
	.ALUOperation(alu_operation_wire),
	.A(read_data_1_wire),
	.B(read_data_2_orr_inmmediate_wire),
	.shamt(instruction_bus_wire[10:6]),
	.Zero(zero_wire),
	.ALUResult(alu_result_wire)
);

assign ALUResultOut = alu_result_wire;
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/

DataMemory
#(
	.DATA_WIDTH(32),
	.MEMORY_DEPTH(256)
)
RamMemory
(
	.WriteData(read_data_2_wire),
	.Address(alu_result_wire),
	.MemWrite(wMemWrite),
	.MemRead(wMemRead),
	.clk(clk),
	.ReadData(wReadData)
);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForAluAndRamMemory
(
	.Selector(wMemtoReg),
	.MUX_Data0(alu_result_wire),
	.MUX_Data1(wReadData),
	
	.MUX_Output(wRamAluMux)
);

Multiplexer2to1
#(
	 .NBits(32)
)

MUX_JUMP
(
	.Selector(jump_wire),
	.MUX_Data0((PCBranchNE_wire)),
	.MUX_Data1((offset_Start)),
	.MUX_Output(pc_or_branch_or_jump_wire)
);


ShiftLeft2
ShiftLeft
(
	.DataInput(instruction_bus_wire[25:0]),
	.DataOutput(jump_no_concatenado)
	
);

Adder32bits
ADD_ALU_offset //adder para iniciar el PC en 40000
(
	.Data0({pc_plus_4_wire[31:28], jump_no_concatenado}),
	.Data1(jump_start),
	
	.Result(offset_Start)
);

//se agrego este shift left para dejar alineado a 4 bytes las direcciones de salto de los branches
ShiftLeft2
ShiftLeftBranch
(
	.DataInput(Inmmediate_extend_wire),
	.DataOutput(Inmmediate_shifted_wire)
	
);

Adder32bits
ADD_ALU_Result
(
	.Data0(pc_plus_4_wire),
	.Data1(Inmmediate_shifted_wire),
	.Result((ADDER_ALURESULT_wire))
);

//se agrego este branch para decidir si saltar o no a la direccion puesta dependiendo si se cumple o no la condicion  
Multiplexer2to1
#(
	.NBits(32)
)
MUX_Branch
(
	.Selector(BranchEQ_wire & zero_wire | BranchNE_wire & ~zero_wire), //condicion si se cumple el beq o el bne
	.MUX_Data0(pc_plus_4_wire),
	.MUX_Data1(ADDER_ALURESULT_wire),
	.MUX_Output(PCBranchNE_wire)
);


Multiplexer2to1
#(
	 .NBits(32)
)

//se agrega este mux despues del mux del jump, este mux salta a la direccion del registro deseado siempre y cuando se
// use la instruccion jr
MUX_Jr
(
	.Selector(sel_jr), //nuevo selector declarado en el control
	.MUX_Data0(pc_or_branch_or_jump_wire), //entra al mux la direccion del jump o de algun branch
	.MUX_Data1(read_data_1_wire), //direccion del registro
	.MUX_Output(MUX_PC_wire) //conectada a la entrada del PC
);


//MULTIPLEXOR que activa el registro de ra para escribir en el al ahcer una intruccion jal
Multiplexer2to1
#(
	 .NBits(32)
)

MUX_Jal_31
(
	.Selector(sel_jal), //nuevo wire del control
	.MUX_Data0(register_wire),
	.MUX_Data1(ra), //parametro de 31
	.MUX_Output(write_register_wire)
);

//neuvo mux para activar si se escrbira la direccion de salto a ra o simplemente
// dejara pasar de la ram al registro 

Multiplexer2to1
#(
	.NBits(32)
)
MUX_Jal_WriteRa
(
	.Selector(sel_jal),
	.MUX_Data0(wRamAluMux),
	.MUX_Data1(pc_plus_4_wire),
	
	.MUX_Output(writeData_wire)
);

endmodule

