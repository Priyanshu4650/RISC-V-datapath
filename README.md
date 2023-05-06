# RISC-V-datapath
This Verilog module implements a simple pipeline for a RISC-V instruction set architecture. The pipeline consists of four stages:

* Instruction fetch
* Instruction decode
* Execute
* Memory access
* The pipeline is implemented using a combination of registers and wires to store data between stages, and sub-modules to perform specific tasks such as decoding instructions or accessing the register file.

# Inputs and Outputs
The Verilog module final_pipeline has the following inputs and outputs:

# Inputs
* instruction - 256-bit input holding the current instruction to be executed
* clk - clock signal
* rst - reset signal

# Outputs
* read_val1 - 32-bit output holding the value of the first source register read from the register file
* read_val2 - 32-bit output holding the value of the second source register read from the register file
* imm - 32-bit output holding the immediate value of the instruction being executed
* alu_result - 32-bit output holding the result of the arithmetic logic unit (ALU) operation
* current_instruction - 32-bit output holding the current instruction being executed
* risl_type_register - 4-bit output holding the type of instruction being executed
* ld_store - 2-bit output holding the load/store type of the instruction being executed

# Implementation Details
The pipeline is implemented using a combination of registers and wires to store data between stages. The input instruction is loaded into an instruction cache, and each instruction is fetched from the cache in the first stage of the pipeline. The instruction is then decoded in the second stage, and the register file is accessed to read the source operands. The ALU performs the operation in the third stage, and the result is stored in a register. In the fourth stage, memory is accessed for load/store instructions, and the result is written back to the register file.

*Sub-modules are used to perform specific tasks such as decoding instructions or accessing the register file. The decoder_with_clk sub-module decodes the instruction in the first pipeline stage, while the reg_file sub-module accesses the register file based on the register numbers in the second pipeline stage. The alu_with_clk sub-module performs arithmetic operations on the operands in the third pipeline stage.

*The pipeline also implements an instruction cache and a data cache. The instruction cache is used to store instructions, while the data cache is used to store data accessed by load/store instructions. The memory addresses for the data cache are calculated based on the immediate value of the instruction being executed.

# Limitations
* This implementation is a simple pipeline and does not include features such as forwarding or branch prediction. It assumes that instructions are executed in order and does not handle hazards. Additionally, this implementation does not include all instructions in the RISC-V instruction set architecture.


# Decoder with Clock
This Verilog module decodes instructions for a RISC-V instruction set architecture. The decoder_with_clk module takes in a 32-bit instruction and a clock signal, and outputs the opcode, source registers, destination register, immediate value, and instruction type.

# Inputs and Outputs
The Verilog module decoder_with_clk has the following inputs and outputs:

# Inputs
* instruction - 32-bit input holding the current instruction to be decoded
* clk - clock signal
# Outputs
* opcode - 17-bit output holding the opcode of the instruction being decoded
* rs1 - 5-bit output holding the number of the first source register
* rs2 - 5-bit output holding the number of the second source register
* rd - 5-bit output holding the number of the destination register
* immediate - 12-bit output holding the immediate value of the instruction being decoded
* risl_type - 4-bit output holding the type of instruction being decoded
* The risl_type output is an indicator of whether the instruction is an r-type, i-type, s-type, or l-type instruction. If the instruction is invalid, risl_type is set to 00.

# Implementation Details
* The decoder_with_clk module decodes the instruction based on the opcode and format of the instruction. The input instruction is decoded in the always block, triggered by the positive edge of the clk signal.

* The module uses a case statement to decode the instruction based on the opcode. If the opcode matches a known R-type instruction, the module sets the risl_type output to R_type and extracts the source and destination registers from the instruction. If the opcode matches a known I-type instruction, the module sets the risl_type output to I_type and extracts the immediate value and source register from the instruction. The module also checks for load instructions and sets the risl_type output to L_type accordingly.

* If the opcode matches a known S-type instruction, the module sets the risl_type output to S_type and extracts the immediate value and source registers from the instruction. Otherwise, if the opcode is not recognized, the module sets the risl_type output to 00 to indicate an invalid instruction.

# Limitations
* This implementation assumes that all instructions are 32 bits in length and does not handle unrecognized opcodes or invalid instruction formats. It also does not perform any validation on the register numbers or immediate values extracted from the instruction. Additionally, this implementation does not include all instructions in the RISC-V instruction set architecture.

# Register File
This Verilog module implements a register file for a RISC-V instruction set architecture. The reg_file module takes in two source register numbers, one destination register number, a clock signal, a reset signal, an enable signal, an enable write signal, and a write value. It outputs two read values from the register file.

# Inputs and Outputs
The Verilog module reg_file has the following inputs and outputs:

# Inputs
* rs1 - 5-bit input holding the number of the first source register
* rs2 - 5-bit input holding the number of the second source register
* rd - 5-bit input holding the number of the destination register
* enable_write - input control signal for enabling write operation
* write_value - 32-bit input holding the value to be written to the register file
* clk - clock signal
* rst - reset signal
enable - input control signal for enabling read operation
# Outputs
* read_value_1 - 32-bit output holding the value of the first source register read from the register file
* read_value_2 - 32-bit output holding the value of the second source register read from the register file
# Implementation Details
* The reg_file module uses a 32x32-bit register file to store register values. The input enable_write is used to enable writing to the register file. The module checks if enable_write is 1, and if so, it writes write_value to the register file at the address specified by rd. If enable_write is 0, the module reads the values from the register file at the addresses specified by rs1 and rs2.

* The module also includes a reset signal rst to reset the register file to 0 at the start of the simulation. The enable signal is used to enable the read operation. If enable is 0, the module sets the read_value_1 and read_value_2 outputs to 0.

* The module uses an always block triggered by the positive edge of the clk signal to perform register file operations. The module includes a case statement to handle the different enable write and enable read conditions.

# Limitations
This implementation assumes that all register values are 32 bits in length and does not include any additional features such as forwarding or hazard detection. Additionally, this implementation does not include all instructions in the RISC-V instruction set architecture.

# Booth Multiplier
This Verilog module implements a Booth multiplier for multiplying two 32-bit signed numbers. The booth_multiplier module takes in two 32-bit signed numbers A and B, a clock signal clk, and outputs the 64-bit product answer.

# Inputs and Outputs
The Verilog module booth_multiplier has the following inputs and outputs:

# Inputs
* A - 32-bit input holding the first signed number to be multiplied
* B - 32-bit input holding the second signed number to be multiplied
* clk - clock signal
# Outputs
* answer - 64-bit output holding the product of A and B
# Implementation Details
* The booth_multiplier module implements the Booth's algorithm to multiply two signed numbers. The algorithm includes three main steps:

* Convert the multiplicand A into its 64-bit two's complement representation A_complete, which can be done by padding with 32 zeros if A is positive or padding with 32 ones if A is negative.
* Convert the negation of the multiplicand -A into its 64-bit two's complement representation A_comp_complete, which can be done by padding with 32 ones if A is positive or padding with 32 zeros if A is negative.
* Perform a series of shifts and additions to obtain the product answer.
* The module uses an always block triggered by the positive edge of the clk signal to perform the Booth multiplication algorithm. The module includes a case statement to handle the different enable write and enable read conditions.

* The module uses a flag flag to handle the case when the operation is complete and to reset the accumulator, answer, A_complete, and A_comp_complete variables to 0. The flag is set to 0 at the start of the multiplication operation and set to 1 at the end of the operation.

# Limitations
This implementation assumes that all input and output values are 32 and 64 bits in length, respectively, and does not include any additional features such as overflow detection or signed/unsigned multiplication handling. Additionally, this implementation does not include any additional instructions or functionality beyond the Booth multiplication algorithm.

# Carry Look-Ahead Adder
This Verilog module implements a 4-bit Carry Look-Ahead Adder (CLA) for adding two 4-bit numbers x and y with a carry-in cin. The claAdder module takes in two 4-bit numbers x and y, a carry-in cin, and outputs the 4-bit sum sum, a carry-out cout, and an overflow flag overflow.

# Inputs and Outputs
The Verilog module claAdder has the following inputs and outputs:

# Inputs
* x - 4-bit input holding the first number to be added
* y - 4-bit input holding the second number to be added
* cin - input holding the carry-in bit
# Outputs
* sum - 4-bit output holding the sum of x and y
* cout - output holding the carry-out bit
* overflow - output flag indicating if an overflow occurred during addition
# Implementation Details
* The claAdder module implements a Carry Look-Ahead Adder (CLA) using a series of logic gates and cascading carry chains to compute the sum and carry-out values. The module uses a combination of AND, XOR, and OR gates to compute the carry and sum values for each bit position.

* The module includes a series of assign statements to compute the carry and sum values for each bit position using the x, y, and cin inputs. The carry values are computed using a combination of AND and OR gates, which are used to generate the g and p values for each bit position. The sum values are computed using XOR gates and the cin input.

* The overflow output is computed by XORing the carry-out cout with the carry-out from the 3rd bit position (c3), indicating if an overflow occurred during addition.

# Limitations
This implementation assumes that all input and output values are 4 bits in length and does not include any additional features such as signed/unsigned addition handling or overflow detection beyond the 4th bit. Additionally, this implementation does not include any additional instructions or functionality beyond the 4-bit CLA algorithm.

# Verilog Control Unit
This Verilog module implements a control unit for a processor that can handle three operations: addition, subtraction, and multiplication. The control_unit module takes in a 17-bit opcode opcode and outputs a 3-bit signal enable that indicates which operation to perform.

# Inputs and Outputs
The Verilog module control_unit has the following inputs and outputs:

# Inputs
opcode - 17-bit input holding the opcode for the processor operation
# Outputs
enable - 3-bit output holding the operation to perform (addition, subtraction, or multiplication)
# Implementation Details
* The control_unit module uses a case statement to decode the opcode input and generate the appropriate enable output signal for the operation to perform. The module includes a series of parameter statements to define the different operation codes (Eadd, Esub, and Emul) and a default value (Edefault) for when an invalid opcode is received.

* The case statement checks the value of the opcode input against each of the defined operation codes using bitwise comparisons (==). When a match is found, the corresponding enable signal is assigned to the output. If no match is found, the default value is assigned to the output.

# Limitations
This implementation assumes that all input and output values are 17 and 3 bits in length, respectively, and does not include any additional features such as error handling or additional instructions beyond the three defined operations. Additionally, this implementation does not include any additional logic for handling multiple instructions or program flow control.

# Verilog ALU with Clock
This Verilog module implements an Arithmetic Logic Unit (ALU) with a clock signal for performing arithmetic and logical operations on two 32-bit inputs A and B. The alu_with_clk module takes in two 32-bit inputs A and B, a 17-bit opcode opcode, and a clock signal clk, and outputs two 32-bit signals LH and UH representing the lower and upper halves of the ALU result, as well as an overflow flag overflow.

# Inputs and Outputs
The Verilog module alu_with_clk has the following inputs and outputs:

# Inputs
* A - 32-bit input holding the first operand
* B - 32-bit input holding the second operand
* opcode - 17-bit input holding the opcode for the operation
* clk - clock signal
# Outputs
* UH - 32-bit output holding the upper half of the ALU result
* LH - 32-bit output holding the lower half of the ALU result
* overflow - output flag indicating if an overflow occurred during the operation
# Implementation Details
* The alu_with_clk module implements an ALU that can perform addition, subtraction, and multiplication operations on two 32-bit inputs A and B. The module uses a control_unit module to decode the opcode input and generate a 3-bit enable signal that indicates which operation to perform.

* The module includes a series of wire and reg statements to declare internal signals and registers, as well as parameter statements to define the different operation codes (Eadd, Esub, and Emul) and a default value (Edefault) for when an invalid opcode is received.

* The module includes instances of cla32_adder, cla32_subtracter, and booth_multiplier modules to perform the addition, subtraction, and multiplication operations, respectively. The results of these operations are stored in LH_from_add, LH_from_sub, and mul_ans, respectively.

* The case statement inside the always block checks the value of the enable signal against each of the defined operation codes using bitwise comparisons (==). When a match is found, the corresponding LH, UH, and overflow signals are assigned to the output. If no match is found, the outputs are set to zero.

* The always block is triggered on every clock edge and performs the operation specified by the enable signal. The result of the operation is stored in the LH, UH, and overflow signals, which are updated on each clock cycle.

# Limitations
This implementation assumes that all input and output values are 32 bits in length and does not include any additional features such as error handling or additional instructions beyond the three defined operations. Additionally, this implementation does not include any additional logic for handling multiple instructions or program flow control beyond the clock signal.
