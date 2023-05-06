## RISC-V-datapath
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


# Verilog Decoder with Clock
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

# Verilog Register File
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
