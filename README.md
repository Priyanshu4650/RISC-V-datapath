# RISC-V-datapath
This Verilog module implements a simple pipeline for a RISC-V instruction set architecture. The pipeline consists of four stages:

*Instruction fetch
*Instruction decode
*Execute
*Memory access
*The pipeline is implemented using a combination of registers and wires to store data between stages, and sub-modules to perform specific tasks such as decoding instructions or accessing the register file.

##Inputs and Outputs
The Verilog module final_pipeline has the following inputs and outputs:

#Inputs
*instruction - 256-bit input holding the current instruction to be executed
*clk - clock signal
*rst - reset signal

#Outputs
*read_val1 - 32-bit output holding the value of the first source register read from the register file
*read_val2 - 32-bit output holding the value of the second source register read from the register file
*imm - 32-bit output holding the immediate value of the instruction being executed
*alu_result - 32-bit output holding the result of the arithmetic logic unit (ALU) operation
*current_instruction - 32-bit output holding the current instruction being executed
*risl_type_register - 4-bit output holding the type of instruction being executed
*ld_store - 2-bit output holding the load/store type of the instruction being executed

#Implementation Details
*The pipeline is implemented using a combination of registers and wires to store data between stages. The input instruction is loaded into an instruction cache, and each instruction is fetched from the cache in the first stage of the pipeline. The instruction is then decoded in the second stage, and the register file is accessed to read the source operands. The ALU performs the operation in the third stage, and the result is stored in a register. In the fourth stage, memory is accessed for load/store instructions, and the result is written back to the register file.

*Sub-modules are used to perform specific tasks such as decoding instructions or accessing the register file. The decoder_with_clk sub-module decodes the instruction in the first pipeline stage, while the reg_file sub-module accesses the register file based on the register numbers in the second pipeline stage. The alu_with_clk sub-module performs arithmetic operations on the operands in the third pipeline stage.

*The pipeline also implements an instruction cache and a data cache. The instruction cache is used to store instructions, while the data cache is used to store data accessed by load/store instructions. The memory addresses for the data cache are calculated based on the immediate value of the instruction being executed.

#Limitations
*This implementation is a simple pipeline and does not include features such as forwarding or branch prediction. It assumes that instructions are executed in order and does not handle hazards. Additionally, this implementation does not include all instructions in the RISC-V instruction set architecture.
