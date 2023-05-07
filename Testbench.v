
    
    
    reg [255:0] instruction;
    reg clk, rst;
    reg [31:0] instruction_cache [7:0];
    // the above is the instruction cache which holds the instruction
    
    
    
    
    wire [31:0] rv1, rv2, imm, ans_from_value,current_instruction;
    wire [3:0] risl_type;
    
    pipeline DUT(instruction, clk, rst, rv1, rv2, imm, ans_from_value,current_instruction,risl_type);
    // the instruction passed to the pipeline module is a single string of 8*32 = 256 bits
    
    
    initial begin
      $dumpfile("dump.vcd"); $dumpvars();
    clk = 1'b0;
    rst = 1'b1;
    instruction_cache[0] = {12'd0,5'd29,3'b000,5'd29,7'b0010011};
    instruction_cache[1] = {12'd1,5'd30,3'b000,5'd30,7'b0010011};
    instruction_cache[2] = {12'd0,5'd29, 3'b010, 5'd28, 7'b0000011};
    instruction_cache[3] = {12'd0,5'd30, 3'b010, 5'd27, 7'b0000011};
    instruction_cache[4] = {7'b0, 5'd29, 5'd28, 3'b0, 5'd10, 7'b0110011};
    instruction_cache[5] = {7'b0,5'd29,5'd10, 3'b010, 5'd0, 7'b0100011};
    instruction_cache[6] = 32'h0;
    instruction_cache[7] = 32'h0;
    instruction = {instruction_cache[7],instruction_cache[6],instruction_cache[5],instruction_cache[4],instruction_cache[3],instruction_cache[2],instruction_cache[1],instruction_cache[0]};

    #20
    rst = 1'b0;
//    $display("instruction from testbench is ", instruction);
     instruction = {instruction[7],instruction[6],instruction[5],instruction[4],instruction[3],instruction[2],instruction[1],instruction[0]};
     
    
    
    
    
    
    #320 $finish;

    end
    
    always #5 clk = ~clk;





endmodule
