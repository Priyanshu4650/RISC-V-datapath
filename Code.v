module final_pipeline(
    input [255:0] instruction,
    input clk, rst,
    output reg [31:0] read_val1, read_val2, imm, alu_result,current_instruction,
    output reg [3:0] risl_type_register
    );
    reg [31:0] pip1;
    reg [31:0] pip2 [2:0];
    reg [31:0] pip3 [1:0];
    wire [16:0] opcode_wr;
    wire [4:0] rs1_wr, rs2_wr, rd_wire;
    reg [4:0] destination_register;
    wire [11:0] imm_wr;
    wire [3:0] risl_type_wire;    
   
    parameter [3:0]
    R_type = 4'b1000,
    I_type = 4'b0100,
    S_type = 4'b0010,
    L_type = 4'b0001;
    reg [31:0] d_cache [7:0];
    reg [4:0] rs1, rs2, rd;
    reg enable_write, enable;
    reg [31:0] write_value;
    wire [31:0] read1_wr, read2_wr;
    always @ (posedge clk)
    begin
    enable <= 1'b1;
    end
   
    reg [31:0] A, B;
    wire [31:0] UH_wire, LH_wire;
    wire overflow;
    reg [4:0] data_cache_address;
   
    decoder_with_clk DECODER(pip1,clk,  opcode_wr, rs1_wr, rs2_wr, rd_wire, imm_wr, risl_type_wire);
    reg_file REGISTER_FILE(rs1_wr, rs2_wr, destination_register , enable_write, write_value, clk, rst, enable, read1_wr, read2_wr);
    alu_with_clk ALU(A, B, opcode_wr, clk, UH_wire, LH_wire, overflow_wire);
   

    reg [1:0] ld_store;
   
   
    parameter [2:0]
    STAGE_1 = 3'd1,
    STAGE_2 = 3'd2,
    STAGE_3 = 3'd3,
    STAGE_4 = 3'd4;
   
    reg [2:0] stage;
   
    always @ (posedge clk) begin
        read_val1 <= read1_wr;
        read_val2 <= read2_wr;
        imm <= imm_wr;
        alu_result <= LH_wire;
        risl_type_register <= risl_type_wire;
       
       
   
    end
   
    always @ (posedge clk) begin
        case(risl_type_wire)
        S_type: ld_store = 2'b01;
        L_type: ld_store = 2'b10;
        default: ld_store = 2'b00;
           
       
        endcase
   
   
    end
   
    reg [31:0] i_cache [7:0];
    reg [3:0] address;
   
   
   
    reg [31:0] temp;
    integer i;
    always @ (posedge clk) begin                              // INITIALLY D_CACHE HAS VALUES 45 AND -20 STORED IN 0 AND 1 REGISTERS .
        if (rst) begin
            stage = 3'd1;
            for (i=0;i<8;i=i+1) d_cache[i] = 32'h0;
            ld_store = 2'b0;
            d_cache[0] = 45;
            d_cache[1] = -20;
            address = 4'b0;
            for (i=0;i<8;i = i + 1) i_cache[i] = 32'h0;
            i_cache[0] = instruction[31:0];
            i_cache[1] = instruction[63:32];
            i_cache[2] = instruction[95:64];
            i_cache[3] = instruction[127:96];
            i_cache[4] = instruction[159:128];
            i_cache[5] = instruction[191:160];
            i_cache[6] = instruction[223:192];
            i_cache[7] = instruction[255:224];

       
        end
        else begin
       
            case(stage)
                STAGE_1: begin                        // CYCLE REPEAT
                    if(address == 9) begin
                        stage = STAGE_1;
                        pip1 = 32'h0;
                    end
               
                else begin
               
//            
                    pip1 = i_cache[address];                       //INSTRUCTION FETCHED FROM I_CACHE AND STORED IN PIPELINE REGISTER 1
                    current_instruction = pip1;
                    address = address + 1;
                    enable_write = 1'b0;

                    data_cache_address = 5'h0;
   
                    stage = STAGE_2;
                end
               
               
                end
                STAGE_2: begin
               
                enable_write = 1'b0;


                stage = STAGE_3;
//              
               
                end
                STAGE_3: begin
                A = read1_wr;
//            
                case(risl_type_wire)
                    R_type:
                    begin
                    B = read2_wr;
                    destination_register = rd_wire;
                    end
                    I_type: begin
                   
                            destination_register = rd_wire;
                            if (~imm_wr[11]) B = {20'b0, imm_wr};
                            else B = {20'hfffff, imm_wr};
                           
                    end
                    S_type:begin
                   
                        destination_register = rd_wire;
                        temp = read2_wr;
                       
                       
                        if (~imm_wr[11]) B = {20'b0, imm_wr};
                            else B = {20'hfffff, imm_wr};
//                    
                       
                        data_cache_address = B + temp;
//                      
                   
                    end
                    L_type:
                    begin
                        temp = read1_wr;
                        if (~imm_wr[11]) B = {20'b0, imm_wr};
                            else B = {20'hfffff, imm_wr};
                       
                   
                    end
                   
                   
                    default:
                    begin
                    B = 32'b0;
                    destination_register = rd_wire;
                   
                    end
                   
                   
                   
                endcase
//            
                    stage = STAGE_4;
//                
               
               
                end
                STAGE_4: begin
//            
                   
                    case(ld_store)
                        2'b10:begin
                     
                        enable_write = 1'b1;
                     
                        destination_register = rd_wire;
                        write_value = d_cache[temp+B];

                        end
                        2'b01:
                        begin
                   
                        enable_write = 1'b0;
//                    
                        d_cache[data_cache_address] = A;                        
                       
                        end
                        default:
                        begin
                            enable_write = 1'b1;
                            write_value = LH_wire;
                           
                       
                        end
                       
                       
                    endcase
                    stage = STAGE_1;
                    for (i=0;i<8;i=i+1) $display("%d value is %b",i, d_cache[i]);
                   
               
               
                end
               
                default: begin
                end
           
           
           
            endcase
       
       
       
       
       
        end
    end
   
   

   
   
   
   
   
   
endmodule



module decoder_with_clk(
    input [31:0] instruction,
    input clk,
    output reg [16:0] opcode,
    output reg [4:0] rs1,
    output reg [4:0] rs2,
    output reg [4:0] rd,
    output reg [11:0] immediate,
    output reg [3:0] risl_type
    // ri type is an indicator of whether it is r-type or i-type
   
);
// if instruction is r-type, ri_type=10, if i-type, ri_type = 01, if invalid, ri_type = 00

parameter [3:0]
R_type = 4'b1000,
I_type = 4'b0100,
S_type = 4'b0010,
L_type = 4'b0001;


//always @ (posedge clk, negedge clk)

always @(clk)
//always @ (instruction)
begin
//$display("received instruction is ", instruction);
//opcode <= {instruction[6:0]};
rd = instruction[11:7];
rs1 = instruction[19:15];
//in both r type and i type these are the same
case (instruction[6:0])
//when opcode is 0110011, it is R-Type
7'b0110011:
begin
opcode <= {instruction[31:25],instruction[14:12], instruction[6:0]};
rs2 <= instruction[24:20];
immediate <= 12'b0;
risl_type <= R_type;

end

// when opcode is 00000111 or 0010011 or 1100111, it is I-type
7'b0000011:
begin
opcode = {7'b0,instruction[14:12], instruction[6:0]};

immediate = instruction[31:20];
rs2 = 5'b0;
risl_type = I_type;
    // for load instruction
    case(instruction[14:12])
        3'b010: begin
            risl_type = L_type;
           
        end
        default:begin
       
        end
       
       
    endcase



end
  7'b0010011:
begin
opcode <= {7'b0,instruction[14:12], instruction[6:0]};

immediate <= instruction[31:20];
rs2 <= 5'b0;
risl_type <= I_type;

end
  7'b1100111:
begin
opcode <= {7'b0,instruction[14:12], instruction[6:0]};

immediate <= instruction[31:20];
rs2 <= 5'b0;
risl_type <= I_type;

end

// S_type if opcode is 0100011
7'b0100011: begin
    rd <= 5'b0;
    opcode <= {7'b0,instruction[14:12], instruction[6:0]};

    immediate <= {instruction[31:25], instruction[11:7]};
    rs2 <= instruction[24:20];
    risl_type <= S_type;

end


// invalid opcode
  default:
begin
opcode <= 17'b0;
immediate <= 0;
rs2 <=0;
risl_type <= 2'b00;
rd <= 0;
rs1 <= 0;

end


endcase
//$display("from decoder ",opcode, rs1, rs2, rd, immediate);
end
  endmodule



module reg_file(
//    input [31:0] instruction,
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] rd,
//    input enable_read,
    input enable_write,
    input [31:0] write_value,
    input clk,
    input rst,
    input enable,
   
   
    output reg [31:0] read_value_1,
    output reg [31:0] read_value_2
    );
    reg [31:0] register_file [31:0];
//    always @ (rs1 or rs2 or rd or enable_write)
//    begin
//    end
   
    integer i;
    always @(posedge clk)
    begin
        if(enable == 1)
        begin
            $display("");
            $display("x10 is %b ", register_file[10]);
            $display("x27 is %b", register_file[27]);
            $display("x28 is %b", register_file[28]);
            $display("x29 is %b" , register_file[29]);
            $display("x30 is %b", register_file[30]);
            $display("");
            if(rst)
            begin
                for(i=0;i<32;i = i+1)
                begin
                    register_file[i] = 32'h0;
                end
            end
           
            else
            begin
                case(enable_write)
                // if enable_write is 1
                    1'b1:
                    begin
                       
                        case(rd)
                        //if address is zero write will not happen since x0 hardcoded to zero
                            5'b0:
                            begin
                                read_value_1 <= 32'h0;
                                read_value_2 <= 32'h0;
                            end
                           
                            //any address other than 0
                            default:
                            begin
                                read_value_1 <= 32'h0;
                                read_value_2 <= 32'h0;
                                register_file[rd] <= write_value;
                            end
                        endcase
                    end
                   
                   
                    // if enable_write is 0
                    1'b0:
                    begin
                        read_value_1 <= register_file[rs1];
                        read_value_2 <= register_file[rs2];
                    end
                    default:
                    begin
                   
                    end
                 
               
               
               
                endcase
           
            end
       
        end
       
        else
        // if register is not enabled,
        // that is, enable input is 0
        begin
       
        read_value_1 <= 32'b0;
        read_value_2 <= 32'b0;
        end
   
    end
   
   
   
endmodule


module booth_multiplier(
  input [31:0] A,
  input [31:0] B,
  input clk,
  output reg [63:0] answer
);
 

 
 
  reg [63:0] accumulator;
  reg [63:0] A_complete;
  reg [63:0] A_comp_complete;
  reg flag;
//   initial begin
//     accumulator <= 64'b0;
//     answer <= 64'b0;
//     A_complete <= 64'b0;
//     A_comp_complete <= 64'b0;
//   end
  integer i;
//   always @ (A or B)
 
  always @ (A or B)
    begin
      flag = 1'b0;
    end
 
  always @ (posedge clk)
    begin
      if(~flag)
        begin
      accumulator <= 64'b0;
    answer <= 64'b0;
    A_complete <= 64'b0;
    A_comp_complete <= 64'b0;
//       accumulator <=64'b0;
          if (~A[31])
            begin
              A_complete = {32'b0,A};
              A_comp_complete = {32'hffffffff,-A};
            end
          else
            begin
              A_complete ={32'hffffffff,A};
              A_comp_complete = {32'b0,-A};
            end
       
      for (i=0;i<32;i = i+1)
        begin
//        $display(answer);
          if(i==0)
            begin
              if (B[0])
                begin
                  accumulator = A_comp_complete;
                  A_complete = A_complete << 1;
                  A_comp_complete = A_comp_complete << 1;
                 
                end
              else
                begin
                  A_complete = A_complete << 1;
                  A_comp_complete = A_comp_complete << 1;
                end
            end
          else
            begin
              if(B[i-1] - B[i] == 1)
                begin
                  accumulator = accumulator + A_complete;
                  A_complete = A_complete << 1;
                  A_comp_complete = A_comp_complete << 1;
                end
              else if (B[i-1] - B[i] == -1)
                begin
                  accumulator = accumulator + A_comp_complete;
                  A_complete = A_complete << 1;
                  A_comp_complete = A_comp_complete << 1;
                end
              else
                begin
                  A_complete = A_complete << 1;
                  A_comp_complete = A_comp_complete << 1;
                end
            end
         
          answer <= accumulator;
        end
    end
      else
    begin
//       answer <= accumulator;
      flag = 1'b1;
    end
    end
 
   
       
 
endmodule
module claAdder(
    input [3:0] x,
    input [3:0] y,
    input cin,
    output [3:0] sum,
    output cout,
    output overflow
    );
  wire cneg1;
    assign cneg1 = cin;
    wire c0,c1,c2,c3,g0,g1,g2,g3,p0,p1,p2,p3;
    assign c0 = x[0]&y[0] | x[0]&cin | y[0]&cin;
    assign p0 = x[0]^y[0];
    assign p1 = x[1]^y[1];
    assign p2 = x[2]^y[2];
    assign p3 = x[3]^y[3];
    assign g0 = x[0] & y[0];
    assign g1 = x[1] & y[1];
    assign g2 = x[2] & y[2];
    assign g3 = x[3] & y[3];
    assign c1 = g1 | (p1&c0);
    assign c2 = g2 | (p2&g1) | p2&p1&c0;
    assign c3 = g3 | p3&c2 | p3&p2&g1 | p3&p2&p1&c0;
   
    assign overflow = c3^c2;
    assign sum[0] = p0 ^ cneg1;
    assign sum[1] = p1 ^ c0;
    assign sum[2] = p2 ^ c1;
    assign sum[3] = p3 ^ c2;
    assign cout =  c3;
endmodule



module cla32(
    input [31:0] A,
    input [31:0] B,
    output [31:0] C,
    output overflow_from_module
    );
    wire [31:0] ans;
    wire carry[7:0];
    wire overflow [7:0];
    claAdder cla1(A[3:0],B[3:0],0,ans[3:0],carry[0],overflow[0]);
    claAdder cla2(A[7:4],B[7:4],carry[0],ans[7:4],carry[1],overflow[1]);
    claAdder cla3(A[11:8],B[11:8],carry[1],ans[11:8],carry[2],overflow[2]);
    claAdder cla4(A[15:12],B[15:12],carry[2],ans[15:12],carry[3],overflow[3]);
    claAdder cla5(A[19:16],B[19:16],carry[3],ans[19:16],carry[4],overflow[4]);
    claAdder cla6(A[23:20],B[23:20],carry[4],ans[23:20],carry[5],overflow[5]);
    claAdder cla7(A[27:24],B[27:24],carry[5],ans[27:24],carry[6],overflow[6]);
    claAdder cla8(A[31:28],B[31:28],carry[6],ans[31:28],carry[7],overflow[7]);
    assign overflow_from_module = overflow[7];
    assign C = ans;
endmodule



module cla32_adder(input [31:0] A,input [31:0] B, output [31:0] answer, output overflow);
    cla32 ADDER(A,B,answer,overflow);
endmodule

module cla32_subtracter(input [31:0] A,input [31:0] B, output [31:0] answer, output overflow);
    wire [31:0] B_complement;
    assign B_complement = ~B + 1;
    cla32 SUBTRACTER(A,B_complement, answer, overflow);
endmodule





module control_unit(input [16:0] opcode, output reg [2:0] enable);
parameter [2:0]
Eadd = 3'b100,
Esub = 3'b010,
Emul = 3'b001,
Edefault = 3'b0;
always @ (opcode)
begin
case(opcode)
17'b00000000000110011: enable <= Eadd;
17'b00000000000010011:enable <= Eadd;
17'b01000000000110011: enable <= Esub;
17'b00000010000110011: enable <= Emul;
default: enable <= Edefault;
endcase
end
endmodule




module alu_with_clk(
input [31:0] A,
input [31:0] B,
input [16:0] opcode,
input clk,
output reg [31:0] UH,
output reg [31:0] LH,
output reg overflow
//output reg alu_clk_reg
    );
   
    parameter [2:0]
    Eadd = 3'b100,
    Esub = 3'b010,
    Emul = 3'b001,
    Edefault = 3'b0;
   
   
   
   
    wire alu_clk;
//     reg change;
//     clk_divider_32 clkdivider(clk,alu_clk);
   
    always @ (A or B or opcode)
//    always @ (posedge clk)    
    begin
//    $display("a and b in alu are ", A, B);
//    $display("opcode is ", opcode);
//       change = 1'b0;
    UH <= 32'b0;
    LH <= 32'b0;
    overflow <= 0;
    end
   
    wire [2:0] enable;
    control_unit CU(opcode,enable);
   
    wire [31:0] LH_from_add;
    wire [31:0] LH_from_sub;
    wire [31:0] LH_from_mul;
    wire overflow_add,overflow_sub;
    wire [31:0] UH_from_mul;
    wire [63:0] mul_ans;
    assign LH_from_mul = mul_ans[31:0];
    assign UH_from_mul = mul_ans[63:32];
   
    cla32_adder ADD(A,B,LH_from_add,overflow_add);
    cla32_subtracter SUB(A,B,LH_from_sub,overflow_sub);
  booth_multiplier MUL(A,B,clk,mul_ans);
 
//  always @ (posedge clk, negedge clk)
  always @ (clk)
   
    begin
//    $display("a and b in alu are ", A, B);
//       change = 0
//       $display(enable);
    case(enable)
    Eadd:
    begin
    LH <= LH_from_add;
    UH <= 32'b0;
    overflow <= overflow_add;
 
    end
    Esub:
    begin
    LH <= LH_from_sub;
    UH <= 32'b0;
    overflow <= overflow_sub;
 
   
    end
    Emul:
    begin
//        change = ~change;
    LH <= LH_from_mul;
    UH <= UH_from_mul;
    overflow <= 1'b0;
    end
 
 
    default:
    begin
    UH <= 32'b0;
    LH <= 32'b0;
    overflow <= 32'b0;
    end
    endcase
//   $display("output from alu is ", LH);
   
    end

endmodule
