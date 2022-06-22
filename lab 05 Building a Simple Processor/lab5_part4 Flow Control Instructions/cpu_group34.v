/*
    Group 34
    E/18/349
    E/18/54
    Lab 05 part 4
*/


module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    reg [31:0] INSTRUCTION;
    
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory

    reg [7:0] instr_mem [0:1023];

    
    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)
    always@ (PC) begin
        #2
        INSTRUCTION = {instr_mem[PC + 3], instr_mem[PC + 2], instr_mem[PC + 1], instr_mem[PC]};
    end
    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        //{instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;
        //{instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;
        //{instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    
    /* 
    -----
     CPU
    -----
    */
    cpu mycpu(PC, INSTRUCTION, CLK, RESET);

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        RESET = 1'b1;
        #5 RESET = 1'b0;

        // #20 RESET = 1'b1;
        // #1 RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        
        // finish simulation after some time
        #65
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule


module cpu(PC, INSTRUCTION, CLK, RESET);
    input [31:0] INSTRUCTION;
    input CLK, RESET;
    output reg signed [31:0] PC;
    wire [31:0] nextPC, jumpPC, NEWPC;
    wire [2:0] READREG1, READREG2, WRITEREG, ALUOP;
    wire [7:0] IMMEDIATE, aluOut, REGOUT1, REGOUT2, REGOUT2MINUS, JUMPIMMED;
    wire [7:0] OUTmux1, OUTmux2, ALURESULT, OPCODE;
    wire isSUB, isIMD, isJMP, isBEQ, write, ZERO;
    reg [7:0] IN;

    // Update PC value by 4
    PCupdate pcModule(PC, nextPC, RESET);

    // Decode the INSTRUCTION
    decode mydecode1(INSTRUCTION, READREG1, READREG2, WRITEREG, IMMEDIATE, JUMPIMMED, OPCODE);

    // Control Unit
    controlUnit myCntrl(OPCODE, isSUB, isIMD, isJMP, isBEQ, ALUOP, write);

    // Jump / beq adder for pc
    PCAdder myPCAdder(nextPC, jumpPC, isJMP, isBEQ, JUMPIMMED);

    // Read / write to registers
    reg_file myreg1(IN, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, write, CLK, RESET);

    // Get minus of two's complement of a value
    twosComplement myminus1(REGOUT2, REGOUT2MINUS);

    // MUX to select between positive and negative value
    MUX2_1 mymux1(REGOUT2, REGOUT2MINUS, isSUB, OUTmux1);

    // MUX to select between immediate value and output of mymux1
    MUX2_1 mymux2(OUTmux1, IMMEDIATE, isIMD, OUTmux2);

    // ALU
    alu myalu1(REGOUT1, OUTmux2, ALURESULT, ALUOP, ZERO);

    // assign alu output to the reg_file input
    always@ (ALURESULT) begin
        IN = ALURESULT;
    end

    MUX3_1 mymux3(nextPC, jumpPC, isJMP, isBEQ, NEWPC, ZERO);

    // Update PC value to next PC address
    always@ (posedge CLK) begin
        #1 PC = NEWPC;
    end

endmodule

// PC update module
module PCupdate(PCvalue, newPC, RESET);
    input signed [31:0] PCvalue;
    input RESET;
    output reg signed [31:0] newPC;

    always@ (PCvalue) begin
        #1
        newPC = PCvalue + 4;
    end
    
    // Reset pc to 0 when reset is high
    always@ (RESET) begin
        if(RESET) begin
            newPC = 0;
        end
    end
    
endmodule

// PC adder used for compute jupm/branch target address
module PCAdder(PC, newPC, isJump, isBeq, increment);
    input signed [31:0] PC;
    input signed [7:0] increment;
    input [7:0] OPCODE;
    input isJump, isBeq, ZERO;
    output reg signed [31:0] newPC;

    always@ (isJump or isBeq) begin
        #1
        if(isJump) newPC = PC + (increment * 4);
        else if(isBeq) newPC = PC + (increment * 4);
        else newPC = PC;
    end
    
endmodule

module controlUnit(OPCODE, isSUB, isIMD, isJump, isBeq, ALUOP, write);
    input [7:0] OPCODE;
    output reg isSUB, isIMD, write, isJump, isBeq;
    output reg [2:0] ALUOP;

    always@ (OPCODE) begin
        case(OPCODE) 
        8'b00000000:    // loadi
        begin
            ALUOP = 3'b000;
            isSUB = 0;
            isIMD = 1;
            write = 1;
            isJump = 0;
            isBeq = 0;
        end
        8'b00000001:    // MOV
        begin
            ALUOP = 3'b000;
            isSUB = 0;
            isIMD = 0;
            write = 1;
            isJump = 0;
            isBeq = 0;
        end
        8'b00000010:   // ADD
        begin
            ALUOP = 3'b001;
            isSUB = 0;
            isIMD = 0;
            write = 1;
            isJump = 0;
            isBeq = 0;
        end
        8'b00000011:    // SUB
        begin
            ALUOP = 3'b001;
            isSUB = 1;
            isIMD = 0;
            write = 1;
            isJump = 0;
            isBeq = 0;
        end
        8'b00000100:    // AND
        begin
            ALUOP = 3'b010;
            isSUB = 0;
            isIMD = 0;
            write = 1;
            isJump = 0;
            isBeq = 0;
        end
        8'b00000101:    // OR
        begin
            ALUOP = 3'b011;
            isSUB = 0;
            isIMD = 0;
            write = 1;
            isJump = 0;
            isBeq = 0;
        end
        8'b00000110:    // J
        begin
            isSUB = 0;
            isIMD = 0;
            write = 0;
            isJump = 1;
            isBeq = 0;
        end
        8'b00000111:    // BEQ
        begin
            ALUOP = 3'b001;
            isSUB = 1;
            isIMD = 0;
            write = 0;
            isJump = 0;
            isBeq = 1;
        end
        endcase
    end


endmodule

// Decode the given instruction
/*
    00000101 00000000 00000100 00000000
    READREG2 READREG1 RESULT    OP_CODE
    IMMDEATE          WRITEREG
*/
module decode(INSTRUCTION, data1, data2, result, immediate, jumpImmed, op_code);
    input [31:0] INSTRUCTION;
    output reg [2:0] data1, data2, result;
    output reg [7:0] immediate, op_code;
    output reg signed [7:0] jumpImmed;

    always@ (INSTRUCTION) begin
        #1;
        // Decode the instruction
        op_code = INSTRUCTION [31:24];
        result = INSTRUCTION [18:16];
        data1 = INSTRUCTION [10:8];
        data2 = INSTRUCTION [2:0];
        immediate = INSTRUCTION [7:0];
        jumpImmed = INSTRUCTION [23:16];

    end
endmodule

// module to convert second operand 
module twosComplement(DATA, OUTPUT);
    input [7:0] DATA;
    output reg [7:0] OUTPUT;

    always@ (DATA) begin
        #1
        OUTPUT = ~DATA;
        OUTPUT = OUTPUT + 1;
    end

endmodule

// 2 to 1 Multiplexer
module MUX2_1(input0, input1, select, outVal);
    input [7:0] input0, input1;
    input select;
    output reg [7:0] outVal;

    always@ (input0, input1, select) begin
        case(select)
        0: outVal = input0;
        1: outVal = input1;
        endcase
    end
endmodule

module MUX3_1(input0, input1, ISJMP, ISBEQ, OUTVAL, ZERO);
    input [31:0] input0, input1;
    input ISJMP,ISBEQ, ZERO;
    output reg [31:0] OUTVAL;

    always@ (input0, input1, ISJMP, ISBEQ, ZERO) begin
        if(ISJMP) OUTVAL = input1;
        else if(ISBEQ) OUTVAL = input1;
        else OUTVAL = input0;
    end

endmodule

// ALU
module alu(DATA1, DATA2, RESULT, SELECT, ZERO);
    // Define nputs and output
    input [7:0] DATA1, DATA2;
    input [2:0] SELECT;
    output reg [7:0] RESULT;
    output reg ZERO;

    wire [7:0] Rfor, Radd, Rand, Ror;

    // Instentiate
    forwardmodule mod0(DATA2, Rfor);
    addmodule mod1(DATA1, DATA2, Radd);
    andmodule mod2(DATA1, DATA2, Rand);
    ormodule mod3(DATA1, DATA2, Ror);

    always@ (SELECT or DATA1 or DATA2) begin
    // always@ (SELECT or Rfor or Radd or Rand or Ror)
        // Select output according to the selection
        case(SELECT)
        3'b000: assign RESULT = Rfor;
        3'b001: assign RESULT = Radd;
        3'b010: assign RESULT = Rand;
        3'b011: assign RESULT = Ror;
        endcase
    end
    always@ (Radd) begin
        
        if(Radd == 0) ZERO = 1;
        else ZERO = 0;

    end


    
endmodule

module forwardmodule(DATA2, RESULT);
    // Define nputs and output
    input [7:0] DATA2;
    output reg [7:0] RESULT;

    always@ (DATA2)
    begin
        #1 RESULT = DATA2;
    end

endmodule

module addmodule(DATA1, DATA2, RESULT);
    // Define nputs and output
    input [7:0] DATA1;
    input [7:0] DATA2;
    output reg[7:0] RESULT;
    reg [7:0] D2;

    always@ (DATA1, DATA2)
    begin
        #2 RESULT = DATA1 + DATA2;
    end
endmodule

module andmodule(DATA1, DATA2, RESULT);
    // Define nputs and output
    input [7:0] DATA1;
    input [7:0] DATA2;
    output reg[7:0] RESULT;

    always@ (DATA1 or DATA2)
    begin
        #1 RESULT = DATA1 & DATA2;
    end
endmodule

module ormodule(DATA1, DATA2, RESULT);
    // Define nputs and output
    input [7:0] DATA1;
    input [7:0] DATA2;
    output reg[7:0] RESULT;

    always@ (DATA1 or DATA2)
    begin
        #1 RESULT = DATA1 | DATA2;
    end
endmodule

// register file module
module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
    
    // Declare ports
    input [2:0] INADDRESS, OUT1ADDRESS, OUT2ADDRESS;
    input [7:0] IN;
    output [7:0] OUT1, OUT2;
    input WRITE, CLK, RESET;

    // Register for check the written values from wavedata. This will not affect the flow.
    reg [7:0] written;
    
    // registers as an array
    reg [7:0] registers[0:7];

    // ayschronously read from registers

    assign #1 OUT1 = registers[OUT1ADDRESS];
    assign #1 OUT2 = registers[OUT2ADDRESS];


    // synchronise with positive edge of clock
    always @(posedge CLK) begin
        // If reset is high, reset all registers: assign to 0
        if(RESET == 1)
        begin
            #1 for(integer i = 0; i < 8; i = i + 1)
            begin
                registers[i] <= 0;
            end
        end

        // When write is high and reset is low, read the value in given register
        if(WRITE == 1 && RESET == 0) 
        begin
            #1 registers[INADDRESS] <= IN;
            // remove below line.
            written <= IN;
        end
    end
endmodule
