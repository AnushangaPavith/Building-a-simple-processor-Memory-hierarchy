module testbench;
    reg [7:0] data1/* = 8'b1001_0001*/;
    reg signed [7:0] data2/* = 8'b1011_1011*/;
    wire signed [7:0] result;
    reg [2:0] select = 3'b000;
    reg CLK = 1;

    alu alu1(data1, data2, result, select);

    initial
    begin
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("alu_wavedata.vcd");
		$dumpvars(0, testbench);
        
        $monitor($time, "  select = %b\tdata1: %d\tdata2: %d\tresult: %d", select, data1, data2, result);

        data1 = 8'b1001_0001;
        data2 = 8'b1011_1011;

        // data1 = 8'b0010_1010;
        // data2 = 8'b0010_1010;

        #2 select = 3'b001;
        #1 select = 3'b010;
        #1 select = 3'b011;
        
        #2 $finish;
    end

    always
        #1 CLK = ~CLK;
        
endmodule


module alu(DATA1, DATA2, RESULT, SELECT);
    // Define nputs and output
    input [7:0] DATA1, DATA2;
    input signed [2:0] SELECT;
    output reg [7:0] RESULT;

    wire [7:0] Rfor, Radd, Rand, Ror;

    always@ (SELECT or DATA1 or DATA2)
    begin
        // Select output according to the selection
        case(SELECT)
        3'b000: assign RESULT = Rfor;
        3'b001: assign RESULT = Radd;
        3'b010: assign RESULT = Rand;
        3'b011: assign RESULT = Ror;
        endcase

    end
    
    // Instentiate
    forwardmodule mod0(DATA2, Rfor);
    addmodule mod1(DATA1, DATA2, Radd);
    andmodule mod2(DATA1, DATA2, Rand);
    ormodule mod3(DATA1, DATA2, Ror);

endmodule


module forwardmodule(DATA2, RESULT);
    // Define nputs and output
    input signed [7:0] DATA2;
    output reg [7:0] RESULT;

    always@ (DATA2)
    begin
        #1 assign RESULT = DATA2;
    end

endmodule


module addmodule(DATA1, DATA2, RESULT);
    // Define nputs and output
    input [7:0] DATA1;
    input signed [7:0] DATA2;
    output reg[7:0] RESULT;
    reg [7:0] D2;

    always@ (DATA1 or DATA2)
    begin
        #2 RESULT = DATA1 + DATA2;
    end
endmodule


module andmodule(DATA1, DATA2, RESULT);
    // Define nputs and output
    input [7:0] DATA1;
    input signed [7:0] DATA2;
    output reg[7:0] RESULT;

    always@ (DATA1 or DATA2)
    begin
        #1 RESULT = DATA1 & DATA2;
    end
endmodule


module ormodule(DATA1, DATA2, RESULT);
    // Define nputs and output
    input [7:0] DATA1;
    input signed [7:0] DATA2;
    output reg[7:0] RESULT;

    always@ (DATA1 or DATA2)
    begin
        #1 RESULT = DATA1 | DATA2;
    end
endmodule
