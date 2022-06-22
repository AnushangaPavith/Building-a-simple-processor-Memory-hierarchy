/**
 * Group 34
 * E/18/349
 * E/18/354
*/

module testbench;    
    reg [7:0] WRITEDATA;
    reg [2:0] WRITEREG, READREG1, READREG2;
    reg CLK, RESET, WRITEENABLE; 
    wire [7:0] REGOUT1, REGOUT2;
    
    reg_file myregfile(WRITEDATA, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, WRITEENABLE, CLK, RESET);
       
    initial
    begin
        CLK = 1'b1;
        
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("reg_file_wavedata.vcd");
		$dumpvars(0, testbench);
        
        // assign values with time to input signals to see output 
        RESET = 1'b0;
        WRITEENABLE = 1'b0;
        
        #4
        RESET = 1'b1;
        READREG1 = 3'd0;
        READREG2 = 3'd4;
        
        #6
        RESET = 1'b0;
        
        #2
        WRITEREG = 3'd2;
        WRITEDATA = 8'd95;
        WRITEENABLE = 1'b1;
        
        #7
        WRITEENABLE = 1'b0;
        
        #1
        READREG1 = 3'd2;
        
        #7
        WRITEREG = 3'd1;
        WRITEDATA = 8'd28;
        WRITEENABLE = 1'b1;
        READREG1 = 3'd1;
        
        #8
        WRITEENABLE = 1'b0;
        
        #8
        WRITEREG = 3'd4;
        WRITEDATA = 8'd6;
        WRITEENABLE = 1'b1;
        
        #8
        WRITEDATA = 8'd15;
        WRITEENABLE = 1'b1;
        
        #10
        WRITEENABLE = 1'b0;
        
        #6
        WRITEREG = -3'd1;
        WRITEDATA = 8'd50;
        WRITEENABLE = 1'b1;
        
        #5
        WRITEENABLE = 1'b0;
        
        #10
        $finish;
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
         
endmodule

// register file module
module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
    
    // Declare ports
    input [2:0] INADDRESS, OUT1ADDRESS, OUT2ADDRESS;
    input [7:0] IN;
    output [7:0] OUT1, OUT2;
    input WRITE, CLK, RESET;
    
    // registers as an array
    reg [7:0] registers[0:7];

    // ayschronously read from registers
    assign #2 OUT1 = registers[OUT1ADDRESS];
    assign #2 OUT2 = registers[OUT2ADDRESS];

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
        end
    end
endmodule
