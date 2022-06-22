`timescale 1ns/100ps

/*
    Group 34
    E/18/354 Tharaka K.K.D.R.
    E/18/349 Thalisha W.G.A.P.

*/

module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    wire [31:0] INSTRUCTION;
    wire WRITE, READ, BUSYWAIT;
    wire [7:0] ADDRESS, DATAWRITE, DATAREAD;

    // Signals asset with data memory and data cache
	wire mem_read, mem_write, mem_busywait;
	wire [5:0] mem_address;
	wire [31:0] mem_writedata, mem_readdata;

    wire DATA_BUSYWAIT, INSTCACHE_BUSYWAIT;

    // Signals assert with instruction memory and instruction cache
    wire instruction_read, instruction_busywait;
    wire [5:0] instruction_address;
    wire [127:0] instruction_data;
    
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */

    reg [7:0] instr_mem [0:1023];


    // always@ (PC) begin
    //     #2
    //     INSTRUCTION = {instr_mem[PC + 3], instr_mem[PC + 2], instr_mem[PC + 1], instr_mem[PC]};
    // end
    
    initial
    begin        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        // $readmemb("instr_mem.mem", instr_mem);
    end
    
    /* 
    -----
     CPU
    -----
    */
    cpu mycpu(PC, INSTRUCTION, CLK, RESET, WRITE, READ, ADDRESS, DATAWRITE, DATAREAD, BUSYWAIT);

	// Connect to cache controller
	// dcache (clock, reset, CPUread, CPUwrite, CPUaddress, CPUwritedata, CPUreaddata, busywait, mem_read, mem_write,mem_address,mem_writedata,mem_readdata,mem_busywait);
	dcache mycache(CLK, RESET, READ, WRITE, ADDRESS, DATAWRITE, DATAREAD, DATA_BUSYWAIT, mem_read, mem_write, mem_address, mem_writedata, mem_readdata, mem_busywait);

    // Connect the data memory
    data_memory myDataMemory(CLK, RESET, mem_read, mem_write, mem_address, mem_writedata, mem_readdata, mem_busywait);

    // Instruction memory
    instruction_memory myInstMem(CLK, instruction_read, instruction_address, instruction_data, instruction_busywait);

    // Instruction memory cache
    instruction_cache myInstCache(CLK, RESET, PC[9:0], INSTCACHE_BUSYWAIT, instruction_read, instruction_address, instruction_data, instruction_busywait, INSTRUCTION);
    // module instruction_cache(clock, reset, PC, CACHE_BUSYWAIT, INST_MEM_READ, INST_MEM_ADDRESS, INST_MEM_READDATA, INST_MEM_BUSYWAIT, INSTRUCTION);


    assign BUSYWAIT = (DATA_BUSYWAIT || INSTCACHE_BUSYWAIT);

    initial
    begin

        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb);
        
        RESET = 1'b0;
        CLK = 1'b0;
        RESET = 1'b1;
        #6 RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        
        // finish simulation after some time
        #1450
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule


module cpu(PC,INSTRUCTION,CLK,RESET, OUTWRITE, OUTREAD, ADDRESS, DATAWRITE, DATAREAD, BUSYWAIT);
// module cpu(PC,INSTRUCTION,CLK,RESET, DATA_WRITE, DATA_READ, ADDRESS, DATAWRITE, DATAREAD, BUSYWAIT);
/*This module combines the alu and register file in order to do different operations*/
	input CLK,RESET;        //These are two inputs CLOCK and RESET
	wire WRITE;         //This is the wire which signals to Register file to write
	wire [2:0] WRITEREG;    //This is the wire which holds the register number where result should be written
	wire [2:0] READREG1,READREG2;   //These are the two wires which holds the register numbers where operands should be read from
	wire [2:0] ALUOP;         //This is the register which holds the operator which should be delivered to the alu
	reg [7:0] OPERAND0;   //This are the registers which holds the OPERAND1 and the data sholud be written the register file
	//These are the wires which holds operator taken from instruction,two values read from register file,result from alu,immediate value taken from instruction,result of mux1 and operand2
	wire [7:0] OPCODE,REGOUT1,REGOUT2,ALURESULT,IMMEDIATE,MUX1RESULT,MUX2RESULT,OPERAND2,JUMPADD,COM_OUT,OPERAND1,WRITEDATA,COM_IMMEDIATE, newALUResult;
	input [31:0] INSTRUCTION;       //This is the inputs which takes the instruction
	output [31:0] PC;           //This is the output which delivers the PC
	reg [31:0] NextPC;             //This is the wire which holds the next pc value
	wire isLEFT,isSRALEFT;                  //This wire holds the signal whether istruction is sll or not

	// Inpouts and Outputs that needed to make the data memory
	input BUSYWAIT;							// Memory assert signal that hold the pc update and instruction memory fetching
	input [7:0] DATAREAD;                   // Read data from the data memory
	output reg OUTREAD, OUTWRITE;				// Signal that whether read or write should done in the data memory
	output reg [7:0] ADDRESS, DATAWRITE;
	wire isLWD;
	wire DATA_WRITE, DATA_READ;

	pc p1(PC,NextPC,CLK, BUSYWAIT);               //Instantiating the instance of pc module
	complimenter mycomplimenter1(REGOUT2,COM_OUT);   //Instantiating an instance of complimenter
	complimenter mycomplimenter2(IMMEDIATE,COM_IMMEDIATE);  //Instantiating an instance of complimenter
	decoder mydecoder(INSTRUCTION,OPCODE,WRITEREG,READREG1,READREG2,IMMEDIATE,JUMPADD, BUSYWAIT);     ////Instantiating the instance of decoder
	control_unit myc_o(OPCODE,WRITE,ALUOP,IMMEDIATE[7],isLEFT,isSRALEFT, DATA_WRITE, DATA_READ, isLWD, BUSYWAIT);         //Instantiating the instance of control_unit
	mux1 m1(COM_OUT,REGOUT2,OPCODE,MUX1RESULT);     //Instantiating the instance of mux1 module 
	mux2 m2(IMMEDIATE,MUX1RESULT,OPCODE,MUX2RESULT);  //Instantiating the instance of mux2 module
	mux3 m3(MUX2RESULT,COM_IMMEDIATE,isSRALEFT,OPERAND2);
	reg_file r1(WRITEDATA,REGOUT1,REGOUT2,WRITEREG,READREG1,READREG2, (WRITE && !BUSYWAIT), CLK, RESET);      //Instantiating the instance of reg_file module
	alu a1(OPERAND1,OPERAND2,ALURESULT,ALUOP);      //Instantiating the instance of alu module
	mux_layer mm1(OPERAND0[0],OPERAND0[7],OPERAND0[1],OPERAND0[6],OPERAND0[2],OPERAND0[5],OPERAND0[3],OPERAND0[4],OPERAND0[4],OPERAND0[3],OPERAND0[5],OPERAND0[2],OPERAND0[6],OPERAND0[1],OPERAND0[7],OPERAND0[0],isLEFT,OPERAND1[0],OPERAND1[1],OPERAND1[2],OPERAND1[3],OPERAND1[4],OPERAND1[5],OPERAND1[6],OPERAND1[7]);
	mux_layer mm2(ALURESULT[0],ALURESULT[7],ALURESULT[1],ALURESULT[6],ALURESULT[2],ALURESULT[5],ALURESULT[3],ALURESULT[4],ALURESULT[4],ALURESULT[3],ALURESULT[5],ALURESULT[2],ALURESULT[6],ALURESULT[1],ALURESULT[7],ALURESULT[0],isLEFT,newALUResult[0],newALUResult[1],newALUResult[2],newALUResult[3],newALUResult[4],newALUResult[5],newALUResult[6],newALUResult[7]);


    always@(*) begin
        OUTREAD = DATA_READ;
        OUTWRITE = DATA_WRITE;
    end

	// Set the NextPC value (defalut)
	// Only happen when busywait signal is low
	always@(posedge CLK)
	begin
		if(BUSYWAIT != 1) #2 NextPC = PC + 4;
	end

	//Special cases of NextPC
	always@(*)                //If RESET=1, 
	begin
		if(RESET==1)    NextPC = 0;
	end

	// PC operations will only happen if the busywait signal is low
	always@(posedge CLK)
	begin
		if(BUSYWAIT != 1) begin
			if(OPCODE==8'b00000110)   //If instruction is 'j'
			begin
				NextPC = NextPC + (JUMPADD*4);
			end
			else if(OPCODE==8'b00000111)   //If instruction is 'beq'
			begin
				if(ALURESULT==8'b00000000)
				begin        
					NextPC = NextPC + (JUMPADD*4);
				end
			end
			else if(OPCODE==8'b00010001)   //If instruction is 'bne'
			begin
				if(ALURESULT!=8'b00000000)
				begin        
					NextPC = NextPC + (JUMPADD*4);
				end
			end
		end

	end

	always@(REGOUT1) OPERAND0=REGOUT1;               //Assign the value given from the reg_file to OPERAND1

	// Assign values according to accessing the data memory
	always@* begin
			
	// Location in the memory beign accessed in data memory
	ADDRESS = ALURESULT;

	// assign reg output to writedata in data memory
	case(OPCODE)
		8'b00001010:           // swd
		begin
			DATAWRITE = REGOUT1;
		end
		8'b00001011 :           // swi
		begin
			DATAWRITE = REGOUT1;
		end
		default: ;
	endcase
	end

	// MUX for select ReadData value from data_Memory or ALURESULT
	mux3 m4(newALUResult, DATAREAD, isLWD, WRITEDATA);

endmodule


module pc(PC,NextPC,CLK, busywait);
/*This module works as a program counter*/
        output reg [31:0] PC;                //This is the input which holds the current program counter value
        input [31:0] NextPC;       //This is the input which holds the next program counter value
        input CLK, busywait;                //This input takes the CLOCK and RESET values

        always@(posedge CLK)                     //This update the next program counter value according to the current value
        begin
			if(busywait != 1) begin
				#1 PC = NextPC;    //Otherwise, set NextPC value to PC
			end
        end
endmodule


module control_unit(OPCODE,WRITE,ALUOP,SignBIT,isLEFT,isSRALEFT, DATA_WRITE, DATA_READ, isLWD, BUSYWAIT);
        input [7:0] OPCODE;
        input SignBIT;
        output reg WRITE,isLEFT,isSRALEFT, isLWD;
        output reg [2:0] ALUOP;
        wire MUXresult;

        output reg DATA_WRITE, DATA_READ;
        input BUSYWAIT;

        atomic_mux mm3(1'b0,1'b1,SignBIT,MUXresult);

        // When data memory read or write done, make read, write signals to 0
        always@(negedge BUSYWAIT) begin
                DATA_READ = 0;
                DATA_WRITE = 0;
        end

        always@*
        begin
                case (OPCODE)                   //Assigning operators which should be sent to the alu
                        8'b00000000 :           //loadi
                                begin
                                        ALUOP = 3'b000;
                                        isLEFT = 0; 
                                        isSRALEFT = 0;   
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00000001 :           //mov
                                begin
                                        ALUOP = 3'b000;
                                        isLEFT = 0;
                                        isSRALEFT = 0;
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00000010 :           //add
                                begin
                                        ALUOP = 3'b001;
                                        isLEFT = 0;
                                        isSRALEFT = 0;    
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00000011 :           //sub
                                begin
                                        ALUOP = 3'b001;
                                        isLEFT = 0; 
                                        isSRALEFT = 0;   
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00000100 :   //and
                                begin
                                        ALUOP = 3'b010;
                                        isLEFT = 0;
                                        isSRALEFT = 0;    
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00000101 :           //or
                                begin
                                        ALUOP = 3'b011;
                                        isLEFT = 0;
                                        isSRALEFT = 0;    
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00000110 :           //j
                                begin
                                        isLEFT = 0;
                                        isSRALEFT = 0;
                                        WRITE = 0;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00000111 :           //beq
                                begin
                                        ALUOP = 3'b000;
                                        isLEFT = 0;
                                        isSRALEFT = 0;    
                                        WRITE = 0;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00010001 :           //bne
                                begin
                                        ALUOP = 3'b000;
                                        isLEFT = 0; 
                                        isSRALEFT = 0;   
                                        WRITE = 0;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00001001 :           //mlt
                                begin
                                        ALUOP = 3'b100;
                                        isLEFT = 0; 
                                        isSRALEFT = 0;   
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00001101 :           //sll (logical shift left)
                                begin
                                        ALUOP = 3'b101;    
                                        isLEFT = 1;
                                        isSRALEFT = 0;
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00001110 :           //srl (logical shift right)
                                begin
                                        ALUOP = 3'b101;
                                        isLEFT = 0;
                                        isSRALEFT = 0;
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00001111 :           //sra (arithmatic shift right)
                                begin
                                        ALUOP = 3'b101;
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                        isLEFT = MUXresult;
                                        if(isLEFT) isSRALEFT = 1;
                                        else isSRALEFT = 0;
                                end 
                        8'b00010000 :           //ror
                                begin
                                        ALUOP = 3'b111;
                                        isLEFT = 0;
                                        isSRALEFT = 0;    
                                        WRITE = 1;
                                        DATA_READ = 0;
                                        DATA_WRITE = 0;
                                        isLWD = 0;
                                end
                        8'b00001000 :           // lwd (read from memory(address in reg file) and write in regfile)
                                begin
                                        ALUOP = 3'b000;
                                        isLEFT = 0;
                                        isSRALEFT = 0;
                                        WRITE = 1;
                                        DATA_READ = 1;
                                        DATA_WRITE = 0;
                                        isLWD = 1;
                                end
                        8'b00011111 :           // lwi (read from memory(address in immediate) and write in regfile)
                                begin
                                        ALUOP = 3'b000;
                                        isLEFT = 0;
                                        isSRALEFT = 0;
                                        WRITE = 1;
                                        DATA_READ = 1;
                                        DATA_WRITE = 0;
                                        isLWD = 1;
                                end
                        8'b00001010 :           // swd (read from regfile and write on memory(address in reg file))
                                begin
                                        ALUOP = 3'b000;
                                        isLEFT = 0;
                                        isSRALEFT = 0;
                                        WRITE = 0;
                                        DATA_READ = 0;
                                        DATA_WRITE = 1;
                                        isLWD = 0;
                                end
                        8'b00001011 :           // swi (read from regfile and write on memory(address in immediate))
                                begin
                                        ALUOP = 3'b000;
                                        isLEFT = 0;
                                        isSRALEFT = 0;
                                        WRITE = 0;
                                        DATA_READ = 0;
                                        DATA_WRITE = 1;
                                        isLWD = 0;
                                end
                        default: ;
                endcase
  
        end

        
endmodule


module complimenter(INPUT,OUTPUT);
	/*This module compliments the input*/
	input [7:0] INPUT;      //This is the input
	wire [7:0] OUT;
	output [7:0] OUTPUT;        //This is the complimented output

	//Instatntiating 8 parallel NOT gates to negate the INPUT value
	not n0(OUT[0],INPUT[0]);
	not n1(OUT[1],INPUT[1]);
	not n2(OUT[2],INPUT[2]);
	not n3(OUT[3],INPUT[3]);
	not n4(OUT[4],INPUT[4]);
	not n5(OUT[5],INPUT[5]);
	not n6(OUT[6],INPUT[6]);
	not n7(OUT[7],INPUT[7]);

	//Instatntiatig ADD module to add 1 to the negated value
	ADD aa1(OUT,8'b00000001,OUTPUT);

endmodule


module decoder(INSTRUCTION,OPCODE,WRITEREG,READREG1,READREG2,IMMEDIATE,JUMPADD, BUSYWAIT);
        /*This module takes necessary parts out of the instruction*/
        input [31:0] INSTRUCTION;
        output reg [7:0] OPCODE,IMMEDIATE,JUMPADD;
        output reg [2:0] WRITEREG,READREG1,READREG2;
        input BUSYWAIT;

        always@(negedge BUSYWAIT) begin
                OPCODE = 8'b00000000;
        end

        // Decoding the isntruction
        always@(INSTRUCTION)
        begin
        #1;
                OPCODE    = INSTRUCTION[31:24];  //Observing the OPCODE from the instruction
                WRITEREG  = INSTRUCTION[18:16];  //Observing the destination register number from the instruction
                READREG1  = INSTRUCTION[10: 8];  //Observing the source (for operand1) register number from the instruction
                READREG2  = INSTRUCTION[2 : 0];  //Observing the source (for operand2) register number from the instruction
                IMMEDIATE = INSTRUCTION[7: 0];   //Observing the immediate value from the instruction
                JUMPADD   = INSTRUCTION[23:16];  //Observing the number of instructions should be jumped
        end
endmodule

module mux1(COM_INPUT,OPERAND2,SELECT,RESULT);
/*This module works as a mux. If the current operation is a substraction, this converts the second operand to the negative 2s compliment*/
	input [7:0] COM_INPUT,OPERAND2,SELECT;    //These are the inputs which takes complimented operand,operand and operation
	output reg [7:0] RESULT;        //This is the output which sends the result

	always@*
	begin
	if(SELECT==8'b00000011)        //If operation is substraction
			#1 RESULT = COM_INPUT;        //Complimenting the operand
	else if(SELECT==8'b00000111)   //If operation is beq
			#1 RESULT = COM_INPUT;        //Complimenting the operand
	else if(SELECT==8'b00010001)   //If operation is bne
			#1 RESULT = COM_INPUT;        //Complimenting the operand
	else
			RESULT = OPERAND2;      //Otherwise, send the original value
	end

endmodule


module mux2(IMMEDIATE,OPERAND2,SELECT,RESULT);
/*This module works as a mux. If the current operation is a loading, this assigns the immediate value as the operand, otherwise the value taken from the register file*/
	input [7:0] IMMEDIATE,OPERAND2,SELECT;  //These are the inputs which takes immediate value, value taken from register file and the operator
	output reg [7:0] RESULT;                //this is the output chich sends the result

	always@*
	begin
	if(SELECT==8'b00000000)         //If operation is loading
			RESULT = IMMEDIATE;     //Assigning the immediate value
	else if(SELECT==8'b00001101)         //If operation is logical shift left
			RESULT = IMMEDIATE;     //Assigning the immediate value
	else if(SELECT==8'b00001110)         //If operation is logical shift right
			RESULT = IMMEDIATE;     //Assigning the immediate value
	else if(SELECT==8'b00001111)         //If operation is arithmetic shift right
			RESULT = IMMEDIATE;     //Assigning the immediate value
	else if(SELECT==8'b00010000)         //If operation is rotate right
			RESULT = IMMEDIATE;     //Assigning the immediate value

	// for lwi & swi immediate value is used
	else if(SELECT==8'b00001011)         //If operation is swi
			RESULT = IMMEDIATE;     //Assigning the immediate value
	else if(SELECT==8'b00011111)         //If operation is lwi
			RESULT = IMMEDIATE;     //Assigning the immediate value
			
	else
			RESULT = OPERAND2;      //Otherwise, assigning the value taken out from register file
	end

endmodule

module mux3(MUX2RESULT,COM_IMMEDIATE,isSRALEFT,OPERAND2);
	//This module behaves as a mux. If control signal is 1, output is Complimented IMMEDIATE value. Otherwise the result of the MUX2
	//This module supprts to the excecution of sra instruction (if it is sra instruction and if immediate value should be complimented, then only isSRALEFT is 1)
	input [7:0] MUX2RESULT,COM_IMMEDIATE;   //These are inputs
	input isSRALEFT;        //This is the control signal
	output reg [7:0] OPERAND2;      //This is the output
	
	always@*
	begin
	if(isSRALEFT)   
			OPERAND2 = COM_IMMEDIATE;
	else            
			OPERAND2 = MUX2RESULT;
	end

endmodule

module alu(DATA1,DATA2,RESULT,SELECT);
/*This module executes a mux which selects the correct answer to be sent
* according to the 'SELECT' input*/
	input [7:0] DATA1, DATA2;       //These are the two inputs operand 1 and 2
	input [2:0] SELECT;             //This is the input whch indicates the operation to be done
	
	output reg [7:0] RESULT;        //This is the output which sends the result out
	wire [7:0] FORWARDresult,ADDresult,ANDresult,ORresult,MLTresult,SRLresult,RORresult;      //This wire holds the answers of different operations

	//Instantiating modules
	Forward forward1(DATA2,FORWARDresult);  //Doing the forward operation
	Add add1(DATA1,DATA2,ADDresult);        //Doing the adding operation
	And and1(DATA1,DATA2,ANDresult);        //Doing the AND operation
	Or or1(DATA1,DATA2,ORresult);           //Doing the OR operation
	Mlt mlt1(DATA1,DATA2,MLTresult);        //Doing the multipling operation
	Srl srl1(DATA1,DATA2,SRLresult);        //Doing the logical shift left operation
	Ror ror1(DATA1,DATA2,RORresult);        //Doing the rotate right operation

	//This is the MUX part
	always @(DATA1 or DATA2)
	begin
		case (SELECT)
			3'b000:         //When SELECT==000
				#1 RESULT = FORWARDresult;      //FORWARD (loadi,mov)

			3'b001:         //when SELECT==001
				#2 RESULT = ADDresult;  //ADD (add,sub)

			3'b010:         //When SELECT==010
				#1 RESULT = ANDresult;  //AND (and)

			3'b011:         //When SELECT==011
				#1 RESULT = ORresult;   //OR (or)

			3'b100:         //When SELECT==100
				#1 RESULT = MLTresult;   //MLT (mlt)
			
			3'b101:         //When SELECT==101
				#1 RESULT = SRLresult;   //SRL (logical shift right)

			3'b111:         //When SELECT==111
				#1 RESULT = RORresult;   //ROR (rotate right)
			
			default:;       //Otherwise

		endcase
	end
endmodule

module Forward(operand,result);
	/*This module assigns the value of the operand to the result*/
	input [7:0] operand;    //This is the input
	output [7:0] result;    //This is the output
	assign result = operand;

endmodule

module Add(operand1,operand2,result);
	/*This module assigns the sum of operand 1 and 2 to the result*/
	input [7:0] operand1,operand2;  //These are the inputs
	output [7:0] result;            //This is the output
	assign result = operand1 + operand2;

endmodule

module And(operand1,operand2,result);
	/*This module assigns the bitwise and of operand 1.2 to the result*/
	input [7:0] operand1,operand2;  //These are the inputs
	output [7:0] result;            //This is the output
	assign result = operand1 & operand2;

endmodule

module Or(operand1,operand2,result);
	/*This module assigns the bitwise or of operand 1,2 to the result*/
	input [7:0] operand1,operand2;  //These are the inputs
	output [7:0] result;            //This is the input
	assign result = operand1 | operand2;

endmodule


module Ror(IN,OFFSET,OUT);
    //This module rotates a 8-bit binary value
    input [7:0] IN,OFFSET;      //IN is the operand to be rotated and OFFSET is the amount in which 'IN' should be rotated 
    output [7:0] OUT;           //OUT is the output value after 'IN' being rotated
    //These below wires are used to connect mux layers
    wire o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12,o13,o14,o15,o16,o17,o18,o19,o20,o21,o22,o23,o24,o25,o26,o27,o28,o29,o30,o31,o32;        
    wire o33,o34,o35,o36,o37,o38,o39,o40,o41,o42,o43,o44,o45,o46,o47,o48,o49,o50,o51,o52,o53,o54,o55,o56;
    
    //Instantiating the mux layers (8 mux layers)
    mux_layer m9(IN[0],IN[1],IN[1],IN[2],IN[2],IN[3],IN[3],IN[4],IN[4],IN[5],IN[5],IN[6],IN[6],IN[7],IN[7],IN[0],OFFSET[0],o1,o2,o3,o4,o5,o6,o7,o8);
    mux_layer m10(o1,o3,o2,o4,o3,o5,o4,o6,o5,o7,o6,o8,o7,o1,o8,o2,OFFSET[1],o9,o10,o11,o12,o13,o14,o15,o16);
    mux_layer m11(o9,o13,o10,o14,o11,o15,o12,o16,o13,o9,o14,o10,o15,o11,o16,o12,OFFSET[2],o17,o18,o19,o20,o21,o22,o23,o24);
    mux_layer m12(o17,o17,o18,o18,o19,o19,o20,o20,o21,o21,o22,o22,o23,o23,o24,o24,OFFSET[3],o25,o26,o27,o28,o29,o30,o31,o32);
    mux_layer m13(o25,o25,o26,o26,o27,o27,o28,o28,o29,o29,o30,o30,o31,o31,o32,o32,OFFSET[4],o33,o34,o35,o36,o37,o38,o39,o40);
    mux_layer m14(o33,o33,o34,o34,o35,o35,o36,o36,o37,o37,o38,o38,o39,o39,o40,o40,OFFSET[5],o41,o42,o43,o44,o45,o46,o47,o48);
    mux_layer m15(o41,o41,o42,o42,o43,o43,o44,o44,o45,o45,o46,o46,o47,o47,o48,o48,OFFSET[6],o49,o50,o51,o52,o53,o54,o55,o56);
    mux_layer m16(o49,o49,o50,o50,o51,o51,o52,o52,o53,o53,o54,o54,o55,o55,o56,o56,OFFSET[7],OUT[0],OUT[1],OUT[2],OUT[3],OUT[4],OUT[5],OUT[6],OUT[7]);

endmodule;


module Srl(IN,OFFSET,OUT);
    //This module shifts bits of a 8-bit binary value to its right
    input [7:0] IN,OFFSET;      //IN is the operand to be shifted and OFFSET is the amount in which 'IN' should be shifted
    output [7:0] OUT;           //OUT is the output value after 'IN' being shifted
    //These below wires are used to connect mux layers
    wire o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12,o13,o14,o15,o16,o17,o18,o19,o20,o21,o22,o23,o24,o25,o26,o27,o28,o29,o30,o31,o32;
    wire o33,o34,o35,o36,o37,o38,o39,o40,o41,o42,o43,o44,o45,o46,o47,o48,o49,o50,o51,o52,o53,o54,o55,o56;
    
    //Instantiating the mux layers (8 mux layers)
    mux_layer right(IN[0],IN[1],IN[1],IN[2],IN[2],IN[3],IN[3],IN[4],IN[4],IN[5],IN[5],IN[6],IN[6],IN[7],IN[7],1'b0,OFFSET[0],o1,o2,o3,o4,o5,o6,o7,o8);
    mux_layer m2(o1,o3,o2,o4,o3,o5,o4,o6,o5,o7,o6,o8,o7,1'b0,o8,1'b0,OFFSET[1],o9,o10,o11,o12,o13,o14,o15,o16);
    mux_layer m3(o9,o13,o10,o14,o11,o15,o12,o16,o13,1'b0,o14,1'b0,o15,1'b0,o16,1'b0,OFFSET[2],o17,o18,o19,o20,o21,o22,o23,o24);
    mux_layer m4(o17,1'b0,o18,1'b0,o19,1'b0,o20,1'b0,o21,1'b0,o22,1'b0,o23,1'b0,o24,1'b0,OFFSET[3],o25,o26,o27,o28,o29,o30,o31,o32);
    mux_layer m5(o25,1'b0,o26,1'b0,o27,1'b0,o28,1'b0,o29,1'b0,o30,1'b0,o31,1'b0,o32,1'b0,OFFSET[4],o33,o34,o35,o36,o37,o38,o39,o40);
    mux_layer m6(o33,1'b0,o34,1'b0,o35,1'b0,o36,1'b0,o37,1'b0,o38,1'b0,o39,1'b0,o40,1'b0,OFFSET[5],o41,o42,o43,o44,o45,o46,o47,o48);
    mux_layer m7(o41,1'b0,o42,1'b0,o43,1'b0,o44,1'b0,o45,1'b0,o46,1'b0,o47,1'b0,o48,1'b0,OFFSET[6],o49,o50,o51,o52,o53,o54,o55,o56);
    mux_layer m8(o49,1'b0,o50,1'b0,o51,1'b0,o52,1'b0,o53,1'b0,o54,1'b0,o55,1'b0,o56,1'b0,OFFSET[7],OUT[0],OUT[1],OUT[2],OUT[3],OUT[4],OUT[5],OUT[6],OUT[7]);

endmodule;


module mux_layer(IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10,IN11,IN12,IN13,IN14,IN15,IN16,SELECT,OUT1,OUT2,OUT3,OUT4,OUT5,OUT6,OUT7,OUT8);
    //This is a module which operates 8, 2to1 muxes parallelly
    input IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10,IN11,IN12,IN13,IN14,IN15,IN16,SELECT;        //These are the 16 inputs to 8 muxes and SELECT is the selecting input of muxes (same for all muxes)
    output OUT1,OUT2,OUT3,OUT4,OUT5,OUT6,OUT7,OUT8;             //These are the 8 paralllel outputs of the muxes

    ////Instantiating the muxes (8 muxes)
    atomic_mux a1(IN1 ,IN2 ,SELECT,OUT1);
    atomic_mux a2(IN3 ,IN4 ,SELECT,OUT2);
    atomic_mux a3(IN5 ,IN6 ,SELECT,OUT3);
    atomic_mux a4(IN7 ,IN8 ,SELECT,OUT4);
    atomic_mux a5(IN9 ,IN10,SELECT,OUT5);
    atomic_mux a6(IN11,IN12,SELECT,OUT6);
    atomic_mux a7(IN13,IN14,SELECT,OUT7);
    atomic_mux a8(IN15,IN16,SELECT,OUT8);

endmodule


module atomic_mux(INPUT0,INPUT1,SELECT,OUTPUT);
    //This is a simple 2to1 mux
    input INPUT0,INPUT1,SELECT;         //INPUT0 and INPUT1 are the inputs and SELECT is the selecting input
    output reg OUTPUT;                  //OUTPUT is the output of the mux
    always@*
    begin
        if(SELECT)  OUTPUT=INPUT1;
        else        OUTPUT=INPUT0;
    end
endmodule


module Mlt(IN,OPERAND,OUTPUT);
    //This module generates the multiple of two given binary values
    input [7:0] IN,OPERAND;     //These are the two operands
    output [7:0] OUTPUT;        //This is the output
    //These below wires are used to hold the outputs of AND gate layers
    wire [7:0] o1,o2,o3,o4,o5,o6,o7,o8,p1,p2,p3,p4,p5,p6,p7,p8;
    
    //Instatiating 8 AND gate layers
    and_layer al1(IN[0],IN[1],IN[2],IN[3],IN[4],IN[5],IN[6],IN[7],OPERAND[0],o1[0],o1[1],o1[2],o1[3],o1[4],o1[5],o1[6],o1[7]);
    and_layer al2(1'b0,IN[0],IN[1],IN[2],IN[3],IN[4],IN[5],IN[6],OPERAND[1],o2[0],o2[1],o2[2],o2[3],o2[4],o2[5],o2[6],o2[7]);
    and_layer al3(1'b0,1'b0,IN[0],IN[1],IN[2],IN[3],IN[4],IN[5],OPERAND[2],o3[0],o3[1],o3[2],o3[3],o3[4],o3[5],o3[6],o3[7]);
    and_layer al4(1'b0,1'b0,1'b0,IN[0],IN[1],IN[2],IN[3],IN[4],OPERAND[3],o4[0],o4[1],o4[2],o4[3],o4[4],o4[5],o4[6],o4[7]);
    and_layer al5(1'b0,1'b0,1'b0,1'b0,IN[0],IN[1],IN[2],IN[3],OPERAND[4],o5[0],o5[1],o5[2],o5[3],o5[4],o5[5],o5[6],o5[7]);
    and_layer al6(1'b0,1'b0,1'b0,1'b0,1'b0,IN[0],IN[1],IN[2],OPERAND[5],o6[0],o6[1],o6[2],o6[3],o6[4],o6[5],o6[6],o6[7]);
    and_layer al7(1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,IN[0],IN[1],OPERAND[6],o7[0],o7[1],o7[2],o7[3],o7[4],o7[5],o7[6],o7[7]);
    and_layer al8(1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,IN[0],OPERAND[7],o8[0],o8[1],o8[2],o8[3],o8[4],o8[5],o8[6],o8[7]);

    //Instantiating 7 Full adders to add the values stored in wires
    ADD ad1(o1,o2,p1);
    ADD ad2(p1,o3,p2);
    ADD ad3(p2,o4,p3);
    ADD ad4(p3,o5,p4);
    ADD ad5(p4,o6,p5);
    ADD ad6(p5,o7,p6);
    ADD ad7(p6,o8,OUTPUT);

endmodule;


module and_layer(IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,SELECT,OUT1,OUT2,OUT3,OUT4,OUT5,OUT6,OUT7,OUT8);
    //This is an AND gate layer which includes 8 parallel AND gates. A particular one input is common for all the gatese and rest are different
    input IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10,IN11,IN12,IN13,IN14,IN15,IN16,SELECT;        //These are the inputs and SELECT is the common input
    output OUT1,OUT2,OUT3,OUT4,OUT5,OUT6,OUT7,OUT8;     //These are the outputs

    //Instatntiating 8 AND gates
    and a1(OUT1,IN1,SELECT);
    and a2(OUT2,IN2,SELECT);
    and a3(OUT3,IN3,SELECT);
    and a4(OUT4,IN4,SELECT);
    and a5(OUT5,IN5,SELECT);
    and a6(OUT6,IN6,SELECT);
    and a7(OUT7,IN7,SELECT);
    and a8(OUT8,IN8,SELECT);

endmodule


module ADD(op1,op2,result);
/*This module executes 8-Bit adding operation using fulladders*/
	input [7:0] op1,op2;            //These are two inputs,operand 1 and 2
	output [7:0] result;            //This is the outputs which sends the SUM
	wire cout1,cout2,cout3,cout4,cout5,cout6,cout7,cout8;	//These wires delivers carry outs to next adders
		
	//These full adders executes the 8-Bit adding operation
	fulladder f1(op1[0],op2[0],1'b0,result[0],cout1);
	fulladder f2(op1[1],op2[1],cout1,result[1],cout2);
	fulladder f3(op1[2],op2[2],cout2,result[2],cout3);
	fulladder f4(op1[3],op2[3],cout3,result[3],cout4);
	fulladder f5(op1[4],op2[4],cout4,result[4],cout5);
	fulladder f6(op1[5],op2[5],cout5,result[5],cout6);
	fulladder f7(op1[6],op2[6],cout6,result[6],cout7);
	fulladder f8(op1[7],op2[7],cout7,result[7],cout8);
	
endmodule


module fulladder(op1,op2,CarryIN,Answer,CarryOUT);
        //This module is a full adder which adds two bits
	input op1,op2,CarryIN;  //These are two operands and CarryIN is the carry in to the operation from previous operation
	output Answer,CarryOUT; //These are the two outsouts answer and carry out to the next operation
	wire w1,w2,w3;

	xor x1(w1,op1,op2);
	xor x2(Answer,w1,CarryIN);
	and a1(w2,w1,CarryIN);
	and a2(w3,op1,op2);
	or o1(CarryOUT,w2,w3);
endmodule



module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET);
/*This module executes Register File which stores eight 8-bit values*/
        input [2:0] INADDRESS,OUT1ADDRESS,OUT2ADDRESS;  //These are inputs which takes addresses of registers
        input [7:0] IN;                                 //This is the input which takes the value to written
        output reg [7:0] OUT1,OUT2;                         //These are outputs which sends values
        input CLK,RESET,WRITE;                          //These are inputs Clock,Reset,Write
        reg [7:0] regarray[7:0];                        //This is the array which contains 8 registers
        //integer i;

        always@(OUT1ADDRESS or OUT2ADDRESS)
        begin
                #1;
                OUT1 = regarray[OUT1ADDRESS];         //Assign value to the output port
                OUT2 = regarray[OUT2ADDRESS];
        end

        integer i;
        always@(posedge CLK)                                    //At the positive edge
        begin
                #1;
                if(RESET==1)                                    //If RESET==1,
                begin
                        for(i = 0; i < 8; i = i + 1)    //Reset all registers (set to 0)
                        begin
                                regarray[i]=0;
                        end
                end
                else if(WRITE==1)                               //If RESET==0 & WRITE=1,
                begin
                        regarray[INADDRESS] = IN;               //Write the value in the register
                end
        end

endmodule

// Data Memoery
module data_memory(clock, reset, read, write, address, writedata, readdata, busywait);

    input				clock;
    input           	reset;
    input           	read;
    input           	write;
    input[5:0]      	address;
    input[31:0]     	writedata;
    output reg [31:0]	readdata;
    output reg      	busywait;

    //Declare memory array 256x8-bits 
    reg [7:0] memory_array [255:0];

    //Detecting an incoming memory access
    reg readaccess, writeaccess;
    always @(read, write)
    begin
        busywait = (read || write)? 1 : 0;
        readaccess = (read && !write)? 1 : 0;
        writeaccess = (!read && write)? 1 : 0;
    end

    //Reading & writing
    always @(posedge clock)
    begin
        if(readaccess)
        begin
            readdata[7:0]   = #40 memory_array[{address,2'b00}];
            readdata[15:8]  = #40 memory_array[{address,2'b01}];
            readdata[23:16] = #40 memory_array[{address,2'b10}];
            readdata[31:24] = #40 memory_array[{address,2'b11}];
            busywait = 0;
            readaccess = 0;
        end
        if(writeaccess)
        begin
            memory_array[{address,2'b00}] = #40 writedata[7:0];
            memory_array[{address,2'b01}] = #40 writedata[15:8];
            memory_array[{address,2'b10}] = #40 writedata[23:16];
            memory_array[{address,2'b11}] = #40 writedata[31:24];
            busywait = 0;
            writeaccess = 0;
        end
    end
    integer i;
    //Reset memory
    always @(posedge reset)
    begin
        if (reset)
        begin
            for (i=0;i<256; i=i+1)
                memory_array[i] = 0;
            
            busywait = 0;
            readaccess = 0;
            writeaccess = 0;
        end
    end

endmodule

/* ==========================  Cache ========================== */

module dcache (clock, reset, CPUread, CPUwrite, CPUaddress, CPUwritedata, CPUreaddata, busywait, mem_read, mem_write,mem_address,mem_writedata,mem_readdata,mem_busywait);

	// Signals associate with CPU
    input clock;
    input reset;
    input CPUread;
    input CPUwrite;
    input[7:0] CPUaddress;
    input[7:0] CPUwritedata;
    output [7:0] CPUreaddata;
    output reg busywait;

	// Signals associate with data memory
    input [31:0] mem_readdata;
    input mem_busywait;
    output reg mem_read, mem_write;
    output reg [5:0] mem_address;
    output reg [31:0] mem_writedata;

    parameter IDLE = 3'b000, MEM_READ = 3'b001, MEM_WRITE = 3'b010, UPD_CACHE = 3'b011;
    reg [2:0] state, next_state;

	// wires and regs need inside the data cache
    wire [2:0] index;
    wire [2:0] tag;
    wire [1:0] offset;
    wire dirty, hit, valid, isTagEqual;

    reg [31:0] cacheMemory [0:7];
    reg [31:0] block;
    reg [2:0] TagsArray [7:0];
    reg [7:0] ValidBitArray, DirtyBitArray;

    /* Cache

    +-------+ +-------+ +-------+ +-------------+ +----------------+----------------+----------------+----------------+
    | index | | dirty | | valid | | tag (3bits) | | data01 (8bits) | data02 (8bits) | data03 (8bits) | data04 (8bits) |
    +-------+ +-------+ +-------+ +-------------+ +----------------+----------------+----------------+----------------|
    |  000  | |       | |       | |             | |                |                |                |                |
    |  001  | |       | |       | |             | |                |                |                |                |
    |  010  | |       | |       | |             | |                |                |                |                |
    |  011  | |       | |       | |             | |                |                |                |                |
    |  100  | |       | |       | |             | |                |                |                |                |
    |  101  | |       | |       | |             | |                |                |                |                |
    |  110  | |       | |       | |             | |                |                |                |                |
    |  111  | |       | |       | |             | |                |                |                |                |
    +-------+ +-------+ +-------+ +-------------+ +----------------+----------------+----------------+----------------+
    */

    /* Cache Controller FSM Start */

    always@ (CPUread, CPUwrite) begin
        // If read or write 
        if(CPUread || CPUwrite)
            busywait = 1;
        else
            busywait = 0;
    end

    assign index = CPUaddress[4:2];     // 7 6 5 | 4 3 2 | 1 0

    always@(*) begin
        #1 block = cacheMemory[CPUaddress[4:2]];
    end

    assign #1 tag = TagsArray[index];
    assign #1 valid = ValidBitArray[index];
    assign #1 dirty = DirtyBitArray[index];

    // Compare Address Tag and Tag in Cache put final value to isTagEqual
    tagcmp compareTags(tag, CPUaddress[7:5], isTagEqual);
    // find hit or miss from valid bit & tag equal
	assign hit = valid && isTagEqual;
    // Assign the wanted data from cache block
    selectionMUX MUX1(block, CPUaddress[1:0], CPUreaddata, CPUread);

	always@ (posedge clock) begin
		if(hit && CPUwrite) begin
			#1;
			DirtyBitArray[index] = 1;
			case (CPUaddress[1:0])
			2'b00: cacheMemory[index][7:0] = CPUwritedata;
			2'b01: cacheMemory[index][15:8] = CPUwritedata;
			2'b10: cacheMemory[index][23:16] = CPUwritedata;
			2'b11: cacheMemory[index][31:24] = CPUwritedata;
			default: ;
			endcase
		end
	end

    // combinational next state logic
    always @(*)
    begin
        case (state)
        IDLE:
			// When read/write miss & not a dirty cache block, read the required data block from memory
            if ((CPUread || CPUwrite) && !dirty && !hit)  
                next_state = MEM_READ;

			// When read / write miss & a dirty cache block write back the cache data into the data_memory
            else if ((CPUread || CPUwrite) && dirty && !hit)
                next_state = MEM_WRITE;

            else
                next_state = IDLE;
        
        MEM_READ:
			// IF data memory done fetching the data block go to update cache state
            if (!mem_busywait)
                next_state = UPD_CACHE;
			// stay in MEM_READ state untill mem_busywait de-asserts
            else    
                next_state = MEM_READ;

        MEM_WRITE:
			// When write back is completed, go to MEM_READ state and fetch the required data block from memory
            if (!mem_busywait)
                next_state = MEM_READ;
			// Stay in MEM_WRITE state untill mem_busywait de-asserts
            else
                next_state = MEM_WRITE;

        UPD_CACHE:
			// When cache update is done go to idle state
            next_state = IDLE;
            
        endcase
    end

    // combinational output logic
    always @(state)
    begin
        case(state)
        IDLE:
            begin
                mem_read = 0;
                mem_write = 0;
                mem_address = 5'dx;
                mem_writedata = 5'dx;
                busywait = 0;
            end
         
        MEM_READ: 
            begin
                mem_read = 1;
                mem_write = 0;
                mem_address = {CPUaddress[7:2]};
                mem_writedata = 32'dx;
                busywait = 1;
            end

        MEM_WRITE:
            begin
                mem_read = 0;
                mem_write = 1;
                mem_address = {tag, CPUaddress[4:2]};
                mem_writedata = block;
            end
        
        UPD_CACHE:
            begin
                mem_read = 0;
                mem_write = 0;
                mem_address = 5'dx;
                mem_writedata = 5'dx;
                busywait = 0;

                #1 cacheMemory[index] = mem_readdata;
                TagsArray[index] = CPUaddress[7:5];
                ValidBitArray[index] = 1;
                DirtyBitArray[index] = 0;
            end

        endcase
    end

    // sequential logic for state transitioning 
    always @(posedge clock, reset)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

    integer i;

    // Reset the cache
    always@(posedge reset) begin
        for(i = 0; i < 8; i++) begin
			// Set unknown values
            cacheMemory[i] = 32'dx;
            TagsArray[i] = 3'dx;
            ValidBitArray[i] = 0;
            DirtyBitArray[i] = 0;
        end
    end

    always@(posedge clock) begin
        if(hit) begin
            busywait = 0;
        end
    end

    /* Cache Controller FSM End */

endmodule

// Module to compare two tags
// assign #0.9 TAG_COMPARE_MATCH = (TAG_BLOCK == TAG) ? 1 : 0;

module tagcmp(tag1, tag2, isEqual);
    input [2:0] tag1, tag2;
    output isEqual;
    wire w1, w2, w3;

    xnor g1(w1, tag1[0], tag2[0]);
    xnor g2(w2, tag1[1], tag2[1]);
    xnor g3(w3, tag1[2], tag2[2]);

    assign #0.9 isEqual = w1 && w2 && w3;

endmodule

// Module to select wanted data block from Cache data blocks according to offset
module selectionMUX(ALLDATA, select, OUTDATA, IsRead);
    input [31:0] ALLDATA;
    input [1:0] select; // Address offset
    input IsRead;
    output reg [7:0] OUTDATA;

    always@ (*) begin
    if(IsRead) begin

        case(select)
            2'b00: begin
                #1 OUTDATA = ALLDATA[7:0];
            end
            2'b01: begin
                #1 OUTDATA = ALLDATA[15:8];
            end
            2'b10: begin
                #1 OUTDATA = ALLDATA[23:16];
            end
            2'b11: begin
                #1 OUTDATA = ALLDATA[31:24];
            end

            default: ;
        endcase

    end
    end

endmodule

// Instruction cache
// PC -> 011  101    11   00
//       tag index offset 00
module instruction_cache(clock, reset, PC, INSTCACHE_BUSYWAIT, INST_MEM_READ, INST_MEM_ADDRESS, INST_MEM_READDATA, INST_MEM_BUSYWAIT, INSTRUCTION);
    input clock, reset;
    input [9:0] PC;
    input INST_MEM_BUSYWAIT;
    input [127:0] INST_MEM_READDATA;

    // Read signal to the instruction memory
    output reg INST_MEM_READ;
    // Address for fetch instructions
    output reg [5:0] INST_MEM_ADDRESS;

    // Signal to stall the CPU
    output reg INSTCACHE_BUSYWAIT;
    // Send fetched Instruction to the CPU
    output [31:0] INSTRUCTION;

    // Instruction cache memory
    reg [127:0] INSTRUCTION_CACHE [0:7];
    // Instruction block of selected address (4 instructions; 4 Bytes each, 16 Bytes total)
    reg [127:0] INSTRUCTION_DATABLOCK;
    // Arrays to store tags & valid bits
    reg [2:0] InstTagsArray [0:7];
    reg [7:0] InstValidArray;

    wire VALID, HIT, ISTAGMATCH;
    wire [2:0] InstTag, INDEX;

    // Get the Index from current PC address
    assign INDEX = PC[6:4];

    // Fetch the corresponding Instruction block from cache
    always@(*) begin
        #1 INSTRUCTION_DATABLOCK = INSTRUCTION_CACHE[INDEX];
    end

    // Assign tag, valid bit
    assign #1 InstTag = InstTagsArray[INDEX];
    assign #1 VALID = InstValidArray[INDEX];

    // Tag match and check hit or miss
    tagcmp compareTags1(InstTag, PC[9:7], ISTAGMATCH);
    assign HIT = VALID && ISTAGMATCH;

    // At the begining, Stall the CPU until instructions are fetched
    always@(PC) begin
        if(PC == 0)
            INSTCACHE_BUSYWAIT = 1;
    end

    // If there is a hit, de assert the busywait
    always@ (posedge clock) begin
        if(HIT)
            INSTCACHE_BUSYWAIT = 0;
    end

    // When there is a miss, Stall  the cpu
    always@(HIT) begin
        if(!HIT)
            INSTCACHE_BUSYWAIT = 1;
    end

    // Read Requested instruction according to pc from the block (using offset)
    assign #1 INSTRUCTION = ((PC[3:2] == 2'b00) && HIT) ? INSTRUCTION_DATABLOCK[31:0] :
                            ((PC[3:2] == 2'b01) && HIT) ? INSTRUCTION_DATABLOCK[63:32] :
                            ((PC[3:2] == 2'b10) && HIT) ? INSTRUCTION_DATABLOCK[95:64] : INSTRUCTION_DATABLOCK[127:96];


    /*** Instruction cache controller FSM ***/
    parameter IDLE = 2'b00, INST_READ = 2'b01, UPD_CACHE = 2'b10;
    reg [2:3] STATE, NEXT_STATE;

    // Next state logic
    always @(*) begin
        case(STATE)
            IDLE:
                if(!HIT)
                    NEXT_STATE = INST_READ;
                else
                    NEXT_STATE = IDLE;
            
            INST_READ:
                if(!INST_MEM_BUSYWAIT)
                    NEXT_STATE = UPD_CACHE;
                else
                    NEXT_STATE = INST_READ;

            UPD_CACHE:
                NEXT_STATE = IDLE;

        endcase
    end

    // Output logic
    always@ (STATE) begin
        case(STATE)
            IDLE:
            begin
                INST_MEM_READ = 0;
                INST_MEM_ADDRESS = 6'bx;
                INSTCACHE_BUSYWAIT = 0;
            end
            
            INST_READ:
            begin
                // Access instruction memory with read signal and address from PC
                INST_MEM_READ = 1;
                INST_MEM_ADDRESS = {PC[9:4]};
                INSTCACHE_BUSYWAIT = 1;
            end

            UPD_CACHE:
            begin
                INST_MEM_READ = 0;
                INST_MEM_ADDRESS = 6'bx;
                // Update the cache with new fetched block with #1 time delay
                #1
                INSTRUCTION_CACHE[INDEX] = INST_MEM_READDATA;
                InstTagsArray[INDEX] = PC[9:7];
                InstValidArray[INDEX] = 1;
                INSTCACHE_BUSYWAIT = 0;
            end

        endcase
    end

    // State transitioning
    always@ (posedge clock, reset) begin
        if(reset)
            STATE = IDLE;
        else
            STATE = NEXT_STATE;
    end


    // Reset the instruction cache when reset is high
    integer i;
    always @(posedge reset) begin
        if(reset) begin
            for(i = 0; i < 8; i = i + 1) begin
                INSTRUCTION_CACHE[i] = 0;
                InstTagsArray[i] = 0;
                InstValidArray[i] = 0;
            end
        end
    end

endmodule

/* Primitive instruction memory module for CO224 Lab 6 - Part 3
This memory allows instructions to be read as 16-Byte blocks */

module instruction_memory(clock, read, address, readinst, busywait);
    input				clock;
    input				read;
    input[5:0]			address;
    output reg [127:0]	readinst;
    output	reg			busywait;

    reg readaccess;

    //Declare memory array 1024x8-bits 
    reg [7:0] memory_array [1023:0];

    //Initialize instruction memory
    initial
    begin
        busywait = 0;
        readaccess = 0;

        // Sample program given below. You may hardcode your software program here, or load it from a file:
    {memory_array[10'd3],  memory_array[10'd2],  memory_array[10'd1],  memory_array[10'd0]}  = 32'b00000000000001000000000000011001; // loadi 4 #25
    {memory_array[10'd7],  memory_array[10'd6],  memory_array[10'd5],  memory_array[10'd4]}  = 32'b00000000000001010000000000100011; // loadi 5 #35
    {memory_array[10'd11], memory_array[10'd10], memory_array[10'd9],  memory_array[10'd8]}  = 32'b00000010000001100000010000000101; // add 6 4 5
    {memory_array[10'd15], memory_array[10'd14], memory_array[10'd13], memory_array[10'd12]} = 32'b00000000000000010000000001011010; // loadi 1 #90
    {memory_array[10'd19], memory_array[10'd18], memory_array[10'd17], memory_array[10'd16]} = 32'b00000011000000010000000100000100; // sub 1 1 4
    {memory_array[10'd23], memory_array[10'd22], memory_array[10'd21], memory_array[10'd20]} = 32'b00000100000000100000010000000101; // and 2 4 5
    end

    //Detecting an incoming memory access
    always @(read)
    begin
        busywait = (read)? 1 : 0;
        readaccess = (read)? 1 : 0;
    end

    //Reading
    always @(posedge clock)
    begin
        if(readaccess)
        begin
            readinst[7:0]     = #40 memory_array[{address,4'b0000}];
            readinst[15:8]    = #40 memory_array[{address,4'b0001}];
            readinst[23:16]   = #40 memory_array[{address,4'b0010}];
            readinst[31:24]   = #40 memory_array[{address,4'b0011}];
            readinst[39:32]   = #40 memory_array[{address,4'b0100}];
            readinst[47:40]   = #40 memory_array[{address,4'b0101}];
            readinst[55:48]   = #40 memory_array[{address,4'b0110}];
            readinst[63:56]   = #40 memory_array[{address,4'b0111}];
            readinst[71:64]   = #40 memory_array[{address,4'b1000}];
            readinst[79:72]   = #40 memory_array[{address,4'b1001}];
            readinst[87:80]   = #40 memory_array[{address,4'b1010}];
            readinst[95:88]   = #40 memory_array[{address,4'b1011}];
            readinst[103:96]  = #40 memory_array[{address,4'b1100}];
            readinst[111:104] = #40 memory_array[{address,4'b1101}];
            readinst[119:112] = #40 memory_array[{address,4'b1110}];
            readinst[127:120] = #40 memory_array[{address,4'b1111}];
            busywait = 0;
            readaccess = 0;
        end
    end
    
endmodule
