// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

`timescale 1 ns / 1 ps

module testbench;

    parameter enable_vec = 1;
	integer i;
	reg clk = 1;
	reg resetn = 0;
	wire trap;

	always #5 clk = ~clk;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("testbench.vcd");
			$dumpvars(0, testbench);
		end
		repeat (1) @(posedge clk);
		resetn <= 1;
		repeat (1500) @(posedge clk);
		$finish;
	end
	integer ix;
	wire mem_valid;
	wire mem_instr;
	reg  mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	reg  [31:0] mem_rdata;
	wire mem_delayed_ready;
	wire  [31:0] mem_delayed_rdata;

    //For vector coprocessor
    wire  vec_mem_valid;
    reg  vec_mem_ready;
	wire [31:0] vec_mem_addr;
	wire [31:0] vec_mem_wdata;
	wire [3:0]  vec_mem_wstrb;
	reg  [31:0] vec_mem_rdata;
	
    
    // For vector instructions
	wire	 pcpi_vec_valid; //Valid for vector co-processor
	wire	[31:0] pcpi_vec_insn;  //insn to be sent to vector co-processor
	wire    [31:0] pcpi_vec_rs1; //Value stored in cpu rs1 transferred to pcpi core
	wire 	[31:0] pcpi_vec_rs2; //Only used by vselvl instrn
	wire  	[31:0] pcpi_vec_rd; //The output of pcpi_co-processor
	wire 		   pcpi_vec_wait;
	wire 		   pcpi_vec_ready; //Flag to notify if the instruction is executed or not
	wire		   pcpi_vec_wr;	 //Flag to notify the main processor to write to cpu reg
	

	picorv32 #(
        .ENABLE_VEC(enable_vec)
	) uut (
		.clk         (clk        ),
		.resetn      (resetn     ),
		.trap        (trap       ),
		.mem_valid   (mem_valid  ),
		.mem_instr   (mem_instr  ),
		.mem_ready   (mem_ready  ),
		.mem_addr    (mem_addr   ),
		.mem_wdata   (mem_wdata  ),
		.mem_wstrb   (mem_wstrb  ),
		.mem_rdata   (mem_rdata  ),

        //For vector coprocessor
        .pcpi_vec_valid(pcpi_vec_valid),
        .pcpi_vec_insn(pcpi_vec_insn),
        .pcpi_vec_rs1(pcpi_vec_rs1),
        .pcpi_vec_rs2(pcpi_vec_rs2),
        .pcpi_vec_rd(pcpi_vec_rd),
        .pcpi_vec_wait(pcpi_vec_wait),
        .pcpi_vec_ready(pcpi_vec_ready),
        .pcpi_vec_wr(pcpi_vec_wr)
	);

    // For vector instructions
	generate if (enable_vec) begin
		picorv32_pcpi_vec #(
			
		) pcpi_vec(
			.clk(clk),
			.resetn(resetn),
			.pcpi_valid(pcpi_vec_valid),
			.pcpi_insn(pcpi_vec_insn),
			.pcpi_cpurs1(pcpi_vec_rs1),
			.pcpi_cpurs2(pcpi_vec_rs2),
			.pcpi_wr(pcpi_vec_wr),
			.pcpi_rd(pcpi_vec_rd),
			.pcpi_wait(pcpi_vec_wait),
			.pcpi_ready(pcpi_vec_ready), //Becomes 1 if the output of co-processor is ready
            //Memory interface
            .mem_valid(vec_mem_valid),
            .mem_ready(vec_mem_ready),
            .mem_addr(vec_mem_addr),
            .mem_wdata(vec_mem_wdata),
            .mem_wstrb(vec_mem_wstrb),
            .mem_rdata(vec_mem_rdata)
		);
	end else begin
		assign pcpi_vec_wr = 0;
		assign pcpi_vec_rd = 32'bx;
		assign pcpi_vec_wait = 0;
		assign pcpi_vec_ready = 0;
	end endgenerate



	reg [31:0] memory [0:255];

	initial begin
		for(i = 0;i <256; i=i+1)
			memory[i] = 32'h 00000093;//NOP

        //Vl is the number of elements to modify every time
        memory[0] = 32'h 00400113; //---> to set vap as 4 (Addi x2,x0,4) 
        memory[1] = 32'h 00100093; //---> to set elem_off as 1 (Addi x1,x0,1)  000000000001 00000 000 00001 0010011
        // funct  rs2(off) vap(reg)  fun        opcode
        //1000000 00001    00010 111 00000 1011011
        memory[2] = 32'b 10000000000100010111000001011011; //Setting the value of vap as 4
		memory[3] = 32'h 00300113; //---> to set vl as 3  (Addi x2,x0,3)
		memory[4] = 32'b 00000000000000010111001001010111; //Vsetvli x4,x2, LMUL=1 E8 --->  0 00000000000 00010 111 00100 1010111 ---> 00817257 (sew - 8)
		memory[5] = 32'h 19000093; //li x1,400(x0)   --> addi
        memory[6] = 32'h 00300393; //li x7,3(x0) --> addi (loading stride)
        //31 30 29 26 25  24 	 20 19  15 14 12 11    7 6     0
		// 00 | 0000 | vm | 00000 |  rs1 | width |  vd  |1011011| vleu_varp
		// 00 | 0001 | vm |  rs2  |  rs1 | width |  vd  |1011011| vles_varp
        memory[7] = 32'b 00000110011100001111000011011011; //  00 0001 1 00111 00001 111 00001 1011011  vles_varp.v v1, (x1), x7
        memory[8] = 32'h 00000413; //li x8,0(x0) --> stride for second matrix
		memory[9] = 32'h 19100193; //li x3,401(x0) --> addi
        memory[10] = 32'h 19200293; //li x5,402(x0) --> addi
        memory[11] = 32'h 1b800313; //li x6,440(x0) --> addi
		memory[12] = 32'h 1b400213; //li x4,436(x0) --> addi (for loading 0 into Vd)
        memory[13] = 32'b 00000110011100011111000101011011; //  00 0001 1 00111 00011 111 00010 1011011  vles_varp.v v2, (x3), x7
        memory[14] = 32'b 00000110011100101111000111011011; //  00 0001 1 00111 00101 111 00011 1011011  vles_varp.v v3, (x5), x7
        memory[15] = 32'b 00000110100000110111001001011011; //  00 0001 1 01000 00110 111 00100 1011011  vles_varp.v v4, (x6), x8 (rs8 --> stride)
		//Loading 0 into Vd initially
        memory[16] = 32'b 00000110100000100111010001011011; //  00 0001 1 01000 00100 111 01000 1011011  vles_varp.v v8, x4, x8 (rs8 --> stride)
		memory[17] = 32'b 11000110010000001000010001011011; //  11 00011 00100 00001 000 01000 1011011  vdotvarp.vv v8,v4,v1 
		memory[18] = 32'h 1bb00313; //li x6,443(x0) --> addi
        memory[19] = 32'b 00000110100000110111001001011011; //  00 0001 1 01000 00110 111 00100 1011011  vles_varp.v v4, (x6), x8 (rs8 --> stride)
		memory[20] = 32'b 11000110010000010000010001011011; //  11 00011 00100 00010 000 01000 1011011  vdotvarp.vv v8,v4,v2 
		memory[21] = 32'h 1be00313; //li x6,446(x0) --> addi
        memory[22] = 32'b 00000110100000110111001001011011; //  00 0001 1 01000 00110 111 00100 1011011  vles_varp.v v4, (x6), x8 (rs8 --> stride)
		memory[23] = 32'b 11000110010000011000010001011011; //  11 00011 00100 00011 000 01000 1011011  vdotvarp.vv v8,v4,v3 
		
		// // //second column, loading 0 into Vd (v9) initially
		memory[24] = 32'b 00000110100000100111010011011011; //  00 0001 1 01000 00100 111 01001 1011011  vles_varp.v v9, x4, x8 (rs8 --> stride)
		memory[22] = 32'h 1b900313; //li x6,441(x0) --> addi
		memory[24] = 32'b 00000110100000110111001001011011; //  00 0001 1 01000 00110 111 00100 1011011  vles_varp.v v4, (x6), x8
		memory[23] = 32'b 11000110010000001000010011011011; //  11 00011 00100 00001 000 01001 1011011  vdotvarp.vv v9,v4,v1
		memory[25] = 32'h 1bc00313; //li x6,444(x0) --> addi
		memory[24] = 32'b 00000110100000110111001001011011; //  00 0001 1 01000 00110 111 00100 1011011  vles_varp.v v4, (x6), x8
		memory[23] = 32'b 11000110010000010000010011011011; //  11 00011 00100 00010 000 01001 1011011  vdotvarp.vv v9,v4,v2
		memory[28] = 32'h 1bf00313; //li x6,447(x0) --> addi
		memory[24] = 32'b 00000110100000110111001001011011; //  00 0001 1 01000 00110 111 00100 1011011  vles_varp.v v4, (x6), x8
		memory[23] = 32'b 11000110010000011000010011011011; //  11 00011 00100 00011 000 01001 1011011  vdotvarp.vv v9,v4,v3
		
		// // //third column, loading 0 into Vd (v10) initially
		memory[31] = 32'b 00000110100000100111010101011011; //  00 0001 1 01000 00100 111 01010 1011011  vles_varp.v v10, x4, x8 (rs8 --> stride)
		memory[32] = 32'h 1ba00313; //li x6,442(x0) --> addi
		memory[24] = 32'b 00000110100000110111001001011011; //  00 0001 1 01000 00110 111 00100 1011011  vles_varp.v v4, (x6), x8
		memory[23] = 32'b 11000110010000001000010101011011; //  11 00011 00100 00001 000 01010 1011011  vdotvarp.vv v10,v4,v1
		memory[35] = 32'h 1bd00313; //li x6,445(x0) --> addi
		memory[24] = 32'b 00000110100000110111001001011011; //  00 0001 1 01000 00110 111 00100 1011011  vles_varp.v v4, (x6), x8
		memory[23] = 32'b 11000110010000010000010101011011; //  11 00011 00100 00010 000 01010 1011011  vdotvarp.vv v10,v4,v2
		memory[38] = 32'h 1c000313; //li x6,448(x0) --> addi
		memory[24] = 32'b 00000110100000110111001001011011; //  00 0001 1 01000 00110 111 00100 1011011  vles_varp.v v4, (x6), x8
		memory[23] = 32'b 11000110010000011000010101011011; //  11 00011 00100 00011 000 01010 1011011  vdotvarp.vv v10,v4,v3
		

		/*
        Matrix is stored in memory from addr 100 - addr 108
        ex: [1 2 3; 
			 4 5 6; 
			 7 8 9]
            [1 2 3
             4 5 6
             7 8 9] 
        mem[100] = 1;
        mem[101] = 2; etc
		*/
        //First matrix
		memory[100] = 32'h 04030201;
		memory[101] = 32'h 08070605;
        memory[102] = 32'h 0c0b0a09;
		memory[103] = 32'h 00080007;
        memory[104] = 32'h 000a0009;
		memory[105] = 32'h 00000008;
        memory[106] = 32'h 00000009;
		memory[107] = 32'h 00000008;
        memory[108] = 32'h 00000009; 
		memory[109] = 32'h 00000000;
		//Contains the second matrix
		memory[110] = 32'h 04030201;
        memory[111] = 32'h 08070605;
		memory[112] = 32'h 00500032;
		memory[113] = 32'h 003c001e;
        memory[114] = 32'h 0000005a;
		memory[115] = 32'h 0000003c;
		memory[116] = 32'h 00000046;
        memory[117] = 32'h 00000050;
		memory[118] = 32'h 0000005a;


		// for(ix=0;ix<4;ix=ix+1)begin
		// 	memory[200+4*ix]   = 32'h 00000001;
		// 	memory[200+4*ix+1] = 32'h 00000002;
		// 	memory[200+4*ix+2] = 32'h 00000003;
		// 	memory[200+4*ix+3] = 32'h 00000004;
		// 	memory[239+4*ix]   = 32'h 00000005;
		// 	memory[239+4*ix+1] = 32'h 00000006;
		// 	memory[239+4*ix+2] = 32'h 00000007;
		// 	memory[239+4*ix+3] = 32'h 00000008;
		// end

		//Vttpe reg is 00000000000, vtype[1:0] -> vlmul[1:0] (sets LMUL value)
		//							vtype[4:2] -> vsew[2:0] (sets SEW value)
		//							vtype[6:5] -> vdiv[1:0] (used by EDIV extension)
		//							vlen gets it's value from 00010 reg i.e it gets 16
	end

	always @(posedge clk) begin
		mem_ready <= 0;
		if (mem_valid && !mem_ready) begin
			if (mem_addr < 1024) begin
				mem_ready <= 1;
				mem_rdata <= memory[mem_addr >> 2];
				// $display("Time:%d ,Data read from memory: %x, addr: %d", $time,mem_rdata, mem_addr);
				if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
				if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
				if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
				if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
			end
			/* add memory-mapped IO here */
		end
	end


	always @(posedge clk) begin
		vec_mem_ready <= 0;
		if (vec_mem_valid && !vec_mem_ready) begin
			// $display("mem_addr: %d, mem_data: %x,mem_ready:%d, time:%d",vec_mem_addr, vec_mem_rdata,vec_mem_ready, $time);
			if (vec_mem_addr < 1024) begin
				vec_mem_rdata <= memory[vec_mem_addr >> 2];
				vec_mem_ready <= 1;
				// $display("Time:%d ,Data read from memory: %x, addr: %d", $time,vec_mem_rdata, vec_mem_addr);
				if (vec_mem_wstrb[0]) memory[vec_mem_addr >> 2][ 7: 0] <= vec_mem_wdata[ 7: 0];
				if (vec_mem_wstrb[1]) memory[vec_mem_addr >> 2][15: 8] <= vec_mem_wdata[15: 8];
				if (vec_mem_wstrb[2]) memory[vec_mem_addr >> 2][23:16] <= vec_mem_wdata[23:16];
				if (vec_mem_wstrb[3]) memory[vec_mem_addr >> 2][31:24] <= vec_mem_wdata[31:24];
			end
			/* add memory-mapped IO here */
		end
	end
endmodule


// li x1, 0x3BC
// li x3, 0x320
// li x2, 0x020
// vsetvli x4, x2, e8, m1
// vle.v v0, x1
// vle.v v1, x3
// vmult.vv v2, v0, v1
// vse.v v2, x1
// nop
// nop