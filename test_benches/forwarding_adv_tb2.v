//This tb is for percentages of 10, 20, 30

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
		repeat (15000) @(posedge clk);
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
			memory[i] = 32'h 00000093; //NOP
		
		//Vl is the number of elements to modify every time
		memory[0] = 32'h 00800113; //---> (Addi x2,x0,8) to set vl(no of elements) as 8 
		memory[1] = 32'b 00000000000000000000100010010011;  //addi x17, x0, 000000000000b
		// Ox 00017257
		memory[2] = 32'b 00000000100000010111001001010111; //Vsetvli x4,x2, LMUL=1 E32 --->  0 00000001000 00010 111 00100 1010111 ---> 00817257 (sew - 32)
		memory[3] = 32'h 19000093; //addi x1,x0,400
        memory[4] = 32'h 00400393; //addi x7,x0,4  --> Loading the stride
		
        memory[5] = 32'b 10010110100100000000010100010011;  //addi x10, x0, 100101101001b
		memory[6] = 32'b 00000000000000000000010110010011;  //addi x11, x0, 000000000000b
		memory[7] = 32'b 00000000001100000000011000010011;  //addi x12, x0, 000000000011b
		memory[8] = 32'b 00111110100000000000011010010011;  //addi x13, x0, 1000d {store address}
		// memory[12] = 32'b 00000000001101010111011110010011;  //andi x15, x10, 000000000011b    --> x15 contains 000000000001
		// memory[13] = 32'b 00000000111101101010000000100011;  //sw x13, 0(x15)
		// memory[14] = 32'b 00000000001101010111011110010011;  //andi x15, x10, 000000000011b    --> x15 contains 000000000001
		// memory[15] = 32'b 00000000111101101010000000100011;  //sw x13, 0(x15)

        //31 29 28 26 25 24    20 19  15 14 12 11    7 6     0
		// nf | mop | vm |  rs2 |  rs1 | width |  vd  |0000111| VL* strided
        memory[9] = 32'b 00001010011100001111000010000111; //  000 010 1 00111 00001 111 00001 0000111 vlse.v v1, (x1), x7

		memory[10] = 32'h 19000093; //addi x1,x0,400
		memory[11] = 32'b 00001010011100001111000100000111; //  000 010 1 00111 00001 111 00010 0000111 vlse.v v2, (x1), x7

		memory[12] = 32'h 1E400093; //addi x1,x0,484   ---> Loading 00000093 repetetively into v8
		memory[13] = 32'b 00001010011100001111010000000111; //  000 010 1 00111 00001 111 01000 0000111 vlse.v v8, (x1), x7
		memory[14] = 32'b 11100110001000001000010001010111; //  111001 1 00010 00001 000 01000 1010111  vdot.vv v8, v2, v1
		// Strided store instruction with the same stride in x7 i.e 8
		memory[15] = 32'h 32000093; //addi x1,x0,800
		memory[16] = 32'b 00001010011100001111010000100111; //  000 010 1 00111 00001 111 01000 0100111 vsse.v v8, (x1), x7

		memory[17] = 32'b 00000001010000000000100000010011;  //addi x16, x0, 000000001010b
		memory[18] = 32'b 00000000000110001000100010010011;  //addi x17, x17, 000000000001b
		memory[19] = 32'b 11111011000010001110111011100011;  //BLTU  1 111101 10000 10001 110 1110 1 1100011

		memory[100] = 32'h 00000201;
		memory[101] = 32'h 00000605;
        memory[102] = 32'h 00000a09;
		memory[103] = 32'h 00000e0d;
        memory[104] = 32'h 14131211;
		memory[105] = 32'h 18171615;
        memory[106] = 32'h 1c1b1a19;
		memory[107] = 32'h 101f1e1d;
        memory[108] = 32'h 24232221; 
		memory[109] = 32'h 28272625;
	
		memory[110] = 32'h 0000000a;
        memory[111] = 32'h 00000014;
		memory[112] = 32'h 0000001e;
		memory[113] = 32'h 00000028;
        memory[114] = 32'h 00000032;
		memory[115] = 32'h 0000003c;
		memory[116] = 32'h 00000046;
        memory[117] = 32'h 00000050;
		memory[118] = 32'h 0000005a;
		memory[119] = 32'h 1000000a;
        memory[120] = 32'h 11000014;
		
		memory[121] = 32'h 1200000a;
        memory[122] = 32'h 13000014;
		memory[123] = 32'h 1400001e;
		memory[124] = 32'h 15000028;
        memory[125] = 32'h 16000032;
		memory[126] = 32'h 1700003c;
		memory[127] = 32'h 18000046;
        memory[128] = 32'h 19000050;
		memory[129] = 32'h 2000005a;
		memory[130] = 32'h 21000050;
		memory[131] = 32'h 2200005a;

		//Vtype reg is 00000000000, vtype[1:0] -> vlmul[1:0] (sets LMUL value)
		//							vtype[4:2] -> vsew[2:0] (sets SEW value)
		//							vtype[6:5] -> vdiv[1:0] (used by EDIV extension)
		//							vl gets it's value from 00010 reg i.e it gets 16
	end

	always @(posedge clk) begin
		mem_ready <= 0;
		if (mem_valid && !mem_ready) begin
			if (mem_addr < 1024) begin
				mem_ready <= 1;
				mem_rdata <= memory[mem_addr >> 2];
				// $display("Time:%d ,Data read from memory: %x, addr: %d", $time,mem_rdata, mem_addr);
				if(mem_wstrb != 4'b0)
					$display("Data written to memory addr: %d is %b, mem_wstrb: %b, time:%d", mem_addr, mem_wdata, mem_wstrb, $time);
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
			if (vec_mem_addr < 1024) begin
				vec_mem_rdata <= memory[vec_mem_addr >> 2];
				vec_mem_ready <= 1;
				// if(vec_mem_wstrb == 4'b0)
					// $display("mem_addr: %d, mem_data: %x,mem_ready:%d, time:%d",vec_mem_addr, vec_mem_rdata,vec_mem_ready, $time);
				if(vec_mem_wstrb != 4'b0)
					$display("Data written to memory addr: %d is %x, mem_wstrb: %b, time:%d", vec_mem_addr, vec_mem_wdata, vec_mem_wstrb, $time);
				if (vec_mem_wstrb[0]) memory[vec_mem_addr >> 2][ 7: 0] <= vec_mem_wdata[ 7: 0];
				if (vec_mem_wstrb[1]) memory[vec_mem_addr >> 2][15: 8] <= vec_mem_wdata[15: 8];
				if (vec_mem_wstrb[2]) memory[vec_mem_addr >> 2][23:16] <= vec_mem_wdata[23:16];
				if (vec_mem_wstrb[3]) memory[vec_mem_addr >> 2][31:24] <= vec_mem_wdata[31:24];
			end
			/* add memory-mapped IO here */
		end
	end
endmodule
