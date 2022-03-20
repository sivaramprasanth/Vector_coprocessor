
/***************************************************************
 * picorv32_pcpi_vec: A PCPI core that implements the vector instructions
 ***************************************************************/
module picorv32_pcpi_vec #(
	parameter [31:0] vlen = 32'h00000200 //No of bits in vector 
)(
	input clk, resetn,
	input pcpi_valid,
	input       [31:0]  pcpi_insn,
	input       [31:0]  pcpi_cpurs1, //Value in cpu_regs, used by vsetvl, vsetvli, vload, vstore
	input 		[31:0]  pcpi_cpurs2, //Value in cpu_regs, only used by vsetvl instrn
    output reg          pcpi_wr,
	output reg  [31:0]  pcpi_rd,
	output reg          pcpi_wait,
	output reg 	        pcpi_ready,

	//Memory interface
	input mem_ready, //Given by memory 
	input [31:0] mem_rdata, //data from memory for vload
	output reg mem_valid, //Assigned by coprocessor
	output reg [31:0] mem_addr, //Given to memory by coprocessor
	output reg [31:0] mem_wdata, //For store
	output reg [3:0]  mem_wstrb  //For store
);

	localparam [10:0] flat_reg_len = 512;
	localparam [4:0] no_alus = 16;

	//Used for vector arithmetic instructions
	localparam p_instr_vadd__vv = 8'h00 ;
	localparam p_instr_vmul__vv = 8'h01 ;
	localparam p_instr_vdot__vv = 8'h02 ;
	localparam p_instr_vaddvarp = 8'h03 ;
	localparam p_instr_vmulvarp = 8'h04 ;
	localparam p_instr_vdotvarp = 8'h05 ;

	localparam [10:0] alu_reg_len = 512;  //Alu register length used for alu operations
	reg [31:0] reg_op1; //stores the value of pcpi_cpurs1
	reg [31:0] reg_op2; //stores the value of pcpi_cpurs2

	//Instruction decoder variables
	reg instr_vsetvli, instr_vsetvl, instr_vsetprecision; //Vec instrn to set the csr reg values
	reg instr_vload,instr_vload_str,instr_vstore, instr_vstore_str;   //Vec load and store instr
	reg instr_vdot,instr_vadd, instr_vmul, port3_data, temp_var3; //For dot product and addition
	wire is_vec_instr, is_vap_instr, is_port3_instr; ////To check whether the forwarded instruction to coprocessor is vector instruction or not

	//Instructions for variable bit precision {Currently, doesn't support subtraction}
	reg instr_vleuvarp, instr_vlesvarp, instr_vseuvarp, instr_vsesvarp, instr_vaddvarp, instr_vsubvarp, instr_vmulvarp, instr_vdotvarp;



	//Memory Interface
	reg [1:0] mem_wordsize; //To tell whether to read/write whole word or a part of the word
	reg [31:0] mem_rdata_word; // //Stores the data depending on mem_wordsize from mem_rdata 

	reg mem_do_rdata; //Flag to read data
	reg mem_do_wdata; //Flag to write data
	reg mem_str_ready, mem_str_ready2;  //Used as the ready signal for strided load instruction

//memory interface 
always @(posedge clk) begin
    // (* full_case *)
    if(pcpi_valid) begin
        // $display("mem_rdata:%x, time:%d",mem_rdata,$time);
        case (mem_wordsize)
            0: begin
                mem_str_ready2 <= 0;
                if(mem_ready == 1) begin
                    // $display("Inside mem_interface, mem_rdata:%x, time:%d", mem_rdata,$time);
                    if(instr_vload || instr_vload_str || instr_vleuvarp || instr_vlesvarp) begin
                        mem_rdata_word <= mem_rdata; //reads 32 bits
                    end
                    mem_str_ready2 <= 1; //str_ready will be 1 irrespective of the instruction
                end
            end
            1: begin
                mem_str_ready <= 0;
                if(instr_vload || instr_vload_str) begin
                    if(SEW == 10'b0000001000) begin
                        // $display("Inside mem_wsize1,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
                        mem_rdata_word[7:0]   <= ld_data[ind1 +: 8];
                        mem_rdata_word[15:8]  <= ld_data[(ind1+reg_op2*8) +: 8];
                        mem_rdata_word[23:16]  <= ld_data[(ind1+reg_op2*16) +: 8];
                        mem_rdata_word[31:24] <= ld_data[(ind1+reg_op2*24) +: 8];
                        mem_str_ready <= 1; 
                        ind1 <= ind1 + 4*8*reg_op2;
                    end
                    else if(SEW == 10'b0000010000) begin
                        // $display("Inside mem_wsize1,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
                        mem_rdata_word[15:0]   <= ld_data[ind1 +: 16];
                        mem_rdata_word[31:16]  <= ld_data[(ind1+reg_op2*8) +: 16];
                        mem_str_ready <= 1; 
                        ind1 <= ind1 + 2*8*reg_op2;
                    end
                    //SEW is 32
                    else if(SEW == 10'b0000100000) begin
                        mem_rdata_word <= ld_data[ind1 +: 32];
                        mem_str_ready <= 1; 
                        ind1 <= ind1 + 8*reg_op2;
                    end
                end
                else if(instr_vleuvarp || instr_vlesvarp) begin
                    if(vap == 10'b0000000001) begin
                        //The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
                        mem_rdata_word[0]   <= ld_data[ind1 +: 1];
                        mem_rdata_word[1]   <= ld_data[(ind1+reg_op2*8) +: 1];
                        mem_rdata_word[2]   <= ld_data[(ind1+reg_op2*16) +: 1];
                        mem_rdata_word[3]   <= ld_data[ind1+reg_op2*24 +: 1];
                        mem_rdata_word[4]   <= ld_data[(ind1+reg_op2*32) +: 1];
                        mem_rdata_word[5] <= ld_data[(ind1+reg_op2*40) +: 1];
                        mem_rdata_word[6] <= ld_data[(ind1+reg_op2*48) +: 1];
                        mem_rdata_word[7] <= ld_data[(ind1+reg_op2*56) +: 1];
                        mem_rdata_word[8] <= ld_data[ind1+reg_op2*8*8 +: 1];
                        mem_rdata_word[9] <= ld_data[(ind1+reg_op2*9*8) +: 1];
                        mem_rdata_word[10] <= ld_data[(ind1+reg_op2*10*8) +: 1];
                        mem_rdata_word[11] <= ld_data[ind1+reg_op2*11*8 +: 1];
                        mem_rdata_word[12] <= ld_data[(ind1+reg_op2*12*8) +: 1];
                        mem_rdata_word[13] <= ld_data[(ind1+reg_op2*13*8) +: 1];
                        mem_rdata_word[14] <= ld_data[(ind1+reg_op2*14*8) +: 1];
                        mem_rdata_word[15] <= ld_data[(ind1+reg_op2*15*8) +: 1];
                        mem_rdata_word[16]   <= ld_data[(ind1 + reg_op2*8*16) +: 1];
                        mem_rdata_word[17]   <= ld_data[(ind1+reg_op2*17*8) +: 1];
                        mem_rdata_word[18]   <= ld_data[(ind1+reg_op2*18*8) +: 1];
                        mem_rdata_word[19]   <= ld_data[ind1+reg_op2*19*8 +: 1];
                        mem_rdata_word[20]   <= ld_data[(ind1+reg_op2*20*8) +: 1];
                        mem_rdata_word[21] <= ld_data[(ind1+reg_op2*21*8) +: 1];
                        mem_rdata_word[22] <= ld_data[(ind1+reg_op2*22*8) +: 1];
                        mem_rdata_word[23] <= ld_data[(ind1+reg_op2*23*8) +: 1];
                        mem_rdata_word[24] <= ld_data[ind1+reg_op2*24*8 +: 1];
                        mem_rdata_word[25] <= ld_data[(ind1+reg_op2*25*8) +: 1];
                        mem_rdata_word[26] <= ld_data[(ind1+reg_op2*26*8) +: 1];
                        mem_rdata_word[27] <= ld_data[ind1+reg_op2*27*8 +: 1];
                        mem_rdata_word[28] <= ld_data[(ind1+reg_op2*28*8) +: 1];
                        mem_rdata_word[29] <= ld_data[(ind1+reg_op2*29*8) +: 1];
                        mem_rdata_word[30] <= ld_data[(ind1+reg_op2*30*8) +: 1];
                        mem_rdata_word[31] <= ld_data[(ind1+reg_op2*31*8) +: 1];
                        mem_str_ready <= 1; 
                        ind1 <= ind1 + 32*8*reg_op2;
                    end
                    else if(vap == 10'b0000000010) begin
                        //The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
                        mem_rdata_word[1:0]   <= ld_data[ind1 +: 2];
                        mem_rdata_word[3:2]   <= ld_data[(ind1+reg_op2*8) +: 2];
                        mem_rdata_word[5:4]   <= ld_data[(ind1+reg_op2*16) +: 2];
                        mem_rdata_word[7:6]   <= ld_data[ind1+reg_op2*24 +: 2];
                        mem_rdata_word[9:8]   <= ld_data[(ind1+reg_op2*32) +: 2];
                        mem_rdata_word[11:10] <= ld_data[(ind1+reg_op2*40) +: 2];
                        mem_rdata_word[13:12] <= ld_data[(ind1+reg_op2*48) +: 2];
                        mem_rdata_word[15:14] <= ld_data[(ind1+reg_op2*56) +: 2];
                        mem_rdata_word[17:16] <= ld_data[ind1+reg_op2*8*8 +: 2];
                        mem_rdata_word[19:18] <= ld_data[(ind1+reg_op2*9*8) +: 2];
                        mem_rdata_word[21:20] <= ld_data[(ind1+reg_op2*10*8) +: 2];
                        mem_rdata_word[23:22] <= ld_data[ind1+reg_op2*11*8 +: 2];
                        mem_rdata_word[25:24] <= ld_data[(ind1+reg_op2*12*8) +: 2];
                        mem_rdata_word[27:26] <= ld_data[(ind1+reg_op2*13*8) +: 2];
                        mem_rdata_word[29:28] <= ld_data[(ind1+reg_op2*14*8) +: 2];
                        mem_rdata_word[31:30] <= ld_data[(ind1+reg_op2*15*8) +: 2];
                        mem_str_ready <= 1; 
                        ind1 <= ind1 + 16*8*reg_op2;
                    end
                    else if(vap == 10'b0000000011) begin
                        //The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
                        mem_rdata_word[2:0]   <= ld_data[ind1 +: 3];
                        mem_rdata_word[5:3]   <= ld_data[(ind1+reg_op2*8) +: 3];
                        mem_rdata_word[8:6]   <= ld_data[(ind1+reg_op2*16) +: 3];
                        mem_rdata_word[11:9]   <= ld_data[ind1+reg_op2*24 +: 3];
                        mem_rdata_word[14:12]   <= ld_data[(ind1+reg_op2*32) +: 3];
                        mem_rdata_word[17:15] <= ld_data[(ind1+reg_op2*40) +: 3];
                        mem_rdata_word[20:18] <= ld_data[(ind1+reg_op2*48) +: 3];
                        mem_rdata_word[23:21] <= ld_data[(ind1+reg_op2*56) +: 3];
                        mem_rdata_word[26:24] <= ld_data[ind1+reg_op2*8*8 +: 3];
                        mem_rdata_word[29:27] <= ld_data[(ind1+reg_op2*9*8) +: 3];
                        mem_rdata_word[31:30] <= ld_data[(ind1+reg_op2*10*8) +: 2];
                        mem_str_ready <= 1; 
                        ind1 <= ind1 + 10*8*reg_op2 + 2;
                    end
                    else if(vap == 10'b0000000100) begin
                        // $display("Inside mem_wsize2,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
                        //The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
                        mem_rdata_word[3:0]   <= ld_data[ind1 +: 4];
                        mem_rdata_word[7:4]   <= ld_data[(ind1+reg_op2*8) +: 4];
                        mem_rdata_word[11:8]  <= ld_data[(ind1+reg_op2*16) +: 4];
                        mem_rdata_word[15:12] <= ld_data[ind1+reg_op2*24 +: 4];
                        mem_rdata_word[19:16] <= ld_data[(ind1+reg_op2*32) +: 4];
                        mem_rdata_word[23:20] <= ld_data[(ind1+reg_op2*40) +: 4];
                        mem_rdata_word[27:24] <= ld_data[(ind1+reg_op2*48) +: 4];
                        mem_rdata_word[31:28] <= ld_data[(ind1+reg_op2*56) +: 4];
                        mem_str_ready <= 1; 
                        ind1 <= ind1 + 8*8*reg_op2;
                    end
                    if(vap == 10'b0000001000) begin
                        // $display("Inside mem_wsize1,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
                        mem_rdata_word[7:0]   <= ld_data[ind1 +: 8];
                        mem_rdata_word[15:8]  <= ld_data[(ind1+reg_op2*8) +: 8];
                        mem_rdata_word[23:16]  <= ld_data[(ind1+reg_op2*16) +: 8];
                        mem_rdata_word[31:24] <= ld_data[(ind1+reg_op2*24) +: 8];
                        mem_str_ready <= 1; 
                        ind1 <= ind1 + 4*8*reg_op2;
                    end
                    if(vap == 10'b0000001010) begin
                        // $display("Inside mem_wsize1,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
                        mem_rdata_word[9:0]   <= ld_data[ind1 +: 10];
                        mem_rdata_word[19:10]  <= ld_data[(ind1+reg_op2*8) +: 10];
                        mem_rdata_word[29:20]  <= ld_data[(ind1+reg_op2*16) +: 10];
                        mem_rdata_word[31:30] <= ld_data[(ind1+reg_op2*24) +: 2];
                        mem_str_ready <= 1; 
                        ind1 <= ind1 + 4*8*reg_op2;
                    end
                end
            end
            2:  begin
                if(condition_bit == 1) begin
                    mem_str_ready <= 0;
                    if(instr_vstore || instr_vstore_str) begin
                        vecregs_rstrb1 <= 1 << (temp_count2+1);
                        if(SEW == 10'b0000001000) begin
                            $display("vecreg_data: %x, cnt:%d, time:%d", vecregs_rdata1, cnt, $time);
                            st_data[cnt +: 8] <= vecregs_rdata1[7:0];
                            st_data[(cnt+reg_op2*8) +: 8] <= vecregs_rdata1[15:8];
                            st_data[(cnt+reg_op2*2*8) +: 8] <= vecregs_rdata1[23:16];
                            st_data[(cnt+reg_op2*3*8) +: 8] <= vecregs_rdata1[31:24];
                            st_strb[cnt >> 3] <= 1;
                            st_strb[(cnt >> 3) + reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 2*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 3*reg_op2] <= 1;
                            cnt <= cnt + 4*8*reg_op2;
                            mem_str_ready <= 1; 
                        end
                        //SEW is 16
                        else if(SEW == 10'b0000010000) begin
                            st_data[cnt +: 16] <= vecregs_rdata1[15:0];
                            st_data[(cnt+reg_op2*8) +: 16] <= vecregs_rdata1[31:16];
                            st_strb[(cnt >> 3) +: 2] <= 2'b11;
                            st_strb[((cnt >> 3) + reg_op2) +: 2] <= 2'b11;
                            cnt <= cnt + 2*8*reg_op2;
                            mem_str_ready <= 1; 
                        end
                        //SEW is 32
                        else if(SEW == 10'b0000100000) begin
							// $display("vecreg_data: %x, cnt:%d, st_data:%x, time:%d", vecregs_rdata1, cnt, st_data, $time);
                            st_data[cnt +: 32] <= vecregs_rdata1;
                            st_strb[(cnt >> 3) +: 4] <= 4'b1111;
                            cnt <= cnt + 8*reg_op2;
                            mem_str_ready <= 1; 
                        end
                        condition_bit <= 0;
                    end
                    else if(instr_vseuvarp || instr_vsesvarp) begin
						vecregs_rstrb1 <= 1 << (temp_count2+1);
                        //SEW is 1
                        if(vap == 10'b0000000001) begin
                            $display("vecreg_data: %x, cnt:%d, time:%d", vecregs_rdata1, cnt, $time);
                            st_data[cnt +: 1] <= vecregs_rdata1[0];
                            st_data[(cnt+reg_op2*8) +: 1] <= vecregs_rdata1[1];
                            st_data[(cnt+reg_op2*2*8) +: 1] <= vecregs_rdata1[2];
                            st_data[(cnt+reg_op2*3*8) +: 1] <= vecregs_rdata1[3];
                            st_data[(cnt+reg_op2*4*8) +: 1] <= vecregs_rdata1[4];
                            st_data[(cnt+reg_op2*5*8) +: 1] <= vecregs_rdata1[5];
                            st_data[(cnt+reg_op2*6*8) +: 1] <= vecregs_rdata1[6];
                            st_data[(cnt+reg_op2*7*8) +: 1] <= vecregs_rdata1[7];
                            st_data[(cnt+reg_op2*8*8) +: 1] <= vecregs_rdata1[8];
                            st_data[(cnt+reg_op2*9*8) +: 1] <= vecregs_rdata1[9];
                            st_data[(cnt+reg_op2*10*8) +: 1] <= vecregs_rdata1[10];
                            st_data[(cnt+reg_op2*11*8) +: 1] <= vecregs_rdata1[11];
                            st_data[(cnt+reg_op2*12*8) +: 1] <= vecregs_rdata1[12];
                            st_data[(cnt+reg_op2*13*8) +: 1] <= vecregs_rdata1[13];
                            st_data[(cnt+reg_op2*14*8) +: 1] <= vecregs_rdata1[14];
                            st_data[(cnt+reg_op2*15*8) +: 1] <= vecregs_rdata1[15];
                            st_data[(cnt+reg_op2*16*8) +: 1] <= vecregs_rdata1[16];
                            st_data[(cnt+reg_op2*17*8) +: 1] <= vecregs_rdata1[17];
                            st_data[(cnt+reg_op2*18*8) +: 1] <= vecregs_rdata1[18];
                            st_data[(cnt+reg_op2*19*8) +: 1] <= vecregs_rdata1[19];
                            st_data[(cnt+reg_op2*20*8) +: 1] <= vecregs_rdata1[20];
                            st_data[(cnt+reg_op2*21*8) +: 1] <= vecregs_rdata1[21];
                            st_data[(cnt+reg_op2*22*8) +: 1] <= vecregs_rdata1[22];
                            st_data[(cnt+reg_op2*23*8) +: 1] <= vecregs_rdata1[23];
                            st_data[(cnt+reg_op2*24*8) +: 1] <= vecregs_rdata1[24];
                            st_data[(cnt+reg_op2*25*8) +: 1] <= vecregs_rdata1[25];
                            st_data[(cnt+reg_op2*26*8) +: 1] <= vecregs_rdata1[26];
                            st_data[(cnt+reg_op2*27*8) +: 1] <= vecregs_rdata1[27];
                            st_data[(cnt+reg_op2*28*8) +: 1] <= vecregs_rdata1[28];
                            st_data[(cnt+reg_op2*29*8) +: 1] <= vecregs_rdata1[29];
                            st_data[(cnt+reg_op2*30*8) +: 1] <= vecregs_rdata1[30];
                            st_data[(cnt+reg_op2*31*8) +: 1] <= vecregs_rdata1[31];
                            st_strb[cnt >> 3] <= 1;
                            st_strb[(cnt >> 3) + reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 2*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 3*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 4*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 5*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 6*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 7*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 8*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 9*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 10*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 11*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 12*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 13*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 14*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 15*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 16*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 17*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 18*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 19*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 20*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 21*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 22*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 23*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 24*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 25*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 26*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 27*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 28*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 29*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 30*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 31*reg_op2] <= 1;
                            cnt <= cnt + 32*8*reg_op2;
                            mem_str_ready <= 1; 
                        end
                        if(vap == 10'b0000000010) begin
                            $display("vecreg_data: %x, cnt:%d, time:%d", vecregs_rdata1, cnt, $time);
                            st_data[cnt +: 2] <= vecregs_rdata1[1:0];
                            st_data[(cnt+reg_op2*8) +: 2] <= vecregs_rdata1[3:2];
                            st_data[(cnt+reg_op2*2*8) +: 2] <= vecregs_rdata1[5:4];
                            st_data[(cnt+reg_op2*3*8) +: 2] <= vecregs_rdata1[7:6];
                            st_data[(cnt+reg_op2*4*8) +: 2] <= vecregs_rdata1[9:8];
                            st_data[(cnt+reg_op2*5*8) +: 2] <= vecregs_rdata1[11:10];
                            st_data[(cnt+reg_op2*6*8) +: 2] <= vecregs_rdata1[13:12];
                            st_data[(cnt+reg_op2*7*8) +: 2] <= vecregs_rdata1[15:14];
                            st_data[(cnt+reg_op2*8*8) +: 2] <= vecregs_rdata1[17:16];
                            st_data[(cnt+reg_op2*9*8) +: 2] <= vecregs_rdata1[19:18];
                            st_data[(cnt+reg_op2*10*8) +: 2] <= vecregs_rdata1[21:20];
                            st_data[(cnt+reg_op2*11*8) +: 2] <= vecregs_rdata1[23:22];
                            st_data[(cnt+reg_op2*12*8) +: 2] <= vecregs_rdata1[25:24];
                            st_data[(cnt+reg_op2*13*8) +: 2] <= vecregs_rdata1[27:26];
                            st_data[(cnt+reg_op2*14*8) +: 2] <= vecregs_rdata1[29:28];
                            st_data[(cnt+reg_op2*15*8) +: 2] <= vecregs_rdata1[31:30];
                            st_strb[cnt >> 3] <= 1;
                            st_strb[(cnt >> 3) + reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 2*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 3*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 4*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 5*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 6*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 7*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 8*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 9*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 10*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 11*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 12*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 13*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 14*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 15*reg_op2] <= 1;
                            cnt <= cnt + 16*8*reg_op2;
                            mem_str_ready <= 1; 
                        end
                        if(vap == 10'b0000000100) begin
                            $display("vap condition, vecreg_data: %x, cnt:%d, time:%d", vecregs_rdata1, cnt, $time);
                            st_data[cnt +: 4] <= vecregs_rdata1[3:0];
                            st_data[(cnt+reg_op2*8) +: 4] <= vecregs_rdata1[7:4];
                            st_data[(cnt+reg_op2*2*8) +: 4] <= vecregs_rdata1[11:8];
                            st_data[(cnt+reg_op2*3*8) +: 4] <= vecregs_rdata1[15:12];
                            st_data[(cnt+reg_op2*4*8) +: 4] <= vecregs_rdata1[19:16];
                            st_data[(cnt+reg_op2*5*8) +: 4] <= vecregs_rdata1[23:20];
                            st_data[(cnt+reg_op2*6*8) +: 4] <= vecregs_rdata1[27:24];
                            st_data[(cnt+reg_op2*7*8) +: 4] <= vecregs_rdata1[31:28];
                            st_strb[cnt >> 3] <= 1;
                            st_strb[(cnt >> 3) + reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 2*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 3*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 4*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 5*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 6*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 7*reg_op2] <= 1;
                            cnt <= cnt + 8*8*reg_op2;
                            mem_str_ready <= 1; 
                        end
                        else if(vap == 10'b0000001000) begin
                            $display("vecreg_data: %x, cnt:%d, time:%d", vecregs_rdata1, cnt, $time);
                            st_data[cnt +: 8] <= vecregs_rdata1[7:0];
                            st_data[(cnt+reg_op2*8) +: 8] <= vecregs_rdata1[15:8];
                            st_data[(cnt+reg_op2*2*8) +: 8] <= vecregs_rdata1[23:16];
                            st_data[(cnt+reg_op2*3*8) +: 8] <= vecregs_rdata1[31:24];
                            st_strb[cnt >> 3] <= 1;
                            st_strb[(cnt >> 3) + reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 2*reg_op2] <= 1;
                            st_strb[(cnt >> 3) + 3*reg_op2] <= 1;
                            cnt <= cnt + 4*8*reg_op2;
                            mem_str_ready <= 1; 
                        end
                        condition_bit <= 0;
                    end
                end
            end
            3: begin
                mem_str_ready <= 0;
                if(mem_ready == 1) begin
                    // $display("Inside mem_interface, mem_wdata:%x, ind1:%d, ind2:%d, time:%d",st_data[ind1 +: 32], ind1, ind2, $time);
                    if(instr_vstore || instr_vstore_str || instr_vsesvarp || instr_vseuvarp) begin
                        mem_wdata <= st_data[ind1 +: 32]; //reads 32 bits
                        mem_wstrb <= st_strb[ind2 +: 4];
                        ind1 <= ind1 + 32;
                        ind2 <= ind2 + 4;
                    end
                    mem_str_ready <= 1; //str_ready will be 1 irrespective of the instruction
                end
            end
        endcase
    end
end

	//Variables used by Vadd and Vdot
	reg [1:0] unpack_data; //States to unpack the data for alu
	reg [9:0] unpack_index, unpack_index2;
	reg arth_data_ready; //Used to synchronize the data forwarding to ALU
	reg [alu_reg_len-1:0] opA, opB, opC; //Using these registers to unpack the data before sending it to the ALUs
	wire [alu_reg_len-1:0] alu_out;
	reg [31:0] alu_wdata;
	reg alu_enb;
	reg [7:0] micro_exec_instr;

	always @(posedge clk) begin
		if(pcpi_valid) begin
			case(unpack_data)
				1: begin
					if(condition_bit == 1) begin
						arth_data_ready <= 0;
						if(instr_vadd || instr_vmul || instr_vdot) begin
							// vecregs_rstrb1 <= 1 << (temp_count2+1);
							// $display("vecreg_data: %x, port3:%d, time:%d", vecregs_rdata1, port3_data, $time);
							if(!port3_data) begin
                            	vecregs_rstrb1 <= 1 << (temp_count2+1);
								opA[unpack_index +: 32]  <= vecregs_rdata1[31:0];
								opB[unpack_index +: 32]  <= vecregs_rdata2[31:0];
                                unpack_index <= unpack_index+32;
							end
							else begin
                                vecregs_rstrb1 <= 1 << (temp_count3+1);
								opC[unpack_index2 +: 32]  <= vecregs_rdata1[31:0];
								unpack_index2 <= unpack_index2+32;
							end
                            arth_data_ready <= 1; 
							condition_bit <= 0;
						end
						else if(instr_vaddvarp || instr_vsubvarp || instr_vmulvarp || instr_vdotvarp) begin
							if(vap == 10'b0000000001) begin
								$display("vecreg_data: %x, cnt:%d, time:%d", vecregs_rdata1, cnt, $time);
								//Converting sew of 1 to 8 bits to operate on them and zero padding on left side (for better multiplication)
								if(!port3_data) begin
                                    vecregs_rstrb1 <= 1 << (temp_count2+1);
                                    opA[unpack_index +: 32]     <= {{7{vecregs_rdata1[3]}},vecregs_rdata1[3],{7{vecregs_rdata1[2]}},vecregs_rdata1[2],{7{vecregs_rdata1[1]}},vecregs_rdata1[1],{7{vecregs_rdata1[0]}},vecregs_rdata1[0]};
                                    opA[unpack_index+32 +: 32]  <= {{7{vecregs_rdata1[7]}},vecregs_rdata1[7],{7{vecregs_rdata1[6]}},vecregs_rdata1[6],{7{vecregs_rdata1[5]}},vecregs_rdata1[5],{7{vecregs_rdata1[4]}},vecregs_rdata1[4]};
                                    opA[unpack_index+64 +: 32]  <= {{7{vecregs_rdata1[11]}},vecregs_rdata1[11],{7{vecregs_rdata1[10]}},vecregs_rdata1[10],{7{vecregs_rdata1[9]}},vecregs_rdata1[9],{7{vecregs_rdata1[8]}},vecregs_rdata1[8]};
                                    opA[unpack_index+96 +: 32]  <= {{7{vecregs_rdata1[15]}},vecregs_rdata1[15],{7{vecregs_rdata1[14]}},vecregs_rdata1[14],{7{vecregs_rdata1[13]}},vecregs_rdata1[13],{7{vecregs_rdata1[12]}},vecregs_rdata1[12]};
                                    opA[unpack_index+128 +: 32] <= {{7{vecregs_rdata1[19]}},vecregs_rdata1[19],{7{vecregs_rdata1[18]}},vecregs_rdata1[18],{7{vecregs_rdata1[17]}},vecregs_rdata1[17],{7{vecregs_rdata1[16]}},vecregs_rdata1[16]};
                                    opA[unpack_index+160 +: 32] <= {{7{vecregs_rdata1[23]}},vecregs_rdata1[23],{7{vecregs_rdata1[22]}},vecregs_rdata1[22],{7{vecregs_rdata1[21]}},vecregs_rdata1[21],{7{vecregs_rdata1[20]}},vecregs_rdata1[20]};
                                    opA[unpack_index+192 +: 32] <= {{7{vecregs_rdata1[27]}},vecregs_rdata1[27],{7{vecregs_rdata1[26]}},vecregs_rdata1[26],{7{vecregs_rdata1[25]}},vecregs_rdata1[25],{7{vecregs_rdata1[24]}},vecregs_rdata1[24]};
                                    opA[unpack_index+224 +: 32] <= {{7{vecregs_rdata1[31]}},vecregs_rdata1[31],{7{vecregs_rdata1[30]}},vecregs_rdata1[30],{7{vecregs_rdata1[29]}},vecregs_rdata1[29],{7{vecregs_rdata1[28]}},vecregs_rdata1[28]};
                                    opB[unpack_index +: 32]     <= {vecregs_rdata2[3],7'b0,vecregs_rdata2[2],7'b0,vecregs_rdata2[1],7'b0,vecregs_rdata2[0],7'b0};
                                    opB[unpack_index+32 +: 32]  <= {vecregs_rdata2[7],7'b0,vecregs_rdata2[6],7'b0,vecregs_rdata2[5],7'b0,vecregs_rdata2[4],7'b0};
                                    opB[unpack_index+64 +: 32]  <= {vecregs_rdata2[11],7'b0,vecregs_rdata2[10],7'b0,vecregs_rdata2[9],7'b0,vecregs_rdata2[8],7'b0};
                                    opB[unpack_index+96 +: 32]  <= {vecregs_rdata2[15],7'b0,vecregs_rdata2[14],7'b0,vecregs_rdata2[13],7'b0,vecregs_rdata2[12],7'b0};
                                    opB[unpack_index+128 +: 32] <= {vecregs_rdata2[19],7'b0,vecregs_rdata2[18],7'b0,vecregs_rdata2[17],7'b0,vecregs_rdata2[16],7'b0};
                                    opB[unpack_index+160 +: 32] <= {vecregs_rdata2[23],7'b0,vecregs_rdata2[22],7'b0,vecregs_rdata2[21],7'b0,vecregs_rdata2[20],7'b0};
                                    opB[unpack_index+192 +: 32] <= {vecregs_rdata2[27],7'b0,vecregs_rdata2[26],7'b0,vecregs_rdata2[25],7'b0,vecregs_rdata2[24],7'b0};
                                    opB[unpack_index+224 +: 32] <= {vecregs_rdata2[31],7'b0,vecregs_rdata2[30],7'b0,vecregs_rdata2[29],7'b0,vecregs_rdata2[28],7'b0};
                                    unpack_index <= unpack_index+256;
                                end
                                else begin
                                    vecregs_rstrb1 <= 1 << (temp_count3+1);
                                    opC[unpack_index2 +: 32]     <= {{7{vecregs_rdata1[3]}},vecregs_rdata1[3],{7{vecregs_rdata1[2]}},vecregs_rdata1[2],{7{vecregs_rdata1[1]}},vecregs_rdata1[1],{7{vecregs_rdata1[0]}},vecregs_rdata1[0]};
                                    opC[unpack_index2+32 +: 32]  <= {{7{vecregs_rdata1[7]}},vecregs_rdata1[7],{7{vecregs_rdata1[6]}},vecregs_rdata1[6],{7{vecregs_rdata1[5]}},vecregs_rdata1[5],{7{vecregs_rdata1[4]}},vecregs_rdata1[4]};
                                    opC[unpack_index2+64 +: 32]  <= {{7{vecregs_rdata1[11]}},vecregs_rdata1[11],{7{vecregs_rdata1[10]}},vecregs_rdata1[10],{7{vecregs_rdata1[9]}},vecregs_rdata1[9],{7{vecregs_rdata1[8]}},vecregs_rdata1[8]};
                                    opC[unpack_index2+96 +: 32]  <= {{7{vecregs_rdata1[15]}},vecregs_rdata1[15],{7{vecregs_rdata1[14]}},vecregs_rdata1[14],{7{vecregs_rdata1[13]}},vecregs_rdata1[13],{7{vecregs_rdata1[12]}},vecregs_rdata1[12]};
                                    opC[unpack_index2+128 +: 32] <= {{7{vecregs_rdata1[19]}},vecregs_rdata1[19],{7{vecregs_rdata1[18]}},vecregs_rdata1[18],{7{vecregs_rdata1[17]}},vecregs_rdata1[17],{7{vecregs_rdata1[16]}},vecregs_rdata1[16]};
                                    opC[unpack_index2+160 +: 32] <= {{7{vecregs_rdata1[23]}},vecregs_rdata1[23],{7{vecregs_rdata1[22]}},vecregs_rdata1[22],{7{vecregs_rdata1[21]}},vecregs_rdata1[21],{7{vecregs_rdata1[20]}},vecregs_rdata1[20]};
                                    opC[unpack_index2+192 +: 32] <= {{7{vecregs_rdata1[27]}},vecregs_rdata1[27],{7{vecregs_rdata1[26]}},vecregs_rdata1[26],{7{vecregs_rdata1[25]}},vecregs_rdata1[25],{7{vecregs_rdata1[24]}},vecregs_rdata1[24]};
                                    opC[unpack_index2+224 +: 32] <= {{7{vecregs_rdata1[31]}},vecregs_rdata1[31],{7{vecregs_rdata1[30]}},vecregs_rdata1[30],{7{vecregs_rdata1[29]}},vecregs_rdata1[29],{7{vecregs_rdata1[28]}},vecregs_rdata1[28]};
                                    unpack_index2 <= unpack_index2+256;
                                end
								arth_data_ready <= 1; 
							end
							if(vap == 10'b0000000010) begin
								// $display("vecreg_data1: %x, vecreg_data2:%x, index:%d, time:%d", vecregs_rdata1, vecregs_rdata2, unpack_index, $time);
                                if(!port3_data) begin
                                    vecregs_rstrb1 <= 1 << (temp_count2+1);
                                    opA[unpack_index +: 32]     <= {{6{vecregs_rdata1[7]}},vecregs_rdata1[7:6],{6{vecregs_rdata1[5]}},vecregs_rdata1[5:4],{6{vecregs_rdata1[3]}},vecregs_rdata1[3:2],{6{vecregs_rdata1[1]}},vecregs_rdata1[1:0]};
                                    opA[unpack_index+32 +: 32]  <= {{6{vecregs_rdata1[15]}},vecregs_rdata1[15:14],{6{vecregs_rdata1[13]}},vecregs_rdata1[13:12],{6{vecregs_rdata1[11]}},vecregs_rdata1[11:10],{6{vecregs_rdata1[9]}},vecregs_rdata1[9:8]};
                                    opA[unpack_index+64 +: 32]  <= {{6{vecregs_rdata1[23]}},vecregs_rdata1[23:22],{6{vecregs_rdata1[21]}},vecregs_rdata1[21:20],{6{vecregs_rdata1[19]}},vecregs_rdata1[19:18],{6{vecregs_rdata1[17]}},vecregs_rdata1[17:16]};
                                    opA[unpack_index+96 +: 32]  <= {{6{vecregs_rdata1[31]}},vecregs_rdata1[31:30],{6{vecregs_rdata1[29]}},vecregs_rdata1[29:28],{6{vecregs_rdata1[27]}},vecregs_rdata1[27:26],{6{vecregs_rdata1[25]}},vecregs_rdata1[25:24]};
                                    opB[unpack_index +: 32]     <= {vecregs_rdata2[7:6],6'b0,vecregs_rdata2[5:4],6'b0,vecregs_rdata2[3:2],6'b0,vecregs_rdata2[1:0],6'b0};
                                    opB[unpack_index+32 +: 32]  <= {vecregs_rdata2[15:14],6'b0,vecregs_rdata2[13:12],6'b0,vecregs_rdata2[11:10],6'b0,vecregs_rdata2[9:8],6'b0};
                                    opB[unpack_index+64 +: 32]  <= {vecregs_rdata2[23:22],6'b0,vecregs_rdata2[21:20],6'b0,vecregs_rdata2[19:18],6'b0,vecregs_rdata2[17:16],6'b0};
                                    opB[unpack_index+96 +: 32]  <= {vecregs_rdata2[31:30],6'b0,vecregs_rdata2[29:28],6'b0,vecregs_rdata2[27:26],6'b0,vecregs_rdata2[25:24],6'b0};
                                    unpack_index <= unpack_index+128;
                                end
                                else begin
                                    vecregs_rstrb1 <= 1 << (temp_count3+1);
                                    opC[unpack_index2 +: 32]     <= {{6{vecregs_rdata1[7]}},vecregs_rdata1[7:6],{6{vecregs_rdata1[5]}},vecregs_rdata1[5:4],{6{vecregs_rdata1[3]}},vecregs_rdata1[3:2],{6{vecregs_rdata1[1]}},vecregs_rdata1[1:0]};
                                    opC[unpack_index2+32 +: 32]  <= {{6{vecregs_rdata1[15]}},vecregs_rdata1[15:14],{6{vecregs_rdata1[13]}},vecregs_rdata1[13:12],{6{vecregs_rdata1[11]}},vecregs_rdata1[11:10],{6{vecregs_rdata1[9]}},vecregs_rdata1[9:8]};
                                    opC[unpack_index2+64 +: 32]  <= {{6{vecregs_rdata1[23]}},vecregs_rdata1[23:22],{6{vecregs_rdata1[21]}},vecregs_rdata1[21:20],{6{vecregs_rdata1[19]}},vecregs_rdata1[19:18],{6{vecregs_rdata1[17]}},vecregs_rdata1[17:16]};
                                    opC[unpack_index2+96 +: 32]  <= {{6{vecregs_rdata1[31]}},vecregs_rdata1[31:30],{6{vecregs_rdata1[29]}},vecregs_rdata1[29:28],{6{vecregs_rdata1[27]}},vecregs_rdata1[27:26],{6{vecregs_rdata1[25]}},vecregs_rdata1[25:24]};
                                    unpack_index2 <= unpack_index2+128;
                                end
								arth_data_ready <= 1; 
							end
							if(vap == 10'b0000000100) begin
								// $display("vecreg_data1: %x, vecreg_data2:%x, index:%d, time:%d", vecregs_rdata1, vecregs_rdata2, unpack_index, $time);
								//Sign extending opA
                                if(!port3_data) begin
                                    vecregs_rstrb1 <= 1 << (temp_count2+1);
                                    opA[unpack_index +: 32]     <= {{4{vecregs_rdata1[15]}},vecregs_rdata1[15:12],{4{vecregs_rdata1[11]}},vecregs_rdata1[11:8],{4{vecregs_rdata1[7]}},vecregs_rdata1[7:4],{4{vecregs_rdata1[3]}},vecregs_rdata1[3:0]};
                                    opA[unpack_index+32 +: 32]  <= {{4{vecregs_rdata1[31]}},vecregs_rdata1[31:28],{4{vecregs_rdata1[27]}},vecregs_rdata1[27:24],{4{vecregs_rdata1[23]}},vecregs_rdata1[23:20],{4{vecregs_rdata1[19]}},vecregs_rdata1[19:16]};
                                    opB[unpack_index +: 32]     <= {vecregs_rdata2[15:12],4'b0,vecregs_rdata2[11:8],4'b0,vecregs_rdata2[7:4],4'b0,vecregs_rdata2[3:0],4'b0};
                                    opB[unpack_index+32 +: 32]  <= {vecregs_rdata2[31:28],4'b0,vecregs_rdata2[27:24],4'b0,vecregs_rdata2[23:20],4'b0,vecregs_rdata2[19:16],4'b0};
                                    unpack_index <= unpack_index+64;
                                end
                                else begin
                                    vecregs_rstrb1 <= 1 << (temp_count3+1);
                                    opC[unpack_index2 +: 32]     <= {{4{vecregs_rdata1[15]}},vecregs_rdata1[15:12],{4{vecregs_rdata1[11]}},vecregs_rdata1[11:8],{4{vecregs_rdata1[7]}},vecregs_rdata1[7:4],{4{vecregs_rdata1[3]}},vecregs_rdata1[3:0]};
								    opC[unpack_index2+32 +: 32]  <= {{4{vecregs_rdata1[31]}},vecregs_rdata1[31:28],{4{vecregs_rdata1[27]}},vecregs_rdata1[27:24],{4{vecregs_rdata1[23]}},vecregs_rdata1[23:20],{4{vecregs_rdata1[19]}},vecregs_rdata1[19:16]};
                                    unpack_index2 <= unpack_index2+64;
                                end
								arth_data_ready <= 1; 
							end
							else if(vap == 10'b0000001000) begin
								// $display("vecreg_data: %x, cnt:%d, time:%d", vecregs_rdata1, cnt, $time);
                                if(!port3_data) begin
                                    vecregs_rstrb1 <= 1 << (temp_count2+1);
                                    opA[unpack_index +: 32]  <= vecregs_rdata1[31:0];
                                    opB[unpack_index +: 32]  <= vecregs_rdata2[31:0];
							    	unpack_index <= unpack_index+32;
                                end
                                else begin
                                    vecregs_rstrb1 <= 1 << (temp_count3+1);
                                    opC[unpack_index2 +: 32]  <= vecregs_rdata1[31:0];
                                    unpack_index2 <= unpack_index2+32;
                                end
								arth_data_ready <= 1; 
							end
							condition_bit <= 0;
						end
					end
				end
				2: begin
					arth_data_ready <= 0;
					if(instr_vadd || instr_vmul || instr_vdot) begin
						alu_wdata[31:0]   <= alu_out[ind1 +: 32];
						arth_data_ready <= 1; 
						ind1 <= ind1 + 4*8;
					end
					if(instr_vaddvarp || instr_vmulvarp || instr_vdotvarp || instr_vsubvarp) begin
						if(vap == 10'b0000000001) begin
							//The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
							alu_wdata[0]   <= alu_out[ind1 +: 1];
							alu_wdata[1]   <= alu_out[(ind1+8) +: 1];
							alu_wdata[2]   <= alu_out[(ind1+16) +: 1];
							alu_wdata[3]   <= alu_out[ind1+24 +: 1];
							alu_wdata[4]   <= alu_out[(ind1+32) +: 1];
							alu_wdata[5]   <= alu_out[(ind1+40) +: 1];
							alu_wdata[6]   <= alu_out[(ind1+48) +: 1];
							alu_wdata[7]   <= alu_out[(ind1+56) +: 1];
							alu_wdata[8]   <= alu_out[ind1+8*8 +: 1];
							alu_wdata[9]   <= alu_out[(ind1+9*8) +: 1];
							alu_wdata[10]  <= alu_out[(ind1+10*8) +: 1];
							alu_wdata[11]  <= alu_out[ind1+11*8 +: 1];
							alu_wdata[12]  <= alu_out[(ind1+12*8) +: 1];
							alu_wdata[13]  <= alu_out[(ind1+13*8) +: 1];
							alu_wdata[14]  <= alu_out[(ind1+14*8) +: 1];
							alu_wdata[15]  <= alu_out[(ind1+15*8) +: 1];
							alu_wdata[16]  <= alu_out[(ind1+16*8) +: 1];
							alu_wdata[17]  <= alu_out[(ind1+17*8) +: 1];
							alu_wdata[18]  <= alu_out[(ind1+18*8) +: 1];
							alu_wdata[19]  <= alu_out[ind1+19*8 +: 1];
							alu_wdata[20]  <= alu_out[(ind1+20*8) +: 1];
							alu_wdata[21]  <= alu_out[(ind1+21*8) +: 1];
							alu_wdata[22]  <= alu_out[(ind1+22*8) +: 1];
							alu_wdata[23]  <= alu_out[(ind1+23*8) +: 1];
							alu_wdata[24]  <= alu_out[ind1+24*8 +: 1];
							alu_wdata[25]  <= alu_out[(ind1+25*8) +: 1];
							alu_wdata[26]  <= alu_out[(ind1+26*8) +: 1];
							alu_wdata[27]  <= alu_out[ind1+27*8 +: 1];
							alu_wdata[28]  <= alu_out[(ind1+28*8) +: 1];
							alu_wdata[29]  <= alu_out[(ind1+29*8) +: 1];
							alu_wdata[30]  <= alu_out[(ind1+30*8) +: 1];
							alu_wdata[31]  <= alu_out[(ind1+31*8) +: 1];
							arth_data_ready <= 1; 
							ind1 <= ind1 + 32*8;
						end
						else if(vap == 10'b0000000010) begin
							//The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
							// $display("aluout:%b, ind1:%d, time:%d", alu_out[95:0], ind1, $time);
							alu_wdata[1:0]   <= alu_out[ind1 +: 2];
							alu_wdata[3:2]   <= alu_out[(ind1+8) +: 2];
							alu_wdata[5:4]   <= alu_out[(ind1+16) +: 2];
							alu_wdata[7:6]   <= alu_out[ind1+24 +: 2];
							alu_wdata[9:8]   <= alu_out[(ind1+32) +: 2];
							alu_wdata[11:10] <= alu_out[(ind1+40) +: 2];
							alu_wdata[13:12] <= alu_out[(ind1+48) +: 2];
							alu_wdata[15:14] <= alu_out[(ind1+56) +: 2];
							alu_wdata[17:16] <= alu_out[ind1+8*8 +: 2];
							alu_wdata[19:18] <= alu_out[(ind1+9*8) +: 2];
							alu_wdata[21:20] <= alu_out[(ind1+10*8) +: 2];
							alu_wdata[23:22] <= alu_out[ind1+11*8 +: 2];
							alu_wdata[25:24] <= alu_out[(ind1+12*8) +: 2];
							alu_wdata[27:26] <= alu_out[(ind1+13*8) +: 2];
							alu_wdata[29:28] <= alu_out[(ind1+14*8) +: 2];
							alu_wdata[31:30] <= alu_out[(ind1+15*8) +: 2];
							arth_data_ready <= 1; 
							ind1 <= ind1 + 16*8;
						end
						else if(vap == 10'b0000000100) begin
							// $display("Inside mem_wsize2,ind1:%d, alu_out:%x, mem_rdata_word:%x, time:%d",ind1, alu_out[127:0], mem_rdata_word, $time);
							//The first addr will be ind1, next will be ind1+regop2*1*8, next reg_op2*2*8 etc
							alu_wdata[3:0]   <= alu_out[ind1 +: 4];
							alu_wdata[7:4]   <= alu_out[(ind1+8) +: 4];
							alu_wdata[11:8]  <= alu_out[(ind1+16) +: 4];
							alu_wdata[15:12] <= alu_out[ind1+24 +: 4];
							alu_wdata[19:16] <= alu_out[(ind1+32) +: 4];
							alu_wdata[23:20] <= alu_out[(ind1+40) +: 4];
							alu_wdata[27:24] <= alu_out[(ind1+48) +: 4];
							alu_wdata[31:28] <= alu_out[(ind1+56) +: 4];
							arth_data_ready <= 1; 
							ind1 <= ind1 + 8*8;
						end
						if(vap == 10'b0000001000) begin
							// $display("Inside mem_wsize1,ind1:%d, mem_rdata_word:%x, time:%d",ind1, mem_rdata_word, $time);
							alu_wdata[31:0]   <= alu_out[31:0];
							arth_data_ready <= 1; 
							ind1 <= ind1 + 4*8;
						end
					end
				end
			endcase
		end
	end
		
	assign is_vec_instr = |{instr_vsetvli,instr_vsetvl,instr_vsetprecision,instr_vload,instr_vload_str, instr_vleuvarp, instr_vlesvarp, instr_vstore, instr_vstore_str, instr_vseuvarp,
							instr_vsesvarp, instr_vdot, instr_vadd, instr_vmul, instr_vaddvarp, instr_vsubvarp, instr_vmulvarp, instr_vdotvarp};
	assign is_vap_instr = |{instr_vleuvarp, instr_vlesvarp, instr_vsesvarp, instr_vseuvarp, instr_vsetprecision, instr_vaddvarp, instr_vsubvarp, instr_vmulvarp, instr_vdotvarp}; //To heck whether the instruction is variable bit one
	assign is_port3_instr = |{instr_vdot, instr_vdotvarp};
	reg [4:0] decoded_vs1, decoded_vs2, decoded_vd; //For vect instrns
	reg [10:0] decoded_vimm; //For vect instrns

	//Instruction decoder
	always@(posedge clk) begin
		if (!resetn || !pcpi_valid) begin
			instr_vsetvl <= 0;
			instr_vsetvli <= 0;
			instr_vsetprecision <= 0;
			instr_vload <= 0;
			instr_vload_str <= 0;
			instr_vstore <= 0;
			instr_vstore_str <= 0; //For strided store
			instr_vleuvarp <= 0; 
			instr_vlesvarp <= 0;
			instr_vseuvarp <= 0;
			instr_vsesvarp <= 0;
			instr_vdot  <= 0;
			instr_vadd  <= 0;
			instr_vmul <= 0;
			instr_vaddvarp <= 0;
			instr_vsubvarp <= 0;
			instr_vmulvarp <= 0;
			instr_vdotvarp <= 0;
			mem_wstrb <= 4'b0;
			unpack_data <= 0; //Used in case statement
			port3_data <= 0;
		end
		else begin
			// $display("Inside decode stage, instrn: %x, time: %d", pcpi_insn, $time);
			decoded_vs1 <= pcpi_insn[19:15];
			decoded_vs2 <= pcpi_insn[24:20];
			decoded_vd  <= pcpi_insn[11:7];
			decoded_vimm <= pcpi_insn[30:20];
			//Load and store currently supports only unit stride
			instr_vload   <= (pcpi_insn[24:20]==5'b00000) && (pcpi_insn[28:26]==3'b000) && (pcpi_insn[6:0] == 7'b0000111); // NF not supported
			instr_vload_str <= (pcpi_insn[28:26]==3'b010) && (pcpi_insn[14:12]==3'b111) && (pcpi_insn[6:0] == 7'b0000111); //Strided load
			instr_vleuvarp <= (pcpi_insn[29:26]==4'b0000 && pcpi_insn[14:12]==3'b111 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:30] == 2'b00); //Unit strided load for vbp
			instr_vlesvarp <= (pcpi_insn[29:26]==4'b0001 && pcpi_insn[14:12]==3'b111 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:30] == 2'b00); //Strided load for vbp
			instr_vseuvarp <= (pcpi_insn[29:26]==4'b0000 && pcpi_insn[14:12]==3'b111 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:30] == 2'b01); //Unit strided store for vbp
			instr_vsesvarp <= (pcpi_insn[29:26]==4'b0001 && pcpi_insn[14:12]==3'b111 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:30] == 2'b01); //Strided store for vbp
			instr_vstore  <= (pcpi_insn[24:20]==5'b00000) && (pcpi_insn[28:26]==3'b000) && (pcpi_insn[6:0] == 7'b0100111); // only unit stride supported,NF not supported 
			instr_vstore_str <= (pcpi_insn[28:26]==3'b010) && (pcpi_insn[14:12]==3'b111) && (pcpi_insn[6:0] == 7'b0100111); //Strided store, works for 32 bit SEW
			instr_vsetvl  <= (pcpi_insn[14:12]==3'b111) && (pcpi_insn[31]==1) && (pcpi_insn[6:0] == 7'b1010111); 
			instr_vsetvli <= (pcpi_insn[14:12]==3'b111) && (pcpi_insn[31]==0) && (pcpi_insn[6:0] == 7'b1010111);
			instr_vsetprecision <= (pcpi_insn[14:12]==3'b111 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:25] == 7'b1000000);
			instr_vdot    <= (pcpi_insn[31:26]==6'b111001) && (pcpi_insn[14:12] == 3'b000) && (pcpi_insn[6:0]==7'b1010111);
			instr_vadd    <= (pcpi_insn[31:26]==6'b000000) && (pcpi_insn[14:12] == 3'b000) && (pcpi_insn[6:0]==7'b1010111);
			instr_vmul    <= (pcpi_insn[31:26]==6'b100101) && (pcpi_insn[14:12] == 3'b000) && (pcpi_insn[6:0]==7'b1010111);
			//Arithmetic instr for vap
			instr_vaddvarp <= (pcpi_insn[29:25]==5'b00000 && pcpi_insn[14:12]==3'b000 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:30] == 2'b11);
			instr_vsubvarp <= (pcpi_insn[29:25]==5'b00001 && pcpi_insn[14:12]==3'b000 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:30] == 2'b11);
			instr_vmulvarp <= (pcpi_insn[29:25]==5'b00010 && pcpi_insn[14:12]==3'b000 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:30] == 2'b11);
			instr_vdotvarp <= (pcpi_insn[29:25]==5'b00011 && pcpi_insn[14:12]==3'b000 && pcpi_insn[6:0] == 7'b1011011 && pcpi_insn[31:30] == 2'b11);
			v_enc_width   <= (instr_vload || instr_vstore || instr_vload_str || instr_vstore_str || instr_vleuvarp || instr_vlesvarp || instr_vseuvarp || instr_vsesvarp)? pcpi_insn[14:12]:0;
		end
	end
	
	//For vector instructions
	wire [31:0] vcsr_vlenb = 32'h00000080; //vector reg length in bytes
	reg [31:0] vcsr_vlen;
	reg [31:0] vcsr_vtype; //Can be updated by vsetvli or vsetvl instr
	reg [31:0] vcsr_vl;    //No of elements to be updated by a vector instrn
	reg [31:0] vcsr_vap; //Register used by custom inst for variable precision
	wire [3:0] vap;  //Contains the SEW for variable precision (1 to 15)
	reg [31:0] vcsr_elem_offset; //Used for strided load and store (Against RISC V specs)
	assign vap = vcsr_vap[3:0]; //Storing the SEW in vap
	wire [2:0]vsew  = vcsr_vtype[4:2]; //Encoding for the no of bits in an element (part of vtype)
	wire [1:0]vlmul = vcsr_vtype[1:0]; //Encoding for the no of vec regs in a group (part of vtype)
	wire [9:0] SEW;
	assign SEW  = (4*(2<<vsew));
    wire [6:0] sew_bytes; //No of bytes in a SEW - 1 (Used to calculate final addr for ld and st)
	wire [1:0] sew_bytes_vap; //No of bytes in a SEW (Used for vap load and store instructions)
	reg [31:0] str_bits; //Used for vec_str instruction
    assign sew_bytes = ((1<<vsew)-1);  //Used for vector load and store instruction
	assign sew_bytes_vap = (vap-1) >> 2; //if vap <= 8, it should be 0 else 1
	wire [31:0]LMUL = (1 << (vlmul)); //No of vector regs in a group (2^vlmul)
	wire [31:0]VLMAX = (512/SEW)*LMUL; //Represents the max no of elements that can be operated on with a vec instrn

	//Variables used by vload and vstore
    reg [6:0] mem_read_no; //No of memory reads for load instrn
	reg [6:0] new_mem_read_no; //Used when no of reads are more
    reg [9:0] init_addr; //Used for vector ld and st
    reg [9:0] final_addr; //Used for vector ld and st
	reg [5:0] vecldstrcnt;
	reg [5:0] var_vlen; //Used to load the vector reg partially
	reg [31:0] vreg_op1;  //Data from v1 for all vector instr
	reg [31:0] vreg_op2;  //Data from v2 for all vector instr
	reg [31:0] vreg_op3;  //Data from vd for dot product instruction
	reg [31:0] vreg_rdata1_latched; //data readfrom vector regs, used for vstore
	reg [2:0] v_enc_width;
	reg [10:0] v_membits; //Contains the number of bits to be loaded from memory
	reg [15:0] vecregs_wstrb_temp; //Used to store write strobe
    reg [flat_reg_len-1:0] ld_data; //To use flat memory
	reg [flat_reg_len-1:0] st_data; //To use flat memory for store
	reg [15:0] mem_write_no; //No of memory writes required for vector store
	reg [15:0] new_mem_write_no;
	reg [63:0] st_strb; //64 bit strb used for vector store (1 bit is for 1 byte)
    reg [9:0] cnt;
	reg temp_var = 0; //Used for a reset condition for vector load
	reg condition_bit; //Used in memory FSM
    reg [9:0] ind1; //Used for vec load
	reg [5:0] ind2; //Used to index the strb
    reg [5:0] no_words; //No of words to read
	reg [7:0] new_no_words;
	reg [5:0] read_count, read_count2; //Used in ld_mem state
	reg [4:0] bits_remaining; //Bits remaining after words are loaded
    reg [5:0] temp_count; //Used for indexing (strb in vector regs)
	reg [4:0] temp_count2, temp_count3; //Used for reading data from vec reg during store
	//Variables used by Vadd and Vdot
	wire done1, done2, done3, done4, done5, done6, done7, done8, done9, done10, done11, done12, done13, done14, done15, done16;
	wire alu_done; //Used to check whether all the ALUs have returned the outputs or not
	assign alu_done = &{done1, done2, done3, done4, done5, done6, done7, done8, done9, done10, done11, done12, done13, done14, done15, done16};
	reg [9:0] elem_n; //Maximum no of elements is 512 if SEW is 1
	reg [31:0] vecrs1;
	reg [31:0] vecrs2;
	reg [31:0] vecrs3; //For dit product

	reg set_mem_do_rdata;
	reg set_mem_do_wdata;

	// For vector instructions
	//vector register bank for vector instructions
	reg vecregs_write;  //Write enable
	reg [4:0] vecregs_waddr; 
	reg [15:0] vecregs_wstrb;
	reg [4:0] vecregs_raddr1;
	reg [4:0] vecregs_raddr2;
	reg port3_en;  //Used to read the third register
	reg [15:0] vecregs_rstrb1;
	reg [31:0] vecregs_wdata;
	wire [31:0] vecregs_rdata1;
	wire [31:0] vecregs_rdata2;
		
	vector_regs vecreg_inst (
		.clk(clk),
		.wen(resetn && vecregs_write),
		.waddr(vecregs_waddr),
		.vec_wstrb(vecregs_wstrb),
		.raddr1(vecregs_raddr1),
		.raddr2(vecregs_raddr2),
		.port3_en(port3_en),
		.vec_rstrb1(vecregs_rstrb1),
		.wdata(vecregs_wdata),
		.rdata1(vecregs_rdata1),
		.rdata2(vecregs_rdata2)
	);	

	//Main state machine
	localparam cpu_state_fetch   = 8'b10000000;
	localparam cpu_state_ld_rs1  = 8'b01000000;
	localparam cpu_state_exec    = 8'b00100000;
	localparam cpu_state_stmem   = 8'b00010000;
	localparam cpu_state_stmem2  = 8'b00001000;
    localparam cpu_state_ldmem   = 8'b00000100;
	localparam cpu_state_ldmem2  = 8'b00000010; 
	localparam cpu_state_exec2    = 8'b00000001;

	reg [7:0] cpu_state;
	reg latched_vstore;  //Added for vector instruction
	reg latched_stalu;	//This wil be 1 if the result to be written to register is the output of ALU

	always @(posedge clk) begin
		// $display("inside vstore, latched_vs:%b, time:%d", latched_vstore, $time);
		case(1'b1)
			//latched_vstore will be 1 for 1 clk cycle
			latched_vstore: begin
				if(!instr_vstore && !instr_vstore_str) begin //If the instr is not store
					vecregs_wstrb <= vecregs_wstrb_temp;
					vecregs_wdata <= latched_stalu ? alu_out:vreg_op1;
					vecregs_write <= 1;  //wen for load instruction
				end
			end
		endcase
	end


	always@(posedge clk) begin
		set_mem_do_rdata = 0;
		set_mem_do_wdata = 0;
		if(!resetn || !pcpi_valid || !is_vec_instr) begin
			// $display("Inside reset condition, time: %d, reset:%b, pcpi_valid:%b, is_vec_instr: %b", $time, resetn, pcpi_valid, is_vec_instr);
			pcpi_rd <= 0;
			pcpi_wait <= 0;
	        pcpi_ready <= 0;
            pcpi_wr <= 0;
			cpu_state <= cpu_state_fetch; //Default state
			latched_stalu <= 0;
			latched_vstore <= 0;
			vecregs_write <= 0; //If pcpi_valid is 0, make wen as 0
			mem_valid <= 0; //If pcpi_valid is 0, mem_valid = 0
			mem_do_wdata <= 0;
			vecregs_rstrb1 <= 16'b0;
			vecregs_wstrb_temp <= 16'b0;
			elem_n <= 0;
			port3_en <= 0;
		end
		else begin
			pcpi_wait <= 1;
			vecregs_write = 0; //Will be 1 for only 1 clk cycle
			pcpi_ready <= 0; //Will be 1 for only 1 clk cycle
			latched_vstore <= 0; 
			case(cpu_state)
				cpu_state_fetch: begin
					$display("Inside fetch state, mem_do_wdata:%b, time: %d", mem_do_wdata, $time);
					mem_valid <= 0; //Not initializing the memory in fetch stage
					reg_op1 <= pcpi_cpurs1;
					reg_op2 <= pcpi_cpurs2;
					str_bits <= vcsr_vl*pcpi_cpurs2*8;
					latched_stalu <= 0;
					mem_wordsize <= 0; //Has to write/read 32 bit data
					cpu_state <= cpu_state_ld_rs1;
				end
				cpu_state_ld_rs1: begin
					case(1'b1)
						(instr_vsetvli || instr_vsetvl):  begin	
							$display("Inside vsetvli condition, time:%d", $time);
							vcsr_vl <= (decoded_vs1!=5'b00000) ? pcpi_cpurs1:vcsr_vl;
							vcsr_vtype <= (instr_vsetvl) ? pcpi_cpurs2:decoded_vimm;
							// $display("pcpi_cpurs1: %d",pcpi_cpurs1);
							// $display("Inside coproc, vcsr_vl = %d, vcsr_vtype = %b, decoded_vs1 = %b", vcsr_vl,vcsr_vtype, decoded_vs1);
							cpu_state <= cpu_state_ld_rs1;
							pcpi_ready <= 1;
							pcpi_wait <= 0;
						end
						(instr_vsetprecision): begin
							$display("Inside vsetprecsion condition, vcsr_vap:%d, elem_off:%d, time:%d", reg_op1, reg_op2, $time);
							vcsr_vap <= reg_op1;
							vcsr_elem_offset <= reg_op2;
							cpu_state <= cpu_state_ld_rs1;
							pcpi_ready <= 1;
							pcpi_wait <= 0;
						end
						(instr_vload): begin
							$display("Inside v_load condition, time:%d", $time);
							cpu_state <= cpu_state_ldmem;
                            init_addr <= reg_op1 >> 2; //Initial word addrr
							//Since unit strided, stride will be 1 i.e reg_op2 will be 1
                            final_addr <= ((reg_op1 + (vcsr_vl-1) + sew_bytes) >> 2);  //Calculates the word addr of final byte 
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_ready2 <= 0;
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
                            no_words <= ((vcsr_vl*SEW)>>5); //No of words to read
                            cnt <= 0;
							temp_var <= 0;
                            temp_count <= 0;
                            ind1 <= (reg_op1[1:0] << 3); //byte addr to bit addr, used to read data from ld_data reg
							reg_op2 <= 1;
							//This is used when no of mem_reads crosses flat_reg_len bits
							new_no_words <= (flat_reg_len >> 8)*SEW;  //Derives from { flat_reg_len*SEW / (32*8*stride)} and stride is 1
							read_count <= 0;
							//Number of bits to read in each cycle
							// if(SEW == 10'b0000100000)
							mem_wordsize <= 0;
						end
						(instr_vload_str): begin
							$display("Inside v_load_stride condition, new_no_words:%d, time:%d",(((flat_reg_len >> 8)*SEW)), $time);
							cpu_state <= cpu_state_ldmem;
                            init_addr <= reg_op1 >> 2; //Initial word addrr
                            final_addr <= ((reg_op1 + (vcsr_vl-1)*reg_op2 + sew_bytes) >> 2);  //Calculates the word addr of final byte 
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_ready2 <= 0;
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
                            no_words <= ((vcsr_vl*SEW)>>5); //No of words to read
                            cnt <= 0;
							temp_var <= 0;
                            temp_count <= 0;
                            ind1 <= (reg_op1[1:0] << 3); //byte addr to bit addr
							//This is used when no of mem_reads crosses flat_reg_len bits
							new_no_words <= (reg_op2==0)? 64: (((flat_reg_len >> 8)*SEW) / reg_op2);  //Derives from { flat_reg_len*SEW / (32*8*stride) }
							read_count <= 0;
							//Number many bits to read in each cycle
							// if(SEW == 10'b0000100000)
							mem_wordsize <= 0;
						end
						(instr_vleuvarp): begin
							$display("Inside vap_unit_load condition, time:%d", $time);
							cpu_state <= cpu_state_ldmem;
                            init_addr <= reg_op1 >> 2; //Initial word addrr
							//Since unit strided, stride will be 1 i.e reg_op2 will be 1
                            final_addr <= ((reg_op1 + (vcsr_vl-1)*1 + sew_bytes_vap) >> 2);  //Calculates the word addr of final byte 
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_ready2 <= 0;
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:vap); 
                            no_words <= ((vcsr_vl*vap)>>5); //No of words to read
                            cnt <= 0;
							temp_var <= 0;
                            temp_count <= 0;
                            ind1 <= (reg_op1[1:0] << 3); //byte addr to bit addr
							reg_op2 <= 1;
							//This is used when no of mem_reads crosses flat_reg_len bits
							new_no_words <= (flat_reg_len >> 8)*vap;  //Derives from { flat_reg_len*SEW / (32*8*stride) } and stride is 1
							read_count <= 0;
							mem_wordsize <= 0;
						end
						(instr_vlesvarp): begin
							$display("Inside vap_load_stride condition time:%d", $time);
							cpu_state <= cpu_state_ldmem;
                            init_addr <= reg_op1 >> 2; //Initial word addrr
                            final_addr <= ((reg_op1 + (vcsr_vl-1)*reg_op2 + sew_bytes_vap) >> 2);  //Calculates the word addr of final byte 
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							mem_str_ready <= 0; //Initial value of mem_str_ready
							mem_str_ready2 <= 0;
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:vap); 
                            no_words <= ((vcsr_vl*vap)>>5); //No of words to readEntered vector instruction condition in 1840, time:                 1280

                            cnt <= 0;
							temp_var <= 0;
                            temp_count <= 0;
                            ind1 <= (reg_op1[1:0] << 3); //byte addr to bit addr
							//This is used when no of mem_reads crosses flat_reg_len bits
							$display("reg_op1:%d, reg_op2:%d, time:%d", reg_op1, reg_op2, $time);
							new_no_words <= (reg_op2==0) ? 128 : (((flat_reg_len >> 8)*vap) / reg_op2);  //Derives from { flat_reg_len*SEW / (32*8*stride) }
							read_count <= 0;
							mem_wordsize <= 0;
						end
						(instr_vstore): begin
							$display("Inside v_store condition, time:%d", $time);
							cpu_state <= cpu_state_stmem;
							// var_vlen <= 17 - ((vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW)) >> 5);//right shift to divide with 32
							// $display("var_length:%d, time:%d",var_vlen, $time);
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vd; //specifies v register holding store data
							vecregs_raddr2 <= decoded_vs2;
							vecregs_rstrb1 <= 16'b1;  //To read the first word
							mem_str_ready <= 0; //Initial value of mem_str_ready
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
							no_words <= ((vcsr_vl*SEW)>>5); //No of words to read from vec reg
							cnt <= (reg_op1[1:0] << 3); //Start index to store data in st_data reg
							temp_var <= 0;
							temp_count2 <= 0;
							temp_count <= 0;
							condition_bit <= 0;
							mem_write_no <= vcsr_vl >> 2; //No of mem writes required and derived from (reg_op2*vcsr_vl) >> 2 where stride is 1
							reg_op2 <= 1;
							// ind1 <= 0;
							// ind2 <= 0; //Used to index the mem strb
							//This is used when no of mem_reads crosses flat_reg_len bits
							new_no_words <= (flat_reg_len >> 8)*SEW;  //Derives from { flat_reg_len*SEW / (32*8*stride) } and stride is 1
							read_count <= 0;
							st_data <= 0;
							st_strb <= 0;
							mem_wordsize <= 2;
						end
						(instr_vstore_str): begin
							$display("Inside v_store_stride condition, time:%d", $time);
							cpu_state <= cpu_state_stmem;
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vd; //specifies v register holding store data
							vecregs_raddr2 <= decoded_vs2;
							vecregs_rstrb1 <= 16'b1;  //To read the first word
							mem_str_ready <= 0; //Initial value of mem_str_ready
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
							no_words <= ((vcsr_vl*SEW)>>5); //No of words to read from vec reg
							cnt <= (reg_op1[1:0] << 3); //Start index to store data in st_data reg
							temp_var <= 0;
							temp_count2 <= 0;
							temp_count <= 0;
							condition_bit <= 0;
							mem_write_no <= (reg_op2*vcsr_vl) >> 2;
							// ind1 <= 0; 
							// ind2 <= 0;
							//This is used when no of mem_reads crosses flat_reg_len bits
							new_no_words <= (reg_op2==0)? 64: (((flat_reg_len >> 8)*SEW) / reg_op2);  //Derives from { flat_reg_len*SEW / (32*8*stride) }
							read_count <= 0;
							st_data <= 0;
							st_strb <= 0;
							mem_wordsize <= 2;
						end
						(instr_vseuvarp): begin
							$display("Inside vap_unit_store condition, time:%d", $time);
							cpu_state <= cpu_state_stmem;
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vd; //specifies v register holding store data
							vecregs_raddr2 <= decoded_vs2;
							vecregs_rstrb1 <= 16'b1;  //To read the first word
							mem_str_ready <= 0; //Initial value of mem_str_ready
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:vap); 
							no_words <= ((vcsr_vl*vap)>>5); //No of words to read from vec reg
							cnt <= (reg_op1[1:0] << 3); //Start index to store data in st_data reg
							temp_var <= 0;
							temp_count2 <= 0;
							temp_count <= 0;
							condition_bit <= 0;
							mem_write_no <= vcsr_vl >> 2; //No of mem writes required
							reg_op2 <= 1;
							// ind1 <= 0;
							// ind2 <= 0; //Used to index the mem strb
							//This is used when no of mem_reads crosses flat_reg_len bits
							new_no_words <= (flat_reg_len >> 8)*SEW;  //Derives from { flat_reg_len*SEW / (32*8*stride) }
							read_count <= 0;
							st_data <= 0;
							st_strb <= 0;
							mem_wordsize <= 2;
						end
						(instr_vsesvarp): begin
							$display("Inside vap_strided_store condition, time:%d", $time);
							cpu_state <= cpu_state_stmem;
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vd; //specifies v register holding store data
							vecregs_raddr2 <= decoded_vs2;
							vecregs_rstrb1 <= 16'b1;  //To read the first word
							mem_str_ready <= 0; //Initial value of mem_str_ready
							//Updating mem_bits depending on vcsr_vl and v_enc_width(i.e instr[14:12])
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:vap); 
							no_words <= ((vcsr_vl*vap)>>5); //No of words to read from vec reg
							cnt <= (reg_op1[1:0] << 3); //Start index to store data in st_data reg
							temp_var <= 0;
							temp_count2 <= 0;
							temp_count <= 0;
							condition_bit <= 0;
							mem_write_no <= (str_bits[4:0]==0) ? (str_bits>>5): ((str_bits>>5)+1); //No of mem writes required (vcsr*stride*8/32)
							// ind1 <= 0;
							// ind2 <= 0; //Used to index the mem strb
							//This is used when no of mem_reads crosses flat_reg_len bits
							new_no_words <= (reg_op2==0)? 128 : (((flat_reg_len >> 8)*vap) / reg_op2);  //Derives from { flat_reg_len*SEW / (32*8*stride) }
							read_count <= 0;
							st_data <= 0;
							st_strb <= 0;
							mem_wordsize <= 2;
						end
						(instr_vadd || instr_vmul): begin
							$display("Instr vadd/vmulvec_reg1;%d, vec_reg2:%d, vec_vd:%d, time %d", decoded_vs1, decoded_vs2, decoded_vd, $time);
							cpu_state <= cpu_state_exec;
							if(instr_vadd)
								micro_exec_instr <= p_instr_vadd__vv;
							if(instr_vmul) begin
								micro_exec_instr <= p_instr_vmul__vv;
							end
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							vecregs_raddr2 <= decoded_vs2;
							vecregs_rstrb1 <= 16'b1;  //To read the first word
							arth_data_ready <= 0; //Initial value of arth_data_ready
							cnt <= 0;
							ind1 <= 0;
							elem_n <= 0;
							temp_var <= 0;
							temp_count <= 0;
							temp_count2 <= 0;
							condition_bit <= 0;
							no_words <= ((vcsr_vl*SEW)>>5); //No of words to read from vec reg
							read_count <= 0;
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
						end
						(instr_vaddvarp || instr_vmulvarp): begin
							$display("Instr_vaddvap/vmulvap vec_reg1;%d, vec_reg2:%d, vec_vd:%d, time %d", decoded_vs1, decoded_vs2, decoded_vd, $time);
							cpu_state <= cpu_state_exec;
							if(instr_vaddvarp)
								micro_exec_instr <= p_instr_vaddvarp;
							if(instr_vmulvarp)
								micro_exec_instr <= p_instr_vmulvarp;							
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							vecregs_raddr2 <= decoded_vs2;
							vecregs_rstrb1 <= 16'b1;  //To read the first word
							arth_data_ready <= 0; //Initial value of mem_str_ready
							// unpack_data <= 1;
							cnt <= 0;
							ind1 <= 0;
							elem_n <= 0;
							temp_var <= 0;
							temp_count <= 0;
							temp_count2 <= 0;
							condition_bit <= 0;
							//No of words to read from vec reg 
							no_words <= ((vcsr_vl*vap)>>5); 
							read_count <= 0;
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? vap:0); 
						end
						(instr_vdot): begin
							$display("Instr_vdot vec_reg1;%d, vec_reg2:%d, vec_vd:%d, time %d", decoded_vs1, decoded_vs2, decoded_vd, $time);
							cpu_state <= cpu_state_exec;
							micro_exec_instr <= p_instr_vdot__vv;
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							vecregs_raddr2 <= decoded_vs2;
							vecregs_rstrb1 <= 16'b1;  //To read the first word
							arth_data_ready <= 0;
							cnt <= 0;
							elem_n <= 0;
							ind1 <= 0;
							temp_var <= 0;
							temp_var3 <= 0;
							temp_count <= 0;
							temp_count2 <= 0;
                            temp_count3 <= 0;
							condition_bit <= 0;
                            unpack_index <= 0;
                            unpack_index2 <= 0;
							//No of words to read from vec reg 
							no_words <= ((vcsr_vl*SEW)>>5); 
							read_count <= 0;		
                            read_count2 <= 0;					
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:SEW); 
						end
						(instr_vdotvarp): begin
							$display("Instr_vdotvap vec_reg1;%d, vec_reg2:%d, vec_vd:%d, time %d", decoded_vs1, decoded_vs2, decoded_vd, $time);
							cpu_state <= cpu_state_exec;
							micro_exec_instr <= p_instr_vdotvarp;
							vecregs_waddr <= decoded_vd;
							vecregs_raddr1 <= decoded_vs1;
							vecregs_raddr2 <= decoded_vs2;
							vecregs_rstrb1 <= 16'b1;  //To read the first word
							arth_data_ready <= 0;
							cnt <= 0;
							elem_n <= 0;
							ind1 <= 0;
							temp_var <= 0;
                            temp_var3 <= 0;
							temp_count <= 0;
							temp_count2 <= 0;
                            temp_count3 <= 0;
							condition_bit <= 0;
                            unpack_index <= 0;
                            unpack_index2 <= 0;
							//No of words to read from vec reg 
							no_words <= ((vcsr_vl*vap)>>5); 
							read_count <= 0;
                            read_count2 <= 0;							
							// v_membits <= vcsr_vl * ((v_enc_width==3'b000)? 8:(v_enc_width==3'b101)?16:(v_enc_width==3'b110)?32:vap); 
							v_membits <= vcsr_vl * ((v_enc_width==3'b000)? vap:0);
						end
					endcase
				end 
				
				cpu_state_ldmem: begin
                    //Calculate for the first time and make valid as 1 
                    if(temp_var == 0) begin
						$display("Inside cnt=0 condition, mem_read_no:%d, no_words:%d, new_no_words:%d, time: %d",final_addr - init_addr + 1, no_words, new_no_words, $time);
						bits_remaining <= v_membits[4:0]; //remainder after dividing mem_bits with 32
						mem_read_no <= final_addr - init_addr + 1; //No of mem_reads required
						if(v_membits[4:0] > 0)
							no_words <= no_words+1;
						if(is_vap_instr)
							new_mem_read_no <= (new_no_words*8*reg_op2) / vap ;
						else
							new_mem_read_no <= (new_no_words*8*reg_op2) / SEW ;
					   	mem_valid <= 1; //Making the valid bit 1 after changing the address for the first time
						temp_var <= 1; //So that it enters this block only once
                    end
					if(mem_read_no*32 <= flat_reg_len) begin
						// $display("Inside if cond cnt=0 condition, mem_read_no:%d, no_words:%d, new_no_words:%d, time: %d",final_addr - init_addr + 1, no_words, new_no_words, $time);
						//New line added
						ind1 <= (reg_op1[1:0] << 3);
						if(mem_read_no >= 1) begin
							if(mem_ready == 1) begin //This is how memory works
							// $display("Inside mem-ready, time:%d", $time);
								reg_op1 = reg_op1 + 4;
							end
							if((mem_str_ready2 == 1)) begin 
								ld_data[cnt +: 32] <= mem_rdata_word;  //Selects 32 bits starting from cnt
								// $display("Inside if mem_addr:%d, mem_rdata: %x, mem_read_no: %d, time:%d",reg_op1>>2, mem_rdata_word, mem_read_no, $time);
								mem_read_no <= mem_read_no - 1;
								cnt <= cnt + 32;
								//If we are loading the last word, then go to ldmem2 stage to load the data into vector reg
								if(mem_read_no == 1) begin
									// $display("Inside mem_str_ready2,no_words:%d, ld_data :%x, time:%d",no_words, ld_data, $time);
									//Irrespective of SEW, the mem_wordsize will be 1 (used in mem FSM)
									mem_wordsize <= 1;
									mem_valid <= 0;
									mem_str_ready2 <= 0; //Will be made 1 again in mem_wordsize FSM
									mem_str_ready <= 0;  //Will be made 1 again in mem_wordsize FSM
									cpu_state <= cpu_state_ldmem2;
								end
							end
						end
					end
					else begin
						if(read_count < new_mem_read_no) begin
							if(mem_ready == 1) begin //This is how memory works
								reg_op1 = reg_op1 + 4;
							end
							if((mem_str_ready2 == 1)) begin 
								ld_data[cnt +: 32] <= mem_rdata_word;  //Selects 32 bits starting from cnt
								// $display("Inside else mem_addr:%d, mem_rdata: %x, read_count: %d, time:%d",reg_op1>>2, mem_rdata_word, read_count, $time);
								read_count <= read_count + 1;
								cnt <= cnt + 32;
								//If we are loading the last word, then go to ldmem2 stage to load the data into vector reg
								if(read_count == new_mem_read_no - 1) begin
									// $display("Inside read_count state, mem_read_no:%d, read_count :%d, time:%d",mem_read_no - new_mem_read_no, read_count, $time);
									//Irrespective of SEW, the mem_wordsize will be 1 (used in mem FSM)
									mem_wordsize <= 1;
									mem_valid <= 0;
									mem_str_ready2 <= 0; //Will be made 1 again in mem_wordsize FSM
									mem_str_ready <= 0;  //Will be made 1 again in mem_wordsize FSM
									cpu_state <= cpu_state_ldmem2;
									mem_read_no <= mem_read_no - new_mem_read_no;
									read_count <= 0;
									cnt <= 0; //To use the flat memory again for next iteration
								end
							end
						end
					end
					mem_addr = reg_op1; //Updating the mem_addr
				end
                cpu_state_ldmem2: begin
					if(no_words <= new_no_words) begin
						if(no_words > 0) begin
							if(mem_str_ready == 1) begin
							//    $display("Inside if ld_mem2, ld_data:%x, mem_rdata:%x, time: %d", ld_data[95:0], mem_rdata_word, $time);
								vreg_op1 <= mem_rdata_word;
								vecregs_wstrb_temp <= 1 << temp_count; //left shift temp count digits
								temp_count <= temp_count+1;
								latched_vstore <= 1;
								no_words <= no_words-1;
							end 
						end
					end
					else begin
						if(read_count < new_no_words) begin
							// $display("Entered else in ld_mem2 state, time:%d", $time);
							if(mem_str_ready == 1) begin
							//    $display("Inside else mem_str_ready ld_mem2, ld_data:%x, mem_rdata:%x, time: %d", ld_data[95:0], mem_rdata_word, $time);
								vreg_op1 <= mem_rdata_word;
								vecregs_wstrb_temp <= 1 << temp_count; //left shift temp count digits
								temp_count <= temp_count+1;
								latched_vstore <= 1;
								read_count <= read_count+1;									
								if(read_count == new_no_words-1) begin
									// $display("Inside read_cnt stage ld_mem2, read_cnt:%d, ind1:%d, time: %d", read_count, (reg_op1[1:0] << 3), $time);
									mem_wordsize <= 0; //Again read data from main memory and store it in flat register
									mem_str_ready <= 0; //Will be made 1 again in mem_wordsize FSM
									cpu_state <= cpu_state_ldmem;
									no_words <= no_words - new_no_words;
									read_count <= 0;
									mem_valid <= 1; //Enabling memory again
								end
							end 
						end
					end
                    if(no_words == 0) begin
						// $display("V_membits:%d, bits remaining after words: %d, time:%d", v_membits, bits_remaining, $time);
						mem_str_ready <= 0; //Will be made 1 again in mem_wordsize FSM
						cpu_state <= cpu_state_fetch;
						pcpi_wait <= 0;
						pcpi_ready <= 1;
                    end
                end
				cpu_state_stmem: begin
                    //Calculate for the first time and make valid as 1 
                    if(temp_var == 0) begin
						$display("Inside cnt=0 condition, mem_read_no:%d, no_words:%d, new_no_words:%d, time: %d",final_addr - init_addr + 1, no_words, new_no_words, $time);
						bits_remaining <= v_membits[4:0]; //Remainder after dividing mem_bits with 32
						// mem_wordsize <= 2;
						condition_bit <= 1;
						if(v_membits[4:0] > 0)
							no_words <= no_words+1;
						if(is_vap_instr) begin
							if(str_bits[4:0]+vap > (32-cnt[4:0]))    //cnt is the start index to store data in st_data reg
								mem_write_no <= mem_write_no + 1;
							new_mem_write_no <= (new_no_words*8*reg_op2) / vap ;
						end
						else begin
							// $display("remaining bits:%d, cnt[4:0]:%d, time:%d", str_bits[4:0], cnt[4:0], $time);
							if((str_bits[4:0]+SEW) > (32-cnt[4:0]))  begin  //cnt is the start index to store data in st_data reg
								mem_write_no <= mem_write_no + 1;
							end
							new_mem_write_no <= (new_no_words*8*reg_op2) / SEW ;
						end
						temp_var <= 1; //So that it enters this block only once
                    end
					if(str_bits < flat_reg_len) begin
						if(read_count < no_words) begin
							if((mem_str_ready == 1)) begin 
								// $display("Inside if mem_str_ready, no_words:%d, read_count:%d, time:%d",no_words,read_count, $time);
								read_count <= read_count + 1;
								temp_count2 <= temp_count2 + 1;
								condition_bit <= 1;
								mem_str_ready <= 0;
								//If we are loading the last word, then go to stmem2 stage to load the data into vector reg
								if(read_count == no_words-1) begin
									//Irrespective of SEW, the mem_wordsize will be 1 (used in mem FSM)
									ind1 <= 0;
									ind2 <= 0;
									mem_wordsize <= 3;
									mem_valid <= 1;
									cpu_state <= cpu_state_stmem2;
								end
							end
						end
					end
					else begin
						if(read_count < new_no_words) begin
							if((mem_str_ready == 1)) begin 
								// $display("Inside else mem_str_ready, new_mem_write_no:%d, time:%d", new_mem_write_no, $time);
								read_count <= read_count + 1;
								temp_count2 <= temp_count2 + 1;
								condition_bit <= 1;
								mem_str_ready <= 0;
								//If we are loading the last word, then go to stmem2 stage to load the data into vector reg
								if(read_count == new_no_words-1) begin
									//Irrespective of SEW, the mem_wordsize will be 1 (used in mem FSM)
									ind1 <= 0;
									ind2 <= 0;
									mem_wordsize <= 3;
									mem_valid <= 1;
									cpu_state <= cpu_state_stmem2;
									no_words <= no_words - new_no_words;
									read_count <= 0;
									str_bits <= str_bits - new_mem_write_no*32;
								end
							end
						end
					end
				end
				cpu_state_stmem2: begin
					if(mem_write_no <= new_mem_write_no) begin
						// $display("Inside if st_mem2 mem_str_ready, str_bits:%d, mem_write_no:%d, new_mem_write_no:%d, st_data:%x, time:%d",str_bits, mem_write_no, new_mem_write_no, st_data, $time);
						if(mem_str_ready == 1) begin
							if(mem_write_no > 1) begin
								set_mem_do_wdata = 1;
								reg_op1 <= reg_op1 + 4;
								mem_write_no <= mem_write_no - 1;
							end
							else if(mem_write_no == 1) begin
								$display("pcpi_ready condition: %d", $time);
								mem_wordsize <= 0;
								mem_valid <= 0;
								pcpi_wait <= 0;
								pcpi_ready <= 1;
								cpu_state <= cpu_state_fetch;
							end
						end
					end
					else begin
						// $display("Inside else st_mem2 mem_str_ready, mem_write_no:%d, new_mem_write_no:%d time:%d",mem_write_no, new_mem_write_no, $time);
						if(mem_str_ready == 1) begin
							if(read_count < new_mem_write_no) begin
								read_count <= read_count + 1;
								set_mem_do_wdata = 1;
								reg_op1 <= reg_op1 + 4;
							end
							else if(read_count == new_mem_write_no) begin
								// $display("Entered read_count == new_mem_write_no condition, time:%d", $time);
								cpu_state <= cpu_state_stmem;
								mem_wordsize <= 2;
								mem_valid <= 0;
								mem_str_ready <= 0;
								read_count <= 0;
								mem_write_no <= mem_write_no - new_mem_write_no;
								cnt <= (reg_op1[1:0] << 3); //To use flat memory again for next iteration
							end
						end
					end
					mem_addr <= reg_op1;
				end
				cpu_state_exec: begin
					if(temp_var == 0) begin
						$display("Inside cnt=0 condition, is_port3_insn:%b, no_words:%d, v_membits:%d, time: %d",is_port3_instr, no_words, v_membits[4:0], $time);
						unpack_data <= 1;  //We have to read three registers here 
						bits_remaining <= v_membits[4:0]; //Remainder after dividing mem_bits with 32
						condition_bit <= 1;   //Used to synchronize the data forwarding
						if(v_membits[4:0] > 0)
							no_words <= no_words+1;
						temp_var <= 1; //So that it enters this block only once
                    end
					else begin
                        if((read_count < no_words) && (unpack_index <= 512)) begin
                            if(arth_data_ready == 1) begin 
                                // $display("Inside if arth_data_ready,temp_count2:%b, unpacked data A: %x, unpacked data B: %x, time:%d",temp_count2, opA[127:0], opB[127:0], $time);
                                read_count <= read_count + 1;
                                temp_count2 <= temp_count2 + 1;
                                condition_bit <= 1;
                                arth_data_ready <= 0;
                                if( ((read_count == no_words-1) || (unpack_index == 512)) && !is_port3_instr) begin
                                    unpack_data <= 0;
                                    alu_enb <= 1;
                                end
                                else if(((read_count == no_words-1) || (unpack_index == 512)) && is_port3_instr) begin
                                    vecregs_raddr1 <= decoded_vd;
                                    vecregs_rstrb1 <= 1;
                                    port3_data <= 1;
                                    condition_bit <= 0;
                                end 
                            end
                        end
                        if((read_count == no_words) && is_port3_instr) begin
                            if((read_count2 < no_words) && (unpack_index2 <= 512)) begin
                                //To generate 1 clk delay so that the opC data will be available
                                if(!temp_var3) begin
                                    condition_bit <= 1;
                                    temp_var3 <= 1;
                                end
                                if(arth_data_ready == 1) begin 
                                    // $display("Inside if arth_data_ready,read_count2: %d, temp_count3:%b, unpacked data C: %x, time:%d", read_count2, temp_count3, opC[127:0], $time);
                                    read_count2 <= read_count2 + 1;
                                    temp_count3 <= temp_count3 + 1;
                                    condition_bit <= 1;
                                    arth_data_ready <= 0;
                                    if( ((read_count2 == no_words-1) || (unpack_index2 == 512))) begin
                                        unpack_data <= 0;
                                        alu_enb <= 1;
                                    end
                                end
                            end
                        end
                        if((read_count == no_words) && ((!is_port3_instr) || ((read_count2 == no_words) && is_port3_instr))) begin
                            if(alu_done) begin
                                if(vap == 2) begin
									$display("\nopA:%b, \nopB:%b, \nopC:%b", opA[95:0], opB[95:0], opC[95:0]);
									$display("out:%b, time:%d\n", alu_out[95:0], $time);
								end
								else begin
									$display("\nopA:%x, \nopB:%x, \nopC:%x", opA[511:0], opB[511:0], opC[511:0]);
									$display("out:%x, time:%d\n", alu_out[511:0], $time);
								end
                                port3_data <= 0;
                                unpack_data <= 2;
                                ind1 <= 0;
                                alu_enb <= 0;
                                cpu_state <= cpu_state_exec2;
                            end
                        end
					end
				end
				cpu_state_exec2: begin
					if(no_words > 0) begin
						if(arth_data_ready == 1) begin
						//    $display("Inside exec2,alu_out:%x, alu_enb:%b, alu_wdata:%x, time: %d",alu_out[127:0], alu_enb, alu_wdata, $time);
							vreg_op1 <= alu_wdata;
							vecregs_wstrb_temp <= 1 << temp_count; //left shift temp count digits
							temp_count <= temp_count+1;
							latched_vstore <= 1;
							no_words <= no_words-1;
						end 
					end
					if(no_words == 0) begin
						// $display("V_membits:%d, bits remaining after words: %d, time:%d", v_membits, bits_remaining, $time);
						unpack_data <= 0;
						arth_data_ready <= 0; //Will be made 1 again in mem_wordsize FSM
						cpu_state <= cpu_state_fetch;
						pcpi_wait <= 0;
						pcpi_ready <= 1;
                    end
				end
			endcase
		end
		if (set_mem_do_rdata)
			mem_do_rdata <= 1;
		if (set_mem_do_wdata)
			mem_do_wdata <= 1;
	end

 	vector_processing_element pe1(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done1),.opA(opA[511:480]),.opB(opB[511:480]),.opC(opC[511:480]),.peout(alu_out[511:480]),.SEW(SEW),.vap(vap));
    vector_processing_element pe2(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done2),.opA(opA[479:448]),.opB(opB[479:448]),.opC(opC[479:448]),.peout(alu_out[479:448]),.SEW(SEW),.vap(vap));
    vector_processing_element pe3(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done3),.opA(opA[447:416]),.opB(opB[447:416]),.opC(opC[447:416]),.peout(alu_out[447:416]),.SEW(SEW),.vap(vap));
    vector_processing_element pe4(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done4),.opA(opA[415:384]),.opB(opB[415:384]),.opC(opC[415:384]),.peout(alu_out[415:384]),.SEW(SEW),.vap(vap));
    vector_processing_element pe5(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done5),.opA(opA[383:352]),.opB(opB[383:352]),.opC(opC[383:352]),.peout(alu_out[383:352]),.SEW(SEW),.vap(vap));
    vector_processing_element pe6(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done6),.opA(opA[351:320]),.opB(opB[351:320]),.opC(opC[351:320]),.peout(alu_out[351:320]),.SEW(SEW),.vap(vap));
    vector_processing_element pe7(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done7),.opA(opA[319:288]),.opB(opB[319:288]),.opC(opC[319:288]),.peout(alu_out[319:288]),.SEW(SEW),.vap(vap));
    vector_processing_element pe8(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done8),.opA(opA[287:256]),.opB(opB[287:256]),.opC(opC[287:256]),.peout(alu_out[287:256]),.SEW(SEW),.vap(vap));
    vector_processing_element pe9(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done9),.opA(opA[255:224]),.opB(opB[255:224]),.opC(opC[255:224]),.peout(alu_out[255:224]),.SEW(SEW),.vap(vap));
    vector_processing_element pe10(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done10),.opA(opA[223:192]),.opB(opB[223:192]),.opC(opC[223:192]),.peout(alu_out[223:192]),.SEW(SEW),.vap(vap));
    vector_processing_element pe11(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done11),.opA(opA[191:160]),.opB(opB[191:160]),.opC(opC[191:160]),.peout(alu_out[191:160]),.SEW(SEW),.vap(vap));
    vector_processing_element pe12(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done12),.opA(opA[159:128]),.opB(opB[159:128]),.opC(opC[159:128]),.peout(alu_out[159:128]),.SEW(SEW),.vap(vap));
    vector_processing_element pe13(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done13),.opA(opA[127:96]),.opB(opB[127:96]),.opC(opC[127:96]),.peout(alu_out[127:96]),.SEW(SEW),.vap(vap));
    vector_processing_element pe14(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done14),.opA(opA[95:64]),.opB(opB[95:64]),.opC(opC[95:64]),.peout(alu_out[95:64]),.SEW(SEW),.vap(vap));
    vector_processing_element pe15(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done15),.opA(opA[63:32]),.opB(opB[63:32]),.opC(opC[63:32]),.peout(alu_out[63:32]),.SEW(SEW),.vap(vap));
    vector_processing_element pe16(.clk(clk),.reset(resetn),.instruction(micro_exec_instr),.start(alu_enb),.done(done16),.opA(opA[31:0]),.opB(opB[31:0]),.opC(opC[31:0]),.peout(alu_out[31:0]),.SEW(SEW),.vap(vap));


endmodule