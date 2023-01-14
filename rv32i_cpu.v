//
//  Author: Prof. Taeweon Suh
//          Computer Science & Engineering
//          Korea University
//  Date: July 14, 2020
//  Description: Skeleton design of RV32I Single-cycle CPU
//

`timescale 1ns/1ns
`define simdelay 1

module rv32i_cpu (
		      input         clk, reset,
            output [31:0] pc,		  		// program counter for instruction fetch
            input  [31:0] inst, 			// incoming instruction
            output        Memwrite, 	// 'memory write' control signal
            output [31:0] Memaddr,  	// memory address 
            output [31:0] MemWdata, 	// data to write to memory
            input  [31:0] MemRdata); 	// data read from memory

  wire        auipc, lui;
  wire        alusrc, regwrite;
  wire [4:0]  alucontrol;
  wire        memtoreg, memwrite;
  wire        branch, jal, jalr;

  // ######  Minseong Bae : Start  #######
  // assign Memwrite = memwrite ;
  wire [31:0] IF_ID_inst;
  wire 		  memread;	//'memory read' control signal
  // ######  Minseong Bae : End  #######
  
  // Instantiate Controller
  controller i_controller(
		// ######  Minseong Bae : Start  #######
      .opcode		(IF_ID_inst[6:0]), 
		.funct7		(IF_ID_inst[31:25]), 
		.funct3		(IF_ID_inst[14:12]), 
		.memread		(memread),
		// ######  Minseong Bae : End  #######
		.auipc		(auipc),
		.lui			(lui),
		.memtoreg	(memtoreg),
		.memwrite	(memwrite),
		.branch		(branch),
		.alusrc		(alusrc),
		.regwrite	(regwrite),
		.jal			(jal),
		.jalr			(jalr),
		.alucontrol	(alucontrol));

  // Instantiate Datapath
  datapath i_datapath(
		.clk				(clk),
		.reset			(reset),
		.auipc			(auipc),
		.lui				(lui),
		.memtoreg		(memtoreg),
		.memwrite		(memwrite),
		.branch			(branch),
		.alusrc			(alusrc),
		.regwrite		(regwrite),
		.jal				(jal),
		.jalr				(jalr),
		.alucontrol		(alucontrol),
		.pc				(pc),
		.inst				(inst),
		.MemWdata		(MemWdata),
		.MemRdata		(MemRdata),
		// ######  Minseong Bae : Start  #######
		.memread			  (memread),
		.EX_MEM_aluout	  (Memaddr), 
		.IF_ID_inst      (IF_ID_inst),
		.EX_MEM_memwrite (Memwrite));
		// ######  Minseong Bae : End  #######

endmodule


//
// Instruction Decoder 
// to generate control signals for datapath
//
module controller(input  [6:0] opcode,
                  input  [6:0] funct7,
                  input  [2:0] funct3,
                  output       auipc,
                  output       lui,
                  output       alusrc,
                  output [4:0] alucontrol,
                  output       branch,
                  output       jal,
                  output       jalr,
                  output       memtoreg,
                  output       memwrite,
                  output       regwrite,
		// ######  Minseong Bae : Start  #######
						output		 memread);
		// ######  Minseong Bae : End  #######
		
	maindec i_maindec(
		.opcode		(opcode),
		.auipc		(auipc),
		.lui			(lui),
		.memtoreg	(memtoreg),
		.memwrite	(memwrite),
		.branch		(branch),
		.alusrc		(alusrc),
		.regwrite	(regwrite),
		.jal			(jal),
		.jalr			(jalr),
		// ######  Minseong Bae : Start  #######
		.memread 	(memread));
		// ######  Minseong Bae : End  #######
		
	aludec i_aludec( 
		.opcode     (opcode),
		.funct7     (funct7),
		.funct3     (funct3),
		.alucontrol (alucontrol));


endmodule


//
// RV32I Opcode map = Inst[6:0]
//
`define OP_R			7'b0110011
`define OP_I_ARITH	7'b0010011
`define OP_I_LOAD  	7'b0000011
`define OP_I_JALR  	7'b1100111
`define OP_S			7'b0100011
`define OP_B			7'b1100011
`define OP_U_LUI		7'b0110111
`define OP_J_JAL		7'b1101111

//
// Main decoder generates all control signals except alucontrol 
//
//
module maindec(input  [6:0] opcode,
               output       auipc,
               output       lui,
               output       regwrite,
               output       alusrc,
					// ######  Minseong Bae : Start  #######
               output       memtoreg, memwrite, memread,
					// ######  Minseong Bae : End  #######
               output       branch, 
               output       jal,
               output       jalr);
	
  // ######  Minseong Bae : Start  #######
  reg [9:0] controls;

  assign {auipc, lui, regwrite, alusrc, 
			 memtoreg, memwrite, memread, branch, jal, 
			 jalr} = controls;

  always @(*)
  begin
    case(opcode)
      `OP_R: 			controls <= #`simdelay 10'b0010_0000_00; // R-type
      `OP_I_ARITH: 	controls <= #`simdelay 10'b0011_0000_00; // I-type Arithmetic
      `OP_I_LOAD: 	controls <= #`simdelay 10'b0011_1010_00; // I-type Load : memread = 1
		`OP_I_JALR:    controls <= #`simdelay 10'b0011_0000_01; // jalr
      `OP_S: 			controls <= #`simdelay 10'b0001_0100_00; // S-type Store
      `OP_B: 			controls <= #`simdelay 10'b0000_0001_00; // B-type Branch
      `OP_U_LUI: 		controls <= #`simdelay 10'b0111_0000_00; // LUI
      `OP_J_JAL: 		controls <= #`simdelay 10'b0011_0000_10; // JAL
      default:    	controls <= #`simdelay 10'b0000_0000_00; // ???
    endcase
  end

endmodule

//
// ALU decoder generates ALU control signal (alucontrol)
//
module aludec(input      [6:0] opcode,
              input      [6:0] funct7,
              input      [2:0] funct3,
              output reg [4:0] alucontrol);

  always @(*)

    case(opcode)

      `OP_R:   		// R-type
		begin
			case({funct7,funct3})
			 10'b0000000_000: alucontrol <= #`simdelay 5'b00000; // addition (add)
			 10'b0100000_000: alucontrol <= #`simdelay 5'b10000; // subtraction (sub)
			 10'b0000000_111: alucontrol <= #`simdelay 5'b00001; // and (and)
			 10'b0000000_110: alucontrol <= #`simdelay 5'b00010; // or (or)
          default:         alucontrol <= #`simdelay 5'bxxxxx; // ???
        endcase
		end

      `OP_I_ARITH:   // I-type Arithmetic
		begin
			case(funct3)
			 3'b000:  alucontrol <= #`simdelay 5'b00000; // addition (addi)
			 // ######  Minseong Bae : Start  #######
			 3'b001:  alucontrol <= #`simdelay 5'b00100; // shift left (slli)
			 // ######  Minseong Bae : End  #######
			 3'b100:  alucontrol <= #`simdelay 5'b00011;	// xor (xori)
			 3'b110:  alucontrol <= #`simdelay 5'b00010; // or (ori)
			 3'b111:  alucontrol <= #`simdelay 5'b00001; // and (andi)
          default: alucontrol <= #`simdelay 5'bxxxxx; // ???
        endcase
		end

      `OP_I_LOAD: 	// I-type Load (LW, LH, LB...)
      	alucontrol <= #`simdelay 5'b00000;  // addition 
			
		`OP_I_JALR: 	// jalr
      	alucontrol <= #`simdelay 5'b00000;  // addition

      `OP_B:   		// B-type Branch (BEQ, BNE, ...)
      	alucontrol <= #`simdelay 5'b10000;  // subtraction 

      `OP_S:   		// S-type Store (SW, SH, SB)
      	alucontrol <= #`simdelay 5'b00000;  // addition 

      `OP_U_LUI: 		// U-type (LUI)
      	alucontrol <= #`simdelay 5'b00000;  // addition

      default: 
      	alucontrol <= #`simdelay 5'b00000;  // 

    endcase
    
endmodule


//
// CPU datapath
//
module datapath(input         clk, reset,
                input  [31:0] inst,
                input         auipc,
                input         lui,
                input         regwrite,
                input         memtoreg,
                input         memwrite,
                input         alusrc, 
                input  [4:0]  alucontrol,
                input         branch,
                input         jal,
                input         jalr,
                output reg [31:0] pc,
					 output 		[31:0] MemWdata,
                input  		[31:0] MemRdata,
					 // ######  Minseong Bae : Start  #######
					 input 				 memread,
                output reg [31:0] EX_MEM_aluout,
					 output reg			 EX_MEM_memwrite,
					 output reg [31:0] IF_ID_inst);
					 // ######  Minseong Bae : End  #######

  wire [4:0]  rs1, rs2, rd;
  wire [2:0]  funct3;
  // ######  Minseong Bae : Start  #######
  reg [31:0] rs1_data, rs2_data;
  // ######  Minseong Bae : End  #######
  reg  [31:0] rd_data;
  wire [20:1] jal_imm;
  wire [31:0] se_jal_imm;
  wire [12:1] br_imm;
  wire [31:0] se_br_imm;
  wire [31:0] se_imm_itype;
  wire [31:0] se_imm_stype;
  wire [31:0] auipc_lui_imm;
  reg  [31:0] alusrc1;
  reg  [31:0] alusrc2;
  wire [31:0] branch_dest, jal_dest, jalr_dest;
  wire		  Nflag, Zflag, Cflag, Vflag;
  wire		  f3beq, f3blt, f3bgeu;
  // ######  Minseong Bae : Start (5th Milestone)  #######
  wire 		  f3bne;
  wire 		  branch_taken; // integrated branch taken signal for branch instructions
  wire		  bne_taken;
  // ######  Minseong Bae : End (5th Milestone)  #######
  wire		  beq_taken;
  wire		  blt_taken;
  wire		  bgeu_taken;
  // ######  Minseong Bae : Start (Last Milestone)  #######
  wire 		  f3bge, f3bltu;
  wire 		  bge_taken, bltu_taken;
  // ######  Minseong Bae : End (Last Milestone)  #######
  
  // ######  Minseong Bae : Start   #######
  assign rs1 = IF_ID_inst[19:15];
  assign rs2 = IF_ID_inst[24:20];
  assign rd  = IF_ID_inst[11:7];
  
  wire stall;
  // ######  Minseong Bae : Start (5th Milestone) #######
  wire IF_flush;
  // ######  Minseong Bae : End (5th Milestone) #######
  
  
  wire [31:0] rf_rs1_data, rf_rs2_data;
  wire [31:0] aluout;
  wire forward_alusrc1_EX_MEM, forward_alusrc1_MEM_WB, forward_alusrc2_EX_MEM, forward_alusrc2_MEM_WB;
  // ######  Minseong Bae : Start (Last Milestone)  #######
  wire forward_rs1_ID_EX, forward_rs1_EX_MEM, forward_rs1_memrdata, forward_rs1_MEM_WB, forward_rs2_ID_EX, forward_rs2_EX_MEM, forward_rs2_memrdata, forward_rs2_MEM_WB;
  // ######  Minseong Bae : End (Last Milestone)  #######
  
  // FF for IF/ID
  reg [31:0] IF_ID_pc;
  always @(posedge clk)
  begin 
	if (stall)
		begin
			IF_ID_inst <= IF_ID_inst;
			IF_ID_pc   <= IF_ID_pc;
		end
   // ######  Minseong Bae : Start (5th Milestone) #######
	else if (IF_flush)
		begin
			IF_ID_inst <= 8'h13; // nop : addi x0, x0, 0
			IF_ID_pc   <= pc;
		end
	// ######  Minseong Bae : End (5th Milestone) #######
	else
		begin
			IF_ID_inst <= inst;
			IF_ID_pc   <= pc;
		end
  end
	
  // FF for ID/EX
	
  reg ID_EX_auipc, ID_EX_lui, ID_EX_regwrite, ID_EX_alusrc, ID_EX_memtoreg, ID_EX_memwrite, ID_EX_branch, ID_EX_jal, ID_EX_jalr, ID_EX_memread;
  reg [4:0]  ID_EX_rs1, ID_EX_rs2, ID_EX_rd, ID_EX_alucontrol;
  reg [31:0] ID_EX_inst, ID_EX_rs1_data, ID_EX_rs2_data, ID_EX_pc, ID_EX_jal_imm, ID_EX_br_imm, ID_EX_imm_itype, ID_EX_imm_stype, ID_EX_auipc_lui_imm;
  
  always @(posedge clk)
  begin
      // ######  Minseong Bae : Start (5th Milestone) #######
		if (stall | IF_flush)
	   // ######  Minseong Bae : End (5th Milestone) #######
			begin
				ID_EX_auipc			<= 1'b0;
				ID_EX_lui			<= 1'b0;
				ID_EX_regwrite 	<= 1'b0;
				ID_EX_alusrc   	<= 1'b0;
				ID_EX_memtoreg 	<= 1'b0;
				ID_EX_memwrite 	<= 1'b0;
				ID_EX_branch   	<= 1'b0;
				ID_EX_jal      	<= 1'b0;
				ID_EX_jalr     	<= 1'b0;
				ID_EX_memread  	<= 1'b0;
				ID_EX_alucontrol	<= 5'b0;
				ID_EX_inst			<= 32'b0;
			end
		else
			begin
				ID_EX_auipc				<= auipc;	
				ID_EX_lui				<= lui;
				ID_EX_regwrite			<= regwrite;
				ID_EX_alusrc			<= alusrc;
				ID_EX_memtoreg			<= memtoreg;
				ID_EX_memwrite			<= memwrite;
				ID_EX_branch			<= branch;
				ID_EX_jal				<= jal;
				ID_EX_jalr				<= jalr;
				ID_EX_memread			<= memread;
				ID_EX_rs1				<= rs1;
				ID_EX_rs2				<= rs2;
				ID_EX_rd					<= rd;
				ID_EX_alucontrol		<= alucontrol;
				ID_EX_rs1_data			<= rs1_data;
				ID_EX_rs2_data			<= rs2_data;
				ID_EX_inst           <= IF_ID_inst;
				ID_EX_pc					<= IF_ID_pc;
				ID_EX_jal_imm			<= se_jal_imm;
				ID_EX_br_imm			<= se_br_imm;
				ID_EX_imm_itype		<= se_imm_itype;
				ID_EX_imm_stype		<= se_imm_stype;
				ID_EX_auipc_lui_imm  <= auipc_lui_imm;
			end
  end
	
  // FF for EX/MEM
  
  // ######  Minseong Bae : Start (5th Milestone)  #######
  // branch / jump destination calcuation in EX so don't need to forward some signals
  reg        EX_MEM_regwrite, EX_MEM_memtoreg, EX_MEM_jal, EX_MEM_jalr, EX_MEM_memread;
  reg [31:0] EX_MEM_rs2_data, EX_MEM_pc;
  reg [4:0]  EX_MEM_rd;
  
  always @(posedge clk)
  begin
	EX_MEM_regwrite		<= ID_EX_regwrite;
	EX_MEM_memwrite		<= ID_EX_memwrite;
	EX_MEM_memtoreg		<= ID_EX_memtoreg;
	EX_MEM_jal				<= ID_EX_jal;
	EX_MEM_jalr				<= ID_EX_jalr;
	EX_MEM_memread			<= ID_EX_memread;
	EX_MEM_rs2_data		<= ID_EX_rs2_data;
	EX_MEM_pc 				<= ID_EX_pc;
	EX_MEM_rd				<= ID_EX_rd;
	EX_MEM_aluout			<= aluout;
  end
  // ######  Minseong Bae : End (5th Milestone)  #######
  
  // FF for MEM/WB
  
  reg 			MEM_WB_regwrite, MEM_WB_memtoreg, MEM_WB_jal, MEM_WB_jalr;
  reg [31:0] 	MEM_WB_MemRData, MEM_WB_aluout, MEM_WB_pc;
  reg [4:0]    MEM_WB_rd;
  
  always @(posedge clk)
  begin
	MEM_WB_regwrite	<= EX_MEM_regwrite;
	MEM_WB_memtoreg	<= EX_MEM_memtoreg;
	MEM_WB_jal			<= EX_MEM_jal;
	MEM_WB_jalr			<= EX_MEM_jalr;
	MEM_WB_MemRData	<= MemRdata;
	MEM_WB_aluout		<= EX_MEM_aluout;
	MEM_WB_pc			<= EX_MEM_pc;
	MEM_WB_rd			<= EX_MEM_rd;
  end
	
  // (data) hazard detection unit : create stall signal
  assign stall = (ID_EX_memread & (ID_EX_rd != 5'b0) & ((ID_EX_rd == rs1)||(ID_EX_rd == rs2)));
  
  // ######  Minseong Bae : Start (5th Milestone) #######
  
  // IF_flush signal
  assign IF_flush = (branch_taken | ID_EX_jal | ID_EX_jalr);
  
  // ######  Minseong Bae : End (5th Milestone) #######
	
  
  // forwarding unit : create control signals for multiplexers
  
  assign forward_alusrc1_EX_MEM = (EX_MEM_regwrite) & (EX_MEM_rd != 5'b0) & (ID_EX_rs1 == EX_MEM_rd); // M -> E (rs1)
  assign forward_alusrc1_MEM_WB = (MEM_WB_regwrite) & (MEM_WB_rd != 5'b0) & (ID_EX_rs1 == MEM_WB_rd) & (~((EX_MEM_regwrite) & (EX_MEM_rd != 5'b0) & (ID_EX_rs1 == EX_MEM_rd))); // W -> E (rs1)
  assign forward_alusrc2_EX_MEM = (EX_MEM_regwrite) & (EX_MEM_rd != 5'b0) & (ID_EX_rs2 == EX_MEM_rd) & (~ID_EX_alusrc); // M -> E (rs2)
  assign forward_alusrc2_MEM_WB = (MEM_WB_regwrite) & (MEM_WB_rd != 5'b0) & (ID_EX_rs2 == MEM_WB_rd) & (~((EX_MEM_regwrite) & (EX_MEM_rd != 5'b0) & (ID_EX_rs2 == EX_MEM_rd))) & (~ID_EX_alusrc); // W -> E (rs2)
  // ######  Minseong Bae : Start (Last Milestone) #######
  assign	forward_rs1_ID_EX      = (~ID_EX_memwrite) & (rs1 != 5'b0) & (rs1 == ID_EX_rd);  // E -> D (rs1)
  assign	forward_rs2_ID_EX      = (~ID_EX_memwrite) & (rs2 != 5'b0) & (rs2 == ID_EX_rd);  // E -> D (rs2)
  // ######  Minseong Bae : End (Last Milestone) #######
  assign forward_rs1_EX_MEM	  = (~EX_MEM_memread) & (rs1 != 5'b0) & (rs1 == EX_MEM_rd); // M -> D (rs1)
  assign forward_rs1_memrdata   = (EX_MEM_memread)  & (rs1 != 5'b0) & (rs1 == EX_MEM_rd);	// M -> D (M is lw) (rs1)
  assign forward_rs1_MEM_WB     = (rs1 != 5'b0) & (rs1 == MEM_WB_rd); // W -> D (rs1)
  assign forward_rs2_EX_MEM     = (~EX_MEM_memread) & (rs2 != 5'b0) & (rs2 == EX_MEM_rd); // M -> D (rs2)
  assign forward_rs2_memrdata   = (EX_MEM_memread) & (rs2 != 5'b0) & (rs2 == EX_MEM_rd);  // M -> D (M is lw) (rs2)
  assign forward_rs2_MEM_WB     = (rs2 != 5'b0) & (rs2 == MEM_WB_rd); // W -> D (rs2)
  
  // select rs1_data and rs2_data (forwarding)
  // ######  Minseong Bae : Start (Last Milestone) #######
  always @(*)
  begin
   if			(forward_rs1_ID_EX)     rs1_data = aluout;
	else if  (forward_rs1_EX_MEM)    rs1_data = EX_MEM_aluout;
	else if  (forward_rs1_memrdata)  rs1_data = MemRdata;
   else if  (forward_rs1_MEM_WB)    rs1_data = rd_data;
   else                   	 			rs1_data = rf_rs1_data;
  end
  
  always @(*)
  begin
   if			(forward_rs2_ID_EX)     rs2_data = aluout;
	else if  (forward_rs2_EX_MEM)		rs2_data = EX_MEM_aluout;
	else if  (forward_rs2_memrdata)  rs2_data = MemRdata;
   else if  (forward_rs2_MEM_WB)    rs2_data = rd_data;
   else                   	 			rs2_data = rf_rs2_data;
  end
  // ######  Minseong Bae : End (Last Milestone) #######
  // ######  Minseong Bae : End  #######

  //
  // PC (Program Counter) logic 
  //
  
  // ######  Minseong Bae : Start  #######
	
  assign funct3 = ID_EX_inst[14:12];
  
  assign f3beq  = (funct3 == 3'b000);
  assign f3blt  = (funct3 == 3'b100);
  assign f3bgeu = (funct3 == 3'b111); // funct3 for bgeu
  // ######  Minseong Bae : Start (5th Milestone)  #######
  assign f3bne  = (funct3 == 3'b001); 
  
  // ######  Minseong Bae : Start (Last Milestone)  #######
  assign f3bge  = (funct3 == 3'b101);
  assign f3bltu = (funct3 == 3'b110);
  
  assign bge_taken  =  ID_EX_branch & f3bge & (Nflag == Vflag);
  assign bltu_taken =  ID_EX_branch & f3bltu & (~Cflag);
  // ######  Minseong Bae : End (Last Milestone)  #######
  
  assign beq_taken  =  ID_EX_branch & f3beq & Zflag;
  assign blt_taken  =  ID_EX_branch & f3blt & (Nflag != Vflag);
  assign bgeu_taken =  ID_EX_branch & f3bgeu & Cflag; // condition for bgeu
  assign bne_taken  =  ID_EX_branch & f3bne & (~Zflag);
  
  // ######  Minseong Bae : Start (Last Milestone)  #######
  assign branch_taken = (beq_taken | blt_taken | bgeu_taken | bne_taken | bge_taken | bltu_taken);
  // ######  Minseong Bae : End (Last Milestone)  #######
  
  // ######  Minseong Bae : End (5th Milestone)  #######

  assign branch_dest = (ID_EX_pc + ID_EX_br_imm);
  assign jal_dest 	= (ID_EX_pc + ID_EX_jal_imm);
  assign jalr_dest	= {aluout[31:1],1'b0}; // destination for jalr

  always @(posedge clk, posedge reset)
  begin
     if (reset)  pc <= 32'b0;
	  else 
	  begin
			// ######  Minseong Bae : Start (5th Milestone)  #######
	      if (branch_taken) // branch
				pc <= #`simdelay branch_dest;
		   else if (ID_EX_jal) // jal
				pc <= #`simdelay jal_dest;
			else if (ID_EX_jalr) // jalr
				pc <= #`simdelay jalr_dest;
			// ######  Minseong Bae : End (5th Milestone)  #######
			else if (stall)
				pc <= #`simdelay pc;
		   else 
				pc <= #`simdelay (pc + 4);
	  end
  end

  // JAL immediate
  assign jal_imm[20:1] = {IF_ID_inst[31],IF_ID_inst[19:12],IF_ID_inst[20],IF_ID_inst[30:21]};
  assign se_jal_imm[31:0] = {{11{jal_imm[20]}},jal_imm[20:1],1'b0};

  // Branch immediate
  assign br_imm[12:1] = {IF_ID_inst[31],IF_ID_inst[7],IF_ID_inst[30:25],IF_ID_inst[11:8]};
  assign se_br_imm[31:0] = {{19{br_imm[12]}},br_imm[12:1],1'b0};

  // ######  Minseong Bae : End  #######

  // 
  // Register File 
  //
  regfile i_regfile(
    .clk			(clk),
	 // ######  Minseong Bae : Start  #######
    .we			(MEM_WB_regwrite),
    .rs1			(rs1),
    .rs2			(rs2),
    .rd			(MEM_WB_rd),
    .rd_data	(rd_data),
    .rs1_data	(rf_rs1_data),
    .rs2_data	(rf_rs2_data));

	assign MemWdata = EX_MEM_rs2_data;
	// ######  Minseong Bae : End  #######

	//
	// ALU 
	//
	alu i_alu(
		.a			(alusrc1),
		.b			(alusrc2),
		// ######  Minseong Bae : Start  #######
		.alucont	(ID_EX_alucontrol),
		// ######  Minseong Bae : End  #######
		.result	(aluout),
		.N			(Nflag),
		.Z			(Zflag),
		.C			(Cflag),
		.V			(Vflag));
	
	// ######  Minseong Bae : Start  #######
	// select alusrc1 and alusrc2 (forwarding)
	// 1st source to ALU (alusrc1)
	always@(*)
	begin
		if		  (forward_alusrc1_EX_MEM)	alusrc1        =  EX_MEM_aluout[31:0];
		else if (forward_alusrc1_MEM_WB)	alusrc1        =  rd_data[31:0];
		else if (ID_EX_auipc)				alusrc1[31:0]  =  ID_EX_pc;
		else if (ID_EX_lui) 					alusrc1[31:0]  =  32'b0;
		else          							alusrc1[31:0]  =  ID_EX_rs1_data[31:0];
	end
	
	// 2nd source to ALU (alusrc2)
	always@(*)
	begin
		if      (forward_alusrc2_EX_MEM)				alusrc2       = EX_MEM_aluout[31:0];
		else if (forward_alusrc2_MEM_WB)				alusrc2       = rd_data[31:0];
		else if (ID_EX_auipc | ID_EX_lui)			alusrc2[31:0] = ID_EX_auipc_lui_imm[31:0];
		else if (ID_EX_alusrc & ID_EX_memwrite)	alusrc2[31:0] = ID_EX_imm_stype[31:0];
		else if (ID_EX_alusrc)							alusrc2[31:0] = ID_EX_imm_itype[31:0];
		else													alusrc2[31:0] = ID_EX_rs2_data[31:0];
	end
	
	assign se_imm_itype[31:0] = {{20{IF_ID_inst[31]}},IF_ID_inst[31:20]};
	assign se_imm_stype[31:0] = {{20{IF_ID_inst[31]}},IF_ID_inst[31:25],IF_ID_inst[11:7]};
	assign auipc_lui_imm[31:0] = {IF_ID_inst[31:12],12'b0};
	

	// Data selection for writing to RF
	always@(*)
	begin
		if	     (MEM_WB_jal | MEM_WB_jalr)	rd_data[31:0] = MEM_WB_pc + 4; // return address
		else if (MEM_WB_memtoreg)				rd_data[31:0] = MEM_WB_MemRData;
		else											rd_data[31:0] = MEM_WB_aluout;
	end
	// ######  Minseong Bae : End  #######
	
endmodule
