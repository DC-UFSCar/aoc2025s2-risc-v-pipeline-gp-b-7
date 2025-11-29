module riscvpipeline (
    input 	  clk,
    input 	  reset,
    output [31:0] PC,
    input  [31:0] Instr,
    output [31:0] Address,  
    output [31:0] WriteData, 
    output        MemWrite,  
    input  [31:0] ReadData);

   function isALUreg; input [31:0] I; isALUreg=(I[6:0]==7'b0110011); endfunction
   function isALUimm; input [31:0] I; isALUimm=(I[6:0]==7'b0010011); endfunction
   function isBranch; input [31:0] I; isBranch=(I[6:0]==7'b1100011); endfunction
   function isJALR;   input [31:0] I; isJALR  =(I[6:0]==7'b1100111); endfunction
   function isJAL;    input [31:0] I; isJAL   =(I[6:0]==7'b1101111); endfunction
   function isAUIPC;  input [31:0] I; isAUIPC =(I[6:0]==7'b0010111); endfunction
   function isLUI;    input [31:0] I; isLUI   =(I[6:0]==7'b0110111); endfunction
   function isLoad;   input [31:0] I; isLoad  =(I[6:0]==7'b0000011); endfunction
   function isStore;  input [31:0] I; isStore =(I[6:0]==7'b0100011); endfunction
   function isSYSTEM; input [31:0] I; isSYSTEM=(I[6:0]==7'b1110011); endfunction

   function [4:0] rs1Id; input [31:0] I; rs1Id = I[19:15];      endfunction
   function [4:0] rs2Id; input [31:0] I; rs2Id = I[24:20];      endfunction
   function [4:0] shamt; input [31:0] I; shamt = I[24:20];      endfunction
   function [4:0] rdId;  input [31:0] I; rdId  = I[11:7];       endfunction
   function [1:0] csrId; input [31:0] I; csrId = {I[27],I[21]}; endfunction

   function [2:0] funct3; input [31:0] I; funct3 = I[14:12]; endfunction
   function [6:0] funct7; input [31:0] I; funct7 = I[31:25]; endfunction

   function isEBREAK; input [31:0] I; isEBREAK = (isSYSTEM(I) && funct3(I) == 3'b000); endfunction

   function [31:0] Uimm; input [31:0] I; Uimm={I[31:12],{12{1'b0}}}; endfunction
   function [31:0] Iimm; input [31:0] I; Iimm={{21{I[31]}},I[30:20]}; endfunction
   function [31:0] Simm; input [31:0] I; Simm={{21{I[31]}},I[30:25],I[11:7]}; endfunction
   function [31:0] Bimm; input [31:0] I; Bimm = {{20{I[31]}},I[7],I[30:25],I[11:8],1'b0}; endfunction
   function [31:0] Jimm; input [31:0] I; Jimm = {{12{I[31]}},I[19:12],I[20],I[30:21],1'b0}; endfunction

   function writesRd; input [31:0] I; writesRd = !isStore(I) && !isBranch(I); endfunction
   function readsRs1; input [31:0] I; readsRs1 = !(isJAL(I) || isAUIPC(I) || isLUI(I)); endfunction
   function readsRs2; input [31:0] I; readsRs2 = isALUreg(I) || isBranch(I) || isStore(I); endfunction

   reg [31:0] F_PC;
   reg [31:0] FD_PC;
   reg [31:0] FD_instr;
   reg        FD_nop;
   assign PC = F_PC;

   wire [31:0] jumpOrBranchAddress;
   wire        jumpOrBranch;

   always @(posedge clk) begin
      if (!F_stall) begin
         FD_instr <= Instr;
         FD_PC    <= F_PC;
         F_PC     <= F_PC + 4;
      end

      if (jumpOrBranch) begin
         F_PC <= jumpOrBranchAddress;
      end

      FD_nop <= D_flush | reset;

      if (reset) begin
         F_PC <= 0;
      end
   end


   reg [31:0] DE_PC;
   reg [31:0] DE_instr;
   reg [31:0] DE_rs1;
   reg [31:0] DE_rs2;

   wire        writeBackEn;
   wire [31:0] writeBackData;
   wire [4:0]  wbRdId;

   reg [31:0] RegisterBank [0:31];
   always @(posedge clk) begin
      if (!D_stall) begin
         DE_PC <= FD_PC;
         DE_instr <= (E_flush | FD_nop) ? 32'b0 : FD_instr;
      end

      if (E_flush) begin
         DE_instr <= 32'b0;
      end

      // Read register bank with write-back bypass for same cycle
      if (writeBackEn && wbRdId == rs1Id(FD_instr) && rs1Id(FD_instr) != 0) begin
         DE_rs1 <= writeBackData;
      end else begin
         DE_rs1 <= rs1Id(FD_instr) ? RegisterBank[rs1Id(FD_instr)] : 32'b0;
      end

      if (writeBackEn && wbRdId == rs2Id(FD_instr) && rs2Id(FD_instr) != 0) begin
         DE_rs2 <= writeBackData;
      end else begin
         DE_rs2 <= rs2Id(FD_instr) ? RegisterBank[rs2Id(FD_instr)] : 32'b0;
      end

      if (writeBackEn) begin
         RegisterBank[wbRdId] <= writeBackData;
      end
   end

   wire F_stall, D_stall;
   wire D_flush, E_flush;

   // detect load-use hazards: when DE is load and FD reads its rd
   wire rs1Hazard = readsRs1(FD_instr) && rs1Id(FD_instr) != 0 && (writesRd(DE_instr) && rs1Id(FD_instr) == rdId(DE_instr));
   wire rs2Hazard = readsRs2(FD_instr) && rs2Id(FD_instr) != 0 && (writesRd(DE_instr) && rs2Id(FD_instr) == rdId(DE_instr));
   wire dataHazard = !FD_nop && isLoad(DE_instr) && (rs1Hazard || rs2Hazard);

   assign F_stall = dataHazard | halt;
   assign D_stall = dataHazard | halt;

   assign D_flush = jumpOrBranch;
   assign E_flush = jumpOrBranch | dataHazard;

   reg [31:0] EM_PC;
   reg [31:0] EM_instr;
   reg [31:0] EM_rs2;
   reg [31:0] EM_Eresult;
   reg [31:0] EM_addr;
   // Forwarding from EM and MW stages into Execute
   wire E_M_fwd_rs1 = (rdId(EM_instr) != 0) && writesRd(EM_instr) && (rdId(EM_instr) == rs1Id(DE_instr));
   wire E_W_fwd_rs1 = (rdId(MW_instr) != 0) && writesRd(MW_instr) && (rdId(MW_instr) == rs1Id(DE_instr));
   wire E_M_fwd_rs2 = (rdId(EM_instr) != 0) && writesRd(EM_instr) && (rdId(EM_instr) == rs2Id(DE_instr));
   wire E_W_fwd_rs2 = (rdId(MW_instr) != 0) && writesRd(MW_instr) && (rdId(MW_instr) == rs2Id(DE_instr));

   wire [31:0] E_rs1 = E_M_fwd_rs1 ? EM_Eresult :
                        E_W_fwd_rs1 ? writeBackData :
                        DE_rs1;

   wire [31:0] E_rs2 = E_M_fwd_rs2 ? EM_Eresult :
                        E_W_fwd_rs2 ? writeBackData :
                        DE_rs2;

   wire [31:0] E_aluIn1 = E_rs1;
   wire [31:0] E_aluIn2 = (isALUreg(DE_instr) | isBranch(DE_instr)) ? E_rs2 : Iimm(DE_instr);
   wire [4:0]  E_shamt  = isALUreg(DE_instr) ? DE_rs2[4:0] : shamt(DE_instr);
   wire E_minus = DE_instr[30] & isALUreg(DE_instr);
   wire E_arith_shift = DE_instr[30];

   // The adder is used by both arithmetic instructions and JALR.
   wire [31:0] E_aluPlus = E_aluIn1 + E_aluIn2;

   // Use a single 33 bits subtract to do subtraction and all comparisons
   // (trick borrowed from swapforth/J1)
   wire [32:0] E_aluMinus = {1'b1, ~E_aluIn2} + {1'b0,E_aluIn1} + 33'b1;
   wire        E_LT  = (E_aluIn1[31] ^ E_aluIn2[31]) ? E_aluIn1[31] : E_aluMinus[32];
   wire        E_LTU = E_aluMinus[32];
   wire        E_EQ  = (E_aluMinus[31:0] == 0);

   // Flip a 32 bit word. Used by the shifter (a single shifter for
   // left and right shifts, saves silicium !)
   function [31:0] flip32;
      input [31:0] x;
      flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7],
		x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15],
		x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
		x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
   endfunction

   wire [31:0] E_shifter_in = funct3(DE_instr) == 3'b001 ? flip32(E_aluIn1) : E_aluIn1;
   wire [31:0] E_shifter = $signed({E_arith_shift & E_aluIn1[31], E_shifter_in}) >>> E_aluIn2[4:0];
   wire [31:0] E_leftshift = flip32(E_shifter);