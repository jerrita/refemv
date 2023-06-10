// Multi cycle cpu
// ref: https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/README.md

`include "inc/define.vh"


module mcu(
    input clk, rstn,
    input [`BUS] mem_rdata,
    // input mem_rbusy, mem_wbusy,
    output [`BUS] mem_addr, mem_wdata,
    output mem_rstrb, mem_wstrb,
    // output [3:0] mem_wmask, mem_rmask,
    output [`BUS] a0  // for debug
);

integer i;

// wire [`BUS] mem_wdata, mem_addr, mem_rdata;
// wire [3:0] mem_wmask, mem_rmask;
// wire mem_rbusy, mem_wbusy;
wire [`BUS] wdata;
wire [`BUS] aluain, alubin;
wire we_reg;


/*
    Decoder
    *real time*
    because of the elegance in the riscv design.
*/
reg [`BUS] instr;
wire isALUreg  =  (instr[6:0] == 7'b0110011); // rd <- rs1 OP rs2   
wire isALUimm  =  (instr[6:0] == 7'b0010011); // rd <- rs1 OP Iimm
wire isBranch  =  (instr[6:0] == 7'b1100011); // if(rs1 OP rs2) PC<-PC+Bimm
wire isJALR    =  (instr[6:0] == 7'b1100111); // rd <- PC+4; PC<-rs1+Iimm
wire isJAL     =  (instr[6:0] == 7'b1101111); // rd <- PC+4; PC<-PC+Jimm
wire isAUIPC   =  (instr[6:0] == 7'b0010111); // rd <- PC + Uimm
wire isLUI     =  (instr[6:0] == 7'b0110111); // rd <- Uimm   
wire isLoad    =  (instr[6:0] == 7'b0000011); // rd <- mem[rs1+Iimm]
wire isStore   =  (instr[6:0] == 7'b0100011); // mem[rs1+Simm] <- rs2
wire isSYSTEM  =  (instr[6:0] == 7'b1110011); // special

wire [4:0] rs1    = instr[19:15];
wire [4:0] rs2    = instr[24:20];
wire [4:0] shamt  = instr[24:20];
wire [4:0] wd     = instr[11:7];
wire [2:0] funct3 = instr[14:12];
wire [6:0] funct7 = instr[31:25];

wire [31:0] immU = {    instr[31],   instr[30:12], {12{1'b0}}};
wire [31:0] immI = {{21{instr[31]}}, instr[30:20]};
wire [31:0] immS = {{21{instr[31]}}, instr[30:25],instr[11:7]};
wire [31:0] immB = {{20{instr[31]}}, instr[7],instr[30:25],instr[11:8],1'b0};
wire [31:0] immJ = {{12{instr[31]}}, instr[19:12],instr[20],instr[30:21],1'b0};


//* Bellow codes should not hold any wire, only assign to reg

// PC Module
reg [`BUS] pc;
initial pc = 0;
wire [`BUS] pc4 = pc + 4;
wire [`BUS] pcimm = pc + (instr[3] ? immJ :
                          instr[4] ? immU :
                          immB);

/*
    Register file
    read: real timem
    write: 1 clk
*/
reg [`BUS] regfile [0:31];
reg [`BUS] rs1d, rs2d, a0r;
initial for (i = 0; i < 32; i = i + 1) begin
    regfile[i] = 0;
end


assign wdata = (isJAL || isJALR) ? pc4 :
               isLUI             ? immU :
               isAUIPC           ? pcimm :
               isLoad            ? mem_load :
               aluout;
assign we_reg = (state == WRITE_BACK) && 
    (isALUreg || isALUimm || isJAL || isJALR || isLUI || isAUIPC || isLoad);


// We must read on clk, so it can be synthesized to bram
always @(posedge clk) begin
    rs1d <= (rs1 == 5'b0) ? 0 : regfile[rs1];
    rs2d <= (rs2 == 5'b0) ? 0 : regfile[rs2];
    a0r <= regfile[5'd10];
    if (we_reg) begin
        regfile[wd] <= wdata;
    end
end

assign a0 = a0r;

/*
    ALU
*/
assign aluain = rs1d;
assign alubin = (isALUreg || isBranch) ? rs2d : immI;
reg [`BUS] aluout;

wire [32:0] aluMinus = {1'b0, aluain} - {1'b0, alubin};
wire [31:0] aluPlus = aluain + alubin;

reg needBranch;
wire EQ   = (aluMinus[31:0] == 0);
wire LTU  = aluMinus[32];
wire LT   = (aluain[31] ^ alubin[31]) ? aluain[31] : aluMinus[32];

wire [31:0] shifter = $signed({funct7[5] & aluain[31], aluain}) >>> shamt;

// for result
always @(*) begin
    case (funct3)
        3'b000: aluout = funct7[5] ? aluMinus[31:0] : aluPlus;
        3'b001: aluout = aluain << shamt;
        3'b010: aluout = {31'b0, LT};
        3'b011: aluout = {31'b0, LTU};
        3'b100: aluout = aluain ^ alubin;
        3'b101: aluout = shifter;
        3'b110: aluout = aluain | alubin;
        3'b111: aluout = aluain & alubin;
    endcase
end

// for branch
always @(*) begin
    case (funct3)
        3'b000: needBranch = EQ;
        3'b001: needBranch = ~EQ;
        3'b100: needBranch = LT;
        3'b101: needBranch = ~LT;
        3'b110: needBranch = LTU;
        3'b111: needBranch = ~LTU;
        default: needBranch = 1'b0;
    endcase
end

/*
    Memory
    connect to outside world
*/
wire [`BUS] mem_addr = (state == MEM_ACCESS) ? rs1d + immI : pc;
wire [15:0] load_halfword = mem_addr[1] ? mem_rdata[31:16] : mem_rdata[15:0];
wire [7:0] load_byte = mem_addr[0] ? load_halfword[15:8] : load_halfword[7:0];

wire mem_byteacc = funct3[1:0] == 2'b00;
wire mem_halfacc = funct3[1:0] == 2'b01;
wire load_sign = !funct3[2] & (mem_byteacc ? load_byte[7] : load_halfword[15]);
wire [`BUS] mem_load = mem_byteacc ? {{24{load_sign}}, load_byte} :
                       mem_halfacc ? {{16{load_sign}}, load_halfword} :
                       mem_rdata;

assign mem_rstrb = (state == INSTR_FETCH)
                   || (state == MEM_ACCESS && isLoad);
assign mem_wstrb = (state == MEM_ACCESS && isStore);

/*
    FSM
*/
localparam INSTR_FETCH = 0;
localparam WAIT_INSTR = 1;
localparam WAIT_REG = 2;
localparam EXECUTE = 3;
localparam MEM_ACCESS = 4;
localparam WAIT_DATA = 5;
localparam WRITE_BACK = 6;
reg [3:0] state;
initial state = INSTR_FETCH;

// TODO: (* parallel_case *) can be used to create pipelined FSM
// TODO: But now, we use a multi cycle CPU
//* Sequential FSM
wire [`BUS] nextpc = (isBranch && needBranch || isJAL) ? pcimm :
                     isJALR  ? {aluPlus[31:1], 1'b0} :
                     pc4;

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        state <= INSTR_FETCH;
    end else begin
        case (state)
            INSTR_FETCH: begin
                state <= WAIT_INSTR;
            end
            WAIT_INSTR: begin
                instr <= mem_rdata;
                state <= WAIT_REG;
            end
            WAIT_REG: begin
                state <= EXECUTE;
            end
            EXECUTE: begin
                if (!isSYSTEM) begin
                    pc <= nextpc;
                end
                state <= isLoad ? MEM_ACCESS : WRITE_BACK;
            end
            MEM_ACCESS: begin
                state <= WAIT_DATA;
            end
            WAIT_DATA: begin
                state <= WRITE_BACK;
            end
            WRITE_BACK: begin
                state <= INSTR_FETCH;
            end
        endcase
    end
end


/*
    Debugger
*/
`ifdef BENCH
always @(posedge clk) begin
    $display("#%0d", pc);
    case (1'b1)
        isALUreg: $display("ALUreg rd = %d rs1 = %d rs2 = %d funct3 = %b", wd, rs1, rs2, funct3);
        isALUimm: $display("ALUimm rd = %d rs1 = %d imm = %0d funct3 = %b", wd, rs1, immI, funct3);
        isBranch: $display("BRANCH");
        isJAL:    $display("JAL");
        isJALR:   $display("JALR");
        isAUIPC:  $display("AUIPC");
        isLUI:    $display("LUI");	
        isLoad:   $display("LOAD");
        isStore:  $display("STORE");
        isSYSTEM: $display("SYSTEM");
    endcase
end
`endif

endmodule