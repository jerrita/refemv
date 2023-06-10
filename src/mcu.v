// Multi cycle cpu
// ref: https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/README.md

`include "inc/define.vh"


module mcu(
    input clk, rstn,
    // input [`BUS] mem_rdata,
    // input mem_rbusy, mem_wbusy,
    // output [`BUS] mem_addr, mem_wdata,
    // output [3:0] mem_wmask, mem_rmask
    output reg [`BUS] a0  // for debug
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
    real time
*/
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

/*
    Hardcoded instr_rom
*/
reg [7:0] instr_rom [0:255];
reg [`BUS] instr;
initial begin
    $readmemh("build/prog.hex", instr_rom);
    instr = 0;
end

/*
    Register file
    read: real time
    write: 1 clk
*/
reg [`BUS] regfile [0:31];
reg [`BUS] rs1d, rs2d;
initial for (i = 0; i < 32; i = i + 1) begin
    regfile[i] = 0;
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        for (i = 0; i < 32; i = i + 1) begin
            regfile[i] <= 0;
        end
    end else begin
        rs1d <= (rs1 == 5'b0) ? 0 : regfile[rs1];
        rs2d <= (rs2 == 5'b0) ? 0 : regfile[rs2];
        a0 <= regfile[10];
        if (we_reg) begin
            regfile[wd] <= wdata;
        end
    end
end



/*
    ALU
*/
assign aluain = rs1d;
assign alubin = isALUreg ? rs2d : immI;
reg [`BUS] aluout;

always @(*) begin
    case (funct3)
        3'b000: aluout = funct7[5] ? (aluain - alubin) : (aluain + alubin);
        3'b001: aluout = aluain << shamt;
        3'b010: aluout = $signed(aluain) < $signed(alubin);
        3'b011: aluout = aluain < alubin;
        3'b100: aluout = aluain ^ alubin;
        3'b101: aluout = funct7[5] ? ($signed(aluain) >>> shamt) : (aluain >> shamt);
        3'b110: aluout = aluain | alubin;
        3'b111: aluout = aluain & alubin;
    endcase
end

assign wdata = (isJAL || isJALR) ? (pc + 4) : aluout;
assign we_reg = (state == WRITE_BACK) && (isALUreg || isALUimm || isJAL || isJALR);

/*
    FSM
*/
localparam INSTR_FETCH = 0;
localparam EXECUTE = 1;
localparam MEM_ACCESS = 2;
localparam WRITE_BACK = 3;

reg [1:0] state;
initial state = INSTR_FETCH;

// TODO: (* parallel_case *) can be used to create pipelined FSM
// TODO: But now, we use a multi cycle CPU
//* Sequential FSM
wire [`BUS] nextpc = isJAL   ? (pc + immJ) :
                     isJALR  ? (rs1d + immI) :
                     (pc + 4);
always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        state <= INSTR_FETCH;
    end else begin
        case (state)
            INSTR_FETCH: begin
                instr <= {instr_rom[pc+3], instr_rom[pc+2], instr_rom[pc+1], instr_rom[pc]};
                state <= EXECUTE;
            end
            EXECUTE: begin
                pc <= nextpc;
                state <= INSTR_FETCH;
            end
            default: state <= INSTR_FETCH;  // because we use assign for hard-coded mem now
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