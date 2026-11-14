// RV32I 2-stage pipelined core (single-file submission)
// Stage 1: IF+ID   Stage 2: EX+MEM+WB.  Top module: pipelined_riscv
`timescale 1ns / 1ps

// ---------------- alu ----------------
module alu (
    input  logic [31:0] a,
    input  logic [31:0] b,
    input  logic        fn_en,    // 1: OP / OP-IMM (use funct3); 0: plain add
    input  logic        is_reg,   // 1: R-type (SUB allowed)
    input  logic        alt,      // instruction[30]
    input  logic [2:0]  fn3,
    output logic [31:0] result
);
    logic [4:0]         shamt;
    logic [31:0]        sra_res, srl_res;
    assign shamt   = b[4:0];
    assign sra_res = $unsigned($signed(a) >>> shamt);
    assign srl_res = a >> shamt;

    always_comb begin
        result = a + b;
        if (fn_en) begin
            case (fn3)
                3'b000:  result = (is_reg & alt) ? (a - b) : (a + b);
                3'b001:  result = a << shamt;
                3'b010:  result = {31'b0, ($signed(a) < $signed(b))};
                3'b011:  result = {31'b0, (a < b)};
                3'b100:  result = a ^ b;
                3'b101:  result = alt ? sra_res : srl_res;
                3'b110:  result = a | b;
                3'b111:  result = a & b;
                default: result = a + b;
            endcase
        end
    end
endmodule

// ---------------- branch_unit ----------------
module branch_unit (
    input  logic [31:0] rs1,
    input  logic [31:0] rs2,
    input  logic [2:0]  fn3,
    output logic        taken
);
    always_comb begin
        case (fn3)
            3'b000:  taken = (rs1 == rs2);                       // BEQ
            3'b001:  taken = (rs1 != rs2);                       // BNE
            3'b100:  taken = ($signed(rs1) <  $signed(rs2));     // BLT
            3'b101:  taken = ($signed(rs1) >= $signed(rs2));     // BGE
            3'b110:  taken = (rs1 <  rs2);                       // BLTU
            3'b111:  taken = (rs1 >= rs2);                       // BGEU
            default: taken = 1'b0;
        endcase
    end
endmodule

// ---------------- control_unit ----------------
module control_unit (
    input  logic [6:0] opcode,
    output logic       reg_write,
    output logic       mem_read,
    output logic       mem_write,
    output logic       branch,
    output logic       jal,
    output logic       jalr,
    output logic       a_pc,       // ALU A = PC
    output logic       a_zero,     // ALU A = 0
    output logic       b_imm,      // ALU B = imm
    output logic       alu_fn_en,  // OP or OP-IMM
    output logic       is_reg      // R-type
);
    always_comb begin
        reg_write = 0; mem_read = 0; mem_write = 0; branch = 0;
        jal = 0; jalr = 0; a_pc = 0; a_zero = 0; b_imm = 0;
        alu_fn_en = 0; is_reg = 0;
        case (opcode)
            7'b0110111: begin reg_write = 1; a_zero = 1; b_imm = 1; end              // LUI
            7'b0010111: begin reg_write = 1; a_pc   = 1; b_imm = 1; end              // AUIPC
            7'b1101111: begin reg_write = 1; jal = 1; a_pc = 1; b_imm = 1; end       // JAL
            7'b1100111: begin reg_write = 1; jalr = 1; b_imm = 1; end                // JALR
            7'b1100011: begin branch = 1; a_pc = 1; b_imm = 1; end                   // BRANCH
            7'b0000011: begin reg_write = 1; mem_read = 1; b_imm = 1; end            // LOAD
            7'b0100011: begin mem_write = 1; b_imm = 1; end                          // STORE
            7'b0010011: begin reg_write = 1; b_imm = 1; alu_fn_en = 1; end           // OP-IMM
            7'b0110011: begin reg_write = 1; alu_fn_en = 1; is_reg = 1; end          // OP
            default: ;                                                               // NOP
        endcase
    end
endmodule

// ---------------- imm_gen ----------------
module imm_gen (
    input  logic [31:0] instr,
    output logic [31:0] imm
);
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;

    assign imm_i = {{20{instr[31]}}, instr[31:20]};
    assign imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
    assign imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    assign imm_u = {instr[31:12], 12'b0};
    assign imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

    always_comb begin
        case (instr[6:0])
            7'b0110111,
            7'b0010111: imm = imm_u;   // LUI, AUIPC
            7'b1101111: imm = imm_j;   // JAL
            7'b1100011: imm = imm_b;   // BRANCH
            7'b0100011: imm = imm_s;   // STORE
            default:    imm = imm_i;   // JALR, LOAD, OP-IMM
        endcase
    end
endmodule

// ---------------- regfile ----------------
module regfile (
    input  logic        clk,
    input  logic        reset,       // async, active high
    input  logic [4:0]  raddr1,
    input  logic [4:0]  raddr2,
    output logic [31:0] rdata1,
    output logic [31:0] rdata2,
    input  logic        we,
    input  logic [4:0]  waddr,
    input  logic [31:0] wdata
);
    logic [31:0] regs [31:0];

    // Reads with write-through bypass; x0 always reads 0
    always_comb begin
        if (raddr1 == 5'd0)                  rdata1 = 32'b0;
        else if (we && waddr == raddr1)      rdata1 = wdata;
        else                                 rdata1 = regs[raddr1];

        if (raddr2 == 5'd0)                  rdata2 = 32'b0;
        else if (we && waddr == raddr2)      rdata2 = wdata;
        else                                 rdata2 = regs[raddr2];
    end

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < 32; i++) regs[i] <= 32'b0;
        end else if (we && waddr != 5'd0) begin
            regs[waddr] <= wdata;
        end
    end
endmodule

// ---------------- pc_reg ----------------
module pc_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        redirect,
    input  logic [31:0] redirect_pc,
    output logic [31:0] pc
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) pc <= 32'h8000_0000;
        else       pc <= redirect ? redirect_pc : pc + 32'd4;
    end
endmodule

// ---------------- id_ex_reg ----------------
module id_ex_reg (
    input  logic        clk,
    input  logic        reset,
    input  logic        flush,        // squash the instruction entering stage 2

    input  logic [31:0] d_pc,
    input  logic [31:0] d_imm,
    input  logic [31:0] d_rs1,
    input  logic [31:0] d_rs2,
    input  logic [4:0]  d_rd,
    input  logic [2:0]  d_fn3,
    input  logic        d_alt,
    input  logic        d_reg_write,
    input  logic        d_mem_read,
    input  logic        d_mem_write,
    input  logic        d_branch,
    input  logic        d_jal,
    input  logic        d_jalr,
    input  logic        d_a_pc,
    input  logic        d_a_zero,
    input  logic        d_b_imm,
    input  logic        d_alu_fn_en,
    input  logic        d_is_reg,

    output logic        e_valid,
    output logic [31:0] e_pc,
    output logic [31:0] e_imm,
    output logic [31:0] e_rs1,
    output logic [31:0] e_rs2,
    output logic [4:0]  e_rd,
    output logic [2:0]  e_fn3,
    output logic        e_alt,
    output logic        e_reg_write,
    output logic        e_mem_read,
    output logic        e_mem_write,
    output logic        e_branch,
    output logic        e_jal,
    output logic        e_jalr,
    output logic        e_a_pc,
    output logic        e_a_zero,
    output logic        e_b_imm,
    output logic        e_alu_fn_en,
    output logic        e_is_reg
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            e_valid <= 0;  e_pc <= 0;  e_imm <= 0;  e_rs1 <= 0;  e_rs2 <= 0;
            e_rd <= 0;  e_fn3 <= 0;  e_alt <= 0;
            e_reg_write <= 0;  e_mem_read <= 0;  e_mem_write <= 0;
            e_branch <= 0;  e_jal <= 0;  e_jalr <= 0;
            e_a_pc <= 0;  e_a_zero <= 0;  e_b_imm <= 0;
            e_alu_fn_en <= 0;  e_is_reg <= 0;
        end else begin
            e_valid     <= ~flush;
            e_pc        <= d_pc;
            e_imm       <= d_imm;
            e_rs1       <= d_rs1;
            e_rs2       <= d_rs2;
            e_rd        <= d_rd;
            e_fn3       <= d_fn3;
            e_alt       <= d_alt;
            e_reg_write <= d_reg_write;
            e_mem_read  <= d_mem_read;
            e_mem_write <= d_mem_write;
            e_branch    <= d_branch;
            e_jal       <= d_jal;
            e_jalr      <= d_jalr;
            e_a_pc      <= d_a_pc;
            e_a_zero    <= d_a_zero;
            e_b_imm     <= d_b_imm;
            e_alu_fn_en <= d_alu_fn_en;
            e_is_reg    <= d_is_reg;
        end
    end
endmodule

// ---------------- pipelined_riscv ----------------
module pipelined_riscv (
    input  logic        clk,
    input  logic        reset,
    input  logic [31:0] instruction,
    input  logic [31:0] mem_out,
    output logic [31:0] address,
    output logic [31:0] alu_out,
    output logic [31:0] rs2_data,
    output logic        mem_write,
    output logic        mem_read,
    output logic [2:0]  fn3
);
    // ---------------- Stage 1 signals ----------------
    logic [31:0] pc;
    logic [31:0] d_imm, d_rs1, d_rs2;
    logic        d_reg_write, d_mem_read, d_mem_write, d_branch, d_jal, d_jalr;
    logic        d_a_pc, d_a_zero, d_b_imm, d_alu_fn_en, d_is_reg;

    // ---------------- Stage 2 signals ----------------
    logic        e_valid;
    logic [31:0] e_pc, e_imm, e_rs1, e_rs2;
    logic [4:0]  e_rd;
    logic [2:0]  e_fn3;
    logic        e_alt, e_reg_write, e_mem_read, e_mem_write;
    logic        e_branch, e_jal, e_jalr, e_a_pc, e_a_zero, e_b_imm;
    logic        e_alu_fn_en, e_is_reg;

    logic [31:0] alu_a, alu_b, alu_res;
    logic        br_taken, redirect;
    logic [31:0] redirect_pc, wb_data;
    logic        wb_we;

    // =====================================================
    // STAGE 1 : IF + ID
    // =====================================================
    assign address = pc;

    pc_reg u_pc (
        .clk(clk), .reset(reset),
        .redirect(redirect), .redirect_pc(redirect_pc),
        .pc(pc)
    );

    control_unit u_ctrl (
        .opcode(instruction[6:0]),
        .reg_write(d_reg_write), .mem_read(d_mem_read), .mem_write(d_mem_write),
        .branch(d_branch), .jal(d_jal), .jalr(d_jalr),
        .a_pc(d_a_pc), .a_zero(d_a_zero), .b_imm(d_b_imm),
        .alu_fn_en(d_alu_fn_en), .is_reg(d_is_reg)
    );

    imm_gen u_imm (.instr(instruction), .imm(d_imm));

    regfile u_rf (
        .clk(clk), .reset(reset),
        .raddr1(instruction[19:15]), .raddr2(instruction[24:20]),
        .rdata1(d_rs1), .rdata2(d_rs2),
        .we(wb_we), .waddr(e_rd), .wdata(wb_data)
    );

    // =====================================================
    // Pipeline register
    // =====================================================
    id_ex_reg u_idex (
        .clk(clk), .reset(reset),
        .flush(redirect),                 // wrong-path instruction is squashed
        .d_pc(pc), .d_imm(d_imm), .d_rs1(d_rs1), .d_rs2(d_rs2),
        .d_rd(instruction[11:7]), .d_fn3(instruction[14:12]), .d_alt(instruction[30]),
        .d_reg_write(d_reg_write), .d_mem_read(d_mem_read), .d_mem_write(d_mem_write),
        .d_branch(d_branch), .d_jal(d_jal), .d_jalr(d_jalr),
        .d_a_pc(d_a_pc), .d_a_zero(d_a_zero), .d_b_imm(d_b_imm),
        .d_alu_fn_en(d_alu_fn_en), .d_is_reg(d_is_reg),

        .e_valid(e_valid), .e_pc(e_pc), .e_imm(e_imm), .e_rs1(e_rs1), .e_rs2(e_rs2),
        .e_rd(e_rd), .e_fn3(e_fn3), .e_alt(e_alt),
        .e_reg_write(e_reg_write), .e_mem_read(e_mem_read), .e_mem_write(e_mem_write),
        .e_branch(e_branch), .e_jal(e_jal), .e_jalr(e_jalr),
        .e_a_pc(e_a_pc), .e_a_zero(e_a_zero), .e_b_imm(e_b_imm),
        .e_alu_fn_en(e_alu_fn_en), .e_is_reg(e_is_reg)
    );

    // =====================================================
    // STAGE 2 : EX + MEM + WB
    // =====================================================
    assign alu_a = e_a_zero ? 32'b0 : (e_a_pc ? e_pc : e_rs1);
    assign alu_b = e_b_imm  ? e_imm : e_rs2;

    alu u_alu (
        .a(alu_a), .b(alu_b),
        .fn_en(e_alu_fn_en), .is_reg(e_is_reg), .alt(e_alt), .fn3(e_fn3),
        .result(alu_res)
    );

    branch_unit u_br (.rs1(e_rs1), .rs2(e_rs2), .fn3(e_fn3), .taken(br_taken));

    assign redirect    = e_valid & ((e_branch & br_taken) | e_jal | e_jalr);
    assign redirect_pc = e_jalr ? (alu_res & 32'hFFFF_FFFE) : alu_res;

    // Data memory interface
    assign alu_out   = alu_res;
    assign rs2_data  = e_rs2;
    assign fn3       = e_fn3;
    assign mem_write = e_valid & e_mem_write;
    assign mem_read  = e_valid & e_mem_read;

    // Write-back
    assign wb_data = (e_jal | e_jalr) ? (e_pc + 32'd4) :
                      e_mem_read      ? mem_out :
                                        alu_res;
    assign wb_we   = e_valid & e_reg_write & (e_rd != 5'd0);

endmodule
