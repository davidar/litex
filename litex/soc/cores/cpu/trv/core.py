#
# This file is part of LiteX.
#
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2024 David A Roberts <d@vidr.cc>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect import wishbone
from litex.soc.cores.cpu import CPU, CPU_GCC_TRIPLE_RISCV32

# Variants -----------------------------------------------------------------------------------------

CPU_VARIANTS = {
    "standard": "standard",
}

# GCC Flags ----------------------------------------------------------------------------------------

GCC_FLAGS = {
    #                       /------------ Base ISA
    #                       |    /------- Hardware Multiply + Divide
    #                       |    |/----- Atomics
    #                       |    ||/---- Compressed ISA
    #                       |    |||/--- Single-Precision Floating-Point
    #                       |    ||||/-- Double-Precision Floating-Point
    #                       i    macfd
    "standard": "-march=rv32i2p0      -mabi=ilp32",
}

# TRV ------------------------------------------------------------------------------------------


class TRV(CPU):
    category = "softcore"
    family = "riscv"
    name = "trv"
    human_name = "TRV"
    variants = CPU_VARIANTS
    data_width = 32
    endianness = "little"
    gcc_triple = CPU_GCC_TRIPLE_RISCV32
    linker_output_format = "elf32-littleriscv"
    nop = "nop"
    io_regions = {0x8000_0000: 0x8000_0000}  # Origin, Length.

    # GCC Flags.
    @property
    def gcc_flags(self):
        flags = GCC_FLAGS[self.variant]
        flags += " -D__trv__ "
        return flags

    def __init__(self, platform, variant="standard"):
        self.platform = platform
        self.variant = variant
        self.human_name = f"TRV-{variant}"
        self.reset = Signal()
        self.idbus = idbus = wishbone.Interface(
            data_width=32, address_width=32, addressing="byte"
        )
        self.periph_buses = [idbus]  # Peripheral buses (Connected to main SoC's bus).
        self.memory_buses = []  # Memory buses (Connected directly to LiteDRAM).

        # from build/software/include/generated/csr.h
        CSR_BASE = 0x82000000
        CSR_LEDS_OUT_ADDR = CSR_BASE + 0x1000

        # led = Signal()
        # counter = Signal(26)
        # self.comb += led.eq(counter[22])
        # self.sync += counter.eq(counter + 1)

        mem = Memory(1, 16, init=[0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1])
        self.specials += mem

        # wrport = mem.get_port(write_capable=True)
        # self.specials += wrport
        # self.comb += [
        #     wrport.adr.eq(produce),
        #     wrport.dat_w.eq(din),
        #     wrport.we.eq(1)
        # ]

        mem_rdport = mem.get_port(async_read=True)
        self.specials += mem_rdport
        # self.comb += [
        #     mem_rdport.adr.eq(consume),
        #     dout.eq(mem_rdport.dat_r)
        # ]

        pc = Signal(32)
        # self.comb += pc.eq(counter[22:26])

        register_bank = Memory(32, 32)
        self.specials += register_bank

        rs1_rdport = register_bank.get_port(async_read=True)
        self.specials += rs1_rdport

        rs2_rdport = register_bank.get_port(async_read=True)
        self.specials += rs2_rdport

        rd_wrport = register_bank.get_port(write_capable=True)
        self.specials += rd_wrport

        # reg [31:0] instr;
        # ...
        # wire isALUreg  =  (instr[6:0] == 7'b0110011); // rd <- rs1 OP rs2
        # wire isALUimm  =  (instr[6:0] == 7'b0010011); // rd <- rs1 OP Iimm
        # wire isBranch  =  (instr[6:0] == 7'b1100011); // if(rs1 OP rs2) PC<-PC+Bimm
        # wire isJALR    =  (instr[6:0] == 7'b1100111); // rd <- PC+4; PC<-rs1+Iimm
        # wire isJAL     =  (instr[6:0] == 7'b1101111); // rd <- PC+4; PC<-PC+Jimm
        # wire isAUIPC   =  (instr[6:0] == 7'b0010111); // rd <- PC + Uimm
        # wire isLUI     =  (instr[6:0] == 7'b0110111); // rd <- Uimm
        # wire isLoad    =  (instr[6:0] == 7'b0000011); // rd <- mem[rs1+Iimm]
        # wire isStore   =  (instr[6:0] == 7'b0100011); // mem[rs1+Simm] <- rs2
        # wire isSYSTEM  =  (instr[6:0] == 7'b1110011); // special
        instr = Signal(32)
        isALUreg = instr[:7] == 0b0110011  # rd <- rs1 OP rs2
        isALUimm = instr[:7] == 0b0010011  # rd <- rs1 OP Iimm
        isBranch = instr[:7] == 0b1100011  # if(rs1 OP rs2) PC<-PC+Bimm
        isJALR = instr[:7] == 0b1100111  # rd <- PC+4; PC<-rs1+Iimm
        isJAL = instr[:7] == 0b1101111  # rd <- PC+4; PC<-PC+Jimm
        isAUIPC = instr[:7] == 0b0010111  # rd <- PC + Uimm
        isLUI = instr[:7] == 0b0110111  # rd <- Uimm
        isLoad = instr[:7] == 0b0000011  # rd <- mem[rs1+Iimm]
        isStore = instr[:7] == 0b0100011  # mem[rs1+Simm] <- rs2
        isSYSTEM = instr[:7] == 0b1110011  # special

        # wire [4:0] rs1Id = instr[19:15];
        # wire [4:0] rs2Id = instr[24:20];
        # wire [4:0] rdId  = instr[11:7];
        # wire [2:0] funct3 = instr[14:12];
        # wire [6:0] funct7 = instr[31:25];
        rdId = instr[7:12]
        funct3 = instr[12:15]
        rs1Id = instr[15:20]
        rs2Id = instr[20:25]
        funct7 = instr[25:]

        # wire [31:0] Uimm={    instr[31],   instr[30:12], {12{1'b0}}};
        # wire [31:0] Iimm={{21{instr[31]}}, instr[30:20]};
        # wire [31:0] Simm={{21{instr[31]}}, instr[30:25],instr[11:7]};
        # wire [31:0] Bimm={{20{instr[31]}}, instr[7],instr[30:25],instr[11:8],1'b0};
        # wire [31:0] Jimm={{12{instr[31]}}, instr[19:12],instr[20],instr[30:21],1'b0};
        sign = Replicate(instr[31], 20)
        Uimm = instr & 0xFFFFF000
        Iimm = Cat(rs2Id, funct7, sign)
        Simm = Cat(rdId, funct7, sign)
        Bimm = Cat(rdId & 0b11110, funct7[:6], rdId[0], sign)
        Jimm = Cat(rs2Id & 0b11110, funct7[:6], rs2Id[0], sign)

        rs1 = Signal(32)
        rs2 = Signal(32)

        self.instr_fsm = FSM(reset_state="FETCH_INSTR")
        self.instr_fsm.act(
            "FETCH_INSTR",
            # instr.eq(mem[pc]),
            NextState("FETCH_OPERANDS"),
        )
        self.instr_fsm.act(
            "FETCH_OPERANDS",
            # rs1.eq(register_bank[rs1Id]),
            # rs2.eq(register_bank[rs2Id]),
            rs1_rdport.adr.eq(rs1Id),
            rs2_rdport.adr.eq(rs2Id),
            rs1.eq(rs1_rdport.dat_r),
            rs2.eq(rs2_rdport.dat_r),
            NextState("EXECUTE"),
        )
        self.instr_fsm.act(
            "EXECUTE",
            NextValue(pc, pc + 1),
            NextState("FETCH_INSTR"),
        )

        write_back_data = Signal(32)
        write_back_enable = Signal()
        self.comb += [
            rd_wrport.adr.eq(rdId),
            rd_wrport.dat_w.eq(write_back_data),
            rd_wrport.we.eq(write_back_enable & rdId > 0),
        ]

        latch = Signal()
        write = 1
        read = 0

        led = Signal()
        self.comb += [mem_rdport.adr.eq(pc[20:24]), led.eq(mem_rdport.dat_r)]

        self.fsm = fsm = FSM(reset_state="WAIT")
        fsm.act(
            "WAIT",
            # Latch Address + Bytes to Words conversion.
            NextValue(idbus.adr, CSR_LEDS_OUT_ADDR),
            # Latch Wdata/WMask.
            NextValue(idbus.dat_w, led),
            NextValue(idbus.sel, 0xF),
            # If Read or Write, jump to access.
            If(read | write, NextValue(idbus.we, write), NextState("WB-ACCESS")),
        )
        fsm.act(
            "WB-ACCESS",
            idbus.stb.eq(1),
            idbus.cyc.eq(1),
            # mbus.wbusy.eq(1),
            # mbus.rbusy.eq(1),
            If(
                idbus.ack,
                # mbus.wbusy.eq(0),
                # mbus.rbusy.eq(0),
                latch.eq(1),
                NextState("WAIT"),
            ),
        )

        # Latch RData on Wishbone ack.
        mbus_rdata = Signal(32)
        self.sync += If(latch, mbus_rdata.eq(idbus.dat_r))
        # self.comb += mbus.rdata.eq(mbus_rdata)             # Latched value.
        # self.comb += If(latch, mbus.rdata.eq(idbus.dat_r)) # Immediate value.

    def set_reset_address(self, reset_address):
        self.reset_address = reset_address
        # self.cpu_params.update(p_RESET_ADDR=Constant(reset_address, 32))

    def do_finalize(self):
        assert hasattr(self, "reset_address")
        # self.specials += Instance("FemtoRV32", **self.cpu_params)
