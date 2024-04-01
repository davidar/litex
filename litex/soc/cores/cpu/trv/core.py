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

# from .riscv_assembler import RiscvAssembler

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

        self.PC = Signal(32)

        register_bank = Memory(32, 32)
        self.specials += register_bank

        rs1_rdport = register_bank.get_port(async_read=True)
        self.specials += rs1_rdport

        rs2_rdport = register_bank.get_port(async_read=True)
        self.specials += rs2_rdport

        rd_wrport = register_bank.get_port(write_capable=True)
        self.specials += rd_wrport

        instr = Signal(32)
        instr_type = Signal(7)
        self.comb += instr_type.eq(instr[:7])

        isALUreg = Signal()
        isALUimm = Signal()
        isBranch = Signal()
        isJALR = Signal()
        isJAL = Signal()
        isAUIPC = Signal()
        isLUI = Signal()
        isLoad = Signal()
        isStore = Signal()
        isSYSTEM = Signal()
        self.comb += [
            isALUreg.eq(instr_type == 0b0110011),  # rd <- rs1 OP rs2
            isALUimm.eq(instr_type == 0b0010011),  # rd <- rs1 OP Iimm
            isBranch.eq(instr_type == 0b1100011),  # if(rs1 OP rs2) PC<-PC+Bimm
            isJALR.eq(instr_type == 0b1100111),  # rd <- PC+4; PC<-rs1+Iimm
            isJAL.eq(instr_type == 0b1101111),  # rd <- PC+4; PC<-PC+Jimm
            isAUIPC.eq(instr_type == 0b0010111),  # rd <- PC + Uimm
            isLUI.eq(instr_type == 0b0110111),  # rd <- Uimm
            isLoad.eq(instr_type == 0b0000011),  # rd <- mem[rs1+Iimm]
            isStore.eq(instr_type == 0b0100011),  # mem[rs1+Simm] <- rs2
            isSYSTEM.eq(instr_type == 0b1110011),  # special
        ]

        rdId = Signal(5)
        funct3 = Signal(3)
        rs1Id = Signal(5)
        rs2Id = Signal(5)
        funct7 = Signal(7)
        self.comb += [
            rdId.eq(instr[7:12]),
            funct3.eq(instr[12:15]),
            rs1Id.eq(instr[15:20]),
            rs2Id.eq(instr[20:25]),
            funct7.eq(instr[25:]),
        ]

        sign = Replicate(instr[31], 20)
        Uimm = Signal(32)
        Iimm = Signal(32)
        Simm = Signal(32)
        Bimm = Signal(32)
        Jimm = Signal(32)
        self.comb += [
            Uimm.eq(instr & 0xFFFFF000),
            Iimm.eq(Cat(rs2Id, funct7, sign)),
            Simm.eq(Cat(rdId, funct7, sign)),
            Bimm.eq(Cat(rdId & 0b11110, funct7[:6], rdId[0], sign)),
            Jimm.eq(Cat(rs2Id & 0b11110, funct7[:6], rs2Id[0], sign)),
        ]

        rs1 = Signal(32)
        rs2 = Signal(32)
        rs1Signed = Signal((32, True))
        rs2Signed = Signal((32, True))
        self.comb += [
            rs1Signed.eq(rs1),
            rs2Signed.eq(rs2),
        ]

        aluIn1 = rs1
        aluIn2 = Mux(isALUreg, rs2, Iimm)
        aluOut = Signal(32)
        shift_amount = Mux(isALUreg, rs2[:5], rs2Id)
        aluIn1Signed = Signal((32, True))
        aluIn2Signed = Signal((32, True))
        self.comb += [
            aluIn1Signed.eq(aluIn1),
            aluIn2Signed.eq(aluIn2),
        ]

        self.comb += Case(
            funct3,
            {
                0b000: aluOut.eq(
                    Mux(funct7[5] & instr[5], aluIn1 - aluIn2, aluIn1 + aluIn2)
                ),
                0b001: aluOut.eq(aluIn1 << shift_amount),
                0b010: aluOut.eq(aluIn1Signed < aluIn2Signed),
                0b011: aluOut.eq(aluIn1 < aluIn2),
                0b100: aluOut.eq(aluIn1 ^ aluIn2),
                0b101: aluOut.eq(
                    Mux(funct7[5], aluIn1Signed >> shift_amount, aluIn1 >> shift_amount)
                ),
                0b110: aluOut.eq(aluIn1 | aluIn2),
                0b111: aluOut.eq(aluIn1 & aluIn2),
            },
        )

        take_branch = Signal()
        self.comb += Case(
            funct3,
            {
                0b000: take_branch.eq(rs1 == rs2),
                0b001: take_branch.eq(rs1 != rs2),
                0b100: take_branch.eq(rs1Signed < rs2Signed),
                0b101: take_branch.eq(rs1Signed >= rs2Signed),
                0b110: take_branch.eq(rs1 < rs2),
                0b111: take_branch.eq(rs1 >= rs2),
                "default": take_branch.eq(0),
            },
        )

        mem_dat_r = Signal(32)
        mem_dat_w = Signal(32)

        loadstore_addr = Signal(32)
        LOAD_halfword = Signal(16)
        LOAD_byte = Signal(8)
        LOAD_sign = Signal()
        LOAD_data = Signal(32)
        mem_byteAccess = funct3[:2] == 0b00
        mem_halfwordAccess = funct3[:2] == 0b01
        self.comb += [
            loadstore_addr.eq(rs1 + Mux(isLoad, Iimm, Simm)),
            LOAD_halfword.eq(
                Mux(
                    loadstore_addr[1],
                    mem_dat_r[16:],
                    mem_dat_r[:16],
                )
            ),
            LOAD_byte.eq(Mux(loadstore_addr[0], LOAD_halfword[8:], LOAD_halfword[:8])),
            LOAD_sign.eq(
                ~funct3[2] & Mux(mem_byteAccess, LOAD_byte[7], LOAD_halfword[15])
            ),
            If(
                mem_byteAccess,
                LOAD_data.eq(Cat(LOAD_byte, Replicate(LOAD_sign, 24))),
            )
            .Elif(
                mem_halfwordAccess,
                LOAD_data.eq(Cat(LOAD_halfword, Replicate(LOAD_sign, 16))),
            )
            .Else(
                LOAD_data.eq(mem_dat_r),
            ),
            mem_dat_w.eq(
                Cat(
                    rs2[:8],
                    Mux(loadstore_addr[0], rs2[:8], rs2[8:16]),
                    Mux(loadstore_addr[1], rs2[:8], rs2[16:24]),
                    Mux(
                        loadstore_addr[0],
                        rs2[:8],
                        Mux(loadstore_addr[1], rs2[8:16], rs2[24:]),
                    ),
                )
            ),
        ]

        self.fsm = FSM(reset_state="FETCH_INSTR")
        self.fsm.act(
            "FETCH_INSTR",
            NextValue(idbus.adr, self.PC),
            NextValue(idbus.we, 0),
            NextState("WAIT_INSTR"),
        )
        self.fsm.act(
            "WAIT_INSTR",
            idbus.stb.eq(1),
            idbus.cyc.eq(1),
            If(
                idbus.ack,
                NextValue(instr, idbus.dat_r),
                NextState("FETCH_OPERANDS"),
            ),
        )
        self.fsm.act(
            "FETCH_OPERANDS",
            rs1_rdport.adr.eq(rs1Id),
            rs2_rdport.adr.eq(rs2Id),
            NextValue(rs1, rs1_rdport.dat_r),
            NextValue(rs2, rs2_rdport.dat_r),
            NextState("EXECUTE"),
        )
        self.fsm.act(
            "EXECUTE",
            rd_wrport.adr.eq(rdId),
            If(isJAL | isJALR, rd_wrport.dat_w.eq(self.PC + 4))
            .Elif(isLUI, rd_wrport.dat_w.eq(Uimm))
            .Elif(isAUIPC, rd_wrport.dat_w.eq(self.PC + Uimm))
            .Else(rd_wrport.dat_w.eq(aluOut)),
            rd_wrport.we.eq((~isBranch & ~isStore & ~isLoad) & (rdId > 0)),
            If(isJAL, NextValue(self.PC, self.PC + Jimm))
            .Elif(isJALR, NextValue(self.PC, rs1 + Iimm))
            .Elif(isBranch & take_branch, NextValue(self.PC, self.PC + Bimm))
            .Else(NextValue(self.PC, self.PC + 4)),
            If(isLoad, NextState("LOAD"))
            .Elif(isStore, NextState("STORE"))
            .Else(
                NextState("FETCH_INSTR"),
            ),
        )
        self.fsm.act(
            "LOAD",
            NextValue(idbus.adr, loadstore_addr),
            NextValue(idbus.we, 0),
            NextState("WAIT_LOAD"),
        )
        self.fsm.act(
            "WAIT_LOAD",
            idbus.stb.eq(1),
            idbus.cyc.eq(1),
            If(
                idbus.ack,
                NextValue(mem_dat_r, idbus.dat_r),
                rd_wrport.adr.eq(rdId),
                rd_wrport.dat_w.eq(LOAD_data),
                rd_wrport.we.eq(rdId > 0),
                NextState("FETCH_INSTR"),
            ),
        )
        self.fsm.act(
            "STORE",
            NextValue(idbus.adr, loadstore_addr),
            NextValue(idbus.dat_w, mem_dat_w),
            NextValue(
                idbus.sel,
                Mux(
                    mem_byteAccess,
                    Mux(
                        loadstore_addr[1],
                        Mux(loadstore_addr[0], 0b1000, 0b0100),
                        Mux(loadstore_addr[0], 0b0010, 0b0001),
                    ),
                    Mux(
                        mem_halfwordAccess,
                        Mux(loadstore_addr[1], 0b1100, 0b0011),
                        0b1111,
                    ),
                ),
            ),
            NextValue(idbus.we, 1),
            NextState("WAIT_STORE"),
        )
        self.fsm.act(
            "WAIT_STORE",
            idbus.stb.eq(1),
            idbus.cyc.eq(1),
            If(
                idbus.ack,
                NextState("FETCH_INSTR"),
            ),
        )

        self.sync += [
            If(
                self.fsm.ongoing("FETCH_INSTR"),
                Display(""),
                Display("FETCH PC=%d", self.PC),
            ),
            If(
                self.fsm.ongoing("FETCH_OPERANDS"),
                Display("FETCH rs1=%d rs2=%d", rs1Id, rs2Id),
            ),
            If(
                self.fsm.ongoing("EXECUTE"),
                #       "0000000_11111_00011_001_00011_0010011"
                Display("         rs2   rs1       rd          "),
                Display(
                    "%b_%b_%b_%b_%b_%b",
                    funct7,
                    rs2Id,
                    rs1Id,
                    funct3,
                    rdId,
                    instr_type,
                ),
                If(
                    isALUreg,
                    Display(
                        "ALUreg rd=%d rs1=%d rs2=%d funct3=%b",
                        rdId,
                        rs1Id,
                        rs2Id,
                        funct3,
                    ),
                ),
                If(
                    isALUimm,
                    Display(
                        "ALUimm rd=%d rs1=%d imm=%0d funct3=%b",
                        rdId,
                        rs1Id,
                        Iimm,
                        funct3,
                    ),
                ),
                If(isBranch, Display("BRANCH")),
                If(isJAL, Display("JAL")),
                If(isJALR, Display("JALR")),
                If(isAUIPC, Display("AUIPC")),
                If(isLUI, Display("LUI")),
                If(isLoad, Display("LOAD")),
                If(isStore, Display("STORE")),
                If(isSYSTEM, Display("SYSTEM")),
                If(
                    (
                        ~isALUreg
                        & ~isALUimm
                        & ~isBranch
                        & ~isJAL
                        & ~isJALR
                        & ~isAUIPC
                        & ~isLUI
                        & ~isLoad
                        & ~isStore
                        & ~isSYSTEM
                    ),
                    Display("UNKNOWN %b", instr),
                ),
                Display(
                    "EXECUTE (%b | %b | %b | %b) & (%d > 0)",
                    isALUreg,
                    isALUimm,
                    isJAL,
                    isJALR,
                    rdId,
                ),
                If(rd_wrport.we, Display("rd=%d <- %d", rdId, rd_wrport.dat_w)),
            ),
            If(
                self.fsm.ongoing("LOAD"),
                Display("LOAD %x", loadstore_addr),
            ),
            If(
                self.fsm.ongoing("STORE"),
                Display("STORE %x <- %x", loadstore_addr, mem_dat_w),
            ),
        ]

    def set_reset_address(self, reset_address):
        self.reset_address = reset_address
        self.PC.reset = reset_address

    def do_finalize(self):
        assert hasattr(self, "reset_address")
