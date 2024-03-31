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

        CSR_BASE = 0x82000000
        CSR_LEDS_OUT_ADDR = CSR_BASE + 0x1000

        led = Signal()
        counter = Signal(26)
        self.comb += led.eq(counter[22])
        self.sync += counter.eq(counter + 1)

        latch = Signal()
        write = 1
        read = 0

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
