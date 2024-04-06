#
# This file is part of LiteX.
#
# Copyright (c) 2022 Sylvain Lefebvre <sylvain.lefebvre@inria.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *

from litex.gen import *

from litex.soc.interconnect import wishbone, stream
from litex.soc.interconnect.csr import CSRStorage
from litex.soc.cores.gpu import GPU
from litex.soc.cores.video import video_timing_layout

# Variants -----------------------------------------------------------------------------------------

GPU_VARIANTS = {
    "standard": "firev",
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

# FireV ------------------------------------------------------------------------------------------


class Raster(GPU):
    category = "softcore"
    variants = GPU_VARIANTS
    data_width = 32
    endianness = "little"
    nop = "nop"
    io_regions = {0x8000_0000: 0x8000_0000}  # Origin, Length.

    # GCC Flags.
    @property
    def gcc_flags(self):
        flags = GCC_FLAGS[self.variant]
        flags += " -D__firev__ "
        return flags

    def __init__(self, platform, variant="standard"):
        self.platform = platform
        self.variant = variant
        self.human_name = f"FireV-{variant.upper()}"
        self.reset = Signal()
        self.idbus = idbus = wishbone.Interface(
            data_width=32, adr_width=32, addressing="byte"
        )
        self.periph_buses = [idbus]  # Peripheral buses (Connected to main SoC's bus).
        self.memory_buses = []  # Memory buses (Connected directly to LiteDRAM).

        self.framebuffer_base = CSRStorage(32)
        self.framebuffer_base_out = Signal(32)

        self.vtg_sink = stream.Endpoint(video_timing_layout)

        fbuffer = Signal()
        valid = Signal()

        self.gpu_params = dict(
            # Clk / Rst.
            i_clock=ClockSignal("sys"),
            i_reset=(ResetSignal("sys") | self.reset),
            # Framebuffer.
            i_in_framebuffer_base=self.framebuffer_base.storage,
            o_out_fbuffer=fbuffer,
            # Video Timing Generator.
            i_in_vsync=self.vtg_sink.vsync,
            # I/D Bus.
            o_out_sd_addr=idbus.adr,
            o_out_sd_in_valid=valid,
            o_out_sd_wmask=idbus.sel,
            o_out_sd_data_in=idbus.dat_w,
            o_out_sd_rw=idbus.we,
            i_in_sd_done=idbus.ack,
            i_in_sd_data_out=idbus.dat_r,
        )

        self.comb += If(
            self.framebuffer_base.storage != 0,
            self.framebuffer_base_out.eq(
                self.framebuffer_base.storage + 640 * 480 * 4 * fbuffer
            ),
        )

        # Adapt FireV Mem Bus to Wishbone.
        # --------------------------------
        self.fsm = fsm = FSM(reset_state="WAIT")
        fsm.act(
            "WAIT", If(valid, idbus.stb.eq(1), idbus.cyc.eq(1), NextState("WB-ACCESS"))
        )
        fsm.act(
            "WB-ACCESS",
            idbus.stb.eq(1),
            idbus.cyc.eq(1),
            If(idbus.ack, NextState("WAIT")),
        )

        # self.sync += [
        #     If(
        #         valid,
        #         Display(
        #             "rw=%d, addr=%08x, data=%08x, mask=%x",
        #             idbus.we,
        #             idbus.adr,
        #             idbus.dat_w,
        #             idbus.sel,
        #         ),
        #     ),
        #     If(idbus.ack, Display("data=%08x", idbus.dat_r)),
        # ]

        # Add Verilog sources.
        # --------------------
        self.add_sources(platform, variant)

    def set_reset_address(self, reset_address):
        self.reset_address = reset_address
        # self.gpu_params.update(i_in_boot_at=Constant(reset_address, 32))

    @staticmethod
    def add_sources(platform, variant):
        platform.add_verilog_include_path(os.getcwd())
        platform.add_source("video_sdram_test.si.v")

    def do_finalize(self):
        assert hasattr(self, "reset_address")
        self.specials += Instance("M_frame_drawer", **self.gpu_params)
