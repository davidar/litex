#
# This file is part of LiteX.
#
# Copyright (c) 2015-2022 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2017-2018 Tim 'mithro' Ansell <me@mith.ro>
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys
import inspect
import importlib

from migen import *

from litex.gen import *

# GPU (Generic) ------------------------------------------------------------------------------------

class GPU(LiteXModule):
    category             = None
    family               = None
    name                 = None
    data_width           = None
    endianness           = None
    gcc_triple           = None
    gcc_flags            = None
    clang_triple         = None
    clang_flags          = None
    linker_output_format = None
    interrupts           = {}
    mem_map              = {"csr": 0x82000000}
    io_regions           = {}
    use_rom              = False
    csr_decode           = True
    reset_address_check  = True

    def __init__(self, *args, **kwargs):
        pass

    def set_reset_address(self, reset_address):
        pass # pass must be overloaded (if required)

    def enable_reset_address_check(self):
        self.reset_address_check = True

    def disable_reset_address_check(self):
        self.reset_address_check = False

# GPU None (Used for SoC without a GPU) ------------------------------------------------------------

class GPUNone(GPU):
    variants            = ["standard"]
    endianness          = "little"
    reset_address       = 0x00000000
    reset_address_check = False
    periph_buses        = []
    memory_buses        = []
    mem_map             = {
        "csr"      : 0x0000_0000,
        "ethmac"   : 0x0002_0000, # FIXME: Remove.
        "spiflash" : 0x1000_0000, # FIXME: Remove.
    }

    def __init__(self, data_width=32, addr_width=32):
        self.io_regions = {0: int(2**float(addr_width))} # origin, length
        self.data_width = data_width

# GPUs Collection ----------------------------------------------------------------------------------

def collect_gpus():
    gpus  = {"None" : GPUNone}
    paths = [
        # Add litex.soc.cores.gpu path.
        os.path.dirname(__file__),
        # Add execution path.
        os.getcwd()
    ]

    exec_dir = os.getcwd()

    # Search for GPUs in paths.
    for path in paths:
        for file in os.listdir(path):

            # Verify that it's a path...
            gpu_path = os.path.join(path, file)
            if not os.path.isdir(gpu_path):
                continue

            # ... and that core.py is present.
            gpu_core = os.path.join(gpu_path, "core.py")
            if not os.path.exists(gpu_core):
                continue

            # OK, it seems to be a GPU; now get the class and add it to dict.
            gpu = file
            sys.path.append(path)
            for gpu_name, gpu_cls in inspect.getmembers(importlib.import_module(gpu), inspect.isclass):
                if gpu_name.lower() in [gpu, gpu.replace("_", "")]:
                    gpus[gpu] = gpu_cls

    # Return collected GPUs.
    return gpus

GPUS = collect_gpus()
