#===================================================================
#
# Makefile
# --------
# Makefile for compiling the building the rosc entropy
# core and and top level simulations.
#
#
# Author: Joachim Strombergson
# Copyright (c) 2015, NORDUnet A/S All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
# - Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
#
# - Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
# - Neither the name of the NORDUnet nor the names of its contributors may
#   be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#===================================================================

CORE_SRC=../src/rtl/rosc_entropy_core.v ../src/rtl/rosc.v
TOP_SRC=../src/rtl/rosc_entropy.v


CC=iverilog
CC_FLAGS = -Wall

LINT=verilator
LINT_FLAGS = +1364-2001ext+ --lint-only  -Wall -Wno-fatal -Wno-DECLFILENAME


all: rosc.sim


rosc.sim: $(TOP_SRC) $(CORE_SRC)
	$(CC) $(CC_FLAGS) -o rosc.sim $(TOP_SRC) $(CORE_SRC)


sim-rosc: rosc.sim
	./rosc.sim


lint:  $(TOP_SRC) $(CORE_SRC)
	$(LINT) $(LINT_FLAGS) --top-module rosc_entropy $(TOP_SRC) $(CORE_SRC)


clean:
	rm -f *.sim


help:
	@echo "Build system for simulation of rosc entropy core."
	@echo ""
	@echo "Supported targets:"
	@echo "------------------"
	@echo "all:         Build all simulation targets."
	@echo "rosc.sim:    Build the ROSC sim target."
	@echo "sim-rosc:    Run ROSC simulation."
	@echo "lint:        Run linter on the ROSC source."
	@echo "clean:       Delete all built files."

#===================================================================
# EOF Makefile
#===================================================================
