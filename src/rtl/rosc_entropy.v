//======================================================================
//
// rosc_entropy.v
// --------------
// Top level wrapper for the ring oscillator entropy core.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2014, NORDUnet A/S All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// - Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// - Neither the name of the NORDUnet nor the names of its contributors may
//   be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module rosc_entropy(
                    input wire           clk,
                    input wire           reset,

                    input wire           cs,
                    input wire           we,
                    input wire  [7 : 0]  address,
                    input wire  [31 : 0] write_data,
                    output wire [31 : 0] read_data
                   );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam ADDR_NAME0       = 8'h00;
  localparam ADDR_NAME1       = 8'h01;
  localparam ADDR_VERSION     = 8'h02;

  localparam ADDR_CTRL        = 8'h08;
  localparam CTRL_ENABLE_BIT  = 0;

  localparam ADDR_STATUS      = 8'h09;
  localparam STATUS_VALID_BIT = 1;

  localparam ADDR_OP_A        = 8'h18;
  localparam ADDR_OP_B        = 8'h19;

  localparam ADDR_ENTROPY     = 8'h20;
  localparam ADDR_RAW         = 8'h21;
  localparam ADDR_ROSC_OUTPUTS= 8'h22;

  localparam DEFAULT_OP_A     = 32'haaaaaaaa;
  localparam DEFAULT_OP_B     = ~DEFAULT_OP_A;

  localparam CORE_NAME0       = 32'h726f7363; // "rosc"
  localparam CORE_NAME1       = 32'h20656e74; // " ent"
  localparam CORE_VERSION     = 32'h302e3130; // "0.10"


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg          enable_reg;
  reg          enable_we;

  reg [31 : 0] op_a_reg;
  reg          op_a_we;

  reg [31 : 0] op_b_reg;
  reg          op_b_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  wire [31 : 0] core_entropy_data;
  wire          core_entropy_valid;
  reg           core_entropy_ack;
  wire [31 : 0] core_raw_entropy;
  wire [31 : 0] core_rosc_outputs;

  reg [31 : 0]  tmp_read_data;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign read_data            = tmp_read_data;


  //----------------------------------------------------------------
  // module instantiations.
  //----------------------------------------------------------------
  rosc_entropy_core core(
                         .clk(clk),
                         .reset(reset),

                         .enable(enable_reg),

                         .opa(op_a_reg),
                         .opb(op_b_reg),

                         .raw_entropy(core_raw_entropy),
                         .rosc_outputs(core_rosc_outputs),
                         .entropy_data(core_entropy_data),
                         .entropy_valid(core_entropy_valid),
                         .entropy_ack(core_entropy_ack)
                        );


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk or posedge reset)
    begin
      if (reset)
        begin
          enable_reg <= 1'h1;
          op_a_reg   <= DEFAULT_OP_A;
          op_b_reg   <= DEFAULT_OP_B;
        end
      else
        begin
          if (enable_we)
            enable_reg <= write_data[CTRL_ENABLE_BIT];

          if (op_a_we)
            op_a_reg <= write_data;

          if (op_b_we)
            op_b_reg <= write_data;
         end
    end // reg_update


  //----------------------------------------------------------------
  // api_logic
  //
  // Implementation of the api logic. If cs is enabled will either
  // try to write to or read from the internal registers.
  //----------------------------------------------------------------
  always @*
    begin : api_logic
      enable_we        = 1'h0;
      op_a_we          = 1'h0;
      op_b_we          = 1'h0;
      core_entropy_ack = 1'h0;
      tmp_read_data    = 32'h0;

      if (cs)
        begin
          if (we)
            begin
              case (address)
                ADDR_CTRL: enable_we = 1;

                ADDR_OP_A: op_a_we   = 1;

                ADDR_OP_B: op_b_we   = 1;

                default:
                  begin
                  end
              endcase // case (address)
            end

          else
            begin
              case (address)
                ADDR_NAME0:   tmp_read_data = CORE_NAME0;

                ADDR_NAME1:   tmp_read_data = CORE_NAME1;

                ADDR_VERSION: tmp_read_data = CORE_VERSION;

                ADDR_CTRL:    tmp_read_data[CTRL_ENABLE_BIT] = enable_reg;

                ADDR_STATUS:  tmp_read_data[STATUS_VALID_BIT] = core_entropy_valid;

                ADDR_OP_A:    tmp_read_data = op_a_reg;

                ADDR_OP_B:    tmp_read_data = op_b_reg;

                ADDR_ENTROPY:
                  begin
                    tmp_read_data    = core_entropy_data;
                    core_entropy_ack = 1'h1;
                  end

                ADDR_RAW:          tmp_read_data = core_raw_entropy;

                ADDR_ROSC_OUTPUTS: tmp_read_data = core_rosc_outputs;

                default:
                  begin
                  end
              endcase // case (address)
            end
        end
    end

endmodule // rosc_entropy_core

//======================================================================
// EOF rosc_entropy_core.v
//======================================================================
