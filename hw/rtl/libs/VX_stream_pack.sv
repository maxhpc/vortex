// Copyright © 2019-2023
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

`include "VX_platform.vh"

`TRACING_OFF
module VX_stream_pack #(
    parameter NUM_REQS      = 1, 
    parameter DATA_WIDTH    = 1, 
    parameter TAG_WIDTH     = 1,    
    parameter TAG_SEL_BITS  = 0,
    parameter `STRING ARBITER = "P",
    parameter OUT_BUF       = 0
) (
    input wire                          clk,
    input wire                          reset,

    // input
    input wire [NUM_REQS-1:0]           valid_in,
    input wire [NUM_REQS-1:0][DATA_WIDTH-1:0] data_in,
    input wire [NUM_REQS-1:0][TAG_WIDTH-1:0] tag_in,
    output wire [NUM_REQS-1:0]          ready_in,

    // output
    output wire                         valid_out,
    output wire [NUM_REQS-1:0]          mask_out,
    output wire [NUM_REQS-1:0][DATA_WIDTH-1:0] data_out,
    output wire [TAG_WIDTH-1:0]         tag_out,
    input wire                          ready_out
);
    localparam LOG_NUM_REQS = `CLOG2(NUM_REQS);

    if (NUM_REQS > 1) begin

        wire [LOG_NUM_REQS-1:0] grant_index;
        wire grant_valid;
        wire grant_ready;

        VX_generic_arbiter #(
            .NUM_REQS (NUM_REQS),
            .LOCK_ENABLE (1),
            .TYPE (ARBITER)
        ) arbiter (
            .clk         (clk),
            .reset       (reset),
            .requests    (valid_in), 
            .grant_valid (grant_valid),
            .grant_index (grant_index),
            `UNUSED_PIN (grant_onehot),
            .grant_unlock(grant_ready)
        );

        reg [NUM_REQS-1:0] valid_sel;
        reg [NUM_REQS-1:0] ready_sel;
        wire ready_unqual;

        wire [TAG_WIDTH-1:0] tag_sel = tag_in[grant_index];
        
        always @(*) begin                
            valid_sel = '0;              
            ready_sel = '0;            
            for (integer i = 0; i < NUM_REQS; ++i) begin
                if (tag_in[i][TAG_SEL_BITS-1:0] == tag_sel[TAG_SEL_BITS-1:0]) begin
                    valid_sel[i] = valid_in[i];                    
                    ready_sel[i] = ready_unqual;
                end
            end
        end                            

        assign grant_ready = ready_unqual;
        
        VX_elastic_buffer #(
            .DATAW   (NUM_REQS + TAG_WIDTH + (NUM_REQS * DATA_WIDTH)),
            .SIZE    (`TO_OUT_BUF_SIZE(OUT_BUF)),
            .OUT_REG (`TO_OUT_BUF_REG(OUT_BUF))
        ) out_buf (
            .clk       (clk),
            .reset     (reset),
            .valid_in  (grant_valid),        
            .data_in   ({valid_sel, tag_sel, data_in}),
            .ready_in  (ready_unqual),      
            .valid_out (valid_out),
            .data_out  ({mask_out, tag_out, data_out}),
            .ready_out (ready_out)
        );  

        assign ready_in = ready_sel;     
        
    end else begin

        `UNUSED_VAR (clk)
        `UNUSED_VAR (reset)
        assign valid_out = valid_in;
        assign mask_out  = 1'b1;
        assign data_out  = data_in;
        assign tag_out   = tag_in;
        assign ready_in  = ready_out;

    end

endmodule
`TRACING_ON
