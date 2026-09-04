`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 08/29/2026 10:40:53 AM
// Design Name:
// Module Name: slave_split
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   Same write behaviour as slave.v (mem[addr] <= wdata on a captured,
//   selected write). A read, however, cannot be answered in one cycle:
//   the slave asserts split_o for one cycle to tell the arbiter to park
//   the current master and free the bus, waits WAIT_CYCLES fixed cycles,
//   then (once mready_i says the parked/resumed master is actually ready)
//   asserts resume_o for one cycle and shifts the read data back out on
//   rdata_o_ser/rvalid_o so the arbiter re-grants the parked master and
//   the transaction completes. mready_i is sampled only in RESUME - if
//   it's already high the cycle WAIT_CYCLES elapses, this costs zero
//   extra latency versus a plain fixed-cycle resume; if it's low, RESUME
//   just holds an extra cycle at a time until it goes high.
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   Serial rewrite: request in as a shared 24-bit {addr,we,wdata} frame
//   (addr_data_i/valid_i, gated by cs_i), response out via a Serializer
//   instance on rdata_o_ser/rvalid_o - mirrors slave.v's mechanics.
//
//////////////////////////////////////////////////////////////////////////////////

module slave_split #(
    parameter ADDR_W      = 12,
    parameter DATA_W      = 8,
    parameter RW          = 1,
    parameter WAIT_CYCLES = 10
)(
    input wire clk,
    input wire rst,

    input wire cs_i,        // chip select from the bus (slave_sel3 && !ext_redirect)
    input wire addr_data_i, // shared serial {addr,we,wdata} request frame, MSB first
    input wire valid_i,     // shared frame-start strobe
    input wire mready_i,    // granted master's mready, forwarded from the bus - gates RESUME

    output wire rdata_o_ser, // serial rdata response, MSB first
    output wire rvalid_o,    // held-high-during-frame response valid

    output reg  split_o,  // pulses 1 cycle: "park me, I can't answer next cycle"
    output reg  resume_o  // pulses 1 cycle, alongside the response frame starting
);

    reg [DATA_W-1:0] mem [0:15];

    // ---------------------------------------------------------
    // Frame capture: shared addr_data_deserializer does the shift-register
    // work, gated by cs_i (same as slave.v).
    // ---------------------------------------------------------
    wire              we_c;
    wire [14:0]       addr_c;
    wire [DATA_W-1:0] wdata_c;
    wire              frame_done;

    addr_data_deserializer #(
        .ADDR_W (15),
        .RW     (RW),
        .DATA_W (DATA_W)
    ) u_deserializer (
        .clk         (clk),
        .rst         (rst),
        .cs_i        (cs_i),
        .addr_data_i (addr_data_i),
        .valid_i     (valid_i),
        .we_o        (we_c),
        .addr_o      (addr_c),
        .wdata_o     (wdata_c),
        .frame_done  (frame_done)
    );

    // FSM states
    localparam IDLE     = 2'd0,
               WAIT      = 2'd1,
               RESUME    = 2'd2,
               WAIT_LOW  = 2'd3; // wait for the master to drop cs_i before going back to IDLE

    reg [1:0] state;
    reg [$clog2(WAIT_CYCLES+1)-1:0] wait_cnt;
    reg [3:0] addr_latch;
    reg [DATA_W-1:0] rdata_reg;
    reg              ser_trigger;
    integer          mem_i;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state       <= IDLE;
            split_o     <= 1'b0;
            resume_o    <= 1'b0;
            wait_cnt    <= 0;
            addr_latch  <= 4'b0;
            rdata_reg   <= {DATA_W{1'b0}};
            ser_trigger <= 1'b0;
            for (mem_i = 0; mem_i < 16; mem_i = mem_i + 1) begin
                mem[mem_i] <= {DATA_W{1'b0}};
            end
        end else begin
            // default: all pulses low unless set below
            split_o     <= 1'b0;
            resume_o    <= 1'b0;
            ser_trigger <= 1'b0;

            case (state)
                IDLE: begin
                    if (frame_done) begin
                        if (we_c) begin
                            // write completes immediately, same as slave.v
                            mem[addr_c[3:0]] <= wdata_c;
                        end else begin
                            // read: can't answer next cycle -> split the bus
                            addr_latch <= addr_c[3:0];
                            split_o    <= 1'b1;
                            wait_cnt   <= 0;
                            state      <= WAIT;
                        end
                    end
                end

                WAIT: begin
                    // hold for WAIT_CYCLES fixed cycles before resuming
                    if (wait_cnt == WAIT_CYCLES - 1) begin
                        state <= RESUME;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                RESUME: begin
                    // Wait here (costing no extra cycles if mready_i is
                    // already high) until the parked master is actually
                    // ready to accept the response, then fire it.
                    if (mready_i) begin
                        resume_o    <= 1'b1;
                        rdata_reg   <= mem[addr_latch];
                        ser_trigger <= 1'b1;
                        state       <= WAIT_LOW;
                    end
                end

                WAIT_LOW: begin
                    // The master only drops cs_i one cycle after seeing the
                    // response start, so cs_i may still be asserted here.
                    // Sit tight so we don't misread the stale cs_i as a
                    // fresh request, and only return to IDLE once it
                    // actually clears.
                    if (!cs_i) begin
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    Serializer u_serializer (
        .clk_in         (clk),
        .rst_n          (rst),
        .data_in        (rdata_reg),
        .data_valid     (ser_trigger),
        .serial_out     (rdata_o_ser),
        .data_valid_out (rvalid_o),
        .done           ()
    );

endmodule
