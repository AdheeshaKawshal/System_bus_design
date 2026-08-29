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
//   Same write behaviour as slave.v (mem[addr_i] <= wdata_i on a valid,
//   selected write). A read, however, cannot be answered in one cycle:
//   the slave asserts split_o for one cycle to tell the arbiter to park
//   the current master and free the bus, waits WAIT_CYCLES fixed cycles,
//   then asserts resume_o for one cycle together with the read data
//   (rdata_o/rvalid_o/ready_o) so the arbiter re-grants the parked
//   master and the transaction completes.
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module slave_split #(
    parameter ADDR_W      = 12,
    parameter DATA_W      = 8,
    parameter WAIT_CYCLES = 10
)(
    input wire clk,
    input wire rst,

    input wire                  cs_i,     // chip select from addr_decoder
    input wire                  valid_i,  // master has a valid transaction
    input wire                  we_i,     // 1 = write, 0 = read
    input wire [ADDR_W-1:0]     addr_i,
    input wire [DATA_W-1:0]     wdata_i,

    output reg [DATA_W-1:0]     rdata_o,  // data back to master on a read
    output reg                  ready_o,  // pulses when the transaction (write or read) is done
    output reg                  rvalid_o, // pulses when rdata_o holds valid read data

    output reg                  split_o,  // pulses 1 cycle: "park me, I can't answer next cycle"
    output reg                  resume_o  // pulses 1 cycle, alongside the ready read data
);

    reg [DATA_W-1:0] mem [0:(1<<ADDR_W)-1];

    wire sel = cs_i && valid_i;

    // FSM states
    localparam IDLE     = 2'd0,
               WAIT      = 2'd1,
               RESUME    = 2'd2,
               WAIT_LOW  = 2'd3; // wait for the master to drop sel before going back to IDLE

    reg [1:0] state;
    reg [$clog2(WAIT_CYCLES+1)-1:0] wait_cnt;
    reg [ADDR_W-1:0] addr_latch;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state      <= IDLE;
            rdata_o    <= {DATA_W{1'b0}};
            ready_o    <= 1'b0;
            rvalid_o   <= 1'b0;
            split_o    <= 1'b0;
            resume_o   <= 1'b0;
            wait_cnt   <= 0;
            addr_latch <= {ADDR_W{1'b0}};
        end else begin
            // default: all pulses low unless set below
            ready_o  <= 1'b0;
            rvalid_o <= 1'b0;
            split_o  <= 1'b0;
            resume_o <= 1'b0;

            case (state)
                IDLE: begin
                    if (sel) begin
                        if (we_i) begin
                            // write completes immediately, same as slave.v
                            mem[addr_i] <= wdata_i;
                            ready_o     <= 1'b1;
                        end else begin
                            // read: can't answer next cycle -> split the bus
                            addr_latch <= addr_i;
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
                    // data is ready: tell the arbiter to resume the parked
                    // master and hand back the read data in the same cycle
                    resume_o <= 1'b1;
                    rdata_o  <= mem[addr_latch];
                    rvalid_o <= 1'b1;
                    ready_o  <= 1'b1;
                    state    <= WAIT_LOW;
                end

                WAIT_LOW: begin
                    // The master only drops sel one cycle after seeing
                    // ready_o, so sel is still asserted here. Sit tight so
                    // we don't misread the stale sel as a fresh request,
                    // and only return to IDLE once it actually clears.
                    if (!sel) begin
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
