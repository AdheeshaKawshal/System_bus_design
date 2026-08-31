module slave #(
    parameter ADDR_W = 12,
    parameter DATA_W = 8
)(
    input wire clk,
    input wire rst,

    input wire                  cs_i,     // chip select from addr_decoder
    input wire                  valid_i,  // master has a valid transaction
    input wire                  we_i,     // 1 = write, 0 = read
    (* MARK_DEBUG = "TRUE" *) input wire [ADDR_W-1:0]     addr_i,
    (* MARK_DEBUG = "TRUE" *) input wire [DATA_W-1:0]     wdata_i,

    (* MARK_DEBUG = "TRUE" *) output reg [DATA_W-1:0]     rdata_o,  // data back to master on a read
    output reg                  ready_o,  // pulses when the transaction (write or read) is done
    output reg                  rvalid_o  // pulses when rdata_o holds valid read data
);

    reg [DATA_W-1:0] mem [0:15];

    wire sel = cs_i && valid_i;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rdata_o  <= {DATA_W{1'b0}};
            ready_o  <= 1'b0;
            rvalid_o <= 1'b0;
        end else begin
            ready_o  <= 1'b0;
            rvalid_o <= 1'b0;
            // Write only happens once the master has driven a valid,
            // selected transaction (sel = cs_i && valid_i) on this edge.
            if (sel) begin
                if (we_i) begin
                    mem[addr_i[3:0]] <= wdata_i;
                end else begin
                    rdata_o  <= mem[addr_i];
                    rvalid_o <= 1'b1;
                end
                ready_o <= 1'b1;
            end
        end
    end

endmodule
