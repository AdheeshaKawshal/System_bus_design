module master #(
    parameter ADDR_W = 15,
    parameter DATA_W = 8
)(
    input wire clk,
    input wire rst,

    // Arbiter interface
    output reg req_o,
    input wire grant_i,

    // Bus interface (driven only while granted)
    output reg [ADDR_W-1:0] addr_o,
    output reg [DATA_W-1:0] wdata_o,
    output reg we_o,
    output reg valid_o,

    input wire [DATA_W-1:0] rdata_i,
    input wire ready_i
);

    localparam IDLE    = 2'd0,
               REQUEST = 2'd1,
               ACTIVE  = 2'd2;

    reg [1:0] state;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state   <= IDLE;
            req_o   <= 1'b0;
            valid_o <= 1'b0;
            addr_o  <= {ADDR_W{1'b0}};
            wdata_o <= {DATA_W{1'b0}};
            we_o    <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    // TODO: replace with real trigger condition
                    state <= REQUEST;
                    req_o <= 1'b1;
                end

                REQUEST: begin
                    if (grant_i) begin
                        state   <= ACTIVE;
                        valid_o <= 1'b1;
                    end
                end

                ACTIVE: begin
                    if (ready_i) begin
                        state   <= IDLE;
                        req_o   <= 1'b0;
                        valid_o <= 1'b0;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
