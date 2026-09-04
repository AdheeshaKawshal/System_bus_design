module addr_redirect #(
    parameter NUM_SLAVES = 3,
    parameter SEL_W      = 2,
    parameter TAP_BITS   = SEL_W + 1,  // internal-flag + slave-select bits (3)
    parameter OUT_DELAY  = 4           // cycles the whole frame is held back, >= TAP_BITS+1 so slave_sel* is stable first
)(
    input  wire clk,
    input  wire rst_n,

    input  wire serial_in,        // serial frame line, MSB first
    input  wire frame_valid_in,   // strobe: a new frame starts now

    // latched right after the first TAP_BITS bits arrive - held until the
    // next frame starts
    output reg  slave_sel1,
    output reg  slave_sel2,
    output reg  slave_sel3,
    output reg  ext_redirect,  // alongside slave_sel3 only for an external address - tells S2 to bridge, not service
    output reg  addr_invalid,

    // serial_in/frame_valid_in, delayed OUT_DELAY cycles, forwarded as-is
    output wire addr_data_o,
    output wire frame_valid_o
);

    localparam TAP_CNT_W = $clog2(TAP_BITS + 1);

    // ---------------------------------------------------------
    // Tap capture: shift in just the first TAP_BITS bits of each frame to
    // decide the slave select early - the rest of the frame is never
    // inspected here, only delayed and passed through below.
    // ---------------------------------------------------------
    reg [TAP_BITS-1:0]   tap_shift;
    reg [TAP_CNT_W-1:0]  tap_count;
    reg                  tap_capturing;
    reg                  early_valid;
    reg [TAP_BITS-1:0]   early_bits;

    wire [TAP_BITS-1:0] tap_shift_next = {tap_shift[TAP_BITS-2:0], serial_in};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap_shift     <= {TAP_BITS{1'b0}};
            tap_count     <= {TAP_CNT_W{1'b0}};
            tap_capturing <= 1'b0;
            early_valid   <= 1'b0;
            early_bits    <= {TAP_BITS{1'b0}};
        end else begin
            early_valid <= 1'b0;

            if (!tap_capturing) begin
                if (frame_valid_in) begin
                    tap_shift     <= tap_shift_next;
                    tap_count     <= {{(TAP_CNT_W-1){1'b0}}, 1'b1};
                    tap_capturing <= 1'b1;
                end
            end else begin
                tap_shift <= tap_shift_next;
                tap_count <= tap_count + 1'b1;

                if ((tap_count + 1'b1) == TAP_BITS) begin
                    early_valid   <= 1'b1;
                    early_bits    <= tap_shift_next;
                    tap_capturing <= 1'b0;
                end
            end
        end
    end

    wire dec_sel1, dec_sel2, dec_sel3, dec_ext_redirect, dec_addr_invalid;

    addr_decoder #(
        .NUM_SLAVES (NUM_SLAVES),
        .SEL_W      (SEL_W)
    ) u_addr_decoder (
        .sel_bits     (early_bits),
        .valid_i      (early_valid),
        .slave_sel1   (dec_sel1),
        .slave_sel2   (dec_sel2),
        .slave_sel3   (dec_sel3),
        .ext_redirect (dec_ext_redirect),
        .addr_invalid (dec_addr_invalid)
    );

    // Latch the decode result and hold it for the whole transaction,
    // clearing only when the next frame starts.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slave_sel1   <= 1'b0;
            slave_sel2   <= 1'b0;
            slave_sel3   <= 1'b0;
            ext_redirect <= 1'b0;
            addr_invalid <= 1'b0;
        end else if (early_valid) begin
            slave_sel1   <= dec_sel1;
            slave_sel2   <= dec_sel2;
            slave_sel3   <= dec_sel3;
            ext_redirect <= dec_ext_redirect;
            addr_invalid <= dec_addr_invalid;
        end else if (frame_valid_in && !tap_capturing) begin
            slave_sel1   <= 1'b0;
            slave_sel2   <= 1'b0;
            slave_sel3   <= 1'b0;
            ext_redirect <= 1'b0;
            addr_invalid <= 1'b0;
        end
    end

    // ---------------------------------------------------------
    // Pure delay line: forward serial_in/frame_valid_in unchanged,
    // OUT_DELAY cycles later, so the slave sees the frame only after
    // slave_sel* above has settled.
    // ---------------------------------------------------------
    reg [OUT_DELAY-1:0] serial_delay;
    reg [OUT_DELAY-1:0] valid_delay;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            serial_delay <= {OUT_DELAY{1'b0}};
            valid_delay  <= {OUT_DELAY{1'b0}};
        end else begin
            serial_delay <= {serial_delay[OUT_DELAY-2:0], serial_in};
            valid_delay  <= {valid_delay[OUT_DELAY-2:0], frame_valid_in};
        end
    end

    assign addr_data_o   = serial_delay[OUT_DELAY-1];
    assign frame_valid_o = valid_delay[OUT_DELAY-1];

endmodule
