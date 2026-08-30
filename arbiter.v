module arbiter (
    input wire clk,
    input wire rst,

    input wire req_M0,
    input wire req_M1,
    input wire split,
    input wire resume,
    input wire xfer_done,   // pulses when the transfer currently on the bus completes

    // Fed back from addr_decoder: the address currently on the bus
    // (belonging to cur_owner) falls outside this bus. Used to figure
    // out, per master, whether its requests should route to the local
    // bus or the external one.
    input wire ext_valid_i,

    output reg grant_M0,
    output reg grant_M1,

    output reg addr_sel,
    output reg data_sel,
    output reg ctr_sel,

    output reg parked_id,

    // Per-master routing flags for control_mux, latched from ext_valid_i
    // at the moment that master owns the bus and its address is decoded.
    output reg ext_sel_M0,
    output reg ext_sel_M1
);
    // ---------------------------------------------------------
    // FSM states
    // ---------------------------------------------------------
    // IDLE        : bus idle, nobody granted.
    // BUSY        : cur_owner owns the bus normally (no park in effect).
    // PARKED_IDLE : parked_id is split-parked; bus currently idle, waiting
    //               either for resume or for the other master to request.
    // PARKED_BUSY : parked_id is split-parked; the OTHER master (cur_owner)
    //               has been lent the bus in the meantime.
    localparam IDLE        = 2'd0,
               BUSY         = 2'd1,
               PARKED_IDLE  = 2'd2,
               PARKED_BUSY  = 2'd3;

    reg [1:0] state;
    reg       cur_owner;      // 0=M0, 1=M1 : who currently drives the physical bus
    reg       resume_pending; // resume arrived while the other master was borrowing the bus

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state          <= IDLE;
            grant_M0       <= 1'b0;
            grant_M1       <= 1'b0;
            addr_sel       <= 1'b0;
            data_sel       <= 1'b0;
            ctr_sel        <= 1'b0;
            cur_owner      <= 1'b0;
            parked_id      <= 1'b0;
            resume_pending <= 1'b0;
            ext_sel_M0     <= 1'b0;
            ext_sel_M1     <= 1'b0;
        end else begin
            // While a master holds the grant, latch its routing flag from
            // the live decode result. Once it loses the grant, drop back
            // to 0 (internal) so its *next* request starts fresh instead
            // of inheriting a stale decision from an unrelated transfer.
            ext_sel_M0 <= grant_M0 ? ext_valid_i : 1'b0;
            ext_sel_M1 <= grant_M1 ? ext_valid_i : 1'b0;

            case (state)
                // ---------------------------------------------------
                IDLE: begin
                    if (req_M0) begin
                        grant_M0  <= 1'b1;
                        cur_owner <= 1'b0;
                        addr_sel  <= 1'b0; data_sel <= 1'b0; ctr_sel <= 1'b0;
                        state     <= BUSY;
                    end else if (req_M1) begin
                        grant_M1  <= 1'b1;
                        cur_owner <= 1'b1;
                        addr_sel  <= 1'b1; data_sel <= 1'b1; ctr_sel <= 1'b1;
                        state     <= BUSY;
                    end
                end

                // ---------------------------------------------------
                BUSY: begin
                    if (split) begin
                        // Park whoever currently owns the bus and free it
                        // up so the other master can be granted meanwhile.
                        parked_id      <= cur_owner;
                        grant_M0       <= 1'b0;
                        grant_M1       <= 1'b0;
                        resume_pending <= 1'b0;
                        state          <= PARKED_IDLE;
                    end else if (cur_owner == 1'b0 && !req_M0) begin
                        grant_M0 <= 1'b0;
                        state    <= IDLE;
                    end else if (cur_owner == 1'b1 && !req_M1) begin
                        grant_M1 <= 1'b0;
                        state    <= IDLE;
                    end
                end

                // ---------------------------------------------------
                PARKED_IDLE: begin
                    if (resume) begin
                        // Nobody borrowed the bus meanwhile: hand it
                        // straight back to the parked master.
                        grant_M0  <= (parked_id == 1'b0);
                        grant_M1  <= (parked_id == 1'b1);
                        cur_owner <= parked_id;
                        addr_sel  <= parked_id;
                        data_sel  <= parked_id;
                        ctr_sel   <= parked_id;
                        state     <= BUSY;
                    end else if (parked_id == 1'b0 && req_M1) begin
                        // M0 is parked, M1 wants the bus: lend it out.
                        grant_M1  <= 1'b1;
                        cur_owner <= 1'b1;
                        addr_sel  <= 1'b1; data_sel <= 1'b1; ctr_sel <= 1'b1;
                        state     <= PARKED_BUSY;
                    end else if (parked_id == 1'b1 && req_M0) begin
                        // M1 is parked, M0 wants the bus: lend it out.
                        grant_M0  <= 1'b1;
                        cur_owner <= 1'b0;
                        addr_sel  <= 1'b0; data_sel <= 1'b0; ctr_sel <= 1'b0;
                        state     <= PARKED_BUSY;
                    end
                end

                // ---------------------------------------------------
                PARKED_BUSY: begin
                    // Latch resume even if it arrives mid-transfer; only
                    // act on it at a clean transfer boundary so the
                    // borrowing master is never yanked off mid-transfer.
                    if (resume) begin
                        resume_pending <= 1'b1;
                    end

                    if (xfer_done && (resume || resume_pending)) begin
                        // Boundary reached and the parked master wants
                        // back on: give it the bus immediately.
                        grant_M0       <= (parked_id == 1'b0);
                        grant_M1       <= (parked_id == 1'b1);
                        cur_owner      <= parked_id;
                        addr_sel       <= parked_id;
                        data_sel       <= parked_id;
                        ctr_sel        <= parked_id;
                        resume_pending <= 1'b0;
                        state          <= BUSY;
                    end else if (xfer_done &&
                                 ((cur_owner == 1'b0 && !req_M0) ||
                                  (cur_owner == 1'b1 && !req_M1))) begin
                        // Borrowing master ran out of work on its own;
                        // free the bus but keep waiting for resume.
                        grant_M0 <= 1'b0;
                        grant_M1 <= 1'b0;
                        state    <= PARKED_IDLE;
                    end
                    // else: let the borrowing master keep using the bus
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
