module arbiter (
    input wire clk,
    input wire rst,
    
    input wire req_M0,
    input wire req_M1,
    input wire split,
    input wire resume,
    
    
    output reg grant_M0,
    output reg grant_M1,

    // Mux control: which master currently owns the shared addr/data bus.
    // 0 = M0, 1 = M1. Stays latched across a split/resume gap so the
    // addr/data muxes keep routing the parked master's signals.
    output reg addr_sel,
    output reg data_sel,
    output reg ctr_sel
);
    // ---------------------------------------------------------
    // Internal state
    // ---------------------------------------------------------

    reg pending;
    reg split_master_id;
    reg master_sel;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            grant_M0 <= 1'b0;
            grant_M1 <= 1'b0;
            pending <= 1'b0;
            split_master_id <= 1'b0;
            master_sel <= 1'b0;
            addr_sel <= 1'b0;
            data_sel <= 1'b0;
            ctr_sel <= 1'b0;
        end
        else begin
            if (split) begin
                // If split is asserted, we need to remember which master was granted
                if (grant_M0) begin
                    split_master_id <= 1'b0; // Master 0 was granted
                    grant_M0 <= 1'b0; // Reset grant for Master 0
                end else if (grant_M1) begin
                    grant_M1 <= 1'b0; // Reset grant for Master 1
                    split_master_id <= 1'b1; // Master 1 was granted
                end
                // addr_sel/data_sel stay latched on the parked master
            end
            else if (resume)begin
                // If resume is asserted, we can grant access to the master that was previously granted
                addr_sel <= master_sel;
                data_sel <= master_sel;
                ctr_sel  <= master_sel;
                if (split_master_id == 1'b0) begin
                    grant_M0 <= 1'b1;
                    grant_M1 <= 1'b0;
                    master_sel <= 1'b0;
                end else begin
                    grant_M0 <= 1'b0;
                    grant_M1 <= 1'b1;
                    master_sel <= 1'b1;
                    
                end
            end
            else begin
                addr_sel <= master_sel;
                data_sel <= master_sel;
                ctr_sel  <= master_sel;
                // If split is not asserted, we can grant access based on requests
                if (req_M0 && !pending) begin
                    pending <= 1'b1;
                    grant_M0 <= 1'b1;
                    grant_M1 <= 1'b0;
                    master_sel <= 1'b0;

                end else if (req_M1 && !pending) begin
                    pending <= 1'b1;
                    grant_M0 <= 1'b0;
                    grant_M1 <= 1'b1;
                    master_sel <= 1'b1; 
                end else if (pending && (!req_M0 && !req_M1)) begin
                    // No requests or already granted, maintain current state
                    grant_M0 <= 1'b0;
                    grant_M1 <= 1'b0;
                    pending <= 1'b0;
                    master_sel <= 1'b0; // Reset to default master (M0) when no requests
                end
            end
        end
    end
endmodule
    