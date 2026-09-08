`timescale 1ns / 1ps
module tb_debug_remote;
    reg clk, rst;
    reg [3:0] pkt_sel;
    reg pkt_valid;
    wire [3:0] led;
    wire mc_tx, sc_tx;

    // Self-loop this single board's bridge back onto itself: its own
    // bb_slave_core (outgoing remote request) talks to its own
    // bb_master_core (incoming remote request), so entry 7/8's "other bus"
    // is this same board's bus, serviced twice through the bridge chain.
    serial_bus_top #(.ADDR_W(15), .DATA_W(8), .RW(1), .NUM_SLAVES(3),
                      .M0_START_TXN(0), .M0_REQ_DELAY(10), .M0_WRITE_DELAY(26))
        dut (.clk(clk), .rst(rst),
             .mc_uart_tx_o(mc_tx), .mc_uart_rx_i(sc_tx),
             .sc_uart_tx_o(sc_tx), .sc_uart_rx_i(mc_tx),
             .m0_pkt_sel_i(pkt_sel), .pkt_valid_i(pkt_valid),
             .led_o(led));

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        rst = 1'b1; pkt_sel = 4'd0; pkt_valid = 1'b0;
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;
    end

    task select_txn(input [3:0] idx);
        begin
            @(posedge clk);
            pkt_sel = idx;
            pkt_valid = 1'b1;
            wait (dut.u_master0.state == 3'd2);
            @(posedge clk);
            pkt_valid = 1'b0;
            wait (dut.u_master0.state == 3'd1);
            $display("[%0t] txn %0d done, rdata_mem=%0d (0x%h)", $time, idx, dut.u_master0.rdata_mem[idx], dut.u_master0.rdata_mem[idx]);
        end
    endtask

    reg mc_frame_err_d, sc_frame_err_d;
    initial begin mc_frame_err_d = 0; sc_frame_err_d = 0; end
    always @(posedge clk) begin
        if (dut.u_bb_master_core.u_req_rx.frame_err_o !== mc_frame_err_d) begin
            $display("[%0t] bb_master_core RX frame_err_o -> %b", $time, dut.u_bb_master_core.u_req_rx.frame_err_o);
            mc_frame_err_d <= dut.u_bb_master_core.u_req_rx.frame_err_o;
        end
        if (dut.u_bb_slave_core.u_reply_rx.frame_err_o !== sc_frame_err_d) begin
            $display("[%0t] bb_slave_core reply RX frame_err_o -> %b", $time, dut.u_bb_slave_core.u_reply_rx.frame_err_o);
            sc_frame_err_d <= dut.u_bb_slave_core.u_reply_rx.frame_err_o;
        end
    end

    always @(dut.u_bb_slave_core.rstate)
        $display("[%0t] bb_slave_core.rstate -> %0d (tx_send=%b tx_busy=%b tx_done=%b)", $time,
            dut.u_bb_slave_core.rstate, dut.u_bb_slave_core.tx_send, dut.u_bb_slave_core.tx_busy, dut.u_bb_slave_core.tx_done);
    always @(dut.u_bb_slave_core.u_req_tx.state)
        $display("[%0t] bb_slave_core.u_req_tx.state -> %0d tx_o=%b", $time, dut.u_bb_slave_core.u_req_tx.state, dut.u_bb_slave_core.u_req_tx.tx_o);
    always @(dut.u_bb_master_core.u_req_rx.state)
        $display("[%0t] bb_master_core.u_req_rx.state -> %0d rx_sync=%b rx_i=%b", $time,
            dut.u_bb_master_core.u_req_rx.state, dut.u_bb_master_core.u_req_rx.rx_sync, dut.u_bb_master_core.uart_rx_i);

    // trace every hop of the remote round trip
    always @(posedge clk) begin
        if (dut.u_bb_slave_core.frame_done)
            $display("[%0t] bb_slave_core CAPTURED request we=%b addr=%h wdata=%0d(0x%h)", $time,
                dut.u_bb_slave_core.we_c, dut.u_bb_slave_core.addr_c, dut.u_bb_slave_core.wdata_c, dut.u_bb_slave_core.wdata_c);
        if (dut.u_bb_master_core.u_req_rx.valid_o)
            $display("[%0t] bb_master_core RX got request pkt_we=%b pkt_addr=%h pkt_wdata=%0d(0x%h)", $time,
                dut.u_bb_master_core.pkt_we, dut.u_bb_master_core.pkt_addr, dut.u_bb_master_core.pkt_wdata, dut.u_bb_master_core.pkt_wdata);
        if (dut.u_slave1.frame_done)
            $display("[%0t] u_slave1(far local slave, slave_sel2) frame_done we=%b addr=%h wdata=%0d(0x%h)", $time,
                dut.u_slave1.we_c, dut.u_slave1.addr_c, dut.u_slave1.wdata_c, dut.u_slave1.wdata_c);
        if (dut.u_bb_master_core.u_deserializer.data_valid)
            $display("[%0t] bb_master_core's own deserializer (captures far LOCAL bus reply) data_out=%0d(0x%h)", $time,
                dut.u_bb_master_core.u_deserializer.data_out, dut.u_bb_master_core.u_deserializer.data_out);
        if (dut.u_bb_master_core.m_txn_rdata_valid)
            $display("[%0t] bb_master_core.u_master txn_rdata_valid rdata=%0d(0x%h)", $time,
                dut.u_bb_master_core.m_txn_rdata, dut.u_bb_master_core.m_txn_rdata);
        if (dut.u_bb_master_core.u_reply_tx.send_i)
            $display("[%0t] bb_master_core UART reply TX send data=%0d(0x%h)", $time,
                dut.u_bb_master_core.rd_byte, dut.u_bb_master_core.rd_byte);
        if (dut.u_bb_slave_core.u_reply_rx.valid_o)
            $display("[%0t] bb_slave_core UART reply RX got data=%0d(0x%h)", $time,
                dut.u_bb_slave_core.rx_data, dut.u_bb_slave_core.rx_data);
        if (dut.u_master0.u_deserializer.data_valid)
            $display("[%0t] master0's OWN deserializer (final capture) data_out=%0d(0x%h)", $time,
                dut.u_master0.u_deserializer.data_out, dut.u_master0.u_deserializer.data_out);
    end

    initial begin
        wait (rst == 1'b0);
        #200;
        $display("=== select 7 (write 5 to remote addr 0x5008) ===");
        select_txn(7);
        #500;
        $display("=== select 8 (read remote addr 0x5008) ===");
        select_txn(8);
        #500;
        $display("DONE");
        $finish;
    end

    initial begin #200000; $display("TIMEOUT"); $finish; end
endmodule
