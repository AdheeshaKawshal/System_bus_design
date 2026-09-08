`timescale 1ns / 1ps
module tb_debug_split;
    reg clk, rst;
    reg [3:0] m0_sel;
    reg m0_valid;
    wire [3:0] led;

    serial_2bus_top #(
        .M0_START_TXN(0), .M0_REQ_DELAY(5), .M0_WRITE_DELAY(10)
    ) dut (
        .clk (clk), .rst (rst),
        .sc_uart_tx_o (), .sc_uart_rx_i (1'b1),
        .m0_pkt_sel_i   (m0_sel), .m0_pkt_valid_i (m0_valid),
        .m1_pkt_sel_i   (4'd0),   .m1_pkt_valid_i (1'b0),
        .led_o (led)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        rst = 1'b1; m0_sel = 0; m0_valid = 0;
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;
    end

    localparam ST_IDLE = 3'd1;

    task drive(input [3:0] idx);
        begin
            @(posedge clk);
            m0_sel = idx; m0_valid = 1'b1;
            @(posedge clk);
            m0_valid = 1'b0;
            wait (dut.u_master0.state == ST_IDLE);
            $display("[%0t] txn%0d done rdata_mem=%0d", $time, idx, dut.u_master0.rdata_mem[idx]);
        end
    endtask

    always @(dut.u_slave0.state)
        $display("[%0t] u_slave0.state -> %0d addr_latch=%0d rdata_reg=%0d mem1=%0d", $time,
            dut.u_slave0.state, dut.u_slave0.addr_latch, dut.u_slave0.rdata_reg, dut.u_slave0.mem[1]);

    always @(posedge clk) begin
        if (dut.u_slave0.resume_o)
            $display("[%0t] u_slave0 resume_o pulse", $time);
        if (dut.u_slave0.ser_trigger)
            $display("[%0t] u_slave0 ser_trigger=1 rdata_reg=%0d addr_latch=%0d", $time, dut.u_slave0.rdata_reg, dut.u_slave0.addr_latch);
        if (dut.u_slave0.u_serializer.data_valid_out)
            $display("[%0t] u_slave0.u_serializer transmitting shift_reg=%0d serial_out=%b", $time, dut.u_slave0.u_serializer.shift_reg, dut.u_slave0.u_serializer.serial_out);
        if (dut.u_master0.u_deserializer.data_valid)
            $display("[%0t] master0 deserializer data_out=%0d", $time, dut.u_master0.u_deserializer.data_out);
        if (dut.mready_bus !== dut.u_slave0.mready_i)
            $display("[%0t] MISMATCH mready_bus=%b vs slave0.mready_i=%b", $time, dut.mready_bus, dut.u_slave0.mready_i);
    end

    initial begin
        wait (rst == 1'b0);
        #100;
        drive(5); // write 4 to split slave addr1
        #200;
        drive(6); // read it back
        #500;
        $display("mem[1] final = %0d", dut.u_slave0.mem[1]);
        $finish;
    end
    initial begin #100000; $display("TIMEOUT"); $finish; end
endmodule
