`timescale 1ns / 1ps
module tb_single;
    reg clk, rst;
    integer errors;

    serial_bus_top #(.ADDR_W(15), .DATA_W(8), .RW(1), .NUM_SLAVES(3)) dut (.clk(clk), .rst(rst));

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        rst = 1'b0;
        repeat (5) @(posedge clk);
        @(negedge clk);
        rst = 1'b1;
    end

    initial begin
        force dut.u_master1.req_o = 1'b0;
    end

    task check_byte(input [127:0] name, input [7:0] got, input [7:0] expected);
        begin
            if (got !== expected) begin
                $display("FAIL: %0s got=%h expected=%h", name, got, expected);
                errors = errors + 1;
            end else begin
                $display("PASS: %0s == %h", name, got);
            end
        end
    endtask

    initial begin
        errors = 0;
        wait (rst == 1'b1);
        #200000;
        $display("---- Master 0 only ----");
        check_byte("M0 tx1", dut.u_master0.rdata_mem[1], 8'h11);
        check_byte("M0 tx3", dut.u_master0.rdata_mem[3], 8'h22);
        check_byte("M0 tx7", dut.u_master0.rdata_mem[7], 8'h44);
        if (errors == 0) $display("ALL PASS");
        else $display("%0d FAILED", errors);
        $finish;
    end
    initial begin #500000; $display("TIMEOUT"); $finish; end
endmodule
