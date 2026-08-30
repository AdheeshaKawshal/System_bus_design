module uart_tx #(
	parameter CLOCKS_PER_PULSE = 16,
              DATA_WIDTH  =  8
)
(
	input [DATA_WIDTH -1:0] data_in,
	input data_en,
	input clk,
	input rstn,
	output reg tx,
	output tx_busy
);

	localparam TX_IDLE  = 2'b00,    
	           TX_START = 2'b01, 
	           TX_DATA  = 2'b11,  
			   TX_END   = 2'b10;

	// State variable
	reg [1:0] state;

	// Data and control signals
	reg [DATA_WIDTH -1:0] data;
	reg [$clog2(DATA_WIDTH)-1:0] c_bits;
	reg [$clog2(CLOCKS_PER_PULSE)-1:0] c_clocks;
	
	// Sequential logic
	always @(posedge clk or negedge rstn) begin
		if (!rstn) begin
			c_clocks <= 0;
			c_bits <= 0;
			data <= 0;
			tx <= 1'b1;
			state <= TX_IDLE;
		end else begin 
			case (state)
				TX_IDLE: begin
					if (data_en) begin
						state <= TX_START;
						data <= data_in;
						c_bits <= 0;
						c_clocks <= 0;
					end else tx <= 1'b1;
				end
				TX_START: begin
					if (c_clocks == CLOCKS_PER_PULSE-1) begin
						state <= TX_DATA;
						c_clocks <= 0;
					end else begin
						tx <= 1'b0;
						c_clocks <= c_clocks + 1;
					end
				end
				TX_DATA: begin
					if (c_clocks == CLOCKS_PER_PULSE-1) begin
						c_clocks <= 0;
						if (c_bits == DATA_WIDTH-1) begin
							state <= TX_END;
						end else begin
							c_bits <= c_bits + 1;
							tx <= data[c_bits];
						end
					end else begin
						tx <= data[c_bits];
						c_clocks <= c_clocks + 1;
					end
				end
				TX_END: begin
					if (c_clocks == CLOCKS_PER_PULSE-1) begin
						state <= TX_IDLE;
						c_clocks <= 0;
					end else begin
						tx <= 1'b1;
						c_clocks <= c_clocks + 1;
					end
				end
				default: state <= TX_IDLE;
			endcase
		end
	end
	
	// Output to indicate busy state
	assign tx_busy = (state != TX_IDLE);
	
endmodule