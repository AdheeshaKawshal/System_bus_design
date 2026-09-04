module deserializer(
	input  wire clk_in,
	input  wire rst_n,

	input  wire serial_in,
	input  wire data_valid_in,

	output wire [7:0] data_out,
	output wire       data_valid
);
	reg [2:0] bit_counter;
	reg [7:0] shift_reg;
	reg       receiving;

	assign data_valid = !receiving && (bit_counter == 3'b111);

	always @(posedge clk_in or negedge rst_n) begin
		if (!rst_n) begin
			bit_counter <= 3'b000;
			shift_reg <= 8'b00000000;
			receiving <= 1'b0;
		end else begin
			if (data_valid_in && !receiving) begin
				receiving <= 1'b1;
				bit_counter <= 3'b000;
			end else if (receiving) begin
				if (bit_counter < 3'b111) begin
					shift_reg <= {shift_reg[6:0], serial_in}; // Shift in new bit
					bit_counter <= bit_counter + 1;
				end else begin
					receiving <= 1'b0; // Reception complete
				end
			end
		end
	end

	assign data_out = shift_reg;

endmodule