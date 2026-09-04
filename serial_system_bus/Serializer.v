module Serializer (
	input  wire clk_in,
	input  wire rst_n,

	// Input data to be serialized
	input  wire [7:0] data_in,
	input  wire       data_valid,

	// Serialized output
	output wire       serial_out,
	output wire       data_valid_out,
	output wire       done
);

	reg [2:0] bit_counter;
	reg [7:0] shift_reg;
	reg       transmitting;

	assign data_valid_out = transmitting;
	assign done = !transmitting && data_valid;

	always @(posedge clk_in or negedge rst_n) begin
		if (!rst_n) begin
			bit_counter <= 3'b000;
			shift_reg <= 8'b00000000;
			transmitting <= 1'b0;
		end else begin
			if (data_valid && !transmitting) begin
				shift_reg <= data_in;
				bit_counter <= 3'b000;
				transmitting <= 1'b1;
			end else if (transmitting) begin
				if (bit_counter < 3'b111) begin
					bit_counter <= bit_counter + 1;
					shift_reg <= {shift_reg[6:0], 1'b0}; // Shift left
				end else begin
					transmitting <= 1'b0; // Transmission complete
				end
			end
		end
	end

	assign serial_out = transmitting ? shift_reg[7] : 1'b0;

endmodule