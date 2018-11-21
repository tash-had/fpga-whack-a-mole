module one_sec_timer(SW, HEX0, CLOCK_50);
    input [3:0] SW; // SW[0], SW[1] = Speed selection | SW[2] = paralell_load | SW[3] = clear_b
    output [6:0] HEX0;
    input CLOCK_50;

    wire [27:0] rd0_out;
    reg update_display;

    wire [3:0] hi0_out;


    // module countdown1sec(clock, clear_b, enable, par_load, p_load_in, out);
    countdown1sec cd0(
        .clock(CLOCK_50),
        .clear_b(SW[3]),
        .enable(1'b1),
        .par_load(SW[2]),
        .out(rd0_out)
    );

    always @(*)
	begin
        if (rd0_out == 0)
            update_display = 1'b1;
        else
            update_display = 1'b0;
	end 

    //module display_new_number(clock, Q, clear, enable);

    display_new_number dnn0(CLOCK_50, hi0_out, SW[3], update_display);
	hex_decoder hd0(hi0_out, HEX0); // should be called with number and hex display

endmodule

// RateDivider
module countdown1sec(clock, clear_b, enable, par_load, out);
    input clock; // wire
    input clear_b; // wire
    input enable, par_load; // wire

    output reg [26:0] out;

    always @(posedge clock) 
    begin
        if (clear_b == 1'b0) 
            out <= 0;
        else if (par_load == 1'b1 || out == 0)
            out <= 26'b10111110101111000010000000 - 1'b1;
        else if (enable == 1'b1)
            out <= out - 1'b1;
    end
endmodule

module display_new_number(clock, Q, clear, countdown_finished);
	input clock, countdown_finished, clear;
	output reg [3:0] Q;

	always @(posedge clock)
	begin
		if(clear == 1'b0)
			Q <= 0;
		else if(countdown_finished == 1'b1)
			// Q <= Q + 1'b1;
			/** SET Q TO A RANDOM NUMBER **/
	end
endmodule

module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule