/*****************************
	ONE SEC TIMER
******************************/

module fpga_whack_a_mole(CLOCK_50, SW, KEY, LEDR, HEX0, HEX1, HEX2, HEX3);
    input [3:0] SW; // SW[0], SW[1] = Speed selection | SW[2] = paralell_load | SW[3] = clear_b
    input CLOCK_50;
	 
	 input [1:0] KEY;
    output [9:0] LEDR;
    output [6:0] HEX0, HEX1, HEX2, HEX3;

    wire [27:0] rd0_out;
    reg update_display;

    wire [3:0] hi0_out;


    // module countdown1sec(clock, clear_b, enable, par_load, p_load_in, out);
    countdown1sec cd0(
        .clock(CLOCK_50),
        .clear_b(KEY[0]),
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

   //display_new_number dnn0(CLOCK_50, hi0_out, SW[3], update_display);
	
	//hex_decoder hd0(hi0_out, HEX0); // should be called with number and hex display
	random_display rd0(update_display, KEY, LEDR, HEX0, HEX1, HEX2, HEX3);

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
            out <= 25'b1011111010111100000111111 - 1'b1; //out <= 26'b10111110101111000010000000 - 1'b1;
        else if (enable == 1'b1)
            out <= out - 1'b1;
    end
endmodule

module random_display(clk, KEY, LEDR, HEX0, HEX1, HEX2, HEX3);
    input clk;
    input [1:0] KEY;
    output [9:0] LEDR;
    output reg [6:0] HEX0, HEX1, HEX2, HEX3;
	
    wire [6:0] out;
    wire [3:0] digit;
    wire [1:0] display;
    
    HEX_choose_lfsr d1(
		.clk(clk),
		.reset_n(KEY[0]),
		.curr_display(display)
	);

    display_num_lfsr d2(
		.clk(clk),
		.reset_n(KEY[0]),
		.curr(digit)
	);
	
	assign LEDR[3:0] = digit;

	hex_decoder H0(digit, out);
	
    always @(*)
		begin
        case (display)
            2'b00: begin
				HEX0 = out;
				HEX1 = 7'b111_1111;
				HEX2 = 7'b111_1111;
				HEX3 = 7'b111_1111;
				end
            2'b01: begin
				HEX1 = out;
				HEX0 = 7'b111_1111;
				HEX2 = 7'b111_1111;
				HEX3 = 7'b111_1111;
				end
            2'b10: begin
				HEX2 = out;
				HEX0 = 7'b111_1111;
				HEX1 = 7'b111_1111;
				HEX3 = 7'b111_1111;
				end
				2'b11: begin
				HEX3 = out;
				HEX0 = 7'b111_1111;
				HEX2 = 7'b111_1111;
				HEX1 = 7'b111_1111;
            end
				default: begin
				HEX0 = out;
				HEX1 = 7'b111_1111;
				HEX2 = 7'b111_1111;
				HEX3 = 7'b111_1111;
				end
        endcase
		end        
endmodule

module HEX_choose_lfsr(input clk, input reset_n, output reg [1:0] curr_display);

reg [1:0] next_display;

always @* begin
  next_display[1] = curr_display[1]^next_display[0];
  next_display[0] = curr_display[0]^next_display[1];
  if (curr_display == 2'b00)
	next_display[1:0] = 2'b10;
end

always @(posedge clk or negedge reset_n)
  if(!reset_n)
    curr_display <= 2'b00;
  else
    curr_display <= next_display;

endmodule

module display_num_lfsr(
  input clk,
  input reset_n,

  output reg [3:0] curr
);

reg [3:0] next;

always @* begin
  next[3] = curr[3]^next[0];
  next[2] = curr[2]^next[1];
  next[1] = curr[1]^next[3];
  next[0] = curr[0]^next[2];
  if (curr == 4'h0)
	next[3:0] = 2'b0010;
end

always @(posedge clk or negedge reset_n)
  if(!reset_n)
    curr <= 4'h0;
  else
    curr <= next;

endmodule

/*module display_new_number(clock, Q, clear, countdown_finished);
	input clock, countdown_finished, clear;
	output reg [3:0] Q;

	always @(posedge clock)
	begin
		if(clear == 1'b0)
			Q <= 0;
		else if(countdown_finished == 1'b1)
			Q <= 
	end
endmodule*/

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