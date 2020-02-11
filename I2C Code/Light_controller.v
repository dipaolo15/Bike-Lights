module light_controller(
	input sys_clk,
    input PWR,
	input [1:0] brake_intensity, 
	input acc_valid,
	input tilt_valid,
	input clk_1Hz,
		
	output reg yel_middle,
	output reg[2:0] yel_left,
	output reg[2:0] yel_right
	
);

	dclock #(.divider(208))  clkdiv  // for internal 2.08MHz clock 
( 
  	.reset           (1),
	//.reset	     (globalreset),	
	.clk         	 (sys_clk),
	.clko         	 (clk_pwm)
	);
	
pwm #(.CTR_LEN(6)) pwm_2 // brake and turn brigthness low
(
.rst(1),
.clk(clk_pwm),
.pulse_width(6'd000001),//change this value to adjust brightness
.pwm(pwm_low)
);

	
	always@(*) begin
	
		if(PWR) begin
	
			if(tilt_valid) begin
				yel_left = {clk_1Hz, clk_1Hz, clk_1Hz};
				yel_right = {clk_1Hz, clk_1Hz, clk_1Hz};
				yel_middle = 0;
			end
			else begin
				case(brake_intensity)
				
					00:begin  // no braking, middle light is on to indicate powers on
						yel_left = 3'b000;
						yel_right = 3'b000;
						yel_middle = pwm_low; //indicates the power is on
					
					end
					
					2'b01: begin
						yel_left = 3'b001;
						yel_right = 3'b001;
						yel_middle = 0;
					 end
					
					
					2'b11:begin
						yel_left = 3'b111;
						yel_right = 3'b111;
						yel_middle = 0;
					end
					
					
					
					default: begin
						yel_left = 3'b000;
						yel_right = 3'b000;
						yel_middle = 0;
					
					end
				endcase
			end
			
		end
		else begin
				yel_left = 3'b000;
				yel_right = 3'b000;
				yel_middle = 0;
		end
	end
		
	
	
	
endmodule