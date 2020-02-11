//---------------------------------------------------------------------
// Project		: BOSCH BMA280 sensor implementation
// Module Name	: algorithm.v
// Author		: Nicholas DiPaolo, Nick Petrescu
// Created		: 01/02/2019
// Company		: 
//----------------------------------------------------------------------
// Description : 
// This Module interprets the G Forces data in the X, Y, Z, as well as the tilt from an accelerometer (BMA280).
// Using the G forces, be able to determine the deceleration of a Car/Bik
//------------------------------------------------------------------------
// Changes: 
//petren: changed signals names for better reading. Jan 19, 2019
//DiPaolo: changed for BMA280 sensor. Feb 1 2019
//DiPaolo: Added temperature readings. Feb 19, 2019
////----------------------------------------------------------------------
// Note about data sampling:
// Default acc data filtering/sampling is 2Khz or 0.5ms with BMA280. It can be tuned further by changing the "filter" parameter
// actual data rate  when default filtering is used after FSM is 2ms
// The Arduino data posting is at 100ms rate
// The proposed data filtering to filter bumps and such is that anything under 250ms and everything over 1s must be filtered out.

module deceleration_algorithm #(
	parameter TEMP_WARNING_SELECT = 00)
	/*
	TEMP_WARNING_SELECT is a 8 bit 2-complement number. 
	Selecting TEMP_WARNING_SELECT value:
	
	0 = 23 C
	1 = 23.5 C
	.
	.
	.
	64 = 55 C
	.
	.
	.
	127 = 86.5 C Max
	
	256 = 22.5 C
	.
	.
	.
	192 = -9 C
	.
	.
	.
	128 = -41 C Min
	
	Refer to Conversions.exc for full list
	
	
	
	
	*/
	(
	input sys_clk,
	input clk_i2c, //Clock that I2C is running at (100KHz)
	input rst_n, 
	input test,
	input [1:0] sen_setting,
	output SCL, // serial clock for the I2C Bus
	inout SDA, //I2C data bus
	output reg [1:0] acc_light_out, //  00 = no lights, 01 = first stage light turns on, 11 =  first and second stage turn on.
	output tipover,//indicating turn signal using tilt
	output acc_valid,
	output temp_warning
	 
	);
//------------------------------------------------------	
//WIRES/REG	
//------------------------------------------------------	


//reg [1:0] light_output;
reg [1:0] decel_intensity;
wire [13:0] G_level_Z, G_level_X; //Current Z G level
wire Z_valid, X_valid;
//reg trigger;
wire [1:0] decel_sen_filtered;
reg tipover_unfiltered;

assign acc_valid = (acc_light_out) ? 1:0; // petren acc_valid is 1 if light_output is not 00
assign temp_warning = (temp_DATA >= TEMP_WARNING_SELECT && temp_DATA <= 127)?1:0; // temp_warning = 1 when temperature is greater then TEMP_WARNING_SELECT. Note: if temp_DATA is > 127 then the temperature is less then 23 degrees Celsius
//assign acc_light_out = light_output;

//------------------------------------------------------	
//I2C CONTROLLER
//------------------------------------------------------	

//Takes around 2ms for the g_levels to be refreshed
I2C_Controller I2C_contr(//Module controls the I2C, reads X data, Y data, Z data, and Tilt data.
	.CLK(clk_i2c),
	.RSTn(rst_n),
	.SDA(SDA),
	.SCL(SCL),
	.Z_DATA(G_level_Z), //G force in the Z
	.Z_valid(Z_valid),
	.X_DATA(G_level_X), //G force in the X
	.X_valid(X_valid),
	.temp_DATA(temp_DATA)
	
);

//------------------------------------------------------	
//DECELERATION FILTERING FOR BUMPS AND TURNS - TWO STAGES
//------------------------------------------------------

acc_filter_dual #(.CLK_PERIOD(480),.DEBOUNCE_HI_PERIOD(250),.DEBOUNCE_LO_PERIOD(0),.MIN_HI_PERIOD(750), .CUT_OFF_AFTER_HI_PERIOD(1), .RESTART_HI_PERIOD(0))  GLEVEL_filter_dual_1 
//1480 for 2.08Mhz sys clock
//in ms  petren  Any pulse high shorter than DEBOUNCE_PERIOD will be filtered 
// MIN_HI_PERIOD(1000)  = hysteresis time the amount of time (in ms) the output will stay high after the short initial posedge of the input pulse.
(
.rst_n(rst_n),
.clk(sys_clk),
.a_pulse_in(decel_intensity[0]),
.sigout(decel_sen_filtered[0]) //filtered input to the controller. Adjust filtered pulse width within the filter module
);

acc_filter_dual #(.CLK_PERIOD(480),.DEBOUNCE_HI_PERIOD(250),.DEBOUNCE_LO_PERIOD(0), .MIN_HI_PERIOD(750), .CUT_OFF_AFTER_HI_PERIOD(1), .RESTART_HI_PERIOD(0))  GLEVEL_filter_dual_2 //in ms  petren  Any pulse high shorter than DEBOUNCE_PERIOD will be filtered
//480 for 2.08Mhz
(
.rst_n(rst_n),
.clk(sys_clk),
.a_pulse_in(decel_intensity[1]),
.sigout(decel_sen_filtered[1]) //filtered input to the controller. Adjust filtered pulse width within the filter module
);

//------------------------------------------------------	
//TIPOVER
//------------------------------------------------------

acc_filter_dual #(.CLK_PERIOD(480),.DEBOUNCE_HI_PERIOD(100),.DEBOUNCE_LO_PERIOD(0))  pwm_filter_dual_1 //in ms  petren  Any pulse high shorter than DEBOUNCE_PERIOD will be filtered
//140 FOR 7MHz 480 for 2.08Mhz
(
.rst_n(rst_n),
.clk(sys_clk),
.a_pulse_in(tipover_unfiltered),
.sigout(tipover) //filtered input to the controller. Adjust filtered pulse width within the filter module
);

always@(*)begin
			if (X_valid) begin 	
				if(G_level_X >= 3300 && G_level_X < 13084) begin //  LOW3 < G < MAX1G then turn on 4 WAYS
						tipover_unfiltered = 1;
				end
				else begin
						tipover_unfiltered = 0;
				end
			end
end

//------------------------------------------------------	
//TEST OR NORMAL OPERATION SELECTION
//------------------------------------------------------

//note that by setting DEBOUNCE_HI_PERIOD =0 this is equivalent to eliminating the filter essentially making the next process equivalent.

//assign acc_light_out = decel_sen_filtered;

always@(*)begin 
	case(test)
		0: acc_light_out = decel_sen_filtered;
		1: acc_light_out = decel_intensity;
		default: acc_light_out = decel_sen_filtered;
	endcase
end	

/*
always@(*)begin 
	if(test == 2'b10)
			acc_light_out = decel_intensity; //THIS IS TEST ONLY
			else begin
			acc_light_out = decel_sen_filtered; // Output is going thru the filter
			end
						
end
*/
//------------------------------------------------------	
//SENSITIVITY SELECTION AND DECELERATION LEVELS
//------------------------------------------------------

localparam MAX1G = 4000; // 1G
localparam LOW1 = 2050; // 0.5G
localparam LOW2 = 1845; // 0.45G
localparam LOW3 = 1640; //0.4G
localparam MED1 = 1435; //0.35G
localparam MED2 = 1230; //0.3G
localparam MED3 = 1025; //0.25G
localparam HIGH1 = 820; //0.2G
localparam HIGH2 = 615; //0.155G
localparam HIGH3 = 410; //0.1G

//<<<<HIGHEST SENSITIVITY------------------------------LOWEST SENSITIVITY>>>>>>>>>
//|HIGH3	|HIGH2	|HIGH1	|MED3	|MED2	|MED1	|LOW3	|LOW2	|LOW1	|MAX1G
//							|-----LOW_S1--------------------|
//															|-----LOW_S2----|
//					|-----MED_S1------------|	
//								 		    |-----MED_S1--------------------|
//			|--HI_S1--------|
//    						|--------------------HI_S2----------------------|



always@(posedge sys_clk or negedge rst_n)begin
	
	if (!rst_n) begin
				decel_intensity <= 2'b00;
			end
				else begin
					if (Z_valid) begin 	// FSM must read all G_Level_Z 14 bits in order for data to be valid
					//decel_intensity = 2'b00;
					case(sen_setting) 
											
						2'b00: //Low sen_setting
						begin
						if(G_level_Z >= MED3 && G_level_Z < LOW2) 
								begin 
								decel_intensity <= 2'b01;
								end
							else if(G_level_Z >= LOW2 && G_level_Z < MAX1G)
								begin
								decel_intensity <= 2'b11;				
								end
						else begin
								decel_intensity <= 2'b00;					
							 end
						end
										
						2'b01:
						begin//Medium sen_setting
							if(G_level_Z >= HIGH1 && G_level_Z < MED1)begin //HIGH3 < G < MED3 then turn on first LED
								decel_intensity <= 2'b01;
							end
							else if(G_level_Z >= MED1 && G_level_Z < MAX1G)begin
								decel_intensity <= 2'b11;		//  LOW1 < G < MAX1G then turn on both LEDs		
							end
							else begin
								decel_intensity <= 2'b00;					
							end				
						end
										
						2'b10:
						begin//High sen_setting 
							if(G_level_Z >= HIGH2 && G_level_Z < MED3)begin 	//HIGH1 < G < MED1 then turn on first LED
								decel_intensity <= 2'b01;
							end
							else if(G_level_Z >= MED3 && G_level_Z < MAX1G)begin
								decel_intensity <= 2'b11;			//  MED2 < G < MAX1G then turn on both LEDs		
							end
							else begin
								decel_intensity <= 2'b00;					
							end			
						end
						2'b11: begin
								decel_intensity <= 0;	//accelerometer disabled
						end
						
						default: decel_intensity <= 0;
					endcase
				end
				end
	end										

endmodule

