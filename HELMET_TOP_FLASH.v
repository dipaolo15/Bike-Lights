//---------------------------------------------------------------------
// Project		: UART TEST
// Module Name	: UART_BT_TOP.v
// Author		: Nick Petrescu www.ohmics.com
// Created		: 3/03/2018
// Company		: Ohmics
//----------------------------------------------------------------------
// Description : 
/*
This module is performing the following functions

- Brake light (dual left and right)

*/
//------------------------------------------------------------------------
// Modification history : 
// Nick Petrescu (petren)
//Nick DiPaolo: Added temperature warning (middle top lights come on when the temperature reaches a certain point) - Feb 2, 2018
////----------------------------------------------------------------------
 
module HELMET_TOP (
						//input 	wire   SYS_CLK,	//The system external clock is 10Mhz when crystal is populated or 7MHz when internal osc is instantiated
						//input 	wire   		RST,
						input  	wire[1:0]	sensitivity, 
					
						input  	wire   		POWER,//
						
                       	output 	wire[2:0]   YEL_RIGHT,//Right light
						output 	wire[2:0]   YEL_LEFT,//Left lights
						inout 	wire 		SDA,
						output 	wire		SCL,
						output 	wire   		BRAKE_PWM_CENTER, //middle lights
						output 	wire 		temp_warning // warns you when the temperature is too high
						
        );




/*******************************************
*************Metastability	BEGIN***********
 *******************************************
*/
		
	
assign BRAKE_PWM_CENTER = MIDDLE_3;assign YEL_LEFT = LEFT_3;
assign YEL_RIGHT = RIGHT_3;

reg[2:0] 	LEFT_2, RIGHT_2, LEFT_3, RIGHT_3;
reg 		MIDDLE_2, MIDDLE_3;
wire [2:0] 	LEFT, RIGHT;
wire 		MIDDLE;

always@(posedge sys_clk)begin
	
	LEFT_2 <= LEFT;
	LEFT_3 <= LEFT_2;
	
	RIGHT_2<=RIGHT;
	RIGHT_3 <= RIGHT_2;
	
	MIDDLE_2<=MIDDLE;
	MIDDLE_3 <= MIDDLE_2;
	
	
	end
	 
/*******************************************
*************Metastability	END***********
 *******************************************
*/

/*******************************************
***I2C communication w/ accelerometer BEGIN****  
***********************************************/
wire clk_1Hz;
wire i2c_clk;
dclock #(.divider(20))  clkdiv_i2c  // for internal 2.08MHz clock
//dclock #(.divider(70))  clkdiv_i2c  // for internal 7MHz clock
( 
  	.reset           (RST),
	//.reset	     (globalreset),	
	.clk         	 (sys_clk),
	.clko         	 (i2c_clk)
	);
	
wire sample_clk;
dclock #(.divider(2080000))  clkdiv_Sec  //This is the sample rate for sampling the acceleration
( 
  	.reset           (RST),
	//.reset	     (globalreset),	
	.clk         	 (sys_clk),
	.clko         	 (clk_1Hz)
	);

	
wire light_clk;
dclock #(.divider(260000))  light_clock  //This is the sample rate for sampling the acceleration
( 
  	.reset           (RST),
	//.reset	     (globalreset),	
	.clk         	 (sys_clk),
	.clko         	 (light_clk)
	);
	
wire [1:0] brake_intensity;


reg brake_center_out;
reg [2:0] yel_left,yel_right;
//assign YEL_RIGHT = yel_right;
//assign YEL_LEFT = yel_left;
//assign BRAKE_PWM_CENTER = brake_center_out;

wire acc_valid;
wire tipover;


/*
sensitivity settings:

2'b00 = Low
2'b01 = Medium
2'b01 = High
2'b11 = disable deceleration functions


*/

deceleration_algorithm #(.TEMP_WARNING_SELECT(27)) test(
	.sys_clk(sys_clk), //System Clock
	.clk_i2c(i2c_clk), //100KHz - 400KHz clock for the i2c operation
	//.clk_1Hz(clk_1Hz),
	.rst_n(power_on), //indicates power on
	.test(0), //indicates test mode on/off
	.sen_setting(sensitivity), //sets deceleration sensitivity
	.SCL(SCL), //i2c serial clock
	.SDA(SDA), // i2c Data line
	.acc_light_out(brake_intensity), ////brake intensity: Low, High, or Off
	.acc_valid(acc_valid),//indicates you have an valid deceleration
	.tipover(tipover),//indicates you have are tiped over
	.temp_warning(temp_warning) //indicates the temperature of the accelerometer is higher then selcted
);

 
	
light_controller Lights_1( //Light decoder
	.sys_clk(sys_clk), //System Clock
	.clk_1Hz(clk_1Hz), //1Hz clock used for the tilt pulsing
	.PWR(power_on), //indicates the power is on.
	.acc_valid(acc_valid), //indicates you have an valid deceleration
	.tilt_valid(tipover), //indicates you have are tiped over
	//.light_clk(light_clk),
	.brake_intensity(brake_intensity), //the brake intensity: Low, High, or Off
	.yel_right(RIGHT), //Light out Right
	.yel_middle(MIDDLE), //Light out Middle
	.yel_left(LEFT) //Light out Left
);





/*******************************************
***I2C communication w/ accelerometer END****  
***********************************************/


		wire RST;
		assign RST = 1; //RST is always disabled, dont have an RST button/switch
		
//Internal Oscillator instantiation
//assign sys_clk = SYS_CLK; //use this for external 10MHz oscillator

wire sys_clk,mem_clk;
//defparam OSCH_inst.NOM_FREQ = "7.00";// This is the default frequency
defparam OSCH_inst.NOM_FREQ = "2.08"; //MHz
OSCH OSCH_inst
( 
.STDBY(1'b0), // 0=Enabled, 1=Disabled; also Disabled with Bandgap=OFF
.OSC(sys_clk), // Disable this for external oscillator
.SEDSTDBY()// this signal is not required if not using SED
); 

GSR     GSR_INST (.GSR (RST)); //inferring global reset



wire power_on;

power_switch  power_switch1 // Press any of the button for 3 seconds and power will toggle
( 
.clk(sys_clk), // 0=Enabled, 1=Disabled; also Disabled with Bandgap=OFF
.rst(RST), // Disable this for external oscillator
.tsl_button_in(!POWER),
.tsr_button_in(),
.status(power_on)
);

endmodule 



