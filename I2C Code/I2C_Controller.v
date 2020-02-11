//---------------------------------------------------------------------
// Project		: BOSCH BMA280 sensor implementation
// Module Name	: I2C_Controller.v
// Author		: Nicholas DiPaolo
// Created		: 01/02/2019
// Company		: 
//----------------------------------------------------------------------
// Description : 
// Uses I2C_READ_WRITE module to read from a BOsch BMA280 acceleration sensor and outputs// Z,Y,Z axis data in 8 bit format for further processing
//------------------------------------------------------------------------
// Changes: 
//petren: changed signals names for better reading. Jan 19, 2019
//DiPaolo: changed for BMA280 sensor. Feb 1 2019
//DiPaolo: Added temperature readings. Feb 19, 2019
////----------------------------------------------------------------------
//Note: 
//BMA280 slave address is a 7 bit value 0011000 (0x18)

module I2C_Controller 
(
	input CLK,
	input RSTn,
	output SCL,
	inout SDA, 
	output reg [13:0] X_DATA,
	output reg [13:0] Z_DATA,
	output reg [7:0] temp_DATA,
	output reg X_valid,
	output reg Z_valid
	);

	parameter 	filter = 15; //set BW filter
	parameter  	G_range = 3; // set g_level range to 2, 4, 8, or 16
	
	/* More infomation can be found on pg 18-19 of the BOSCH BMA280 datashet 
	Filtering:	(write to address 0x10 bits [4:0])				
 	(8)  5'b01000 - 7.81Hz 	(64ms)
	(9)  5'b01001 - 15.63Hz (32ms)
	(10) 5'b01010 - 31.25HZ (16ms)
	(11) 5'b01011 - 62.5Hz	(8ms)
	(12) 5'b01100 - 125Hz	(4ms)
	(13) 5'b01101 - 250Hz	(2ms)
	(14) 5'b01110 - 500Hz	(1ms)
	(15) 5'b01111 - 1000Hz (0.5ms) - default
	 
	
	G_Range: (write to address 0x0F bits [3:0])
	(3) 4'b0011 - 2g - default
	(5) 4'b0101 - 4g
	(8) 4'b1000 - 8g
	(12)4'b1100 - 16g
	
	*/
	
	
	
	wire end_r_w; //1 read/write cycle is completed
	wire [7:0] DATA; //read data is stored in this register
	wire start; //start I2C communication
	//assign start_r_w = Power_on & RSTn & EN_I2C;//petren
	
	// petren debug
	//assign start_I2C = start_r_w;

	localparam IDLE 					=	0; 
	localparam SET_FILTER 				= 	1;
	localparam SET_G_RANGE 				=	2;
	localparam READ_X_LSB				=	3; //reading x acceleration register 
	localparam READ_X_MSB 				=	4; //reading Y acceleration register
	localparam READ_Z_LSB 				=	5; //reading Z acceleration register
	localparam READ_Z_MSB 				= 	6;	
	localparam READ_TEMP 				 = 	7; //Reading temperature. Temperature is represented by a 8 bit 2's complement value.

	
	reg [7:0] reg_addr; //register address of the acclerometer 
	reg [7:0] data_write;
	reg r_w; //read or write bit (read =1, write =0)
	reg [2:0] state, nxt_state;
	//reg [1:0] count = 0; //counts to 3 before reading data again
	reg EN_I2C; //when set low, I2C is stopped
	assign start_r_w = RSTn & EN_I2C;
	
	always@(posedge CLK or negedge RSTn)begin
		if(!RSTn) begin
			state <= IDLE;
		end
		else begin
			state <= nxt_state;
		end
	end
	
	always@(*) begin
		case(state)
		
		
		IDLE:
			begin
				EN_I2C = 0;
				reg_addr = 0;
				r_w = 0;
				Z_valid = 1;
				nxt_state = SET_FILTER;
				
			end
		
			
		SET_FILTER: //writing to reg 0x10 to set the BW 
			begin
				Z_valid = 1;
				r_w = 0;
				EN_I2C = 1;
				reg_addr = 16; //reg 0x10
				data_write = filter;
				if(end_r_w)
					nxt_state = SET_G_RANGE;
						else 
							nxt_state = SET_FILTER;
			
			
			end
			
			
		SET_G_RANGE: //writing to reg 0x0F to set the G range
			begin
				Z_valid = 1;
				r_w = 0;
				EN_I2C = 1;
				reg_addr = 15; //reg 0x0F
				data_write = G_range;
				if(end_r_w)
					nxt_state = READ_X_LSB;
						else 
							nxt_state = SET_G_RANGE;
			end
			
		
			
		READ_X_LSB: //reading from reg 0x02 to acquire x g_level LSB [5:0]
			begin
				X_valid = 1;
				Z_valid = 1;
				r_w = 1;
				EN_I2C = 1;
				reg_addr = 2; //reg 0x02
				if(end_r_w)begin
					X_valid = 0;
					nxt_state = READ_X_MSB;
					X_DATA[5:0] <= DATA[7:2]; //6 LSB of the g-level is equal to the 6 MSB in register 2
				end
						else 
							nxt_state = READ_X_LSB;
			
			
			
			end
			
			
			
		READ_X_MSB:  //reading from reg 0x03 to acquire x g_level MSB [13:6]
			begin
				X_valid = 0;
				Z_valid = 1;
				r_w = 1;
				EN_I2C = 1;
				reg_addr = 3; //reg 0x03
				if(end_r_w)begin
					X_valid = 1;
					nxt_state = READ_Z_LSB;
					X_DATA[13:6] <= DATA;
				end
						else 
							nxt_state = READ_X_MSB;
			
			
			end
			
			
			
		READ_Z_LSB:  //reading from reg 0x06 to acquire Z g_level LSB [5:0]
			begin
				Z_valid = 1;
				r_w = 1;
				EN_I2C = 1;
				reg_addr = 6; //reg 0x06
				if(end_r_w)begin
					Z_valid = 0;//Z_acc is not valid since itonly has half the vaule
					nxt_state = READ_Z_MSB;
					Z_DATA[5:0] <= DATA[7:2]; //6 LSB of the g-level is equal to the 6 MSB in register 6
				end
						else 
							nxt_state = READ_Z_LSB;
			
			
			end
			 
			
			
		READ_Z_MSB: //reading from reg 0x07 to acquire Z g_level MSB [13:6]
			begin
				Z_valid = 0;
				r_w = 1;
				EN_I2C = 1;
				reg_addr = 7; //reg 0x07
				if(end_r_w)begin
					nxt_state = READ_TEMP;
					Z_DATA[13:6] <= DATA;
					Z_valid = 1; //Z_acc is a valid vaule
				end
					else 
					nxt_state = READ_Z_MSB;
			end
			
		READ_TEMP: ///reading from reg 0x08 to acquire the temperature of the chip
			begin
				Z_valid = 1;
				X_valid = 1;
				EN_I2C = 1;
				reg_addr = 8; //temperature data is held in reg 0x08
				if(end_r_w) begin
					temp_DATA = DATA; //temp_DATA is equal to the 8 bit read out from the i2c SDA 
					nxt_state = READ_X_LSB;
				end
						else 
							nxt_state = READ_TEMP;
			
			
			
			end
		
		default: nxt_state = IDLE;
			
			
		endcase
	
	end

	//I2C Write/Read FSM
i2c_master_read_write #( .slave_addr(24)) I2C_RW1  //Device address is 0x18 = d24
(
.i2c_clk(CLK), 
.RSTn(RSTn), 
.start_r_w(start_r_w),  
.r_w(r_w),
.data_read(DATA), 
.I2C_SCLK(SCL),
.I2C_SDAT(SDA),
.end_r_w(end_r_w),
.data_write(data_write),
.reg_addr(reg_addr)
);

endmodule

//Below is the version with all 3 axis enabled

//---------------------------------------------------------------------
// Project		: BOSCH BMA280 sensor implementation
// Module Name	: I2C_Controller.v
// Author		: Nicholas DiPaolo
// Created		: 01/02/2019
// Company		: 
//----------------------------------------------------------------------
// Description : 
// Uses I2C_READ_WRITE module  to read from a BOsch BMA220 acceleration sensor and outputs// Z,Y,Z axis data in 8 bit format for further processing
//------------------------------------------------------------------------
// Changes: 
//petren: changed signals names for better reading. Jan 19, 2019
//NickDiP: changed for BMA280 sensor. Feb 1 2019
////----------------------------------------------------------------------
//Note: 
//BMA220 slave address is 0011000x 8th bit(x) is either 0 for write or 1 for read




/*DELETE when using




module I2C_Controller(
	input CLK,
	input RSTn,
	output SCL,
	inout SDA,
	output reg [13:0] X_DATA,
	output reg [13:0] Z_DATA,
	output reg [13:0] Y_DATA,
	output reg X_valid
	output reg Y_valid
	output reg Z_valid
	);

	parameter 	filter = 15; //set BW filter
	parameter  	G_range = 3; // set g_level range to 2, 4, 8, or 16
	/*
	Filtering:	(write to address 0x10 bits [4:0])				
 	(8)  5'b01000 - 7.81Hz 	(64ms)
	(9)  5'b01001 - 15.63Hz (32ms)
	(10) 5'b01010 - 31.25HZ (16ms)
	(11) 5'b01011 - 32.5Hz	(8ms)
	(12) 5'b01100 - 125Hz	(4ms)
	(13) 5'b01101 - 250Hz	(2ms)
	(14) 5'b01110 - 500Hz	(1ms)
	(15) 5'b01111 - unfiltered (0.5ms)
	
	G_Range: (write to address 0x0F bits [3:0])
	(3) 4'b0011 - 2g
	(5) 4'b0101 - 4g
	(8) 4'b1000 - 8g
	(12)4'b1100 - 16g
	
	*/
	
	
	
	/*DELETE when using
	
	
	
	
	wire end_r_w; //1 read cycle is completed
	wire [7:0] DATA_read; //Data from the SDA line when reading
	wire start; //start I2C communication
	//assign start_r_w = Power_on & RSTn & EN_I2C;//petren
	
	// petren debug
	//assign start_I2C = start_r_w;

	localparam IDLE =			0; 
	localparam SET_FILTER = 	1;
	localparam SET_G_RANGE =	2;
	localparam READ_X_LSB =		3; //reading X LSB register 0x02 
	localparam READ_X_MSB =		4; //reading X MSB register 0x03
	localparam READ_Y_LSB =		5; //reading Y LSB register 0x04 
	localparam READ_Y_MSB =		6; //reading Y MSB register 0x05
	localparam READ_Z_LSB =		7; //reading Z LSB register 0x06
	localparam READ_Z_MSB = 	8; //reading Z SB register 0x07
	
	
	reg [7:0] reg_addr; //register address of the acclerometer 
	reg [7:0] data_write;
	reg r_w; //read or write bit (read =1, write =0)
	reg [3:0] state, nxt_state;
	//reg [1:0] count = 0; //counts to 3 before reading data again
	reg EN_I2C; //when set low, I2C is stopped
	assign start_r_w = RSTn & EN_I2C;
	
	always@(posedge CLK or negedge RSTn)begin
		if(!RSTn) begin
			state <= IDLE;
		end
		else begin
			state <= nxt_state;
		end
	end
	
	always@(*) begin
		case(state)
		
		
		IDLE:
			begin
				EN_I2C = 0;
				reg_addr = 0;
				r_w = 0;
				Z_valid = 1;
				nxt_state = SET_FILTER; 
				
			end
			
			
		SET_FILTER:
			begin
				X_valid = 0;
				Y_valid = 0;
				Z_valid = 0;
				r_w = 0;
				EN_I2C = 1;
				reg_addr = 16; //reg 0x10
				data_write = filter;
				if(end_r_w)
					nxt_state = SET_G_RANGE;
						else 
							nxt_state = SET_FILTER;
			
			
			end
			
			
		SET_G_RANGE:
			begin
				X_valid = 0; 
				Y_valid = 0;
				Z_valid = 0;
				r_w = 0;
				EN_I2C = 1;
				reg_addr = 15; //reg 0x0F
				data_write = G_range;
				if(end_r_w)
					nxt_state = READ_X_LSB;
						else 
							nxt_state = SET_G_RANGE;
			end
			
			
			
		READ_X_LSB:
			begin
				X_valid = 1;
				Y_valid = 1;
				Z_valid = 1;
				r_w = 1;
				EN_I2C = 1;
				reg_addr = 2; //reg 0x02
				if(end_r_w)begin
					X_valid = 0;
					nxt_state = READ_X_MSB;
					X_DATA[5:0] = DATA_read[7:2]; //6 MSB of reg 0x02 = 6 LSB of X_ACC
				end
						else 
							nxt_state = READ_X_LSB;
			
			
			
			end
			
			
			
		READ_X_MSB:
			begin
				X_valid = 0;
				Y_valid = 1;
				Z_valid = 1;
				r_w = 1;
				EN_I2C = 1;
				reg_addr = 3; //reg 0x03
				if(end_r_w)begin
					X_valid = 1;
					nxt_state = READ_Y_LSB;
					X_DATA[13:6] = DATA_read; //reg 0x03 = 8 MSB of X_ACC
				end
						else 
							nxt_state = READ_X_MSB;
			
			
			end
			
		READ_Y_LSB:
			begin
				X_valid = 1;
				Y_valid = 1;
				Z_valid = 1;
				r_w = 1;
				EN_I2C = 1;
				reg_addr = 4; //reg 0x04
				if(end_r_w)begin
					Y_valid = 0;
					nxt_state = READ_Y_MSB;
					Y_DATA[5:0] = DATA_read[7:2]; //6 MSB of reg 0x04 = 6 LSB of Y_ACC
				end
						else 
							nxt_state = READ_Y_LSB;
			
			
			
			end
			
			
			
		READ_Y_MSB:
			begin
				X_valid = 1;
				Y_valid = 0;
				Z_valid = 1;
				r_w = 1;
				EN_I2C = 1;
				reg_addr = 5; //reg 0x05
				if(end_r_w)begin
					Y_valid = 1;
					nxt_state = READ_Z_LSB;
					Y_DATA[13:6] = DATA_read; //reg 0x05 = 8 MSB of Y_ACC
				end
						else 
							nxt_state = READ_X_MSB;
			
			
			end
			
			
			
		READ_Z_LSB:
			begin
				X_valid = 1;
				Y_valid = 1;
				Z_valid = 1;;
				r_w = 1;
				EN_I2C = 1;
				reg_addr = 6; //reg 0x06
				if(end_r_w)begin
					Z_valid = 0;//Z_acc is not valid since itonly has half the vaule
					nxt_state = READ_Z_MSB;
					Z_DATA[5:0] = DATA_read[7:2]; //6 MSB of reg 0x06 = 6 LSB of Z_ACC
				end
						else 
							nxt_state = READ_Z_LSB;
			
			
			end
			 
			
			
		READ_Z_MSB:
			begin
				Z_valid = 0;
				r_w = 1;
				EN_I2C = 1;
				reg_addr = 7; //reg 0x07
				if(end_r_w)begin
					nxt_state = READ_X_LSB;
					Z_DATA[13:6] = DATA_read; //reg 0x07 = 8 MSB of Z_ACC
					Z_valid = 1; //Z_acc is a valid vaule
				end
				else 
				nxt_state = READ_Z_MSB;
			
			
			end
			
			
			
		
		
		endcase
	
	end

	//I2C Write/Read FSM
i2c_master_read_write #( .slave_addr(24)) I2C_RW1  //Device address is 0x18 = d24
(
.i2c_clk(CLK), 
.RSTn(RSTn), 
.start_r_w(start_r_w),  
.r_w(r_w),
.data_read(DATA_read), 
.I2C_SCLK(SCL),
.I2C_SDAT(SDA),
.end_r_w(end_r_w),
.data_write(data_write),
.reg_addr(reg_addr)
);

endmodule


DELETE when using*/