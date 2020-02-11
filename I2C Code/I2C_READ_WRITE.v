//---------------------------------------------------------------------
// Project		: BOSCH BMA280 sensor implementation
// Module Name	: I2C_READ_WRITE.v
// Author		: Nicholas DiPaolo
// Created		: 01/02/2019
// Company		: 
//----------------------------------------------------------------------
// Description : 
// This module implements a I2C master module  and it is an 8 bit I2C FSM. It has both read and write functionality.
//------------------------------------------------------------------------
// Changes: petren: changed signals names for better reading. Jan 19, 2019
////----------------------------------------------------------------------
//Notes: 
//BMA280 slave address is 0x18
//Bandwith setting is register 0x13 (undiltered is 2Khz) filtered is between 8-500Hz datasheet page 18
//range setting is register 0x0F  and for 2g  is 0.244g increments LSB setting is 0011 page 19 datasheet
/*---------------------------------------------------------------------------------------------------------------

i2c FSM
READ Operation:

	   |*****Sending Slave Address *****|  |*******Sending Reg Address********|				     |**Sending Slave Address w/ R/W bit**|   |******Reading Data from register******|
|Start| SA6|SA5|SA4|SA3|SA2|SA1|SA0|0|ACK|  RA7|RA6|RA5|RA4|RA3|RA2|RA1|RA0|ACK|  |STOP| |Start|  SA6|SA5|SA4|SA3|SA2|SA1|SA0|1(R/W)|ACK|  D7|D6|D5|D4|D3|D2|D1|D0|1(stop reading)| STOP|


WRITE Operation: 

	   |*****Sending Slave Address *****|  |*******Sending Reg Address********|	|**Writing Data to register**|
|Start| SA6|SA5|SA4|SA3|SA2|SA1|SA0|0|ACK|  RA7|RA6|RA5|RA4|RA3|RA2|RA1|RA0|ACK|  |W7|W6|W5|W4|W3|W2|W1|W0|ACK|  |STOP|

Refer to Pg 103-104 for more info
---------------------------------------------------------------------------------------------------------------*/

module i2c_master_read_write (
	input i2c_clk,  //clock //104KHz 
	input RSTn,  // Reset the module 
	input start_r_w, // 
	input r_w,     //read/write (1 = read)
	input [7:0] data_write, //Data that will be written into the selected register
	output reg [7:0]data_read,//use only low or  caps 
	output reg end_r_w,   // 
	output I2C_SCLK,//I2C CLOCK
 	inout  I2C_SDAT,//I2C DATA
	input [7:0] reg_addr
);

	parameter slave_addr = 00;//address of the device // petren: because datasheet says BMA220 slave address is 0001011x but because CSB pin is tied high
	
	
	localparam RESET =0; //Reset state
	localparam START =1; //send a start bit
	localparam SETUP_SLAVE =2; // sends the slave address with a write bit (bit 8)
	localparam SETUP_REG =3; // sends the register address with a dont care for a R/W bit. in the module it is sent to 1.
	localparam STOP_SETUP =4; // Sends Stop bit. this is only used if reading
	localparam START_READ =5; // sends start bit. this is only used if reading
	localparam SLAVE_READ = 6; // sends slave address with a read bit.
	localparam DATA_READ =7; // stores the 8 bits from the slave into read_reg_addr register.
	localparam DATA_WRITE =8; // writes 8 bits to SDA
	localparam ACK =9; //sets to high impedance allowing the slave to Acknowledge
	//localparam WAIT =10; //sets to high impedence allowing the slave to Acknowledge
	localparam STOP =11; // sends the final stop bit for reading and writing
	
reg en_clock_out; //when high and SCL is high, SCL is equal to the inverted i2c_clk
reg SDO; //controls the i2c data line
reg SCLK; // controls the i2c serial clock
 
reg [3:0] count;
reg [3:0] nextstate;// holds the state prior of the ACK state
reg [3:0] state; //current state

/*
********************************************************************************************************

*/
assign I2C_SCLK= SCLK | ( en_clock_out ? ~i2c_clk : 0 ); //petren if en_clock_out=1 then i2c_clk
/*                       <----------- temp   --------->
if(en_clock_out)
  temp = ~i2c_clk;
else
  temp = 0;
I2C_SCLK= SCLK | temp
 */
assign I2C_SDAT=SDO ? 1'bz:0 ; // needs to be high impedence instead of 1 to allow the slave to control the line

//if (SDO) 
//	I2C_SDAT= 1'bz;
//	else 
//I2C_SDAT= 0;



/*
*****************************************************************************************
			****************************I2C FSM****************************
*****************************************************************************************

*/
//Posedge i2c_clk is for writing to SDA
//negedge i2c_clk is for reading SDA

always @(negedge RSTn or posedge i2c_clk) begin
	if (!RSTn | !start_r_w) begin 
		state <= RESET; 
		end_r_w <= 0; 
		SCLK <= 1; 
		SDO <= 1; 
		count <= 1; 
		en_clock_out <= 0; 
		end
	//else if (start_r_w) begin
	else begin
	case(state)
		RESET: begin
			SCLK <= 1;
			SDO <= 1;
			en_clock_out <= 0;
			count <= 1;
			state <= START;
			end_r_w <= 0;
		end


		
		START: begin // 
			en_clock_out <= 0;
				if(count == 1) begin
					SDO <= 0;
					count <= count - 1;
				end
				else begin
					SCLK <= 0;
					count <= count - 1;
					count <= 6;
					state <= SETUP_SLAVE;
				end
			end
	
		SETUP_SLAVE: begin
				en_clock_out <= 1;
			if(count <= 6) begin
				SDO <= slave_addr[count];
				count <= count -1;
			end
			else begin 
				SDO <= 0;
				count <= 6;
				nextstate <= SETUP_REG;
				state <= ACK;
				
			
			end
		end
		
		SETUP_REG: begin
			
			en_clock_out <= 1;
				if(count <= 6) begin
					SDO <= reg_addr[count+1];
					count <= count - 1;
				end
				else begin 
					
					SDO <= reg_addr[0]; 
					if(r_w)begin
						count <= 2;
						nextstate <= STOP_SETUP;
						state <= ACK;	
					end
					else begin
						SDO <= reg_addr[0]; 
						count <= 6;
						nextstate <= DATA_WRITE;
						state <= ACK;
					
					end
				end
		end
			
			

/*
**************************************************************************************
*****************************ONLY WHEN READING****************************************
**************************************************************************************
*/		
		
		STOP_SETUP:	begin  //not sure if this will work may need a 2 bit count for this.
			en_clock_out <= 0;
			if(count <= 2) begin
				if (count == 2) begin
					SDO <= 0;
					SCLK <= 0;
					count <= count -1;
				end
			
				else if(count == 1) begin
					SCLK <= 1;
					count <= count -1;
				end	
				else begin
					SDO <= 1;
					count <= count -1;
				end
			end
			else begin
				count <= 1;
				state <= START_READ;
			end	
		end
		
		
		
		START_READ: begin
			en_clock_out <= 0;
			if(count <= 1) begin
				if(count == 1)begin
					SDO <= 0;
					count <= count -1;
				end
				else begin
					SCLK <= 0;
					count <= count -1;
				end
			end
			else begin
				count <= 6;
				state <= SLAVE_READ;
				
			end
		end
		
		SLAVE_READ: begin
				
			en_clock_out <= 1;
			if(count <= 6) begin
				SDO <= slave_addr[count];
				count <= count -1;
			end
			else begin 
				SDO <= 1;  //read bit
				nextstate <= DATA_READ;
				count <= 7;
				state <= ACK;
				
			end
		end
		
		
			DATA_READ: begin
			SDO <= 1;
			en_clock_out <= 1;
			if(count <= 7) begin
				count <= count -1;
			end
			else begin
				count <= 2;
				SDO <= 1;
				state <= STOP;
			end
		end

/*
**************************************************************************************
*****************************ONLY WHEN WRITING****************************************
**************************************************************************************
*/

		
		DATA_WRITE:	begin
			if(count <= 6) begin
				en_clock_out <= 1;
				SDO <= data_write[count+1];
				count <= count -1;
			end
			else begin
				
				SDO <= data_write[0];
				count <= 2;
				nextstate <= STOP;
				state <= ACK;
			end 
		end
//************************************************************************************		
//************************************************************************************	

	
		ACK: begin
			
			SDO <= 1;
			en_clock_out <= 1;
			state <= nextstate;
			
		end
		
		/*
		WAIT: begin	
			en_clock_out<=0;	
			SDO<=0;
			SCLK <= 0;
			state <= nextstate;

		end	
		*/
		
		
			/*	
**************************************************************************************
***********************Last state for both reading and writing************************
**************************************************************************************
*/					
		
		STOP: begin
			
			en_clock_out <= 0;
			if(count <= 2) begin
				if (count == 2) begin
					SDO <= 0;
					SCLK <= 0;
					count <= count -1;
				end
				else if(count == 1)begin
					SCLK <= 1;
					count <= count -1;
				end
				else begin
					SDO <= 1;
					count <= count -1;
				end
			end
						
			else begin
				end_r_w <= 1;
				state <= RESET;
						
			end
		end
		
	default: state <= state;
		
	endcase	

end


end


//Recording the data on the negedges

always @(negedge i2c_clk) begin
	case(state)
	
		DATA_READ: begin
						
			if(count <= 6) 
				data_read[count+1] <= I2C_SDAT;
						
			
			else if (count == 4'b1111) begin
				data_read[0] <= I2C_SDAT;
			
			end
		
		end	
	endcase
	
end

endmodule
	