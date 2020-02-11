//---------------------------------------------------------------------
// Project		: AdMore Simple Controller
// Module Name	: PWM.v
// Author		: Nick Petrescu www.ohmics.com
// Created		: 05/01/2016
// Company		: Ohmics
//----------------------------------------------------------------------
// Description : Generic PWM module

/*Theory of PWM operation:
  
- Frequency of the resulting PWM signal = f_sysclk/2^n where n is the number of bits used for the pulse width control.
- in this case sysclk=10MHz and we are using 3 bits so the frequency of the PWM clock will be 10MHz/8= 1.25MHz
- to obtain a 50% duty cycle set pulse_width to 4 or 3'b100. For 1/8 duty cycle set pulsewidth to dec 1 or 3'b001, etc.
  
  Input= system clock and pulse width control
  Output= PWM_signal
  pulse_width control is a 3 bit vector= 8 decimal so the duty cycle can be adjusted in 8 increments
  
Example operation for pwm_width=3 and dty cycle 25%: counter counts to 2 for PWM_out to be 1 then zero for the rest of the count.  
  
SPI_CLK-----````|__|```|___|```|___|```|___|```|___|```|___|```|___|```|___|```|___|```|___|```|___|```|___|```|___|```|___|```|___|```|___|  
Counter         000|    001|    010|    011|    100|    101|    110|    111|    000|    001|    010|    011|    100|    101|    110|    111| 
PWM_OUT------------````````|_______________________________________________________|```````````````|________________________________________

*/
module pwm #(parameter CTR_LEN = 3)//CTR_LEN is the PWM pulse width. 3 bits are 8 duty cycles/brightness values
(
    input clk,
    input rst,
    input [CTR_LEN - 1 : 0] pulse_width, //use a larger CTR_LEN for better resolution
    output pwm
  );
// test
 reg pwm_out;
 reg [CTR_LEN - 1: 0] counter; //counters same size as the PWM pulse width
 assign pwm = pwm_out;
   
  always @(posedge clk) begin
    if (~rst) 
		counter <= 1'b0;
    else
      begin		
		counter <= counter + 1'b1;
			if (pulse_width > counter)
				pwm_out = 1'b1;
				else
				pwm_out = 1'b0;
	  end
	end
   
  
endmodule
