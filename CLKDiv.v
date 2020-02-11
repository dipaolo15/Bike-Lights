module dclock(clk, reset, clko);
     input clk;
     input reset;
     output clko;
  parameter divider=10; // select the division factor here
 reg clko;
 integer i=0;
 
 always@(posedge clk)
 begin

 if(!reset)
 begin
 clko=0;
 i=0;
 end


 else if(i<((divider/2)-1))
  begin
  clko=0;
  i=i+1;
  end
 
 else if(i==((divider/2)-1))
 
 begin
 clko=1;
 i=i+1;
 end

 else if(i<(divider-1))
  begin
  clko=1;
  i=i+1;
  end
 else if(i==divider-1)
  begin
  clko=0;
  i=0;
  end 
  

 end

 endmodule