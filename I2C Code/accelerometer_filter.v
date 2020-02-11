// Author               : Lucian Gozu 
// Created              : 3/3/2017
// Modified             : 
// Company              : Ohmics (www.ohmics.com)
//----------------------------------------------------------------------
// Description :  Generic PWM hi and low pulse filter
// This module is a generic de-bouncing circuit but it can be used as a PWM filter. 
// When a_pulse_en signal is high, the circuit will filter any spikes or pulses that have a width smaller than the pulse width plus one clock.
// adjust DEBOUNCE_HI_PERIOD accordingly as any pulse high shorter than DEBOUBCE_HI_PERIOD will be filtered (output will remain low)
// adjust DEBOUNCE_LO_PERIOD accordingly as any pulse high shorter than DEBOUBCE_LO_PERIOD will be filtered (output will remain HI)

//_____|``|____________|``|________|```````````````````|__|``````````````````|__|`````
//      DebHi           4ms                             DebLo                 4ms

module acc_filter_dual #(
                         parameter  CLK_PERIOD              = 480, // clk period in ns 480 for 2.08MHz
                         parameter  DEBOUNCE_HI_PERIOD      = 4,   // in ms  petren  if a high pulse is shorter than  DEBOUNCE_HI_PERIOD - output will be low
                         parameter  DEBOUNCE_LO_PERIOD      = 4,   // in ms  petren if a low pulse is shorter than DEBOUNCE_LO_PERIOD - output will be high
                         parameter  MIN_HI_PERIOD           = 0,   // in ms  the minimum time the FSM will stay in HI state (output will be high),
                                                                   //        ignoring any change on the a_pulse_in input
                         parameter  CUT_OFF_AFTER_HI_PERIOD = 0,   // if Set to 1, then the sigout output will be turned to low after it stayed hi for MIN_HI_PERIOD.
                                                                   // The output will stay low until the FSM transitions back to the LOW state and then back to HI state
                                                                   // Note: it makes no sense to set both CUT_OFF_AFTER_HI_PERIOD and RESTART_HI_PERIOD to 1
                         parameter  RESTART_HI_PERIOD       = 0   // if Set to 1, every time the pulse_in input is sampled 1 while in HI state, the MIN_HI_PERIOD
                                                                   // is restarted. See picture bellow

                         //         ==================  RESTART_HI_PERIOD = 0 ================
                         //            _________________ __________________________ ____________________ ______
                         // state        DEBOUNCE_HI    X       HI                 X DEBOUNCE_LOW       X LOW
                         //                             |<   MIN_HI_PERIOD        >|<DEBOUNCE_LO_PERIOD>|
                         //             _______________________         ____
                         // pulse_in                           |___..._|    |_...______________________________
                         //                                              ^
                         //                                              |
                         //                                            input change ignored while in MIN_HI_PERIOD
                         //
                         // cnt         X  ...    X2X1X0X X X  ...             X1X0X X ...          X1X0X
                         //                              ^ ^                        ^
                         //                              | |                        |
                         //                              | MIN_HI_CLK_CNT-1         DEBOUNCE_LO_CLK_CNT
                         //                              MIN_HI_CLK_CNT
                         //                              __________________________________________
                         // sigout     _________________/                                          \__________


                         //         ==================  RESTART_HI_PERIOD = 1 ================
                         //            _________________ ______________________________________ ____________________ ______
                         // state        DEBOUNCE_HI    X       HI                             X DEBOUNCE_LOW       X LOW
                         //                             |<   MIN_HI_PERIOD    >
                         //                             ...|<   MIN_HI_PERIOD    >
                         //                                        |<   MIN_HI_PERIOD    >
                         //                                             |<   MIN_HI_PERIOD    >|<DEBOUNCE_LO_PERIOD>|
                         //             ___________________         ____
                         // pulse_in                       |___..._|    |________________________________
                         //                                            ^
                         //                                            |
                         //                                           input hi restarts MIN_HI_PERIOD
                         //
                         // cnt         X  ...    X2X1X0X  X  ...  X    X X ...            X1X0X X ...          X1X0X
                         //                              ^^         ^    ^                      ^                  
                         //                              | |        |    |                      |                  
                         //                              | |        |    MIN_HI_CLK_CNT-1       DEBOUNCE_LO_CLK_CNT
                         //                              | |        MIN_HI_CLK_CNT
                         //                              | |
                         //                              | MIN_HI_CLK_CNT
                         //                              MIN_HI_CLK_CNT
                         //                              __________________________________________________________
                         // sigout     _________________/                                                           \__________

                         )
                       (
                        input  wire   clk,
                        input  wire   rst_n,            
                        input  wire   a_pulse_in,   // analog signal - has to be resync-ed to clk domain
                        output reg    sigout
                        );

   localparam   DEBOUNCE_MAX_PERIOD  = DEBOUNCE_HI_PERIOD > DEBOUNCE_LO_PERIOD ? DEBOUNCE_HI_PERIOD : DEBOUNCE_LO_PERIOD;
   localparam   DEBOUNCE_HI_CLK_CNT  = DEBOUNCE_HI_PERIOD*1000000/CLK_PERIOD;
   localparam   DEBOUNCE_LO_CLK_CNT  = DEBOUNCE_LO_PERIOD*1000000/CLK_PERIOD;
   localparam   DEBOUNCE_MAX_CLK_CNT = DEBOUNCE_HI_CLK_CNT > DEBOUNCE_LO_CLK_CNT ? DEBOUNCE_HI_CLK_CNT : DEBOUNCE_LO_CLK_CNT;
   localparam   MIN_HI_CLK_CNT       = MIN_HI_PERIOD*1000000/CLK_PERIOD;
   localparam   MAX_CLK_CNT          = DEBOUNCE_MAX_CLK_CNT > MIN_HI_CLK_CNT ? DEBOUNCE_MAX_CLK_CNT : MIN_HI_CLK_CNT;

   localparam   PULSE_COUNTER_WIDTH = cnt_width(MAX_CLK_CNT);

   localparam   LOW          = 0,
                DEBOUNCE_HI  = 1,
                HI           = 2,
                DEBOUNCE_LOW = 3;

   
   reg [1:0]      state_nxt, state;

  
   reg [PULSE_COUNTER_WIDTH-1:0]  cnt;
   reg [PULSE_COUNTER_WIDTH-1:0]  cnt_start_value;
   reg                            cnt_ld;
   reg                            cnt_en;
   wire                           cnt_zero;

   reg                            pulse_in_meta;
   reg                            pulse_in;

   reg                            sigout_nxt;
   
 //  assign sigout = (state == LOW) || (state == DEBOUNCE_HI);
   
   // re-sync a_pulse_en
   always @(posedge clk) begin
      pulse_in_meta <= a_pulse_in;
      pulse_in      <= pulse_in_meta;
   end

   
   always @(posedge clk or negedge rst_n) begin
      if(rst_n == 0) begin
         cnt      <= 0;
         sigout   <= 0;
         state    <= LOW;
      end
      else begin
         if(!cnt_en)
            cnt <= 0;
         else begin
            if(cnt_ld) 
               cnt <= cnt_start_value;
            else if(cnt > 0)
               cnt <= cnt - 1;
         end
         
         sigout   <= sigout_nxt;
         state    <= state_nxt;
      end
   end   

   assign   cnt_zero = (cnt == 0);

   always @* begin
      cnt_en = 0;
      cnt_ld = 0;
      cnt_start_value = DEBOUNCE_HI_CLK_CNT;
      state_nxt = state;

      sigout_nxt = sigout;

      case(state)
         LOW:
            begin
               sigout_nxt = 0;
               if(pulse_in) begin
                  cnt_en = 1;
                  cnt_ld = 1;
                  cnt_start_value = DEBOUNCE_HI_CLK_CNT;
                  state_nxt = DEBOUNCE_HI;
               end
            end

         DEBOUNCE_HI:
            begin
               sigout_nxt = 0;
               cnt_en = 1;
               cnt_ld = 0;

               if(pulse_in == 0) begin
                  cnt_en = 0;
                  cnt_ld = 0;
                  state_nxt = LOW;
               end
               else if(cnt_zero) begin
                  sigout_nxt  = 1;

                  cnt_en = 1;
                  cnt_ld = 1;
                  cnt_start_value = MIN_HI_CLK_CNT;
                  
                  state_nxt   = HI;
               end
            end

          HI:
             begin
               cnt_en = 1;

                if(CUT_OFF_AFTER_HI_PERIOD && cnt_zero)
                   sigout_nxt  = 0;
                else
                   sigout_nxt  = 1;

               if(!CUT_OFF_AFTER_HI_PERIOD && RESTART_HI_PERIOD && (pulse_in == 1)) begin
                  cnt_en = 1;
                  cnt_ld = 1;
                  cnt_start_value = MIN_HI_CLK_CNT;
               end
               else if(cnt_zero && (pulse_in == 0) ) begin
                  cnt_en = 1;
                  cnt_ld = 1;
                  cnt_start_value = DEBOUNCE_LO_CLK_CNT;
                  state_nxt = DEBOUNCE_LOW;
               end
            end // case: HI

          DEBOUNCE_LOW:
            begin
               if(CUT_OFF_AFTER_HI_PERIOD)
                  sigout_nxt  = 0;
               else
                  sigout_nxt  = 1;

               cnt_en         = 1;
               cnt_ld         = 0;

               if(pulse_in == 1) begin
                  if(!CUT_OFF_AFTER_HI_PERIOD && RESTART_HI_PERIOD) begin
                     cnt_en           = 1;
                     cnt_ld           = 1;
                     cnt_start_value  = MIN_HI_CLK_CNT;
                  end
                  else begin
                     cnt_en     = 0;
                     cnt_ld     = 0;
                  end
                  state_nxt  = HI;
               end
               else if(cnt_zero) begin
                  sigout_nxt = 0;
                  state_nxt = LOW;
               end
            end

         default: state_nxt = LOW;
      endcase

   end // always @ *

   function integer cnt_width(input integer max_count_value);
      for(cnt_width=1;2**cnt_width<max_count_value+1;cnt_width=cnt_width+1);
   endfunction
   
endmodule // pwm_filter_dual