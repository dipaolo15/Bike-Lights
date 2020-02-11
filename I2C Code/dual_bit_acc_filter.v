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
                         parameter  CLK_PERIOD                    = 480, // clk period in ns 480 for 2.08MHz
                         parameter  DEBOUNCE_HI_PERIOD            = 4,   // in ms  petren  if a high pulse is shorter than  DEBOUNCE_HI_PERIOD - output will be low
                         parameter  DEBOUNCE_LO_PERIOD            = 4,   // in ms  petren if a low pulse is shorter than DEBOUNCE_LO_PERIOD - output will be high
                         parameter  MIN_HI_PERIOD                 = 0,   // in ms  the minimum time the FSM will stay in HI state (output will be high),
                                                                         //        ignoring any change on the a_pulse_in input
                         parameter  CUT_OFF_AFTER_HI_PERIOD       = 0,   // if Set to 1, then the sigout output will be turned to low after it stayed hi for MIN_HI_PERIOD.
                                                                         // The output will stay low until the FSM transitions back to the LOW state and then back to HI state
                                                                         // Note: it makes no sense to set both CUT_OFF_AFTER_HI_PERIOD and RESTART_HI_PERIOD to 1
                         parameter  RESTART_HI_PERIOD             = 0,   // if Set to 1, every time the pulse_in input is sampled 1 while in HI state, the MIN_HI_PERIOD
                                                                         // is restarted. See picture bellow
                         parameter  CUT_OFF_BOOST_AFTER_HI_PERIOD = 0,   // Same as CUT_OFF_AFTER_HI_PERIOD but it applied to "boost" output
                         parameter  IGNORE_BOOST_DURING_HI_PERIOD = 0   // If Set to 1, both, pulse_in and boost_in inputs are ignored within the MIN_HI_PERIOD, in HI__NO_BOOST state

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
                        input  wire   a_boost_in,   // analog signal - has to be resync-ed to clk domain
                        output reg    sigout,
                        output reg    boost
                        );

   localparam   DEBOUNCE_MAX_PERIOD  = DEBOUNCE_HI_PERIOD > DEBOUNCE_LO_PERIOD ? DEBOUNCE_HI_PERIOD : DEBOUNCE_LO_PERIOD;
   localparam   DEBOUNCE_HI_CLK_CNT  = DEBOUNCE_HI_PERIOD*1000000/CLK_PERIOD;
   localparam   DEBOUNCE_LO_CLK_CNT  = DEBOUNCE_LO_PERIOD*1000000/CLK_PERIOD;
   localparam   DEBOUNCE_MAX_CLK_CNT = DEBOUNCE_HI_CLK_CNT > DEBOUNCE_LO_CLK_CNT ? DEBOUNCE_HI_CLK_CNT : DEBOUNCE_LO_CLK_CNT;
   localparam   MIN_HI_CLK_CNT       = MIN_HI_PERIOD*1000000/CLK_PERIOD;
   localparam   MAX_CLK_CNT          = DEBOUNCE_MAX_CLK_CNT > MIN_HI_CLK_CNT ? DEBOUNCE_MAX_CLK_CNT : MIN_HI_CLK_CNT;

   localparam   PULSE_COUNTER_WIDTH = cnt_width(MAX_CLK_CNT);

   localparam   LOW                   = 0,
                DEBOUNCE_HI           = 1,
                HI__NO_BOOST          = 2,
                HI__DEBOUNCE_BOOST    = 3,
                HI__WITH_BOOST        = 4,
                HI__DEBOUNCE_NO_BOOST = 5,
                DEBOUNCE_LOW          = 6;

   
   reg [2:0]      state_nxt, state;

  
   reg [PULSE_COUNTER_WIDTH-1:0]  cnt;
   reg [PULSE_COUNTER_WIDTH-1:0]  cnt_start_value;
   reg                            cnt_ld;
   reg                            cnt_en;
   wire                           cnt_zero;

   reg                            pulse_in_meta;
   reg                            pulse_in;

   reg                            boost_in_meta;
   reg                            boost_in;

   reg                            sigout_nxt;
   reg                            boost_nxt;
   
 //  assign sigout = (state == LOW) || (state == DEBOUNCE_HI);
   
   // re-sync a_pulse_in
   always @(posedge clk) begin
      pulse_in_meta <= a_pulse_in;
      pulse_in      <= pulse_in_meta;
   end

   // re-sync a_boost_in
   always @(posedge clk) begin
      boost_in_meta <= a_boost_in;
      boost_in      <= boost_in_meta;
   end

   
   always @(posedge clk or negedge rst_n) begin
      if(rst_n == 0) begin
         cnt    <= 0;
         sigout <= 0;
         boost  <= 0;
         state  <= LOW;
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
         
         sigout <= sigout_nxt;
         boost  <= boost_nxt;
         state  <= state_nxt;
      end
   end   

   assign   cnt_zero = (cnt == 0);

   always @* begin
      cnt_en           = 0;
      cnt_ld           = 0;
      cnt_start_value  = DEBOUNCE_HI_CLK_CNT;
      state_nxt        = state;

      sigout_nxt       = sigout;
      boost_nxt        = boost;

      case(state)
         LOW:
            begin
               sigout_nxt = 0;
               boost_nxt = 0;
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
               boost_nxt = 0;
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
                  
                  state_nxt   = HI__NO_BOOST;
               end
            end

          HI__NO_BOOST:
             begin
                cnt_en = 1;

                // We could be in this state after one of this three possible transitions:
                // 1) DEBOUNCE_HI -> HI__NO_BOOST
                // 2) DEBOUNCE_HI -> HI__NO_BOOST -> HI__DEBOUNCE_BOOST -> HI__NO_BOOST with sigout HI active for less than MIN_HI_CLK_CNT since the transition out of DEBOUNCE_HI state
                // 3) DEBOUNCE_HI -> HI__NO_BOOST -> HI__DEBOUNCE_BOOST -> HI__NO_BOOST with sigout HI active for more than MIN_HI_CLK_CNT since the transition out of DEBOUNCE_HI state
                // 4) DEBOUNCE_HI -> HI__NO_BOOST -> HI__DEBOUNCE_BOOST -> HI__WITH_BOOST -> HI__DEBOUNCE_NO_BOOST -> HI__NO_BOOST
                //
                // For cases 1) and 2) sigout should be HI until the MIN_HI_CLK_CNT period is done (see the MIN_HI_CLK_CNT requirement for sigout HI)
                // For case  4) the FSM stays in HI__WITH_BOOST for at least MIN_HI_CLK_CNT period, while sigout and boost outputs are kept HI, so at the transition back to HI__NO_BOOST the sigout and boost can go low
                // For case  3) sigout and boost outputs should be low
                //
                // So the boost output should be LOW in this state, but sigout could be HI (cases 1), 2) ) or LOW (cases 3) and 4) ), while the counter is still running.
                // The value of sigout should be set before the FSM transitions to HI__NO_BOOST state and should be preserved while the counter is still running, in this state.
                boost_nxt  = 0;
                if(cnt_zero) begin // The counter may have been loaded in DEBOUNCE_HI state or in HI__DEBOUNCE_BOOST state
                   if(CUT_OFF_AFTER_HI_PERIOD)
                      sigout_nxt  = 0;
                   else
                      sigout_nxt  = sigout;
                end
                else begin
                   sigout_nxt  = sigout;
                end

               if(boost_in && (!IGNORE_BOOST_DURING_HI_PERIOD || cnt_zero)) begin
                  cnt_en = 1;
                  cnt_ld = 1;
                  cnt_start_value = DEBOUNCE_HI_CLK_CNT;

                  state_nxt = HI__DEBOUNCE_BOOST;
               end
               else if(!CUT_OFF_AFTER_HI_PERIOD && RESTART_HI_PERIOD && (pulse_in == 1)) begin
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

         HI__DEBOUNCE_BOOST:
            begin
               sigout_nxt  = sigout; // keep sigout output in the state it was before the transition to HI__DEBOUNCE_BOOST state ( case 2) or case 3) ).
               boost_nxt = 0;
               cnt_en = 1;
               cnt_ld = 0;

               if( (boost_in == 0) || (pulse_in == 0) ) begin
                  
                  cnt_en           = 1;
                  cnt_ld           = 1;
                  if(sigout)
                     cnt_start_value  = MIN_HI_CLK_CNT; // load the counter with MIN_HI_CLK_CNT for case 2), explained in the comments in the HI__NO_BOOST state
                  else
                     cnt_start_value  = 0;              // clear the counter for case 3), explained in the comments in the HI__NO_BOOST state

                  state_nxt        = HI__NO_BOOST;
               end
               else if(cnt_zero) begin
                  sigout_nxt  = 1;
                  boost_nxt  = 1;

                  // load the counter with MIN_HI_CLK_CNT so that sigout and boost outputs stay asserted for at least MIN_HI_CLK_CNT 
                  cnt_en = 1;
                  cnt_ld = 1;
                  cnt_start_value = MIN_HI_CLK_CNT;
                  
                  state_nxt   = HI__WITH_BOOST;
               end
            end // case: HI__DEBOUNCE_BOOST

         HI__WITH_BOOST:
            begin
               sigout_nxt  = 1;
               boost_nxt = 1;

               cnt_en = 1;

               // should sigout and boost outputs be deasserted after MIN_HI_CLK_CNT ???
               if(cnt_zero) begin // The counter may have been loaded in DEBOUNCE_HI state or in HI__DEBOUNCE_BOOST state
                  if(CUT_OFF_BOOST_AFTER_HI_PERIOD) begin // maybe another parameter like CUT_OFF_BOOST_AFTER_HI_PERIOD could be used
                     boost_nxt   = 0;
                     if(CUT_OFF_AFTER_HI_PERIOD)
                        sigout_nxt  = 0;
                  end
               end

               if(cnt_zero && ( (boost_in == 0) || (pulse_in == 0) ) ) begin // sigout and boost outputs must stay asserted for at least MIN_HI_CLK_CNT
                  cnt_en = 1;
                  cnt_ld = 1;
                  cnt_start_value = DEBOUNCE_LO_CLK_CNT;

                  state_nxt = HI__DEBOUNCE_NO_BOOST;
               end
            end // case: HI__WITH_BOOST

         HI__DEBOUNCE_NO_BOOST:
            begin
               sigout_nxt = sigout;
               boost_nxt  = boost;

               cnt_en         = 1;
               cnt_ld         = 0;

               if(boost_in == 1) begin
                  cnt_en = 1;
                  cnt_ld = 1;
                  cnt_start_value = 0;
                  
                  state_nxt  = HI__WITH_BOOST;
               end
               else if(cnt_zero) begin
                  boost_nxt = 0;
                  if(CUT_OFF_AFTER_HI_PERIOD)
                     sigout_nxt  = 0;

                  state_nxt = HI__NO_BOOST;
               end
            end

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
                  state_nxt  = HI__NO_BOOST;
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