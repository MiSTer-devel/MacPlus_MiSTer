`timescale 1ns / 100ps

/*
 * Zilog 8530 SCC module for minimigmac.
 *
 * Located on high data bus, but writes are done at odd addresses as
 * LDS is used as WR signals or something like that on a Mac Plus.
 * 
 * We don't care here and just ignore which side was used.
 * 
 * NOTE: We don't implement the 85C30 or ESCC additions such as WR7'
 * for now, it's all very simplified
 */

module scc
(
	input clk,
	input cep,
	input cen,

	input	reset_hw,

	/* Bus interface. 2-bit address, to be wired
	 * appropriately upstream (to A1..A2).
	 */
	input	cs,
	input	we,
	input [1:0]	rs, /* [1] = data(1)/ctl [0] = a_side(1)/b_side */
	input [7:0]	wdata,
	output [7:0]	rdata,
	output	_irq,

	/* A single serial port on Minimig */
	input	rxd,
	output	txd,
	input	cts, /* normally wired to device DTR output
				* on Mac cables. That same line is also
				* connected to the TRxC input of the SCC
				* to do fast clocking but we don't do that
				* here
				*/
	output	rts, /* on a real mac this activates line
				* drivers when low */

	/* DCD for both ports are hijacked by mouse interface */
	input	dcd_a, /* We don't synchronize those inputs */
	input	dcd_b,

	/* Write request */
	output	wreq
);

	/* Register access is semi-insane */
	reg [3:0]	rindex;
	reg [3:0]	rindex_latch;
	wire 		wreg_a;
	wire 		wreg_b;

	/* Resets via WR9, one clk pulses */
	wire		reset_a;
	wire		reset_b;
	wire		reset;

	/* Data registers */
//	reg [7:0] 	data_a = 0;
	wire[7:0] 	data_a ;
	reg [7:0] 	data_b = 0;

	/* Read registers */
	wire [7:0] 	rr0_a;
	wire [7:0] 	rr0_b;
	wire [7:0] 	rr1_a;
	wire [7:0] 	rr1_b;
	wire [7:0] 	rr2_b;
	wire [7:0] 	rr3_a;
	wire [7:0] 	rr10_a;
	wire [7:0] 	rr10_b;
	wire [7:0] 	rr15_a;
	wire [7:0] 	rr15_b;

	/* Write registers. Only some are implemented,
	 * some result in actions on write and don't
	 * store anything
	 */
	reg [7:0] 	wr1_a;
	reg [7:0] 	wr1_b;
	reg [7:0] 	wr2;
	reg [7:0] 	wr3_a;   /* synthesis keep */
	reg [7:0] 	wr3_b;
	reg [7:0] 	wr4_a;
	reg [7:0] 	wr4_b;
	reg [7:0] 	wr5_a;
	reg [7:0] 	wr5_b;
	reg [7:0] 	wr6_a;
	reg [7:0] 	wr6_b;
	reg [7:0] 	wr8_a;
	reg [7:0] 	wr8_b;
	reg [5:0] 	wr9;
	reg [7:0] 	wr10_a;
	reg [7:0] 	wr10_b;
	reg [7:0] 	wr12_a;
	reg [7:0] 	wr12_b;
	reg [7:0] 	wr13_a;
	reg [7:0] 	wr13_b;
	reg [7:0] 	wr14_a;
	reg [7:0] 	wr14_b;
	reg [7:0] 	wr15_a;
	reg [7:0] 	wr15_b;

	/* Status latches */
	reg		latch_open_a;
	reg		latch_open_b;
	reg		cts_latch_a;
	reg		dcd_latch_a;
	reg		dcd_latch_b;
	wire		cts_ip_a;
	wire		dcd_ip_a;
	wire		dcd_ip_b;
	wire		do_latch_a;
	wire		do_latch_b;
	wire		do_extreset_a;
	wire		do_extreset_b;	

	/* IRQ stuff */
	wire		rx_irq_pend_a;
	wire		rx_irq_pend_b;
	wire		tx_irq_pend_a;
	wire		tx_irq_pend_b;
	wire		ex_irq_pend_a;
	wire		ex_irq_pend_b;
	reg		ex_irq_ip_a;
	reg		ex_irq_ip_b;
	wire [2:0] 	rr2_vec_stat;	

	reg [7:0] tx_data_a;
	reg wr8_wr_a;
	reg wr8_wr_b;
		
	/* Register/Data access helpers */
	assign wreg_a  = cs & we & (~rs[1]) &  rs[0];
	assign wreg_b  = cs & we & (~rs[1]) & ~rs[0];

	// make sure rindex changes after the cpu cycle has ended so
	// read data is still stable while cpu advances
	always@(posedge clk) if(~cs) rindex <= rindex_latch;

	/* Register index is set by a write to WR0 and reset
	 * after any subsequent write. We ignore the side
	 */
	reg wr_data_a;
	reg wr_data_b;
	
	reg rx_wr_a_latch;
	reg rx_first_a=1;
	always@(posedge clk /*or posedge reset*/) begin
	
		if (rx_wr_a) begin
			rx_wr_a_latch<=1;
		end
	
	
		wr_data_a<=0;
		wr_data_b<=0;
		if (reset) begin
		  rindex_latch <= 0;
			//data_a <= 0;
			tx_data_a<=0;
			data_b <= 0;
			rx_wr_a_latch<=0;
			wr_data_a<=0;
			rx_first_a<=1;
		end else if (cen && cs) begin
			if (!rs[1]) begin
				/* Default, reset index */
				rindex_latch <= 0;

				/* Write to WR0 */
				if (we && rindex == 0) begin
					/* Get low index bits */
					rindex_latch[2:0] <= wdata[2:0];
				  
					/* Add point high */
					rindex_latch[3] <= (wdata[5:3] == 3'b001);
					/* enable int on next rx char */
					if (wdata[5:3] == 3'b100)
						rx_first_a<=1;
				end
			end else begin
				if (we) begin
					if (rs[0]) begin 
						//data_a <= wdata;
						tx_data_a <= wdata;
						wr_data_a<=1;
					end
					else
						begin
						data_b <= wdata;
						wr_data_b<=1;
						end
					end
				else begin
					// clear the read register?
					if (rs[0]) begin 
						rx_wr_a_latch<=0;
						rx_first_a<=0;
					end
					else begin
					
					end
				end
			end
		end
	end

	/* Reset logic (write to WR9 cmd)
	 *
	 * Note about resets: Some bits are documented as unchanged/undefined on
	 * HW reset by the doc. We apply this to channel and soft resets, however
	 * we _do_ reset every bit on an external HW reset in this implementation
	 * to make the FPGA & synthesis tools happy.
	 */
	assign reset   = ((wreg_a | wreg_b) & (rindex == 9) & (wdata[7:6] == 2'b11)) | reset_hw;
	assign reset_a = ((wreg_a | wreg_b) & (rindex == 9) & (wdata[7:6] == 2'b10)) | reset;	
	assign reset_b = ((wreg_a | wreg_b) & (rindex == 9) & (wdata[7:6] == 2'b01)) | reset;

	/* WR1
	 * Reset: bit 5 and 2 unchanged */
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr1_a <= 0;
		else if(cen) begin
			if (reset_a)
			  wr1_a <= { 2'b00, wr1_a[5], 2'b00, wr1_a[2], 2'b00 };
			else if (wreg_a && rindex == 1)
			  wr1_a <= wdata;
		end
	end
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr1_b <= 0;
		else if(cen) begin
			if (reset_b)
			  wr1_b <= { 2'b00, wr1_b[5], 2'b00, wr1_b[2], 2'b00 };
			else if (wreg_b && rindex == 1)
			  wr1_b <= wdata;
		end
	end

	/* WR2
	 * Reset: unchanged 
	 */
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr2 <= 0;
		else if (cen && (wreg_a || wreg_b) && rindex == 2)
		  wr2 <= wdata;			
	end

	/* WR3
	 * Reset: unchanged 
	 */
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr3_a <= 0;
		else if (cen && wreg_a && rindex == 3)
		  wr3_a <= wdata;
	end
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr3_b <= 0;		
		else if (cen && wreg_b && rindex == 3)
		  wr3_b <= wdata;
	end
	/* WR4
	 * Reset: unchanged 
	 */
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr4_a <= 0;
		else if (cen && wreg_a && rindex == 4)
		  wr4_a <= wdata;
	end
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr4_b <= 0;		
		else if (cen && wreg_b && rindex == 4)
		  wr4_b <= wdata;
	end

	/* WR5
	 * Reset: Bits 7,4,3,2,1 to 0
	 */
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr5_a <= 0;
		else if(cen) begin
			if (reset_a)
			  wr5_a <= { 1'b0, wr5_a[6:5], 4'b0000, wr5_a[0] };			
			else if (wreg_a && rindex == 5)
			  wr5_a <= wdata;
		end
	end
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr5_b <= 0;
		else if(cen) begin
			if (reset_b)
			  wr5_b <= { 1'b0, wr5_b[6:5], 4'b0000, wr5_b[0] };			
			else if (wreg_b && rindex == 5)
			  wr5_b <= wdata;
		end
	end

	/* WR8 : write data to serial port -- a or b?
	 * 
	 */
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw) begin
			wr8_a <= 0;
			wr8_wr_a <= 1'b0;
		end
		else if (cen && (rs[1] & we ) && rindex == 8) begin
			wr8_wr_a <= 1'b1;
			wr8_a <= wdata;			
		end
		else begin
	          wr8_wr_a <= 1'b0;
		end
	end

	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw) begin
		  wr8_b <= 0;
	          wr8_wr_b <= 1'b0;
		end
		else if (cen && (wreg_b ) && rindex == 8)
		begin
	          wr8_wr_b <= 1'b1;
		  wr8_b <= wdata;			
		end
		else
		begin
	          wr8_wr_b <= 1'b0;
		end
	end
	
	/* WR9. Special: top bits are reset, handled separately, bottom
	 * bits are only reset by a hw reset
	 */
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr9 <= 0;
		else if (cen && (wreg_a || wreg_b) && rindex == 9)
		  wr9 <= wdata[5:0];			
	end

	/* WR10
	 * Reset: all 0, except chanel reset retains 6 and 5
	 */
	always@(posedge clk or posedge reset) begin
		if (reset)
		  wr10_a <= 0;
		else if(cen) begin
			if (reset_a)
			  wr10_a <= { 1'b0, wr10_a[6:5], 5'b00000 };
			else if (wreg_a && rindex == 10)
			  wr10_a <= wdata;
		end		
	end
	always@(posedge clk or posedge reset) begin
		if (reset)
		  wr10_b <= 0;
		else if(cen) begin
			if (reset_b)
			  wr10_b <= { 1'b0, wr10_b[6:5], 5'b00000 };
			else if (wreg_b && rindex == 10)
			  wr10_b <= wdata;
		end		
	end

	/* WR12
	 * Reset: Unchanged
	 */
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr12_a <= 0;
		else if (cen && wreg_a && rindex == 12)
		  wr12_a <= wdata;
	end
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr12_b <= 0;		
		else if (cen && wreg_b && rindex == 12)
		  wr12_b <= wdata;
	end

	/* WR13
	 * Reset: Unchanged
	 */
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr13_a <= 0;
		else if (cen && wreg_a && rindex == 13)
		  wr13_a <= wdata;
	end
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr13_b <= 0;		
		else if (cen && wreg_b && rindex == 13)
		  wr13_b <= wdata;
	end

	/* WR14
	 * Reset: Full reset maintains  top 2 bits,
	 * Chan reset also maitains bottom 2 bits, bit 4 also
	 * reset to a different value
	 */
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr14_a <= 0;
		else if(cen) begin
			if (reset)
			  wr14_a <= { wr14_a[7:6], 6'b110000 };
			else if (reset_a)
			  wr14_a <= { wr14_a[7:6], 4'b1000, wr14_a[1:0] };
			else if (wreg_a && rindex == 14)
			  wr14_a <= wdata;
		end		
	end
	always@(posedge clk or posedge reset_hw) begin
		if (reset_hw)
		  wr14_b <= 0;
		else if(cen) begin
			if (reset)
			  wr14_b <= { wr14_b[7:6], 6'b110000 };
			else if (reset_b)
			  wr14_b <= { wr14_b[7:6], 4'b1000, wr14_b[1:0] };
			else if (wreg_b && rindex == 14)
			  wr14_b <= wdata;
		end		
	end

	/* WR15 */
	always@(posedge clk or posedge reset) begin
		if (reset) begin
		  wr15_a <= 8'b11111000;
		  wr15_b <= 8'b11111000;
		end else if (cen && rindex == 15) begin
		  if(wreg_a) wr15_a <= wdata;			
		  if(wreg_b) wr15_b <= wdata;			
		end
	end
	
	/* Read data mux */	
	assign rdata = rs[1] && rs[0]       ? data_a :
		       rs[1]                ? data_b :
		       rindex ==  0 && rs[0] ? rr0_a :
		       rindex ==  0          ? rr0_b :
		       rindex ==  1 && rs[0] ? rr1_a :
		       rindex ==  1          ? rr1_b :
		       rindex ==  2 && rs[0] ? wr2 :
		       rindex ==  2          ? rr2_b :
		       rindex ==  3 && rs[0] ? rr3_a :
		       rindex ==  3          ? 8'h00 :
		       rindex ==  4 && rs[0] ? rr0_a :
		       rindex ==  4          ? rr0_b :
		       rindex ==  5 && rs[0] ? rr1_a :
		       rindex ==  5          ? rr1_b :
		       rindex ==  6 && rs[0] ? wr2 :
		       rindex ==  6          ? rr2_b :
		       rindex ==  7 && rs[0] ? rr3_a :
		       rindex ==  7          ? 8'h00 :

		       rindex ==  8 && rs[0] ? data_a :
		       rindex ==  8          ? data_b :
		       rindex ==  9 && rs[0] ? wr13_a :
		       rindex ==  9          ? wr13_b :
		       rindex == 10 && rs[0] ? rr10_a :
		       rindex == 10          ? rr10_b :
		       rindex == 11 && rs[0] ? rr15_a :
		       rindex == 11          ? rr15_b :
		       rindex == 12 && rs[0] ? wr12_a :
		       rindex == 12          ? wr12_b :
		       rindex == 13 && rs[0] ? wr13_a :
		       rindex == 13          ? wr13_b :
		       rindex == 14 && rs[0] ? rr10_a :
		       rindex == 14          ? rr10_b :
		       rindex == 15 && rs[0] ? rr15_a :
		       rindex == 15          ? rr15_b : 8'hff;

	/* RR0 */
	assign rr0_a = { 1'b0, /* Break */
			 1'b1, /* Tx Underrun/EOM */
			 wr15_a[5] ? cts_latch_a : cts_a, /* CTS */
			 1'b0, /* Sync/Hunt */
			 wr15_a[3] ? dcd_latch_a : dcd_a, /* DCD */
			 //1'b1, /*TX EMPTY */
			 ~tx_busy_a, /* Tx Empty */
			 1'b0, /* Zero Count */
			 rx_wr_a_latch  /* Rx Available */
			 };
	assign rr0_b = { 1'b0, /* Break */
			 1'b1, /* Tx Underrun/EOM */
			 1'b0, /* CTS */
			 1'b0, /* Sync/Hunt */
			 wr15_b[3] ? dcd_latch_b : dcd_b, /* DCD */
			 1'b1, /* Tx Empty */
			 1'b0, /* Zero Count */
			 1'b0  /* Rx Available */
			 };

	/* RR1 */
	assign rr1_a = { 1'b0, /* End of frame */
			 1'b0,//frame_err_a, /* CRC/Framing error */
			 1'b0, /* Rx Overrun error */
			 1'b0,//parity_err_a, /* Parity error */
			 1'b0, /* Residue code 0 */
			 1'b1, /* Residue code 1 */
			 1'b1, /* Residue code 2 */
			 ~tx_busy_a  /* All sent */
			 };
	
	assign rr1_b = { 1'b0, /* End of frame */
			 1'b0, /* CRC/Framing error */
			 1'b0, /* Rx Overrun error */
			 1'b0, /* Parity error */
			 1'b0, /* Residue code 0 */
			 1'b1, /* Residue code 1 */
			 1'b1, /* Residue code 2 */
			 1'b1  /* All sent */
			 };
	
	/* RR2 (Chan B only, A is just WR2) */
	assign rr2_b = { wr2[7],
			 wr9[4] ? rr2_vec_stat[0] : wr2[6],
			 wr9[4] ? rr2_vec_stat[1] : wr2[5],
			 wr9[4] ? rr2_vec_stat[2] : wr2[4],
			 wr9[4] ? wr2[3] : rr2_vec_stat[2],
			 wr9[4] ? wr2[2] : rr2_vec_stat[1],
			 wr9[4] ? wr2[1] : rr2_vec_stat[0],
			 wr2[0]
			 };
	

	/* RR3 (Chan A only) */
	assign rr3_a = { 2'b0,
			 rx_irq_pend_a, /* Rx interrupt pending */
			 tx_irq_pend_a, /* Tx interrupt pending */
			 ex_irq_pend_a, /* Status/Ext interrupt pending */
			 rx_irq_pend_b,
			 tx_irq_pend_b,
			 ex_irq_pend_b
			};

	/* RR10 */
	assign rr10_a = { 1'b0, /* One clock missing */
			  1'b0, /* Two clocks missing */
			  1'b0,
			  1'b0, /* Loop sending */
			  1'b0,
			  1'b0,
			  1'b0, /* On Loop */
			  1'b0
			  };
	assign rr10_b = { 1'b0, /* One clock missing */
			  1'b0, /* Two clocks missing */
			  1'b0,
			  1'b0, /* Loop sending */
			  1'b0,
			  1'b0,
			  1'b0, /* On Loop */
			  1'b0
			  };
	
	/* RR15 */
	assign rr15_a = { wr15_a[7],
			  wr15_a[6],
			  wr15_a[5],
			  wr15_a[4],
			  wr15_a[3],
			  1'b0,
			  wr15_a[1],
			  1'b0
			  };

	assign rr15_b = { wr15_b[7],
			  wr15_b[6],
			  wr15_b[5],
			  wr15_b[4],
			  wr15_b[3],
			  1'b0,
			  wr15_b[1],
			  1'b0
			  };
	
	/* Interrupts. Simplified for now
	 *
	 * Need to add latches. Tx irq is latched when buffer goes from full->empty,
	 * it's not a permanent state. For now keep it clear. Will have to fix that.
	* TODO: AJS - look at tx and interrupt logic
	 */
	 
	 /*
	 The TxIP is reset either by writing data to the transmit buffer or by issuing the Reset Tx Int command in WR0
	 */

reg tx_fin_pre;
reg tx_ip;
reg tx_mip;

/*
reg tx_ie;
always @(posedge clk) begin
	if (reset_a|reset_hw|reset)
		tx_ie<=0;
	else if (wreg_a & (rindex == 1) )
		tx_ie<=wdata[1];
end
*/

always @(posedge clk) begin
	if (reset) begin
      tx_ip<=0;
      tx_mip<=0;
	end
	else begin
      tx_fin_pre<=tx_busy_a;
		 
		if (wr5_a[3] &  wr1_a[1] & tx_busy_a & ~tx_fin_pre) begin
			tx_ip<=~tx_mip;
			tx_mip<=0;
		end
		if (wreg_a & (rindex == 0) & (wdata[5:3] == 3'b111)) begin
			tx_ip<=0;
		end
		if (wreg_a & (rindex == 0) & (wdata[5:3] == 3'b101)) begin
          // If CIP=1, inhibit generation of next TX interrupt
          // Actually, "Reset TxInt pend." clears current interrupt
          tx_mip<= ~tx_ip;
          tx_ip<=0;
		end
		if (wr5_a[3]==0)begin
			tx_mip<=0;
			tx_ip<=0;
		end
	end	
end





	 reg tx_busy_a_r;
	 reg tx_latch_a;
	 always @(posedge clk) begin
		tx_busy_a_r <= tx_busy_a;
		// when we transition from empty to full, we create an interrupt
		if (reset | reset_hw | reset_a)
			tx_latch_a<=0;
		else if  (tx_busy_a_r ==1 && tx_busy_a==0)
			tx_latch_a<=1;
		// cleared when we write again
		else if (wr_data_a)
			tx_latch_a<=0;
		// or when we set the reset in wr0
		else if (wreg_a & (rindex == 0) & (wdata[5:3] == 3'b010))
			tx_latch_a<=0;
		//else if (wreg_a & (rindex == 0) & (wdata[5:3] == 3'b111)) // clear highest under service?
	 end

	 
	 
	 wire wreq_n;
	//assign rx_irq_pend_a =  rx_wr_a_latch & ( (wr1_a[3] &&  ~wr1_a[4])|| (~wr1_a[3] &&  wr1_a[4])) & wr3_a[0];	/* figure out the interrupt on / off */
	//assign rx_irq_pend_a =  rx_wr_a_latch & ( (wr1_a[3] &  ~wr1_a[4])| (~wr1_a[3] &  wr1_a[4])) & wr3_a[0];	/* figure out the interrupt on / off */

	/* figure out the interrupt on / off */
	/* rx enable: wr3_a[0] */
	/* wr1_a  4  3
	          0  0  = rx int disable
	          0  1  = rx int on first char or special
				 1  0  = rx int on all rx chars or special
				 1  1  = rx int on special cond only
	*/
	//                       rx enable   char waiting        01,10 only             first char    
	assign rx_irq_pend_a =   wr3_a[0] & rx_wr_a_latch & (wr1_a[3] ^ wr1_a[4]) & ((wr1_a[3] & rx_first_a )|(wr1_a[4]));

//	assign tx_irq_pend_a = 0;
//	assign tx_irq_pend_a = tx_busy_a & wr1_a[1];

	assign tx_irq_pend_a = tx_ip;
//assign tx_irq_pend_a =  wr1_a[1]; /* Tx always empty for now */

   wire cts_interrupt = wr1_a[0] &&  wr15_a[5] || (tx_busy_a_r ==1 && tx_busy_a==0) || (tx_busy_a_r ==0 && tx_busy_a==1);/* if cts changes */

	assign ex_irq_pend_a = ex_irq_ip_a ; 
	assign rx_irq_pend_b = 0;
	assign tx_irq_pend_b = 0 /*& wr1_b[1]*/; /* Tx always empty for now */
	assign ex_irq_pend_b = ex_irq_ip_b;

	assign _irq = ~(wr9[3] & (rx_irq_pend_a |
				  
				  
				  rx_irq_pend_b |
				  tx_irq_pend_a |
				  tx_irq_pend_b |
				  ex_irq_pend_a |
				  ex_irq_pend_b));

	/* XXX Verify that... also missing special receive condition */
	assign rr2_vec_stat = rx_irq_pend_a ? 3'b110 :
			      tx_irq_pend_a ? 3'b100 :
			      ex_irq_pend_a ? 3'b101 :
			      rx_irq_pend_b ? 3'b010 :
			      tx_irq_pend_b ? 3'b000 :
			      ex_irq_pend_b ? 3'b001 : 3'b011;
	
	/* External/Status interrupt & latch logic */
	assign do_extreset_a = wreg_a & (rindex == 0) & (wdata[5:3] == 3'b010);
	assign do_extreset_b = wreg_b & (rindex == 0) & (wdata[5:3] == 3'b010);

	/* Internal IP bit set if latch different from source and
	 * corresponding interrupt is enabled in WR15
	 */
	assign dcd_ip_a = (dcd_a != dcd_latch_a) & wr15_a[3];
	assign cts_ip_a = (cts_a != cts_latch_a) & wr15_a[5];
	assign dcd_ip_b = (dcd_b != dcd_latch_b) & wr15_b[3];

	/* Latches close when an enabled IP bit is set and latches
	 * are currently open
	 */
	assign do_latch_a = latch_open_a & (dcd_ip_a | cts_ip_a  /* | cts... */);
	assign do_latch_b = latch_open_b & (dcd_ip_b /* | cts... */);

	/* "Master" interrupt, set when latch close & WR1[0] is set */
	always@(posedge clk or posedge reset) begin
		if (reset)
		  ex_irq_ip_a <= 0;
		else if(cep) begin
			if (do_extreset_a)
			  ex_irq_ip_a <= 0;
			else if (do_latch_a && wr1_a[0])
			  ex_irq_ip_a <= 1;
		end
	end
	always@(posedge clk or posedge reset) begin
		if (reset)
		  ex_irq_ip_b <= 0;
		else if(cep) begin
			if (do_extreset_b)
			  ex_irq_ip_b <= 0;
			else if (do_latch_b && wr1_b[0])
			  ex_irq_ip_b <= 1;
		end
	end

	/* Latch open/close control */
	always@(posedge clk or posedge reset) begin
		if (reset)
		  latch_open_a <= 1;
		else if(cep) begin
			if (do_extreset_a)
			  latch_open_a <= 1;
			else if (do_latch_a)
			  latch_open_a <= 0;
		end
	end
	always@(posedge clk or posedge reset) begin
		if (reset)
		  latch_open_b <= 1;
		else if(cep) begin
			if (do_extreset_b)
			  latch_open_b <= 1;
			else if (do_latch_b)
			  latch_open_b <= 0;
		end
	end

	/* Latches proper */
	always@(posedge clk or posedge reset) begin
		if (reset) begin
			dcd_latch_a <= 0;
			cts_latch_a <= 0;
			/* cts ... */
		end else if(cep) begin
			if (do_latch_a)
			  dcd_latch_a <= dcd_a;
			  cts_latch_a <= cts_a;
			/* cts ... */
		end
	end
	always@(posedge clk or posedge reset) begin
		if (reset) begin
			dcd_latch_b <= 0;			
			/* cts ... */
		end else if(cep) begin
			if (do_latch_b)
			  dcd_latch_b <= dcd_b;
			/* cts ... */
		end
	end
	


	/* NYI */
//	assign txd = 1;
//	assign rts = 1;

	/* UART */

//wr_3_a
//wr_3_b
// bit 
wire parity_ena_a= wr4_a[0];
wire parity_even_a= wr4_a[1];
reg [1:0] stop_bits_a= 2'b00;
reg [1:0] bit_per_char_a = 2'b00;
/*
76543210
data>>2 & 3
wr4_a[3:2] 
case(wr4_a[3:2])
2'b00:
// sync mode enable
2'b01:
// 1 stop bit
	stop_bits_a <= 2'b0;
2'b10:
// 1.5 stop bit
	stop_bits_a <= 2'b0;
2'b11:
// 2 stop bit
	stop_bits_a <= 2'b1;
default:
	stop_bits_a <= 2'b0;
endcase

*/
/*
76543210
^__ 76 
wr_3_a[7:6]  -- bits per char

                case (wr_3_a[7:6]})
                        2'b00:  // 5
				bit_per_char_a  <= 2'b11;
                        2'b01:  // 7
				bit_per_char_a  <= 2'b01;
                        2'b10:  // 6 
				bit_per_char_a  <= 2'b10;
                        2'b11:  // 8
				bit_per_char_a  <= 2'b00;
		endcase
*/
/*
300 -- 62.668800 /  =  208896
600 -- 62.668800 /  =  104448
1200-- 62.668800 /  =  69632
2400 -- 62.668800 / 2400 = 26112
4800 -- 62.668800 / 4800  = 13056
9600 -- 62.668800 / 9600 = 6528
1440 -- 62.668800 / 14400 = 4352
19200 -- 62.668800 /  19200= 3264
38400 -- 62.668800 / 28800 =  2176
38400 -- 62.668800 / 38400 =  1632
57600 -- 62.668800 / 57600 = 1088
115200 -- 62.668800 / 115200 = 544
230400 -- 62.668800 / 230400 = 272


32.5 / 115200 = 

*/
// case the baud rate based on wr12_a and 13_a
// wr_12_a  -- contains the baud rate lower byte
// wr_13_a  -- contains the baud rate high byte
/*
        always @(posedge clk) begin
                case ({wr13_a,wr12_a})
                        16'd380:  // 300 baud
                                baud_divid_speed_a <= 24'd108333;
                        16'd94:  // 1200 baud
                                baud_divid_speed_a <= 24'd27083;
                        16'd46:  // 2400 baud
                                baud_divid_speed_a <= 24'd13542;
                        16'd22:  // 4800 baud
                                baud_divid_speed_a <= 24'd6770;
                        16'd10:  // 9600 baud
                                baud_divid_speed_a <= 24'd3385;
                        16'd6:  // 14400 baud
                                baud_divid_speed_a <= 24'd2257;
                        16'd4:  // 19200 baud
                                baud_divid_speed_a <= 24'd1693;
                        16'd2:  // 28800 baud
                                baud_divid_speed_a <= 24'd1128;
                        16'd1:  // 38400 baud
                                baud_divid_speed_a <= 24'd846;
                        16'd0:  // 57600 baud
                                baud_divid_speed_a <= 24'd564;
                        default: 
                                baud_divid_speed_a <= 24'd282;
                endcase
        end

*/



//reg [23:0] baud_divid_speed_a = 24'd1088;
//reg [23:0] baud_divid_speed_a = 24'd544;
reg [23:0] baud_divid_speed_a = 24'd282;
//reg [23:0] baud_divid_speed_a = 24'd564;
wire tx_busy_a;
wire rx_wr_a;
wire [30:0] uart_setup_rx_a = { 1'b0, bit_per_char_a, 1'b0, parity_ena_a, 1'b0, parity_even_a, baud_divid_speed_a  } ;
wire [30:0] uart_setup_tx_a = { 1'b0, bit_per_char_a, 1'b0, parity_ena_a, 1'b0, parity_even_a, baud_divid_speed_a  } ;
//wire [30:0] uart_setup_rx_a = { 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, baud_divid_speed_a  } ;
//wire [30:0] uart_setup_tx_a = { 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, baud_divid_speed_a  } ;
rxuart rxuart_a (
	.i_clk(clk), 
	.i_reset(reset_a|reset_hw), 
	.i_setup(uart_setup_rx_a), 
	.i_uart_rx(rxd), 
	.o_wr(rx_wr_a), // TODO -- check on this flag
	.o_data(data_a),   // TODO we need to save this off only if wreq is set, and mux it into data_a in the right spot
	.o_break(break_a),
	.o_parity_err(parity_err_a), 
	.o_frame_err(frame_err_a), 
	.o_ck_uart()
	);
txuart txuart_a
	(
	.i_clk(clk), 
	.i_reset(reset_a|reset_hw), 
	.i_setup(uart_setup_tx_a), 
	.i_break(1'b0), 
	.i_wr(wr_data_a),   // TODO -- we need to send data when we get the register command i guess???
	.i_data(tx_data_a),
	//.i_cts_n(~cts), 
	.i_cts_n(1'b0), 
	.o_uart_tx(txd), 
	.o_busy(tx_busy_a)); // TODO -- do we need this busy line?? probably 

	wire cts_a = ~tx_busy_a;
	
	// RTS and CTS are active low
	assign rts = rx_wr_a_latch;
	assign wreq=1;
endmodule
