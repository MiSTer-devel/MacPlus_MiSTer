/* IWM 

   Mapped to $DFE1FF - $DFFFFF
	
	The 16 IWM one-bit registers are {8'hDF, 8'b111xxxx1, 8'hFF}:
		0	$0		ca0L		CA0 off (0)
		1	$200	ca0H		CA0 on (1)
		2	$400	ca1L		CA1 off (0)
		3	$600	ca1H		CA1 on (1)
		4	$800	ca2L		CA2 off (0)
		5	$A00	ca2H		CA2 on (1)
		6	$C00	ph3L		LSTRB off (low)
		7	$E00	ph3H		LSTRB on (high)
		8	$1000	mtrOff	ENABLE disk enable off
		9	$1200	mtrOn		ENABLE disk enable on
		10	$1400	intDrive	SELECT select internal drive
		11	$1600	extDrive	SELECT select external drive
		12	$1800	q6L		Q6 off
		13	$1A00	q6H		Q6 on
		14	$1C00	q7L		Q7 off, read register
		15	$1E00	q7H		Q7 on, write register
	
	Notes from IWM manual:
	Serial data is shifted in/out MSB first, with a bit transferred every 2 microseconds.
	When writing data, a 1 is written as a transition on writeData at a bit cell boundary time, and a 0 is written as no transition.
	When reading data, a falling transition within a bit cell window is considered to be a 1, and no falling transition is considered a 0.
	When reading data, the read data register will latch the shift register when a 1 is shifted into the MSB.
	The read data register will be cleared 14 fclk periods (about 2 microseconds) after a valid data read takes place-- a valid data read 
	   being defined as both /DEV being low and D7 (the MSB) outputting a one from the read data register for at least one fclk period.
*/		

module iwm
(
	input clk,
	input cep,
	input cen,
	// clk16_en_n and turbo scale only the read-latch clear interval; the 16 us
	// byte rate stays at 8 MHz
	input cen16,
	input turbo,

	input _reset,
	input selectIWM,
	input _cpuRW,
	input _cpuLDS,	
	input [15:0] dataIn,
	input [3:0] cpuAddrRegHi,
	input SEL, // from VIA
	input driveSel, // internal drive select, 0 - upper, 1 - lower
	output [15:0] dataOut,
	input [1:0] insertDisk,
	output [1:0] diskEject,
	input [1:0] img800k,    // mounted file is 819,200 bytes: see floppy.v's port comment
	input drive800k,        // drive mechanism: see floppy.v's port comment
	input [1:0] mediaSides, // what the medium said at mount: see floppy.v
	input [8:0] disk_pwm,   // spindle duty index 0..399: see floppy.v's tachometer
	
	output [1:0] diskMotor,
	output [1:0] diskAct,
	
	// interface to fetch data for internal drive
	output [21:0] dskReadAddrInt,
	input dskReadAckInt,
	output [21:0] dskReadAddrExt,
	input dskReadAckExt,
	input [7:0] dskReadData,

	// floppy write path
	input [1:0] writeProtect, // {ext,int} - OSD toggle ANDed with img_readonly, per drive

	output [21:0] dskWriteAddrInt,
	output [15:0] dskWriteDataInt,
	output        dskWriteReqInt,
	input         dskWriteAckInt,
	output [21:0] dskWriteAddrExt,
	output [15:0] dskWriteDataExt,
	output        dskWriteReqExt,
	input         dskWriteAckExt,

	// DCD (Apple HD20) block-device slot, passed through to rtl/dcd.v
	output [31:0] dcd_sd_lba,
	output        dcd_sd_rd,
	output        dcd_sd_wr,
	input         dcd_sd_ack,
	input   [7:0] dcd_sd_buff_addr,
	input  [15:0] dcd_sd_buff_dout,
	output [15:0] dcd_sd_buff_din,
	input         dcd_sd_buff_wr,
	input         dcd_img_mounted,
	input  [63:0] dcd_img_size,
	input         dcd_img_readonly,

	// commit notice, per drive - see floppy.v's dskCommit* ports
	output        dskCommitDoneInt,
	output [21:0] dskCommitAddrInt,
	output        dskCommitDoneExt,
	output [21:0] dskCommitAddrExt
);

	wire [7:0] dataInLo = dataIn[7:0];
	reg [7:0] dataOutLo;
	assign dataOut = { 8'hBE, dataOutLo };
	
	// IWM state
	reg ca0, ca1, ca2, lstrb, selectExternalDrive, q6, q7;
	reg ca0Next, ca1Next, ca2Next, lstrbNext, selectExternalDriveNext, q6Next, q7Next;
	wire advanceDriveHead; // prevents overrun when debugging, does not exit on a real Mac!
	reg [7:0] readDataLatch;
	wire _iwmBusy, _writeUnderrun;
	// write buffer empty, muxed from the selected drive
	assign _iwmBusy       = ~(selectExternalDrive ? writeBusyExt : writeBusyInt);
	assign _writeUnderrun = ~(selectExternalDrive ? writeUnderrunExt : writeUnderrunInt);

	// floppy disk drives 
	reg diskEnableExt, diskEnableInt;
	reg diskEnableExtNext, diskEnableIntNext;
	wire newByteReadyInt;
	wire [7:0] readDataInt;
	wire senseInt = readDataInt[7]; // bit 7 doubles as the sense line here
	wire newByteReadyExt;
	wire [7:0] readDataExt;
	// a DCD image is mounted (rtl/dcd.v)
	wire dcdPresent;

	// daisy chain (Apple's flow-through flip-flop): LSTRB with /ENBL2 asserted
	// advances past the DCD to the external floppy; releasing /ENBL2 rewinds
	reg  chainSel;          // 1 = the chain has advanced past the DCD
	reg  lstrbPrevChain;
	always @(posedge clk) if (cep) lstrbPrevChain <= lstrb;
	wire chainAdvance = cep && lstrbPrevChain && ~lstrb;

	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0)
			chainSel <= 1'b0;
		else if (~diskEnableExt)              // /ENBL2 released - rewind
			chainSel <= 1'b0;
		else if (chainAdvance & dcdPresent)   // hand the port to the floppy
			chainSel <= 1'b1;
	end

	// the DCD still holds the external port
	wire dcdOwnsPort = dcdPresent & ~chainSel;

	// write path: the target drive follows selectExternalDriveNext, as q7Next/q6Next
	wire dataRegWrite = (_cpuRW == 1'b0) && selectIWM && (_cpuLDS == 1'b0) &&
	                    ({q7Next, q6Next} == 2'b11) && (diskEnableExt | diskEnableInt);
	wire writeReqInt = cen && dataRegWrite && !selectExternalDriveNext;
	wire writeReqExt = cen && dataRegWrite &&  selectExternalDriveNext;

	// dataRegWrite is a level held for three cen samples; the DCD needs an edge
	reg dataRegWriteSeen;
	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0)
			dataRegWriteSeen <= 1'b0;
		else if (cen)
			dataRegWriteSeen <= dataRegWrite;
	end
	wire writeReqDcd = cen && dataRegWrite && !dataRegWriteSeen &&
	                   selectExternalDriveNext;

	wire writeBusyInt, writeUnderrunInt;
	wire writeBusyExt, writeUnderrunExt;

	floppy floppyInt
	(
		.clk(clk),
		.cep(cep),
		.cen(cen),

		._reset(_reset),
		.ca0(ca0),
		.ca1(ca1),
		.ca2(ca2),
		.SEL(SEL),
		.lstrb(lstrb),
		// a real IWM enables one drive at a time; keeps the chain walk's LSTRB
		// (EJECT in state 7) from the internal drive
		._enable(~(diskEnableInt & driveSel & ~selectExternalDrive)),
		// dataInLo directly: a registered copy would lag writeReqInt by one cycle
		.writeData(dataInLo),
		.readData(readDataInt),
		.advanceDriveHead(advanceDriveHead),
		.newByteReady(newByteReadyInt),
		.insertDisk(insertDisk[0]),
		.img800k(img800k[0]),
		.drive800k(drive800k),
		.mediaSides(mediaSides[0]),
		.disk_pwm(disk_pwm),
		.diskEject(diskEject[0]),

		.motor(diskMotor[0]),
		.act(diskAct[0]),

		.dskReadAddr(dskReadAddrInt),
		.dskReadAck(dskReadAckInt),
		.dskReadData(dskReadData),

		.writeReq(writeReqInt),
		.writeProtect(writeProtect[0]),
		.writeBusy(writeBusyInt),
		.writeUnderrun(writeUnderrunInt),
		.writeMode(q7),
		.dskWriteAddr(dskWriteAddrInt),
		.dskWriteData(dskWriteDataInt),
		.dskWriteReq(dskWriteReqInt),
		.dskWriteAck(dskWriteAckInt),

		.dskCommitDone(dskCommitDoneInt),
		.dskCommitAddr(dskCommitAddrInt)
	);

	floppy floppyExt
	(
		.clk(clk),
		.cep(cep),
		.cen(cen),

		._reset(_reset),
		.ca0(ca0),
		.ca1(ca1),
		.ca2(ca2),
		.SEL(SEL),
		.lstrb(lstrb),
		// disabled while the DCD owns the port
		._enable(~(diskEnableExt & ~dcdOwnsPort)),
		.writeData(dataInLo), // see floppyInt's writeData comment above
		.readData(readDataExt),
		.advanceDriveHead(advanceDriveHead),
		.newByteReady(newByteReadyExt),
		.insertDisk(insertDisk[1]),
		.img800k(img800k[1]),
		.drive800k(drive800k),
		.mediaSides(mediaSides[1]),
		.disk_pwm(disk_pwm),
		.diskEject(diskEject[1]),

		.motor(diskMotor[1]),
		.act(diskAct[1]),

		.dskReadAddr(dskReadAddrExt),
		.dskReadAck(dskReadAckExt),
		.dskReadData(dskReadData),

		.writeReq(writeReqExt),
		.writeProtect(writeProtect[1]),
		.writeBusy(writeBusyExt),
		.writeUnderrun(writeUnderrunExt),
		.writeMode(q7),
		.dskWriteAddr(dskWriteAddrExt),
		.dskWriteData(dskWriteDataExt),
		.dskWriteReq(dskWriteReqExt),
		.dskWriteAck(dskWriteAckExt),

		.dskCommitDone(dskCommitDoneExt),
		.dskCommitAddr(dskCommitAddrExt)
	);

	// DCD (Apple HD20) at the head of the chain; it owns the port until the
	// Mac advances the chain
	wire  [7:0] readDataDcd;
	wire        newByteReadyDcd;

	dcd dcd0
	(
		.clk(clk),
		.cep(cep),
		.cen(cen),
		.turbo(turbo),
		._reset(_reset),
		.ca0(ca0),
		.ca1(ca1),
		.ca2(ca2),
		._enable(~(diskEnableExt & ~chainSel)),
		.writeData(dataIn[7:0]),
		.writeReq(writeReqDcd), // one-shot: see dataRegWriteSeen above
		.readData(readDataDcd),
		.newByteReady(newByteReadyDcd),
		.sd_lba(dcd_sd_lba),
		.sd_rd(dcd_sd_rd),
		.sd_wr(dcd_sd_wr),
		.sd_ack(dcd_sd_ack),
		.sd_buff_addr(dcd_sd_buff_addr),
		.sd_buff_dout(dcd_sd_buff_dout),
		.sd_buff_din(dcd_sd_buff_din),
		.sd_buff_wr(dcd_sd_buff_wr),
		.img_mounted(dcd_img_mounted),
		.img_size(dcd_img_size),
		.img_readonly(dcd_img_readonly),
		.present(dcdPresent)
	);

	wire [7:0] readDataExtSel      = dcdOwnsPort ? readDataDcd     : readDataExt;
	wire       newByteReadyExtSel  = dcdOwnsPort ? newByteReadyDcd : newByteReadyExt;

	// sense comes through the same mux, so the ROM's ID probe sees the DCD
	wire senseExt = readDataExtSel[7];

	wire [7:0] readData = selectExternalDrive ? readDataExtSel : readDataInt;
	wire newByteReady = selectExternalDrive ? newByteReadyExtSel : newByteReadyInt;
	
	// iwmMode is written and read back but no bit of it affects behaviour;
	// latch mode is always L=1
	reg [4:0] iwmMode;
	/* IWM mode register: S C M H L
 	 S	Clock speed:
			0 = 7 MHz
			1 = 8 MHz
		Should always be 1 for Macintosh.
	 C	Bit cell time:
			0 = 4 usec/bit (for 5.25 drives)
			1 = 2 usec/bit (for 3.5 drives) (Macintosh mode)
	 M	Motor-off timer:
			0 = leave drive on for 1 sec after program turns
			    it off
			1 = no delay (Macintosh mode)
		Should be 0 for 5.25 and 1 for 3.5.
	 H	Handshake protocol:
			0 = synchronous (software must supply proper
			    timing for writing data)
			1 = asynchronous (IWM supplies timing) (Macintosh Mode)
		Should be 0 for 5.25 and 1 for 3.5.
	 L	Latch mode:
			0 = read-data stays valid for about 7 usec
			1 = read-data stays valid for full byte time (Macintosh mode)
		Should be 0 for 5.25 and 1 for 3.5.
	*/

	// any read/write access to IWM bit registers will change their values
	always @(*) begin
		ca0Next <= ca0;
		ca1Next <= ca1;
		ca2Next <= ca2;
		lstrbNext <= lstrb;
		diskEnableExtNext <= diskEnableExt;
		diskEnableIntNext <= diskEnableInt;
		selectExternalDriveNext <= selectExternalDrive;
		q6Next <= q6;
		q7Next <= q7;

		if (selectIWM == 1'b1 && _cpuLDS == 1'b0) begin
			case (cpuAddrRegHi[3:1])
				3'h0: // ca0
					ca0Next <= cpuAddrRegHi[0];
				3'h1: // ca1
					ca1Next <= cpuAddrRegHi[0];
				3'h2: // ca2
					ca2Next <= cpuAddrRegHi[0];
				3'h3: // lstrb
					lstrbNext <= cpuAddrRegHi[0];
				3'h4: // disk enable
					if (selectExternalDrive)
						diskEnableExtNext <= cpuAddrRegHi[0];
					else
						diskEnableIntNext <= cpuAddrRegHi[0];
				3'h5: // external drive
					selectExternalDriveNext <= cpuAddrRegHi[0];
				3'h6: // Q6 
					q6Next <= cpuAddrRegHi[0];
				3'h7: // Q7 
					q7Next <= cpuAddrRegHi[0];
			endcase
		end
	end

	// update IWM bit registers
	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0) begin
			ca0 <= 0;
			ca1 <= 0;
			ca2 <= 0;
			lstrb <= 0;
			diskEnableExt <= 0;
			diskEnableInt <= 0;
			selectExternalDrive <= 0;
			q6 <= 0;
			q7 <= 0;
		end
		else begin
			ca0 <= ca0Next;
			ca1 <= ca1Next;
			ca2 <= ca2Next;
			lstrb <= lstrbNext;
			diskEnableExt <= diskEnableExtNext;
			diskEnableInt <= diskEnableIntNext;
			selectExternalDrive <= selectExternalDriveNext;
			q6 <= q6Next;
			q7 <= q7Next;
		end
	end
	
	// read IWM state
	always @(*) begin
		dataOutLo = 8'hEF;
		
		// reading any IWM address returns state as selected by Q7 and Q6
		case ({q7Next,q6Next}) 
			2'b00: // data-in register (from disk drive) - MSB is 1 when data is valid
				dataOutLo <= readDataLatch;
			2'b01: // IWM status register - read only
				dataOutLo <= { (selectExternalDriveNext ? senseExt : senseInt), 1'b0, diskEnableExt & diskEnableInt, iwmMode }; 
			2'b10: // handshake - read only
				dataOutLo <= { _iwmBusy, _writeUnderrun, 6'b000000 };
			2'b11: // IWM mode register when not enabled (write-only), or (write?) data register when enabled
				dataOutLo <= 0;
		endcase
	end

	// write IWM state
	// the write-data register is handled by writeReqInt/Ext and dataInLo above
	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0) begin
			iwmMode <= 0;
		end
		else if(cen) begin
			if (_cpuRW == 0 && selectIWM == 1'b1 && _cpuLDS == 1'b0) begin
				// writing to any IWM address modifies state as selected by Q7 and Q6
				case ({q7Next,q6Next})
					2'b11: begin
						if (~(diskEnableExt | diskEnableInt))
							iwmMode <= dataInLo[4:0];
					end
				endcase
			end
		end
	end

	// Manage incoming bytes from the disk drive
	wire iwmRead = (_cpuRW == 1'b1 && selectIWM == 1'b1 && _cpuLDS == 1'b0);
	reg [3:0] readLatchClearTimer;

	// The latch-clear countdown scales with CPU speed: the .Sony driver polls
	// bit 7 about 2.0 us apart at 8 MHz and 1.0 us at 16 MHz, and a latch that
	// has not cleared reads as a fresh byte. Ticks on cen16 under turbo, cen
	// otherwise; the clear runs on the same enable as the countdown.
	wire latchClearCe = turbo ? cen16 : cen;

	always @(posedge clk or negedge _reset) begin
		if (_reset == 1'b0) begin
			readDataLatch <= 0;
			readLatchClearTimer <= 0;
		end
		else begin
			// a countdown timer governs how long after a data latch read before the latch is cleared
			if (latchClearCe) begin
				if (readLatchClearTimer != 0) begin
					readLatchClearTimer <= readLatchClearTimer - 1'b1;
				end
			end

			// reload after a valid CPU read, on latchClearCe rather than cen so the
			// countdown start is not snapped to the 2-cycle grid at 16 MHz
			if (latchClearCe) begin
				if (iwmRead && readDataLatch[7]) begin
					readLatchClearTimer <= 4'hD; // clear latch 14 clocks after the conclusion of a valid read
				end
			end

			// when the drive indicates that a new byte is ready, latch it
			// NOTE: the real IWM must self-synchronize with the incoming data to determine when to latch it
			if (cen && newByteReady) begin
				readDataLatch <= readData;
			end
			else if (latchClearCe && readLatchClearTimer == 4'd1) begin
				readDataLatch <= 0;
			end
		end
	end
	// inert: floppy.v hardwires readyToAdvanceHead to 1
	assign advanceDriveHead = readLatchClearTimer == 1'b1; // prevents overrun when debugging, does not exist on a real Mac!
endmodule
