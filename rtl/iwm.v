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
	// clk16_en_n + the turbo flag, used ONLY to scale the read-data latch
	// clear interval with CPU speed - see readLatchClearTimer below. Every
	// other IWM timing stays on cep/cen at 8 MHz, including the 16 us disk
	// byte rate, which is a property of the drive and must not scale.
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

	// DCD (Apple HD20) on the external drive port. One hps_io block-device
	// slot, passed straight through to rtl/dcd.v.
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

	// SD persistence tap, per drive - see floppy.v's dskCommit* ports
	output        dskCommitDoneInt,
	output [21:0] dskCommitAddrInt,
	output        dskCommitBufWrInt,
	output [7:0]  dskCommitBufAddrInt,
	output [15:0] dskCommitBufDataInt,
	output        dskCommitDoneExt,
	output [21:0] dskCommitAddrExt,
	output        dskCommitBufWrExt,
	output [7:0]  dskCommitBufAddrExt,
	output [15:0] dskCommitBufDataExt
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
	// for writes, a value of 1 here indicates the IWM write buffer is empty -
	// muxed from whichever drive is currently selected, mirroring readData/
	// newByteReady below. See floppy.v's writeBusy/writeUnderrun comment.
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
	// senseExt is muxed with the DCD's line below, next to the readData mux it
	// belongs with - see readDataExtSel.
	// A DCD image is mounted (rtl/dcd.v's `present`). Declared here because
	// floppyExt's enable below uses it; the DCD is instantiated further down.
	wire dcdPresent;

	// ------------------------------------------------------------------
	// Daisy chain: the flow-through flip-flop
	// ------------------------------------------------------------------
	// A real HD 20 has a floppy-out connector on its back panel, and the Mac
	// walks the chain rather than owning one device per port. Apple's DCD
	// specification (May 1985, Figure 2 "Flowthrough Schematic") describes the
	// circuit and the Plus ROM implements it at $418934; a 512K gets the same
	// routine from Apple's HD 20 INIT, as PTCH id=2 (`.Sony`).
	//
	// An advance is an LSTRB pulse while /ENBL2 stays asserted, and releasing
	// /ENBL2 rewinds to the head. Only one hop is modelled - the DCD at
	// position 0 with the external floppy behind it - which is the only
	// ordering Apple allowed, since a floppy cannot pass the chain on.
	//
	// lstrb is also the Sony register-write strobe, so this cannot be a toggle
	// count. Ownership is the discriminator instead, as in Apple's flip-flop:
	// while the DCD owns the port a pulse means hand over, and once the floppy
	// owns it the DCD is disabled and the same pulse is an ordinary LSTRB that
	// floppy.v acts on itself.
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

	// The DCD still holds the external port. With no image mounted this is 0
	// and every expression below collapses to what shipped before the chain.
	wire dcdOwnsPort = dcdPresent & ~chainSel;

	// write path: which drive's data register a CPU write targets follows
	// selectExternalDriveNext, mirroring q7Next/q6Next's use below for the
	// same in-flight access (see the "write IWM state" block further down).
	wire dataRegWrite = (_cpuRW == 1'b0) && selectIWM && (_cpuLDS == 1'b0) &&
	                    ({q7Next, q6Next} == 2'b11) && (diskEnableExt | diskEnableInt);
	wire writeReqInt = cen && dataRegWrite && !selectExternalDriveNext;
	wire writeReqExt = cen && dataRegWrite &&  selectExternalDriveNext;

	// The DCD needs an edge, not a level. dataRegWrite is a level on _cpuLDS,
	// which fx68k holds for three CPU clock periods and so three cen samples.
	// floppy.v never noticed because it refuses a writeReq while its 16 us
	// write-busy is set; dcd_link.v has no such interlock and would take each
	// pulse as a new byte, so $AA $81 $B1 arrives as $AA $AA $AA $81 $81 $81.
	//
	// An edge is safe because consecutive data-register writes are always
	// separated by the handshake read at $1800 ($419AE8), which drops
	// dataRegWrite in between. The floppy's writeReqExt above is left alone.
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
		// The ~selectExternalDrive term is essential. A real IWM has one disk-enable
		// register bit ($1000/$1200) steered by SELECT ($1400/$1600) to /ENBL1
		// or /ENBL2, so exactly one drive is ever enabled; here the two ports
		// are independent latches, and the ROM's chain search asserts the
		// external enable at $418984 without clearing the internal one. The
		// walk's LSTRB in state 7 with SEL=0 is EJECT, so a still-enabled
		// internal drive loses its disk. This term makes the drive deaf while
		// the external port is driven, as /ENBL1 is, and with no external
		// activity it reduces to what shipped.
		._enable(~(diskEnableInt & driveSel & ~selectExternalDrive)),
		// dataInLo directly, not a registered copy: writeReqInt pulses the
		// same cycle a register load from dataInLo would be scheduled, and
		// nonblocking assignments only see pre-edge values, so a register
		// read here would lag the strobe by one cycle.
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
		.dskCommitAddr(dskCommitAddrInt),
		.dskCommitBufWr(dskCommitBufWrInt),
		.dskCommitBufAddr(dskCommitBufAddrInt),
		.dskCommitBufData(dskCommitBufDataInt)
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
		// Held disabled while the DCD is the selected link in the chain. The
		// DCD takes over the readData/newByteReady/sense mux below, but
		// writeReqExt and the LSTRB strobes would still reach this floppy -
		// writing the DCD's command bytes onto its track 0, and ejecting it on
		// the chain walk. floppy.v ignores all three while _enable is high.
		// Once the chain advances this becomes the live drive; with no DCD
		// mounted it reduces to ~diskEnableExt.
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
		.dskCommitAddr(dskCommitAddrExt),
		.dskCommitBufWr(dskCommitBufWrExt),
		.dskCommitBufAddr(dskCommitBufAddrExt),
		.dskCommitBufData(dskCommitBufDataExt)
	);

	// ------------------------------------------------------------------
	// DCD (Apple HD20)
	// ------------------------------------------------------------------
	// A DCD device is a peer of floppy.v on this same byte interface: the
	// corrected DB-19 pinout puts it on the ordinary RD/WR pins with /ENBL2 as
	// its enable, so it hangs off the external drive port and only PH0-PH2 are
	// repurposed, from a drive-register address into a handshake state bus.
	//
	// It sits at the head of the chain, with the external floppy behind it -
	// see the flow-through flip-flop above. Until the Mac advances the chain
	// the DCD owns the port outright, enforced at the floppy's enable as well
	// as at the read mux below. With no DCD image mounted the external port is
	// bit-identical to what it has always been.
	//
	// The DCD's own enable falls away on the hand-over, leaving it in the
	// "phantom" state Apple's spec requires of a deselected link: dcd_link.v's
	// `selected` goes low and its sense line reads 1 in every ID state.
	// _iwmBusy on the external branch is still the floppy's writeBusyExt, which
	// floppy.v holds at 0 while disabled - "ready", which is what a DCD wants.
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

	// The sense line has to come through the same mux as the data. The status
	// register (Q7=0, Q6=1) is where both programs that look for a DCD look:
	// the ROM's ID probe at $418600 and HD Diag's `tst.b $1c00(a2)` at $D938.
	// With the floppy answering, state 7 returns its INSTALLED register - 0 -
	// and the probe fails at $418634's `bpl`. Taking bit 7 of readDataExtSel
	// rather than a second copy of the mux keeps the two in step, and with
	// nothing mounted it reduces to readDataExt[7] exactly.
	wire senseExt = readDataExtSel[7];

	wire [7:0] readData = selectExternalDrive ? readDataExtSel : readDataInt;
	wire newByteReady = selectExternalDrive ? newByteReadyExtSel : newByteReadyInt;
	
	// NOTE: iwmMode is DEAD - it is written below and read back in the status
	// register, but no bit of it affects behaviour anywhere. In particular
	// its L (latch mode) bit is the real IWM control that governs the
	// read-data latch hold time implemented by readLatchClearTimer further
	// down; we always behave as L=1 (Macintosh mode) regardless of what the
	// driver writes here. Do not assume this register is honoured.
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
	// Note: the write-data-register case (diskEnableExt|diskEnableInt) is
	// handled directly by writeReqInt/Ext + dataInLo above (floppy.v does
	// its own byte capture), not by a register here - see the writeData
	// port comment on the floppy instances above for why.
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

	// The latch-clear countdown must scale with CPU speed, or 16 MHz cannot
	// read disks at all. The .Sony driver detects a new byte only by polling
	// bit 7, and EVERY GCR disk byte has bit 7 set, so a latch that has not
	// self-cleared yet is indistinguishable from a fresh byte. Its poll loops
	// are unrolled double reads ~16 CPU cycles apart (boot1.rom @ 0x03552e):
	// 2.0 us at 8 MHz, but only 1.0 us at 16 MHz. Ticking this timer on cen
	// (125 ns) either way puts the clear at a fixed 1.5 us wall-clock, so
	// 8 MHz clears in time with ~33% margin while 16 MHz never does - the
	// driver ingests duplicate bytes and every checksum fails.
	//
	// Ticking on cen16 (62.5 ns) when turbo restores the same ~33% margin
	// (12 ticks = 0.75 us vs a 1.0 us gap). At 8 MHz cen16Ce reduces to cen
	// exactly, so that path stays bit-identical to the hardware-proven
	// behaviour. Only the clear interval moves; the byte rate does not.
	//
	// The clear itself has to run on the SAME enable as the countdown. cen16
	// is a superset of cen (busPhase[0] vs busPhase==01), so a timer ticking
	// at cen16 can pass through 1 on a phase-11 tick that cen never sees -
	// gating the clear on cen alone would let the terminal count slip past
	// unnoticed and the latch would never clear at all.
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

			// the conclusion of a valid CPU read from the IWM will start the timer to clear the latch.
			// Ordered after the decrement so a reload still wins when both fire on the same edge,
			// exactly as it did when both lived in one cen-gated block.
			//
			// The RELOAD must run on latchClearCe too, not cen. cen is one tick
			// per CPU cycle at 8 MHz but only one per TWO CPU cycles at 16 MHz,
			// so gating the reload on it snaps the start of the countdown to a
			// 2-cycle grid while the countdown itself ticks every cycle - the
			// hold then comes out 1 cycle longer or shorter depending on where
			// the access landed in busPhase. That is not harmless jitter: a poll
			// that reads a stale latch re-arms this very timer, so losing the
			// race once pins the latch high for as long as the driver keeps
			// polling. cpu_en_p/n derive from the same busPhase counter, so the
			// alignment never drifts - it is fixed by the wait-stated accesses
			// preceding the poll loop and then stays put, which is why the
			// failure looks random between runs but sticks within one.
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
	// Inert: floppy.v hardwires readyToAdvanceHead to 1 ("TEMP: treat IWM as
	// always ready"), so nothing consumes this. Noted because it is derived
	// from the now-speed-scaled timer and would otherwise look load-bearing.
	assign advanceDriveHead = readLatchClearTimer == 1'b1; // prevents overrun when debugging, does not exist on a real Mac!
endmodule
