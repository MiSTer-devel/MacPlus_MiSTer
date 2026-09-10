/*
 floppy_track_decoder.v

 Decode a raw GCR data field (the same
 stream floppy_track_encoder.v produces for STATE_DHDR onward) back into
 512 bytes of sector payload, verifying it end to end before it is ever
 handed to a caller.

 Consumes one raw disk byte per `ready` pulse (mirrors floppy_track_encoder.v's
 own pacing convention).
 Scans continuously for the D5 AA AD data-field marker (ignoring everything
 else, including the address field's D5 AA 96, whose differing third byte
 never matches), decodes the sector number, then de-nibblizes the whole
 524-byte continuous payload (12-byte Sony tag + 512-byte sector data, one
 unbroken 6:2 group stream per real Apple GCR format - the 12 tag bytes are
 genuine data written by the Mac, NOT a literal sync run, despite this
 core's own read-side encoder faking an all-zero tag as one) back to 524
 bytes while running the C1/C2/C3
 checksum chain in reverse across all of it, discards the first 12
 recovered bytes (the tag - this core has nowhere to expose it) keeping
 only the 512 sector-data bytes, verifies the recovered checksum against
 the trailing 4-byte checksum field, and verifies the DE AA trailer. `sector_valid`
 pulses for exactly one clk if and only if every one of those checks
 passed - never for a partial, corrupt, or truncated field. Any failure
 (bad encoding, checksum mismatch, bad trailer) instead
 pulses `reject` and returns to scanning; a field that simply stops
 arriving (truncation) just leaves the decoder waiting, which is the
 correct behaviour - there is no code path that can assert sector_valid
 without having verified the whole field.

 Address fields (D5 AA 96) are reported, not decoded. A normal sector write
 never contains one - the Mac writes a data field straight behind the
 address field it just read - so one in the write stream means the track is
 being FORMATTED, and floppy_track_encoder.v then needs to know where the
 formatter put its sectors (its header explains why). `amark` pulses once
 per address field with the sector number; the track/side bytes are not
 checked because the drive's own head position decides where the data
 lands, and a formatter writing some other track's number into the field
 would be laying down a broken disk on real media too.

 The same field also carries the format byte, and `fmt_mark`/`fmt_ds`
 report it. It is the one place the medium's own sidedness is ever stated:
 bit 5 set means the track being laid down is two-sided, clear means
 one-sided, and the low five bits are the interleave code, which this
 decoder ignores. Unlike the sector
 number this IS checksum-gated - a mis-synced match that slipped through
 would not cost one sector, it would change the geometry of the whole
 disk - and the checksum byte is the next one along anyway.

 The nibble-recovery arithmetic below is a direct RTL port of the reference
 decoder proved out against this project's
 own RTL encoder dumps: group g's four raw bytes recover group g's own
 three payload bytes directly (payload bytes 3g..3g+2) - there is no
 lookback and no discarded group. (An earlier reading of the reference
 decoder's docstring claimed a one-group lookback with group 0 thrown
 away; that was a misreading of why the original 512-byte-only decode
 happened to come out right - re-derived and empirically confirmed
 against the real, unmodified floppy_track_encoder.v's output during
 the tag fix: decoding its whole 699-byte DZRO+DPRE+DATA region this
 way, with no group discarded, recovers exactly the 12 zero tag bytes
 followed by the real 512-byte sector data, byte-for-byte, using
 precisely the bytes the encoder emits - no invented extra warmup
 group needed.)
 The forward/reverse GCR table below was generated mechanically from
 floppy_track_encoder.v's own table, never hand-transcribed, so it cannot
 silently diverge from what the encoder actually produces.

 The soff/spt geometry math is copied verbatim from floppy_track_encoder.v
 (same track/side/sides inputs) so `addr` lands on exactly the SDRAM byte
 offset the encoder would have read this sector's byte 0 from.
*/

/* verilator lint_off UNUSED */
/* verilator lint_off CASEINCOMPLETE */

module floppy_track_decoder (
   input             clk,
   input             ready,   // one incoming raw disk byte per pulse
   input             rst,

   input             side,
   input             sides,
   input      [6:0]  track,

   input      [7:0]  idata,   // raw byte from the write stream

   // pulses for exactly one clk when a full sector has been decoded and
   // verified - the only time sector/addr/buf_* are meaningful.
   output reg        sector_valid,
   output reg [3:0]  sector,     // decoded sector number, valid with sector_valid
   output reg [21:0] addr,       // SDRAM byte offset of this sector's byte 0

   // pulses for exactly one clk whenever a field is abandoned
   output reg        reject,

   // pulses for exactly one clk when an address field's sector number has
   // gone by in the write stream - only a format writes those; see the
   // header. amark_sector is valid with it.
   output reg        amark,
   output reg [3:0]  amark_sector,

   // pulses for exactly one clk when a whole address field has gone by
   // AND its checksum verified; fmt_ds is that field's format byte bit 5,
   // valid with it. See the header.
   output reg        fmt_mark,
   output reg        fmt_ds,

   // recovered 512-byte payload from the most recently completed sector,
   // registered (1-clk-latency) read port - buf_data reflects buf_addr as
   // it stood one cycle earlier, the standard inferrable-block-RAM idiom
   // (matches floppy_loader.v's buf_rd). An earlier combinational-read
   // version of this port elaborated correctly but could not map to a
   // Cyclone V M10K at all (no combinational read mode), so Quartus built
   // it out of plain registers plus a 512:1 mux - roughly 6,500 ALMs per
   // drive instance, found only once this module was actually synthesized
   // for hardware (it was sim-only before that).
   //
   // The write side had the same problem for a different reason: a group
   // completion could write up to three different buf_mem addresses
   // (buf_mem[buf_idx0]/[buf_idx1]/[buf_idx2]) on the same clock edge, and
   // no Cyclone V memory primitive offers three write ports, so Quartus
   // silently fell back to registers there too (no diagnostic - inference
   // just never triggered). Fixed by latching the up-to-three pending
   // writes when a group completes (state S_GRPC below) and draining them
   // one per cycle, so buf_mem's write side is now a single always-block
   // driving at most one address per edge - the idiom Quartus requires.
   input      [8:0]  buf_addr,
   output reg [7:0]  buf_data
);

   // ------------------------------------------------------------------
   // geometry: soff/spt, copied verbatim from floppy_track_encoder.v
   // ------------------------------------------------------------------
   wire [3:0] spt =
      (track[6:4] == 3'd0)?4'd12:
      (track[6:4] == 3'd1)?4'd11:
      (track[6:4] == 3'd2)?4'd10:
      (track[6:4] == 3'd3)?4'd9:
      4'd8;

   wire [9:0] track_times_12 = { track, 3'b000 } + { 1'b0, track, 2'b00 };
   wire [9:0] track_times_11 = { track, 3'b000 } + { 2'b00, track, 1'b0 } + { 3'b000, track };
   wire [9:0] track_times_10 = { track, 3'b000 } + { 2'b00, track, 1'b0 };
   wire [9:0] track_times_9  = { track, 3'b000 } + { 3'b000, track };
   wire [9:0] track_times_8  = { track, 3'b000 };

   wire [6:0] trackm1 = track - 7'd1;
   wire [9:0] soff =
      (track == 0)?10'd0:
      (trackm1[6:4] == 3'd0)?track_times_12:
      (trackm1[6:4] == 3'd1)?(track_times_11 + 10'd16):
      (trackm1[6:4] == 3'd2)?(track_times_10 + 10'd32 + 10'd16):
      (trackm1[6:4] == 3'd3)?(track_times_9 + 10'd48 + 10'd32 + 10'd16):
      (track_times_8 + 10'd64 + 10'd48 + 10'd32 + 10'd16);

   // Track/side part of the sector address. Deliberately does NOT include
   // the sector term: the whole address is LATCHED in S_SECT (addr_latched
   // below), at the same instant the bounds check passes, and the latched
   // value is what S_DTRL finally publishes ~11ms later.
   //
   // Evaluating this live at S_DTRL instead would separate the check from
   // the thing it protects by a whole field (699 payload bytes at 16us
   // each). track/side/sides are live inputs over that window - driveSide
   // in particular is driven straight off the CA lines with no read strobe
   // (see floppy.v's own "we don't know if this is a true read" comment),
   // so a side flip mid-field could add the `spt*512` term to a field that
   // was validated as side 0 on a single-sided mount, landing the sector
   // up to 6144 bytes away from where it belongs - past the end of a 400K
   // image on the outer tracks. Latching makes validate-and-commit atomic.
   wire [21:0] geom_base =
      { 3'b00, soff, 9'd0 } +
      (sides ? { 3'b00, soff, 9'd0 } : 22'd0) +
      (side  ? { 9'd0, spt, 9'd0 }   : 22'd0);

   // ------------------------------------------------------------------
   // reverse GCR table: disk byte -> 6-bit nibble, or invalid.
   // Generated mechanically from floppy_track_encoder.v's own table - do
   // not hand-edit; regenerate if the encoder's table ever
   // changes.
   // ------------------------------------------------------------------
   function [6:0] rev_lookup; // {valid, nib[5:0]}
      input [7:0] b;
      reg [5:0] nib;
      reg       valid;
      begin
         valid = 1'b1;
         case (b)
         8'h96: nib = 6'h00; 8'h97: nib = 6'h01; 8'h9a: nib = 6'h02; 8'h9b: nib = 6'h03;
         8'h9d: nib = 6'h04; 8'h9e: nib = 6'h05; 8'h9f: nib = 6'h06; 8'ha6: nib = 6'h07;
         8'ha7: nib = 6'h08; 8'hab: nib = 6'h09; 8'hac: nib = 6'h0a; 8'had: nib = 6'h0b;
         8'hae: nib = 6'h0c; 8'haf: nib = 6'h0d; 8'hb2: nib = 6'h0e; 8'hb3: nib = 6'h0f;
         8'hb4: nib = 6'h10; 8'hb5: nib = 6'h11; 8'hb6: nib = 6'h12; 8'hb7: nib = 6'h13;
         8'hb9: nib = 6'h14; 8'hba: nib = 6'h15; 8'hbb: nib = 6'h16; 8'hbc: nib = 6'h17;
         8'hbd: nib = 6'h18; 8'hbe: nib = 6'h19; 8'hbf: nib = 6'h1a; 8'hcb: nib = 6'h1b;
         8'hcd: nib = 6'h1c; 8'hce: nib = 6'h1d; 8'hcf: nib = 6'h1e; 8'hd3: nib = 6'h1f;
         8'hd6: nib = 6'h20; 8'hd7: nib = 6'h21; 8'hd9: nib = 6'h22; 8'hda: nib = 6'h23;
         8'hdb: nib = 6'h24; 8'hdc: nib = 6'h25; 8'hdd: nib = 6'h26; 8'hde: nib = 6'h27;
         8'hdf: nib = 6'h28; 8'he5: nib = 6'h29; 8'he6: nib = 6'h2a; 8'he7: nib = 6'h2b;
         8'he9: nib = 6'h2c; 8'hea: nib = 6'h2d; 8'heb: nib = 6'h2e; 8'hec: nib = 6'h2f;
         8'hed: nib = 6'h30; 8'hee: nib = 6'h31; 8'hef: nib = 6'h32; 8'hf2: nib = 6'h33;
         8'hf3: nib = 6'h34; 8'hf4: nib = 6'h35; 8'hf5: nib = 6'h36; 8'hf6: nib = 6'h37;
         8'hf7: nib = 6'h38; 8'hf9: nib = 6'h39; 8'hfa: nib = 6'h3a; 8'hfb: nib = 6'h3b;
         8'hfc: nib = 6'h3c; 8'hfd: nib = 6'h3d; 8'hfe: nib = 6'h3e; 8'hff: nib = 6'h3f;
         default: begin nib = 6'h00; valid = 1'b0; end
         endcase
         rev_lookup = {valid, nib};
      end
   endfunction

   wire [6:0] rl = rev_lookup(idata);
   wire       nib_valid = rl[6];
   wire [5:0] nib_cur    = rl[5:0];

   // ------------------------------------------------------------------
   // state machine
   // ------------------------------------------------------------------
   localparam S_SCAN = 3'd0;  // hunting for D5 AA AD
   localparam S_SECT = 3'd1;  // 1 byte: sector number
   localparam S_GRPC = 3'd2;  // draining up to 3 pending buf_mem writes
                               // from the group that just completed, one
                               // write per cycle - not gated on `ready`
   localparam S_GRP  = 3'd3;  // 699 bytes across 175 groups (last partial):
                               // recovers the 524-byte tag(12)+data(512)
                               // continuous payload directly, group g ->
                               // payload bytes 3g..3g+2, no lookback
   localparam S_DSUM = 3'd4;  // 4 bytes: checksum
   localparam S_DTRL = 3'd5;  // 2 bytes: DE AA trailer
   localparam S_AMRK = 3'd6;  // the 5 bytes after D5 AA 96: t s h f c -
                               // the sector reported on amark as it goes
                               // by, the format byte on fmt_mark once c
                               // has confirmed the whole field

   reg [2:0]  state;
   reg [23:0] hist;
   reg [2:0]  am_idx;
   reg [5:0]  am_t, am_s, am_h, am_f;
   reg        am_ok;   // every byte of this field so far was valid GCR

   reg [3:0]  sector_reg;
   reg [21:0] addr_latched; // geom_base + sector term, captured in S_SECT

   reg [7:0]  group_index;   // 0..174
   reg [1:0]  byte_in_group; // 0..3
   reg [5:0]  grp_byte0, grp_byte1, grp_byte2;

   reg [7:0]  c1, c2, c3;
   reg [9:0]  recovered_count;

   reg [1:0]  dsum_idx;
   reg [5:0]  dsum0, dsum1, dsum2;

   reg        dtrl_idx;

   // pending buf_mem writes latched at group completion, drained one per
   // cycle in S_GRPC - see the buf_mem port comment above.
   reg [1:0]  commit_step;             // 0,1,2: which of the 3 slots is next
   reg        commit_v0, commit_v1, commit_v2;
   reg [8:0]  commit_a0, commit_a1, commit_a2;
   reg [7:0]  commit_d0, commit_d1, commit_d2;
   reg [2:0]  commit_target;           // state to resume in once drained

   reg [7:0]  buf_mem [0:511];
   always @(posedge clk) buf_data <= buf_mem[buf_addr];

   // group-completion combinational helpers (S_GRP only)
   wire has_s3    = (byte_in_group == 2'd3);
   wire grp_done  = has_s3 || (group_index == 8'd174 && byte_in_group == 2'd2);
   wire [5:0] gs2 = has_s3 ? grp_byte2 : nib_cur; // s2: stored, or live on the partial group's completing byte

   wire [1:0] top0 = grp_byte0[5:4];
   wire [1:0] top1 = grp_byte0[3:2];
   wire [1:0] top2_= grp_byte0[1:0];

   wire [7:0] nib_xor_0 = {top0, grp_byte1};
   wire [7:0] rol_c1    = {c1[6:0], c1[7]};
   wire [7:0] nib_in_1  = nib_xor_0 ^ rol_c1;
   wire [8:0] c3_sum    = {1'b0, c3} + {1'b0, nib_in_1} + {8'd0, c1[7]};
   wire       new_c3x   = c3_sum[8];
   wire [7:0] new_c3    = c3_sum[7:0];

   wire [7:0] nib_xor_1 = {top1, gs2};
   wire [7:0] nib_in_2  = nib_xor_1 ^ new_c3;
   wire [8:0] c2_sum    = {1'b0, c2} + {1'b0, nib_in_2} + {8'd0, new_c3x};
   wire       new_c2x   = c2_sum[8];
   wire [7:0] new_c2    = c2_sum[7:0];

   wire [7:0] nib_xor_2 = {top2_, nib_cur}; // only meaningful when has_s3
   wire [7:0] nib_in_3  = nib_xor_2 ^ new_c2;
   wire [7:0] new_c1_full = rol_c1 + nib_in_3 + {7'd0, new_c2x};

   // buf_mem write indices: recovered_count runs 0..523 across the whole
   // tag+data payload (needs the full 10 bits - it exceeds 511), so the
   // "-12" (drop the tag) has to happen before truncating to the 9-bit
   // buf_mem address, not after.
   wire [9:0] buf_idx0 = recovered_count - 10'd12;
   wire [9:0] buf_idx1 = recovered_count + 10'd1 - 10'd12;
   wire [9:0] buf_idx2 = recovered_count + 10'd2 - 10'd12;

   wire [5:0] want_dsum_top = {c3[7:6], c2[7:6], c1[7:6]};

   task do_reject;
      begin
         reject <= 1'b1;
         state  <= S_SCAN;
         hist   <= 24'd0;
      end
   endtask

   // Synchronous reset, matching floppy_write_committer.v (the other
   // consumer of floppy.v's identical `!_reset || writePathReset` wire, and
   // synchronous there too). writePathReset is a DERIVED
   // combinational pulse, not a true reset rail, so it must not sit on an
   // asynchronous reset pin: a glitch of any width on that AND-tree would
   // abandon the field in progress, and since the write path reports no
   // error for an abandoned field, the Mac would believe a sector it wrote
   // had landed when it never did. clk is free-running, so nothing is lost
   // by waiting for the edge.
   always @(posedge clk) begin
      if (rst) begin
         state        <= S_SCAN;
         hist         <= 24'd0;
         sector_valid <= 1'b0;
         reject       <= 1'b0;
         sector       <= 4'd0;
         addr         <= 22'd0;
         amark        <= 1'b0;
         amark_sector <= 4'd0;
         fmt_mark     <= 1'b0;
         fmt_ds       <= 1'b0;
      end else begin
         sector_valid <= 1'b0;
         reject       <= 1'b0;
         amark        <= 1'b0;
         fmt_mark     <= 1'b0;

         if (state == S_GRPC) begin
            // Drain the up-to-3 pending buf_mem writes latched when the
            // group completed, one write per cycle, never more - so
            // buf_mem has a single write port and can map to a Cyclone V
            // M10K. Not gated on `ready`: `ready` pulses roughly once per
            // 128 clk8 cycles (one incoming disk byte / 16us, see the
            // module header comment) and this drain is a fixed 3 cycles,
            // so no incoming byte can arrive mid-drain.
            case (commit_step)
            2'd0: begin
               if (commit_v0) buf_mem[commit_a0] <= commit_d0;
               commit_step <= 2'd1;
            end
            2'd1: begin
               if (commit_v1) buf_mem[commit_a1] <= commit_d1;
               commit_step <= 2'd2;
            end
            default: begin
               if (commit_v2) buf_mem[commit_a2] <= commit_d2;
               state <= commit_target;
               if (commit_target == S_DSUM)
                  dsum_idx <= 2'd0;
            end
            endcase
         end else if (ready) begin
            case (state)

            S_SCAN: begin
               hist <= {hist[15:0], idata};
               if ({hist[15:0], idata} == 24'hD5AAAD)
                  state <= S_SECT;
               else if ({hist[15:0], idata} == 24'hD5AA96) begin
                  state  <= S_AMRK;
                  am_idx <= 3'd0;
                  am_ok  <= 1'b1;
               end
            end

            S_AMRK: begin
               // D5 AA 96 t s h f c, walked to the end.
               //
               // `s` is still reported the instant it goes by, exactly as
               // before: floppy_track_encoder.v's relay measures the head's
               // position from that byte and its "five bytes behind the
               // D5" arithmetic is calibrated to it. A sector this track
               // cannot hold (or a byte that is not GCR at all) is not
               // reported: nothing could be laid out for it, and a
               // formatter writing such a field is not one whose track
               // needs relaying. D5 and AA are not data nibbles, so the
               // scan cannot have matched inside a field.
               //
               // The walk then continues through h, f and c for the format
               // byte (see the header). Those three bytes are the field's
               // own - the trailer is DE AA, and D5 AA AD cannot start
               // before it - so nothing that could begin a data field is
               // skipped by staying here for them. A field that stops
               // arriving part-way leaves this waiting, the same as a
               // truncated data field does, and reports nothing.
               if (!nib_valid) am_ok <= 1'b0;

               case (am_idx)
               3'd0: am_t <= nib_cur;
               3'd1: begin
                  am_s <= nib_cur;
                  if (nib_valid && nib_cur < {2'b00, spt}) begin
                     amark        <= 1'b1;
                     amark_sector <= nib_cur[3:0];
                  end
               end
               3'd2: am_h <= nib_cur;
               3'd3: am_f <= nib_cur;
               default: begin
                  // c: the field's own checksum over t s h f. Only a field
                  // that survives it may speak for the medium's geometry.
                  if (am_ok && nib_valid &&
                      (am_t ^ am_s ^ am_h ^ am_f) == nib_cur) begin
                     fmt_mark <= 1'b1;
                     fmt_ds   <= am_f[5];
                  end
                  state <= S_SCAN;
                  hist  <= 24'd0;
               end
               endcase

               if (am_idx != 3'd4) am_idx <= am_idx + 3'd1;
            end

            S_SECT: begin
               // Reject before committing to a sector number the checksum
               // chain never covers (it starts at zero right after this
               // byte - see the module header). Two failure modes close
               // here: a mis-synced field aliasing to a sector index past
               // this track's real spt (nib_cur is the full 6-bit nibble,
               // not the truncated 4 bits sector_reg keeps - an alias a
               // 4-bit compare would miss), and a side-1 field landing on
               // a single-sided (sides==0) mount, where geom_base above
               // would otherwise still add the side offset and write
               // outside the image.
               if (!nib_valid) do_reject;
               else if (nib_cur >= spt || (side && !sides)) do_reject;
               else begin
                  sector_reg      <= nib_cur[3:0];
                  // Capture the full address NOW, while the geometry that
                  // was just bounds-checked is still the live geometry.
                  // Uses nib_cur directly, not sector_reg - that register
                  // is being assigned on this same edge.
                  addr_latched    <= geom_base + { 9'd0, nib_cur[3:0], 9'd0 };
                  state           <= S_GRP;
                  group_index     <= 8'd0;
                  byte_in_group   <= 2'd0;
                  c1 <= 8'd0; c2 <= 8'd0; c3 <= 8'd0;
                  recovered_count <= 10'd0;
               end
            end

            S_GRP: begin
               if (!nib_valid) do_reject;
               else begin
                  if (byte_in_group < 2'd3) begin
                     case (byte_in_group)
                     2'd0: grp_byte0 <= nib_cur;
                     2'd1: grp_byte1 <= nib_cur;
                     2'd2: grp_byte2 <= nib_cur;
                     endcase
                  end

                  if (grp_done) begin
                     // groups 0..174 recover the full 524-byte tag+data
                     // payload directly (group g -> payload bytes 3g..3g+2,
                     // no lookback/discard - see header comment) -
                     // bytes 0..11 are the Sony tag, which this core has
                     // nowhere to expose, so only bytes 12..523
                     // (recovered_count-12, i.e. buf index 0..511) are
                     // actually committed to buf_mem. Latch the (up to 3)
                     // pending writes and hand off to S_GRPC to drain them
                     // one per cycle instead of writing buf_mem directly
                     // here - see the buf_mem port comment above.
                     commit_v0 <= (recovered_count >= 10'd12);
                     commit_v1 <= (recovered_count + 10'd1 >= 10'd12);
                     commit_v2 <= has_s3 && (recovered_count + 10'd2 >= 10'd12);
                     commit_a0 <= buf_idx0[8:0]; commit_d0 <= nib_in_1;
                     commit_a1 <= buf_idx1[8:0]; commit_d1 <= nib_in_2;
                     commit_a2 <= buf_idx2[8:0]; commit_d2 <= nib_in_3;
                     commit_step   <= 2'd0;
                     commit_target <= (group_index == 8'd174) ? S_DSUM : S_GRP;
                     state         <= S_GRPC;

                     if (has_s3) begin
                        recovered_count <= recovered_count + 10'd3;
                        c1 <= new_c1_full;
                     end else begin
                        recovered_count <= recovered_count + 10'd2;
                        c1 <= rol_c1; // last (partial) group: no third byte to add
                     end
                     c2 <= new_c2;
                     c3 <= new_c3;

                     byte_in_group <= 2'd0;
                     if (group_index != 8'd174)
                        group_index <= group_index + 8'd1;
                  end else
                     byte_in_group <= byte_in_group + 2'd1;
               end
            end

            S_DSUM: begin
               if (!nib_valid) do_reject;
               else if (dsum_idx == 2'd3) begin
                  if (dsum0 == want_dsum_top && dsum1 == c3[5:0] &&
                      dsum2 == c2[5:0] && nib_cur == c1[5:0]) begin
                     state    <= S_DTRL;
                     dtrl_idx <= 1'b0;
                  end else
                     do_reject;
               end else begin
                  case (dsum_idx)
                  2'd0: dsum0 <= nib_cur;
                  2'd1: dsum1 <= nib_cur;
                  2'd2: dsum2 <= nib_cur;
                  endcase
                  dsum_idx <= dsum_idx + 2'd1;
               end
            end

            S_DTRL: begin
               if (dtrl_idx == 1'b0) begin
                  if (idata != 8'hDE) do_reject;
                  else dtrl_idx <= 1'b1;
               end else begin
                  if (idata != 8'hAA) do_reject;
                  else begin
                     sector       <= sector_reg;
                     addr         <= addr_latched; // captured in S_SECT, see geom_base
                     sector_valid <= 1'b1;
                     state        <= S_SCAN;
                     hist         <= 24'd0;
                  end
               end
            end

            endcase
         end
      end
   end

endmodule
