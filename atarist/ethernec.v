// ethernec.v
//
// Atari ST NE2000/ethernec implementation for the MiST board
// https://github.com/mist-devel/mist-board
//
// Copyright (c) 2014 Till Harbaum <till@harbaum.org>
// Copyright (c) 2026 Eugene Azarov <rusjaz@gmail.com>
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

`define SHIFT_REG(reg_name, depth, in_signal) \
	reg [depth-1:0] reg_name = { depth{1'b0} }; \
	always @(posedge clk) begin \
		reg_name <= { reg_name[depth-2:0], in_signal }; \
	end

`define DELAY_REG(reg_name, in_signal) \
	reg reg_name = 1'b0; \
	always @(posedge clk) begin \
		reg_name <= in_signal; \
	end

module ethernec (
	// cpu register interface
	input            clk,
	input            rst,
	input            rd,
	input            wr,
	input      [4:0] addr,
	input      [7:0] din,
	output reg [7:0] dout,

	// ethernet status word to be read by io controller
	output    [31:0] status,

	// interface to allow the io controller to read frames from the tx buffer
	input            tx_begin,   // rising edge before new tx byte stream is sent
	input            tx_strobe,  // rising edge before each tx byte
	output reg [7:0] tx_byte,    // byte from transmit buffer 

	// interface to allow the io controller to write frames to the tx buffer
	input            rx_begin,   // rising edge before new rx byte stream is sent
	input            rx_strobe,  // rising edge before each rx byte
	input      [7:0] rx_byte,    // byte to be written to rx buffer 

	// interface to allow mac address being set by io controller
	input            mac_begin,  // rising edge before new mac is sent
	input            mac_strobe, // rising edge before each mac byte
	input      [7:0] mac_byte,   // mac address byte

	output           int_n       // nic interrupt
);

wire tx_ready = (start & ~stop & txp);
wire rx_ready = (start & ~stop & ~rx_busy);

// tx_ready[17], rx_ready[16], tx_count[15:0]
assign status = { 8'h00, 6'h00, tx_ready, rx_ready, 5'h00, tbcr };

// ---------- ne2000 internal registers -------------
reg [7:0]  cr;             // command register
reg [7:0]  isr;            // interrupt service register
reg [7:0]  imr;            // interrupt mask register
reg [7:0]  curr;           // current page register
reg [7:0]  clda;           // current local dma page register
reg [10:0] crda;           // current remote dma address register
reg [7:0]  bnry;           // boundary page
reg [7:0]  pstart;         // rx buffer ring start page
reg [7:0]  pstop;          // rx buffer ring stop page
reg [15:0] rbcr;           // receiver byte count register
reg [15:0] rsar;           // receiver address register
reg [10:0] tbcr;           // transmitter byte count register

wire stop  = cr[0];        // stop mode
wire start = cr[1];        // nic started
wire txp   = cr[2];        // transmit packet toggle
wire [1:0] ps = cr[7:6];   // register page select
reg rx_busy;               // previous frame is locked in buffer

wire dma_port = (addr[4:3] == 2'b10); // remote DMA ports ($10 - $17)
wire rst_port = (addr[4:3] == 2'b11); // reset ports ($18 - $1F)

// ----------------- rx/tx buffers ------------------
localparam BUF_SIZE = 2048;

reg  [7:0] rx_buffer[BUF_SIZE-1:0];   // 1 ethernet frame 4 bytes offset
reg [10:0] rx_w_cnt;                  // receive buffer byte counter

reg  [7:0] tx_buffer[BUF_SIZE-1:0];   // 1 ethernet frame
reg [10:0] tx_r_cnt;                  // transmit buffer byte counter

// ---------- io controller signals resync -----------
`DELAY_REG(rd_d, rd)
`DELAY_REG(wr_d, wr)

wire rd_ne = ~rd & rd_d; // 0xFBxxxx
wire wr_ne = ~wr & wr_d; // 0xFAxxxx

`SHIFT_REG(tx_begin_sr, 4, tx_begin)
`SHIFT_REG(tx_strobe_sr, 3, tx_strobe)

wire tx_start =  tx_begin_sr[0] & ~tx_begin_sr[1];
wire tx_stop  = ~tx_begin_sr[2] &  tx_begin_sr[3];
wire tx_strobe_pe = tx_strobe_sr[1] & ~tx_strobe_sr[2];

`SHIFT_REG(rx_begin_sr, 4, rx_begin)
`SHIFT_REG(rx_strobe_sr, 3, rx_strobe)

wire rx_start =  rx_begin_sr[0] & ~rx_begin_sr[1];
wire rx_stop  = ~rx_begin_sr[2] &  rx_begin_sr[3];
wire rx_strobe_pe = rx_strobe_sr[1] & ~rx_strobe_sr[2];

`SHIFT_REG(mac_begin_sr, 2, mac_begin)
`SHIFT_REG(mac_strobe_sr, 3, mac_strobe)

wire mac_start = mac_begin_sr[0] & ~mac_begin_sr[1];
wire mac_strobe_pe = mac_strobe_sr[1] & ~mac_strobe_sr[2];

`DELAY_REG(rx_stop_d, rx_stop)
`DELAY_REG(txp_d, txp)

wire txp_pe = txp & ~txp_d;

// -------------------- reset -----------------------
reg reset = 1'b0;
reg reset_d = 1'b0;

wire reset_pe = rst | (reset & ~reset_d);

always @(posedge clk) begin
	reset_d <= reset;
	if (rst) begin
		reset <= 1'b0;
	end else if (rd_ne) begin
		if (rst_port) reset <= 1'b1;
	end else if (wr_ne) begin
		if (rst_port) reset <= 1'b0;
	end
end

// ------------- set local mac address --------------
reg [7:0] mac [5:0];
reg [2:0] mac_cnt;

// mac address from io controller
always @(posedge clk) begin
	if (mac_start)
		mac_cnt <= 0;
	else if (mac_strobe_pe) begin
		if (mac_cnt < 6) begin
			mac[mac_cnt] <= mac_byte;
			mac_cnt <= mac_cnt + 3'd1;
		end
	end
end

// ------ netusbee: 93c46 eeprom mac-read stub ------
reg [7:0]  ee_cr;
reg [3:0]  ee_bit_cnt;
reg [15:0] ee_shifter;
reg        ee_sclk_d;

wire ee_reg_wr = wr_ne && (ps == 3) && (addr == 1);
wire ee_reset  = ee_reg_wr ? !din[3] : !ee_cr[3];

wire ee_sclk_pe =  ee_cr[2] & ~ee_sclk_d;
wire ee_sclk_ne = ~ee_cr[2] &  ee_sclk_d;

always @(posedge clk) begin
	if (reset_pe) ee_sclk_d <= 1'b0;
	else          ee_sclk_d <= ee_cr[2];
end

always @(posedge clk) begin
	if (reset_pe) begin
		ee_cr      <= 0;
		ee_bit_cnt <= 0;
		ee_shifter <= 0;
	end else begin

		if (ee_reg_wr)
			ee_cr <= din;

		if (ee_reset) begin
			ee_bit_cnt <= 0;
			ee_shifter <= 0;
		end else begin

			if (ee_sclk_pe) begin
				if (ee_bit_cnt < 15)
					ee_bit_cnt <= ee_bit_cnt + 4'd1;

				if (ee_bit_cnt < 10)
					ee_shifter <= { ee_shifter[14:0], ee_cr[1] };
			end

			if (ee_sclk_ne) begin
				if (ee_bit_cnt == 10) begin
					case (ee_shifter[2:0])
						3'b010: ee_shifter <= { mac[1], mac[0] };
						3'b011: ee_shifter <= { mac[3], mac[2] };
						3'b100: ee_shifter <= { mac[5], mac[4] };
						default: ee_shifter <= 0;
					endcase

				end else if (ee_bit_cnt > 10) begin
					ee_shifter <= { ee_shifter[14:0], 1'b0 };
				end
			end
		end
	end
end

wire eeprom_do = (ee_bit_cnt >= 10) ? ee_shifter[15] : 1'b0;

reg [7:0] dma_do;
reg [7:0] dma_do_d;
reg [7:0] rx_buffer_do;
reg [7:0] tx_buffer_do;
reg [7:0] prev; // frame start page

wire is_prom  = (rsar[15:8] == 8'h0);
wire is_frame = (rsar[15:8] == prev);

// 1 cycle delay to align access time with bram
always @(posedge clk) begin
	if (is_prom) begin
		case (crda[3:0])
			4'h0: dma_do_d <= mac[0];
			4'h1: dma_do_d <= mac[1];
			4'h2: dma_do_d <= mac[2];
			4'h3: dma_do_d <= mac[3];
			4'h4: dma_do_d <= mac[4];
			4'h5: dma_do_d <= mac[5];
			4'hE: dma_do_d <= 8'h57;
			4'hF: dma_do_d <= 8'h57;
			default: dma_do_d <= 8'h00;
		endcase
	end else begin
		case (crda[1:0])
			2'd0: dma_do_d <= 8'h1;
			2'd1: dma_do_d <= clda;
			2'd2: dma_do_d <= rx_w_cnt[7:0];
			2'd3: dma_do_d <= { 5'b00000, rx_w_cnt[10:8] };
		endcase
	end
end

// remote dma read
always @(*) begin
	if (is_prom) begin
		// prom data
		dma_do = dma_do_d;
	end else if (is_frame) begin
		if (crda < 4) begin
			// virtual frame header
			dma_do = dma_do_d;
		end else begin
			// frame data
			dma_do = rx_buffer_do;
		end
	end else begin
		// for memory test
		dma_do = tx_buffer_do;
	end
end

// cpu read
always @(*) begin
	dout = 8'h00;
	if (rd) begin
		if (dma_port) begin
			dout = dma_do;
		end else begin
			// registers
			case (ps)
				2'b00: begin
					// page 0
					case (addr)
						5'h00: dout = cr;
						5'h01: dout = rx_w_cnt[7:0];
						5'h02: dout = clda;
						5'h03: dout = bnry;
						5'h04: dout = 8'h01; // tsr: tx ok
						5'h07: dout = isr;
						5'h08: dout = crda[7:0];
						5'h09: dout = (rsar[15:8] + { 5'h00, crda[10:8] });
						5'h0a: dout = rbcr[7:0];
						5'h0b: dout = rbcr[15:8];
						5'h0c: dout = 8'h01; // rsr: rx ok
						5'h0e: dout = 8'h28; // dcfg: 8-bit, fifo=2
						default: dout = 8'h00;
					endcase
				end
				2'b01: begin
					// page 1
					case (addr)
						5'h00: dout = cr;
						5'h01: dout = mac[0];
						5'h02: dout = mac[1];
						5'h03: dout = mac[2];
						5'h04: dout = mac[3];
						5'h05: dout = mac[4];
						5'h06: dout = mac[5];
						5'h07: dout = curr;
						default: dout = 8'h00;
					endcase
				end
				2'b10: begin
					// page 2
					case (addr)
						5'h00: dout = cr;
						default: dout = 8'h00;
					endcase
				end
				2'b11: begin
					// page 3
					case (addr)
						5'h00: dout = cr;
						5'h01: dout = { ee_cr[7:1], eeprom_do };
						5'h03: dout = 8'h18; // config0: rtl8019as, PnP
						default: dout = 8'h00;
					endcase
				end
			endcase
		end
	end
end

wire [10:0] next_rx_w_cnt = rx_w_cnt + 11'd1;
wire  [7:0] next_curr = ((curr + 8'd1) == pstop) ? pstart : (curr + 8'd1);
wire  [7:0] next_clda = ((clda + 8'd1) == pstop) ? pstart : (clda + 8'd1);

// local DMA bytes/pages counter
always @(posedge clk) begin
	if (rx_start) begin
		// reserve page for virtual header
		rx_w_cnt <= 11'd4;
		clda <= next_curr;
	end else if (rx_strobe_pe) begin
		rx_w_cnt <= next_rx_w_cnt;
		// count full pages
		if (next_rx_w_cnt[7:0] == 8'h00) begin
			clda <= next_clda;
		end
	end else if (rx_stop) begin
		// page almost full, it means CRC tail will spill over next page
		if (&rx_w_cnt[7:2] && (rx_w_cnt[1:0] != 2'b00)) begin
			clda <= next_clda;
		end
	end
end

always @(posedge clk) begin
	// local DMA data writer
	if (rx_strobe_pe) begin
		rx_buffer[rx_w_cnt] <= rx_byte;
	end
	// remote DMA data reader
	rx_buffer_do <= rx_buffer[crda];
end

// local DMA TX counter
always @(posedge clk) begin
	if (txp_pe) begin
		tx_r_cnt <= 11'd0;
	end else if (tx_strobe_pe) begin
		tx_r_cnt <= tx_r_cnt + 11'd1;
		tx_byte  <= tx_buffer_do;
	end
end

always @(posedge clk) begin
	// remote DMA data writer
	if (wr_ne) begin
		if (dma_port) begin
			tx_buffer[crda] <= din;
		end
	end
	// local/remote DMA data reader
	tx_buffer_do <= tx_buffer[txp ? tx_r_cnt : crda];
end

wire rbcr_is_0 = (rbcr == 16'd0);
wire rbcr_is_1 = (rbcr == 16'd1);

// cpu write via read
always @(posedge clk) begin
	if (reset_pe || reset) begin
		cr   <= 8'h21; // ABORT, STP
		isr  <= 8'h80; // RST
		imr  <= 8'h00;
 		// ident of netusbee
		rbcr <= 16'h7050;
		// internals
		rx_busy <= 1'b0;
		prev[7] <= 1'b1;
	end else begin

		if (wr_ne) begin
			if (!dma_port) begin
				// register page 0
				if (ps == 0) begin
					case (addr)
						5'h01: pstart <= din;
						5'h02: pstop <= din;
						5'h03: begin
							bnry <= din;
							rx_busy <= 1'b0;
						end
						5'h05: tbcr[7:0] <= din;
						5'h06: tbcr[10:8] <= din[2:0];
						5'h07: isr <= isr & ~din; // write-1-to-clear
						5'h08: rsar[7:0] <= din;
						5'h09: rsar[15:8] <= din;
						5'h0a: rbcr[7:0] <= din;
						5'h0b: rbcr[15:8] <= din;
						5'h0f: imr <= din;
						default: ;
					endcase
				end

				// register page 1
				else if (ps == 1) begin
					case (addr)
						5'h07: curr <= din;
						default: ;
					endcase
				end

				// cr is available on all pages
				if (addr == 0) begin
					cr <= din;
					if (din[1])
						// start
						isr[7] <= 1'b0; // RST
					if (din[5:3] == 3'b100) begin
						// remote dma abort/complete
						isr[6] <= 1'b1; // RDC
						crda <= 11'd0;
						rbcr <= 16'd0;
					end else if (din[3] || din[4]) begin
						// remote dma read or write
						crda <= { 3'h00, rsar[7:0] };
						if (rbcr_is_0) begin
							isr[6] <= 1'b1; // RDC
						end
					end
				end

			end else if (!rbcr_is_0) begin
				// remote dma write
				crda <= crda + 11'd1;
				rbcr <= rbcr - 16'd1;
				if (rbcr_is_1) begin
					isr[6] <= 1'b1; // RDC
				end
			end

		end else if (rd_ne) begin
			if (dma_port && !rbcr_is_0) begin
				// remote dma read
				crda <= crda + 11'd1;
				rbcr <= rbcr - 16'd1;
				if (rbcr_is_1) begin
					isr[6] <= 1'b1; // RDC
				end
			end
		end

		// outgoing frame transmitted
		if (tx_stop) begin
			isr[1] <= 1'b1; // PTX
			cr[2] <= 1'b0;  // TXP
		end

		// incoming frame received
		if (rx_stop) begin
			rx_busy <= 1'b1;
			prev <= curr;
		end else if (rx_stop_d) begin
			isr[0] <= 1'b1; // PRX
			curr <= clda;
		end
	end
end

assign int_n = ~(|(isr & imr) & ~reset);

endmodule
