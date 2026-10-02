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
	// CPU interface
	input            clk,
	input            rst,
	input            rd,
	input            wr,
	input      [4:0] addr,
	input      [7:0] din,
	output reg [7:0] dout,

	// ethernet status word to be read by io controller
	output    [31:0] status,

	// interface to allow the i/o controller to read frames from the tx buffer
	input            tx_begin,   // rising edge before new tx byte stream is sent
	input            tx_strobe,  // rising edge before each tx byte
	output reg [7:0] tx_byte,    // byte from transmit buffer 

	// interface to allow the i/o controller to write frames to the rx buffer
	input            rx_begin,   // rising edge before new rx byte stream is sent
	input            rx_strobe,  // rising edge before each rx byte
	input      [7:0] rx_byte,    // byte to be written to rx buffer 

	// interface to allow MAC address being set by i/o controller
	input            mac_begin,  // rising edge before new mac is sent
	input            mac_strobe, // rising edge before each mac byte
	input      [7:0] mac_byte,   // mac address byte

	output           irq         // interrupt request
);

// tx_ready[17], rx_ready[16], tx_count[15:0]
assign status = { 8'h00, 6'h00, tx_ready, rx_ready, 5'h00, tbcr };

// NE2000 internal registers
reg [7:0]  cr;             // command register
reg [7:0]  isr;            // interrupt service register
reg [7:0]  imr;            // interrupt mask register
reg [7:0]  curr;           // current page register
reg [7:0]  bnry;           // boundary page
reg [7:0]  clda;           // current local dma page register
reg [7:0]  pstart;         // rx buffer ring start page
reg [7:0]  pstop;          // rx buffer ring stop page
reg [15:0] rsar;           // remote start address register
reg [15:0] rbcr;           // remote byte count register
reg [3:0]  tpsr;           // transmit page start address
reg [10:0] tbcr;           // transmitter byte count register

wire stop = cr[0];         // stop mode
wire txp  = cr[2];         // transmit packet toggle
wire [1:0] ps = cr[7:6];   // register page select

wire dma_port = (addr[4:3] == 2'b10); // remote DMA ports ($10 - $17)
wire rst_port = (addr[4:3] == 2'b11); // reset ports ($18 - $1F)

// rx/tx buffers
(* ramstyle = "no_rw_check" *) reg [7:0] rx_buffer[4095:0]; // 2-4 ethernet frames
(* ramstyle = "no_rw_check" *) reg [7:0] tx_buffer[4095:0]; // 2 ethernet frames (ping-pong)

// i/o controller signals resync
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

`DELAY_REG(txp_d, txp)

wire txp_pe = txp & ~txp_d;

// reset
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

// syncing i/o controller rx frame traffic
wire [3:0] used = (curr - bnry);
wire full = (used >= 4'd10);

reg tx_ready;
reg rx_ready;

always @(posedge clk) begin
	tx_ready <= (~stop & txp);
	rx_ready <= (~stop & ~full);
end

// set local MAC address
reg [7:0] mac [5:0];
reg [2:0] mac_cnt;

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

// EEPROM read (rtl8019)
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

reg [7:0] rom_do;
reg [7:0] reg_do;

// PROM read
always @(*) begin
	case (rsar[3:0])
		4'h00: rom_do = mac[0];
		4'h01: rom_do = mac[1];
		4'h02: rom_do = mac[2];
		4'h03: rom_do = mac[3];
		4'h04: rom_do = mac[4];
		4'h05: rom_do = mac[5];
		4'h0e: rom_do = 8'h57;
		4'h0f: rom_do = 8'h57;
		default: rom_do = 8'h00;
	endcase
end

// register read
always @(*) begin
	case (ps)
		2'b00: begin
			// page 0
			case (addr)
				5'h00: reg_do = cr;
				5'h03: reg_do = bnry;
				5'h04: reg_do = 8'h01; // tsr: tx ok
				5'h07: reg_do = isr;
				5'h08: reg_do = rsar[7:0];
				5'h09: reg_do = rsar[15:8];
				5'h0a: reg_do = rbcr[7:0];
				5'h0b: reg_do = rbcr[15:8];
				5'h0c: reg_do = 8'h01; // rsr: rx ok
				5'h0e: reg_do = 8'h48; // dcr: 8-bit mode
				default: reg_do = 8'h00;
			endcase
		end
		2'b01: begin
			// page 1
			case (addr)
				5'h00: reg_do = cr;
				5'h07: reg_do = curr;
				default: reg_do = 8'h00;
			endcase
		end
		2'b10: begin
			// page 2
			case (addr)
				5'h00: reg_do = cr;
				default: reg_do = 8'h00;
			endcase
		end
		2'b11: begin
			// page 3 (rtl8019)
			case (addr)
				5'h00: reg_do = cr;
				5'h01: reg_do = { ee_cr[7:1], eeprom_do }; // 9346cr
				default: reg_do = 8'h00;
			endcase
		end
	endcase
end

wire is_prom = (rsar[15:8] == 8'h0);

// CPU read
always @(*) begin
	dout = 8'h00;
	if (rd) begin
		if (dma_port) begin
			dout = is_prom ? rom_do : rx_buffer_do;
		end else begin
			dout = reg_do;
		end
	end
end

wire [7:0] next_curr = ((curr + 8'd1) == pstop) ? pstart : (curr + 8'd1);
wire [7:0] next_clda = ((clda + 8'd1) == pstop) ? pstart : (clda + 8'd1);

reg rx_fin;
reg rx_fin_d;

wire rx_done = ~rx_fin & rx_fin_d;

reg [11:0] rx_addr;
reg [10:0] rx_length;

// local DMA frame receiver
always @(posedge clk) begin
	if (reset_pe) begin
		rx_fin <= 1'b0;
		rx_fin_d <= 1'b0;
	end else begin
		if (rx_start) begin
			// +4 for crc
			rx_length <= 11'd4;
			// +4 for header
			rx_addr <= { curr[3:0], 8'd4 };
			// first page already used
			clda <= next_curr;
		end else if (rx_strobe_pe) begin
			rx_buffer[rx_addr] <= rx_byte;
			rx_length <= rx_length + 11'd1;
			rx_addr <= rx_addr + 12'd1;
			// count full pages
			if (rx_addr[7:0] == 8'hFF) begin
				clda <= next_clda;
			end
		end else if (rx_stop) begin
			// prepare to write header
			rx_addr <= { curr[3:0], 8'd0 };
			rx_fin <= 1'b1;
		end else if (rx_fin) begin
			// writing frame header
			case (rx_addr[1:0])
				2'd0: rx_buffer[rx_addr] <= 8'h01;
				2'd1: rx_buffer[rx_addr] <= clda;
				2'd2: rx_buffer[rx_addr] <= rx_length[7:0];
				2'd3: begin
					rx_buffer[rx_addr] <= { 5'h00, rx_length[10:8] };
					rx_fin <= 1'b0;
				end
			endcase
			rx_addr <= rx_addr + 12'd1;
		end
	end
	rx_fin_d <= rx_fin;
end

reg [11:0] tx_addr;
reg  [7:0] tx_buffer_do;

// local DMA frame transmitter
always @(posedge clk) begin
	if (txp_pe) begin
		tx_addr <= { tpsr, 8'd0 };
	end else if (tx_strobe_pe) begin
		tx_byte <= tx_buffer_do;
		tx_addr <= tx_addr + 12'd1;
	end
	tx_buffer_do <= tx_buffer[tx_addr];
end

reg [7:0] rx_buffer_do;

// remote DMA data reader
always @(posedge clk) begin
	rx_buffer_do <= rx_buffer[rsar[11:0]];
end

wire rbcr_is_0 = (rbcr == 16'd0);
wire rbcr_is_1 = (rbcr == 16'd1);

// CPU write
always @(posedge clk) begin
	if (reset_pe || reset) begin
		cr   <= 8'h21;    // ABORT, STP
		isr  <= 8'h80;    // RST
		imr  <= 8'h00;
		rbcr <= 16'h7050; // Realtek ID
	end else begin
		if (wr_ne) begin
			if (!dma_port) begin
				// register page 0
				if (ps == 0) begin
					case (addr)
						5'h01: pstart <= din;
						5'h02: pstop <= din;
						5'h03: bnry <= din;
						5'h04: tpsr <= din[3:0];
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
					if (addr == 7) curr <= din;
				end
				// CR is available on all pages
				if (addr == 0) begin
					cr <= din;
					if (din[1])
						// start
						isr[7] <= 1'b0; // RST
					if (din[5]) begin
						// remote dma abort
					end else if (din[3] || din[4]) begin
						// remote dma read or write
						if (rbcr_is_0)
							isr[6] <= 1'b1; // RDC
					end
				end
			end else if (!rbcr_is_0) begin
				// remote dma write (tx)
				rsar <= rsar + 16'd1;
				rbcr <= rbcr - 16'd1;
				if (rbcr_is_1)
					isr[6] <= 1'b1; // RDC
				tx_buffer[rsar[11:0]] <= din;
			end
		end else if (rd_ne) begin
			if (dma_port && !rbcr_is_0) begin
				// remote dma read (rx)
				rsar <= rsar + 16'd1;
				rbcr <= rbcr - 16'd1;
				if (rbcr_is_1)
					isr[6] <= 1'b1; // RDC
			end
		end
		// incoming frame received
		else if (rx_done) begin
			isr[0] <= 1'b1; // PRX
			curr <= clda;
		end
		// outgoing frame transmitted
		else if (tx_stop) begin
			isr[1] <= 1'b1; // PTX
			cr[2] <= 1'b0;  // TXP
		end
	end
end

assign irq = (|(isr & imr) & ~reset);

endmodule
