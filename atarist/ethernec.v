// ethernec.v
//
// Atari ST NE2000/ethernec implementation for the MiST board
// https://github.com/mist-devel/mist-board
//
// Copyright (c) 2014 Till Harbaum <till@harbaum.org>
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

module ethernec (
	// cpu register interface
	input            clk,
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

	output           ne_int      // interrupt occured
);

// some non-zero and non-all-ones bytes as status flags
localparam STATUS_IDLE       = 8'hfe;
localparam STATUS_TX_PENDING = 8'ha5;
localparam STATUS_TX_DONE    = 8'h12;

reg [7:0] statusCode;

wire tx_pending = cr[2] && (rbcr == 0);
assign status = { statusCode, 5'h00, 1'd0 /*FIXME*/, isr[1:0], 5'h00, tbcr };

localparam RX_W_IDLE   = 2'b00;
localparam RX_W_DATA   = 2'b01;
localparam RX_W_HEADER = 2'b10;

reg [1:0] rx_w_state   = RX_W_IDLE;

// ----- bus interface signals as wired up on the ethernec/netusbee ------
// sel[0] = 0xfa0000 -> normal read
// sel[1] = 0xfb0000 -> write through address bus
wire ne_read = rd;
wire ne_write = wr;

reg ne_readD, ne_writeD;

always @(posedge clk) begin
	ne_readD <= ne_read;
	ne_writeD <= ne_write;
end

wire ne_read_en  = ~ne_read  & ne_readD;
wire ne_write_en = ~ne_write & ne_writeD;

reg reset = 1'b1;

// ---------- ne2000 internal registers -------------
reg [7:0]  cr;             // command register
reg [7:0]  isr;            // interrupt service register
reg [7:0]  imr;            // interrupt mask register
reg [7:0]  curr;           // current page register
reg [7:0]  bnry;           // boundary page
reg [7:0]  pstart;         // rx buffer ring start page
reg [7:0]  pstop;          // rx buffer ring stop page
reg [15:0] rbcr;           // receiver byte count register
reg [15:0] rsar;           // receiver address register
reg [10:0] tbcr;           // transmitter byte count register

wire [1:0] ps = cr[7:6];              // register page select
wire [2:0] dma_cmd = din[5:3];        // remote DMA command

wire dma_port = (addr[4:3] == 2'b10); // DMA ports ($10-$17)
wire rst_port = (addr[4:3] == 2'b11); // reset ports ($18-$1F)

// ----------------- rx/tx buffers ------------------
localparam BUF_SIZE = 2048;           // to logic simplify

(* ramstyle = "no_rw_check, M9K" *)
reg [7:0] rx_buffer [BUF_SIZE-1:0];   // 1 ethernet frame + 4 bytes header
reg [10:0] rx_w_cnt = 0;              // receive buffer byte counter

(* ramstyle = "no_rw_check, M9K" *)
reg [7:0] tx_buffer [BUF_SIZE-1:0];   // 1 ethernet frame
reg [10:0] tx_r_cnt = 0;              // transmit buffer byte counter

reg [7:0] rx_buffer_do;

always @(posedge clk) begin
	rx_buffer_do <= rx_buffer[rsar[10:0]];
end

// ----- io controller read access to tx buffer -----
reg [2:0] tx_begin_sr;
reg [2:0] tx_strobe_sr;
reg       rx_begin_d;

always @(posedge clk) begin
	tx_begin_sr  <= { tx_begin_sr[1:0],  tx_begin  };
	tx_strobe_sr <= { tx_strobe_sr[1:0], tx_strobe };
	rx_begin_d   <= rx_begin;
end

wire tx_done = tx_begin_sr[1] & ~tx_begin_sr[2];

always @(posedge clk) begin
	tx_byte <= tx_buffer[tx_r_cnt];

	if (tx_done)
		tx_r_cnt <= 11'd0;
	else if (tx_strobe_sr[1] & ~tx_strobe_sr[2])
		tx_r_cnt <= tx_r_cnt + 11'd1;
end

// ------------- set local mac address --------------
reg [7:0] mac [5:0];
reg [2:0] mac_cnt;

// mac address from io controller
always @(posedge clk) begin
	if (mac_begin)
		mac_cnt <= 0;
	else if (mac_strobe) begin
		if (mac_cnt < 6) begin
			mac[mac_cnt] <= mac_byte;
			mac_cnt <= mac_cnt + 3'd1;
		end
	end
end

// ------ NetUSBee: 93C46 eeprom mac-read stub ------
reg [7:0]  ee_cr;
reg [3:0]  ee_bit_cnt;
reg [15:0] ee_shift_reg;
reg        ee_sclk_D;

wire ee_reg_write = ne_write_en && (ps == 3) && (addr == 1);
wire ee_reset     = ee_reg_write ? !din[3] : !ee_cr[3];

wire ee_sclk_posedge =  ee_cr[2] && !ee_sclk_D;
wire ee_sclk_negedge = !ee_cr[2] &&  ee_sclk_D;

always @(posedge clk) begin
	if (reset) ee_sclk_D <= 1'b0;
	else       ee_sclk_D <= ee_cr[2];
end

always @(posedge clk) begin
	if (reset) begin
		ee_cr        <= 0;
		ee_bit_cnt   <= 0;
		ee_shift_reg <= 0;
	end else begin

		if (ee_reg_write)
			ee_cr <= din;

		if (ee_reset) begin
			ee_bit_cnt   <= 0;
			ee_shift_reg <= 0;
		end else begin

			if (ee_sclk_posedge) begin
				if (ee_bit_cnt < 15)
					ee_bit_cnt <= ee_bit_cnt + 1;

				if (ee_bit_cnt < 10)
					ee_shift_reg <= { ee_shift_reg[14:0], ee_cr[1] };
			end

			if (ee_sclk_negedge) begin
				if (ee_bit_cnt == 10) begin
					case (ee_shift_reg[2:0])
						3'b010:  ee_shift_reg <= { mac[1], mac[0] };
						3'b011:  ee_shift_reg <= { mac[3], mac[2] };
						3'b100:  ee_shift_reg <= { mac[5], mac[4] };
						default: ee_shift_reg <= 0;
					endcase

				end else if (ee_bit_cnt > 10) begin
					ee_shift_reg <= { ee_shift_reg[14:0], 1'b0 };
				end
			end
		end
	end
end

wire ee_do = (ee_bit_cnt >= 10) ? ee_shift_reg[15] : 1'b0;

reg [7:0] reg_do;

always @(*) begin
	reg_do = 8'h00; 

	casez ({ ps, addr })
		// page 0
		{ 2'b00, 5'h00 }: reg_do = cr;
		{ 2'b00, 5'h03 }: reg_do = bnry;
		{ 2'b00, 5'h04 }: reg_do = 8'h23; // tsr: tx ok
		{ 2'b00, 5'h07 }: reg_do = isr;
		{ 2'b00, 5'h08 }: reg_do = rsar[7:0];
		{ 2'b00, 5'h09 }: reg_do = rsar[15:8];
		{ 2'b00, 5'h0a }: reg_do = rbcr[7:0];
		{ 2'b00, 5'h0b }: reg_do = rbcr[15:8];
		{ 2'b00, 5'h0c }: reg_do = 8'h01; // rsr: rx ok
		{ 2'b00, 5'h0e }: reg_do = 8'h48; // dcfg: 8-bit, fifo=4

		// page 1
		{ 2'b01, 5'h00 }: reg_do = cr;
		{ 2'b01, 5'h01 }: reg_do = mac[0];
		{ 2'b01, 5'h02 }: reg_do = mac[1];
		{ 2'b01, 5'h03 }: reg_do = mac[2];
		{ 2'b01, 5'h04 }: reg_do = mac[3];
		{ 2'b01, 5'h05 }: reg_do = mac[4];
		{ 2'b01, 5'h06 }: reg_do = mac[5];
		{ 2'b01, 5'h07 }: reg_do = curr;

		// page 3
		{ 2'b11, 5'h00 }: reg_do = cr;
		{ 2'b11, 5'h01 }: reg_do = { ee_cr[7:1], ee_do };
		{ 2'b11, 5'h03 }: reg_do = 8'h18; // config0: rtl8019as, PnP
		{ 2'b11, 5'h05 }: reg_do = 8'h40; // config2: 10Base-T active
		{ 2'b11, 5'h06 }: reg_do = 8'h40; // config3: full duplex

		// remote dma (0x10 - 0x17)
		{ 2'b00, 5'b10??? }, 
		{ 2'b01, 5'b10??? }: begin
			if (rsar[15:8] == 0) begin
				reg_do = (rsar[2:0] < 6) ? mac[rsar[2:0]] : 8'h00;
			end else begin
				reg_do = rx_buffer_do;
			end
		end

		default: reg_do = 8'h00;
	endcase
end

// cpu register read
always @(posedge clk) begin
	if (ne_read) begin
		// read is active ($faxxxx)
		dout <= reg_do;
	end
end

reg resetD;

// delay internal reset signal
always @(posedge clk) resetD <= reset;

// generate an internal strobe signal to copy setup header
wire int_strobe_en = (rx_w_state == RX_W_HEADER) ? 1'b1 : 1'b0;

// internal header transfer is started at the end of the data transmission
wire int_begin = (reset & !resetD) || header_begin;

wire rx_write_en = (rx_strobe || int_strobe_en) && !int_begin;

reg rx_last_byte;

// the ne2000 page size is 256 bytes. thus the page counters are increased
// every 256 bytes when a data transfer is in progress.
wire [10:0] rx_start = { curr[2:0], 8'h00 };

reg [10:0] rx_len; // number of bytes received from io controller
reg [7:0] next_page;

wire [7:0] header_byte =
	(rx_w_cnt[1:0] == 2'b00) ? 8'h01 :
	(rx_w_cnt[1:0] == 2'b01) ? next_page :
	(rx_w_cnt[1:0] == 2'b10) ? rx_len[7:0] : { 5'b00000, rx_len[10:8] };

always @(posedge clk) begin
	rx_last_byte <= 1'b0;

	if ((rx_w_state == RX_W_HEADER) && (rx_w_cnt == (rx_start + 4)))
		rx_last_byte <= !int_begin;
end

// data transfer on other edge
always @(posedge clk) begin
	if (reset) begin
		rx_w_cnt <= { curr[2:0], 8'h00 } + 11'd4;
	end else begin

		if (rx_write_en) begin
			case (rx_w_state)
				RX_W_DATA:   rx_buffer[rx_w_cnt] <= rx_byte;
				RX_W_HEADER: rx_buffer[rx_w_cnt] <= header_byte;
				default: ;
			endcase
		end

		if (!rx_begin_d && rx_begin) begin
			rx_w_cnt <= rx_start + 4;
		end else if (header_begin) begin
			rx_w_cnt <= rx_start;
		end else if (rx_write_en) begin
			if (rx_w_state != RX_W_IDLE) begin
				rx_w_cnt <= rx_w_cnt + 1;
			end
		end
	end
end

// register to delay receive counter increment by one cycle so this 
// does happen after the read cycle has finished
reg rx_inc;
reg tx_inc;
reg header_begin;

// generate flag indicating that a header transfer is about to begin
always @(posedge clk) begin
	header_begin <= (rx_begin_d & ~rx_begin);
end

// write counter - header size (4) = number of bytes written
always @(posedge clk) begin
	if (reset) begin
		rx_len <= 11'd0;
	end else if (rx_begin_d && ~rx_begin) begin
		rx_len <= rx_w_cnt - (rx_start + 4);
	end
end

// cpu write via read
always @(posedge clk) begin
	// reset state
	if (reset & !resetD) begin
		cr     <= 8'h21;
		isr    <= 8'h80; // RST
		imr    <= 8'h00;
		bnry   <= 8'h40;
		pstart <= 8'h40;
		pstop  <= 8'h47;
		curr   <= 8'h40;
		rsar   <= 16'd0;
		// ident of netusbee 1.1
		rbcr[7:0]  <= 8'h50;
		rbcr[15:8] <= 8'h70;
		// internals
		rx_w_state <= RX_W_IDLE;
		statusCode <= STATUS_IDLE;
		next_page  <= 8'h41;
		rx_inc     <= 1'b0;
		tx_inc     <= 1'b0;
		reset      <= 1'b0;
	end else begin

		if (ne_write_en && (ps == 0) && (addr == 7)) begin
			isr <= isr & (~din); // writing 1 clears bit

			if (din[1]) begin
				// clear PTX
				statusCode <= STATUS_IDLE;
			end
		end

		// last byte ends a mac or header transfer and causes the
		// receiver state machine to return to the idle state
		if (rx_last_byte) begin
			rx_w_state <= RX_W_IDLE;

			if (rx_w_state == RX_W_HEADER) begin
				isr  <= isr | 8'h01; // PRX
				curr <= next_page;

				if (next_page >= pstop - 1'b1)
					next_page <= pstart;
				else
					next_page <= next_page + 8'd1;
			end
		end

		// The rising edge of rx_begin indicates the start of a data transfer
		if (!rx_begin_d && rx_begin) begin
			rx_w_state <= RX_W_DATA;
		end

		// The falling edge of rx_begin marks the end of a data transfer.
		// So we start setting up the pkt header after the end of the transfer
		if (rx_begin_d && !rx_begin) begin
			rx_w_state <= RX_W_HEADER;
		end

		rx_inc <= 1'b0;
		tx_inc <= 1'b0;

		if (rx_inc || tx_inc) begin
			rsar <= rsar + 1'b1;

			if (rbcr != 0) begin
				rbcr <= rbcr - 1'b1;

				if (rbcr == 1) begin
					cr[5:3] <= 3'b011; 
					isr     <= isr | 8'h40; // RDC
				end
			end
		end

		// signal end of transmission if tx buffer has been read by
		// io controller
		if (tx_done) begin
			isr <= isr | 8'h02; // PTX
			statusCode <= STATUS_TX_DONE;
			cr[2] <= 1'b0;
		end

		// if cpu reads have internal side effects then ths is handled
		// here (and not in the "register read" block above)
		if (ne_read_en) begin
			// register page 0
			if (ps == 0) begin
			end

			// register page 1
			if (ps == 1) begin
			end

			// read dma register $10-$17
			if (dma_port)
				rx_inc <= 1'b1;

			// read reset register $18-$1f
			if (rst_port) begin
				reset <= 1'b1; // soft reset
				statusCode <= STATUS_IDLE;
				rx_w_state <= RX_W_IDLE;
			end
		end

		if (ne_write_en) begin
			if (addr == 0) begin
				cr <= din;

				// writing the command register may actually start things ...

				// check for dma is stopped (Remote DMA Abort)
				if (dma_cmd == 4) begin
					isr <= isr | 8'h40; // RDC
					rbcr <= 0;
				end

				// check if TX bit was set
				if (din[2]) begin
					// tx buffer is now full and its contents need to be sent to
					// the io controller which in turn forwards it to its own nic
					statusCode <= STATUS_TX_PENDING;
				end
			end

			// register page 0
			if (ps == 0) begin
				case (addr)
					5'h01: pstart <= din;
					5'h02: pstop <= din;
					5'h03: bnry <= din;
					5'h05: tbcr[7:0] <= din;
					5'h06: tbcr[10:8] <= din[2:0];
					5'h08: rsar[7:0] <= din;
					5'h09: rsar[15:8] <= din;
					5'h0a: rbcr[7:0] <= din;
					5'h0b: rbcr[15:8] <= din;
					5'h0f: imr <= din;
					default: ;
				endcase
			end

			// register page 1
			if (ps == 1) begin
				if (addr == 7) begin
					curr <= din;
					if (din >= pstop - 1'b1)
						next_page <= pstart;
					else
						next_page <= din + 1'b1;
				end
			end

			// write to dma register $10-$17
			if (dma_port) begin
				tx_buffer[rsar[10:0]] <= din;
				tx_inc <= 1'b1;
			end

			// reset register $18-$1f
			if (rst_port)
				reset <= 1'b0; // write to reset register clears reset
		end
	end
end

// INT if not reset, not masked and not stopped
assign ne_int = (| (isr & imr)) && !reset && !cr[0];

endmodule
