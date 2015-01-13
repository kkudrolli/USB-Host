/*
 * usbHost.sv
 *
 * Authors:    Kais Kudrolli  DJ Park
 * Andrew IDs: kkudroll       dongjoop 
 *
 * File that contains code for a simplified USB host. It is able to read 
 * data from and write data to memory on a single USB device (which is 
 * simulated by a tetbench). It uses CRC5 and CRC16 to perform error
 * checking, and uses retries in an attempt to correct errors. 
 *
 */

//////////////////////////////
// START KAIS KUDROLLI PART //
//////////////////////////////

// Struct that is used to represent a data packet
typedef struct packed {
    logic [7:0]  sync;
    logic [7:0]  pid;
    logic [63:0] data;
    logic [6:0]  addr;
    logic [3:0]  endp;
    logic [4:0]  crc5;
    logic [15:0] crc16;
} pkt_t;

/*
 * Top module for the USB Host. Contains all the module instantiations
 * and connections. It interfaces with the USB wires through the wires 
 * parameter.
 *
 * Inputs:
 *  - clk:   the common clock for the system
 *  - rst_L: the system reset
 *  - wires: the interface wires to the USB device
 */
module usbHost
  (input logic clk, rst_L, 
  usbWires wires);

  // Handshake signals between protocol fsm and rw fsm / rw fsm and task
  logic        task_avail, r_task_taken, w_task_taken, trans_avail, trans_taken; 
  logic        success_rw, success_read, success_write;
  logic [1:0]  id;
  logic [3:0]  pid;
  logic [6:0]  addr;
  logic [3:0]  endp;
  logic [15:0] m_page;
  logic [63:0] data_read, data_field, data_out, data_in;

  // Handshake signals in serial out
  logic        oneBitStart, processing, stuffing_out;
  logic        oneBit, stuffedOneBit, diff;

  // Handshake signals in serial in
  logic        start_in, stuffing_in, foundEOP;
  logic        diff_in, stuffedOneBit_in, unstuffedOneBit;

  // Handshake signals between protocol fsm and bit stream encoder
  logic        pkt_out_avail, pkt_out_taken;
  pkt_t        pkt_out;

  // Handshake signals between protocol fsm and bit stream decoder
  logic        pkt_in_avail, corrupt, pkt_in_taken;
  pkt_t        pkt_in;

  /* Tasks needed to be finished to run testbenches */

  task prelabRequest
  // sends an OUT packet with ADDR=5 and ENDP=4
  // packet should have SYNC and EOP too
  (input [7:0] data); // Unused

      task_avail <= 1;
      id <= 2'b00;
      wait (w_task_taken);
      task_avail <= 0;

  endtask: prelabRequest

  task readData
  // host sends memPage to thumb drive and then gets data back from it
  // then returns data and status to the caller
  (input  bit [15:0] mempage, // Page to write
   output bit [63:0] data, // array of bytes to write
   output bit        success);

      wait (~r_task_taken);
      task_avail <= 1; 
      id <= 2'b01; 
      m_page <= mempage;
      wait (r_task_taken);
      task_avail <= 0;
      success = success_read;
      data = data_out;

  endtask: readData

  task writeData
  // Host sends memPage to thumb drive and then sends data
  // then returns status to the caller
  (input  bit [15:0] mempage, // Page to write
   input  bit [63:0] data, // array of bytes to write
   output bit        success);

      wait (~w_task_taken);
      task_avail <= 1; 
      id <= 2'b10;  
      m_page <= mempage;
      data_in <= data;
      wait (w_task_taken);
      task_avail <= 0; 
      success = success_write;

  endtask: writeData

  // Hooked up the modules below here
  readWriteFsm rw_fsm(.clk(clk), .rst_L(rst_L), .task_avail(task_avail),
                      .trans_taken(trans_taken), .success_in(success_rw), 
                      .id(id), .mempage(m_page), .data_in(data_in),
                      .data_read(data_read), .trans_avail(trans_avail), 
                      .r_task_taken(r_task_taken), .w_task_taken(w_task_taken), 
                      .success_read(success_read), .success_write(success_write),
                      .pid(pid), .endp(endp), .addr(addr), 
                      .data_field(data_field), .data_out(data_out));

  protocol_fsm p_fsm(.clk(clk), .rst_L(rst_L), .trans_avail(trans_avail),
                     .corrupt(corrupt), .pkt_in(pkt_in), .pkt_in_avail(pkt_in_avail),
                     .pkt_out_taken(pkt_out_taken), .pid(pid), .endp(endp),
                     .addr(addr), .data0(data_field), .data_read(data_read),
                     .trans_taken(trans_taken), .pkt_out_avail(pkt_out_avail),
                     .success(success_rw), .pkt_out(pkt_out), 
                     .pkt_in_taken(pkt_in_taken));

  BitStreamEncoder bse(.clk(clk), .rst_L(rst_L), .pkt_out_avail(pkt_out_avail),
                       .pkt_out(pkt_out), .oneBit(oneBit), .oneBitStart(oneBitStart),
                       .pkt_out_taken(pkt_out_taken), .processing(processing), .stuffing_out(stuffing_out));
  
  BitStuffing bstuff(.clk(clk), .oneBitStart(oneBitStart), .oneBit(oneBit),
                     .stuffedOneBit(stuffedOneBit), .processing(processing), .stuffing_out(stuffing_out)); 

  NRZI_out nrzi_out(.clk(clk), .oneBitStart(oneBitStart), .stuffedOneBit(stuffedOneBit),
                    .diff(diff), .processing(processing));

  DPDM dpdm(.clk(clk), .rst_L(rst_L), .oneBitStart(oneBitStart), .diff(diff), 
            .processing(processing), .wires(wires), .pkt_in_taken(pkt_in_taken), 
            .diff_in(diff_in), .start_in(start_in), .foundEOP(foundEOP));

  NRZI_in nrzi_in(.clk(clk), .rst_L(rst_L), .diff_in(diff_in), .stuffedOneBit_in(stuffedOneBit_in));

  BitUnstuffing bunstuff(.clk(clk), .stuffedOneBit_in(stuffedOneBit_in), .start_in(start_in),
                         .pkt_in_taken(pkt_in_taken), .unstuffedOneBit(unstuffedOneBit), 
						 .stuffing_in(stuffing_in));

  BitStreamDecoder bsd(.clk(clk), .rst_L(rst_L), .start_in(start_in), .stuffing_in(stuffing_in),
                       .unstuffedOneBit(unstuffedOneBit), .pkt_in_taken(pkt_in_taken), .pkt_in(pkt_in),
					   .corrupt(corrupt), .pkt_in_avail(pkt_in_avail), .foundEOP(foundEOP));

endmodule: usbHost

/*****************************************
 ********RW FSM and Protocol FSM**********
 *****************************************/

/*
 * Mealy style FSM that translates task calls to transactions that are 
 * sent to the protocol FSM. 
 * Data names may be confusing, so here's a diagram:
 *             data_in          data_out
 *               |                 ^
 *               V                 |
 *               ___________________
 *               |                 | 
 *               |     rw fsm      |
 *               |                 |
 *               __________________
 *               |                 ^
 *               V                 |
 *            data_field        data_read
 *
 * Inputs: 
 *  - clk: the system clock
 *  - rst_L: system reset
 *  - task_avail: indicates that a task has been called
 *  - trans_taken: indicates the transaction has been processed
 *  - success_in: indicates a succesful transaction from the protocol fsm 
 *  - id: indicates which task was called
 *  - mempage: the address to which data is written or from which data is read
 *  - data_in: the data from the task to rw fsm
 *  - data_read: the data from the protocol fsm to the rw fsm
 *  - trans_avail: indicates a transaction has been initiated
 *
 * Outputs:
 *  - r_task_taken: asserted when a read task is completed
 *  - w_task_taken: asserted when a write task is completed
 *  - success_read: indicates a successful read
 *  - success_write: indicates a successful write
 *  - pid: the pid of the outgoing packet
 *  - endp: the endpoint
 *  - addr: the address of the device
 *  - data_field: the data from the rw fsm to the protocol fsm
 *  - data_out: the data from the rw fsm to the calling task
 */
module readWriteFsm
    (input  logic        clk, rst_L, task_avail, trans_taken, success_in,
     input  logic [1:0]  id,
     input  logic [15:0] mempage,
     input  logic [63:0] data_in, data_read,
     output logic        trans_avail, r_task_taken, w_task_taken, success_read, success_write,
     output logic [3:0]  pid, endp,
     output logic [6:0]  addr, 
     output logic [63:0] data_field, data_out);

    enum logic [2:0] {init, out, read, write} cs, ns;

    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) cs <= init;
        else cs <= ns;
    end

    // Combined output and next state logic
    always_comb begin 
        ns = init;
        trans_avail = 0;
        r_task_taken = 0;
        w_task_taken = 0;
        pid = 4'd0;
        addr = 7'd0;
        endp = 4'd0;
        data_field = 64'd0;
        data_out = 64'd0;
        success_read = 0;
        success_write = 0;
        case (cs)
            /*
             * IDs:
             *     2'b01 -> read task
             *     2'b10 -> write task
             */
            init: begin
                // If a task is called, fsm goes to the out state
                ns = (task_avail) ? out : init;
                trans_avail = (task_avail) ? 1 : 0;
                pid = (task_avail) ? 4'b0001 : 4'b0000;
                addr = (task_avail) ? 7'd5 : 7'd0;
                endp = (task_avail) ? 4'd4 : 4'd0;
                data_field = (task_avail) ? {mempage, 48'd0} : 64'd0;
            end
            out: begin
                // An OUT transaction is inititated and maintained here
                // From this state, it can go to either read or write based on id (which task was called)
                ns = (trans_taken) ? ((id == 2'b01) ? read : write) : out;
                trans_avail = 1;
                pid = (trans_taken) ? ((id == 2'b01) ? 4'b1001 : 4'b0001) : 4'b0001;
                addr = 7'd5;
                endp = (trans_taken) ? 7'd8 : 7'd4; 
                data_field = (trans_taken) ? ((id == 2'b01) ? 64'd0 : data_in) : {mempage, 48'd0}; 
            end
            read: begin
                // An IN transaction for reading is initiated
                ns = (trans_taken) ? init : read;
                trans_avail = (trans_taken) ? 0 : 1;
                r_task_taken = (trans_taken) ? 1 : 0;
                success_read = (trans_taken) ? 1 : 0;
                data_out = (trans_taken) ? data_read : 64'd0;
                pid = (trans_taken) ? 4'b0000 : 4'b1001;
                addr = (trans_taken) ? 7'd0 : 7'd5;
                endp = (trans_taken) ? 4'd0 : 4'd8;
            end
            write: begin
                // An OUT transaction for writing is initiated
                ns = (trans_taken) ? init : write;
                trans_avail = (trans_taken) ? 0 : 1;
                w_task_taken = (trans_taken) ? 1 : 0;
                success_write = (trans_taken) ? 1 : 0;
                pid = (trans_taken) ? 4'b0000 : 4'b0001;
                addr = (trans_taken) ? 7'd0 : 7'd5;
                endp = (trans_taken) ? 4'd0 : 4'd8;
                data_field = (trans_taken) ? 64'd0 : data_in;
            end
        endcase
    end
endmodule: readWriteFsm

/*
 * FSM that controls takes transactions from the R/W FSM and translates them
 * to packets sent to the bit stream encoder. It also takes in packets from 
 * the bit stream decoder. Finally, it handles corruption and timeout retries
 * when there is data corruption or no reponse from the device.
 *
 * Inputs:
 *  - clk: system clock
 *  - rst_L: system reset
 *  - trans_avail: indicates the transaction has started
 *  - pkt_out_taken: the pkt sent to the encoder is taken
 *  - corrupt: indicates that a corruption was detected by the CRC
 *  - pkt_in_avail: the packet from the decoder is available
 *  - pid: the process id of the out packet
 *  - endp: the endpoint 
 *  - addr: the address of the device
 *  - data0: the data being sent from the rw fsm to the protocol fsm
 *  - pkt_in: the pkt coming in from the decoder
 * Outputs:
 *  - trans_taken: indicates the transaction is taken
 *  - pkt_out_avail: indicates the outgoing pkt is available
 *  - success: indicates a successful transaction
 *  - pkt_in_taken: indicates the pkt from the decoder has been taken
 *  - data_read: data that is read from the pkt_in and sent to the rw fsm
 *  - pkt_out: the pkt sent to the encoder 
 */
module protocol_fsm
    (input  logic        clk, rst_L, trans_avail, pkt_out_taken, corrupt, pkt_in_avail,
     input  logic [3:0]  pid, endp,
     input  logic [6:0]  addr,
     input  logic [63:0] data0,
     input  pkt_t        pkt_in,
     output logic        trans_taken, pkt_out_avail, success, pkt_in_taken, 
     output logic [63:0] data_read,
     output pkt_t        pkt_out);

    enum logic [2:0] {trans, out, in, next, ack_read, w_data} cs, ns;
    enum logic [2:0] {NONE, OUT, IN, DATA0, ACK, NAK}   pkt; 

    logic       inc_corrupt_cnt, inc_time_cnt, inc_cyc_cnt;
    logic       clr_corrupt_cnt, clr_time_cnt, clr_cyc_cnt;
    logic       ack, nak, rd;
    logic [3:0] corrupt_count, timeout_count;
    logic [7:0] cycles_count;

    // Loop counters
    integer a, b, c; 

    // Reversed fields
    logic [63:0] r_data0;
    logic [6:0]  r_addr;
    logic [3:0]  r_endp;
    
    /*
     * FSM code below here. 
     */
    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) begin
            cs <= trans;
            cycles_count <= 8'd0;
            corrupt_count <= 4'd0;
            timeout_count <= 4'd0;
            data_read <= 64'd0;
        end
        else begin
            cs <= ns;
            // Maintain the counters
            cycles_count <= (inc_cyc_cnt) ? cycles_count + 8'd1 : 
                           ((clr_cyc_cnt) ? 8'd0 : cycles_count);
            corrupt_count <= (inc_corrupt_cnt) ? corrupt_count + 4'd1 : 
                            ((clr_corrupt_cnt) ? 4'd0 : corrupt_count);
            timeout_count <= (inc_time_cnt) ? timeout_count + 4'd1 : 
                            ((clr_time_cnt) ? 4'd0 : timeout_count);
            data_read <= (rd) ? pkt_in.data : data_read;
        end
    end

    always_comb begin
        // Just a bunch of initializations 
        ns = trans;
        pkt = NONE;
        pkt_out_avail = 0;
        clr_corrupt_cnt = 0;
        clr_time_cnt = 0;
        clr_cyc_cnt = 0;
        success = 0;
        trans_taken = 0;
        inc_cyc_cnt = 0; 
        clr_cyc_cnt = 0;
        inc_time_cnt = 0;
        inc_corrupt_cnt = 0;
        pkt_in_taken = 0;
        rd = 0;

        case (cs) 
            trans: begin
                // Waits for a transaction then does an OUT or IN based on
                // the pid. The appropriate packet (IN or OUT) is sent on the
                // transition.
                ns = (trans_avail) ? ((pid == 4'b1001) ? in : out) : trans;
                pkt_out_avail = (trans_avail) ? 1 : 0;
                clr_corrupt_cnt = 1;
                clr_time_cnt = 1;
                clr_cyc_cnt = 1;
                pkt = (trans_avail) ? ((pid == 4'b1001) ? IN : OUT) : NONE;
            end
            out: begin
                // After the OUT transaction a DATA0 packet is sent.
                ns = (pkt_out_taken) ? next : out;
            end
            next: begin
                ns = w_data;
                pkt_out_avail = 1;
                pkt = DATA0; 
            end
            in: begin
                // After the IN transaction is sent, the fsm waits for a DATA0
                // packet. It maintains counters for timeouts and corruptions.
                // It will send NAKs and ACKs appropriately. Once it gets the 
                // DATA0 packet it goes to an acknowledge state. 
                ns = (corrupt_count == 4'd8 || timeout_count == 4'd8) ? trans :
                     ((pkt_in_avail) ? ack_read: in);
                pkt = (corrupt || cycles_count == 8'd255) ? NAK :  
                      ((pkt_in_avail) ? ACK : NONE);
                pkt_out_avail = (corrupt || cycles_count == 8'd255 || 
                                 pkt_in_avail) ? 1 : 0;
                trans_taken = (corrupt_count == 4'd8 || timeout_count == 4'd8) ? 1 : 0;
                inc_cyc_cnt = (cycles_count < 8'd255 && ~corrupt && ~pkt_in_avail) ? 1 : 0;
                clr_cyc_cnt = (corrupt || cycles_count == 8'd255) ? 1 : 0;
                inc_time_cnt = (cycles_count == 8'd255) ? 1 : 0;
                inc_corrupt_cnt = (corrupt) ? 1 : 0; 
                pkt_in_taken = (pkt_in_avail) ? 1 : 0;
                rd = (pkt_in_avail) ? 1 : 0;
            end
            ack_read: begin
                // The acknowledge state which waits for the ACK to be taken.
                // The data read from the packet is sent to the rw fsm.
                ns = (pkt_out_taken) ? trans : ack_read;
                success = (pkt_out_taken) ? 1 : 0;
                trans_taken = (pkt_out_taken) ? 1 : 0;
            end
            w_data: begin
                // This resends a DATA0 packet if a corruption is found. It also
                // handles timeouts. Once it gets an ack, 8 timeouts, or 8 corruptions 
                // it will go back to trans. 
                ns = ((pkt_in_avail && ack) || corrupt_count == 4'd8 || 
                       timeout_count == 4'd8) ? trans : w_data; 
                trans_taken = ((pkt_in_avail && ack) || corrupt_count == 4'd8 || 
                                timeout_count == 4'd8) ? 1 : 0;
                success = (pkt_in_avail && ack) ? 1 : 0;
                pkt_out_avail = ((pkt_in_avail && nak) || cycles_count == 8'd255) ? 1 : 0;
                pkt = ((pkt_in_avail && nak) || cycles_count == 8'd255) ? DATA0 : NONE;
                clr_cyc_cnt = ((pkt_in_avail && nak) || cycles_count == 8'd255) ? 1 : 0;
                inc_corrupt_cnt = (pkt_in_avail && nak) ? 1 : 0;
                inc_time_cnt = (cycles_count == 8'd255) ? 1 : 0;
                inc_cyc_cnt = (~pkt_in_avail || cycles_count < 8'd255) ? 1 : 0;
                pkt_in_taken = (pkt_in_avail) ? 1 : 0;
            end
        endcase
    end

    // Determine ack and nak
    assign ack = (pkt_in.pid == 8'b0100_1011) ? 1 : 0;
    assign nak = (pkt_in.pid == 8'b0101_1010) ? 1 : 0;

    // Reverse fields so they get sent lsb->msb
    always_comb begin
        r_data0 = 64'd0;
        r_addr = 7'd0;
        r_endp = 4'd0;
        // Reverse data
        for (a = 0; a < 64; a++) r_data0[a] = data0[63-a];
        // Reverse addr
        for (b = 0; b < 7; b++) r_addr[b] = addr[6-b]; 
        // Reverse endp
        for (c = 0; c < 4; c++) r_endp[c] = endp[3-c]; 
    end

    /*
     * The fields of the packet are set combinationally, but it is only made
     * available when the pkt_out_avail signal is asserted.
     */
    always_comb begin
        // Always the same
        pkt_out.crc5 = 5'd0;
        pkt_out.crc16 = 16'd0;
        pkt_out.sync = 7'b000_0001;
        // Initialized
        pkt_out.pid = 8'd0;
        pkt_out.addr = 7'd0;
        pkt_out.endp = 4'd0;
        pkt_out.data = 64'd0;
        case (pkt) 
            /* 
             * Pid, reversed and complemented in here not outside.
             * The logic below fills the appropriate fields of the packet 
             * being sent out.
             */
            OUT: begin
                pkt_out.pid = 8'b1000_0111;
                pkt_out.addr = r_addr;
                pkt_out.endp = r_endp;
            end
            IN: begin
                pkt_out.pid = 8'b1001_0110;
                pkt_out.addr = r_addr;
                pkt_out.endp = r_endp;
            end
            DATA0: begin
                pkt_out.pid = 8'b1100_0011;
                pkt_out.data = r_data0;
            end
            ACK: begin
                pkt_out.pid = 8'b0100_1011;
            end
            NAK: begin
                pkt_out.pid = 8'b0101_1010;
            end
        endcase
    end
endmodule: protocol_fsm


/********************************
 ******Encoding Pipeline*********
 ********************************/

////////////////////////////
// END KAIS KUDROLLI PART //
////////////////////////////

////////////////////////
// START DJ PARK PART //
////////////////////////

/**
 * BitStreamEncoder receives a packet from protocol fsm and sends out oneBit
 * serially to BitStuffing-NRZI_out-DPDM-Device. When it receives a packet, first
 * pid is extracted, and determines which procedure to follow.
 * CRC5 or CRC16 is calculated as the bit is serially transferred.
 * BitStreamEncoder also sends oneBitStart and processing signal to BitStuffing,
 * NRZI_out, and DPDM so that they would know a packet is being transferred.
 */

module BitStreamEncoder
    (input  logic clk, rst_L, pkt_out_avail, stuffing_out,
     input  pkt_t pkt_out,
     output logic oneBit, oneBitStart, pkt_out_taken, processing);
	// While stuffing_out occurs, oneBit should not be sent to BitStuffing module.

    logic [6:0] counter;
    logic [4:0] poly5;
    logic [15:0] poly16;
    logic [31:0] value32;
    logic [95:0] value96;
    logic [15:0] value16;
	logic [7:0] currentPID;
	logic enableTOKEN, enableDATA, enableHANDSHAKE;
	logic XXJprocessing;

    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) begin
            value32 <= 32'd0;
            value96 <= 96'd0;
            value16 <= 16'd0;
            poly5 <= 4'd0;
            processing <= 1'b0;
			XXJprocessing <= 1'b0;
        end
        else if (pkt_out_avail && (XXJprocessing!==1'b1)) begin
            // packet is available and currently not processing any packet

            value32 <= (enableTOKEN) ? {pkt_out.sync,pkt_out.pid,pkt_out.addr,pkt_out.endp,pkt_out.crc5} : 32'b0;
			value96 <= (enableDATA) ? {pkt_out.sync,pkt_out.pid,pkt_out.data,pkt_out.crc16} : 96'b0;
			value16 <= (enableHANDSHAKE) ? {pkt_out.sync,pkt_out.pid} : 16'b0;

            counter<=7'd0;
            poly5<=5'b11111;
			poly16=16'b11111111_11111111;
            processing <= 1'b1;
			XXJprocessing <= 1'b1;
        end
        else begin // processing == 1
			if(stuffing_out) begin
				// nothing happens
			end
			else begin // stuffing_out==0
				/////////
				//TOKEN//
				/////////
				if(enableTOKEN) begin
					if(counter < 7'd16) begin // SYNC, PID
						value32 <= (value32 << 1);
						counter <= counter+7'd1;
					end
					else if (counter<7'd27) begin // ADDR, ENDP
						value32 <= (value32 << 1);
						counter <= counter+7'd1;
						poly5 <= (poly5[4]^oneBit ? 5'b00101 : 5'b00000) ^ {poly5[3:0],1'b0};
						// CRC5 calculation
					end
					else if (counter<7'd32) begin // CRC5
						if(counter==7'd31) processing <= 1'b0;
						counter <= counter+7'd1;
						poly5 <= (poly5<<1);
						// Keep shifting
					end
					else if (counter<7'd35) begin // XXJ
						counter <= counter+7'd1;
					end
					else if (counter==7'd35) begin
						counter <= counter+7'd1;
						XXJprocessing <= 1'b0;
					end
					else counter <= counter;
				end

				////////
				//DATA//
				////////
				else if(enableDATA) begin
					if(counter < 7'd16) begin // SYNC, PID
						value96 <= (value96 << 1);
						counter <= counter+7'd1;
					end
					else if (counter<7'd80) begin // DATA
						value96 <= (value96 << 1);
						counter <= counter+7'd1;
						poly16 <= (poly16[15]^oneBit ? 16'b10000000_00000101 : 16'b0) ^ {poly16[14:0],1'b0};
						// CRC16 calculation
					end
					else if (counter<7'd96) begin // CRC16
						if(counter==7'd95) processing <= 1'b0;
						counter <= counter+7'd1;
						poly16 <= (poly16<<1);
						// Keep shifting
					end
					else if (counter<7'd99) begin
						counter <= counter+7'd1;
					end
					else if (counter==7'd99) begin
						counter <= counter+7'd1;
						XXJprocessing <= 1'b0;
					end
					else counter <= counter;
				end

				/////////////
				//HANDSHAKE//
				/////////////
				else if(enableHANDSHAKE) begin
					if(counter < 7'd16) begin // SYNC, PID
						if(counter==7'd15) processing <= 1'b0;
						value16 <= (value16 << 1);
						counter <= counter+7'd1;
					end
					else if (counter < 7'd19) begin // XXJ
						counter <= counter+7'd1;
					end
					else if (counter == 7'd19) begin
						counter <= counter+7'd1;
						XXJprocessing <= 1'b0;
					end
					else counter <= counter;
				end
			end
		end
    end

	always_comb begin
        if (~rst_L) begin
            currentPID = 8'd0;
        end
        else if (pkt_out_avail && (XXJprocessing!==1'b1)) begin
			currentPID = pkt_out.pid;
			// Extract pid from the input packet
        end
		else begin
			if(stuffing_out) begin
				// nothing happens
			end
			else begin // stuffing_out==0
				/////////
				//TOKEN//
				/////////
				if(enableTOKEN) begin
					if(counter<7'd27) begin
						oneBit=value32[31];
					end
					else if(counter<7'd32) begin
						oneBit=~poly5[4];
					end
					else begin
						oneBit=1'b0;
					end
					oneBitStart = (counter==7'd0);
					pkt_out_taken = (counter==7'd35);
				end

				////////
				//DATA//
				////////
				else if(enableDATA) begin
					if(counter<7'd80) begin
						oneBit=value96[95];
					end
					else if(counter<7'd96) begin
						oneBit=~poly16[15];
					end
					else begin 
						oneBit=1'b0;
					end
					oneBitStart = (counter==7'd0);
					pkt_out_taken = (counter==7'd99);
				end

				/////////////
				//HANDSHAKE//
				/////////////
				else if(enableHANDSHAKE) begin
					if(counter<7'd16) begin
						oneBit=value16[15];
					end
					else begin
						oneBit=1'b0;
					end
					oneBitStart = (counter==7'd0);
					pkt_out_taken = (counter==7'd19);
				end
			end
		end
	end

	assign enableTOKEN = (currentPID==8'b1000_0111 || currentPID== 8'b1001_0110);
	assign enableDATA = (currentPID==8'b1100_0011);
	assign enableHANDSHAKE = (currentPID==8'b0100_1011 || currentPID==8'b0101_1010);

endmodule: BitStreamEncoder

/**
 * BitStuffing simply inserts 0 when it sees six consecutive 1's.
 * If stuffing occurs, it sends stuffing_out signal to BitStreamEncoder so that
 * nothing happens(counter should not increment!) except that a 0 is transferred
 * in that clock cycle.
 */
module BitStuffing
    (input  logic clk, oneBitStart, oneBit, processing,
     output logic stuffedOneBit, stuffing_out);

	enum logic [2:0] {INIT=3'b000, SAW1=3'b001, SAW11=3'b010, SAW111=3'b011, SAW1111=3'b100, 
					SAW11111=3'b101, SAW111111=3'b110, NOTHING=3'b111} cs, ns;

	logic [6:0] counter;
	always_ff @(posedge clk) begin // Next State logic
		if (oneBitStart) begin 
			cs <= INIT;
			counter <= 7'd0;
		end
		else begin
			if(processing) begin
				if (counter < 7'd14) begin // SYNC, PID
					cs <= INIT;
					counter <= counter + 7'd1;
				end
				else if (counter==7'd14) begin // Bit Stuffing evaluation starts
					cs <= NOTHING;
					counter <= counter + 7'd1;
				end
				else begin
					cs <= ns;
					counter <= counter + 7'd1;
				end
			end
		end
	end

	always_comb begin // Output logic
		if (oneBitStart) begin
			stuffedOneBit = oneBit;
			stuffing_out = 1'b0;
		end
		else begin 
			if (processing) begin
				stuffedOneBit = (cs==SAW111111) ? 1'b0 : oneBit;
				stuffing_out = (cs==SAW111111) ? 1'b1 : 1'b0;
			end
			else begin
				stuffedOneBit = 1'b0;
				stuffing_out = 1'b0;
			end
		end
    end

	always_comb begin
		case(cs)
			INIT:      ns = INIT;
			NOTHING:   ns = (oneBit==1'b1) ? SAW1 : NOTHING;
			SAW1:      ns = (oneBit==1'b1) ? SAW11 : NOTHING;
			SAW11:     ns = (oneBit==1'b1) ? SAW111 : NOTHING;
			SAW111:    ns = (oneBit==1'b1) ? SAW1111 : NOTHING;
			SAW1111:   ns = (oneBit==1'b1) ? SAW11111 : NOTHING;
			SAW11111:  ns = (oneBit==1'b1) ? SAW111111 : NOTHING;
			SAW111111: ns = NOTHING; // Goes back to "saw nothing" state
		endcase
    end
endmodule: BitStuffing

/**
 * Diff is short for differential value.
 * diff==1 represents J, and diff==0 represents K.
 */
module NRZI_out
    (input  logic clk, oneBitStart, stuffedOneBit, processing,
     output logic diff);

    enum logic [1:0] {INIT=2'b00, J_STATE=2'b01, K_STATE=2'b10} cs;

    always_ff @(posedge clk) begin // Next State logic
        if(oneBitStart) begin
            cs <= K_STATE;
        end
        else begin
            if (processing) begin
                cs <= ((stuffedOneBit==1'b0) && (cs==J_STATE)) ? K_STATE :
                      ((stuffedOneBit==1'b0) && (cs==K_STATE)) ? J_STATE :
					  // If one bit input is 0, change the state.
                      (stuffedOneBit==1'b1) ? cs : INIT;
					  // If one bit input is 1, stay at the current state.
            end
        end
    end

    always_comb begin // Output logic
        if(oneBitStart) begin
            diff = 1'b0; // Always outputs K since SYNC starts from 0.
        end
        else begin
            case(cs)
                INIT: diff = 1'b0;
                J_STATE: diff = (stuffedOneBit == 1'b1) ? 1'b1 : 1'b0;
				// J_STATE == Differential 1
                K_STATE: diff = (stuffedOneBit == 1'b1) ? 1'b0 : 1'b1;
				// K_STATE == Differential 0
            endcase
        end
    end
endmodule: NRZI_out

/**
 * DPDM connects host and device, so tristate drivers either send oneBit from the host
 * to the device or drive serial value from the device to the host.
 *
 */
module DPDM
    (input    logic clk, rst_L, oneBitStart, diff, processing, pkt_in_taken,
     output   logic diff_in, start_in, foundEOP,
     usbWires       wires);

    enum logic [3:0] {NOTHING=4'd0, SAW0=4'd1, SAW00=4'd2, SAW000=4'd3, SAW0000=4'd4,
        SAW0000_0=4'd5, SAW0000_00=4'd6, SAW0000_000=4'd7, SAW0000_0001=4'd8, 
		SAWX=4'd9, SAWXX=4'd10, DONE=4'd11} cs, ns;

	logic enable_OUT;
    logic DP_IN, DM_IN;
    logic [2:0] counter;

	///////////
	//PKT OUT//
	///////////
	/*
	 * EOP(XXJ) has to be appended after the packet. Thus, count 3, and manually
	 * append X,X and J.
	 */
    always_ff @(posedge clk) begin
        if(processing) begin
            counter<=3'd0;
        end
        else if (counter<3'd3) begin //XXJ
            counter<=counter+3'd1;
        end
        else counter <= counter;
    end

	always_ff @(posedge clk, negedge rst_L) begin
		if(~rst_L) begin
			cs <= NOTHING;
		end
		else begin
 	    	if(oneBitStart) begin
				cs <= NOTHING;
			end
			else begin
				cs <= ns;
			end
		end
	end

    always_comb begin
        if(oneBitStart) begin
			enable_OUT = 1'b1; // From Host to Device transaction starts
		end
        else begin
			if(counter>3'd2) begin
				enable_OUT = 1'b0; // From Device to Host transaction is possible
			end
        end
    end

	//////////
	//PKT IN//
	//////////
    /*
     * SYNC(0000_0001) tells us where the packet starts. Thus, wait for SYNC, and
     * when it sees it, assert start_in signal so that modules in decoder side
     * (NRZI_in,BitUnstuffing,BitStreamDecoder) would know a new packet is coming from the device.
     * After it receives EOP(XXJ), it finally asserts pkt_in_taken signal and sends to protocol fsm.
     */
    always_comb begin // Next State logic
        case(cs)
            NOTHING: ns = (DP_IN==1'b0 && DM_IN==1'b1) ? SAW0 : NOTHING;
            SAW0: ns = (DP_IN==1'b1 && DM_IN==1'b0) ? SAW00 : NOTHING;
            SAW00: ns = (DP_IN==1'b0 && DM_IN==1'b1) ? SAW000 : NOTHING;
            SAW000: ns = (DP_IN==1'b1 && DM_IN==1'b0) ? SAW0000 : NOTHING;
            SAW0000: ns = (DP_IN==1'b0 && DM_IN==1'b1) ? SAW0000_0 : NOTHING;
            SAW0000_0: ns = (DP_IN==1'b1 && DM_IN==1'b0) ? SAW0000_00 : NOTHING;
            SAW0000_00: ns = (DP_IN==1'b0 && DM_IN==1'b1) ? SAW0000_000 : NOTHING;
            SAW0000_000: ns = (DP_IN==1'b0 && DM_IN==1'b1) ? SAW0000_0001 : NOTHING;
            // Here, we have seen an input of 0000_0001
            SAW0000_0001: ns = (DP_IN==1'b0 && DM_IN==1'b0) ? SAWX: SAW0000_0001;
			SAWX: ns = (DP_IN==1'b0 && DM_IN==1'b0) ? SAWXX: SAWX;
			SAWXX: ns = (DP_IN==1'b1 && DM_IN==1'b0) ? DONE: SAWXX;
			// EOP(XXJ) has been detected
            DONE: ns = (pkt_in_taken) ? NOTHING : DONE;
        endcase
    end

   always_comb // Output logic
        case(cs)
            NOTHING: begin
                       start_in = 1'b0;
                       foundEOP = 1'b0;
                     end
            SAW00: begin
                     start_in = 1'b0;
                     foundEOP = 1'b0;
                   end
            SAW000: begin
                      start_in = 1'b0;
                      foundEOP = 1'b0;
                    end
            SAW0000: begin
                       start_in = 1'b0;
                       foundEOP = 1'b0;
                     end
            SAW0000_0: begin
                         start_in = 1'b0;
                         foundEOP = 1'b0;
                    end
            SAW0000_00: begin
                          start_in = 1'b0;
                          foundEOP = 1'b0;
                        end
            SAW0000_000: begin
                           start_in = (DP_IN==1'b0 && DM_IN==1'b1) ? 1'b1 : 1'b0;
                           foundEOP = 1'b0;
                         end
            SAW0000_0001: begin
                            start_in = 1'b0;
                            foundEOP = 1'b0;
                          end
            SAWX: begin
                    start_in = 1'b0;
                    foundEOP = 1'b0;
                  end
            SAWXX: begin
                     start_in = 1'b0;
                     foundEOP = (DP_IN==1'b1 && DM_IN==1'b0) ? 1'b1 : 1'b0;
                   end
            DONE: begin
                    start_in = 1'b0;
                    foundEOP = 1'b0;
                  end
        endcase

    assign DP_IN = (!enable_OUT) ? wires.DP : 1'b1;
    assign DM_IN = (!enable_OUT) ? wires.DM : 1'b0;
    assign diff_in = DP_IN; // Wait until we see 0000_0001(SYNC)
    assign wires.DP = (oneBitStart) ? 1'b0 : // First must be K
					  (!enable_OUT) ? 1'bz :
                      ((processing==0 && counter==3'd0) || counter==3'd1) ? 1'b0: // XX
                      (counter==3'd2) ? 1'b1: // J
                      (diff==1'b0) ? 1'b0:
                      (diff==1'b1) ? 1'b1: 1'b0;

    assign wires.DM = (oneBitStart) ? 1'b1 : // First must be K
					  (!enable_OUT) ? 1'bz :
                      ((processing==0 && counter==3'd0) || counter==3'd1) ? 1'b0: // XX
                      (counter==3'd2) ? 1'b0: // J
                      (diff==1'b0) ? 1'b1:
                      (diff==1'b1) ? 1'b0: 1'b1;

endmodule: DPDM


/********************************
 ******Decoding Pipeline*********
 ********************************/

/**
 * Differential values(diff_in) are decoded in normal one bit(stuffedOneBit).
 */
module NRZI_in
    (input  logic clk, rst_L, diff_in,
     output logic stuffedOneBit_in);

    enum logic [1:0] {J_STATE=2'b01, K_STATE=2'b10} cs, ns;
    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) cs <= J_STATE;
        else cs <= ns;
    end

    always_comb begin // output logic
        ns = J_STATE;
        stuffedOneBit_in = 0;
        case(cs)
            K_STATE: begin 
                ns = (diff_in) ? J_STATE : K_STATE;
                stuffedOneBit_in = (diff_in==1'b1) ? 1'b0 : 1'b1;
            end
            J_STATE: begin
                ns = (diff_in) ? J_STATE : K_STATE;
                stuffedOneBit_in = (diff_in==1'b1) ? 1'b1 : 1'b0;
            end
        endcase
    end

endmodule: NRZI_in

/**
 * Possibly stuffed one bit is unstuffed. When it sees six consecutive 1's,
 * it assert stuffing_in signal so that BitStreamDecoder would do "nothing" in
 * that clock cycle.
 */
module BitUnstuffing
    (input  logic clk, stuffedOneBit_in, start_in, pkt_in_taken,
     output logic unstuffedOneBit, stuffing_in);

    enum logic [2:0] {INIT=3'b000, SAW1=3'b001, SAW11=3'b010, SAW111=3'b011, SAW1111=3'b100,
                    SAW11111=3'b101, SAW111111=3'b110, NOTHING=3'b111} cs, ns;

    logic [6:0] counter;
    always_ff @(posedge clk) begin // Next State logic
        if (start_in) begin
            cs <= INIT;
            counter <= 7'd0;
        end
        else begin
			if(!pkt_in_taken) begin
                if (counter < 7'd7) begin // PID
                    cs <= INIT;
                    counter <= counter + 7'd1;
                end
                else if (counter==7'd7) begin
                    cs <= NOTHING; // Bit Stuffing starts
                    counter <= counter + 7'd1;
                end
                else begin
                    cs <= ns;
                    counter <= counter + 7'd1;
                end
            end
        end
    end

    always_comb begin // Output logic
        if (start_in) begin
            unstuffedOneBit = stuffedOneBit_in;
            stuffing_in = 1'b0;
        end
        else begin
            if (!pkt_in_taken) begin
                unstuffedOneBit = (cs==SAW111111) ? 1'b0 : stuffedOneBit_in; 
				// When cs==SAW111111, unstuffedOneBit is assigned to 1'b0, but it 
				// can be any random value. Decoder won't take this bit anyway.
                stuffing_in = (cs==SAW111111) ? 1'b1 : 1'b0;
            end
            else begin
                unstuffedOneBit = stuffedOneBit_in;
                stuffing_in = 1'b0;
            end
        end
    end

    always_comb
        case(cs)
            INIT: ns = INIT;
            NOTHING: ns = (stuffedOneBit_in==1'b1) ? SAW1 : NOTHING;
            SAW1: ns = (stuffedOneBit_in==1'b1) ? SAW11 : NOTHING;
            SAW11: ns = (stuffedOneBit_in==1'b1) ? SAW111 : NOTHING;
            SAW111: ns = (stuffedOneBit_in==1'b1) ? SAW1111 : NOTHING;
            SAW1111: ns = (stuffedOneBit_in==1'b1) ? SAW11111 : NOTHING;
            SAW11111: ns = (stuffedOneBit_in==1'b1) ? SAW111111 : NOTHING;
            SAW111111: ns = NOTHING; // Goes back to "saw nothing" state
        endcase
endmodule: BitUnstuffing

/**
 * BitStreamDecoder collects serial bit from the device and "pack" in an appropriate
 * packet form and send it to protocol fsm. It also tests whether the packet has been
 * corrupted or not with CRC.
 * Wait for the first 8 clock cycles to get the PID of the packet, and then decoder
 * decides which procedures it should follow.
 */
module BitStreamDecoder
    (input  logic clk, rst_L, start_in, stuffing_in, unstuffedOneBit, pkt_in_taken, foundEOP,
	 output pkt_t pkt_in,
	 output logic corrupt, pkt_in_avail);

    logic [6:0] counter;
    logic [4:0] poly5;
    logic [15:0] poly16;
    logic [31:0] value32_in;
    logic [95:0] value96_in;
    logic [15:0] value16_in;
	logic enableTOKEN, enableDATA, enableHANDSHAKE;

	logic [7:0] pid;
	logic [63:0] data;
	logic [63:0] r_data;
	logic [6:0] addr;
	logic [6:0] r_addr;
	logic [3:0] endp;
	logic [3:0] r_endp;
	logic [4:0] crc5;
	logic [15:0] crc16;
	integer a,b,c;

	always_ff @(posedge clk, negedge rst_L) begin
		if(~rst_L) begin
			value32_in <= 32'd0;
			value96_in <= 96'd0;
			value16_in <= 16'd0;
			pid <= 8'd0;
		end
		else if(start_in) begin // PID
			counter<=7'd0;
			poly5<=5'b11111;
			poly16<=16'b11111111_11111111;
		end
		else begin

			if(counter < 7'd8) begin
				pid <= {pid[6:0],unstuffedOneBit};
				counter<=counter+7'd1;
			end
			if(stuffing_in) begin
				//nothing happens
			end
			else begin // stuffing_in==0

				/////////
				//TOKEN//
				/////////
				if(enableTOKEN) begin
					if(counter==7'd8) begin
						value32_in = {15'd0,8'b00000001,pid,unstuffedOneBit};
						poly5 <= (poly5[4]^unstuffedOneBit ? 5'b00101 : 5'b00000) ^ {poly5[3:0],1'b0};
						counter<=counter+7'd1;
					end
					else if(counter<7'd19) begin // ADDR, ENDP
						value32_in <= {value32_in[30:0],unstuffedOneBit};
						poly5 <= (poly5[4]^unstuffedOneBit ? 5'b00101 : 5'b00000) ^ {poly5[3:0],1'b0};
						counter<=counter+7'd1;
					end
					else if(counter<7'd24) begin // CRC5
						value32_in <= {value32_in[30:0],unstuffedOneBit};
						poly5 <= (poly5[4]^unstuffedOneBit ? 5'b00101 : 5'b00000) ^ {poly5[3:0],1'b0};
						counter<=counter+7'd1;
					end
					else if(counter==7'd24) begin
						if(foundEOP) begin // WAIT UNTIL EOP IS FOUND
							counter<=counter+7'd1;
						end
					end
					else counter <= counter;
				end

				////////
				//DATA//
				////////
				else if(enableDATA) begin
					if(counter==7'd8) begin
						value96_in = {79'd0,8'b00000001,pid,unstuffedOneBit};
						poly16 <= (poly16[15]^unstuffedOneBit ? 16'b10000000_00000101 : 16'b0) ^ {poly16[14:0],1'b0};
						counter<=counter+7'd1;
					end
					else if(counter<7'd72) begin // DATA
						value96_in = {value96_in[94:0],unstuffedOneBit};
						poly16 <= (poly16[15]^unstuffedOneBit ? 16'b10000000_00000101 : 16'b0) ^ {poly16[14:0],1'b0};
						counter<=counter+7'd1;
					end
					else if(counter<7'd88) begin // CRC16
						value96_in = {value96_in[94:0],unstuffedOneBit};
						poly16 <= (poly16[15]^unstuffedOneBit ? 16'b10000000_00000101 : 16'b0) ^ {poly16[14:0],1'b0};
						counter<=counter+7'd1;
					end
					else if(counter==7'd88)begin
						if(foundEOP) begin // WAIT UNTIL EOP IS FOUND
							counter<=counter+7'd1;
						end
					end
					else counter <= counter;

				end

				/////////////
				//HANDSHAKE//
				/////////////
				else if(enableHANDSHAKE) begin
					if(counter==7'd8) begin
						counter <= counter+7'd1;
					end
					else if(counter==7'd9) begin
						if(foundEOP) begin // WAIT UNTIL EOP IS FOUND
							counter <= counter+7'd1;
						end
					end
					else counter <= counter;
				end
			end
		end
	end

	always_comb begin
		if(~rst_L) begin
			corrupt = 1'b0;
			pkt_in_avail = 0;
			pkt_in = 112'b0;
			data = 64'b0;
			addr = 7'b0;
			endp = 4'b0;
		end
		else begin
			if(counter==7'd8) begin
				enableTOKEN = (pid==8'b1000_0111 || pid== 8'b1001_0110);
				enableDATA = (pid==8'b1100_0011);
				enableHANDSHAKE = (pid==8'b0100_1011 || pid==8'b0101_1010);
			end
			else begin
				if(stuffing_in) begin
					// nothing happens
				end
				else begin
					/////////
					//TOKEN//
					/////////
					if(enableTOKEN) begin
						if(counter==7'd24) begin
							addr = value32_in[15:9];
							endp = value32_in[8:5];
							crc5 = poly5;
 					        for (a = 0; a < 7; a++) r_addr[a] = data[6-a];
							for (b = 0; b < 4; b++) r_endp[b] = endp[3-b];
							if(foundEOP) begin // WAIT UNTIL EOP IS FOUND
								corrupt = (poly5!=5'b01100);
								// poly5 should be this value at this point!
								pkt_in = {8'b00000001,pid,64'b0,r_addr,r_endp,crc5,16'b0};
								if(!corrupt) begin
									pkt_in_avail = 1'b1;
									// Asserted only when not corrupted
								end
							end
						end
						else if (counter>7'd24)begin
							corrupt = 1'b0;
							pkt_in_avail = 1'b0;
							enableTOKEN = 1'b0;
						end
					end

					////////
					//DATA//
					////////
					else if(enableDATA) begin
						if(counter==7'd88) begin
							data = value96_in[79:16];
							crc16 = poly16;
 					        for (c = 0; c < 64; c++) r_data[c] = data[63-c];
							if(foundEOP) begin // WAIT UNTIL EOP IS FOUND
								corrupt = (poly16!=16'h800d); 
								// poly16 should be this value at this point!
								pkt_in = {8'b00000001,pid,r_data,16'd0,crc16};
								if(!corrupt) begin
									pkt_in_avail = 1'b1;
									// Asserted only when not corrupted
								end
							end
						end
						else if(counter>7'd88) begin
							corrupt = 1'b0;
							pkt_in_avail = 1'b0;
							enableDATA = 1'b0;
						end
					end

					////////////
					//HANDSHAKE//
					/////////////
					else if(enableHANDSHAKE)begin
						if(counter==7'd9) begin
							if(foundEOP) begin // WAIT UNTIL EOP IS FOUND
								pkt_in = {8'b0000_0001,pid,96'b0};
								pkt_in_avail = 1'b1;
							end
						end
						else if (counter>7'd9) begin
							pkt_in_avail = 1'b0;
							enableHANDSHAKE = 1'b0;
						end
					end
				end
			end
		end
	end

endmodule: BitStreamDecoder

//////////////////////
// END DJ PARK PART //
//////////////////////



