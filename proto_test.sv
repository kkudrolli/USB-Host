/*
 * proto_test.sv
 *
 * Author:     Kais Kudrolli
 * Andrew IDs: kkudroll
 *
 * File that contains a testbench to test the Protocol FSM.
 */

typedef struct packed {
    logic [7:0]  sync;
    logic [7:0]  pid;
    logic [63:0] data;
    logic [6:0]  addr;
    logic [3:0]  endp;
    logic [4:0]  crc5;
    logic [15:0] crc16;
} pkt_t;

module proto_tb;

    // Inputs to dut
    logic        clk, rst_L, trans_avail, pkt_out_taken, corrupt, pkt_in_avail;
    logic [3:0]  pid, endp;
    logic [6:0]  addr;
    logic [63:0] data0;
    pkt_t        pkt_in;
    // Outputs of dut
    logic        trans_taken, pkt_out_avail, success, pkt_in_taken;
    logic [63:0] data_read;
    pkt_t        pkt_out;

    protocol_fsm dut(.*);
    
    initial begin
        rst_L = 0;
        clk = 0;
        rst_L <= #1 1;
        forever #10 clk = ~clk;
    end

    initial begin
        trans_avail <= 0;
        pkt_out_taken <= 0;
        corrupt <= 0;
        pkt_in_avail <= 0;
        pid <= 4'd0;
        endp <= 4'd0;
        addr <= 7'd0;
        data0 <= 64'd0;
        pkt_in <= 'd0;
        
        @(posedge clk);
        assert (dut.ns.name == "trans")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (~pkt_out_avail);
        assert (~trans_taken);
        assert (~success);
        assert (~pkt_in_taken);

        trans_avail <= 1; pid <= 4'b1001; endp <= 4'd8; addr <= 7'd5;
 
        @(posedge clk);
        assert (dut.ns.name == "in")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (pkt_out_avail);

        pkt_in_avail <= 1; 

        @(posedge clk);
        assert (dut.ns.name == "ack_read")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (pkt_out_avail);
        assert (dut.pkt.name == "ACK")
            else $fatal("%m pkt = %s", dut.pkt.name);

        pkt_in_avail <= 0;
        
        @(posedge clk);
        assert (dut.ns.name == "ack_read")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (dut.pkt.name == "NONE")
            else $fatal("%m pkt = %s", dut.pkt.name);

        pkt_out_taken <= 1; pkt_in.data <= 64'd800;

        @(posedge clk);
        assert (dut.ns.name == "trans")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (~pkt_out_avail);
        assert (~dut.ack);
        assert (success);
        assert (trans_taken);
        assert (data_read == 64'd800) 
            else $fatal("%m data_read = %d", data_read);

        pkt_out_taken <= 0; pkt_in_avail <= 1; pid <= 4'b0001;
        
        @(posedge clk);
        assert (dut.ns.name == "out")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (pkt_out_avail);
        assert (pkt_out.pid == 8'b1000_0111) 
            else $fatal("%m pid = %b", pkt_out.pid);
        assert (pkt_out.addr == 7'b1010000) 
            else $fatal("%m addr = %b", pkt_out.addr);
        assert (pkt_out.endp == 4'b0001) 
            else $fatal("%m endp = %b", pkt_out.endp);

        pkt_in_avail <= 0; pkt_out_taken <= 1; 
 
        @(posedge clk);
        assert (dut.ns.name == "w_data")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (pkt_out_avail);
        assert (dut.pkt.name == "DATA0")
            else $fatal("%m pkt = %s", dut.pkt.name);
        
        pkt_out_taken <= 0; pkt_in.pid <= 8'b0101_1010; pkt_in_avail <= 1;

        @(posedge clk);
        assert (dut.ns.name == "w_data")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (dut.nak);
        assert (pkt_out_avail);
        assert (dut.pkt.name == "DATA0")
            else $fatal("%m pkt = %s", dut.pkt.name);
        assert (pkt_in_taken);

        pkt_in.pid <= 8'b0100_1011;
       
        @(posedge clk);
        assert (dut.ns.name == "trans")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (dut.ack);
        assert (trans_taken);
        assert (success);
        assert (pkt_in_taken);

        $display("\n\nAll tests passed!\n\n");
        $finish;
    end

endmodule: proto_tb

/*
 * FSM that controls takes transactions from the R/W FSM and translates them
 * to packets sent to the bit stream encoder. It also takes in packets from 
 * the bit stream decoder. FSM uses mealy style.
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

    enum logic [2:0] {trans, out, in, ack_read, w_data} cs, ns;
    enum logic [2:0] {NONE, OUT, IN, DATA0, ACK, NAK}   pkt; 

    logic       inc_corrupt_cnt, inc_time_cnt, inc_cyc_cnt;
    logic       clr_corrupt_cnt, clr_time_cnt, clr_cyc_cnt;
    logic       ack, nak;
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
        end
        else begin
            cs <= ns;
            cycles_count <= (inc_cyc_cnt) ? cycles_count + 8'd1 : 
                           ((clr_cyc_cnt) ? 8'd0 : cycles_count);
            corrupt_count <= (inc_corrupt_cnt) ? corrupt_count + 4'd1 : 
                            ((clr_corrupt_cnt) ? 4'd0 : corrupt_count);
            timeout_count <= (inc_time_cnt) ? timeout_count + 4'd1 : 
                            ((clr_time_cnt) ? 4'd0 : timeout_count);
        end
    end

    always_comb begin
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
        data_read = 64'd0;
        case (cs) 
            trans: begin
                ns = (trans_avail) ? ((pid == 4'b1001) ? in : out) : trans;
                pkt_out_avail = (trans_avail) ? 1 : 0;
                clr_corrupt_cnt = 1;
                clr_time_cnt = 1;
                clr_cyc_cnt = 1;
                pkt = (trans_avail) ? ((pid == 4'b1001) ? IN : OUT) : NONE;
            end
            out: begin
                ns = (pkt_out_taken) ? w_data : out;
                pkt_out_avail = (pkt_out_taken) ? 1 : 0;
                pkt = DATA0;
            end
            in: begin
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
            end
            ack_read: begin
                ns = (pkt_out_taken) ? trans : ack_read;
                success = (pkt_out_taken) ? 1 : 0;
                trans_taken = (pkt_out_taken) ? 1 : 0;
                data_read = (pkt_out_taken) ? pkt_in.data : 64'd0;
            end
            w_data: begin
                ns = ((pkt_in_avail && ack) || corrupt_count == 4'd8 || timeout_count == 4'd8) ? trans : w_data; 
                trans_taken = ((pkt_in_avail && ack) || corrupt_count == 4'd8 || timeout_count == 4'd8) ? 1 : 0;
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
        case (pkt) 
            // Pid, reversed and complemented in here not outside
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
                pkt_out.endp = r_data0;
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
