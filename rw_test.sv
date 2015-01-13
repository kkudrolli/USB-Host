/*
 * rw_test.sv
 *
 * Author:     Kais Kudrolli
 * Andrew IDs: kkudroll
 *
 * File that contains a testbench to test the Read/Write FSM.
 */

module rw_tb;
    
    // Inputs to dut
    logic        clk, rst_L, task_avail, trans_taken, success_in;
    logic [1:0]  id;
    logic [15:0] mempage;
    logic [63:0] data_in, data_read;
    // Outputs of dut
    logic        trans_avail, task_taken, success_out;
    logic [3:0]  pid, endp;
    logic [6:0]  addr;
    logic [63:0] data_field, data_out;

    readWriteFsm dut(.*);
    
    initial begin
        rst_L = 0;
        clk = 0;
        rst_L <= #1 1;
        forever #10 clk = ~clk;
    end

    initial begin
        trans_taken <= 0; success_in <= 0; 
        mempage <= 16'd80; data_in <= 64'd0; data_read <= 64'd0;
        task_avail <= 0; id <= 2'b00;
        @(posedge clk);
        assert (dut.ns.name == "init")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (~trans_avail); 
        assert (~task_taken); 
        assert (~success_out); 
        assert (pid == 4'd0);
        assert (endp == 4'd0);
        assert (addr == 7'd0);
        assert (data_field == 64'd0);
        
        task_avail <= 1; id <= 2'b01;

        @(posedge clk);
        assert (dut.ns.name == "out")
            else $fatal("%m cs = %s, ns = %s", dut.cs.name, dut.ns.name);
        assert (trans_avail);
        assert (pid == 4'b0001)
            else $fatal("%m pid = %b", pid);
        assert (endp == 4'd4)
            else $fatal("%m endp = %d", endp);
        assert (addr == 7'd5)
            else $fatal("%m addr = %d", addr);
        assert (data_field == {4'd0, mempage})
            else $fatal("%m data_field = %d", data_field);
        
        @(posedge clk);
        assert (dut.ns.name == "out")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (trans_avail);
        assert (pid == 4'b0001)
            else $fatal("%m pid = %b", pid);
        assert (endp == 4'd4)
            else $fatal("%m endp = %d", endp);
        assert (addr == 7'd5)
            else $fatal("%m addr = %d", addr);
        assert (data_field == {4'd0, mempage})
            else $fatal("%m data_field = %d", data_field);

        trans_taken <= 1; id <= 2'b01;
        
        @(posedge clk);
        assert (dut.ns.name == "read")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (trans_avail);
        assert (pid == 4'b1001)
            else $fatal("%m pid = %b", pid);
        assert (endp == 4'd8)
            else $fatal("%m endp = %d", endp);
        assert (addr == 7'd5)
            else $fatal("%m addr = %d", addr);
        
        trans_taken <= 0; id <= 2'b01;
        
        @(posedge clk);
        assert (dut.ns.name == "read")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (trans_avail);
        assert (pid == 4'b1001)
            else $fatal("%m pid = %b", pid);
        assert (endp == 4'd8)
            else $fatal("%m endp = %d", endp);
        assert (addr == 7'd5)
            else $fatal("%m addr = %d", addr);

        trans_taken <= 1; data_read <= 64'd90; success_in <= 1;
        
        @(posedge clk);
        assert (dut.ns.name == "init")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (~trans_avail);
        assert (trans_taken);
        assert (data_out == 64'd90)
            else $fatal("%m data_out = %d", data_out);
        assert (success_out == 1)

        // Finished read, do a write 
        task_avail <= 0; id <= 2'b00; trans_taken <= 0;
        @(posedge clk);
        assert (dut.ns.name == "init")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (~trans_avail); 
        assert (~task_taken); 
        assert (pid == 4'd0);
        assert (endp == 4'd0);
        assert (addr == 7'd0);
        assert (data_field == 64'd0);
        
        @(posedge clk);
        assert (dut.ns.name == "init")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (~trans_avail); 
        assert (~task_taken); 

        task_avail <= 1; id <= 2'b10;

        @(posedge clk);
        assert (dut.ns.name == "out")
            else $fatal("%m cs = %s, ns = %s", dut.cs.name, dut.ns.name);
        assert (trans_avail);
        
        @(posedge clk);
        assert (dut.ns.name == "out")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (trans_avail);

        trans_taken <= 1; id <= 2'b10; data_in <= 64'd400;
        
        @(posedge clk);
        assert (dut.ns.name == "write")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (trans_avail);
        assert (pid == 4'b0001)
            else $fatal("%m pid = %b", pid);
        assert (endp == 4'd8)
            else $fatal("%m endp = %d", endp);
        assert (addr == 7'd5)
            else $fatal("%m addr = %d", addr);
        assert (data_field == 64'd400)
            else $fatal("%m data_field = %d", data_field); 

        trans_taken <= 0;
        
        @(posedge clk);
        assert (dut.ns.name == "write")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (trans_avail);
        assert (pid == 4'b0001)
            else $fatal("%m pid = %b", pid);
        assert (endp == 4'd8)
            else $fatal("%m endp = %d", endp);
        assert (addr == 7'd5)
            else $fatal("%m addr = %d", addr);
        assert (data_field == 64'd400)
            else $fatal("%m data_field = %d", data_field); 

        trans_taken <= 1; success_in <= 1;
        
        @(posedge clk);
        assert (dut.ns.name == "init")
            else $fatal("%m ns = %s", dut.ns.name);
        assert (~trans_avail);
        assert (task_taken);
        assert (data_field == 64'd0)
            else $fatal("%m data_field = %d", data_field); 
        assert (success_out);

        $display("\n\nAll tests passed!\n\n");
        $finish;
    end

endmodule: rw_tb

/*
 * Mealy style FSM that translates task calls to transactions that are 
 * sent to the protocol FSM.
 */
module readWriteFsm
    (input  logic        clk, rst_L, task_avail, trans_taken, success_in,
     input  logic [1:0]  id,
     input  logic [15:0] mempage,
     input  logic [63:0] data_in, data_read,
     output logic        trans_avail, task_taken, success_out,
     output logic [3:0]  pid, endp,
     output logic [6:0]  addr, 
     output logic [63:0] data_field, data_out);

    enum logic [2:0] {init, out, read, write} cs, ns;

    always_ff @(posedge clk, negedge rst_L) begin
        if (~rst_L) cs <= init;
        else cs <= ns;
    end

    assign data_out = (task_taken) ? data_read : 64'd0;
    assign success_out = (task_taken) ? success_in : 0;

    always_comb begin 
        ns = init;
        trans_avail = 0;
        task_taken = 0;
        pid = 4'd0;
        addr = 7'd0;
        endp = 4'd0;
        data_field = 64'd0;
        case (cs)
            /*
             * IDs:
             *     2'b01 -> read task
             *     2'b10 -> write task
             */
            init: begin
                ns = (task_avail) ? out : init;
                trans_avail = (task_avail) ? 1 : 0;
                pid = (task_avail) ? 4'b0001 : 4'b0000;
                addr = (task_avail) ? 7'd5 : 7'd0;
                endp = (task_avail) ? 4'd4 : 4'd0;
                data_field = (task_avail) ? {48'd0, mempage} : 64'd0;
            end
            out: begin
                ns = (trans_taken) ? ((id == 2'b01) ? read : write) : out;
                trans_avail = 1;
                pid = (trans_taken) ? ((id == 2'b01) ? 4'b1001 : 4'b0001) : 4'b0001;
                addr = 7'd5;
                endp = (trans_taken) ? 7'd8 : 7'd4; 
                data_field = (trans_taken) ? ((id == 2'b01) ? 64'd0 : data_in) : {48'd0, mempage}; 
            end
            read: begin
                ns = (trans_taken) ? init : read;
                trans_avail = (trans_taken) ? 0 : 1;
                task_taken = (trans_taken) ? 1 : 0;
                pid = (trans_taken) ? 4'b0000 : 4'b1001;
                addr = (trans_taken) ? 7'd0 : 7'd5;
                endp = (trans_taken) ? 4'd0 : 4'd8;
            end
            write: begin
                ns = (trans_taken) ? init : write;
                trans_avail = (trans_taken) ? 0 : 1;
                task_taken = (trans_taken) ? 1 : 0;
                pid = (trans_taken) ? 4'b0000 : 4'b0001;
                addr = (trans_taken) ? 7'd0 : 7'd5;
                endp = (trans_taken) ? 4'd0 : 4'd8;
                data_field = (trans_taken) ? 64'd0 : data_in;
            end
        endcase
    end
endmodule: readWriteFsm
