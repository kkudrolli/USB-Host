/*
 * tb.sv
 *
 * Authors:    Kais Kudrolli  DJ Park
 * Andrew IDs: kkudroll       dongjoop
 *
 * File that contains a testbench to test the usbHost.sv.
 */

module test
    (output logic clk, rst_L);

    logic        success;
    logic [63:0] data_read;

    class args;
        rand bit [15:0] memPage;
        rand bit [63:0] data_write;
    endclass: args

    //Set up clk, rst_L, and then call your usbHost.sv tasks here  

    // Timeout
    initial begin
        #1000 $finish;
    end

    initial begin
        clk = 0;
        rst_L = 0;
        rst_L <= #10 1;
        forever #10 clk = ~clk;
    end

    //Ex: host.prelabRequest(data);
    //    host.writeData(memPage, data, success);
    //    host.readData(memPage, data, success);  

    initial begin
        args t_args = new;

        if (t_args.randomize()) begin
            host.writeData(t_args.memPage, t_args.data_write, success);
            host.readData(t_args.memPage, data_read, success);
        end
        else $display($stime, "Bad random numbers!");
        
        #500 $finish;
    end

endmodule: test
