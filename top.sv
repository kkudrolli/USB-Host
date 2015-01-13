/*
 * top.sv
 *
 * Authors:    Kais Kudrolli  DJ Park
 * Andrew IDs: kkudroll       dongjoop 
 *
 * File that contains the interface to the Universal Serial Bus and
 * the necessary module connections.
 *
 */

interface usbWires;

    tri0 DP;
    tri0 DM;

endinterface

module top;

  logic clk, rst_L;
    
  usbWires wires();
  usbDevice #(7'd5) thumbDrive(.*);
  usbHost host(.*);
  usbBusAnalyzer analyzer(.*);
  test tb(.*);

endmodule: top

