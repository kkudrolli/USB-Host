# USB-Host
Contains the System Verilog description for a simplified USB host that implements the transaction, data-link, and physical layers of the bus.

Makefile - file that contains different compile options for the project.
TA_tb_simple.svp - encrypted testbench for the usb host.
TA_tb_faults.svp - encrypted testbench that tests error checking and retries.
TA_tb_full.svp - encrypted testbench that test all usb host operations.
tb.sv - testbench to drive simple usb operations.
proto_test.sv - tests the protocol fsm separately.
rw_test.sv - tests the read/write fsm separately.
top.sv - file containing usb interface definition and connections.
usbHost.sv - the System Verilog description of the usb host.
usbBusAnalyzer.svp - converts the differential signals on the usb interface into more readable outputs.
thumb.sv.e - model for a usb device.
thumb_faulty.sv.e - model for a faulty usb device.

