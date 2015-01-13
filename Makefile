CC=vcs

FLAGS=-sverilog -debug -assert filter -assert enable_diag

default: full

student: top.sv usbBusAnalyzer.svp tb.sv usbHost.sv thumb.sv.e 
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp tb.sv usbHost.sv thumb.sv.e

custom: top.sv usbBusAnalyzer.svp tb.sv testUsb.sv thumb.sv.e 
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp tb.sv testUsb.sv thumb.sv.e

simple: top.sv usbBusAnalyzer.svp TA_tb_simple.svp usbHost.sv thumb.sv.e
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp TA_tb_simple.svp usbHost.sv thumb.sv.e

full: top.sv usbBusAnalyzer.svp TA_tb_full.svp usbHost.sv thumb.sv.e
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp TA_tb_full.svp usbHost.sv thumb.sv.e

faulty: top.sv usbBusAnalyzer.svp TA_tb_faults.svp usbHost.sv thumb_faulty.sv.e
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp TA_tb_faults.svp usbHost.sv thumb_faulty.sv.e

prelab: top.sv usbBusAnalyzer.svp prelab_tb.svp usbHost.sv prelab_thumb.sv.e
	$(CC) $(FLAGS) top.sv usbBusAnalyzer.svp prelab_tb.svp usbHost.sv prelab_thumb.sv.e

clean:
	rm -rf simv
	rm -rf simv.daidir
	rm -rf csrc
	rm -rf ucli.key
	rm -rf simv.vdb
	rm -rf DVEfiles
	rm -rf inter.vpd
