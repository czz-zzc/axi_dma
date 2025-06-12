all:clr vcs sim verdi

run:vcs sim

filelist:
	rf -rf filelist.f
	find ../rtl/src ../tb -name "*.v" -o -name "*.sv" > filelist.f

vcs:
	vcs -f filelist.f \
        -timescale=1ns/1ps \
        -full64 +v2k -sverilog -Mupdate +define+DUMP_FSDB \
        -debug_acc+all -debug_region+cell+encrypt \
        |tee vcs.log

sim:
	./simv |tee sim.log

verdi:
	verdi -sv -f filelist.f -ssf tb_top.fsdb -play verdi.tcl &

clr:
	rm -rf csrc novas* *.log simv* *fsdb* ucli.key *.vpd verdiLog
