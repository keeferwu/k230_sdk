all:
	@$(shell if [ ! -d "elf" ]; then mkdir elf; fi)
	@cd sample_vicap_100ask; make || exit 1
	@cd sample_vo_mipi; make || exit 1
	@cd sample_vivo_mipi; make || exit 1
	
clean:
	@cd sample_vicap_100ask; make clean
	@cd sample_vo_mipi; make clean
	@cd sample_vivo_mipi; make clean
	

