################################################################################
# MAVRIC TRAVIS MAKEFILE
################################################################################

# ------------------------------------------------------------------------------
# MAKEFILE RULES
# ------------------------------------------------------------------------------

# Main rule
all: sample_projects doc	

sample_projects: LEQuad_linux

LEQuad_linux:
	cd ../sample_projects/LEQuad/proj_linux && make clean && make && make clean

LEQuad_avr32:
	cd ../sample_projects/LEQuad/proj_avr32 && make clean && make && make clean

LEQuad_stm32:
	cd ../sample_projects/LEQuad/proj_stm32 && make clean && make && make clean

doc: gh_pages doc_doxygen doc_daux

gh_pages:
	mkdir -p ../doc/gh_pages

doc_doxygen:
	cd ../doc && mkdir -p gh_pages && doxygen > /dev/null 2> /dev/null

doc_daux:
	@cd ../doc/daux.io && ./daux.phar && mkdir -p ../gh_pages && cp -r static/* ../gh_pages