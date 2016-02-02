################################################################################
# MAVRIC TRAVIS MAKEFILE
################################################################################

# ------------------------------------------------------------------------------
# MAKEFILE RULES
# ------------------------------------------------------------------------------

# Main rule
all: sample_projects doc	

sample_projects: LEQuad_linux LEQuad_stm32

LEQuad_linux:
	cd ../sample_projects/LEQuad/proj_linux && make clean && make && make clean

LEQuad_avr32:
	cd ../sample_projects/LEQuad/proj_avr32 && make clean && make && make clean

LEQuad_stm32:
	cd ../sample_projects/LEQuad/proj_stm32 && make clean && make && make clean

doc: doc_doxygen doc_daux

gh_pages:
	mkdir -p ../doc/gh_pages

doc_doxygen: gh_pages
	cd ../doc && doxygen > /dev/null 2> /dev/null

doc_daux: gh_pages
	cd ../doc/daux.io && ./daux.phar && cp -r static/* ../gh_pages