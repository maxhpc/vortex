ROOT_DIR := $(realpath ../..)
include $(ROOT_DIR)/config.mk

all:
	$(MAKE) -C vecadd
	$(MAKE) -C sgemm
	$(MAKE) -C conv3
	$(MAKE) -C psort
	$(MAKE) -C saxpy
	$(MAKE) -C sfilter
	$(MAKE) -C oclprintf
	$(MAKE) -C dotproduct
	$(MAKE) -C transpose
	$(MAKE) -C spmv
	$(MAKE) -C stencil
	$(MAKE) -C lbm
	$(MAKE) -C nearn
	$(MAKE) -C guassian
	$(MAKE) -C kmeans
	$(MAKE) -C blackscholes
	$(MAKE) -C bfs

run-simx:
	$(MAKE) -C vecadd run-simx
	$(MAKE) -C sgemm run-simx
	$(MAKE) -C conv3 run-simx
	$(MAKE) -C psort run-simx
	$(MAKE) -C saxpy run-simx
	$(MAKE) -C sfilter run-simx
	$(MAKE) -C oclprintf run-simx
	$(MAKE) -C dotproduct run-simx
	$(MAKE) -C transpose run-simx
	$(MAKE) -C spmv run-simx
	$(MAKE) -C stencil run-simx
	$(MAKE) -C lbm run-simx
	$(MAKE) -C nearn run-simx
	$(MAKE) -C guassian run-simx
	$(MAKE) -C kmeans run-simx
#	$(MAKE) -C blackscholes run-simx
	$(MAKE) -C bfs run-simx

run-rtlsim:
	$(MAKE) -C vecadd run-rtlsim
	$(MAKE) -C sgemm run-rtlsim
	$(MAKE) -C conv3 run-rtlsim
	$(MAKE) -C psort run-rtlsim
	$(MAKE) -C saxpy run-rtlsim
	$(MAKE) -C sfilter run-rtlsim
	$(MAKE) -C oclprintf run-rtlsim
	$(MAKE) -C dotproduct run-rtlsim	
	$(MAKE) -C transpose run-rtlsim
	$(MAKE) -C spmv run-rtlsim
	$(MAKE) -C stencil run-rtlsim
	$(MAKE) -C lbm run-rtlsim
	$(MAKE) -C nearn run-rtlsim
	$(MAKE) -C guassian run-rtlsim
	$(MAKE) -C kmeans run-rtlsim
#	$(MAKE) -C blackscholes run-rtlsim
	$(MAKE) -C bfs run-rtlsim

run-opae:
	$(MAKE) -C vecadd run-opae
	$(MAKE) -C sgemm run-opae
	$(MAKE) -C conv3 run-opae
	$(MAKE) -C psort run-opae
	$(MAKE) -C saxpy run-opae
	$(MAKE) -C sfilter run-opae
	$(MAKE) -C oclprintf run-opae
	$(MAKE) -C dotproduct run-opae
	$(MAKE) -C transpose run-opae
	$(MAKE) -C spmv run-opae
	$(MAKE) -C stencil run-opae
	$(MAKE) -C lbm run-opae
	$(MAKE) -C nearn run-opae
	$(MAKE) -C guassian run-opae
	$(MAKE) -C kmeans run-opae
#	$(MAKE) -C blackscholes run-opae
	$(MAKE) -C bfs run-opae

clean:
	$(MAKE) -C vecadd clean
	$(MAKE) -C sgemm clean
	$(MAKE) -C conv3 clean
	$(MAKE) -C psort clean
	$(MAKE) -C saxpy clean
	$(MAKE) -C sfilter clean
	$(MAKE) -C oclprintf clean
	$(MAKE) -C dotproduct clean
	$(MAKE) -C transpose clean
	$(MAKE) -C spmv clean
	$(MAKE) -C stencil clean
	$(MAKE) -C lbm clean
	$(MAKE) -C nearn clean
	$(MAKE) -C guassian clean
	$(MAKE) -C kmeans clean
	$(MAKE) -C blackscholes clean
	$(MAKE) -C bfs clean

clean-all:
	$(MAKE) -C vecadd clean-all
	$(MAKE) -C sgemm clean-all
	$(MAKE) -C conv3 clean-all
	$(MAKE) -C psort clean-all
	$(MAKE) -C saxpy clean-all
	$(MAKE) -C sfilter clean-all
	$(MAKE) -C oclprintf clean-all
	$(MAKE) -C dotproduct clean-all
	$(MAKE) -C transpose clean-all
	$(MAKE) -C spmv clean-all
	$(MAKE) -C stencil clean-all
	$(MAKE) -C lbm clean-all
	$(MAKE) -C nearn clean-all
	$(MAKE) -C guassian clean-all
	$(MAKE) -C kmeans clean-all
	$(MAKE) -C blackscholes clean-all
	$(MAKE) -C bfs clean-all