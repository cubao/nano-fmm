# migrated from https://github.com/cubao/cpp-example/blob/main/data/Makefile

# this file keeps track of testing data used by this project.
# all files should be stored somewhere with overwrite protection.
# so these download links 100% guarantee exatly same files across time.

all:
	@echo use "make pull" to pull data
	@echo use "make clean" to clean data

.PHONY: all pull clean
clean:
	@find . -maxdepth 1 ! -name Makefile ! -name . ! -name .. -exec rm -rf {} +

# https://github.com/cubao/fmm/wiki/Sample-data
pull: network.json
network.json: | network.zip
	unzip network.zip
network.zip:
	curl -LO https://github.com/cubao/fmm/files/11698908/network.zip

pull: suzhoubeizhan.json
suzhoubeizhan.json: | suzhoubeizhan.zip
	unzip suzhoubeizhan.zip
suzhoubeizhan.zip:
	curl -LO https://github.com/cubao/fmm/files/12568463/suzhoubeizhan.zip
