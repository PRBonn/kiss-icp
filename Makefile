.PHONY: cpp

editable:
	SETUPTOOLS_ENABLE_FEATURES="legacy-editable" pip install --verbose --prefix=$(shell python3 -m site --user-base) --editable ./python/

install:
	@pip install --verbose ./python/

uninstall:
	@pip -v uninstall kiss_icp

cpp:
	@cmake -Bbuild cpp/kiss_icp/
	@cmake --build build -j$(nproc --all)
