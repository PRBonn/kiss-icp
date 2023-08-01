.PHONY: cpp

editable:
	@pip install --no-build-isolation -ve ./python/

install:
	@pip install --verbose ./python/

uninstall:
	@pip -v uninstall kiss_icp

cpp:
	@cmake -Bbuild cpp/kiss_icp/
	@cmake --build build -j$(nproc --all)
