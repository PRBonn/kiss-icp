.PHONY: cpp

install:
	@pip install --verbose ./python/

uninstall:
	@pip -v uninstall kiss_icp

editable:
	@pip install scikit-build-core pyproject_metadata pathspec pybind11 ninja cmake
	@pip install --no-build-isolation -ve ./python/

test:
	@pytest -rA --verbose ./python/

cpp:
	@cmake -Bbuild cpp/kiss_icp/
	@cmake --build build -j$(nproc --all)
