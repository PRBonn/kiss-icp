editable:
	SETUPTOOLS_ENABLE_FEATURES="legacy-editable" pip install --verbose --prefix=$(shell python3 -m site --user-base) --editable .

install:
	@pip install --verbose .

uninstall:
	@pip -v uninstall kiss_icp

docker:
	@docker build -t gitlab.ipb.uni-bonn.de:4567/ipb-team/ipb-tools/kiss_icp .

docker-push:
	@docker push gitlab.ipb.uni-bonn.de:4567/ipb-team/ipb-tools/kiss_icp

license:
	@addlicense -f LICENSE -ignore **/*.yaml -v .
