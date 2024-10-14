build-all:
	cd examples/adafruit_rgb && \
	  cargo build --target=thumbv6m-none-eabi --examples --all-features
	cd examples/ledmatrix && \
	  cargo build --target=thumbv6m-none-eabi --examples --all-features
