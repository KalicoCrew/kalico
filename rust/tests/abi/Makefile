# ABI smoke test: link a C program against the Rust staticlib.

ABI_DIR := $(dir $(realpath $(firstword $(MAKEFILE_LIST))))
RUST_ROOT := $(abspath $(ABI_DIR)/../..)
TARGET_DIR := $(RUST_ROOT)/target

# Default profile: host (debug), MCU build runs separately.
PROFILE ?= debug
LIB_DIR := $(TARGET_DIR)/$(PROFILE)
CFLAGS := -O2 -Wall -Wextra -Werror

test_eval: $(LIB_DIR)/libkalico_c_api.a $(ABI_DIR)/test_eval.c
	$(CC) $(CFLAGS) $(ABI_DIR)/test_eval.c -L$(LIB_DIR) -lkalico_c_api -lpthread -ldl -lm -o $@

$(LIB_DIR)/libkalico_c_api.a:
	cd $(RUST_ROOT) && cargo build -p kalico-c-api --features host --no-default-features

run: test_eval
	./test_eval

clean:
	rm -f test_eval

.PHONY: clean run
