PRINT_MESSAGE = @echo "\033[1mcompiling and uploading $@\033[0m"
ARDUINO_CLI = arduino-cli --no-color
COMPILE_FLAGS = --warnings all --upload # --export-binaries --optimize-for-debug to export
define get_board_flags
	$(shell arduino-cli board list --format=json | jq -r '.detected_ports | .[] | select($1) | "--fqbn \(.matching_boards | .[0] | .fqbn) --port \(.port.address)"')
endef

include config.mk

all: sender hammer2

sender:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(SENDER_ID)) $@

receiver:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(RECEIVER_ID)) $@

hammer:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(HAMMER_ID)) $@

hammer2:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(RECEIVER_ID)) $@

due_attacker_example:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(SENDER_ID)) $@

due_michican_defender_example:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(RECEIVER_ID)) $@

monitor:
	$(ARDUINO_CLI) monitor $(call get_board_flags, .port.properties.pid == "0x003e")

.PHONY: all sender receiver hammer monitor due_michican_defender_example due_attacker_example hammer2
