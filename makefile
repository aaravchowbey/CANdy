PRINT_MESSAGE = @echo "\033[1mcompiling and uploading $@\033[0m"
ARDUINO_CLI = arduino-cli --no-color
COMPILE_FLAGS = --warnings all --upload # --export-binaries --optimize-for-debug to export
define get_board_flags
	$(shell arduino-cli board list --json | jq -r '.detected_ports[] | select($1) | "--fqbn \(.matching_boards[0].fqbn) --port \(.port.address)"')
endef

include config.mk

all: modem

modem: sender sender2 hammer2

integrated: sender hammer-integrated

sender:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(SENDER)) $@

sender2:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DSENDER2\"" $(call get_board_flags, $(SENDER2)) sender

receiver:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(RECEIVER)) $@

hammer:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(HAMMER)) $@

hammer2:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(RECEIVER)) $@

hammer-integrated:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(SENDER)) $@

due_attacker_example:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(SENDER)) $@

due_michican_defender_example:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(RECEIVER)) $@

uno:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(UNO)) $@

uno-send:
	$(PRINT_MESSAGE)
	@$(ARDUINO_CLI) compile $(COMPILE_FLAGS) $(call get_board_flags, $(UNO)) $@

monitor:
	$(ARDUINO_CLI) monitor $(call get_board_flags, .port.properties.pid == "0x003e")

monitor-uno:
	$(ARDUINO_CLI) monitor $(call get_board_flags, .port.properties.pid == "0x0043")

.PHONY: all sender receiver hammer hammer2 hammer-integrated due_attacker_example due_michican_defender_example uno uno-send monitor monitor-uno
