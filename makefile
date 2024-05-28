.PHONY: sender receiver hammer monitor due_michican_defender_example due_attacker_example hammer2 hammer2_debug

include config.mk

all: sender hammer2

sender:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.detected_ports | .[] | select($(SENDER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli --no-color compile --warnings=all --upload $$FLAGS $@

receiver:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.detected_ports | .[] | select($(RECEIVER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli --no-color compile --warnings=all --upload $$FLAGS $@

hammer:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.detected_ports | .[] | select($(HAMMER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli --no-color compile --warnings=all --upload $$FLAGS $@

hammer2:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.detected_ports | .[] | select($(RECEIVER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli --no-color compile --warnings=all --upload $$FLAGS $@

monitor:
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.detected_ports | .[] | select(.port.properties.pid == "0x003e") | "-p \(.port.address)"'); \
	arduino-cli monitor $$FLAGS

due_attacker_example:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.detected_ports | .[] | select($(SENDER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli --no-color compile --warnings all --upload $$FLAGS $@

due_michican_defender_example:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.detected_ports | .[] | select($(RECEIVER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli --no-color compile --warnings all --upload $$FLAGS $@

hammer2_debug:
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.detected_ports | .[] | select($(RECEIVER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli --no-color compile --warnings=all --export-binaries --optimize-for-debug $$FLAGS hammer2
