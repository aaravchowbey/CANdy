.PHONY: sender receiver hammer monitor due_michican_defender_example due_attacker_example hammer2

all: sender hammer2

SENDER_ID   := .port.properties.serialNumber == "75932313638351300251"
RECEIVER_ID := .port.properties.serialNumber == "85138313234351E09162" or .port.properties.pid == "0x003e"
HAMMER_ID   := .port.properties.serialNumber == "75932313638351803271"
# HAMMER_ID   := .port.properties.serialNumber == "85036313130351715042"

sender:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.[] | select($(SENDER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli compile $$FLAGS $@ && \
	arduino-cli upload $$FLAGS $@

receiver:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.[] | select($(RECEIVER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli compile $$FLAGS $@ && \
	arduino-cli upload $$FLAGS $@

hammer:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.[] | select($(HAMMER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli compile $$FLAGS $@ && \
	arduino-cli upload $$FLAGS $@

hammer2:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.[] | select($(RECEIVER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli compile $$FLAGS $@ && \
	arduino-cli upload $$FLAGS $@

monitor:
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.[] | select(.port.properties.pid == "0x003e") | "-p \(.port.address)"'); \
	arduino-cli monitor $$FLAGS

due_attacker_example:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.[] | select($(SENDER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli compile $$FLAGS $@ && \
	arduino-cli upload $$FLAGS $@

due_michican_defender_example:
	@echo "\033[1mcompiling and uploading $@\033[0m"
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.[] | select($(RECEIVER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli compile $$FLAGS $@ && \
	arduino-cli upload $$FLAGS $@
