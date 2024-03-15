.PHONY: sender receiver

all: sender receiver

SENDER_ID   := .port.properties.serialNumber == "75932313638351300251" or .port.properties.pid == "0x003e"
RECEIVER_ID := .port.properties.serialNumber == "85138313234351E09162"

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

monitor:
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.[] | select(.port.properties.pid == "0x003e") | "-p \(.port.address)"'); \
	arduino-cli monitor $$FLAGS
