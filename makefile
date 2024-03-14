.PHONY: sender receiver

SENDER_ID   := .port.properties.serialNumber == "75932313638351300251" or .port.properties.pid == "0x003e"
RECEIVER_ID := .port.properties.serialNumber == "85138313234351E09162"

sender:
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.[] | select($(SENDER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli compile $$FLAGS $@; \
	arduino-cli upload $$FLAGS $@

receiver:
	@FLAGS=$$(arduino-cli board list --format=json | jq -r '.[] | select($(RECEIVER_ID)) | "-b \(.matching_boards | .[0] | .fqbn) -p \(.port.address)"'); \
	arduino-cli compile $$FLAGS $@; \
	arduino-cli upload $$FLAGS $@
