# Makefile per Guardyx - ESP32-S3 con SIM7670G

.PHONY: build clean upload monitor format help

# Ambiente PlatformIO
ENV = lilygo-tsim7670g-s3
PORT ?= /dev/cu.usbserial-0001

help:
	@echo "Guardyx Build System"
	@echo ""
	@echo "Comandi disponibili:"
	@echo "  make build          - Compila il firmware"
	@echo "  make build-v        - Compila con output verbose"
	@echo "  make clean          - Pulisce gli oggetti compilati"
	@echo "  make rebuild        - Clean + build"
	@echo "  make upload         - Carica il firmware su board"
	@echo "  make upload-verbose - Upload con debug output"
	@echo "  make monitor        - Apri il monitor seriale"
	@echo "  make clean-all      - Pulisce tutto (incluso .pio)"
	@echo ""

build:
	./scripts/build.sh

build-v:
	./scripts/build.sh --verbose

clean:
	pio run -e $(ENV) --target clean

rebuild: clean build

upload:
	pio run -e $(ENV) --target upload

upload-verbose:
	pio run -e $(ENV) --target upload -v

monitor:
	pio device monitor -e $(ENV) -b 115200

flash: upload monitor

clean-all:
	rm -rf .pio build firmware.bin
	pio run -e $(ENV) --target clean

info:
	@echo "Informazioni progetto:"
	@echo "  Board: ESP32-S3 DevKit"
	@echo "  Modem: SIM7670G"
	@echo "  Framework: Arduino"
	@echo "  Librerie: TinyGSM, ArduinoJson, TinyGPSPlus, MPU6050"
	@echo ""
	@echo "Porte seriali disponibili:"
	@ls -la /dev/cu.* 2>/dev/null || echo "  Nessuna porta trovata"
	@echo ""

.DEFAULT_GOAL := help
