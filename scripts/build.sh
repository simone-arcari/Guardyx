#!/bin/bash

# Script di compilazione per progetto Guardyx
# Utilizza PlatformIO per compilare il firmware per ESP32-S3

set -e

# Colori per output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Variabili
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
ENV="lilygo-tsim7670g-s3"
BUILD_DIR="${PROJECT_DIR}/.pio/build/${ENV}"
FIRMWARE_BIN="${BUILD_DIR}/firmware.bin"

echo -e "${YELLOW}🔨 Compilazione Guardyx...${NC}"
echo "Progetto: $PROJECT_DIR"
echo "Ambiente: $ENV"
echo ""

# Opzioni da riga di comando
VERBOSE=""
CLEAN=""

case "${1:-}" in
    "-v"|"--verbose")
        VERBOSE="-v"
        echo -e "${YELLOW}Modalità verbose abilitata${NC}"
        ;;
    "-c"|"--clean")
        CLEAN="clean"
        echo -e "${YELLOW}Clean build...${NC}"
        ;;
    "-cv"|"-vc"|"--clean-verbose")
        CLEAN="clean"
        VERBOSE="-v"
        echo -e "${YELLOW}Clean build con verbose${NC}"
        ;;
    "-h"|"--help")
        echo "Utilizzo: $0 [opzione]"
        echo ""
        echo "Opzioni:"
        echo "  (nessuna)      Compilazione standard"
        echo "  -v, --verbose  Output dettagliato"
        echo "  -c, --clean    Clean build (cancella oggetti precedenti)"
        echo "  -cv, -vc       Clean build + verbose"
        echo "  -h, --help     Mostra questo aiuto"
        exit 0
        ;;
esac

cd "$PROJECT_DIR"

# Esegui compilazione
if [ -n "$CLEAN" ]; then
    pio run -e $ENV --target clean $VERBOSE
fi

echo ""
if pio run -e $ENV $VERBOSE; then
    echo ""
    echo -e "${GREEN}Compilazione completata con successo!${NC}"
    echo ""
    echo "Firmware generato:"
    echo "  $FIRMWARE_BIN"
    
    # Mostra dimensione firmware
    if [ -f "$FIRMWARE_BIN" ]; then
        SIZE=$(du -h "$FIRMWARE_BIN" | cut -f1)
        echo "  Dimensione: $SIZE"
    fi
    
    echo ""
    echo "Prossimi step:"
    echo "  - Carica con: pio run -e $ENV --target upload"
    echo "  - Monitor seriale: pio device monitor -e $ENV"
    
else
    echo ""
    echo -e "${RED}Compilazione fallita!${NC}"
    exit 1
fi
