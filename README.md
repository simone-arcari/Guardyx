<p align="center">
  <img src="assets/img/guardyx-logo.png" alt="Guardyx Logo" width="180">
</p>

# Guardyx — Intelligent Vehicle Protection Firmware

**Guardyx** is an embedded security system designed to provide **advanced vehicle protection**, combining real‑time GPS tracking, secure telemetry, and event‑driven safety logic.
The firmware is optimized for low‑power hardware and delivers reliable, continuous monitoring for motorcycles, cars, and other vehicles.

## 🎯 Key Features

- **🚗 Vehicle Protection** — Real-time motion detection and anti-theft alerts
- **📍 GPS Tracking** — Continuous location sharing via Telegram
- **📱 Telegram Integration** — Full remote control and status monitoring
- **🔋 Low Power** — Optimized for battery operation with deep sleep
- **📡 LTE/GPS Hybrid** — Fast location fallback without waiting for GPS fix
- **🎚️ Calibration** — Per-vehicle motion thresholds with sensitivity adjustment
- **☎️ Escalation** — Progressive alerts: Telegram → Location share → Live tracking → Phone call
- **⚙️ Easy Config** — Per-vehicle settings stored in NVS (survive power loss)

---

## 📋 Prerequisites

Before you can compile and deploy Guardyx, ensure you have:

### **System Requirements**

- **macOS / Linux / Windows** with a terminal
- **Python 3.6+** (check: `python3 --version`)
- **Git** (check: `git --version`)
- **USB cable** (USB-C for ESP32-S3 board)

### **Hardware Requirements**

- **ESP32-S3 DevKit** board
- **LILYGO T-SIM7670G-S3** modem module
- **SIM card** with data plan
- **GSM & GPS antennas**
- **MPU-6050** accelerometer (I2C)

### **Software Stack**

This project uses **PlatformIO** as the build system, which automatically manages:

- Espressif ESP32-S3 toolchain
- Arduino framework
- All C++ dependencies (TinyGSM, ArduinoJson, TinyGPSPlus, MPU6050)

---

## 🚀 Quick Start

### **1. Install PlatformIO**

PlatformIO is a professional IDE and build system for embedded development.

#### **Option A: Via Python (Recommended)**

```bash
pip install platformio
```

#### **Option B: Via Homebrew (macOS)**

```bash
brew install platformio
```

#### **Verify Installation**

```bash
pio --version
# Expected output: PlatformIO Core, version 6.1.x
```

### **2. Clone or Open Project**

```bash
cd /Users/<username>/workspace/Guardyx
```

### **3. Install Project Dependencies**

```bash
pio project init --ide vscode
```

This generates the VS Code IntelliSense configuration and downloads all libraries.

---

## 🔨 Building the Project

This project provides **three build methods** for maximum flexibility:

### **Method 1: Simple Bash Script** (Fastest)

```bash
./scripts/build.sh              # Standard build
./scripts/build.sh -v           # Verbose output
./scripts/build.sh -c           # Clean build (recompile from scratch)
```

### **Method 2: Using Make** (Recommended for this project) ⭐

The Makefile provides a clean, Unix-style interface:

#### **Available Commands**

```bash
make help               # Show all available commands
make build             # Compile the firmware
make build-v           # Compile with verbose output
make clean             # Remove compiled objects
make rebuild           # Clean + build
make upload            # Flash firmware to board
make upload-verbose    # Upload with debug output
make monitor           # Open serial monitor (115200 baud)
make flash             # Upload + start monitoring (all-in-one)
make clean-all         # Deep clean everything
make info              # Display project information
```

#### **Example Workflow**

```bash
# Compile
make build

# If successful, upload to board
make upload

# Monitor the serial output
make monitor

# Or do everything at once
make flash
```

### **Method 3: Python Manager** (Advanced)

```bash
./build.py build       # Compile
./build.py upload      # Flash
./build.py monitor     # Serial monitor
./build.py flash       # Everything combined
```

---

## 📊 Build Output

When compilation succeeds, you'll see:

```
✅ Compilazione completata con successo!

Firmware generato:
  .pio/build/lilygo-tsim7670g-s3/firmware.bin
  Dimensione: 374K
  RAM usage: 6.2% (20 KB / 320 KB)
  Flash usage: 11.2% (374 KB / 3.3 MB)
```

The compiled firmware is located at:

```
.pio/build/lilygo-tsim7670g-s3/firmware.bin
```

---

## ⚙️ Configuration

Before uploading, you need to configure your credentials:

### **1. Create Configuration File**

```bash
cp config.example.h config.h
nano config.h
```

### **2. Fill in Required Fields**

- `TELEGRAM_TOKEN` — Your Telegram bot token (from @BotFather)
- `TELEGRAM_CHAT_ID` — Your Telegram user ID (from @userinfobot)
- `APN` — Your mobile operator's APN (see config.example.h for examples)

### **3. Optional Credentials** (for Twilio calls, Geoapify maps, etc)

- `TWILIO_SID`, `TWILIO_AUTH`, etc.

**⚠️ Security Note**: Never commit `config.h` with real credentials to Git. It's protected by `.gitignore`.

---

## 📤 Uploading & Testing

### **Step 1: Connect Your Board**



### **Step 2: Find Serial Port**

```bash
./build.py ports
# Output: /dev/cu.usbserial-0001  (macOS)
#         /dev/ttyUSB0             (Linux)
#         COM3                     (Windows)
```

### **Step 3: Upload Firmware**

```bash
make upload
```

### **Step 4: Monitor Serial Output**

```bash
make monitor
```

Expected output at boot:

```
ESP32-S3 Guardyx Anti-Theft System v1.0
[INFO] Setup initialized
[INFO] Modem ready
[INFO] GPS enabled
[INFO] System ready!
```

## 🧪 Testing

### **Motion Detection Test**

After uploading, the device will start monitoring motion on the configured vehicle.

```
Move the board → Receive alert on Telegram
```

### **GPS Test**

The system automatically acquires GPS position (takes 30-60 seconds outdoors).

```
Wait 1-2 minutes in open sky → /status command shows GPS satellites
```

### **Telegram Commands**

```
/start              - Initialize bot
/attiva             - Arm protection
/disattiva          - Disarm protection
/posizione          - Get current location
/stato              - System status (signal, battery, GPS)
/calibra            - Calibrate motion thresholds
```

---

## 🆘 Troubleshooting

### **"pio: command not found"**

```bash
pip install platformio
```

### **Compilation Fails**

```bash
make clean
make build-v         # Try with verbose output to see errors
```

### **Firmware Won't Upload**

```bash
# Check serial port
./build.py ports

# Try forcing bootloader mode:
# 1. Hold BOOT button
# 2. Press RESET
# 3. Release RESET (keep BOOT pressed)
# 4. Release BOOT after 1 second
# 5. Try make upload again
```

### **Modem Not Responding**

- Verify APN in `config.h` matches your mobile operator
- Wait 1-2 minutes at first boot (modem initialization)
- Check antenna connections

### **No GPS Fix**

- Go outside (GPS needs open sky)
- Wait 1-2 minutes (first fix takes time)
- Ensure modem is connected to network (check LTE bars)
- Enable LTE location fallback: `#define USE_LTE_LOCATION 1`

---

## 🔧 System Stack

| Component                   | Version | Purpose                           |
| --------------------------- | ------- | --------------------------------- |
| **ESP32-S3**          | —      | 240 MHz dual-core microcontroller |
| **SIM7670G**          | —      | LTE/4G modem with GPS/GNSS        |
| **PlatformIO**        | 6.1+    | Build system & IDE                |
| **Arduino Framework** | 3.2+    | Embedded runtime                  |
| **TinyGSM**           | 0.12.0  | Modem abstraction library         |
| **ArduinoJson**       | 7.4+    | JSON serialization                |
| **TinyGPSPlus**       | 1.1.0   | GPS data parsing                  |
| **MPU6050**           | 1.4+    | Accelerometer driver              |
