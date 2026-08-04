# Quick Start — flashing and first setup

This is a short, practical guide: how to get this firmware onto your controller, and
the **minimum** you need to check afterward before riding. It is not a full parameter
reference — see `documentation/` for that.

⚠️ **This project is under construction.** Everything you do here is at your own risk.
Use a fuse between controller and battery. Test on a stand (wheel off the ground) before
riding.

## 0. What you need

- A CANable adapter connected to the bike's CAN bus.
- The matching CANable tool from **this project's own fork**:
  [glodnickim/bafang_canable_pro](https://github.com/glodnickim/bafang_canable_pro).
  Do not use another CANable build — this firmware's CAN protocol has fields the
  original/other tools do not know about.
- A built firmware image: `<version>_M820_BL820.bin` (either build it yourself per
  `documentation/BUILD_FIRMWARE.md`, or use a released `.bin` from this repo's
  [Releases](../../releases) page).

## 1. Flash the firmware

1. Open the CANable tool, connect to the controller.
2. Go to the **Firmware** tab.
3. Under **Mode**, pick **Motor controller**.
4. Under **File**, select your `<version>_M820_BL820.bin`.
5. Click **Start Update Procedure** and wait for it to finish — do not disconnect or
   power off the bike during the update.
6. Reconnect / refresh the connection once it completes.

## 2. Minimum setup before riding — safety-relevant first

Do these in order. Skipping them can mean wrong speed readings, wrong assist power, or
tripped safety limits.

### 2.1 Battery and current limits
Go to **eVistDrive Limits** → **Electrical and battery limits**:
- **System voltage** — match your actual pack's nominal voltage.
- **Maximum battery current** — set to what your battery/BMS can actually deliver.
  This is a hard ceiling on top of every per-level current limit.
- Overvoltage / undervoltage cutoffs — set relative to your pack's real voltages, not
  left at a mismatched default.

### 2.2 Speed sensor and wheel size
Same tab, **Speed, wheel and legal settings**:
- **Speed sensor pulses/revolution** — must match your actual speed sensor/magnet
  setup, or every speed and distance reading (and the legal speed limit) will be wrong.
- **Wheel diameter** / circumference.
- **Legal speed-limit flag** — enable if you need the region's assist cutoff speed
  enforced.

### 2.3 Per-level assist settings
Go to **eVistDrive Profiles**, pick Bank 1, and for at least one assist level check:
- **Maximum motor current** and **Maximum motor power** — sane values for your setup,
  not left at whatever a previous rider's bank held.
- **Minimum pedal load** — how firm a push is needed to start assist.

Read the bank from the controller first (**Read banks**) before changing anything, so
you are editing real values, not offline placeholders.

## 3. Write and save

- **Write (RAM)** buttons send changes to the controller's working memory — test ride
  behavior on a stand first.
- **Save to Flash** (top bar) makes RAM changes permanent, at a full standstill. Nothing
  survives a power cycle until you do this.

## 4. Where to go next

- `documentation/README.md` — index of the deeper technical documentation.
- `CHANGELOG.md` — what changed and why, release by release.
- This repo's [Issues](../../issues) — for questions or problems specific to this fork.
