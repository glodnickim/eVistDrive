#!/usr/bin/env python3
"""Cross-platform verified M820_BL820 developer/repro build.

The compile/link flags and memory gates mirror scripts/build-firmware.ps1, but the path no longer
requires PowerShell. Canonical release-number allocation intentionally remains outside this tool;
this builder is for deterministic Developer (DEV-NONCANONICAL) and explicit Repro identities.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_GCC = "13.2.1"
FLASH_ORIGIN = 0x08005000
CONFIG_A_ORIGIN = 0x0803E800
RAM_ORIGIN = 0x20000000
VERSION_STATE_ROOT = ROOT.parent / ".ebics-version-state"

sys.path.insert(0, str(ROOT / "tools"))
from prepare_m820_bl820 import build_container


def run(cmd: list[str], *, capture=False) -> str:
    p = subprocess.run(cmd, cwd=ROOT, text=True,
                       stdout=subprocess.PIPE if capture else None,
                       stderr=subprocess.STDOUT if capture else None)
    if p.returncode:
        if capture and p.stdout:
            print(p.stdout, end="")
        raise SystemExit(f"command failed ({p.returncode}): {' '.join(cmd)}")
    return p.stdout if capture else ""


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def get_version_state_root(repo_root: Path) -> Path:
    """Compute the same state root as build-version-allocator.psm1 Get-EbicsVersionStateRoot."""
    git_dir = repo_root / ".git"
    if git_dir.exists():
        try:
            import subprocess
            result = subprocess.run(
                ["git", "-c", f"safe.directory={repo_root}", "rev-parse", "--path-format=absolute", "--git-common-dir"],
                cwd=repo_root, capture_output=True, text=True
            )
            if result.returncode == 0 and result.stdout.strip():
                common_git = result.stdout.strip()
                main_repo = Path(common_git).parent.parent
                return main_repo.parent / ".ebics-version-state"
        except Exception:
            pass
    return repo_root.parent / ".ebics-version-state"


def get_canonical_version(state_root: Path) -> tuple[int, str]:
    """Read HWM from allocator state. Returns (hwm_int, formatted_version)."""
    path = state_root / "M820_BL820.json"
    if not path.exists():
        raise SystemExit(f"Version state missing: {path}. Run with --init-state first.")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        raise SystemExit(f"Version state unreadable: {path}: {e}")
    if data.get("schema") != 1 or data.get("target") != "M820_BL820":
        raise SystemExit(f"Invalid version state schema at {path}")
    hwm = int(data.get("hwm", 0))
    if hwm < 459:
        raise SystemExit(f"Version state HWM {hwm} below minimum 459")
    major = hwm // 1000
    minor = hwm % 1000
    return hwm, f"{major}.{minor:03d}"


def reserve_canonical_version(state_root: Path) -> str:
    """Reserve next canonical version atomically. Returns formatted version string."""
    path = state_root / "M820_BL820.json"
    if not path.exists():
        raise SystemExit(f"Version state missing: {path}. Initialize with --init-state.")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        raise SystemExit(f"Cannot read version state: {e}")
    if data.get("schema") != 1 or data.get("target") != "M820_BL820":
        raise SystemExit(f"Invalid version state at {path}")
    hwm = int(data.get("hwm", 0))
    if hwm < 459:
        raise SystemExit(f"Version state HWM {hwm} below minimum 459")
    new_hwm = hwm + 1
    data["hwm"] = new_hwm
    data["updated_utc"] = time.strftime("%Y-%m-%dT%H:%M:%S+08:00")
    # Preserve migration fields
    for key in ["migrated_from", "hwm_reconciled_from", "hwm_reconciled_utc", "hwm_reconciliation_evidence"]:
        if key not in data:
            pass  # keep existing
    path.write_text(json.dumps(data, indent=4, ensure_ascii=False) + "\n", encoding="utf-8")
    major = new_hwm // 1000
    minor = new_hwm % 1000
    return f"{major}.{minor:03d}"


def init_version_state(state_root: Path, initial_hwm: int = 600) -> None:
    """Initialize version state if missing."""
    state_root.mkdir(parents=True, exist_ok=True)
    path = state_root / "M820_BL820.json"
    if path.exists():
        print(f"Version state already exists: {path}")
        return
    marker = state_root / "M820_BL820.migration.json"
    if marker.exists():
        raise SystemExit("Migration marker exists but state is missing. Recover from evidence.")
    import datetime
    now = datetime.datetime.now().isoformat()
    marker_data = {"schema": 1, "target": "M820_BL820", "initial_hwm": initial_hwm,
                   "migrated_from": "manual initialization", "migrated_utc": now}
    marker.write_text(json.dumps(marker_data, indent=4, ensure_ascii=False) + "\n", encoding="utf-8")
    state_data = {"schema": 1, "target": "M820_BL820", "hwm": initial_hwm,
                  "migrated_from": "manual initialization", "updated_utc": now}
    path.write_text(json.dumps(state_data, indent=4, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Initialized version state at {path} with HWM={initial_hwm} (next version: {initial_hwm//1000}.{initial_hwm%1000:03d})")


def reserve_canonical_version_pair(state_root: Path) -> list[str]:
    """Reserve two consecutive canonical versions (normal + diagnostic). Returns [normal_ver, diag_ver]."""
    path = state_root / "M820_BL820.json"
    if not path.exists():
        raise SystemExit(f"Version state missing: {path}. Initialize with --init-state.")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        raise SystemExit(f"Cannot read version state: {e}")
    if data.get("schema") != 1 or data.get("target") != "M820_BL820":
        raise SystemExit(f"Invalid version state at {path}")
    hwm = int(data.get("hwm", 0))
    if hwm < 459:
        raise SystemExit(f"Version state HWM {hwm} below minimum 459")
    new_hwm_1 = hwm + 1
    new_hwm_2 = hwm + 2
    data["hwm"] = new_hwm_2
    data["updated_utc"] = time.strftime("%Y-%m-%dT%H:%M:%S+08:00")
    path.write_text(json.dumps(data, indent=4, ensure_ascii=False) + "\n", encoding="utf-8")
    def fmt(n): return f"{n//1000}.{n%1000:03d}"
    return [fmt(new_hwm_1), fmt(new_hwm_2)]


def _build_one(version: str, variant: str, version_source: str, entries: list[str],
               toolchain: str | None, output_dir: str) -> dict:
    """Build a single variant. Returns dict with version, variant, release_bin path."""
    gcc = find_tool("arm-none-eabi-gcc", toolchain)
    toolbin = str(Path(gcc).resolve().parent)
    objcopy = find_tool("arm-none-eabi-objcopy", toolbin)
    size = find_tool("arm-none-eabi-size", toolbin)
    nm = find_tool("arm-none-eabi-nm", toolbin)
    readelf = find_tool("arm-none-eabi-readelf", toolbin)
    gcc_version = run([gcc, "-dumpfullversion", "-dumpversion"], capture=True).strip().splitlines()[0]
    if gcc_version != EXPECTED_GCC:
        raise SystemExit(f"unsupported Arm GNU Toolchain {gcc_version}; expected {EXPECTED_GCC}")

    outroot = Path(output_dir)
    if not outroot.is_absolute():
        outroot = ROOT / outroot
    userdir = outroot / "M820_BL820"
    work = userdir / "work" / variant
    gen = work / "generated"
    objdir = work / "objects"
    gen.mkdir(parents=True, exist_ok=True)
    objdir.mkdir(parents=True, exist_ok=True)
    (gen / "build_version.h").write_text(
        '#ifndef BUILD_VERSION_H\n#define BUILD_VERSION_H\n'
        '/* Generated in the build directory; never edit or commit. */\n'
        f'#define EBICS_BUILD_VERSION "{version}"\n#endif\n', encoding="ascii")

    defs = ["-DGD32F30X_HD", "-DGD_ECLIPSE_GCC", "-DUSE_STDPERIPH_DRIVER", "-DBOOTLOADER=820",
            f"-DCAN_DIAGNOSTICS_ENABLE={1 if variant == 'diagnostic' else 0}"]
    common = ["-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"]
    inc = [f"-I{gen}", f"-I{ROOT/'inc'}", f"-I{ROOT/'Firmware/CMSIS'}",
           f"-I{ROOT/'Firmware/CMSIS/GD/GD32F30x/Include'}",
           f"-I{ROOT/'Firmware/GD32F30x_standard_peripheral/Include'}"]
    cflags = common + ["-O0", "-g3", "-fmessage-length=0", "-fsigned-char",
                       "-ffunction-sections", "-fdata-sections", "-Wall"] + defs + inc

    objects: list[str] = []
    for entry in entries:
        obj = objdir / (re.sub(r"[\\/:]", "_", entry) + ".o")
        run([gcc, *cflags, "-c", str(ROOT / entry), "-o", str(obj)])
        objects.append(str(obj))
    startup_obj = objdir / "startup_gd32f30x_hd.S.o"
    run([gcc, *common, *defs, *inc, "-x", "assembler-with-cpp", "-c",
         str(ROOT / "gcc_startup/startup_gd32f30x_hd.S"), "-o", str(startup_obj)])
    objects.append(str(startup_obj))

    base = version
    elf, rawbin, hexp = work/base, work/f"{base}.bin", work/f"{base}.hex"
    elf = elf.with_suffix(".elf")
    mapf = work/f"{base}.map"
    suffix = "_DIAG" if variant == "diagnostic" else ""
    final = userdir/f"{base}_M820_BL820{suffix}.bin"
    manifest = userdir/f"{base}_M820_BL820{suffix}.manifest.json"
    linker = ROOT / "ldscripts/gd32f30x_flash.ld"
    link = [gcc, *common, f"-T{linker}", "-Wl,--gc-sections", "-Wl,--print-memory-usage",
            f"-Wl,-Map,{mapf}", "-Wl,--start-group", *objects,
            f"-L{ROOT/'Firmware/CMSIS'}", "-larm_cortexM4lf_math", "-specs=nano.specs", "-specs=nosys.specs",
            "-Wl,--end-group", "-o", str(elf)]
    run(link)
    run([objcopy, "-O", "binary", str(elf), str(rawbin)])
    run([objcopy, "-O", "ihex", str(elf), str(hexp)])

    size_out = run([size, str(elf)], capture=True)
    size_a = run([size, "-A", str(elf)], capture=True)
    (work/f"{base}.size.txt").write_text(size_out + "\n" + size_a, encoding="ascii")
    ph = run([readelf, "-l", str(elf)], capture=True)
    (work/f"{base}.program-headers.txt").write_text(ph, encoding="ascii")
    if re.search(r"\bRWE\b", ph):
        raise SystemExit("ELF contains an RWE load segment")
    nm_text = run([nm, "--defined-only", str(elf)], capture=True)
    symbols = symbol_map(nm_text)
    app_start = require_symbol(symbols, "__app_flash_start")
    app_limit = require_symbol(symbols, "__app_flash_limit")
    image_end = require_symbol(symbols, "__flash_image_end")
    config_a = require_symbol(symbols, "__config_a_start")
    if app_start != FLASH_ORIGIN or config_a != CONFIG_A_ORIGIN or image_end > app_limit or app_limit > config_a:
        raise SystemExit("linked memory map does not match M820_BL820 target")
    diag_symbol = "diag_session_dump_step" in symbols
    if diag_symbol != (variant == "diagnostic"):
        raise SystemExit("diagnostic link-marker mismatch")

    cols = size_out.strip().splitlines()[-1].split()
    if len(cols) < 4:
        raise SystemExit("cannot parse arm-none-eabi-size output")
    text_b, data_b, bss_b = map(int, cols[:3])
    sdata, edata = require_symbol(symbols, "_sdata"), require_symbol(symbols, "_edata")
    sbss, ebss = require_symbol(symbols, "_sbss"), require_symbol(symbols, "_ebss")
    sp = require_symbol(symbols, "_sp")

    packed, packinfo = build_container(rawbin.read_bytes())
    final.parent.mkdir(parents=True, exist_ok=True)
    final.write_bytes(packed)
    if version.encode("ascii") not in final.read_bytes():
        raise SystemExit("firmware identity string missing from final BL820 artifact")

    commit = git_value("rev-parse", "HEAD")
    describe = git_value("describe", "--tags", "--always")
    dirty = False
    if (ROOT / ".git").exists():
        dirty = bool(git_value("status", "--porcelain", "--untracked-files=normal"))
    doc = {
        "schema_version": 1, "target": "M820_BL820", "profile": "debug", "variant": variant,
        "diagnostics_enabled": variant == "diagnostic", "version": version,
        "version_source": version_source,
        "git_commit": commit, "git_description": describe, "worktree_dirty": dirty,
        "hardware_approved_profile": True,
        "toolchain": "Arm GNU Toolchain arm-none-eabi", "toolchain_version": gcc_version,
        "linker": "ldscripts/gd32f30x_flash.ld", "source_manifest": "scripts/sources-m820.txt",
        "source_count": len(entries),
        "flash": {"origin": f"0x{app_start:08X}", "image_end": f"0x{image_end:08X}",
                  "limit": f"0x{app_limit:08X}", "config_a": f"0x{config_a:08X}",
                  "text_bytes": text_b, "gnu_size_data_bytes": data_b,
                  "data_load_bytes": edata-sdata, "binary_bytes": rawbin.stat().st_size},
        "ram": {"data_bytes": edata-sdata, "bss_bytes": ebss-sbss,
                "heap_stack_reserved_bytes": sp-ebss,
                "used_including_heap_stack_bytes": sp-RAM_ORIGIN,
                "gnu_size_bss_bytes": bss_b},
        "packaging": {k: (f"0x{v:08X}" if k == "stm32_crc32" else f"0x{v:04X}" if k in ("header_size_modulo","header_crc16") else v)
                      for k,v in packinfo.items()},
        "artifacts": {"elf": str(elf), "map": str(mapf), "raw_binary": str(rawbin),
                      "raw_binary_sha256": sha256(rawbin), "final_binary": str(final),
                      "final_binary_bytes": final.stat().st_size, "final_binary_sha256": sha256(final)}
    }
    manifest.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    print(f"\n=== {variant.upper()} variant ===")
    print(f"Version:   {version}")
    print(f"Compiler:  {gcc_version}")
    print(f"Sources:   {len(entries)} + startup")
    print(f"Raw BIN:   {rawbin.stat().st_size} B")
    print(f"Final BIN: {final}")
    print(f"SHA256:    {sha256(final)}")

    # Copy final BIN to releases/<version>_M820_BL820[_DIAG].bin
    release_bin = ""
    if version_source == "auto_global":
        releases_dir = ROOT / "releases"
        releases_dir.mkdir(parents=True, exist_ok=True)
        dest = releases_dir / final.name
        shutil.copy2(final, dest)
        release_bin = str(dest)
        print(f"RELEASE:   {dest}")
        print(f"Size:      {dest.stat().st_size} B")

    print("==================================================")
    return {"version": version, "variant": variant, "release_bin": release_bin, "sha256": sha256(final)}
    if not (ROOT / ".git").exists():
        return "NO-GIT"
    return run(["git", "-c", f"safe.directory={ROOT}", "-c", "core.excludesFile=",
                "-C", str(ROOT), *args], capture=True).strip()


def git_value(*args: str) -> str:
    if not (ROOT / ".git").exists():
        return "NO-GIT"
    return run(["git", "-c", f"safe.directory={ROOT}", "-c", "core.excludesFile=",
                "-C", str(ROOT), *args], capture=True).strip()


def source_entries() -> list[str]:
    out: list[str] = []
    for line in (ROOT / "scripts" / "sources-m820.txt").read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            if not (ROOT / line).is_file():
                raise SystemExit(f"source manifest entry missing: {line}")
            out.append(line)
    if not out:
        raise SystemExit("source manifest is empty")
    return out


def find_tool(name: str, toolchain: str | None) -> str:
    suffix = ".exe" if os.name == "nt" else ""
    exe = name + suffix
    if toolchain:
        p = Path(toolchain)
        if p.is_file() and p.name.lower().startswith(name.lower()):
            return str(p.resolve())
        q = p / exe
        if q.is_file():
            return str(q.resolve())
        # Be tolerant when a Windows toolchain path is passed from a POSIX-like shell.
        q2 = p / name
        if q2.is_file():
            return str(q2.resolve())
        raise SystemExit(f"tool not found under --toolchain: {exe}")
    found = shutil.which(exe) or shutil.which(name)
    if not found and os.name == "nt":
        # Same canonical location used by the original PowerShell build. This keeps the Python
        # gate usable on a normal Windows install even when the Arm bin directory is not in PATH.
        default_bin = Path(r"C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1\bin")
        candidate = default_bin / exe
        if candidate.is_file():
            found = str(candidate)
    if not found:
        raise SystemExit(f"{name} not found; install Arm GNU Toolchain {EXPECTED_GCC} or pass --toolchain BIN_DIR")
    return found


def symbol_map(nm_text: str) -> dict[str, int]:
    d: dict[str, int] = {}
    for line in nm_text.splitlines():
        m = re.match(r"^\s*([0-9A-Fa-f]+)\s+\w\s+(\S+)\s*$", line)
        if m:
            d[m.group(2)] = int(m.group(1), 16)
    return d


def require_symbol(symbols: dict[str, int], name: str) -> int:
    if name not in symbols:
        raise SystemExit(f"required linker symbol not found: {name}")
    return symbols[name]


def check_tree() -> list[str]:
    entries = source_entries()
    required = [
        "gcc_startup/startup_gd32f30x_hd.S",
        "ldscripts/gd32f30x_flash.ld",
        "Firmware/CMSIS/arm_math.h",
        "Firmware/CMSIS/libarm_cortexM4lf_math.a",
        "Firmware/CMSIS/GD/GD32F30x/Include/gd32f30x.h",
    ]
    missing = [p for p in required if not (ROOT / p).is_file()]
    if missing:
        raise SystemExit("build tree incomplete: " + ", ".join(missing))
    print(f"BUILD TREE: PASS ({len(entries)} manifest sources + startup/linker/CMSIS/HAL)")
    return entries


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--toolchain", help="Arm GNU toolchain bin directory or arm-none-eabi-gcc path")
    ap.add_argument("--variant", choices=["normal", "diagnostic"], default="normal")
    ap.add_argument("--mode", choices=["developer", "repro", "auto"], default="developer",
                    help="developer=DEV-NONCANONICAL, repro=explicit version, auto=reserve from global allocator")
    ap.add_argument("--version", default="", help="required for --mode repro")
    ap.add_argument("--output-dir", default=".build/python-target")
    ap.add_argument("--check-only", action="store_true", help="validate complete build inputs without requiring compiler")
    ap.add_argument("--init-state", type=int, metavar="HWM", default=0,
                    help="initialize version state with given HWM (e.g. --init-state 600)")
    args = ap.parse_args()

    # Handle --init-state
    if args.init_state > 0:
        state_root = get_version_state_root(ROOT)
        init_version_state(state_root, args.init_state)
        return 0

    entries = check_tree()
    if args.check_only:
        # Exercise the portable container code on a nontrivial deterministic fixture too.
        fixture = bytes((i * 37 + 11) & 0xFF for i in range(4097))
        packed, info = build_container(fixture)
        if len(packed) != len(fixture) + 36 or info["raw_length"] != len(fixture):
            raise SystemExit("BL820 packager check failed")
        print("BL820 PACKAGER: PASS")
        return 0

    # Resolve version(s)
    versions: list[str] = []
    version_source = ""
    if args.mode == "developer":
        versions = ["DEV-NONCANONICAL"]
        version_source = "developer_noncanonical"
    elif args.mode == "repro":
        if not args.version:
            raise SystemExit("--mode repro requires --version")
        versions = [args.version]
        version_source = "repro_explicit"
    elif args.mode == "auto":
        if args.version:
            raise SystemExit("--mode auto allocates globally; do not supply --version (use --mode repro for explicit version)")
        state_root = get_version_state_root(ROOT)
        current_hwm, current_ver = get_canonical_version(state_root)
        # Reserve TWO consecutive versions: normal + diagnostic (matches build-canonical-pair.ps1)
        versions = reserve_canonical_version_pair(state_root)
        version_source = "auto_global"
        print(f"VERSION PRECHECK")
        print(f"Highest issued canonical version: {current_ver}")
        print(f"Allocator source: {state_root}")
        print(f"Atomic reservation: YES")
        print(f"Reserved pair: NORMAL={versions[0]} DIAG={versions[1]}")
        print(f"Monotonic candidate: PASS")
        print()

    if not versions:
        raise SystemExit("Version not resolved")
    for v in versions:
        if not re.match(r"^[0-9A-Za-z][0-9A-Za-z._+-]{0,47}$", v):
            raise SystemExit(f"unsafe version string: {v}")

    # Auto mode builds the canonical PAIR; every other mode builds the variant asked for.
    # Honouring --variant here is not cosmetic: without it a diagnostic target build
    # silently produced a normal binary, so the diagnostic variant was never compiled by
    # any gate that passes --variant with --mode developer.
    variants_to_build = [(args.variant,)]
    if args.mode == "auto":
        variants_to_build = [("normal",), ("diagnostic",)]
        if len(versions) < 2:
            raise SystemExit("auto mode requires 2 reserved versions (normal + diagnostic)")

    results = []
    for idx, (variant,) in enumerate(variants_to_build):
        version = versions[idx]
        result = _build_one(version, variant, version_source, entries, args.toolchain, args.output_dir)
        results.append(result)
        # In auto mode, print a separator between variants
        if args.mode == "auto" and idx < len(variants_to_build) - 1:
            print()

    if args.mode == "auto":
        print("\nPAIR BUILD: PASS")
        print(f"NORMAL: {results[0]['version']} -> {results[0]['release_bin']}")
        print(f"DIAG:   {results[1]['version']} -> {results[1]['release_bin']}")
        return 0

    # Single-variant mode (developer/repro): return first result
    return 0

    gcc = find_tool("arm-none-eabi-gcc", args.toolchain)
    toolbin = str(Path(gcc).resolve().parent)
    objcopy = find_tool("arm-none-eabi-objcopy", toolbin)
    size = find_tool("arm-none-eabi-size", toolbin)
    nm = find_tool("arm-none-eabi-nm", toolbin)
    readelf = find_tool("arm-none-eabi-readelf", toolbin)
    gcc_version = run([gcc, "-dumpfullversion", "-dumpversion"], capture=True).strip().splitlines()[0]
    if gcc_version != EXPECTED_GCC:
        raise SystemExit(f"unsupported Arm GNU Toolchain {gcc_version}; expected {EXPECTED_GCC}")

    outroot = Path(args.output_dir)
    if not outroot.is_absolute():
        outroot = ROOT / outroot
    userdir = outroot / "M820_BL820"
    work = userdir / "work" / args.variant
    gen = work / "generated"
    objdir = work / "objects"
    gen.mkdir(parents=True, exist_ok=True)
    objdir.mkdir(parents=True, exist_ok=True)
    (gen / "build_version.h").write_text(
        '#ifndef BUILD_VERSION_H\n#define BUILD_VERSION_H\n'
        '/* Generated in the build directory; never edit or commit. */\n'
        f'#define EBICS_BUILD_VERSION "{version}"\n#endif\n', encoding="ascii")

    defs = ["-DGD32F30X_HD", "-DGD_ECLIPSE_GCC", "-DUSE_STDPERIPH_DRIVER", "-DBOOTLOADER=820",
            f"-DCAN_DIAGNOSTICS_ENABLE={1 if args.variant == 'diagnostic' else 0}"]
    common = ["-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"]
    inc = [f"-I{gen}", f"-I{ROOT/'inc'}", f"-I{ROOT/'Firmware/CMSIS'}",
           f"-I{ROOT/'Firmware/CMSIS/GD/GD32F30x/Include'}",
           f"-I{ROOT/'Firmware/GD32F30x_standard_peripheral/Include'}"]
    cflags = common + ["-O0", "-g3", "-fmessage-length=0", "-fsigned-char",
                       "-ffunction-sections", "-fdata-sections", "-Wall"] + defs + inc

    objects: list[str] = []
    for entry in entries:
        obj = objdir / (re.sub(r"[\\/:]", "_", entry) + ".o")
        run([gcc, *cflags, "-c", str(ROOT / entry), "-o", str(obj)])
        objects.append(str(obj))
    startup_obj = objdir / "startup_gd32f30x_hd.S.o"
    run([gcc, *common, *defs, *inc, "-x", "assembler-with-cpp", "-c",
         str(ROOT / "gcc_startup/startup_gd32f30x_hd.S"), "-o", str(startup_obj)])
    objects.append(str(startup_obj))

    base = version
    elf, rawbin, hexp = work/base, work/f"{base}.bin", work/f"{base}.hex"
    elf = elf.with_suffix(".elf")
    mapf = work/f"{base}.map"
    suffix = "_DIAG" if args.variant == "diagnostic" else ""
    final = userdir/f"{base}_M820_BL820{suffix}.bin"
    manifest = userdir/f"{base}_M820_BL820{suffix}.manifest.json"
    linker = ROOT / "ldscripts/gd32f30x_flash.ld"
    link = [gcc, *common, f"-T{linker}", "-Wl,--gc-sections", "-Wl,--print-memory-usage",
            f"-Wl,-Map,{mapf}", "-Wl,--start-group", *objects,
            f"-L{ROOT/'Firmware/CMSIS'}", "-larm_cortexM4lf_math", "-specs=nano.specs", "-specs=nosys.specs",
            "-Wl,--end-group", "-o", str(elf)]
    run(link)
    run([objcopy, "-O", "binary", str(elf), str(rawbin)])
    run([objcopy, "-O", "ihex", str(elf), str(hexp)])

    size_out = run([size, str(elf)], capture=True)
    size_a = run([size, "-A", str(elf)], capture=True)
    (work/f"{base}.size.txt").write_text(size_out + "\n" + size_a, encoding="ascii")
    ph = run([readelf, "-l", str(elf)], capture=True)
    (work/f"{base}.program-headers.txt").write_text(ph, encoding="ascii")
    if re.search(r"\bRWE\b", ph):
        raise SystemExit("ELF contains an RWE load segment")
    nm_text = run([nm, "--defined-only", str(elf)], capture=True)
    symbols = symbol_map(nm_text)
    app_start = require_symbol(symbols, "__app_flash_start")
    app_limit = require_symbol(symbols, "__app_flash_limit")
    image_end = require_symbol(symbols, "__flash_image_end")
    config_a = require_symbol(symbols, "__config_a_start")
    if app_start != FLASH_ORIGIN or config_a != CONFIG_A_ORIGIN or image_end > app_limit or app_limit > config_a:
        raise SystemExit("linked memory map does not match M820_BL820 target")
    diag_symbol = "diag_session_dump_step" in symbols
    if diag_symbol != (args.variant == "diagnostic"):
        raise SystemExit("diagnostic link-marker mismatch")

    cols = size_out.strip().splitlines()[-1].split()
    if len(cols) < 4:
        raise SystemExit("cannot parse arm-none-eabi-size output")
    text_b, data_b, bss_b = map(int, cols[:3])
    sdata, edata = require_symbol(symbols, "_sdata"), require_symbol(symbols, "_edata")
    sbss, ebss = require_symbol(symbols, "_sbss"), require_symbol(symbols, "_ebss")
    sp = require_symbol(symbols, "_sp")

    packed, packinfo = build_container(rawbin.read_bytes())
    final.parent.mkdir(parents=True, exist_ok=True)
    final.write_bytes(packed)
    if version.encode("ascii") not in final.read_bytes():
        raise SystemExit("firmware identity string missing from final BL820 artifact")

    commit = git_value("rev-parse", "HEAD")
    describe = git_value("describe", "--tags", "--always")
    dirty = False
    if (ROOT / ".git").exists():
        dirty = bool(git_value("status", "--porcelain", "--untracked-files=normal"))
    doc = {
        "schema_version": 1, "target": "M820_BL820", "profile": "debug", "variant": args.variant,
        "diagnostics_enabled": args.variant == "diagnostic", "version": version,
        "version_source": version_source,
        "git_commit": commit, "git_description": describe, "worktree_dirty": dirty,
        "hardware_approved_profile": True,
        "toolchain": "Arm GNU Toolchain arm-none-eabi", "toolchain_version": gcc_version,
        "linker": "ldscripts/gd32f30x_flash.ld", "source_manifest": "scripts/sources-m820.txt",
        "source_count": len(entries),
        "flash": {"origin": f"0x{app_start:08X}", "image_end": f"0x{image_end:08X}",
                  "limit": f"0x{app_limit:08X}", "config_a": f"0x{config_a:08X}",
                  "text_bytes": text_b, "gnu_size_data_bytes": data_b,
                  "data_load_bytes": edata-sdata, "binary_bytes": rawbin.stat().st_size},
        "ram": {"data_bytes": edata-sdata, "bss_bytes": ebss-sbss,
                "heap_stack_reserved_bytes": sp-ebss,
                "used_including_heap_stack_bytes": sp-RAM_ORIGIN,
                "gnu_size_bss_bytes": bss_b},
        "packaging": {k: (f"0x{v:08X}" if k == "stm32_crc32" else f"0x{v:04X}" if k in ("header_size_modulo","header_crc16") else v)
                      for k,v in packinfo.items()},
        "artifacts": {"elf": str(elf), "map": str(mapf), "raw_binary": str(rawbin),
                      "raw_binary_sha256": sha256(rawbin), "final_binary": str(final),
                      "final_binary_bytes": final.stat().st_size, "final_binary_sha256": sha256(final)}
    }
    manifest.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    print("\n==================================================")
    print("eVistDrive M820_BL820 CROSS-PLATFORM BUILD: PASS")
    print(f"Version:   {version}")
    print(f"Compiler:  {gcc_version}")
    print(f"Sources:   {len(entries)} + startup")
    print(f"Raw BIN:   {rawbin.stat().st_size} B")
    print(f"Final BIN: {final}")
    print(f"SHA256:    {sha256(final)}")
    print("==================================================")

    # Copy final BIN to releases/<version>_M820_BL820.bin (canonical releases only)
    if version_source == "auto_global":
        releases_dir = ROOT / "releases"
        releases_dir.mkdir(parents=True, exist_ok=True)
        dest = releases_dir / final.name
        shutil.copy2(final, dest)
        print(f"RELEASE:   {dest}")
        print(f"Size:      {dest.stat().st_size} B")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
