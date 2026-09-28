#!/usr/bin/env python3
"""M820 target stack-budget gate (fail-closed regression gate for the M820/GD32F303RC build).

Runs on a FINAL linked ELF, its linker map and the GCC -fstack-usage (.su) files of the same
build, and fails when the worst reachable stack does not fit the linker-reserved stack with
margin, or when any input needed for that proof is missing, ambiguous or unsupported.

Method (reviewed in TQ-06 E30 root-cause review, 2026-09-28; hardened after the 09530fb review):
  * functions are keyed by ADDRESS (equal static names in different units do not collapse);
  * every instruction that writes SP must be a SUPPORTED, BOUNDED form: push/stmdb sp!/vpush/
    vstmdb sp!, sub sp #imm, str/strd [sp,#-imm]! (decrements); pop/ldm sp!/vpop, add sp #imm,
    ldr [sp],#imm (releases); `mov sp, r7` only when every r7 write in that function is a
    frame form (add r7, sp, #imm / mov r7, sp / add r7, #imm) or a restore directly before a
    return. Anything else is a DYNAMIC/UNSUPPORTED SP WRITE and fails;
  * frame of a function from a C object of this build = its own .su record (exactly one
    record, qualifier "static", equal to the audited disassembly). A missing .su file, missing
    record, duplicate record or contradiction fails. Functions from toolchain libraries or
    assembly (no .su exists) use the audited disassembly frame as the verified alternative;
  * call graph from direct bl/blx/b<cond> to a function start (self edges included; tail
    calls counted as calls = over-approximation). Every indirect call (blx/bx rN) may reach any
    address-taken function (address in an allocated data section, a .text literal or a
    movw/movt pair, outside the vector table). Any reachable cycle, direct or indirect, fails;
  * foreground = Reset_Handler graph; ISR addition = realistic nesting of the CURRENT M820
    NVIC topology: one priority-0 entry (ADC/TIMER/CAN/SysTick/default; equal preemption), or
    one lower-priority EXTI handler preempted by one priority-0 handler. Exception frames:
    104 B FP-extended + 4 B alignment when the preempted context executes FP code, else 32+4.
    Fault/NMI handlers are terminal and are not part of the ride budget.
  * The ISR model is valid ONLY for the NVIC topology fingerprinted below (vector slots and the
    decoded priority-configuration calls). Any difference fails with "IRQ TOPOLOGY CHANGED":
    the model must be reviewed and this gate updated; it never assumes the old model silently.
"""
from __future__ import annotations
import argparse, json, os, re, subprocess, sys
from bisect import bisect_right as _bisect_right
from pathlib import Path

PHYSICAL_RAM_ORIGIN = 0x20000000
PHYSICAL_RAM_END = 0x2000C000          # GD32F303RC: 48 KiB SRAM
MIN_MARGIN_BYTES = 1024                # meaningful headroom below the reserved stack
FP_FRAME = 104 + 4
BASIC_FRAME = 32 + 4

# --- Current M820 NVIC topology (source: src/main.c nvic_config(), src/systick.c) ---------
EXPECTED_VECTOR_SLOTS = {              # vector index -> handler (all other slots: default)
    1: "Reset_Handler", 2: "NMI_Handler", 3: "HardFault_Handler", 4: "MemManage_Handler",
    5: "BusFault_Handler", 6: "UsageFault_Handler", 11: "SVC_Handler", 12: "DebugMon_Handler",
    14: "PendSV_Handler", 15: "SysTick_Handler", 24: "EXTI2_IRQHandler",
    34: "ADC0_1_IRQHandler", 37: "CAN0_RX1_IRQHandler", 44: "TIMER1_IRQHandler",
    45: "TIMER2_IRQHandler", 56: "EXTI10_15_IRQHandler",
}
PRIORITY_FUNCS = {"nvic_irq_enable": 3, "nvic_priority_group_set": 1, "NVIC_SetPriority": 2}
EXPECTED_PRIORITY_CALLS = {            # caller -> ordered decoded calls
    "nvic_config": [
        ("nvic_irq_enable", (21, 0, 0)), ("nvic_irq_enable", (40, 2, 0)),
        ("nvic_irq_enable", (8, 2, 0)), ("nvic_priority_group_set", (0x600,)),
        ("nvic_irq_enable", (28, 0, 0)), ("nvic_irq_enable", (29, 0, 0)),
        ("nvic_irq_enable", (18, 0, 0)), ("nvic_irq_enable", (50, 0, 0)),
    ],
    "SysTick_Config": [("NVIC_SetPriority", (-1, 15))],
    "systick_config": [("NVIC_SetPriority", (-1, 0))],
}
LIBRARY_INTERNAL_CALLERS = {"nvic_irq_enable"}   # its fallback group set is not a config site
LOWER_PRIORITY_HANDLERS = {"EXTI2_IRQHandler", "EXTI10_15_IRQHandler"}  # pre 2 before PRE1_SUB3
TERMINAL_HANDLERS = {"NMI_Handler", "HardFault_Handler", "MemManage_Handler", "BusFault_Handler",
                     "UsageFault_Handler"}

_HDR = re.compile(r"^([0-9a-f]{8}) <([^>]+)>:")
_INS = re.compile(r"^\s*([0-9a-f]+):\s+(\S+)\s*(.*)$")
_TGT = re.compile(r"^([0-9a-f]+)\s+<([^>+]+)(\+0x[0-9a-f]+)?>")
_COND = ("eq", "ne", "cs", "hs", "cc", "lo", "mi", "pl", "vs", "vc", "hi", "ls", "ge", "lt",
         "gt", "le", "al")
_NO_DEST = {"str", "strb", "strh", "strd", "stm", "stmia", "stmdb", "stmea", "vstr", "vstm",
            "vstmia", "vstmdb", "push", "vpush", "cmp", "cmn", "tst", "teq", "vcmp", "vcmpe",
            "b", "bl", "blx", "bx", "cbz", "cbnz", "it", "nop", "dmb", "dsb", "isb", "pld",
            "tbb", "tbh", "bkpt", "svc", "wfi", "wfe", "cpsid", "cpsie"}


class GateError(Exception):
    pass


def _run(cmd: list[str]) -> str:
    p = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    if p.returncode:
        raise GateError(f"{Path(cmd[0]).name} failed: {p.stderr.strip()[:300]}")
    return p.stdout


def _tool(toolbin: str, name: str) -> str:
    for cand in (Path(toolbin) / (name + ".exe"), Path(toolbin) / name):
        if cand.is_file():
            return str(cand)
    raise GateError(f"{name} not found in {toolbin}")


def _base(op: str) -> tuple[str, bool]:
    """Mnemonic without width suffix and condition code; second value = conditional."""
    b = op.split(".")[0]
    if b in _NO_DEST or b in ("pop", "vpop", "ldmia", "ldm", "add", "sub", "mov", "ldr"):
        return b, False
    for c in _COND:
        if b.endswith(c) and len(b) > len(c) and b[:-len(c)] in (
                "pop", "vpop", "push", "add", "sub", "mov", "ldr", "ldmia", "ldm", "b", "bx"):
            return b[:-len(c)], True
    for c in ("s",):   # flag-setting forms: adds/subs/movs
        if b.endswith(c) and b[:-1] in ("add", "sub", "mov"):
            return b[:-1], False
    return b, False


def _regs(args: str) -> list[str]:
    m = re.search(r"\{([^}]*)\}", args)
    if not m:
        return []
    out = []
    for part in (p.strip() for p in m.group(1).split(",")):
        if "-" in part:
            a, b = part.split("-")
            pre = re.match(r"[a-z]+", a).group(0)
            out += [f"{pre}{i}" for i in range(int(a[len(pre):]), int(b[len(pre):]) + 1)]
        elif part:
            out.append(part)
    return out


def _imm(s: str) -> int | None:
    m = re.search(r"#(-?\d+)", s)
    return int(m.group(1)) if m else None


def _parse_disassembly(text: str):
    """Functions (address -> name, instructions) and .text literal words (address -> value).
    Each instruction is (address, mnemonic, operands, pc-relative literal address or None)."""
    funcs: dict[int, dict] = {}
    words: dict[int, int] = {}
    refs: set[int] = set()
    cur = None
    for line in text.splitlines():
        m = _HDR.match(line)
        if m:
            cur = {"name": m.group(2), "ins": []}
            funcs[int(m.group(1), 16)] = cur
            continue
        m = _INS.match(line)
        if m and cur is not None:
            raw = m.group(3)
            op, args = m.group(2), raw.split("@")[0].strip()
            lit = re.search(r"@\s*\(([0-9a-f]+)\s", raw)
            if op == ".word":
                try:
                    words[int(m.group(1), 16)] = int(args.split()[0], 16)
                except ValueError:
                    pass
            else:
                cur["ins"].append((int(m.group(1), 16), op, args, int(lit.group(1), 16) if lit else None))
                if not op.startswith(("b", "cb")):
                    refs.update(int(x, 16) for x in re.findall(r"([0-9a-f]{7,8}) <", raw))
    return funcs, words, refs


def _writes(op: str, args: str, reg: str) -> bool:
    """Does the instruction write `reg` (as destination, writeback or register list)?"""
    b, _ = _base(op)
    if b.startswith(("pop", "vpop", "ldm", "vldm")) and reg in _regs(args):
        return True
    if re.search(rf"\[{reg}(,[^\]]*)?\]!|\[{reg}\],", args):          # base writeback
        return True
    if b in _NO_DEST:
        return False
    ops = [o.strip() for o in args.split(",")]
    if b in ("ldrd", "ldrexd", "umull", "smull", "umlal", "smlal", "vmov") and reg in ops[:2]:
        return True
    return ops[0] == reg


def _sp_effect(op: str, args: str) -> tuple[str, int]:
    """Classify an SP-writing instruction. Returns (kind, bytes): kind in
    'dec', 'rel', 'none', 'fp' (mov sp, r7) or 'bad'."""
    b, _ = _base(op)
    if b in ("push",):
        return "dec", 4 * len(_regs(args))
    if b in ("vpush",):
        r = _regs(args)
        return "dec", (8 if r and r[0].startswith("d") else 4) * len(r)
    if b in ("stmdb", "stmfd") and args.startswith("sp!"):
        return "dec", 4 * len(_regs(args))
    if b in ("vstmdb",) and args.startswith("sp!"):
        r = _regs(args)
        return "dec", (8 if r and r[0].startswith("d") else 4) * len(r)
    if b in ("pop", "vpop") or (b in ("ldmia", "ldm", "ldmfd", "vldmia") and args.startswith("sp!")):
        if "sp" in _regs(args):
            return "bad", 0
        return "rel", 0
    dest = args.split(",")[0].strip()
    m = re.match(r"^\S+,\s*(?:\S+,\s*)?\[sp,\s*#(-?\d+)\]!$", args)          # pre-index writeback
    if m:
        if dest == "sp":
            return "bad", 0
        n = int(m.group(1))
        if b in ("str", "strd", "strb", "strh") and n < 0:
            return "dec", -n
        return "bad", 0
    m = re.match(r"^\S+,\s*(?:\S+,\s*)?\[sp\],\s*#(-?\d+)$", args)            # post-index writeback
    if m:
        if dest == "sp":
            return "bad", 0
        n = int(m.group(1))
        if b in ("ldr", "ldrd") and n > 0:
            return "rel", 0
        return "bad", 0
    if args.startswith("sp!"):                       # any other SP writeback form (stmia sp!, ...)
        return "bad", 0
    first = dest
    if first == "sp" and b not in _NO_DEST:
        if b in ("sub", "subw") and re.fullmatch(r"sp,\s*(sp,\s*)?#\d+", args):
            return "dec", _imm(args)
        if b in ("add", "addw") and re.fullmatch(r"sp,\s*(sp,\s*)?#\d+", args):
            return "rel", 0
        if b == "mov" and re.fullmatch(r"sp,\s*r7", args):
            return "fp", 0
        return "bad", 0
    if b in ("msr",) and re.match(r"(msp|psp)", args, re.I):
        return "bad", 0
    if "sp" in _regs(args) and b.startswith(("ldm", "pop")):
        return "bad", 0
    return "none", 0


def _audit_function(f: dict) -> tuple[int, bool, list[str]]:
    """Returns (frame bytes from SP decrements, uses FP, unsupported SP writes)."""
    frame, uses_fp, bad = 0, False, []
    ins = f["ins"]
    has_fp_restore = False
    for i, (a, op, args, _lit) in enumerate(ins):
        b, _ = _base(op)
        if b.startswith("v") and b not in ("vpush", "vpop"):
            uses_fp = True
        kind, n = _sp_effect(op, args)
        if kind == "dec":
            frame += n
            if b in ("vpush", "vstmdb"):
                uses_fp = True
        elif kind == "bad":
            bad.append(f"{a:08x} {op} {args}")
        elif kind == "fp":
            has_fp_restore = True
    if has_fp_restore:
        frame_form = False
        for i, (a, op, args, _lit) in enumerate(ins):
            if _sp_effect(op, args)[0] == "fp" and not frame_form:
                bad.append(f"{a:08x} {op} {args} (mov sp, r7 before the r7 frame-pointer setup)")
            if not _writes(op, args, "r7"):
                continue
            b, _ = _base(op)
            if b == "add" and re.fullmatch(r"r7,\s*sp,\s*#\d+", args):
                frame_form = True
                continue
            if b == "mov" and re.fullmatch(r"r7,\s*sp", args):
                frame_form = True
                continue
            if not frame_form:
                bad.append(f"{a:08x} {op} {args} (r7 written before its frame-pointer setup)")
                continue
            if b == "add" and re.fullmatch(r"r7,\s*(r7,\s*)?#\d+", args):
                continue
            is_restore = (b in ("pop", "ldmia", "ldm") and (args.startswith("sp!") or b == "pop")) or \
                         (b == "ldr" and re.fullmatch(r"r7,\s*\[sp\],\s*#\d+", args))
            nxt = ins[i + 1] if i + 1 < len(ins) else None
            returns = "pc" in _regs(args) or (nxt is not None and _base(nxt[1])[0] == "bx"
                                              and nxt[2].strip() == "lr")
            if is_restore and returns:
                continue
            bad.append(f"{a:08x} {op} {args} (r7 not a proven frame pointer for mov sp, r7)")
        if not frame_form:
            bad.append("mov sp, r7 without a proven r7 frame-pointer setup")
    return frame, uses_fp, bad


def _literal_call_target(f: dict, idx: int, reg: str, words: dict[int, int]) -> int | None:
    """Value of `reg` at instruction idx if it is PROVABLY loaded by `ldr reg, [pc, #]` from a
    .text literal, with no write to reg and no in-function branch target in between.
    Returns the literal value, or None when not provable (caller then stays conservative)."""
    ins = f["ins"]
    targets = set()
    for a, op, args, _ in ins:
        m = _TGT.match(args)
        if _base(op)[0] == "b" and m and m.group(2) == f["name"]:
            targets.add(int(m.group(1), 16))
    for j in range(idx - 1, -1, -1):
        a, op, args, lit = ins[j]
        if ins[j + 1][0] in targets:
            return None
        if _writes(op, args, reg):
            if _base(op)[0] == "ldr" and lit is not None and re.match(rf"{reg},\s*\[pc", args):
                return words.get(lit)
            return None
    return None


def _edges(addr: int, f: dict, starts: set[int], words: dict[int, int], sorted_starts: list[int]):
    """Direct callees (self CALLS kept: recursion), unresolved indirect call sites, and jumps
    into the body of another function. A jump into another function's body is treated as a
    call of that whole function (its frame is added: over-approximation). An indirect call
    through a provable literal is resolved: 0 -> no call (weak undefined), function -> edge."""
    direct, indirect, cross = set(), [], []
    for i, (a, op, args, _) in enumerate(f["ins"]):
        b, _ = _base(op)
        if b not in ("b", "bl", "blx", "bx"):
            continue
        m = _TGT.match(args)
        if m:
            tgt = int(m.group(1), 16)
            if m.group(3) is None and tgt in starts:
                if b in ("bl", "blx") or tgt != addr:     # a plain jump to its own start is a loop
                    direct.add(tgt)
            elif m.group(2) != f["name"]:
                k = _bisect_right(sorted_starts, tgt) - 1
                if k < 0:
                    cross.append(f"{a:08x} {op} {args}")
                else:
                    direct.add(sorted_starts[k])
        elif b == "blx" or (b == "bx" and args.strip() != "lr"):
            reg = args.strip()
            val = _literal_call_target(f, i, reg, words)
            if val == 0:
                continue                                   # weak undefined: guarded null call
            if val is not None and (val & 1) and (val & ~1) in starts:
                direct.add(val & ~1)
                continue
            indirect.append(a)
    return direct, indirect, cross


def _movw_movt_words(funcs: dict) -> set[int]:
    out = set()
    for f in funcs.values():
        lo: dict[str, int] = {}
        for _, op, args, _lit in f["ins"]:
            b = op.split(".")[0]
            m = re.match(r"(r\d+|ip|lr|fp|sl),\s*#(\d+)", args)
            if b == "movw" and m:
                lo[m.group(1)] = int(m.group(2))
            elif b == "movt" and m and m.group(1) in lo:
                out.add((int(m.group(2)) << 16) | lo.pop(m.group(1)))
    return out


def _alloc_data_words(objdump: str, readelf: str, elf: str) -> set[int]:
    words: set[int] = set()
    secs, alloc_rows = [], set()
    for line in _run([readelf, "-S", "-W", elf]).splitlines():
        m = re.match(r"^\s*\[\s*\d+\]\s+(\S+)\s+(\S+)\s+[0-9a-f]+\s+[0-9a-f]+\s+[0-9a-f]+\s+\S+\s+(\S*)", line)
        if m and "A" in m.group(3):
            alloc_rows.add(m.group(1))
            if m.group(2) != "NOBITS" and m.group(1) not in (".vectors", ".text"):
                secs.append(m.group(1))
    if not {".vectors", ".text"} <= alloc_rows:        # the section table parse itself must work
        raise GateError(f"section table not understood for pointer scan (allocated: {sorted(alloc_rows)})")
    for sec in secs:
        p = subprocess.run([objdump, "-s", "-j", sec, elf], capture_output=True, text=True)
        if p.returncode:
            raise GateError(f"cannot read section {sec}")
        data, base = bytearray(), None
        for line in p.stdout.splitlines():
            m = re.match(r"^ ([0-9a-f]{4,8}) ((?:[0-9a-f]{2,8} ?){1,4})", line)
            if m:
                base = int(m.group(1), 16) if base is None else base
                data += bytes.fromhex(m.group(2).replace(" ", ""))
        if base is not None:
            for off in range((-base) % 4, len(data) - 3, 4):
                words.add(int.from_bytes(data[off:off + 4], "little"))
    return words


def _vectors(objdump: str, elf: str) -> list[int]:
    p = subprocess.run([objdump, "-s", "-j", ".vectors", elf], capture_output=True, text=True)
    data = bytearray()
    for line in p.stdout.splitlines():
        m = re.match(r"^ ([0-9a-f]{4,8}) ((?:[0-9a-f]{2,8} ?){1,4})", line)
        if m:
            data += bytes.fromhex(m.group(2).replace(" ", ""))
    if len(data) < 8:
        raise GateError("cannot read .vectors")
    return [int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data) - 3, 4)]


def _symbols(nm: str, elf: str) -> tuple[dict[str, int], dict[int, set[str]]]:
    by_name, by_addr = {}, {}
    for line in _run([nm, "--defined-only", elf]).splitlines():
        p = line.split()
        if len(p) == 3:
            v = int(p[0], 16)
            by_name.setdefault(p[2], v)
            if p[1] in "TtWw":
                by_addr.setdefault(v & ~1, set()).add(p[2])
    return by_name, by_addr


def _map_objects(mapfile: str) -> list[tuple[int, int, str]]:
    text = Path(mapfile).read_text(encoding="utf-8", errors="replace")
    i = text.find("Linker script and memory map")
    if i < 0:
        raise GateError("linker map has no memory map section")
    out, pending = [], None
    for line in text[i:].splitlines():
        m = re.match(r"^ (\.\S+)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(.+)$", line)
        if m:
            out.append((int(m.group(2), 16), int(m.group(3), 16), m.group(4).strip()))
            pending = None
            continue
        m = re.match(r"^ (\.\S+)\s*$", line)
        if m:
            pending = m.group(1)
            continue
        m = re.match(r"^\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S.+)$", line)
        if m and pending:
            out.append((int(m.group(1), 16), int(m.group(2), 16), m.group(3).strip()))
        pending = None
    out = [o for o in out if o[1] > 0]
    if not out:
        raise GateError("linker map lists no input sections")
    return sorted(out)


def _object_of(addr: int, sections: list[tuple[int, int, str]]) -> str | None:
    lo, hi = 0, len(sections) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        s, n, o = sections[mid]
        if addr < s:
            hi = mid - 1
        elif addr >= s + n:
            lo = mid + 1
        else:
            return o
    return None


def _norm(p: str) -> str:
    return os.path.normcase(os.path.abspath(p))


def _load_su(path: str) -> tuple[dict[str, list[int]], list[str]]:
    recs: dict[str, list[int]] = {}
    problems = []
    for line in open(path, encoding="utf-8", errors="replace"):
        parts = line.rstrip("\n").split("\t")
        if not line.strip():
            continue
        if len(parts) != 3 or not parts[1].isdigit():
            problems.append(f"malformed .su line in {os.path.basename(path)}: {line.strip()[:80]}")
            continue
        name = parts[0].rsplit(":", 1)[-1]
        if parts[2] != "static":
            problems.append(f"DYNAMIC STACK ALLOCATION: {name} ({parts[2]}) in {os.path.basename(path)}")
        recs.setdefault(name, []).append(int(parts[1]))
    return recs, problems


def _decode_args(ins: list, idx: int, nargs: int) -> tuple | None:
    vals: dict[str, int] = {}
    for a, op, args, _lit in reversed(ins[max(0, idx - 12):idx]):
        b, _ = _base(op)
        if b in ("b", "bl", "blx", "bx", "cbz", "cbnz", "pop") or b.startswith("it"):
            break
        for r in (f"r{i}" for i in range(nargs)):
            if r in vals or not _writes(op, args, r):
                continue
            m = re.fullmatch(rf"{r},\s*#(-?\d+)", args)
            if b in ("mov", "movw") and m:
                v = int(m.group(1)) & 0xFFFFFFFF
                vals[r] = v - (1 << 32) if v & 0x80000000 else v
            else:
                vals[r] = None
    if any(vals.get(f"r{i}") is None for i in range(nargs)):
        return None
    return tuple(vals[f"r{i}"] for i in range(nargs))


def _irq_topology(vec, funcs, by_addr, name_to_addr) -> list[str]:
    errs = []
    counts: dict[int, int] = {}
    for w in vec[1:]:
        if w:
            counts[w & ~1] = counts.get(w & ~1, 0) + 1
    default = max(counts, key=counts.get)
    for slot, w in enumerate(vec):
        if slot == 0:
            continue
        exp = EXPECTED_VECTOR_SLOTS.get(slot)
        names = by_addr.get(w & ~1, set()) if w else set()
        if exp is None:
            if w and (w & ~1) != default:
                errs.append(f"IRQ TOPOLOGY CHANGED: vector slot {slot} has new handler {sorted(names)}")
        elif exp not in names:
            errs.append(f"IRQ TOPOLOGY CHANGED: vector slot {slot} expected {exp}, found {sorted(names) or 'none'}")
    targets = {name_to_addr[n]: n for n in PRIORITY_FUNCS if n in name_to_addr}
    found: dict[str, list] = {}
    for addr, f in funcs.items():
        caller = f["name"]
        for i, (a, op, args, _lit) in enumerate(f["ins"]):
            m = _TGT.match(args)
            if _base(op)[0] != "bl" or not m or int(m.group(1), 16) not in targets:
                continue
            callee = targets[int(m.group(1), 16)]
            if caller in LIBRARY_INTERNAL_CALLERS:
                continue
            decoded = _decode_args(f["ins"], i, PRIORITY_FUNCS[callee])
            if decoded is None:
                errs.append(f"IRQ TOPOLOGY UNDECODABLE: {caller} {a:08x} call to {callee}")
                continue
            found.setdefault(caller, []).append((callee, decoded))
    if found != EXPECTED_PRIORITY_CALLS:
        errs.append(f"IRQ TOPOLOGY CHANGED: priority configuration {found} != reviewed "
                    f"{EXPECTED_PRIORITY_CALLS}; review the ISR model and update the gate")
    return errs


def check(elf: str, objdir: str, mapfile: str, toolbin: str, variant: str) -> dict:
    for p, what in ((elf, "ELF"), (mapfile, "linker map"), (objdir, "object directory")):
        if not p or not os.path.exists(p):
            raise GateError(f"missing {what}: {p}")
    objdump, nm, readelf = (_tool(toolbin, n) for n in
                            ("arm-none-eabi-objdump", "arm-none-eabi-nm", "arm-none-eabi-readelf"))
    sym, by_addr = _symbols(nm, elf)
    for need in ("_sp", "_heap_end", "_ebss", "__stack_size", "__heap_size", "Reset_Handler"):
        if need not in sym:
            raise GateError(f"required symbol missing: {need}")
    sp, heap_end, ebss = sym["_sp"], sym["_heap_end"], sym["_ebss"]
    stack_size, heap_size = sym["__stack_size"], sym["__heap_size"]
    vec = _vectors(objdump, elf)
    errors: list[str] = []
    if not (PHYSICAL_RAM_ORIGIN < ebss <= heap_end < sp <= PHYSICAL_RAM_END):
        errors.append("RAM layout outside physical 0x20000000..0x2000C000 or out of order")
    if sp - heap_end != stack_size:
        errors.append(f"_sp-_heap_end={sp - heap_end} != __stack_size={stack_size}")
    if heap_end - ebss < heap_size:
        errors.append(f"heap {heap_end - ebss} B < __heap_size {heap_size}")
    if vec[0] != sp:
        errors.append(f"initial MSP 0x{vec[0]:08X} != _sp 0x{sp:08X}")

    funcs, text_words, text_refs = _parse_disassembly(_run([objdump, "-d", "--no-show-raw-insn", elf]))
    if not funcs:
        raise GateError("no functions in disassembly")
    starts = set(funcs)
    name = {a: f["name"] for a, f in funcs.items()}
    name_to_addr: dict[str, int] = {}
    for a, names in by_addr.items():
        if a in funcs:
            for n in names:
                name_to_addr.setdefault(n, a)
    errors += _irq_topology(vec, funcs, by_addr, name_to_addr)

    audit = {a: _audit_function(f) for a, f in funcs.items()}
    sorted_starts = sorted(starts)
    graph = {a: _edges(a, f, starts, text_words, sorted_starts) for a, f in funcs.items()}
    vector_targets = {w & ~1 for w in vec[1:] if w}
    taken = set(text_words.values()) | _alloc_data_words(objdump, readelf, elf) | _movw_movt_words(funcs)
    address_taken = {w & ~1 for w in taken if (w & 1) and (w & ~1) in starts} | (text_refs & starts)

    def succ(a: int) -> list[int]:
        direct, indirect, _ = graph[a]
        return sorted(direct | (address_taken if indirect else set()))

    reset = vec[1] & ~1
    if reset not in funcs or name_to_addr.get("Reset_Handler") != reset:
        raise GateError("stack root Reset_Handler not found at vector slot 1")
    roots = {"foreground": reset}
    handlers = {}
    for slot, w in enumerate(vec[2:], 2):
        a = w & ~1
        if not w:
            continue
        if a not in funcs:
            errors.append(f"IRQ ROOT MISSING: vector slot {slot} target 0x{w:08X} has no code")
            continue
        handlers[a] = name[a]
    top = [a for a, n in handlers.items() if n not in LOWER_PRIORITY_HANDLERS | TERMINAL_HANDLERS]
    lower = [a for a, n in handlers.items() if n in LOWER_PRIORITY_HANDLERS]
    if not any(n == "ADC0_1_IRQHandler" for n in handlers.values()) or not lower:
        errors.append("IRQ ROOT MISSING: ADC0_1_IRQHandler / EXTI handlers not found")

    # Reachability + cycle detection (explicit DFS with an active stack).
    reach: set[int] = set()
    cycles: list[str] = []
    color: dict[int, int] = {}
    for root in [reset] + top + lower:
        if color.get(root):
            continue
        stack = [(root, iter(succ(root)))]
        path = [root]
        color[root] = 1
        while stack:
            node, it = stack[-1]
            nxt = next(it, None)
            if nxt is None:
                color[node] = 2
                reach.add(node)
                stack.pop()
                path.pop()
                continue
            c = color.get(nxt, 0)
            if c == 1:
                cyc = path[path.index(nxt):] + [nxt]
                if len(cycles) < 5:
                    cycles.append(" -> ".join(name[x] for x in cyc))
            elif c == 0:
                color[nxt] = 1
                stack.append((nxt, iter(succ(nxt))))
                path.append(nxt)
    for c in cycles:
        errors.append(f"RECURSIVE CALL GRAPH: {c}")
    for a in sorted(reach):
        if graph[a][1] and not address_taken:
            errors.append(f"UNRESOLVED INDIRECT CALL: {name[a]} and no address-taken functions")
        for c in graph[a][2]:
            errors.append(f"UNSUPPORTED CONTROL FLOW: {name[a]} jumps into another function: {c}")

    # Frames: build objects need their own exact .su record; libraries use audited disassembly.
    sections = _map_objects(mapfile)
    objdir_n = _norm(objdir)
    su_cache: dict[str, tuple[dict[str, list[int]], list[str]] | None] = {}
    frame: dict[int, int] = {}
    missing_su, frame_source = [], {}
    for a in sorted(reach):
        fr, _, bad = audit[a]
        for b in bad:
            errors.append(f"DYNAMIC/UNSUPPORTED SP WRITE: {name[a]} {b}")
        obj = _object_of(a, sections)
        if obj is None:
            errors.append(f"UNMAPPED FUNCTION: {name[a]} 0x{a:08X} not in any linker-map input section")
            continue
        # Every linked C object (not an archive member) is a build object and needs its .su;
        # toolchain archives and assembly objects have no .su and use audited disassembly.
        build_obj = obj.endswith(".c.o") and "(" not in obj
        if build_obj and _norm(os.path.dirname(obj)) != objdir_n:
            errors.append(f"C OBJECT OUTSIDE BUILD OBJDIR: {name[a]} from {obj}")
            continue
        if not build_obj:
            frame[a] = fr
            frame_source[name[a]] = "disassembly:" + os.path.basename(obj)
            continue
        su_path = obj[:-2] + ".su"
        if su_path not in su_cache:
            su_cache[su_path] = _load_su(su_path) if os.path.isfile(su_path) else None
            if su_cache[su_path] is not None:
                errors += su_cache[su_path][1]
        entry = su_cache[su_path]
        if entry is None:
            missing_su.append(f"{name[a]} (no {os.path.basename(su_path)})")
            continue
        recs = entry[0].get(name[a])
        if not recs:
            # GCC writes clone records without the clone number: foo.constprop.0 -> foo.constprop
            m = re.fullmatch(r"(.+\.(?:isra|constprop|part|cold))\.\d+", name[a])
            recs = entry[0].get(m.group(1)) if m else None
        if not recs:
            missing_su.append(name[a])
            continue
        if len(recs) != 1:
            errors.append(f"AMBIGUOUS STACK USAGE: {name[a]} has {len(recs)} records in "
                          f"{os.path.basename(su_path)}")
            continue
        if recs[0] != fr:
            errors.append(f"STACK USAGE CONTRADICTION: {name[a]} .su {recs[0]} B != disassembly {fr} B")
            continue
        frame[a] = recs[0]
        frame_source[name[a]] = ".su:" + os.path.basename(su_path)
    if missing_su:
        errors.append("MISSING STACK USAGE: " + ", ".join(missing_su[:40]) +
                      (f" (+{len(missing_su) - 40} more)" if len(missing_su) > 40 else ""))

    report = {"schema": 2, "variant": variant, "elf": str(elf),
              "irq_model": "M820 NVIC topology fingerprint (tools/m820_stack_gate.py)",
              "physical_ram_end": f"0x{PHYSICAL_RAM_END:08X}", "sp": f"0x{sp:08X}",
              "initial_msp": f"0x{vec[0]:08X}", "heap_end": f"0x{heap_end:08X}",
              "ebss": f"0x{ebss:08X}", "ram_above_sp_bytes": PHYSICAL_RAM_END - sp,
              "reserved_stack_bytes": stack_size, "reserved_heap_bytes": heap_end - ebss,
              "reachable_functions": len(reach),
              "address_taken_functions": sorted(name[a] for a in address_taken)}
    if errors or any(a not in frame for a in reach):
        report.update(errors=errors, verdict="FAIL")
        return report

    memo: dict[int, tuple[int, list[int]]] = {}

    def worst(a: int) -> tuple[int, list[int]]:
        if a not in memo:
            best = max((worst(t) for t in succ(a)), key=lambda x: x[0], default=(0, []))
            memo[a] = (frame[a] + best[0], [a] + best[1])
        return memo[a]

    def uses_fp(root: int) -> bool:
        seen, todo = set(), [root]
        while todo:
            n = todo.pop()
            if n not in seen:
                seen.add(n)
                todo.extend(succ(n))
        return any(audit[n][1] for n in seen)

    sys.setrecursionlimit(max(10000, len(funcs) * 4))
    fg_bytes, fg_path = worst(reset)
    frame1 = FP_FRAME if uses_fp(reset) else BASIC_FRAME
    top_worst = max((worst(h) for h in top), key=lambda x: x[0])
    isr_direct = frame1 + top_worst[0]
    isr_nested, nested_path = 0, []
    for h in lower:
        lw = worst(h)
        cand = frame1 + lw[0] + (FP_FRAME if uses_fp(h) else BASIC_FRAME) + top_worst[0]
        if cand > isr_nested:
            isr_nested, nested_path = cand, lw[1]
    isr = max(isr_direct, isr_nested)
    total = fg_bytes + isr
    margin = stack_size - total
    if margin < MIN_MARGIN_BYTES:
        errors.append(f"STACK BUDGET UNSAFE: foreground {fg_bytes} + ISR {isr} = {total} B, "
                      f"reserved {stack_size} B, margin {margin} B < required {MIN_MARGIN_BYTES} B")
    report.update({
        "foreground_bytes": fg_bytes, "foreground_path": [name[a] for a in fg_path],
        "isr_top_software_bytes": top_worst[0], "isr_top_path": [name[a] for a in top_worst[1]],
        "isr_first_frame_bytes": frame1, "isr_direct_bytes": isr_direct,
        "isr_nested_bytes": isr_nested, "isr_nested_lower_path": [name[a] for a in nested_path],
        "isr_addition_bytes": isr, "total_worst_bytes": total, "margin_bytes": margin,
        "min_margin_bytes": MIN_MARGIN_BYTES, "minimum_msp": f"0x{sp - total:08X}",
        "frame_sources": {name[a]: frame_source.get(name[a]) for a in fg_path + top_worst[1]},
        "errors": errors, "verdict": "PASS" if not errors else "FAIL",
    })
    return report


def run_gate(elf: str, objdir: str, mapfile: str, toolbin: str, variant: str,
             out_json: str | None) -> dict:
    try:
        report = check(elf, objdir, mapfile, toolbin, variant)
    except GateError as e:
        report = {"schema": 2, "variant": variant, "elf": str(elf), "errors": [str(e)], "verdict": "FAIL"}
    except Exception as e:  # any unexpected condition is a failure, never a pass
        report = {"schema": 2, "variant": variant, "elf": str(elf),
                  "errors": [f"UNEXPECTED GATE CONDITION: {type(e).__name__}: {e}"], "verdict": "FAIL"}
    if out_json:
        Path(out_json).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if report["verdict"] == "PASS":
        print(f"STACK GATE {variant.upper()}: PASS  fg={report['foreground_bytes']} B "
              f"isr={report['isr_addition_bytes']} B total={report['total_worst_bytes']} B "
              f"reserved={report['reserved_stack_bytes']} B margin={report['margin_bytes']} B "
              f"_sp={report['sp']}")
    else:
        print(f"STACK GATE {variant.upper()}: FAIL", file=sys.stderr)
        for e in report["errors"]:
            print(f"  - {e}", file=sys.stderr)
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--elf", required=True)
    ap.add_argument("--map", required=True, help="linker map of the same link")
    ap.add_argument("--objdir", required=True, help="directory with the build's objects and .su files")
    ap.add_argument("--toolbin", required=True, help="Arm GNU toolchain bin directory")
    ap.add_argument("--variant", default="normal")
    ap.add_argument("--json", help="write the report here")
    a = ap.parse_args()
    return 0 if run_gate(a.elf, a.objdir, a.map, a.toolbin, a.variant, a.json)["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
