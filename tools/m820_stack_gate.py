#!/usr/bin/env python3
"""M820 target stack-budget gate (regression gate for the known M820/GD32F303RC architecture).

Runs on a FINAL linked ELF plus the GCC -fstack-usage (.su) files of the same build and
fails the build when the worst reachable stack no longer fits the linker-reserved stack.

Method (reviewed in TQ-06 E30 root-cause review, 2026-09-28):
  * frame of every function = SP decrements in its disassembly (push/stmdb/vpush/vstmdb,
    sub sp #imm, str/strd [sp,#-n]!); every function that also has a .su entry must match it;
  * call graph from direct bl/blx/b<cond> to a function start (tail calls are counted as
    calls, which only over-approximates); functions are keyed by ADDRESS, so equal static
    names in different translation units do not collapse;
  * every indirect call (blx/bx rN) is bounded by the worst function whose address is taken
    (appears in .rodata/.data or a .text literal pool, outside the vector table);
  * reachable recursion, dynamic SP adjustment or a .su "dynamic" entry fail the gate;
  * foreground = Reset_Handler graph; ISR addition = realistic M820 nesting: one entry of the
    priority-0 group (ADC/TIMER/CAN/SysTick/default; equal preemption, no mutual nesting), or
    one lower-priority EXTI handler preempted by one priority-0 handler. Exception frames:
    104 B FP-extended + 4 B alignment when the preempted context executes FP code, else 32+4.
    Fault/NMI handlers are terminal (they stop) and are not part of the ride budget.
"""
from __future__ import annotations
import argparse, glob, json, os, re, subprocess, sys
from pathlib import Path

PHYSICAL_RAM_ORIGIN = 0x20000000
PHYSICAL_RAM_END = 0x2000C000          # GD32F303RC: 48 KiB SRAM
MIN_MARGIN_BYTES = 1024                # meaningful headroom below the reserved stack
LOWER_PRIORITY_HANDLERS = {"EXTI2_IRQHandler", "EXTI10_15_IRQHandler"}  # nvic_config(): pre 2 before PRE1_SUB3
TERMINAL_HANDLERS = {"NMI_Handler", "HardFault_Handler", "MemManage_Handler", "BusFault_Handler",
                     "UsageFault_Handler"}
FP_FRAME = 104 + 4
BASIC_FRAME = 32 + 4

_HDR = re.compile(r"^([0-9a-f]{8}) <([^>]+)>:")
_INS = re.compile(r"^\s*([0-9a-f]+):\s+(\S+)\s*(.*)$")
_TGT = re.compile(r"^([0-9a-f]+)\s+<([^>+]+)(\+0x[0-9a-f]+)?>")


class GateError(Exception):
    pass


def _run(cmd: list[str]) -> str:
    p = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    if p.returncode:
        raise GateError(f"{Path(cmd[0]).name} failed: {p.stderr.strip()}")
    return p.stdout


def _tool(toolbin: str, name: str) -> str:
    for cand in (Path(toolbin) / (name + ".exe"), Path(toolbin) / name):
        if cand.is_file():
            return str(cand)
    raise GateError(f"{name} not found in {toolbin}")


def _reg_count(args: str) -> int:
    m = re.search(r"\{([^}]*)\}", args)
    if not m:
        return 0
    n = 0
    for part in (p.strip() for p in m.group(1).split(",")):
        if "-" in part:
            a, b = part.split("-")
            n += int(b[1:]) - int(a[1:]) + 1
        elif part:
            n += 1
    return n


def _parse_disassembly(text: str):
    funcs: dict[int, dict] = {}
    words: set[int] = set()
    cur = None
    for line in text.splitlines():
        m = _HDR.match(line)
        if m:
            cur = {"name": m.group(2), "ins": []}
            funcs[int(m.group(1), 16)] = cur
            continue
        m = _INS.match(line)
        if m and cur is not None:
            op, args = m.group(2), m.group(3)
            if op == ".word":
                try:
                    words.add(int(args.split()[0], 16))
                except ValueError:
                    pass
            else:
                cur["ins"].append((int(m.group(1), 16), op, args))
    return funcs, words


def _frame(f: dict) -> tuple[int, bool, bool]:
    total, dynamic, uses_fp = 0, False, False
    for _, op, args in f["ins"]:
        base = op.split(".")[0]
        if base.startswith("v") and base not in ("vpush", "vpop"):
            uses_fp = True
        if base == "push" or (base == "stmdb" and args.startswith("sp!")):
            total += 4 * _reg_count(args)
        elif base == "vpush" or (base == "vstmdb" and args.startswith("sp!")):
            uses_fp = True
            total += (8 if "{d" in args else 4) * _reg_count(args)
        elif base in ("sub", "subw") and re.match(r"sp,\s*(sp,\s*)?#", args):
            total += int(re.search(r"#(\d+)", args).group(1))
        elif base == "sub" and re.match(r"sp,\s*(sp,\s*)?r\d", args):
            dynamic = True
        elif base in ("str", "strd") and re.search(r"\[sp,\s*#-\d+\]!", args):
            total += int(re.search(r"#-(\d+)\]!", args).group(1))
    return total, dynamic, uses_fp


def _edges(addr: int, f: dict, starts: set[int]):
    direct, indirect = set(), []
    for a, op, args in f["ins"]:
        base = op.split(".")[0]
        if base.startswith("b") and base not in ("bic", "bfi", "bfc", "bkpt"):
            m = _TGT.match(args)
            if m and m.group(3) is None:
                tgt = int(m.group(1), 16)
                if tgt in starts and tgt != addr:
                    direct.add(tgt)
            elif base == "blx" or (base == "bx" and args.strip() != "lr"):
                indirect.append(a)
    return direct, indirect


def _section_words(objdump: str, elf: str, sections: list[str]) -> set[int]:
    words: set[int] = set()
    for sec in sections:
        p = subprocess.run([objdump, "-s", "-j", sec, elf], capture_output=True, text=True)
        if p.returncode:
            continue
        data = bytearray(); base = None
        for line in p.stdout.splitlines():
            m = re.match(r"^ ([0-9a-f]{4,8}) ((?:[0-9a-f]{2,8} ?){1,4})", line)
            if m:
                if base is None:
                    base = int(m.group(1), 16)
                data += bytes.fromhex(m.group(2).replace(" ", ""))
        if base is None:
            continue
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


def _symbols(nm: str, elf: str) -> dict[str, int]:
    out = {}
    for line in _run([nm, "--defined-only", elf]).splitlines():
        p = line.split()
        if len(p) == 3:
            out.setdefault(p[2], int(p[0], 16))
    return out


def _su(objdir: str) -> tuple[dict[str, set[int]], list[str]]:
    su: dict[str, set[int]] = {}
    dynamic: list[str] = []
    files = glob.glob(os.path.join(objdir, "*.su"))
    if not files:
        raise GateError(f"no .su files in {objdir} (build must use -fstack-usage)")
    for path in files:
        for line in open(path, encoding="utf-8", errors="replace"):
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 3:
                continue
            name = parts[0].rsplit(":", 1)[-1]
            su.setdefault(name, set()).add(int(parts[1]))
            if parts[2] != "static":
                dynamic.append(f"{name} ({parts[2]})")
    return su, dynamic


def check(elf: str, objdir: str, toolbin: str, variant: str) -> dict:
    objdump, nm = _tool(toolbin, "arm-none-eabi-objdump"), _tool(toolbin, "arm-none-eabi-nm")
    sym = _symbols(nm, elf)
    for need in ("_sp", "_heap_end", "_ebss", "__stack_size", "__heap_size"):
        if need not in sym:
            raise GateError(f"required linker symbol missing: {need}")
    sp, heap_end, ebss = sym["_sp"], sym["_heap_end"], sym["_ebss"]
    stack_size, heap_size = sym["__stack_size"], sym["__heap_size"]
    vec = _vectors(objdump, elf)
    layout_errors = []
    if not (PHYSICAL_RAM_ORIGIN < ebss <= heap_end < sp <= PHYSICAL_RAM_END):
        layout_errors.append("RAM layout outside physical 0x20000000..0x2000C000 or out of order")
    if sp - heap_end != stack_size:
        layout_errors.append(f"_sp-_heap_end={sp - heap_end} != __stack_size={stack_size}")
    if heap_end - ebss < heap_size:
        layout_errors.append(f"heap {heap_end - ebss} B < __heap_size {heap_size}")
    if vec[0] != sp:
        layout_errors.append(f"initial MSP 0x{vec[0]:08X} != _sp 0x{sp:08X}")

    funcs, text_words = _parse_disassembly(_run([objdump, "-d", "--no-show-raw-insn", elf]))
    starts = set(funcs)
    frames = {a: _frame(f) for a, f in funcs.items()}
    graph = {a: _edges(a, f, starts) for a, f in funcs.items()}
    name = {a: f["name"] for a, f in funcs.items()}
    by_name = {}
    for a, n in name.items():
        by_name.setdefault(n, a)

    vector_targets = {w & ~1 for w in vec[1:] if w}
    taken_words = text_words | _section_words(objdump, elf, [".rodata", ".data"])
    address_taken = {w & ~1 for w in taken_words if (w & 1) and (w & ~1) in starts}

    su, su_dynamic = _su(objdir)
    memo: dict[int, tuple[int, list[int]]] = {}
    visiting: set[int] = set()
    errors: list[str] = []

    def worst(a: int, allow_indirect: bool = True) -> tuple[int, list[int]]:
        if a in memo:
            return memo[a]
        if a in visiting:
            raise GateError(f"reachable recursion through {name[a]}")
        visiting.add(a)
        best = (0, [])
        direct, indirect = graph[a]
        for t in direct:
            d = worst(t)
            if d[0] > best[0]:
                best = d
        if indirect:
            if not address_taken:
                raise GateError(f"unbounded indirect call in {name[a]}")
            for t in address_taken:
                if t in visiting:
                    continue  # a pointer target calling back up the chain is already counted
                d = worst(t)
                if d[0] > best[0]:
                    best = d
        visiting.discard(a)
        res = (frames[a][0] + best[0], [a] + best[1])
        memo[a] = res
        return res

    def reach(root: int) -> set[int]:
        seen, todo = set(), [root]
        while todo:
            n = todo.pop()
            if n in seen:
                continue
            seen.add(n)
            direct, indirect = graph[n]
            todo.extend(direct)
            if indirect:
                todo.extend(address_taken)
        return seen

    def audit(root: int) -> bool:
        uses_fp = False
        for n in reach(root):
            fr, dyn, fp = frames[n]
            uses_fp |= fp
            if dyn:
                errors.append(f"dynamic SP adjustment in reachable {name[n]}")
            if name[n] in su and fr not in su[name[n]]:
                errors.append(f"frame {name[n]}: disassembly {fr} B not in .su {sorted(su[name[n]])}")
        return uses_fp

    reset = vec[1] & ~1
    if reset not in funcs:
        raise GateError("Reset_Handler not found in disassembly")
    fg_bytes, fg_path = worst(reset)
    fg_fp = audit(reset)

    handlers = sorted({w & ~1 for w in vec[2:] if w and (w & ~1) != reset and (w & ~1) in funcs})
    unknown = [f"0x{w:08X}" for w in vec[2:] if w and (w & ~1) not in funcs]
    if unknown:
        errors.append(f"vector entries without code: {unknown[:4]}")
    top, lower = [], []
    for h in handlers:
        n = name[h]
        if n in TERMINAL_HANDLERS:
            continue
        (lower if n in LOWER_PRIORITY_HANDLERS else top).append(h)
    if not top:
        raise GateError("no priority-0 interrupt handlers found")
    for h in top + lower:
        audit(h)
    frame1 = FP_FRAME if fg_fp else BASIC_FRAME
    top_worst = max((worst(h) for h in top), key=lambda x: x[0])
    isr_direct = frame1 + top_worst[0]
    isr_nested = 0
    nested_path: list[int] = []
    for h in lower:
        lw = worst(h)
        frame2 = FP_FRAME if audit(h) else BASIC_FRAME
        cand = frame1 + lw[0] + frame2 + top_worst[0]
        if cand > isr_nested:
            isr_nested, nested_path = cand, lw[1]
    isr = max(isr_direct, isr_nested)
    total = fg_bytes + isr
    margin = stack_size - total
    if su_dynamic:
        errors.append("dynamic .su entries: " + ", ".join(su_dynamic[:5]))
    errors += layout_errors
    if margin < MIN_MARGIN_BYTES:
        errors.append(f"stack budget unsafe: foreground {fg_bytes} + ISR {isr} = {total} B, "
                      f"reserved {stack_size} B, margin {margin} B < required {MIN_MARGIN_BYTES} B")

    report = {
        "schema": 1, "variant": variant, "elf": str(elf),
        "physical_ram_end": f"0x{PHYSICAL_RAM_END:08X}", "sp": f"0x{sp:08X}",
        "initial_msp": f"0x{vec[0]:08X}", "heap_end": f"0x{heap_end:08X}", "ebss": f"0x{ebss:08X}",
        "ram_above_sp_bytes": PHYSICAL_RAM_END - sp,
        "reserved_stack_bytes": stack_size, "reserved_heap_bytes": heap_end - ebss,
        "foreground_bytes": fg_bytes, "foreground_path": [name[a] for a in fg_path],
        "isr_top_software_bytes": top_worst[0], "isr_top_path": [name[a] for a in top_worst[1]],
        "isr_first_frame_bytes": frame1, "isr_direct_bytes": isr_direct,
        "isr_nested_bytes": isr_nested, "isr_nested_lower_path": [name[a] for a in nested_path],
        "isr_addition_bytes": isr, "total_worst_bytes": total,
        "margin_bytes": margin, "min_margin_bytes": MIN_MARGIN_BYTES,
        "minimum_msp": f"0x{sp - total:08X}",
        "address_taken_functions": sorted(name[a] for a in address_taken),
        "errors": errors, "verdict": "PASS" if not errors else "FAIL",
    }
    return report


def run_gate(elf: str, objdir: str, toolbin: str, variant: str, out_json: str | None) -> dict:
    try:
        report = check(elf, objdir, toolbin, variant)
    except GateError as e:
        report = {"schema": 1, "variant": variant, "elf": str(elf), "errors": [str(e)], "verdict": "FAIL"}
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
    ap.add_argument("--objdir", required=True, help="directory with the build's .su files")
    ap.add_argument("--toolbin", required=True, help="Arm GNU toolchain bin directory")
    ap.add_argument("--variant", default="normal")
    ap.add_argument("--json", help="write the report here")
    a = ap.parse_args()
    return 0 if run_gate(a.elf, a.objdir, a.toolbin, a.variant, a.json)["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
