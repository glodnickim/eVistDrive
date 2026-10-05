#!/usr/bin/env python3
"""Permanent regression tests for tools/m820_stack_gate.py (needs Arm GNU Toolchain 13.2.1).

A small real ARM ELF fixture is generated with the SAME vector slots and NVIC priority calls
the gate fingerprints for M820, so it passes every check except the defect injected by a
test. A positive control must PASS first; every negative test must exit non-zero AND report
its specific rejection. Test G rebuilds the real firmware in the known-bad configuration
(G53 at -O0, 2 KiB stack) and requires the build and the gate to reject it. Tests M*/N*
(REV-50B8758-STACK-01/02) cover conditional calls and literal call targets that a call or an
unproven path invalidates; `--gate <old gate>` runs everything against another gate version
(the gate of 880ea55 must FAIL this self-test).
"""
from __future__ import annotations
import argparse, json, os, shutil, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import m820_stack_gate as gate  # noqa: E402

CPU = ["-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"]
LINKER = """
MEMORY { FLASH (rx) : ORIGIN = 0x08005000, LENGTH = 230K
         RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 48K }
ENTRY(Reset_Handler)
SECTIONS {
  __stack_size = 4096; __heap_size = 1024;
  .vectors : { KEEP(*(.vectors)) } > FLASH
  .text : { *(.text*) *(.rodata*) } > FLASH
  .data : { *(.data*) } > RAM AT > FLASH
  .bss : { *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
  .heap_stack (NOLOAD) : { . = ALIGN(8); . = . + __heap_size; _heap_end = .;
                           . = . + __stack_size; _sp = .; } > RAM
}
"""

DEFECTS = {
    "DIRECT_REC": "int rec_a(int n) { return n ? rec_a(n - 1) + 1 : 0; }\n"
                  "static void defect(void) { sink = rec_a(sink); }\n",
    "INDIRECT_REC": "int rec_b(int n);\n"
                    "int rec_a(int n) { return n ? rec_b(n - 1) + 1 : 0; }\n"
                    "int rec_b(int n) { return n ? rec_a(n - 1) + 2 : 0; }\n"
                    "static void defect(void) { sink = rec_a(sink); }\n",
    "LONG_REC": "int rec_b(int n); int rec_c(int n);\n"
                "int rec_a(int n) { return n ? rec_b(n - 1) + 1 : 0; }\n"
                "int rec_b(int n) { return n ? rec_c(n - 1) + 2 : 0; }\n"
                "int rec_c(int n) { return n ? rec_a(n - 1) + 3 : 0; }\n"
                "static void defect(void) { sink = rec_a(sink); }\n",
    "DYNAMIC_SP": "void dyn_sp(void) {\n"
                  "  __asm volatile(\"mov r3, sp\\n\\tmov r2, sp\\n\\tsub.w r2, r2, #8192\\n\\t\"\n"
                  "                 \"mov sp, r2\\n\\tstr r3, [sp]\\n\\tmov sp, r3\" ::: \"r2\", \"r3\", \"memory\");\n"
                  "}\nstatic void defect(void) { dyn_sp(); }\n",
    "FAKE_FRAME_POINTER": "void fake_fp(void) {\n"
                          "  __asm volatile(\"sub.w r7, sp, #8192\\n\\tmov sp, r7\" ::: \"memory\");\n"
                          "}\nstatic void defect(void) { fake_fp(); }\n",
    "SP_LOAD": "void sp_load(unsigned *p) {\n"
               "  __asm volatile(\"ldr.w sp, [%0]\" :: \"r\"(p) : \"memory\");\n"
               "}\nstatic void defect(void) { sp_load((unsigned *)&sink); }\n",
    "UNSET_FRAME_POINTER": "__attribute__((naked)) void unset_fp(void) {\n"
                           "  __asm volatile(\"mov sp, r7\\n\\tbx lr\");\n"
                           "}\nstatic void defect(void) { unset_fp(); }\n",
    "NONE": "static void defect(void) { }\n",
    # --- REV-50B8758-STACK-01: a conditional call is a call (review reproduction, verbatim) ---
    "COND_CALL": "void deep(void) { volatile int pad[2048]; pad[0] = sink; sink = pad[0]; }\n"
                 "static void defect(void) {\n"
                 "  __asm volatile(\"movs r0, #1\\n\\tcmp r0, #0\\n\\tit ne\\n\\tblne deep\"\n"
                 "                 ::: \"r0\", \"r1\", \"r2\", \"r3\", \"lr\", \"cc\", \"memory\");\n"
                 "}\n",
    "COND_PRIORITY_CALL": "void extra_cfg(void) {\n"
                          "  __asm volatile(\"movs r0, #8\\n\\tmovs r1, #2\\n\\tmovs r2, #0\\n\\tcmp r0, #0\\n\\t\"\n"
                          "                 \"it ne\\n\\tblne nvic_irq_enable\"\n"
                          "                 ::: \"r0\", \"r1\", \"r2\", \"r3\", \"ip\", \"lr\", \"cc\", \"memory\");\n"
                          "}\nstatic void defect(void) { extra_cfg(); }\n",
    "PC_WRITE": "__attribute__((naked)) void pc_write(void) { __asm volatile(\"ldr.w pc, [r0]\"); }\n"
                "static void defect(void) { pc_write(); }\n",
    "INTRA_CALL_RECURSION": "__attribute__((naked)) void intra_rec(void) {\n"
                            "  __asm volatile(\"push {r4, lr}\\n1:\\tpush {r5, lr}\\n\\tcbz r0, 2f\\n\\t\"\n"
                            "                 \"subs r0, #1\\n\\tbl 1b\\n2:\\tpop {r5, pc}\");\n"
                            "}\nstatic void defect(void) { intra_rec(); }\n",
    # --- REV-50B8758-STACK-02: a literal value does not survive a call or an unproven path ---
    "CALL_CLOBBER_NULL": "void deep(void) { volatile int pad[2048]; pad[0] = sink; sink = pad[0]; }\n"
                         "__attribute__((naked)) void set_r3(void) { __asm volatile(\"ldr r3, =deep\\n\\tbx lr\"); }\n"
                         "static void defect(void) {\n"
                         "  __asm volatile(\"ldr r3, 1f\\n\\tbl set_r3\\n\\tblx r3\\n\\tb 2f\\n\\t.align 2\\n1: .word 0\\n2:\"\n"
                         "                 ::: \"r0\", \"r1\", \"r2\", \"r3\", \"lr\", \"memory\");\n"
                         "}\n",
    "CALL_RETARGET": "void small_fn(void) { sink++; }\n"
                     "void deep(void) { volatile int pad[2048]; pad[0] = sink; sink = pad[0]; }\n"
                     "__attribute__((naked)) void set_r3(void) { __asm volatile(\"ldr r3, =deep\\n\\tbx lr\"); }\n"
                     "static void defect(void) {\n"
                     "  __asm volatile(\"ldr r3, =small_fn\\n\\tbl set_r3\\n\\tblx r3\"\n"
                     "                 ::: \"r0\", \"r1\", \"r2\", \"r3\", \"lr\", \"memory\");\n"
                     "}\n",
    "LITERAL_PATH_CBZ": "void deep(void) { volatile int pad[2048]; pad[0] = sink; sink = pad[0]; }\n"
                        "static void defect(void) {\n"
                        "  __asm volatile(\"movs r0, #0\\n\\tldr r3, =deep\\n\\tcbz r0, 4f\\n\\tldr r3, 1f\\n\"\n"
                        "                 \"4:\\tblx r3\\n\\tb 2f\\n\\t.align 2\\n1: .word 0\\n2:\"\n"
                        "                 ::: \"r0\", \"r1\", \"r2\", \"r3\", \"lr\", \"memory\");\n"
                        "}\n",
    "WEAK_NULL_PROVEN": "static void defect(void) {\n"
                        "  __asm volatile(\"ldr r3, 1f\\n\\tcbz r3, 2f\\n\\tblx r3\\n\\tb 2f\\n\\t.align 2\\n1: .word 0\\n2:\"\n"
                        "                 ::: \"r0\", \"r1\", \"r2\", \"r3\", \"lr\", \"memory\");\n"
                        "}\n",
}


# REV-A1R-01: the first reviewed nvic_config call replaced by asm whose argument is written
# under an IT condition that is FALSE at run time - the textual value equals the reviewed one,
# the executed one does not, so the decoder must refuse it.
_NVIC_CLOBBER = '::: "r0", "r1", "r2", "r3", "ip", "lr", "cc", "memory"'
NVIC_FIRST_CALL = {
    # review fixture Z3 (verbatim): executes nvic_irq_enable(8, 0, 0)
    "COND_ARG_R0": '__asm volatile("movs r0, #8\\n\\tcmp r0, #8\\n\\tit ne\\n\\tmovne r0, #21\\n\\t'
                   'movs r1, #0\\n\\tmovs r2, #0\\n\\tbl nvic_irq_enable" ' + _NVIC_CLOBBER + ');',
    # conditional r2 before unconditional r0/r1: executes nvic_irq_enable(21, 0, 7)
    "COND_ARG_R2": '__asm volatile("movs r2, #7\\n\\tcmp r2, #7\\n\\tit ne\\n\\tmovne r2, #0\\n\\t'
                   'movs r0, #21\\n\\tmovs r1, #0\\n\\tbl nvic_irq_enable" ' + _NVIC_CLOBBER + ');',
}


def fixture_source(defect: str, extra_slot: int | None = None, nvic_first: str | None = None) -> str:
    slots = dict(gate.EXPECTED_VECTOR_SLOTS)
    if extra_slot is not None:
        slots[extra_slot] = "TIMER4_IRQHandler"
    handlers = sorted(set(slots.values()) - {"Reset_Handler"})
    out = ["typedef void (*vec_t)(void);", "extern unsigned long _sp;",
           "volatile int sink;", "void Reset_Handler(void);", "void Default_Handler(void);"]
    out += [f"void {h}(void);" for h in handlers]
    out += ["void nvic_priority_group_set(unsigned g) { sink = (int)g; }",
            "void nvic_irq_enable(int irq, unsigned char pre, unsigned char sub) { sink = irq + pre + sub; }",
            "void NVIC_SetPriority(int irq, unsigned pri) { sink = irq + (int)pri; }"]
    for caller, calls in gate.EXPECTED_PRIORITY_CALLS.items():
        body = " ".join(f"{fn}({', '.join(str(v) for v in args)});" for fn, args in calls)
        if nvic_first is not None and caller == "nvic_config":
            first = f"{calls[0][0]}({', '.join(str(v) for v in calls[0][1])});"
            body = body.replace(first, NVIC_FIRST_CALL[nvic_first], 1)
        out.append(f"void {caller}(void) {{ {body} }}")
    out.append(DEFECTS[defect])
    out += ["static int work_leaf(int x) { volatile int pad[8]; pad[0] = x; return pad[0] + 1; }",
            "static int work_mid(int x) { volatile float f = (float)x * 1.5f; return work_leaf((int)f); }",
            "void ADC0_1_IRQHandler(void) { volatile int buf[16]; buf[0] = sink; sink = work_leaf(buf[0]); }",
            "void Default_Handler(void) { for (;;) { } }"]
    for h in handlers:
        if h == "ADC0_1_IRQHandler":
            continue
        if h in gate.TERMINAL_HANDLERS:
            out.append(f"void {h}(void) {{ for (;;) {{ }} }}")
        else:
            out.append(f"void {h}(void) {{ sink++; }}")
    calls = " ".join(f"{c}();" for c in gate.EXPECTED_PRIORITY_CALLS)
    out.append(f"int main(void) {{ {calls} for (;;) {{ sink = work_mid(sink); defect(); }} }}")
    out.append("void Reset_Handler(void) { main(); }")
    n = max(slots) + 20
    entries = ["(vec_t)&_sp"] + [slots.get(i, "Default_Handler") for i in range(1, n)]
    out.append("__attribute__((section(\".vectors\"), used)) const vec_t vectors[] = {\n  "
               + ",\n  ".join(entries) + "\n};")
    return "\n".join(out) + "\n"


class Runner:
    def __init__(self, toolbin: str, work: Path, gate_path: Path = ROOT / "tools/m820_stack_gate.py"):
        self.toolbin, self.work, self.gate_path = toolbin, work, gate_path
        self.gcc = gate._tool(toolbin, "arm-none-eabi-gcc")
        self.failures: list[str] = []
        self.results: list[tuple[str, str]] = []

    def build(self, tag: str, defect: str, extra_slot: int | None = None,
              nvic_first: str | None = None) -> dict:
        d = self.work / tag
        objdir = d / "objects"
        objdir.mkdir(parents=True, exist_ok=True)
        src = d / "src_fixture.c"
        src.write_text(fixture_source(defect, extra_slot, nvic_first), encoding="ascii")
        (d / "fixture.ld").write_text(LINKER, encoding="ascii")
        obj = objdir / "src_fixture.c.o"
        subprocess.run([self.gcc, *CPU, "-O0", "-g", "-ffreestanding", "-fstack-usage",
                        "-ffunction-sections", "-c", str(src), "-o", str(obj)], check=True)
        elf, mapf = d / "fixture.elf", d / "fixture.map"
        subprocess.run([self.gcc, *CPU, "-nostdlib", f"-T{d / 'fixture.ld'}", f"-Wl,-Map,{mapf}",
                        str(obj), "-o", str(elf)], check=True)
        return {"elf": str(elf), "map": str(mapf), "objdir": str(objdir), "su": objdir / "src_fixture.c.su"}

    def gate(self, fx: dict, tag: str) -> tuple[int, dict]:
        js = self.work / f"{tag}.gate.json"
        p = subprocess.run([sys.executable, str(self.gate_path), "--elf", fx["elf"],
                            "--map", fx["map"], "--objdir", fx["objdir"], "--toolbin", self.toolbin,
                            "--variant", tag, "--json", str(js)], capture_output=True, text=True)
        return p.returncode, json.loads(js.read_text(encoding="utf-8"))

    def expect(self, name: str, rc: int, rep: dict, want_pass: bool, needle: str = "") -> None:
        errs = " | ".join(rep.get("errors", []))
        if want_pass:
            ok = rc == 0 and rep.get("verdict") == "PASS"
            label = "PASS" if ok else f"UNEXPECTED FAIL rc={rc}: {errs}"
        else:
            ok = rc != 0 and rep.get("verdict") == "FAIL" and needle in errs
            label = "REJECTED" if ok else f"FALSE PASS / WRONG REASON rc={rc}: {errs or 'no errors'}"
        self.results.append((name, label))
        print(f"{'OK  ' if ok else 'FAIL'} {name}: {label}" + (f"  [{needle}]" if ok and needle else ""))
        if not ok:
            self.failures.append(name)


def firmware_known_bad(toolbin: str, work: Path, gate_path: Path) -> tuple[int, dict, int]:
    """Rebuild the real NORMAL firmware with G53 at -O0 and a 2 KiB stack (the configuration of
    base aab3c3c): the build must refuse it, and the gate CLI must reject the produced ELF."""
    out = work / "known-bad-o0-2k"
    code = ("import sys; sys.path.insert(0, r'%s'); import build_firmware as b;"
            "b._build_one('DEV-NONCANONICAL', 'normal', 'stack_gate_negative_test', b.source_entries(),"
            " r'%s', r'%s', g53_optimized=False, extra_link_flags=('-Wl,--defsym=__stack_size=2048',))"
            % (ROOT / "tools", toolbin, out))
    p = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True)
    wd = out / "M820_BL820" / "work" / "normal"
    rep_path = wd / "DEV-NONCANONICAL.stack-gate.json"
    rep = json.loads(rep_path.read_text(encoding="utf-8")) if rep_path.exists() else {}
    final = out / "M820_BL820" / "DEV-NONCANONICAL_M820_BL820.bin"
    if final.exists():
        rep.setdefault("errors", []).append("known-bad build produced a final BIN")
    q = subprocess.run([sys.executable, str(gate_path),
                        "--elf", str(wd / "DEV-NONCANONICAL.elf"), "--map", str(wd / "DEV-NONCANONICAL.map"),
                        "--objdir", str(wd / "objects"), "--toolbin", toolbin, "--variant", "known-bad"],
                       capture_output=True, text=True)
    return p.returncode, rep, q.returncode


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--toolchain", required=True, help="Arm GNU toolchain bin directory")
    ap.add_argument("--skip-firmware", action="store_true", help="skip test G (full firmware rebuild)")
    ap.add_argument("--gate", type=Path, help="run the fixtures against this gate script instead of "
                    "tools/m820_stack_gate.py (mutation check: a defective gate must make this FAIL)")
    a = ap.parse_args()
    work = ROOT / ".build" / ("stack-gate-selftest" if a.gate is None else f"stack-gate-selftest-{a.gate.stem}")
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True)
    r = Runner(a.toolchain, work, *(() if a.gate is None else (a.gate.resolve(),)))

    base = r.build("positive", "NONE")
    rc, rep = r.gate(base, "positive")
    r.expect("POSITIVE CONTROL (M820-shaped fixture)", rc, rep, True)

    # A literal null call proven at the call site (the weak-undefined tm_clones pattern of the
    # production image) stays resolved: no address-taken function exists, so an unresolved
    # call would fail with UNRESOLVED INDIRECT CALL.
    fx = r.build("P2-weak-null-call-proven", "WEAK_NULL_PROVEN")
    rc, rep = r.gate(fx, "P2-weak-null-call-proven")
    r.expect("P2-weak-null-call-proven", rc, rep, True)

    # REV-50B8758-STACK-01/02: deep() has an 8200 B frame against a 4096 B reserve; each case
    # passed (356 B) on the gate of 880ea55 because the call to deep() was dropped.
    for tag, defect, needle in (("M1-conditional-call", "COND_CALL", "STACK BUDGET UNSAFE"),
                                ("M2-conditional-priority-call", "COND_PRIORITY_CALL",
                                 "IRQ TOPOLOGY UNDECODABLE: extra_cfg"),
                                ("M3-unsupported-pc-write", "PC_WRITE",
                                 "UNSUPPORTED CONTROL FLOW: pc_write"),
                                ("M4-intra-function-call-recursion", "INTRA_CALL_RECURSION",
                                 "UNSUPPORTED CONTROL FLOW: intra_rec"),
                                ("N1-literal-call-clobber-null", "CALL_CLOBBER_NULL", "STACK BUDGET UNSAFE"),
                                ("N2-literal-call-retarget", "CALL_RETARGET", "STACK BUDGET UNSAFE"),
                                ("N3-literal-unproven-path", "LITERAL_PATH_CBZ", "STACK BUDGET UNSAFE")):
        fx = r.build(tag, defect)
        rc, rep = r.gate(fx, tag)
        r.expect(tag, rc, rep, False, needle)

    # REV-A1R-01: an NVIC priority argument written under an IT condition is not decodable.
    for tag, variant in (("M5-conditional-nvic-arg-r0", "COND_ARG_R0"),
                         ("M6-conditional-nvic-arg-r2", "COND_ARG_R2")):
        fx = r.build(tag, "NONE", nvic_first=variant)
        rc, rep = r.gate(fx, tag)
        r.expect(tag, rc, rep, False, "IRQ TOPOLOGY UNDECODABLE: nvic_config")

    for tag, defect, needle in (("A-direct-recursion", "DIRECT_REC", "RECURSIVE CALL GRAPH: rec_a -> rec_a"),
                                ("B-indirect-recursion", "INDIRECT_REC", "rec_a -> rec_b -> rec_a"),
                                ("C-long-recursion", "LONG_REC", "rec_a -> rec_b -> rec_c -> rec_a"),
                                ("F-dynamic-sp", "DYNAMIC_SP", "DYNAMIC/UNSUPPORTED SP WRITE: dyn_sp"),
                                ("F2-fake-frame-pointer", "FAKE_FRAME_POINTER",
                                 "DYNAMIC/UNSUPPORTED SP WRITE: fake_fp"),
                                ("F3-sp-loaded-from-memory", "SP_LOAD",
                                 "DYNAMIC/UNSUPPORTED SP WRITE: sp_load"),
                                ("F4-unset-frame-pointer", "UNSET_FRAME_POINTER",
                                 "DYNAMIC/UNSUPPORTED SP WRITE: unset_fp")):
        fx = r.build(tag, defect)
        rc, rep = r.gate(fx, tag)
        r.expect(tag, rc, rep, False, needle)

    fx = r.build("D-missing-su-record", "NONE")
    lines = fx["su"].read_text(encoding="utf-8").splitlines()
    fx["su"].write_text("\n".join(l for l in lines if not l.split("\t")[0].endswith(":work_leaf")) + "\n",
                        encoding="utf-8")
    rc, rep = r.gate(fx, "D-missing-su-record")
    r.expect("D-missing-su-record", rc, rep, False, "MISSING STACK USAGE: work_leaf")

    fx = r.build("E-unrelated-su-only", "NONE")
    fx["su"].unlink()
    (Path(fx["objdir"]) / "src_unrelated.c.su").write_text("unrelated.c:1:6:unrelated_fn\t8\tstatic\n",
                                                           encoding="utf-8")
    rc, rep = r.gate(fx, "E-unrelated-su-only")
    r.expect("E-unrelated-su-only", rc, rep, False, "MISSING STACK USAGE")

    fx = r.build("E2-unrelated-record-in-own-su", "NONE")
    fx["su"].write_text("src_fixture.c:1:6:unrelated_fn\t8\tstatic\n", encoding="utf-8")
    rc, rep = r.gate(fx, "E2-unrelated-record-in-own-su")
    r.expect("E2-unrelated-record-in-own-su", rc, rep, False, "MISSING STACK USAGE")

    fx = r.build("H-duplicate-su-record", "NONE")
    lines = fx["su"].read_text(encoding="utf-8").splitlines()
    dup = next(l for l in lines if l.split("\t")[0].endswith(":work_leaf"))
    fx["su"].write_text("\n".join(lines + [dup]) + "\n", encoding="utf-8")
    rc, rep = r.gate(fx, "H-duplicate-su-record")
    r.expect("H-duplicate-su-record", rc, rep, False, "AMBIGUOUS STACK USAGE: work_leaf")

    fx = r.build("I-su-contradiction", "NONE")
    lines = fx["su"].read_text(encoding="utf-8").splitlines()
    fx["su"].write_text("\n".join((l.split("\t")[0] + "\t4\tstatic") if l.split("\t")[0].endswith(":work_leaf")
                                  else l for l in lines) + "\n", encoding="utf-8")
    rc, rep = r.gate(fx, "I-su-contradiction")
    r.expect("I-su-contradiction", rc, rep, False, "STACK USAGE CONTRADICTION: work_leaf")

    fx = r.build("J-irq-topology-changed", "NONE", extra_slot=66)
    rc, rep = r.gate(fx, "J-irq-topology-changed")
    r.expect("J-irq-topology-changed", rc, rep, False, "IRQ TOPOLOGY CHANGED")

    fx = dict(base)
    fx["map"] = str(work / "does-not-exist.map")
    rc, rep = r.gate(fx, "K-missing-map")
    r.expect("K-missing-map", rc, rep, False, "missing linker map")

    fx = dict(base)
    fx["elf"] = str(work / "does-not-exist.elf")
    rc, rep = r.gate(fx, "L-missing-elf")
    r.expect("L-missing-elf", rc, rep, False, "missing ELF")

    if not a.skip_firmware:
        rc_build, rep, rc_gate = firmware_known_bad(a.toolchain, work, r.gate_path)
        fg, total = rep.get("foreground_bytes", 0), rep.get("total_worst_bytes", 0)
        ok = (rc_build != 0 and rc_gate != 0 and rep.get("verdict") == "FAIL"
              and rep.get("reserved_stack_bytes") == 2048 and 3000 <= fg <= 3300 and 4200 <= total <= 4500
              and any("STACK BUDGET UNSAFE" in e for e in rep.get("errors", [])))
        label = (f"REJECTED (fg={fg} total={total} reserved={rep.get('reserved_stack_bytes')} "
                 f"build rc={rc_build} gate rc={rc_gate})") if ok else f"FALSE PASS / WRONG REASON: {rep}"
        print(f"{'OK  ' if ok else 'FAIL'} G-known-bad-O0-2K-firmware: {label}")
        r.results.append(("G-known-bad-O0-2K-firmware", label))
        if not ok:
            r.failures.append("G-known-bad-O0-2K-firmware")

    (work / "RESULTS.json").write_text(json.dumps(r.results, indent=2) + "\n", encoding="utf-8")
    print(f"STACK GATE SELF-TEST: {'PASS' if not r.failures else 'FAIL ' + ', '.join(r.failures)}")
    return 0 if not r.failures else 1


if __name__ == "__main__":
    sys.exit(main())
