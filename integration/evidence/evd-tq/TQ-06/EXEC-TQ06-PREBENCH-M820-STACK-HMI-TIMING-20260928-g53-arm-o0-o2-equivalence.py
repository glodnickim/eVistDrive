"""Independent reviewer harness: executes g53_port_update from two real M820 ELFs
(Cortex-M4 Unicorn, exact 48 KiB RAM, SP=_sp from ELF, AAPCS-hard ABI, fixture in
reserved heap). (1) Thumb instruction count per call (block hook, halfword-length
decode; INSTRUCTIONS, NOT CYCLES). (2) differential O0-vs-O2 byte equality of the
output struct AND all 18 G53 static-state objects after every call.
Usage: python <this> probe <elf>  |  python <this> equiv <o0.elf> <o2.elf> <seeds> <calls>
Needs unicorn 2.1.4 + pyelftools (optionally in <repo>/.build/e30-python-deps)."""
import sys, os, struct, random, json, hashlib
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), *(['..'] * 4), '.build', 'e30-python-deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_BLOCK
from unicorn.arm_const import *

G53 = ["control_remainder","dropped_ticks","normal_permission","pas","port_trace","chain_tick","fast_phase","state_0","state_1","state_2","state_3","state_4","state_5","state_6","state_7","supervisor_phase","ax","eb74"]  # all static objects of the four G53 translation units
RET = 0x0803F000

class Target:
    def __init__(self, path, count=False):
        self.uc = uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
        uc.mem_map(0x08000000, 0x40000)
        uc.mem_map(0x20000000, 0xC000)           # exact GD32F303RC SRAM
        with open(path, 'rb') as f:
            elf = ELFFile(f)
            for seg in elf.iter_segments():
                if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                    uc.mem_write(seg['p_vaddr'], seg.data())
            st = elf.get_section_by_name('.symtab')
            self.sym = {}; self.size = {}
            for s in st.iter_symbols():
                self.sym.setdefault(s.name, s['st_value']); self.size.setdefault(s.name, s['st_size'])
        # CPACR full access so FP code in O0/O2 runs (as SystemInit does)
        self.sp = self.sym['_sp']; self.ebss = self.sym['_ebss']; self.heap_end = self.sym['_heap_end']
        assert self.ebss < self.heap_end < self.sp <= 0x2000C000
        self.inp = (self.ebss + 7) & ~7; self.out = self.inp + 32; self.outsz = 0x200
        self.count = 0; self.minsp = self.sp; self.cache = {}
        if count:
            uc.hook_add(UC_HOOK_BLOCK, self._blk)
    def _blk(self, uc, addr, size, _):
        n = self.cache.get((addr, size))
        if n is None:
            b = bytes(uc.mem_read(addr, size)); i = 0; n = 0
            while i < size:
                hw = b[i] | (b[i+1] << 8)
                i += 4 if (hw >> 11) in (0x1D, 0x1E, 0x1F) else 2; n += 1
            self.cache[(addr, size)] = n
        self.count += n
        sp = uc.reg_read(UC_ARM_REG_SP)
        if sp < self.minsp: self.minsp = sp
    def call(self, name, r0=0, r1=0):
        uc = self.uc; self.count = 0; self.minsp = self.sp
        uc.reg_write(UC_ARM_REG_SP, self.sp); uc.reg_write(UC_ARM_REG_LR, RET | 1)
        uc.reg_write(UC_ARM_REG_R0, r0); uc.reg_write(UC_ARM_REG_R1, r1)
        uc.emu_start(self.sym[name] | 1, RET, count=50_000_000)
        assert uc.reg_read(UC_ARM_REG_PC) == RET, 'did not return'
        return self.count
    def update(self, raw):
        self.uc.mem_write(self.inp, raw); self.uc.mem_write(self.out, b'\0' * self.outsz)
        n = self.call('g53_port_update', self.inp, self.out)
        return n, bytes(self.uc.mem_read(self.out, self.outsz))
    def state(self):
        return b''.join(bytes(self.uc.mem_read(self.sym[s], self.size[s])) for s in G53)

def pack(adc, clu, pas, lvl, spd, el, imax, tv, di, rs, sc):
    return struct.pack('<HHBB2xIIi4B', adc, clu, pas, lvl, spd, el, imax, tv, di, rs, sc)

def probe(path):
    t = Target(path, count=True); t.call('g53_port_reset')
    seq = [0, 2, 3, 1]; st = []; stack = 0
    for tick in range(100):
        n, _ = t.update(pack(1200, 2000, seq[(tick // 10) % 4], 3, 0, 4, 900, 1, 0, 0, 0))
        st.append(n); stack = max(stack, t.sp - t.minsp)
    cu = []
    for k in range(5):   # 64-step catch-up at several points, then 20 normal steps between
        n, _ = t.update(pack(1200, 2000, seq[k % 4], 3, 0, 256, 900, 1, 0, 0, 0)); cu.append(n)
        stack = max(stack, t.sp - t.minsp)
        for j in range(20): t.update(pack(1200, 2000, seq[(j // 10 + k) % 4], 3, 0, 4, 900, 1, 0, 0, 0))
    # load-bearing: level 5, speed 1500, clu ramp, PAS forward fast
    ld = []
    for tick in range(200):
        n, _ = t.update(pack(1500 + tick, 4000 + 40 * tick, seq[(tick // 3) % 4], 5, 1500, 4, 1500, 1, 0, 0, 0)); ld.append(n)
    return {'mean': sum(st) / len(st), 'min': min(st), 'max': max(st), 'catchup64': cu,
            'loaded_mean': sum(ld) / len(ld), 'loaded_min': min(ld), 'loaded_max': max(ld),
            'port_only_stack_max': stack, 'sp': hex(t.sp), 'fixture_end': hex(t.out + t.outsz)}

def equivalence(p0, p2, seeds, calls):
    a = Target(p0); b = Target(p2); vec = 0; h = hashlib.sha256()
    for seed in range(seeds):
        rnd = random.Random(seed)
        a.call('g53_port_reset'); b.call('g53_port_reset')
        assert a.state() == b.state(), 'reset state differs'
        pas = 0; fwd = [0, 2, 3, 1]; idx = 0; clu = rnd.randrange(0, 65536); adc = rnd.randrange(0, 4096)
        lvl = rnd.choice([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, rnd.randrange(256)])
        mode = seed % 6
        for c in range(calls):
            if mode == 0:   # fully random
                raw = pack(rnd.randrange(4096), rnd.randrange(65536), rnd.randrange(256), rnd.randrange(256),
                           rnd.choice([0, rnd.randrange(8000), rnd.randrange(1 << 32)]), rnd.choice([0, 1, 4, 4, 4, 8, rnd.randrange(300), rnd.randrange(1 << 32)]),
                           rnd.choice([0, 900, 1500, rnd.randrange(-2**31, 2**31)]), rnd.randrange(2), rnd.randrange(2), rnd.randrange(2), rnd.randrange(2))
            else:           # physically shaped ride: PAS forward/back/stop, torque random walk
                r = rnd.random()
                if mode in (1, 2) and c % rnd.choice([1, 2, 3, 5, 10, 40]) == 0: idx = (idx + (1 if r < 0.9 else -1)) % 4
                if mode == 3 and r < 0.2: idx = rnd.randrange(4)
                if mode == 4 and c > calls // 2: idx = idx  # stop pedalling
                if mode == 5 and c % 7 == 0: idx = (idx - 1) % 4  # backpedal
                clu = max(0, min(65535, clu + rnd.randrange(-900, 1000)))
                adc = max(0, min(4095, adc + rnd.randrange(-60, 61)))
                if rnd.random() < 0.01: lvl = rnd.choice([0, 1, 2, 3, 4, 5, 9])
                el = 4 if rnd.random() < 0.9 else rnd.choice([1, 2, 3, 5, 8, 16, 64, 255, 256, 400])
                raw = pack(adc, clu, fwd[idx], lvl, rnd.randrange(0, 4500), el,
                           rnd.choice([1500, 1500, 900, 0]), 1 if rnd.random() > 0.01 else 0,
                           1 if rnd.random() < 0.01 else 0, 1 if rnd.random() < 0.01 else 0, 1 if rnd.random() < 0.01 else 0)
            _, oa = a.update(raw); _, ob = b.update(raw); vec += 1
            sa, sb = a.state(), b.state()
            h.update(oa); h.update(sa)
            if oa != ob or sa != sb:
                off = next(i for i in range(max(len(oa + sa), len(ob + sb))) if (oa + sa)[i:i+1] != (ob + sb)[i:i+1])
                return {'vectors': vec, 'match': False, 'first_mismatch': {'seed': seed, 'call': c, 'input': raw.hex(),
                        'where': 'output' if off < len(oa) else 'state', 'offset': off}}
    return {'vectors': vec, 'match': True, 'first_mismatch': None, 'trace_sha256': h.hexdigest()}

if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'probe': print(json.dumps(probe(sys.argv[2]), indent=1))
    else: print(json.dumps(equivalence(sys.argv[2], sys.argv[3], int(sys.argv[4]), int(sys.argv[5])), indent=1))
