#include "g53_port_chain.h"
#include <string.h>

/* Portable C transcription of pinned TQ-02B models; original address labels
 * identify private state only. No access to hardware or original firmware RAM.
 * Explicit little-endian writes preserve overlapping register publications.
 * Fixed D4 configuration is initialized below. No heap, floating point or PI1/2.
 */
enum { G=0x20000888, T=0x2000356c, D=0x200011e8, DG=D,
       M=0x200039a4, E=0x20001344, Q=0x20000f6c, HB=0x200012dc,
       DIAG=0x20000ee0, P=0x20003ec4, CUR1=0x200014cc };
static uint32_t chain_tick;
static uint8_t fast_phase, supervisor_phase;
static uint8_t state_0[576];
static uint8_t state_1[1616];
static uint8_t state_2[8];
static uint8_t state_3[96];
static uint8_t state_4[56];
static uint8_t state_5[768];
static uint8_t state_6[20];
static uint8_t state_7[16];
static uint8_t *state_byte(uint32_t address)
{
    if (address >= 0x20000888u && address < 0x20000ac8u) return &state_0[address - 0x20000888u];
    if (address >= 0x20000e80u && address < 0x200014d0u) return &state_1[address - 0x20000e80u];
    if (address >= 0x20003000u && address < 0x20003008u) return &state_2[address - 0x20003000u];
    if (address >= 0x2000356cu && address < 0x200035ccu) return &state_3[address - 0x2000356cu];
    if (address >= 0x20003700u && address < 0x20003738u) return &state_4[address - 0x20003700u];
    if (address >= 0x200039a4u && address < 0x20003ca4u) return &state_5[address - 0x200039a4u];
    if (address >= 0x20003f00u && address < 0x20003f14u) return &state_6[address - 0x20003f00u];
    if (address >= 0x20006100u && address < 0x20006110u) return &state_7[address - 0x20006100u];
    return NULL; /* outside closed fixed-profile state */
}
static int64_t read_8(uint32_t address) { const uint8_t *p=state_byte(address); return p ? *p : 0; }
static int64_t read_16(uint32_t a) { return read_8(a) | (read_8(a+1) << 8); }
static int64_t read_32(uint32_t a) { return read_16(a) | (read_16(a+2) << 16); }
static void write_8(uint32_t a, int64_t v) { uint8_t *p=state_byte(a); if(p) *p=(uint8_t)v; }
static void write_16(uint32_t a, int64_t v) { uint16_t w=(uint16_t)v; write_8(a,w); write_8(a+1,w>>8); }
static void write_32(uint32_t a, int64_t v) { uint32_t w=(uint32_t)v; write_16(a,w); write_16(a+2,w>>16); }
static int64_t sxth(int64_t v) { uint16_t w=(uint16_t)v; return w>INT16_MAX ? (int32_t)w-65536 : w; }
static int64_t sxtb(int64_t v) { uint8_t w=(uint8_t)v; return w>INT8_MAX ? (int32_t)w-256 : w; }
static int64_t signed32(int64_t v) { uint32_t w=(uint32_t)v; return w>INT32_MAX ? (int64_t)w-INT64_C(4294967296) : w; }
static int64_t read_s8(uint32_t a) { return sxtb(read_8(a)); }
static int64_t signed64(uint64_t v) { return v<=INT64_MAX ? (int64_t)v : -1-(int64_t)(UINT64_MAX-v); }
static int64_t multiply64(int64_t a,int64_t b) { return signed64((uint64_t)a*(uint64_t)b); }
static int64_t shift_left(int64_t a,unsigned b) { return signed64((uint64_t)a<<b); }
static int64_t asr64(int64_t a,unsigned b) { return a>=0 ? (int64_t)((uint64_t)a>>b) : -1-(int64_t)((uint64_t)(-(a+1))>>b); }
static int64_t floor_div(int64_t a,int64_t b) { int64_t q=a/b,r=a%b; return q-((r!=0)&&((a<0)!=(b<0))); }
static int64_t sdiv(int64_t a,int64_t b) { return b==0 ? 0 : a/b; }
static int64_t udiv(int64_t a,int64_t b) { return (uint32_t)b==0 ? 0 : (uint32_t)a/(uint32_t)b; }
static int64_t abs64(int64_t a) { return a<0 ? -a : a; }
static int64_t max64(int64_t a,int64_t b) { return a>b ? a : b; }
static void clear_live(void) { write_32(T+0x46,0); }
static int64_t rider_lut(int64_t x)
{
    static const uint16_t xs[]={10,30,40,50,60,70}, ys[]={200,400,520,620,710,740};
    if(x<=xs[0]) return ys[0];
    if(x>=xs[5]) return ys[5];
    for(unsigned i=0;i<5;++i) if(x>=xs[i] && x<xs[i+1]) return ys[i]+sdiv((x-xs[i])*(ys[i+1]-ys[i]),xs[i+1]-xs[i]);
    return 0;
}
enum chain_label {
    LABEL_BEEA,
    LABEL_BF0E,
    LABEL_BF4A,
    LABEL_BFA8,
    LABEL_BFB6,
    LABEL_BFBA,
    LABEL_C026,
    LABEL_C090,
    LABEL_C0D2,
    LABEL_C122,
    LABEL_C12E,
    LABEL_C132,
    LABEL_C160,
    LABEL_C1B6,
    LABEL_C1CC,
    LABEL_C212,
    LABEL_C266,
    LABEL_C27C,
    LABEL_C282,
    LABEL_C288,
    LABEL_C2A6,
    LABEL_C362,
    LABEL_C36C,
    LABEL_C370,
    LABEL_C386,
    LABEL_C38C,
    LABEL_C390,
    LABEL_C398,
    LABEL_C39A,
    LABEL_C39E,
    LABEL_S11_C1A6,
    LABEL_S11_C2BC,
    LABEL_S11_C2CA,
    LABEL_S11_C2D8,
    LABEL_S11_C2E8,
    LABEL_S11_C2FC,
    LABEL_S11_C314,
    LABEL_S11_C328,
    LABEL_S11_C340,
    LABEL_S11_C34C,
    LABEL_S11_C362
};
static int64_t state_label(int64_t s) { switch (s) {
case 0: return LABEL_BFB6;
case 1: return LABEL_BF4A;
case 2: return LABEL_BF0E;
case 4: return LABEL_BFBA;
case 5: return LABEL_C026;
case 6: return LABEL_C090;
case 7: return LABEL_C0D2;
case 10: return LABEL_C12E;
case 11: return LABEL_C160;
default: return LABEL_BFA8; }}
static int64_t service_label(int64_t s) { switch (s) {
case 0: return LABEL_S11_C1A6;
case 1: return LABEL_S11_C362;
case 2: return LABEL_S11_C2BC;
case 3: return LABEL_S11_C2CA;
case 10: return LABEL_S11_C2E8;
case 11: return LABEL_S11_C2FC;
case 12: return LABEL_S11_C314;
case 13: return LABEL_S11_C328;
case 21: return LABEL_S11_C362;
case 22: return LABEL_S11_C340;
case 23: return LABEL_S11_C34C;
default: return LABEL_S11_C2D8; }}

/* Pinned tq02b_fsm.py:64; register-width writes remain explicit. */
static void chain_fsm_model(void)
{
    int64_t cand = 0;
    int64_t d = 0;
    int64_t diff = 0;
    int64_t g04 = 0;
    int64_t g05 = 0;
    int64_t g0b = 0;
    int64_t lvl = 0;
    int64_t m = 0;
    int64_t new = 0;
    int64_t out = 0;
    int64_t prod = 0;
    int64_t st = 0;
    int64_t step = 0;
    int64_t t08 = 0;
    int64_t t0c = 0;
    int64_t t0e = 0;
    int64_t t14 = 0;
    int64_t t34 = 0;
    int64_t t36 = 0;
    int64_t t38 = 0;
    int64_t t3a = 0;
    int64_t t46 = 0;
    int64_t t48 = 0;
    int64_t x = 0;
    g04 = read_8((G + 4LL));
    g05 = read_8((G + 5LL));
    g0b = read_8((G + 11LL));
    lvl = read_s8((G + 7LL));
    x = read_16((G + 542LL));
    t0e = read_8((T + 14LL));
    m = (g05 & t0e);
    write_16((T + 4LL), x);
    write_8((T + 6LL), lvl);
    write_8((T + 0LL), g04);
    write_8((T + 1LL), m);
    write_8((T + 2LL), g0b);
    if ((((g04 == 1LL)) || ((m == 1LL)))) {
        st = read_8((T + 50LL));
        write_16((T + 54LL), 0LL);
        write_16((T + 0LL), 0LL);
        if (((st == 4LL))) {
            write_8((T + 50LL), 5LL);
        }
    }
    step = (((lvl >= 1LL)) ? read_16((T + (30LL + multiply64(2LL, (lvl & 255LL))))) : 0LL);
    write_16((T + 68LL), step);
    st = read_8((T + 50LL));
    t46 = 0;
    t48 = 0;
    if (((st > 5LL))) {
        write_8((T + 50LL), 0LL);
    } else {
        if (((st == 0LL))) {
            write_32((T + 52LL), 0LL);
            write_16((T + 56LL), 0LL);
            write_8((T + 77LL), 0LL);
            write_8((T + 75LL), 0LL);
            write_8((T + 50LL), 1LL);
        } else {
            if (((st == 1LL))) {
                t14 = read_16((T + 20LL));
                t34 = read_16((T + 52LL));
                if (((x > t14))) {
                    t0c = read_16((T + 12LL));
                    if (((t34 >= t0c))) {
                        write_16((T + 52LL), t0c);
                        write_8((T + 75LL), 1LL);
                    } else {
                        write_16((T + 52LL), (t34 + 10LL));
                    }
                } else {
                    if (((t34 == 0LL))) {
                        write_8((T + 75LL), 0LL);
                        write_8((T + 50LL), 2LL);
                    } else {
                        write_16((T + 52LL), (t34 - 10LL));
                    }
                }
                clear_live();
            } else {
                if (((st == 2LL))) {
                    if (((x <= read_16((T + 18LL))))) {
                        if (((x > read_16((T + 20LL))))) {
                            write_8((T + 50LL), 4LL);
                            write_16((T + 58LL), 10LL);
                            write_8((T + 74LL), 1LL);
                        }
                    } else {
                        write_8((T + 50LL), 3LL);
                    }
                    clear_live();
                } else {
                    if (((st == 3LL))) {
                        if (((x <= read_16((T + 20LL))))) {
                            write_8((T + 77LL), 0LL);
                            write_16((T + 56LL), 0LL);
                            write_8((T + 50LL), 2LL);
                        } else {
                            t38 = read_16((T + 56LL));
                            if (((t38 > 24LL))) {
                                write_32((T + 70LL), 0LL);
                                write_8((T + 77LL), 1LL);
                            } else {
                                write_16((T + 56LL), (t38 + 1LL));
                            }
                        }
                    } else {
                        if (((st == 4LL))) {
                            if (((x <= read_16((T + 18LL))))) {
                                t14 = read_16((T + 20LL));
                                if (((x >= t14))) {
                                    if (((x >= read_16((T + 22LL))))) {
                                        write_16((T + 72LL), 4096LL);
                                    } else {
                                        d = (x - t14);
                                        write_16((T + 60LL), d);
                                        write_16((T + 72LL), multiply64(d, read_16((T + 24LL))));
                                    }
                                } else {
                                    write_8((T + 50LL), 2LL);
                                }
                            } else {
                                write_8((T + 50LL), 3LL);
                            }
                        } else {
                            if (((st == 5LL))) {
                                t36 = read_16((T + 54LL));
                                if (((x > read_16((T + 10LL))))) {
                                    cand = (t36 + 10LL);
                                    write_16((T + 54LL), cand);
                                    if ((((cand & 65535LL) > read_16((T + 12LL))))) {
                                        write_16((T + 54LL), read_16((T + 12LL)));
                                        write_8((T + 76LL), 1LL);
                                    }
                                } else {
                                    if (((t36 == 0LL))) {
                                        write_8((T + 50LL), 2LL);
                                        write_8((T + 76LL), 0LL);
                                    } else {
                                        write_16((T + 54LL), (t36 - 10LL));
                                    }
                                }
                                clear_live();
                            }
                        }
                    }
                }
            }
        }
    }
    t3a = read_16((T + 58LL));
    if (((t3a != 0LL))) {
        write_16((T + 58LL), (t3a - 1LL));
    } else {
        write_8((T + 74LL), 0LL);
    }
    t48 = read_16((T + 72LL));
    t46 = read_16((T + 70LL));
    prod = multiply64(t48, 10LL);
    diff = ((prod & 65535LL) - t46);
    write_32((T + 64LL), diff);
    t08 = read_8((T + 8LL));
    write_16((T + 72LL), prod);
    new = (((diff > sxth(step))) ? (t46 + step) : prod);
    write_16((T + 70LL), new);
    if (((t08 == 1LL))) {
        out = floor_div((new & 65535LL), 10LL);
        write_16((G + 546LL), out);
        write_8((G + 510LL), read_8((T + 75LL)));
        write_8((G + 515LL), read_8((T + 74LL)));
        write_8((G + 511LL), read_8((T + 77LL)));
        write_8((G + 512LL), read_8((T + 76LL)));
    } else {
        write_16((G + 546LL), 0LL);
        write_16((G + 510LL), 0LL);
        write_8((G + 515LL), 0LL);
        write_8((G + 512LL), 0LL);
    }
    if ((((g0b == 1LL)) && ((lvl == 9LL)))) {
        write_16((G + 546LL), 4096LL);
    }
}

/* Pinned tq02b_d7ec.py:381; register-width writes remain explicit. */
static void chain__tail_G05(int64_t mode)
{
    int64_t d1c = 0;
    int64_t ea = 0;
    int64_t outp = 0;
    int64_t r1 = 0;
    int64_t r7 = 0;
    if ((((read_8((D + 24LL)) == 1LL)) || ((read_8((D + 25LL)) == 1LL)))) {
        write_16((D + 24LL), 0LL);
        write_16((D + 234LL), 0LL);
        ea = 0LL;
    } else {
        ea = read_16((D + 234LL));
    }
    d1c = read_16((D + 28LL));
    r7 = 0LL;
    if (((ea < d1c))) {
        ea = (ea + 10LL);
        write_16((D + 234LL), ea);
        if (((read_8((D + 22LL)) == 1LL))) {
            if (((((read_16((D + 12LL)) + 150LL) & 65535LL) >= 151LL))) {
                write_16((D + 234LL), (d1c - 20LL));
            }
        }
        write_16((D + 238LL), 0LL);
        r7 = 1LL;
    }
    write_8((D + 240LL), r7);
    write_8((M + 666LL), r7);
    r1 = (((read_8((D + 32LL)) == 1LL)) ? read_16((D + 196LL)) : 0LL);
    outp = read_32((M + 744LL));
    write_16((D + 236LL), r1);
    write_16((0LL + outp), r1);
    (void)mode;
}

/* Pinned tq02b_d7ec.py:376; register-width writes remain explicit. */
static void chain__tail_from_DC30(int64_t fp, int64_t mode)
{
    write_16((D + 238LL), 0LL);
    chain__tail_G05(mode);
    (void)fp;
}

/* Pinned tq02b_d7ec.py:101; register-width writes remain explicit. */
static void chain_d7ec_model(void)
{
    int64_t c2 = 0;
    int64_t c4 = 0;
    int64_t candA = 0;
    int64_t candB = 0;
    int64_t conv = 0;
    int64_t convh = 0;
    int64_t cur = 0;
    int64_t d17 = 0;
    int64_t d1c = 0;
    int64_t d2 = 0;
    int64_t d24 = 0;
    int64_t d26 = 0;
    int64_t d4 = 0;
    int64_t d6 = 0;
    int64_t d78 = 0;
    int64_t den = 0;
    int64_t diff = 0;
    int64_t env = 0;
    int64_t env16 = 0;
    int64_t evid = 0;
    int64_t fall = 0;
    int64_t fp = 0;
    int64_t idx = 0;
    int64_t in_history = 0;
    int64_t ip2 = 0;
    int64_t k = 0;
    int64_t k16 = 0;
    int64_t lim = 0;
    int64_t limA = 0;
    int64_t live = 0;
    int64_t lr_in = 0;
    int64_t lvl = 0;
    int64_t m28 = 0;
    int64_t m2a = 0;
    int64_t m2c = 0;
    int64_t m2e5 = 0;
    int64_t m2ed = 0;
    int64_t m52 = 0;
    int64_t maxr = 0;
    int64_t mode = 0;
    int64_t move = 0;
    int64_t new = 0;
    int64_t prev = 0;
    int64_t prev_r = 0;
    int64_t ptr = 0;
    int64_t q = 0;
    int64_t q5 = 0;
    int64_t r1 = 0;
    int64_t r2 = 0;
    int64_t r2c = 0;
    int64_t r2v = 0;
    int64_t r3 = 0;
    int64_t r5 = 0;
    int64_t r6 = 0;
    int64_t r7 = 0;
    int64_t ratio = 0;
    int64_t raw = 0;
    int64_t rawu = 0;
    int64_t ready = 0;
    int64_t rise = 0;
    int64_t rise_r = 0;
    int64_t sb = 0;
    int64_t sbp = 0;
    int64_t sbv = 0;
    int64_t sel = 0;
    int64_t sl = 0;
    int64_t sl2 = 0;
    int64_t step = 0;
    int64_t taper = 0;
    int64_t target = 0;
    int64_t tdiff = 0;
    int64_t x = 0;
    mode = read_8((D + 32LL));
    if (((mode == 0LL))) {
        return;
    }
    lvl = read_s8((M + 740LL));
    write_8((D + 0LL), lvl);
    write_8((D + 24LL), read_8((M + 665LL)));
    write_8((D + 25LL), (read_8((M + 52LL)) & read_8((D + 158LL))));
    write_16((D + 2LL), read_16((M + 26LL)));
    m52 = read_16((M + 82LL));
    write_16((D + 4LL), m52);
    ptr = read_32((M + 116LL));
    sl = read_16((0LL + ptr));
    evid = read_16((M + 148LL));
    write_16((D + 14LL), evid);
    m2e5 = read_8((M + 741LL));
    write_8((D + 1LL), m2e5);
    write_16((D + 6LL), sl);
    sb = read_16((M + 80LL));
    write_16((D + 8LL), sb);
    raw = read_16((M + 146LL));
    write_16((D + 12LL), raw);
    fp = sxth(raw);
    m2c = read_16((M + 44LL));
    write_16((D + 20LL), m2c);
    d1c = max64(m2c, read_16((D + 156LL)));
    write_16((D + 28LL), d1c);
    m28 = read_16((M + 40LL));
    write_16((D + 16LL), m28);
    m2a = read_16((M + 42LL));
    write_16((D + 18LL), m2a);
    m2ed = read_8((M + 749LL));
    write_8((D + 26LL), m2ed);
    move = read_8((M + 150LL));
    write_8((D + 22LL), move);
    if (((mode == 1LL))) {
        cur = read_16((0LL + CUR1));
        d17 = read_8((M + 671LL));
    } else {
        d17 = 0LL;
        if (((fp <= 0LL))) {
            cur = 0LL;
        } else {
            if (((sl <= read_16((D + 124LL))))) {
                cur = read_16((D + 126LL));
            } else {
                cur = ((((m52 - 500LL) >= sl)) ? read_16((D + 128LL)) : read_16((D + 130LL)));
            }
        }
    }
    write_16((D + 10LL), cur);
    write_8((D + 23LL), d17);
    d24 = read_16((D + 36LL));
    d26 = read_8((D + 38LL));
    write_16((D + 192LL), d24);
    write_16((D + 190LL), d26);
    r2 = 0;
    in_history = 0LL;
    env = 0;
    if (((fp <= (-1LL)))) {
        write_16((D + 170LL), 0LL);
        write_32((D + 194LL), 0LL);
        write_16((D + 212LL), 0LL);
        write_16((D + 218LL), 0LL);
        write_16((D + 236LL), 0LL);
        chain__tail_from_DC30(fp, mode);
        return;
    }
    prev = read_16((D + 170LL));
    k = multiply64(fp, 8LL);
    if (((fp == 0LL))) {
        k = 40LL;
    }
    write_16((D + 188LL), k);
    if (((cur > prev))) {
        write_16((D + 170LL), cur);
        env = cur;
    } else {
        k16 = (k & 65535LL);
        env = udiv(multiply64(prev, k16), (k16 | 1LL));
        write_16((D + 170LL), env);
        if (((cur == 0LL))) {
            in_history = 1LL;
        }
    }
    if ((!in_history)) {
        write_16((D + 172LL), evid);
        write_16((D + 174LL), env);
        write_16((D + 176LL), sl);
        write_16((D + 178LL), 0LL);
        write_16((D + 182LL), env);
        write_16((D + 184LL), env);
    } else {
        if (((move == 1LL))) {
            write_16((D + 178LL), (evid - read_16((D + 172LL))));
            r6 = (evid - read_16((D + 172LL)));
        } else {
            env = 0LL;
            r6 = 0LL;
            write_32((D + 182LL), 0LL);
            write_32((D + 170LL), 0LL);
            write_32((D + 174LL), 0LL);
            write_16((D + 178LL), 0LL);
        }
        sl2 = m2a;
        ip2 = m28;
        sbp = 136LL;
        if (((lvl >= 1LL))) {
            if (((read_16((D + 176LL)) > read_16((D + 134LL))))) {
                sbp = (136LL + multiply64(2LL, lvl));
            }
        }
        limA = read_16((D + sbp));
        sbv = (r6 & 65535LL);
        write_16((D + 180LL), limA);
        candA = 0LL;
        if (((sbv < limA))) {
            if (((read_16((D + 182LL)) != 0LL))) {
                r5 = read_16((D + 174LL));
                candA = (r5 - udiv(multiply64(r5, sbv), limA));
            }
        }
        write_16((D + 182LL), candA);
        candB = 0LL;
        if (((sl2 < ip2))) {
            if (((read_16((D + 184LL)) != 0LL))) {
                r1 = read_16((D + 174LL));
                candB = (r1 - udiv(multiply64(r1, sl2), ip2));
            }
        }
        d2 = read_8((D + 132LL));
        write_16((D + 184LL), candB);
        sel = candB;
        if (((d2 == 0LL))) {
            sel = candA;
        }
        if ((((candA & 65535LL) < (candB & 65535LL)))) {
            sel = candA;
        }
        write_16((D + 186LL), sel);
        if ((((sel & 65535LL) < (env & 65535LL)))) {
            write_16((D + 170LL), sel);
            env = sel;
        }
    }
    env16 = (env & 65535LL);
    ready = (((evid >= d26)) || ((env16 >= d24)));
    if ((!ready)) {
        if ((((move == 0LL)) || ((d17 == 1LL)))) {
            c2 = 0LL;
            write_16((D + 194LL), 0LL);
        } else {
            c2 = read_16((D + 194LL));
        }
    } else {
        if (((fp <= 20LL))) {
            x = (multiply64(env16, 700LL) & 4294967295LL);
        } else {
            x = (multiply64((multiply64(env16, raw) & 4294967295LL), 35LL) & 4294967295LL);
        }
        c2 = (floor_div(x, 10000LL) & 4294967295LL);
        write_16((D + 194LL), c2);
    }
    c4 = floor_div((multiply64((c2 & 65535LL), 637LL) & 4294967295LL), 1000LL);
    write_16((D + 196LL), c4);
    r2 = (m2e5 + asr64((m2e5 + 1LL), 1LL));
    if (((sxtb(r2) >= 8LL))) {
        r2 = 8LL;
    }
    write_8((D + 200LL), r2);
    r3 = (lvl & 255LL);
    d78 = read_8((D + 120LL));
    if ((((d78 == 1LL)) && ((r3 == (r2 & 255LL))))) {
        r7 = read_16((D + 64LL));
        den = udiv(multiply64(read_16((D + 110LL)), 100LL), r7);
        write_16((D + 198LL), den);
        write_8((D + 202LL), 0LL);
        write_8((D + 203LL), lvl);
        maxr = read_16((D + (64LL + multiply64(2LL, r3))));
        if (((maxr <= r7))) {
            ratio = maxr;
        } else {
            ratio = (r7 + udiv((multiply64((maxr - r7), c4) & 4294967295LL), (den & 65535LL)));
        }
        write_16((D + 204LL), ratio);
        if ((((ratio & 65535LL) > maxr))) {
            ratio = maxr;
        }
        write_16((D + 204LL), ratio);
        idx = lvl;
        rise = read_16((D + (40LL + multiply64(2LL, idx))));
        lim = read_16((D + (88LL + multiply64(2LL, idx))));
    } else {
        if (((lvl < 1LL))) {
            write_16((D + 204LL), 0LL);
            write_16((D + 232LL), 0LL);
            ratio = 0LL;
            rise = 0LL;
            lim = 0LL;
        } else {
            ratio = read_16((D + (64LL + multiply64(2LL, r3))));
            write_16((D + 204LL), ratio);
            rise = read_16((D + (40LL + multiply64(2LL, r3))));
            lim = read_16((D + (88LL + multiply64(2LL, r3))));
        }
    }
    if ((((lvl >= 1LL)) || (((d78 == 1LL)) && ((r3 == (r2 & 255LL)))))) {
        write_16((D + 232LL), rise);
    }
    write_16((D + 220LL), lim);
    if (((m2ed == 1LL))) {
        ratio = read_16((D + 84LL));
        rise = read_16((D + 60LL));
        lim = read_16((D + 108LL));
        write_16((D + 204LL), ratio);
        write_16((D + 232LL), rise);
        write_16((D + 220LL), lim);
    }
    if (((sl <= 1000LL))) {
        taper = 200LL;
    } else {
        if (((asr64(sl, 5LL) > 74LL))) {
            taper = 0LL;
        } else {
            taper = (200LL - sdiv((sl - 1000LL), 7LL));
        }
    }
    write_16((D + 210LL), taper);
    if (((read_8((D + 34LL)) == 1LL))) {
        if ((((ratio & 65535LL) > (taper & 65535LL)))) {
            write_16((D + 204LL), taper);
            ratio = taper;
        }
    }
    prev_r = read_16((D + 208LL));
    step = sxth(read_16((D + 122LL)));
    diff = (ratio - prev_r);
    r2v = ratio;
    if (((sxth(diff) > step))) {
        r2v = (step + prev_r);
    }
    d4 = (1LL ? floor_div(multiply64(c4, (r2v & 65535LL)), 100LL) : 0LL);
    write_16((D + 212LL), d4);
    write_16((D + 206LL), diff);
    write_16((D + 208LL), r2v);
    lr_in = read_16((D + 8LL));
    if (((read_8((D + 33LL)) == 1LL))) {
        lr_in = fp;
    }
    d6 = rider_lut((lr_in & 65535LL));
    write_16((D + 214LL), d6);
    conv = (udiv(multiply64(read_16((D + 212LL)), 1000LL), (d6 & 65535LL)) & 4294967295LL);
    write_16((D + 218LL), conv);
    convh = (conv & 65535LL);
    if (((cur == 0LL))) {
        r2c = sxth(read_16((D + 12LL)));
        if ((((r2c <= read_8((D + 86LL)))) && 0LL)) {
        }
        if (((r2c > read_8((D + 86LL))))) {
            write_16((D + 222LL), read_16((D + 220LL)));
        } else {
            if (((r2c >= read_8((D + 87LL))))) {
            } else {
                write_16((D + 222LL), 0LL);
            }
        }
    }
    if (((asr64(convh, 3LL) > 124LL))) {
        convh = 1000LL;
        write_16((D + 218LL), 1000LL);
    } else {
        r2c = read_16((D + 222LL));
        if ((!((r2c < convh)))) {
            convh = r2c;
            write_16((D + 218LL), r2c);
        }
    }
    q = ((uint64_t)multiply64((shift_left(convh, 12LL) & 4294967295LL), 274877907LL) >> 38LL);
    q5 = multiply64(q, 5LL);
    target = ((((q & 61440LL) != 0LL)) ? 4294942720LL : (shift_left(q5, 1LL) & 4294967295LL));
    live = read_16((D + 226LL));
    rise_r = read_16((D + 232LL));
    tdiff = ((target & 65535LL) - live);
    write_16((D + 224LL), target);
    write_32((D + 228LL), tdiff);
    if (((tdiff > sxth(rise_r)))) {
        new = (rise_r + live);
    } else {
        fall = sxth(read_16((D + 62LL)));
        new = target;
        if (((tdiff < fall))) {
            new = (fall + live);
        }
    }
    write_16((D + 226LL), new);
    write_16((D + 238LL), floor_div((new & 65535LL), 10LL));
    if (((cur == 0LL))) {
        rawu = read_16((D + 12LL));
        if ((((rawu == 0LL)) || ((read_8((D + 23LL)) == 1LL)))) {
            write_16((D + 170LL), 0LL);
            write_32((D + 194LL), 0LL);
            write_16((D + 212LL), 0LL);
            write_16((D + 218LL), 0LL);
            write_32((D + 224LL), 0LL);
            write_16((D + 238LL), 0LL);
        }
    }
    chain__tail_G05(mode);
}

/* Pinned tq02b_e1e8.py:48; register-width writes remain explicit. */
static int64_t chain_lut_model(int64_t x, int64_t n, int64_t tab)
{
    int64_t ip = 0;
    int64_t lr = 0;
    int64_t r0 = 0;
    int64_t r1 = 0;
    int64_t r3 = 0;
    int64_t r4 = 0;
    int64_t r5 = 0;
    int64_t r6 = 0;
    int64_t x0 = 0;
    int64_t x1 = 0;
    int64_t y0 = 0;
    r1 = ((n - 1LL) & UINT32_MAX);
    r3 = sxth(r1);
    lr = read_16((0LL + tab));
    r1 = read_16((0LL + ((tab + shift_left(r3, 2LL)) & UINT32_MAX)));
    if (((lr > r1))) {
        if (((r1 >= x))) {
            return sxth(read_16((0LL + (((tab + shift_left(r3, 2LL)) + 2LL) & UINT32_MAX))));
        }
        if (((lr <= x))) {
            return sxth(read_16((0LL + (tab + 2LL))));
        }
        ip = 1LL;
        r6 = 65535LL;
    } else {
        if (((r1 <= x))) {
            return sxth(read_16((0LL + (((tab + shift_left(r3, 2LL)) + 2LL) & UINT32_MAX))));
        }
        if (((lr >= x))) {
            return sxth(read_16((0LL + (tab + 2LL))));
        }
        ip = 0LL;
        r6 = 1LL;
    }
    r1 = 0LL;
    if (((r3 >= 1LL))) {
        while (1LL) {
            r4 = sxth(r3);
            r4 = ((r4 - r1) & UINT32_MAX);
            r4 = ((r1 + asr64(r4, 1LL)) & UINT32_MAX);
            r5 = read_16((0LL + ((tab + shift_left(sxth(r4), 2LL)) & UINT32_MAX)));
            r5 = ((x - r5) & UINT32_MAX);
            r5 = (multiply64(r5, r6) & UINT32_MAX);
            r5 = sxth(r5);
            if (((r5 > 0LL))) {
                r1 = ((r4 + 1LL) & UINT32_MAX);
                r1 = sxth(r1);
                r4 = r3;
            } else {
                r1 = sxth(r1);
            }
            r3 = sxth(r4);
            if ((!((r3 > r1)))) {
                break;
            }
        }
        lr = read_16((0LL + ((tab + shift_left(r1, 2LL)) & UINT32_MAX)));
    }
    if (((lr <= x))) {
        if (((lr >= x))) {
            return sxth(read_16((0LL + (((tab + shift_left(r1, 2LL)) + 2LL) & UINT32_MAX))));
        }
        r6 = (ip ^ 1LL);
        r3 = (r1 - ip);
        r1 = (r1 + r6);
    } else {
        r1 = (r1 + ip);
        r3 = (r1 - 1LL);
    }
    r6 = sxth(read_16((0LL + (((tab + shift_left(r1, 2LL)) + 2LL) & UINT32_MAX))));
    x1 = sxth(read_16((0LL + ((tab + shift_left(r1, 2LL)) & UINT32_MAX))));
    x0 = sxth(read_16((0LL + ((tab + shift_left(r3, 2LL)) & UINT32_MAX))));
    y0 = sxth(read_16((0LL + (((tab + shift_left(r3, 2LL)) + 2LL) & UINT32_MAX))));
    r0 = ((x - x0) & UINT32_MAX);
    r6 = ((r6 - y0) & UINT32_MAX);
    r0 = (multiply64(r0, r6) & UINT32_MAX);
    r1 = ((x1 - x0) & UINT32_MAX);
    r0 = (sdiv(signed32(r0), signed32(r1)) & UINT32_MAX);
    r0 = ((r0 + y0) & UINT32_MAX);
    return sxth(r0);
}

/* Pinned tq02b_e1e8.py:108; register-width writes remain explicit. */
static void chain_pi_reset(int64_t s)
{
    write_32((0LL + (s + 32LL)), 0LL);
    write_32((0LL + (s + 36LL)), 0LL);
    write_16((0LL + (s + 40LL)), 0LL);
}

/* Pinned tq02b_e1e8.py:115; register-width writes remain explicit. */
static void chain_pi_step(int64_t s)
{
    int64_t hi = 0;
    int64_t ip = 0;
    int64_t lo = 0;
    int64_t r1 = 0;
    int64_t r2 = 0;
    int64_t r2s = 0;
    int64_t r3 = 0;
    int64_t r3k = 0;
    r1 = read_16((0LL + s));
    r2 = read_16((0LL + (s + 2LL)));
    ip = read_32((0LL + (s + 32LL)));
    r3 = ((r1 - r2) & UINT32_MAX);
    r2 = sxth(read_16((0LL + (s + 6LL))));
    r1 = sxth(r3);
    ip = ((multiply64(r2, r1) + ip) & UINT32_MAX);
    hi = signed32(read_32((0LL + (s + 12LL))));
    lo = signed32(read_32((0LL + (s + 16LL))));
    write_16((0LL + (s + 24LL)), r3);
    write_32((0LL + (s + 32LL)), ip);
    if (((signed32(ip) > hi))) {
        ip = (hi & UINT32_MAX);
        write_32((0LL + (s + 32LL)), ip);
    } else {
        if (((signed32(ip) < lo))) {
            ip = (lo & UINT32_MAX);
            write_32((0LL + (s + 32LL)), ip);
        }
    }
    r2 = ((r3 - read_16((0LL + (s + 8LL)))) & UINT32_MAX);
    r3k = sxth(read_16((0LL + (s + 4LL))));
    write_16((0LL + (s + 24LL)), r2);
    r3 = (multiply64(r3k, sxth(r2)) & UINT32_MAX);
    write_32((0LL + (s + 28LL)), r3);
    if (((signed32(r3) > hi))) {
        r3 = (hi & UINT32_MAX);
        write_32((0LL + (s + 28LL)), r3);
    } else {
        if (((signed32(r3) < lo))) {
            r3 = (lo & UINT32_MAX);
            write_32((0LL + (s + 28LL)), r3);
        }
    }
    r2 = ((r3 + ip) & UINT32_MAX);
    write_32((0LL + (s + 36LL)), r2);
    if (((signed32(r2) > hi))) {
        r2 = (hi & UINT32_MAX);
        write_32((0LL + (s + 36LL)), r2);
    } else {
        if (((signed32(r2) < lo))) {
            r2 = (lo & UINT32_MAX);
            write_32((0LL + (s + 36LL)), r2);
        }
    }
    write_16((0LL + (s + 40LL)), (asr64(r2, 12LL) & 65535LL));
    r2s = sxth((asr64(r2, 12LL) & 65535LL));
    r1 = read_16((0LL + (s + 20LL)));
    if (((r2s > sxth(r1)))) {
        write_16((0LL + (s + 40LL)), r1);
        return;
    }
    r1 = read_16((0LL + (s + 22LL)));
    if (((r2s >= sxth(r1)))) {
        return;
    }
    write_16((0LL + (s + 40LL)), r1);
}

/* Pinned tq02b_e1e8.py:530; register-width writes remain explicit. */
static int64_t chain_e754(int64_t r1)
{
    int64_t r3 = 0;
    int64_t r7 = 0;
    r3 = ((r1 - read_16((0LL + (E + 40LL)))) & UINT32_MAX);
    r7 = ((r1 - read_16((0LL + (E + 42LL)))) & UINT32_MAX);
    r3 = (((r3 != 0LL)) ? 1LL : 0LL);
    r7 = (((r7 != 0LL)) ? 1LL : 0LL);
    write_16((0LL + (E + 198LL)), 0LL);
    if ((((r3 & r7) != 0LL))) {
        return 0LL;
    }
    if (((read_8((0LL + (E + 106LL))) != 0LL))) {
        write_8((0LL + (E + 212LL)), 1LL);
    }
    return 0LL;
}

/* Pinned tq02b_e1e8.py:165; register-width writes remain explicit. */
static void chain_e1e8_model(void)
{
    int64_t absm44 = 0;
    int64_t dd = 0;
    int64_t diag = 0;
    int64_t do_c2 = 0;
    int64_t e26 = 0;
    int64_t eq = 0;
    int64_t fp = 0;
    int64_t goto616 = 0;
    int64_t goto_common = 0;
    int64_t h60 = 0;
    int64_t h64 = 0;
    int64_t hi = 0;
    int64_t ip = 0;
    int64_t ipx = 0;
    int64_t lr = 0;
    int64_t m44 = 0;
    int64_t m4a = 0;
    int64_t m4a_ = 0;
    int64_t n = 0;
    int64_t prod = 0;
    int64_t q = 0;
    int64_t r0 = 0;
    int64_t r0c8 = 0;
    int64_t r0c9 = 0;
    int64_t r0ca = 0;
    int64_t r0x = 0;
    int64_t r1 = 0;
    int64_t r1_ = 0;
    int64_t r1x = 0;
    int64_t r2 = 0;
    int64_t r25 = 0;
    int64_t r2v = 0;
    int64_t r3 = 0;
    int64_t r3h = 0;
    int64_t r3k = 0;
    int64_t r3v = 0;
    int64_t r4 = 0;
    int64_t r5 = 0;
    int64_t r7 = 0;
    int64_t r7k = 0;
    int64_t r8 = 0;
    int64_t sb = 0;
    int64_t sl = 0;
    int64_t sp0 = 0;
    int64_t sp4 = 0;
    int64_t sp8 = 0;
    int64_t tab = 0;
    int64_t x = 0;
    int64_t zero_path = 0;
    write_8((E + 0LL), read_8((M + 740LL)));
    diag = read_32(DIAG);
    write_32((E + 4LL), diag);
    h60 = read_16((HB + 96LL));
    h64 = read_16((HB + 100LL));
    sp0 = h60;
    sp4 = h64;
    write_16((E + 10LL), h64);
    sb = read_8((Q + 80LL));
    write_16((E + 8LL), h60);
    sl = read_16((DG + 238LL));
    write_8((E + 29LL), read_8((M + 670LL)));
    m44 = sxth(read_16((M + 68LL)));
    absm44 = abs64(m44);
    write_16((E + 14LL), read_16((M + 72LL)));
    write_16((E + 136LL), read_16((E + 64LL)));
    ip = read_16((M + 46LL));
    r1 = read_16((E + 70LL));
    fp = read_16((M + 36LL));
    write_32((E + 152LL), read_32((M + 478LL)));
    write_16((E + 138LL), r1);
    write_32((E + 160LL), 16777216LL);
    write_8((E + 1LL), sb);
    write_16((E + 22LL), ip);
    write_16((E + 24LL), sl);
    write_16((E + 26LL), fp);
    write_16((E + 12LL), (absm44 & 65535LL));
    write_32((E + 164LL), 0LL);
    write_32((E + 168LL), 4096LL);
    r5 = read_8((M + 57LL));
    r1 = read_8((E + 100LL));
    lr = read_16((M + 78LL));
    eq = ((r5 == r1));
    m4a = read_16((M + 74LL));
    sp8 = absm44;
    write_16((E + 156LL), 0LL);
    write_16((E + 16LL), lr);
    write_16((E + 18LL), m4a);
    write_16((E + 20LL), m4a);
    write_8((E + 28LL), r5);
    if (eq) {
        r4 = read_8((E + 101LL));
    } else {
        r4 = (((r5 != 0LL)) ? 1LL : 0LL);
        r2 = (((read_8((E + 90LL)) != 0LL)) ? 1LL : 0LL);
        r4 = (r4 & r2);
        write_8((E + 101LL), r4);
        write_8((E + 100LL), r5);
    }
    if ((((r4 == 1LL)) || ((read_8((E + 102LL)) == 1LL)))) {
        r2 = read_16((E + 96LL));
        r4 = ((read_16((E + 104LL)) + r2) & UINT32_MAX);
        r7 = read_8((E + 91LL));
        r5 = read_8((E + 103LL));
        write_16((E + 104LL), r4);
        if (((r5 >= r7))) {
            r5 = 0LL;
        } else {
            write_8((E + 103LL), (r5 + 1LL));
            r5 = 1LL;
        }
        write_8((E + 102LL), r5);
    } else {
        r4 = ((read_16((E + 94LL)) + read_16((E + 104LL))) & UINT32_MAX);
        write_8((E + 103LL), 0LL);
        write_16((E + 104LL), r4);
    }
    r5 = read_16((E + 92LL));
    r8 = (r4 & 65535LL);
    if (((r8 < r5))) {
        r4 = r5;
        write_16((E + 104LL), r5);
    } else {
        if (((r8 > 4096LL))) {
            r5 = 4096LL;
            r4 = r5;
            write_16((E + 104LL), r5);
        }
    }
    r3 = ((sb - 1LL) & UINT32_MAX);
    r0 = (m44 & UINT32_MAX);
    goto_common = 1LL;
    if (((r3 <= 10LL))) {
        if ((((r3 == 0LL)) || ((r3 == 1LL)))) {
            r1x = 0;
            if ((((r3 == 0LL)) && ((read_8((E + 34LL)) == 2LL)))) {
            } else {
                if (((r3 == 0LL))) {
                    if (((ip == 0LL))) {
                        if (((sl != 0LL))) {
                            write_16((E + 118LL), (((uint64_t)multiply64((multiply64(read_16((E + 40LL)), sp0) & UINT32_MAX), 1374389535LL) >> 37LL) & 65535LL));
                        }
                    } else {
                        write_16((E + 118LL), (((uint64_t)multiply64((multiply64(read_16((E + 42LL)), sp4) & UINT32_MAX), 1374389535LL) >> 37LL) & 65535LL));
                    }
                } else {
                    if (((ip == 0LL))) {
                        if (((sl != 0LL))) {
                            write_16((E + 118LL), (((uint64_t)multiply64((multiply64(read_16((E + 40LL)), sp0) & UINT32_MAX), 1374389535LL) >> 37LL) & 65535LL));
                        }
                    } else {
                        write_16((E + 118LL), (((uint64_t)multiply64((multiply64(read_16((E + 42LL)), sp4) & UINT32_MAX), 1374389535LL) >> 37LL) & 65535LL));
                    }
                }
                r4 = 0LL;
                write_32((E + 120LL), 0LL);
                chain_pi_reset((E + 148LL));
                if (((r3 == 0LL))) {
                    r0c8 = read_8((E + 200LL));
                    write_8((E + 201LL), 0LL);
                    write_16((E + 192LL), 0LL);
                    if (((r0c8 != 1LL))) {
                        write_8((E + 200LL), 1LL);
                        write_16((E + 206LL), read_16((E + 26LL)));
                    }
                    dd = read_32((E + 4LL));
                    if ((((dd == 0LL)) && ((sxtb(read_8((E + 0LL))) > 0LL)))) {
                    } else {
                        write_32((E + 140LL), 0LL);
                    }
                } else {
                    dd = read_32((E + 4LL));
                    write_8((E + 201LL), 0LL);
                    write_16((E + 192LL), 0LL);
                    if ((((dd == 0LL)) && ((sxtb(read_8((E + 0LL))) > 0LL)))) {
                    } else {
                        write_32((E + 140LL), 0LL);
                    }
                }
            }
        } else {
            if (((r3 == 3LL))) {
                write_16((E + 124LL), (r0 & 65535LL));
                r0ca = read_8((E + 202LL));
                write_16((E + 192LL), 512LL);
                write_8((E + 106LL), 0LL);
                write_32((E + 120LL), 0LL);
                write_8((E + 201LL), 1LL);
                if (((r0ca == 1LL))) {
                    write_8((E + 202LL), 0LL);
                    chain_pi_reset((E + 148LL));
                    fp = read_16((E + 26LL));
                }
                write_16((E + 204LL), fp);
            } else {
                if (((r3 == 4LL))) {
                    write_16((E + 118LL), read_16((E + 44LL)));
                    write_8((E + 106LL), 1LL);
                    write_16((E + 136LL), read_16((E + 66LL)));
                    r0c9 = read_8((E + 201LL));
                    write_8((E + 202LL), 1LL);
                    m4a_ = m4a;
                    if (((r0c9 == 1LL))) {
                        write_8((E + 201LL), 0LL);
                        chain_pi_reset((E + 148LL));
                        m4a_ = read_16((E + 18LL));
                    }
                    r0 = asr64(m4a_, 3LL);
                    if (((r0 > 74LL))) {
                        r0 = 0LL;
                        write_16((E + 118LL), 0LL);
                    } else {
                        r0 = read_16((E + 118LL));
                    }
                    write_32((E + 120LL), r0);
                    write_16((E + 124LL), m4a_);
                    write_16((E + 192LL), 4096LL);
                    write_8((E + 200LL), 0LL);
                } else {
                    if ((((r3 == 5LL)) || ((r3 == 6LL)))) {
                        e26 = read_8((E + 38LL));
                        if (((r3 == 5LL))) {
                            prod = (multiply64(read_16((E + 42LL)), sp4) & UINT32_MAX);
                            write_8((E + 106LL), read_8((E + 33LL)));
                            r25 = read_8((E + 37LL));
                        } else {
                            prod = (multiply64(read_16((E + 40LL)), sp0) & UINT32_MAX);
                            write_8((E + 106LL), read_8((E + 32LL)));
                        }
                        hi = ((uint64_t)multiply64(prod, 1374389535LL) >> 32LL);
                        write_16((E + 118LL), (asr64(hi, 5LL) & 65535LL));
                        q = (asr64(hi, 5LL) & 65535LL);
                        r5 = read_16((E + 46LL));
                        r1_ = read_16((E + 58LL));
                        if (((e26 != 1LL))) {
                            r5 = ((q + r1_) & UINT32_MAX);
                        }
                        write_32((E + 120LL), r5);
                        if (((e26 == 1LL))) {
                            lr = absm44;
                        }
                        write_16((E + 124LL), (lr & 65535LL));
                        if (((r3 == 5LL))) {
                            ipx = ip;
                            if ((((r4 & 65535LL) < ip))) {
                                ipx = r4;
                            }
                            if (((r25 != 1LL))) {
                                ipx = r4;
                            }
                            write_16((E + 192LL), (ipx & 65535LL));
                        } else {
                            write_16((E + 192LL), (r4 & 65535LL));
                        }
                        write_8((E + 200LL), 0LL);
                    } else {
                        if ((((r3 == 9LL)) || ((r3 == 10LL)))) {
                            write_8((E + 106LL), 0LL);
                            write_16((E + 192LL), 4096LL);
                        }
                    }
                }
            }
        }
    }
    r0 = read_32((E + 120LL));
    r2 = read_32((E + 140LL));
    r3 = ((r0 + shift_left(r0, 2LL)) & UINT32_MAX);
    r7 = sxth(read_16((E + 136LL)));
    r1 = (shift_left(r3, 1LL) & UINT32_MAX);
    r3 = ((shift_left(r3, 1LL) - r2) & UINT32_MAX);
    write_32((E + 128LL), r1);
    write_32((E + 132LL), r3);
    if (((signed32(r3) > r7))) {
        r1 = ((r2 + r7) & UINT32_MAX);
    } else {
        r7 = sxth(read_16((E + 138LL)));
        if (((signed32(r3) < r7))) {
            r1 = ((r2 + r7) & UINT32_MAX);
        }
    }
    r3h = ((uint64_t)multiply64(r1, 3435973837LL) >> 32LL);
    r7 = read_16((E + 192LL));
    r2 = read_16((E + 76LL));
    write_32((E + 140LL), r1);
    r1 = asr64(r3h, 3LL);
    write_16((E + 144LL), (r1 & 65535LL));
    if (((r7 < r2))) {
        write_16((E + 192LL), r2);
    }
    r2 = read_8((E + 1LL));
    if (((r2 != 4LL))) {
        if (((r2 == 5LL))) {
            if (((read_8((E + 34LL)) == 1LL))) {
                r2 = 2048LL;
                r3 = 8388608LL;
            } else {
                r2 = 1024LL;
                r3 = 4194304LL;
            }
        } else {
            r3 = 16777216LL;
            r2 = 4096LL;
        }
        write_16((E + 168LL), r2);
        write_32((E + 160LL), r3);
    }
    write_16((E + 148LL), (r1 & 65535LL));
    r3 = read_16((E + 124LL));
    n = read_16((E + 80LL));
    tab = read_32((E + 84LL));
    x = (r0 & 65535LL);
    write_16((E + 150LL), r3);
    r0 = chain_lut_model(x, n, tab);
    r2 = read_16((E + 88LL));
    r1 = read_8((E + 1LL));
    if (((r0 < r2))) {
        r0 = r2;
    }
    write_16((E + 154LL), (r0 & 65535LL));
    do_c2 = 0LL;
    if (((r1 == 4LL))) {
        write_32((E + 152LL), 134221824LL);
        r1 = sxth(read_16((E + 78LL)));
        write_16((E + 168LL), (r1 & 65535LL));
        write_16((E + 170LL), 0LL);
        r0 = (shift_left(r1, 12LL) & UINT32_MAX);
        do_c2 = 1LL;
    } else {
        if (((r1 == 5LL))) {
            if (((read_8((E + 35LL)) <= 6LL))) {
                write_32((E + 152LL), 525312LL);
                write_32((E + 168LL), 4096LL);
                r0 = 16777216LL;
                do_c2 = 1LL;
            }
        }
    }
    if (do_c2) {
        write_32((E + 160LL), r0);
        write_32((E + 164LL), 0LL);
        write_16((E + 156LL), 0LL);
    }
    chain_pi_step((E + 148LL));
    r0 = read_8((E + 1LL));
    r1 = read_8((E + 107LL));
    if (((r0 != r1))) {
        if ((((r1 == 1LL)) && ((((r0 - 5LL) & UINT32_MAX) <= 2LL)))) {
            r1 = read_16((M + 674LL));
            write_32((E + 180LL), (shift_left(r1, 12LL) & UINT32_MAX));
            r2 = read_16((E + 124LL));
            write_16((E + 188LL), r1);
            write_32((E + 140LL), (shift_left((r2 + shift_left(r2, 2LL)), 1LL) & UINT32_MAX));
        }
        write_8((E + 107LL), r0);
    }
    r1 = read_16((E + 188LL));
    if (((r0 == 4LL))) {
        write_16((E + 210LL), r1);
        goto616 = 1LL;
    } else {
        write_16((E + 194LL), r1);
        goto616 = 1LL;
        if (((r0 == 5LL))) {
            r0x = read_16((E + 204LL));
            if ((!((r1 > r0x)))) {
                write_16((E + 194LL), r0x);
                write_32((E + 180LL), (shift_left(r0x, 12LL) & UINT32_MAX));
                write_16((E + 188LL), r0x);
            }
        } else {
            if (((((r0 & 254LL) == 6LL)) && ((read_8((E + 29LL)) == 1LL)))) {
                write_8((E + 106LL), 1LL);
            }
        }
    }
    r0 = (read_8((E + 106LL)) | 2LL);
    if (((r0 == 3LL))) {
        if (((read_8((E + 38LL)) == 1LL))) {
            r1 = read_16((E + 192LL));
            r0 = read_16((E + 194LL));
            if (((r1 >= r0))) {
                write_16((E + 196LL), r0);
                write_16((E + 104LL), r0);
            } else {
                write_16((E + 196LL), r1);
                chain_pi_reset((E + 148LL));
                r0 = read_16((E + 196LL));
            }
        } else {
            r0 = read_16((E + 194LL));
            write_16((E + 196LL), r0);
        }
    } else {
        r0 = read_16((E + 192LL));
        write_16((E + 196LL), r0);
    }
    r1 = read_16((E + 206LL));
    r3 = sxth(read_16((E + 72LL)));
    r2 = sxth(((r0 - r1) & UINT32_MAX));
    write_32((E + 132LL), (r2 & UINT32_MAX));
    if (((r2 > r3))) {
        r0 = ((r3 + r1) & UINT32_MAX);
    } else {
        r3 = sxth(read_16((E + 74LL)));
        if (((r2 < r3))) {
            r0 = ((r3 + r1) & UINT32_MAX);
        }
    }
    r1 = read_16((E + 118LL));
    r2 = ((read_16((E + 58LL)) + r1) & UINT32_MAX);
    r3 = ((read_16((E + 62LL)) + r1) & UINT32_MAX);
    r7 = read_16((E + 56LL));
    write_16((E + 206LL), (r0 & 65535LL));
    write_16((E + 110LL), (r2 & 65535LL));
    write_16((E + 112LL), (r3 & 65535LL));
    zero_path = 0LL;
    if (((r1 > r7))) {
        r7 = ((r1 - r7) & UINT32_MAX);
    } else {
        if (((r1 == 0LL))) {
            zero_path = 1LL;
        }
    }
    if (zero_path) {
        write_32((E + 108LL), 0LL);
        write_16((E + 112LL), 0LL);
        write_8((E + 212LL), 0LL);
        r2 = chain_e754(r1);
    } else {
        r5 = read_16((E + 18LL));
        write_16((E + 108LL), (r7 & 65535LL));
        r7 = (r7 & 65535LL);
        write_8((E + 212LL), 0LL);
        if (((r5 < r7))) {
            r2 = 4096LL;
            write_16((E + 198LL), r2);
        } else {
            r2v = (r2 & 65535LL);
            if (((r5 < r2v))) {
                r3k = read_16((E + 60LL));
                r5 = ((r5 - r7) & UINT32_MAX);
                r3k = ((4096LL - r3k) & UINT32_MAX);
                r3k = (multiply64(r3k, r5) & UINT32_MAX);
                r2v = ((r2v - r7) & UINT32_MAX);
                r2 = ((4096LL - udiv(r3k, r2v)) & UINT32_MAX);
                write_16((E + 198LL), (r2 & 65535LL));
            } else {
                r3v = (r3 & 65535LL);
                if (((r5 < r3v))) {
                    r7k = read_16((E + 60LL));
                    r5 = ((r5 - r2v) & UINT32_MAX);
                    r5 = (multiply64(r7k, r5) & UINT32_MAX);
                    r2v = ((r3v - r2v) & UINT32_MAX);
                    r2 = ((r7k - udiv(r5, r2v)) & UINT32_MAX);
                    write_16((E + 198LL), (r2 & 65535LL));
                } else {
                    r2 = chain_e754(r1);
                }
            }
        }
    }
    r3 = read_8((E + 106LL));
    if (((r3 < 2LL))) {
        r2 = 4096LL;
    }
    write_16((E + 208LL), (r2 & 65535LL));
    write_16((M + 82LL), r1);
    write_16((M + 674LL), (r0 & 65535LL));
    write_16((M + 676LL), (r2 & 65535LL));
    (void)goto616;
    (void)goto_common;
    (void)r1x;
    (void)sp8;
}

/* Pinned tq02b_bde8.py:49; register-width writes remain explicit. */
static void chain_bde8_model(void)
{
    int64_t a = 0;
    int64_t cnt = 0;
    int64_t code = 0;
    int64_t diag = 0;
    int64_t fatal = 0;
    int64_t g1 = 0;
    int64_t g2 = 0;
    int64_t ip = 0;
    int64_t lab = 0;
    int64_t ptr = 0;
    int64_t q1c = 0;
    int64_t q3 = 0;
    int64_t q4 = 0;
    int64_t q5 = 0;
    int64_t q8_zero = 0;
    int64_t r0 = 0;
    int64_t r0s = 0;
    int64_t r2 = 0;
    int64_t r3 = 0;
    int64_t r3p = 0;
    int64_t r4 = 0;
    r0 = asr64(read_16((M + 20LL)), 1LL);
    r4 = ((uint64_t)multiply64(r0, 3020636341LL) >> 32LL);
    write_8((Q + 1LL), read_8((M + 9LL)));
    write_8((Q + 0LL), read_8((M + 10LL)));
    write_16((Q + 6LL), read_16((M + 708LL)));
    write_16((Q + 8LL), read_16((M + 46LL)));
    write_16((Q + 10LL), read_16((DG + 238LL)));
    write_16((Q + 12LL), read_16((536875000LL + 92LL)));
    write_16((Q + 14LL), read_16((M + 678LL)));
    write_8((Q + 2LL), read_8((M + 740LL)));
    write_16((Q + 18LL), read_16((M + 68LL)));
    write_16((Q + 22LL), read_16((M + 78LL)));
    write_16((Q + 20LL), read_16((M + 74LL)));
    write_16((Q + 24LL), read_16(read_32((M + 116LL))));
    write_16((Q + 26LL), read_16((536874624LL + 32LL)));
    write_8((Q + 28LL), 0LL);
    write_8((Q + 120LL), 0LL);
    write_8((Q + 118LL), 0LL);
    write_16((Q + 42LL), asr64(r4, 6LL));
    r2 = read_16((E + 210LL));
    diag = read_32(DIAG);
    write_16((Q + 3LL), 256LL);
    write_16((Q + 30LL), r2);
    write_8((Q + 40LL), read_8((M + 11LL)));
    write_8((Q + 5LL), 1LL);
    write_32((Q + 56LL), diag);
    write_8((Q + 44LL), read_8((M + 92LL)));
    fatal = (diag & 6192LL);
    write_32((Q + 84LL), fatal);
    lab = LABEL_BEEA;
    if (((diag != 0LL))) {
        if (((fatal == 0LL))) {
            if (((read_8((Q + 80LL)) == 10LL))) {
                r2 = read_8((M + 11LL));
                lab = LABEL_C132;
            } else {
                write_8((Q + 80LL), 1LL);
                lab = LABEL_BF4A;
            }
        } else {
            write_8((Q + 80LL), 2LL);
            write_8(read_32((Q + 32LL)), 0LL);
            write_8((Q + 121LL), 0LL);
            r2 = read_32(DIAG);
            write_8((Q + 123LL), 1LL);
            write_8((Q + 122LL), ((asr64(r2, 5LL) & 1LL) ? 255LL : 0LL));
        }
    }
    r0 = 0LL;
    r3 = 0LL;
    ip = 0LL;
    while (1LL) {
        if (((lab == LABEL_BEEA))) {
            r0 = read_8((Q + 80LL));
            if ((((r0 > 11LL)) || (((r0 == 3LL) || (r0 == 8LL) || (r0 == 9LL))))) {
                lab = LABEL_BFA8;
            } else {
                lab = state_label(r0);
            }
        } else {
            if (((lab == LABEL_BFA8))) {
                write_32((Q + 108LL), 0LL);
                write_32((Q + 112LL), 0LL);
                write_16((Q + 116LL), 900LL);
                lab = LABEL_BFB6;
            } else {
                if (((lab == LABEL_BFB6))) {
                    r0 = 2LL;
                    lab = LABEL_C39A;
                } else {
                    if (((lab == LABEL_BF0E))) {
                        write_16((Q + 116LL), 900LL);
                        r2 = read_32((M + 728LL));
                        write_8((Q + 106LL), 0LL);
                        write_8((Q + 5LL), 0LL);
                        write_32((Q + 90LL), 0LL);
                        write_16((Q + 96LL), 0LL);
                        write_32((Q + 108LL), 0LL);
                        write_32((Q + 112LL), 0LL);
                        write_8(r2, 0LL);
                        lab = (((read_32((Q + 84LL)) != 0LL)) ? LABEL_C39E : LABEL_C36C);
                    } else {
                        if (((lab == LABEL_BF4A))) {
                            write_8((Q + 3LL), 1LL);
                            write_16((Q + 116LL), 900LL);
                            r0 = read_32((M + 728LL));
                            write_16((Q + 90LL), 0LL);
                            write_16((Q + 96LL), 0LL);
                            write_32((Q + 112LL), 0LL);
                            write_8(r0, 0LL);
                            r0 = sxth(read_16((Q + 18LL)));
                            write_8((Q + 88LL), 0LL);
                            if (((r0 <= 0LL))) {
                                lab = LABEL_C1B6;
                            } else {
                                if (((read_8((Q + 61LL)) == 1LL))) {
                                    write_16((Q + 92LL), 0LL);
                                    write_8((Q + 106LL), 30LL);
                                }
                                r2 = read_16((Q + 102LL));
                                r0 = read_16((Q + 12LL));
                                if (((r2 > r0))) {
                                    write_16((Q + 102LL), r0);
                                }
                                r2 = read_16((Q + 104LL));
                                r0 = read_16((Q + 14LL));
                                if (((r2 > r0))) {
                                    write_16((Q + 104LL), r0);
                                }
                                lab = LABEL_C1CC;
                            }
                        } else {
                            if (((lab == LABEL_C1B6))) {
                                write_16((Q + 92LL), 0LL);
                                write_8((Q + 106LL), 0LL);
                                write_8((Q + 5LL), 0LL);
                                write_16((Q + 102LL), 4096LL);
                                write_16((Q + 104LL), 4096LL);
                                lab = LABEL_C1CC;
                            } else {
                                if (((lab == LABEL_C1CC))) {
                                    if ((((read_32((Q + 56LL)) == 0LL)) && ((read_8(read_32((Q + 36LL))) == 0LL)))) {
                                        lab = LABEL_C288;
                                    } else {
                                        if (((read_32((Q + 84LL)) != 0LL))) {
                                            lab = LABEL_C39E;
                                        } else {
                                            if (((read_8((Q + 44LL)) != 1LL))) {
                                                lab = LABEL_C390;
                                            } else {
                                                if (((read_8((Q + 45LL)) == 21LL))) {
                                                    lab = LABEL_C398;
                                                } else {
                                                    lab = LABEL_C39E;
                                                }
                                            }
                                        }
                                    }
                                } else {
                                    if (((lab == LABEL_C288))) {
                                        r0 = sxtb(read_8((Q + 2LL)));
                                        r2 = read_16((Q + 24LL));
                                        if (((((r0 + 1LL) != 0LL)) || ((r2 > 600LL)))) {
                                            lab = LABEL_C2A6;
                                        } else {
                                            if (((read_8((Q + 1LL)) == 0LL))) {
                                                lab = LABEL_C390;
                                            } else {
                                                r0 = 4LL;
                                                lab = LABEL_C38C;
                                            }
                                        }
                                    } else {
                                        if (((lab == LABEL_C2A6))) {
                                            if (((r0 < 1LL))) {
                                                lab = LABEL_C370;
                                            } else {
                                                if (((read_16((Q + 8LL)) == 0LL))) {
                                                    lab = LABEL_C386;
                                                } else {
                                                    if (((r2 < read_16((Q + 70LL))))) {
                                                        lab = LABEL_C386;
                                                    } else {
                                                        r0 = 6LL;
                                                        lab = LABEL_C38C;
                                                    }
                                                }
                                            }
                                        } else {
                                            if (((lab == LABEL_C370))) {
                                                if (((read_8((Q + 44LL)) != 1LL))) {
                                                    lab = LABEL_C390;
                                                } else {
                                                    r0 = (((read_8((Q + 45LL)) == 21LL)) ? 10LL : 11LL);
                                                    lab = LABEL_C38C;
                                                }
                                            } else {
                                                if (((lab == LABEL_C386))) {
                                                    r0 = read_16((Q + 10LL));
                                                    if (((r0 == 0LL))) {
                                                        lab = LABEL_C390;
                                                    } else {
                                                        r0 = 7LL;
                                                        lab = LABEL_C38C;
                                                    }
                                                } else {
                                                    if (((lab == LABEL_BFBA))) {
                                                        write_8((Q + 106LL), 2LL);
                                                        write_8(read_32((M + 728LL)), 0LL);
                                                        write_16((Q + 100LL), read_16((Q + 72LL)));
                                                        r2 = read_16((Q + 30LL));
                                                        write_16((Q + 116LL), 900LL);
                                                        write_16((Q + 90LL), r2);
                                                        r2 = read_8((Q + 1LL));
                                                        write_8((Q + 118LL), 1LL);
                                                        write_8((Q + 120LL), 1LL);
                                                        if (((r2 == 0LL))) {
                                                            write_8((Q + 88LL), 1LL);
                                                        }
                                                        r3 = sxtb(read_8((Q + 2LL)));
                                                        if (((((r3 + 1LL) != 0LL)) || ((read_16((Q + 24LL)) > 600LL)) || ((read_8((Q + 0LL)) != 1LL)))) {
                                                            lab = LABEL_C27C;
                                                        } else {
                                                            if (((r2 == 0LL))) {
                                                                lab = LABEL_C39E;
                                                            } else {
                                                                if (((read_8((Q + 88LL)) != 1LL))) {
                                                                    lab = LABEL_C39E;
                                                                } else {
                                                                    write_8((Q + 88LL), 0LL);
                                                                    r0 = 5LL;
                                                                    lab = LABEL_C39A;
                                                                }
                                                            }
                                                        }
                                                    } else {
                                                        if (((lab == LABEL_C27C))) {
                                                            lab = (((r3 >= 0LL)) ? LABEL_C36C : LABEL_C39E);
                                                        } else {
                                                            if (((lab == LABEL_C026))) {
                                                                write_8((Q + 106LL), 2LL);
                                                                r0 = read_32((M + 728LL));
                                                                write_8(r0, 1LL);
                                                                write_16((Q + 116LL), 900LL);
                                                                r0 = read_16((Q + 64LL));
                                                                r3 = read_16((Q + 68LL));
                                                                r4 = sxth(read_16((Q + 6LL)));
                                                                write_16((Q + 100LL), r0);
                                                                if (((r3 > r4))) {
                                                                    r3 = r4;
                                                                }
                                                                r0 = read_8((Q + 2LL));
                                                                write_8((Q + 118LL), 1LL);
                                                                write_16((Q + 90LL), r3);
                                                                r2 = 1LL;
                                                                if ((((r0 != 255LL)) || ((read_8((Q + 63LL)) != 1LL)))) {
                                                                    lab = LABEL_C282;
                                                                } else {
                                                                    if ((((read_8((Q + 1LL)) | read_8((M + 102LL))) != 0LL))) {
                                                                        lab = LABEL_C39E;
                                                                    } else {
                                                                        r2 = sxth(read_16((Q + 18LL)));
                                                                        write_16((Q + 90LL), 0LL);
                                                                        if (((r2 > 0LL))) {
                                                                            lab = LABEL_C39E;
                                                                        } else {
                                                                            write_8((Q + 88LL), 0LL);
                                                                            r0 = 4LL;
                                                                            lab = LABEL_C39A;
                                                                        }
                                                                    }
                                                                }
                                                            } else {
                                                                if (((lab == LABEL_C282))) {
                                                                    write_8((Q + 80LL), (r2 & 255LL));
                                                                    lab = LABEL_C39E;
                                                                } else {
                                                                    if (((lab == LABEL_C090))) {
                                                                        write_8((Q + 106LL), 2LL);
                                                                        r2 = read_16((Q + 78LL));
                                                                        write_8((Q + 28LL), 1LL);
                                                                        write_16((Q + 116LL), r2);
                                                                        r0 = read_16((Q + 72LL));
                                                                        r2 = read_16((Q + 76LL));
                                                                        r3 = sxth(read_16((Q + 6LL)));
                                                                        write_16((Q + 100LL), r0);
                                                                        if (((r2 > r3))) {
                                                                            r2 = r3;
                                                                        }
                                                                        r0 = read_16((Q + 8LL));
                                                                        r2 = (multiply64(r0, sxth(r2)) & UINT32_MAX);
                                                                        write_16((Q + 90LL), (asr64(r2, 12LL) & 65535LL));
                                                                        if (((r0 != 0LL))) {
                                                                            lab = LABEL_C122;
                                                                        } else {
                                                                            r0 = read_16((Q + 10LL));
                                                                            if (((r0 == 0LL))) {
                                                                                lab = LABEL_C36C;
                                                                            } else {
                                                                                r0 = 7LL;
                                                                                lab = LABEL_C39A;
                                                                            }
                                                                        }
                                                                    } else {
                                                                        if (((lab == LABEL_C0D2))) {
                                                                            write_8((Q + 106LL), 2LL);
                                                                            r2 = read_16((Q + 78LL));
                                                                            write_8((Q + 28LL), 1LL);
                                                                            r0 = read_16((Q + 72LL));
                                                                            write_16((Q + 116LL), r2);
                                                                            r2 = read_16((Q + 76LL));
                                                                            r3 = sxth(read_16((Q + 6LL)));
                                                                            write_16((Q + 100LL), r0);
                                                                            r0 = read_16((Q + 8LL));
                                                                            if (((r2 > r3))) {
                                                                                r2 = r3;
                                                                            }
                                                                            q8_zero = ((r0 == 0LL));
                                                                            r0 = read_16((Q + 10LL));
                                                                            r2 = (multiply64(r0, sxth(r2)) & UINT32_MAX);
                                                                            write_16((Q + 90LL), (asr64(r2, 12LL) & 65535LL));
                                                                            if ((!q8_zero)) {
                                                                                r2 = read_16((Q + 24LL));
                                                                                r3 = read_16((Q + 70LL));
                                                                                if (((r2 >= r3))) {
                                                                                    r0 = 6LL;
                                                                                    lab = LABEL_C39A;
                                                                                    continue;
                                                                                }
                                                                            }
                                                                            if (((r0 == 0LL))) {
                                                                                lab = LABEL_C36C;
                                                                            } else {
                                                                                lab = LABEL_C122;
                                                                            }
                                                                        } else {
                                                                            if (((lab == LABEL_C122))) {
                                                                                lab = (((sxtb(read_8((Q + 2LL))) > 0LL)) ? LABEL_C39E : LABEL_C36C);
                                                                            } else {
                                                                                if (((lab == LABEL_C12E))) {
                                                                                    r2 = read_8((Q + 40LL));
                                                                                    lab = LABEL_C132;
                                                                                } else {
                                                                                    if (((lab == LABEL_C132))) {
                                                                                        write_8((Q + 106LL), 21LL);
                                                                                        if (((r2 != 1LL))) {
                                                                                            r0 = 1LL;
                                                                                            ip = 2LL;
                                                                                            r3 = 0LL;
                                                                                            write_8((Q + 123LL), 1LL);
                                                                                        } else {
                                                                                            r3 = read_16((Q + 42LL));
                                                                                            if (((r3 < 181LL))) {
                                                                                                ip = 1LL;
                                                                                                write_8((Q + 123LL), 1LL);
                                                                                            } else {
                                                                                                write_8((Q + 123LL), 0LL);
                                                                                                r3 = ((104LL - r3) & UINT32_MAX);
                                                                                                ip = 1LL;
                                                                                            }
                                                                                        }
                                                                                        write_8((Q + 122LL), (r3 & 255LL));
                                                                                        write_8((Q + 121LL), (ip & 255LL));
                                                                                        if ((((read_8((Q + 60LL)) == 1LL)) && ((r2 == 1LL)))) {
                                                                                            if (((read_8((Q + 41LL)) == 0LL))) {
                                                                                                a = (P + 8764LL);
                                                                                                cnt = ((read_16(a) + 1LL) & 65535LL);
                                                                                                r4 = read_16((M + 20LL));
                                                                                                write_16(a, cnt);
                                                                                                write_16((a + 4LL), read_16((M + 13LL)));
                                                                                                write_32((a + 6LL), read_32((M + 15LL)));
                                                                                                ptr = read_32((M + 188LL));
                                                                                                write_16((a + 2LL), r4);
                                                                                                write_8(ptr, ((read_8(ptr) + 1LL) & 255LL));
                                                                                                write_8((Q + 41LL), 1LL);
                                                                                                write_8(read_32((Q + 36LL)), 0LL);
                                                                                                write_8(read_32((Q + 32LL)), 0LL);
                                                                                                lab = LABEL_C266;
                                                                                            } else {
                                                                                                lab = LABEL_C212;
                                                                                            }
                                                                                        } else {
                                                                                            lab = LABEL_C212;
                                                                                        }
                                                                                        if (((lab == LABEL_C212))) {
                                                                                            if (((r2 == 0LL))) {
                                                                                                write_8((Q + 41LL), 0LL);
                                                                                            }
                                                                                            lab = LABEL_C266;
                                                                                        }
                                                                                    } else {
                                                                                        if (((lab == LABEL_C266))) {
                                                                                            if (((read_8(read_32((Q + 32LL))) != 0LL))) {
                                                                                                lab = LABEL_C39E;
                                                                                            } else {
                                                                                                if (((read_8(read_32((Q + 36LL))) != 0LL))) {
                                                                                                    lab = LABEL_C39E;
                                                                                                } else {
                                                                                                    lab = LABEL_C362;
                                                                                                }
                                                                                            }
                                                                                        } else {
                                                                                            if (((lab == LABEL_C160))) {
                                                                                                r0 = read_8((Q + 45LL));
                                                                                                r2 = read_16((Q + 72LL));
                                                                                                write_16((Q + 100LL), r2);
                                                                                                code = r0;
                                                                                                if ((((code == 0LL) || (code == 1LL) || (code == 2LL) || (code == 3LL) || (code == 10LL) || (code == 11LL) || (code == 12LL) || (code == 13LL) || (code == 21LL) || (code == 22LL) || (code == 23LL)))) {
                                                                                                    lab = service_label(code);
                                                                                                } else {
                                                                                                    lab = LABEL_S11_C2D8;
                                                                                                }
                                                                                            } else {
                                                                                                if (((lab == LABEL_S11_C1A6))) {
                                                                                                    write_8((Q + 106LL), 0LL);
                                                                                                    write_16((Q + 90LL), 0LL);
                                                                                                    write_16((Q + 96LL), 0LL);
                                                                                                    write_32((Q + 112LL), 0LL);
                                                                                                    write_16((Q + 116LL), 900LL);
                                                                                                    lab = LABEL_C362;
                                                                                                } else {
                                                                                                    if (((lab == LABEL_S11_C2BC))) {
                                                                                                        r2 = read_16((Q + 50LL));
                                                                                                        write_8((Q + 106LL), 2LL);
                                                                                                        write_16((Q + 90LL), r2);
                                                                                                        write_16((Q + 116LL), 900LL);
                                                                                                        lab = LABEL_C362;
                                                                                                    } else {
                                                                                                        if (((lab == LABEL_S11_C2CA))) {
                                                                                                            r2 = read_16((Q + 48LL));
                                                                                                            write_8((Q + 106LL), 3LL);
                                                                                                            write_16((Q + 96LL), r2);
                                                                                                            lab = LABEL_C362;
                                                                                                        } else {
                                                                                                            if (((lab == LABEL_S11_C2D8))) {
                                                                                                                write_8((Q + 106LL), 0LL);
                                                                                                                write_32((Q + 108LL), 0LL);
                                                                                                                write_32((Q + 112LL), 0LL);
                                                                                                                write_16((Q + 116LL), 900LL);
                                                                                                                lab = LABEL_C362;
                                                                                                            } else {
                                                                                                                if (((lab == LABEL_S11_C2E8))) {
                                                                                                                    write_8((Q + 4LL), 0LL);
                                                                                                                    write_8((Q + 106LL), 10LL);
                                                                                                                    write_16((Q + 96LL), read_16((Q + 48LL)));
                                                                                                                    write_16((Q + 112LL), read_16((Q + 52LL)));
                                                                                                                    lab = LABEL_C362;
                                                                                                                } else {
                                                                                                                    if (((lab == LABEL_S11_C2FC))) {
                                                                                                                        write_8((Q + 4LL), 0LL);
                                                                                                                        write_8((Q + 106LL), 11LL);
                                                                                                                        write_16((Q + 90LL), read_16((Q + 50LL)));
                                                                                                                        write_16((Q + 112LL), read_16((Q + 52LL)));
                                                                                                                        lab = LABEL_C362;
                                                                                                                    } else {
                                                                                                                        if (((lab == LABEL_S11_C314))) {
                                                                                                                            write_8((Q + 4LL), 0LL);
                                                                                                                            write_8((Q + 106LL), 12LL);
                                                                                                                            write_16((Q + 96LL), read_16((Q + 48LL)));
                                                                                                                            write_16((Q + 114LL), read_16((Q + 54LL)));
                                                                                                                            lab = LABEL_C362;
                                                                                                                        } else {
                                                                                                                            if (((lab == LABEL_S11_C328))) {
                                                                                                                                write_8((Q + 4LL), 0LL);
                                                                                                                                write_8((Q + 106LL), 13LL);
                                                                                                                                write_16((Q + 90LL), read_16((Q + 50LL)));
                                                                                                                                write_16((Q + 114LL), read_16((Q + 54LL)));
                                                                                                                                lab = LABEL_C362;
                                                                                                                            } else {
                                                                                                                                if (((lab == LABEL_S11_C340))) {
                                                                                                                                    write_8((Q + 106LL), 0LL);
                                                                                                                                    write_16((Q + 90LL), 0LL);
                                                                                                                                    lab = LABEL_C362;
                                                                                                                                } else {
                                                                                                                                    if (((lab == LABEL_S11_C34C))) {
                                                                                                                                        write_8((Q + 106LL), 23LL);
                                                                                                                                        r0 = read_16((Q + 50LL));
                                                                                                                                        r2 = read_16((Q + 54LL));
                                                                                                                                        write_16((Q + 90LL), r0);
                                                                                                                                        write_16((Q + 116LL), ((r2 + 900LL) & 65535LL));
                                                                                                                                        lab = LABEL_C362;
                                                                                                                                    } else {
                                                                                                                                        if (((lab == LABEL_S11_C362))) {
                                                                                                                                            lab = LABEL_C362;
                                                                                                                                        } else {
                                                                                                                                            if (((lab == LABEL_C362))) {
                                                                                                                                                if (((read_8((Q + 44LL)) == 0LL))) {
                                                                                                                                                    lab = LABEL_C36C;
                                                                                                                                                } else {
                                                                                                                                                    if (((read_8((Q + 2LL)) == 0LL))) {
                                                                                                                                                        lab = LABEL_C39E;
                                                                                                                                                    } else {
                                                                                                                                                        lab = LABEL_C36C;
                                                                                                                                                    }
                                                                                                                                                }
                                                                                                                                            } else {
                                                                                                                                                if (((lab == LABEL_C36C))) {
                                                                                                                                                    r0 = 1LL;
                                                                                                                                                    lab = LABEL_C39A;
                                                                                                                                                } else {
                                                                                                                                                    if (((lab == LABEL_C38C))) {
                                                                                                                                                        write_8((Q + 80LL), (r0 & 255LL));
                                                                                                                                                        lab = LABEL_C390;
                                                                                                                                                    } else {
                                                                                                                                                        if (((lab == LABEL_C390))) {
                                                                                                                                                            if (((read_8(read_32((Q + 32LL))) != 1LL))) {
                                                                                                                                                                lab = LABEL_C39E;
                                                                                                                                                            } else {
                                                                                                                                                                lab = LABEL_C398;
                                                                                                                                                            }
                                                                                                                                                        } else {
                                                                                                                                                            if (((lab == LABEL_C398))) {
                                                                                                                                                                r0 = 10LL;
                                                                                                                                                                lab = LABEL_C39A;
                                                                                                                                                            } else {
                                                                                                                                                                if (((lab == LABEL_C39A))) {
                                                                                                                                                                    write_8((Q + 80LL), (r0 & 255LL));
                                                                                                                                                                    lab = LABEL_C39E;
                                                                                                                                                                } else {
                                                                                                                                                                    if (((lab == LABEL_C39E))) {
                                                                                                                                                                        break;
                                                                                                                                                                    } else {
                                                                                                                                                                        return; /* unreachable label outside the closed dispatch table */
                                                                                                                                                                    }
                                                                                                                                                                }
                                                                                                                                                            }
                                                                                                                                                        }
                                                                                                                                                    }
                                                                                                                                                }
                                                                                                                                            }
                                                                                                                                        }
                                                                                                                                    }
                                                                                                                                }
                                                                                                                            }
                                                                                                                        }
                                                                                                                    }
                                                                                                                }
                                                                                                            }
                                                                                                        }
                                                                                                    }
                                                                                                }
                                                                                            }
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    r3 = read_16((Q + 90LL));
    r0 = read_16((Q + 92LL));
    r4 = sxth(read_16((Q + 100LL)));
    r2 = ((r3 - r0) & UINT32_MAX);
    ip = read_16((Q + 96LL));
    write_16((Q + 98LL), r2);
    r2 = sxth(r2);
    write_16((Q + 110LL), ip);
    if (((r2 > r4))) {
        r3 = ((r4 + r0) & UINT32_MAX);
    } else {
        r4 = sxth(read_16((Q + 74LL)));
        if (((r2 < (-r4)))) {
            r3 = ((r0 - r4) & UINT32_MAX);
        }
    }
    q3 = read_8((Q + 3LL));
    write_16((Q + 92LL), (r3 & 65535LL));
    q4 = read_8((Q + 4LL));
    q5 = read_8((Q + 5LL));
    q1c = read_8((Q + 28LL));
    g1 = (((q3 == 1LL)) ? read_16((Q + 102LL)) : read_16((Q + 12LL)));
    r0 = (multiply64(g1, sxth((r3 & 65535LL))) & UINT32_MAX);
    write_16((Q + 94LL), (asr64(r0, 12LL) & 65535LL));
    r0s = sxth((asr64(r0, 12LL) & 65535LL));
    g2 = read_16((Q + (((q3 == 1LL)) ? 104LL : 14LL)));
    r4 = read_16((Q + 26LL));
    r0 = (multiply64(g2, r0s) & UINT32_MAX);
    write_16((Q + 94LL), (asr64(r0, 12LL) & 65535LL));
    r2 = (multiply64(r4, q1c) & UINT32_MAX);
    if (((q4 == 1LL))) {
        r3 = ((r2 + asr64(r0, 12LL)) & UINT32_MAX);
    }
    r0 = (multiply64(r3, q5) & UINT32_MAX);
    write_16((M + 682LL), (r0 & 65535LL));
    write_16((M + 668LL), 0LL);
    write_8((M + 664LL), read_8((Q + 106LL)));
    write_16((M + 684LL), read_16((Q + 112LL)));
    write_16((Q + 108LL), (r3 & 65535LL));
    write_16((M + 686LL), read_16((Q + 114LL)));
    write_16((M + 688LL), read_16((Q + 116LL)));
    r0 = read_8((Q + 118LL));
    r2 = read_8((Q + 121LL));
    r3p = read_32((M + 752LL));
    write_16((M + 680LL), ip);
    write_8((M + 665LL), r0);
    write_8(r3p, r2);
    write_8(read_32((M + 756LL)), read_8((Q + 122LL)));
    write_8(read_32((M + 760LL)), read_8((Q + 123LL)));
}

static uint8_t level_accel[10] = {1,4,4,5,5,6,6,7,8,8};
static uint16_t level_ratio[10] = {1,45,95,155,215,260,310,370,525,525};

static void g53_chain_apply_levels(void)
{
    uint8_t slot;
    write_16(0x20001210u,163);
    write_16(0x20001228u,1);
    for(slot=1;slot<10;slot++) {
        uint8_t n=level_accel[slot];
        if(n<1) n=1;
        if(n>8) n=8;
        write_16(0x20001210u+2u*slot,40960u/(250u-(uint16_t)(n-1u)*32u));
        write_16(0x20001228u+2u*slot,level_ratio[slot]>1000 ? 1000 : level_ratio[slot]);
    }
}

void g53_chain_set_levels(const uint8_t accel[10], const uint16_t ratio[10])
{
    uint8_t slot;
    for(slot=1;slot<10;slot++) {
        level_accel[slot]=accel[slot];
        level_ratio[slot]=ratio[slot];
    }
    g53_chain_apply_levels();
}

uint16_t g53_chain_rise_step(uint8_t slot)
{
    return slot<10 ? (uint16_t)read_16(0x20001210u+2u*slot) : 0;
}

uint16_t g53_chain_ratio(uint8_t slot)
{
    return slot<10 ? (uint16_t)read_16(0x20001228u+2u*slot) : 0;
}

void g53_chain_reset(void)
{
    memset(state_0,0,sizeof(state_0));
    memset(state_1,0,sizeof(state_1));
    memset(state_2,0,sizeof(state_2));
    memset(state_3,0,sizeof(state_3));
    memset(state_4,0,sizeof(state_4));
    memset(state_5,0,sizeof(state_5));
    memset(state_6,0,sizeof(state_6));
    memset(state_7,0,sizeof(state_7));
    chain_tick=0; fast_phase=0; supervisor_phase=0;
    write_8(0x20000f8cu, 48);
    write_8(0x20000f8du, 55);
    write_8(0x20000f8fu, 32);
    write_8(0x20000f90u, 49);
    write_8(0x20000f91u, 55);
    write_8(0x20000f93u, 32);
    write_8(0x20000fa8u, 1);
    write_8(0x20000fabu, 1);
    write_8(0x20000facu, 10);
    write_8(0x20000fb0u, 100);
    write_8(0x20000fb1u, 25);
    write_8(0x20000fb2u, 50);
    write_8(0x20000fb4u, 50);
    write_8(0x20000fb6u, 50);
    write_8(0x20000fb8u, 100);
    write_8(0x20000fb9u, 25);
    write_8(0x20000fbau, 132);
    write_8(0x20000fbbu, 3);
    write_8(0x20000fbcu, 1);
    write_8(0x20000fd3u, 16);
    write_8(0x20000fd5u, 16);
    write_8(0x20001208u, 1);
    write_8(0x20001209u, 1);
    write_8(0x2000120cu, 196);
    write_8(0x2000120du, 9);
    write_8(0x2000120eu, 4);
    write_8(0x20001210u, 163);
    write_8(0x20001212u, 39);
    write_8(0x20001213u, 6);
    write_8(0x20001214u, 39);
    write_8(0x20001215u, 6);
    write_8(0x20001216u, 39);
    write_8(0x20001217u, 6);
    write_8(0x20001218u, 39);
    write_8(0x20001219u, 6);
    write_8(0x2000121au, 39);
    write_8(0x2000121bu, 6);
    write_8(0x2000121cu, 39);
    write_8(0x2000121du, 6);
    write_8(0x2000121eu, 39);
    write_8(0x2000121fu, 6);
    write_8(0x20001220u, 39);
    write_8(0x20001221u, 6);
    write_8(0x20001222u, 39);
    write_8(0x20001223u, 6);
    write_8(0x20001226u, 103);
    write_8(0x20001227u, 254);
    write_8(0x20001228u, 1);
    write_8(0x2000122au, 45);
    write_8(0x2000122cu, 95);
    write_8(0x2000122eu, 155);
    write_8(0x20001230u, 215);
    write_8(0x20001232u, 4);
    write_8(0x20001233u, 1);
    write_8(0x20001234u, 54);
    write_8(0x20001235u, 1);
    write_8(0x20001236u, 114);
    write_8(0x20001237u, 1);
    write_8(0x20001238u, 13);
    write_8(0x20001239u, 2);
    write_8(0x2000123au, 13);
    write_8(0x2000123bu, 2);
    write_8(0x2000123eu, 5);
    write_8(0x2000123fu, 5);
    g53_chain_apply_levels();
    write_8(0x20001256u, 2);
    write_8(0x20001262u, 10);
    write_8(0x20001270u, 232);
    write_8(0x20001271u, 3);
    write_8(0x20001272u, 232);
    write_8(0x20001273u, 3);
    write_8(0x20001274u, 232);
    write_8(0x20001275u, 3);
    write_8(0x20001276u, 232);
    write_8(0x20001277u, 3);
    write_8(0x20001278u, 232);
    write_8(0x20001279u, 3);
    write_8(0x2000127au, 232);
    write_8(0x2000127bu, 3);
    write_8(0x2000127cu, 232);
    write_8(0x2000127du, 3);
    write_8(0x2000127eu, 232);
    write_8(0x2000127fu, 3);
    write_8(0x20001280u, 232);
    write_8(0x20001281u, 3);
    write_8(0x20001282u, 232);
    write_8(0x20001283u, 3);
    write_8(0x20001286u, 1);
    write_8(0x2000128cu, 134);
    write_8(0x2000128du, 157);
    write_8(0x2000128eu, 2);
    write_8(0x2000128fu, 8);
    write_8(0x2000133cu, 100);
    write_8(0x20001340u, 100);
    write_8(0x20001364u, 1);
    write_8(0x20001365u, 1);
    write_8(0x20001367u, 3);
    write_8(0x20001369u, 1);
    write_8(0x2000136cu, 148);
    write_8(0x2000136du, 17);
    write_8(0x2000136eu, 148);
    write_8(0x2000136fu, 17);
    write_8(0x20001370u, 244);
    write_8(0x20001371u, 1);
    write_8(0x2000137cu, 200);
    write_8(0x2000137eu, 150);
    write_8(0x20001382u, 200);
    write_8(0x20001384u, 90);
    write_8(0x20001385u, 10);
    write_8(0x20001386u, 100);
    write_8(0x2000138au, 166);
    write_8(0x2000138bu, 245);
    write_8(0x2000138du, 1);
    write_8(0x2000138eu, 196);
    write_8(0x2000138fu, 255);
    write_8(0x20001392u, 100);
    write_8(0x20001393u, 25);
    write_8(0x20001394u, 2);
    write_8(0x20001399u, 48);
    write_8(0x2000139bu, 32);
    write_8(0x2000139cu, 16);
    write_8(0x2000139eu, 1);
    write_8(0x2000139fu, 50);
    write_8(0x200013a0u, 153);
    write_8(0x200013a1u, 1);
    write_8(0x200013a3u, 1);
    write_8(0x200013a4u, 196);
    write_8(0x200013a5u, 255);
    write_8(0x200013adu, 16);
    write_8(0x20003002u, 16);
    write_8(0x20003004u, 255);
    write_8(0x20003005u, 255);
    write_8(0x20003006u, 16);
    write_8(0x20003574u, 1);
    write_8(0x20003576u, 110);
    write_8(0x20003578u, 82);
    write_8(0x20003579u, 3);
    write_8(0x2000357au, 1);
    write_8(0x2000357cu, 60);
    write_8(0x2000357eu, 214);
    write_8(0x2000357fu, 1);
    write_8(0x20003580u, 120);
    write_8(0x20003582u, 104);
    write_8(0x20003583u, 1);
    write_8(0x20003584u, 17);
    write_8(0x20003586u, 25);
    write_8(0x20003588u, 250);
    write_8(0x2000358au, 163);
    write_8(0x2000358cu, 39);
    write_8(0x2000358du, 6);
    write_8(0x2000358eu, 39);
    write_8(0x2000358fu, 6);
    write_8(0x20003590u, 39);
    write_8(0x20003591u, 6);
    write_8(0x20003592u, 39);
    write_8(0x20003593u, 6);
    write_8(0x20003594u, 39);
    write_8(0x20003595u, 6);
    write_8(0x20003596u, 39);
    write_8(0x20003597u, 6);
    write_8(0x20003598u, 39);
    write_8(0x20003599u, 6);
    write_8(0x2000359au, 39);
    write_8(0x2000359bu, 6);
    write_8(0x2000359cu, 39);
    write_8(0x2000359du, 6);
    write_8(0x200039adu, 1);
    write_8(0x200039aeu, 1);
    write_8(0x20003a18u, 8);
    write_8(0x20003a19u, 55);
    write_8(0x20003a1bu, 32);
    write_8(0x20003a60u, 32);
    write_8(0x20003a61u, 55);
    write_8(0x20003a63u, 32);
    write_8(0x20003b83u, 4);
    write_8(0x20003b84u, 16);
    write_8(0x20003c68u, 100);
    write_8(0x20003c69u, 25);
    write_8(0x20003c7du, 55);
    write_8(0x20003c7fu, 32);
    write_8(0x20003c8cu, 16);
    write_8(0x20003c8du, 63);
    write_8(0x20003c8fu, 32);
    write_8(0x20003c94u, 16);
    write_8(0x20003c95u, 55);
    write_8(0x20003c97u, 32);
    write_8(0x20003c98u, 17);
    write_8(0x20003c99u, 55);
    write_8(0x20003c9bu, 32);
    write_8(0x20003c9cu, 18);
    write_8(0x20003c9du, 55);
    write_8(0x20003c9fu, 32);
}

void g53_chain_step(const g53_chain_input_t *in, g53_chain_output_t *out)
{
    /* One explicit M820 owner advances both logical phase counters. They are
     * stored separately to make the chosen common reset point reviewable. */
    memset(out,0,sizeof(*out));
    write_16(M+0x92,in->pas.cadence);
    write_16(M+0x94,in->pas.evidence);
    write_8(M+0x96,in->pas.movement);
    write_16(M+0x28,in->pas.base_timeout_ticks);
    write_16(M+0x2a,in->pas.no_transition_ticks);
    write_16(M+0x2c,in->pas.max_timeout_ticks);
    write_8(M+0x40,in->pas.anti_rock);
    if(fast_phase==7) {
        write_16(G+0x21e,in->x); write_8(G+7,in->level);
        write_8(G+4,read_8(M+0x299)); write_8(G+5,read_8(M+0x29a));
        write_8(G+0xb,0); chain_fsm_model();
    }
    write_16(M+0x2e,read_16(G+0x222));
    write_8(M+0x31,read_8(T+0x4a)); write_8(M+0x32,read_8(T+0x4b));
    write_8(M+0x33,read_8(T+0x4d)); write_8(M+0x34,read_8(T+0x4c));
    write_8(M+0x39,in->external_inhibit);
    write_8(M+0x2e4,in->level); write_16(M+0x44,in->speed_native);
    write_32(DIAG,in->diag_word);
    write_16(0x20001054,in->g1_q12); write_16(M+0x2a6,in->g2_q12);
    write_16(0x20003708,abs64(in->speed_native));
    chain_bde8_model();
    if(supervisor_phase==0) {
        write_8(M+0x2e5,4); write_16(M+0x50,0); write_16(M+0x52,0);
        write_8(M+0x2ed,0); write_8(M+0x29f,0);
        write_16(read_32(M+0x74),in->speed_native);
        write_16(CUR1,in->rider_input_native);
        chain_d7ec_model();
    }
    if(supervisor_phase==3) chain_e1e8_model();
    out->trace.logical_tick = chain_tick;
    out->trace.fast_phase = fast_phase;
    out->trace.supervisor_phase = supervisor_phase;
    out->trace.phases_executed = 1 | 4 | (fast_phase==7 ? 2 : 0) | (supervisor_phase==0 ? 8 : 0) | (supervisor_phase==3 ? 16 : 0);
    out->trace.x = in->x;
    out->trace.fsm_state = read_8((T + 50LL));
    out->trace.fsm_output = read_16((G + 546LL));
    out->trace.fsm_t34 = read_16((T + 52LL));
    out->trace.fsm_t36 = read_16((T + 54LL));
    out->trace.fsm_t38 = read_16((T + 56LL));
    out->trace.fsm_t3a = read_16((T + 58LL));
    out->trace.fsm_t3c = read_16((T + 60LL));
    out->trace.fsm_t40 = read_16((T + 64LL));
    out->trace.fsm_t44 = read_16((T + 68LL));
    out->trace.fsm_t46 = read_16((T + 70LL));
    out->trace.fsm_t48 = read_16((T + 72LL));
    out->trace.fsm_t4a = read_8((T + 74LL));
    out->trace.fsm_t4b = read_8((T + 75LL));
    out->trace.fsm_t4c = read_8((T + 76LL));
    out->trace.fsm_t4d = read_8((T + 77LL));
    out->trace.fsm_g1fe = read_8((G + 510LL));
    out->trace.fsm_g1ff = read_8((G + 511LL));
    out->trace.fsm_g200 = read_8((G + 512LL));
    out->trace.fsm_g203 = read_8((G + 515LL));
    out->trace.pas_direction = in->pas.direction;
    out->trace.pas_direction_candidate = in->pas.direction_candidate;
    out->trace.cadence = in->pas.cadence;
    out->trace.evidence = in->pas.evidence;
    out->trace.movement = in->pas.movement;
    out->trace.base_timeout = in->pas.base_timeout_ticks;
    out->trace.full_timeout = in->pas.full_timeout_ticks;
    out->trace.max_timeout = in->pas.max_timeout_ticks;
    out->trace.no_transition = in->pas.no_transition_ticks;
    out->trace.pas_code = in->pas.code;
    out->trace.pas_current_delta = in->pas.current_delta;
    out->trace.pas_previous_delta = in->pas.previous_delta;
    out->trace.pas_transition_count = in->pas.transition_count;
    out->trace.pas_elapsed = in->pas.elapsed_ticks;
    out->trace.pas_span = in->pas.span;
    out->trace.pas_magnitude = in->pas.magnitude;
    out->trace.pas_raw_cadence = in->pas.raw_cadence;
    out->trace.pas_filter_accumulator = in->pas.filter_accumulator;
    out->trace.pas_filtered_cadence = in->pas.filtered_cadence;
    out->trace.pas_plausibility_budget = in->pas.plausibility_budget;
    out->trace.pas_plausibility_base = in->pas.plausibility_base;
    out->trace.pas_plausibility_latch = in->pas.plausibility_latch;
    out->trace.pas_plausibility_previous_latch = in->pas.plausibility_previous_latch;
    out->trace.pas_plausibility_flag = in->pas.plausibility_flag;
    out->trace.pas_anti_rock = in->pas.anti_rock;
    out->trace.d7ec_mode = read_8((D + 32LL));
    out->trace.d7ec_selected_current = read_16((D + 10LL));
    out->trace.d7ec_speed = read_16((D + 6LL));
    out->trace.d7ec_m50 = read_16((D + 8LL));
    out->trace.d7ec_m52 = read_16((D + 4LL));
    out->trace.d7ec_m29f = read_8((D + 23LL));
    out->trace.d7ec_base_timeout = read_16((D + 16LL));
    out->trace.d7ec_no_transition = read_16((D + 18LL));
    out->trace.d7ec_max_timeout = read_16((D + 20LL));
    out->trace.d7ec_envelope = read_16((D + 170LL));
    out->trace.d7ec_readiness = (((read_16((D + 14LL)) >= read_8((D + 38LL)))) || ((read_16((D + 170LL)) >= read_16((D + 36LL)))));
    out->trace.d7ec_rider_intermediate = read_16((D + 194LL));
    out->trace.d7ec_rider = read_16((D + 196LL));
    out->trace.d7ec_assist_ratio = read_16((D + 204LL));
    out->trace.d7ec_ratio = read_16((D + 208LL));
    out->trace.d7ec_taper = read_16((D + 210LL));
    out->trace.d7ec_requested = read_16((D + 212LL));
    out->trace.d7ec_lut = read_16((D + 214LL));
    out->trace.d7ec_converted = read_16((D + 218LL));
    out->trace.d7ec_hold = read_16((D + 222LL));
    out->trace.d7ec_accel_target = read_16((D + 224LL));
    out->trace.d7ec_accel_live = read_16((D + 226LL));
    out->trace.d7ec_accel_rise = read_16((D + 232LL));
    out->trace.d7ec_accel = read_16((D + 238LL));
    out->trace.d7ec_accel_window = read_16((D + 234LL));
    out->trace.d7ec_accel_counter = read_16((D + 236LL));
    out->trace.d7ec_history_delta = read_16((D + 178LL));
    out->trace.d7ec_history_a = read_16((D + 182LL));
    out->trace.d7ec_history_b = read_16((D + 184LL));
    out->trace.d7ec_history_selected = read_16((D + 186LL));
    out->trace.d7ec_external_guard = read_8((M + 666LL));
    out->trace.d7ec_d19 = read_8((D + 25LL));
    out->trace.e1e8_state = read_8((0LL + (Q + 80LL)));
    out->trace.e1e8_factor = read_16((0LL + (E + 104LL)));
    out->trace.e1e8_taper = read_16((0LL + (E + 208LL)));
    out->trace.e1e8_pi = read_16((0LL + (E + 188LL)));
    out->trace.e1e8_target = read_16((0LL + (E + 196LL)));
    out->trace.e1e8_output = read_16((0LL + (E + 206LL)));
    out->trace.e1e8_output_factor = read_16((0LL + (M + 674LL)));
    out->trace.e1e8_taper_factor = read_16((0LL + (M + 676LL)));
    out->trace.e1e8_latch = read_8((0LL + (E + 101LL)));
    out->trace.bde8_q50 = read_8((0LL + (Q + 80LL)));
    out->trace.bde8_q5a = read_16((0LL + (Q + 90LL)));
    out->trace.bde8_q5c = read_16((0LL + (Q + 92LL)));
    out->trace.bde8_mode = read_8((0LL + (M + 664LL)));
    out->trace.bde8_demand = read_16((0LL + (M + 682LL)));
    out->trace.bde8_angle = read_16((0LL + (M + 688LL)));
    out->trace.bde8_g04 = read_8((0LL + (M + 665LL)));
    out->trace.g1 = in->g1_q12;
    out->trace.g2 = in->g2_q12;
    out->trace.m298 = read_8((0LL + (M + 664LL)));
    out->trace.m299 = read_8((0LL + (M + 665LL)));
    out->trace.m28 = read_16((0LL + (M + 40LL)));
    out->trace.m2a = read_16((0LL + (M + 42LL)));
    out->trace.m2c = read_16((0LL + (M + 44LL)));
    out->trace.m34 = read_8((0LL + (M + 52LL)));
    out->trace.m40 = read_8((0LL + (M + 64LL)));
    out->trace.m50 = read_16((0LL + (M + 80LL)));
    out->trace.m52 = read_16((0LL + (M + 82LL)));
    out->trace.m29f = read_8((0LL + (M + 671LL)));
    out->trace.m2a2 = read_16((0LL + (M + 674LL)));
    out->trace.m2a4 = read_16((0LL + (M + 676LL)));
    out->trace.m2a8 = read_16((0LL + (M + 680LL)));
    out->trace.m2aa = read_16((0LL + (M + 682LL)));
    out->trace.m2ac = read_16((0LL + (M + 684LL)));
    out->trace.m2ae = read_16((0LL + (M + 686LL)));
    out->trace.m2b0 = read_16((0LL + (M + 688LL)));
    out->trace.diag = (in->diag_word & 4294967295LL);
    out->trace.external_inhibit = ((in->external_inhibit != 0LL));
    out->trace.fatal_fault = (((read_32((0LL + DIAG)) & 6192LL) != 0LL));
    out->trace.nonfatal_fault = (((read_32((0LL + DIAG)) && (!(read_32((0LL + DIAG)) & 6192LL))) != 0LL));
    out->trace.rider_input_native=in->rider_input_native;
    out->m2aa_native=(uint16_t)out->trace.m2aa;
    /* Observation of the normal BDE8 drive-command mode, no extra PAS gate. */
    out->normal_permission=(out->trace.m298==2);
    ++chain_tick;
    fast_phase=(uint8_t)((fast_phase+1u)%10u);
    supervisor_phase=(uint8_t)((supervisor_phase+1u)%10u);
}

uint8_t g53_chain_level(uint8_t assist_level)
{
    static const uint8_t slots[6]={0,2,4,6,8,9};
    return assist_level<6 ? slots[assist_level] : 0;
}
