# FW-112 — Rodzina kart: finalna architektura i HARDWARE CLOSURE (CANONICAL)

**STATUS: FW-112 / FW-112.2 — HARDWARE CLOSED.** To jest canonical summary całej rodziny FW-112
(v2 → FW-112-DIAG → FW-112-DIAG.1 → FW-112.1 → FW-112.2). Szczegółowe karty pozostają źródłem
prawdy dla własnych scenariuszy/mutacji/buildów i są tu zlinkowane; ten dokument opisuje ewolucję
problemu, finalną architekturę, finalne dowody sprzętowe, invarianty i decyzję zamknięcia.

Karty:
- [FW-112 v2 — Odzyskiwanie estymatora RUN przy rearm](FW-112_REARM_RUN_ESTIMATOR_RECOVERY_PL.md)
- [FW-112-DIAG — Rejestrator zdarzeń całego łańcucha pedal→Iq](FW-112-DIAG_WHOLECHAIN_RECORDER_PL.md)
- [FW-112.1 — Separacja REAL_STOP od aktywności wstecznej/invalid](FW-112.1_REAL_STOP_LIVENESS_SEPARATION_PL.md)
- [FW-112.2 — REAL_STOP vs ROLLING COAST (finalny raport)](FW-112.2_ROLLING_COAST_REAL_STOP_PL.md)

**FLASH / COMMIT / PUSH = NO.** Nic nie zostało wgranane ani zacommitowane przez tę kartę dokumentacyjną.

---

## 1. Problem pierwotny

Po cofnięciu pedału (reverse) wspomaganie nie wracało płynnie: estymator RUN (`torque_run_filtered`)
przesuwał swoje 180-stopniowe okno próbkami zerowego/małego nacisku, więc po nawrocie świeżego silnego
nacisku Iq było słabe przez całą połowę obrotu (~600 ms w zapisie FW-111). Do tego sesja bywała gaszona
do `COLD` przez fałszywe REAL_STOP mimo fizycznie toczącego się roweru. Rodzina FW-112 rozwiązała to
warstwami: najpierw szybki powrót RUN/Iq (v2), potem obserwowalność (DIAG/DIAG.1), potem separacja
liveness od kierunku (FW-112.1), wreszcie rozróżnienie postoju prawdziwego od tocznego coastu (FW-112.2).

## 2. FW-112 v2 — odzyskiwanie RUN, szybki powrót Iq po reverse

Pierwotny fix dotyczył:

    reverse during active ride
    -> SUSPENDED_BY_DIRECTION
    -> rolling rearm
    -> forward permission recovery

Potwierdzone (host S1–S15 + FW-109/FW-111 integracje, patrz karta):

- rolling reverse nie powodował starej ~1–2 s NOT_LATCHED dziury,
- permission mogło wrócić bez cold start_steps READY,
- recovery posiadało stany `IDLE` / `WAIT_FRESH_LOAD` / `TRACK_FAST`,
- `WAIT_FRESH_LOAD` przy zero demand trzymało Iq=0,
- `TRACK_FAST` używało FAST source,
- prawdziwy PAS timeout nadal prowadził COLD.

Pierwotny status:

    FW-112 HARDWARE CLOSED

Karta została **później ponownie otwarta** na podstawie nowych hardware observations dotyczących
restartu podczas jazdy. **Historia ta jest celowo zachowana** — nie udajemy, że wcześniejsze testy były
błędne. Nowy przypadek sprzętowy ujawnił **dodatkowy lifecycle problem** (kolejne warstwy poniżej).

## 3. FW-112-DIAG — whole-chain event recorder

Dodano rekorder całego łańcucha pedal→Iq (moduł diag-only `src/fw112_diag.c`), pozwalający zobaczyć:

    BLOCKED
    GRANTED
    REVOKED
    RECOVERY_ENTER
    RECOVERY_EXIT
    RECOVERY_COLLAPSE
    ZEROED
    HOLD_ARMED
    HOLD_EXPIRED

oraz session/rearm/recovery/Iq pipeline (ramki `0x1022A`–`0x1022E`). Rekorder startowy:

    8 x 32 B

zapełniał się dokładnie przy `RECOVERY_ENTER`, chowając ogon odzyskiwania.

## 4. FW-112-DIAG.1 — pojemność 8 → 24

Rekorder zwiększono:

    8 x 32 B  ->  24 x 32 B

Policy: **reject-on-full** (odrzucenie nowego, nigdy nadpisanie — zrzut nie może stracić historii sesji).
Production behavior: **NONE**. DIAG RAM delta: **+512 B** (bss 23932 → 24444 B w DIAG=1; DIAG=0 bez
zmian). To umożliwiło złapanie pełnej sekwencji kilku epizodów.

## 5. FW-112.1 — REAL_STOP oparty na liveness, nie na idle-timerze

Root cause #1:

    REAL_STOP był oparty na PAS liveness,
    ale INVALID physical edges nie odświeżały stop timer.

Dodano osobne `pas_liveness`, odświeżane przez **każdą** physical PAS transition:

    forward
    reverse
    invalid

Reverse/invalid:

    refresh liveness ONLY

Nigdy:

    permission
    cadence assist
    demand
    start_steps
    positive Iq

Hardware potwierdził, że separation działa, **ale problem restartu nie został jeszcze całkowicie
rozwiązany**. Powód: nawet poprawny PAS REAL_STOP po >200 ms bez edge nadal powodował `COLD`, mimo że
rower nadal się toczył.

## 6. FW-112.2 — REAL_STOP vs ROLLING COAST (karta finalna)

Root cause #2:

    PAS stop / REAL_STOP
    był nadal traktowany jako terminal ride-session stop
    bez rozróżnienia rolling bicycle vs true vehicle stop.

Dodano `ride_wheel_valid` / `rolling_valid` i rozdzielono:

    PAS liveness
    od
    ride/vehicle rolling context.

Końcowy contract:

    terminal =
        non_direction_safety_cut
        || assist_off
        || (real_stop && !rolling_valid)

ACTIVE może przejść do SUSPENDED przy:

    direction_inhibit_active
    ||
    (real_stop && rolling_valid)

SUSPENDED pozostaje zachowane podczas rolling coast.

Rearm nadal wymaga:

    !direction_inhibit_active
    &&
    (confirm_edge || forward_pedaling)

**BARDZO WAŻNE: `rolling_valid` NIE daje permission.** Wheel movement oznacza wyłącznie „nie niszcz
jeszcze ride context", a NIE „włącz assist".

## 7. Finalna architektura FW-112 — trzy niezależne pojęcia

1. **DIRECTION PERMISSION** — forward confirmation kontroluje możliwość dodatniego assist.
2. **PAS LIVENESS** — physical PAS edge dowolnego kierunku pokazuje ruch korby.
3. **RIDE / WHEEL LIVENESS** — wheel validity pokazuje, czy rower nadal znajduje się w rolling context
   mimo chwilowego braku PAS.

Finalny model (A–F):

A)

    FORWARD + valid ride conditions
        -> ACTIVE
        -> permission allowed

B)

    REVERSE / INVALID
        -> SUSPENDED_BY_DIRECTION
        -> permission = 0
        -> no positive assist

C)

    PAS stops BUT wheel still valid
        -> rolling coast
        -> SUSPENDED_BY_DIRECTION retained
        -> permission = 0
        -> no positive assist

D)

    forward returns while rolling context preserved
        -> rolling rearm
        -> GRANTED
        -> recovery path

E)

    PAS stopped AND wheel validity expires
        -> REAL_STOP
        -> COLD

F)

    next start from COLD
        -> normal conservative cold-start path

### Safety invariants

- reverse never generates positive assist,
- invalid direction never generates positive assist,
- coast never generates positive assist,
- wheel movement alone never grants permission,
- `rolling_valid` alone never triggers fast rearm,
- no-PAS coast has zero new positive demand,
- true vehicle stop always leads to COLD,
- cold start remains conservative,
- rolling rearm does not replace all starts,
- `assist_without_rotation` was NOT used as coast retention mechanism.

### Niezmienione elementy (FW-112.2 NIE zmieniało)

`WAIT_FRESH_LOAD`, `TRACK_FAST`, fresh-load threshold, torque FAST, torque RUN, torque filters, torque
calibration, `assist_hold` semantics, `start_steps`, cold-start thresholds, hard-cut ramp, boost,
power mapping, PAS direction classifier, forward confirmation count, motor current, FOC, throttle,
Walk Assist / FW-113.

## 8. Finalne hardware evidence

Ostatni hardware test: **eVD 0.0366 (DIAG=1, M820 / BL820)**.

### A. Rolling-coast hardware acceptance — PASS

Potwierdzone przykładowe sekwencje recordera:

    REVOKED
    -> BLOCKED / SUSPENDED
    -> GRANTED
    -> RECOVERY_ENTER
    -> ZEROED / recovery continuation

oraz wielokrotne podobne cykle **bez false COLD**.

- ROLLING REARM = HW PASS
- ROLLING COAST RETENTION = HW PASS
- REAL_STOP QUALIFICATION WHILE ROLLING = HW PASS

### B. Terminal-stop hardware acceptance — PASS

Log: `log-2026-08-17-12-35-39-n0.log`.

    REAL_STOP -> terminal COLD

oraz kolejny start:

    BLOCKED -> GRANTED -> COLD START

bez `RECOVERY_ENTER` — po prawdziwym stopie rolling rearm nie został użyty.

- TRUE REAL STOP -> COLD = HW PASS
- COLD START AFTER TRUE STOP = HW PASS

## 9. Host / regression status (zachowany dokładnie)

FW-112.2:

    fw112_2_rolling_coast_host S1-S14 PASS
    ride_session_host 768 cells PASS
    fw112_diag_host + S15 PASS

Mutations: **M1–M5 proven** (szczegóły w karcie FW-112.2).

Regressions:

    fw112_2 PASS
    fw112_1 PASS
    fw112_run_rearm_recovery PASS
    ride_control_rearm PASS
    ride_session PASS
    fw112_diag PASS
    pas_direction PASS
    pas_quadrature PASS
    rearm_trace_raw_integration PASS

Ale:

    FULL run-host-tests.ps1 = BLOCKED BY DEFENDER

**NIE zgłaszamy „FULL HOST SUITE PASS".**

## 10. Ograniczenia (zachowane historycznie)

- **2 s WAIT_FRESH_LOAD: HOST PROVEN, HW NOT REPRODUCED.** Nie zmienione na HW PASS.
- **reject-on-full** rekordera FW-112-DIAG pozostaje świadomym ograniczeniem diagnostycznym, a nie
  production behavior issue.
- **TARGET_RECOVERED** historycznie mogło oznaczać recovery osi RUN, a nie faktyczny pozytywny output
  silnika — NIE jest używane jako finalny dowód HW. Finalny dowód opiera się na: session state,
  permission, recovery event, Iq/output path, true-stop vs cold-start separation.

## 11. Finalne buildy (referencje — bez nowego builda)

| Rola | Build | text | bss | flashowalny bin |
|---|---|---|---|---|
| **Final hardware image** | **`0.0366_M820_BL820.bin`** (DIAG=1) | 127420 | 24452 | 127724 |
| **Production reference** | **`0.0367_M820_BL820.bin`** (DIAG=0) | 98284 | 11700 | 98588 |

Nie tworzono 0.0368 „dla zamknięcia dokumentacji". Finalny hardware acceptance użył **0.0366 DIAG=1**;
produkcja = **0.0367 DIAG=0**. (Wcześniejsze pary rodziny: FW-112-DIAG 0.0358/0.0360, FW-112-DIAG.1
0.0362/0.0363, FW-112.1 0.0364/0.0365 — patrz karty.)

## 12. Finalna decyzja

FW-112 / FW-112.2 hardware acceptance is complete.

Rolling reverse, invalid-direction activity and no-pedal rolling coast no longer incorrectly collapse
the ride session into COLD while the bicycle is still moving.

Positive assist remains prohibited during reverse, invalid direction and coast.

Confirmed forward motion may restore rolling assist through the established rolling-rearm/recovery path.

A genuine vehicle stop still terminates the session conservatively into COLD, and the next start follows
the normal cold-start path.

**FW-112.2 — HARDWARE CLOSED**

---

**FLASH / COMMIT / PUSH = NO.** **STOP** — rodzina FW-112 zamknięta; nie rozpoczyna się FW-113 ani
żadnej następnej karty w ramach tego zadania.