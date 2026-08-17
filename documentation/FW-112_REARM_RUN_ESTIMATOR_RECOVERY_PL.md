# FW-112 v2 — Odzyskiwanie estymatora RUN przy rearm (szybki powrót Iq po reverse)

**Werdykt: GOTOWE DO PRZEGLĄDU. NIE DO JAZDY.** Ta karta nie została potwierdzona na rowerze;
wszystkie dowody są testami hostowymi na prawdziwych modułach + buildami ARM.

---

## 1. Cel

Usunięcie opóźnionego odbudowywania Iq po reverse (rearm). Przyczyną jest przesuwanie
180-stopniowego okna estymatora RUN (`torque_run_filtered`) próbkami o zerowym/małym nacisku
podczas fazy po reverse. Po nawrocie silnego świeżego nacisku estymator odzyskuje wartość
jedną próbką na krok korby (48 kroków), więc Iq po rearm jest słabe przez całą połowę obrotu
(~600 ms w zapisie FW-111, t_commit=552 → t_target_recovered=3131).

**Kryterium właściciela (twarde): powrót RUN/Iq do ≥80% wartości ciepłej w ≤150 ms / ≤8
kroków po dowolnym okresie bez nacisku.** Nigdy w czasie przesuwania okna RUN; lekki nacisk po
reverse nie może odtworzyć starego wysokiego Iq.

## 2. Architektura v2 (co różni ją od v1)

v1 to był "latched WAIT-wide fast-track": przez cały `WAIT_REARM_LOAD` estymator re-seedował
RUN do próbki FAST przy każdym kroku i był rozbrajany przez dokładnie te krawędzie, które nie
powinny go rozbrajać. v2 usuwa etap WAIT i dwufazowy commit całkowicie:

- **Permission** (`SUSPENDED_BY_DIRECTION → ACTIVE`) jest CZYSTYM faktem kierunkowym — przyznawana
  na krawędzi potwierdzenia kierunku, bez warunku momentu. **Demand** jest liczona na świeżo
  każdego ticku przez `assist_modes_calculate()`; permission != demand.
- **Odzyskiwanie RUN** to ONE-SHOT automat trójstanowy (IDLE / WAIT_FRESH_LOAD / TRACK_FAST)
  w `torque_input.c`, otwierany jednym zdarzeniem `torque_input_begin_rolling_rearm()` z
  `ride_control.c` w ticku `fast_rearm_this_tick`, a zamykany przez `cancel_rolling_rearm()`
  (!latched) lub ukończone odzyskiwanie. Automat nie ma timeoutu i nigdy nie jest anulowany
  przez liczbę kroków forward.

### FSM odzyskiwania (src/torque_input.c, `torque_input_update()`)

| stan | wejście | zachowanie | wyjście |
|------|---------|------------|---------|
| `IDLE` | start / po odzyskaniu / cancel | normalne uśrednianie FW-085 | `begin_rolling_rearm()` (tick rearm) → `WAIT_FRESH_LOAD` |
| `WAIT_FRESH_LOAD` | begin | RUN re-seedowany do bieżącego FAST przy każdym kroku forward (`torque_input_run_filter_step`) — stare okno nie może przetrwać rearm | FAST ≥ deadband → `TRACK_FAST` |
| `TRACK_FAST` | WAIT_FRESH_LOAD | `run_value_native = FAST` co tick sterowania (35 ms) — RUN śledzi świeży nacisk w czasie filtra FAST, niezależnie od kadencji kroków | FAST < deadband → `WAIT_FRESH_LOAD` (uczciwy kolaps); FAST ≥ deadband przez `TORQUE_ROLLING_REARM_STABLE_TICKS` → `seed_run()` raz i `IDLE` (odzyskanie zakończone) |

**Stale-sample fix (strona czytająca):** snapshot jeźdźca jest budowany w main.c PRZED
`ride_control_update()`, więc w ticku permission nadal niesie średnią okna sprzed rearm.
Podczas gdy automat nie jest IDLE, `torque_input_recovery_run_native()` zwraca bieżący FAST,
który `ride_control.c` podstawia w kalkulacji mode na `torque_run_filtered` — bez kroku
opóźnienia.

### Dlaczego okno stabilności = 4× stała czasowa filtra FAST

`TORQUE_ROLLING_REARM_STABLE_MS = 4 * TORQUE_ASSIST_FILTER_MS` (140 ms). Jedna stała czasowa
(35 ms) NIE dowodzi, że świeży sygnał "przejął": w t=τ EMA 35 ms pokrywa tylko 1−e⁻¹ ≈ 63%
drogi do nowego poziomu. Wyjście wtedy re-seedowałoby RUN w połowie narastania (~70% celu), a
wolne okno FW-085 potrzebowałoby potem ~26 kroków do 80% (łamie twarde kryterium). Cztery stałe
czasowe (1−e⁻⁴ ≈ 98%) to prawdziwe plateau — i przez cały czas TRACK_FAST RUN = FAST, więc
estymator osiąga 80% w ~6 krokach (~63 ms) nawet po pełnym oknie bez nacisku.

> **ŚWIADOME ODCHYLENIE OD SPECYFIKACJI.** Wartość 140 ms (`4 × TORQUE_ASSIST_FILTER_MS`) jest
> celowym odstępstwem od pierwotnego założenia FW-112 (pojedyncza stała czasowa 35 ms). Jest to
> decyzja projektowa z uzasadnieniem powyżej, NIE błąd implementacji — świadomie NIE
> implementowane jako pojedyncza stała czasowa; zachowane w FW-085 jako `STABLE_TICKS`.
> Dokumentacja i kod są z tym zgodne; harness S2/S7 potwierdza, że twarde kryterium ≤150 ms
> nadal jest spełnione mimo dłuższego okna stabilizacji (RUN = FAST przez cały TRACK_FAST).

## 3. Reprodukcja (harness `tests/host/fw112_run_rearm_recovery_host.c`)

Harness łączy **prawdziwe** moduły: torque_input.c, ride_control.c, ride_session.c,
pas_direction.c, pas_quadrature.c, assist_modes.c, motor_core.c i cały łańcuch mocy. Tick
kontrolny 4 kHz, korba 60 rpm (krok co 42 ticki), realny estymator RUN. Scenariusze S1–S15:
S1–S13 to sekwencje RUN/rearm/recovery (S13 = tick WAIT_FRESH_LOAD przy zerowym demandzie),
S14a–c to macierz długości reverse (2/300/512 kroków), S15a–f to macierz trybów banku
(1/2/3/5/6 działają, tryb 4 odrzucony przez `bank_mode_valid`).

## 4. Wyniki na POPRAWIONYM kodzie (v2)

```
FW-112 v2 RUN estimator lifecycle around the real rearm chain (permission vs demand)
  window 180 deg = 48 steps   cadence 60 rpm (step every 42 ticks)   fast filter 35 ms
  S1 warm RUN target = 309
  S1: permission at step 1 (run=301 filtered=140 iq=43)   recovery at step 1 (10 ms, 0.0 rev)
  S2: run during the wait = 0 (of warm 309)   rearm tick=1 (mode=0 live=0 actual=0)
   S2 FSM -> closed to IDLE at step 13 with run=304 (target 247)
  S2: recovery at step 6 (63 ms, 0.1 rev)   fsm first step=2        <- twarde kryterium
  S3: run after 4 zero-pressure steps = 294 (of warm 309)   recovery at step 1
  S4: warm RUN = 309, light-pressure recovery max RUN = 30          <- było: 175 (stale high)
  S5: permission at step 1, rearm tick all-zero (mode=0 live=0 actual=0), zero for 59 steps
  S7: 2s wait kept WAIT_FRESH_LOAD (run=0), late resume -> recovery at step 6 (63 ms), IDLE back
  S8: TRACK_FAST tracked fast every tick (no PAS edges), fast rose 241 -> 271
  S9: terminal-inhibit matrix (14 combinations) all closed the recovery to IDLE in the same tick
  S10: boost after warmup -> state=2 peak=1546 centikg
  S10: reverse cancelled the boost same-tick, rearm delivered 0, post-rearm boost state=2
  S11: held at 0 under torque > deadband with a 0 mode result (filtered=310)
  S12: positive demand armed the hold (5600), grace min-pull, expiry kept ACTIVE
  S13: TRACK_FAST -> WAIT grace suppressed (hold 5600 -> 0, floor 0, target 0, setpoint 0 same-tick), 2s clean, late resume at step 6 (63 ms), IDLE back
  S14a: 2R -> rearm at confirm step 1, RUN recovered, recovery -> IDLE (rev_run=0)
  S14b: 300R -> rearm at confirm step 1, RUN recovered, recovery -> IDLE (rev_run=0)
  S14c: 512R -> rearm at confirm step 1, RUN recovered, recovery -> IDLE (rev_run=0)
  S15a: mode 1 rearm -> WAIT zero (0/0/0), late resume at step 6 (63 ms), IDLE back
  S15b: mode 2 rearm -> WAIT zero (0/0/0), late resume at step 6 (63 ms), IDLE back
  S15c: mode 3 rearm -> WAIT zero (0/0/0), late resume at step 6 (63 ms), IDLE back
  S15d: mode 5 rearm -> WAIT zero (0/0/0), late resume at step 6 (63 ms), IDLE back
  S15e: mode 6 rearm -> WAIT zero (0/0/0), late resume at step 6 (63 ms), IDLE back
  S15f: mode 4 rejected by the wire guard bank_mode_valid (stays unsupported/no-assist)
  S6: cold-start run = 63  (fast = 301)
All FW-112 v2 RUN lifecycle checks passed.
```

Co pinuje każdy scenariusz (kontrakt R1–R7 w nagłówku harnessa):

| scenariusz | kontrakt | dowód |
|------------|----------|-------|
| S1 | reverse/INVALID zeruje **demand i pre-ramp TARGET** w tym samym ticku sterowania; bieżące odniesienie prądu podąża za istniejącym, należącym do firmware `RIDE_HARD_CUT_RAMP_MS` (świadoma polityka FW-037, ~200 ms łagodzenia uderzenia przekładni — `MS.i_q_setpoint` NIE jest asertowany na 0 same-tick przy reverse); natychmiastowy silny resume → permission w ≤4 kroków, Iq pełnej wartości w czasie FAST | permission step 1, recovery step 1 (10 ms) |
| S2 | pełne okno zanieczyszczenia uczciwie kolapsuje RUN (run=0); NA TYM SAMYM TICKU rearm (mode demand, pre-ramp target i MS.i_q_setpoint = 0); Iq zostaje 0 (permission != demand); silny resume → **twarde kryterium** ≤150 ms/≤8 kroków + piny FSM (WAIT_FRESH_LOAD/TRACK_FAST→IDLE) | rearm tick=1 all-zero, recovery step 6 (63 ms), FSM zamknięty do IDLE przy step 13 z run=304 |
| S3 | okno rearm to TYMCZASOWA pomoc — po odzyskaniu wraca uśrednianie FW-085 (4 kroki bez nacisku → spadek o kilka próbek, nie kolaps do FAST) | run po 4 zero-steps = 294 |
| S4 | lekki nacisk po reverse nie odtwarza starego wysokiego RUN/Iq | max RUN = 30 ≪ warm/2 = 154 |
| S5 | brak nacisku po reverse: NA TYM SAMYM TICKU rearm demand/pre-ramp/MS.i_q_setpoint = 0 i Iq 0 przez 59 kroków (rearm re-seeduje RUN do zdekayowanego sygnału) | rearm tick all-zero, zero dla 59 kroków |
| S6 | zimny start po realnym stopie bez zmian (RUN seeduje się do świeżego FAST) | cold-start run=63, iq>0 |
| S7 | 2 s bez nacisku po reverse: WAIT_FRESH_LOAD bez timeoutu, ACTIVE przez cały czas, demand/motor 0, bez wycieku floor; późny silny nacisk → **twarde kryterium** ≤150 ms/≤8 kroków + FSM → IDLE | recovery step 6 (63 ms), powrót do IDLE |
| S8 | TRACK_FAST śledzi FAST co tick sterowania (nie tylko co krok PAS) — brak krawędzi przez całe okno, a RUN podąża za świeżym naciskiem | fast rose 241 → 271 co tick |
| S9 | macierz 14 kombinacji terminal/inhibit (reverse/invalid/walk/kalibracja/safety-cut/assist 0/real stop × WAIT_FRESH_LOAD/TRACK_FAST) zamyka odzyskiwanie do IDLE w tym samym ticku | 14/14 closed same-tick |
| S10 | Extended Boost w kontekście rearm: warmup armuje boost (state=2, peak=1546 centikg); reverse kasuje go w tym samym ticku (IDLE, cancel REVERSE/SAFETY_CUT); okno bez nacisku kolapsuje RUN; rearm dostarcza 0 (bez stale boost prądu); po rearm świeża re-kwalifikacja boostu (state=2, nigdy ACTIVE) | boost state=2 → reverse → IDLE same-tick, rearm 0, post-rearm state=2 |
| S11 | HOLD OWNER (negative — detektor mutacji D): ACTIVE + permission + brak reverse/hard cut, `assist_hold_ticks` startowo 0, `torque_assist_filtered` (≈310 native) wyraźnie > `run_deadband_mv` (5), ale `mode_output.iq_request == 0` (stan deterministyczny: `battery_voltage_mv == 0` przy silnym nacisku — guardy voltage w `assist_modes.c` zwracają `supported=true` z demandem 0; limiter używa osobnego `voltage_raw`, więc nie tnie) → hold NIE może się uzbroić (owner to wynik MODE, nie surowy torque); bez floor (`iq_after_latch_floor == 0`), `live_target == 0`, `MS.i_q_setpoint == 0`; z tą samą zmutowaną produkcją (`supported && torque_assist_filtered ≥ deadband`) S11 FAIL 4/4 | hold=0 przez całe okno, floor=0, target=0, setpoint=0 (filtered=310) |
| S12 | HOLD OWNER (positive + expiry — przypadek dodatni): pozytywny demand (silny nacisk, `mode_output.iq_request > 0`) uzbraja/odnawia hold; po zniknięciu demand (okno zero-pressure kolapsuje RUN) istniejący hold zachowuje starą semantykę grace/min-Iq (min-pull floor przez całą grace); wygaśnięcie holda NIE zmienia ACTIVE → COLD (permission należy do ride_session, nie do licznika holda); po wygaśnięciu i braku demand setpoint=0 po fazie release (650 ms) | armed=5600, grace min-pull, expiry → ACTIVE + setpoint 0 |
| S13 | **BLOCKER v2 (wyciek min-Iq przez WAIT)**: recovery schodzi TRACK_FAST → WAIT_FRESH_LOAD, a tick daje `iq_request == 0` — absolutny kontrakt WAIT+zero-demand: `assist_hold_ticks` 0, floor 0, pre-ramp target 0 i `MS.i_q_setpoint` 0 NA TYM SAMYM TICKU (wcześniejszy pozytywny TRACK_FAST uzbroił hold 5600 — ten tick musi go wytłumić); 2 s czystego okna; późny silny nacisk → **twarde kryterium** ≤150 ms/≤8 kroków (step 6, 63 ms) + FSM → IDLE | WAIT: hold 5600 → 0, floor 0, target 0, setpoint 0 same-tick; 2 s clean; late resume step 6 |
| S14a–c | macierz długości reverse (2/300/512 kroków; rev_run nasyca się na 255): `SUSPENDED_BY_DIRECTION` + `live_target == 0` przez cały reverse; potwierdzenie kierunku na step 1 → rearm ACTIVE; RUN odzyskany; recovery zamyka się do IDLE; **niezależne od nasycenia uint8_t rev_run** | rearm step 1, RUN recovered, recovery → IDLE (rev_run=0), 2/300/512 |
| S15a–f | macierz trybów banku (serializacja bloba → `set_bank_mode()` patchuje `buffer[13 + (L-1)*48]`, CRC-16/CCITT na 253): tryby 1/2/3/5/6 każdy: ciepły ACTIVE z pozytywnym demandem, reverse blokuje, rearm zero, WAIT trzyma, późny resume step 6/63 ms, powrót do IDLE; tryb 4 (`EMTB_CUSTOM`) odrzucony przez `bank_mode_valid` (akceptuje tylko 1,2,3,5,6) — `apply_bank_blob` zwraca false | 5/5 trybów pełny cykl PASS; tryb 4 rejected |

- **FW-109 integracja** `ride_control_rearm_host.c` (te same zmodyfikowane moduły): PASS.
- **FW-111 pełny łańcuch** `rearm_trace_raw_integration_host.c` (quadrature + direction + session
  + rearm + pas_trace + pas_raw): ALL PASS (22 scenariusze).

### Macierz mutacji A–G (FW-112 v2)

Każda mutacja: tymczasowa zmiana produkcyjna → build `fw112_mutX.exe` (repo-local) → nazwa
asercji która FAIL → przywrócenie czystej produkcji → full PASS. Harness: `fw112_run_rearm_recovery_host.c`.

| mutacja | zmiana produkcyjna (tymczasowo) | harness | padająca asercja | wynik |
|---------|---------------------------------|---------|------------------|-------|
| A | `torque_input_cancel_rolling_rearm()` gdy `fwd_run ≥ start_steps` (v1 WAIT-kill) | fw112_mutA | `S2 R4 FSM: recovery automaton open (WAIT_FRESH_LOAD/TRACK_FAST)` + 24 dalsze (S7/S8/S9) | **DETECTED** (25) |
| B | timeout 4000 ticków (1 s) zamykający WAIT_FRESH_LOAD → IDLE | fw112_mutB | `S7: 2s no-pressure WAIT - ACTIVE + WAIT_FRESH_LOAD throughout...` + 3 (hard criterion, FSM) | **DETECTED** (4) |
| C | usunięcie substytucji recovery-run (stale sample) | fw112_mutC | `S2 SAME-TICK: mode demand is 0 on the exact rearm tick` + 11 (S2/S5/S7/S10 same-tick, S10 stale boost) | **DETECTED** (12) |
| D | owner holda: `supported && torque_assist_filtered ≥ run_deadband_mv` zamiast `supported && mode_output.iq_request > 0` | fw112_mutD_hold | `S11 HOLD OWNER: the raw torque above the deadband did NOT arm the hold (the mode result owns it)` + 3 (floor, target, setpoint) | **DETECTED** (4) — wcześniej „non-observable” |
| E | usunięcie pinu per-tick TRACK_FAST (`&& false`) | fw112_mutE | `S8: RUN tracked the fast signal after EVERY control tick (no PAS edge) - per-tick, not per-step` + 2 (S2/S7 hard criterion) | **DETECTED** (3) |
| F | usunięcie `cancel_rearm_recovery()` na krawędziach terminalnych (`!latched`) | fw112_mutF | `reverse in WAIT_FRESH_LOAD: recovery closed to IDLE in the SAME tick` + 9 (10 krawędzi terminalnych) | **DETECTED** (10) |
| G | v1 WAIT_REARM_LOAD: `torque_present` w `ride_session_input_t` + gate przejścia SUSPENDED→ACTIVE | fw112_mutG | `S2: permission returned (confirm edge fired)` + 35 (S2/S5/S7/S9/S10) | **DETECTED** (36) |
| H1 | usunięcie tłumienia grace w `if (latched)` (bez `recovery_wait` — hold NIE zerowany w WAIT) | fw112_mutH1 | `S13: WAIT_FRESH_LOAD grace suppressed (hold 5600 -> 0)` + 4 (hold, floor, target, setpoint — wyciek odtworzony dokładnie: `rec=1 session=1 hold=5598 demand=0 live=14 setpoint=14`) | **DETECTED** (5) |
| H2 | usunięcie `|| recovery_wait` z warunku `force_zero_reference` (krawędź rearm po WAIT) | fw112_mutH2 | `S13: MS.i_q_setpoint is 0 on the exact same tick WAIT_FRESH_LOAD && mode demand == 0` — padło DOKŁADNIE na `MS.i_q_setpoint == 0 same-tick` (`hold=0 demand=0 live=0 setpoint=13` — zanikające odniesienie po FW-037) | **DETECTED** (1, ta sama-tick asercja) |

Po każdej mutacji czysta produkcja → `fw112_run_rearm_recovery_host.exe`: **ALL PASS (S1–S15)**.
H1 i H2 to jedyne mutacje, które targały **kodem produkcyjnym naprawy** (nie automatem): obie
przywrócone, a po przywróceniu pełna suita S1–S15 znów PASS. Patrz też sekcja 7 (kompatybilność
pre-FW112).

## 5. Implementacja (v2)

### `inc/torque_input.h`

- `torque_recovery_state_t` (IDLE / WAIT_FRESH_LOAD / TRACK_FAST) i prototypy:
  `torque_input_begin_rolling_rearm()`, `torque_input_cancel_rolling_rearm()`,
  `torque_input_recovery_active()`, `torque_input_recovery_run_native()`,
  `torque_input_recovery_state()`.
- `TORQUE_ROLLING_REARM_STABLE_MS = 4 * TORQUE_ASSIST_FILTER_MS` (patrz sekcja 2).

### `src/torque_input.c`

- `torque_input_begin_rolling_rearm()`: `seed_run(bieżący FAST)` + stan `WAIT_FRESH_LOAD`.
- `torque_input_run_filter_step()`: w `WAIT_FRESH_LOAD` re-seeduje RUN do bieżącego FAST na
  każdy krok forward (stare okno nie przetrwa rearm).
- `torque_input_update()`: automat — WAIT_FRESH_LOAD→TRACK_FAST gdy FAST ≥ deadband; TRACK_FAST
  publikuje `run = FAST` co tick; TRACK_FAST→IDLE po `STABLE_TICKS` (genuine plateau) z
  pojedynczym `seed_run()`; TRACK_FAST→WAIT_FRESH_LOAD gdy FAST spadnie pod deadband.
- `torque_input_recovery_run_native()` zwraca bieżący FAST (stale-sample fix).

### `src/ride_control.c`

- W ticku `fast_rearm_this_tick`: `torque_input_begin_rolling_rearm()` + `rearm_permission_active
  = true`.
- Gdy sesja przestaje być ACTIVE (`!latched`): `torque_input_cancel_rolling_rearm()`. To jedyna
  krawędź poza ukończonym odzyskiwaniem, która zamyka automat.
- Podczas `torque_input_recovery_active()`: w kalkulacji mode na `torque_run_filtered` podstawiane
  `torque_input_recovery_run_native()` (FAST), a nie ostatni RUN z okna.

## 6. Pomiar kosztu (ARM)

Pełny link (wszystkie 59 źródeł + startup, arm-none-eabi-gcc 13.2.1, -O0, gc-sections, BL820;
dokładne wymiary **top-level sekcji** z mapy linkera, po --gc-sections):

| sekcja | DIAG=0 (0.0348) | DIAG=1 (0.0349) |
|--------|-----------------|-----------------|
| `.vectors` | 304 | 304 |
| `.text` | 90932 | 116028 |
| `.rodata` | 6600 | 6704 |
| `.ARM` | 8 | 8 |
| `.init_array` | 4 | 4 |
| `.fini_array` | 4 | 4 |
| `.data` (load address we FLASH) | 260 | 260 |
| `.bss` | 8612 | 20500 |
| `.heap_stack` | 3072 | 3072 |

**FLASH — pełny zajęty span od ORIGIN(FLASH) = 0x08005000 (początek `.vectors`) do końca load
image (`__flash_image_end = LOADADDR(.data) + SIZEOF(.data)`):**
- DIAG=0: `0x08005000 → 0x0801CF40` = **98112 B**
- DIAG=1: `0x08005000 → 0x080231B0` = **123312 B**

(Pełny load image == 230 KiB przestrzeń aplikacji od ORIGIN; span liczony do symbolu
`__flash_image_end` z mapy. Konwencja ta sama dla obu wariantów. Binarki flashowalne są o
36 B większe niż span z mapy — objcopy dopasowuje długość wyjścia (`0.0348_M820_BL820.bin`
= 98 148 B, `0.0349_M820_BL820.bin` = 123 348 B); poprzednio podane „text+rodata+data =
97808/122752" było osobnym, nieporównywalnym wzorem bez `.vectors`/`.ARM`/`.init_array`/
`.fini_array` — skorygowane do powyższego spójnego wzoru.)

**RAM:**
- `.data + .bss` (statyczne dane robocze): DIAG=0 = **8872 B**, DIAG=1 = **20760 B**
- `.data + .bss + .heap_stack` (z zarezerwowanym stosem): DIAG=0 = **11944 B**, DIAG=1 = **23832 B**

Symbole FW-112 v2 (z mapy): `torque_input_begin_rolling_rearm` 0x2c, `cancel_rolling_rearm`
0x24, `recovery_active` 0x20, `recovery_run_native` 0x18, `recovery_state` 0x18 (48–44 B razem);
bss: `recovery_state` 1 B, `recovery_stable_ticks` 2 B (torque_input.c), `rearm_permission_active`
1 B (ride_control.c) — 4 B RAM łącznie na całą automatykę odzyskiwania. FLASH 0x39800 (235520 B)
zajęte w ~42–52% (span/235520), RAM 48 KiB w ~18–42% (data+bss) lub ~24–48% (z heap+stack) —
bez presji na budżet. `inc/diag_budget.h` bez zmian
(torque_input/ride_control nie są modułami DIAG; ich stan nie wchodzi do 12 KiB budżetu DIAG).

Mapy linkera zachowane jako dowód pomiaru (świeże buildy po tej sesji: `0.0348` DIAG=0 i `0.0349`
DIAG=1, BL820): `%TEMP%\opencode\fw112_maps\0.0348.map` i `0.0349.map`. Wersje 0.0346/0.0347
(poprzednia para) też zachowane w tym samym katalogu. `arm-none-eabi-size` dla 0.0348 (DIAG=0):
text 97844 / data 268 / bss 11684 / dec 109796. Binarki flashowalne: `0.0348_M820_BL820.bin`
= 98 148 B, `0.0349_M820_BL820.bin` = 123 348 B (bin = objcopy-dopasowany load image; span
z mapy = 98 112 / 123 312 B, patrz wyżej — różnica 36 B to alignment wyjścia objcopy). RAM
(data+bss, a z heap+stack) identyczny z poprzednią parą 0.0347/0.0346 (konwencja pomiaru bez
zmian); FLASH (span) różni się o ~240 B względem 0.0347/0.0346 z powodu bieżącego working-tree
(wersja string, przywrócenie H1/H2).

Dla obserwowalności scenariuszy S11/S12 dodano wyłącznie read-only getter
`ride_control_get_assist_hold_ticks()` (prototyp w `inc/ride_control.h`, implementacja w
`src/ride_control.c` po `ride_control_get_session_state`) — zwraca `static assist_hold_ticks`
jako `uint16_t`. **Zero nowego RAM** (nie dodaje żadnego stanu/bss). **Koszt FLASH w finalnym
obrazie ARM = 0 B**: w obu mapach (0.0348 i 0.0349) sekcja `.text.ride_control_get_assist_hold_ticks`
jest pod adresem 0x00000000 — tj. **discarded przez --gc-sections**, bo getter nie ma konsumenta
w firmware (w przeciwieństwie do żywego `ride_control_get_session_state` pod adresem 0x08018dd4).
Getter istnieje wyłącznie dla host harnessu i może być z niego wołany.

## 7. Kompatybilność pre-FW112 (sekcja audytowa 7)

Pytanie audytu: czy FW-112 v2 zmienił zachowanie **normalnej jazdy** względem ostatniego
baseline PRE-FW112 w git? Uruchomiono tę samą deterministyczną pipeline na dwóch drzewach
(baseline = commit `154479339808fad6078ae70965ce0480721b5998` — zweryfikowany jako pre-FW112:
`src/ride_control.c` na HEAD ma ZERO markerów rearm/recovery, brak `ride_session.c`/
`pas_direction.c`/`ride_session.h`, brak infrastruktury regression; current = working tree na tym
commicie, fingerprint diffu 4 kluczowych plików `d371e49f…`), ten sam instrument harnessów,
te same flagi gcc, scenariusze RUN_60/80/100/110/120 + CADENCE_RAMP_50_120, porównanie bajt po
bajcie:

| warstwa | wynik | szczegóły |
|---------|-------|-----------|
| **torque** (`torque_trace`: torque_fast, torque_run, load) | **IDENTYCZNE** | wszystkie 6 scenariuszy, każdy wiersz bajt-identyczny (24 001×5 + 56 001 wierszy) |
| **power** (`power_pipeline`: iq_request, torque, support, human/raw power, cadence_comp) | **IDENTYCZNE** | 16 z 17 kolumn bajt-identycznych we wszystkich 6 scenariuszach, w tym ilość sterująca `iq_request`, `torque_fast/run`, `raw_motor_power_w`, `support_pct` |
| power — jedyna rozbieżna kolumna `motor_power_w` (filtr) | **NIE jest FW-112** | `filter_motor_power()` czyta `config->power_rise/power_fall_filter_ms` (assist_modes.c:359–360); per-level domyślne dynamiki zmieniła **osobna, niezaksięgowana** karta „Calmer high assist levels: per-level dynamics defaults” (CHANGELOG), NIE FW-112 v2 — kierunek zgodny z wolniejszym filtrem (poziom 3: 150→220 ms rise, 375→500 ms fall) |
| **ride** (final iq/setpoint, `ride_control_pipeline`) | **NOT PROVEN** | harness v2 NIE kompiluje się na baseline: `ride_control_input_t` nie ma `safety_cut_non_direction` (harness:184); `ride_session.c`/`pas_direction.c`/`ride_session.h` nie istnieją na HEAD (zmiana kontraktu: ride_control.h +65, assist_dynamics.h +12). Argument uzupełniający (niezmierzony): dodatki FW-112 (automat odzyskiwania, sesja, force_zero_reference) są INERTNE w ciepłej nieprzerwanej jeździe (recovery NONE, sesja ACTIVE bez fast_rearm, force_zero_reference false — bo iq_target > 0), a warstwy torque+power są udowodnione identyczne; krawędź rearm/grace to **celowe** odchylenie (sama naprawa), pinowane przez S13. |

Wniosek: FW-112 v2 **nie zmienił** ilości sterujących normalnej jazdy (RUN, iq_request); jedyna
zmierzona różnica w pipeline jest celowa i pochodzi z innej karty. Warstwa ride nie została
porównana wprost z powodu zmiany kontraktu — zgłoszona uczciwie jako NOT PROVEN z argumentem
inercyjności + pinem S13.

## 8. Obszary NIETKNIĘTE (zakres chroniony)

- `src/pas_direction.c` — automat kierunku bez zmian.
- `src/ride_session.c` — to jest czwarty stan: w v2 to **jedyny właściciel** permission
  (fast rearm / cold-start split / normalizacja stanów) — wszystkie zmiany v2 opisane w sekcji 5.
- `src/main.c` — kolejność ticku bez zmian.
- Progi PAS/torque, `TORQUE_RUN_ATTACK_STEPS`, licznik `Backwards_counter`, limiter, ramki/schema
  FW-111 — bez zmian.
- `torque_input_coast_update()`/kalibracja — bez zmian.

## 9. Uwagi i ryzyka

- Automat odzyskiwania to wyłącznie dodatek cyklu życia rearm; w normalnej jeździe ACTIVE bez
  reverse jest nigdy nieotwierany (stan = IDLE), więc zachowanie FW-085 jest identyczne —
  potwierdzone przez regression pipeline.
- Świeża-bramka (`FAST ≥ TORQUE_ASSIST_DEADBAND_NATIVE`) + powrót do WAIT_FRESH_LOAD przy
  utracie nacisku gwarantują, że długie oczekiwanie przy zerowym nacisku nie konsumuje budżetu
  odzyskiwania, a re-seed utrzymuje RUN uczciwie na ~0.
- Pozostały dług infrastruktury (bez związku z FW-112): pełna suita `run-host-tests.ps1`
  buduje exe do `$env:TEMP`, które Windows Defender kwarantannuje
  (`Trojan:Win32/Wacatac.B!ml`) — **BLOCKED BY DEFENDER**. Dotknięte harnessy (FW-112 z S1–S15,
  FW-109 integracja, FW-111 pełny łańcuch, main wiring) zweryfikowane indywidualnie z buildem do
  katalogu repozytorium (repo-local) — wszystkie PASS; `run_regression.ps1` PASS; ARM/link/map
  PASS (patrz sekcja 6). Problem infrastruktury pozostaje do osobnego rozwiązania — w tej sesji
  nie zastosowano żadnego obejścia (bez wykluczeń, bez admina, bez zmiany TEMP skryptu).
- Higiena komentarzy (bez zmian kodu): `sample_tick` w `inc/rider_input.h` i `src/main.c:2358`
  — FW-112 v2 przeniósł gwarancję świeżości do automatu odzyskiwania torque_input; pole nie ma
  już konsumenta, komentarz opisuje to uczciwie (pole zostaje jako kotwica obserwowalności, nie
  usuwane); `inc/torque_input.h` `cancel_rolling_rearm` — „ONLY edge that ends the automaton"
  poprawione na „forced exit for terminal session edges" (automat ma też dwa naturalne wyjścia:
  stable-completion → IDLE i collapse → WAIT_FRESH_LOAD).

## 10. Uruchomienie

- Harness: `powershell -File tests/host/run-host-tests.ps1` (suita FW-112) lub build gcc
  wg sekcji 3 z wyjściem poza `$env:TEMP`.
- Regression: `powershell -File tests/host/run_regression.ps1`.
- Build ARM: `powershell -File build_firmware.ps1 -BootloaderMode 820 -ArtifactName 0.0348`
  (+ `-CanDiagnostics` dla DIAG=1).