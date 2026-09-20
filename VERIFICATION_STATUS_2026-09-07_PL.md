# EVistDrive v3 — FW145 + ap2_torque_chain checkpoint

**Baseline wejściowy:** `EvistDrive06092026v3.zip`
**Linia rozwoju:** FW139 -> FW145 + ap2_torque_chain
**Cel checkpointu:** samowystarczalny projekt do dalszej pracy przez kolejnego agenta, z real-module tests, whole-pipeline regression, supervisory SIL, electrical FOC/PMSM/Hall/QZERO SIL oraz pełnym Walk Assist SIL.
**Stan:** PC/SIL VERIFIED; exact ARM target build DONE (0.610 NORMAL / 0.611 DIAG); Arm GNU 13.2.1 available.

## 0. Nowe w tym checkpointie (ap2_torque_chain)

Zastąpiono `ap2_rider_demand.c` modułem `ap2_torque_chain.c`:
- **6-state engagement gating** (RESET→GATE→CONFIRM→ACTIVE→HYSTERESIS→RECOVERY) zamiennie z `ap2_pas_state.c`
- **Base/dynamic split** przeniesiony z poprawioną logiką ACTIVE state (wychodzi tylko na `!in->pedaling`, śledzi demand poniżej `active_threshold` zamiast zerować)
- **Elapsed-time LPF** (20 ms) dla effort, crank-angle window dla base term (1.5 udaru)
- Naprawione: base term nie kolapsuje między udarami (S1), ceiling śledzi referencję przy block_positive (S17)
- Wszystkie 56 testów głównych PASS, regresja deterministyczna PASS

## 1. Zmiany produkcyjne zachowane z FW139–FW142

### FW139 — PAS
- fizycznie niemożliwe krótkie reverse bounce nie trafia do direction safety;
- prawdziwy reverse nadal natychmiast odbiera permission.

### FW139 — START
- usunięty Hall-gated Gear Preload jako drugi owner normalnego startu;
- finalny `fast_iq_slew` pozostaje jednym właścicielem trajektorii Iq.

### FW140 — cadence
- control używa jednej conditioned cadence;
- raw cadence pozostaje do telemetry/diagnostyki;
- A/B w SIL zmniejszył peak-to-peak Iq z 71 do 19 counts w scenariuszu nierównej korby.

### FW141 — torque timebase
- FAST/RUN filtering jest zależne od rzeczywistego `elapsed_ticks`, nie od liczby wywołań foreground;
- missed-tick regression zachowuje identyczny wynik 35-ms FAST filtra.

### FW142 — testowalny current loop / electrical SIL
- PI Id/Iq + wspólny vector limiter wydzielone do jednego production module `foc_current_loop.c`;
- target firmware i SIL wykonują ten sam kod;
- parity test legacy-vs-helper: 2000 losowych stanów PASS;
- real `FOC.c`, inverse Park, SVPWM, PMSM, physical Hall, `rotor_angle`, QZERO są wykonywane w electrical SIL.

## 2. FW143 — Walk Assist 10–60 rpm

Kontrakt produkcyjny:

```text
normal target: 10..60 chainring/output rpm
default:       30 rpm
70 / 80 / >80: invalid/range/overspeed tests, nie normalne targety Walk
```

Zakres jest scentralizowany w `inc/config.h` i konsumowany przez bank/config validation oraz `walk_assist_motor.c`.

Nie wymagamy laboratoryjnego servo-speed. Niewielkie pływanie pod małym obciążeniem jest metryką jakości, nie FAIL. Twarde kryteria to brak runaway, poprawny start/lifecycle, bezpieczny stall, zero na release/brake/fault/wheel-cut i respektowanie ceiling Iq.

### Full Walk electrical matrix

Real production path:

```text
walk_assist_motor.c
 -> walk_speed_controller.c
 -> final Iq path
 -> real FOC.c / PI / vector limit / SVPWM
 -> virtual PMSM
 -> physical Hall edges
 -> production Hall feedback
 -> Walk governor
```

Macierz deterministyczna:

```text
7 targetów: 10 / 15 / 20 / 30 / 40 / 50 / 60 rpm
3 obciążenia
6 pozycji startowych Hall
= 126 przypadków
```

Wynik:

- **126 / 126 safety/lifecycle PASS**;
- `failures=0`;
- `safeStalls=2` — ciężkie przypadki kończą w bezpiecznym stall zamiast runaway;
- `trackingWarnings=23` — lekkie obciążenie/niskie rpm może pływać; warning, nie FAIL;
- worst first Hall ~206 ms w macierzy Walk;
- invalid targets `0, 9, 61, 70, 80, 100` nie stają się ukrytym targetem Walk i wracają do bezpiecznego default 30 rpm.

## 3. Aktualny gate FW143

### Real-module host suites
- **69 / 69 PASS**.

### Whole-pipeline deterministic regression
- **18 / 18 traces PASS**;
- missed-tick regression PASS;
- repeat RUN_100 byte-identical PASS.

### Fast supervisory closed-loop SIL
- clean/loaded start PASS;
- PAS bounce: 0 false reverse;
- cadence A/B: 71 -> 19 Iq p-p;
- STOP -> restart PASS, first restart Iq = 1 count;
- deterministic fuzz: **10 000 / 10 000 PASS**;
- ASan/UBSan fuzz: **1 000 / 1 000 PASS**.

### Real electrical FOC/PMSM/Hall/QZERO SIL
- real SVPWM geometry sweep through `_U_MAX=1920`: PASS;
- locked 6-sector current tests PASS;
- moving current tracking 50..220 erps PASS;
- Hall start sweep: **24 angles x 2 loads = 48 / 48 PASS**;
- worst normal permission->first Hall ~45 ms;
- initial 3-Hall uncertainty bounded at ~30 electrical degrees;
- full electrical deterministic fuzz: **1 000 / 1 000 PASS**;
- ASan/UBSan electrical fuzz: **100 / 100 PASS**;
- QZERO branch executes; current virtual plant uses safe abort rather than forcing an unproven handback.

### Build/packaging infrastructure
- source manifest: **55/55 production C files listed; 81 total build entries**;
- target-tree/startup/linker/CMSIS/HAL self-check PASS;
- BL820 packager independent CRC/container regression PASS;
- cross-platform builder: `tools/build_firmware.py`;
- Windows one-command gate/build: `VERIFY_AND_BUILD_WINDOWS.bat`.

## 4. Co znajduje się w tej paczce

Paczka jest przygotowana jako nowy workspace projektu i zawiera:

- pełne production `src/` + `inc/`;
- GD32 HAL/CMSIS, startup i linker;
- build/source manifests;
- host tests, whole-pipeline regression i oba SIL-y;
- protocol/config schema;
- pełną historię `.git` tej linii rozwoju;
- `START_HERE.md`, `AGENTS.md`, aktualną mapę architektury i test contract FW143;
- wybrane kluczowe dokumenty reverse G532/Fake Taxi/TSDZ2/M820 w `docs/reference/`.

Nowy agent nie powinien potrzebować wcześniejszych czatów do zrozumienia bieżącej architektury.

## 5. Czego paczka celowo NIE zawiera

Nie bundlujemy zewnętrznego toolchainu wykonywalnego. Do finalnego target build potrzebny jest:

```text
Arm GNU Toolchain arm-none-eabi 13.2 Rel1
GCC 13.2.1
```

Ten wymóg istniał już w oryginalnym build systemie v3.

Do PC verification potrzebne są Python 3 + host C compiler. Git jest potrzebny do normalnej dalszej pracy z historią.

## 6. Jak rozpocząć nowy projekt / nowego agenta

Pierwsza instrukcja dla agenta:

```text
Przeczytaj START_HERE.md oraz AGENTS.md. Traktuj tę paczkę jako authoritative EVistDrive baseline.
Uruchom quick gate przed zmianami. Nie przywracaj historycznych workaroundów bez reprodukcji i testu.
```

Potem:

```bash
python tools/verify_all.py --quick
```

Pełny PC gate:

```bash
python tools/verify_all.py
```

Przed wygenerowaniem kandydata do flashowania:

```bash
python tools/verify_all.py --require-target
```

## 7. Nadal wymagające sprzętu

SIL nie zastępuje ostatniego gate na realnym M820. Po exact target build trzeba zweryfikować przede wszystkim:

1. brak start deadlock/opóźnienia pod realnym obciążeniem;
2. brak ciągłego pump/szarpania w 20..120 rpm;
3. true reverse cut;
4. końcowy STOP click/QZERO na prawdziwej mechanice;
5. ARMED_ZERO restart;
6. Walk 10..60 rpm pod rzeczywistym obciążeniem — drobne pływanie akceptowalne, runaway/stall safety nie;
7. rzeczywisty runtime bank/config dump.

Nie stroić FOC/QZERO/Walk pod parametry wirtualnego PMSM bez potwierdzenia sprzętowego.

---

# FW144 UPDATE — LEVEL 4 VIRTUAL BIKE / BATTERY / SOC / REPLAY

Ta sekcja nadpisuje liczby FW143 tam, gdzie projekt ma już dodatkowe moduły/testy.

## Produkcyjna zmiana testowalności SOC

Matematyka SOC została wydzielona z `main.c` do produkcyjnego `src/soc_core.c`. Ten sam moduł jest linkowany
przez firmware i Level 4. Nie jest to nowy algorytm SOC: host parity porównuje OCV, limp i 20 000 losowych
przejść 1 Hz ze starą implementacją — PASS.

## Level 4

Pętla obejmuje wirtualnego rowerzystę, torque/PAS, produkcyjny supervisory control, realny final-Iq/FOC/PI/SVPWM,
wirtualny PMSM, rower/drogę/nachylenie oraz fizyczny model baterii z napięciem, R0, dynamic sag i true SOC.

Fixed routes: **9/9 PASS** (0/5/10/15% grade, cadence do 120 rpm, SOC 90/80/50/30/20/10/5%, high sag,
matched i mismatched chemistry).

SOC endurance:
- start 100/90/80/60/40/20/10/5%;
- matched LG profile: max abs error ~0.19% — PASS;
- FEB21700G test profile: mismatch produkcyjnej OCV mapy dochodzi do ~14% w części zakresu — evidence/open,
  nie ukryty PASS i nie automatyczna zgoda na zmianę algorytmu.

Level-4 randomized physics:
- **100/100 PASS**;
- `safePhysicalStalls=34` — rider fizycznie nie ma wystarczającego momentu;
- `marginalStarts=10` — zapas statyczny 2..15%, osobna klasa;
- robust start (>15% zapasu momentu) wymaga normalnego assist/Hall start;
- Level-4 ASan/UBSan fuzz **25/25 PASS**.

## Real-ride replay

Dodano:
- `tools/import_ride_log.py` — alias/mapping -> canonical CSV;
- `tools/run_replay.py` — replay sensor history przez produkcyjne moduły C;
- `tools/run_replay_regression.py` — deterministic smoke + registered hardware cases;
- `tools/register_ride_case.py` — zapis realnego problemu jako permanent regression;
- `protocol/EVISTDRIVE_LIVE_RIDE_LOG_CONTRACT.md` — kontrakt loggera;
- `tools/analyze_l4_trace.py` — szybki raport amplitud/SOC/replay delta.

Smoke na istniejącym `RUN_60_ride.csv`: **24 000 / 24 000 rows**, repeated replay byte-identical PASS.
Stary recorded output różni się od bieżącego kodu (max `Iq_ref` delta 124 counts), co jest raportowane zamiast
maskowane; trace nie zawiera V/I/ERPS, więc replay jawnie zgłasza brakujące kanały/defaulty.

## Aktualny host/build gate po FW145 + ap2_torque_chain

- source manifest: **55/55 production C**, 81 total entries — PASS;
- real-module host suites: **56/56 PASS**;
- whole-pipeline: **18/18 PASS**, missed-tick + determinism PASS;
- supervisory closed-loop fuzz: **10 000/10 000 PASS**;
- supervisory ASan/UBSan: **1000/1000 PASS**;
- electrical real-FOC/PMSM/Hall fuzz: **1000/1000 PASS** (deterministic non-overlapping shards);
- electrical ASan/UBSan: **100/100 PASS**;
- Hall-start sweep: **48/48 PASS**;
- Walk real-FOC/PMSM/Hall matrix: **126/126 safety/lifecycle PASS**;
- QZERO STOP -> safe exit -> restart: PASS in the current virtual plant;
- Level 4 fixed routes: **9/9 PASS**;
- Level 4 randomized whole-bike physics: **100/100 PASS**, including 34 expected physical stalls and 10 marginal starts classified separately from firmware no-start;
- Level 4 ASan/UBSan: **25/25 PASS**;
- matched-chemistry LG SOC endurance: max abs estimator error ~**0.19%**;
- FEB21700G virtual-profile comparison: up to ~**14%** estimator mismatch in part of the SOC range — OPEN evidence for future real-data calibration, not a test failure;
- ride replay smoke: **24 000/24 000 rows**, repeated output byte-identical; registration/accepted-baseline workflow self-test PASS;
- BL820 build-tree/packager checks: PASS.

The integrated `verify_all.py` contains all of these gates. In this execution environment one monolithic full run exceeds the per-process wall-time while entering the heavy electrical stage, so the full stress stages were also executed separately and all results above are PASS. This is a tooling wall-time limitation, not a skipped test or hidden FAIL.

## Source of truth dla Level 4

Przeczytaj:
- `docs/FW144_LEVEL4_VIRTUAL_BIKE.md`;
- `sim/l4/README.md`;
- `sim/replay/README.md`;
- `protocol/EVISTDRIVE_LIVE_RIDE_LOG_CONTRACT.md`.

Nie przedstawiać parametrów PMSM/bike/rider/sag z Level 4 jako zmierzonych stałych M820. To plant testowy.

---

# FW145 UPDATE — LIVE CAN TELEMETRY FOR LEVEL 4

FW145 dodaje brakujący most pomiędzy prawdziwym M820 a przygotowanym w FW144 replayem Level 4.
Zmiana jest diagnostyczna: nie tworzy nowego ownera sterowania i domyślnie występuje tylko w
`diagnostic` buildzie.

## Firmware

- nowy `src/ride_telemetry.c` / `inc/ride_telemetry.h`;
- chroniony blok EFID `0x10400..0x10407`, rozłączny od STOP_TRACE `0x10300..0x10307`;
- około 47.6 spójnych snapshotów/s;
- snapshot budowany ~48 Hz, nie 4 kHz;
- żadnego CAN TX z FOC ISR 16 kHz;
- QZERO obserwowane wyłącznie przez ISR-owned read-only mirror — foreground nie czyta state machine;
- critical HMI queue/multiframe/dumpy mają pierwszeństwo;
- brak busy-wait i brak retry-flood po zaakceptowanym, ale nieudanym mailboxie.

Transmitowane obserwacje obejmują torque FAST/RUN/load, raw/control cadence, permission/debug/session,
Iq request/allowed/ref/actual, Id, ERPS, battery V/I/SOC, wheel speed, u_abs/limiter flags, QZERO,
theta/Hall/trust, lifecycle/PWM oraz stan PAS/direction.

## CANable / replay

`tools/decode_canable_ride_log.py` czyta bezpośrednio istniejący tekstowy format loggera CANable,
składa 7 ramek po wspólnym ticku i generuje rich decoded CSV + canonical replay CSV + metadata loss
report. Snapshot z brakującym fragmentem pozostaje jawnie niekompletny; czas nie jest kompresowany.
Do replay wymagane są CORE + STATE, pozostałe braki zostają `nan`/metadata.

`tools/register_canable_ride_case.py` potrafi zapisać surowy `.log`, decoded observations, canonical
input i po świadomej akceptacji bieżący output jako permanent regression.

Stary rzeczywisty log `log-2026-09-06-08-52-10-n0.log`: 4436 poprawnie sparsowane ramki, 0 ramek
fałszywie rozpoznanych jako FW145 telemetry — oczekiwane, ponieważ pochodzi sprzed FW145.

Nowy synthetic wire-to-replay gate: firmware schema -> raw CANable text -> decoder -> canonical CSV ->
production C replay: PASS. Host real-module suite po integracji: **56/56 PASS**.

Do fizycznego logowania należy zbudować `diagnostic` target (0.611):

```text
VERIFY_AND_BUILD_DIAGNOSTIC_WINDOWS.bat
```
Builduje wersję 0.611 (DIAG) z recorderami.

Exact `.bin` nadal wymaga Arm GNU GCC 13.2.1 na maszynie target-build.

**Ważne:** Do jazdy wgrywać wersję **NORMAL (0.610)**. Wersja DIAG (0.611) zawiera recordery i ma inny układ pamięci — nie jest przeznaczona do normalnej jazdy.
