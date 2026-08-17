# TORQUE_SENSOR — `src/torque_input.c`

**PURPOSE** Jedyny właściciel toru czujnika nacisku: surowe mV → auto-zero → skorygowana
wartość → dwa równoległe filtry (FAST 35 ms, RUN wg kąta korby) → kg. Moduł czysty
(zero MS/MP), już testowalny na hoście.

**INPUTS** `torque_input_correct(raw_native)` → skorygowana wartość;
`torque_input_update(raw_native, corrected, sensor_valid)` — wołane co tick;
`torque_input_run_filter_step()` — wołane raz na KAŻDY krok kwadratury naprzód (patrz
`PAS.md`), NIE co tick; `torque_input_coast_update(...)` — auto-rezero na coast.

**OUTPUTS** `torque_snapshot_t` (`torque_input_get_snapshot()`): `raw_native`,
`corrected_native`, `assist_delta_native`, `assist_delta_filtered_native` (FAST),
`assist_delta_run_native` (RUN), `load_centikg`.

**STATE** Statyki pliku: `offset_correction` (auto-zero), `assist_filter_q` (FAST, Q8),
`run_buffer[96]`/`run_sum`/`run_head` (RUN, bufor kołowy), FSM kalibracji użytkownika
(`cal_state`), liczniki coast/fault.

**TIMEBASE** DWA różne systemy w jednym module:
- FAST filter: ticki × `TORQUE_INPUT_TICKS_PER_MS=4` (zakłada 4 kHz WYWOŁAŃ, kategoria B —
  patrz `../architecture/TIMEBASES.md`, zmierzone: 247 vs 86 przy 140 vs 35 wywołaniach).
- RUN filter: KROKI KORBY (3.75° każdy, okno domyślnie 180°=48 kroków), NIEZALEŻNE od
  cadence — kategoria "kąt", wzorcowo poprawne (FW-085, naprawiło pulsowanie assist raz
  na nogę przy niskiej kadencji).

**INVARIANTS** `TORQUE_ZERO_TARGET_NATIVE=740` (mV native). Domyślna krzywa kg jest
łamana (piecewise): 146 native=6 kg, 1580 native=84 kg. Kalibracja użytkownika (jeśli
aktywna) zamienia to na liniowy zakres 60 kg. RUN śledzi FAST 1:1 gdy okno=0 (wyłączone).

**TEST SEAMS** `tests/host/torque/torque_trace_host.c` (L1, tylko ten moduł, TEST-001) i
`tests/host/torque/torque_revolution_bench_host.c` (L1, rewolucja-okno, TEST-002) — oba
łączą się z tym samym prawdziwym modułem. `tests/host/pipeline/power_pipeline_host.c`/
`ride_control_pipeline_host.c`/`power_revolution_bench_host.c` (dalej w pipeline).
Scenariusze RUN_60..RUN_120 używają IDENTYCZNEGO profilu nacisku wg kąta — patrz
`../testing/REGRESSION_SCENARIOS.md`.

TEST-001 zmierzył (stałe 6 s): `torque_run` ripple malejący 0.098→0.028 (60→120 rpm) i
SFORMUŁOWAŁ hipotezę, że to artefakt stałego czasu obserwacji, nie wada filtra — ale nie
sprawdził tego empirycznie. **TEST-002 to POTWIERDZIŁ empirycznie**
(`documentation/TEST_002_HIGH_CADENCE_BENCHMARK_REPORT_PL.md`, sekcja 9): przy IDENTYCZNEJ
liczbie zmierzonych obrotów (20, po 8 obrotach rozgrzewki) w każdej kadencji, ripple jest
PRAKTYCZNIE STAŁY na całym zakresie 60-120 rpm (0,0217-0,0248, różnica <15% i bez trendu
z kadencją) — wobec 0,098→0,028 (różnica 3,5×, silny trend) przy metodzie stałego czasu.
Hipoteza TEST-001 była WŁAŚCIWA; okno FAST/RUN samo w sobie jest kątowe/czasowe jak
opisano wyżej i nie zależy od kadencji, gdy porówna się tę samą liczbę obrotów.

**RELATED SOURCE FILES** `src/torque_input.c`, `inc/torque_input.h`.

**KNOWN ISSUES** `src/torque_input.c` NIE kompiluje się czysto pod `-Wall -Wextra -Werror`
bez wyjątku `-Wno-type-limits` (linie ~242/248, porównanie `uint16_t < 0U` zawsze fałszywe
— martwe pod `TORQUE_RUN_ATTACK_STEPS=0`, ożywa gdy ktoś ustawi tę stałą >0). Udokumentowany
wyjątek w `tests/host/run_regression.ps1`, NIE naprawione (poza zakresem karty). Patrz
`../TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md` finding testability-1.
