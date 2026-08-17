# TEST_ARCHITECTURE — warstwy L1-L4 i golden-trace design

**PURPOSE** Jak testy w tym repo są warstwowane, co istnieje dziś, i jak jest
zaprojektowany (ale świadomie NIE zaimplementowany) mechanizm golden traces.

> **Aktualizacja (TEST-002, `../TEST_002_HIGH_CADENCE_BENCHMARK_REPORT_PL.md`):** do
> porównań MIĘDZY kadencjami (np. "czy zachowanie zmienia się z 60 na 120 rpm") NIE
> używaj już metody stałego czasu (`torque_trace_host.c`/`power_pipeline_host.c` z tej
> karty, tryb domyślny) — TEST-002 empirycznie potwierdził, że stałe okno czasowe daje
> różną liczbę obrotów na różnych kadencjach i to SAMO W SOBIE zmienia zmierzony ripple
> (3,5× rozrzut, bez prawdziwej przyczyny w filtrach). Użyj
> `torque_revolution_bench_host.c` / `power_revolution_bench_host.c` (tryb REV20: stała
> liczba zmierzonych obrotów po rozgrzewce) zamiast tego. Metoda stałego czasu pozostaje
> poprawna do pojedynczych, nieporównawczych pomiarów (np. RUN_100 w
> `run_regression.ps1`'s determinism smoke-test).

## Warstwy

**L1 UNIT** — pojedynczy moduł/funkcja, host-native, linkuje PRAWDZIWY plik `.c`.
Istniało przed tą kartą: `tests/host/fw100_extended_boost_host.c`,
`fw101_episode_host.c`, `fw102_pas_trace_host.c` (pass/fail, `check()`/`failures`).
Ta karta dodała: `tests/host/torque/torque_trace_host.c` (traces, nie pass/fail).

**L2 CONTRACT** — granica moduł↔moduł. Nie ma dziś osobnej formy (żadnego
`assert(iq_request>=0)`-stylu testu kontraktu) — kontrakty są dziś tylko udokumentowane
w nagłówkach (`inc/*.h`) i w `documentation/assist/*.md`. Kandydat na przyszłą kartę.

**L3 PIPELINE** — kilka modułów razem, kończy się na `motor_command_t`. Ta karta dodała
DWA harnessy o różnej głębokości (patrz `TEST_INTERFACES.md` po uzasadnienie, dlaczego
osobne, nie jeden):
- `tests/host/pipeline/power_pipeline_host.c` — torque_input → rider_input →
  assist_modes. Kończy na `iq_request` (przed ride latch/rampą/limiterem).
- `tests/host/pipeline/ride_control_pipeline_host.c` — jw. + ride_control (latch,
  Extended Boost, limits, smooth start, dynamics) + motor_core. Kończy na `iq_final`
  (`MotorState_t.i_q_setpoint`).

**L4 HARDWARE/LOG** — prawdziwy sterownik/silnik/logi z jazdy. Nie zaimplementowane w
tej karcie (karta zabrania budowy fizycznego stanowiska — sekcja 16). Projekt patrz
`../ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md` sekcje 17-18, oraz "Real-log replay" niżej.

## Granica deterministic-software-test

Kończy się na `motor_command_t`/wejściu `motor_core` (karta, sekcja 4). Wszystko od
`src/FOC.c` w dół (Hall, PWM, dynamika silnika/mostka/baterii) wymaga modelu fizycznego
lub HIL — nie próbowano tego "uprościć" do unit testu, bo test nie powiedziałby nic
prawdziwego o zachowaniu silnika.

## Golden-trace design (projekt, NIE zaimplementowane jako "zatwierdzony golden")

`tests/host/golden/candidates/` istnieje i zawiera `metrics_summary.csv` z BIEŻĄCEGO
przebiegu tej karty — to jest KANDYDAT, nie zatwierdzony golden. Docelowy przepływ:

```
scenariusz --> stary firmware --> trace + metryki (candidates/<data>/...)
scenariusz --> nowy firmware  --> trace + metryki
                                        |
                                        v
                          Compare-Traces (tools/RegressionTools.ps1)
                                        |
                                        v
                    SAME/DIFFERENT per warstwa + FIRST DIVERGENCE
```

**Golden ≠ bit-identyczny.** Tolerancje per warstwa (przykładowe, do kalibracji na
realnych danych z jazdy zanim staną się progami PASS/FAIL — ta karta ich NIE ustawia
agresywnie, zgodnie z sekcją 12 karty):

| Warstwa | Tolerancja (wstępna, do weryfikacji) |
|---|---|
| torque_raw/corrected | ~0 (deterministyczna arytmetyka całkowita) |
| torque_fast/run | ±1-2 jednostki natywne (zaokrąglenie filtru) |
| human/motor_power_w | ±1 W lub ±1% |
| iq_request/iq_final | ±1 jednostka Iq (punktowo), lub porównanie CZASU dojścia do
  wartości ±5% dla `iq_final` (rampa — porównanie tick-po-ticku jest zbyt kruche, patrz
  `../assist/ASSIST_DYNAMICS.md`) |

**Zatwierdzanie golden jest RĘCZNE.** Żaden skrypt w tej karcie nie nadpisuje
`golden/candidates/` automatycznie na podstawie wyniku testu — `run_regression.ps1`
kopiuje bieżące metryki tam PRZY KAŻDYM URUCHOMIENIU jako "co jest teraz", ale nic nie
oznacza tego jako "zatwierdzone". Przyszła karta powinna dodać osobny katalog
`golden/approved/<scenario>/` wypełniany wyłącznie ręczną decyzją (np. `git add` po
przeglądzie), nigdy przez ten skrypt.

## Real-log -> regression (projekt, NIE zaimplementowane)

Rozróżnienie wymagane przez kartę (sekcja 15):

**A. LOG-DERIVED APPROXIMATE SCENARIO** — dzisiejsze logi CAN (0x3200/0x3201 itd., ~poll
280-1500 ms) dają cadence/load/speed/voltage jako przybliżenie do budowy scenariusza
(parametry generatora `crank_model`), NIE surowy przebieg 4 kHz. Wykonalne dziś, bez
zmian firmware.

**B. EXACT RAW INPUT REPLAY** — wymaga przyszłego rejestratora surowego PAS/torque z
rozdzielczością 4 kHz (analogicznie do `pas_trace.c`, ale ciągły, nie tylko wokół
podejrzanego zdarzenia). NIE istnieje dziś. Diagnostyka deweloperska
(`CAN_DIAGNOSTICS_ENABLE=1`) zbliża się najbardziej (ramki `0x000102xx`), ale jest
domyślnie wyłączona i nie loguje throttle/brake/CAN-HMI w ogóle.

## Uruchomienie

```
powershell -File tests/host/run_regression.ps1
```

Buduje wszystkie harnessy (bez toolchaina ARM), uruchamia P0 scenariusze, liczy metryki,
wykonuje smoke-test determinizmu (dwa niezależne przebiegi RUN_100 muszą dać IDENTYCZNY
wynik przez `Compare-Traces` — jeśli nie, PROBLEM z samą infrastrukturą, nie z firmware),
zapisuje `tests/host/out/REGRESSION_RESULTS.md`.

Istniejące `tests/host/run-host-tests.ps1` (FW-100/101/102, pass/fail) pozostaje
osobne i niezmienione — to nie jest podmiana, to rozszerzenie.

**RELATED SOURCE FILES** `tests/host/run_regression.ps1`, `tests/host/tools/RegressionTools.ps1`.

**KNOWN ISSUES** Brak realnych baseline'ów historycznych do porównania (ta karta jest
PIERWSZYM uruchomieniem — nie ma "starego firmware" do zestawienia, tylko determinizm
tego samego build'u ze sobą). Patrz `../TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md`.
