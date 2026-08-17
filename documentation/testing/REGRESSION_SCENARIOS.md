# REGRESSION_SCENARIOS — pełna lista, status implementacji

**PURPOSE** Rejestr wszystkich planowanych scenariuszy regresji. Tylko P0 (i część P1) są
zaimplementowane w tej karcie — reszta jest tu opisana, żeby przyszła karta miała gotowy
projekt zamiast projektować od zera.

Legenda `Poziom`: L1/L2/L3/L4 patrz `TEST_ARCHITECTURE.md`. `Status`: ✅ zaimplementowane
w tej karcie / 📋 opisane, nie zaimplementowane.

| Scenariusz | Priorytet | Poziom | Status | Inputs | Główne outputs | Invarianty (do przyszłej kalibracji) |
|---|---|---|---|---|---|---|
| RUN_60 | P0 | L2/L3 | ✅ | crank@60rpm, stały profil nacisku | torque_fast/run, human/motor power, iq_request(/final) | brak nagłego skoku między sąsiednimi cadence |
| RUN_80 | P0 | L2/L3 | ✅ | jw. @80rpm | jw. | jw. |
| RUN_100 | P0 | L2/L3 | ✅ | jw. @100rpm | jw. | jw. |
| RUN_110 | P0 | L2/L3 | ✅ | jw. @110rpm | jw. | jw. |
| RUN_120 | P0 | L2/L3 | ✅ | jw. @120rpm | jw. | jw. |
| CADENCE_RAMP_50_120 | P0 | L2/L3 | ✅ | rampa 50→120rpm/10s, ten sam profil | jw. + ciągłość podczas rampy | brak skoku w trakcie rampy |
| MISSED_TICK_BURST | P1 | L1 (torque_input, ride_episode) | ✅ | normalna vs "burst" gęstość wywołań, ta sama linia bazowa zdarzeń | t_latch/t_recover (ride_episode), FAST filter (torque_input), aliasing kroków PAS (ilustracyjne) | kategoria A: duration-from-anchor poprawny; kategoria B: rozjazd zmierzony, nie naprawiony |
| START_STANDSTILL | P1 | L3 | 📋 | crank od 0, load rosnący do progu, prędkość=0 | moment zazbrojenia latcha, iq_target | latch nie zazbraja przed progiem kg i przed liczbą kroków |
| ROLLING_RESTART | P1 | L3 | 📋 | prędkość>0, crank od 0 | jw., próg niższy (`riding_start_load_centikg`) | próg niższy tylko gdy speed≥1km/h |
| RELEASE | P1 | L3 | 📋 | pedałowanie ustaje | fade wg `release_ms` konfiguracji poziomu | czas do zera = dokładnie release_ms |
| BRAKE_CUT | P1 | L3 | 📋 | `safety_cut=true` (już pole w `ride_control_input_t` — patrz `../inputs/BRAKE.md`) | iq→0 w ≤200ms | RIDE_HARD_CUT_RAMP_MS niezmieniony |
| SPEED_LIMIT | P1 | L3 | 📋 | prędkość rosnąca do/powyżej limitu | taper 0.01km/h okno | monotoniczne wygaszanie |
| LOW_VOLTAGE | P1 | L3 | 📋 | `voltage_raw` spada poniżej progu | iq skalowane liniowo do 0 | brak skoku |
| BATTERY_CURRENT_LIMIT | P1 | L1 (main.c, niewydzielone) | 📋 | prąd baterii > limit | przełączenie regulacji PI | wymaga wydzielenia z main.c najpierw |
| TEMPERATURE_LIMIT | P1 | L3 | 📋 | temp 75→95°C | derating, kod błędu 10 | zgodność z progami TEMP_WARN/CUTOFF/CLEAR |
| MICROREVERSE | P1 | L1 (PAS decoder, niewydzielony) | 📋 | 1-2 kroki wstecz, gap krótki | `fwd_run` przeżywa, latch przeżywa | wymaga wydzielenia dekodera PAS z main.c |
| REVERSE_CONFIRMED | P0 (bezpieczeństwo) | L1 | 📋 | ≥3 kroki wstecz z rzędu | Backwards_counter≥4, hard_cut | jw. |
| TORQUE_RAMP | P2 | L2 | 📋 | load 0→60kg liniowo, stała kadencja | iq_request monotoniczny | brak przeskoku >1 kroku kwantyzacji |
| DEAD_SPOT | P2 | L1 | 📋 | profil z realnym minimum w martwym punkcie (generator już to umie: `dead_spot_depth_pct`/`dead_spot_width_deg`) | torque_run NIE pulsuje | wariancja RUN filter niska |
| THROTTLE | P2 | L3 | 📋 | throttle 0→max, bez pedałowania | klasyfikacja NON_PEDAL, taper 5-7km/h | throttle nigdy nie dostaje limitu pedałowego |
| PAS_GLITCH | P2 | L1 (PAS decoder, niewydzielony) | 📋 | pojedyncza tranzycja gap≤3 ticków | nie liczy się jako reverse | wymaga wydzielenia dekodera |
| SPEED_GLITCH | P2 | L1 (speed, niewydzielony) | 📋 | impuls >25km/h/s | odrzucony | wymaga wydzielenia Speed_processing |
| COAST_TORQUE_REZERO | P3 | L1 | 📋 | dłuższy coast, stabilny sygnał | offset korygowany max/coast | brak skoku offsetu |
| EXT_BOOST_HOLD | P2 | L1 | już ma test (`fw100_extended_boost_host.c`, sprzed tej karty) | — | — | — |

## Generator (`tests/host/common/crank_model.c`) — co już umie

`crank_torque_shape_t{mean_native_delta, ripple_pct, asymmetry_pct, dead_spot_depth_pct,
dead_spot_width_deg, phase_shift_deg}` + `crank_cadence_ramp()`. Wystarcza dziś dla:
RUN_*, CADENCE_RAMP, TORQUE_RAMP (zmieniając `mean_native_delta` w czasie zamiast
cadence), DEAD_SPOT (już parametryzowane). NIE wystarcza dla: MICROREVERSE/REVERSE_CONFIRMED/
PAS_GLITCH (wymagają symulacji cofnięcia kierunku — generator dziś zawsze idzie naprzód;
rozszerzenie jest małe: dodać znak do przyrostu kąta), SPEED_GLITCH (osobny generator
zdarzeń EXTI, nie crank model).

**RELATED SOURCE FILES** `tests/host/common/crank_model.h/.c`, wszystkie pliki w
`tests/host/torque/`, `pipeline/`, `scenarios/`.

**KNOWN ISSUES** 14 z 21 scenariuszy jest tylko opisanych (📋), zgodnie z kartą (sekcja 17:
"nie próbuj implementować wszystkich 20 scenariuszy w jednym zadaniu"). REVERSE_CONFIRMED
ma najwyższy realny priorytet bezpieczeństwa spośród niezaimplementowanych — kandydat na
następną kartę razem z wydzieleniem dekodera PAS (bez wydzielenia, ten scenariusz
wymagałby duplikowania logiki dekodera w teście, co audyt już odradza).
