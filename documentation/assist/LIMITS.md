# LIMITS — `src/assist_limits.c`

**PURPOSE** Wspólny limiter: napięcie, temperatura, prędkość (prawo/offroad). Jedna
funkcja, bezstanowa, wołana OSOBNO dla żądania pedałowego i dla throttle (żeby throttle
nigdy nie dziedziczył wyższego limitu pedałowego — FW-091).

**INPUTS** `assist_limits_apply(iq_request, assist_limits_input_t*)`:
`voltage_raw`/`voltage_min_raw`, `controller_temperature_c`, `source`
(`PEDAL_CONFIRMED`|`NON_PEDAL`), `speed_x100`/`speed_limit_x100`, `legal_enabled`,
`offroad`, `walk_active`.

**OUTPUTS** Ograniczony `iq_request` (int32_t).

**STATE** Brak — czysta funkcja.

**TIMEBASE** Brak — jeden tick, brak filtracji wewnątrz (rampowanie jest zadaniem
`assist_dynamics.c`, downstream).

**INVARIANTS** Trzy niezależne taper'y liniowe (`map()`): napięcie
`[voltage_min_raw, voltage_min_raw+176] → [0, iq_request]`; temperatura `[75,90]°C →
[iq_request, 0]`; prędkość — `PEDAL_CONFIRMED` używa `[speed_limit_x100,
speed_limit_x100+200]`, `NON_PEDAL` używa stały `[500,700]` (0.01 km/h) niezależnie od
skonfigurowanego limitu. `source=PEDAL_CONFIRMED` wymaga, żeby ride latch był aktywny
(dowód realnego pedałowania) — NIE cadence filtrowana (FW-091 naprawiło błędną
klasyfikację po przerwie w pedałowaniu).

**TEST SEAMS** Moduł już w pełni testowalny (deklarowany "motor-agnostic" w FW-094 po
usunięciu zależności od `main.h`). Ta karta linkuje go w
`tests/host/pipeline/ride_control_pipeline_host.c` z parametrami dobranymi tak, by ŻADEN
limiter nie był aktywny w bazowym przebiegu (napięcie 2000 raw wobec progu ~1496,
temperatura 25°C wobec progu 75°C, prędkość 15 km/h wobec limitu 25 km/h) — celowo, żeby
metryki RUN_60..120 pokazywały czysty tor torque/cadence, nie interakcję z limiterem.
Scenariusze LOW_VOLTAGE/TEMPERATURE_LIMIT/SPEED_LIMIT (`../testing/REGRESSION_SCENARIOS.md`,
P1) NIE zaimplementowane w tej karcie, ale wymagają zero zmian API — tylko innych wartości
w `assist_limits_input_t`.

**RELATED SOURCE FILES** `src/assist_limits.c`, `inc/assist_limits.h`.

**KNOWN ISSUES** `extern int32_t map(...)` deklarowane lokalnie (bez wspólnego nagłówka) —
audyt finding F4. Ta karta MUSIAŁA dodać `tests/host/common/map_adapter.c` (kopia
`main.c`'s `map()`) żeby móc zlinkować ten moduł bez `main.c` — patrz
`../testing/TEST_INTERFACES.md`.
