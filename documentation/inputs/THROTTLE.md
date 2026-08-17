# THROTTLE — manetka, `adc_value[1]`

**PURPOSE** Mapuje napięcie manetki na żądanie prądu niezależne od pedałowania, zawsze
klasyfikowane jako `ASSIST_LIMIT_SOURCE_NON_PEDAL` (niższy limit prędkości w trybie legal).

**INPUTS** `adc_value[1]`, `MP.throttle_offset`/`throttle_max` (kalibrowane automatycznie
na starcie w typowej konfiguracji).

**OUTPUTS** `ride_control_input_t.throttle_iq` (main.c: `map(adc_value[1],
throttle_offset, throttle_max, 0, phase_current_max)`).

**STATE** Brak — bezstanowe mapowanie liniowe co tick.

**TIMEBASE** Co tick, bez filtracji.

**INVARIANTS** Poniżej `throttle_offset` zwraca 0 (odłączona/nieużywana manetka nic nie
daje). Łączony z żądaniem pedałowym przez `max()` w `ride_control.c` (throttle nie sumuje
się z assist — wygrywa większe z dwóch, FW-091).

**TEST SEAMS** `ride_control_pipeline_host.c` ustawia `throttle_iq=0` we wszystkich
scenariuszach tej karty (throttle nie jest jeszcze zamiatany jako osobny scenariusz —
patrz `../testing/REGRESSION_SCENARIOS.md`, scenariusz THROTTLE, priorytet P2, nie
zaimplementowany w tej karcie).

**RELATED SOURCE FILES** `src/main.c` (linia budowy `ride_input.throttle_iq`),
`src/ride_control.c` (separne wywołanie `assist_limits_apply` dla throttle).

**KNOWN ISSUES** Brak — moduł trywialny, brak dziś znanych problemów.
