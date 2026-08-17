# BRAKE — `MS.brake_active_flag`

**PURPOSE** Sygnał hamulca, część `hard_cut` w `ride_control.c` (natychmiastowe
wyzerowanie żądania, rampa ograniczona do `RIDE_HARD_CUT_RAMP_MS=200 ms`, niezależnie od
`release_ms` poziomu).

**INPUTS** GPIOC13, aktywny nisko.

**OUTPUTS** `MS.brake_active_flag`, wchodzi w skład `ride_control_input_t.safety_cut`
(razem z cofnięciem, przegrzaniem, błędem torque, kalibracją).

**STATE** Brak.

**TIMEBASE** Próbkowany w PĘTLI GŁÓWNEJ (`main.c:831-832`), NIE w `reg_ADC_processing()`
(4 kHz) — jedyny sygnał bezpieczeństwa czytany w innej bazie czasu niż reszta łańcucha.
Pętla główna zwykle działa szybciej niż 4 kHz, więc w praktyce prawdopodobnie
nieszkodliwe, ale brak jawnego zapewnienia "co najmniej raz na tick" i brak debounce.

**INVARIANTS** Brak debounce — pojedynczy glitch na pinie trafia bezpośrednio do
`hard_cut`.

**TEST SEAMS** `ride_control_pipeline_host.c` ustawia `safety_cut=false` we wszystkich
scenariuszach tej karty. Scenariusz BRAKE_CUT (`../testing/REGRESSION_SCENARIOS.md`,
priorytet P1) NIE zaimplementowany w tej karcie — `ride_control.c` już przyjmuje
`safety_cut` jako zwykłe pole `bool` w `ride_control_input_t`, więc dodanie tego
scenariusza w przyszłej karcie nie wymaga zmian API.

**RELATED SOURCE FILES** `src/main.c` (linia 831-832), `src/ride_control.c` (`hard_cut`).

**KNOWN ISSUES** Audyt finding F9 (inna baza czasu, brak debounce) — niezmienione, poza
zakresem tej karty.
