# PAS — dekoder kwadratury i cadence

**PURPOSE** Zamienia dwa sygnały cyfrowe (PAS A/B) na kierunek, licznik kroków naprzód
(`fwd_run`), cadence i sygnały wejściowe dla `torque_input`'s RUN estimator oraz dla ride
latch w `ride_control.c`.

**INPUTS** GPIOC12 (A), GPIOD2 (B), próbkowane co tick w `reg_ADC_processing()`
(`src/main.c:1868`). Tabela stanów kwadratury `qd[16]` + `PAS_DIR_SIGN` (`config.h`).

**OUTPUTS** `MS.cadence`, `fwd_run` (kroki naprzód z rzędu, capped 250), `Backwards_counter`
(latch cofnięcia), `pas_fwd_accum` (pozycja na tarczy), wywołania
`torque_input_run_filter_step()` (raz na krok naprzód) i `ride_episode_forward_step()` /
`ride_episode_reverse_step()`.

**STATE** ~20 zmiennych statycznych w `main.c` (nie w osobnym module — patrz KNOWN ISSUES):
`pas_qstate`, `pas_idle_ticks`, `pas_cycle_ticks`, `pas_fwd_steps`, `pas_rev_run`, itd.

**TIMEBASE** Próbkowanie co wywołanie `reg_ADC_processing()` (nominalnie 4 kHz, patrz
`../architecture/TIMEBASES.md` — dekoder jest kategorii C: jeśli tick zostanie pominięty,
kroki fizycznie zaszłe w tym oknie są utracone/aliasowane, nie tylko opóźnione).
`PAS_STOP_TICKS`/`PAS_STOP_TICKS_MAX` (adaptacyjny timeout zatrzymania) i
`BACKWARD_CONFIRM_STEPS=3` są w tickach.

**INVARIANTS** Krok kwadratury = 3.75° (96 kroków/obrót = `PAS_STEPS_PER_PULSE=4` ×
24 pulsy/obrót). Cadence pulse co 4 kroki naprzód. Jeden krok wstecz zeruje `fwd_run`
natychmiast (bezpieczeństwo — cofnięcie odbiera assist w tym samym ticku).

**TEST SEAMS** Dekoder NIE jest wydzieloną funkcją — żyje inline w `reg_ADC_processing()`.
Test infrastructure tej karty NIE wykonuje prawdziwego dekodera: generator
`tests/host/common/crank_model.c` liczy PRAWDZIWĄ liczbę kroków z modelu kąta korby i
woła `torque_input_run_filter_step()` bezpośrednio tyle razy, ile kroków naprawdę zaszło —
to podaje dekoderowi TAKI SAM efekt, jaki dałby poprawnie działający dekoder, ale samego
dekodera (jego GPIO-owej maszyny stanów) nie wykonuje. Zobacz
`../testing/TEST_INTERFACES.md`.

**RELATED SOURCE FILES** `src/main.c` (`reg_ADC_processing`, linie ~1866-2076),
`inc/config.h` (`PAS_STEPS_PER_PULSE`, `PAS_DIR_SIGN`, `PAS_STOP_TICKS*`).

**KNOWN ISSUES** Audyt finding F3 (martwy kod `PAS_processing()`/`EXTI10_15_IRQHandler` —
prawdziwy dekoder to inline blok, nie ta funkcja). Wydzielenie dekodera do osobnego modułu
to etap D w audycie ("Koszt reorganizacji") — NIE wykonane w tej karcie (zakaz
przebudowy `main.c`). To jest jedyny istotny OBSERVABILITY GAP tej karty: nie ma
zautomatyzowanego testu prawdziwego dekodera GPIO, tylko testy jego udokumentowanego
kontraktu wyjściowego.
