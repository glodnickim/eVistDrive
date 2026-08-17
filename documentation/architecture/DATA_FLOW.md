# DATA_FLOW — end-to-end tor jazdy

**PURPOSE**
Jedna strona: gdzie w firmware leży każdy etap toru od sygnału fizycznego do polecenia
silnika. Szczegóły każdego etapu są w osobnych dokumentach (routing: `../INDEX.md`).

**PIPELINE (kolejność wykonania w `reg_ADC_processing()`, `src/main.c`, ~4 kHz)**

```
PAS A/B (GPIO) ---\
torque ADC --------+--> torque_input.c --> rider_input.c --\
throttle ADC ------/                                        |
speed EXTI ---(main.c, osobny licznik)----------------------+--> ride_control.c
brake GPIO ---(main.c, pętla główna, NIE 4 kHz)--------------/        |
                                                                        v
                                                            assist_modes.c (Power/eMTB/Torque)
                                                                        |
                                                          ride latch (inline w ride_control.c)
                                                                        |
                                                       assist_extended_boost.c (opcjonalny)
                                                                        |
                                                              assist_limits.c (V/temp/prawo)
                                                                        |
                                                              assist_start.c (smooth start)
                                                                        |
                                                            assist_dynamics.c (rampy Iq)
                                                                        |
                                                                        v
                                                                 motor_core.c
                                                          (JEDYNY writer MS.i_q_setpoint)
                                                                        |
                                                     src/FOC.c + main.c ISR (Hall/PWM/FOC)
                                                          <- POZA zakresem test infra dziś
```

**INPUTS** Wejścia fizyczne, patrz `../inputs/*.md`.

**OUTPUTS** `motor_command_t{iq_target, id_target, enable, emergency_stop}` — patrz
`../motor/MOTOR_COMMAND.md`.

**STATE** Rozproszony: każdy moduł od `torque_input` do `assist_dynamics` trzyma własny
stan lokalny (statyki pliku), zero globali `MS`/`MP`. `ride_control.c` dodatkowo trzyma
ride latch i gear preload jako własne statyki (nie osobny moduł — patrz
`../assist/START_REARM_RELEASE.md`).

**TIMEBASE** 4 kHz nominalnie (`CONTROL_TIMEBASE_HZ`), patrz `TIMEBASES.md` — NIE zakładaj,
że "4 kHz" oznacza "raz na milisekundę × 4" bez wyjątków; część liczników w tym torze liczy
WYWOŁANIA, nie realny czas (patrz TIMEBASES.md).

**INVARIANTS**
- `motor_core_set_command()` jest jedynym miejscem piszącym `MS.i_q_setpoint`/`i_d_setpoint`
  (zweryfikowane grepem podczas audytu i podczas budowy tej karty).
- Kolejność w `reg_ADC_processing()` ma znaczenie: torque najpierw korygowany, potem
  dekoder PAS (który woła `torque_input_run_filter_step()` na krok naprzód), potem
  `torque_input_coast_update()`, potem `torque_input_update()` — harnessy testowe muszą
  powtarzać dokładnie tę kolejność (patrz `../testing/TEST_INTERFACES.md`).

**TEST SEAMS** Cały tor od `torque_input` do `motor_core` jest dziś testowalny na hoście —
patrz `tests/host/pipeline/` i `../testing/TEST_ARCHITECTURE.md`. `FOC`/Hall/PWM nie są.

**RELATED SOURCE FILES** `src/main.c` (reg_ADC_processing), `src/torque_input.c`,
`src/rider_input.c`, `src/assist_modes.c`, `src/ride_control.c`, `src/assist_dynamics.c`,
`src/assist_limits.c`, `src/assist_start.c`, `src/assist_extended_boost.c`,
`src/motor_core.c`.

**KNOWN ISSUES** Zob. `ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md` findings F1, F5, F7. Pełna
lista znalezisk tej karty: `../TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md` sekcja 16.
