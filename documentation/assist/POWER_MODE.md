# POWER_MODE — `src/assist_modes.c`

**PURPOSE** Liczy żądanie asysty (moc, Iq) z jednego z pięciu trybów: Power
Linear/Progressive/Curve, eMTB, Torque. Jeden entry point, czysty moduł (zero MS/MP,
zweryfikowane grepem).

**INPUTS** `assist_modes_calculate(rider_input_t*, assist_level_config_t*,
battery_voltage_mv, iq_limit, assist_mode_output_t*)`. Poziom pobierany przez
`assist_modes_get_default_level(index)` (0..5) po `assist_modes_init()` +
`assist_modes_set_active_bank(0|1)`.

**OUTPUTS** `assist_mode_output_t`: `human_power_w`, `assist_basis_power_w`,
`raw_motor_power_w`, `motor_power_w` (po filtrze rise/fall), `applied_support_ratio_pct`,
`iq_request` (PRZED ride latch/limiterem/rampą — to nie jest finalny Iq, patrz
`../motor/MOTOR_COMMAND.md`), `startup_boost_extra_pct`, `cadence_comp_permille`.

**STATE** `power_filter_state` (adres configu + rise/fall ms, wykrywa zmianę poziomu i
resetuje filtr), `bank_config[2][6]` (zainicjalizowane z compiled-in defaults, patrz
`assist_modes_init()`).

**TIMEBASE** Filtr mocy (`filter_motor_power`): ticki × `CONTROL_TICKS_PER_MS=4` (zakłada
4 kHz wywołań — kategoria B, patrz `../architecture/TIMEBASES.md`). Startup boost curve
zależy od cadence, nie czasu.

**INVARIANTS** Cadence compensation (`assist_modes_get_cadence_comp_enabled()`) domyślnie
WYŁĄCZONA (`BANK_CADENCE_COMP_DEFAULT=0`) — RUN_60..RUN_120 (patrz
`../testing/REGRESSION_SCENARIOS.md`) testują tor BEZ tej kompensacji celowo (karta,
sekcja 7: "to NIE jest test cadence compensation"). Startup boost (curve CADENCE, domyślnie
włączony) zanika do ~0 powyżej ~30-40 rpm z compiled-in `cadence_step=20` — zmierzone:
`startup_boost_extra_pct=0` w całym zakresie RUN_60..RUN_120 w bazowym przebiegu tej karty.

**TEST SEAMS** `tests/host/pipeline/power_pipeline_host.c` — pełne pokrycie L2/L3 (bez
ride_control). Zmierzone bazowo (ta karta): `iq_request` zmienia się <1% między RUN_60 a
RUN_120 (113.85→114.51 średnio w 6 s) przy identycznym profilu nacisku — brak
nieoczekiwanej zależności od cadence przy domyślnej konfiguracji.

**RELATED SOURCE FILES** `src/assist_modes.c`, `inc/assist_modes.h`,
`src/cadence_comp.c`, `src/power_curve.c`.

**KNOWN ISSUES** `src/assist_modes.c` NIE kompiluje się czysto pod `-Werror` bez
`-Wno-type-limits` (linia ~258, `valid_wa_max_wheel_x10`: `value<=255U` zawsze prawdziwe
dla `uint8_t`). Udokumentowany wyjątek w `tests/host/run_regression.ps1`, nienaprawione.
`motor_voltage_utilization` nie jest realny w harnessach tej karty (brak FOC) — pomija
gałąź cross-check moc→napięcie w `finish_power_request()`, patrz
`../testing/TEST_INTERFACES.md`.
