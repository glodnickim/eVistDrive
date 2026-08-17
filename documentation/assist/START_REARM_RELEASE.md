# START_REARM_RELEASE — ride latch, gear preload, smooth start, Extended Boost

**PURPOSE** Wszystko co decyduje KIEDY assist zaczyna/kończy płynąć, poza samą rampą Iq
(`ASSIST_DYNAMICS.md`). Cztery różne mechanizmy, trzy pliki:

| Mechanizm | Gdzie | Moduł? |
|---|---|---|
| Ride latch (próg kg, floor prądu, hold timer) | `ride_control.c:270-362` | NIE — inline |
| Gear preload (cap prądu przy starcie z zera) | `ride_control.c:576-594` | NIE — inline |
| Smooth start (obwiednia rampy przy starcie) | `src/assist_start.c` | TAK |
| Extended Boost (trzyma prąd po zatrzymaniu korby) | `src/assist_extended_boost.c` | TAK |

**INPUTS/OUTPUTS** Patrz nagłówki: `inc/assist_start.h`, `inc/assist_extended_boost.h`.
Ride latch/preload czytają `rider_input_t` + `ride_control_input_t` bezpośrednio w
`ride_control.c` (brak osobnego struktu wejścia).

**STATE** Ride latch: `assist_latched`, `assist_hold_ticks` (statyki `ride_control.c`).
Gear preload: `preload_active`, `preload_ticks`. Extended Boost: FSM 4-stanowy
(`IDLE→QUALIFY→ARMED→ACTIVE`).

**TIMEBASE** Ride latch hold: `tuning_config_assist_hold_ticks()` (ticki, kategoria B).
Extended Boost: `EXT_BOOST_CONTROL_TICKS_PER_MS=4` (kategoria B — `confirm_ticks`,
`arm_idle_ticks`, `active_ticks_left` liczą WYWOŁANIA, nie realny czas).

**INVARIANTS** Ride latch wymaga: kierunek naprzód + próg kroków (`fwd_run`, złagodzony o
1 gdy `bike_rolling`) + próg kg (`minimum_pedal_load_centikg` lub niższy
`riding_start_load_centikg` gdy już się toczy). Extended Boost startuje na ZBOCZU
zatrzymania pedałowania, nigdy na poziomie — domyślnie WYŁĄCZONY (`duration_ms=0`).

**TEST SEAMS** Ride latch i gear preload: pokryte tylko pośrednio przez
`ride_control_pipeline_host.c` (nie mają własnego harnessu L1, bo nie są modułami — patrz
`../ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md` sekcja 4, `ride_control`). Extended Boost:
już ma test hosta (`tests/host/fw100_extended_boost_host.c`, sprzed tej karty).

**RELATED SOURCE FILES** `src/ride_control.c`, `src/assist_start.c`,
`src/assist_extended_boost.c`.

**KNOWN ISSUES** Ride latch/gear preload jako logika inline w `ride_control.c` (nie
moduł) to naturalna granica do wydzielenia w przyszłości (audyt, sekcja 4) — NIE
wykonane w tej karcie.
