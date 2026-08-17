# ASSIST_DYNAMICS — rampy Iq, `src/assist_dynamics.c`

**PURPOSE** Ostatni krok przed `motor_core`: prowadzi bieżący Iq do celu po rampie
zależnej od prędkości/kadencji (szybciej przy wyższej prędkości), z osobną, krótszą,
firmware-owned rampą dla "hard cut" (brake/cofnięcie/przegrzanie) niezależną od
konfigurowalnego `release_ms` poziomu.

**INPUTS** `assist_dynamics_apply(iq_target, iq_reference, assist_dynamics_input_t*)`:
`speed_x100`, `cadence_rpm`, `iq_scale`, `phase_current_max`, `walk_active`,
`immediate_cut`, `safety_cut`, `profile_pedaling_active`, `profile_release_ms`,
4× rampy (`ramp_up/down_slow/fast_ms`, per-poziom), `coast_release`.

**OUTPUTS** `int32_t` — Iq po jednym kroku rampy (funkcja jest wołana co tick, rampa
buduje się przez wiele wywołań).

**STATE** `static int32_t iq_reference_q` (Q8 fixed-point) — TU żyje prawdziwy stan
rampy, NIE w parametrze `iq_reference` (który jest tylko echem w gałęziach
walk/immediate_cut — częsty błąd przy pisaniu testu, patrz TEST SEAMS).

**TIMEBASE** Ticki × `CONTROL_TICKS_PER_MS=4` (kategoria B, `../architecture/TIMEBASES.md`)
— zakłada 4 kHz wywołań.

**INVARIANTS** `RIDE_HARD_CUT_RAMP_MS=200` (assert `≤250` w `ride_control.c`) jest
firmware-owned, NIGDY z `level->release_ms` (rider-configurowalne do 3000 ms).

**TEST SEAMS** Wołany wewnątrz `ride_control_update()` — nie ma osobnego harnessu L1 w
tej karcie (pokryty przez `ride_control_pipeline_host.c`). WAŻNE dla przyszłego testu
jednostkowego: `iq_reference` parametr NIE jest tym, co trzeba karmić z poprzedniego
ticku dla poprawności rampy głównej ścieżki — stan jest w statyku modułu, więc test
jednostkowy musi wołać funkcję W PĘTLI (nie pojedynczo z ręcznie utrzymywanym
"poprzednim" argumentem) tak jak `main.c` faktycznie robi.

**RELATED SOURCE FILES** `src/assist_dynamics.c`, `inc/assist_dynamics.h`.

**KNOWN ISSUES** `extern int32_t map(...)` bez wspólnego nagłówka (jak `LIMITS.md`) —
ta sama F4. `map_adapter.c` wymagany do linkowania w izolacji.
