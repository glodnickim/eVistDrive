# RIDER_INPUT — `src/rider_input.c`

**PURPOSE** Jeden spójny, tylko-do-odczytu snapshot stanu jeźdźca (`rider_input_t`),
budowany raz na tick w `main.c` z wielu źródeł (torque_input, dekoder PAS, ADC), czytany
przez cały ride core.

**INPUTS** `rider_input_update(const rider_input_t *sample)` — jeden struct, budowany
przez wywołującego (main.c: `reg_ADC_processing`, linie ~2109-2139, lub harness testowy).

**OUTPUTS** `rider_input_get()` → wskaźnik do ostatniego snapshotu. Pola: torque
(raw/corrected/filtered/assist_filtered/run_filtered/load_centikg), cadence_rpm,
wheel_speed_x100, motor_erps, motor_voltage_utilization, pas_forward/backward,
pedaling_active, crank_forward_steps, crank_direction_ok, start_phase,
torque_sensor_valid, pas_sensor_valid.

**STATE** Jeden statyczny `rider_input_t` (ostatni snapshot).

**TIMEBASE** Bezstanowe — moduł sam nie ma pojęcia o czasie, tylko "co jest teraz".

**INVARIANTS** Snapshot jest KOMPLETNY — konsumenci (assist_modes, ride_control) nie
sięgają do MS/MP bezpośrednio dla danych jeźdźca. `throttle_iq` NIE jest tu polem
(throttle wchodzi do `ride_control_input_t` osobno, żeby limiter mógł go zawsze
klasyfikować jako non-pedal — patrz `LIMITS.md`).

**TEST SEAMS** Trywialny moduł, ale JEDYNY punkt styku między "wejściami fizycznymi" a
"logiką assist" — każdy harness pipeline (`tests/host/pipeline/*.c`) buduje własny
`rider_input_t` ręcznie i woła `rider_input_update()`.

**RELATED SOURCE FILES** `src/rider_input.c`, `inc/rider_input.h`.

**KNOWN ISSUES** `torque_filtered` (pole `MS.torque_filtered`-owy odpowiednik — EMA licząca
`torque_cumulated>>TQfilter` w main.c) NIE jest tym samym co `torque_assist_filtered`
(z torque_input.c). Test infrastructure tej karty zostawia `torque_filtered=0` w
harnessach pipeline (pole nieużywane przez `assist_modes.c` — sprawdzone czytaniem kodu),
więc nie było potrzeby go replikować. Jeśli przyszła karta doda konsumenta tego pola,
trzeba będzie go faktycznie policzyć.
