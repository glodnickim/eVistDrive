# MOTOR_COMMAND — granica ride core / motor-specific, `src/motor_core.c`

**PURPOSE** Jedyny punkt, przez który ride core zamawia prąd silnika. Ta granica jest
GRANICĄ TEJ KARTY: wszystko powyżej (torque → ... → `motor_command_t`) jest testowalne
deterministycznie na hoście; wszystko poniżej (FOC, Hall, PWM) nie jest.

**INPUTS** `motor_core_set_command(const motor_command_t *command)`:
`{iq_target, id_target, enable, emergency_stop}`.

**OUTPUTS** Zapis `MotorState_t.i_q_setpoint`/`i_d_setpoint` (przez wskaźnik ustawiony w
`motor_core_init()`).

**STATE** `static MotorState_t *state` (wskaźnik, nie właściciel danych).

**TIMEBASE** Brak — bezstanowy zapis co wywołanie.

**INVARIANTS** `motor_core_set_command()` jest JEDYNYM pisarzem
`i_q_setpoint`/`i_d_setpoint` w całym repo — zweryfikowane grepem podczas audytu I
podczas budowy tej karty (`grep "i_q_setpoint\s*=" src/*.c` → tylko `motor_core.c`).
`emergency_stop || !enable` → natychmiast 0 (bez rampy).

**TEST SEAMS** `tests/host/pipeline/ride_control_pipeline_host.c` linkuje ten moduł
naprawdę, przez adapterowe nagłówki wendorowe (patrz KNOWN ISSUES i
`../testing/TEST_INTERFACES.md`) — pierwsza w tym repo host-testowa ścieżka sięgająca aż
tutaj.

**RELATED SOURCE FILES** `src/motor_core.c`, `inc/motor_core.h`, `inc/main.h` (typ
`MotorState_t`).

**KNOWN ISSUES**
- `inc/motor_core.h` `#include`uje `inc/main.h`, które ciągnie prawdziwe nagłówki
  wendorowe GD32 (`gd32f30x.h`, `arm_math.h`) — niekompilowalne na hoście bez adaptera.
  Ta karta dodała `tests/host/common/host_stubs/{gd32f30x.h,arm_math.h}` (podmienione
  na ścieżce include TYLKO dla testów hosta, `inc/main.h` sam NIE zmieniony) — patrz
  `../testing/TEST_INTERFACES.md` po pełne uzasadnienie.
- Poniżej tej granicy (`src/FOC.c`, ISR w `src/main.c`: Hall, dynamiczny wybór fazy ADC,
  Park/Clarke, SVPWM) NIE ma dziś testu ani planu testu w tej karcie — wymaga modelu
  fizycznego silnika/mostka (L4, sprzęt/log), nie unit testu. Patrz
  `../ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md` sekcja 13.
