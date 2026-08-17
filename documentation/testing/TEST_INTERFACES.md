# TEST_INTERFACES — jak harnessy tej karty naprawdę linkują się do firmware

**PURPOSE** Dokładna lista: które moduły `.c` są wykonywane naprawdę, które adaptery
były potrzebne do linkowania, i dlaczego. Czytaj to PRZED pisaniem nowego harnessu, żeby
nie odkrywać tych samych problemów od nowa.

## Moduły faktycznie linkowane (nie reimplementowane)

| Harness | Linkuje naprawdę |
|---|---|
| `torque/torque_trace_host.c` | `src/torque_input.c` |
| `pipeline/power_pipeline_host.c` | `src/torque_input.c`, `src/rider_input.c`, `src/assist_modes.c`, `src/cadence_comp.c`, `src/power_curve.c`, `src/assist_start.c`, `src/assist_extended_boost.c`, `src/tuning_config.c` |
| `pipeline/ride_control_pipeline_host.c` | jw. + `src/ride_control.c`, `src/assist_dynamics.c`, `src/assist_limits.c`, `src/motor_core.c` |
| `scenarios/missed_tick_burst_host.c` | `src/torque_input.c`, `src/ride_episode.c` |

Generator w `tests/host/common/crank_model.c` jest STYMULEM testowym (jak generator
sygnałowy na stole laboratoryjnym), nie firmware — patrz `../inputs/PAS.md` "KNOWN
ISSUES" po dokładne uzasadnienie, dlaczego to jest w porządku mimo że dekoder PAS sam
nie jest wykonywany.

## Adaptery, które BYŁY potrzebne (i dlaczego mniejsza zmiana nie wystarczała)

### 1. `tests/host/common/host_stubs/gd32f30x.h`, `arm_math.h`

**Problem:** `inc/motor_core.h` → `#include "inc/main.h"` → `#include "gd32f30x.h"` (realny
nagłówek CMSIS GD32, tysiące linii definicji rejestrów) + `#include "gd32f307c_eval.h"`
(TEŻ realny plik projektu w `inc/`, który sam włącza `gd32f30x.h` i używa dziesiątek
makr `GPIO_PIN_*`/`RCU_*`/`EXTI_*`). Żaden z nich nie ma sensu na hoście.

**Próba 1 (odrzucona): adapter/wrapper bez zmiany include path.** Niemożliwe —
`inc/gd32f307c_eval.h` jest znajdowany PRZED jakimkolwiek `-I` (reguła C: `#include
"..."` szuka najpierw w katalogu pliku włączającego, czyli `inc/`, gdzie ten plik
naprawdę istnieje) — nie da się go podmienić bez fizycznej zmiany pliku.

**Zaimplementowane: podmiana TYLKO dwóch prawdziwie wendorowych nagłówków.**
`gd32f30x.h` nie istnieje w `inc/`, więc DA się go podmienić przez `-I` wskazujące
najpierw na `tests/host/common/host_stubs/`. Stub dostarcza wyłącznie to, czego
`inc/main.h` i prawdziwy `inc/gd32f307c_eval.h` faktycznie potrzebują: `FlagStatus`,
`ErrStatus`, dwa minimalne structy CAN (nigdy nie dereferencjonowane), i ~25 opakowanych
stałych całkowitych (`GPIO_PIN_12` itd.) które `gd32f307c_eval.h` łańcuchuje we własne
`#define`. `inc/main.h` (i typy `MotorState_t`/`MotorParams_t`/`PI_control_t`) NIE są
zmienione — kompilują się jako PRAWDZIWY plik. Zweryfikowane: kompiluje się czysto pod
`-Wall -Wextra -Werror`, zero ostrzeżeń.

**Behavior-preserving?** Tak, bezwzględnie — żaden plik produkcyjny nie jest zmieniony;
adapter istnieje wyłącznie na ścieżce include testu hosta, nigdy widziany przez build ARM
(`.cproject` wskazuje na `Firmware/CMSIS/...`, nie na `tests/`).

### 2. `tests/host/common/map_adapter.c`

**Problem:** `src/assist_dynamics.c` i `src/assist_limits.c` deklarują lokalnie `extern
int32_t map(...)` zamiast włączać wspólny nagłówek — jedyna definicja jest w `src/main.c`
(audyt, finding F4). Nie da się zlinkować żadnego z tych dwóch modułów bez `main.c` LUB
bez tej samej funkcji gdzie indziej.

**Zaimplementowane:** kopia bajt-w-bajt ciała `map()` z `main.c:2826-2841`, z komentarzem
wskazującym źródło i to, że jest to obejście znanego problemu, nie naprawa.

**Ryzyko:** jeśli `main.c`'s `map()` kiedyś się zmieni, ten plik trzeba zaktualizować
ręcznie — nic tego dziś nie pilnuje automatycznie. To JEST realny koszt tego obejścia,
zapisany tu świadomie.

### 3. `tests/host/common/motor_service_stub.c`

**Problem:** `src/ride_control.c` woła `walk_assist_iq_request()` i
`hall_calibration_iq_request()` (zadeklarowane w `inc/motor_service.h`, zdefiniowane w
`src/main.c`) wewnątrz gałęzi `if (input->walk_active)` / `if
(input->position_calibration_active)`. Symbole muszą istnieć przy LINKOWANIU nawet jeśli
żaden scenariusz w tej karcie nigdy nie wchodzi w te gałęzie.

**Zaimplementowane:** dwie funkcje zwracające `0`, z komentarzem że są nieosiągalne przy
`walk_active=false`/`position_calibration_active=false` (prawda dla każdego scenariusza
tej karty).

## Znane luki (co NIE jest testowane, świadomie)

- **Dekoder PAS** (`main.c`, inline w `reg_ADC_processing`) — generator podaje LICZBĘ
  kroków wprost do `torque_input_run_filter_step()`, nie wykonuje maszyny stanów GPIO.
  Patrz `../inputs/PAS.md`.
- **`motor_voltage_utilization`** — zawsze 0 w harnessach pipeline (brak FOC → brak
  realnego duty cycle). Pomija jedną gałąź w `assist_modes.c`'s
  `finish_power_request()`. Patrz `../assist/POWER_MODE.md`.
- **`MP.assist_settings[][0]` (legacy per-poziomowy limit % z EEPROM)** — nie
  replikowane; `ride_core_iq_limit`/`iq_scale` ustawione na pełny `PH_CURRENT_MAX` w
  każdym scenariuszu. Udokumentowane jako stałe założenie w
  `ride_control_pipeline_host.c`.
- **FOC/Hall/PWM** — poza granicą tej karty, patrz `../motor/MOTOR_COMMAND.md`.

## Trace CSV format

Kolumny różnią się per harness (celowo — każdy harness ujawnia tylko to, co faktycznie
liczy, karta sekcja 11: "Nie dodawaj sztucznych pól"). Wspólne kolumny bazowe:
`tick, time_s, crank_angle_deg, pas_state, cadence_input`. `pas_state` to
`step_count % 4` z generatora — ILUSTRACYJNY indeks fazy 0-3, NIE dosłowny odczyt dwóch
linii GPIO A/B (patrz `../inputs/PAS.md`). Reszta kolumn per harness — patrz nagłówek CSV
w każdym pliku `tests/host/out/*.csv` (self-describing, pierwsza linia).

**RELATED SOURCE FILES** Wszystkie pliki w `tests/host/common/`, `tests/host/torque/`,
`tests/host/pipeline/`, `tests/host/scenarios/`.
