# INDEX — router, not a manual

Jeśli pracujesz nad konkretnym tematem, przeczytaj TYLKO dokumenty wskazane niżej — nie
czytaj całej reszty `documentation/`. Cel: minimalny kontekst potrzebny do zadania, zwykle
1-4 krótkie dokumenty. Jeśli okaże się, że potrzebujesz więcej niż wskazano, to sygnał, że
ten routing jest niepełny — dopisz brakującą pozycję zamiast czytać wszystko "na wszelki
wypadek".

Każdy dokument niżej ma stały szablon: PURPOSE / INPUTS / OUTPUTS / STATE / TIMEBASE /
INVARIANTS / TEST SEAMS / RELATED SOURCE FILES / KNOWN ISSUES.

Historia decyzji (karty FW-XXX, CHANGELOG, starsze audyty) NIE jest tu routowana — to inny
poziom kontekstu, patrz `documentation/README.md` ("Przewodnik po dokumentacji"), które
pozostaje źródłem prawdy dla tematów historycznych/kart FW.

## Routing

| Zadanie | Czytaj |
|---|---|
| **PAS** (dekodowanie, cadence, kierunek, re-engagement gate) | [inputs/PAS.md](inputs/PAS.md) + [architecture/TIMEBASES.md](architecture/TIMEBASES.md) |
| **torque filtering / RUN estimator** | [inputs/TORQUE_SENSOR.md](inputs/TORQUE_SENSOR.md) |
| **high cadence power** (czy zachowanie zmienia się z cadence) | [inputs/TORQUE_SENSOR.md](inputs/TORQUE_SENSOR.md) + [assist/POWER_MODE.md](assist/POWER_MODE.md) + [assist/LIMITS.md](assist/LIMITS.md) + [architecture/TIMEBASES.md](architecture/TIMEBASES.md) jeśli problem dotyczy timingu |
| **re-engagement** (cofnięcie pedałowania, powrót assist) | [inputs/PAS.md](inputs/PAS.md) + [assist/START_REARM_RELEASE.md](assist/START_REARM_RELEASE.md) |
| **SOC / bateria** | [inputs/BATTERY.md](inputs/BATTERY.md) |
| **throttle** | [inputs/THROTTLE.md](inputs/THROTTLE.md) + [assist/RIDER_INPUT.md](assist/RIDER_INPUT.md) (sekcja "throttle floor") |
| **brake / hard cut** | [inputs/BRAKE.md](inputs/BRAKE.md) + [assist/ASSIST_DYNAMICS.md](assist/ASSIST_DYNAMICS.md) (sekcja "hard cut ramp") |
| **speed sensor / limit prędkości** | [inputs/SPEED_SENSOR.md](inputs/SPEED_SENSOR.md) + [assist/LIMITS.md](assist/LIMITS.md) |
| **limiter mocy / prądu / napięcia / temperatury** | [assist/LIMITS.md](assist/LIMITS.md) |
| **ride latch / gear preload / rampy Iq** | [assist/ASSIST_DYNAMICS.md](assist/ASSIST_DYNAMICS.md) + [assist/START_REARM_RELEASE.md](assist/START_REARM_RELEASE.md) |
| **finalny Iq/Id do silnika** | [motor/MOTOR_COMMAND.md](motor/MOTOR_COMMAND.md) |
| **FOC / Hall / PWM** | NIE opisane tu jeszcze — poza zakresem tej karty (test infrastructure foundation kończy się na `motor_command_t`, patrz [motor/MOTOR_COMMAND.md](motor/MOTOR_COMMAND.md) "KNOWN ISSUES"). Do czasu napisania osobnej karty czytaj bezpośrednio `src/FOC.c` i `src/main.c` (ISR sekcja) plus `documentation/ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md` sekcja 3/4. |
| **pisanie/uruchamianie testów regresyjnych** | [testing/TEST_ARCHITECTURE.md](testing/TEST_ARCHITECTURE.md) + [testing/TEST_INTERFACES.md](testing/TEST_INTERFACES.md) |
| **dodawanie nowego scenariusza regresji** | [testing/REGRESSION_SCENARIOS.md](testing/REGRESSION_SCENARIOS.md) + [testing/TEST_INTERFACES.md](testing/TEST_INTERFACES.md) |
| **planowanie nowej karty zmian** (jakie moduły/testy dotknie) | [testing/CHANGE_CARD_TEMPLATE.md](testing/CHANGE_CARD_TEMPLATE.md) |

## Co NIE jest tu routowane (świadomie)

- Pełna architektura repo / gotowość motor-agnostyczna → `ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md`
  (duży dokument-parasol, czytaj tylko gdy zadanie faktycznie dotyczy reorganizacji, nie
  pojedynczego modułu).
- Wynik konkretnego uruchomienia testów → `TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md`
  (raport z jednego przebiegu prac, nie kontrakt modułu — nie routuj tu przyszłych zadań).
- Historia kart FW-XXX i CHANGELOG → `README.md` w tym katalogu.
