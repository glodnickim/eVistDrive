# Ride Core refactor

## Cel etapu 1

Rozdzielić decyzję o charakterze wspomagania od wykonania zadanego prądu
`Iq`, zachowując bit po bicie obecną kolejność obliczeń i zachowanie Legacy.
Nowe tryby jazdy nie są częścią tego etapu.

Punkt przywracania przed refaktoryzacją:

```text
commit: d6bc69c
tag:    m820-before-ride-core-refactor
build:  0.0136 M820/BL820 — OK
```

## Inwentaryzacja żądania Iq

### Finalny `MS.i_q_setpoint`

Przed refaktoryzacją był zapisywany w `src/main.c` przez:

1. inicjalizację stanu w `main()`,
2. watchdog komunikacji HMI,
3. natychmiastowe cięcia bezpieczeństwa w `reg_ADC_processing()`,
4. czasową lub krokową rampę w `reg_ADC_processing()`,
5. procedurę autodetekcji położenia Halla.

Obecnie wszystkie te ścieżki używają `motor_command_t`, a finalne pola
`MS.i_q_setpoint` i `MS.i_d_setpoint` zapisuje wyłącznie `src/motor_core.c`.
FOC tylko odczytuje finalną wartość w `runPIcontrol()` i w wywołaniu
`FOC_calculation()`.

### Surowe żądanie `MS.i_q_setpoint_temp`

`legacy_assist_calculate_monolith()` nadal miesza w jednej funkcji:

- hamulec,
- Walk Assist,
- throttle,
- tryby wspomagania pedałowania,
- startup boost i Extended Boost,
- smooth start,
- limit napięcia,
- limit temperatury,
- ograniczenie prędkości,
- kalibrację kąta Halla.

To pole pozostaje częścią Legacy do czasu wydzielenia `legacy_assist` oraz
`assist_limits`. Refaktoryzacja nie zmienia jeszcze żadnego wzoru ani progu.

## Kolejność bezpiecznej migracji

1. Dodać neutralny interfejs `motor_core` i podłączyć go do istniejącego stanu. — wykonane
2. Przenieść przez interfejs inicjalizację oraz finalny wynik rampy. — wykonane
3. Przenieść awaryjne wyzerowania, zachowując ich natychmiastowe działanie. — wykonane
4. Odseparować specjalną procedurę autodetekcji od ścieżki Ride Control.
5. Potwierdzić wyszukiwaniem, że poza `motor_core.c` nie ma zapisu finalnego
   `MS.i_q_setpoint` ani `MS.i_d_setpoint`.
6. Zbudować firmware i porównać rozmiar oraz ostrzeżenia z buildem bazowym.

Każdy z powyższych kroków kończy się osobnym buildem. Test na rowerze jest
wymagany przed rozpoczęciem portu Power TSDZ2.

## Rider Input

`rider_input` publikuje jeden spójny obraz wejść po istniejącym dekoderze PAS,
filtrze momentu i diagnostyce czujnika. Używa dotychczasowych jednostek
stałoprzecinkowych, aby nie wprowadzać kosztownych konwersji `float` do pętli
4 kHz. Legacy nadal czyta `MS` i dotychczasowe liczniki; przepięcie odbiorców
będzie osobnym krokiem po sprawdzeniu kompilacji.

## Legacy Assist

Aktywna ścieżka wywołuje `legacy_assist_calculate()`. Obecna implementacja
pozostaje jawnie oznaczona jako `legacy_assist_calculate_monolith()`, aby nowe
tryby nie dopisywały kolejnych warunków do starej funkcji. Przenoszenie jej
wnętrza odbywa się małymi krokami z buildem po każdym kroku.

## Assist Limits

Kolejność ograniczeń Legacy została wydzielona do
`assist_limits_apply_legacy()`:

1. rampa niskiego napięcia,
2. ograniczenie temperatury sterownika,
3. prawne wygaszanie według prędkości.

Progi i kolejność obliczeń nie zostały zmienione.

## Ride Control

`ride_control_update_request()` jest teraz jedynym wyborem silnika jazdy.
`RIDE_ENGINE_LEGACY` pozostaje domyślny, a `RIDE_ENGINE_TSDZ` uruchamia nową
ścieżkę Power Linear. Walk zachowuje priorytet i do czasu wydzielenia własnego
modułu używa dotychczasowej ścieżki Legacy.

## Assist Dynamics

Dotychczasowa rampa `Iq` została przeniesiona do
`assist_dynamics_apply_legacy()`. Moduł zachowuje:

- natychmiastowe cięcia hamulca, ruchu wstecz i przegrzania,
- zależność tempa od prędkości i kadencji,
- szybszą rampę używaną przez Walk Assist,
- dotychczasową arytmetykę stałoprzecinkową.

Po rampie `ride_control` buduje `motor_command_t` i przekazuje go przez
`motor_core_set_command()`. `main` dostarcza tylko `ride_control_input_t` z
aktualną prędkością, kadencją, limitami i stanem cięcia bezpieczeństwa.

## Power Linear

Build `0.0149` rozwija `assist_modes` z pierwszym nowym trybem. Power Linear
używa istniejącego przeliczenia mocy człowieka EBICS, współczynnika wsparcia
poziomu oraz wzorca TSDZ2 `requested current = motor power / battery voltage`.
Żądany prąd jest zamieniany na natywne jednostki `Iq` przez `CAL_I` i
ograniczany przed przekazaniem do wspólnej dynamiki.

Wewnętrzne obliczenie zachowuje miliwaty do momentu dzielenia przez napięcie.
Każdy poziom ma pola `assist_without_rotation` i
`without_rotation_threshold_mv`. Gdy opcja jest świadomie włączona, poprawny
sygnał momentu ponad próg tworzy wyłącznie lokalne `cadence_for_assist=1`;
globalne `MS.cadence`, snapshot wejść i zachowanie Legacy nie są modyfikowane.
Wszystkie poziomy domyślne mają tę opcję wyłączoną.

Tryb jest dostępny przez `RIDE_ENGINE_TSDZ`, ale
`RIDE_ENGINE_DEFAULT=0` pozostawia Legacy jako bezpieczny domyślny silnik.
Power nie steruje FOC, PWM ani rampami bezpośrednio.

Przejściowe funkcje `motor_core_legacy_set_*` zostały usunięte. Inicjalizacja,
watchdog komunikacji, zwykła jazda i autodetekcja Halla korzystają z tego
samego interfejsu komendy. Bezpośrednie zapisy finalnego `Iq/Id` występują
wyłącznie w `motor_core.c`.
