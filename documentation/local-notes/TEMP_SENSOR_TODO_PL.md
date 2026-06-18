# Temperatura — do zrobienia: obsługa błędu czujnika (M820)

Status: **odłożone na przyszłość** (świadomie nie zaimplementowane na gałęzi `M820-temp-sensor`).

## Kontekst
Na M820 nie ma czujnika temperatury silnika — NTC czytany przez `adc_value[6]` (PB1 / ADC ch9)
mierzy temperaturę **sterownika**. Wartość `MS.int_Temperature = T_NTC(...)`:
- jest wysyłana na HMI w ramce 0x3201 jako temperatura silnika (bajt [7]) oraz w polu calories (0x3205),
- napędza wbudowane ograniczanie mocy (ramp-down 75→90 °C, [src/main.c](../../src/main.c)).

Odczyt jest filtrowany EMA /16, ale **nie ma detekcji uszkodzenia czujnika**.

## Problem do rozwiązania
NTC odczytywany jest jednym kanałem ADC bez kontroli poprawności:
- **rozwarty NTC** (przerwany przewód / brak czujnika) → `adc_value[6]` ≈ 4095 → `T_NTC` zwraca
  fałszywie niską/ujemną temperaturę → ramp-down nigdy nie zadziała mimo realnego przegrzania;
- **zwarty NTC** (zwarcie do masy) → `adc_value[6]` ≈ 0 → fałszywie wysoka temperatura →
  ramp-down niepotrzebnie obcina moc.

## Wymagana logika (do dorobienia)
1. Wykryć skrajne wartości surowego ADC (np. `adc_value[6] < PROG_ZWARCIE` lub `> PROG_ROZWARCIE`).
2. Oznaczyć stan błędu (flaga w `MotorState_t`).
3. Fail-safe: nie wpuszczać błędnej wartości do ramp-down — np. ograniczyć moc (tryb bezpieczny)
   albo utrzymać ostatnią wiarygodną temperaturę przez krótki czas.
4. Zasygnalizować błąd na HMI (np. wartość specjalna / kod w polu temperatury).

## Punkty zaczepienia w kodzie
- Odczyt + TODO: [src/main.c](../../src/main.c) — blok `MS.int_Temperature = T_NTC(temp_adc_cumulated >> 4);`
- Ramp-down: [src/main.c](../../src/main.c) — `map(MS.int_Temperature, 75, 90, ...)`
- Wysyłka na HMI: [src/CAN_Display.c](../../src/CAN_Display.c) — case `0x3201`, bajt [7]
