# FW-061 — poprawna klasyfikacja ruch/postój, uzbrajanie blokady i pełna diagnostyka zerowania

- **Data:** 2026-07-30
- **Status:** ZAIMPLEMENTOWANE, **niezbudowane i nieprzetestowane na sprzęcie**
  (build po stronie właściciela).
- **Zakres:** firmware M820 (`torque_input.c/.h`, `main.c`, `CAN_Display.c`),
  telemetria `0x6025` v2, parser i zakładka Torque w Canable.
- **Powiązane:** `FW-058_COAST_REZERO_RATE.md`, `FW-059_COAST_SAMPLE_QUALITY.md`.
- **Poza zakresem:** ruszanie z miejsca, charakterystyka wspomagania, PAS, rampy
  Iq. Nic z tych rzeczy nie było dotykane.

---

## 1. Błąd klasyfikacji ruch/postój (poprawka 1)

`bike_moving` było odczytywane **w chwili zakończenia wybiegu**. To źle z dwóch
powodów:

- wybieg rozpoczęty w jeździe i zakończony po zatrzymaniu roweru był
  klasyfikowany jako **postój**, więc omijał blokadę 60 s,
- `MS.Speedx100` poniżej ok. 3 km/h okresowo **spada do zera** między impulsami
  koła, więc nawet w ruchu klasyfikacja bywała fałszywa.

Teraz `main.c` **zatrzaskuje** informację o ruchu na cały epizod bez pedałowania:

```
reset     : pas_idle_ticks == 0            (pedałowanie wznowione = nowy epizod)
ustawienie: krawędź impulsu koła
         || MS.Speedx100 >= TQ_RECAL_MOVING_X100
         || Speed_counter < SPEED_STOP_TICKS
```

Trzy niezależne źródła połączone sumą logiczną przez cały epizod. Zmienna startuje
w stanie **„ruch"**, więc stan nieznany jest traktowany jako jazda — zgodnie
z wymaganiem, że niepewność ma skutkować blokadą.

**Konsekwencja, o której trzeba wiedzieć:** wybieg, który zaczął się w jeździe,
pozostaje „jazdą" także wtedy, gdy stoisz na światłach. Nieograniczone zerowanie
na postoju z FW-058 obejmuje więc tylko epizody rozpoczęte już na postoju.
To jest wprost wymagany przypadek testowy 1, ale w praktyce oznacza, że blokada
60 s obowiązuje niemal zawsze.

## 2. Uzbrajanie blokady (poprawka 2)

`apply_offset_step()` zwraca teraz **rzeczywisty krok po ograniczeniu do ±5 mV**.
Blokada 60 s uzbraja się wyłącznie, gdy krok jest różny od zera; ocena z `diff=0`
raportuje `NO_CHANGE` i **nie blokuje** następnej potrzebnej korekty. Korekta
uznana za wykonaną w jeździe zawsze uruchamia blokadę.

## 3. Pasmo bez zmian (poprawka 3)

`TQ_RECAL_BAND_MV` = 30, `TQ_REACQUIRE_MAX_MV` = 40, `TQ_REACQUIRE_COASTS` = 3 —
**nietknięte**. Przekroczenie 40 mV nie jest już jednak nieme: dostaje osobny
wynik `OUT_OF_REACQUIRE_RANGE` i własny licznik.

## 4. Pobieranie próbki FW-059 (poprawka 4)

Bez zmian: okno po 5 s bez PAS i przy `i_q=0`, próbka zbierana przez 0,5 s,
zamrażana, bramka `max-min <= 10 mV`, krok 5 mV. Żadna z tych wartości nie
została ruszona.

## 5. Diagnostyka — `0x6025` w wersji 2 (poprawka 5)

Wybrano telemetrię czujnika momentu, nie `0x6029` — to jej naturalne miejsce.
Blok rośnie z 24 B do **56 B**, pierwsze 22 bajty bez zmian, CRC na końcu.
Parser Canable czyta v1 i v2.

| Pole | Bajty |
|---|---|
| `raw_native` | 22–23 |
| zamrożony kandydat | 24–25 |
| rozrzut okna (max−min) | 26–27 |
| ostatni krok, ze znakiem | 28–29 |
| pozostała blokada [s] | 30 |
| flagi: `coast_active`, `coast_was_moving`, `candidate_stable` | 31 |
| wynik ostatniej oceny (enum) | 32 |
| liczniki: okna rozpoczęte / ukończone / korekty | 34–39 |
| liczniki odrzuceń: TOO_SHORT, UNSTABLE, LOCKOUT, OUT_OF_RANGE, IMPLAUSIBLE | 40–49 |
| licznik NO_CHANGE | 50–51 |
| skumulowane przesunięcie zera | 52–53 |

`zero_effective_native` był już w v1 (bajty 6–7). **Liczniki są kumulacyjne i nie
zerują się przy odczycie.**

Wynik oceny: `NONE`, `APPLIED`, `NO_CHANGE`, `TOO_SHORT`, `UNSTABLE`, `LOCKOUT`,
`OUT_OF_REACQUIRE_RANGE`, `IMPLAUSIBLE_RAW`.

**Jedno zastrzeżenie:** zadana lista enumów nie ma wartości na „dryf 31–40 mV
czeka na potwierdzenie trzech próbek". Mapuję ten przypadek na `NO_CHANGE`, więc
`NO_CHANGE` oznacza dwie różne rzeczy: zero już na celu **albo** trwa
potwierdzanie dryfu. Rozróżnia je licznik odrzuceń `OUT_OF_RANGE` w połączeniu
z kandydatem i rozrzutem, ale nie da się tego odczytać z samego enuma. Jeśli ma
być rozdzielone, potrzebna jest dodatkowa wartość `REACQUIRE_PENDING`.

W Canable doszła sekcja **„Automatic zero re-calibration (coast)"** w zakładce
Torque, z wynikiem, kandydatem, krokiem, blokadą, wszystkimi licznikami i
jednozdaniowym wyjaśnieniem każdego wyniku.

## 6. Kontrakt `0x6029` (poprawka 6)

Potwierdzony rozjazd. Firmware pakuje bit 2 = hamulec, bit 3 = usterka czujnika
momentu; parser eVistDrive nazywał je `pedal_release_active` i `release_latched`,
a interfejs wyświetlał jako „Pedal release / latched". Poprawione na
`brake_active` i `torque_fault`, etykieta w interfejsie zmieniona na
**„Brake / torque fault"**. Przy okazji parser wystawia pozostałe cztery bity,
które dotąd były odrzucane (cofanie, kalibracja, utrata komunikacji, PWM).

Wersja `0x6029` **nie była podnoszona** — układ bajtów się nie zmienia, poprawiono
wyłącznie nazwy po stronie odczytu.

## 7. Testy (poprawka 7)

`tests/fw058_coast_rezero.js` pokrywa FW-058, FW-059 i FW-061 → **PASS**.
Doszedł port zatrzasku ruchu z `main.c` i wszystkie wymagane przypadki:

- wybieg w ruchu zakończony po zatrzymaniu **nadal jest ruchem**,
- impulsy koła przy `Speedx100 = 0` liczą się jako ruch,
- prawdziwy postój nadal klasyfikuje się jako postój,
- `diff=0` nie uzbraja blokady, a następna realna korekta przechodzi,
- dwa wybiegi w ruchu 30 s po sobie → drugi raportuje `LOCKOUT`, nie `UNSTABLE`,
- ponad 60 s → obie korekty dozwolone,
- dryf 30 mV natychmiast, 31 i 40 mV po trzech zgodnych próbkach, 41 mV →
  `OUT_OF_REACQUIRE_RANGE` i zero bez zmian,
- niestabilne okno zwiększa licznik `UNSTABLE`,
- brak okna 5,5 s → `TOO_SHORT`, licznik `UNSTABLE` pozostaje zerowy,
- nacisk po zamrożeniu próbki nie rusza zera,
- liczniki są kumulacyjne.

Zerowanie startowe (`torque_input_startup_zero()`) omija `apply_offset_step()`,
więc pozostaje nietknięte.

## 8. Kryterium odbioru — do wykonania na rowerze

Co najmniej 10 prób: jazda → wybieg ponad 5,5 s → lekkie ponowne naciśnięcie.
Zero nie może zmieniać się przy każdym dopięciu, a każda zmiana musi być widoczna
w sekcji „Automatic zero re-calibration" wraz z kandydatem, krokiem i powodem.

Kontrola braku regresji: ruszanie z miejsca bez zmian, charakterystyka
wspomagania bez zmian.
