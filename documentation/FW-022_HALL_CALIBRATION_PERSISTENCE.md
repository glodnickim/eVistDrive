# Karta zmiany FW-022 — kalibracja Halla i trwałe wartości domyślne

- **Data:** 2026-07-23
- **Status:** DIAGNOZA POTWIERDZONA CZĘŚCIOWO. Telemetria została wdrożona,
  zbudowana i wgrana jako `0.0187`. Kalibracja naprawiła Walk Assist przed
  flashowaniem, lecz dokładne wartości po kalibracji nie zostały wtedy
  odczytane. Finalny zestaw wartości i poprawka trwałości czekają na następne
  podłączenie roweru.
- **Zakres:** firmware sterownika M820; Canable służy tylko do wywołania
  kalibracji i surowego odczytu.
- **Powiązane:** `0.0186` (naprawa `Para1`), diagnostyka jazdy `0x6029`,
  kalibracja pozycji `0x6200`.

---

## 1. Prostym językiem — co znaleziono

Brak wspomagania miał dwie niezależne warstwy:

1. W `0.0185` uszkodzony rekord `Para1` zawierał m.in. próg podnapięciowy
   `0xFFFF`, więc wspólny limiter zerował TSDZ, Legacy i Walk Assist. Naprawiono
   to w `0.0186`.
2. Po odblokowaniu limitera Walk Assist dochodził już do sterowania silnikiem,
   ale silnik tylko buczał i nie obracał koła. Kalibracja pozycji Halla
   `0x6200` naprawiła ten objaw: użytkownik potwierdził, że Walk Assist zaczął
   poprawnie obracać kołem.

Walk Assist nie potrzebuje nacisku na pedał i nie korzysta z algorytmu jazdy
TSDZ/Legacy. Jeżeli jego żądanie prądu dochodzi do FOC, a wirnik tylko buczy,
przyczyny trzeba szukać w komutacji: kolejności Halla, kątach przejść albo
`angle_correction`.

---

## 2. Dowody z testu sprzętowego

### Potwierdzone

- Firmware `0.0186` po starcie naprawił krytyczne pola `Para1`.
- Napięcie akumulatora było poprawne, czujnik nacisku miał ważny stan
  (`zero` około `473–474 mV`, `span=1139`), a hamulec, torque fault i watchdog
  CAN nie blokowały napędu.
- Podczas Walk Assist diagnostyka pokazała aktywny PWM oraz żądanie docierające
  do silnika (`peak iq_request=157`, `peak iq_setpoint=146`).
- Przed kalibracją koło nie ruszało, a silnik buczał jak przy złej
  synchronizacji.
- Komenda `0x6200` dostała prawidłowy ACK i procedura zakończyła fazę prądową.
- Po kalibracji użytkownik potwierdził: **Walk Assist działa i obraca kołem**.

### Po wgraniu 0.0187

Odczyt `0x6017` zwrócił:

| Pole | Odczyt |
|---|---:|
| `hall_order` | `-1` |
| `Hall_13` | `1825361405` |
| `Hall_32` | `-1789569490` |
| `Hall_26` | `-966367405` |
| `Hall_64` | `-322122295` |
| `Hall_45` | `381775140` |
| `Hall_51` | `1169185830` |
| `angle_correction` | `0` |
| stan kalibracji | `1` (normalna praca) |

Są to dokładnie stare wartości skompilowane w `src/main.c`, nie zapis nowego
wyniku, którego oczekiwano po kalibracji. Po flashowaniu użytkownik zgłosił
ponowny brak wspomagania.

### Czego nie udało się potwierdzić

- Nie odczytano `0x6017` bezpośrednio po pierwszej skutecznej kalibracji, bo
  telemetria nie istniała jeszcze w `0.0186`.
- Log jazdy po `0.0187` nie został zapisany.
- Nie rozstrzygnięto jeszcze, czy stronę wirtualnego EEPROM kasuje bootloader,
  czy wynik traci się w innej części sekwencji startowej. Związek
  kalibracja → działający Walk Assist jest potwierdzony; mechanizm utraty po
  flashu pozostaje hipotezą do sprawdzenia.

---

## 3. Zmiana w 0.0187 — odczyt `0x6017`

Dodano techniczną telemetrię tylko dla Canable/BESST (`source=5`). Odczyt nie
zmienia ustawień i nie uruchamia silnika.

- **CAN command:** `0x6017`
- **Operacja:** READ
- **Odpowiedź:** multiframe, 36 B
- **Wywołanie przez bieżący serwer Canable:** `READ_RAW:2:96:23`

| Bajty | Typ LE | Zawartość |
|---:|---|---|
| 0–1 | ASCII | magic `HC` |
| 2 | `uint8` | wersja formatu = `1` |
| 3 | `uint8` | `hall_angle_detect_flag` |
| 4–7 | `int32` | `i32_hall_order` |
| 8–11 | `int32` | `Hall_13` |
| 12–15 | `int32` | `Hall_32` |
| 16–19 | `int32` | `Hall_26` |
| 20–23 | `int32` | `Hall_64` |
| 24–27 | `int32` | `Hall_45` |
| 28–31 | `int32` | `Hall_51` |
| 32–35 | `int32` | `MP.angle_correction` |

Komenda jest celowo odrzucana dla HMI i innych źródeł. Nie zmienia protokołu
wyświetlacza.

---

## 4. Obecny zapis wirtualnego EEPROM

Kąty są przechowywane na początku strony `0x0803F000`, przed `MotorParams_t`:

1. `i32_hall_order`,
2. sześć kątów Halla,
3. od offsetu 28 — `MotorParams_t`, w tym `angle_correction`.

`write_virtual_eeprom()` kasuje całą stronę i zapisuje ponownie kąty oraz
parametry. Odczyt uznaje kąty za istniejące, jeśli tylko drugi zapisany word
nie jest równy `0xFFFFFFFF`. Brakuje:

- magic,
- wersji formatu,
- długości,
- CRC,
- rozróżnienia pełnego i częściowego zapisu.

To za słaba walidacja. Pusta pamięć pozostawia skompilowane wartości Halla,
natomiast `MP.angle_correction` zaczyna od `0`.

---

## 5. Następny test — dokładna kolejność

1. Wgrać lub pozostawić `0.0187`.
2. Zdjąć łańcuch, stabilnie unieruchomić rower i nie dotykać silnika.
3. Odczytać początkowe `0x6017`.
4. Uruchomić kalibrację `0x6200`.
5. Poczekać, aż `hall_angle_detect_flag` wróci do `1` i prąd spadnie do zera.
6. **Bez restartu i bez flashowania** odczytać `0x6017`.
7. Sprawdzić Walk Assist ze zdjętym łańcuchem.
8. Zapisać odczytane wartości jako domyślne w kodzie.
9. Zbudować następną wersję, wgrać ją i ponownie odczytać `0x6017`.
10. Sprawdzić kolejno Walk Assist, Legacy i TSDZ pod bezpiecznym obciążeniem.

Jeżeli podczas próby silnik tylko buczy, nie zwiększać poziomu ani czasu
zasilania. Przerwać test i wrócić do kalibracji.

---

## 6. Plan finalnej poprawki

Po uzyskaniu dokładnego odczytu:

- zastąpić stare stałe Halla wartościami skalibrowanymi dla tego silnika,
- ustawić skalibrowaną wartość domyślną `angle_correction`,
- zachować możliwość ponownego wykonania `0x6200`,
- dodać wiarygodną walidację rekordu Halla (magic, wersja i CRC),
- rozróżnić wartości domyślne od poprawnego rekordu zapisanego przez
  kalibrację,
- sprawdzić, czy bootloader kasuje stronę `0x0803F000` podczas aktualizacji.

Wpisanie wartości domyślnych rozwiąże problem po flashowaniu nawet wtedy, gdy
bootloader nie zachowuje strony EEPROM. Magic/CRC zapobiegną natomiast
zaakceptowaniu zapisu pustego lub częściowo uszkodzonego.

---

## 7. Build i pliki

**Build testowy:**

- plik: `.build/0.0187_M820_BL820.bin`
- rozmiar: `81156 B`
- SHA-256:
  `04EF08B1A5BF71FA8CFE10B9836CC51D12E920DAEAE95BBC2D7993869FED9D07`
- wynik flashowania: sukces

**Kod:**

- `src/CAN_Display.c` — odczyt `0x6017`,
- `src/main.c` — kąty Halla, kalibracja `0x6200`, zapis i odczyt EEPROM,
- `inc/main.h` — `MotorState_t.hall_angle_detect_flag` i
  `MotorParams_t.angle_correction`.

**Dokumentacja/protokół:**

- `CHANGELOG.md`,
- `documentation/EBICS_ZMIANY_I_KONFIGURACJA_PL.md`,
- `protocol/HMI_COMMAND_AUDIT.md`,
- `protocol/ebics_config_schema.yaml`.

**Poza zakresem tej wersji:** zmiana FOC, nowe strojenie TSDZ/Legacy, zmiana
kalibracji czujnika nacisku i automatyczne uruchamianie kalibracji Halla.
