# Audyt komend HMI/CAN i bloków Para

Aktualizacja: 2026-07-13
Gałąź: `refactor/ride-core`
Status: audyt firmware + jednego lokalnego logu; kod aplikacji Canable nie jest
dostępny w tym workspace.

## 1. Decyzja

Nie przydzielać nowych pól Ride Core do bajtów wyglądających na wolne w
`Para0`, `Para1` ani `Para2`. Nie przydzielać też nowej komendy wyłącznie
na podstawie braku jej obsługi w `CAN_Display.c`.

Powody:

- bloki `Para0/1/2` są częścią istniejącego kontraktu HMI/BESST;
- HMI używa komend, których aktualny firmware świadomie nie obsługuje, np.
  `0x6400` i `0x6401`;
- lokalny log zawiera również `0xF203` oraz deweloperskie `0x0203/0x0204`;
- bajt 63 każdego bloku jest checksumą;
- pełny profil Ride Core nie mieści się bezpiecznie w pozostałych bajtach.

Nowa konfiguracja ma dostać osobny, wersjonowany blok multiframe. Jego bazowy
numer komendy pozostaje nieprzydzielony do czasu audytu kodu Canable i szerszych
logów z HMI/BESST.

## 2. Format rozszerzonego identyfikatora

Firmware rozkłada 29-bitowy identyfikator następująco:

| Bity | Pole | Zakres |
|---|---|---|
| 0..15 | `command` | `0x0000..0xFFFF` |
| 16..18 | `operation` | 0..7 |
| 19..23 | `target` | 0..31 |
| 24..28 | `source` | 0..31 |

Operacje używane przez obecny kod:

| Kod | Nazwa w firmware | Znaczenie |
|---:|---|---|
| 0 | `WRITE_CMD` | zapis / ramka operacyjna |
| 1 | `READ_CMD` | odczyt |
| 2 | `NORMAL_ACK` | potwierdzenie |
| 3 | `ERROR_ACK` | błąd/specjalna odpowiedź |
| 4 | `LONG_START_CMD` | początek danych multiframe |
| 5 | `LONG_TRANG_CMD` | środkowa ramka multiframe |
| 6 | `LONG_END_CMD` | ostatnia ramka multiframe |
| 7 | `LONG_WARNING_CMD` | ramka ostrzegawcza/broadcast |

Znane węzły: kontroler `source=2`, HMI `source=3`, BESST/Canable
`source=5`, broadcast `target=31`. W logach narzędzie CAN ustawia dodatkowo
bit EFF `0x80000000`; nie należy go mylić z częścią identyfikatora firmware.

## 3. Komendy zajęte lub zarezerwowane

| Komenda / zakres | Rola | Stan |
|---|---|---|
| `0x0203`, `0x0204` | deweloperska telemetria FOC | Zaobserwowane w logu; normalnie OFF |
| `0x1200` | warning / brake broadcast | Aktywna |
| `0x3000` | status broadcast | Aktywna |
| `0x3005` | reset do bootloadera | Aktywna |
| `0x3100` | emulacja czujnika momentu/kadencji | Deweloperska, domyślnie OFF |
| `0x3200` | SOC, dystans, kadencja, moment, zasięg | Aktywna telemetria |
| `0x3201` | prędkość, prąd, napięcie, temperatury | Aktywna telemetria |
| `0x3202` | keepalive HMI | Aktywna |
| `0x3203` | limit prędkości i obwód koła | Aktywny odczyt/zapis |
| `0x3205` | kalorie | Aktywna telemetria |
| `0x320F` | status active broadcast | Aktywna |
| `0x6000` | producent / hardware ID | Aktywna multiframe |
| `0x6001` | EBICS + wersja builda | Aktywna multiframe |
| `0x6002` | model | Aktywna multiframe |
| `0x6003` | informacja silnika i specjalny tag odpowiedzi HMI | Aktywna/specjalna |
| `0x6007` | błędy | Aktywna multiframe |
| `0x6010` | `Para0` | Aktywny blok 64 B |
| `0x6011` | `Para1` | Aktywny blok 64 B |
| `0x6012` | `Para2` + marker zakończenia HMI | Aktywny blok 64 B |
| `0x6017` | dokładna telemetria kalibracji Halla (`FW-022`) | Aktywna od 0.0187, tylko Canable/BESST (`source=5`), odczyt 36 B |
| `0x6025` | telemetria czujnika nacisku i stan kalibracji | Aktywna, tylko Canable/BESST (`source=5`) |
| `0x6026` | operacje kalibracji nacisku EBICS | Aktywna, tylko Canable/BESST (`source=5`): start/capture/commit/cancel/default |
| `0x6029` | diagnostyka jazdy v2 | Aktywna, tylko Canable/BESST (`source=5`): blokady wspólne, żądanie `i_q`, PWM |
| `0x602B` | próg napięcia pełnej baterii dla SOC 100% | Aktywna, tylko Canable/BESST (`source=5`), CRC-8/SMBUS |
| `0x6101` | reset ustawień fabrycznych | Aktywna; historyczny przycisk `CalibrateTorqueSensor`, NIE używać do kalibracji EBICS |
| `0x6200` | autodetekcja / kalibracja pozycji | Aktywna |
| `0x62D9` | `TS_coeff` / startup angle | Aktywna |
| `0x6300..0x6304` | operacyjne ramki HMI | Zakres zarezerwowany; aktywne `0x6300`, `0x6303` |
| `0x6400`, `0x6401` | zapytania wersja/model | Zaobserwowane; świadomie bez odpowiedzi |
| `0xF203` | komenda z lokalnego logu | Znaczenie nieustalone; traktować jako zajętą |

Komendy `0x0000..` widoczne w logu z operacjami 5/6 są indeksami fragmentów
trwającego transferu multiframe, a nie wolnym zakresem aplikacyjnym.

## 4. Rzeczywista mapa bloków Para

„Nieużywane przez aktualny parser” nie oznacza „wolne w protokole HMI”.

### Para0 — komenda 0x6010

| Bajty | Użycie firmware |
|---|---|
| 2, 4, 6, 8, 9 | filtr/„Ride Mode” poziomów 1..5 |
| 12..13, 16..17, 20..21, 24..27 | progi `TQO_threshold[1..5]`, 16-bit LE |
| 63 | suma bajtów 0..62 modulo 256 |
| pozostałe | nieodczytywane przez obecny parser; status protokołu nieznany |

### Para1 — komenda 0x6011

| Bajty | Użycie firmware |
|---|---|
| 0..4 | napięcia i limit prądu baterii |
| 7..12 | pojemność, limit fazowy, limp SOC, wykładnik kadencji |
| 14 | `legalflag` |
| 18..21 | kierunek, przełożenie, impulsy koła, decay |
| 24..25 | `MagicNumber` |
| 34..39 | throttle, Walk Current, boost/PAS/ramp Legacy |
| 41,43,45,47,48 | limity prądu poziomów |
| 50,52,54,56,57 | limity prędkości poziomów |
| 60..61 | prędkość Walk Assist |
| 63 | suma bajtów 0..62 modulo 256 |
| pozostałe | nieodczytywane przez obecny parser; status protokołu nieznany |

### Para2 — komenda 0x6012

| Bajty | Użycie firmware |
|---|---|
| 0..29 | `assist_profile` (30 B) |
| 31..35 | czas Extended Boost poziomów 1..5 |
| 37..41 | siła Extended Boost poziomów 1..5 |
| 63 | suma bajtów 0..62 modulo 256 |
| 30, 36, 42..62 | nieodczytywane przez obecny parser; status protokołu nieznany |

Checksumy są generowane przy budowaniu bloków. Obecny odbiornik nie sprawdza
ich przed `parse_DPparams()`, więc nie mogą być jedyną ochroną nowego formatu.

## 5. Zapis trwały

Firmware nie zapisuje surowych `Para0/1/2` do flasha. Po odebraniu transferu:

1. dane trafiają do tablic Para,
2. `parse_DPparams()` aktualizuje `MotorParams_t`,
3. cały `MotorParams_t` jest zapisywany przez `write_virtual_eeprom()`,
4. po starcie `parse_MOparams()` odtwarza bloki Para i checksumy.

Nowy blok Ride Core nie powinien powiększać `MotorParams_t` bez wersjonowania
układu flash. Potrzebuje jawnego nagłówka co najmniej:

- magic,
- wersja schematu,
- długość payloadu,
- rewizja/generacja,
- CRC32 całego payloadu.

## 6. Logiczny plan nowych operacji

Numery komend są nadal `null`. Po wyborze bezpiecznej bazy potrzebne są osobne
operacje:

| Offset logiczny | Operacja | Efekt |
|---:|---|---|
| +0 | capabilities/schema | wersja, rozmiar, obsługiwane operacje |
| +1 | read saved | odczyt konfiguracji zapisanej we flashu |
| +2 | read runtime | odczyt konfiguracji działającej w RAM |
| +3 | apply RAM | walidacja i aktywacja bez zapisu flash |
| +4 | save flash | trwały zapis wcześniej zwalidowanej konfiguracji |
| +5 | revert RAM | powrót RAM do ostatniej zapisanej wersji |

`save flash` nie może przyjmować innego payloadu niż ten, który przeszedł
walidację `apply RAM`. Odpowiedź błędu powinna wskazać klucz pola oraz rodzaj
błędu zakresu/reguły.

## 7. Warunki przydzielenia numerów

1. Udostępnić kod źródłowy aktualnego Canable i zebrać używane command ID.
2. Zebrać pełne logi: start HMI, ekran Info, ustawienia, zapis profili, BESST.
3. Wybrać jeden prywatny zakres niezajęty w kodzie i logach.
4. Zarezerwować cały zakres w jednym pliku, nie pojedyncze ID w kodzie.
5. Dopiero wtedy wpisać `wire_id` do YAML i uruchomić generator.
