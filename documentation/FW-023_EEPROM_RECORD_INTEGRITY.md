# Karta zmiany FW-023 — integralność rekordu wirtualnego EEPROM

- **Data:** 2026-07-23
- **Status:** WDROŻONE I POTWIERDZONE NA SPRZĘCIE (`0.0191`, 2026-07-23).
  Zakres 1+2+3 zaakceptowany, zaimplementowany i zbudowany razem z wartościami
  domyślnymi z FW-022. Rekord EEPROM zapisuje się i waliduje poprawnie, silnik
  wspomaga na obu silnikach jazdy.
- **Zakres:** firmware sterownika M820, wyłącznie zapis i odczyt strony
  parametrów `0x0803F000`. Bez zmian w FOC, strojeniu ride core/Legacy i protokole
  wyświetlacza.
- **Powiązane:** [[FW-022]] (wartości domyślne kątów Halla), `0.0186` (naprawa
  `Para1` z `0xFFFF`), `0.0185` (awaria, która ujawniła problem).
- **Wersja docelowa:** `0.0191` (ostatni zbudowany firmware to `0.0190`).

---

## 1. Prostym językiem — co jest nie tak

Przy każdym zapisie ustawień firmware **najpierw kasuje całą stronę pamięci, a
dopiero potem zapisuje ją od nowa**. Przez te kilkadziesiąt milisekund w
pamięci nie ma kompletnych danych. Jeśli w tym momencie zniknie zasilanie,
zostaje rekord zapisany w połowie — a przy starcie firmware nie ma jak poznać,
że jest niekompletny, bo sprawdza tylko **jedno słowo**.

Skutek takiego rekordu to dokładnie objawy, które już wystąpiły: silnik buczy i
nie kręci kołem (zepsute kąty Halla) albo wspomaganie znika całkowicie
(`voltage_min` = `0xFFFF`, awaria `0.0185`).

Ten sam firmware zapisuje stan naładowania baterii w sposób **odporny na zanik
zasilania**. Ta karta przenosi ten sprawdzony wzorzec na stronę z ustawieniami.

---

## 2. Dowody z kodu

### 2.1. Zapis kasuje i przepisuje całą stronę

`write_virtual_eeprom()` w `src/main.c`:

```c
fmc_erase_pages();                                  // cała strona -> 0xFFFFFFFF
fmc_program_hall_angles();                          // 7 słów
fmc_multi_word_program(FMC_OFFSET_MP, (uint8_t*)&MP, (sizeof(MP)+3)/4);
```

### 2.2. Walidacja to jedno słowo

`read_virtual_eeprom()` uznaje całą stronę za ważną, jeśli drugie słowo (czyli
`Hall_13`) nie jest puste:

```c
if(0xFFFFFFFF != (*(ptrd+1))){ ... }
```

Następnie `memcpy` bezwarunkowo wczytuje całe `MotorParams_t`. Brak magic,
wersji, długości i sumy kontrolnej.

### 2.3. Okno ryzyka i jego długość

| Faza | Czas (rząd wielkości) | Zanik zasilania tutaj |
|---|---|---|
| kasowanie strony | ~20–40 ms | strona pusta → wartości domyślne, **bezpiecznie** |
| `hall_order` | ~50 µs | strona pusta → **bezpiecznie** |
| `Hall_13` | ~50 µs | ⚠️ od tej chwili strona liczy się jako ważna |
| pozostałe kąty + całe `MP` (~140 słów) | ~10 ms | 💥 **rekord uszkodzony** |

Groźne jest ostatnie ~10 ms z ~40 ms całości. Czasy programowania flasha są
rzędu wielkości z noty katalogowej, nie zmierzone.

### 2.4. Okno otwiera się przy każdym zapisie ustawień

`write_virtual_eeprom()` jest wywoływane z:

| Miejsce | Kiedy |
|---|---|
| `CAN_Display.c:154` | zapis `0x62D9` (`TS_coeff`) |
| `CAN_Display.c:243` | zapis parametrów z Canable/HMI |
| `CAN_Display.c:354` | zapis `0x3203` (limit prędkości i obwód koła) |
| `parser.c:272` | autonaprawa `Para0`/`Para1` po starcie |
| `parser.c:340` | `InitEEPROM()` — reset fabryczny `0x6101` |
| `main.c:1688` | kalibracja czujnika nacisku |
| `main.c:2790` | koniec kalibracji Halla `0x6200` |

Zapis `0x3203` jest **bezwarunkowy** — nie sprawdza, czy przysłane wartości w
ogóle różnią się od zapisanych. Jeśli wyświetlacz wysyła tę ramkę przy każdym
włączeniu roweru, oznacza to jedno kasowanie strony na każdą jazdę.

### 2.5. Co konkretnie zostaje po przerwanym zapisie

Wszystko niezapisane czyta się jako `0xFFFFFFFF`:

| Pole | Wartość | Skutek |
|---|---|---|
| `Hall_32`, `Hall_26`, `Hall_64`, `Hall_45`, `Hall_51` | `-1` | pięć z sześciu przejść zapada się w jeden kąt → **silnik buczy, nie kręci** |
| `voltage_min` | `0xFFFF` | limiter zeruje ride core, Legacy i Walk Assist → **brak wspomagania** |
| `speedLimitx100`, `phase_current_max`, `battery_current_max` | `0xFFFF` | bezsensowne limity |

Podpis awarii `0.0185` (`Para1` z `0xFFFF`) jest z tym zgodny.

### 2.6. Druga droga uszkodzenia — bez zaniku zasilania

Zmiana układu `MotorParams_t` przy zachowanej starej stronie. Nowe pola muszą
być dopisywane **na końcu** — stąd powtarzające się komentarze „appended at end
to keep EEPROM offsets stable" w `inc/main.h`. Wstawienie pola w środku
przesuwa wszystkie kolejne i psuje cały rekord przy pierwszym starcie nowego
firmware, deterministycznie.

Bloki z własnym magic (`bank_store_magic`, `tuning_store_magic`,
`torque_cal_magic`, `ride_engine_magic`, `soc_full_magic`) są chronione. Kąty
Halla, `angle_correction`, `voltage_min` oraz limity prądu i prędkości — **nie
są**.

### 2.7. Wzorzec, który już działa

Zapis SOC na stronie 127 (`0x0803F800`) jest odporny na zanik zasilania:

```c
s.crc = soc_crc32((const uint8_t*)&s, 28);   // CRC liczone z całego rekordu
...
for(k=0; k<SOC_SLOT_WORDS; k++) fmc_word_program(addr, w[k]);  // CRC leci OSTATNIE
```

Odczyt odrzuca slot, którego CRC się nie zgadza (`main.c:2260`). Ponieważ CRC
leży na końcu struktury, przerwany zapis nigdy nie da rekordu uznanego za
ważny. Dostępne do ponownego użycia: `soc_crc32()` (CRC32, wielomian
odwrócony `0xEDB88320`).

---

## 3. Zakres przyjęty

Zaakceptowano punkty **1 + 2 + 3**:

1. brak zapisu, gdy nic się nie zmieniło,
2. kontrola sensowności kątów Halla przy odczycie,
3. magic + wersja + długość + CRC zapisywane na końcu rekordu.

**Świadomie poza zakresem:**

- dwie strony na przemian (A/B) — eliminowałaby też *utratę* ustawień przy
  zaniku zasilania, ale wymaga dodatkowej strony 2 kB i potwierdzenia w
  linkerze oraz bootloaderze, że strona 125 jest wolna,
- odkładanie zapisu do zatrzymania silnika,
- migracja starego rekordu bez CRC (uzasadnienie w punkcie 6).

---

## 4. Projekt zmiany

### 4.1. Nowy układ strony `0x0803F000`

Offsety kątów i `MotorParams_t` **pozostają bez zmian**. Stopka dopisywana jest
za `MP`:

| Offset | Rozmiar | Zawartość |
|---:|---:|---|
| 0 | 4 | `i32_hall_order` |
| 4–24 | 24 | `Hall_13`, `Hall_32`, `Hall_26`, `Hall_64`, `Hall_45`, `Hall_51` |
| 28 (`FMC_OFFSET_MP`) | `sizeof(MotorParams_t)` | `MotorParams_t` |
| `FMC_OFFSET_FOOTER` | 16 | stopka rekordu |

```c
#define FMC_OFFSET_FOOTER  (FMC_OFFSET_MP + ((sizeof(MotorParams_t)+3u)/4u)*4u)
#define PARAM_REC_MAGIC    ((uint32_t)0xEB1C5001U)
#define PARAM_REC_VERSION  ((uint16_t)1U)

typedef struct {
    uint32_t magic;    // PARAM_REC_MAGIC
    uint16_t version;  // PARAM_REC_VERSION
    uint16_t length;   // liczba bajtów objętych CRC (= FMC_OFFSET_FOOTER)
    uint32_t reserved; // 0
    uint32_t crc;      // soc_crc32(strona, length) — ZAPISYWANE JAKO OSTATNIE
} param_footer_t;

_Static_assert(FMC_OFFSET_FOOTER + sizeof(param_footer_t)
               <= (FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR),
               "param record does not fit in one page");
```

Efekt uboczny, pożądany: gdy `MotorParams_t` zmieni rozmiar, stopka przenosi
się pod inny offset, stary rekord przestaje przechodzić walidację i firmware
wraca do wartości domyślnych — zamiast po cichu wczytać śmieci. To zamyka też
drogę uszkodzenia z punktu 2.6.

### 4.2. Kolejność zapisu

`write_virtual_eeprom()`:

1. Jeśli rekord w pamięci jest ważny (magic, wersja, długość, CRC) **i** jego
   kąty oraz `MP` są bajt w bajt identyczne z bieżącymi — **nie rób nic**
   (punkt 1 zakresu). Porównanie przez `memcmp` wprost na pamięci flash, bez
   dodatkowego bufora w RAM.
2. `fmc_erase_pages()`.
3. Zapisz siedem słów kątów.
4. Zapisz `MotorParams_t`.
5. Zapisz `magic`, `version`, `length`, `reserved`.
6. **Na samym końcu** zapisz `crc`.

Rekord staje się ważny dopiero w kroku 6. Przerwanie w krokach 2–5 daje rekord
odrzucany przy odczycie.

Krok 1 jest jedynym potrzebnym miejscem na strażnika — wszystkie siedem
wywołań z punktu 2.4 przechodzi przez tę funkcję.

### 4.3. Odczyt i walidacja

`read_virtual_eeprom()`:

1. Wczytaj stopkę. Odrzuć rekord, jeśli `magic`, `version` lub `length` się nie
   zgadzają.
2. Policz `soc_crc32(strona, length)` i porównaj z `crc`. Niezgodność →
   odrzuć.
3. **Rekord odrzucony:** zostaw skompilowane kąty Halla i wartości domyślne
   `MP`, nie wykonuj `memcpy`. Zgłoś to przez `0x6017` (punkt 4.5).
4. **Rekord ważny:** wczytaj kąty i `MP` jak dotychczas, po czym wykonaj
   kontrolę z punktu 4.4.

Startowe sprawdzenie „czy EEPROM był kiedykolwiek zapisany" w `main.c:461-464`
korzysta z tej samej walidacji zamiast z porównania jednego słowa.

### 4.4. Kontrola sensowności kątów Halla

Stosowana po każdym wczytaniu kątów z pamięci:

- `i32_hall_order` musi być `1` albo `-1`;
- sześć kątów posortowanych rosnąco musi mieć **wszystkie sześć odstępów
  (z zawinięciem przez pełny obrót) w przedziale 45°–75°**, licząc w
  arytmetyce `uint32` — zawinięcie obsługuje się samo.

Progi w jednostkach q31: `45° = 536870925`, `75° = 894784875`.

Niespełnienie któregokolwiek warunku → wszystkie siedem wartości wraca do
skompilowanych domyślnych.

Sprawdzenie na rzeczywistych danych:

| Zestaw | Odstępy | Wynik |
|---|---|---|
| skalibrowany (FW-022) | 60°, 62°, 60°, 59°, 60°, 59° | przechodzi |
| stary skompilowany | 69°, 54°, 59°, 66°, 55°, 57° | przechodzi |
| pięć kątów = `-1` (przerwany zapis) | pięć odstępów zerowych | **odrzucony** |

Tolerancja jest więc dość szeroka, żeby nie odrzucić poprawnej kalibracji, i
dość wąska, żeby złapać rekord uszkodzony.

### 4.5. Rozszerzenie telemetrii `0x6017`

Do istniejącego odczytu diagnostycznego (36 B, tylko `source=5`) dochodzi bajt
stanu rekordu, żeby dało się odróżnić trzy sytuacje bez zgadywania:

| Wartość | Znaczenie |
|---:|---|
| `0` | rekord ważny, kąty przeszły kontrolę sensowności |
| `1` | brak ważnego rekordu → wartości domyślne z kodu |
| `2` | rekord ważny, ale kąty odrzucone przez kontrolę sensowności |

Format bumpuje się do wersji `2`. Zmiana dotyczy wyłącznie Canable/BESST i nie
rusza protokołu wyświetlacza.

---

## 5. Pliki do zmiany

| Plik | Zakres | Stan |
|---|---|---|
| `src/main.c` | `param_footer_t`, `param_record_valid()`, `hall_angles_plausible()`, `hall_load_defaults()`, przebudowane `write_virtual_eeprom()` i `read_virtual_eeprom()`, sprawdzenie startowe w `main()`, `_Static_assert` na stopkę | zrobione |
| `src/CAN_Display.c` | `0x6017` format v2: bajt stanu rekordu | zrobione |
| `CHANGELOG.md` | wpis `0.0191` + uzupełnienie `0.0189`/`0.0190` | zrobione |
| `documentation/EBICS_ZMIANY_I_KONFIGURACJA_PL.md` | opis zachowania po aktualizacji | do zrobienia po teście |
| `protocol/HMI_COMMAND_AUDIT.md` | `0x6017` wersja formatu 2 | do zrobienia po teście |

**Odstępstwo od projektu:** `param_footer_t` trafił do `src/main.c`, a nie do
`inc/main.h`. Powód: `soc_slot_t` — bliźniaczy rekord dla strony SOC — też jest
zdefiniowany lokalnie w `main.c`, a struktura nie jest potrzebna nigdzie indziej.
`inc/main.h` pozostaje nietknięty, więc **układ `MotorParams_t` się nie zmienia**.

Bez zmian: `parser.c`, `torque_input.c`, `assist_modes.c`, `tuning_config.c` —
ich bloki mają własne magic i CRC i pozostają nietknięte wewnątrz `MP`.

---

## 6. Konsekwencje i ryzyka

**Jednorazowy powrót ustawień do domyślnych.** Stara strona nie ma stopki, więc
przy pierwszym uruchomieniu `0.0191` zostanie odrzucona. Ustawienia użytkownika
trzeba raz wprowadzić od nowa. Kąty Halla są już wpisane w kodzie (FW-022),
więc **kalibracji `0x6200` nie trzeba powtarzać**.

Rozważono migrację (zaakceptować stary rekord bez stopki, jeśli kąty są sensowne,
i od razu przepisać go ze stopką). **Odrzucona:** wymagałaby jednorazowego
zaufania dokładnie temu rekordowi, którego wiarygodności ta karta dowodzi jako
niewystarczającej — a znany zły rekord z `0.0185` miał sensowne kąty i zepsute
`Para1`. Czysty start jest bezpieczniejszy niż oszczędzenie jednego wprowadzenia
ustawień.

**Czego ta zmiana NIE robi.** Zanik zasilania w trakcie zapisu nadal **kasuje
ustawienia** do domyślnych — chroni przed uszkodzeniem, nie przed utratą.
Zamknięcie i tego wymaga dwóch stron na przemian (poza zakresem).

**Zużycie pamięci flash.** Punkt 1 zakresu zmniejsza liczbę kasowań strony,
więc zmiana działa tu na korzyść.

**Ryzyko regresji.** Główne to błąd w wyznaczeniu offsetu stopki albo w
warunku porównania z punktu 4.2 — pierwszy dałby stale odrzucany rekord
(ustawienia nie zapisują się między jazdami), drugi mógłby zablokować zapis
zmian. Oba wychodzą od razu w teście 7.1 i 7.2.

---

## 7. Plan testu

Rower na stojaku, łańcuch zdjęty, do punktu 7.5 włącznie.

1. **Zapis i odczyt.** Zmienić limit prędkości w aplikacji, wyłączyć i włączyć
   rower. Ustawienie musi się utrzymać. `0x6017` → stan rekordu `0`.
2. **Brak zbędnego zapisu.** Włączyć rower dwa razy bez zmiany czegokolwiek.
   Drugi start nie może wykonać kasowania strony (weryfikacja przez licznik
   zapisów w logu diagnostycznym albo przez czas trwania startu).
3. **Pierwszy start po aktualizacji.** `0x6017` → stan rekordu `1`, ustawienia
   domyślne, kąty Halla równe skalibrowanym z FW-022.
4. **Odporność na przerwanie.** Zmienić ustawienie w aplikacji i **odciąć
   zasilanie w trakcie zapisu** (kilka prób, żeby trafić w okno). Po każdym
   ponownym włączeniu firmware musi wstać z wartościami domyślnymi i
   `0x6017` = `1`. **Nigdy** nie może wstać z buczącym silnikiem ani z brakiem
   wspomagania przy sensownym rekordzie.
5. **Kalibracja nadal działa.** Uruchomić `0x6200`, poczekać na stan końcowy
   `1`, odczytać `0x6017` — nowe kąty, stan rekordu `0`. Po restarcie kąty
   muszą się utrzymać.
6. **Walk Assist** ze zdjętym łańcuchem.
7. **Legacy i ride core** pod bezpiecznym obciążeniem.

Jeżeli w którymkolwiek punkcie silnik tylko buczy — nie zwiększać poziomu ani
czasu zasilania, przerwać test.

---

## 8. Build

- plik: `.build/0.0191_M820_BL820.bin`
- rozmiar: `81912 B` (0.0190 miał `81124 B`, przyrost `+788 B`)
- SHA-256:
  `1F297672E7207E69FEFD23D7F30148089CB2A8537BE85018708F51D9743A131E`
- kompilacja: bez błędów; ostrzeżenia wyłącznie zastane (`-Wpointer-sign`
  w `CAN_Display.c`, nieużywana zmienna `fw_ver` w `main.c`, segment LOAD RWX
  z linkera) — żadne nie pochodzi z kodu FW-022/FW-023
- wynik flashowania: **sukces**, potwierdzony odczytem CAN

### Weryfikacja po wgraniu (2026-07-23, odczyt przez Canable)

- `0x6001` zwraca `eVD 0.0191` — na sterowniku pracuje ten build.
- `0x6017` odpowiada w **formacie 2** (37 B) — nowe pole stanu rekordu działa.
- Wszystkie osiem wartości kalibracji zgadza się **dokładnie** z wpisanymi do
  kodu: `hall_order=1`, kąty `-134/-74/-12/+48/+107/+167°`,
  `angle_correction=+6°`.
- **Stan rekordu = `0`** (rekord ważny, kąty przeszły kontrolę sensowności).
  Oznacza to, że stopka z CRC została poprawnie zapisana i odczytana.
- Odstępy między kątami: `60/62/60/59/60/59°` — kontrola przechodzi z zapasem.
- Sterownik zgłasza `state_number=0` (brak błędu), napięcie `36,89 V`,
  temperatura `22 °C`.

**Uwaga do planu testu:** punkt 7.3 zapowiadał stan rekordu `1` przy pierwszym
starcie. To było błędne oczekiwanie. `main()` przy nieważnym rekordzie wywołuje
`InitEEPROM()`, które od razu zapisuje kompletny rekord ze stopką, więc
`read_virtual_eeprom()` widzi już rekord ważny i ustawia stan `0`. Stan `1`
pojawi się tylko wtedy, gdy sam zapis do pamięci się nie powiedzie.

### Test ruchowy — POTWIERDZONY

- **Walk Assist obraca kołem** — sprawdzone ze zdjętym łańcuchem i z założonym.
- **Silnik ożył i wspomaga w jeździe — zarówno na Legacy, jak i na ride core.**
  Pierwotna awaria (całkowity brak wspomagania) jest usunięta.
- Przez cały test sterownik nie zgłosił błędu (`state_number=0`), napięcie
  trzymało się `36,65–36,89 V` bez zapadania.
- Ponowny odczyt `0x6017` po testach zwrócił rekord bajt w bajt identyczny,
  stan nadal `0` — zapis jest stabilny.

### Otwarte, poza zakresem tej karty

1. **Walk Assist przestrzeliwuje prędkość na stojaku.** Cel to `6,0 km/h`, a
   koło rozpędziło się do `17,2 km/h`. Zabezpieczenie zadziałało poprawnie
   (prąd spadł do zera, koło swobodnie zwalniało), więc pętla regulacji jest
   sprawna. Przyczyną przestrzelenia jest najpewniej brak obciążenia: koło wisi
   w powietrzu, a start WA to z założenia pełny prąd fazowy (`WA_START_PCT=100`).
   **Do weryfikacji na ziemi przed jakimkolwiek strojeniem** — stojak usuwa
   dokładnie to obciążenie, dla którego ten boost powstał.
2. **Ustawienia użytkownika wróciły do fabrycznych** (jednorazowy, zapowiedziany
   skutek odrzucenia starego rekordu bez stopki). Walk Assist ma teraz `6,0 km/h`
   i `30%` prądu, limit prędkości `25 km/h`, obwód koła `2218 mm`. Do wpisania
   na nowo w Canable, jeśli wartości użytkownika były inne.
3. Celowe przerwanie zapisu (test odporności) — nadal niewykonane.
