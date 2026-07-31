# FW-051 — wszystkie ustawienia Walk Assist osobno dla każdego banku

- **Data:** 2026-07-28
- **Status:** WDROŻONE. Build M820 BL820: `0.0238_M820_BL820.bin`.
- **Zakres:** firmware M820, format banku profilu `0x6020/0x6021`, zapis `0x6022`
  oraz zakładka `eVistDrive Walk` w Canable.
- **Powiązane:** `FW-029_WALK_ASSIST_MOTOR_SPEED_PLAN.md`,
  `FW-043_WA_RPM_UNLOCK_BANK_CUTOFF.md`, `FW-044_WA_RATIO_CONFIRMED_UI.md`.

---

## 0. Stan po wdrożeniu

Wdrożenie wykonane w buildzie `0.0238_M820_BL820.bin`.

### Firmware

- `ASSIST_BANK_BLOB_LEN` zwiększono z `185` do `187`.
- Serializer banku zapisuje zawsze format `BANK_BLOB_VERSION = 2`.
- Parser banku przyjmuje oba formaty:
  - `v1`: nagłówek 8 B, rekordy od offsetu 8, CRC przy 183,
  - `v2`: nagłówek 10 B, rekordy od offsetu 10, CRC przy 185.
- Dla `v1` current/RPM są migrowane z globalnych, zwalidowanych pól
  `MP.walk_assist_current` i `MP.walk_assist_speed`.
- Dla `v2` current/RPM/cut-off są czytane z nagłówka banku.
- Runtime Walk Assist używa getterów aktywnego banku:
  - `assist_modes_get_wa_max_wheel_x100()`,
  - `assist_modes_get_wa_current_pct()`,
  - `assist_modes_get_wa_target_rpm()`.
- Przełączenie banku gestem resetuje kontroler WA przez `walk_motor_reset()`,
  więc nowy bank startuje świeżą rampą, bez skoku prądu.
- `MP.bank_store[2][192]` i `BankBlob[192]` pozostały bez zmiany rozmiaru.
- Globalne pola `Para1[36]` i `Para1[60..61]` zostały zachowane jako zgodność
  i źródło migracji, ale nie są już runtime source of truth dla WA po bankach v2.

### Canable

- `bafang-parser.js` rozpoznaje bank blob `v1` i `v2`.
- `canbus.js` serializuje banki jako `v2`, 187 B.
- Obiekt banku zawiera teraz:
  - `wa_cutoff_kmh`,
  - `wa_current_pct`,
  - `wa_target_rpm`,
  - `bank_schema_version`.
- Zakładka `eVistDrive Walk` używa jednego selektora Bank 1/2 dla wszystkich
  trzech wartości WA.
- `Apply Walk settings` wysyła `WRITE_BANK` dla wybranego banku, a nie
  `WRITE_LONG_P1` dla globalnych pól.
- `Save Walk settings to flash` wysyła `SAVE_BANKS`; zapis nadal następuje na
  postoju.
- Po odczycie starego banku `v1` Canable uzupełnia current/RPM z odczytanego P1,
  żeby pierwsza migracja nie zgubiła dotychczasowych ustawień.

### Weryfikacja wykonana

- Build firmware M820 BL820: OK.
- Artefakt: `.build/0.0238_M820_BL820.bin`.
- `node --check`: OK dla `bafang-parser.js`, `canbus.js`,
  `ui/js/ebics-compat.js`, `ui/js/tab-ebics.js`, `ui/js/websocket.js`.
- Canable server zrestartowany i odpowiada pod `http://127.0.0.1:8080`.
- `git diff --check` w firmware pokazuje wcześniejsze whitespace z brudnego
  worktree; nie są częścią tej zmiany i nie były czyszczone.

### Test sprzętowy do wykonania

1. W Canable kliknąć `Sync` w `eVistDrive Walk`.
2. Ustawić różne wartości dla Bank 1 i Bank 2, np.:
   - Bank 1: `30%`, `45 RPM`, `7,0 km/h`,
   - Bank 2: `45%`, `50 RPM`, `6,0 km/h`.
3. Dla każdego banku kliknąć `Apply Walk settings`.
4. Kliknąć `Save Walk settings to flash` i odczekać pełny postój.
5. Po restarcie wykonać `Sync` i potwierdzić, że oba banki zachowały wartości.
6. Na stojaku sprawdzić, że aktywny bank zmienia current/RPM/cut-off, a zmiana
   banku podczas trzymania WA startuje od nowej rampy.

---

## 1. Cel i decyzja

Walk Assist ma mieć jeden spójny zestaw ustawień **dla każdego banku**:

| Parametr | Jednostka | Zakres | Domyślnie |
|---|---:|---:|---:|
| siła / limit prądu WA | `%` | 1–100 | 30 |
| docelowa prędkość zębatki | RPM | 20–60 | 50 |
| awaryjne odcięcie od prędkości roweru | km/h | 1,0–25,5 | 7,0 |

Bank 1 i Bank 2 mogą mieć różne wartości wszystkich trzech parametrów.
Po przełączeniu aktywnego banku firmware korzysta z kompletnego zestawu WA
tego banku.

**Decyzja UI:** ustawienia pozostają w osobnej zakładce `eVistDrive Walk`.
Nie przenosimy ich do edytora poziomów w `Profiles`, ponieważ nie zależą od
ECO/TOUR/SPORT/SPORT+/BOOST. Zakładka Walk dostaje jeden selektor Bank 1/2,
który steruje wszystkimi trzema polami.

---

## 2. Stan obecny

Obecna konfiguracja jest niespójna:

| Parametr | Obecne źródło | Zakres obowiązywania |
|---|---|---|
| `MP.walk_assist_current` | `Para1[36]` | globalny |
| `MP.walk_assist_speed` (obecnie RPM zębatki) | `Para1[60..61]` | globalny |
| `wa_cutoff_kmh` | nagłówek banku, bajt 7 | per bank |

Runtime przekazuje do `walk_assist_motor`:

```text
walk_current_pct       = MP.walk_assist_current
target_chainring_rpm   = MP.walk_assist_speed
max_wheel_speed_x100   = assist_modes_get_wa_max_wheel_x100()
```

Skutek: przełączenie banku zmienia obecnie tylko twardy próg prędkości koła.
Siła i RPM pozostają takie same w obu bankach.

---

## 3. Docelowy model danych

Każdy bank dostaje trzy pola nagłówka:

```text
wa_cutoff_x10       u8   0,1 km/h
wa_current_pct      u8   %
wa_target_rpm       u8   RPM zębatki
```

Firmware udostępnia trzy gettery aktywnego banku:

```c
uint16_t assist_modes_get_wa_max_wheel_x100(void);
uint8_t  assist_modes_get_wa_current_pct(void);
uint8_t  assist_modes_get_wa_target_rpm(void);
```

`main.c` nie może już używać `MP.walk_assist_current` ani
`MP.walk_assist_speed` jako bieżącego źródła sterowania WA. Pola `MP` zostają
w strukturze wyłącznie dla zgodności EEPROM/Para1 i migracji starych ustawień.

---

## 4. Format banku v2

### 4.1. Obecny format v1

```text
header       8 B
5 poziomów  5 × 35 B = 175 B
CRC16        2 B
razem      185 B
```

### 4.2. Nowy format v2

```text
header      10 B
5 poziomów  5 × 35 B = 175 B
CRC16        2 B
razem      187 B
```

Dokładny nagłówek:

| Offset | Rozmiar | Pole | Znaczenie |
|---:|---:|---|---|
| 0 | 1 | magic 0 | `0x45` |
| 1 | 1 | magic 1 | `0x42` |
| 2 | 1 | version | `2` |
| 3 | 1 | bank index | `0` albo `1` |
| 4 | 1 | level count | `5` |
| 5 | 1 | record length | `35` |
| 6 | 1 | active bank | `0` albo `1` |
| 7 | 1 | WA cut-off | km/h × 10 |
| 8 | 1 | WA current | procent siły/limitu WA |
| 9 | 1 | WA target RPM | RPM zębatki |
| 10 | 175 | level records | pięć rekordów po 35 B |
| 185 | 2 | CRC16-CCITT | po bajtach 0–184 |

Stałe:

```text
BANK_BLOB_VERSION    2
BANK_BLOB_HEADER_LEN 10
ASSIST_BANK_BLOB_LEN 187
```

Istniejące bufory mają 192 bajty:

```text
MP.bank_store[2][192]
BankBlob[192]
```

Nie zmienia się rozmiar `MotorParams_t`, położenie pól EEPROM ani liczba ramek
multiframe: zarówno 185, jak i 187 bajtów wymagają 24 ramek po 8 bajtów.

---

## 5. Migracja v1 → v2 bez utraty ustawień

Firmware musi czytać oba formaty.

### Bank v1

1. Zweryfikować magic, wersję 1, długość 185 i CRC z offsetu 183.
2. Zachować istniejący `wa_cutoff_x10` z bajtu 7.
3. Wziąć globalne, wcześniej zwalidowane:
   - `MP.walk_assist_current`,
   - `MP.walk_assist_speed`.
4. Skopiować te dwie wartości do konfiguracji WA obu banków.
5. Nie wykonywać natychmiastowego zapisu flash.
6. Przy następnym `SAVE_BANKS` zapisać oba banki już jako v2.

Wynik: po aktualizacji oba banki zachowują dokładnie dotychczasową siłę i RPM,
a ich istniejące progi odcięcia pozostają niezależne.

### Bank v2

1. Zweryfikować magic, wersję 2, długość 187 i CRC z offsetu 185.
2. Odczytać wszystkie trzy pola WA z nagłówka.
3. Zastosować walidację:
   - current poza 1–100 → 30,
   - RPM poza 20–60 → 50,
   - cut-off poniżej 1,0 km/h → 7,0 km/h.

### Brak prawidłowego banku

Użyć wartości domyślnych `30% / 50 RPM / 7,0 km/h`. Uszkodzony CRC nie może
częściowo zastosować ani profilu jazdy, ani ustawień WA.

### Para1

`Para1[36]` i `Para1[60..61]` zostają:

- nie zmieniamy układu `MotorParams_t`,
- starsze narzędzia nadal mogą je odczytać,
- służą jako źródło migracji banku v1,
- po prawidłowym załadowaniu banku v2 nie sterują już runtime WA.

Nowe Canable nie zapisuje tych pól dla firmware obsługującego bank v2.

---

## 6. Zmiany firmware

### `inc/assist_modes.h`

- zwiększyć `ASSIST_BANK_BLOB_LEN` z 185 do 187,
- dodać gettery prądu i RPM aktywnego banku,
- opisać wersjonowanie v1/v2.

### `src/assist_modes.c`

- dodać tablice:

```c
static uint8_t bank_wa_current_pct[ASSIST_BANK_COUNT];
static uint8_t bank_wa_target_rpm[ASSIST_BANK_COUNT];
```

- inicjalizować oba banki wartościami 30% i 50 RPM,
- serializować zawsze format v2,
- deserializować v1 i v2,
- nie modyfikować żadnego banku przed pełnym sprawdzeniem CRC i pól,
- zwracać ustawienia aktywnego banku przez gettery.

### `src/main.c`

Przed wczytaniem zapisanych banków przekazać do modułu zwalidowane globalne
wartości `MP` jako fallback migracyjny. Następnie wejście WA ma używać:

```text
walk_current_pct       = getter aktywnego banku
target_chainring_rpm   = getter aktywnego banku
max_wheel_speed_x100   = getter aktywnego banku
```

Przełączenie banku podczas aktywnego WA nie może skokowo zmienić prądu. Należy:

- albo zablokować zmianę banku do zwolnienia WA,
- albo przerwać WA i ponownie uruchomić jego rampę od stanu START.

Preferowane: świeży start/rampa po zmianie banku.

### `src/CAN_Display.c`

- nadal używać `BankBlob[192]`,
- dla `0x6020` wysyłać 187 bajtów v2,
- dla `0x6021` przyjmować v1 lub v2 do 192 bajtów,
- `0x6022` bez zmian: zapis obu banków i tuningu dopiero na postoju.

### `inc/main.h` i `src/parser.c`

- nie usuwać pól `walk_assist_current` i `walk_assist_speed`,
- nie zmieniać `bank_store[2][192]`,
- opisać stare pola jako fallback/alias zgodności,
- walidację globalnych wartości zachować dla migracji v1.

---

## 7. Zmiany Canable

### Parser i serializer

`bafang-parser.js`:

- rozpoznawać wersję banku 1 i 2,
- liczyć CRC oraz offset rekordów zależnie od wersji,
- dla v2 zwracać:

```js
wa_cutoff_kmh
wa_current_pct
wa_target_rpm
bank_schema_version
```

`canbus.js`:

- serializować v2 jako 187 bajtów,
- umieszczać current i RPM w bajtach 8 i 9,
- CRC zapisywać w bajtach 185–186.

Po połączeniu ze starym firmware zwracającym v1 Canable zachowuje tryb
zgodności: current i RPM są globalne w `0x6011`, a bankowe pola pozostają
nieaktywne.

### Zakładka `eVistDrive Walk`

Jeden selektor banku steruje wszystkimi polami:

```text
Bank: [Bank 1 | Bank 2]

Walk motor current       [%]
Walk chainring speed     [RPM]
Walk assist cut-off      [km/h]
```

Zasady:

- `Sync` czyta oba banki,
- żadnego banku nie wolno zapisać przed jego prawidłowym odczytem,
- zmiana selektora nie przełącza aktywnego banku w rowerze,
- `Apply selected bank (RAM)` wysyła tylko wybrany bank,
- `Save all (Flash)` wysyła `SAVE_BANKS`; zapis następuje na postoju,
- etykieta pokazuje, który bank jest aktualnie aktywny.

Globalne pola current/RPM z `0x6011` znikają z edycji Walk dla banku v2.
Mogą pozostać wyłącznie w ukrytej diagnostyce zgodności.

### Zakładka `Profiles`

Nie dodawać tam drugiego edytora WA. Opcjonalnie można pokazać nieedytowalne
podsumowanie wartości WA wybranego banku. Jedno źródło edycji pozostaje w
zakładce Walk.

---

## 8. Inwarianty bezpieczeństwa

1. Nie wolno wysłać banku z wartościami domyślnymi przed `Sync`.
2. Current, RPM i cut-off muszą pochodzić z tego samego aktywnego banku.
3. Uszkodzony bank lub CRC oznacza odrzucenie całego bloku.
4. Zmiana nie może zwiększyć prądu ani RPM przez błędną migrację.
5. Próg prędkości koła pozostaje twardym odcięciem bezpieczeństwa.
6. Hamulec, fault, jam/stall oraz zwolnienie przycisku zachowują obecny
   priorytet i nie zależą od banku.
7. Przełączenie banku w trakcie WA wymusza bezpieczny restart rampy.
8. Zapis flash nadal jest dozwolony wyłącznie na pełnym postoju.

---

## 9. Kolejność wdrożenia

1. Dodać model WA per bank i gettery w firmware.
2. Dodać parser v1/v2 oraz migrację globalnych pól do obu banków.
3. Przełączyć runtime WA na gettery aktywnego banku.
4. Dodać serializer v2 i test round-trip firmware.
5. Zaktualizować parser/serializer Canable.
6. Rozszerzyć selektor banku w `eVistDrive Walk` na wszystkie trzy pola.
7. Usunąć zapis current/RPM przez `WRITE_LONG_P1` dla banku v2.
8. Zbudować firmware i wykonać test bez obciążenia.
9. Dopiero po testach usunąć starą kartę `Banks`.
10. Zaktualizować `WALK_ASSIST_DZIALANIE.md`,
    `RIDE_CORE_STATUS_CANABLE.md` i `protocol/ebics_config_schema.yaml`.

---

## 10. Testy

### Test protokołu

1. Bank v1 z prawidłowym CRC jest przyjęty.
2. Current/RPM z `Para1` zostają skopiowane do obu banków.
3. Bank v2 przechodzi serialize → parse bez zmiany żadnego pola.
4. Uszkodzony CRC v1 i v2 jest odrzucany.
5. Nieprawidłowe current/RPM/cut-off wracają do bezpiecznych wartości.
6. Zapis jednego banku nie zmienia drugiego.

### Test Canable

1. `Sync` odczytuje Bank 1 i Bank 2.
2. Ustawić:
   - Bank 1: 30%, 45 RPM, 7,0 km/h,
   - Bank 2: 45%, 50 RPM, 6,0 km/h.
3. Przełączać selektor — wartości muszą pozostać niezależne.
4. `Apply` Banku 1 nie zmienia Banku 2.
5. `Save all`, postój, restart, `Sync` — wszystkie sześć wartości
   (po trzy na każdy bank) musi przetrwać.
6. Zapis WA nie może zmienić trybów ani parametrów pięciu poziomów banku.

### Test firmware na stojaku

1. Aktywny Bank 1 wykorzystuje 30%, 45 RPM i 7,0 km/h.
2. Po bezpiecznym przełączeniu na Bank 2 runtime wykorzystuje 45%, 50 RPM
   i 6,0 km/h.
3. `target_erps` odpowiada wybranemu RPM przez przelicznik 4/3.
4. Przełączenie banku nie powoduje skoku prądu; WA startuje ponownie rampą.
5. Hamulec i zwolnienie przycisku natychmiast odcinają WA w obu bankach.
6. Jam/stall działa identycznie dla obu banków.
7. Cut-off każdego banku odcina WA przy zapisanej prędkości.

### Test migracji

1. Wgrać firmware z bankami v1 i globalnymi ustawieniami WA.
2. Zaktualizować do FW-051.
3. Oba banki muszą dostać dotychczasowy globalny current/RPM.
4. Istniejące różne progi cut-off obu banków muszą zostać zachowane.
5. Pierwszy odczyt Canable pokazuje v2 bez resetu innych ustawień.
6. Po `Save all` i restarcie oba banki pozostają v2.

---

## 11. Kryteria akceptacji

Zmiana jest zakończona dopiero, gdy:

- wszystkie trzy ustawienia WA są przechowywane per bank,
- runtime nie czyta globalnych pól P1 po załadowaniu banku v2,
- migracja v1 zachowuje dotychczasowe zachowanie,
- nie następuje reset EEPROM,
- Canable edytuje oba banki z jednej zakładki Walk,
- zapis WA nie narusza profili jazdy,
- test stojaka potwierdza różne current/RPM/cut-off obu banków,
- przełączenie banku podczas lub tuż przed WA nie powoduje skoku prądu.
