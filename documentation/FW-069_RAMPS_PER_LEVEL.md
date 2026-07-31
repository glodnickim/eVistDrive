# FW-069 — rampy prądu per poziom i per bank

- **Data:** 2026-07-31
- **Status:** ZAIMPLEMENTOWANE, **niezbudowane i nieprzetestowane na sprzęcie**
  (build po stronie właściciela).
- **Zakres:** firmware M820 — `inc/assist_modes.h`, `src/assist_modes.c`,
  `inc/assist_dynamics.h`, `src/assist_dynamics.c`, `inc/tuning_config.h`,
  `src/tuning_config.c`, `src/ride_control.c`, `inc/main.h`, `src/CAN_Display.c`.
  Canable — `canbus.js`, `bafang-parser.js`, `profiles.js`, `dynamics.js`,
  `websocket.js`, `ui/index.html`, `ui/style.css`.
- **Powiązane:** `FW-068_START_CONDITION_CONFIGURABLE.md` — wspólny build, wspólny blob
  banków i wspólny jednorazowy reset EEPROM.

---

## 1. Problem

`Acceleration — current rise` i `Deceleration — current fall` (cztery wartości: slow/fast ×
góra/dół) były **globalne** — jeden komplet na cały rower, w blobie tuningu. Eco i Boost
narastały tak samo, bank Power Linear i bank eMTB też.

To jest niespójne z tym, co już było: `release_ms`, `power_rise_filter_ms`
i `power_fall_filter_ms` **od dawna są per poziom** w blobie banków. Rampy stały w innym
miejscu niż reszta ustawień narastania mocy, choć to właśnie one decydują o charakterze
najmocniej.

## 2. Zmiana

Cztery wartości przeniesione do rekordu poziomu w blobie banków. Bank jest tam wymiarem
tablicy (`bank_store[2]`), więc **per poziom daje per bank za darmo** — i to bez dokładania
niczego do nagłówka bloba.

`assist_dynamics_apply()` nie sięga już do `tuning_config_ramp_*`; wartości przychodzą przez
`assist_dynamics_input_t`, tą samą drogą, którą już wcześniej szedł `profile_release_ms`.
Wypełnia je `ride_control` z konfiguracji aktywnego poziomu.

**Walk Assist nie korzysta z tych ramp i nie ma korzystać.** `assist_dynamics_apply()` wychodzi
z funkcji **przed** kodem ramp, gdy ustawione jest `walk_active` — WA ma własny kontroler
prędkości (FW-060/FW-067) i sam prowadzi całą trajektorię Iq, łącznie ze startem i zejściem.
Dołożenie drugiego elementu dynamicznego za jego regulatorem tylko pogorszyłoby stabilność
pętli. W `ride_control` pola ramp zostają na gałęzi WA celowo zerowe; zero wybiera też
wkompilowane wartości awaryjne, więc gdyby kiedyś jakaś ścieżka mimo wszystko doszła do kodu
ramp, nie dostanie rampy o zerowej długości.

Gettery `tuning_config_ramp_*` usunięte razem z ich jedynym konsumentem. `assist_dynamics.c`
zachowuje wkompilowane wartości awaryjne używane wyłącznie wtedy, gdy wywołujący zostawi pole
na zerze — zero nie jest legalną długością rampy i nie może zamienić się w natychmiastowy skok
prądu.

## 3. Zgodność wsteczna i pułapka, którą trzeba było przy okazji naprawić

Bajty 4–11 bloba tuningu **zostają na drucie**, żeby starszy Canable nadal wysyłał blob, który
ta firmware przyjmuje. Ale **firmware ich nie czyta** — rampy zapisane starszym narzędziem nic
nie zmienią. Bez tej informacji w changelogu wygląda to jak usterka.

Ważniejsze: `assist_modes_apply_bank_blob()` odrzucał blob, gdy `buffer[5] != BANK_RECORD_LEN`,
i liczył pozycję CRC z tej samej stałej kompilacyjnej. Bajt „długość rekordu" był więc
**sprawdzany jak stała, zamiast być użyty jako krok odczytu**. Konsekwencja: pierwsze
powiększenie rekordu w historii projektu odrzuciłoby każdy zapisany bank i po cichu skasowało
całą konfigurację profili.

Teraz `buffer[5]` jest rzeczywistym krokiem. Akceptowany zakres to
`BANK_RECORD_LEN_V5 (35) .. BANK_RECORD_LEN (46)`, z niego liczona jest pozycja CRC i przesuw
rekordu, a pola powyżej starej długości uzupełniane są wartościami domyślnymi tego poziomu.
Starszy Canable dalej zapisuje, a przyszłe rozszerzenia nie kasują profili.

## 4. Twardy limit protokołu — 255 bajtów

Pierwotny projekt zakładał rekord 56 B i blob 295 B. **To nie przechodzi.**

Protokół wieloramkowy przenosi całkowitą długość transferu **w jednym bajcie**:
`rx_data_length` w `CAN_Display.c` jest `uint8_t` i pochodzi wprost z `rx_data[0]` pierwszej
ramki, a `send_multiframe()` przyjmuje `uint8_t length`. Blob powyżej 255 B nie zostałby
przesłany w ogóle — zapis kończyłby się błędem CRC bez żadnej wskazówki co do przyczyny.

Dlatego trzy pola FW-068 są na drucie zwężone (bezstratnie dla ich zakresów):

| pole | w strukturze | na drucie |
|---|---|---|
| `start_load_reduction_mv` | u16 (0–100) | u8 |
| `start_rise_mv` | u16 (0–100) | u8 |
| `start_rise_window_ms` | u16 (0–2000) | u8 w jednostkach 10 ms |
| 4 rampy | u16 (20–5000) | u16 bez zmian |

Wynik: rekord **46 B**, blob **245 B**, czyli 10 B zapasu do limitu.

> **Do zapamiętania na przyszłość:** blob banków jest już prawie pełny. Zapas 10 B to
> 2 bajty na poziom. Kolejne pole per poziom wymaga **najpierw** przejścia protokołu na
> długość 16-bitową po obu stronach.

## 5. Rozmiary, bufory, ramki

| | dziś | po zmianie |
|---|---|---|
| `BANK_BLOB_VERSION` | 5 | 6 |
| `BANK_RECORD_LEN` | 35 | 46 (`BANK_RECORD_LEN_V5` = 35 zachowane) |
| `ASSIST_BANK_BLOB_LEN` | 190 | 245 |
| `MP.bank_store` | `[2][192]` | `[2][256]` |
| `BankBlob` | `[192]` | `[256]` |
| granica ramek 0x6021 | `< 23` | `< 30` |
| `TUNING_BLOB_LEN` | 24 | 32 |
| `MP.tuning_store` | `[24]` | `[64]` |
| `TuningBlob` | `[24]` | `[32]` |
| granica ramek 0x6024 | `< 2` | `< 3` |

Granice ramek to najłatwiejsze do przeoczenia miejsce w całej zmianie: bez ich podniesienia
końcowe ramki są odrzucane **bez śladu w logu**, a zapis kończy się błędem CRC, który wygląda
jak problem transmisji.

## 6. Canable

Pola i **oba wykresy** (narastanie / opadanie) przeniesione z karty Dynamics do Profiles, obok
pozostałych ustawień narastania mocy. Wykresy rysują poziom aktualnie edytowany.

Dochodzą **dwa** przełączniki kopiowania na poziomy, oba działające wyłącznie w obrębie
**aktywnego banku** (drugi bank nigdy nie jest ruszany, bo karta edytuje jeden bank naraz):

- **„Apply my edits to all levels in this bank"** — szeroki: dowolne edytowane pole karty
  trafia na wszystkie 5 poziomów.
- **„Apply acceleration/deceleration ramps to all levels"** — wąski: kopiuje wyłącznie cztery
  rampy. To jest typowy przypadek strojenia — rampy identyczne wszędzie, a wsparcie, boost
  i progi różne per poziom. Szerokim przełącznikiem trzeba by go włączać i wyłączać wokół
  każdej edycji.

Obejmuje **narastanie i opadanie**, nie samo narastanie: rampy stroi się parami, a kontrolka
pokrywająca tylko przyspieszanie zostawiłaby opadanie bez odpowiednika. Oba przełączniki
sumują się (OR): wystarczy jeden z nich, żeby pole rampy poszło na wszystkie poziomy.

Świadomie **nie ma** opcji „zastosuj do obu banków". Dwa banki istnieją właśnie po to, żeby
się różniły, a przycisk kasujący tę różnicę jednym kliknięciem to najprostsza droga do
przypadkowej utraty strojenia. Gdyby po testach okazał się potrzebny — to kilka linii.

Panel System pokazywał „stored fall ramps" z bloba tuningu. Te bajty są teraz martwe, więc
`updateDiagTuning()` czyta rampy z wybranego poziomu profilu, czyli z tego, co kontroler
faktycznie wykonuje.

## 7. Testy na rowerze

1. Bank 1 poziom 1 rise = 2000 ms, bank 1 poziom 5 rise = 100 ms → różnica narastania mocy
   między poziomami wyraźnie wyczuwalna. Dowód, że per poziom działa.
2. Bank 2 z innymi wartościami → przełączenie banku zmienia charakter. Dowód per bank.
3. Szeroki checkbox „apply to all levels": zaznaczyć, zmienić rise na jednym poziomie → Read
   pokazuje tę samą wartość na wszystkich pięciu **tego banku**, drugi bank nietknięty.
3a. Wąski checkbox „apply ramps to all levels": zaznaczyć **tylko jego**, zmienić rampę →
   rozchodzi się na wszystkie poziomy; następnie zmienić „Maximum motor current" → **nie**
   rozchodzi się, zostaje na edytowanym poziomie. To jest cały sens tej drugiej kontrolki.
4. Walk Assist działa **dokładnie jak dotąd** — ustawienie skrajnych ramp na poziomie
   (np. rise 2000 ms) nie może zmienić charakteru startu WA. Gdyby zmieniało, znaczy że
   wczesne wyjście z `assist_dynamics_apply()` przestało działać.
5. Zapis w karcie Profiles kończy się ACK, nie NACK/timeoutem. Timeout tutaj oznacza błędną
   granicę ramek dla 0x6021.
6. Starszy Canable (rekord 35 B) do nowej firmware — zapis przyjęty, nowe pola domyślne.
   To weryfikuje naprawę kroku odczytu z `buffer[5]`.
