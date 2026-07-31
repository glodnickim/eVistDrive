# FW-051..054 — Walk Assist w blobie banku, nowe domyślne latcha, latch po puszczeniu przycisku

- **Data:** 2026-07-28/29
- **Status:** WDROŻONE przez dewelopera, równolegle do FW-048/049/050 (build `0.0236` i nowsze).
  Ta karta dokumentuje zmiany, które nie miały jeszcze opisu.
- **Zakres:** `assist_modes.c/.h` (blob banku v2→v3), `main.c` (migracja, latch po puszczeniu,
  histereza prędkości WA), `walk_assist_motor.c` (retuning), Canable (`ebics-compat.js`,
  `tab-banks.js`).

---

## FW-051 — ustawienia Walk Assist przeniesione do bloku banku, PER BANK

Wcześniej (FW-042/043/044) cel RPM i siła WA siedziały w `MP.walk_assist_speed` /
`MP.walk_assist_current` (Para1, jedna wartość dla całego sterownika). Deweloper przeniósł je
**do bloku banku**, więc **każdy bank ma teraz własne ustawienia WA**.

Nagłówek bloku banku urósł: **8 → 10 → 12 bajtów**, z zachowaniem wstecznej zgodności:

| Wersja | Nagłówek | Co niesie |
|---|---|---|
| v1 | 8 B | (bez pól WA — stare blobi sprzed FW-043) |
| v2 | 10 B | + `buffer[7]` = próg odcięcia WA (FW-043, bez zmian) |
| v3 (obecna) | 12 B | + `buffer[8]` = siła WA (%), `buffer[9]` = cel RPM, `buffer[10]` = latch po puszczeniu (FW-054), `buffer[11]` = timeout latcha (s) |

`assist_modes_apply_bank_blob` rozpoznaje wersję z `buffer[2]` i czyta tyle, ile ta wersja
niesie — starsze blobi nie są odrzucane. **Migracja jednorazowa**: przy starcie
`assist_modes_seed_wa_defaults(MP.walk_assist_current, MP.walk_assist_speed)` przepisuje stare
wartości z Para1 do obu banków, zanim ewentualny zapisany blob v2/v3 je nadpisze — więc
istniejące ustawienie WA nie ginie przy pierwszym starcie na tym firmware.

**Nowe granice celu RPM:** `WA_MOTOR_TARGET_RPM_MIN/MAX` = **20..60** (było 5..120 w FW-042/044)
— zawężone do sensownego zakresu marszu. Wartości spoza zakresu → domyślne 50 RPM.

**Próg odcięcia WA zmienił charakter — histereza zamiast twardego cięcia.** Wcześniej (FW-043)
przekroczenie progu dawało natychmiastowe zero. Teraz jest **pauza z powrotem**:
powyżej progu → WA wstrzymane; wraca automatycznie dopiero **0,5 km/h poniżej** progu
(`WA_SPEED_RESUME_HYST_X100`, `config.h`). To usuwa ryzyko szarpanego włącz/wyłącz na granicy
progu, o którym ostrzegałem w FW-029/043 — deweloper to naprawił.

## FW-052 — podłoga prądu (latch floor) zmniejszona: 4% → 2%

`tuning_config.c`: domyślne `min_iq_pct` **2** (było 4). Stare zapisane bloki tuningu z wartością
4 (domyślną z poprzedniej wersji) są migrowane do 2 automatycznie przy odczycie starszej wersji
blobu (`if (version < TUNING_VERSION && min_iq_pct == 4U) min_iq_pct = 2U;`) — rozmyślne
domyślne, nie przypadkowe „4" z ręcznego strojenia użytkownika.

## FW-053 — czas trzymania latcha wydłużony: 700 ms → 1400 ms

`tuning_config.c`: domyślne `hold_ms` **1400** (było 700). Ta sama migracja jak wyżej dla
starych blobów z wartością 700.

**Odpowiedź na pytanie „jaka jest podłoga wspomagania przy pedałowaniu":** patrz sekcja
poniżej — mechanizm jest w `ride_control.c` (FW-031/032, bez zmian w tej turze), tylko jego
domyślne wartości poszły w dół (mniejsza podłoga) i w górę (dłuższe trzymanie).

## FW-054 (nowe) — Walk Assist: kontynuacja po puszczeniu przycisku (timed run)

Wcześniej WA działał wyłącznie jako „martwy człowiek" — trzeba trzymać przycisk. Teraz, per
bank, opcjonalnie: `wa_latch_after_release` (bool) + `wa_latch_timeout_s` (1–120 s, domyślnie 30).

Zasada (`main.c`, ~1748-1799):
- puszczenie przycisku po legalnym trzymaniu **uzbraja** zatrzask na `wa_latch_timeout_s` sekund,
- w trakcie zatrzasku WA jedzie dalej **bez trzymania** przycisku,
- **każde** z poniższych **natychmiast** przerywa zatrzask: ponowne wciśnięcie przycisku
  Walk, przycisk góra/dół, zmiana oświetlenia, zmiana poziomu wspomagania, wciśnięcie
  manetki mocy (`adc_value[5]<2800`), hamulec, błąd, przekroczenie progu prędkości,
- po przerwaniu jest **blokada** (`ui8_wa_latch_cancel_block`) do puszczenia wszystkich
  tych wyzwalaczy — zapobiega natychmiastowemu ponownemu uzbrojeniu tym samym gestem,
- przy `wa_latch_after_release = false` (domyślne) zachowanie jest identyczne jak wcześniej
  (trzymaj przycisk).

## Retuning regulatora WA (bez zmiany architektury FW-029)

Deweloper przestroił stałe w `walk_assist_motor.c` (start wolniejszy i łagodniejszy — rampa
3,2 s zamiast 2,4 s, wykrycie ruchu po pełnym obrocie elektrycznym zamiast progu ERPS,
regulator prądu zależny od prędkości zębatki przy starcie — „droop control"). Mechanizm
bezpieczeństwa (bezwzględny limit prądu, watchdog zacięcia, brak hamowania aktywnego) —
**bez zmian koncepcyjnych**, tylko liczby.

### ⚠️ Kolizja numeracji FW — do wiadomości, nie do naprawy w kodzie

Deweloper oznaczył swoje zmiany w `walk_assist_motor.c` jako **„FW-047" i „FW-048"**
(governor prędkości silnika i „current droop controller"). Te numery **są już zajęte** w tym
repo przez inne, niepowiązane zmiany:
- **FW-047** = wolny „ogon" na końcu rampy wygaszania (`assist_dynamics.c`, ten sam dzień).
- **FW-048** = wybieg silnika zamiast dowożenia prądu przy zatrzymaniu — klik przekładni
  (`ride_control.c`/`assist_dynamics.c`).

Dwa różne obszary (Walk Assist vs. zwykła jazda) dostały te same numery niezależnie. Nie
przenumerowuję kodu dewelopera — zbyt ryzykowne bez jego wiedzy. Odnotowane tutaj, żeby przy
czytaniu komentarzy „FW-047"/„FW-048" w `walk_assist_motor.c` nie mylić ich z kartami
`FW-047_FADE_TAIL.md` / `FW-048_COAST_RELEASE.md`, które dotyczą zupełnie czego innego.

## Canable — gdzie to jest w UI

- **Zakładka eVistDrive Walk** (`ebics-compat.js`): pełny edytor WA dla **wybranego banku**
  (rozwijane menu banku) — siła (%), cel RPM, próg odcięcia (km/h), latch po puszczeniu,
  timeout latcha. Sync czyta P1 (tylko do jednorazowej migracji) + oba banki. Apply/Save
  piszą przez `WRITE_BANK`/`SAVE_BANKS` — **inna ścieżka zapisu niż reszta tej zakładki**
  (Limits/System piszą przez Para1 od razu; Walk teraz zapisuje przez blok banku, Apply = RAM,
  Save = flash na postoju). To jest opisane w UI, ale warto pamiętać przy diagnozowaniu.
- **Zakładka Banks** (`tab-banks.js`): pokazuje i edytuje **tylko** próg odcięcia
  (`wa_cutoff_kmh`) z tych pięciu pól WA. Siła/cel RPM/latch/timeout **nie mają tam kontrolek**
  — istnieją w danych banku (wysyłane w `WRITE_BANK`), ale edytowalne są wyłącznie w zakładce
  eVistDrive Walk.

## Test

1. Bank 1 i bank 2 z różnymi ustawieniami WA (siła/RPM/próg/latch) — przełączenie banku
   (gestem lub Canable) zmienia realne zachowanie WA bez ponownego wpisywania ustawień.
2. Migracja: stary blok tuningu z `min_iq_pct=4`/`hold_ms=700` (sprzed tej zmiany) po odczycie
   pokazuje nowe domyślne 2%/1400 ms, nie stare wartości.
3. Histereza progu WA: rozpędzić się powyżej progu banku, WA się wstrzymuje; zwolnić dokładnie
   do progu — WA **nie** wraca; zwolnić 0,5 km/h poniżej — wraca.
4. Latch po puszczeniu: włączyć w banku, puścić przycisk WA w trakcie jazdy — silnik jedzie
   dalej do `wa_latch_timeout_s`; nacisnąć hamulec — przerywa natychmiast i blokuje ponowne
   uzbrojenie do puszczenia hamulca.
5. `wa_latch_after_release=false` (domyślne) — zachowanie jak przed FW-054 (trzymaj przycisk).
