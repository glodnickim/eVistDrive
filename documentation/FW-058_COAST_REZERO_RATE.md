# FW-058 — rzadsze automatyczne zerowanie czujnika momentu

- **Data:** 2026-07-29
- **Status:** ZAIMPLEMENTOWANE, **niezbudowane i nieprzetestowane na sprzęcie**
  (build po stronie właściciela).
- **Zakres:** firmware M820 — `inc/config.h`, `src/torque_input.c`,
  `inc/torque_input.h`, jedno wywołanie w `src/main.c`.
  **Bez zmian w protokole i w Canable.**
- **Powiązane:** `FW-013` (kalibracja czujnika), `FW-023`, `FW-033`.

---

## 1. Problem

Wspomaganie dopina się w jeździe nieregularnie i za każdym razem przy innej sile
nacisku, podczas gdy start z miejsca jest powtarzalny.

Znalezione w kodzie liczby:

- próg załączenia wspomagania (`Minimum pedal load`) = **18 mV** ≈ **0,67 kg**
  przy zmierzonej charakterystyce 27 mV/kg,
- automatyczne zerowanie na wybiegu może przy **jednym** wybiegu przesunąć zero
  o `TQ_RECAL_MAX_STEP` = **20 mV** ≈ **0,74 kg**,
- warunek odpalenia to było **1,5 s** bez impulsu PAS przy zerowym prądzie,
  czyli **praktycznie każdy dłuższy wybieg** — w mieście kilkadziesiąt razy na
  jazdę.

Jedno przesunięcie bywa więc **większe niż cały próg startu**.

Dlaczego objaw widać tylko w jeździe:

- na wybiegu w ruchu obie stopy leżą na pedałach, więc zapisywany „spoczynek"
  bywa dociążony; na postoju zwykle stawiasz nogę na ziemi i czujnik widzi
  prawdziwe zero,
- ruszając z miejsca naciskasz kilka–kilkanaście kg, więc przesunięcie o 0,74 kg
  jest niewyczuwalne; dopinając w jeździe operujesz w okolicy 1–2 kg, gdzie to
  samo przesunięcie decyduje o wszystkim.

## 2. Zmiana

| Parametr | Było | Jest |
|---|---|---|
| bezczynność PAS uprawniająca do zerowania | 1,5 s | **5 s** |
| minimalny odstęp korekt **w ruchu** | brak | **60 s** |
| zerowanie **na postoju** | — | **bez zmian** |

Wybieg musi teraz trwać co najmniej **5,5 s** (5 s bezczynności + 0,5 s
ustabilizowania z istniejącego `TQ_RECAL_SETTLE_TICKS`), żeby w ogóle doszło do
oceny.

Postój celowo pozostaje nieograniczony: to jest ta wiarygodna kalibracja i to
ona ma nadal kompensować dryf termiczny.

### Pliki

**`inc/config.h`**

```c
#define TQ_RECAL_IDLE_TICKS 20000         // ~5 s @4kHz (było 6000 = 1,5 s)
#define TQ_RECAL_MIN_PERIOD_TICKS 240000U // ~60 s @4kHz, tylko w ruchu
#define TQ_RECAL_MOVING_X100 100          // >= 1,0 km/h liczy się jako jazda
```

`pas_idle_ticks` nasyca się na 64000, więc 20000 nie przekręca licznika.

**`src/torque_input.c`**

- nowa statyczna `recal_lockout_ticks`, dekrementowana w
  `torque_input_coast_update()` — funkcja jest wołana co tik 4 kHz niezależnie
  od tego, czy trwa wybieg, więc jest naturalnym źródłem czasu,
- `coast_evaluate()` przyjmuje `bike_moving`:
  - **kontrola wiarygodności `rest_raw_plausible()` i `cal_fault` zostają przed
    blokadą** — wykrywanie usterki czujnika (Error 25) działa w dotychczasowym
    tempie; blokada wstrzymuje wyłącznie ruszanie zera,
  - w ruchu przy aktywnej blokadzie → wyjście bez korekty,
  - nowe `apply_and_arm()` uzbraja blokadę **tylko** po korekcie w ruchu.

Pasmo `TQ_RECAL_BAND_MV`, ścieżka `drift_confirmed()` i ograniczenie kroku
`TQ_RECAL_MAX_STEP` — **bez zmian**.

`torque_input_startup_zero()` ustawia offset bezpośrednio, z pominięciem
`apply_offset_step()`, więc zerowanie przy starcie sterownika jest nietknięte.

**`src/main.c`** (jedyne wywołanie) — dochodzi trzeci argument
`MS.Speedx100 >= TQ_RECAL_MOVING_X100`.

## 3. Weryfikacja wykonana

`node tests/fw058_coast_rezero.js` → **PASS**. Progi czytane z prawdziwych
nagłówków, logika wybiegu przeniesiona wiernie do Node (brak hostowego gcc).
Przypadki: wybieg krótszy od okna ustabilizowania nie koryguje; w ruchu dwa
wybiegi 30 s po sobie → tylko pierwszy; 70 s po sobie → oba dopuszczone; na
postoju trzy pod rząd → wszystkie; korekta na postoju nie blokuje następnej
w jeździe; blokada **nie** wstrzymuje wykrycia niewiarygodnej bazy; licznik
blokady nie schodzi poniżej zera przy 300 s jazdy bez wybiegów.

Test właściwości, dla której powstała zmiana — symulowana jazda z wybiegiem co
20 s i losowo zmiennym zanieczyszczeniem próbki:

| | najgorsza wędrówka zera w oknie 60 s |
|---|---|
| bez ograniczenia (stan przed) | **26 mV** |
| z ograniczeniem, w ruchu | **19 mV** |

Build: **nie uruchamiany** — kompiluje właściciel. Przy budowaniu spodziewane
ostrzeżenia to wyłącznie zastane (niezgodność znaku wskaźnika w `CAN_Display.c`,
nieużywane `fw_ver`, segment LOAD RWX); cokolwiek z `torque_input.c` lub
`main.c:1681` byłoby regresją tej zmiany.

## 4. Uczciwa ocena skuteczności

**Ta zmiana ogranicza zjawisko, ale go nie usuwa.** Widać to wprost w liczbach
powyżej: 19 mV wędrówki zera na minutę to nadal więcej niż cały próg załączenia
(18 mV). Pojedyncza korekta wciąż może wynieść pełne 20 mV, bo
`TQ_RECAL_MAX_STEP` nie był ruszany — zmieniła się tylko **częstotliwość**.

Spodziewany efekt jazdowy: siła potrzebna do dopięcia wspomagania ma się zmieniać
**rzadziej** (najwyżej raz na minutę zamiast po każdym wybiegu), a nie przestać
zmieniać się w ogóle.

## 5. Test na rowerze

1. W zakładce **eVistDrive Torque** obserwować odczyt przy **całkowicie luźnych
   pedałach**: w jeździe baza może drgnąć najwyżej raz na minutę, nie po każdym
   wybiegu.
2. Po dłuższym postoju zerowanie ma działać jak dotąd.
3. Seria dopięć wspomagania w jeździe — czy siła nadal skacze z próby na próbę.
4. Brak regresji: start z miejsca bez zmian, Error 25 nadal wykrywany.

## 6. Co dalej — zrobione w FW-059

Druga, głębsza przyczyna (próbka brana na **końcu wybiegu**, z szybkiej średniej
o stałej czasowej 16 ms, czyli już w trakcie narastania nacisku) została
naprawiona w `FW-059_COAST_SAMPLE_QUALITY.md`: próbka zamrażana w środku wybiegu,
odrzucanie niespokojnych wybiegów, krok korekty 20 → 5 mV.

Po obu kartach razem najgorsza wędrówka zera w oknie 60 s spada z **26 mV do
5 mV**, czyli wyraźnie poniżej progu załączenia (18 mV). Liczba 19 mV w tabeli
powyżej dotyczy samego FW-058 i jest zachowana jako zapis stanu pośredniego.

Niezależnie od tego wciąż otwarty jest drugi możliwy mechanizm zmiennej siły
dopięcia — czekanie na impuls PAS przy dopinaniu w jeździe. Rozstrzyga go pomiar
nacisku w chwili załączenia wspomagania z bloku `0x6029`: wartość stała ≈ 18 mV
wskazuje na zero, wartość skacząca — na PAS.
