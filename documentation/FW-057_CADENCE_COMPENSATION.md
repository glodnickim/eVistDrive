# FW-057 / CB-057 — kompensacja kadencji, włączana osobno dla każdego banku

- **Data:** 2026-07-29
- **Status:** ZAIMPLEMENTOWANE, **niezbudowane i nieprzetestowane na sprzęcie**.
  Build firmware nie był uruchamiany — czeka na wyraźne polecenie.
- **Zakres:** firmware M820 (`cadence_comp.c/.h`, `assist_modes.c/.h`,
  telemetria `0x6029`), format banku profilu `0x6020/0x6021`, zakładka
  `eVistDrive Profiles` w Canable.
- **Powiązane:** `FW-056_POWER_CURVE_MODE.md` (format banku v4),
  `FW-033_TORQUE_CONDITIONING_PLAN.md`.
- **Poza zakresem:** field weakening. Zgodnie ze specyfikacją najpierw sprawdzamy
  samą kompensację; dopiero gdy `u_abs` dobija do limitu przy 120 rpm i moc nie
  wraca do 500 W, wraca temat osłabiania pola.

---

## 1. Po co to jest — prostym językiem

Im szybciej kręcisz, tym mniej momentu silnik jest w stanie oddać przy tym samym
żądaniu. Efekt: przy 80–100 rpm wspomaganie siada, mimo że pedałujesz tak samo
mocno. Kompensacja podnosi żądanie zależnie od kadencji, żeby moc trzymała się
równo w całym zakresie.

Mnożnik według zmierzonej charakterystyki:

| Kadencja | Mnożnik |
|---|---|
| 0–70 rpm | 100% |
| 80 rpm | 82% |
| 100 rpm | 93% |
| 110 rpm | 106% |
| 120 rpm | 132% |
| powyżej 120 rpm | 132% (trzymane, bez ekstrapolacji) |

Między punktami interpolacja liniowa, więc nie ma skoków — największa zmiana
między sąsiednimi obrotami to 2,6%.

**132% podnosi żądanie, nie limity.** Wszystkie zabezpieczenia — maksymalna moc,
maksymalny prąd, limit prądu baterii, zapas napięcia, temperatura — działają
niezmienione i wciąż obcinają wynik.

---

## 2. Gdzie dokładnie wchodzi w tor sterowania

Wszystkie trzy tryby oparte na pedałowaniu (Power, eMTB, Torque) zbiegają się
w jednej funkcji `finish_power_request()` w `src/assist_modes.c`. Kompensacja
siedzi na jej początku, czyli dokładnie tam, gdzie chce specyfikacja:

```
moc czlowieka -> wspomaganie bazowe -> [KOMPENSACJA KADENCJI]
   -> limit max_motor_power_w
   -> filtr mocy
   -> przeliczenie P/U (limit napieciowy)
   -> limit max_iq_pct
   -> rampa Iq
   -> FOC
```

Mnożnik stosowany jest **jednocześnie** do `motor_power_mw` i do
`phase_iq_request`, zgodnie ze schematem.

### Kiedy jest wyłączona

| Sytuacja | Jak jest zapewnione |
|---|---|
| manetka | throttle jest dokładany jako podłoga prądu **po** `assist_modes_calculate()`, w `ride_control.c` — nigdy nie przechodzi przez ten kod |
| Walk Assist | osobny tor (`walk_motor`), nie wywołuje `assist_modes_calculate()` |
| hamowanie | `safety_cut` zeruje cel w `ride_control.c` |
| sztuczna kadencja przy starcie | jawny warunek `!input->cadence_seeded` |
| start bez obracania korbą | kadencja rzeczywista = 0 → mnożnik 100%, czyli brak wpływu |

---

## 3. Włącznik osobno dla każdego banku

Ustawienie jest **właściwością banku**, nie poziomu: obejmuje wszystkie pięć
poziomów i wszystkie tryby w tym banku. Bank 1 może mieć kompensację włączoną,
a bank 2 wyłączoną — i odwrotnie.

### Format banku v5

Nagłówek bloku był zapełniony (bajty 0–11), więc flaga dostaje nowy bajt:

| | v4 (FW-056) | **v5 (FW-057)** |
|---|---|---|
| nagłówek | 12 B | **13 B** (bajt 12 = flaga) |
| rekord poziomu | 35 B | 35 B — bez zmian |
| cały blok | 189 B | **190 B** |

190 B nadal mieści się w `MP.bank_store[2][192]`, w `BankBlob[192]` oraz
w limicie multiframe (24 ramki × 8 B = 192 B). **Układ EEPROM bez zmian.**

Parser przyjmuje v1/v2/v3/v4/v5. Starsze bloki nie mają tego bajtu, więc po
migracji kompensacja jest **wyłączona** — zachowanie roweru nie zmienia się samo
z siebie po aktualizacji firmware.

Canable odsyła v5 wyłącznie sterownikowi, który sam zgłosił v5; starsze
sterowniki dostają dokładnie taki blok jak dotąd.

**Zapas po tej zmianie: 2 bajty.** Kolejna flaga per bank już się nie zmieści bez
powiększenia buforów — jeśli przewidujemy więcej przełączników, następny powinien
być maską bitową, a nie kolejnym bajtem.

---

## 4. Telemetria — `0x6029` w wersji 4

Blok diagnostyczny rośnie z 37 B do 47 B. Dochodzi dokładnie to, czego wymaga
kryterium odbioru:

| Bajty | Pole |
|---|---|
| 35–36 | zastosowany mnożnik [promile], 1000 = brak kompensacji (peak) |
| 37–38 | moc silnika **przed** kompensacją [W] (peak) |
| 39–40 | `u_abs` (peak) — nasyca się na `_U_MAX`, więc widać dobicie do limitu |
| 41–42 | napięcie paczki [mV] |
| 43 | kadencja bieżąca (nie peak) |
| 44 | ustawienie banku: kompensacja wł./wył. |

Już wcześniej w bloku były: kadencja szczytowa, moc człowieka, wsparcie, moc
silnika po wszystkim, `iq_request`, `i_q_setpoint`, zmierzone `i_q` oraz flaga
`BC_limit`. Razem daje to pełną listę ze schematu.

Canable czyta v1–v4; na starszym firmware nowe pola są `null`, żeby interfejs
mówił „niedostępne", a nie sugerował, że kompensacja zadziałała i nic nie dała.

---

## 5. Pliki

### Firmware

| Plik | Zmiana |
|---|---|
| `inc/cadence_comp.h`, `src/cadence_comp.c` | nowe — mapa i interpolacja |
| `src/assist_modes.c` | wpięcie w `finish_power_request()`, flaga per bank, blob v5 |
| `inc/assist_modes.h` | `ASSIST_BANK_BLOB_LEN` 189 → **190**, getter flagi, dwa pola diagnostyczne |
| `src/main.c` | peak-hold: moc przed kompensacją, mnożnik, `u_abs` |
| `src/CAN_Display.c` | `0x6029` v4 (47 B) |
| `inc/main.h` | tylko komentarz |
| `tests/fw057_cadence_comp.js` | nowy — test mapy |

### Canable

| Plik | Zmiana |
|---|---|
| `bafang-parser.js` | blob v5, `0x6029` v4 |
| `canbus.js` | serializacja v5 (190 B), bajt 12 |
| `ui/index.html` | pasek przełącznika „Cadence compensation" pod wyborem banku |
| `ui/style.css` | styl paska: duży suwak, kolorowa lewa krawędź, plakietka stanu |
| `ui/js/tab-ebics.js` | obsługa przełącznika, blokada na starszym firmware, opis zawsze widoczny |
| `tests/fw056_bank_blob_roundtrip.js` | rozszerzony o v5 |

---

## 6. Weryfikacja wykonana

- `node tests/fw057_cadence_comp.js` → **PASS**. Wszystkie punkty
  charakterystyki trafione co do jednego promila, wartość powyżej 120 rpm
  trzymana do 255 rpm, brak zmiany kierunku wewnątrz odcinków, **największy skok
  między sąsiednimi obrotami 2,6%**, mnożnik nigdy poza 82–132%.
- `node tests/fw056_bank_blob_roundtrip.js` → **PASS**. v5 ma dokładnie 190 B,
  mieści się w 192 B i w 24 ramkach, bajt 12 przenosi flagę w obie strony,
  rekordy poziomów czytają się poprawnie z przesuniętego offsetu, pola Walk
  Assist bez zmian, a sterownik v4 nadal dostaje blok 189 B.
- `node --check` na `bafang-parser.js`, `canbus.js`, `ui/js/tab-ebics.js` → OK.

## 7. Czego NIE zrobiono

- **Build firmware nie był uruchamiany.**
- Brak testów sprzętowych.
- Field weakening — świadomie poza zakresem.
- Canable nie wyświetla jeszcze nowych pól `0x6029` w żadnej zakładce; parser je
  zwraca, więc są dostępne w logu i w zakładce Debug, ale dedykowanego widoku
  kompensacji nie ma.

---

## 8. Kryterium odbioru (do testu sprzętowego)

1. Moc w zakresie 80–120 rpm ma być możliwie bliska 500 W.
2. Przejścia mają być płynne — bez wyczuwalnych skoków przy przekraczaniu
   80/100/110/120 rpm.
3. Przy 120 rpm sprawdzić w `0x6029`, czy `u_abs` nie dobija do limitu.
   **Jeśli dobija — sama kompensacja nie odzyska pełnych 500 W** i dopiero wtedy
   ma sens rozważanie field weakening.
4. Bank z wyłączoną kompensacją musi zachowywać się dokładnie jak przed zmianą.
5. Manetka, Walk Assist i hamowanie bez zmiany zachowania.
6. Limity mocy, prądu, temperatury i napięcia nadal obcinają wynik — przy 132%
   żaden z nich nie może zostać przekroczony.
