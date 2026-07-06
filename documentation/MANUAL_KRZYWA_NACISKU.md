# Manual: krzywa nacisku (expo per poziom)

*Dla użytkownika — bez wiedzy programistycznej. Stan: w kodzie od buildu następnego po 0.0131.*

## Co to robi

W trybie naciskowym siła silnika zależy od tego, **jak mocno naciskasz pedał**. Krzywa expo
decyduje, **jak nacisk przekłada się na moc** — osobno dla każdego poziomu wspomagania
(Eco, Tour, Sport, Sport+, Boost). Jedna liczba na poziom, od −100 do +100:

| Ustawienie | Charakter | Dla kogo |
|---|---|---|
| `0` | **Prosta** — moc rośnie równo z naciskiem (2× mocniej = 2× więcej) | punkt wyjścia, „Bosch" |
| `+30…+70` | **Progresywna** — lekkie pedałowanie prawie bez asysty, moc przychodzi gdy DOCISKASZ | sport, eMTB, góry |
| `−30…−70` | **Degresywna** — pełna pomoc od pierwszego dotknięcia, u góry już nie przybywa | miasto, komfort, ruszanie |

Końce krzywej są zawsze stałe: zero nacisku = zero mocy, pełny nacisk = pełna moc poziomu.
Gałka wygina tylko środek. Moc **nigdy nie spada** przy rosnącym nacisku — to gwarantuje matematyka
(potęga z dodatnim wykładnikiem), nie dodatkowe warunki.

**Pobaw się symulacją** (suwaki na żywo, tabela wartości):
https://claude.ai/code/artifact/2fd06015-0b0a-40d6-bf53-2dfb3e6df175

## Jak włączyć / zmienić

Plik `inc/config.h`:

```c
#define ASSIST_TORQUE_MODE 2      // 0=kadencyjny (fabryczny EBICS), 1=naciskowy prosta, 2=naciskowy z krzywą

#define ASSIST_CURVE_EXPO_L1 -40  // Eco    - czuły, miejski
#define ASSIST_CURVE_EXPO_L2 -20  // Tour
#define ASSIST_CURVE_EXPO_L3 0    // Sport  - prosta
#define ASSIST_CURVE_EXPO_L4 30   // Sport+ - lekko progresywny
#define ASSIST_CURVE_EXPO_L5 60   // Boost  - czeka na docisk, potem kopie
```

Po zmianie: przebudowa `build_firmware.ps1` → wgranie `.build\<wersja>_M820_BL820.bin`.
Wartości powyżej to **przykład** — domyślnie wszystkie są `0` (czysta prosta).

## Czego ta zmiana NIE dotyka

- **Progi startu i podtrzymanie** — warunki „kiedy silnik w ogóle działa" (kroki korby do przodu
  `START_MIN_STEPS`, minimalny nacisk `TQ_GATE_MIN`, zatrzask trzymający asystę podczas pedałowania)
  są liczone PRZED krzywą i są wspólne dla wszystkich trybów. Krzywa zmienia tylko **ile** mocy,
  nigdy **czy**.
- **Płynność** — rampa czasowa (narastanie/opadanie prądu) działa ZA krzywą, bez zmian.
- **Tryb kadencyjny** (`ASSIST_TORQUE_MODE 0`, domyślny) — działa dokładnie jak dotąd; krzywa
  jest martwym kodem, dopóki nie włączysz trybu 2.
- **Hamulec, wstecz, przegrzanie, limit prędkości** — tną moc tak samo jak zawsze.

## Szczegóły techniczne (dla ciekawych)

Wzór: `moc = pełna_moc_poziomu × x^(1+e)`, gdzie `x` = nacisk znormalizowany 0..1
(po odjęciu progu poziomu, do `TQ_FULL_SCALE_MV`), a wykładnik z procentu gałki:
`E≥0 → 1+E/33.3` (maks. 4.0), `E<0 → 1/(1−E/33.3)` (min. 0.25). Wykładnik liczy się raz przy
zmianie poziomu na HMI; w pętli sterowania to jedno `powf` — tyle samo co człon kadencyjny.
Inspiracja: „Throttle Curve" z VESC Tool (tryb Polynomial).
