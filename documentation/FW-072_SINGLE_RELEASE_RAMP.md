# FW-072 — wygaszanie wspomagania to jedna rampa

- **Data:** 2026-07-31
- **Status:** ZAIMPLEMENTOWANE, **niezbudowane i nieprzetestowane na sprzęcie**
  (build po stronie właściciela).
- **Zakres:** wyłącznie `src/assist_dynamics.c` (kształt wygaszania Iq) + opis pola
  `release_ms` w Canable. **Bez zmian w protokole CAN, formacie banków, WA, FOC, throttle.**
- **Powiązane:** `FW-047_FADE_TAIL.md` — karta wycofana tą zmianą. `FW-040` — poprawka
  liczenia czasu od rzeczywistego prądu, **zachowana**.

---

## 1. Co było nie tak

FW-047 dokładał wolną „końcówkę" na ostatnich 15 % poziomu, od którego zaczynało się
wygaszanie. Założenie było takie, że klik przekładni bierze się ze zdejmowania momentu ze
stałą prędkością aż do zera.

**Testy tego nie potwierdziły — klik został.** Za to końcówka miała skutek uboczny, którego
nie widać w interfejsie: `release_ms` odmierzało zejście tylko do 15 %, a potem szło **jeszcze
600 ms** (250 ms przy cięciu bezpieczeństwa) do rzeczywistego zera. Ustawione 650 ms zachowywało
się jak około 1250 ms. Pole w Canable mówiło jedno, firmware robiło drugie.

## 2. Co robi teraz

```
PAS STOP → pojedyncze RELEASE → OFF
```

Prąd z chwili zatrzymania pedałowania → 0 przez **dokładnie `release_ms`**, niezależnie od
tego, jak wysoki był ten prąd.

Krok rampy liczony **raz**, przy starcie wygaszania:

```c
ticks = release_ms * CONTROL_TICKS_PER_MS;
profile_release_step_q = (iq_reference_q + ticks - 1) / ticks;
```

Zaokrąglenie **w górę** jest celowe: gwarantuje, że rampa faktycznie dochodzi do zera w zadanym
czasie, zamiast pełzać na ostatnim ułamku kroku.

## 3. Co usunięto

- stałe `RIDE_FADE_TAIL_PCT`, `RIDE_FADE_TAIL_MS`, `RIDE_FADE_TAIL_SAFETY_MS`,
- stan `profile_release_tail_q`, `profile_release_tail_step_q` wraz ze wszystkimi
  inicjalizacjami i zerowaniami,
- **dodatkowo** `profile_release_start_q` — po tej zmianie zapisywana w siedmiu miejscach
  i nieczytana ani razu. Rolę znacznika „wygaszanie trwa" pełni `profile_release_step_q != 0`.
  Martwy stan w ścieżce wygaszania to proszenie się o pomyłkę przy następnej zmianie.

`input->safety_cut` przestało być używane w tym module — krótszy czas dla hamulca, cofania
i błędów przychodzi jako osobny `profile_release_ms` z `ride_control` (200 ms,
`RIDE_SAFETY_RELEASE_MS`), więc zachowanie jest bez zmian. Pole w strukturze zostaje; jest
częścią wejścia modułu, nie zmienną lokalną, więc nie generuje ostrzeżenia.

## 4. Co zachowane bez zmian

- **FW-040** — czas liczony od **rzeczywistego** Iq w chwili zatrzymania, a nie od pełnej skali
  prądu silnika. Niski prąd początkowy też jest wygaszany przez pełne `release_ms`.
- `immediate_cut` (wyjście z Walk Assist) — natychmiastowe zero.
- Zachowanie Walk Assist — WA prowadzi własną trajektorię Iq i wychodzi z funkcji przed kodem ramp.
- `coast_release` (FW-048) — może zakończyć rampę wcześniej.
- Ograniczenie `PROFILE_RELEASE_MAX_MS` = 3000 ms.
- Osobny, krótszy czas dla hamulca / cofania korbą / błędu czujnika.
- `release_ms = 0` → wygaszanie prowadzą istniejące rampy `iq_fall_slow_ms` / `iq_fall_fast_ms`.

## 5. Czego świadomie NIE zrobiono

Zgodnie z poleceniem: bez overrunu, bez zmiany `PAS_STOP_TICKS`, bez włączania
`EXTENDED_BOOST_ENABLE`, bez zmian w throttle, bez zmian protokołu i formatu banków, bez
powrotu do liczenia rampy od pełnej skali Iq.

Ewentualny overrun to **osobne, następne zadanie**:
`PAS STOP → opcjonalny OVERRUN → pojedyncze RELEASE → OFF`.

## 6. Canable

Bez nowych parametrów. Opis `release_ms` mówi teraz wprost, że jest to **całkowity** czas
liniowego zejścia od aktualnego prądu do zera, i że **nie ma po nim żadnej dodatkowej końcówki**.

## 7. Testy akceptacyjne

1. `release_ms = 650`, start z wysokiego prądu → **około 650 ms** do zera.
2. `release_ms = 650`, start z ~25 % prądu → **nadal około 650 ms** (to sprawdza FW-040:
   gdyby krok liczył się od pełnej skali, byłoby dużo szybciej).
3. Hamulec w czasie jazdy → krótki czas bezpieczeństwa (200 ms), **bez dodatkowych 600 ms**.
4. Cofanie korbą i błąd czujnika → jak wyżej, bez końcówki.
5. `coast_release` → może zakończyć rampę wcześniej, przy bardzo niskich obrotach.
6. Ponowne rozpoczęcie pedałowania w trakcie rampy → wygaszanie **anulowane**, wspomaganie
   wraca płynnie (krok zeruje się i wraca normalna rampa narastania).
7. `release_ms = 0` → działają `iq_fall_slow_ms` / `iq_fall_fast_ms` z poziomu.
8. Kompilacja bez ostrzeżeń o nieużywanych zmiennych.

**Uwaga do pomiaru:** czasu nie da się wiarygodnie ocenić „na ucho" — najlepiej z logu
diagnostycznego (`iq_setpoint` w czasie) albo z buildu diagnostycznego. Różnica 650 vs 1250 ms
była właśnie dlatego niezauważona przez tak długi czas.
