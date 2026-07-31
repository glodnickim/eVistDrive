# Karta zmiany FW-024 — pewne wykrywanie kierunku PAS (zatrzask cofania)

- **Data:** 2026-07-24
- **Status:** ZAMKNIĘTE — POTWIERDZONE NA SPRZĘCIE (`0.0194`, 2026-07-24).
  Test `0x6029`: kręcenie wstecz → flaga cofania TAK i trzyma (41 próbek);
  w przód → „pedałuje", flaga zgaszona (43 próbki); zero przecieku. Wymagało
  poprawki FW-024b (patrz 7a) — sam zatrzask (0.0193) był kasowany przez blok
  `main.c:1775`.
- **Zakres:** wyłącznie warstwa obsługi czujnika PAS (kierunek obrotu korby).
  Bez zmian w silnikach jazdy (Legacy/ride core), rampach, limitach.
- **Powiązane:** [[pas-pulses-per-rev]], diagnostyka `0x6029`.

---

## 1. Prostym językiem

W trybie ride core cofanie korbami nie ucinało wspomagania od razu — moc schodziła
łagodnie, zamiast być odcięta. Pomiar potwierdził: przy 28 s równego kręcenia
wstecz firmware ani razu nie uznał tego pewnie za cofanie.

Przyczyną nie był zły czujnik ani zła matematyka kwadratury (Legacy, który
czyta ten sam czujnik, działa dobrze). Przyczyną był sposób liczenia kierunku:
kroki wstecz i w przód **odejmowały się nawzajem** z progiem 4. Naturalny drobny
jitter korby przy cofaniu wstrzykiwał pojedyncze kroki „w przód", które kasowały
licznik cofania, zanim doszedł do progu.

Poprawka: pierwszy krok wstecz **zatrzaskuje** kierunek na cofanie, a skasować go
może dopiero seria czystych kroków w przód (histereza).

---

## 2. Dowód pomiarowy

Diagnostyka `0x6029` (rower na stojaku, 5 sesji, ride core aktywny):
- 28 s ciągłego kręcenia wstecz (`pas_idle_ms` 1–16 ms → korba się rusza);
- flaga cofania (`Backwards_counter>=4`) i flaga „pedałuje" — **0 na 111 próbek**.

Wniosek: dekoder produkuje mieszane kroki przód/tył; nettowanie z progiem 4 przy
realnym (nieidealnym) cofaniu się nie zapełnia.

---

## 3. Dlaczego Legacy działał, a ride core nie

Oba silniki czytają ten sam wspólny dekoder kwadraturowy (`main.c:1528-1573`);
osobnej obsługi PAS dla Legacy nie ma (`PAS_processing` wyłączony, `main.c:575`).

Różnica była w konsumpcji:
- **Legacy** reaguje na `fwd_run` — każdy krok wstecz zeruje `fwd_run`
  (`main.c:1568`), zatrzask puszcza, moc = 0. Wystarcza złapać cofanie
  sporadycznie.
- **ride core** czeka na twardy `safety_cut` = `Backwards_counter>=4`
  (`main.c:1751`) — a nettowany licznik do tego progu nie dobijał.

---

## 4. Zmiana

Warstwa czujnika, dwa pliki:

- `inc/config.h` — nowy `#define BACKWARD_LATCH_COUNT 8`.
- `src/main.c` (gałąź kroku wstecz, ~`1565-1570`): zamiast
  `if(Backwards_counter<10)Backwards_counter++;` teraz
  `Backwards_counter=BACKWARD_LATCH_COUNT;`.

Mechanika:
- **1 krok wstecz → `Backwards_counter=8`** (natychmiast ≥4 → `safety_cut`
  aktywny w ride core, bramka `Backwards_counter<4` w Legacy).
- Krok w przód dalej odejmuje 1 (`main.c:1539`), więc do skasowania zatrzasku
  trzeba ~5 czystych kroków w przód — histereza zbliżona do re-engage `fwd_run`
  (`START_MIN_STEPS=4`).
- Odporność na jitter: nawet przy naprzemiennych +1/−1 licznik trzyma się
  ~7–8, więc cut nie miga.

Nie zmieniono: samej tablicy `qd[16]` (poprawna), silników, rampy, limitów,
wykrywania stopu (`PAS_STOP_TICKS`).

---

## 5. Ryzyko

Pojedynczy fałszywy krok „wstecz" podczas jazdy w przód (np. martwy punkt korby,
luz łańcucha) zatrzaśnie cofanie na ~5 kroków w przód → krótki zanik wspomagania.
To zachowanie **jest zbieżne z Legacy**, który na taki krok też zeruje `fwd_run`
i re-arm zajmuje 4 kroki. Legacy działa dobrze w jeździe, więc ryzyko oceniane
jako niskie. Fałszywy `−1` przy czystej jeździe w przód jest mało prawdopodobny:
zgubiona próbka daje `qd=0` (połknięte), nie `−1`.

Osobny mechanizm zerujący `Backwards_counter` po ~1 s bez momentu
(`main.c:1771`) nie przeszkadza: przy aktywnym cofaniu kroki wstecz re-latchują
licznik w ciągu ms.

---

## 6. Poza zakresem

- Wykrywanie zatrzymania (`PAS_STOP_TICKS`, stałe 500 ms) — drugi objaw
  („nie docina po zaprzestaniu pedałowania") tkną tu tylko częściowo; reszta to
  rampa w warstwie silnika. Do osobnej oceny po teście tej zmiany.
- Walidacja sekwencji stanów i debounce (pełny wariant ride core) — niepotrzebne,
  zatrzask rozwiązuje zmierzony problem mniejszym kosztem.
- Rozstrzygnięcie typu czujnika (kwadratura vs prędkość+kierunek) — nieistotne
  dla tej poprawki; kadencja i tak liczona poprawnie.

---

## 7. Test po wgraniu

1. `0x6029`, rower na stojaku, ride core aktywny.
2. Kręcić wstecz — flaga cofania (`0x10`) ma zapalać się **natychmiast** i
   trzymać przez całe cofanie.
3. Kręcić w przód — flaga cofania zgaszona, „pedałuje" zapalone.
4. Jazda: cofnięcie korbą ma **od razu** ucinać wspomaganie w ride core.
5. Regresja: Legacy nadal tnie na cofaniu jak dotąd, brak migania wspomagania
   przy normalnej jeździe w przód.

---

## 7a. FW-024b — poprawka po teście 0.0193 (zatrzask był kasowany)

Test `0x6029` na 0.0193 (182 próbki): flaga cofania **nadal ani razu** się nie
zapaliła. Kręcenie w przód wykrywane poprawnie (`pedaluje=tak`), wstecz dawało
`WSTECZ=-`, `pedaluje=-`, korba w ruchu (`pas_idle_ms` 0–29).

Przyczyna: osobny blok „sprzątający" (`main.c:1775-1781`), który po ~1 s bez
nacisku do przodu zeruje `Backwards_counter`. Cofanie nie ma nacisku ani impulsów
kadencji do przodu, więc blok odpalał się CO TAKT i kasował zatrzask 250 µs po
jego ustawieniu. To psuło zarówno stare nettowanie, jak i zatrzask z FW-024.

Poprawka (FW-024b): kasować `Backwards_counter` w tym bloku **tylko gdy korba
faktycznie stanęła** (`pas_idle_ticks>PAS_STOP_TICKS`). Przy ruchu wstecz zatrzask
przeżywa; po zatrzymaniu stara flaga cofania czyści się normalnie.

## 8. Build

- **`0.0194` (z FW-024b) — DO WGRANIA I TESTU:**
  - plik: `.build/0.0194_M820_BL820.bin`, rozmiar `81928 B`
  - SHA-256: `D930641A9D74AEB5E0004E74874F0C4B20765E2E3D681947C179E02FE6819E7D`
  - kompilacja: bez błędów
- `0.0193` (sam zatrzask, bez FW-024b) — WGRANE, przetestowane: flaga cofania
  NIE zapalała się (zatrzask kasowany przez blok `main.c:1775`). Zastąpione 0.0194.
- `0.0192` = identyczny build pośredni 0.0193 (podwójne uruchomienie skryptu).
