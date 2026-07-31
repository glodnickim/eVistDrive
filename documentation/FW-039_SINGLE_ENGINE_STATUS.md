# FW-039 — status trybu ride core-only (karta zbiorcza, w toku)

- **Data:** 2026-07-27
- **Status:** DZIAŁA na rowerze. NIE jest to zamknięty temat — kolejne poprawki
  wychodzą sukcesywnie w miarę jazdy testowej. Ta karta zastępuje statyczne
  „czeka na test" w FW-030/031/032 (nieaktualne — te buildy były wielokrotnie
  przejechane od czasu wydania).
- **Powiązane:** [[FW-030_SINGLE_RIDE_ENGINE]] (baza, jeden silnik), oraz cały
  łańcuch poprawek zbudowany na tej bazie: FW-031 (latch), FW-032 (latch→Canable),
  FW-033 (torque_run, anti-puls nóg), FW-034 (poziom 0 off), FW-035 (bumpless
  start), FW-036 (filtr fałszywych impulsów speed), FW-037 (rampa odcięcia
  zamiast twardego cutu).

---

## Sens tej karty

Od `0.0201` (FW-030, jeden tryb ride core) do `0.0206` powstało osiem kolejnych kart
poprawek — nie dlatego, że ride core nie działał, ale dlatego, że każda jazda testowa
odkrywała kolejny niuans (przeciąganie, dołki mocy, puls nóg, poziom 0, klik
startu/stopu, fałszywa prędkość). To jest normalny tryb dopracowywania nowego
silnika jazdy, nie seria awarii.

**Traktować jako żywy dokument**, nie jako "zamknięte/otwarte" zadanie. Aktualizować
listę potwierdzonych i otwartych punktów po każdej sesji jazdy.

## Potwierdzone jazdą (stan na 0.0206, 2026-07-27)

- Silnik jedzie i nie ciągnie samoistnie (FW-028, `0.0199` — znany-dobry bazowy).
- Pulsowanie „lewa/prawa noga" rozwiązane (FW-033 torque_run, „bardzo dobrze").
- Poziom wspomagania 0 = całkowicie OFF (FW-034).
- Klik przy starcie wyraźnie zmalał (FW-035 bumpless).
- FW-025 (szybsze cięcie po zaprzestaniu pedałowania, 200 ms okno) — potwierdzone,
  zostaje jako OK.

## Otwarte / w toku (stan na 0.0206)

- **Rampa odcięcia (FW-037) — za krótka w praktyce.** Root cause zdiagnozowany:
  `assist_dynamics.c` liczy tempo zejścia od PEŁNEJ skali prądu, więc `release_ms`
  nie daje realnego czasu przy normalnym (niepełnym) poziomie wspomagania. Fix:
  przerobić na fixed-time fade od bieżącej wartości. NIE WDROŻONE jeszcze.
- **FW-036 (filtr fałszywych impulsów speed)** — wdrożone w `0.0206`, konkretny
  scenariusz (cofnięcie korby przy narastającej mocy) jeszcze nie przetestowany.
- **FW-038 (gear preload, start)** — odłożone warunkowo; robić tylko jeśli klik
  startowy nadal przeszkadza po ocenie bumplessu.
- **FW-029 (Walk Assist po prędkości silnika)** — osobny, niezależny plan, wciąż
  tylko plan, zero implementacji.
- **FW-033 reszta** (eMTB curve blend, peak guard) — odłożone, tylko jeśli eMTB
  nadal pulsuje mocniej niż Power Linear.

## Jak z tego korzystać

Przy pytaniu "co zaległe" — patrz sekcja "Otwarte / w toku" wyżej zamiast
przeglądać statusy pojedynczych starszych kart FW-030/031/032 (ich "czeka na
test" jest przestarzałe, mechanizmy w nich opisane działają i są przejechane).
