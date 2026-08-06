# FW-087 — Jawna faza startu zamiast fałszywej kadencji

- **Data:** 2026-08-05
- **Status:** WDROŻONE 2026-08-05 — testy hostowe zielone (9 firmware, 7 Canable).
  **NIEPRZETESTOWANE NA ROWERZE.** Build firmware jeszcze nie wykonany.
- **Cel:** usunąć `START_CADENCE_SEED_RPM = 1` — wartość, która udaje kadencję, a jest flagą.
- **Zakres:** `inc/config.h`, `src/main.c`, `inc/rider_input.h`, `src/assist_modes.c`, testy.
  Bez zmian w transporcie, bloku nastaw i UI.
- **Powiązane:** `FW-086_CADENCE_FIRST_PULSE.md` (§5 pkt 1 — stąd ta karta),
  `FW-016_RIDE_CORE_START_FIX.md`.

## 1. Problem

Po dwóch krokach korby przy ruszaniu firmware wpisywał do `MS.cadence` sztuczną wartość
**1 obr/min** i podnosił `cadence_seeded`. To nie jest kadencja — nikt nie pedałuje z
prędkością 1 obr/min. To **flaga „trwa start" przebrana za wielkość fizyczną**.

Skutki tego przebrania:

1. **Kruchość.** Każdy, kto skasuje `cadence_seeded`, wyłącza całą ochronę startu.
   Dokładnie to robiła usterka z FW-086: jeden zepsuty pomiar kasował flagę i wspomaganie
   zapadało się do podłogi.
2. **Dwuznaczność.** `MS.cadence` znaczy raz „zmierzona kadencja", raz „trwa start".
   Konsument nie ma jak ich odróżnić bez zaglądania do drugiej zmiennej.
3. **Zakłamane wskazania.** Sztuczna 1 idzie na wyświetlacz HMI i w telemetrię CAN.

TSDZ2 OSF rozwiązuje ten sam problem czysto — osobnym trybem *Startup assist* i osobnymi
typami wspomagania (Power / Torque / Cadence) — a nie wstrzykiwaniem fałszywej kadencji
do wzoru na moc.

## 2. Ustalenie kluczowe: semantyka JUŻ jest właściwa

Audyt wszystkich konsumentów `cadence_seeded` pokazuje, że **każdy z nich traktuje tę
flagę jako „faza startu" i podstawia 0 za kadencję**:

| Miejsce | Co robi przy `cadence_seeded` |
|---|---|
| `assist_modes.c:753, 839, 922` | `power_cadence = 0` (moc ludzka liczona z zerową kadencją) |
| `assist_modes.c:829` | eMTB podstawia 0 |
| `assist_modes.c:648` | pomija kompensację kadencji |
| `assist_modes.c:695` | pomija sufit prądu wyprowadzony z mocy |

Czyli **nikt nie korzysta z wartości 1** — wszyscy ją omijają. Wartość istnieje wyłącznie
po to, żeby przejść dwie bramki. To czyni zmianę bezpieczniejszą, niż wygląda: usuwamy
coś, czego żadne obliczenie nie czyta.

## 3. Gdzie sztuczna 1 jest nośna (i co trzeba poprawić razem)

Fałszywa kadencja przechodzi **dwie** bramki, nie jedną. Pominięcie drugiej zamknęłoby
start podwójnie:

1. `assist_modes.c:593` — `cadence_for_assist == 0` → `return false`.
2. `main.c:1745` — `forward_pedaling = (MS.cadence>0 && ...)`, a `forward_pedaling` steruje
   `ride_core_pedaling` (`main.c:1781`) → `pedaling_active`, sprawdzane w tej samej
   bramce (`assist_modes.c:592`).

Dodatkowo `MS.cadence>0` bierze udział w wykrywaniu bezczynności (auto-off, `main.c:803`).

## 4. Zmiana

1. **`inc/config.h`** — `START_CADENCE_SEED_RPM` **usunięte**. `START_CADENCE_SEED_ENABLE`
   i `START_CADENCE_SEED_STEPS` → `START_PHASE_ENABLE`, `START_PHASE_STEPS`.
2. **`src/main.c`** — blok startu ustawia **wyłącznie flagę**; `MS.cadence`,
   `uint16_cadence_filtered` i `MS.p_human` nie są już zaśmiecane sztuczną wartością.
   `forward_pedaling` i wykrywanie bezczynności honorują fazę startu jawnie.
   Globalna `cadence_seeded` → `start_phase`.
3. **`inc/rider_input.h`** — pole `cadence_seeded` → `start_phase`.
4. **`src/assist_modes.c`** — bramka przepuszcza start po fladze, a nie po wartości
   kadencji. Lokalna sztuczna `cadence_for_assist = 1` w gałęzi
   `assist_without_rotation` znika (ta gałąź ma już własną flagę
   `without_rotation_active`). Nazwy pól i warunków przeniesione na `start_phase`.

Faza startu kończy się dokładnie tam, gdzie dotąd: przy pierwszym prawdziwym impulsie
kadencji albo przy wykryciu postoju.

## 5. Co się NIE zmienia

- Moment uzbrojenia latcha i wszystkie progi startu (`START_MIN_STEPS`, `TQ_GATE_MIN`,
  „Crank movement to start") — bez zmian.
- Ścieżka prądu na starcie — nadal `calculate_load_iq_request()`, czyli z obciążenia,
  bez kadencji.
- Pominięcie sufitu prądu z mocy i kompensacji kadencji podczas startu — bez zmian,
  tylko warunek nazywa się teraz zgodnie z tym, czym jest.
- Wynik obliczeń wspomagania — żadne obliczenie nie czytało wartości 1 (§2).

**Efekt uboczny, zamierzony:** podczas fazy startu HMI i telemetria pokazują kadencję **0**
zamiast fałszywej 1. To jest wskazanie prawdziwe.

## 6. Testy

`tests/fw087_start_phase.js`:

1. Po dwóch krokach do przodu z naciskiem faza startu jest aktywna, a `MS.cadence`
   pozostaje **0** (dawniej 1).
2. `pedaling_active` jest prawdziwe podczas fazy startu mimo zerowej kadencji — inaczej
   bramka zamknęłaby się drugą drogą.
3. Bramka wspomagania przepuszcza start przy zerowej kadencji, gdy flaga jest podniesiona.
4. Bramka nadal odrzuca zerową kadencję **bez** fazy startu i bez `assist_without_rotation`.
5. Pierwszy prawdziwy impuls kadencji kończy fazę startu (spójne z FW-086).
6. Wykrycie postoju kończy fazę startu.
7. Strukturalnie: `START_CADENCE_SEED_RPM` nie występuje już nigdzie w kodzie.
8. Strukturalnie: `forward_pedaling` honoruje fazę startu.

Plus pełna regresja obu repozytoriów.

## 7. Kryteria odbioru

- `START_CADENCE_SEED_RPM` usunięte z kodu;
- `MS.cadence` niesie wyłącznie zmierzone kadencje — nigdy wartości zastępczej;
- start z miejsca zachowuje się jak przed zmianą (to refaktor, nie zmiana charakteru);
- ruszanie działa również na poziomach z `assist_without_rotation`;
- pełny zestaw testów firmware i Canable bez regresji.

## 8. Nadal otwarte (NIE w tej karcie)

`support_ratio` w trybach Progressive/Curve liczone z mocy = 0 podczas startu spada do
`support_min_pct` (`src/assist_modes.c:753`). Osobna usterka, opisana w FW-086 §2.1.
