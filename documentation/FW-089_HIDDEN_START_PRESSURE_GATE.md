# FW-089 — Ukryty próg nacisku unieważniał ustawiony próg startu

- **Data:** 2026-08-05
- **Status:** WDROŻONE 2026-08-05 — testy hostowe zielone. **NIEPRZETESTOWANE NA ROWERZE.**
- **Cel:** żeby próg startu w kilogramach był jedynym progiem nacisku, jaki decyduje o starcie.
- **Zakres:** `src/main.c` (warunek fazy startu), test integracyjny. Bez zmian w transporcie i UI.
- **Powiązane:** `FW-077` (progi startu w kg), `FW-087_EXPLICIT_START_PHASE.md`,
  `FW-088_START_CURVE_INPUT.md`.

## 1. Problem

Faza startu wymagała dodatkowego, nieudokumentowanego progu nacisku:

```c
MS.torque_on_crank > (750 + TQ_GATE_MIN)   /* > 768 */
```

Zero czujnika wynosi 740 (`TORQUE_ZERO_TARGET_NATIVE`), więc warunek żąda **29 jednostek
ponad zero**. W kilogramach:

| Charakterystyka | Ukryty próg |
|---|---|
| fabryczna (146 jednostek = 6,00 kg) | **1,19 kg** |
| kalibracja użytkownika (span 1139 = 60 kg) | **1,53 kg** |
| po dowolnej rekalibracji | zmienia się, bo próg jest w jednostkach ADC, nie w kg |

Tymczasem progi, które ustawia użytkownik, są znacznie niższe
(`inc/assist_modes.h:124,128`):

- start z postoju: **0,70 kg**
- start w jeździe: **0,30 kg**

**Skutek: dla nacisku w paśmie 0,70–1,19 kg rowerzysta przekracza próg, który ustawił,
a firmware i tak nie uznaje startu.** Faza startu się nie zapala, więc `forward_pedaling`
pozostaje fałszywe, `pedaling_active` też, latch się nie uzbraja i `iq_target = 0`.

Wspomaganie rusza dopiero na 5. przejściu PAS, gdy pojawia się pierwszy prawdziwy pomiar
kadencji i `forward_pedaling` staje się prawdziwe tą drugą drogą — czyli **3,75° obrotu
korby (ok. 15–30 ms) później niż powinno**.

Opóźnienie jest niewielkie, ale problem jest zasadniczy: **ustawienie użytkownika jest po
cichu nadpisywane przez stałą, której nie widzi i której nie da się zmienić z aplikacji**,
a której wartość w kilogramach dodatkowo zmienia się po kalibracji czujnika.

### 1.1. To nie jest regresja

Ten sam warunek bramkował poprzednika fazy startu (ziarno kadencji) — FW-087 przepisało go
bez zmiany. Usterka jest starsza; audyt ją ujawnił, nie stworzył.

## 2. Zmiana

Warunek nacisku znika z fazy startu. Faza startu zależy odtąd **wyłącznie od prawidłowego
ruchu korby do przodu** (`fwd_run >= START_PHASE_STEPS`), a jedynym progiem nacisku jest
konfigurowalny próg w kg sprawdzany w `ride_control`.

## 3. Dlaczego to jest bezpieczne

Obawa z komentarza przy `TQ_GATE_MIN` brzmiała: „bez tego kręcenie korbą w przód i w tył
bez nacisku pobudza silnik". Dziś chronią przed tym dwie inne rzeczy, niezależnie:

1. **`fwd_run`** — liczy *kolejne* kroki do przodu i **zeruje się przy każdym kroku
   wstecz**, więc bujanie korbą nie zbiera się w start (`src/main.c`, gałąź kroku wstecz).
2. **Latch w `ride_control`** — uzbraja się dopiero przy nacisku ≥ ustawiony próg w kg.
   Bez tego `iq_target = 0`, choćby faza startu była podniesiona.

Czyli faza startu może się teraz zapalić bez nacisku, ale **samo to nie daje ani miliampera
prądu** — decyduje wyłącznie próg w kilogramach. Test 4 pilnuje tej własności wprost.

Efekty uboczne podniesienia flagi bez nacisku są znikome: `forward_pedaling` i licznik
bezczynności (auto-off) uznają to za aktywność rowerzysty — czym w istocie jest, bo korba
się obraca do przodu.

## 4. Testy

`tests/fw089_start_pressure_gate.js` — test **integracyjny** całego łańcucha startu
(`main.c` → `rider_input` → `assist_modes` → `ride_control`), bo poprzednie testy sprawdzały
te warstwy osobno i dlatego tej usterki nie wykryły:

1. Przypadek z audytu: nacisk **0,8 kg** przy progu **0,7 kg**, kadencja 0, cztery kroki PAS
   → **Iq > 0**. Przed zmianą: Iq = 0.
2. Start w jeździe: nacisk 0,4 kg przy progu 0,3 kg, trzy kroki (złagodzone o jeden w ruchu)
   → Iq > 0.
3. **Nacisk poniżej ustawionego progu nadal nie startuje** — zmiana nie może zamienić bramki
   w przepustkę.
4. Faza startu bez nacisku nie daje prądu (latch trzyma).
5. Krok wstecz zeruje `fwd_run`, więc bujanie korbą nie zbiera się w start.
6. Ukryty próg w jednostkach ADC nie występuje już w warunku fazy startu (strukturalnie).
7. Wyliczenie z §1 pozostaje udokumentowane: 29 jednostek to 1,19 kg fabrycznie i 1,53 kg
   po kalibracji — test liczy to z prawdziwych stałych, więc zmiana którejkolwiek go zerwie.

## 5. Kryteria odbioru

- próg w kilogramach jest **jedynym** progiem nacisku decydującym o starcie;
- nacisk tuż powyżej ustawionego progu startuje wspomaganie bez czekania na kadencję;
- nacisk poniżej progu nadal nie startuje;
- bujanie korbą w przód i w tył nadal nie uruchamia wspomagania;
- pełny zestaw testów firmware i Canable bez regresji.
