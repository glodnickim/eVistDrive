# FW-088 — Krzywa wsparcia nie może czytać startu jako „brak wysiłku"

- **Data:** 2026-08-05
- **Status:** WDROŻONE 2026-08-05 — testy hostowe zielone. **NIEPRZETESTOWANE NA ROWERZE.**
- **Cel:** usunąć słaby start na poziomach Power Progressive i Power Curve.
- **Zakres:** `inc/config.h` (jedna stała), `src/assist_modes.c` (tryby mocy), test hostowy.
  Bez zmian w transporcie, bloku nastaw i UI.
- **Powiązane:** `FW-086_CADENCE_FIRST_PULSE.md` §2.1 (stąd ta karta),
  `FW-087_EXPLICIT_START_PHASE.md`, `FW-056` (krzywa mocy).

## 1. Problem

Moc = nacisk na pedał × prędkość korby. Przy ruszaniu z miejsca korba ledwo się obraca,
więc **moc jest bliska zeru niezależnie od tego, jak mocno naciskasz**.

Krzywa wsparcia dostawała właśnie tę moc jako wejście:

```c
support_ratio_pct = calculate_support_ratio_pct(assist_basis_power_mw, ...);
```

Przy zerowej mocy `input_permille = 0`, więc krzywa zwracała **`support_min_pct`** —
najmniejsze wsparcie dokładnie wtedy, gdy ruszanie pod górę potrzebuje największego.

Skutek zależał od trybu:

| Tryb | Skąd `support_ratio` | Start |
|---|---|---|
| Power Linear | stała (`calculate_power_linear_support_pct`) | **prawidłowy** |
| **Power Progressive** | z mocy przez krzywą | **słaby** |
| **Power Curve** | z mocy przez krzywą | **słaby** |
| eMTB / Torque | z momentu², nie z krzywej mocy | prawidłowy |

To wyjaśnia, dlaczego problem dotyczył tylko części poziomów — Linear był odporny, bo jego
współczynnik jest stały i nie pyta o moc.

## 2. Zmiana

Podczas fazy startu (`FW-087`) **wejście krzywej** jest liczone przy nominalnej kadencji
zamiast przy zerowej:

```c
uint32_t curve_basis_power_mw = prepared.start_phase ?
    calculate_human_power_mw(prepared.assist_load_centikg, START_PHASE_CURVE_RPM) :
    assist_basis_power_mw;
support_ratio_pct = calculate_support_ratio_pct(curve_basis_power_mw, ...);
```

Czyli: **za ten sam nacisk na pedał dostajesz przy ruszaniu ten sam współczynnik wsparcia,
co przy normalnym pedałowaniu.** Wysiłek jest prawdziwy, choć moc jeszcze nie.

`START_PHASE_CURVE_RPM = 60` — zgodne z `PREVIEW_CADENCE_RPM` w podglądzie Canable, więc
wykres i rower mówią to samo.

### 2.1. Co pozostaje nietknięte — celowo

Podstawiona wartość idzie **wyłącznie do krzywej**. Nie zmienia się:

- `human_power_mw` — raportowana moc rowerzysty zostaje prawdziwa (~0), więc wyświetlacz
  i telemetria nie kłamią;
- `assist_basis_power_mw` i `motor_power_mw` — liczone dalej z prawdziwej mocy, więc sufit
  mocy nie jest zawyżony (a i tak jest pomijany podczas startu, patrz `finish_power_request`);
- kształt krzywej, okno `support_min/max`, wszystkie limity i rampy.

Zmienia się jeden współczynnik — nic więcej.

## 3. Dlaczego nominalna kadencja, a nie „maksymalne wsparcie na starcie"

Podstawienie stałego maksimum byłoby prostsze, ale zniszczyłoby proporcjonalność: lekkie
dotknięcie pedału dostawałoby tyle samo, co świadome mocne dociśnięcie. Ocena krzywej przy
nominalnej kadencji **zachowuje pełną charakterystykę** — mocniej naciskasz, więcej
dostajesz — a jedynie usuwa zależność od prędkości korby, której na starcie jeszcze nie ma.
Test 3 pilnuje tej monotoniczności.

## 4. Testy

`tests/fw088_start_curve_input.js`:

1. Stan sprzed zmiany: 30 kg nacisku przy ruszaniu dawało dokładnie `support_min_pct`.
2. Po zmianie ten sam nacisk daje ten sam współczynnik, co przy normalnym pedałowaniu.
3. **Monotoniczność zachowana** — mocniejszy nacisk nadal daje więcej wsparcia, a okno
   `support_max` nadal ogranicza (to nie jest „max na starcie").
4. Po zmierzeniu kadencji faza startu się kończy i nic nie jest podstawiane.
5. Strukturalnie: raportowana moc rowerzysty i `motor_power_mw` nadal liczone z prawdziwej
   mocy — podstawienie czyta wyłącznie krzywa.
6. Strukturalnie: podstawienie zależy wyłącznie od fazy startu.

## 5. Kryteria odbioru

- ruszanie z miejsca na poziomach Progressive i Curve jest tak samo mocne jak na Linear
  przy tym samym nacisku;
- proporcjonalność nacisk → wsparcie zachowana na starcie;
- raportowana moc rowerzysty na starcie nadal ~0 (bez fałszywego skoku na wyświetlaczu);
- brak zmian w zachowaniu po zmierzeniu kadencji;
- pełny zestaw testów firmware i Canable bez regresji.
