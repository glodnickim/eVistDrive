# Plan: Walk Assist — siła na starcie + trzymanie zadanej prędkości

> **STATUS (2026-07-05):** Plan pisany na zepsutej 0.0114 (martwa pętla za RESCUE return).
> W bazie be40f75 (0.0115+) pętla PI WA jest ŻYWA: kick 180 ms, anti-windup, clamp, limity z Canable.
> **Wdrożone dodatkowo (w kodzie, czeka na build):** fade sufitu mocy przed celem (`WA_FADE_BAND` 1,5 km/h,
> `WA_NEAR_HOLD_PCT` 25%), twarde zero nad celem (`WA_OVERSPEED_MARGIN` 0,5 km/h), martwa strefa
> integratora (`WA_DEADBAND` 0,2 km/h), kasacja martwego `PUSHASSIST_CURRENT`. Wzorzec: TSDZ2
> `apply_walk_assist` — tempo korekt proporcjonalne do odległości od celu.
> **Zostało:** strojenie PI/fade po jeździe (KROK 3) + ew. parametry WA przez CAN (sekcja niżej).
>
> **STATUS (2026-07-06, po jeździe na 0.0132):** KROK 3 wykonany — 4 poprawki po feedbacku:
> (1) zwłoka załączania: WA na szybkiej rampie zewnętrznej (`IQ_RAMP_*_FAST_TICKS` gdy
> `pushassist_flag`); (2) start boost 2×: `WA_START_BOOST_PCT`=200, `WA_START_FULL_SPEED`=300,
> twardy sufit `WA_BOOST_CEIL_PCT`=60% fazowego + podłoga startowa poniżej 3 km/h;
> (3) anty-przelot: `WA_FADE_BAND` 150→250, `WA_NEAR_HOLD_PCT` 25→15, szybka rampa w dół;
> (4) `walk_assist_current` domyślnie 50→30% (boost 2× = równo 60% fazowego).
> Dodatkowo pedał-assist: `TQ_GATE_MIN` 25→18 (niższy odczuwalny próg startu).
> **Zostało:** weryfikacja na rowerze; ew. strojenie PI (KP/KI) i parametry WA przez CAN.

## Objaw (zgłoszony)
Start (pierwsza chwila) OK — jest siła. Ale potem rower **zbyt się rozpędza, przelatuje
zadaną prędkość**, po czym **włącza się cut** w algorytmie. Brakuje trzymania prędkości.

## Przyczyna źródłowa (znaleziona w kodzie)
`update_setpoint()` (main.c:2545) ma **bezwarunkowy `return` w linii 2552** (ścieżka RESCUE).
Regulator prędkości Walk Assist (**pętla PI**, linie 2566-2586) jest **za tym returnem → martwy,
nigdy się nie wykonuje.** Skutki:
- WA jedzie przez ścieżkę „legacy" (`assist_legacy_running_current`) **bez regulacji prędkości**.
- W trybie WA limiter prędkości „legal" jest **pomijany** (`!MS.pushassist_flag` w main.c:2510),
  więc nic nie ogranicza rozpędzania → **przelatuje prędkość**.
- „Cut" to skutek uboczny (timeout WA 10 s / spadek prądu legacy gdy nie pedałujesz / idle) —
  nie kontrolowane trzymanie prędkości. Stąd oscylacja rozpęd→cut.
- „Start OK" = resztkowa siła/kick na początku, ale bez pętli nie ma na czym się zatrzymać.

`PUSHASSIST_CURRENT` (config.h:101=300) jest **zdefiniowane, ale nigdzie nie używane** — martwa stała.

---

## Jak WA MIAŁ działać w EBICS (martwa pętla PI, 2566-2586) — dobra baza

To jest poprawny regulator prędkości i wystarczy go **wykonać**:
```c
wa_max  = phase_current_max * walk_assist_current/100;      // twardy limit siły
wa_limit= walk_assist_speed ? walk_assist_speed : 600;      // zadana prędkość (0.01 km/h)
err     = wa_limit - Speedx100;
out     = P*err + I(integrator);                            // PI na błędzie prędkości
if(Speedx100 >= wa_limit + WA_OVERSPEED_MARGIN){ integral=0; out=0; }  // ANTY-PRZELOT
// integrator co 100 Hz (WA_KI_PERIOD_TICKS=40), anti-windup, clamp 0..wa_max
cap = wa_max * wa_ramp_ticks/WA_RAMP_TICKS;                 // miękki sufit przy starcie
if(out>cap) out=cap;
```
Stałe (config.h:128-134): `WA_RAMP_TICKS=2400` (600 ms kick), `WA_KP_SHIFT=4`, `WA_KI_SHIFT=11`,
`WA_KI_PERIOD_TICKS=40`, `WA_OVERSPEED_MARGIN=50` (0.5 km/h), `WA_KICK_SPEED=50`.
Kick logic (2554-2557): przy postoju `wa_ramp_ticks=0` → sufit prądu narasta od 0 (miękki start);
w ruchu wznów bez kicka. Parametry z CAN: `walk_assist_speed=Para1[60-61]`, `walk_assist_current=Para1[36]`.

**Wniosek: „siła na start" (kick + wa_max) i „trzymanie prędkości" (PI + overspeed→0) już są
zaprojektowane — trzeba je tylko URUCHOMIĆ i dostroić.**

---

## Jak robi to TSDZ2 (`apply_walk_assist`, ebike_app.c:1035) — dla porównania

Reguluje **duty cycle** (nie prąd), z **twardym limitem prądu** i **regulacją na obrotach silnika (erps) z martwą strefą**:
1. **Start:** silnik stoi (erps==0) → `duty = WALK_ASSIST_DUTY_CYCLE_STARTUP` (kick).
2. **Uczenie celu:** narasta duty aż koło ruszy; z `wheel_speed` liczy **erps docelowy** dla
   zadanej prędkości i pasmo `[erps_min, erps_max]` (± próg), ustawia `speed_flag=1`.
3. **Trzymanie (pętla na erps z deadbandem):** erps<min → **powoli** zwiększ duty; erps>max →
   **powoli** zmniejsz (licznik `adj_delay` = wolne kroki → brak przelotu). W paśmie: nie ruszaj.
4. **Twardy limit prądu:** `WALK_ASSIST_ADC_BATTERY_CURRENT_MAX` — ogranicza siłę.
5. **Miękka rampa duty:** `WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP`.

Anty-przelot w TSDZ2 = **martwa strefa wokół celu + wolne korekty + limit prądu**. EBICS
osiąga to samo inaczej (PI + overspeed→0 + limit wa_max) — obie podejścia są OK, byle wykonać.

### Porównanie
| | EBICS (docelowo) | TSDZ2 |
|---|---|---|
| Wielkość regulowana | prąd i_q (PI) | duty cycle |
| Trzymanie prędkości | PI na `Speedx100` + overspeed→0 | pętla na erps z deadbandem, wolne kroki |
| Siła na start | kick: sufit prądu narasta 600 ms | duty startowy `STARTUP` |
| Limit siły | `wa_max = ph_cur×walk_current/100` | `WALK_ASSIST_ADC_BATTERY_CURRENT_MAX` |
| Anty-przelot | overspeed margin → 0 + anti-windup | martwa strefa + adj_delay (wolno) |
| Stan dziś | **MARTWY (return 2552)** | działa |

---

## Plan naprawy (krok po kroku)

### KROK 1 — URUCHOMIĆ regulator WA (przed RESCUE return)
Najmniejsza zmiana: na początku `update_setpoint()`, **przed** linią 2548/2552, dodać:
```c
if(MS.pushassist_flag){
    ... (blok kick 2554-2557 + pętla PI 2566-2586) ...
    return assist_apply_common_limits((uint16_t)out);
}
```
Czyli przenieść istniejący blok WA nad RESCUE. WA przestaje jechać ścieżką legacy.
(Alternatywnie usunąć RESCUE return całkiem — ale to uruchamia też martwą maszynę pedał-assist;
bezpieczniej najpierw tylko WA.)

### KROK 2 — Potwierdzić anty-przelot (rdzeń problemu użytkownika)
- `WA_OVERSPEED_MARGIN=50` (0.5 km/h): po przekroczeniu celu prąd→0 i integrator zerowany
  (2572-2574). To eliminuje „przelatuje prędkość". Zweryfikować, że działa po uruchomieniu.
- Rozważyć **martwą strefę** jak w TSDZ2 (±margin wokół celu bez korekt) dla mniejszego „pompowania".

### KROK 3 — Dostroić PI (żeby trzymał, nie oscylował)
- `WA_KP_SHIFT=4` (P≈1/16 i_q na 0.01 km/h błędu), `WA_KI_SHIFT=11`, I co 100 Hz (`WA_KI_PERIOD_TICKS=40`).
- Za duże P → oscylacja wokół celu; za małe I → nie dobija do celu. Stroić na rowerze:
  zacząć od trzymania stałej prędkości na płaskim, potem pod górę (I kompensuje obciążenie).
- Anti-windup i clamp `0..wa_max` już są (2577-2580) — zostawić.

### KROK 4 — Start (zostaje, działa)
Kick przez `wa_ramp_ticks` (sufit prądu 0→wa_max w 600 ms) daje „siłę na start" bez szarpnięcia.
`WA_KICK_SPEED=50`: przy postoju kick, w ruchu wznów bez kicka. Ewentualnie skrócić/wydłużyć 600 ms.

### KROK 5 — Sprzątanie
- Usunąć nieużywane `PUSHASSIST_CURRENT` (config.h:101) albo użyć jako `wa_max` fallback.
- Upewnić się, że w trybie WA limiter „legal" (2510) ma być pomijany — TAK (WA ma własny limit prędkości).

---

## Nowe/konfigurowalne zmienne (CAN w przyszłości)
Już z CAN: `walk_assist_speed` (Para1[60-61]), `walk_assist_current` (Para1[36]).
Wystawić dodatkowo (wzorem istniejących Para):
| Zmienna | Rola |
|---|---|
| `WA_OVERSPEED_MARGIN` | ile nad cel = twarde zero (anty-przelot) |
| `WA_KP_SHIFT` / `WA_KI_SHIFT` | strojenie PI bez rekompilacji |
| `WA_RAMP_TICKS` | czas kicka/miękkości startu |
| (opcja) `wa_deadband` | martwa strefa wokół celu (styl TSDZ2) |

---

## Weryfikacja (na rowerze / stojaku)
1. Log `Speedx100`, `i_q_setpoint`, `wa_integral`, `walk_assist_speed` (UART main.c:563 / CAN).
2. **Start:** przycisk WA z postoju → siła narasta miękko (~600 ms), rower rusza.
3. **Trzymanie:** prędkość stabilizuje się na `walk_assist_speed` (domyślnie 6 km/h),
   `i_q` maleje przy zbliżaniu do celu; **brak przelotu** (przy celu+0.5 km/h prąd→0).
4. **Pod górę:** integrator dobija prąd, prędkość trzymana; z górki prąd→0, brak rozpędzania.
5. **Puszczenie przycisku / 10 s timeout:** płynne zejście do 0, brak szarpnięcia.
6. Potwierdzić brak oscylacji (jeśli jest — zmniejszyć P przez większy `WA_KP_SHIFT`).

## Ryzyka
- Uruchomienie WA nad RESCUE nie może zepsuć pedał-assist (tamten dalej idzie legacy) — trzymać w osobnym commit.
- Strojenie PI zależne od roweru/masy — udostępnić gainy przez CAN (KROK „nowe zmienne”).
- Overspeed→0 zbyt ostry może dawać „pompowanie” tuż przy celu → martwa strefa (TSDZ2) łagodzi.
