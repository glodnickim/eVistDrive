# Walk Assist — jak działa (stan po zmianach „start ×2, utrzymanie ÷2", przed buildem 0.0136)

Dokument opisuje pełny łańcuch trybu prowadzenia roweru (WA): warunki włączenia, etapy mocy
w funkcji prędkości, regulator PI, rampy i zabezpieczenia. Kod: `update_setpoint()` w
`src/main.c` (gałąź `MS.pushassist_flag`), aktywacja ~linia 1545+, parametry w `inc/config.h`
(sekcja `WA_*`, ~linie 139–152 i 332–337).

---

## 1. Włączenie i wyłączenie

`walk_active` = **koniunkcja wszystkich** warunków (liczona co tick, 4 kHz):

| Warunek | Szczegóły |
|---|---|
| `MS.walk_can_request` | HMI wysyła żądanie WA po CAN (przytrzymany „minus"/ikona WA na wyświetlaczu) |
| przycisk fizyczny PA4 | ADC w oknie `WA_BUTTON_THRESHOLD_LOW..HIGH` (3000–3700); debounce 20 ticków = 5 ms na wciśnięcie i tyle samo na puszczenie (histereza anty-chatter) |
| prędkość < 7 km/h | `Speedx100 < 700` — powyżej WA się nie włączy |
| hamulec nie wciśnięty | `!brake_active_flag` |
| brak błędu | `!error_state` |
| brak blokady timeoutu | `!ui8_WA_blocked` |

**Timeout 10 s** (`WA_TIMEOUT_MS`): ciągła praca WA dłużej niż 10 s ustawia `ui8_WA_blocked`
i tnie wspomaganie. Blokada puszcza dopiero, gdy **jednocześnie** zniknie żądanie CAN
i zostanie puszczony przycisk PA4 — trzeba „odkliknąć" i wcisnąć od nowa.

**Moment włączenia (edge):**
- prędkość < 0,5 km/h (`WA_KICK_SPEED`) → **kick**: licznik `wa_ramp_ticks=0`, sufit prądu
  narasta od zera przez 180 ms (`WA_RAMP_TICKS`=720 @4 kHz) — jędrne, ale bez szarpnięcia;
- prędkość ≥ 0,5 km/h (rower już się toczy) → **resume**: sufit od razu otwarty, bez kopa;
- integrator PI zawsze zerowany przy załączeniu (bumpless).

---

## 2. Dwa poziomy odniesienia prądu

Od zmiany po teście 0.0135 start i utrzymanie są **rozdzielone**:

| Wielkość | Wzór | Przy Walk Current = 30% |
|---|---|---|
| `wa_start` — pchnięcie startowe | `phase_current_max × WA_START_PCT/100` = **100% prądu fazowego** | 100% fazowego |
| `wa_hold` — sufit utrzymania | `phase_current_max × walk_assist_current × WA_HOLD_PCT/10000` = **połowa ustawionego Walk Current** | 15% fazowego |

- `wa_start` jest **bezwzględny** — nie zależy od Walk Current zapisanego w HMI/Canable
  (ta sama filozofia co niezależny od poziomów boost pedałowy).
- `wa_hold` skaluje się w firmware (÷2), więc działa niezależnie od tego, co jest zapisane
  w Para1[36]; użytkownik dalej stroi jedną wartością Walk Current w Canable.
- Historycznie (0.0133–0.0135): start = min(200%·wa_max, 60% fazowego) = 60%, hold = pełne 30%.

---

## 3. Etapy wzdłuż osi prędkości (przykład: cel `walk_assist_speed` = 6 km/h)

```
prąd (sufit)
100% faz ┤■ wa_start
         │ ■
         │   ■           strefa 1: PODŁOGA STARTOWA (wymuszony pełny sufit)
         │     ■
         │       ■
         │         ■____________________
 15% faz ┤   wa_hold                    ■■■■        strefa 3: FADE
2,25%faz ┤                                   ■■■■■  (15% wa_hold przy celu)
      0  ┤                                        █ strefa 4: CIĘCIE
         └────┬─────┬───────────────┬───────┬────┬──→ prędkość
              0    3 km/h         3,5     6 km/h 6,5
                (START_FULL_SPEED)  (cel−FADE_BAND) (cel+0,5)
```

**Strefa 1 — start, 0 → 3 km/h** (`WA_START_FULL_SPEED`=300):
- sufit `wa_cap` = liniowo od `wa_start` (100% faz. przy 0) do `wa_hold` (przy 3 km/h);
- **podłoga startowa**: poniżej 3 km/h wyjście jest *wymuszane* do pełnego `wa_cap`
  niezależnie od chwilowego wyjścia PI — gwarantowane dopchnięcie;
- narastanie kształtuje tylko kick-slew 180 ms (przy starcie z miejsca).
- To jest strefa odpowiedzialna za objaw „gna po 1 sekundzie" z testu 0.0135 — dlatego
  teraz podłoga schodzi do dużo niższego `wa_hold`, zamiast trzymać 60% fazowego.

**Strefa 2 — regulacja PI, 3 km/h → (cel − 2,5 km/h):**
- błąd `err = walk_assist_speed − Speedx100` (jednostka 0,01 km/h);
- `out = err·3/16 + integrator>>11` (`WA_KP_NUM/SHIFT`, `WA_KI_SHIFT`);
- anti-windup: integrator liczy tylko, gdy wyjście nie jest w saturacji w kierunku błędu;
- deadband ±0,2 km/h (`WA_DEADBAND`=20): przy celu integrator zamrożony (nie pompuje prądem);
- integrator ograniczony do `wa_hold` (start nie może się nawinąć w całkę);
- clamp wyjścia: 0..`wa_cap` (0 = wybieg, nigdy nie hamuje silnikiem).

**Strefa 3 — fade przed celem, ostatnie 2,5 km/h** (`WA_FADE_BAND`=250):
- sufit dodatkowo wygaszany liniowo od `wa_cap` do `wa_hold × WA_NEAR_HOLD_PCT/100`
  (15% z `wa_hold`, czyli ~2,25% fazowego przy Walk Current 30);
- siła jest ograniczana WCZEŚNIEJ, żeby bezwładność nie przeniosła roweru za cel
  (podejście jak w TSDZ2).

**Strefa 4 — twarde anty-przekroczenie, ≥ cel + 0,5 km/h** (`WA_OVERSPEED_MARGIN`=50):
- wyjście = 0 i integrator wyzerowany; rower toczy się wybiegiem, aż prędkość spadnie.

---

## 4. Rampy — co kształtuje narastanie prądu

Działają dwie warstwy:

1. **Kick-slew wewnątrz WA** — 180 ms (`WA_RAMP_TICKS`=720): sufit `wa_cap` narasta liniowo
   od 0 do pełna po załączeniu z postoju. Tylko przy starcie z miejsca (kick), nie przy resume.
2. **Zewnętrzna rampa `i_q`** (wspólna dla całego firmware, ~linia 1583+): w trybie WA jest
   **przełączana na szybki pas** — narastanie `IQ_RAMP_UP_FAST_TICKS`=1200 (0,3 s pełnej skali),
   opadanie `IQ_RAMP_DOWN_FAST_TICKS`=560 (0,14 s). Bez tego przełączenia WA miałby ~1 s
   zwłoki na postoju (wolny pas 2400 ticków po zmianie pod boost pedałowy; wcześniej 9200),
   a cięcie anty-przelotowe działałoby z opóźnieniem ~1 s zamiast 0,14 s.

Efektywny czas od wciśnięcia do pełnego pchnięcia z postoju ≈ max(0,18 s; 0,3 s) ≈ **0,3 s**
(obie rampy biegną równolegle, ogranicza wolniejsza — zewnętrzna).

**Cięcia natychmiastowe (bez rampy):** hamulec, pedałowanie wstecz (≥4 kroki), przegrzanie
stopnia 2 — `i_q` ustawiane skokowo (w WA hamulec dodatkowo zeruje setpoint na wejściu).

---

## 5. Parametry — ściąga

| Parametr | Wartość | Znaczenie / jednostka |
|---|---|---|
| `WA_START_PCT` | 100 | % prądu fazowego przy 0 km/h (bezwzględny) |
| `WA_START_FULL_SPEED` | 300 | 0,01 km/h — koniec strefy startowej (3 km/h) |
| `WA_HOLD_PCT` | 50 | % z (fazowy × Walk Current) jako sufit utrzymania |
| `walk_assist_current` | 30 (Canable Para1[36]) | % — baza sufitu utrzymania |
| `walk_assist_speed` | MP (Canable) | 0,01 km/h — prędkość docelowa |
| `WA_KP_NUM`/`WA_KP_SHIFT` | 3 / 4 | P = err·3/16 |
| `WA_KI_SHIFT` | 11 | I = integrator>>11 (większy = wolniejszy trym) |
| `WA_DEADBAND` | 20 | ±0,2 km/h — zamrożenie integratora przy celu |
| `WA_FADE_BAND` | 250 | 2,5 km/h — szerokość wygaszania przed celem |
| `WA_NEAR_HOLD_PCT` | 15 | % z `wa_hold` pozostawiane PRZY celu |
| `WA_OVERSPEED_MARGIN` | 50 | +0,5 km/h — próg twardego cięcia |
| `WA_KICK_SPEED` | 50 | <0,5 km/h przy załączeniu = kick, inaczej resume |
| `WA_RAMP_TICKS` | 720 | 180 ms kick-slew |
| `WA_TIMEOUT_MS` | 10000 | 10 s ciągłej pracy → blokada do puszczenia przycisku |
| `IQ_RAMP_UP/DOWN_FAST_TICKS` | 1200 / 560 | zewn. rampa w WA: 0,3 s góra / 0,14 s dół |

## 6. Strojenie po następnym teście

| Objaw | Pierwszy ruch |
|---|---|
| start za ostry / koło buksuje | `WA_START_PCT` 100→80→60 |
| start dalej za słaby w pierwszej chwili | skrócić zewn. rampę w WA (osobny szybszy pas tylko dla WA) lub `WA_RAMP_TICKS` 720→480 |
| za wolno dochodzi do celu po starcie | `WA_HOLD_PCT` 50→65 |
| dalej przelatuje cel | `WA_FADE_BAND` 250→350 albo `WA_NEAR_HOLD_PCT` 15→10 |
| „pompowanie" wokół celu | zwiększyć `WA_DEADBAND` 20→30; ostrożnie z KP/KI (dotąd nieruszane) |

Historia strojeń: `CHANGELOG.md` (sekcje Walk Assist), plan i status testów:
`todo/PLAN_walk_assist_speed_hold.md`.
