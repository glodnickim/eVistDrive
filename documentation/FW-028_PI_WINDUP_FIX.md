# Karta zmiany FW-028 — przyczyna i naprawa „przeciągania" wspomagania w ride core

- **Data:** 2026-07-25
- **Status:** WDROŻONE I POTWIERDZONE NA SPRZĘCIE (`0.0199`). Autor poprawki:
  deweloper. „Przeciąganie" zniknęło — silnik jedzie i nie ciągnie po zaprzestaniu
  pedałowania.
- **Zakres:** tor prądu ride core (regulator PI) + twarde odcięcie PWM + rozszerzona
  telemetria diagnostyczna.
- **Powiązane:** [[FW-027_RIDE_CORE_RUNAWAY_DIAG]] (diagnostyka, która NIE mierzyła
  właściwego punktu), [[FW-024_PAS_DIRECTION_LATCH]].

---

## 1. Objaw

W ride core po zaprzestaniu pedałowania silnik napędzał rower **stałą mocą przez kilka
sekund** (bez nacisku, bez kręcenia), a odcięcie działało dopiero po **~2 obrotach
korby do tyłu**. Powtarzalne. Właściciel: „na pewno nie działa cut, jak kręcę w tył".

## 2. Przyczyna podstawowa — windup regulatora PI prądu

Ścieżka ride core (ride-core) ustawiała `MS.i_q_setpoint = 0`, gdy wspomaganie miało
zniknąć — ale **NIE zerowała członu całkującego regulatora prądu**
(`PI_iq.integral_part`, `PI_id.integral_part`). Monolit Legacy miał takie
sprzątanie (`if(!i_q_setpoint_temp && PI_iq.integral_part){...=0}`); ride-core je
POMIJAŁ.

Skutek: **komenda** (żądany prąd) spadała do zera, ale **całka PI zostawała
nawinięta** i regulator dalej wystawiał napięcie → mostek dalej robił moment →
silnik ciągnął jeszcze kilka sekund. Cofnięcie korby (`safety_cut`) wymuszało
zerowanie i zatrzymanie — stąd „2 obroty, zanim przestał" (próg
`Backwards_counter>=4`).

## 3. Dlaczego diagnostyka FW-027 tego NIE pokazała

Telemetria FW-027 logowała **komendę** `MS.i_q_setpoint` (Data1) — która spadała
do zera POPRAWNIE, gdy przestawało się pedałować. Błąd był **za nią**, w regulatorze
prądu: **rzeczywisty** prąd `MS.i_q` i napięcie `MS.u_q` dalej były wysokie.
Mierzyłem zły punkt łańcucha — nakaz zamiast wykonania.

**To jest główny wniosek diagnostyczny:** mierzyć rzeczywiste wyjście siłownika
(prąd/napięcie z FOC), nie tylko komendę. Wszystkie wcześniejsze hipotezy
(pętla kadencji, EMI, `assist_without_rotation`, dryf zera, filtr momentu, rampa)
były SŁUSZNIE odrzucone — bo przyczyna leżała poniżej toru komendy.

## 4. Naprawa (deweloper, `src/main.c`)

Po `ride_control_update()`:
- **Zerowanie całki PI przy zerowej komendzie / safety_cut** (`main.c:1765-1768`):
  ```c
  if(ride_input.safety_cut || MS.i_q_setpoint==0){
      PI_iq.integral_part=0; PI_id.integral_part=0;
  }
  ```
  → gdy żądanie znika, całka pada natychmiast, mostek nie robi już momentu.
- **Twarde odcięcie PWM przy safety_cut** (`main.c:1771-1780`): hamulec/cofanie/
  usterka → natychmiastowe wyłączenie PWM, bez ścieżki „czekaj aż wirnik stanie".

## 5. Nowa telemetria FW-028 (ID `0x00010204`)

Rozszerzenie ramki diagnostycznej o **rzeczywisty stan FOC** po torze komendy:
| Bajty | Pole | Znaczenie |
|---|---|---|
| 0–1 | `MS.i_q` (signed) | **rzeczywisty** prąd — pokazuje windup |
| 2–3 | `MS.u_abs` | napięcie wypadkowe |
| 4–5 | `MS.u_q` (signed) | napięcie z regulatora PI |
| 6 | flagi2 | safety_cut, pwm, pwm_cutoff, brake, backward≥4, tq_fault, BC_limit, overrun |
| 7 | licznik pół-obrotu ×4 ms | do 1020 ms |

Logger `debug-logger-cli.js` łączy `0x00010203` + `0x00010204` w jeden wiersz CSV.

## 6. Build i wynik

- `0.0199` (z FW-028). „Przeciąganie" **zniknęło** — potwierdzone w jeździe.
- Silnik jedzie i nie ciągnie po zaprzestaniu pedałowania. To jest **dobry,
  znany-dobry punkt bazowy do dalszej pracy.**

## 7. Otwarte / do dalszej pracy

- Drobny regres z FW-024 (zatrzask cofania): ~3% rzędów w logu = fałszywe cofanie
  przy jeździe w przód. Do przejrzenia (czułość zatrzasku), ale nie psuje jazdy.
- Ewentualne dostrojenie odczucia po naprawie windup.
