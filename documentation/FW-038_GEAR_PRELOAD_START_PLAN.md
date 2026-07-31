# FW-038 - plan: gear preload (ciche kasowanie luzu przy starcie)

- **Data:** 2026-07-26
- **Status:** ODROZONE. Robic TYLKO jesli po tescie 0.0206 (bumpless FW-035) klik startowy zostaje.
- **Cel wlasciciela:** delikatne, ciche, niewyczuwalne kasowanie luzu uzebienia przy ruszaniu
  ze spoczynku; po skasowaniu luzu -> przejscie na wlasciwy moment.
- **Zakres:** rozruch mostka w main.c (przy PWM off->on). Bez zmian w safety_cut, reverse,
  hamulcu, Walk Assist, torque_run.

## Zweryfikowane fakty (0.0206)

- FW-035 juz robi elektryczny bumpless: zero PI_iq.out/PI_id.out + neutralny PWM przed ENABLE.
- `get_standstill_position()` ma BLOKUJACY `delay_1ms(25)` (main.c:2302) - zamraza petle 25 ms
  przed ENABLE. Maszyna stanow musi to obejsc nieblokujaco.
- Wykrycie ruchu wirnika dostepne: `ui16_erps` (main.c) + przejscia Halla (`ui8_hall_state`).
- Mostek wlaczany przy pierwszym `MS.i_q_setpoint != 0` (main.c ~739).

## Dwie sciezki

### A. Prosta - "preload cap" na istniejacej rampie (mniejsze ryzyko)
Na SWIEZYM starcie (PWM bylo off, silnik stoi, iq_request>0, PAS forward, !safety_cut):
- ogranicz iq_target do IQ_PRELOAD_MAX (~8-10) przez ~200 ms; istniejaca rampa Iq robi
  plynne narastanie 0->preload = delikatne kasowanie luzu,
- po wykryciu ruchu (erps/hall) LUB timeoutcie (~300 ms) zdejmij limit; ta sama rampa
  ciagnie preload->iq_request = bumpless handover (bez resetu rampy),
- Startup Boost zablokowany do momentu zdjecia limitu.
Zaleta: minimum kodu, bez ruszania blokujacego 25 ms (limit dziala po ENABLE).

### B. Pelna maszyna stanow (propozycja dewelopera; wiecej kontroli, wieksze ryzyko)
OFF -> PRECHARGE (2 ms neutralny PWM) -> PRELOAD (Iq 0->~8-12 przez 200-300 ms, adaptacyjnie
az do ruchu) -> SETTLE (utrzymaj preload ~30 ms po pierwszym ruchu) -> HANDOVER (plynnie do
iq_request, `iq_reference_q = preload<<8` przed assist_dynamics) -> RUNNING.
Wymaga: nieblokujacej wersji get_standstill_position (przerobka delay_1ms(25)), maszyny stanow
w petli 4 kHz, PRELOAD_TIMEOUT/ABORT, Boost zablokowany do HANDOVER.
Parametry startowe: PWM_PRECHARGE_MS 2, IQ_PRELOAD_MAX 8, IQ_PRELOAD_RAMP_MS 200,
PRELOAD_SETTLE_MS 30, PRELOAD_TIMEOUT_MS 300, HANDOVER_RAMP_MS 250.

## Zabezpieczenia (oba warianty)
- Tylko przy realnym swiezym starcie (PWM off->on, silnik stoi) - nie przy kazdej zmianie nacisku.
- Timeout/abort - nigdy nie wisiec, gdy preload za slaby.
- safety_cut / reverse / hamulec / blad - dzialaja bez zmian, tna natychmiast.
- Handover bez resetu rampy (bumpless), zeby nie wrocil klik.

## Test (obowiazkowo stojak najpierw)
Start ze spoczynku: cichy, bez wyczuwalnego kliku, potem przyjmuje wlasciwy moment; silnik
zawsze rusza (nie wisi); reakcja akceptowalna; safety cut natychmiast. Dopiero potem droga.
