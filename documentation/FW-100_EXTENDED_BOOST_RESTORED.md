# FW-100 — Extended Boost: przywrócony dociąg po zatrzymaniu korby

**Status: wdrożone w kodzie, build 0.0310. NIEPRZETESTOWANE NA ROWERZE.**

## Skąd ta karta

Extended Boost istniał od FW-084, ale nigdy nie był przejechany. Przy porządkach FW-095
**odwróciłem jego semantykę** — z „trzymaj moment po zatrzymaniu korby" na „dodawaj prąd tylko
podczas pedałowania" — wykonując dosłownie punkty 6, 10 i 11 pierwotnego zadania, które
zakazywały ciągnięcia po zaniku PAS.

Właściciel doprecyzował, że opisywał tam **funkcję, którą chce mieć**, a nie zakaz jej
istnienia. Ta karta przywraca zachowanie post-PAS z trzema zmianami wobec FW-084.

## Co się zmieniło wobec FW-084

| | FW-084 | FW-100 |
|---|---|---|
| Moc boostu | mapa: nacisk ponad próg → prąd | **ostatnie normalne, ograniczone żądanie Iq podczas pedałowania** |
| Rola progu | uzbrajał **i** wpływał na moc | **tylko uzbraja** |
| Czas maks. | 1000 ms | **2000 ms** |
| Tryb legal | NON_PEDAL → zanik 5–7 km/h | **PEDAL_CONFIRMED → limit 25 km/h** |

Przy `strength_pct = 100` prąd **nie zmienia się** w chwili zatrzymania korby — po prostu
przestaje opadać. To jest ciągłość, nie nowa wartość.

## Zachowanie

| Faza | Warunek |
|---|---|
| **Uzbrojenie** | nacisk ≥ próg, utrzymany 30 ms, przy pedałowaniu, uzbrojonym zatrzasku jazdy i rowerze w ruchu. Uzbrojenie wygasa po 1,5 s |
| **Start** | zbocze zatrzymania pedałowania |
| **Prąd** | `ostatnie_pedal_iq × strength_pct / 100`, przycięte sufitem poziomu i wszystkimi wspólnymi limitami |
| **Czas** | `duration_ms`, domyślnie 0 = wyłączone, maks. 2000 |
| **Koniec** | timer / wznowienie pedałowania / hamulec / cofanie / błąd czujnika / walk / kalibracja / poziom 0 / zmiana poziomu-banku / zapis banku / zatrzymanie roweru |

Wznowienie pedałowania kończy boost **natychmiast jako stan**, ale bez skoku momentu: cel
wraca do wyniku trybu, a `assist_dynamics` prowadzi prąd tam zwykłą rampą. Świadomie **nie**
`max(boost, normal)` — to nakładałoby dwa źródła momentu.

## ⚠ PRZYJĘTE RYZYKO

Silnik napędza przy **nieruchomej korbie**, na rowerze bez niezależnego czujnika hamulca.
W oknie boostu zatrzymać go mogą tylko hamulec i timer.

FW-095 usunęło dokładnie to zachowanie na polecenie, które opisywało zagrożenie. Właściciel
poprosił o przywrócenie, znając powyższe. **To jego decyzja o jego rowerze** i jest tu zapisana,
żeby nikt jej później nie „poprawił", czytając wyłącznie argument bezpieczeństwa.

To samo dotyczy prędkości: aktywny boost jest klasyfikowany jako `PEDAL_CONFIRMED`, więc
w trybie legal podlega limitowi 25 km/h zamiast zanikowi 5–7 km/h. **Wykracza to poza EPAC**,
gdzie wspomaganie bez pedałowania należy do drugiej kategorii. Decyzja właściciela, nie
przeoczenie firmware'u. Zapisana w `ride_control.c` w miejscu, gdzie jest podejmowana.

Co **nie** zostało przywrócone: `profile_hold_active`. Sprawdzone — rampa zwalniania odpala się
wyłącznie przy `iq_target == 0`, a podczas boostu cel jest niezerowy, więc flaga była zbędna.
Zasada z pierwotnego zadania zostaje w mocy: **nic nie fałszuje stanu pedałowania.**

## Zmienione pliki

`inc/assist_extended_boost.h`, `src/assist_extended_boost.c`, `src/ride_control.c`,
`ui/js/evistdrive/profiles.js`, `ui/js/evistdrive/system.js`,
`tests/fw100_extended_boost.js`, `tests/host/fw100_extended_boost_host.c`.

Limit 2000 ms mieści się w istniejącym `put_u16` — **format banku bez zmian**, żaden zapisany
profil nie traci ważności. Layout EEPROM nietknięty.

## Plan testów — stanowisko PRZED jazdą

1. Koło w górze, jeden poziom z `duration = 500 ms`
2. Mocny nacisk → przestań pedałować → silnik ciągnie i **kończy po zadanym czasie**
3. Powtórz i **wciśnij hamulec w trakcie** — moment znika natychmiast
4. Powtórz i **wznów pedałowanie** — boost oddaje sterowanie, bez skoku momentu
5. Uzbrój i **nie zatrzymuj korby przez 2 s** — uzbrojenie ma wygasnąć, boost nie odpala
6. `duration = 0` → funkcja całkowicie cicha

**Dopiero potem jazda**, w bezpiecznym miejscu, nie na technicznym podjeździe.

**Powrót:** `duration = 0` w aplikacji wyłącza funkcję bez wgrywania firmware'u.

## Wynik testów automatycznych

`tests/fw100_extended_boost.js` — PASS. Harness C cross-kompiluje się i linkuje pod
`-Wall -Wextra -Werror`, ale **nie został uruchomiony** — brak kompilatora hosta, runner
raportuje SKIPPED.
