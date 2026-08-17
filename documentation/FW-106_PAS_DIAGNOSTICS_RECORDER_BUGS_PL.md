# FW-106 — Dwa błędy rejestratora diagnostycznego (ride_episode / pas_trace CAN), plan diagnostyki v6

Status: **WDROŻONE 2026-08-10** (akceptacja właściciela: „Akceptuję zmianę FW-106").
Firmware NIE zbudowane (`build_firmware.ps1` wymaga osobnego polecenia) i NIE testowane
na rowerze. Co zweryfikowano: wszystkie moduły i `main.c` kompilują się pod ARM GCC 13.2
w OBU wariantach (`CAN_DIAGNOSTICS_ENABLE` 0 i 1), a wszystkie zestawy testów hosta
przechodzą — w tym nowy `tests/host/fw106_recorder_host.c`, który po cofnięciu poprawki
Błędu 1 pada dokładnie na tym niezmienniku, który zawiódł 22 razy w logu z jazdy.

**Odstępstwa od specyfikacji v6, świadome — patrz „Jak wdrożono" na końcu dokumentu.**

Poniższy plan zachowany jako zapis uzgodnień. Kodu nie zmieniano w chwili jego pisania. Sekcje 1-2 to oryginalne znalezisko z jazdy (z
dwiema korektami dowodów z v3, oznaczonymi niżej). Sekcja "Plan wdrożenia v6" to pełna,
jednoznaczna implementacyjnie specyfikacja — zweryfikowana przeze mnie względem obecnego
kodu (`main.c`, `ride_episode.c/h`, `pas_trace.c/h`, `gd32f30x_can.c`), NIE
zaimplementowana. Zgodnie z `collaboration-protocol.md`: żadna zmiana w `src/`/`inc/`
bez zdania „Akceptuję zmianę FW-106".

**Co się zmieniło względem v2 (przegląd właściciela, zamknięte w v3):** v2 zakładało
buforowanie WSZYSTKICH ramek diagnostycznych podczas jazdy — policzalnie niewykonalne.
v2 chciało też podmienić źródło zegara produkcyjnych `pas_idle_ticks`/`pas_cycle_ticks`
w ramach tej samej karty — właściciel słusznie to odrzucił: te liczniki sterują realną
jazdą, więc to NIE jest "measurement only" i wymaga osobnej karty.

**Co się zmieniło względem v3 (przegląd właściciela, zamknięte w v4) — cztery luki
blokujące:**
1. **Surowy tor ISR** miał tylko odwołanie "Zmiana 2 z v2" bez pełnej specyfikacji —
   sekcja 8 niżej definiuje format zdarzenia, dokładny budżet RAM, mechanizm
   PRE/trigger/POST, co uruchamia przechwycenie, nowe CAN ID zrzutu i sposób korelacji
   z istniejącym śladem dekodera.
2. **Brak layoutu 0x10210-0x10214 z session_id/episode_id** — zamiast wciskać nowe pola
   w już pełne ramki (0x10210/11/13 mają dziś 0 wolnych bajtów), dodano NOWĄ ramkę
   nagłówkową 0x1021A (sekcja 6) niosącą `schema_version`+`session_id`+`episode_id`;
   0x10210-0x10214 zostają BAJT W BAJT bez zmian. CAN ID pełni rolę `fragment_id`, jak
   zaproponował właściciel — osobne pole nie jest potrzebne.
3. **"Kilka slotów" bez twardego limitu** — sekcja 7 definiuje `N_DECODER_SLOTS=2` z
   dokładnym rachunkiem bajtów i `_Static_assert`; sekcja 1 sumuje WSZYSTKIE nowe
   bufory do jednego twardego budżetu z zastrzeżeniem, że rzeczywistej wolnej pamięci
   RAM (48 KB to CAŁKOWITA pamięć kontrolera, nie wolna) nie zmierzono w tej karcie —
   zweryfikowanie tego z prawdziwej mapy konsolidacji trafia do planu testów (sekcja 10).
4. **Cykl życia sesji nieokreślony** — nowa sekcja 2 definiuje start, stabilny koniec,
   moment zamrożenia, moment inkrementacji `session_id`, politykę przerwanego zrzutu
   (odrzucenie, nie wznawianie) i sposób liczenia niewysłanych rekordów.

Dodatkowo poprawiono uzasadnienie usunięcia `torque_fast` z `0x10217` (sekcja 7): NIE
jest ono "dostępne na żywo w 0x10219" — pod v4 żadna ramka nie jest żywa podczas jazdy —
tylko świadomie pominięte jako mniej istotne, żeby zwolnić miejsce na indeks i
`capture_id`.

**Co się zmieniło względem v4 (przegląd właściciela, zamknięte w v5) — pięć
sprzeczności wykonawczych:**
1. **Nieblokujący CAN używał błędnie API.** v4 mówiło "wysyłaj tylko gdy
   `can_transmit_states()` pokazuje wolny mailbox" — to nieprawda: sprawdzone w
   [gd32f30x_can.c:440-486](/C:/Projekty/EBICS/BAFANG_GD32F303RCT6/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_can.c#L440-L486)
   i [:498-545](/C:/Projekty/EBICS/BAFANG_GD32F303RCT6/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_can.c#L498-L545),
   to `can_message_transmit()` SAMO wybiera wolną skrzynkę i zwraca `CAN_NOMAILBOX`, a
   `can_transmit_states()` wymaga numeru JUŻ ZAJĘTEJ skrzynki. Sekcja 3 ma teraz
   poprawny automat sześciostanowy.
2. **Rachunek kolejki po porzuceniu był błędny:** `enqueued_total - sent_total` nigdy
   nie wraca do zera po odrzuceniu rekordu. Sekcja 2 ma teraz `discarded_total` i
   wzór `pending = enqueued_total - sent_total - discarded_total`, z `sent_total`
   inkrementowanym dopiero po POTWIERDZONYM wysłaniu CAŁEGO rekordu.
3. **`session_end_tick` po domknięciu 3 s ciszy doliczał te 3 s postoju do metryk
   jazdy.** Sekcja 2 ma teraz mechanizm kandydata: `candidate_end_tick` + kopia
   liczników zapisywane na POCZĄTKU ciszy, zatwierdzane po 3 s, odrzucane przy ruchu.
4. **`0x10218` nie ma miejsca na dane sesji** (wszystkie 8 bajtów zajęte —
   zweryfikowane w `main.c:3459-3467`). Sekcja 9 definiuje nową ramkę `0x1021D` z
   pełnym layoutem. Ponadto: firmware NIE MOŻE policzyć `diag_header_lost` — utrata w
   loggerze jest niewidoczna dla sterownika. Zostaje tylko `tx_header_failed` (stan
   kontrolera CAN); wykrycie braku nagłówka po stronie loggera należy do parsera, nie
   do firmware.
5. **Nadpisywanie najstarszego wpisu w pierścieniu RAW to normalna praca, nie
   przepełnienie.** Sekcja 8 rozdziela `raw_ring_wraps` (informacyjny, rośnie
   normalnie) od `raw_capture_overrun` (rzeczywista utrata podczas snapshotu, MUSI
   zostać zerowy), dodaje atomowy snapshot indeksu ISR przed kopiowaniem PRE i
   pomijanie wartości `0xFF` w generatorze `capture_id` (bo `0xFF` jest sentinelem).

**Co się zmieniło względem v5 (przegląd właściciela, zamknięte w v6) — cztery problemy
telemetrii sesji:**
1. **`0x10218` nadal nie było per-session.** Sam snapshot na końcu nie wystarcza, bo
   liczniki kumulują się od bootu. v6 dodaje `session_start_missed_ticks` /
   `session_start_missed_events` (odejmowane) oraz `session_worst_burst` śledzony
   ODDZIELNIE dla aktywnej sesji (maksimum globalne jest nieodejmowalne — sekcja 9).
2. **`0x1021D` zawierało wyniki PRZYSZŁEGO zrzutu.** `discards_this_session`,
   `diag_tx_failed`, retry i `tx_header_failed` nie są znane w chwili zakończenia jazdy.
   Wybrany wariant (a) z Twoich dwóch: `0x1021D` zostaje NAGŁÓWKIEM sesji z danymi
   znanymi na koniec jazdy, a wyniki transmisji trafiają do NOWEJ ramki końcowej
   `0x1021E` (trailer). Dodana też mała kolejka podsumowań sesji, żeby zatrzymanie
   sesji B nie nadpisało zamrożonych metryk sesji A z przerwanym zrzutem (sekcja 9).
3. **Sprzeczność częstotliwości** między sekcją 3 („`0x10218`/`0x1021D` raz/s") a
   sekcją 9 („`0x1021D` raz na sesję"). Ustalone jednoznacznie: `0x10218` opcjonalnie
   live raz/s podczas postoju ORAZ finalnie po nagłówku; `0x1021D` wyłącznie raz na
   zakończoną sesję (sekcje 3 i 9 zgodne).
4. **`pas_idle_ticks_diag`/`pas_cycle_ticks_diag` nie miały żadnego pola CAN** — nie
   trafiłyby do logu, więc deklarowane porównanie z produkcyjnymi byłoby niewykonalne.
   v6 je USUWA i zapisuje, że rzeczywisty czas liczy się offline z `0x1021C.tick`
   (sekcja 4) — to dane lepsze niż skalar, bo per-tranzycja.

Dodatkowo doprecyzowana polityka pełnej kolejki: **odrzucenie nowego rekordu**, nie
wyparcie starego, z osobnym licznikiem — `discarded_total` rośnie WYŁĄCZNIE dla
rekordu, który wcześniej zwiększył `enqueued_total` (sekcja 2).

Powiązane: FW-097/098/099 (odróżnianie odbicia od realnego cofnięcia), FW-101 (rejestrator
epizodu), FW-102 (surowy ślad PAS), audyt `ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md`
finding F1 (baza czasu).

---

## Błąd 1 — `t_latch` wcześniejszy niż `t_last_reverse` (22 epizody w logu)

**Dowód:** w 22 epizodach z jazdy zarejestrowana wartość `t_latch_ms` jest MNIEJSZA niż
`t_last_reverse_ms` w tym samym rekordzie — matematycznie niemożliwe, jeśli obie miary są
liczone od tego samego zakotwiczenia i latch naprawdę zbroi się PO ostatnim cofnięciu (co
zakłada komentarz w kodzie: "t_latch - t_last_reverse is therefore the honest cost of
re-engagement").

**Przyczyna, zweryfikowana w kodzie
([ride_episode.c:139-164](/C:/Projekty/EBICS/BAFANG_GD32F303RCT6/src/ride_episode.c#L139-L164)):**

1. Latch zbroi się raz: `have_arm` i `t_latch` są ustawiane w `ride_episode_tick()`
   tylko przy pierwszej zmianie `arm_seq` (`if (!have_arm && ...)`) i NIC ich nie zeruje,
   dopóki epizod trwa (`have_arm` czyści dopiero `clear_episode()`).
2. `clear_episode()` wywołuje się TYLKO z dwóch miejsc: (a) pełne opublikowanie epizodu
   (`ride_episode_tick()`, stan RECOVER→publikacja), (b) ponowne zakotwiczenie w
   `ride_episode_reverse_step()`, ale WYŁĄCZNIE gdy `intact==true`
   ([linia 139](/C:/Projekty/EBICS/BAFANG_GD32F303RCT6/src/ride_episode.c#L139)) —
   czyli tylko ze stanu `IDLE` albo `WAIT_DIP` z prądem wciąż ≥90% referencji.
3. Jeśli latch już zazbroił się (epizod jest w `WAIT_RECOVER`) i wtedy przychodzi KOLEJNE
   cofnięcie (np. krótkie kołysanie korby), `intact` wychodzi `false` (bo warunek
   dopuszcza tylko `IDLE`/`WAIT_DIP`) — epizod NIE jest zakotwiczany od nowa, `t_latch`
   zostaje z PIERWSZEGO zbrojenia. Ale linia
   [162](/C:/Projekty/EBICS/BAFANG_GD32F303RCT6/src/ride_episode.c#L162)
   (`t_last_reverse = ticks_to_ms(elapsed_ticks(now_tick));`) i
   `clear_forward_milestones()` wykonują się BEZWARUNKOWO przy każdym cofnięciu, dopóki
   `state != IDLE` — więc `t_last_reverse` skacze naprzód do tego DRUGIEGO cofnięcia,
   podczas gdy `t_latch` wciąż wskazuje PIERWSZE zbrojenie.

**Skutek:** opublikowany rekord miesza dwa cykle: pola zbrojenia (`t_latch`,
`arm_load_centikg`, `arm_fast_native`, `arm_run_seed_native`, `arm_iq_after_limits`)
należą do cyklu 1, a `t_last_reverse`/`t_first_forward`/`t_steps_ready`/`t_load_ready`
należą do cyklu 2 (poprawnie wyzerowane przez `clear_forward_milestones()`, ale
odnoszące się do INNEGO cofnięcia niż to, które zbroiło latch). Odejmowanie
`t_latch - t_last_reverse` z takiego rekordu daje liczbę bez znaczenia (może wyjść
ujemna po odjęciu w analizie), zgodnie z obserwacją w logu.

**To jest błąd w module diagnostycznym (`ride_episode.c`), NIE w torze sterowania jazdą.**
Rejestrator jest "measurement only" (żadna decyzja jazdy go nie czyta) — błąd wypacza
TYLKO interpretację logów, nie zachowanie roweru.

---

## Błąd 2 — ramka `0x10217` nie niesie indeksu próbki

**Dowód z kodu**
([main.c:3395-3423](/C:/Projekty/EBICS/BAFANG_GD32F303RCT6/src/main.c#L3395-L3423)):

```
FW-102 diag (ID 0x00010216/0x00010217): one trace sample, two frames because one does[n't fit]
  0x10216  Data1=index Data2=gap_ticks Data3 hi=from_to lo=flags Data4 hi=disc_pos lo=0
  0x10217  Data1=load_centikg Data2=torque_raw_mv Data3=torque_fast Data4=iq_setpoint
```

Komentarz mówi wprost "one trace sample, two frames" — zakładając, że odbiorca sparuje
`0x10216` z `0x10217` po indeksie. Ale layout `0x10217` wykorzystuje WSZYSTKIE cztery pola
Data na dane obciążenia/momentu, żadne nie niesie indeksu — para jest w rzeczywistości
łączona wyłącznie po KOLEJNOŚCI/CZASIE odbioru na magistrali, nie po jawnym kluczu.

**Dowód z logu (obserwacja użytkownika), SKORYGOWANY:** 2140 ramek `0x10216` wobec 2025
ramek `0x10217` w tej samej jeździe. **Poprawka właściciela:** ta różnica NIE dowodzi
samej utraty na magistrali CAN — część rozbieżności mogła powstać przez scalanie/redukcję
po stronie loggera, nie tylko przez zgubioną ramkę. Co różnica dowodzi JEDNOZNACZNIE: że
parowanie 0x10216↔0x10217 bez jawnego indeksu jest niewiarygodne, niezależnie od tego,
GDZIE dokładnie po drodze traci się spójność liczby ramek.

**Skutek:** nie da się pewnie połączyć śladu PAS (`0x10216`: gap/kierunek/pozycja na
tarczy) z odpowiadającym mu naciskiem (`0x10217`: kg/mV/FAST/Iq) dla tej samej próbki w
zarejestrowanym logu — dokładnie luka, którą `documentation/testing/TEST_INTERFACES.md`
(karta TEST-002) już nazwał "OBSERVABILITY GAP" dla tego mechanizmu, tu potwierdzona na
prawdziwej jeździe. To samo zastrzeżenie (przyczyna nieustalona, ale skutek pewny)
dotyczy brakującego epizodu 48 wspomnianego w oryginalnej notatce: mógł zniknąć przez
nadpisanie `published`, zgubioną ramkę CAN, albo redukcję w loggerze — nie da się dziś
rozstrzygnąć, KTÓRY wariant zaszedł. Kolejka epizodów + jawne identyfikatory (sekcja 6
niżej) rozwiązują problem dla WSZYSTKICH trzech wariantów naraz, więc rozróżnianie
przyczyny nie jest warunkiem wdrożenia poprawki.

---

## Wniosek (zachowany z oryginalnej notatki)

Plan **rearm_after_reverse** pozostaje architektonicznie właściwy: log potwierdza, że
ponowne wymaganie startowego progu nacisku po cofnięciu odpowiada za większość długich
powrotów asysty. NIE rozwiąże jednak fałszywych cofnięć powodowanych utratą próbek na
magistrali/w dekoderze — to osobny problem, który trzeba najpierw odróżnić od realnego
kołysania korby.

## Plan wdrożenia v6 (pełna specyfikacja)

Zakres: WYŁĄCZNIE diagnostyka (build z `CAN_DIAGNOSTICS_ENABLE=1`). Zero zmian w
`start_steps`, progach nacisku, `Backwards_counter`, i zero wdrożenia
`rearm_after_reverse` — jak zastrzeżono w oryginalnej notatce. Zero zmian też w
produkcyjnych `pas_idle_ticks`/`pas_cycle_ticks` (sekcja 4) — te sterują realną jazdą,
nie tylko diagnostyką. Cel jednej jazdy: pomiar bez własnych błędów, nie zmiana
zachowania roweru.

### 1. Budżet RAM — bilans całościowy, twardy limit

**Poprawka właściciela, zweryfikowana rachunkiem:** 23 ramki × 8 B × (112 s / 0,04 s =
2800 wywołań) ≈ 515 KB samego payloadu — buforowanie WSZYSTKICH ramek
`print_debug_on_CAN()` podczas jazdy jest fizycznie niewykonalne. **Druga poprawka
właściciela (v4): kontroler ma 48 KB RAM ŁĄCZNIE, nie 48 KB WOLNEGO** — ile z tego jest
już zajęte przez resztę firmware (stan silnika, bufory CAN, EEPROM shadow, stosy) NIE
zostało w tej karcie zmierzone z prawdziwej mapy konsolidacji (`.map`), więc każdy nowy
bajt jest traktowany jako rzadki, nie jako "prawdopodobnie się zmieści".

**Bilans wszystkich nowych buforów, każdy z twardym `_Static_assert` przy
implementacji:**

| Bufor | Sekcja | Rozmiar | Skąd się bierze |
|---|---|---|---|
| Wolnobieżny pierścień surowych zdarzeń ISR | 8 | 2048 B | 256 zdarzeń × 8 B (`pas_raw_event_t`) |
| Zamrożony slot surowego przechwycenia (1 slot) | 8 | 2048 B | 256 zdarzeń × 8 B, **N_RAW_SLOTS=1** |
| Drugi slot `pas_trace` (dekoder) — NOWY, ponad dzisiejszy 1 | 7 | 3584 B | 256 próbek × 14 B (`pas_trace_sample_t`), **N_DECODER_SLOTS=2** (1 istniejący + 1 nowy) |
| Kolejka epizodów | 6 | ≤2560 B | 64 wpisy × ≤40 B (pole `ride_episode_result_t` ~33 B + `session_id` 1 B + wyrównanie) |
| Kolejka podsumowań sesji | 9 | 128 B | `N_SESSION_SUMMARIES=4` × ~32 B (klucz sesji, delty metryk, czas trwania, kotwice liczników) |
| Liczniki/agregaty/znaczniki sesji (skalar) | 2, 3, 6, 7, 8, 9 | ≤192 B | `session_id`, `session_start_tick`, `session_end_tick`, `candidate_end_tick`, `candidate_metrics` (3×u32), `session_start_missed_ticks`, `session_start_missed_events`, `session_worst_burst` (sekcja 9), `diag_capture_id` (generator), `raw_event_seq`, CZWÓRKI `enqueued_total`/`sent_total`/`discarded_total`/`rejected_total` dla 3 kolejek (sekcja 2), `inflight_mailbox`+stan automatu TX (sekcja 3), `diag_tx_nomailbox`, `diag_tx_failed`, `tx_header_failed`, `capture_slots_full`, `session_summary_rejected`, `raw_ring_wraps`, `raw_capture_overrun`, `raw_freeze_skipped` |
| **Razem (nowe, ponad dzisiejszy stan)** | | **≈10,2 KB** | Istniejący pojedynczy `pas_trace` slot (3584 B) NIE jest liczony jako nowy — już jest w produkcji. `pas_idle_ticks_diag`/`pas_cycle_ticks_diag` USUNIĘTE w v6 (sekcja 4). |

**Twardy ogólny limit:** `_Static_assert(sizeof(fw106_diag_buffers) <= 12*1024, "FW-106 diagnostic RAM budget exceeded")` — 12 KB, z ok. 2 KB zapasu na wyrównanie struktur nieznane dopóki kod nie skompiluje się naprawdę.

**Warunek wejścia (przeniesiony też do sekcji 10):** build diagnostyczny musi być
zbudowany i jego prawdziwa mapa konsolidacji (`.map`, sekcje `.bss`+`.data`) sprawdzona
PRZED jazdą — jeśli po dodaniu tych ~10,2 KB wolne RAM (stos + sterta + margines) spadnie
poniżej bezpiecznego zapasu, karta wraca do cięcia liczby slotów, nie jedzie się z
przepełnionym RAM.

**Co dalej NIE jest buforowane:** pojedyncze ramki `print_debug_on_CAN()` (0x10203 i
in., "na żywo") — te opisują stan chwilowy i podczas jazdy po prostu nie mają odbiorcy
(sekcja 3); nie ma sensu ich przechowywać osobno, skoro liczniki/kolejka/sloty już niosą
to, co istotne retrospektywnie.

### 2. Cykl życia sesji diagnostycznej (`session_id`)

**Luka z przeglądu v3:** `session_id` nie może być tylko "stały od
`ride_episode_init()`" — kontroler diagnostyczny może przejść przez wiele cykli
stop-jazda w jednym włączeniu, i bez jawnych granic sesji dane z różnych postojów
mieszają się dokładnie tak, jak dziś miesza je Błąd 1/2.

**Start sesji:** przejście stop→jazda. Zdarzenie startowe = pierwsze z: impuls koła,
pierwszy krok PAS naprzód/wstecz, albo `MS.i_q_setpoint>0` — cokolwiek nastąpi
najpierw PO zakończonym, stabilnym postoju (patrz niżej). `session_id` (uint8_t,
zawija się przy 256) inkrementuje się DOKŁADNIE w tym momencie, nie wcześniej.

**Warunek stabilnego zakończenia (koniec sesji):** koło nieruchome (brak impulsu) ORAZ
`MS.i_q_setpoint==0` ORAZ PAS bezczynny (`fwd_run==0`, brak kroków wstecz) — wszystkie
trzy jednocześnie, nieprzerwanie przez **3000 ms**. To odróżnia prawdziwy postój od
czerwonego światła z chwilowym dotoczeniem czy resztkowym prądem hamowania regeneracyjnego.

**Poprawka właściciela (v5) — mechanizm KANDYDATA, nie stempla po domknięciu.** v4
ustawiało `session_end_tick` w chwili, gdy 3-sekundowe okno ciszy się DOMYKA — a to
dolicza te 3 sekundy postoju do metryk jazdy (`missed_control_ticks` i pokrewne z
sekcji 9 rosłyby o postój, którego z definicji nie mają mierzyć). Poprawnie:

1. **Początek ciszy** (pierwszy tick, w którym wszystkie trzy warunki są spełnione):
   zapisz `candidate_end_tick = control_time_ticks` ORAZ KOPIĘ wszystkich liczników
   metryk (`missed_control_ticks`, `missed_control_events`, `worst_missed_burst_ticks`)
   do `candidate_metrics`. Nic więcej się nie dzieje — sesja trwa dalej.
2. **Cisza trwa pełne 3000 ms:** kandydat ZATWIERDZONY —
   `session_end_tick = candidate_end_tick`, metryki sesji = `candidate_metrics`. Czyli
   wartości sprzed 3 sekund postoju, nie po nich.
3. **Ruch pojawia się przed upływem 3000 ms:** kandydat ODRZUCONY (`candidate_end_tick`
   i `candidate_metrics` unieważnione), sesja trwa nieprzerwanie dalej, licznik ciszy
   wyzerowany. Kolejna cisza zaczyna wszystko od nowa.

**Kolejność zdarzeń przy zatrzymaniu:**
1. Cisza zaczyna się → `candidate_end_tick` + `candidate_metrics` zapisane (krok 1
   wyżej).
2. Cisza dotrwała 3 s → kandydat zatwierdzony: `session_end_tick` i zamrożone metryki
   przyjmują wartości SPRZED postoju (sekcja 9).
3. Zaczyna się zrzut (drenowanie kolejek zebranych PODCZAS TEJ WŁAŚNIE sesji).
4. `session_id` NIE zmienia się w trakcie zrzutu ani postoju — dopiero przy kolejnym
   starcie (punkt "Start sesji" wyżej).

**Znaczenie `session_id` w kolejce (kluczowe dla sekcji 6/7/8):** `session_id`
zapisywany jest do każdego rekordu (epizod, zamrożony slot `pas_trace`, zamrożony slot
surowy) **w momencie WSTAWIENIA do kolejki (zakończenia epizodu / zadziałania
triggera), nie w momencie zrzutu.** Powód: jeśli zrzut po stopie A zostanie przerwany
ponownym ruchem koła (patrz niżej), a rower zatrzyma się po raz drugi zanim kolejka z
sesji A w pełni się opróżniła, w JEDNEJ kolejce znajdą się obok siebie rekordy z sesji A
(jeszcze niewysłane) i sesji B (nowo dodane) — każdy musi nieść WŁASNY `session_id`
odczytany w chwili powstania, bo w chwili zrzutu "aktualna sesja" już nic nie mówi o
tym, kto go wyprodukował.

**Przerwany zrzut — ODRZUCENIE, nie wznowienie:** jeśli koło ruszy w trakcie zrzutu
(sekcja 3), JEDEN rekord będący akurat w trakcie wysyłania (część fragmentów już
poszła, część nie) jest PORZUCANY w całości — żaden kolejny fragment tego konkretnego
rekordu nie jest wysyłany. Pozostała część kolejki (rekordy jeszcze nietknięte) zostaje
nietknięta i czeka na następny postój. Wybrano odrzucenie zamiast wznowienia świadomie:
sklejanie połowy rekordu wysłanej teraz z drugą połową wysłaną przy następnym postoju
(potencjalnie po kolejnej jeździe, z inną `session_id`) odtwarzałoby dokładnie ten sam
rodzaj błędu co Błąd 1/2 — rekord z dwóch różnych chwil sklejony w jeden.

**Liczenie niewysłanych rekordów — poprawka właściciela (v5).** v4 podawało
`pending = enqueued_total - sent_total`, co jest BŁĘDNE: po każdym porzuconym rekordzie
ta różnica zostaje trwale zawyżona i nigdy nie wraca do zera, więc „ile jeszcze czeka"
przestaje odpowiadać rzeczywistości już po pierwszym przerwanym zrzucie. Poprawnie —
każda z trzech kolejek (epizody, sloty `pas_trace`, slot surowy) ma TRZY wolnobieżne
liczniki uint32, inkrementowane, nigdy nie zerowane poza bootem:

| Licznik | Kiedy rośnie |
|---|---|
| `enqueued_total` | rekord PRZYJĘTY do kolejki (koniec epizodu / zadziałanie triggera, gdy było miejsce) |
| `sent_total` | **dopiero po POTWIERDZONYM wysłaniu CAŁEGO rekordu** — czyli gdy OSTATNI fragment dostał `CAN_TRANSMIT_OK` (sekcja 3). Nigdy po samym `can_message_transmit()`, nigdy po pojedynczym fragmencie. |
| `discarded_total` | rekord **wcześniej PRZYJĘTY** (a więc taki, który zwiększył `enqueued_total`) porzucony w całości: ruch koła w trakcie zrzutu albo przekroczone `DIAG_TX_MAX_RETRY` |
| `rejected_total` | rekord **NIEPRZYJĘTY** przy pełnej kolejce — nigdy nie zwiększył `enqueued_total`, więc NIE zwiększa też `discarded_total` (patrz polityka niżej) |

```
pending = enqueued_total - sent_total - discarded_total
```

Ten wzór wraca do zera po każdej w pełni obsłużonej sesji, niezależnie od tego, ile
rekordów wysłano, a ile porzucono — i to jest kryterium testowe w sekcji 10.
`queue_depth_now` w nagłówku 0x1021A (sekcja 6) jest wygodnym, ale wtórnym odczytem tej
samej wielkości.

**Poprawka właściciela (v6) — jednoznaczna polityka PEŁNEJ kolejki: ODRZUCENIE NOWEGO,
nie wyparcie starego.** v5 wymieniało „wyparcie z pełnej kolejki" jako powód wzrostu
`discarded_total`, co rozjeżdżało wzór na dwa sposoby naraz: rekord wyparty przed
przyjęciem nigdy nie zwiększył `enqueued_total`, a rekord wyparty po przyjęciu znikał z
kolejki bez śladu w `pending`. Ustalone:

- Gdy kolejka jest pełna, **nowy rekord NIE jest przyjmowany** (`rejected_total++`),
  a zawartość kolejki pozostaje nietknięta. Zero wypierania.
- **`discarded_total` rośnie WYŁĄCZNIE dla rekordu, który wcześniej zwiększył
  `enqueued_total`.** To niezmiennik: każdy przyjęty rekord kończy DOKŁADNIE jednym
  zdarzeniem — `sent_total++` albo `discarded_total++`. Nigdy obydwoma, nigdy żadnym.
- Uzasadnienie wyboru „odrzuć nowy" zamiast „wyprzyj stary": rekordy w kolejce są
  starsze, czyli BLIŻSZE zdarzeniu, które badamy (moment utraty asysty); nowsze rekordy
  z tej samej jazdy są mniej wartościowe. Dodatkowo wypieranie kolidowałoby z rekordem
  będącym akurat w locie (sekcja 3). Utrata i tak jest jawna — przez `rejected_total`
  i bit w `tx_error_summary` (sekcja 9).

### 3. CAN nieblokujący W OBIE STRONY — poprawny automat wg prawdziwego API sterownika

**Poprawka właściciela:** zrzut po zatrzymaniu roweru NIE może używać tego samego wzorca
`while(timeout--)`, który jest źródłem problemu w pierwszej kolejności — inaczej zrzut
sam staje się nowym blokującym mechanizmem, tylko przesuniętym w czasie.

**Druga poprawka właściciela (v5) — v4 używało API BŁĘDNIE.** v4 mówiło "wysyłaj tylko
gdy `can_transmit_states()` pokazuje wolny mailbox". Sprawdzone w źródle sterownika i
POTWIERDZONE:

- [`can_message_transmit()`](/C:/Projekty/EBICS/BAFANG_GD32F303RCT6/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_can.c#L440-L457)
  SAM przegląda `CAN_TSTAT_TME0/1/2` i wybiera wolną skrzynkę; gdy żadna nie jest
  wolna, zwraca `CAN_NOMAILBOX` i **nie zapisuje niczego do rejestrów** — nic nie
  ginie, komunikat po prostu nie został przyjęty.
- [`can_transmit_states(periph, mailbox_number)`](/C:/Projekty/EBICS/BAFANG_GD32F303RCT6/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_can.c#L498-L545)
  wymaga NUMERU JUŻ ZAJĘTEJ skrzynki (0/1/2) — dla `CAN_NOMAILBOX` trafia w `default`
  i zwraca `CAN_TRANSMIT_FAILED`. Nie jest to funkcja "czy jest wolna skrzynka".

**Poprawny automat (jeden krok na jedno wywołanie `print_debug_on_CAN()`, ZERO pętli
spinujących):**

```
stan IDLE (nic w locie):
    mb = can_message_transmit(CAN0, &msg_dla_biezacego_fragmentu);
    if (mb == CAN_NOMAILBOX):
        diag_tx_nomailbox++;      // informacyjny, normalna praca przy zajętej magistrali
        NIE przesuwaj kursora fragmentu; wyjdź, spróbuj w następnym wywołaniu
    else:
        inflight_mailbox = mb; stan = PENDING; wyjdź

stan PENDING (fragment w locie, mailbox znany):
    st = can_transmit_states(CAN0, inflight_mailbox);   // JEDNO sprawdzenie, bez pętli
    if (st == CAN_TRANSMIT_PENDING):
        wyjdź, sprawdź ponownie w następnym wywołaniu
    if (st == CAN_TRANSMIT_OK):
        PRZESUŃ kursor fragmentu (fragment zatwierdzony); stan = IDLE
        jeśli to był OSTATNI fragment rekordu: sent_total++ (sekcja 2)
    if (st == CAN_TRANSMIT_FAILED):
        diag_tx_failed++;
        NIE przesuwaj kursora — ten sam fragment zostanie PONOWIONY; stan = IDLE
        jeśli diag_tx_failed_streak dla tego fragmentu przekroczy DIAG_TX_MAX_RETRY=8:
            porzuć CAŁY rekord (discarded_total++, sekcja 2), przejdź do następnego
```

Kluczowe własności: kursor fragmentu przesuwa się **wyłącznie po `CAN_TRANSMIT_OK`** —
nigdy „na wiarę" po samym `can_message_transmit()`. `CAN_NOMAILBOX` NIE jest błędem i
NIE gubi fragmentu (sterownik nic wtedy nie zapisał). `DIAG_TX_MAX_RETRY=8` chroni przed
zaklinowaniem całego zrzutu na jednym fragmencie przy trwałej awarii magistrali —
porzucenie jest wtedy jawne i policzone, nie ciche.

**Przerwanie ruchem koła:** jeśli koło ruszy w trakcie zrzutu — zrzut przerywamy
NATYCHMIAST (priorytet: jazda, nie diagnostyka). Fragment będący akurat w stanie
PENDING zostawiamy sterownikowi (nie wołamy `can_transmission_stop()` — sprzęt sam go
dokończy albo nie, to bez znaczenia), a CAŁY rekord jest porzucany zgodnie z polityką z
sekcji 2 (`discarded_total++`).

**Poprawka właściciela (v6) — jednoznaczna częstotliwość ramek sesyjnych.** v5 mówiło
tu „`0x10218`/`0x1021D` (raz/s)", a sekcja 9 „`0x1021D` raz na sesję" — sprzeczność.
Obowiązuje:

| Ramka | Kiedy wysyłana |
|---|---|
| `0x10218` | OPCJONALNIE live raz/s podczas postoju (wartości wolnobieżne od bootu, jak dziś) ORAZ obowiązkowo RAZ jako wersja FINALNA, bezpośrednio po nagłówku `0x1021D` — wtedy niesie delty sesji (sekcja 9). Parser rozpoznaje wersję finalną po tym, że poprzedza ją `0x1021D`. |
| `0x1021D` | WYŁĄCZNIE raz na zakończoną sesję, jako pierwsza ramka jej zrzutu |
| `0x1021E` | WYŁĄCZNIE raz na zakończoną sesję, jako ostatnia ramka jej zrzutu (trailer) |

Wszystkie przechodzą przez ten sam automat wyżej — bez `while(timeout--)` nigdzie.

### 4. NIE zmieniać teraz produkcyjnych `pas_idle_ticks`/`pas_cycle_ticks`

**Poprawka właściciela — najważniejsza korekta zakresu w tej karcie.** Te liczniki
(`main.c`, wewnątrz `reg_ADC_processing()`) faktycznie liczą dziś WYWOŁANIA, nie
rzeczywisty czas (ten sam wzorzec, który FW-103/104 naprawiły dla prędkości i
`ride_episode`, ale nie tu — "Commit B" w pamięci projektu). ALE `pas_cycle_ticks`
napędza `MS.cadence = 10000/pas_cycle_ticks` i bramkę `pas_cycle_ticks>70`, a
`pas_idle_ticks` napędza wykrywanie zatrzymania (`pas_stop_timeout`) — obie rzeczy
sterują REALNĄ asystą, nie tylko diagnostyką. Zmiana ich źródła zegara, nawet przy
zachowanej semantyce, jest zmianą funkcjonalną i wymaga OSOBNEJ karty z własnym planem
testów na rowerze — nie mieści się w „measurement only" tej karty.

**Poprawka właściciela (v6) — równoległe liczniki `_diag` USUNIĘTE z zakresu.** v5
deklarowało `pas_idle_ticks_diag`/`pas_cycle_ticks_diag`, ale nie przypisało im ŻADNEGO
pola CAN — nie trafiłyby więc do logu i deklarowane porównanie z wartościami
produkcyjnymi byłoby niewykonalne. Zamiast dodawać kolejną ramkę tylko dla dwóch
skalarów, w v6 te zmienne znikają, a porównanie robi się OFFLINE z danych, które karta
i tak wysyła:

- **Rzeczywisty czas** między tranzycjami PAS = różnica pól `tick` z kolejnych ramek
  `0x1021C` (surowy tor ISR, sekcja 8) — dane per-tranzycja, z rozdzielczością 4 kHz,
  odporne na opóźnienia pętli głównej z definicji (stempluje je ISR).
- **Co widział dekoder** = `gap_ticks` z odpowiadającej ramki `0x10216` (ten sam
  `capture_id`, sekcja 7).
- **Rozjazd między nimi** to dokładnie odpowiedź na pytanie "czy produkcyjny licznik
  zaniża rzeczywisty czas i o ile" — i to odpowiedź BOGATSZA niż dwa skalary, bo pokazuje
  rozkład błędu w czasie, a nie jedną wartość.

Ograniczenie zapisane wprost: ta analiza działa w oknach przechwycenia (256 zdarzeń
wokół triggera), nie na całej jeździe. Do decyzji "czy i jak naprawiać produkcyjne
liczniki" (osobna, przyszła karta) to wystarcza — potrzebny jest dowód rozjazdu i jego
skala, nie ciągły zapis całej jazdy.

### 5. `ride_episode` — jeden anchor, poprawiony reset przy kolejnym reverse

**Poprawka właściciela względem v2:** NIE wprowadzać nowego `attempt_anchor` (to
zmieniłoby znaczenie istniejących pól/ramek CAN). Zamiast tego: wszystkie `t_*` nadal
liczone od PIERWSZEGO `anchor_tick` epizodu, bez zmian. Naprawa dotyczy WYŁĄCZNIE resetu
stanu zbrojenia przy kolejnym cofnięciu.

**Plan:** każde KOLEJNE cofnięcie aktywnego epizodu — również gdy `iq_setpoint_now<=0`
(dziś wczesny `return` w `ride_episode_reverse_step()` pomija to całkowicie, wymaga
restrukturyzacji funkcji, żeby reset zaszedł PRZED tym `return`, gdy epizod jest
aktywny) — resetuje: `have_arm=false`, `arm_seq_at_anchor=`bieżący `arm_seq` (żeby
kolejna PRAWDZIWA zmiana liczyła się jako nowe zbrojenie), `t_latch=NONE`,
`t_target_ready=NONE` (dziś NIE czyszczone przez `clear_forward_milestones()` — to
osobna, dodatkowa poprawka), snapshot zbrojenia (`arm_load_centikg`, `arm_fast_native`,
`arm_run_seed_native`, `arm_iq_after_limits`), oraz milestones forward/load (już robi
`clear_forward_milestones()` — bez zmian). Nowy `t_latch`, gdy zajdzie, jest liczony
znowu od PIERWSZEGO `anchor_tick` — czyli `t_latch - t_last_reverse` pozostaje poprawne
z definicji (oba mierzone od tego samego, jednego punktu), a ŻADNA istniejąca ramka CAN
nie zmienia znaczenia pól.

### 6. Kolejka epizodów + nagłówek 0x1021A (CAN ID = `fragment_id`)

**Poprawka właściciela względem v2:** nie tylko 0x10216/0x10217 potrzebują wspólnego
klucza — **0x10210-0x10214 też**. Zweryfikowane w kodzie: te pięć ramek jest dziś
budowanych z JEDNEGO odczytu `ride_episode_get_result(&ep)` (`main.c:3216-3217`) w
ramach jednego wywołania `print_debug_on_CAN()`, więc parowanie "po współbieżności"
działa DZIŚ tylko dlatego, że jest jeden `published` na raz. Kolejka epizodów bez
wspólnego klucza na wszystkich pięciu ramkach zepsułaby to natychmiast, gdy więcej niż
jeden epizod czeka na wysłanie.

**Luka z przeglądu v3, zamknięta tutaj:** próba wciśnięcia `session_id`+`episode_id`
bezpośrednio w 0x10210-0x10214 nie działa arytmetycznie — sprawdzone bajt po bajcie w
kodzie (`main.c:3218-3354`): **0x10210, 0x10211 i 0x10213 wykorzystują dziś WSZYSTKIE
8 bajtów, zero wolnych**; tylko 0x10212 i 0x10214 mają po 3 bajty rezerwy (FW-107:
0x10212 data[5] przestał być rezerwą — patrz sekcja 11 — więc od schema_version 2 zostają
tam już tylko 2 wolne bajty, data[6]-[7]). Zamiast
okaleczać istniejące, już przetestowane pola, dodano NOWĄ ramkę nagłówkową i
zastosowano sugestię właściciela: **sam CAN ID pełni rolę `fragment_id`** — 0x10210 to
zawsze "fragment 1 z 5", 0x10214 to zawsze "fragment 5 z 5", więc osobne pole
`fragment_id` w danych jest zbędne.

**Kolejka:** bufor pierścieniowy 64 wpisów `ride_episode_result_t` zamiast jednego pola
`published` (wypiera dzisiejsze nadpisywanie, patrz Błąd 2). Każdy wpis dostaje
`session_id` (sekcja 2, zapisany PRZY WSTAWIENIU, nie przy zrzucie) w chwili
opublikowania epizodu. Przy pełnej kolejce obowiązuje polityka z sekcji 2: nowy epizod
jest ODRZUCANY (`rejected_total` tej kolejki rośnie), zawartość nietknięta — nigdy
wyparcie starszego wpisu.

**Nowa ramka `0x0001021A` — nagłówek grupy epizodu, wysyłana jako PIERWSZA (przed
0x10210), jedna na każdy zdrenowany epizod:**

| Bajt | Pole | Znaczenie |
|---|---|---|
| 0 | `schema_version` | Wersja UKŁADU ramek diagnostycznych PAS/epizod (nie wersja tego dokumentu). To wdrożenie (v6) = `1`; od FW-107 (sekcja 11) = `2`. Przyszła karta zmieniająca layout MUSI ją podbić. |
| 1 | `session_id` | Z sekcji 2, zapisany przy wstawieniu do kolejki |
| 2-3 | `episode_id` | = `ep.number` (u16, big-endian) — redundantne z 0x10210 Data0-1, ale NIEZALEŻNE: identyfikuje grupę nawet jeśli 0x10210 akurat zgubi się na magistrali |
| 4 | `fragment_count` | Ile ramek fragmentów następuje (dziś zawsze `5`: 0x10210..0x10214) |
| 5 | `queue_depth_now` | Ile kompletnych epizodów czeka jeszcze ZA tym w kolejce (nasyca się na 255) |
| 6 | `flags` | bit0: zarezerwowany (zawsze 0 — polityka ODRZUCENIA z sekcji 2 oznacza, że wznowiony zrzut nigdy nie występuje) |
| 7 | (rezerwa, 0) | |

**0x10210-0x10214 pozostają BAJT W BAJT bez zmian** względem dzisiejszego kodu — zero
ryzyka regresji na pięciu już wysyłanych, już przetestowanych layoutach.

**Zachowanie, gdy nagłówek 0x1021A akurat przepadnie** (rzadkie po naprawie z sekcji 3,
ale nie niemożliwe): `episode_id` nadal odzyskiwalny z 0x10210, więc epizod pozostaje w
pełni użyteczny do analizy WEWNĄTRZ sesji (czasy `t_*` są spójne). Nieznany jest
wyłącznie jego `session_id`.

**Poprawka właściciela (v5) — kto to wykrywa:** firmware liczy TYLKO `tx_header_failed`
(nagłówek, którego kontroler CAN nie zdołał wysłać mimo `DIAG_TX_MAX_RETRY` prób — jedyna
utrata widoczna dla sterownika). Utratę po stronie loggera wykrywa PARSER, po dziurze w
ciągłości `episode_id` — pełne uzasadnienie i podział odpowiedzialności w sekcji 9.

### 7. Skończony budżet dla śladu PAS (dekoder) — multi-slot + `capture_id`

**Poprawka właściciela względem v2/v3:** zamiast nieograniczonego "kilku slotów" —
jawny, policzony budżet z `_Static_assert`, i dokładne zachowanie po zapełnieniu.

**Dlaczego w ogóle więcej niż jeden slot:** pod v4 (sekcja 1/3) NIC nie drenuje się
podczas jazdy — cały zrzut czeka do postoju. Dzisiejszy pojedynczy bufor `pas_trace.c`
(`PAS_TRACE_LEN=256`) zamraża się przy triggerze i NIE przyjmuje kolejnego, dopóki ktoś
go nie zwolni (`pas_trace_release()`) — czyli drugi trigger w tej samej jeździe, zanim
rower się zatrzyma, dziś nadpisałby albo zgubił pierwszy. Multi-slot = kolejka
przechwyceń, analogiczna do kolejki epizodów (sekcja 6), nie ciągły strumień.

**Budżet, dokładnie policzony:** `sizeof(pas_trace_sample_t)` = 14 B przy naturalnym
wyrównaniu (2+1+1+1+1(pad)+2+2+2+2 = 14; do zweryfikowania `sizeof()`-em w kodzie hosta
przy implementacji, nie tylko na papierze). `PAS_TRACE_LEN=256` × 14 B = 3584 B na
slot. Dzisiejszy JEDEN slot (3584 B) już jest w produkcji — nie liczy się jako nowy
budżet tej karty. **`N_DECODER_SLOTS=2`** (dzisiejszy 1 + 1 NOWY) — koszt tej karty to
dokładnie **+3584 B**, z `_Static_assert(sizeof(pas_trace_pool) <= 2*256*sizeof(pas_trace_sample_t), ...)`.
Asymetria względem surowego toru ISR (sekcja 8, tylko 1 slot) jest świadoma: wynik
dekodera (kg/moment/kierunek/pozycja) jest GŁÓWNYM artefaktem analizy, surowy ISR jest
tylko kontrolą wiarygodności dekodera — przy ograniczonym RAM priorytet dostaje główny
artefakt.

**Zachowanie po zapełnieniu obu slotów:** kolejny trigger w tej samej jeździe NIE
zamraża nic nowego — licznik `capture_slots_full` rośnie, zdarzenie jest POLICZONE, ale
bez szczegółowego śladu. To ten sam, uczciwie udokumentowany kompromis co przy kolejce
epizodów (64 wpisy) — jawna, mierzalna granica zamiast cichej utraty.

**`capture_id` — wspólny klucz surowy↔dekoder (zamyka Błąd 2 KOŃCOWO):** jeden globalny
licznik `uint8_t diag_capture_id`, inkrementowany RAZ na każde zadziałanie triggera
(dwubitowy skok, zbyt krótki odstęp, utrata latcha — dokładnie dzisiejsze warunki w
`pas_trace.c`, BEZ nowego, drugiego kryterium), i zapisywany RÓWNOCZEŚNIE do zamrożonego
slotu dekodera ORAZ do zamrożonego slotu surowego (sekcja 8) — oba pochodzą z TEGO
SAMEGO fizycznego zdarzenia, więc dostają TEN SAM klucz. 8 bitów wystarcza z dużym
zapasem: przy `N_DECODER_SLOTS=2` i `N_RAW_SLOTS=1` w jednej jeździe da się i tak
zapamiętać co najwyżej 2 przechwycenia na raz — szerokość pola wybrana dla wyrównania
bajtowego, nie z potrzeby zakresu.

**Zaktualizowany layout `0x00010215` (status, dekoder) — wykorzystuje dotychczasowy
wolny bajt Data1:**

| Bajt | Pole | Zmiana |
|---|---|---|
| 0 | `ready` | bez zmian |
| 1 | `capture_id` | **NOWE** (był 0/rezerwa) — który `capture_id` jest teraz wystawiony do strumieniowania |
| 2-3 | `count` | bez zmian |
| 4-5 | `trigger_index` | bez zmian |
| 6-7 | `stream_next` | bez zmian |

**Zaktualizowany layout `0x00010216` (próbka, część 1/2) — wykorzystuje dotychczasowy
wolny bajt Data7:**

| Bajt | Pole | Zmiana |
|---|---|---|
| 0-1 | `index` | bez zmian (0..255 mieściłoby się w 1 B, ale pole zostaje 2 B — nietykanie działającego pola) |
| 2-3 | `gap_ticks` | bez zmian |
| 4 | `from_to` | bez zmian |
| 5 | `flags` | bez zmian |
| 6 | `disc_pos` | bez zmian |
| 7 | `capture_id` | **NOWE** (był 0/rezerwa) |

**Zaktualizowany layout `0x00010217` (próbka, część 2/2) — `torque_fast` USUNIĘTE:**

| Bajt | Pole | Zmiana |
|---|---|---|
| 0-1 | `load_centikg` | bez zmian |
| 2-3 | `torque_raw_mv` | bez zmian |
| 4 | `index` | **NOWE** — 1 B wystarcza (`PAS_TRACE_LEN=256`, zakres 0..255 bez straty) |
| 5 | `capture_id` | **NOWE** — ten sam klucz co w 0x10215/0x10216 |
| 6-7 | `iq_setpoint` | bez zmian |

**Poprawka uzasadnienia (przegląd v3, punkt o `torque_fast`):** `torque_fast` NIE jest
"dostępne na żywo w 0x10219" — pod v4 (sekcja 3) żadna ramka nie jest żywa podczas
jazdy, więc to zdanie z v3 było błędne. Poprawne uzasadnienie: `torque_fast` jest
świadomie pominięte jako MNIEJ ISTOTNE dla tej konkretnej diagnozy (kierunek/pozycja/
nacisk/Iq mówią więcej o tym, czy krok był prawdziwy niż wartość filtra 35 ms w tym
samym momencie) — usunięte wyłącznie po to, by zwolnić 2 bajty na `index`+`capture_id`.
Pole zostaje w strukturze `pas_trace_sample_t` (inne miejsca mogą go nadal używać) —
zmienia się TYLKO to, co main.c pakuje do tej konkretnej ramki CAN.

**Pliki:** `src/pas_trace.c` + `inc/pas_trace.h` potrzebują nowego, świadomego
API dla wielu slotów (np. `pas_trace_slot_ready(slot)`, `pas_trace_slot_get(slot, idx, &out)`,
`pas_trace_slot_capture_id(slot)`) — dzisiejsze API (`pas_trace_ready()`,
`pas_trace_count()`, `pas_trace_get()`) zakłada jeden bufor. To REALNA zmiana w tych
plikach, nie tylko w `main.c` — dodane do listy plików niżej.

### 8. Surowy tor ISR — pełna specyfikacja

**Luka z przeglądu v3:** poprzednia wersja miała tylko odwołanie "Zmiana 2 z v2" bez
formatu zdarzenia, budżetu, mechanizmu trigger/PRE/POST, tego co uruchamia
przechwycenie, identyfikatorów CAN zrzutu ani sposobu korelacji z dzisiejszym śladem
dekodera. Poniżej wszystkie sześć punktów po kolei.

**Dlaczego w ogóle osobny tor.** Dekodowanie kwadratury PAS (`main.c:1868`,
`uint8_t s = ((GPIO_ISTAT(GPIOC)&GPIO_PIN_12)?1:0) | ((GPIO_ISTAT(GPIOD)&GPIO_PIN_2)?2:0);`)
żyje dziś w `reg_ADC_processing()`, wołanej z pętli głównej — DOKŁADNIE tam, gdzie
opóźnienie pętli (blokujące wysyłki CAN, sekcja 3) mogło je czytać spóźnione względem
rzeczywistego stanu linii. `TIMER1_IRQHandler()` (`main.c:1553-1567`) dziś robi
wyłącznie `control_time_ticks++; reg_ADC_flag=1;` — nic więcej, więc jest to miejsce
wolne od jakiegokolwiek opóźnienia pętli głównej.

**1. Format zdarzenia `{tick, state, sequence}`:**

```c
typedef struct {
    uint32_t tick;        /* control_time_ticks w ISR w chwili tej tranzycji */
    uint16_t seq;         /* wolnobieżny licznik zdarzeń ISR, zawija się przy 65536,
                              NIE zerowany per przechwycenie — świadectwo, że nic nie
                              przeskoczyło między dwoma kolejnymi zdarzeniami */
    uint8_t  from_to;     /* (poprzedni_stan<<4)|nowy_stan, stany 0..3 — ta sama
                              konwencja co pas_trace_sample_t.from_to */
    uint8_t  capture_id;  /* 0xFF w wolnobieżnym pierścieniu (nieprzypisane do
                              żadnego zamrożenia); realna wartość TYLKO w kopii
                              zamrożonej do slotu (patrz punkt 3) */
} pas_raw_event_t;  /* 8 B: 4+2+1+1, bez dopełnienia przy wyrównaniu do 4 B */

_Static_assert(sizeof(pas_raw_event_t) == 8,
    "pas_raw_event_t musi zostac 8 B - jedna probka = jedna ramka CAN 0x1021C");
```

**2. Co uruchamia zapis do wolnobieżnego pierścienia (NIE to samo co trigger
zamrożenia — patrz punkt 3):** `TIMER1_IRQHandler()`, na KAŻDYM tiku 4 kHz, czyta
`s=(GPIOC12?1:0)|(GPIOD2?2:0)` i porównuje z lokalnym (ISR-owym, niezależnym od
`pas_qstate` z pętli głównej) `last_raw_state_isr`. Gdy `s != last_raw_state_isr`:
zapisuje `{tick=control_time_ticks, seq=raw_event_seq++, from_to=(last_raw_state_isr<<4)|s, capture_id=0xFF}`
do pierścienia (nadpisując najstarszy wpis), potem `last_raw_state_isr=s`. To
NIEZALEŻNY, zawsze-aktywny "czarny rejestrator" — działa od `ride_episode_init()`
(startu sesji, sekcja 2) bez osobnego uzbrajania, dokładnie jak `control_time_ticks`
sam w sobie.

**Poprawka właściciela (v5) — nadpisywanie w pierścieniu to NORMALNA PRACA, nie
przepełnienie.** v4 mówiło o jednym liczniku `raw_ring_overflow`, co zlewało dwie
zupełnie różne rzeczy w jedną liczbę. Rozdzielone:

| Licznik | Znaczenie | Oczekiwana wartość |
|---|---|---|
| `raw_ring_wraps` | Wolnobieżny pierścień zatoczył pełne koło i zaczął nadpisywać najstarsze wpisy | **Rośnie normalnie** — informacyjny. Przy 256 wpisach i typowej kadencji to zwykłe zachowanie bufora PRE, nie usterka. |
| `raw_capture_overrun` | Podczas KOPIOWANIA PRE do zamrożonego slotu ISR zdążył nadpisać wpis, który miał być skopiowany — REALNA UTRATA danych w snapshocie | **MUSI zostać 0.** Niezerowa wartość unieważnia dane przechwycenia i jest kryterium FAIL w sekcji 10. |

**3. Mechanizm PRE/trigger/POST i co uruchamia PRZECHWYCENIE (zamrożenie):**
przechwycenie NIE ma własnego, drugiego kryterium — używa DOKŁADNIE tego samego
triggera co dzisiejszy `pas_trace.c` (dwubitowy skok `PAS_TR_TWO_BIT`, zbyt krótki
odstęp `PAS_TR_SHORT_GAP`, utrata latcha `PAS_TR_LATCH_LOSS`), oceniany w tym samym
miejscu co dziś (`pas_trace_transition()`/`pas_trace_latch_loss()`). Gdy ten trigger
zadziała: (a) dekoder zamraża swój bufor jak dziś (sekcja 7), z nowym `capture_id`;
(b) TEN SAM `capture_id` trafia do nowej funkcji `pas_raw_freeze(capture_id)` — kopiuje
`PAS_RAW_FREEZE_PRE=128` najświeższych zdarzeń już obecnych w pierścieniu do
dedykowanego slotu zamrożonego, stemplując każdą kopię tym `capture_id`; ISR dalej
dopisuje do TEGO SAMEGO slotu, aż osiągnie `PAS_RAW_FREEZE_POST=128` kolejnych zdarzeń
(razem 256 — ten sam kształt PRE/POST co `pas_trace.c`, więc oba przechwycenia są
koncepcyjnie porównywalne indeks-po-indeksie, mimo że tor surowy biegnie kilkadziesiąt
µs-kilka ms PRZED torem dekodera).

**Poprawka właściciela (v5) — ATOMOWY SNAPSHOT indeksu przed kopiowaniem PRE.**
`pas_raw_freeze()` biegnie w pętli głównej (jest wołane z toru dekodera), a ISR w tym
czasie DALEJ dopisuje do pierścienia — bez zabezpieczenia kopiowane PRE może
zostać nadpisane w trakcie kopiowania i nikt się o tym nie dowie. Wymagana sekwencja:

1. **Atomowo** (`__disable_irq()` / `__enable_irq()`, kilka instrukcji, nie całe
   kopiowanie) odczytać `write_index` ORAZ `raw_event_seq` do zmiennych lokalnych. Oba
   naraz, w jednym zablokowanym oknie — inaczej mogą pochodzić z dwóch różnych chwil.
2. Kopiować 128 wpisów PRE POZA sekcją krytyczną (ISR pracuje normalnie — nie wolno
   blokować przerwań na czas kopiowania 128 rekordów; to byłby ten sam grzech co
   blokujące wysyłki CAN, tylko w innym miejscu).
3. Po skopiowaniu **ponownie atomowo** odczytać `raw_event_seq`. Jeśli
   `seq_po - seq_przed >= PAS_RAW_RING_LEN - PAS_RAW_FREEZE_PRE` (czyli ISR zdążył
   dopisać tyle, że mógł sięgnąć w kopiowany obszar): `raw_capture_overrun++`, a
   przechwycenie jest oznaczone jako podejrzane (bit w `tx_error_summary`, sekcja 9).
   Wykrycie po fakcie zamiast blokady — dane są wtedy jawnie unieważnione, nie cicho
   przekłamane.

**Poprawka właściciela (v5) — generator `capture_id` POMIJA `0xFF`.** `0xFF` jest
sentinelem "wpis w wolnobieżnym pierścieniu, nieprzypisany do żadnego zamrożenia"
(punkt 1). Gdyby generator kiedyś wydał `0xFF` jako prawdziwy identyfikator,
zamrożone wpisy stałyby się nieodróżnialne od niezamrożonych. Reguła:
`do { diag_capture_id++; } while (diag_capture_id == 0xFF);` — zakres roboczy 0x00-0xFE
(255 wartości, w zupełności wystarczy przy 1-2 aktywnych przechwyceniach naraz).

**4. Budżet RAM, dokładny:** pierścień wolnobieżny `PAS_RAW_RING_LEN=256` × 8 B =
2048 B. Jeden slot zamrożony (**`N_RAW_SLOTS=1`**, świadomie mniej niż dekodera —
uzasadnienie w sekcji 7) × 256 × 8 B = 2048 B. **Razem 4096 B**,
`_Static_assert(sizeof(pas_raw_ring)+sizeof(pas_raw_slot) <= 4096, "FW-106 raw PAS diagnostic RAM budget exceeded")`.
Zachowanie po zapełnieniu jedynego slotu: slot zamraża się TYLKO przy PIERWSZYM
triggerze od ostatniego zrzutu; każdy kolejny trigger w tej samej jeździe — niezależnie
od tego, czy akurat jest wolny slot dekodera (sekcja 7) — tylko inkrementuje licznik
`raw_freeze_skipped`, bez próby dopasowania "najbardziej przydatnego" przechwycenia
dekodera. Prosta, deterministyczna, łatwa do przetestowania reguła zamiast
skomplikowanej heurystyki.

**5. Nowe identyfikatory CAN zrzutu:**

`0x0001021B` — status surowego przechwycenia (odpowiednik 0x10215 dla dekodera):

| Bajt | Pole |
|---|---|
| 0 | `ready` (1 = zamrożony slot czeka na wysłanie) |
| 1 | `capture_id` (ten sam klucz co w 0x10215/16/17 dla tego samego zdarzenia) |
| 2-3 | `count` (liczba zdarzeń w tym przechwyceniu, ≤256) |
| 4-5 | `trigger_index` (które zdarzenie było najbliższe chwili triggera) |
| 6-7 | `stream_next` (jak daleko zaszło drenowanie) |

`0x0001021C` — próbka surowa, JEDNA ramka = JEDNO zdarzenie (struktura ma dokładnie
8 B, więc — inaczej niż 0x10216/0x10217 — nie trzeba dzielić na dwie ramki):

| Bajt | Pole |
|---|---|
| 0-3 | `tick` (u32, big-endian) |
| 4-5 | `seq` (u16, big-endian) |
| 6 | `from_to` |
| 7 | `capture_id` |

**6. Korelacja surowego ISR z istniejącym śladem dekodera:** WYŁĄCZNIE przez wspólny
`capture_id` (sekcja 7) — nie przez kolejność, nie przez czas odbioru na magistrali,
nie przez zgadywanie. Ponieważ oba przechwycenia (surowe i dekodera) uzbrajają się z
TEGO SAMEGO wywołania triggera dla TEGO SAMEGO fizycznego zdarzenia, analiza offline
łączy je trywialnie: znajdź wiersze `0x1021C` i `0x10216`/`0x10217` o tym samym
`capture_id`, porównaj `tick` (surowy, bezwzględny) z `index`/`gap_ticks` (dekoder,
względny wobec startu przechwycenia dekodera) — rozjazd między nimi JEST odpowiedzią na
pytanie, które zapoczątkowało tę kartę: czy dekoder widział to, co naprawdę było na
liniach, czy coś zgubił/przesunął w czasie.

### 9. Telemetria sesji: prawdziwe delty + nagłówek `0x1021D` i trailer `0x1021E`

**Poprawka właściciela:** dziś `missed_control_ticks` i pokrewne liczą się od `while(1)`
(od startu kontrolera), nie per-sesja. Jeśli zrzut po zatrzymaniu (nawet poprawiony w
sekcji 3) sam zużywa jakiś czas pętli głównej, zanieczyszcza to wynik "ile stracono
PODCZAS JAZDY" czasem straconym podczas POSTOJU.

**Poprawka właściciela (v6) — sam snapshot na końcu NIE czyni metryki per-session.**
v5 zamrażało wartości w chwili końca sesji, ale te wartości kumulują się od bootu, więc
druga i każda kolejna sesja raportowałaby sumę wszystkich poprzednich. Potrzebne są
kotwice startowe i osobne śledzenie maksimum:

| Zmienna | Kiedy ustawiana | Jak używana |
|---|---|---|
| `session_start_missed_ticks` (u32) | przy STARCIE sesji (sekcja 2) = bieżąca wartość `missed_control_ticks` | publikowana wartość = `missed_control_ticks - session_start_missed_ticks` |
| `session_start_missed_events` (u32) | j.w., z `missed_control_events` | publikowana = różnica, j.w. |
| `session_worst_burst` (u16) | **zerowana** przy starcie sesji | aktualizowana `max()` przy KAŻDYM zdarzeniu straty w trakcie sesji — **maksimum globalne jest nieodejmowalne**, więc musi być śledzone niezależnie, obok produkcyjnego `worst_missed_burst_ticks` (którego nie ruszamy) |

Wszystkie trzy przechodzą przez mechanizm KANDYDATA z sekcji 2: przy początku ciszy
zapisywana jest kopia **już policzonych delt** (nie wartości surowych), zatwierdzana po
3 s ciszy albo odrzucana przy ruchu. Dzięki temu publikowane liczby opisują wyłącznie
okres jazdy — bez postoju, bez zrzutu, bez poprzednich sesji.

**Layout `0x10218` pozostaje BAJT W BAJT bez zmian** — zmieniają się wyłącznie wartości
(delty sesji zamiast sum od bootu) w wersji FINALNEJ tej ramki. Wersja live raz/s
(opcjonalna, sekcja 3) nadal niesie wartości wolnobieżne, jak dziś.

**Poprawka właściciela (v5) — `0x10218` nie ma miejsca na dane sesji.** Zweryfikowane w
kodzie (`main.c:3459-3467`): wszystkie 8 bajtów zajęte (`missed_control_ticks` u32 +
`missed_control_events` u16 + `worst_missed_burst_ticks` u16), zero rezerwy. Klucz sesji
wymaga WŁASNEJ ramki — stąd `0x1021D` poniżej, wysyłany BEZPOŚREDNIO PRZED finalnym
`0x10218`, żeby parser wiedział, do której sesji odnosi się następna ramka metryk.

#### Struktura zrzutu jednej sesji (kolejność obowiązująca)

```
0x1021D   nagłówek sesji     — tylko dane ZNANE na koniec jazdy
0x10218   metryki finalne    — delty tej sesji (bez klucza; klucz z nagłówka wyżej)
   ... rekordy sesji: grupy 0x1021A+0x10210..14, próbki 0x10215/16/17, surowe 0x1021B/1C ...
0x1021E   trailer sesji      — wyniki TRANSMISJI, znane dopiero teraz
```

**Poprawka właściciela (v6) — `0x1021D` nie może zawierać wyników przyszłego zrzutu.**
v5 wpisywało do niego `discards_this_session` i `tx_error_summary`, które w chwili
zakończenia jazdy jeszcze NIE ISTNIEJĄ — zrzut się nawet nie zaczął. Wybrany wariant
(a) z dwóch zaproponowanych: nagłówek niesie wyłącznie fakty z jazdy, a wyniki
transmisji trafiają do osobnej ramki końcowej.

**`0x0001021D` — NAGŁÓWEK sesji (raz na zakończoną sesję, jako pierwsza ramka zrzutu):**

| Bajt | Pole | Znaczenie |
|---|---|---|
| 0 | `schema_version` | Ta sama wersja układu co w 0x1021A (to wdrożenie = `1`) |
| 1 | `session_id` | Klucz sesji — ten sam, którym opieczętowane są rekordy z tej sesji (sekcja 2) |
| 2-3 | `session_duration_ds` | Czas trwania sesji w dziesiątych sekundy = `(session_end_tick - session_start_tick)` przeliczone z 4 kHz, nasyca się na 0xFFFF (≈109 min). Różnica ticków jest odporna na 32-bitowe zawinięcie (odejmowanie unsigned) |
| 4-5 | `episodes_this_session` | Ile epizodów ta sesja wyprodukowała (przyrost `enqueued_total` w jej trakcie), nasyca się na 0xFFFF |
| 6-7 | `records_to_send` | Ile rekordów ŁĄCZNIE (wszystkie trzy kolejki) czeka na zrzut dla tej sesji. Parser porównuje to z tym, co faktycznie odebrał — a niezgodność jest miarą utraty w loggerze, niezależną od firmware |

**`0x0001021E` — TRAILER sesji (raz na zakończoną sesję, jako ostatnia ramka zrzutu):**

| Bajt | Pole | Znaczenie |
|---|---|---|
| 0 | `schema_version` | j.w. |
| 1 | `session_id` | Ten sam co w nagłówku — domyka klamrę; odebranie trailera = zrzut tej sesji zakończony |
| 2-3 | `records_sent` | Ile rekordów tej sesji wysłano w CAŁOŚCI (przyrost `sent_total`, sekcja 2) |
| 4 | `records_discarded` | Ile PRZYJĘTYCH rekordów porzucono (przyrost `discarded_total`), nasyca się na 255 |
| 5 | `records_rejected` | Ile rekordów odrzucono przy pełnej kolejce (przyrost `rejected_total`, sekcja 2), nasyca się na 255 |
| 6 | `tx_error_summary` | bit0: `diag_tx_failed>0` · bit1: `DIAG_TX_MAX_RETRY` przekroczone choć raz · bit2: `capture_slots_full>0` · bit3: kolejka epizodów odrzuciła rekord · bit4: `raw_capture_overrun>0` (sekcja 8) · bit5: `tx_header_failed>0` · bit6: zrzut tej sesji był PRZERYWANY ruchem koła co najmniej raz · bit7 rezerwa |
| 7 | `dump_complete` | `1` = wszystkie rekordy sesji obsłużone (wysłane lub jawnie porzucone); `0` = zrzut kończy się z rekordami wciąż w kolejce (możliwe tylko przy wyparciu przez nowszą sesję — patrz kolejka podsumowań) |

Trailer z definicji nie może raportować własnego niepowodzenia transmisji — jego brak w
logu JEST tą informacją i parser tak go traktuje.

#### Kolejka podsumowań sesji (poprawka właściciela, v6)

Bez niej zatrzymanie sesji B nadpisałoby zamrożone metryki sesji A, której zrzut został
przerwany ruchem koła — i dane sesji A przepadłyby bez śladu, mimo że jej rekordy wciąż
czekają w kolejkach. Dlatego podsumowania sesji (komplet: `session_id`, delty metryk,
`session_duration_ds`, kotwice liczników) trafiają do **własnego bufora pierścieniowego
na `N_SESSION_SUMMARIES=4` wpisy** (4 × ~32 B = 128 B — mieści się w pozycji
"Liczniki/agregaty" budżetu z sekcji 1).

- Zrzut obsługuje podsumowania w kolejności FIFO: najpierw domyka sesję A (nagłówek,
  metryki, jej rekordy, trailer), dopiero potem przechodzi do B.
- Przy pełnym buforze podsumowań obowiązuje ta sama polityka co w sekcji 2:
  **odrzucenie NOWEGO** podsumowania (`session_summary_rejected++`), nigdy wyparcie
  starszego — dane najbliższe badanemu zdarzeniu są cenniejsze.
- Cztery wpisy wystarczają z zapasem: żeby je zapełnić, trzeba czterech kolejnych cykli
  jazda-postój, z których każdy kończy się przerwanym zrzutem. Gdyby test pokazał, że to
  za mało, liczba jest jedną stałą do podniesienia, kosztem 32 B za wpis.

#### `diag_header_lost` jest NIEPOLICZALNY przez firmware (poprawka v5, zachowana)

Jeśli nagłówek przepadnie po stronie loggera (redukcja, przepełnienie bufora USB,
zgubiona ramka na odbiorze), sterownik NIE MA JAK się o tym dowiedzieć — widzi wyłącznie
stan swojego kontrolera CAN. Zostaje więc:

- **`tx_header_failed`** (firmware, realne): nagłówek, który po `DIAG_TX_MAX_RETRY` prób
  nadal nie dostał `CAN_TRANSMIT_OK`. To jedyna utrata nagłówka, którą sterownik
  naprawdę stwierdza. Raportowany jako bit5 w `tx_error_summary` trailera.
- **Utrata w loggerze**: wykrywa ją PARSER, nie firmware — po dziurze w ciągłości
  `episode_id` przy danym `session_id`, a od v6 dodatkowo po niezgodności
  `records_to_send` (nagłówek) z liczbą faktycznie odebranych rekordów i z `records_sent`
  (trailer). Te trzy liczby razem pozwalają odróżnić „firmware nie wysłał" od „logger nie
  zapisał" — czego v5 nie potrafiło.

### 10. Test przed jazdą (kryterium wejścia na rower)

Nowy build diagnostyczny musi przez min. 30 s spełnić: pominięte okresy sterowania
<1%, najgorsza luka <1 ms, brak przepełnienia żadnego z buforów (sekcje 6-8), komplet
indeksów/`capture_id` próbek. **Nowe w v4:** przed jazdą sprawdzić PRAWDZIWĄ mapę
konsolidacji (`.map`, sekcje `.bss`+`.data`) zbudowanego firmware — budżet z sekcji 1
(~10,2 KB) musi zmieścić się z bezpiecznym zapasem w rzeczywistym wolnym RAM, nie tylko
w założeniu.

Test hosta na prawdziwym `ride_episode.c` (nie reimplementacja): sekwencja R-F-R-F,
kolejne cofnięcie PO zazbrojonym latchu, cofnięcie przy `Iq==0`. Te trzy scenariusze
rozszerzą `tests/host/fw101_episode_host.c` (ten sam wzorzec co istniejące testy
FW-101/104 tamże) — dokładnie sprawdzą, że sekcja 5 nie psuje żadnego z 16 testów już
tam obecnych.

**Nowe scenariusze hosta dla `pas_trace.c` (sekcja 7), rozszerzają
`tests/host/fw102_pas_trace_host.c`:** dwa triggery w jednej "jeździe" zanim cokolwiek
się zdrenuje (oba `capture_id` muszą się zachować, drugi slot musi się zapełnić, nie
nadpisać pierwszego); trzeci trigger przy obu slotach zajętych (musi tylko zliczyć
`capture_slots_full`, nie uszkodzić żadnego zamrożonego slotu); zgodność `capture_id`
między zamrożonym slotem dekodera a zamrożonym slotem surowym dla tego samego
wywołania triggera.

**Nowy scenariusz hosta dla cyklu sesji (sekcja 2):** dwie sesje w jednym uruchomieniu
(stop→jazda→stop→jazda→stop) z zamierzonym przerwaniem zrzutu między nimi (symulowany
ruch koła w trakcie drenowania) — sprawdza, że `session_id` inkrementuje dokładnie raz
na cykl i że porzucony rekord nie pojawia się później sklejony z inną sesją.

**Nowe scenariusze z przeglądu v5 (cztery, wskazane wprost przez właściciela):**

| Scenariusz | Co wymusza | Kryterium PASS |
|---|---|---|
| **mailbox busy → pending → OK** (sekcja 3) | Atrapa CAN zwraca `CAN_NOMAILBOX` przez N wywołań, potem przyjmuje i przez M wywołań raportuje `CAN_TRANSMIT_PENDING`, wreszcie `CAN_TRANSMIT_OK` | Kursor fragmentu NIE przesuwa się ani przy `NOMAILBOX`, ani przy `PENDING`; przesuwa się DOKŁADNIE RAZ, przy `OK`. Żaden fragment nie jest wysłany dwa razy ani pominięty. `sent_total` rośnie dopiero po OSTATNIM fragmencie rekordu. |
| **mailbox failed → retry** (sekcja 3) | Atrapa zwraca `CAN_TRANSMIT_FAILED` k razy dla tego samego fragmentu, potem `OK` | Ten sam fragment jest ponawiany, `diag_tx_failed` rośnie o k, kursor przesuwa się dopiero po `OK`. Przy k > `DIAG_TX_MAX_RETRY`: cały rekord porzucony, `discarded_total++`, zrzut przechodzi do NASTĘPNEGO rekordu (nie zacina się). |
| **anulowanie 3-sekundowego kandydata** (sekcja 2) | Cisza trwa 2,5 s, potem impuls koła, potem znów cisza pełne 3 s | Pierwszy `candidate_end_tick` ODRZUCONY; sesja nie kończy się w połowie; ostatecznie zatwierdzony jest kandydat z DRUGIEJ ciszy, a zamrożone metryki NIE zawierają 3 s postoju (porównanie z licznikiem odniesienia sprzed ciszy). |
| **rachunek kolejki po porzuceniu** (sekcja 2) | Sesja: 5 rekordów wstawionych, 3 wysłane w całości, 1 porzucony przez ruch koła, 1 porzucony przez `DIAG_TX_MAX_RETRY` | `enqueued_total=5`, `sent_total=3`, `discarded_total=2`, a `pending = 5-3-2 = 0`. Stary wzór v4 (`5-3=2`) dałby tu fałszywe "dwa rekordy wciąż czekają" — to jest dokładnie ten regres, który ten test blokuje. |

**Nowe scenariusze z przeglądu v6 (cztery):**

| Scenariusz | Co wymusza | Kryterium PASS |
|---|---|---|
| **delty per-session, dwie sesje z rzędu** (sekcja 9) | Sesja A gubi 100 ticków / 10 zdarzeń, najgorsza seria 40; sesja B gubi 50 / 5, najgorsza seria 12 | Finalne `0x10218` sesji B pokazuje **50 / 5 / 12**, NIE 150 / 15 / 40. Test wprost blokuje regres v5 (sumy od bootu). Kluczowe: `session_worst_burst` sesji B = 12, mimo że globalne maksimum wynosi 40 — dowód, że maksimum jest śledzone osobno, a nie odejmowane. |
| **pełna kolejka: odrzucenie nowego** (sekcja 2) | Kolejka 64 wpisów zapełniona, wpływa 65. rekord | 65. rekord ODRZUCONY (`rejected_total=1`), `enqueued_total` NIE rośnie, `discarded_total` NIE rośnie, zawartość kolejki (64 najstarsze) nietknięta, `pending` nadal = 64. Niezmiennik "każdy przyjęty rekord kończy dokładnie jednym zdarzeniem" zachowany. |
| **kolejka podsumowań: sesja B nie kasuje sesji A** (sekcja 9) | Sesja A kończy się, zrzut przerwany po 2 z 5 rekordów; jazda; sesja B kończy się | Podsumowanie A NADAL w buforze z własnymi deltami; zrzut wznawia od A (FIFO), domyka ją trailerem, dopiero potem robi B. Metryki A nie są nadpisane przez B. |
| **nagłówek/trailer domykają klamrę** (sekcja 9) | Pełny zrzut jednej sesji z 3 rekordami | Kolejność ramek dokładnie: `0x1021D` → finalne `0x10218` → 3 grupy rekordów → `0x1021E`. `records_to_send` z nagłówka = liczba faktycznie wysłanych grup = `records_sent` z trailera. `dump_complete=1`. |

Dodatkowo do kryteriów wejścia na rower dochodzi: **`raw_capture_overrun == 0`** po
całym teście (sekcja 8 — `raw_ring_wraps` może być dowolnie duże, to normalna praca).

### 11. FW-107: znacznik fast-rearm w 0x10212, `schema_version` 1 → 2

Karta FW-107 (nieregularne wznowienie wspomagania po krótkim cofnięciu korby) dodaje do
`ride_episode_result_t` jedno pole `fast_rearm` (1 = to uzbrojenie epizodu poszło ścieżką
szybkiego rearmu po pojedynczym reverse, a nie normalnym progiem obciążenia). Zamiast
nowej ramki, wykorzystano jeden z 3 udokumentowanych w sekcji 6 wolnych bajtów rezerwy
ramki `0x10212`:

| Bajt | Pole | Znaczenie |
|---|---|---|
| 5 | `fast_rearm` | Do `schema_version=1`: zawsze `0`, zarezerwowany, nieużywany. Od `schema_version=2`: `1` = to uzbrojenie poszło ścieżką fast-rearm, `0` = normalny latch progiem obciążenia (lub brak uzbrojenia). |

**Dlaczego to podbija `schema_version`, mimo że stary parser nie widzi różnicy:** reguła w
kodzie (`src/main.c`, definicja `DIAG_SCHEMA_VERSION`) brzmi "podbij przy KAŻDEJ zmianie
layoutu ramki" — a to jest zmiana layoutu: bajt, który wcześniej nie niósł żadnej
informacji, teraz ją niesie. Nowy parser, który chce odczytać `fast_rearm`, musi umieć
odróżnić log ze `schema_version=1` (gdzie `data[5]=0` nic nie znaczy) od logu ze
`schema_version=2` (gdzie `data[5]=0` oznacza realnie "nie fast-rearm"). Parser, który
IGNORUJE bajty rezerwy, działa bez zmian na obu wersjach — to jest zamierzona,
wsteczna kompatybilność, nie powód do POMINIĘCIA bumpa.

**Pozostała rezerwa w 0x10212 po tej karcie:** `data[6]` i `data[7]`, nadal wolne.

**Poza zakresem tej karty:** `Backwards_counter`/`BACKWARDS_CONFIRM_STEPS` (twardy cut po
potwierdzonym reverse) — nietknięte. `start_steps`, progi nacisku — nietknięte. Pełny opis
mechanizmu fast-rearm (automat `rearm_state`, warunek "świeżego nacisku", sesja
wznowieniowa dla `assist_modes_calculate`) jest w `src/ride_control.c`, nie w tym
dokumencie — ten dokument opisuje wyłącznie zmianę w RAMCE diagnostycznej.

## Kryteria rozstrzygnięcia z takiej jazdy (zachowane bez zmian)

- **Czyste krawędzie reverse w surowym A/B** → realne kołysanie korby → wdrożyć szybki
  rearm (plan rearm_after_reverse, OSOBNA przyszła karta).
- **Bounce widoczny w surowym A/B** → problem sprzętowy: czujnik, przewody, zakłócenia
  elektryczne — INNY kierunek naprawy niż zmiana logiki firmware.
- **Brak błędów bez ciężkiej diagnostyki (dzisiejszy build)** → problem był
  WPROWADZANY przez sam build diagnostyczny (sekcje 1/3 wyżej są głównym podejrzanym,
  zweryfikowanym w kodzie) — wskazywałoby na dokończenie F1 (Commit B) jako priorytet
  przed czymkolwiek innym w PAS. Odpowiedź liczbową daje porównanie OFFLINE (sekcja 4):
  `gap_ticks` dekodera z `0x10216` przeciwko różnicom `tick` z `0x1021C` dla tego samego
  `capture_id`.

## Poza zakresem tej karty (świadomie, potwierdzone w v3/v4/v6)

`start_steps`, progi nacisku, `Backwards_counter`, `rearm_after_reverse` — nietknięte.
**Produkcyjne `pas_idle_ticks`/`pas_cycle_ticks` — nietknięte** (sekcja 4); v6 nie
dodaje też ich diagnostycznych odpowiedników — rzeczywisty czas liczy się offline z
`0x1021C.tick`. Realny dekoder PAS w
`reg_ADC_processing()` (main loop) — surowy tor ISR (sekcja 8) go NIE zastępuje, tylko
dostawia niezależny tor odniesienia obok niego. Dokładna wielkość WOLNEGO RAM
kontrolera (w odróżnieniu od 48 KB całkowitych) — NIE zmierzona w tej karcie, przeniesiona
do obowiązkowego sprawdzenia mapy konsolidacji w sekcji 10.

## Pliki, których dotknie wdrożenie

`src/main.c` (`TIMER1_IRQHandler` — nowy pierścień surowych zdarzeń, sekcja 8;
`reg_ADC_processing` — produkcyjne liczniki PAS nietknięte, bez dodatków (sekcja 4);
`print_debug_on_CAN` — przebudowa na nieblokujący automat TX (sekcja 3) i
zrzut po zatrzymaniu, nowe ramki 0x1021A/0x1021B/0x1021C/0x1021D/0x1021E, zmienione
layouty 0x10215/0x10216/0x10217, delty sesji w finalnym 0x10218 (sekcja 9),
agregaty/kolejki/sloty zamiast pełnych ramek),
`src/ride_episode.c` + `inc/ride_episode.h` (kolejka epizodów z `session_id`, poprawiony
reset przy kolejnym reverse — bez nowych pól czasowych, sekcja 5-6), `src/pas_trace.c` +
`inc/pas_trace.h` (NOWE API wieloslotowe, `capture_id`, sekcja 7 — realna zmiana, nie
tylko w `main.c`), `tests/host/fw101_episode_host.c` (3 nowe scenariusze),
`tests/host/fw102_pas_trace_host.c` (3 nowe scenariusze wieloslotowe), NOWY host-test
dla automatu TX, telemetrii sesji i cyklu życia sesji z atrapą CAN (8 scenariuszy z obu
tabel w sekcji 10 — atrapa `can_message_transmit`/`can_transmit_states`, żeby
przetestować `NOMAILBOX`/`PENDING`/`OK`/`FAILED`, delty per-session, politykę pełnej
kolejki i klamrę nagłówek-trailer bez sprzętu). Zero zmian w `inc/config.h` progów jazdy,
`torque_input.c`, `assist_modes.c`, `ride_control.c`, oraz w samym sterowniku
`gd32f30x_can.c` (używamy jego API poprawnie, nie zmieniamy go).

## Co dalej

v6 gotowe do przeglądu — cztery problemy telemetrii sesji z v5 zamknięte: prawdziwe
delty per-session (kotwice startowe + osobno śledzone maksimum serii), rozdzielenie
nagłówka `0x1021D` (fakty z jazdy) od trailera `0x1021E` (wyniki transmisji), kolejka
podsumowań sesji chroniąca dane sesji z przerwanym zrzutem, jednoznaczna częstotliwość
ramek sesyjnych, usunięcie liczników `_diag` bez pola CAN na rzecz analizy offline z
`0x1021C.tick`, oraz jednoznaczna polityka pełnej kolejki (odrzucenie nowego +
`rejected_total` rozdzielony od `discarded_total`).

Pełna lista nowych/zmienionych ramek CAN w tej karcie: NOWE `0x1021A` (nagłówek grupy
epizodu), `0x1021B`/`0x1021C` (surowy tor ISR), `0x1021D` (nagłówek sesji), `0x1021E`
(trailer sesji); ZMIENIONE layouty `0x10215`/`0x10216`/`0x10217`; `0x10218` bez zmian
layoutu, ze zmienioną semantyką wersji finalnej; `0x10210`-`0x10214` BEZ ZMIAN.

Karta zaakceptowana i wdrożona 2026-08-10 — szczegóły niżej.

---

## Jak wdrożono (2026-08-10) — odstępstwa i uzupełnienia

Wszystko z v6 zostało zaimplementowane. Cztery rzeczy zrobiono INACZEJ niż dosłownie
zapisano w karcie; każda jest zmianą formy, nie funkcji, i każda ma powód:

**1. Surowy tor ISR to OSOBNY MODUŁ (`src/pas_raw.c` + `inc/pas_raw.h`), nie kod w
`main.c`.** Karta wskazywała `main.c`. Ale sekcja 10 wymaga przetestowania
`raw_capture_overrun == 0` i zgodności `capture_id` między torem surowym a dekoderem —
a niczego w `main.c` nie da się uruchomić testem hosta. To jest dokładnie ta lekcja,
którą nagłówki `ride_episode.h` i `pas_trace.h` już opisują: pierwszy rejestrator
FW-101 napisano w `main.c` i miał trzy wady, których jego własne wyjście nie mogło
pokazać. `TIMER1_IRQHandler` woła jedną funkcję modułu.

**2. Atomowy snapshot indeksu ISR zrobiono SEQLOCKIEM, nie `__disable_irq()`.** Karta
podawała sekcję krytyczną jako przykład. Wymaganie brzmiało: para `(head, seq)` ma
pochodzić z jednej chwili. Seqlock (czytaj `seq`, `head`, `seq` ponownie; powtórz przy
zmianie) daje tę samą gwarancję, nie maskując ani na chwilę przerwań — a to sterownik
silnika, w którym FOC i Halle chodzą z przerwań i build diagnostyczny nie ma prawa ich
opóźniać. Wykrycie nadpisania po fakcie (`raw_capture_overrun`) działa bez zmian.

**3. Zrzut chodzi z PĘTLI GŁÓWNEJ, nie co 40 ms.** Przy jednej potwierdzonej ramce na
40 ms pełne przechwycenie schodziłoby ~50 s postoju — zrzut, który realnie nigdy się nie
kończy, nie jest pomiarem. `diag_dump_step()` wołany jest z `while(1)`, więc tempo
wyznacza sprzęt CAN, a każdy krok to nadal najwyżej jedna operacja na skrzynce, bez
żadnego czekania. Zasada „nic nie leci, gdy rower jedzie" bez zmian.

**Skorygowane w piątej iteracji (patrz niżej).** Zdanie „tempo wyznacza sprzęt CAN"
okazało się nieprawdziwe w praktyce: `while(1)` woła `diag_dump_step()` dużo częściej
niż raz na tik sterowania 4 kHz, więc „najwyżej jedna operacja na skrzynce" wcale nie
ograniczało tempa wysyłki — ograniczało tylko ilość pracy na jedno wywołanie. Realny log
z jazdy (`log-2026-08-11-17-41-08-n0.log`) to potwierdził: sterownik CAN sam potwierdził
wysłanie każdej z 18 rekordów zrzutu, a logger USB zapisał tylko 766 z ~1389 wysłanych
ramek. Zob. sekcję „Poprawka wdrożenia (piąta iteracja)".

**4. Ramki „na żywo" 0x10203-0x1020F i 0x10219 są wysyłane RAZ, w ramach zrzutu.**
Karta mówiła tylko, że nie są buforowane podczas jazdy. Usunięcie ich całkiem
skasowałoby liczniki zbiorcze (`metric_*`, `gap_*`, `pas_rev_*`), na których opierają
się wcześniejsze karty — a sekcja 1 wprost każe agregaty zachować. Wysyłane są więc jako
stan końca jazdy, z NIEZMIENIONYM układem bajtów. Wartości chwilowe (0x10203, 0x10219)
opisują wtedy postój, co jest uczciwe i nadal użyteczne jako kontrola.

**Czego NIE udało się pokryć testem hosta:** automat TX i cykl życia sesji są statyczne
w `main.c`, więc cztery scenariusze z tabeli sekcji 10 (mailbox busy→pending→OK,
failed→retry, anulowanie kandydata 3 s, rachunek kolejki po porzuceniu na poziomie
sesji) nie mają testu automatycznego. Rachunek kolejki jest pokryty na poziomie
`ride_episode` (blok B2 w `fw106_recorder_host.c`); pozostałe trzy wymagają albo
wydzielenia tych funkcji do własnego modułu (kandydat na osobną kartę), albo
weryfikacji na stanowisku. **Zapisane jako znany dług, nie przemilczane.**

**Zmierzony koszt RAM** (`sizeof` na hoście, nie szacunek): kolejka epizodów
64 × 36 B = 2304 B, drugi slot `pas_trace` 3584 B, pierścień+slot surowy 4096 B,
podsumowania sesji 4 × ~32 B = 128 B, liczniki ≤192 B → **≈10,2 KB**, zgodnie z
budżetem z sekcji 1 i poniżej twardego limitu 12 KB. `pas_raw_event_t` ma dokładnie 8 B,
pilnowane `_Static_assert`-em.

**Wada znaleziona przez własny test, naprawiona:** pierwsza wersja `pas_raw.c` uznawała
przechwycenie za gotowe dopiero po zapełnieniu całych 256 pozycji, więc przy krótkiej
historii PRE okno POST rozciągało się zamiast pozostać stałe. Poprawione: długość
przechwycenia ustalana jest w chwili zamrożenia jako `pre_n + POST` — krótka historia
jest raportowana krótko, nigdy dopychana kosztem okna POST (ta sama zasada co w
`pas_trace.c`).

---

## Poprawka wdrożenia (druga iteracja) — osiem błędów zgłoszonych przez właściciela

Pierwsze wdrożenie działało, ale przegląd wykazał osiem realnych wad. Wszystkie naprawione.

**1. Mieszanie sesji.** `pas_trace` i `pas_raw` miały tylko `capture_id`, więc zrzut sesji A
mógł zabrać rekord sesji B. Teraz każdy slot dostaje `session_id` **w chwili uzbrojenia**
(`pas_trace_set_session_id`/`pas_raw_set_session_id`), a zrzut pyta o rekordy WYŁĄCZNIE
przez `count(session_id)`/`frame(session_id, …)` — nie „co jest na czele kolejki".
`records_to_send` liczy tylko rekordy tej sesji. Capture otwarty na końcu jazdy jest
**pieczętowany jako częściowy** (wybrane spójne zachowanie, nie odrzucenie) — zachowuje
to, co widział, z flagą „ucięty", i nie może zbierać POST z następnej jazdy.

**2. Jawny rekord w locie.** `diag_dump_started_record` (bool) zastąpiony przez
`diag_record_kind_t` — `NONE`/`EPISODE`/`TRACE`/`RAW`. Po ruszeniu roweru porzucany jest
dokładnie ten rekord, którego dotyczy stan; przerwanie w ramkach zbiorczych lub w nagłówku
sesji nie usuwa żadnego epizodu. Rekordy nierozpoczęte zostają w kolejce.

**3. Cztery liczniki na źródło.** `enqueued`/`sent`/`discarded`/`rejected` osobno dla
EPISODE, TRACE i RAW, z niezmiennikiem `pending = enqueued - sent - discarded`. Wspólny
`dropped_total` usunięty (`ride_episode_queue_dropped()` → `_removed()`, i nie służy już
do rozliczania). Rekord odrzucony przy pełnej kolejce zwiększa wyłącznie `rejected`.

**4. `records_sent`.** Przyczyną podwójnego liczenia był efekt uboczny wewnątrz
przebiegu ramek zbiorczych, wykonywany ponownie przy każdym wejściu. Builder ramek jest
teraz **czysty** — retire rekordu i inkrementacja liczników przeniesione do źródeł
rekordów, wołanych dokładnie raz. `records_sent` rośnie po `CAN_TRANSMIT_OK` OSTATNIEGO
fragmentu i przeżywa przerwanie (żyje w podsumowaniu sesji, nie jest zerowane w IDLE).

**5. GIVEUP.** Kursor przesuwa się tylko po `CAN_TRANSMIT_OK`. Po przekroczeniu limitu:
rekord porzucany w całości (`discarded++`), fragment 0 epizodu dodatkowo ustawia
`DIAG_ERR_HEADER_FAILED`; ramki zbiorcze i sesyjne przechodzą dalej, a trailer, którego
nie dało się wysłać, i tak zdejmuje podsumowanie z kolejki — nic nie może zakleszczyć
zrzutu. `CAN_NOMAILBOX` i `PENDING` nie przesuwają kursora i nie są utratą.

**6. Trailer 0x1021E.** `records_sent`/`discarded`/`rejected` są per-sesja. Bity błędów to
**delty tej sesji** (kotwice `refused_at_open`/`refused_at_close`), nie globalne flagi.
`dump_complete=1`, gdy żaden rekord tej sesji już nie czeka — wcześniejsze przerwanie ma
własny bit i **nie** wymusza `0`.

**7. Testowalność.** Automat TX, cykl sesji i sekwencer zrzutu wydzielone do
`src/diag_session.c` + `inc/diag_session.h` — bez MS/MP i bez sterownika GD32, sterowane
przez wstrzykiwane tablice operacji. `tests/host/fw106_session_host.c` uruchamia
PRAWDZIWY kod produkcyjny z atrapą CAN.

**8. RAM i integracja.** `_Static_assert` na budżet każdego modułu i na sumę ≤ 12 KB
(`inc/diag_budget.h`). `pas_raw_init()` wołane z `diag_diagnostics_init()`.
`src/diag_session.c` i `src/pas_raw.c` dopisane do `scripts/sources-m820.txt`.
Warianty rozdzielone: `pas_raw.c` i `diag_session.c` są kompilowane WYŁĄCZNIE przy
`CAN_DIAGNOSTICS_ENABLE=1`, a `PAS_TRACE_SLOTS` i `RIDE_EPISODE_QUEUE_LEN` spadają do 1
w produkcji.

**Zmierzony koszt RAM czterech modułów FW-106** (`arm-none-eabi-size`, `data+bss`):

| Wariant | pas_raw | pas_trace | ride_episode | diag_session | razem |
|---|---|---|---|---|---|
| `CAN_DIAGNOSTICS_ENABLE=0` | 0 B | 3625 B | 132 B | 0 B | **3757 B** |
| `CAN_DIAGNOSTICS_ENABLE=1` | 4130 B | 7237 B | 2400 B | 356 B | **14123 B** |

Produkcja płaci tyle co przed kartą (3625 B to istniejący wcześniej slot `pas_trace`).
Przyrost diagnostyczny ≈ 10,4 KB, poniżej pułapu 12 KB.

---

## Poprawka wdrożenia (trzecia iteracja) — cztery pozostałe błędy

Druga implementacja przechodziła wszystkie testy, ale przegląd znalazł cztery realne
wady. Wszystkie naprawione, każda z nowym testem regresyjnym potwierdzonym przez
CELOWE cofnięcie poprawki na kopii roboczej (test musi PAŚĆ bez poprawki, PRZEJŚĆ z
poprawką — sprawdzone dla wszystkich czterech, nie tylko napisane).

**1. Start sesji przed pierwszym triggerem.** Był realny wyścig: `pas_trace_transition()`
w `main.c` był wołany, ZANIM `diag_session_tick()` (dalej w tej samej iteracji pętli)
zdążył podbić `session_id` po przejściu stop→jazda. Pierwszy podejrzany impuls PAS
rozpoczynający jazdę mógł więc zostać opieczętowany poprzednim (lub zerowym)
`session_id` — sierota, której żaden zrzut nigdy by nie znalazł. Naprawione nowym
punktem wejścia `diag_session_note_activity()` (`inc/diag_session.h`,
`src/diag_session.c`): wołany w `main.c` DOKŁADNIE w miejscu wykrycia zmiany stanu
kwadratury, PRZED zbudowaniem `pt_in` i wywołaniem `pas_trace_transition()`. Uruchamia
sesję (jeśli nieaktywna) i NATYCHMIAST propaguje nowy `session_id` do `ride_episode`,
`pas_trace` i `pas_raw` — trzy linie tuż po nowym wywołaniu, zanim cokolwiek innego się
wydarzy w tej samej iteracji. Funkcja jest idempotentna (`if (!session_active)`), więc
późniejsze, pełne `diag_session_tick()` w tej samej iteracji NIE podbija ID drugi raz —
współdzielą jedną funkcję pomocniczą `start_session()`. Start impulsem koła/Iq nadal
działa przez samo `diag_session_tick()`, bez zmian. Dodatkowo: `ride_episode_init()` i
`pas_trace_init()` są teraz wywoływane PONOWNIE w `diag_diagnostics_init()` (obok
`pas_raw_init()`) — w czystym punkcie tuż przed `while(1)`, żeby zdarzenia z kalibracji
rozruchowej nie tworzyły rekordów sesji 0.

Nowy test integracyjny `tests/host/fw106_integration_host.c` (NIE model, prawdziwe
`diag_session.c`+`pas_trace.c`+`pas_raw.c`+`ride_episode.c` linkowane razem) odtwarza
dokładną kolejność wywołań z `main.c`: postój (`session_id=0`) → pierwszy ruch to
podejrzane przejście PAS (SHORT_GAP) → trigger trace+RAW → dalsza jazda → koniec sesji
→ zrzut. Zweryfikowane cofnięciem: przy starej kolejności (trigger przed propagacją)
pada 10 asercji, w tym `pending` dla TRACE i RAW nigdy nie wraca do zera.

**2. Pełna kolejka podsumowań już nie osiera rekordów.** `diag_session_tick()` w
gałęzi „kolejka podsumowań pełna" (`src/diag_session.c`) teraz, obok
`summary_rejected++`, PĘTLĄ po wszystkich trzech źródłach i woła ich `release(session_id,
false)` tak długo, jak `count(session_id)>0` — każdy taki rekord trafia do
`cnt_discarded[i]`, NIGDY do `cnt_sent`/`cnt_rejected`. Cztery starsze podsumowania
zostają nietknięte. Test 14 w `fw106_session_host.c`: zapełnia 4 sloty, tworzy piątą
sesję z EPISODE+TRACE+RAW, sprawdza `summary_rejected==1`, trzy `discarded`, zero
rekordów sesji 5 w kolejkach źródeł, `pending==0` po zrzucie czterech pozostałych, i
że żadna ramka sesji 5 nie pojawia się w logu. Zweryfikowane cofnięciem: bez poprawki
pada 5 asercji.

**3. RAW nie gubi już zdarzenia trafiającego w środek kopiowania PRE.**
Przeprojektowano `pas_raw.c`: zamiast stanu `COPYING`, w którym ISR ignorował
przechwycenie, `pas_raw_freeze()` teraz NAJPIERW otwiera obszar POST (`slot_state =
SLOT_FILLING_POST`, `slot_target = pre_n + POST`), DOPIERO POTEM kopiuje PRE do
`slot[0..pre_n)`. ISR dopisuje od `pre_n` w górę — obszary zapisu głównej pętli i ISR
są od tego momentu ROZŁĄCZNE, więc zdarzenie trafiające w trakcie kopiowania PRE
ląduje jako pierwszy element POST, nie ginie między dwiema połówkami. Wykrywanie
realnego nadpisania (`raw_capture_overrun`) działa bez zmian — porównuje, jak daleko
`seq` przesunęło się podczas kopiowania względem zapasu w pierścieniu. Bez maskowania
przerwań. Nowy blok D4 w `fw106_recorder_host.c` używa świadomego test seamu
`PAS_RAW_COPY_HOOK` (kompilowanego WYŁĄCZNIE do testu hosta — `run-host-tests.ps1`
dodaje `-DPAS_RAW_COPY_HOOK` tylko dla zestawu „FW-106 recorders"), który woła
PRAWDZIWY `pas_raw_isr_sample()` Z WNĘTRZA pętli kopiującej `pas_raw_freeze()`, w
połowie kopiowania PRE — czego test jednowątkowy wywołujący ISR dopiero po powrocie z
`pas_raw_freeze()` nie mógł pokryć. Zweryfikowane cofnięciem (przywrócenie starego
stanu `COPYING`): pada 4 asercje, w tym ciągłość `seq` na granicy PRE/POST.

**4. `DIAG_ERR_RAW_OVERRUN` publikowany per-sesja.** Dodane pole `raw_overrun_total`
do wstrzykiwanego `diag_ops_t` (`inc/diag_session.h`) — monotoniczny licznik
(`pas_raw_capture_overrun()` w `main.c`'s `diag_raw_overrun_total()`), kotwiczony przy
starcie sesji (`session_anchor_raw_overrun`) i migawkowany przy kandydacie ciszy
(`cand_raw_overrun`) DOKŁADNIE jak `missed_ticks`/`missed_events` — więc overrun
podczas 3-sekundowego potwierdzania postoju PO migawce nie zanieczyszcza właśnie
zamkniętej jazdy. Bit `0x10` w trailerze ustawiany z `s->raw_overrun_delta > 0`, per
sesja, nie z surowego globalnego licznika. Trzy testy (15/16/17) w
`fw106_session_host.c`: sesja A z overrun ma bit; sesja B bez nowego overrun NIE
dziedziczy bitu A; overrun w trakcie 3 s postoju po sesji C nie trafia do trailera C.
Zweryfikowane dwoma cofnięciami: usunięcie sprawdzenia w ogóle → pada test 15;
podmiana na surowy globalny licznik (styl „sticky flag") → padają testy 16 i 17.

**5. Budżet RAM mierzy CAŁY stan modułu, nie wybrane tablice.** Każdy z czterech
modułów grupuje TERAZ WSZYSTKO, co mutowalne (bufory, liczniki, indeksy, flagi), w
jeden plikowy `struct` (`S` w `pas_raw.c`, `T` w `pas_trace.c`, `E` w `ride_episode.c`,
`D` w `diag_session.c`) i sprawdza `sizeof(tej struktury)` — dodanie nowego skalara
GDZIEKOLWIEK w module powiększa `sizeof` i może złamać ten sam assert, który pilnuje
buforów. `inc/diag_budget.h` ma teraz limity zmierzone bezpośrednio na docelowym ARM
GCC (`arm-none-eabi-size`, `CAN_DIAGNOSTICS_ENABLE=1`, ta sama flaga co
`build_firmware.ps1`), nie oszacowane: pas_raw 4136 B, pas_trace 7240 B, ride_episode
2412 B, diag_session 388 B — każdy limit to zmierzona wartość + 64 B zapasu na dryf
kompilatora. `DIAG_BUDGET_PAS_TRACE_PREEXISTING_BYTES` też zmierzone wprost (3628 B,
`CAN_DIAGNOSTICS_ENABLE=0`), nie zaokrąglone. Suma po odjęciu preexisting ≈ 10,80 KB,
poniżej 12 KB.

**Zmierzony koszt RAM (finalny, `arm-none-eabi-size`, `data+bss`):**

| Wariant | pas_raw | pas_trace | ride_episode | diag_session | razem |
|---|---|---|---|---|---|
| `CAN_DIAGNOSTICS_ENABLE=0` | 0 B | 3628 B | 144 B | 0 B | **3772 B** |
| `CAN_DIAGNOSTICS_ENABLE=1` | 4136 B | 7240 B | 2412 B | 388 B | **14176 B** |

**Doprecyzowanie (nie błąd):** starszy rekord pominięty w `records_to_send` NOWSZEJ
sesji jest zachowaniem POPRAWNYM, dopóki nadal istnieje jego WŁASNE podsumowanie w
kolejce czterech — `records_to_send` sesji liczy tylko rekordy JEJ sesji, celowo.
Problemem opisanym w punkcie 2 był wyłącznie rekord sesji, której PODSUMOWANIE zostało
odrzucone (5. sesja przy pełnej kolejce) — wtedy rekord nie ma już czyjego zrzutu mógłby
kiedykolwiek go zażądać, i to jest jedyny przypadek wymagający natychmiastowego
`discarded`.

---

## Poprawka wdrożenia (czwarta iteracja) — cztery kolejne błędy

Trzecia implementacja przechodziła wszystkie testy, ale przegląd znalazł cztery
kolejne realne wady — wszystkie w miejscach, gdzie „zamrożone" dane po cichu wciąż
zależały od czegoś liczonego na żywo, albo gdzie utrata całej sesji nie zostawiała
żadnego śladu na magistrali. Wszystkie naprawione, każda z nowym testem regresyjnym
zweryfikowanym cofnięciem poprawki na kopii roboczej (test musi PAŚĆ bez poprawki).

**1. `diag_session_note_activity()` anuluje aktywnego kandydata ciszy.** Wywołanie to
dotąd tylko startowało nową sesję (`if (!session_active)`), ale nic nie robiło, gdy
sesja już trwała — a to jest dokładnie sytuacja „podejrzane przejście PAS w połowie
3-sekundowego okna ciszy". Dwubitowy nielegalny skok w dekoderze daje `qd[]=0`, więc
NIE wpada ani w gałąź naprzód, ani wstecz — `fwd_run`/`pas_idle_ticks`/
`Backwards_counter` (z których liczone jest `moving`) w ogóle się nie zmieniają. Taki
impuls mógł więc siedzieć WEWNĄTRZ już tykającego okna ciszy, nigdy go nie resetując —
sesja zamykałaby się „3 s nieprzerwanej ciszy", mimo że ciszy nie było. Naprawione:
`diag_session_note_activity()` teraz BEZWARUNKOWO czyści `D.quiet`, niezależnie od
tego, czy sesja się właśnie zaczyna, czy już trwa — kolejne wywołanie
`diag_session_tick()` z `moving=false` samo uzbraja nowego kandydata od tego tiku
(to samo re-kotwiczenie co przy prawdziwym postoju). Test 18 w
`fw106_session_host.c`: nielegalna zmiana PAS w połowie okna, potem druga połowa
oryginalnego okna, `summaries_pending` musi zostać 0; dopiero pełne nieprzerwane okno
OD miejsca przerwania zamyka sesję.

**2. Builder 0x10212 nie czyta już niczego na żywo.** Wcześniej wołał
`ride_control_get_arm_snapshot()` i `ride_episode_get_state()` w chwili ZRZUTU — a
rekord w kolejce może czekać na zrzut długo, po innych epizodach, a nawet po innych
sesjach; „żywy" stan `ride_control`/`ride_episode` w tamtej chwili nie ma nic wspólnego
z tym, co było prawdą dla TEGO konkretnego epizodu. Naprawione: `ride_episode.c` sam
zapisuje `iq_pre_ramp_at_target_ready` DOKŁADNIE w chwili ustawienia `t_target_ready`
(ten sam tik, ten sam klamrowany blok) — nowe pole w `ride_episode_result_t`, umieszczone
w istniejącym paddingu struktury (34→36 B), więc **`sizeof` nie wzrósł ani o bajt**, a
koszt kolejki 64 rekordów się nie zmienił. Bajt „stan rejestratora" (dawniej
`ride_episode_get_state()`) zastąpiony stałą `3` („opublikowany/w kolejce") z
komentarzem — żywe wartości 0/1/2 (idle/wait_dip/wait_recover) i tak nie miały sensu
dla epizodu, który już jest w kolejce. Test A4 w `fw106_recorder_host.c`: dwa epizody
z RÓŻNYMI wartościami (85 i 50), plus trzeci, niedokończony epizod symulujący „zmianę
stanu na żywo" pomiędzy — obydwa zakolejkowane rekordy zachowują WŁASNĄ, oryginalną
wartość na końcu.

**3. Ramki zbiorcze sesji zamrażane przy `candidate_end_tick`.** Wybrany wariant
główny (nie alternatywny): 13 ramek bloku zbiorczego (0x10203–0x1020F, 0x10219) jest
teraz przechwytywanych DOKŁADNIE w chwili wzięcia kandydata ciszy — tej samej chwili,
w której migawkowane są `missed_ticks`/`missed_events`/`raw_overrun_delta` — i
zapisywanych PROSTO do slotu podsumowania, który ten kandydat by zajął, gdyby się
potwierdził (bez osobnego bufora kandydata — oszczędność ~150 B). Faza `PH_AGGREGATE`
zrzutu czyta teraz WYŁĄCZNIE z zamrożonego `s->agg_data[]`, nigdy z
`ops->aggregate_frame()` na żywo — więc zrzut sesji A wznowiony PO przejechaniu sesji B
nadal wysyła DOKŁADNIE to, co było prawdą na koniec jazdy A. Skrajny przypadek (kolejka
podsumowań pełna dokładnie w chwili wzięcia kandydata) obsłużony jawnie: taka sesja
dostaje `agg_count=0` (uczciwie pusty blok zbiorczy), zamiast ryzykować nadpisanie
cudzych, wciąż ważnych danych w tym samym slocie. Koszt: nowy `DIAG_AGGREGATE_SNAPSHOT_MAX=16`
× (4+8) B × 4 podsumowania ≈ 780 B — zmierzone, nie oszacowane (patrz budżet niżej).
Test 19 w `fw106_session_host.c`: A → zrzut przerwany przed blokiem zbiorczym → B (z
INNĄ wartością „na żywo") → dokończenie A. A wysyła swoją oryginalną wartość, B swoją.

**4. `DIAG_ERR_SESSION_LOST` (bit `0x80`, wolny w bajcie `err`) w trailerze.** Utrata
CAŁEJ sesji (podsumowanie odrzucone przy pełnej kolejce czterech) nie miała dotąd
ŻADNEGO śladu na magistrali — taka sesja nie dostaje własnego nagłówka ani trailera,
więc `session_summary_rejected` była widoczna wyłącznie przez akcesor testowy. Naprawione:
nowy znak wodny `summary_rejected_reported` w module — jeśli globalny licznik
`summary_rejected` różni się od tego, co było zgłoszone przy OSTATNIM faktycznie
wysłanym trailerze, NASTĘPNY wysłany trailer (którejkolwiek sesji) niesie bit `0x80`.
Znak wodny przesuwa się WYŁĄCZNIE po `CAN_TRANSMIT_OK` (nie po `GIVEUP`) — trailer,
który sam nie trafił na magistralę, nie „połyka" cichaczem informacji o utracie; spróbuje
ją zgłosić następny. Delta, nie sticky flaga — ten sam wzorzec co `DIAG_ERR_RAW_OVERRUN`
z poprzedniej iteracji. Test 20 w `fw106_session_host.c`: zapełnij 4 podsumowania, zgub
piąte, sprawdź że PIERWSZY wysłany potem trailer niesie `0x80`, a DRUGI już nie.

**Zmierzony koszt RAM (finalny, `arm-none-eabi-size`, `data+bss`):**

| Wariant | pas_raw | pas_trace | ride_episode | diag_session | razem |
|---|---|---|---|---|---|
| `CAN_DIAGNOSTICS_ENABLE=0` | 0 B | 3628 B | 148 B | 0 B | **3776 B** |
| `CAN_DIAGNOSTICS_ENABLE=1` | 4136 B | 7240 B | 2416 B | 1176 B | **14968 B** |

Największy przyrost (+788 B) to koszt zamrożenia bloku zbiorczego per sesja (fix 3) —
świadomy, bo alternatywa (czytanie na żywo) była realnym błędem korumpującym dane.
Budżet w `inc/diag_budget.h` zaktualizowany na te zmierzone wartości + 64 B zapasu na
moduł; suma po odjęciu preexisting ≈ 11,32 KB, wciąż poniżej pułapu 12 KB (margines
~676 B — wyraźnie ciaśniej niż poprzednio, odnotowane wprost w komentarzu budżetu).

## Poprawka wdrożenia (piąta iteracja) — ograniczenie tempa zrzutu diagnostycznego

### Dowody z logu `log-2026-08-11-17-41-08-n0.log`

Realna jazda ujawniła utratę większości ramek zrzutu — nie w firmware, tylko między
sterownikiem CAN a plikiem logu. Każdy poniższy fakt zweryfikowano wprost w surowym
logu (nie na słowo z opisu):

| Fakt | Zweryfikowane |
|---|---|
| Sesja 2, czas trwania, epizody | Nagłówek `0x1021D`, dane `01 02 02 57 00 0F 00 12`: `session_id=2`, `duration_ds=0x0257=599`→**59,9 s**, `episodes=0x000F=`**15**, `records_to_send=0x0012=`**18** |
| Trailer | `0x1021E`, dane `01 02 00 12 00 11 04 01`: `records_sent=0x0012=`**18**, `discarded=0`, `rejected=0x11=`**17**, `err=0x04`, `dump_complete=1` — bajt po bajcie zgodnie z opisem |
| Czas nagłówek→trailer | Znaczniki czasu logu: 26673204351 → 26674161801 = **957 450** jednostek ≈ **957 ms** |
| Pełne przechwycenia 256 próbek | Ramki statusu `0x1021B` (RAW) i `0x10215` (TRACE), obie dane `01 00 01 00 00 7F 00 02`: bajt 0 = `1` (**nie** `2`="sealed short", czyli **pełne**), bajty 2-3 = `0x0100=`**256** |
| Oczekiwane vs zapisane ramki | Zliczono WSZYSTKIE ID `0x80010200`–`0x8001021F` w logu: dokładnie **766** ramek. Oczekiwane (zgodnie z deklaracją `records_to_send=18` i pełnymi przechwyceniami) rzędu **~1389** |
| TRACE: 1 status + 255×`0x10216` + 272×`0x10217` | Policzone wprost: `0x10215`→1, `0x10216`→**255**, `0x10217`→**272** (zamiast dwóch pełnych `1+256+256` przy `PAS_TRACE_SLOTS=2`) |
| RAW: 131 z 256 `0x1021C` | Policzone wprost: `0x1021C`→**131** |
| Logowanie trwało ~21 s po trailerze | Trailer o 17:42:15, koniec logu („Stoping sniffer...") o 17:42:36 — sniffer nie został zatrzymany przedwcześnie, miał czas zapisać więcej, a nie zapisał |
| `rejected=17`/`err=0x04` to osobne zjawisko | `DIAG_ERR_CAPTURES_FULL` (`inc/diag_session.h`) — brak wolnych slotów przechwycenia dla NOWYCH triggerów w trakcie jazdy. Ustawiane wyłącznie z `refused_at_close[i] != refused_at_open[i]` (`src/diag_session.c`, faza `PH_TRAILER`) — licznik „drzwi", nie licznik transmisji. Niezależne od tego, ile ramek już zaakceptowanego rekordu logger zdołał zapisać |

**Wniosek:** firmware wysłał (i sterownik CAN to potwierdził) dokładnie to, co
`records_sent=18`/`dump_complete=1` mówią — żaden rekord nie zaginął PO STRONIE
FIRMWARE. Ramki zginęły PO DRODZE, między magistralą a plikiem logu.

### Przyczyna źródłowa (potwierdzona w kodzie)

`src/main.c`, `while(1)`:
```c
if(Speed_flag)Speed_processing();
if(reg_ADC_flag)reg_ADC_processing();
#if CAN_DIAGNOSTICS_ENABLE
    diag_dump_step();   // <- bez żadnego ograniczenia tempa
#endif
```
`diag_dump_step()` (a w niej `diag_session_dump_step()`) była wołana w KAŻDEJ iteracji
pętli głównej, bez żadnego warunku tikowego. Komentarz nad nią głosił „tempo wyznacza
sprzęt CAN" — ale pętla główna wykonuje się dużo częściej niż raz na tik sterowania
4 kHz (warunki `if(Speed_flag)`/`if(reg_ADC_flag)` w większości iteracji nic nie robią),
więc „najwyżej jedna operacja na skrzynce na wywołanie" ograniczało ilość PRACY na
wywołanie, nie ilość wywołań na sekundę. Efekt: zrzut potrafił wysłać setki ramek w ułamku
sekundy (957 ms na 18 rekordów, ~1389 ramek — to grubo ponad 1000 ramek/s), z prędkością,
której logger USB fizycznie nie nadążał zapisywać.

### Poprawka

Nieblokujące ograniczenie: co najwyżej jedna NOWA ramka na `DIAG_TX_FRAME_INTERVAL_MS`
(**10 ms**, czyli **100 ramek/s**), zdefiniowane w `inc/diag_session.h` i wyprowadzone z
`CONTROL_TIMEBASE_HZ`:
```c
#define DIAG_TX_FRAME_INTERVAL_MS    10U
#define DIAG_TX_FRAME_INTERVAL_TICKS ((uint32_t)DIAG_TX_FRAME_INTERVAL_MS * CONTROL_TIMEBASE_HZ / 1000U)
```

API zmienione na `diag_session_dump_step(uint32_t now_tick)` — `main.c` przekazuje
bezpośrednio `control_time_ticks` (wolnobieżny licznik ISR 4 kHz, ten sam, którego
używają `diag_session_note_activity()`/`diag_session_tick()`), nie jego lokalną
migawkę sprzed `reg_ADC_processing()`.

Ograniczenie w `tx_step()` (`src/diag_session.c`) chroni WYŁĄCZNIE rozpoczęcie nowej
transmisji — dokładnie w miejscu, gdzie automat miałby zawołać
`can_ops->transmit()` dla świeżej ramki (w tym ponowienie po `FAILED`, bo to też nowe
wywołanie `transmit()`):
```c
if ((uint32_t)(now_tick - D.tx_pace_last_tick) < DIAG_TX_FRAME_INTERVAL_TICKS) {
    return STEP_BUSY;
}
```
Odejmowanie, nie `now_tick >= deadline` — odporne na zawinięcie `uint32_t` tak samo, jak
każda inna różnica tików w tym kodzie. Nic ponad tę jedną gałąź nie jest opóźnione:
sprawdzenie wyniku ramki już w skrzynce, wykrycie wznowienia jazdy i przerwanie zrzutu,
zachowanie kursora przy `NOMAILBOX`/`PENDING` — wszystko dzieje się przy KAŻDYM
wywołaniu, niezależnie od tego, co robi timer pacingu. `D.tx_pace_last_tick`
aktualizowane jest wyłącznie w momencie realnego przyjęcia ramki przez sterownik
(`transmit()` zwraca coś innego niż `DIAG_CAN_NOMAILBOX`) — odmowa `NOMAILBOX` nic nie
kosztuje z budżetu czasu.

### `records_sent` a kompletność w loggerze — dwie różne rzeczy

To rozróżnienie było już w projekcie v6 (`records_to_send`, sekcja 9: „Parser porównuje
to z tym, co faktycznie odebrał — a niezgodność jest miarą utraty w loggerze,
niezależną od firmware") — ten incydent jest pierwszym realnym potwierdzeniem, że to
rozróżnienie jest konieczne, nie teoretyczne:

- **`records_sent`** (trailer `0x1021E`) rośnie, gdy sterownik CAN potwierdzi
  (`CAN_TRANSMIT_OK`) OSTATNI fragment danego rekordu. To jest prawda o TRANSMISJI —
  firmware nie ma i nie może mieć wiedzy o tym, co dalej dzieje się z ramką na
  magistrali (adapter USB, bufor sniffera, plik logu).
- **Kompletność w loggerze** to zupełnie inna liczba: ile ramek danego rekordu
  RZECZYWIŚCIE trafiło do pliku logu. Jedyny sposób, by to sprawdzić, to policzyć ramki
  w logu i porównać z tym, co nagłówek/trailer deklarują (`records_to_send` w nagłówku,
  `records_sent` w trailerze) — dokładnie to, co ujawniło ten incydent (18 zadeklarowane
  i potwierdzone, 766 zamiast ~1389 ramek fizycznie w pliku).
- Ograniczenie tempa (wyżej) **zmniejsza ryzyko** utraty w loggerze — nie eliminuje go z
  definicji, bo zależy od realnej przepustowości konkretnego sniffera/USB. Dlatego
  kompletność TRACE/RAW w kolejnym logu z jazdy pozostaje **obowiązkową weryfikacją
  sprzętową**, nie założeniem.

### Przewidywany czas zrzutu

Przy 100 ramek/s: zrzut z tego konkretnego logu (~1389 ramek) potrwałby
≈ 1389 × 10 ms ≈ **13,9 s** ≈ **~14 s** — zgodnie z oczekiwaniem, i wyraźnie poniżej
zalecanego postoju **30 s** (sekcja 10). Najgorszy praktyczny przypadek (pełne
przechwycenia TRACE **i** RAW jednocześnie, `PAS_TRACE_SLOTS=2`, maks. epizodów) mieści
się w tym samym rzędzie wielkości i nadal zostaje w budżecie 30 s z zapasem.

### Testy (wykonane na prawdziwym `src/diag_session.c`, `tests/host/fw106_session_host.c`)

| # | Sprawdza |
|---|---|
| 21 | wiele wywołań w TYM SAMYM ticku zaczyna co najwyżej jedną nową ramkę |
| 22 | tick przed upływem interwału — kolejna ramka NIE startuje |
| 23 | dokładnie po interwale — kolejna ramka MOŻE wystartować |
| 24 | ograniczenie przeżywa zawinięcie `uint32_t` (w tym przypadek, w którym naiwne `now_tick >= deadline` zawiodłoby, a odejmowanie — nie) |
| 25 | `NOMAILBOX` nie uzbraja zegara pacingu, nie przesuwa kursora, nie gubi fragmentu |
| 26 | ponowienie po `FAILED` oferuje IDENTYCZNY fragment i samo też podlega pacingowi |
| 27 | ruch roweru przerywa zrzut NATYCHMIAST, nawet gdy timer transmisji wciąż blokuje kolejną ramkę |
| 28 | duży, realistyczny zrzut (15 epizodów + pełne przechwycenie TRACE 256 + pełne RAW 256 — kształt tego właśnie logu) kończy się poprawnym trailerem, `records_sent=17`, `dump_complete=1`, dokładnie 851 ramkami |

**Zweryfikowane cofnięciem** (kopia robocza w scratchpadzie, nigdy w repozytorium):
usunięcie WYŁĄCZNIE gałęzi pacingu z `tx_step()` (zachowując sygnaturę z `now_tick`)
powoduje, że testy 21, 22, 24 i 26 PADAJĄ — reszta przechodzi bez zmian, co pokazuje, że
to właśnie te cztery testy pilnują tej konkretnej poprawki.

### Zmierzony przyrost RAM

`struct D` w `src/diag_session.c` urosła o dokładnie **4 B** (`tx_pace_last_tick`,
`uint32_t`): 1176 B → **1180 B** (`arm-none-eabi-nm --print-size`, zmierzone na
izolowanym obiekcie ARM, `CAN_DIAGNOSTICS_ENABLE=1`). `inc/diag_budget.h` zaktualizowany:
`DIAG_BUDGET_SESSION_BYTES` 1240→**1244 B** (zmierzone + 64 B zapasu, ten sam wzorzec co
pozostałe trzy moduły). Suma budżetu po odjęciu preexisting ≈ **11,33 KB**, wciąż poniżej
pułapu 12 KB.

### Zakres NIE ruszony (zgodnie z poleceniem)

`PAS_TRACE_SLOTS`, rozmiar RAW, kolejki, budżet przechwyceń, formaty i CAN ID
`0x10203`–`0x1021E`, kryteria końca sesji, dekoder PAS/latch/rearm_after_reverse/progi
nacisku/start_steps/sterowanie silnikiem — wszystko bez zmian. `rejected=17`/
`capture_slots_full` (osobne zjawisko, patrz tabela dowodów wyżej) pozostaje osobną,
NIE naprawianą teraz decyzją.

## Co dalej

Przed jazdą, zgodnie z sekcją 10: zbudować firmware (osobne polecenie) i sprawdzić
mapę konsolidacji `.map` OBU wariantów — czy po dodaniu ~11,3 KB (DIAG=1) zostaje
bezpieczny zapas wolnego RAM, zwłaszcza że margines budżetu skurczył się do ~676 B.

Pełny audyt mapy konsolidacji (obie wersje, DIAG=0 i DIAG=1) oraz jedno instalowalne
firmware diagnostyczne (`0.0330`, BL820, PRZED poprawką pacingu z tej karty) już
powstały, w osobnych turach, po osobnych poleceniach — udokumentowane poza tym plikiem.

**Poprawka pacingu z tej iteracji jeszcze NIE trafiła do żadnego zbudowanego firmware.**
W tej turze zweryfikowano wyłącznie kompilację obiektową (`arm-none-eabi-gcc -c`)
każdego pliku źródłowego w obu wariantach (DIAG=0 i DIAG=1) — bez nowych ostrzeżeń, bez
zmiany layoutu żadnej ramki — nie pełne łączenie i nie plik do instalacji. **Kolejny
krok:** zbudować NOWE firmware diagnostyczne zawierające tę poprawkę (osobne polecenie),
zainstalować, i dopiero WTEDY, na realnej jeździe, potwierdzić na kolejnym logu, że
TRACE i RAW faktycznie docierają w komplecie — dopóki taki log nie powstanie, kompletność
loggera przy nowym tempie 100 ramek/s jest przewidywana, nie zmierzona.

---

## FW-112-DIAG: piąte źródło zrzutu, `DIAG_SCHEMA_VERSION` 3 → 4

Karta FW-112-DIAG (osobny raport: `FW-112-DIAG_WHOLECHAIN_RECORDER_PL.md`) dodaje **piąte źródło
zrzutu** (`DIAG_SRC_FW112 = 4`, `DIAG_SRC_COUNT = 5`). Każdy rekord to nagłówek `0x1022A` + 4×8 B
ramek danych `0x1022B..0x1022E`; rekord ma dokładnie 32 B (`_Static_assert`), więc serializacja nie
ma paddingu. **Wszystkie ramki 0x10200..0x10228 i layouty zostają BAJT W BAJT bez zmian** — log v3
czyta je identycznie i po prostu nie zawiera 0x1022x (reguła „podbij przy KAŻDEJ zmianie" z sekcji 11).

| EFID | zawartość |
|------|-----------|
| `0x0001022A` | nagłówek rekordu: `[0]=FW112_DIAG_SCHEMA_VERSION(1)` `[1]=session_id` `[2-3]=event_id` BE `[4]=event_type` `[5]=reason_bits` `[6-7]=0` |
| `0x0001022B` | `[0]=session_state` `[1]=dir_state` `[2]=recovery_state` `[3]=fwd_run` `[4]=crank_forward_steps` `[5]=required_steps` `[6]=start_steps` `[7]=cadence_rpm` |
| `0x0001022C` | `[0-1]=assist_hold_ticks` BE `[2-3]=load_centikg` BE `[4-5]=load_threshold_centikg` BE `[6]=flags` `[7]=0` |
| `0x0001022D` | `[0-1]=iq_request` `[2-3]=iq_pre_ramp` `[4-5]=iq_setpoint` `[6-7]=iq_actual` (i16 BE) |
| `0x0001022E` | `[0-3]=elapsed_ticks` u32 BE (odstęp od poprzedniego eventu; pierwszy = 0) `[4-7]=0` |

Trailer sesji `0x1021E` zyskuje bit `0x04` (`DIAG_TRAILER_F_FW112_REJECTED`): kolejka FW-112-DIAG
odrzuca (nie nadpisuje) przy pełnej pojemności, a odrzucenie jest widoczne w logu. Polityka pełnej
kolejki pozostaje odrzucenie nowego + `rejected_total`, jak dla pozostałych źródeł.
