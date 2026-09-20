# G5300 — materiał referencyjny dla dalszego rozwoju EVistDrive

> **Aktualizacja 2026-09-14, 11:49 +02:00:** dostarczono pełniejszy raport closure i appendix adresowy w `external/datasheets`. Bieżący stan wiedzy, domknięcia i korekty zawiera [CLOSURE_REVIEW_2026-09-14_PL.md](CLOSURE_REVIEW_2026-09-14_PL.md) — **czytaj go najpierw**. Poniższy intake zachowuje wcześniejszy stan źródeł. Alpha≈0,10 zostało wyjaśnione dodatkowym skalowaniem; nadal nie rozstrzygnięto 72/144 accepted events na obrót. Nowe dokumenty opisują downstream E1E8 i wycofują fizyczną interpretację AUTO „200 W”. Sam BIN i pomiary HW nie zostały dostarczone w tym pakiecie.

Zapis: 2026-09-14T08:38:46+02:00. Zakres: przyjęcie materiału i analiza porównawcza, bez portowania kodu.

## Najpierw przeczytaj to

To opis reverse **innego sterownika**, przekazany przez użytkownika jako inspiracja. Nie jest specyfikacją M820 ani dowodem pomiarowym naszego roweru. Zachowujemy rozdział funkcji i pomysły do testowania; nie kopiujemy automatycznie stałych, skali torque, liczby przejść PAS ani czasów.

Źródło zachowane bez zmian: [USER_HANDOFF_2026-09-14.txt](USER_HANDOFF_2026-09-14.txt).

SHA256 otrzymanego tekstu: `c42d126011a0f5da88096b932aba07c7fbb67f59731f2a2ed7325dfa874fb936`.

**Status dowodów:** USER_SUPPLIED / opis analizy zewnętrznej; szczegóły G5300 mają w tym projekcie status SUPPORTED, a nie niezależnie CONFIRMED_REVERSE. W tej sesji nie otrzymano BIN-a, disassembly ani trzech pełnych dokumentów wymienionych w tekście. Nie znaleziono ich pod nazwami zawierającymi G5300 w workspace. Adresy i hashe poniżej są wskazówkami do późniejszej identyfikacji, nie zweryfikowanym materiałem pierwotnym.

Wymienione dokumenty: `G5300_FULL_ASSIST_AUTO_ACCELERATION_TO_EVISTDRIVE_AGENT_HANDOFF.md`, `SPORTPLUS_AUTO_REVERSE_G5300.md`, `G5300_PAS_FULL_REVERSE.md`. W tekście podano hash głównego dokumentu `ff94fefe127467812be19bfa5fc3f27d6619f0c50b1546b1f833f156ebb8c66e` przed kolejnym opisem aktualizacji; jego przynależność do ostatniej wersji jest niezweryfikowana. Deklarowany hash pełnego PAS: `f83b1d80d842dee4f74c091928a99a6dc6aa97bd29aaaf324f36f05015b0ce0f`.

## Ustalenia po uwzględnieniu korekt wewnątrz źródła

| Obszar | Ostatnia wersja twierdzenia autora reverse | Granica zastosowania |
|---|---|---|
| Zegar | PAS co 1 ms; torque-state/AUTO/assist nominalnie co 10 ms | EVistDrive ma własne domeny 4/16 kHz. Portować sekundy/zdarzenia, nie liczby wywołań. |
| Kierunek | Sekwencja Gray 00→01→11→10→00; odwrotna = reverse; reverse zeruje forward evidence | Polaryzacja kanałów zależy od sprzętu. EVistDrive używa `PAS_DIR_SIGN=-1`. |
| Pierwsza kadencja | Po pełnym stopie pierwszy edge ustala kierunek, dopiero trzy następne tworzą pomiar: **4 fizyczne transitions**, nie 3 | To warunek pomiaru kadencji, nie automatycznie zgoda na wspomaganie. |
| AUTO readiness | evidence ≥4 LUB effort envelope ≥2500 | Gate estymatora; **nie zezwolenie na torque-only start**. Jednostka 2500 nie jest ustalona dla M820. |
| True stop | `clamp(180000/(144*cadence+1),25,208)` ms; dodatkowe opóźnienie per-level 0…350 ms, w opisanym BIN-ie +0 | Stała 144 i zakres odnoszą się do konfiguracji tamtego PAS. Nie mylić z Acceleration 1–8. |
| Reset STOP | movement/cadence/filter/count/epoch reset; ponowne zebranie pomiaru | Długi envelope nie może podtrzymać permission po true stop. |
| Reverse → AUTO clear | Pierwszy pewny reverse usuwa dodatnią cadence; następnie trzeba zebrać okno ujemnej cadence; clear AUTO do ok. 0–9 ms **od publikacji ujemnej cadence** | Wcześniejsze „0–9 ms od reverse edge” jest zastąpione późniejszą korektą autora. |
| Kadencja | Dynamiczne okno kilku transitions; filtr aktualizowany zdarzeniami PAS | Event-driven nie jest błędem timebase. Wymaga jawnej jednostki zdarzeń i geometrii PAS. |
| Invalid | Niesąsiedni przeskok nie daje nowego kierunku ani forward evidence; stock zachowuje poprzedni kierunek | To inna polityka niż lokalne fail-safe inhibit. Nie przenosić bez odrębnej analizy. |
| Anti-rock | Osobny detektor kołysania przód/tył | Kandydat do testów, nie gotowy algorytm z podanymi progami. |
| Single-wire | Syntetyczne forward A/B; brak fizycznej informacji reverse | Nie udawać wiarygodnej informacji o kierunku z jednego kanału. |
| Torque-state | Osobna maszyna 0–5 i wewnętrzne progi/debounce | Bez jednostek i pełnego automatu nie kopiować progów 10/15/20/30/40 ani guardów. |
| AUTO ratio | Deklarowany wzrost 10 pp/10 ms = 1000 pp/s; 1→525% ok. 0,524 s | To dynamika ratio, nie narastanie całego Iq ani czas rozpoznania pedałowania. |
| Trajektoria | Deklarowane RUN rise full-scale ok. 2,50…0,26 s; RUN fall ok. 0,20 s | **RUN fall nie jest dowodem czasu pedal stop, reverse ani brake/fault → measured Iq=0.** |

Autor wskazuje PAS `0x080182F2`, filtr `0x08019A84`, slot AUTO `D7EC` i selektor `G+0x13E`. Ich znaczenie wymaga BIN-a odpowiadającego tej analizie.

## Niejasności, których nie wolno zamienić w stałe produkcyjne

1. **Współczynnik filtra:** podano coefficient=1638, shift=12 i alpha≈0,10. Samo `1638/4096≈0,3999`. Alpha≈0,10 wymaga dodatkowej skali, np. 4, albo innego efektywnego przesunięcia. Nie przesądzamy, że reverse jest błędny: streszczenie może pomijać skalowanie. Nie używać tabeli tau≈9,5 events bez pełnego równania IIR.
2. **Różnice stanów Gray:** delta ±1/±3 ma sens po zamianie stanów Gray na kolejne rangi cyklu. Surowe wartości 0→1→3→2 nie mają stale takich różnic (1→3 daje +2 mimo legalnego kroku). Implementować zweryfikowaną tablicę przejść; nie odejmować bezpośrednio surowych bitów.
3. **Nieznany downstream czas:** materiał jawnie pozostawia otwarty pierwszy reverse edge / brake / fault → rzeczywisty Iq=0. Nie wyprowadzać go z 1 ms, 10 ms ani 200 ms RUN fall.
4. **Parameter2:** niepełne przypisanie nazw do konsumentów; nie tworzyć timerów na podstawie podobnie brzmiących nazw.
5. „Dwa miejsca z pamięcią” traktować jako zasadę ograniczania kaskady demand/trajectory, nie literalny zakaz pamięci w PAS, estymatorach AUTO/load, zabezpieczeniach czy PI.
6. Proponowany „v3”, stare tryby TORQUE/POWER/eMTB oraz shadow OLD/NEW w źródle są poradą z innego etapu prac. Nie są nowym zleceniem. Obecny projekt ma V2 i wymagane ECO/TRAIL/SPORT/SPORT+/AUTO/AUTO SPORT+; nie odtwarzać usuniętego legacy.

## Porównanie z bieżącym EVistDrive (CONFIRMED_CODE)

Sprawdzono HEAD `9dc0b0a`; przyszły agent powinien najpierw sprawdzić, czy kod się zmienił.

- `pas_quadrature.c` ma tablicę przejść; `pas_sampler.c` osobno odrzuca bardzo krótkie nie-forward zakłócenia. Nie wprowadzać drugiego dekodera z G5300.
- `pas_direction.c` blokuje po kwalifikowanym reverse/invalid, niezależnie od publikacji ujemnej rpm. To właściwa granica zgody na wspomaganie; stockowe opóźnienie clear AUTO jej nie zastępuje.
- `pas_liveness.c` już oddziela activity od kierunku i opiera się na rzeczywistych timestampach. `main.c:2952–2969` dobiera timeout jako 2×ostatni odstęp, ograniczony do **200…500 ms**, i resetuje kadencję przy true stop. Nie twierdzić, że EVistDrive nie posiada tego mechanizmu.
- `main.c:2839` aktualizuje `cadence_filter` po świeżym pomiarze PAS, nie co tick. Aktualnie jest to IIR 1/8, seedowany pierwszym pomiarem; pomiar obejmuje 4 transitions przy skonfigurowanych 96 transitions/rev. Event-driven już istnieje, ale ma inną geometrię niż opisane G5300.
- `ap2_rider_demand.c` realizuje base/dynamic, a `ap2_pas_state.c` odebranie permission. Zachować tę separację.
- `assist_pipeline.c:trajectory()` używa `release_ms` zarówno dla opadania podczas jazdy, jak i zatrzymania. RUN FALL liczy czas pełnej skali, RELEASE czas do zera z aktualnego stanu. Tryby są już odróżnione, lecz jedno ustawienie steruje dwiema różnymi wielkościami czasowymi.

## Wytyczne do wykorzystania

**Najpierw poprawki z audytu V2, potem strojenie na podstawie porównania.** [Audyt i korekta K1](../../AUDIT_ASSIST_PIPELINE_V2_2026-09-14_PL.md).

Docelowy kontrakt do testów:

```text
sensor validity / PAS direction / activity / cadence
  → permission (niezależne od envelope)
  → rider demand + dead-spot base/dynamic
  → profile / AUTO
  → desired Iq + jawne ograniczenia
  → jedyny właściciel trajektorii Iq
     START / RUN RISE / RUN FALL / PEDAL RELEASE / REVERSE / SAFETY
  → PI / FOC
```

Nie jest to nakaz tworzenia sześciu nowych timerów ani enumów. Przyczyny i zachowanie mają być rozróżnialne; wspólny kod można zachować.

Testy porównawcze:

1. Oznaczyć oddzielnie: ostatni fizyczny PAS edge, wykrycie stopu/reverse, utratę permission, zero targetu, zero referencji i measured-current below threshold z czasem ustalenia. Dla hardware jawnie ustalić tolerancję prądu wokół zera.
2. Zmierzyć całkowity stop przy 20/40/60/90/120 rpm, różnych naciskach i fazach korby, także z wysokim zachowanym envelope. Rozdzielić budżet detekcji, planowania trajektorii i odpowiedzi elektrycznej. Nie skracać timeoutu do 25 ms bez sprawdzenia naszego PAS.
3. Rozróżnić: słabszy nacisk przy ciągłym PAS, martwy punkt, true stop, reverse oraz brake/fault. Osobno kryteria RUN fall i pedal release; nie używać jednej nastawy jako zastępstwa całego kontraktu.
4. Testować start/reseed, kołysanie, single-edge bounce, utratę jednego edge, utrzymujące się invalid i overflow. Sama tolerancja stocku dla invalid nie uprawnia do osłabienia lokalnej blokady.
5. Dla filtra cadence porównać odpowiedź w liczbie pomiarów oraz milisekundach przy różnych rpm. Audytowa wada LPF elapsed-time w `ap2_math.h` jest innym zagadnieniem niż świadomie event-driven cadence.
6. Każdą zapożyczoną stałą opisać jednostką, targetem źródłowym, przeliczeniem i testem M820. Mechanizmy uznać za sprawdzone na M820 dopiero po odpowiednich testach tego targetu.

## Jak kontynuować

Przy otrzymaniu pełnych MD/BIN zachować pliki i ich faktyczne hashe, powiązać twierdzenia z adresami/instrukcjami/skalowaniem, rozstrzygnąć alpha oraz opóźnienia downstream. Do tego czasu można używać podziału odpowiedzialności i scenariuszy testowych; nie deklarować wiernego portu ani zmierzonej poprawy odczucia jazdy.
