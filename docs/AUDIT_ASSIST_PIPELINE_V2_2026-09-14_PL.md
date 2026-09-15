# Audyt Assist Pipeline V2 — wytyczne do poprawek

> **Nowszy audyt dla agenta (2026-09-14, 13:36):** [konfiguracja V2, CANable i gotowość wydania](AUDIT_CONFIG_CANABLE_RELEASE_2026-09-14_PL.md). Obejmuje poprawki na `226cb54`, wyniki bramki i nowe blokery C1–C7. To aktualne wytyczne dalszej pracy; poniższe ustalenia pozostają historią wcześniejszego stanu.

> **Zakres historyczny:** findings poniżej dotyczą `9dc0b0a`. Po audycie pojawiły się commity poprawek; stan bieżącej implementacji wymaga ponownego review. K1/K2 uzupełniają wiedzę referencyjną, nie nadają PASS późniejszemu kodowi. Aktualny stan reverse: [kwalifikacja closure G5300](reference/g5300/CLOSURE_REVIEW_2026-09-14_PL.md).

REPORT_TIMESTAMP: 2026-09-14T08:33:06+02:00

EXECUTION_ID: EXEC-EVD-ASSIST-V2-AUDIT-001  
TASK_ID: NONE (ad-hoc), AUTHORISED_BY: USER  
PROJECT_ID: EVD; REPOSITORY: motor-controller-firmware; TARGET: M820  
Reviewer: Codex, sesja niezależna od implementacji.  
Zakres: audyt implementacji wobec przekazanego „REBUILD ASSIST PIPELINE”; bez poprawek produkcji.  
Baseline: `feature/assist-pipeline-v2`, HEAD `9dc0b0a`, czysty status przed audytem.  
Recenzowana implementacja: `e89b2ce`, `c37cc51`, `bfc98b9`, `9a5d8b8` oraz późniejsza dokumentacja.  
FINAL_STATUS: COMPLETED (audyt); REVIEW: **CHANGES_REQUIRED**; VERIFICATION REVIEW STATE: INSUFFICIENT dla odbioru całej implementacji.

## Werdykt

To jest rzeczywista wymiana architektury, z nowymi modułami demand/PAS/estimatorów/profili/limitów i wspólnym punktem publikacji. Nie ma podstaw do odrzucenia całego rozwiązania i ponownego przepisywania. Nie spełnia jednak jeszcze Definition of Done. Problemy obejmują sterowanie dodatnim Iq po reverse, integrację ograniczeń, ciągłość AUTO i działający tor obserwacji/replay.

Poniższe ustalenia są CONFIRMED_CODE, a wskazane wyniki sondy dodatkowo potwierdzają zachowanie wykonywanego kodu C. Nie są potwierdzeniem zachowania fizycznego roweru.

## 1. HIGH — reverse zeruje target, ale pozostawia dodatnią referencję Iq przez 200 ms

**Miejsce:** `src/assist_pipeline.c:23,230–242`, `src/fast_iq_slew.c:239`, `src/main.c:3745`; test `tests/host/ap2_pipeline_scenarios_host.c:369`.

`block_positive` wybiera `FIS_MODE_SAFETY` z czasem 200 ms. ISR realizuje liniowy zjazd z bieżącego Iq, a jego wynik trafia do PI. To nie jest wyłącznie fizyczne zanikanie prądu po wyzerowaniu referencji: regulator nadal dostaje dodatnie zadanie. Sonda produkcyjnego slew: początek 400, pierwszy ISR po komendzie 400, po 100 ms 200.

**Niezgodność:** wymaganie §19 zabrania podtrzymania/ramp-down ciągnącego silnik po reverse. Obecny test sprawdza tylko zerowy `final_iq_request` i wręcz wymaga `FIS_MODE_SAFETY`, więc utrwala słabszy kontrakt.

**Poprawka:** odróżnić reverse od zwykłego puszczenia pedałów; dodatnią referencję usunąć w pierwszym ISR konsumującym komendę. Zachować kontrolowany lifecycle PI/PWM, zamiast utożsamiać zero referencji z wyłączeniem mostka.

**Odbiór:** produkcyjny PAS → pipeline → mailbox → fast slew → wejście PI. Reverse przy kilku dodatnich Iq, w starcie, AUTO, pod limitem i po wznowieniu: target oraz dodatnia referencja znikają bez 200 ms ogona. Fizyczne zanikanie prądu i klik wymagają osobnego pomiaru HW.

## 2. HIGH — Walk kasuje histerezę limitera baterii w każdym ticku

**Miejsce:** `src/ride_control.c:194`, `src/assist_pipeline.c:95`, `src/battery_iq_cap.c:45–60`.

Każdy tick Walk wywołuje `assist_pipeline_reset()`, który resetuje również wspólne `ap2_limits`. Limiter traci pamięć wejścia w ograniczenie. Dla limitu 10 A: po 11 A aktywny cap wynosi 210; przy 9,5 A powinien pozostać 210 do zejścia poniżej 9 A. Po resecie Walk wynosi 700. Walk publikuje wynik przez BYPASS, więc ta zmiana nie przechodzi przez dynamikę assist.

**Poprawka:** oddzielić reset stanu pedałowania od resetu wspólnych zabezpieczeń. Zachować stan limitera w trakcie Walk i świadomie obsłużyć przejścia między źródłami.

**Odbiór:** `ride_control_update()` z aktywnym Walk, sekwencja 11 → 9,5 → 8,9 A oraz szum wokół progu. Sprawdzić cap, flagę i referencję, a nie tylko sam moduł baterii.

## 3. HIGH — zachowany parametr ograniczenia Iq poziomu nie działa

**Miejsce:** `src/assist_modes.c:539,691`, `src/ride_control.c:245`, `src/main.c:3294`, `src/ap2_limits.c`.

`max_iq_pct` jest przyjmowany i zapisywany, ale wszystkie jego wystąpienia w produkcyjnym `src` to wartość domyślna oraz serializacja/deserializacja. Nie jest używany w sterowaniu. `level_iq_limit` otrzymuje globalny `ride_core_iq_limit_scaled`, a nie limit skonfigurowanego poziomu. Ustawienie np. 20% nie ogranicza więc Iq poziomu do 20%.

**Poprawka:** podłączyć istniejący limit do wspólnego etapu phase/Iq; jawnie zdefiniować semantykę 0%. Nie używać zera równocześnie jako „brak limitu” i „zerowy dozwolony prąd”. Przejrzeć migrację innych ograniczeń: `assist_modes_profile_override()` odrzuca także `max_motor_power_w` dla starych identyfikatorów trybów. Migracja profilu nie powinna bez świadomej decyzji podnosić zapisanych ograniczeń.

**Odbiór:** zmiana tylko `max_iq_pct` 20/50/100 przy tym samym wejściu i nasyconym demand daje odpowiednie pułapy. Round-trip CAN/flash i migracja starszego banku zachowują uzgodnione limity.

## 4. HIGH — limitery ograniczają target, ale referencja może długo przekraczać nowy pułap

**Miejsce:** `src/assist_pipeline.c:trajectory()`, `src/ap2_limits.c`, `src/fast_iq_slew.c:214–236`, `src/main.c:3735–3750`.

Po obniżeniu dodatniego targetu przez limiter `trajectory()` używa zwykłego `release_ms`. Fast slew schodzi od wcześniejszej referencji, pozostając powyżej nowego targetu. Nie otrzymuje osobnego maksymalnego dopuszczalnego Iq. Main wprost potwierdza brak post-slew battery clamp. Zatem ograniczenie żądania nie dowodzi ograniczenia prądu zadawanego PI; czas reakcji na limiter zależy od nastaw odczucia jazdy.

**Poprawka:** rozdzielić rider target od pułapu bezpieczeństwa i doprowadzić pułap do jedynego właściciela referencji. Zdefiniować osobno szybkość redukcji i płynne odzyskiwanie dostępnego prądu; bez drugiego pisarza Iq i bez przebudowy FOC. Udokumentować dopuszczalny transient dla każdego zabezpieczenia.

**Odbiór:** nagłe obniżenie phase/battery/power/thermal cap przy już wysokiej referencji, także dla długiego release. Mierzyć `Iq_ref` i prąd rzeczywisty w SIL, nie tylko `final_iq_request`; testować również wyjście spod limitu. To finding integracyjny, nie twierdzenie o zmierzonym przeciążeniu sprzętu.

## 5. MEDIUM — AUTO ma skok charakterystyki przy factor 500

**Miejsce:** `src/ap2_profiles.c:265`.

Parametry są interpolowane, lecz krzywa zmienia się dyskretnie: `factor >= 500`. Przy base=500 kształt SOFT daje 300, LINEAR 500; AUTO SPORT+ zmienia LINEAR 500 na EAGER 620. Ciągły gain nie usuwa tego skoku. Rampa może go rozciągnąć w czasie, lecz nie czyni charakterystyki ciągłą. Brak histerezy umożliwia wielokrotne przejście przez próg.

**Poprawka:** interpolować wynik obu krzywych lub ich węzły tym samym factor. Nie naprawiać dodatkowym filtrem ani przełączaniem trybów z histerezą.

**Odbiór:** sweep factor 490…510 w obu kierunkach przy kilku stałych base i bez ograniczników. Sprawdzić ciągłość response oraz monotoniczność dla AUTO i AUTO SPORT+.

## 6. HIGH — nowa telemetria nie ma zgodnego dekodera i dokumentacji

**Miejsce:** `inc/ride_telemetry.h:43–44`, `src/ride_telemetry.c:60–63,98–109`, `tools/decode_canable_ride_log.py:207,343`, `protocol/RIDE_TELEMETRY_CAN.md`.

Firmware wysyła schema 2 i 9 ramek, zmienia znaczenie pól CORE oraz dodaje 0x10408/0x10409. Dekoder dopuszcza wyłącznie schema 1, interpretuje dwa pola CORE nadal jako torque FAST/RUN i nie dekoduje nowych obserwacji base/dynamic/estimatorów. Dokument protokołu nadal opisuje schema 1 oraz 7 ramek. Samo zwiększenie wersji nadajnika nie kończy integracji.

**Poprawka:** dodać jawne dekodowanie obu wersji, nowe pola i kompletność 9 ramek, uzupełnić protokół. Zdefiniować obsługę logu zaczynającego się przed META; nie zgadywać jednostek. Zachować odczyt dawnych logów.

**Odbiór:** fixture pochodzący z produkcyjnego serializatora schema 2 → raw CANable → decoded/canonical → replay. Asercje na wartości i jednostki wszystkich nowych pól, utratę ramek, META oraz zgodność schema 1.

## 7. MEDIUM — deklarowana niezmienność względem elapsed time nie zachodzi

**Miejsce:** `inc/ap2_math.h:100–126`, `src/ap2_estimators.c:window_delta()`.

LPF używa kroku Eulera, a przy elapsed >= tau przeskakuje do targetu. Produkcyjna funkcja dla 20 ms: 80 wywołań po jednym ticku daje 634, jedno wywołanie reprezentujące 80 ticków daje 1000. To różna odpowiedź na ten sam czas stałego wejścia. W estymatorach delta po zamknięciu okna jest dzielona przez nominalny czas, mimo że rzeczywiste okno mogło być dłuższe; nadmiar czasu zostaje skasowany.

**Poprawka:** określić maksymalny dopuszczalny stall i tolerancję. Dla LPF zastosować zgodne czasowo rozwinięcie/catch-up; pochodne normalizować rzeczywiście upływającym czasem. Obsługę bardzo długiego stallu oddzielić od estymacji normalnej jazdy.

**Odbiór:** jednakowe sygnały z podziałami czasu 1/4/16/80 ticków i jitterem; porównanie demand, base, aggression, load, AUTO oraz start/stop względem zdefiniowanej tolerancji.

## 8. MEDIUM — replay i test tłumienia nie stanowią jeszcze pełnego dowodu jakości

**Miejsce:** `tools/run_replay_regression.py:21–31`, `sim/replay/cases/*/manifest.json`, `tools/analyze_assist_ripple.py:69–105`.

Sześć rzeczywistych przypadków W1 ma `accepted_output_sha256: null`. Runner mimo tego drukuje CASE PASS po wykonaniu replay, bez kryteriów zachowania dla tych przypadków. To test wykonania, nie potwierdzenie poprawnego wspomagania. Nie należy przypinać hashy wadliwego wyniku tylko po to, żeby uzyskać zieloną bramkę.

Analizator tłumienia pomija brakujące scenariusze, a `rows_seen` zwiększa przed sprawdzeniem poprawności torque/Iq. Nieprawidłowe lub zerowe Iq może pominąć ocenę i pozostawić `worst=0`. Ochrona przed nasyceniem obejmuje response, ale nie wymusza braku ograniczania prądu przez limitery.

**Poprawka:** osobno raportować REPLAY_EXECUTED i BEHAVIOR_ACCEPTED. Dodać ilościowe kryteria dla W1: tłumienie poza nasyceniem, reakcja na zmianę nacisku, zjazd i restart. Analizator ma wymagać wszystkich scenariuszy, skończonych danych i niezerowej odpowiedzi oraz ujawniać aktywność limiterów.

**Odbiór:** testy negatywne z brakującym CSV, NaN, zerowym Iq i wyjściem przypiętym limiterem muszą odrzucić dowód jakości. Rozszerzyć ocenę o 20/40 rpm oraz wszystkie profile; obecne CRUISE 60/80/100 na jednym poziomie nie pokrywają pełnej specyfikacji.

## 9. MEDIUM — bramka Windows może uruchomić stary plik zamiast świeżego EXE

**Miejsce:** `tools/run_regression.py:50–52`.

W tym checkoutcie kompilator utworzył `assist_pipeline.exe`, a runner uruchomił istniejący `.build/regression-linux/obj/assist_pipeline` bez rozszerzenia i otrzymał WinError 193. Pełna bramka zatrzymała się przed SIL/Level-4/replay. Nie jest to błąd kompilacji firmware, lecz brak odporności runnera na pozostałości builda z innej platformy.

**Poprawka:** jawne rozszerzenie platformowe wyjścia i uruchamianie dokładnie tej ścieżki, którą linkowano; sprawdzić analogiczne runnery. Nie uzależniać poprawności od ręcznego czyszczenia katalogu.

**Odbiór:** bramka Windows w czystym katalogu oraz z obcym/starym plikiem bez rozszerzenia; odrębnie Linux.

## Weryfikacja faktycznie wykonana

- 53 zestawy hosta: PASS, w tym scenariusze V2. Manifest i pakowanie BL820: PASS.
- `verify_all.py --quick --require-target`: FAIL na uruchamianiu regresji (pkt 9). Następne etapy tego uruchomienia nie zostały wykonane; sanitizery nie były uruchamiane.
- Osobny kanoniczny builder `build_firmware.py --mode developer`, NORMAL, Arm GCC 13.2.1: PASS. FLASH 113912 B, RAM 39664 B. Wariant diagnostic w audycie NOT_RUN. Nie utworzono wydania ani nie flashowano roweru.
- Sonda kompilująca produkcyjne `ap2_profiles.c`, `battery_iq_cap.c`, `fast_iq_slew.c` i `ap2_math.h`: wyniki liczbowe pkt 1/2/5/7 potwierdzone.
- HW/odczucie jazdy/klik/realny prąd baterii: NOT_RUN. Nie potwierdzam ich na podstawie host PASS.

Dowody lokalne: `.build/audit-v2-gate.log`, `.build/audit-v2-target.log`, `.build/audit-v2-probe.log`, `.build/audit_v2_probe.c`. Są to artefakty audytu w katalogu generowanym, nie trwała regresja produkcyjna. Bin NORMAL SHA256: `ab5a63e074284fcaff9af614546cb6d508213c1400f3f4fd0908723d8eb8276f`.

## Co zachować i co przekazać agentowi

Zachować podział na moduły, jeden punkt publikacji, wspólny model profili oraz rozdział agresji i obciążenia. W audycie nie znaleziono potrzeby przepisywania FOC. Base/dynamic jest zaimplementowane jako asymetryczne estymatory i dodatni nadmiar demand nad base; sam fakt nazwania tego modelem cyklu nie zastępuje pomiarów tłumienia i odpowiedzi przejściowej.

Kolejność poprawek: 1–4 (referencja i ograniczenia), 5 i 7 (ciągłość/czas), 6/8/9 (obserwacja i dowody). Każdy reprodukowalny błąd powinien otrzymać trwały test regresyjny w swoim module lub na granicy integracji. Następnie pełna bramka oraz oba targety NORMAL/DIAGNOSTIC; dopiero potem ocena fizycznego start/stop, reverse, limiterów i odczucia jazdy. Nie odbierać całości wyłącznie na podstawie zielonego builda.

## Zakończenie wykonania

SCOPE EXECUTED: REPOSITORY_LOCAL review z indeksem projektu. Produkcja, konfiguracja i protokoły nie zostały zmienione. Dodano raport, wpisy indeksowe i generowane dowody audytu. Nie wykonano commitów. STARTED_AT: UNKNOWN; timestamp raportu odczytany z zegara systemowego. Poprawki są poza zakresem tego wykonania, zgodnie z poleceniem użytkownika.

NEXT EXACT ACTION: przekazać wykonawcy pkt 1–4 wraz z obowiązkiem reprodukcji na granicy pipeline → fast slew → PI; po poprawkach ponowić niezależny review.

## Korekta K1 — materiał reverse G5300, 2026-09-14T08:38:46+02:00

EXECUTION_ID: EXEC-EVD-ASSIST-V2-AUDIT-K1; AUTHORISED_BY: USER; TASK_ID: NONE. FINAL_STATUS: COMPLETED (analiza i dokumentacja). STARTED_AT: UNKNOWN. Zapis append-only; wcześniejsze wyniki sondy i testów pozostają historycznym dowodem.

Źródło i notatka do ponownego wykorzystania: [G5300 — mechanizmy, ograniczenia dowodów i mapowanie na EVistDrive](reference/g5300/README.md). Oryginalny przekaz użytkownika zachowano byte-for-byte w tym katalogu. Otrzymany materiał jest streszczeniem reverse innego modelu; pełne MD/BIN nie zostały niezależnie sprawdzone.

**Werdykt pozostaje CHANGES_REQUIRED. Żaden błąd implementacji nie został naprawiony tym dopiskiem.** Korekta dotyczy interpretacji wymagań i zakresu dalszych testów:

1. **Doprecyzowanie pkt 1:** trzeba rozróżnić immediate inhibit, zero targetu, zero referencji i zanik fizycznego prądu. Sonda dowodzi dodatniej referencji przez 200 ms; nie mierzy fizycznego prądu. G5300 nie dostarcza dowodu, że stock usuwa measured Iq w 10 ms, ani że natychmiastowa zmiana referencji gwarantuje brak kliku. Wymóg usunięcia dodatniego podtrzymania pozostaje oparty na pierwotnym zadaniu użytkownika (§19), a nie na stocku. Wskazanie pierwszego ISR w pkt 1 jest proponowanym kryterium software dla tego wymagania, **nie wynikiem reverse producenta ani zweryfikowaną rekomendacją elektryczną**. Nie utożsamiać go z natychmiastowym wyłączeniem PI/PWM. Ewentualny osobny krótki reverse-release musi mieć jawny kontrakt, uzasadnienie i dowód; nie uznawać obecnych 200 ms za sprawdzone na podstawie stockowego RUN fall.
2. **Rozszerzenie oceny STOP:** wcześniejszy audyt nie domyka opóźnienia od ostatniego fizycznego edge. Lokalny PAS ma już osobny true-stop i reset cadence, ale timeout wynosi 200…500 ms (`main.c:2952–2969`, `config.h:712–713`), wobec deklarowanych 25…208 ms innego targetu. Różnica sama nie stanowi dowodu błędu. Trzeba zmierzyć całkowity czas: detekcja → permission → trajectory → measured Iq. Nie poprawiać overrun wyłącznie zmianą `release_ms` i nie kopiować timeoutu 25 ms bez uwzględnienia geometrii M820.
3. **Rozdział RUN FALL / PEDAL RELEASE / REVERSE / SAFETY:** lokalne enumy FALL i RELEASE już są oddzielne, lecz korzystają z jednego `release_ms` w różnej semantyce (pełna skala vs czas do zera). Uzupełnić wytyczne strojenia i testy tych przyczyn osobno; jeśli potrzebna niezależna regulacja, rozdzielić parametry wraz z jawną migracją. Nie traktować deklarowanych 200 ms stockowego normal fall jako czasu zatrzymania, hamulca czy fault.
4. **Doprecyzowanie pkt 7:** krytyka elapsed-time dotyczy funkcji obiecującej niezmienną odpowiedź czasową w `ap2_math.h` i normalizacji pochodnych. Nie oznacza zakazu filtrów zależnych od zdarzeń. Lokalna kadencja już jest aktualizowana po pomiarze PAS; nie trzeba dodawać kolejnego filtra tylko dlatego, że stock ma event-driven IIR. W przekazanym streszczeniu współczynnik alpha wymaga wyjaśnienia: 1638/2^12≈0,40, nie 0,10.
5. **Co zachować:** osobnych właścicieli validity, activity/direction, dead-spot envelope, AUTO, trajektorii i PI/FOC. Długi envelope może mostkować przerwy w nacisku podczas wiarygodnego pedałowania, lecz nie może przedłużać zgody po true stop/reverse. Stockowe tolerowanie pojedynczego invalid nie jest automatyczną poprawką obecnego fail-safe inhibit.
6. **Pozostałe ustalenia 2–6 i 8–9 pozostają otwarte.** Materiał G5300 nie rozwiązuje resetu baterii Walk, pominiętego limitu poziomu, ograniczeń po slew, skoku AUTO, niezgodnego dekodera ani luk w bramce. Nie uzasadnia kolejnego przepisywania całości na „v3” ani odtwarzania legacy.

Weryfikacja K1: odczyt przekazanego tekstu, sprawdzenie jego wewnętrznych korekt i arytmetyki, porównanie lokalnych PAS/cadence/liveness oraz trajectory, zachowanie źródła z SHA256 i kontrola diff. BUILD/TEST RUNTIME: NOT_RUN (wyłącznie dokumentacja); HW: NOT_RUN; niezależny BIN reverse: NOT_RUN. Produkcja bez zmian. Pliki dotknięte: ten raport, katalog `docs/reference/g5300`, lokalny i globalny indeks historii.

NEXT EXACT ACTION K1: realizując poprawki audytu, dodać pomiar pełnej osi stop/reverse od fizycznego PAS do referencji i rzeczywistego prądu; wykorzystywać G5300 jako referencję architektury, a stałe dopiero po kwalifikacji dla M820.

## Korekta K2 — pełniejszy closure G5300 i appendix adresowy

Timestamp: 2026-09-14T11:49:20+02:00. Wykonanie i dowody: [EXEC-EVD-G5300-CLOSURE-REVIEW-001](reference/g5300/CLOSURE_REVIEW_2026-09-14_PL.md). Źródła kanoniczne pozostają w `external/datasheets`; notatka zawiera ich hashe, matrycę kompletności i priorytety dalszego reverse.

**Co zostało wyjaśnione względem K1:**

- D7EC nie jest ostatnią trajektorią current-command. Opisano E1E8 (`H+CE`→`M+2A2`→`F+16`) z osobnymi krokami +256/−16 na slot 10 ms. To skala wewnętrzna; nie dowód amperów lub czasu zaniku fizycznego Iq.
- 200 ms RUN FALL należy do pełnej skali D7EC. True stop/negative cadence mogą hard-clearować jego stan, a downstream może kontynuować inną trajektorię. Nie kopiować 200 ms jako release M820.
- Wyjaśniono brakujący czynnik 4 w filtrze: efektywne alpha≈1638/16384≈0,10 dzięki dodatkowej skali akumulatora. Pytanie K1 o samo alpha jest zamknięte na poziomie dostarczonego opisu. **Czas w ms pozostaje warunkowy**: poprzedni tekst zakładał 144 events/rev, nowy daje tabelę dla 72.
- AUTO 1…525% oraz denominator 200 opisano równaniem. Jednostka rider_output=200 **nie jest potwierdzona jako 200 W**; wcześniejszego przykładu w watach nie używać jako stockowej kalibracji.
- Istnieją opisane warstwy ograniczeń poza comfort slew: Q12 factor i hard MOE. Ich istnienie nie jest dowodem kompletnego przypisania kanałów battery/thermal/speed ani bezpośredniego clampowania referencji Iq. Nadal potrzebny pełny dataflow.

**Czego K2 nie zamyka:** brake input→konkretny inhibit, first reverse→MOE/prąd, current-command→A, dokładne Hz szybkiej ISR, pełne równania i recovery limiterów, pokrycie Walk/calibration, kompletny torque state4/state5 i seedy E1E8, wszystkie nazwy Parameter2. Listing adresowy jest mocniejszym materiałem niż streszczenie, ale bez BIN-a nie został niezależnie zdisasemblowany w tej sesji.

**Korekta liczbowa do źródła:** ratio 1→525% przy +10 pp/slot wymaga 53 aktualizacji, czyli budżetu 530 ms przy 10 ms/slot; 0,524 s jest modelem ciągłym. Opóźnienie od zdarzenia wymaga uwzględnienia fazy schedulera.

**Wpływ na review kodu:** obserwowany HEAD to `5b251b2`, poprzedzony `48194d2`; są także niezacommitowane zmiany w `src/fast_iq_slew.c` i `sim/evist_sil.c`. To wykonanie nie jest ponownym audytem tych poprawek. Historyczne CHANGES_REQUIRED ani opisane objawy z `9dc0b0a` nie powinny być automatycznie przenoszone na aktualny kod. Odbiór poprawek wymaga oddzielnego sprawdzenia ich implementacji i testów.

NEXT EXACT ACTION K2: wykorzystać nową matrycę reverse przy dalszej pracy; dla kodu M820 ponowić review poprawek na konkretnym HEAD/working tree, szczególnie końcowego Iq i aktualnych ograniczeń. Nie zlecać ponownego przepisania V2 na podstawie architektury innego kontrolera.
