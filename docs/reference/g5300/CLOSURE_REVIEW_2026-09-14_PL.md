# G5300 — kwalifikacja nowych dokumentów reverse i aktualny stan braków

REPORT_TIMESTAMP: 2026-09-14T11:49:20+02:00  
EXECUTION_ID: EXEC-EVD-G5300-CLOSURE-REVIEW-001  
TASK_ID: NONE; AUTHORISED_BY: USER; PROJECT_ID: EVD  
FINAL_STATUS: COMPLETED — odczyt, analiza, zapis wiedzy; nie implementacja.  
STARTED_AT: UNKNOWN. Wykonawca: Codex.

## Odpowiedź

**Nowe pliki częściowo domykają braki.** Największy postęp to opis downstream `E1E8`, rozdzielenie demand/current-command/PWM, skalowanie filtra kadencji i wyjaśnienie jednostek AUTO. Materiał wystarcza do lepszego określenia architektury i testów M820. Nie wystarcza do uznania wszystkich limiterów i fizycznego start/stop za zreversowane ani do kopiowania stałych prądowych.

Ten dokument jest bieżącą oceną kompletności wiedzy po wcześniejszym [intake](README.md). Zastępuje jego status „brak pełniejszego raportu” oraz niejasność alpha, ale zachowuje źródłowy tekst i historię wniosków.

## Źródła i granice weryfikacji

Przeczytano oba pliki w całości:

- [Raport closure](../../../../external/datasheets/G5300_REVERSE_CLOSURE_EVISTDRIVE_M820_20260914.md), 41280 B; SHA256 `8111ad9cf362f1a799b6f007b6d646de7c020936828fc7bacfbeab72a1c40a38`.
- [Appendix adresowy](../../../../external/datasheets/G5300_REVERSE_EVIDENCE_APPENDIX_20260914.md), 5181 B; SHA256 `106494e909a26ce03e929f779234b698e79c4c798d20ed8edfdfc00e0df81e29`.

Uwaga techniczna do ścieżek: źródła należą do wspólnego workspace `external/datasheets`, poza repo motor-controller-firmware. Nie tworzymy drugiej kopii raportu producenta w repo firmware.

Źródło identyfikuje BIN `MMG5300C4819F801001.5_72t TS_42t Speed sensor-500W_20250626.bin`, 149864 B, SHA256 `69bd7e533780dd108cd1624b1e5341288edb98e766d6dbc219cb44296be25e10`, vector base `0x0800A800` po 0x20 B nagłówka. W przejrzanym katalogu są dwa MD i `.gitkeep`, **nie ma samego BIN-a**. Tożsamość BIN-a jest deklaracją źródła, nie hashem policzonym przez nas z obrazu.

Raport i appendix pochodzą z tego samego reverse; nie są dwoma niezależnymi potwierdzeniami. Etykiety `CONFIRMED_BIN` pozostają etykietami autora. Lokalnie przyjmujemy **SUPPORTED / SOURCE_REPORTED_REVERSE**, z adresowymi dowodami w dokumencie, bez promocji do niezależnego CONFIRMED_REVERSE/HW. Sprawdzono spójność i arytmetykę, nie odtworzono instrukcji z BIN-a. Skrócone listingi z `...` nie są samodzielnym kompletnym programem do wykonania.

## Matryca domknięcia sześciu grup wytycznych

| Grupa dalszego reverse | Co dostarczono | Co nadal brakuje | Ocena |
|---|---|---|---|
| STOP / REVERSE / BRAKE / FAULT | Warstwy PAS, D7EC, E1E8, mailbox, lifecycle i MOE; stop/reverse hard-clear D7EC pod warunkami; osobne hard faults. Closure §3–8,11,18–21; appendix A–F,I–N | Pełne warunki gate `DC04`; pierwszy reverse edge→osobny inhibit/MOE; fizyczny brake→konkretny bit; measured Iq i rearm we wszystkich stanach | **Częściowo** |
| Trajektoria prądowa | Równania D7EC i E1E8, skale wewnętrzne, kroki i ownerzy; dwa różne slew; brak restartu timera w zwykłym E1E8. §9–11,13,18–19 | Kalibracja command→A; kompletna tabela przejść i seedów E1E8; warunki wszystkich wariantów START/release; pełna droga `F+16` do operandów PI i dokładne Hz ISR | **Znaczny postęp, nie pełne domknięcie** |
| Limitery | Upstream tabele, późniejszy Q12 factor, hard MOE, ryzyko pozostawania command powyżej desired. §12–13 | Battery/phase/power/thermal/speed: pełne równania, nazwy fizyczne, progi, histerezy, recovery; Walk/calibration; dokładny sygnał skalowany w C6B4 | **Częściowo** |
| Cadence filter i Gray | Dodatkowe skalowanie filtra oraz raw→rank. §6.1,14; appendix G | Dowód liczby accepted filter events/rev dla konfiguracji 72T; rekonstrukcja bit-exact filtra ze wszystkimi instrukcjami, znakami i resztą akumulatora | **Niejasność alpha i rank wyjaśniona; tau w ms warunkowe** |
| Torque / envelope / AUTO | Wewnętrzne progi, przybliżona tabela stanów, envelope, rider_output, ratio i readiness. §15–16 | State 4 i wejścia zewnętrzne state 5, pełny automat i jednostki fizyczne; dokładne warunki wszystkich gate/reset; weryfikacja bit-exact | **Częściowo; równania głównych gałęzi są** |
| Parameter2 i konfiguracja | Offset→runtime→efekt dla wielu pól; jawne UNKNOWN. §17; appendix O | Pełne mapowanie publicznych nazw, zakresy i konsumenci wszystkich pól; tabela nadal zawiera anonimowe soft-limit kanały | **Częściowo** |

„Domknięte” odnosi się tutaj do konkretnej wcześniejszej niejasności w dokumentacji, a nie do fizycznego działania M820.

## Wiedza, którą można już wykorzystać

### G53-01: D7EC nie jest finalnym Iq

Według closure `D7EC` ma accumulator 0…40960 i publikację `/10` w `R+EE`. Niżej `E1E8` ma osobny accumulator `H+CE`, normalnie +256/−16 jednostek command na slot 10 ms, publikowany jako `M+2A2` i kopiowany do `F+16`. D7EC phase0, E1E8 phase3; od publikacji D7EC do E1E8 nominalnie +3 ms.

Nie nazywać tych jednostek amperami. Przykład 1600→0 w 100 krokach E1E8 oznacza 1 s budżetu 100 aktualizacji, **nie dowód sekundowego ciągnięcia silnika**: niżej są dodatkowe ograniczenia i MOE. Nie sumować mechanicznie czasów dwóch kaskad jako czasu startu.

Zastosowanie do M820: oddzielne znaczenie demand i referencji, jawny owner finalnego Iq i niezależne egzekwowanie bezpieczeństwa. Nie kopiować dwóch akumulatorów finalnego Iq tylko dlatego, że stock ma kilka stanów pośrednich.

### G53-02: 200 ms dotyczy normalnego D7EC RUN FALL

Normalne `−2048/10 ms` daje 20 aktualizacji pełnej skali D7EC. True stop może omijać tę rampę przez gate `DC04`; negative cadence czyści D7EC osobną gałęzią. Pierwszy reverse edge publikuje najpierw cadence=0, a torque candidate może wtedy pozostać dodatni w niektórych gałęziach. Nie wyprowadzać uniwersalnego cut z samego zerowania starej kadencji.

Zastosowanie do M820: zachować immediate direction inhibit z własnego kontraktu, nie czekać na pomiar ujemnej rpm. Rozróżniać target, reference, current measured i PWM enable. G5300 nie ustala dopuszczalnego czasu reverse-release M820.

### G53-03: istnieją ograniczenia poza trajektorią komfortu

Closure opisuje `M+2A4` jako Q12 factor używany w 1-ms `C6B4`, obok sprzętowej blokady MOE. To wspiera potrzebę analizy całego łańcucha ograniczeń. Nie dowodzi jeszcze, że factor jest konkretnie limitem baterii lub że bezpośrednio zaciska wejście PI Iq — do tego potrzebna kompletna ścieżka operandów.

Zastosowanie do M820: oceniać wynik jedynego właściciela referencji po uwzględnieniu aktualnego pułapu. Sam spadek rider targetu nie dowodzi dotrzymania pułapu. Nie dodawać bocznego writer Iq.

### G53-04: alpha≈0,10 jest wyjaśnione dodatkową skalą akumulatora

Closure §14 podaje dodatkowy shift `(14-shift)`, czyli przy shift=12 publikację akumulatora przez `>>2`. Efektywne przybliżone alpha = `1638/16384 = 0,0999755859375`, tau≈9,4937 accepted events. Tym samym pytanie intake „skąd brakujący czynnik 4?” otrzymało odpowiedź. Wcześniejsze ostrzeżenie przed bezpośrednim `1638/4096` pozostaje słuszne, ale alpha nie jest już niewyjaśnione.

To model przybliżony: pseudokod ze znakiem `~=` nie zastępuje bit-exact aktualizacji stanu z zachowaniem ułamkowej reszty.

### G53-05: AUTO 200 to rider_output, nie potwierdzone waty

Closure §16 podaje dwa dzielenia całkowite w rider_output oraz interpolację ratio 1…525% z denominator=200. Rozstrzygnięcie: **nie przenosić etykiety „200 W”**. Dla M820 human power powinno pochodzić z własnej kalibracji siły, korby i kadencji, jeżeli taki parametr ma być fizycznymi watami.

Envelope według źródła ma natychmiastowy rise i decay `floor(previous*k/(k+1))` co 10 ms, gdzie `k=8*cadence` dla dodatniej kadencji. To bardzo długie podtrzymanie przy wysokiej kadencji, a nie uniwersalny krótki filtr ani stop timer. Nie przyjmować go jako gotowego zamiennika lokalnego base/dynamic bez replay.

## Dwie korekty i dalsze niejasności w nowym źródle

1. **72T nie rozstrzyga samodzielnie events/rev.** Poprzedni handoff podawał przy 60 rpm tau≈66 ms (zgodne z 144 events/rev). Nowy closure §14 podaje tau≈132 ms **dla założonych 72 transitions/rev**. Obie wartości są arytmetycznie poprawne przy swoich założeniach; źródła różnią się geometrią/częstością zdarzeń. Pole tooth count=72 i stała 144 we wzorze timeoutu nie wystarczają do rozstrzygnięcia. Potrzebny dataflow od fizycznych A/B przez rank/accepted event do wywołania filtra oraz wzór obliczania rpm. Do tego czasu przechowywać tau w events; tabeli ms nie traktować jako zakwalifikowanej dla stocku ani M820.
2. **Ratio 1→525%:** +10 pp na aktualizację wymaga `ceil(524/10)=53` aktualizacji, nie dokładnie 0,524 s. Budżet 53 okresów 10 ms to 530 ms; czas od asynchronicznej zmiany targetu zależy od fazy pierwszego slotu. 0,524 s jest przybliżeniem ciągłym. Analogicznie tablica D7EC 252/…/27 aktualizacji jest poprawna arytmetycznie: 2,52…0,27 s jako liczba okresów. Nie utożsamiać jej bezwarunkowo z opóźnieniem od fizycznego edge.
3. **Warunkowe gałęzie:** fragment `DC04` zawiera pominięte warunki, a state4 torque i seedy E1E8 nie mają kompletnej tabeli. Raport uczciwie sygnalizuje warunkowość, lecz nie zamyka całego automatu tylko przez zamieszczenie adresu.
4. Odwołania źródła do „pomiarów w §17” należy czytać jako §21 (lista pomiarów); §17 to parametry. Nie modyfikujemy dostarczonych źródeł, zapisujemy sprostowanie tutaj.

## Dalszy reverse — kolejność

1. **P0 / statycznie:** zidentyfikować pełne warunki `DC04`, producentów inhibit `S+1C/56/58/59`, gałąź po pierwszym reverse i state/mode dispatch do MOE. Odtworzyć `F+16`→właściwy operand PI oraz C6B4→dokładny sygnał po mnożeniu. Osobno ustalić częstotliwość ISR z konfiguracji timera i zegarów.
2. **P0 / stanowisko lub skalowanie potwierdzone dataflow:** command→A, measured Iq, brake→inhibit/MOE oraz czasy rearm. Śledzić jednocześnie PAS, command, reference/PI, MOE, prąd i DC bus. Nie wnioskować o zaniku prądu wyłącznie z MOE off.
3. **P1:** każdy kanał soft limit osobno: sygnał fizyczny→próg→histereza→stan→ograniczenie przed/po slew→recovery. Powtórzyć dataflow dla Walk i calibration. Clamping integratora PI nie jest dowodem back-calculation z nasycenia SVPWM.
4. **P1:** domknąć licznik accepted events/rev i filtr bit-exact; pełne state4/state5 torque, guardy i seedy E1E8. Przygotować wektory wejście/stan/wyjście z oryginalnego kodu lub emulacji, nie tylko pseudokodu.
5. **P2:** nazwy publiczne Parameter2 i fizyczna skala rider_output. Znany efekt offsetu można dokumentować bez zgadywania nazwy; brak nazwy nie blokuje użycia poprawnie zrozumianego mechanizmu.

## Wpływ na projekt i audyt

- Aktualizacja audytu: [korekta K2](../../AUDIT_ASSIST_PIPELINE_V2_2026-09-14_PL.md). Nie zastępuje ponownego review kodu.
- Repo firmware jest już na `5b251b2`, po `48194d2` i `5b251b2` deklarujących poprawki wcześniejszego audytu. W momencie odczytu były cudze zmiany `src/fast_iq_slew.c` i `sim/evist_sil.c`. **Nie oceniano ich, nie nadpisano i nie przypisano im PASS/FAIL.** Audyt bazowy dotyczy `9dc0b0a`.
- Zmiany tylko dokumentacyjne: bieżąca notatka, router G5300, addendum audytu, link w architekturze i indeksy historii. Źródłowe dwa MD pozostały nietknięte.
- Weryfikacja: przeczytanie obu źródeł, SHA256, sprawdzenie arytmetyki alpha/tau, ratio oraz ośmiu kroków Acceleration; kontrola linków i diff. BUILD/RUNTIME TEST/HW/BIN DISASSEMBLY: NOT_RUN, zgodnie z zakresem przyjęcia wiedzy.

NEXT EXACT ACTION: dalszemu reverse przekazać P0 i rozbieżność 72/144 events; agentowi M820 przekazać mechanizmy G53-01…05 i wymóg niezależnego review bieżących poprawek, bez kopiowania skal stocku.
