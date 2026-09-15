# Audyt konfiguracji V2, CANable i gotowości wydania

REPORT_TIMESTAMP: 2026-09-14T13:36:46+02:00  
EXECUTION_ID: EXEC-EVD-CONFIG-CANABLE-AUDIT-001  
TASK_ID: NONE (ad-hoc); AUTHORISED_BY: USER; TARGET: M820  
STATUS: CHANGES_REQUIRED; HW: NOT_RUN; RELEASE: NOT_APPROVED

## Werdykt i zakres

**Nie jest to obecnie kompletny, gotowy do wydania zestaw firmware + konfigurator.** Poprawiony tor wspomagania przechodzi bramkę PC i kompilację targetu, ale integracja konfiguracji ma odtwarzalne błędy. Najważniejszy: firmware odrzuca własny domyślny bank V2 przy ponownym wczytaniu. CANable pozostał przy modelu starego wspomagania. Odczyt bajtów i zielone testy obu repozytoriów nie oznaczają zgodności ich znaczenia.

Audyt obejmuje firmware `226cb54ec589d61359c30e3211a37d4d6a6ee9ad` (poprawki `48194d2`, `5b251b2`, `dc23faf`) oraz CANable `04c7c9f13ed932920c6d703fb317879bcda89b5a`. W trakcie weryfikacji obecne były cudze niezatwierdzone zmiany `tests/host/run-host-tests.ps1` i nowy `tests/host/main_standstill_wiring_host.c`; wynik 55 zestawów hosta dotyczy tego drzewa roboczego, nie samego czystego commita. Nie zmieniano kodu produkcyjnego ani konfiguratora, nie wykonywano flashowania. Próby pomocnicze zapisano w `.build/`.

Poprzedni audyt: [tor V2, K1/K2](AUDIT_ASSIST_PIPELINE_V2_2026-09-14_PL.md). Referencja producenta: [kwalifikacja G5300](reference/g5300/CLOSURE_REVIEW_2026-09-14_PL.md). G5300 pozostaje innym targetem; jego reverse nie zastępuje pomiarów M820.

## Co zostało ze starego rozwiązania

Usunięto stary algorytm przygotowania wspomagania, m.in. osobne moduły dynamics, extended boost, start, cadence compensation, pedal assist gate, power curve i ride session. Nie ma przełącznika przywracającego cały stary tor. Pozostały transport CAN, banki ustawień, ich serializacja, starsze bloki parametrów, kalibracja, HMI, Walk oraz sterowanie silnika/FOC. Pozostały też obliczenia FAST/RUN w `torque_input.c`; ich obecność nie oznacza, że stary RUN steruje nowym zapotrzebowaniem V2.

Stare identyfikatory są mapowane na nowe profile podczas obliczeń:

| Zapisany stary tryb | Zachowanie w V2 |
|---|---|
| 1 Power Linear, 5 Torque | TRAIL |
| 2 Power Progressive, 3 eMTB, 6 Power Curve | SPORT |
| 4 eMTB Custom | Jest gałąź mapowania na SPORT, ale walidator banku nie przyjmuje ID 4 |

To nie jest zachowanie dawnych charakterystyk. Zapisany identyfikator nadal może pozostać stary, a obliczenia korzystają już z V2. Zachowano m.in. ograniczenie mocy, ograniczenie Iq i progi nacisku; stare współczynniki modelu i boosty nie są odtwarzane. Nazwy pięciu pozycji HMI w aplikacji należy odróżniać od wybranego profilu obliczeniowego.

## Ustalenia wymagające poprawy

### C1 — HIGH: firmware nie przyjmuje własnych banków V2 i nie odtwarza ich przy rozruchu

`src/assist_modes.c:226` (`bank_mode_valid`) dopuszcza tylko 1, 2, 3, 5, 6. Nowe domyślne banki zawierają tryby 7–12. `assist_modes_apply_bank_blob()` sprawdza każdy rekord i odrzuca cały bank (`:665`). Ten sam parser jest używany przy odtwarzaniu konfiguracji w `src/main.c:1167`; błąd pozostawia wartości domyślne.

**Odtworzenie na kodzie produkcyjnym:** `assist_modes_init()` → `assist_modes_serialize_bank(0, blob)` → `assist_modes_apply_bank_blob(blob, 255)` daje `false`. CRC nie był zmieniany, nie uczestniczył w tym CANable. Log: `.build/audit-config-probe.log`, źródło: `.build/audit_config_probe.c`, blob: `.build/audit-bank-v2.bin`.

Skutek: aktualizacja profili do RAM jest odrzucana, a poprawne zapisanie bajtów V2 do flash nie zapewnia odtworzenia po restarcie. Parametry Walk też są w tym banku, więc podlegają temu samemu odrzuceniu. W zwykłym zapisie CANable banki poprzedzają tuning, przez co błąd banku zatrzymuje również dalszy zapis tuningu.

**Poprawka/odbiór:** zdefiniować listę obsługiwanych ID i politykę migracji; zaakceptować wszystkie sześć profili V2. Test każdego banku: domyślne → serialize → apply → serialize; edycja → RAM → flash → reset → readback. Sprawdzić stare wersje banków oraz odrzucenie nieznanych ID bez częściowej mutacji. Nie obchodzić błędu przez przestawienie wszystkich poziomów na stare tryby.

### C2 — HIGH: CANable przedstawia zero jako wyłączenie, firmware traktuje je jako brak dodatkowego limitu

`canable-web/ui/js/evistdrive/profiles.js:249`: przy zerze maksymalnego momentu opis „Assist is switched off at this level.” Pole jest kodowane jako `max_iq_pct`. `assist_modes_level_iq_limit()` (`src/assist_modes.c:323`) nakłada limit tylko dla wartości większych od zera i mniejszych od 100. Próba dla `max_iq_pct=0`, global=700, phase=700 zwraca **700**, nie zero.

**Poprawka/odbiór:** uzgodnić jeden kontrakt zera w firmware, migracji i UI. Zachować intencję istniejących ustawień wyłączających wspomaganie; jeśli potrzebny jest osobny stan „domyślny limit”, nie utożsamiać go z wyłączeniem. Testować 0, 1, 50, 100, restart i przejście z dodatniego Iq. Przeliczenie procentów na Nm musi mieć udokumentowaną podstawę; sam test hosta nie kalibruje momentu fizycznego.

### C3 — HIGH: zapis bez edycji zmienia domyślną dynamikę; poprawka samego C1 ujawni ten błąd

W V2 `iq_rise_fast_ms=0` oznacza zachowanie czasu profilu, np. ECO 500 ms. Serializer CANable (`canbus.js:984`, ograniczenie ramp) zmienia zero na 20 ms. Parser firmware (`assist_modes.c:775–784`) również ogranicza tę wartość do minimum 20 ms.

Próba rzeczywistego domyślnego blobu firmware → parser CANable → serializer CANable: tryby [7,8,9,10,12], attack **0 → 20**, łącznie 27 różnych bajtów. To wynik serializacji bez edycji w UI; nie wszystkie różnice muszą oznaczać aktywną funkcję V2. Log: `.build/audit-canable-probe.log`; wynik: `.build/audit-bank-canable-roundtrip.bin`. Obecnie C1 odrzuca taki bank; nie twierdzimy, że ta próba już zmieniła zachowanie roweru.

**Poprawka/odbiór:** zachować semantykę „użyj profilu” po obu stronach. Zwykłe read → save bez edycji nie może zmieniać efektywnego profilu. Test porównuje także rozstrzygnięte nastawy C, nie tylko strukturę JSON lub poprawny CRC. Osobno sprawdzić fixed/AUTO, zera i jawne override oraz restart.

### C4 — HIGH: konfigurator nie obsługuje wyboru i znaczenia nowych profili

`canable-web/ui/js/evistdrive/common.js:43` definiuje tylko stare tryby 1,2,3,5,6. Edytor `profiles.js` nie ma opcji 7–12; ustawienie wartości nowego trybu w takim select nie zapewnia poprawnego wyboru. Wybranie starej opcji zapisuje stary identyfikator. Nie stwierdzono, że sam odczyt automatycznie przepisuje ID. Sprawdzenie kompatybilności nie traktuje nieznanego ID jako bezwarunkowej blokady. Stare wykresy, presety i opisy parametrów nie opisują V2.

Bank nadal ma schema 8 i 255 B. Sam numer schema 8 nie identyfikuje starego/nowego modelu zachowania.

**Poprawka/odbiór:** jawna informacja o możliwościach/generacji profili, sześć nowych trybów, opis rzeczywiście używanych parametrów. Nieznany profil ma być zachowany i pokazany jako nieobsługiwany, bez cichej konwersji. Nie wystarczy dopisać sześciu nazw do select. Test macierzy stary/nowy firmware × stary/nowy bank, prawdziwego odczytu i zapisu, presetów, importu i trybów AUTO. Rozszerzenie kontraktu nie może bez planu przekroczyć limitu 255 B.

### C5 — HIGH dla diagnostyki wydania: CANable odrzuca nowy pakiet 0x6029

Firmware (`src/CAN_Display.c:1152`) nadaje wersję 7. `canable-web/bafang-parser.js:545` dopuszcza wersje do 6; próba nagłówka DG/ver7 zwraca `Bad diagnostics magic/version`. Nowe pola mają również nowe znaczenie (rider demand, base/dynamic, aggression/load, stan PAS, AUTO, profil). Samo dopuszczenie numeru 7 pokazywałoby dane pod starymi etykietami.

**Poprawka/odbiór:** osobny dekoder i widok v7, testy długości/CRC/pól/jednostek, zachowanie v6. Sprawdzić również odbiorców strumienia ride telemetry schema 2. Naprawiony dekoder offline firmware nie jest dowodem naprawy aplikacji WWW. Surowy logger CANable pozostaje użyteczny do przechwycenia ramek i późniejszego dekodowania narzędziami firmware.

### C6 — MEDIUM: stare ustawienia wyglądają na czynne, choć utraciły wpływ na V2

Tabela poniżej określa konsumentów pól. Usunięcie starych mechanizmów wynikało ze zleconej przebudowy; defektem integracji jest pozostawienie obietnicy ich działania w konfiguratorze. `cadence_step`, `run_deadband_mv`, `assist_hold_ticks` i `min_iq_pct` w globalnym tuningu nie mają aktualnych konsumentów sterowania poza modułem przechowywania ustawień. Okno RUN nadal zmienia stary pomiar/diagnostykę, ale nie wejście nowego modelu zapotrzebowania.

**Poprawka/odbiór:** nieaktywne pola wycofać z edycji V2 i oznaczyć przy imporcie starych ustawień. Każda widoczna kontrolka ma mieć wskazane pole protokołu, konsumenta w produkcyjnym C, jednostkę i test rzeczywistego wpływu. Zachowanie bajtów dla zgodności wstecznej jest dopuszczalne; nie wolno przedstawiać go jako zachowania starej funkcji.

### C7 — MEDIUM: zapis może częściowo się wykonać mimo komunikatu „Nothing was made permanent”

`canable-web/ui/js/evistdrive/global-actions.js`: `performSave('permanent')` najpierw wykonuje `writeLegacyBlocks()` oraz komendę SOC, potem `writeRideSettings()`. Błąd banku powoduje komunikat „Nothing was made permanent”, chociaż wcześniejsze bloki legacy mogły już zostać zapisane trwale. C1 sprawia, że ta ścieżka jest szczególnie istotna dla domyślnych banków V2. Jest to ustalenie przepływu kodu, nie pomiar zapisu na podłączonym sterowniku.

**Poprawka/odbiór:** wstępna walidacja całego zakresu przed pierwszym zapisem; komunikat opisujący faktyczne etapy i ewentualny częściowy sukces. Nie obiecywać atomowości istniejącego protokołu. Test: udany zapis legacy, odrzucony bank, brak tuningu/SAVE_BANKS i poprawna informacja o wcześniejszych zmianach.

## Co jest konfigurowalne

„Aktywne w C” oznacza konsumenta w obecnej implementacji, a nie potwierdzenie transportu na rowerze. **Wszystkie pola banków V2 są obecnie objęte blokadą zapisu C1**, także Walk. Tabela opisuje kontrakt potrzebny do naprawy UI, nie zapewnienie, że obecny przycisk Save zapisze te parametry.

| Ustawienie | Obecne znaczenie w firmware / status CANable |
|---|---|
| Profil ECO/TRAIL/SPORT/SPORT+/AUTO/AUTO SPORT+ | Runtime V2; brak właściwego wyboru i zapisu w CANable |
| Maksymalna moc poziomu | Aktywny dodatkowy sufit, również przy starym ID; 0 nie dokłada limitu. Nie podnosi wbudowanego maksimum profilu |
| Maksymalny Iq poziomu | Niezerowe procenty ograniczają Iq; zero ma sprzeczne znaczenie UI/C, C2 |
| Minimalny nacisk i nacisk startowy | Aktywne progi; przechowywane w banku, C1 |
| `support_ratio_pct` | Dla V2 korekta siły `assist_trim_pct`, nie dawny stosunek mocy silnik/człowiek. Przy starych ID ignorowany |
| `iq_rise_fast_ms` | Dla stałego V2 czas attack; nie dawna rampa zależna od prędkości. Zero=profil, C3. AUTO ignoruje ręczne czasy |
| `smooth_start.duration_ms` | Dla stałego V2 czas start; flaga `enabled` nie steruje nowym startem. Zero=profil |
| `release_ms` | Dla stałego V2 czas release; zero=profil. AUTO używa swojej dynamiki |
| Pozostałe trzy rampy Iq | Przechowywane, nie sterują dynamiką V2 |
| Startup Boost, Extended Boost, assist without rotation | Dawne mechanizmy nie działają w V2 |
| Cadence compensation, stare parametry eMTB, power curve, torque factor | Nie odtwarzają starego wspomagania; pola historyczne |
| Globalne start steps | Aktywny próg kroków PAS |
| Globalna skala pełnego nacisku | Aktywna normalizacja obciążenia w V2 |
| Długość korby | Wpływa na obliczenie mocy rowerzysty w telemetrii; nie jest regulatorem mocy wspomagania |
| Globalne okno RUN | Pozostały estymator pomiarowy/diagnostyczny; nie kształtuje wejścia zapotrzebowania V2 |
| Globalne cadence step/deadband/hold/min Iq oraz dawne rampy | Nie stroją nowego toru; wymagają uporządkowania ekranu Dynamics |
| Walk: 10–60 rpm, limit prądu, odcięcie od koła, latch/timeout | Konsumenci pozostali w main; wspólny bank, więc zapis blokuje C1 |
| Kalibracja torque, parametry baterii/SOC, koła, identyfikacja | Zachowane ścieżki protokołu; ten audyt nie potwierdza każdej operacji na fizycznym urządzeniu |
| Globalny limit prądu fazowego | Przy rozruchu main nadpisuje zapisane pole przez `PH_CURRENT_MAX=700`; nie traktować dowolnego zapisanego numeru jako efektywnego limitu |
| Base/dynamic, base hold, wpływy aggression/load, charakterystyki i reguły AUTO | Zasadniczo tabela/kod `ap2_profiles.c` i estymatorów; brak pełnego strojenia przez CANable |

Wbudowane maksima stałych profili to ECO 250 W, TRAIL 450 W, SPORT 650 W, SPORT+ 900 W; są to parametry modelu, nie pomiar gwarantowanej mocy na kole. Nie ma potrzeby wystawiać wszystkich wewnętrznych parametrów w UI: najpierw spójny zestaw podstawowy, reszta jawnie jako nastawy firmware.

## Wyniki niezależnej weryfikacji poprawek i ograniczenia dowodów

| Próba | Wynik |
|---|---|
| `tools/verify_all.py --require-target` | PASS, 55 zestawów hosta, SIL/fuzz/FOC/Level-4/replay |
| Bramka pulsacji | 14 przypadków PASS; najgorszy stosunek około 0,409 przy limicie 0,6 |
| W1 replay | 6/6 BEHAVIOR_ACCEPTED; brak przypiętych hashy wyniku nie jest bitową zgodnością starego toru |
| ASan/UBSan w tej sesji Windows | SKIPPED: brak linkowalnego runtime. Nie przypisujemy tej sesji PASS sanitizers |
| NORMAL, ARM GNU 13.2.1 | PASS; BIN 115760 B, RAM 39688 B (80,75%) |
| DIAGNOSTIC, ARM GNU 13.2.1 | PASS; BIN 143880 B, RAM 30440 B (61,93%) |
| `npm test` CANable | PASS istniejących testów; nie obejmują wykrytych kontrprzykładów V2 |
| Produkcyjny C: zapis własnego banku | FAIL: apply=0, 255 B — C1 |
| Firmware blob → parser/serializer CANable | Niezgodność: attack 0→20 — C3 |
| CANable: diagnostyka v7 | FAIL na kontroli wersji — C5 |
| Fizyczny CANable, flash/restart, stanowisko i jazda M820 | NOT_RUN |

Przejrzane poprawki wcześniejszych ustaleń (reverse, stan limitera Walk, niezerowy limit Iq poziomu, twarde sufity po dynamice, interpolacja charakterystyki AUTO, dekoder offline, czas filtrów, bramki ripple/replay, runner Windows) mają implementację i przechodzą obecną regresję PC. **To odbiór zakresu poprawionego kodu/testów PC, nie całej integracji ani wydania.** C2 ujawnia nadal nieuzgodnioną semantykę zera limitu, a C5 dotyczy innego odbiorcy niż naprawiony dekoder offline.

Oś STOP w symulacji: od krawędzi do detekcji 197,75 ms; od odebrania permission do zera referencji 187 ms; od krawędzi do zaniku dodatniego napędu 206,25 ms. Są to różne sygnały i przedziały, nie składniki do prostego sumowania. Reverse: referencja odcinana od razu po rozpoznaniu, zanik dodatniego napędu modelu 0,50 ms. **To wynik modelu elektrycznego/SIL, nie zmierzony prąd fizycznego M820.** Nie zamyka HW i nie przenosi stałych G5300.

Logi lokalne: `.build/audit-config-release-gate.log`, `.build/audit-config-diagnostic.log`, `.build/audit-canable-tests.log`, `.build/audit-config-probe.log`, `.build/audit-canable-probe.log`. Katalog `.build` jest roboczy; raport utrwala wyniki i reprodukcję, ale logi nie są automatycznie trwałym archiwum wydania.

SHA256 developerskiego NORMAL: `5d738a8da6ac6b84223d25a855e539a7f1d55581bdc25a868114a0c1ca0efdc9`.  
SHA256 developerskiego DIAGNOSTIC: `238600b7aed901f188c24a144a1648cf11a7cadeeee81b1ec639693642ff11db`.

## Wytyczne dla agenta i kolejność dalszej pracy

1. **Najpierw naprawić kontrakt konfiguracji C1–C3.** Dodać reprodukcje do stałej regresji. Ustalić migrację starych banków i znaczenie zer; testować efektywne ograniczenia/dynamikę oraz restart. Zmiana walidatora bez C3 nie jest kompletną poprawką.
2. **Następnie dostosować CANable C4–C7 do uzgodnionego kontraktu.** Nowe profile, tylko aktywne kontrolki, zrozumiała migracja, poprawny odczyt diagnostyki i uczciwe komunikaty częściowego zapisu. Każda kontrolka ma mieć ślad UI → wire → konsument C. Zachować obsługę starszego firmware.
3. **Wspólna bramka integracyjna:** bank produkcyjnego C → rzeczywisty JS parser/serializer → C apply → zapis/odtworzenie → efektywny profil. Oba banki, każdy profil, AUTO, zero/default/explicit, stare banki, błędny CRC i nieznany ID. Osobno test UI zapisu częściowego i fixture diagnostyki v7. Te testy mają upaść na obecnych błędach.
4. **Po poprawkach pełna regresja** firmware i CANable, sanitizers w środowisku z działającym runtime, oba buildy ARM 13.2.1. Utrwalić dokładny commit obu repozytoriów, stan drzewa i wyniki. Nie mieszać tego z nową przebudową FOC ani kolejną zmianą modelu wspomagania.
5. **Kandydat do testów stanowiskowych:** na rzeczywistym CANable odczyt → zmiana → ACK → readback → trwały zapis → restart. Następnie kontrolowane próby start/stop, reverse, hamulec/fault, limitery i Walk, w tym zachowanie po błędzie komunikacji. Rejestrować surowy log oraz rozdzielać cel Iq, referencję i prąd zmierzony.
6. **Dopiero po stanowisku jazda diagnostyczna i strojenie.** Log realnego problemu wprowadzać jako replay przed jego poprawką. Ocenić komfort, temperaturę, odcięcie i wznowienie pod obciążeniem. Wynik SIL nie wystarcza do oznaczenia tego etapu jako PASS.
7. **Wydanie produkcyjne:** odbiór kodu i HW, jednoznaczna zgodna wersja CANable, konfiguracja startowa/migracja oraz numer BIN i SHA256. Kanoniczny build według zasad projektu przez `--mode auto`; testowe buildy tego audytu to `DEV-NONCANONICAL`. W zastanym `releases/` są 0.603 i 0.604 sprzed przebudowy V2 — nie identyfikują tego audytowanego kodu.

NEXT EXACT ACTION: wykonać ograniczony rework konfiguracji firmware C1–C3 z testem roundtrip i restartu; następnie skoordynowany rework CANable C4–C7. Nie rozpoczynać strojenia jazdą jako sposobu diagnozowania uszkodzonego zapisu ustawień.

## Wytyczne UX dla nowego interfejsu CANable

Interfejs ma pozwalać zrozumieć działanie profilu bez znajomości nazw `attack_ms`, `dynamic_gain` czy `base_hold`. Sama zmiana etykiet nie wystarczy. Każda kontrolka musi jednocześnie pokazywać nazwę użytkową, krótki opis skutku, wykres oraz podgląd wpływu zmiany.

### Widok podstawowy

Dla każdego poziomu pokazać kartę profilu z czterema elementami:

1. **Profil** — ECO, TRAIL, SPORT albo SPORT+.
2. **Reakcja / akceleracja** — spokojna, normalna, szybka albo bardzo szybka. Pod opisem: „jak szybko pomoc rośnie, gdy mocniej naciskasz na pedały”.
3. **Podtrzymanie** — niskie, średnie, wysokie albo bardzo wysokie. Pod opisem: „jak długo pomoc utrzymuje się po zmianie nacisku i pod obciążeniem”.
4. **Limit mocy** — suwak/liczba w W z wyjaśnieniem, że obniża maksymalną moc profilu, ale jej nie podnosi ponad limit wbudowany w firmware.

Przy każdej zmianie interfejs powinien pokazać prosty komunikat skutku, na przykład: „Reakcja szybsza: silnik wcześniej zwiększy pomoc po mocniejszym naciśnięciu”. Wartości należy prezentować jako zakresy i słowa opisowe, nie jako wartości techniczne bez kontekstu.

### Wykresy i grafika

Karta profilu powinna zawierać wykres „nacisk na pedały → pomoc silnika”. Oś pozioma to nacisk rowerzysty, oś pionowa to względna pomoc/Iq. Na wykresie należy narysować:

- krzywą bieżącego profilu;
- krzywą po zmianie jako linię podglądu;
- oznaczenie limitu mocy;
- prostą ilustrację czasu reakcji po nagłym zwiększeniu nacisku.

Drugi mały wykres powinien pokazywać „zmiana nacisku → czas narastania i odpuszczania”. Nie należy udawać, że wykres przedstawia waty na kole lub dokładny prąd fazowy; ma wyjaśniać względne zachowanie profilu. Jednostki i założenia muszą być widoczne przy wykresie.

Pod wykresem można dodać trzy scenariusze podglądu: lekkie ruszanie, normalne przyspieszanie i mocne podjazdowe naciśnięcie. To daje użytkownikowi intuicję, jaki efekt przyniesie zmiana, zanim zapisze ustawienia.

### Tryb zaawansowany

Powinna istnieć opcja „Strojenie zaawansowane”, domyślnie zwinięta. Po jej otwarciu użytkownik może niezależnie ustawić:

- `Attack` — czas narastania reakcji w ms;
- `Start` — łagodność i czas wejścia wspomagania przy ruszaniu;
- `Release` — czas odpuszczania po zmniejszeniu nacisku;
- `Dynamic gain` — siłę odpowiedzi na zmianę nacisku;
- `Base share` — udział stabilnej, podtrzymywanej pomocy;
- `Base hold` — czas podtrzymania po zmianie obciążenia;
- limit mocy w W oraz limit Iq w %.

Każdy parametr musi mieć suwak, zakres bezpieczny, wartość bieżącą, przycisk przywrócenia profilu oraz aktualizowany wykres. Przy suwaku należy pokazać opis „więcej/mniej” i skutek, na przykład: „mniejszy Attack = szybsza reakcja” albo „większy Dynamic gain = mocniejsza odpowiedź na nagłe zwiększenie nacisku”.

Nie należy udostępniać pól, które firmware przechowuje, ale ignoruje w V2 (stare boosty, stare rampy, dawny power curve, nieaktywne globalne hold/deadband). Pola historyczne mogą być pokazane tylko w sekcji migracji jako „zachowane dla zgodności, bez wpływu na V2”.

### Interakcja typu suwak + wykres

Podstawową kontrolką powinien być suwak, podobnie jak w dojrzałych konfiguratorach napędów. Użytkownik przesuwa wartość, a wykres i opis zmieniają się natychmiast, bez zapisu do roweru. Przy każdym suwaku należy pokazać wartość liczbową, jednostkę, bezpieczny zakres oraz przycisk „Profil bazowy”.

Układ karty profilu:

```text
Reakcja / akceleracja       [ spokojna ───●──── szybka ]   350 ms
Podtrzymanie                [ krótkie  ─────●── długie ]   100 %
Limit mocy                  [ 250 W ─────●────────── ]     450 W

       wykres nacisk → pomoc silnika
       linia szara: profil bazowy   linia kolorowa: podgląd
       [Przywróć profil]   [Anuluj]   [Zastosuj]
```

Suwak opisowy może być jednocześnie powiązany z wartością techniczną. Na przykład „Reakcja: szybka” przesuwa `attack_ms` w ustalony punkt, a otwarcie zaawansowane pokazuje dokładne milisekundy. Zmiana `Dynamic gain` powinna aktualizować nachylenie/wygięcie krzywej, a zmiana `Attack` — szerokość czasową narastania na wykresie. Zmiana limitu mocy powinna obniżać zaznaczony sufit, bez zmiany samej krzywej poniżej tego limitu.

Wykres musi mieć tryb „przed/po”, podpowiedź po najechaniu oraz opis słowny aktualnego skutku. Kolory i skala powinny być stałe pomiędzy profilami, aby użytkownik mógł porównać ECO, TRAIL, SPORT i SPORT+ bez dodatkowego tłumaczenia. Wartości krańcowe wymagają czytelnego ostrzeżenia o skutku, ale nie powinny być zastępowane niejasnym komunikatem technicznym.

Podstawowe suwaki mają zmieniać tylko bezpieczne, zatwierdzone kombinacje. Tryb zaawansowany może odsłaniać niezależne parametry, ale przy konflikcie (np. bardzo szybka reakcja i długi start) interfejs musi pokazać przewidywany efekt oraz ograniczyć wartości do zakresów zaakceptowanych przez firmware. Podgląd jest lokalny; dopiero „Zastosuj” wysyła bank do RAM, a „Zapisz trwale” wykonuje osobny krok.

### Zasady zapisu i migracji

Przed wysłaniem interfejs powinien pokazać podsumowanie różnic: profil, reakcja, podtrzymanie, limit mocy oraz wartości zaawansowane przed/po zmianie. Odczyt i zapis bez zmian musi przejść test „no-op”: wykres i efektywne wartości profilu pozostają identyczne. Nie wolno pokazywać „zapisano” po samym wysłaniu do RAM; należy osobno oznaczyć „aktywne tymczasowo” i „zapisane trwale”.

Zmiana profilu starego banku musi być widoczna jako migracja, np. „Power Linear → TRAIL (stara charakterystyka nie jest zachowana)”. Nieznany profil ma pozostać odczytany i oznaczony jako nieobsługiwany; interfejs nie może po cichu zastąpić go innym profilem.

### Kryteria odbioru UX

Odbiór interfejsu wymaga: testu użytkownika bez dokumentacji technicznej, porównania wykresów przed/po każdej kontrolce, roundtripu firmware ↔ CANable bez zmiany ustawień, zachowania wartości po restarcie oraz testu migracji starych banków. Współczynniki wykresu muszą być generowane z tej samej tabeli profili co firmware albo oznaczone jako ilustracyjne; nie wolno tworzyć drugiego, rozbieżnego modelu zachowania w JavaScript.

## Zmiany wykonane w tym audycie

Raport i odnośniki w historii projektu. Kod produkcyjny i konfigurator pozostawiono bez zmian. Zbudowano wyłącznie artefakty developerskie i próby lokalne; nie wydano nowego numeru, nie wgrano niczego do roweru. Cudzy WIP pozostawiono bez ingerencji.
