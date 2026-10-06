# TASK_HISTORY ? motor-controller-firmware

Indeks wykona?; raporty cross-repository w integration/task-reports.

- 2026-10-05T11:06:39+02:00 - EVD / integration + motor-controller-firmware / EXEC-EVD-PWR-001-REVIEW-02,
  TASK-EVD-PWR-001, CROSS_REPOSITORY, PARTIAL (review complete, HW_PENDING), PROBLEM_FLAG=NO in V2 scope.
  Independent V2 PASS PC/reverse: PWR-REV-001/002 RESOLVED, full gate PASS, known-bad regression 8 FAIL,
  repro 0.624/0.625 byte-identical; HW NOT_RUN.
  [Execution](../integration/task-reports/EXEC-EVD-PWR-001-REVIEW-02.md)
  - [Review](../integration/task-reports/REVIEW-EVD-PWR-001-002.md).

- 2026-10-05T10:24:18+02:00 — EVD / EXEC-EVD-PWR-001-04, TASK-EVD-PWR-001, CROSS_REPOSITORY,
  COMPLETED (PC), READY_FOR_REVIEW, PROBLEM_FLAG=YES. Mostek Walk ograniczony czasem (PWR-REV-001, D7a-D7l),
  adresy stock w komentarzach na runtime (PWR-REV-002); commit 880ea55; full gate PASS; wsady 0.624/0.625
  (0.626/0.627 duplikat); HW NOT_RUN. [Raport](../integration/task-reports/EXEC-EVD-PWR-001-04.md).

- 2026-10-05T09:54:37+02:00 — EVD / EXEC-EVD-PWR-001-REVIEW-01, TASK-EVD-PWR-001,
  CROSS_REPOSITORY, COMPLETED (niezależny review), CHANGES_REQUIRED, PROBLEM_FLAG=YES.
  AC10 FAIL przy 2 ms calls (bridge >250 ms); stock header0x20 wymaga korekty map/skryptów.
  Full gate i ARM normal/diagnostic PASS; HW NOT_RUN; kod produkcyjny bez zmian.
  [Raport wykonania](../integration/task-reports/EXEC-EVD-PWR-001-REVIEW-01.md)
  · [Review](../integration/task-reports/REVIEW-EVD-PWR-001-001.md).

- 2026-09-08T14:22:34+02:00 ? EXEC-2026-09-08-004, TASK NONE, USER, CROSS_REPOSITORY ? COMPLETED (audyt i plan), READY_FOR_REVIEW, REVIEW NOT_RUN. Globalna mapa toru wspomagania, konfiguracja CANable, 34 pr?by host, plan przebudowy; produkcyjny kod nietkni?ty, naprawa jeszcze niewdro?ona. [Raport](../integration/task-reports/EXEC-2026-09-08-004.md), [master plan](documentation/ASSIST_PIPELINE_MASTER_PLAN.md).

- 2026-09-08T19:19:45+02:00 ? uzupe?nienie EXEC-2026-09-08-004 na polecenie USER: firmware-first; obecne CANable nie ogranicza architektury. Dodano kontrakt docelowego toru silnika, projekt CANable odroczony do ustalenia firmware. Kod produkcyjny bez zmian. [Raport](../integration/task-reports/EXEC-2026-09-08-004.md).

- 2026-09-08T20:40:59+02:00 — EXEC-2026-09-08-005: przygotowano podział assist pipeline na agentów, trzy pierwsze karty i późniejsze obszary z bramkami; Master odbiera i dopuszcza etapy. Zmiany dokumentacyjne; brak implementacji/flash. [Raport](../integration/task-reports/EXEC-2026-09-08-005.md).

- 2026-09-09T07:58:19+02:00 — review AP-01 V1: CHANGES_REQUIRED (R1–R6). 42 odtworzenia PASS, 21 CSV i baza potwierdzone; kontrprzykłady metryk zatrzymania, błędne steady okno i brak propozycji progów. Wydano prompt rework dla zewnętrznego wykonawcy; kod wykonawcy/produkcja nietknięte. [Raport]( ../integration/task-reports/REVIEW-EVD-AP-01-001.md ).

- 2026-09-09T08:18:00+02:00 — AP-01 review V2 CHANGES_REQUIRED; 19+5 testów autora pokazuje PASS, ale test runnera kończy się ModuleNotFoundError przed native. Utrwalono kontrprzykłady i zawężony rework; kod autora nietknięty. [Raport](../integration/task-reports/REVIEW-EVD-AP-01-002.md).

- 2026-09-09T08:52:45.7785296+02:00 - AP-01 review V3: CHANGES_REQUIRED. 61 tests PASS; stop, runner failure handling and inventory accepted within reviewed scope. Remaining: Iq stage classification and waveform comparison. Narrow REWORK-003 issued for external worker; AP-03 blocked. Production code unchanged. [Report](../integration/task-reports/REVIEW-EVD-AP-01-003.md).

- 2026-09-09T09:09:54+02:00 — AP-01 V4: PASS zakresu REWORK-003, 69/69 testów i niezależne przeliczenie 21 przypadków. Klasyfikacja/porównanie naprawione. Cały TASK IN_PROGRESS: 7 kategorii przejść otwartych; AP-03 nadal zamknięte. Progi pilota host przyjęte w zakresie review; kolejny krok: osobny przydział rozszerzenia harnessa. [Raport](../integration/task-reports/REVIEW-EVD-AP-01-004.md).

- 2026-09-09T09:15:23+02:00 — Wydano AP-01 TRANSIENTS-001 dla zewnętrznego wykonawcy: skok/spadek nacisku, drugi przedział pedałowania, nowe pomiary i wykresy. Ściśle określony zapis harnessa; produkcja i stare dowody READ_ONLY. Odbiór po EXEC-EVD-AP-01-005. [Raport](../integration/task-reports/EXEC-EVD-AP-01-DISPATCH-TRANSIENTS-001.md).

- 2026-09-09T09:48:46+02:00 — AP-01 raport 005: CHANGES_REQUIRED. 151 testów PASS, 4 native replay identyczne; generator przyjęty w zakresie review. Rework dotyczy chronologii restartu, kontroli kompletności i czytelności wykresów. Wydano wąski prompt, następny raport 006. Produkcja bez zmian. [Review](../integration/task-reports/REVIEW-EVD-AP-01-005.md).

- 2026-09-09T13:12:17+02:00 — AP-01 review 006: CHANGES_REQUIRED; 184 testy PASS. Zachowane dane potwierdzone hashami; bez odtwarzania historii WIP. Pozostaje chronologia próbek restartu, status prób regresyjnych/liczba powtórzeń, jednostka i stopka wykresu. Wydano rework-002, następny raport 007. [Review](../integration/task-reports/REVIEW-EVD-AP-01-006.md).

- 2026-09-09T13:40:56+02:00 — AP-01 review 007 PASS zakresu REWORK-002. 209 testów PASS, 35 analiz zgodnych; chronione dane zachowane. Zakończono poprawki restartu/manifestu/wykresów. Pełne AP-01 IN_PROGRESS: reverse, invalid, jitter i kryteria przejść wymagają osobnego przydziału. [Review](../integration/task-reports/REVIEW-EVD-AP-01-007.md).

- 2026-09-09T18:02:08+02:00 — Agent A raport 008: CHANGES_REQUIRED, oddano plan bez implementacji i danych; katalog disturbances-001 nie istnieje, hash C bez zmian. Istniejący ISSUED wystarcza do pracy. Skorygowano plan zegarów i ułamkowego kąta korby; wydano kontynuację, następny raport AP-01-009. [Review](../integration/task-reports/REVIEW-EVD-AP-01-008.md).

- 2026-09-13T23:41:07+02:00 — EXEC-EVD-ASSIST-PIPELINE-V2-001: tor wspomagania przebudowany od zera (Assist Pipeline V2) na zlecenie właściciela projektu; stary tor USUNIĘTY w całości (13 modułów, −24131/+7752 linii). Nowe: jeden automat cyklu PAS, model base/dynamic zamiast filtrowania pulsacji, estymatory agresji i obciążenia, profile ECO/TRAIL/SPORT/SPORT+/AUTO/AUTO SPORT+, jeden łańcuch limiterów, pełna telemetria. Walk/CAN/HMI/FOC zachowane; Walk zyskał wspólne zabezpieczenia. Pełna bramka PASS (53 zestawy hosta, sanitizers, Level-4, replay), oba warianty target PASS (NORMAL 48,37 % FLASH / 80,70 % RAM, DIAGNOSTIC 60,30 % / 61,90 %). Tłumienie pulsacji zmierzone 0,39–0,41. Naprawione 3 wady builda sprzed zadania (10dd32c). HW NOT_RUN — nic nie jechało na rowerze; nastawy do strojenia. READY_FOR_REVIEW. [Raport](../integration/task-reports/EXEC-EVD-ASSIST-PIPELINE-V2-001.md).

- 2026-09-14T08:33:06+02:00 — EXEC-EVD-ASSIST-V2-AUDIT-001: niezależny audyt V2, CHANGES_REQUIRED (9 ustaleń z warunkami odbioru). 53 zestawy hosta PASS, target NORMAL ARM 13.2.1 PASS; bramka Windows zatrzymana na WinError 193. Produkcja bez zmian. [Raport](docs/AUDIT_ASSIST_PIPELINE_V2_2026-09-14_PL.md).

- 2026-09-14T08:38:46+02:00 — EXEC-EVD-ASSIST-V2-AUDIT-K1: przyjęto referencję reverse G5300, zachowano oryginał i mapowanie na M820. Doprecyzowano stop/reverse, event-driven cadence i granice dowodów; CHANGES_REQUIRED bez zmian. Produkcja bez zmian. [Korekta K1](docs/AUDIT_ASSIST_PIPELINE_V2_2026-09-14_PL.md#korekta-k1--materiał-reverse-g5300-2026-09-14t0838460200).

- 2026-09-14T11:49:20+02:00 — EXEC-EVD-G5300-CLOSURE-REVIEW-001: przeczytano closure i appendix z external/datasheets, zapisano matrycę domknięcia, nowe mechanizmy i korektę audytu K2. Wyjaśnione alpha i E1E8; nadal otwarte fizyczne skalowanie/prąd, kanały limiterów i 72/144 events. Bez zmian produkcji; nowe poprawki firmware nie były recenzowane. [Raport](docs/reference/g5300/CLOSURE_REVIEW_2026-09-14_PL.md).

- 2026-09-14T12:28:08+02:00 — EXEC-EVD-ASSIST-PIPELINE-V2-AUDIT-FIX-001: usunięte wszystkie 9 ustaleń audytu V2 w kodzie produkcyjnym, każde z testem, który potrafi je złapać. Zrealizowana NEXT EXACT ACTION K1 — pomiar pełnej osi stop/reverse od fizycznej krawędzi PAS do rzeczywistego prądu (stop 206,25 ms, reverse 0,50 ms). Ten pomiar wykrył regresję wprowadzoną poprawką ustalenia 4: zatrzymanie 831 ms zamiast 190 ms — naprawione do 187 ms, przypięte scenariuszem S17. Pełna bramka PASS (54 zestawy hosta, sanitizers, Level-4, replay z osobnym BEHAVIOR_ACCEPTED), oba warianty target PASS. HW NOT_RUN. READY_FOR_REVIEW — werdykt audytu zdejmuje niezależny review na HEAD dc23faf, nie ten raport. [Raport](../integration/task-reports/EXEC-EVD-ASSIST-PIPELINE-V2-AUDIT-FIX-001.md).

- 2026-09-14T13:36:46+02:00 — EXEC-EVD-CONFIG-CANABLE-AUDIT-001: CHANGES_REQUIRED. Poprawiony V2 przechodzi bramkę PC (55 host, cudzy test WIP) i oba buildy ARM 13.2.1; sanitizers w tej sesji SKIPPED, HW NOT_RUN. Wykryto odrzucanie własnego banku V2, sprzeczne zero limitu Iq, utratę default attack oraz niezgodność UI/diagnostyki CANable. Matryca aktywnych ustawień i rework C1–C7; produkcja bez zmian. [Raport](docs/AUDIT_CONFIG_CANABLE_RELEASE_2026-09-14_PL.md).

- 2026-09-14T13:45:00+02:00 — Doprecyzowano wytyczne UX CANable: opis reakcji/podtrzymania/limitu, wykresy nacisk→pomoc i zmianę w czasie, tryb zaawansowanego strojenia oraz kryteria no-op/migracji. Bez zmian kodu UI i firmware.

- 2026-09-14T13:50:00+02:00 — Rozszerzono UX CANable o wzorzec suwak + interaktywny wykres: podgląd przed zapisem, porównanie przed/po, profile bazowe, zakresy bezpieczne i mapowanie suwaków opisowych na parametry zaawansowane. Bez implementacji UI.

- 2026-09-14T14:45:00+02:00 — EXEC-EVD-CONFIG-CONTRACT-FIX-001, TASK NONE, USER, CROSS_REPOSITORY → COMPLETED (zakres C1–C7), READY_FOR_REVIEW, REVIEW NOT_RUN. Firmware przyjmuje własne banki V2 (walidator 0..12, odrzucenie całości bez częściowej mutacji), zero w `max_iq_pct` = poziom wyłączony z osobnym sentinelem `AP2_LIMITS_NO_LEVEL_CEILING`, zero w rampie = „decyduje profil”; blob banku v9 jako sygnał generacji profili (layout v8 bez zmian). Nowy zestaw hosta `assist_bank_contract_host.c` (B1–B7) + scenariusz S18 + mutacje dowodzące, że testy upadają na błędach sprzed poprawki. Bramka `verify_all.py --require-target` PASS, 56/56 hosta; ARM 13.2.1 NORMAL 115884 B / RAM 80,75 %, DIAGNOSTIC 144004 B / RAM 61,93 % (oba DEV-NONCANONICAL). Sanitizery NOT_RUN, HW NOT_RUN. Cudzy WIP nietknięty. [Raport](../integration/task-reports/EXEC-EVD-CONFIG-CONTRACT-FIX-001.md).
- 2026-09-14T19:12:00+02:00 — REVIEW-EVD-CONFIG-CONTRACT-FIX-001: 56/56 testów hosta firmware i 19/19 CANable PASS; C1–C7/P1 potwierdzone programowo. Werdykt ACCEPTED_WITH_LIMITATIONS, nie produkcyjny: HW/CANable/M820 NOT_RUN, ASan/UBSan SKIPPED, UX suwaków/wykresów nie zbudowany. Candidate for bench testing. [Review](../integration/task-reports/REVIEW-EVD-CONFIG-CONTRACT-FIX-001.md).

## 2026-09-16 — wstępny audyt torque M820/G5300

2026-09-16T07:50:39+02:00 · EVD · EXEC-2026-09-16-001 · PROJECT_SHARED · PARTIAL / READY_FOR_REVIEW · PROBLEM: YES. Audyt read-only HEAD e54e527 i plan przed implementacją: V2 base/dynamic, konflikt PAS 96/64, nieustalony testowany DEV BIN; przygotowana karta braków reverse. Kod, konfiguracja i BIN bez zmian. BUILD/TEST/REVIEW/HW NOT_RUN. [Kanoniczny raport](../integration/task-reports/EXEC-2026-09-16-001.md).

2026-09-16T07:52:01+02:00 · EXEC-2026-09-16-001 / K1: właściciel wskazał testowany BIN z fix-config-diag; SHA256 04ed8e00…3c5e7cd zgodny z manifestem, HEAD 226cb54 + dirty. Jazda bez logowania. [Korekta raportu](../integration/task-reports/EXEC-2026-09-16-001.md#korekta-k1--wskazanie-testowanego-obrazu-przez-właściciela). Kod bez zmian.

2026-09-16T08:43:29+02:00 · EVD · EXEC-2026-09-16-002 · PROJECT_SHARED · COMPLETED (archiwizacja). Zachowano kopię historycznego testowanego BIN-a, 10 materiałów reverse/model/wektory i manifest SHA; model self-tests PASS, 275 torque/8 D7EC core zgodne. State5 prose/model discrepancy otwarte. Kod firmware bez zmian. [Raport](../integration/task-reports/EXEC-2026-09-16-002.md) · [Trwały punkt wejścia](../integration/evidence/m820-g5300-torque-20260916/README.md).

2026-09-16T08:46:51+02:00 · EXEC-2026-09-16-003 · PROJECT_SHARED · COMPLETED (dokumentacja). Przygotowano plan i 9 kart agentów dla torque/start–stop; wszystkie NOT_ISSUED, firmware bez zmian. [Plan](../integration/tasks/M820_TORQUE_AGENT_WORK.md) · [Raport](../integration/task-reports/EXEC-2026-09-16-003.md).

- 2026-09-25T07:55:04+02:00 — EVD / EXEC-EVD-TQ-06-PHASE7-HOST-GATE-REWORK-001, TASK-EVD-TQ-06, REPOSITORY_LOCAL, BLOCKED. Suites 41/42 PASS; after runner correction suite 57 runs the actual G53 pipeline, but P6/P7/P8 do not reach positive M2AA (full gate 63/64). Phase 4 commit 20650bb and Phase 5 remain unchanged; no later phase started. [Report](integration/evidence/evd-tq/TQ-06/EXEC-TQ06-PHASE7-HOST-GATE-001.md).

- 2026-09-25T09:35:45+02:00 - EVD / EXEC-EVD-TQ-06-PHASE7-HOST-GATE-REWORK-001, TASK-EVD-TQ-06: accepted Phase-5 PAS vector connected to suite 57; suite 57 PASS and canonical host gate 64/64 PASS; Phase 7 ready for local commit. Phase 4/5/6 commits unchanged; Phase 8 pending. [Append-only report correction](integration/evidence/evd-tq/TQ-06/EXEC-TQ06-PHASE7-HOST-GATE-001.md).

- 2026-09-25T09:40:44+02:00 - EVD / TASK-EVD-TQ-06: Phase 8 downstream handoff verification PASS after Phase 7 commit 15034f19. ap2_limits, one final mailbox publication, sole fast_iq_slew_tick writer, normal BYPASS and native safety/reverse policy are retained; no implementation changes or READ_ONLY modifications. Host coverage suites 41/42/43/57 and full 64/64 gate PASS. Phase 9 is the next phase. [Evidence](integration/evidence/evd-tq/TQ-06/EXEC-TQ06-PHASE7-HOST-GATE-001.md).

- 2026-09-26T23:57:00+02:00 — EVD / EXEC-TQ06-PHASE9-CONTINUATION-20260926-001, TASK-EVD-TQ-06, REPOSITORY_LOCAL, COMPLETED. Phase 9 offline gates zielone: host 64/64, Level-4 9/9 i fuzz 25/25 (EXPECTED_START 12/12), STOP_RESTART PASS, W1 6/6, ARM normal/diagnostic PASS; stan READY_FOR_INDEPENDENT_REVIEW. Dirty WIP zachowany, bez commit/push/merge/flash; Phase 10 nie rozpoczęta. [Raport](integration/evidence/evd-tq/TQ-06/EXEC-TQ06-PHASE9-CONTINUATION-20260926-001.md).

- 2026-10-05T09:31:25+02:00 — EVD / EXEC-EVD-PWR-001-01..03, TASK-EVD-PWR-001, REPOSITORY_LOCAL, READY_FOR_REVIEW, HW_PENDING. Linia PA4 jak w oryginalnej aplikacji M820 / FT (DISC-010): wyłączanie trzymanym power (~2,1 s), przycisk Walk z tolerancją zaniku i mostkiem 250 ms, PB8, test obwodu i błąd 36, ochrona naciśnięcia z chwili włączenia; jeden właściciel PA4 `src/pa4_buttons.c`. Commity 00ca24d, 5589c16, 500e1ae na fix/power-button-ft-parity (od 001b143); wsady 0.622/0.623; host 70/70; review NOT_RUN. [Raport 03](../integration/task-reports/EXEC-EVD-PWR-001-03.md) · [Karta](../integration/tasks/TASK-EVD-PWR-001.md).

- 2026-10-05T12:43:56+02:00 — EVD / motor-controller-firmware / EXEC-EVD-COMMUNICATION-REGISTRY-001, ad-hoc USER, CROSS_REPOSITORY, COMPLETED (dokumentacja), READY_FOR_REVIEW. Wspólny rejestr CAN/BLE i obowiązkowe odwzorowanie konfiguracji; odsyłacz produktu, ochrona factory i jawne luki obsługi. Zmiany dokumentacyjne; runtime/HW/BESST NOT_RUN. AFFECTED_REPOSITORIES: integration, motor-controller-firmware, hmi-firmware, canable-web, mobile-app. PROBLEM_FLAG: YES (COMM-GAP-01..09 w rejestrze). [Raport](../integration/task-reports/EXEC-EVD-COMMUNICATION-REGISTRY-001.md).

- 2026-10-05T14:46:42+02:00 — EVD / EXEC-EVD-FW-0624-STACK-GATE-REWORK-001, A1 REWORK (REVIEW-EVD-FW-0624-LINE-001), CROSS_REPOSITORY, COMPLETED, review PASS. `tools/m820_stack_gate.py`: wywołania warunkowe w grafie i analizach NVIC, wartość literału nie przeżywa wywołania ani nieudowodnionej ścieżki, warunkowy argument NVIC nieodczytywalny (`600584c`, `6cfc232`); stałe testy M1–M6, N1–N3, P2; BIN NORMAL/DIAG bez zmian. [Raport](../integration/task-reports/EXEC-EVD-FW-0624-STACK-GATE-REWORK-001.md) · [Review 002](../integration/task-reports/REVIEW-EVD-FW-0624-STACK-REWORK-002.md).

- 2026-10-05T15:06:37+02:00 — EVD / EXEC-EVD-FW-0624-STACK-GATE-TECH-001, ad-hoc TECH, REPOSITORY_LOCAL, READY_FOR_REVIEW. Utwardzenie bramki stosu REV-A1R-02..09 + regex `text_refs` (`6832a65`, branch `tech/stack-gate-hardening`, poza linią do czasu review); testy R01–R14; BIN bez zmian. [Raport](../integration/task-reports/EXEC-EVD-FW-0624-STACK-GATE-TECH-001.md).

- 2026-10-05T18:11:52+02:00 — EVD / EXEC-EVD-FW-0624-STACK-GATE-TECH-001 — review PASS ([REVIEW-EVD-FW-0624-STACK-TECH-001](../integration/task-reports/REVIEW-EVD-FW-0624-STACK-TECH-001.md)); `6832a65` scalone `--no-ff` (`8f9bce8`). Ograniczenia bramki stosu: `integration/TOOL_REGISTRY.md` TOOL-EVD-001.
