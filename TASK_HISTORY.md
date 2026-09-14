# TASK_HISTORY ? motor-controller-firmware

Indeks wykona?; raporty cross-repository w integration/task-reports.

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
