# TASK EXECUTION REPORT — TASK-EVD-AP-02 — faza blind: ZATRZYMANA (skażone wejście)

```text
REPORT_TIMESTAMP:  2026-09-17T10:55:21+02:00
```

## Tożsamość wykonania

```text
PROJECT_ID:             EVD
REPOSITORY:             motor-controller-firmware
COMPONENT:              assist pipeline (tor wspomagania) — faza analizy/projektu
TARGET:                 M820 / BL820 (analiza kodu; HW NOT_TESTED)
FEATURE_ID:             (nie przydzielony w karcie neutralnej)
TASK_ID:                TASK-EVD-AP-02
TASK_TITLE:             Independent Architect — blind, niezależne rozwiązanie toru wspomagania
EXECUTION_ID:           EXEC-AP-02-BLIND-001
STARTED_AT:             2026-09-17T10:4x:xx+02:00 (początek sesji; dokładna wartość UNKNOWN — nie rekonstruuję)
ENDED_AT:               2026-09-17T10:55:21+02:00

EXECUTION_SCOPE:        REPOSITORY_LOCAL (deliverable) / odczyt z PROJECT_SHARED
PRIMARY_OWNER:          Independent Architect (Worker)
AFFECTED_REPOSITORIES:  motor-controller-firmware (tylko ten katalog)

FINAL_STATUS:           STOPPED
```

## DLA UŻYTKOWNIKA

**Czego dotyczyło zadanie:** miałem niezależnie, „na ślepo", zaprojektować własną architekturę
toru wspomagania (od czujników do silnika) dla sterownika M820 — bez zaglądania do rozwiązania,
które projekt już wypracował. Sens ćwiczenia polega na tym, że dwa rozwiązania powstają
niezależnie i dopiero potem się je porównuje.

**Co zrobiono:** przeczytałem dokumenty procesowe i zacząłem analizować kod z przygotowanej dla
mnie „czystej" kopii repozytorium. Przy pierwszym pliku, który dotyczy bezpośrednio problemu
(przetwarzanie sygnału z czujnika nacisku), okazało się, że ta kopia **już zawiera gotowe
rozwiązanie tego samego problemu** — łącznie z uzasadnieniem, wynikami pomiarów i wskazaniem,
który filtr usunięto i dlaczego.

**Wynik:** warunek „na ślepo" jest **złamany**. Przerwałem pracę projektową zamiast
wyprodukować dokument, który nazywałby się „niezależny", a nim nie był.

**Stan:** `STOPPED`

**Czy coś pozostało:** tak — całe zadanie projektowe. Nie powstał żaden snapshot architektury,
żaden hash, żadne warianty.

**Najważniejszy problem:** kopia repozytorium wybrana jako „baseline sprzed rozwiązania" jest
o jeden commit **za późna**.

**Dlaczego wystąpił:** cała praca AP-01/AP-03 weszła do repozytorium jednym commitem o tytule
„snapshot work in progress before cross-machine sync" — tytuł nie zdradza, co zawiera, więc przy
wyborze punktu odcięcia wyglądał niegroźnie.

**Co dalej:** powtórzyć fazę blind w **nowej sesji, z nowym agentem** (ja jestem już trwale
skażony treścią rozwiązania), na worktree wyciętym z commita `9b41070`, a nie `31854af`.

## Wykonanie

> `REPORT WHAT WAS ACTUALLY DONE, NOT WHAT WAS INTENDED.`

**SCOPE EXECUTED:** czytanie dozwolonego wejścia + weryfikacja integralności warunku blind.
Praca projektowa (AC2/AC3/AC4) **nie została rozpoczęta**.

**WHAT WAS ACTUALLY DONE:**

1. Przeczytano dozwolone wejście (pełna lista niżej, §Pliki wejściowe).
2. Rozpoczęto analizę toru czujnik→silnik od `src/torque_input.c` w worktree
   `motor-controller-firmware-AP02-blind-baseline` (HEAD `31854af`).
3. Wykryto w tym pliku skażenie: zacommitowane, opisane rozwiązanie problemu pulsacji
   oznaczone jako `AP-03`.
4. Zweryfikowano zasięg skażenia (`git grep`, `git log -S`) i ustalono dokładny commit,
   który je wprowadził.
5. Wykonano `STOP` zgodnie z pkt 5 ograniczeń karty i spisano niniejszy raport.

**WHAT WAS NOT DONE:** AC2 (mapa sensor→silnik), AC3 (dwa warianty + eksperyment
dyskryminujący), AC4 (scenariusze konfliktu), AC5 (etykiety pewności dla własnej propozycji),
snapshot + SHA256. Brak jakiegokolwiek dokumentu architektury — **celowo**.

**FILES / COMPONENTS TOUCHED:** utworzono wyłącznie ten plik, w
`motor-controller-firmware\documentation\assist-pipeline-work\AP-02\`. Kod produkcyjny,
worktree blind-baseline i katalog AP-01 — **nietknięte**. Nic nie commitowano.

**IMPORTANT DECISIONS:**

- `D-1` — **nie** produkować „ślepej" propozycji z zanieczyszczonego wejścia. Uzasadnienie:
  RULE 31 (DAL-3 wymaga *niezależnego* blind), RULE 7 / RULE 44 (rozwiązanie przepisane od
  kogoś innego nie staje się niezależnym potwierdzeniem przez to, że napisał je inny model),
  RULE 55 (raportuj, co faktycznie zrobiono). Dokument nazwany „blind", powstały po przeczytaniu
  cudzego rozwiązania, fałszowałby dowód, dla którego całe AP-02 istnieje.
- `D-2` — nie próbować „zapomnieć" i kontynuować. Kontaminacja dotyczy dokładnie rdzenia
  problemu (estymator RUN i filtr odpowiedzialny za tętnienie), nie peryferiów.

## Problemy

```text
PROBLEM_ID:     P-1
PROBLEM:        Worktree "blind baseline" (HEAD 31854af) zawiera zacommitowane rozwiązanie
                problemu, który AP-02 ma rozwiązać niezależnie.
CAUSE:          POTWIERDZONA. Cała praca AP-01/AP-03 weszła jednym commitem 10dd32c
                ("motor-controller: snapshot work in progress before cross-machine sync").
                31854af = 10dd32c + poprawka skryptu builda, więc dziedziczy całą treść AP-03.
                Tytuł 10dd32c nie sugeruje zawartości AP-0x — stąd błędny wybór punktu odcięcia.
IMPACT:         Warunek blind dla TASK-EVD-AP-02 NIEWAŻNY. Niezależność RULE 31 niemożliwa
                do osiągnięcia w tej sesji.
ACTION TAKEN:   STOP zgodnie z pkt 5 ograniczeń karty; brak deliverable projektowego;
                raport + rekomendacja czystego commita.
STATUS:         BLOCKING
```

```text
PROBLEM_ID:     P-2
PROBLEM:        integration/PROJECT_START_HERE.md — plik jawnie DOZWOLONY w karcie jako
                "czytaj wyłącznie jako router" — zawiera w banerach u góry wnioski
                rozwiązaniowe dotyczące tego samego problemu (m.in. werdykt o filtrze
                opadania, odsetek próbek omijających go po przerwie, historia skal czujnika).
CAUSE:          POTWIERDZONA konstrukcyjnie: router gromadzi datowane banery statusu, więc
                z czasem przestaje być samym routerem. Karta zakładała, że wystarczy zakazać
                KLIKANIA w linki; treść wycieka samym banerem, bez otwierania linku.
IMPACT:         Dodatkowe, niezależne od P-1 skażenie fazy blind. Samo unikanie linków
                NIE wystarcza jako mechanizm izolacji.
ACTION TAKEN:   Zgłoszone jako finding; treść banerów nie została użyta do żadnej analizy.
STATUS:         UNRESOLVED (wymaga decyzji koordynatora — patrz NEXT EXACT ACTION)
```

## Jeśli STOPPED lub BLOCKED

```text
WORK COMPLETED BEFORE STOP:
    Lektura dozwolonego wejścia; audyt integralności warunku blind; ustalenie commita
    wprowadzającego skażenie i wskazanie czystego kandydata.

STOP / BLOCK CONDITION:
    Pkt 5 ograniczeń karty TASK-EVD-AP-02: "If you notice that a proposed solution already
    appears to be baked into anything you're allowed to read, STOP and report that as a
    finding instead of quietly using it."

WHY CONTINUING WAS NOT ALLOWED:
    Kontynuacja wyprodukowałaby dokument o statusie "independent blind solution", który
    niezależny nie jest. Późniejsze porównanie AP-02 vs Master mierzyłoby wtedy zbieżność
    dwóch wariantów tego samego źródła, a nie niezależne potwierdzenie — czyli dawałoby
    fałszywie wysoką pewność dokładnie tam, gdzie DAL-3 miał ją podnieść naprawdę.

PARTIAL / UNCOMMITTED STATE:
    Brak. Jedyny zapis to ten plik. Repozytoria bez zmian, nic nie zacommitowano,
    worktree blind-baseline czysty (git status pusty).

SAFE STATE:
    TAK. Kod produkcyjny nietknięty, build nietknięty, AP-01 nietknięte.

WHAT IS REQUIRED TO RESUME:
    Nowy worktree read-only wycięty z commita 9b41070 ("FW-110 v5: give Hall calibration a
    gated trigger instead of no trigger") — zweryfikowane: przy 9b41070 `git grep -E "AP-0[1-9]"`
    po src/ inc/ sim/ tests/ nie zwraca NICZEGO, podczas gdy przy 31854af zwraca trafienia
    m.in. w src/torque_input.c, inc/torque_input.h, tests/host/torque/torque_run_asym_host.c,
    sim/controller_lab/.
    ORAZ: nowa sesja z NOWYM agentem. Ta sesja nie nadaje się do wznowienia — przeczytała
    treść rozwiązania i nie da się tego cofnąć.
    ORAZ: rozstrzygnięcie P-2 (izolacja od banerów w PROJECT_START_HERE.md).
```

## Build / Test / Review

```text
BUILD:   NOT_RUN   (karta: Build Impact = none; nie dotykano celu builda)
TEST:    NOT_RUN
REVIEW:  NOT_RUN
```

## Verification

```text
VERIFICATION EXECUTED:
  method:          inspekcja kodu źródłowego + audyt historii git (metadane i treść drzewa)
  tool:            git grep, git log -S, git rev-parse; Read
  input:           worktree C:\Projekty\eVistDrive\motor-controller-firmware-AP02-blind-baseline
                   @ 31854af oraz drzewo @ 9b41070
  expected:        baseline wolny od materiału rozwiązaniowego AP-0x
  actual:          @31854af — trafienia AP-03/AP-01 w kodzie produkcyjnym, testach i sim;
                   src/torque_input.c zawiera pełne uzasadnienie usunięcia filtru FW-112.4
                   wraz z liczbowym wynikiem A/B i wskazaniem katalogu evidence.
                   @9b41070 — zero trafień.
  RESULT:          FAIL (warunek blind niespełniony)
  evidence_location: ten raport, §Dowód skażenia

NOT EXECUTED:
  method:          cała weryfikacja własnej propozycji architektury (AC2-AC5)
  reason:          propozycja nie powstała — STOP przed jej wytworzeniem
  impact:          TASK-EVD-AP-02 faza blind pozostaje NIEWYKONANA

REGRESSION ASSET CREATED:   NO
REGRESSION_ASSET_LOCATION:  N/A

HARDWARE TEST:              NOT_REQUIRED (karta: HW NOT_TESTED)

VERIFICATION EXECUTION STATE:   NOT_RUN  (dla przedmiotu zadania)
```

`VERIFICATION REVIEW STATE` nadaje Reviewer, nie Worker.

## Dowód skażenia

### D-1. Kod produkcyjny w rzekomo czystym baseline

`src/torque_input.c` @ `31854af`, linie 223-246 — blok komentarza rozpoczynający się:

```text
AP-03: the FW-112.4 asymmetric time-domain filter USED TO LIVE HERE. It is deleted.
```

Blok zawiera: (a) diagnozę przyczyny tętnienia na poziomie architektury, (b) argument,
dlaczego okno w domenie **kąta korby** jest właściwą jednostką zamiast stałej czasowej,
(c) liczbowy wynik A/B na nagranej jeździe (autokorelacja przy okresie stroke'u oraz czas
narastania 10-90 %), (d) ścieżkę do katalogu evidence, (e) konsekwencję dla poziomu wspomagania.

To jest jednocześnie **rozwiązanie** problemu z karty AP-02 **i** szkic eksperymentu
dyskryminującego, którego wymaga AC3.

Dalsze trafienia AP-03 w tym samym pliku: linie 191, 298, 805.
`inc/torque_input.h`: linie 84 (AP-03) i 103 (odwołanie do scenariusza kroku z **AP-01**).
`tests/host/torque/torque_run_asym_host.c`: linie 318, 355 — testy dostosowane do usunięcia filtru.
`tests/host/fw129b_state_hygiene_host.c`: linia 237.
`sim/controller_lab/` — scenariusze i README odwołujące się do AP-01 (transients-001,
disturbances-001, REVIEW-EVD-AP-01-010/011) oraz do katalogu
`documentation/assist-pipeline-work/AP-01/`.

### D-2. Punkt wejścia skażenia

```text
31854af  fix: restore build_firmware.py entry point and git_value helper   <- wybrany baseline
10dd32c  motor-controller: snapshot work in progress before cross-machine sync   <- WPROWADZA AP-01/AP-03
9b41070  FW-110 v5: give Hall calibration a gated trigger instead of no trigger  <- CZYSTY
```

`git log -S"AP-03"` i `git log -S"AP-01"` wskazują ten sam, pojedynczy commit `10dd32c`.
`git grep -E "AP-0[1-9]" 9b41070 -- src inc sim tests` → brak wyników.

Uwaga metodyczna: czytałem wyłącznie **tytuły** commitów (metadane historii), nie ich treść,
i wyłącznie w zakresie potrzebnym do wskazania czystego punktu odcięcia.

### D-3. Skażenie drugim, niezależnym kanałem (P-2)

`integration/PROJECT_START_HERE.md` — plik wskazany w karcie jako dozwolony „router" — w
banerach datowanych 2026-09-10 i 2026-09-16 zawiera wnioski rozwiązaniowe dotyczące tego
samego toru wspomagania. Baner jest widoczny natychmiast po otwarciu pliku, bez otwierania
żadnego linku, więc instrukcja „nie klikaj w linki o torze wspomagania" nie chroni przed nim.

Ironicznie, jeden z tych banerów kończy się zdaniem, że AP-02 **nie może** dostać transkryptu
tamtej sesji — a sam baner podaje jej wnioski.

## Ocena wpływu na TASK-EVD-AP-02 jako całość

To nie jest usterka kosmetyczna. Karta jest `DAL-3` i jej wartość bierze się wyłącznie
z niezależności (RULE 31). Gdyby skażona faza blind przeszła cicho:

1. porównanie AP-02 vs Master pokazałoby zbieżność — i wyglądałoby na **potwierdzenie**;
2. w rzeczywistości byłaby to ta sama informacja policzona dwa razy (RULE 44: zgoda agentów
   nie jest dowodem, powtórzenie nie jest potwierdzeniem);
3. decyzja architektoniczna dostałaby wyższą pewność, niż na to zasługuje, dokładnie w tym
   miejscu, gdzie proces miał ją urealnić.

Dlatego jedyny poprawny wynik tej sesji to `STOPPED`, a nie „częściowy deliverable".

## Metadane wykonawcy

```text
AGENT_PLATFORM:            Claude Code (Claude Agent SDK), subagent
WORKER_POOL:               NOT_EXPOSED
ACTUAL_MODEL:              claude-opus-5
MODEL_SELECTION:           STATIC (narzucony przez wywołującego)
AVAILABILITY_VERIFIED_AT:  2026-09-17T10:55:21+02:00
```

## Discoveries

```text
DISCOVERIES CAPTURED:   brak wpisu DISC-xxx (nie mam uprawnień zapisu do integration/;
                        kandydaci opisani niżej, do przechwycenia przez koordynatora)
DID ANY DISCOVERY CHANGE CURRENT TASK?   YES
```

Kandydat `DISC-A` — **opaque WIP commit jako pułapka izolacji**. Commit o tytule
„snapshot work in progress before cross-machine sync" przeniósł całą pracę AP-01/AP-03.
Wybór punktu odcięcia po tytułach commitów jest z tego powodu zawodny. Kandydat na regułę
procesu: punkt odcięcia dla ćwiczenia blind **weryfikuje się treścią** (`git grep` po
markerach zadania w drzewie kandydata), a nie tytułem commita — i wynik tej weryfikacji
zapisuje się w karcie zadania.

Kandydat `DISC-B` — **router gromadzi treść**. `PROJECT_START_HERE.md` deklaruje, że jest
routerem bez szczegółów, ale banery statusu czynią go nośnikiem wniosków. Kandydat na regułę:
dla ćwiczeń blind przygotowuje się jawny, okrojony wariant wejścia, zamiast polegać na
dyscyplinie agenta przy czytaniu pliku ogólnego przeznaczenia.

Zgodnie z RULE 42 odkrycia zostały przechwycone, nie rozwinięte w osobne zadanie.

## Pozostała praca

**REMAINING WORK:** całość TASK-EVD-AP-02 faza blind — AC1-AC5, snapshot, SHA256.
Nic z tego nie zostało wykonane i nic nie jest ukryte pod statusem częściowym.

## NEXT EXACT ACTION

> Koordynator: utworzyć nowy worktree read-only z commita **`9b41070`** (nie `31854af`),
> rozstrzygnąć izolację od banerów `PROJECT_START_HERE.md` (P-2), i wydać kartę
> TASK-EVD-AP-02 **nowej sesji z nowym agentem** — bez transkryptu tej sesji.

## Status wykonawcy

```text
STATUS: READY_FOR_REVIEW   (faza blind: STOPPED)
```

Worker nie zatwierdza sam siebie (RULE 1). Ten raport **zapisuje** fakt zatrzymania —
nie nadaje mu werdyktu.

## Pliki wejściowe faktycznie przeczytane (pełne ścieżki)

```text
C:\Projekty\eVistDrive\CLAUDE.md                                   (auto-załadowany przez harness)
C:\Projekty\eVistDrive\integration\PROJECT_START_HERE.md           (dozwolony; patrz P-2)
C:\Projekty\eVistDrive\integration\AGENTS.md                       (całość)
C:\Projekty\eVistDrive\integration\templates\TASK_EXECUTION_REPORT.md
C:\Projekty\eVistDrive\motor-controller-firmware-AP02-blind-baseline\AGENTS.md
C:\Projekty\eVistDrive\motor-controller-firmware-AP02-blind-baseline\docs\ARCHITECTURE_CURRENT.md
C:\Projekty\eVistDrive\motor-controller-firmware-AP02-blind-baseline\src\torque_input.c   <- tu wykryto skażenie
```

Dodatkowo: listingi katalogów `src/`, `inc/`, `docs/` oraz wyniki `git grep` / `git log`
w worktree blind-baseline (tytuły commitów, nazwy plików i numery linii).

**NIE przeczytano** (zgodnie z ograniczeniami karty): niczego z
`C:\Projekty\eVistDrive\motor-controller-firmware\` (bieżący checkout z „Assist Pipeline V2"),
niczego z `integration\tasks\`, `features\`, `decisions\`, `claims\`, `evidence\`,
`task-reports\`, `handoffs\`, `roadmap\`, `PROJECT_STATE.md`, `CLAIM_LEDGER.md`,
`DISCOVERY_LEDGER.md`, `PROJECT_HISTORY.md`, ani niczego z `hmi-firmware\`, `canable-web\`,
`mobile-app\`, ani katalogu `documentation\assist-pipeline-work\AP-01\`.

**Nie przeczytano też** — i jest to odstępstwo od karty wymagające odnotowania — docelowej
lektury obowiązkowej roli Architect: `integration\docs\04_IMPACT_ANALYSIS.md`,
`docs\17_PROCESS_PROFILES.md`, `docs\21_DECISION_ASSURANCE_AND_ANALYSIS_BUDGET.md`,
`docs\25_POLICY_HIERARCHY_AND_CORE_GOVERNANCE.md`, `integration\PROJECT_PROFILE.md`.
Powód: skażenie wykryto zanim lektura doszła do tego etapu, a po `STOP` dalsze czytanie
materiału przygotowującego do projektowania nie miało celu. Przy wznowieniu nowy agent
wykonuje tę lekturę w całości.

**TSDZ2:** nie kopiowano kodu TSDZ2 ani nie portowano jego algorytmów. W tej sesji nie
powstała żadna propozycja architektury, więc oświadczenie jest trywialnie spełnione.

## Korekty (append-only)

```text
CORRECTED / SUPERSEDED BY:   (brak)
REASON:                      (brak)
NEW EVIDENCE:                (brak)
```
