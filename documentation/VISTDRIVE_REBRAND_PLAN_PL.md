# Plan zmiany nazwy projektu na eVistDrive

Status: **WDROŻONE (edycje) — 2026-07-23.** Zmiany w plikach zrobione, ale **NIE zacommitowane** (oba repo mają wcześniejszy WIP; commit do decyzji właściciela). Szczegóły w §9.
Data przygotowania: 2026-07-23
Decyzje już podjęte przez właściciela:
- Zaczynamy od **pełnego planu pisemnego** (ten dokument), wdrożenie dopiero po akceptacji.
- Napis na wyświetlaczu roweru (pole HMI `0x6001`): **`eVD <wersja>`** (np. `eVD 0.0187`).
- **Zmieniamy nazwy TYLKO w nowych kartach CANable.** Nowe karty = produkt eVistDrive; pojawiają się **po wykryciu firmware eVistDrive** (`CONTROLLER_FLAVOR.EBICS`). Wtedy stare (fabryczne Bafang) się chowają. Konfiguracja idzie **wyłącznie przez nowe karty**.
- **Starych (fabrycznych Bafang) kart NIE ruszamy — ani nazwy, ani konfiguracji.** Zostają dokładnie jak są.
- **Powłoki aplikacji (tytuł strony, górny nagłówek, `Canable.bat`, `package.json`) na razie NIE ruszamy** — to nie są „nowe karty". Do osobnej decyzji później (patrz §6, sekcja OPEN).
- W efekcie w CANable dotykamy tylko: (1) tekstów widocznych w nowych kartach, (2) widocznych etykiet wykrywania, gdy wykryto eVistDrive.

---

## 1. Prostym językiem — co i dlaczego

Zmieniamy **markę** (to, co widzi człowiek: nazwy, napisy, opisy) z „Bafang / EBICS / Open Bafang" na **eVistDrive**.

Czego **NIE** zmieniamy, żeby nic nie przestało działać:
- **Ramki CAN, których oczekuje wyświetlacz** — pola producenta i modelu zostają fabryczne.
- **Nazwy techniczne w kodzie** (np. `ebics-live`, `EBICS_BUILD_VERSION`) — to „wewnętrzne etykiety", ich zmiana groziłaby zepsuciem połączeń w kodzie, a nie daje nic widocznego użytkownikowi.
- **Historia projektu i licencje** EBiCS oraz ride core — zostają, dopisujemy tylko informację, że eVistDrive jest osobnym forkiem.

Robimy to **etapami, osobnymi commitami** — nigdy jednym wielkim. Niczego nie budujemy bez osobnego polecenia.

---

## 2. Zasady nadrzędne (obowiązują we wszystkich etapach)

1. **Kompatybilność HMI/CAN ma pierwszeństwo.** Nie zmieniamy formatów ramek ani pól, których wyświetlacz używa do identyfikacji sprzętu.
2. **Rozróżniamy markę od techniki.** „EBICS Ride Core" na przycisku = marka (kandydat do zmiany). `data-tab="ebics-live"` = technika (zostaje).
3. **Pochodzenie w kodzie zostaje.** Techniczne nazwy pochodzenia algorytmu (`ebics_foc`, makro `EBICS_BUILD_VERSION`, identyfikatory `ebics-*`) zostają.
4. **Nowe karty = jeden, czysty opis eVistDrive.** W nowych kartach nie ma podwójnych nazw — wszystko przechodzi na eVistDrive (łącznie z „Needs EBICS firmware…" → „Needs eVistDrive firmware…").
5. **Podwójne opisy „Bafang / EBiCS" istnieją TYLKO w starych kartach i w powłoce aplikacji — NIE ruszamy ich.** Uważać, by przy edycji nowych kart nie podmienić przypadkiem opisów EBiCS w starych kartach.
6. **Nowe karty dostają nazewnictwo eVistDrive nawet jeśli pod spodem używają tych samych ramek CAN co stare pola.** Zmieniamy nazwę karty/UI, a nie protokół — wspólna ramka nie blokuje nowej nazwy w nowej karcie.
7. **Bez wielkiego commita.** Każdy etap = osobny, przeglądalny commit, najlepiej osobno per repozytorium.
8. **Bez budowania bez polecenia.**

---

## 3. Inwentaryzacja — gdzie dziś jest stara nazwa

### Repozytorium A: Firmware — `EBICS\BAFANG_GD32F303RCT6`

| Miejsce | Plik / linia | Obecnie | Rodzaj |
|---|---|---|---|
| README nagłówek | `README.md:1` | „Master Branch … CRA101C" | marka (opis) |
| Napis na HMI (pole 0x6001) | `src/CAN_Display.c:597` | `"EBICS %s"` | **marka na wyświetlaczu** |
| Pole 0x6000 (producent) | `src/CAN_Display.c:590` | `"CR X30P.250.FC 2.1"` | **fabryczne — ZOSTAJE** |
| Pole 0x6002 (model) | `src/CAN_Display.c:605` | `"CR X30P.250.FC"` | **fabryczne — ZOSTAJE** |
| Makro wersji | `src/CAN_Display.c:38-39` | `EBICS_BUILD_VERSION "dev"` | technika — nazwa makra ZOSTAJE |
| Nazwa wersji z buildu | `build_firmware.ps1:281,285` | `EBICS_BUILD_VERSION = $ArtifactName` | technika + nazwa pliku (patrz §5) |
| Nazwy plików wynikowych | `build_firmware.ps1:343-383` | `0.0187.bin`, `..._M820_BL820.bin` | pliki wynikowe |
| Dokumentacja protokołu | `documentation/CAN_PROTOCOL_REFERENCE.md:262` | opis „EBICS %s" | dokumentacja |
| CHANGELOG, pozostałe docs | `CHANGELOG.md`, `documentation/*.md` | „EBICS…" w wielu miejscach | dokumentacja (głównie techniczna) |

### Repozytorium B: Narzędzie CANable — `bafang_canable_pro`

| Miejsce | Plik / linia | Obecnie | Rodzaj |
|---|---|---|---|
| README nagłówek/opis | `README.md:1,23-25` | „Bafang Besst… / EBICS Ride Core" | marka |
| Tytuł strony (zakładka przeglądarki) | `ui/index.html:6` | `Bafang CANable Pro` | marka |
| Nagłówek widoczny w UI | `ui/index.html:24` | „Bafang / EBiCS CANable Pro" | marka |
| Etykiety zakładek | `ui/index.html:52-59` | „EBICS Live/Profiles/Torque/…" | marka (tekst) — patrz decyzja D1 |
| Kickery / tytuły sekcji | `ui/index.html:844-1231` | „EBICS Ride Core" itd. | marka (tekst) — patrz decyzja D1 |
| Identyfikatory zakładek | `ui/index.html` `data-tab="ebics-*"`, klasy `ebics-*` | — | **technika — ZOSTAJE** |
| Opisy/hinty w nowych kartach | `ui/index.html` (liczne `form-hint`, `info-message`, `warning-message` z „EBICS…") | teksty pomocnicze | marka (tekst) — patrz §6, zadanie „przegląd opisów" |
| Widoczne teksty wykrywania | `ui/js/ebics-detection.js:22-25,81-95` | „Controller: EBICS", „EBICS Ride Core bank probe…" | marka (tekst) |
| Stany/identyfikatory wykrywania | `ui/js/ebics-detection.js` `CONTROLLER_FLAVOR.EBICS='ebics'`, `data-ebics-only` | — | **technika — ZOSTAJE** |
| Launcher | `Canable.bat:3,19,47,64` | „EBICS Canable" | marka |
| Nazwa pakietu | `package.json:2`, `package-lock.json` | `openbafang-cannable` | technika/publiczne — decyzja D2 |
| Opis pakietu | `package.json:4` | „CAN bus interface for Bafang…" | marka |
| Nazwa pliku wydania (exe) | `releases/openbafang-cannable-win-x64-v2.1.exe` | — | plik wynikowy |
| Myląca podpowiedź ścieżki logów | `ui/index.html:1556,1603` | „→ C:\snapshot\bafang_canable_pro\logs" | kosmetyka (przy okazji) |

### Repozytorium C: Aplikacja Android „BafangCon"
**Nie ma jej w bieżących katalogach roboczych.** Rebranding aplikacji (nazwa „eVistDrive Config", opis, ekran startowy, docelowo package name `org.vistdrive.config`) zrobimy osobno, gdy wskażesz repozytorium. Package name zmieniamy **później** (wpływa na aktualizacje) — zgodnie z Twoją specyfikacją.

---

## 4. Decyzje do podjęcia przed wdrożeniem

**D1 — Widoczne etykiety „EBICS Ride Core / EBICS Live / EBICS Torque" w UI CANable. → ROZSTRZYGNIĘTE: wariant (a).**
Właściciel potwierdził: nowe karty to produkt eVistDrive, stare fabryczne są wycofywane, cała konfiguracja i wszystkie opisy przechodzą na eVistDrive.
- (a) [WYBRANE] Zmienić cały widoczny tekst nowych kart na „eVistDrive …" (identyfikatory `ebics-*` w kodzie zostają).
- Obejmuje to również widoczne teksty wykrywania w `ebics-detection.js` (np. „Controller: EBICS" → „Controller: eVistDrive").

**D2 — `package.json` „name": `openbafang-cannable`.**
Techniczne, ale bywa widoczne. Opcje: zmienić na `vistdrive-config` / zostawić. Zmiana pociąga `package-lock.json` i pole `bin`. Rekomendacja: zmienić w **Etapie 3** (nie jest pilne, nic nie psuje przy `node server.js`).

**D3 — Schemat wersji firmware (Twój pkt 6).**
Dziś wersja = licznik typu `0.0187`, i **ta sama liczba jest nazwą pliku `.bin`**. Twoja specyfikacja proponuje `MAJOR.MINOR.PATCH` (np. `0.1.0`) oraz nazwy plików `eVistDrive-M820-v0.1.0.bin`. To wymaga **rozdzielenia** (patrz §5). Opcje:
- (a) Zostawić obecny licznik `0.0187` na HMI (`eVD 0.0187`), zmienić tylko nazwę pliku wynikowego na `eVistDrive-M820-0.0187.bin`.
- (b) Przejść na `0.1.0` i pokazywać `eVD 0.1.0` + plik `eVistDrive-M820-v0.1.0.bin`.
- Rekomendacja: **(a) na teraz** (nie ruszamy sprawdzonego licznika, którego używa też logika buildu), a przejście na `0.1.0` przy pierwszym publicznym wydaniu eVistDrive.

---

## 5. Ważna zależność techniczna: wersja HMI ↔ nazwa pliku

W `build_firmware.ps1` zmienna `$ArtifactName` (np. `0.0187`) jest używana **jednocześnie**:
- jako `EBICS_BUILD_VERSION` → napis na wyświetlaczu (`src/CAN_Display.c:597`),
- oraz jako nazwa plików `.bin/.hex` (`build_firmware.ps1:343-383`).

Skutek: „napis na HMI" i „nazwa pliku wynikowego" są dziś **tym samym łańcuchem**. Żeby napis brzmiał `eVD 0.0187`, a plik nazywał się `eVistDrive-M820-0.0187.bin`, trzeba w skrypcie budowania rozdzielić:
- **prefix napisu HMI**: `"EBICS "` → `"eVD "` (w `CAN_Display.c:597`, format `"eVD %s"`),
- **nazwa pliku**: dodać prefix `eVistDrive-M820-` do `$ArtifactName` przy zapisie `.bin/.hex` (skrypt), **bez** zmiany `$ArtifactName` używanego w wersji.

To jest jedyna zmiana „techniczna" w Etapie 2 po stronie firmware i wymaga ostrożności (dotyka skryptu, który wg pamięci wstrzykuje też inne rzeczy — uruchamiany tylko na wyraźne polecenie).

---

## 6. Plan etapowy (kolejność wdrażania po akceptacji)

### ETAP 1 — Branding tekstowy (zero ryzyka, nic wykonawczego)
Commit A (repo firmware):
- `README.md` → nagłówek `# eVistDrive Firmware` + opis wg Twojego pkt 10, z zachowaniem informacji o pochodzeniu (EBiCS/ride core) i licencji GPL; dopisać zdanie z pkt 8 („eVistDrive is derived from EBiCS…").

Commit B (repo CANable) — **PRZENIESIONE DO OPEN** (README to dokumentacja, nie „nowa karta"; czekamy na decyzję o powłoce aplikacji).

### ETAP 2 — To, co widzi użytkownik + pliki wynikowe
Zakres potwierdzony: **canable + firmware**.

Commit C (repo firmware):
- `src/CAN_Display.c:597`: `"EBICS %s"` → `"eVD %s"` (pola 0x6000/0x6002 bez zmian).
- `documentation/CAN_PROTOCOL_REFERENCE.md:262`: zaktualizować opis do `"eVD %s"`.
- (jeśli D3=a lub b) `build_firmware.ps1`: prefix nazwy pliku wynikowego `eVistDrive-M820-…` (patrz §5). **Bez budowania** — sam kod skryptu.

> **Bezpieczeństwo jazdy (zweryfikowane w kodzie):** zmiana `0x6001` NIE wpływa na jazdę. To pasywne pole „serial/customer" (fabrycznie „FAKE TAXI"), odpytywane tylko przy ekranie info sterownika. Sterowanie (moment/kadencja/prędkość/prąd) to inne ramki (CAN_Display.c:425+). Pola identyfikacyjne `0x6000`/`0x6002` zostają fabryczne, więc wyświetlacz nadal rozpoznaje sterownik. Bufor `tx_data[64]`, „eVD 0.0187" (dopełn. do 20 B) mieści się z zapasem. Zmiana zadziała dopiero po przebudowie i wgraniu firmware.

Commit D (repo CANable) — WYŁĄCZNIE nowe karty + etykiety wykrywania (zgodnie z doprecyzowaniem: nazwy zmieniamy tylko w nowych kartach, pojawiają się po wykryciu firmware eVistDrive):
- Etykiety zakładek i kickery „EBICS Live / Profiles / Torque / Ride Core …" → „eVistDrive …" (tylko tekst widoczny; `data-tab`/klasy `ebics-*` **bez zmian**).
- **Opisy pomocnicze w nowych kartach** (`form-hint`, `info-message`, `warning-message`): w nowych kartach jest **jeden opis** i w całości przechodzi na **eVistDrive** (w tym „Needs EBICS firmware…" → „Needs eVistDrive firmware…"). Bez podwójnych nazw w nowych kartach.
- **UWAGA — NIE dotykać starych kart.** Podwójne opisy „Bafang / EBiCS" występują **tylko w starych kartach**; tam „EBiCS" **zostaje nietknięty**. Przy edycji nowych kart uważać, żeby nie podmienić przez przypadek opisów EBiCS w starych kartach.
- Widoczne teksty wykrywania w `ui/js/ebics-detection.js:22-25,81-95,99`, pokazywane po wykryciu eVistDrive: „Controller: EBICS" → „Controller: eVistDrive", „EBICS Ride Core bank probe" → „eVistDrive Ride Core bank probe" itd. Identyfikatory `CONTROLLER_FLAVOR.EBICS='ebics'` i `data-ebics-only` **bez zmian**.

**NIE ruszamy w tym etapie (zgodnie z doprecyzowaniem):**
- **Stare fabryczne karty Bafang** — ani nazwy, ani konfiguracji. Zero zmian, bez etykiet „Legacy".
- **Powłoka aplikacji**: `<title>` (`ui/index.html:6`), górny nagłówek (`ui/index.html:24`), `Canable.bat`, `package.json` — zostają „Bafang" na razie.

### OPEN — do osobnej decyzji (poza bieżącym zakresem)
- Czy i kiedy zmienić powłokę aplikacji CANable na „eVistDrive Config" (tytuł, nagłówek, launcher, package.json).
- Czy zmieniać `README.md` narzędzia CANable (to dokumentacja, nie „karta"); firmware README (Commit A) traktujemy osobno.
- Kosmetyka: myląca podpowiedź `→ C:\snapshot\…` (`ui/index.html:1556,1603`) — poprawka niezwiązana z marką, można zrobić przy okazji.

### ETAP 3 — Porządki w kodzie (ostrożnie, opcjonalnie)
- (jeśli D2) `package.json`/`package-lock.json` name → `vistdrive-config`.
- Nowe moduły firmware zakładane od teraz: nazwy `vistdrive_*` / neutralne (`rider_assist.c`, `torque_control.c`).
- **Zostają bez zmian**: `EBICS_BUILD_VERSION` (nazwa makra), identyfikatory `ebics-*` w UI, `ebics_foc.c`, historia commitów, pliki licencji.

---

## 7. Czego ten plan świadomie NIE robi
- Nie zmienia nazw fizycznych folderów repozytoriów (`BAFANG_GD32F303RCT6`, `bafang_canable_pro`) — to osobna, ryzykowna operacja (ścieżki, skrypty, `git`); do decyzji później.
- Nie zmienia package name aplikacji Android.
- Nie rusza ramek CAN poza jednym polem tekstowym `0x6001`.
- Nie uruchamia żadnego builda.

---

## 8. Następny krok
Zakres potwierdzony: **canable + firmware**. Kolejność wdrożenia (osobne commity, bez budowania):
1. Commit A — firmware README.
2. Commit C — firmware HMI (`eVD %s`) + doc protokołu (+ opcjonalnie prefix nazw plików wg D3).
3. Commit D — nowe karty CANable + etykiety wykrywania.

Powłoka aplikacji CANable, README CANable oraz decyzje D2/D3 pozostają w OPEN do osobnego potwierdzenia. Po Twoim „start" wchodzę i wracam po każdym commicie.

---

## 9. Dziennik wdrożenia (2026-07-23)

**Firmware (`BAFANG_GD32F303RCT6`):**
- `README.md` → nagłówek `# eVistDrive Firmware`, opis + nota o pochodzeniu EBiCS/ride core, licencja GPL i wszystkie ostrzeżenia zachowane.
- `src/CAN_Display.c` → pole HMI `0x6001`: `"EBICS %s"` → `"eVD %s"` (+ komentarze). Pola `0x6000`/`0x6002` bez zmian. Makro `EBICS_BUILD_VERSION` bez zmian.
- `documentation/CAN_PROTOCOL_REFERENCE.md` → opis pola `0x6001` zaktualizowany do `eVD`.

**CANable — TYLKO nowe karty (`bafang_canable_pro`):**
- `ui/index.html` → wszystkie widoczne „EBICS" → „eVistDrive" (etykiety zakładek, kickery „Ride Core", opisy). Identyfikatory `ebics-*`/`data-ebics-only` i nagłówek powłoki „Bafang / EBiCS" nietknięte.
- `ui/js/ebics-detection.js` → widoczne napisy wykrywania („Controller: eVistDrive", „eVistDrive Ride Core bank probe" itd.). Identyfikatory `CONTROLLER_FLAVOR.EBICS`, `EBICS_UI_PREVIEW_WITHOUT_DETECTION` bez zmian.
- `ui/js/tab-ebics.js`, `ui/js/ebics-compat.js` → widoczne napisy log/confirm („eVistDrive bank/tuning/compatibility/System…"). Identyfikator `EBICS_MV_PER_KG` bez zmian.
- `ui/js/websocket.js` → widoczny powód wykrycia → „eVistDrive Ride Core".
- Wszystkie 4 pliki JS przechodzą `node --check`.

**Świadomie NIE ruszone:** stare karty Bafang, powłoka aplikacji (tytuł, nagłówek, `Canable.bat`, `package.json`), flaga podglądu `EBICS_UI_PREVIEW_WITHOUT_DETECTION=true` (na razie wszystkie karty widoczne do testów), nazwy plików `.bin`.

**Commit:** NIE wykonany. Oba repo miały wcześniejszy, niepowiązany WIP (firmware: FW-018 w `CAN_Display.c`/`config.h`/`main.c`…; canable: `server.js`/`index.html`…). Rebranding celowo nie został zmieszany z tym WIP — sposób podziału na commity do decyzji właściciela.
