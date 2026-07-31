# Nadrzędny plan uporządkowania i publikacji projektu

> **STATUS:** REALIZACJA ROZPOCZĘTA — infrastruktura Fazy 1 wdrożona;
> sprzętowe domknięcie Fazy 0 oczekuje.
>
> To jest nadrzędne źródło prawdy dla wniosków z audytu obecnego kodu,
> bezpiecznego refaktoru oraz przygotowania projektu do publicznej publikacji.
> Szczegółowe podplany rozwijają wybrane obszary, ale bieżąca faza i kolejność
> całego zadania są utrzymywane tutaj.

Aktualizacja: 2026-07-31.

## 1. Stan wznowienia prac

| Pole | Aktualna wartość |
|---|---|
| Bieżąca faza | **Faza 1 — odtwarzalny build i bezpieczna mapa; równolegle oczekuje sprzętowy punkt bazowy Fazy 0** |
| Ostatni zakończony krok | FW-067: stały COAST 85/70 rpm zastąpiono progami zależnymi od aktywnego celu banku: `target+20 rpm` dla `Iq=0` i `target+5 rpm` dla wznowienia. Zakres celu 20–60 daje bezpieczne progi 40/25…80/65 rpm. Testy 5/5 i oba buildy `0.0264` przeszły |
| Następna czynność | Wgrać wyłącznie normalny `0.0264` i wykonać test stojakowy na najniższym biegu. Kryteria: zdecydowany, płynny start około 0,85 s; spokojny RUN bez szarpania; dla celu 50 rpm COAST około 70 rpm i wznowienie około 55 rpm, bez ponownego START. Test na ziemi nadal zablokowany |
| Implementacja porządków | Infrastruktura build/linker wdrożona; algorytm zwykłej jazdy nie został zmieniony, Walk Assist jest iteracyjnie poprawiany na podstawie testów stojakowych |
| Znany dobry build sprzętowy | `0.0206` — potwierdzona jazda; `0.0258` — odrzucony przez rozpędzanie WA; `0.0259` — odrzucony przez szarpanie; `0.0260` — odrzucony przez agresywne cykle; `0.0261`, `0.0262` i `0.0263` — zastąpione przed testem; `0.0264` — kandydat FW-067 |
| HEAD podczas utworzenia planu | `992c910` |
| Stan repo podczas utworzenia | 22 zmodyfikowane pliki śledzone, 49 nieśledzonych pozycji; nie wolno traktować samego SHA jako kompletnego stanu |
| Szczegółowy aktywny podplan | [Flash, storage i pojemność konfiguracji](PLAN_FLASH_CONFIG_STORAGE_EXPANSION_PL.md) — mapa Fazy 2 wykonana, zamrożenie bazy oczekuje |
| Inwentaryzacja Fazy 0 | [Punkt bazowy i klasyfikacja worktree](PROJECT_CLEANUP_BASELINE_INVENTORY_PL.md) |

Po każdej przerwie należy:

1. Przeczytać tę tabelę.
2. Przeczytać ostatni wpis w dzienniku prac.
3. Sprawdzić `git status`.
4. Otworzyć pierwszy niezaznaczony checkbox w bieżącej fazie.
5. Jeżeli aktywny jest podplan, sprawdzić również jego tabelę wznowienia.

## 2. Cel końcowy

Projekt jest gotowy do publicznej publikacji dopiero wtedy, gdy:

- czysty klon buduje się udokumentowaną komendą;
- build jest odtwarzalny i nie modyfikuje plików źródłowych;
- linker chroni bootloader, konfigurację i SOC;
- kod współdzielony z przerwaniami ma jawne zasady własności i atomowości;
- błędne ramki CAN nie mogą uszkodzić pamięci ani zastosować częściowych danych;
- zapis Flash nie odbywa się podczas aktywnej jazdy;
- istnieje bezpieczny, wersjonowany format konfiguracji z migracją;
- `main.c` jest punktem uruchomienia i harmonogramem, a nie magazynem całej
  logiki;
- transport CAN, protokół, diagnostyka i serializacja są rozdzielone;
- Walk Assist i kalibracja Halla nie zależą od martwego silnika jazdy Legacy;
- najważniejsze algorytmy wykonują ten sam testowany kod C, a nie tylko kopię
  modelu w JavaScript/PowerShell;
- istnieje profil release po audycie ISR i ostrzeżeń;
- dokumentacja nie przeczy kodowi;
- licencje, pochodzenie i artefakty binarne są wyjaśnione;
- publiczna gałąź nie zawiera lokalnych archiwów, sekretów, ścieżek użytkownika
  ani przypadkowych zmian.

## 3. Hierarchia dokumentów

| Dokument | Rola |
|---|---|
| Ten dokument | Kolejność całego porządkowania, stan wznowienia i kryterium publikacji |
| [PLAN_FLASH_CONFIG_STORAGE_EXPANSION_PL.md](PLAN_FLASH_CONFIG_STORAGE_EXPANSION_PL.md) | Szczegółowy podplan linkera, Config A/B, migracji, pojemności blobów i bezpiecznego multiframe CAN |
| `RIDE_CORE_MASTER_CHECKLIST_PL.md` | Zachowanie i rozwój silnika wspomagania Ride Core |
| `RIDE_CORE_REFACTOR.md` | Zapis dotychczasowych decyzji architektonicznych Ride Core |
| `protocol/ebics_config_schema.yaml` | Schemat przyszłych parametrów i ich jednostek |
| `CHANGELOG.md` | Historia zmian funkcjonalnych, nie bieżący plan porządkowania |

Jeżeli dokumenty są sprzeczne w sprawie kolejności porządków i publikacji,
obowiązuje ten plan. W sprawie szczegółów konkretnego podsystemu obowiązuje jego
aktywny podplan.

## 4. Migawka audytu

### 4.1 Rozmiar i główne centra odpowiedzialności

| Element | Stan podczas audytu |
|---|---:|
| Kod projektu `src/*.c + inc/*.h` | 48 plików, około 11 456 linii |
| `src/main.c` | 3213 linii |
| `src/assist_modes.c` | 1284 linie |
| `src/CAN_Display.c` | 969 linii |
| `src/torque_input.c` | 805 linii |
| `main()` | około 429 linii |
| `reg_ADC_processing()` | około 374 linie |
| `sendCAN_Tx()` | około 272 linie |
| `processCAN_Rx()` | około 270 linii |
| Deklaracje `extern` w `CAN_Display.c` | 18 deklaracji obejmujących wiele pól stanu |
| Definicje w `config.h` | 186 |
| Definicje bez tekstowego użycia poza `config.h` | 44 — wymagają ręcznej klasyfikacji |

### 4.2 Pamięć

| Obszar | Stan podczas audytu |
|---|---:|
| Obraz aplikacji Flash | około 92 084 B |
| Bezpieczna przestrzeń aplikacji przed obecną stroną parametrów | 237 568 B |
| RAM `.data + .bss + heap/stack` | około 6 616 B z 48 KiB |
| Rekord konfiguracji na stronie Flash | około 600 B z 2048 B |
| Blob banku | 190 B z 192 B |
| Blob globalnego tuningu | 24 B z 24 B |

Wniosek: fizyczny Flash i RAM mają duży zapas. Wąskie gardła to formaty danych,
granice linkera oraz architektura zapisu.

### 4.3 Testy

Podczas audytu przeszły:

- `tests/fw016_ride_core_model.ps1`;
- `tests/fw056_power_curve.js`;
- `tests/fw057_cadence_comp.js`;
- `tests/fw058_coast_rezero.js`;
- `tests/fw060_walk_speed_controller.js`.

Ograniczenia:

- część testów kopiuje matematykę zamiast wykonywać kod C;
- część sprawdza tekst źródła;
- integracja z Canable jest cicho pomijana, gdy sąsiednie repo nie istnieje;
- brak wspólnego runnera i CI;
- brak automatycznego testu mapy pamięci, stosu i ostrzeżeń.

### 4.4 Mocne strony, które należy zachować

- nowe moduły Ride Core mają lepsze granice niż kod bazowy;
- istnieją testy modelowe najważniejszych nowych algorytmów;
- diagnostykę CAN można wyłączyć przy kompilacji;
- zapis parametrów ma stopkę commit-last i CRC;
- istnieją kontrole wiarygodności kalibracji Halla;
- dokumentacja zawiera dużo wartościowych wyników testów sprzętowych;
- historia pochodzenia projektu i upstream są nadal widoczne.

Porządki nie mogą zniszczyć tych zabezpieczeń ani usunąć historii decyzji bez
przeniesienia jej do właściwego archiwum.

## 5. Rejestr problemów i ryzyk

| ID | Priorytet | Problem | Kierunek rozwiązania |
|---|---|---|---|
| MEM-001 | KRYTYCZNY | Linker M820 pozwala aplikacji wejść w strony parametrów/SOC i poza fizyczny koniec Flash | Poprawna partycja i link-time `ASSERT` |
| IRQ-001 | KRYTYCZNY | Flagi zmieniane przez ISR nie mają spójnego `volatile`/atomowości | Audyt własności ISR, snapshoty i sekcje krytyczne |
| CAN-001 | KRYTYCZNY | Multiframe kopiuje dane bez pełnej kontroli pojemności | API z `capacity`, walidacja DLC, indeksu i sekwencji |
| CAN-002 | WYSOKI | Jeden globalny bufor RX może utracić poprzednią ramkę | Mała kolejka RX |
| HALL-001 | KRYTYCZNY | Dzielenie przez `ui16_timertics` bez ochrony przed zerem | Walidacja wejścia i licznik błędnych zdarzeń |
| TEMP-001 | WYSOKI | Otwarty TODO dla przerwy/zwarcia czujnika temperatury i fail-safe | Jawny stan usterki i bezpieczne ograniczenie/odcięcie |
| STO-001 | WYSOKI | Flash przechowuje surowy `MotorParams_t` zależny od paddingu | Jawny, wersjonowany serializer |
| STO-002 | WYSOKI | Nie wszystkie zapisy są odroczone do bezpiecznego postoju | Jedna kolejka/usługa persistence |
| BLD-001 | WYSOKI | Główny skrypt builda jest lokalnie wykluczony z Git | Śledzony, przenośny proces budowania |
| BLD-002 | WYSOKI | Build modyfikuje `config.h` i linker script oraz generuje lokalny numer wersji | Argumenty kompilatora, osobny katalog builda, wersja z taga/commita |
| BLD-003 | WYSOKI | Jedyny realny profil używa `-O0 -g3 -Wall` | Debug/Release po zakończeniu audytu ISR |
| BLD-004 | ŚREDNI | Ostrzeżenia pointer-sign i unused | Zero nowych ostrzeżeń, naprawa istniejących etapami; segment `RWE` usunięty w Fazie 1 |
| BLD-005 | KRYTYCZNY | `-Os` usuwa ścieżki zależne od współdzielonego stanu ISR; obraz release spada do ok. 22 KB, a wariant diagnostyczny nie spełnia kontraktu | Release zablokowany do zakończenia AUD-200…AUD-211 |
| ARC-001 | WYSOKI | `main.c` łączy sprzęt, sensory, scheduler, SOC, storage i Legacy | Podział według właściciela danych |
| ARC-002 | WYSOKI | `CAN_Display.c` łączy driver, transport, protokół, konfigurację i diagnostykę | Rozdzielenie warstw CAN |
| ARC-003 | WYSOKI | Migracja do jednego ride core zatrzymała się w połowie | Wydzielić Walk i Hall, potem usunąć martwą jazdę Legacy |
| API-001 | WYSOKI | Duża liczba globali i bezpośrednich `extern` | Konteksty modułów, snapshot telemetrii, prywatne `static` |
| CFG-001 | ŚREDNI | `config.h` miesza sprzęt, tuning, flagi builda i historię | Podział konfiguracji i audyt użycia |
| TST-001 | WYSOKI | Testy nie wykonują wystarczająco dużo tego samego kodu C | Hostowe testy C i warstwa HAL do podmiany |
| DEP-001 | ŚREDNI | Generator LUT zapisuje do zakodowanego sąsiedniego repo Canable | Jawny parametr wyjścia lub osobny artefakt integracyjny |
| DOC-001 | WYSOKI | README opisuje inny kontroler niż bieżąca gałąź M820 | Jednoznaczny zakres sprzętu i stan projektu |
| DOC-002 | ŚREDNI | Dokumenty Legacy/ride core i statusy wdrożeń są sprzeczne | Aktualne źródła prawdy + archiwum |
| STYLE-001 | ŚREDNI | Mieszane języki, nazwy, kod zakomentowany i komentarze historyczne | Polityka komentarzy/nazewnictwa bez ryzykownej masowej zmiany |
| GIT-001 | WYSOKI | Duży, brudny worktree i liczne nieśledzone źródła/dokumenty | Inwentaryzacja i logiczne commity |
| PUB-001 | BLOKER PUBLIKACJI | Brak pliku licencji i spisu licencji zależności | `LICENSE` i `THIRD_PARTY_NOTICES.md` po weryfikacji |
| PUB-002 | BLOKER PUBLIKACJI | Niejasne pochodzenie `M820.bin`, bootloadera, ZIP i części binariów | Weryfikacja, usunięcie albo opisany artefakt wydania |
| PUB-003 | ŚREDNI | Historia Git ujawnia prywatny i firmowy adres autora | Świadoma decyzja przed publikacją, bez usuwania autorstwa innych |

## 6. Docelowe granice architektury

```text
platform/gd32
  ISR, ADC, timery, CAN driver, Flash driver, watchdog
        │
        ▼
sensors ──► rider_input_snapshot
                │
                ▼
          ride_control
                │
                ▼
       safety_and_limits
                │
                ▼
           motor_core ──► FOC

CAN RX queue ──► CAN transport ──► HMI protocol/config service
                                          │
                                          ▼
                                  versioned storage

diagnostics/telemetry ◄── read-only snapshots modułów
```

Zasady:

- `main.c` uruchamia moduły i wykonuje harmonogram;
- ISR wykonuje minimalną pracę i publikuje zdarzenia/snapshoty;
- tylko jeden moduł jest właścicielem danego stanu zapisywalnego;
- diagnostyka nie modyfikuje sterowania;
- protokół CAN nie odwołuje się bezpośrednio do przypadkowych globali;
- runtime model i format zapisu Flash są osobnymi strukturami;
- API publiczne modułu jest w nagłówku, reszta funkcji i stanu ma być `static`;
- kod vendor SDK pozostaje wydzielony i nie jest masowo formatowany.

## 7. Plan realizacji

### Faza 0 — zakres, inwentaryzacja i punkt bazowy

- [x] **AUD-000** Potwierdzić, że ten dokument jest nadrzędnym planem porządków.
- [x] **AUD-001** Ustalić wspierany sprzęt: tylko M820/BL820 czy również inne
      kontrolery i bootloadery.
- [ ] **AUD-002** Uzupełnić listę przyszłych funkcji i zmiennych w podplanie
      Flash/storage.
- [x] **AUD-003** Sklasyfikować 22 zmodyfikowane pliki: gotowe, w toku,
      eksperymentalne, niezwiązane.
- [x] **AUD-004** Sklasyfikować 49 nieśledzonych pozycji i nie dodawać zbiorczo
      archiwów/binarek.
- [ ] **AUD-005** Wybrać build potwierdzony testem sprzętowym.
- [ ] **AUD-006** Zapisać SHA-256 binarki, commit, stan worktree i ustawienia
      roweru.
- [x] **AUD-007** Uruchomić pełny zestaw obecnych testów.
- [x] **AUD-008** Utworzyć bezpieczną gałąź porządków bez resetowania
      istniejących zmian.
- [ ] **AUD-009** Zaktualizować tabelę wznowienia na Fazę 1.

Warunek zakończenia: wiadomo, co jest produktem, co historią, co eksperymentem i
do jakiego działającego punktu można wrócić.

Stan częściowy AUD-006: zapisano SHA-256 `0.0258`, HEAD, gałąź i stan worktree.
Brakuje eksportu ustawień roweru oraz powiązania potwierdzonej binarki z
checkpointem.

### Faza 1 — odtwarzalny build i bezpieczna mapa pamięci

- [x] **AUD-100** Przenieść/gotowy skrypt builda do śledzonego katalogu
      `scripts/` bez lokalnego `.git/info/exclude`.
- [x] **AUD-101** Build nie może trwale modyfikować `config.h` ani linker
      scriptu.
- [x] **AUD-102** Zdefiniować jawny target sprzętowy i profil `debug/release`.
- [x] **AUD-103** Przypiąć i udokumentować obsługiwaną wersję toolchaina.
- [x] **AUD-104** Wersję firmware wyprowadzać z taga/commita i jawnego parametru,
      nie z historii lokalnego katalogu `.build`.
- [x] **AUD-105** Skompilować tylko używane sterowniki vendor albo jawnie
      utrzymywać listę źródeł.
- [ ] **AUD-106** Wykonać FS-100…FS-208 z podplanu Flash/storage.
- [x] **AUD-107** Dodać raport rozmiaru Flash/RAM i mapę jako wynik builda.
- [x] **AUD-108** Dodać kontrolę, że normalny build nie zawiera diagnostyki CAN.
- [x] **AUD-109** Budować wariant normalny i diagnostyczny.
- [x] **AUD-110** Dodać `.gitattributes` i ustalić LF dla źródeł/skryptów.
- [ ] **AUD-111** Osobny commit builda; bez zmian algorytmów jazdy.
- [ ] **AUD-112** Zaktualizować stan planu na Fazę 2.

Warunek zakończenia: czysty klon buduje ten sam target, a linker nie pozwala
wejść w dane trwałe.

Profil `debug` jest jedynym profilem dopuszczonym do sprzętu. Profil `release`
jest zdefiniowany, lecz skrypt domyślnie go blokuje do zakończenia audytu
ISR/`volatile`; próba diagnostycznego `-Os` została prawidłowo odrzucona przez
kontrolę symboli.

### Faza 2 — krytyczne bezpieczeństwo runtime

- [ ] **AUD-200** Sporządzić listę wszystkich danych zapisywanych przez ISR i
      czytanych poza ISR.
- [ ] **AUD-201** Nadać każdemu polu właściciela i metodę publikacji:
      `volatile`, atomowy snapshot albo sekcja krytyczna.
- [ ] **AUD-202** Naprawić `receive_flag`, `reg_ADC_flag`, `PAS_flag` i
      `Speed_flag`.
- [ ] **AUD-203** Sprawdzić spójność wielobajtowych wartości Hall/ERPS i
      liczników pomiędzy ISR a główną pętlą.
- [ ] **AUD-204** Zabezpieczyć dzielenie przez zero w ścieżce Halla.
- [ ] **AUD-205** Zliczać i raportować odrzucone/glitchowe zdarzenia Halla.
- [ ] **AUD-206** Zaimplementować fail-safe dla przerwy/zwarcia czujnika
      temperatury.
- [ ] **AUD-207** Określić los istniejącego workaroundu FOC „brak kontroli
      prądu”: zachować z kontraktem albo zastąpić.
- [ ] **AUD-208** Zapewnić natychmiastowe pierwszeństwo hamulca, cofania,
      błędów czujników i utraty komunikacji.
- [ ] **AUD-209** Dodać testy graniczne i fault injection dla nowych zabezpieczeń.
- [ ] **AUD-210** Zmierzyć czas pętli 4 kHz po zmianach.
- [ ] **AUD-211** Wykonać test na stojaku przed jazdą.
- [ ] **AUD-212** Osobne commity dla niezależnych zabezpieczeń.
- [ ] **AUD-213** Zaktualizować stan planu na Fazę 3.

Warunek zakończenia: optymalizator nie może ukryć aktualizacji ISR, a błędny
sygnał nie prowadzi do dzielenia przez zero lub niekontrolowanego momentu.

### Faza 3 — CAN, konfiguracja i trwały zapis

- [ ] **AUD-300** Wprowadzić kolejkę RX zamiast jednego nadpisywanego
      `receive_message`.
- [ ] **AUD-301** Określić politykę przepełnienia kolejki i licznik utraconych
      ramek.
- [ ] **AUD-302** Wykonać FS-300…FS-509 z podplanu Flash/storage.
- [ ] **AUD-303** Wszystkie zapisy konfiguracji skierować do jednej usługi
      persistence.
- [ ] **AUD-304** Odraczać zapis do zatrzymania napędu, PWM i Walk Assist.
- [ ] **AUD-305** Usunąć busy-wait i duplikację z wysyłania CAN albo wprowadzić
      kontrolowany scheduler TX.
- [ ] **AUD-306** Obsłużyć timeout i błąd nadajnika, zamiast tylko kończyć pętlę.
- [ ] **AUD-307** Wykonać FS-600…FS-609 i zatwierdzić nowe pojemności blobów.
- [ ] **AUD-308** Zaktualizować schemat oraz kompatybilny parser Canable.
- [ ] **AUD-309** Wykonać migrację i testy sprzętowe FS-800…FS-809.
- [ ] **AUD-310** Osobne commity: transport, storage, migracja, rozmiary schema.
- [ ] **AUD-311** Zaktualizować stan planu na Fazę 4.

Warunek zakończenia: błędny CAN nie zapisuje poza buforem, a zanik zasilania nie
niszczy jedynej poprawnej konfiguracji.

### Faza 4 — dokończenie migracji z Legacy do jednego Ride Core

- [ ] **AUD-400** Udokumentować dokładne wywołania monolitu Legacy, które nadal
      są aktywne.
- [ ] **AUD-401** Wydzielić obsługę Walk Assist do jawnej usługi bez wywołania
      silnika jazdy Legacy.
- [ ] **AUD-402** Wydzielić kalibrację Halla do osobnej maszyny stanów/usługi.
- [ ] **AUD-403** Zachować kolejność zabezpieczeń i zerowania Iq.
- [ ] **AUD-404** Dodać testy równoważności zachowania przed/po ekstrakcji.
- [ ] **AUD-405** Potwierdzić Walk Assist na stojaku i na ziemi.
- [ ] **AUD-406** Potwierdzić kalibrację oraz trwałość Halla.
- [ ] **AUD-407** Usunąć martwą pedałową ścieżkę Legacy.
- [ ] **AUD-408** Usunąć `RIDE_ENGINE_DEFAULT`, wybór silnika i telemetrię
      udającą aktywny wybór.
- [ ] **AUD-409** Usunąć `legacy_assist` dopiero gdy nie ma aktywnych wywołań.
- [ ] **AUD-410** Zaktualizować sprzeczne dokumenty Legacy/ride core.
- [ ] **AUD-411** Zaktualizować stan planu na Fazę 5.

Warunek zakończenia: istnieje jeden silnik jazdy, a Walk/Hall są niezależnymi,
nazwanymi funkcjami systemu.

### Faza 5 — podział `main.c`

- [ ] **AUD-500** Zostawić w `main.c` start systemu i czytelny harmonogram.
- [ ] **AUD-501** Wydzielić konfigurację sprzętu/platformy GD32.
- [ ] **AUD-502** Wydzielić moduł Halla i pomiaru ERPS.
- [ ] **AUD-503** Wydzielić PAS/kadencję i kierunek.
- [ ] **AUD-504** Wydzielić prędkość koła.
- [ ] **AUD-505** Wydzielić ADC i przeliczanie wejść.
- [ ] **AUD-506** Wydzielić temperaturę i jej zabezpieczenia.
- [ ] **AUD-507** Wydzielić SOC/range.
- [ ] **AUD-508** Wydzielić storage/EEPROM zgodnie z Fazą 3.
- [ ] **AUD-509** Zmniejszyć liczbę globalnych funkcji; prywatne oznaczyć
      `static`.
- [ ] **AUD-510** Po każdym przeniesieniu wykonać build/test, bez zmiany
      zachowania.
- [ ] **AUD-511** Nie wykonywać zbiorczego formatowania w commitach przenoszących
      kod.
- [ ] **AUD-512** Zaktualizować stan planu na Fazę 6.

Warunek zakończenia: odpowiedzialność funkcji można rozpoznać z nazwy modułu, a
`main.c` nie zawiera implementacji podsystemów.

### Faza 6 — podział `CAN_Display.c`

- [ ] **AUD-600** Wydzielić niskopoziomowy driver/adapter CAN.
- [ ] **AUD-601** Wydzielić kolejkę RX i scheduler TX.
- [ ] **AUD-602** Wydzielić transport single-frame/multiframe.
- [ ] **AUD-603** Wydzielić dekodowanie identyfikatora i dispatcher komend HMI.
- [ ] **AUD-604** Wydzielić serializację banków/tuningu/torque.
- [ ] **AUD-605** Wydzielić diagnostykę do modułu kompilowanego warunkowo.
- [ ] **AUD-606** Wprowadzić read-only `telemetry_snapshot_t`.
- [ ] **AUD-607** Zastąpić bezpośrednie `extern` wywołaniami API/snapshotem.
- [ ] **AUD-608** Wprowadzić `const uint8_t *`/`uint8_t *` zamiast mieszania
      `char *`.
- [ ] **AUD-609** Ujednolicić funkcję wysyłania i raportowanie błędów.
- [ ] **AUD-610** Dodać tabelę obsługiwanych ID, kierunku, DLC i wersji payloadu.
- [ ] **AUD-611** Dodać testy każdej aktywnej komendy i nieznanych ramek.
- [ ] **AUD-612** Potwierdzić obciążenie magistrali w normalnym buildzie.
- [ ] **AUD-613** Potwierdzić pełną diagnostykę w buildzie developerskim.
- [ ] **AUD-614** Zaktualizować stan planu na Fazę 7.

Warunek zakończenia: normalna komunikacja, konfiguracja i diagnostyka są
niezależnymi warstwami.

### Faza 7 — konfiguracja, zależności i martwy kod

- [ ] **AUD-700** Podzielić `config.h` na sprzęt, build, bezpieczeństwo i tuning
      albo jawnie wydzielić sekcje z właścicielami.
- [ ] **AUD-701** Sklasyfikować wszystkie 44 definicje bez użycia.
- [ ] **AUD-702** Usuwać stałe dopiero po sprawdzeniu kodu, EEPROM, CAN i
      Canable.
- [ ] **AUD-703** Usunąć zakomentowany martwy kod, którego historia jest w Git.
- [ ] **AUD-704** Zachować komentarze opisujące sprzęt, bezpieczeństwo i
      nieoczywiste inwarianty.
- [ ] **AUD-705** Usunąć przestarzałe selektory Legacy po Fazie 4.
- [ ] **AUD-706** Rozstrzygnąć niezaimplementowany Speed PLL i stary FOC
      workaround.
- [ ] **AUD-707** Oddzielić kod projektu od vendor SDK i biblioteki CMSIS.
- [ ] **AUD-708** Nie formatować automatycznie vendor SDK.
- [ ] **AUD-709** Usunąć zakodowaną ścieżkę do sąsiedniego Canable z generatora.
- [ ] **AUD-710** Generator ma domyślnie zapisywać wyłącznie w bieżącym repo;
      zewnętrzny output tylko przez jawny argument.
- [ ] **AUD-711** Oznaczyć wygenerowane pliki i sprawdzać ich aktualność w teście.
- [ ] **AUD-712** Zmierzyć rozmiar i zależności po porządkach.
- [ ] **AUD-713** Osobne commity dla usuwania martwego kodu i zmian generatora.
- [ ] **AUD-714** Zaktualizować stan planu na Fazę 8.

Warunek zakończenia: aktywny kod i konfiguracja nie zawierają niewyjaśnionych
pozostałości ani ukrytych lokalnych zależności.

### Faza 8 — testy, analiza statyczna i profil release

- [ ] **AUD-800** Dodać jeden runner uruchamiający wszystkie testy hostowe.
- [ ] **AUD-801** Przenieść krytyczną matematykę do C możliwego do kompilacji na
      hoście.
- [ ] **AUD-802** Testować bezpośrednio kod C dla Ride Core, torque, Walk i
      serializacji.
- [ ] **AUD-803** Pozostawić modele JS tylko jako niezależne testy
      właściwości/referencje.
- [ ] **AUD-804** Usunąć kruche asercje sprawdzające tekst, gdy istnieje test API.
- [ ] **AUD-805** Dodać testy parsera CAN na granice i błędne ramki.
- [ ] **AUD-806** Dodać testy kodeków oraz migracji wszystkich wersji.
- [ ] **AUD-807** Dodać analizę statyczną dla kodu projektu, wykluczając lub
      osobno traktując vendor SDK.
- [ ] **AUD-808** Włączyć `-std=c11`, `-Wextra` i wybrane ostrzeżenia etapami.
- [ ] **AUD-809** Naprawić istniejące ostrzeżenia projektu.
- [ ] **AUD-810** Nie używać `-Werror` dla vendor SDK.
- [ ] **AUD-811** Wygenerować raport zużycia stosu i zmierzyć high-water mark.
- [ ] **AUD-812** Po audycie ISR wprowadzić profil release `-Os` lub `-O2`.
- [ ] **AUD-813** Porównać zachowanie debug/release na stojaku.
- [ ] **AUD-814** Dodać CI: testy, normalny build, diagnostyczny build, rozmiar,
      ostrzeżenia i kontrola wygenerowanych plików.
- [ ] **AUD-815** Zaktualizować stan planu na Fazę 9.

Warunek zakończenia: release nie polega na zachowaniu `-O0`, a regresje są
wykrywane automatycznie.

### Faza 9 — nazewnictwo, komentarze i dokumentacja

- [ ] **AUD-900** Ustalić angielski jako język kodu/API; dokumentacja użytkownika
      może pozostać po polsku.
- [ ] **AUD-901** Ustalić nazwy zawierające jednostki, np. `_mv`, `_rpm`,
      `_x100`, `_ticks`.
- [ ] **AUD-902** Nie zmieniać bez migracji nazw/pól będących częścią EEPROM lub
      wire protocol.
- [ ] **AUD-903** Opisać przy API: kontekst ISR/main, częstotliwość, jednostki,
      zakres, side effects i błędy.
- [ ] **AUD-904** Przenieść kronikę `FW-xxx` z komentarzy do dokumentów
      historii; w kodzie zostawić aktualne „dlaczego”.
- [ ] **AUD-905** Poprawić błędy kodowania i literówki w aktywnym kodzie.
- [ ] **AUD-906** Napisać aktualny README: M820, stan projektu, bezpieczeństwo,
      build, flash, konfiguracja i ograniczenia.
- [ ] **AUD-907** Dodać `docs/architecture.md`.
- [ ] **AUD-908** Dodać `docs/build.md`.
- [ ] **AUD-909** Dodać `docs/testing.md`.
- [ ] **AUD-910** Uporządkować dokumentację konfiguracji i protokołu.
- [ ] **AUD-911** Przenieść karty zakończonych eksperymentów do
      `documentation/history/` albo oznaczyć je archiwalnie.
- [ ] **AUD-912** Usunąć sprzeczności Legacy/ride core i stare nazwy gałęzi.
- [ ] **AUD-913** Usunąć lokalne ścieżki `C:\...` z aktualnej dokumentacji.
- [ ] **AUD-914** Skrócić changelog do zmian wydań; dzienniki laboratoryjne
      przenieść do historii.
- [ ] **AUD-915** Zaktualizować stan planu na Fazę 10.

Warunek zakończenia: nowy developer potrafi z README znaleźć build, architekturę,
konfigurację, testy i aktualne ograniczenia bez czytania całej historii.

### Faza 10 — licencje, Git i higiena publikacji

- [ ] **AUD-1000** Ustalić licencję kodu pochodnego zgodnie z zachowanymi
      nagłówkami i źródłem projektu.
- [ ] **AUD-1001** Dodać właściwy plik `LICENSE`.
- [ ] **AUD-1002** Dodać `THIRD_PARTY_NOTICES.md` dla GD32, CMSIS, biblioteki
      ARM i innych zależności.
- [ ] **AUD-1003** Zachować oryginalne informacje o autorach i pochodzeniu forka.
- [ ] **AUD-1004** Zweryfikować pochodzenie i możliwość publikacji
      `documentation/M820.bin`.
- [ ] **AUD-1005** Zweryfikować bootloader i `libarm_cortexM4lf_math.a`.
- [ ] **AUD-1006** Nie dodawać `WA_sterowanie_0.0229.zip` do źródeł.
- [ ] **AUD-1007** Binarki firmware publikować jako opisane artefakty wydań z
      SHA-256, nie przypadkowe pliki w dokumentacji.
- [ ] **AUD-1008** Sprawdzić repo i historię pod kątem sekretów oraz danych
      prywatnych.
- [ ] **AUD-1009** Podjąć decyzję w sprawie firmowego/prywatnego adresu autora w
      historii, bez przepisywania autorstwa innych osób.
- [ ] **AUD-1010** Dodać `CONTRIBUTING.md`.
- [ ] **AUD-1011** Dodać `.editorconfig` i konfigurację formatowania kodu
      projektu.
- [ ] **AUD-1012** Uporządkować `.gitignore`; nie polegać na prywatnym
      `.git/info/exclude` dla plików potrzebnych developerowi.
- [ ] **AUD-1013** Nie publikować lokalnych gałęzi `wip`/`broken` jako gałęzi
      użytkowych.
- [ ] **AUD-1014** Podzielić obecny duży worktree na logiczne, opisane commity.
- [ ] **AUD-1015** Wykonać przegląd czystego klona w nowym katalogu.
- [ ] **AUD-1016** Zaktualizować stan planu na Fazę 11.

Warunek zakończenia: repo ma wyjaśnione prawa do publikowanych plików, czystą
strukturę i nie wymaga lokalnej wiedzy autora.

### Faza 11 — kandydat publicznego wydania

- [ ] **AUD-1100** Czysty clone + build normalny.
- [ ] **AUD-1101** Czysty clone + build diagnostyczny.
- [ ] **AUD-1102** Wszystkie testy hostowe przechodzą.
- [ ] **AUD-1103** Analiza statyczna bez nierozstrzygniętych błędów wysokiego
      ryzyka.
- [ ] **AUD-1104** Brak nowych ostrzeżeń projektu.
- [ ] **AUD-1105** Mapa pamięci potwierdza brak kolizji.
- [ ] **AUD-1106** Migracja ustawień ze wspieranej starej wersji przechodzi.
- [ ] **AUD-1107** Test stojakowy przechodzi.
- [ ] **AUD-1108** Test jazdy i Walk Assist przechodzi.
- [ ] **AUD-1109** Binarka ma wersję, commit, SHA-256 i instrukcję flashowania.
- [ ] **AUD-1110** README, changelog, licencje i dokumentacja odpowiadają
      wydaniu.
- [ ] **AUD-1111** Utworzyć tag release candidate, a następnie stabilny tag po
      potwierdzeniu.
- [ ] **AUD-1112** Oznaczyć ten plan jako ZAKOŃCZONY i przenieść dalsze pomysły
      do osobnego roadmapu.

## 8. Czego nie robić

- Nie usuwać całego Legacy przed wydzieleniem Walk Assist i kalibracji Halla.
- Nie włączać optymalizacji release przed audytem ISR.
- Nie zwiększać tablic w `MotorParams_t` bez migracji istniejącego rekordu.
- Nie zmieniać hurtowo nazw pól protokołu i EEPROM.
- Nie mieszać formatowania z refaktorem funkcjonalnym.
- Nie formatować vendor SDK razem z kodem projektu.
- Nie kasować dokumentów historycznych bez wskazania następcy lub archiwum.
- Nie dodawać wszystkich nieśledzonych plików jednym `git add`.
- Nie używać resetu/checkoutu do czyszczenia istniejącego worktree.
- Nie usuwać informacji o pochodzeniu projektu i autorach.
- Nie publikować binariów o niewyjaśnionym pochodzeniu.

## 9. Reguły odhaczania i pracy po przerwie

Checkbox można zaznaczyć dopiero, gdy istnieje dowód odpowiedni do zadania:

- zmiana kodu — diff + build + właściwy test;
- refaktor bez zmiany zachowania — test przed/po i porównanie;
- zmiana pamięci — mapa, rozmiar, test migracji i test zaniku zapisu;
- zmiana bezpieczeństwa — test graniczny oraz test na stojaku;
- dokumentacja — działające linki i zgodność z kodem;
- publikacja — weryfikacja z czystego klona.

Po każdej sesji należy zaktualizować:

1. checkboxy w tym planie;
2. checkboxy aktywnego podplanu;
3. bieżącą fazę;
4. ostatni zakończony krok;
5. dokładny następny krok;
6. dziennik prac;
7. znane ryzyka i niezakończone pliki.

## 10. Minimalny raport po każdej fazie

- commit/branch i `git status --short`;
- lista plików objętych fazą;
- build normalny i diagnostyczny;
- rozmiar Flash/RAM;
- wynik testów;
- lista ostrzeżeń;
- wymagane testy sprzętowe;
- wynik migracji, jeżeli dotyczy;
- pierwszy następny checkbox.

## 11. Dziennik prac

| Data | Faza | Co zakończono | Dowód | Znane ryzyko/blokada | Następny krok |
|---|---|---|---|---|---|
| 2026-07-30 | Audyt / plan | Zakończono audyt obecnego drzewa; utworzono nadrzędny plan oraz szczegółowy podplan Flash/storage | HEAD `992c910`; testy hostowe 5/5 podczas audytu | Duży zmodyfikowany worktree; brak wskazanego znanego dobrego builda sprzętowego | AUD-001…AUD-006 oraz uzupełnienie `NEW-xxx` w podplanie Flash |
| 2026-07-30 | Faza 0 | Ustalono target M820/BL820, utworzono `cleanup/publication`, sklasyfikowano zmiany i zapisano artefakty bazowe | `PROJECT_CLEANUP_BASELINE_INVENTORY_PL.md`; lokalne SHA-256 buildów 0.0199/0.0206/0.0252/0.0256/0.0257 | `0.0256` nie ma jeszcze testu potwierdzającego; brak eksportu ustawień | Wykonać AUD-002, AUD-005 i AUD-006; następnie checkpoint |
| 2026-07-30 | Faza 0 | Zbudowano normalnego kandydata `0.0258` z globalnie wyłączoną diagnostyką CAN i ponownie uruchomiono pełny zestaw testów | BIN 88 572 B; SHA-256 `13CD342CE86B5086A6A963DB51268A227189694B7EBCEE717B7E106CDF189FD4`; brak symboli diagnostycznych w ELF; testy 5/5 | Kandydat nie ma jeszcze testu na rowerze ani eksportu ustawień; istnieją ostrzeżenia kompilatora/linkera do Fazy 1/8 | Wgrać `0.0258`, wykonać jazdę + Walk Assist i zapisać eksporty; uzupełnić własne pomysły `NEW-xxx` |
| 2026-07-30 | Faza 1 + Flash Faza 2 | Dodano śledzony build M820/BL820, jawny manifest 48 źródeł, toolchain 13.2.1, wersję z Git/parametru, normalny/diagnostyczny debug, raporty oraz bezpieczny linker 230 KiB + Config A/B + SOC | Nowy normalny debug `0.0258` jest bajtowo identyczny: SHA-256 `13CD342C…9FD4`; diagnostyczny debug 93 112 B; Flash kończy się `0x0801A9D8` przed limitem `0x0803E800`; brak `RWE`; testy 5/5 | Release `-Os` ujawnił krytyczny brak kontraktów ISR/`volatile` i pozostaje zablokowany; brak testu sprzętowego i checkpointu | Użytkownik testuje `0.0258`; po wyniku zapisać eksporty i wykonać AUD-111/FS-107 |
| 2026-07-30 | Test sprzętowy 0.0258 | WA z uniesionym kołem i celem 40–50 RPM rozpędził koło do ok. 15 km/h, następnie zadziałało odcięcie | Zachowanie wskazuje na bankowy próg `Walk assist cut-off` w pobliżu 15,0 km/h; kod domyślny to 7,0 km/h, histereza 0,5 km/h | Wymagane maksimum właściciela to ok. 3 km/h; `0.0258` nie jest jeszcze bazą potwierdzoną | Ustawić 3,0 km/h w obu bankach, Save + Read/Sync, ponowić test na stojaku |
| 2026-07-30 | Wynik końcowy testu 0.0258 | Zwykłe wspomaganie i pozostałe funkcje działają jak w poprzednich firmware; WA nie trzyma stałych obrotów i rozpędza się coraz bardziej, więc po kilku sekundach operator musi wyłączyć tryb | Powtarzalna obserwacja na uniesionym kole; próba zmiany 50 na 40 RPM nie utrzymała się po ponownym `Read` w Canable | Błąd WA jest krytyczny; nie wykonywać testu na ziemi. Problem zapisu Canable jest osobną blokadą konfiguracji | Zarejestrować `target/measured ERPS`, błąd PI, Iq i stan Halla w buildzie diagnostycznym; na tej podstawie poprawić regulator/pomiar, dodać regresję i ponowić tylko test stojakowy |
| 2026-07-30 | Diagnoza WA 0.0258 | Znaleziono bezpośrednią przyczynę narastania obrotów: `regulate_iq_floor()` wymusza `5 Iq` dla każdej prędkości od 50% celu wzwyż, również ponad celem; `max(PI, start_floor, iq_floor)` nie pozwala zejść do zera | `src/walk_assist_motor.c`: `WA_MOTOR_HOLD_IQ=5` i warunek `measured_erps >= clear_erps`; test hostowy `overspeed` jawnie oczekuje końcowych `5 Iq`, a modele nie obejmują małego obciążenia | PASS hostowy jest fałszywym potwierdzeniem błędnego wymagania; 5 Iq to ok. 0,475 A dodatniego prądu fazowego, wystarczające do dalszego rozpędzania koła na stojaku | Bez zmiany kodu na etapie diagnozy. Przyszła poprawka ma dopuścić `0 Iq` ponad celem, zachować łagodny coast/reacquire i dodać regresję niemal bez obciążenia |
| 2026-07-30 | FW-062 / build 0.0259 | Podłoga `regulate_iq_floor()` zwraca `0 Iq` przy `measured_erps >= target_erps`; poniżej celu zachowano ochronę `5→60 Iq`. Test overspeed oczekuje zera, dodano model koła bez oporu | Testy hostowe 5/5 PASS. Normalny `0.0259`: 88 588 B, SHA-256 `27CAF8B7…FC7D`, CAN diag OFF. Diagnostyczny: 93 128 B, SHA-256 `E7E2C587…15AF`, CAN diag ON. Oba bez RWE | Brak potwierdzenia sprzętowego; Canable nadal nie utrzymuje zmienionej wartości bankowej po zapisie/Read | Wgrać wyłącznie normalny `0.0259`, najniższy bieg, koło w powietrzu; sprawdzić zatrzymanie narastania. Bez testu na ziemi do zaliczenia stojaka i bezpiecznego progu |
| 2026-07-30 | Test 0.0259 + FW-063 / build 0.0260 | `0.0259` ograniczył rozpędzanie, ale szarpał przy prędkości znacznie poniżej progu koła. Usunięto przełączanie `5/0 Iq`: PI steruje 50–100% celu, anti-stall rośnie `0→60 Iq` tylko między 50% i 20% | Testy 5/5 PASS; regresja ciągłości maks. 3 Iq/ERPS i lekki model bez cyklu. Normalny `0.0260`: 88 588 B, SHA-256 `56BD59ED…7304`, diag OFF. Diagnostyczny: 93 128 B, SHA-256 `F5809890…17AB`, diag ON. Oba bez RWE | `0.0260` nie ma jeszcze testu sprzętowego; zapis banków Canable nadal osobny | Wgrać normalny `0.0260`, koło w powietrzu, najniższy bieg; potwierdzić jednocześnie brak rozpędzania i szarpania |
| 2026-07-30 | Test 0.0260 + FW-064 / build 0.0261 | `0.0260` zbyt mocno doganiał cel, przestrzeliwał go, wykonywał kilka cykli coast/reacquire i przestawał działać do puszczenia przycisku. Bez logu `STALL` pozostaje wnioskiem z zachowania. Zachowano jednorazowy start `80 Iq`; Kp `2→1`, clamp `18→12 ERPS`, handover `60→36 Iq`, seed `12→8 Iq`, normal rise `93,75→46,875 Iq/s`, fall `156,25→250 Iq/s`, anti-stall `60→48 Iq`, reacquire `36→24 Iq` | Regresja powtarza 4 cykle i pilnuje braku ponownego START, całkowania bez Halla oraz doganiania ponad 24 Iq. Próg twardej utraty Halla `30 Iq` jest wyższy od reacquire. Testy 5/5 PASS. Normalny `0.0261`: 88 588 B, SHA-256 `1DE53EA1…50EE`, diag OFF. Diagnostyczny: 93 128 B, SHA-256 `C1006CFF…D39D`, diag ON. Oba bez RWE | `0.0261` nie ma jeszcze testu sprzętowego; realny napęd z wolnobiegiem nie jest w pełni odwzorowany przez model | Wgrać tylko normalny `0.0261`, koło w powietrzu, najniższy bieg; sprawdzić łagodne dojście, brak cyklu i brak wyłączenia przy stale trzymanym WA |
| 2026-07-30 | FW-065 / build 0.0262 | Przed wgraniem `0.0261` właściciel wymagał znacznie dłuższych ramp, ponieważ szybkie dochodzenie do nawet niższego Iq nadal daje nagły moment. START `250→62,5 Iq/s`, anti-stall `93,75→31,25 Iq/s`, normalne PI/reacquire `46,875→15,625 Iq/s`; zejście pozostaje `250 Iq/s`. Pełne `80 Iq` po ok. 1,27 s | Test wymaga ≤30 Iq po 0,45 s, pełnego startu dopiero ok. 1,25–1,30 s, ≤10 Iq po 0,6 s reacquire i ≤32 Iq po 1 s ciężkiego skoku obciążenia. Testy 5/5 PASS. Normalny `0.0262`: 88 588 B, SHA-256 `2A4ABA23…D5477`, diag OFF. Diagnostyczny: 93 128 B, SHA-256 `E403AB0E…70E4B`, diag ON. Oba bez RWE | Wolna rampa może dopuścić zatrzymanie przy nagłej ciężkiej blokadzie; to świadomy wybór bezpieczeństwa, watchdog ma wtedy ograniczyć/wyłączyć WA zamiast wymuszać skok momentu. `0.0261` nie był testowany sprzętowo | Wgrać tylko normalny `0.0262`, koło w powietrzu, najniższy bieg; ocenić czas i płynność narastania bez gwałtownego ręcznego blokowania napędu |
| 2026-07-31 | FW-066 / build 0.0263 | Przed testem `0.0262` doprecyzowano model działania: energiczny jednorazowy START, następnie wolny RUN w stałym zakresie `Iq_min..Iq_max`, bez zerowania przy miękkim celu; zero dopiero przy 80–90 rpm. Wdrożono START `93,75 Iq/s` do `80 Iq`, RUN `5..36 Iq`, wzrost `15,625 Iq/s`, spadek `31,25 Iq/s`, COAST `85/70 rpm`; usunięto anti-stall | Regresja obejmuje START, miękki RUN, pełny model ramp, regulator 85/70, brak Halla przez 1 s, `REACQUIRE <=24 Iq`, lekkie i obciążone koło. Testy 5/5 PASS. Normalny: 88 772 B, SHA-256 `AB4CC378…99447`, diag OFF. Diagnostyczny: 93 312 B, SHA-256 `BDCEC56A…37437`, diag ON. Oba bez RWE | Dodatnie minimum `5 Iq` może na bardzo lekkim kole prowadzić do twardego regulatora 85 rpm; jest to świadomy skutek wymagania i musi zostać oceniony na sprzęcie. `0.0262` nie był testowany sprzętowo | Wgrać tylko normalny `0.0263`, koło w powietrzu, najniższy bieg; potwierdzić płynny START, RUN bez szarpania oraz COAST 85/70 bez ponownego START |
| 2026-07-31 | FW-067 / build 0.0264 | Przed testem `0.0263` właściciel wymagał, aby próg zera nadążał za bankowym celem. Stałe 85/70 rpm zastąpiono `target+20/+5 rpm`, zachowując 15 rpm histerezy. Dla celu 20–60 rpm progi obejmują 40/25…80/65 rpm; nie zmieniono START, RUN ani safety | Test obejmuje cele 20, 40, 50 i 60 rpm, kontrolę zależności progów oraz wcześniejsze modele napędu. Testy 5/5 PASS. Normalny: 88 844 B, SHA-256 `438CC4E6…5CC7`, diag OFF. Diagnostyczny: 93 384 B, SHA-256 `584A0BE4…66DB`, diag ON. Oba bez RWE | Przy celu 50 rpm COAST wystąpi już przy 70 rpm zamiast 85 rpm; to zamierzona konsekwencja nowej reguły. `0.0263` nie był testowany sprzętowo | Wgrać tylko normalny `0.0264`, koło w powietrzu, najniższy bieg; potwierdzić dla odczytanego celu progi `+20/+5 rpm` i brak ponownego START |
