# Plan rozbudowy Flash i przestrzeni konfiguracji

> **STATUS:** MAPA FLASH WDROŻONA — zamrożenie bazy i projekt formatu oczekują.
>
> To jest szczegółowy podplan dokumentu
> [PROJECT_CLEANUP_MASTER_PLAN_PL.md](PROJECT_CLEANUP_MASTER_PLAN_PL.md).
> Obejmuje wyłącznie powiększenie przestrzeni konfiguracji, poprawę podziału
> końca Flash, migrację ustawień i przygotowanie miejsca na następne funkcje.
> Kolejność całego audytu i porządkowania projektu jest utrzymywana w planie
> nadrzędnym. Po każdej sesji aktualizujemy oba dokumenty.

Aktualizacja: 2026-07-30.

## 1. Stan wznowienia prac

| Pole | Aktualna wartość |
|---|---|
| Bieżąca faza | **Faza 1 oczekuje na test sprzętowy; techniczna Faza 2 mapy jest wykonana** |
| Ostatni zakończony krok | Linker ograniczono do 230 KiB i wydzielono Config A/B/SOC; normalny `0.0258` pozostał bajtowo identyczny |
| Następna czynność | Potwierdzić `0.0258`, wyeksportować ustawienia i utworzyć checkpoint; dopisać pozostałe pomysły |
| Implementacja w kodzie | Mapa i kontrole builda wdrożone; format rekordu i migracja nierozpoczęte |
| Znany dobry build sprzętowy dla migracji | `0.0206` — potwierdzona jazda; `0.0258` — kandydat oczekujący na test |
| HEAD podczas utworzenia planu | `992c910` |
| Ważna uwaga | Worktree był zmodyfikowany; sam SHA nie opisuje kompletnego aktualnego firmware |

Po przerwie należy zacząć od tej tabeli, znaleźć pierwszy niezaznaczony checkbox
w bieżącej fazie i przeczytać ostatni wpis w dzienniku prac.

## 2. Cel i zakres

Cele:

1. Nie dopuścić, aby rosnący firmware wszedł w strony Flash z ustawieniami.
2. Zapewnić miejsce na kolejne parametry bez kolejnego doraźnego powiększania
   struktur.
3. Zastąpić zapis surowego `MotorParams_t` jawnym, wersjonowanym formatem.
4. Zachować ustawienia użytkownika przy aktualizacji starego firmware.
5. Uodpornić zapis na zanik zasilania przez dwie kopie konfiguracji A/B.
6. Powiększyć bloki banku i globalnego tuningu bez zerwania zgodności ze
   starszymi wersjami.
7. Zapewnić bezpieczne granice wieloramkowego transportu CAN.

Poza zakresem tego planu:

- ogólne dzielenie `main.c` i `CAN_Display.c`;
- usuwanie silnika Legacy;
- strojenie sposobu wspomagania;
- dodawanie nowych funkcji, dopóki ich miejsce i format nie zostaną określone;
- zmiana fizycznego mikrokontrolera.

## 3. Wynik audytu pamięci

Pierwszy pomiar wykonano na lokalnym ELF `.build/0.0257.elf` z diagnostyką.
Pomiar poniżej zaktualizowano dla normalnego ELF `.build/0.0258.elf`, w którym
diagnostyka CAN jest wyłączona. Po teście sprzętowym trzeba przypisać wynik do
checkpointu Git.

| Obszar | Aktualne użycie | Bezpieczna pojemność | Zapas |
|---|---:|---:|---:|
| Obraz aplikacji Flash | 88 536 B | 237 568 B (232 KiB), do `0x0803F000` | 149 032 B |
| RAM: `.data + .bss` | 6 608 B | 49 152 B (48 KiB) | 42 544 B przed rezerwą stosu |
| Rekord parametrów na stronie 126 | ok. 600 B | 2 048 B | ok. 1 448 B |
| Blob jednego banku | 190 B | 192 B | 2 B |
| Blob globalnego tuningu | 24 B | 24 B | 0 B |

Wniosek: nie kończy się fizyczna pamięć Flash ani RAM. Kończy się miejsce w
dwóch sztywnych formatach konfiguracji: banku profilu i globalnym tuningu.

### 3.1 Obecny układ końca Flash

```text
0x08005000  początek aplikacji M820/BL820
     ...
0x0803F000  strona 126: surowy MotorParams_t + Halle + stopka
0x0803F800  strona 127: wear-leveling SOC
0x08040000  koniec Flash
```

Obecny linker deklaruje dla aplikacji `238K` od adresu `0x08005000`, czyli
pozwala linkować aż do `0x08040800`. To nachodzi na obie strony danych i wychodzi
poza koniec Flash. Aktualny obraz jest znacznie mniejszy, ale brakuje ochrony na
przyszłość.

### 3.2 Proponowany układ docelowy

```text
0x08005000  aplikacja
     ...
0x0803E800  Config A, strona 125, 2 KiB
0x0803F000  Config B, strona 126, 2 KiB
0x0803F800  SOC,      strona 127, 2 KiB
0x08040000  koniec Flash
```

Bezpieczna pojemność aplikacji po tej zmianie wyniesie `230 KiB`
(`0x0803E800 - 0x08005000`). Przy obrazie około 92 KB nadal pozostanie około
140 KB zapasu.

## 4. Decyzje projektowe

| ID | Decyzja | Status |
|---|---|---|
| D-001 | Nie próbujemy „powiększać” fizycznego Flash/RAM programowo; poprawiamy partycję i format danych | PRZYJĘTA |
| D-002 | Rezerwujemy stronę 125 na drugą kopię konfiguracji; SOC pozostaje na stronie 127 | PRZYJĘTA I ZABEZPIECZONA W LINKERZE |
| D-003 | Nowy rekord ma jawny serializer; nie zapisuje surowej struktury C ani paddingu | PRZYJĘTA |
| D-004 | Aktualizacja ze starego rekordu ma migrować ustawienia bez fabrycznego resetu | PRZYJĘTA |
| D-005 | Zapis A/B jest transakcyjny: najpierw nowa kopia, weryfikacja, dopiero potem stara może być skasowana | PRZYJĘTA |
| D-006 | Zapis konfiguracji jest dozwolony wyłącznie przy bezpiecznie zatrzymanym napędzie | PRZYJĘTA |
| D-007 | Pojemność banku docelowo 240 B, tuningu 64 B, ale długość użyta na CAN pozostaje wersjonowana | WSTĘPNA — POTWIERDZIĆ PO SPISIE ZMIENNYCH |
| D-008 | Jednobajtowa długość transportu CAN ogranicza pojedynczy blob do maks. 255 B | OGRANICZENIE PROTOKOŁU |
| D-009 | Każda faza ma osobny commit; nie łączymy migracji pamięci z formatowaniem lub zmianą jazdy | PRZYJĘTA |

Jeżeli decyzja zostanie zmieniona, nie usuwamy starego wpisu. Zmieniamy status,
dopisujemy nową decyzję i uzasadnienie w dzienniku prac.

## 5. Spis przyszłych zmiennych

Ta tabela musi zostać uzupełniona przed zatwierdzeniem rozmiarów. Jedna funkcja
może potrzebować kilku pól.

| ID | Nazwa/idea | Typ, zakres i jednostka | Zakres: globalna/per-bank/per-level | Trwała po restarcie? | CAN: zapis/odczyt/telemetria | Wartość domyślna i migracja | Status |
|---|---|---|---|---|---|---|---|
| NEW-001 | Opcjonalny eMTB curve blend z odroczonej części FW-033 | DO SPECYFIKACJI | Prawdopodobnie per-level | Tak | zapis + odczyt + telemetria użytego blendu | Domyślnie wyłączony; starsze banki zachowują obecne eMTB | ZNANY BACKLOG — WARUNKOWY |
| NEW-002 | Torque peak guard z odroczonej części FW-033 | DO SPECYFIKACJI | Do decyzji: globalna albo per-bank | Tak, jeżeli użytkownik ma stroić | zapis + odczyt + licznik zadziałań w telemetrii | Domyślnie wyłączony | ZNANY BACKLOG — WARUNKOWY |
| NEW-003 | Filtr torque zależny od kadencji | DO SPECYFIKACJI | Do decyzji: globalna albo per-bank | Tak | zapis + odczyt; telemetria aktywnej stałej czasu | Starsze wersje zachowują obecny filtr | ZNANY BACKLOG — WARUNKOWY |
| NEW-004 | Parametry gear preload/cichego kasowania luzu | enable + czas + Iq; zakresy po teście | Do decyzji: globalna albo per-bank | Tak | zapis + odczyt + stan preload w diagnostyce | Domyślnie wyłączone; wdrażać tylko jeśli klik nadal występuje | ZNANY BACKLOG — WARUNKOWY |
| NEW-005 | Rozszerzone progi odrzucania fałszywych impulsów speed | DO SPECYFIKACJI | Globalna | Tak | zapis + odczyt + liczniki odrzuceń | Zachować obecny minimalny filtr jako default | ZNANY BACKLOG |
| NEW-006 | Parametry field weakening | DO SPECYFIKACJI; bezpieczeństwo krytyczne | Globalna/service | Tak tylko po osobnym audycie | service write/read + diagnostyka nasycenia | Domyślnie wyłączone; nie wdrażać bez dowodu `u_abs` saturation | ODROCZONE |
| NEW-100 | Nowy pomysł właściciela nr 1 | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | OCZEKUJE NA OPIS |
| NEW-101 | Nowy pomysł właściciela nr 2 | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | OCZEKUJE NA OPIS |
| NEW-102 | Nowy pomysł właściciela nr 3 | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | DO UZUPEŁNIENIA | OCZEKUJE NA OPIS |

Zasady klasyfikacji:

- **robocza** — tylko RAM, nie zajmuje rekordu konfiguracji;
- **diagnostyczna** — zwykle RAM + telemetria, bez zapisu do Flash;
- **globalna trwała** — rekord konfiguracji/tuningu;
- **per-bank** — format banku i dwie kopie w konfiguracji;
- **per-level** — koszt pola należy pomnożyć przez liczbę poziomów i banków;
- każda zmienna musi mieć jednostkę, zakres, wartość domyślną i zachowanie przy
  odczycie starszej wersji.

## 6. Plan realizacji

### Faza 0 — zatwierdzenie zakresu

- [ ] **FS-000** Uzupełnić tabelę przyszłych zmiennych.
- [ ] **FS-001** Określić, które zmienne są tylko diagnostyczne i nie wymagają
      miejsca w Flash.
- [ ] **FS-002** Obliczyć dokładny koszt nowych pól dla banku, tuningu i rekordu.
- [ ] **FS-003** Potwierdzić albo zmienić pojemności: bank 240 B, tuning 64 B.
- [x] **FS-004** Potwierdzić układ Config A/Config B/SOC.
- [x] **FS-005** Ustalić wspierane cele: tylko M820/BL820 czy również stare
      bootloadery 0/3/38.
- [ ] **FS-006** Zaktualizować tabelę „Stan wznowienia prac” na Fazę 1.

Warunek zakończenia: wszystkie przyszłe pola są sklasyfikowane, rozmiary
policzone, a układ stron zatwierdzony. Bez tego nie zmieniamy struktur.

### Faza 1 — zamrożenie bezpiecznej bazy

- [ ] **FS-100** Wybrać firmware potwierdzony testem sprzętowym.
- [ ] **FS-101** Zapisać commit, status worktree, nazwę binarki i SHA-256.
- [ ] **FS-102** Wyeksportować bieżące ustawienia obu banków i tuningu.
- [ ] **FS-103** Odczytać i zachować diagnostykę Halla oraz stan rekordu.
- [x] **FS-104** Wykonać build normalny i diagnostyczny.
- [x] **FS-105** Uruchomić wszystkie testy hostowe.
- [x] **FS-106** Zapisać rozmiary Flash/RAM i adres końca obrazu.
- [ ] **FS-107** Utworzyć osobny commit/tag punktu bazowego.
- [ ] **FS-108** Zaktualizować stan planu na Fazę 2.

Warunek zakończenia: istnieje odtwarzalny punkt powrotu oraz kopia ustawień
potrzebna do sprawdzenia migracji.

### Faza 2 — bezpieczna granica aplikacji w linkerze

- [x] **FS-200** Zmienić koniec obszaru aplikacji na początek najniższej
      zarezerwowanej strony danych.
- [x] **FS-201** Usunąć stałe `238K` współdzielone przez różne adresy startowe.
- [x] **FS-202** Dodać symbole adresów końca aplikacji i stron danych.
- [x] **FS-203** Dodać link-time `ASSERT`, że sekcje ładowane do Flash kończą się
      przed Config A.
- [x] **FS-204** Sprawdzić, że `.data` ma prawidłowy LMA w Flash.
- [x] **FS-205** Usunąć ostrzeżenie segmentu ELF `RWE` albo udokumentować i
      ograniczyć prawidłowe flagi segmentów.
- [x] **FS-206** Zbudować wszystkie cele pozostawione jako wspierane w FS-005.
- [x] **FS-207** Porównać binarkę i mapę; brak zmian zachowania runtime.
- [ ] **FS-208** Osobny commit wyłącznie dla mapy pamięci.

Warunek zakończenia: zbyt duży firmware kończy się błędem linkera, a nie
nadpisaniem konfiguracji.

### Faza 3 — jawny format rekordu konfiguracji

- [ ] **FS-300** Zaprojektować nagłówek rekordu: magic, wersja, długość,
      sekwencja, flagi i CRC.
- [ ] **FS-301** Zdefiniować jawne kodowanie little-endian bez zależności od
      `sizeof`, wyrównania i paddingu struktur C.
- [ ] **FS-302** Rozdzielić pola runtime od pól trwałych.
- [ ] **FS-303** Zdefiniować sekcje/pola dla Halla, parametrów podstawowych,
      banków i globalnego tuningu.
- [ ] **FS-304** Dodać statyczne limity rozmiaru i kontrolę przepełnienia
      serializera.
- [ ] **FS-305** Dodać parser starego rekordu v1 znajdującego się na stronie 126.
- [ ] **FS-306** Dodać migrację v1 → nowy format bez resetu ustawień.
- [ ] **FS-307** Dodać test round-trip nowego rekordu.
- [ ] **FS-308** Dodać testy: zła wersja, zła długość, CRC, ucięty rekord,
      nieznane pola i wartości spoza zakresu.
- [ ] **FS-309** Osobny commit kodeka i testów, jeszcze bez zapisu do fizycznego
      Flash.

Warunek zakończenia: nowy format można testować na komputerze, a zapisane bajty
nie zależą od kompilatora.

### Faza 4 — dwie kopie Config A/B i bezpieczna migracja

- [ ] **FS-400** Wydzielić sterownik magazynu konfiguracji z `main.c`.
- [ ] **FS-401** Przy starcie wybrać poprawny rekord o najwyższej sekwencji.
- [ ] **FS-402** Zapisywać nowy rekord na nieaktywnej stronie.
- [ ] **FS-403** Zapisywać znacznik zatwierdzenia/CRC jako ostatni element.
- [ ] **FS-404** Po zapisie ponownie odczytać i zweryfikować cały rekord.
- [ ] **FS-405** Nie kasować jedynej poprawnej kopii przed potwierdzeniem nowej.
- [ ] **FS-406** Pierwszą migrację wykonać: odczyt starej strony 126 → zapis
      nowego rekordu na stronie 125 → weryfikacja → dopiero potem strona 126
      może wejść do rotacji A/B.
- [ ] **FS-407** Zablokować zapis podczas jazdy, PWM, kalibracji i aktywnego WA.
- [ ] **FS-408** Dodać stan diagnostyczny: aktywna strona, wersja, sekwencja,
      wynik ostatniego zapisu/migracji.
- [ ] **FS-409** Przetestować symulowane przerwanie zapisu w każdym etapie.
- [ ] **FS-410** Osobny commit warstwy Flash i migracji.

Warunek zakończenia: zanik zasilania w dowolnym momencie zostawia co najmniej
jedną poprawną konfigurację albo bezpieczne wartości domyślne.

### Faza 5 — utwardzenie wieloramkowego CAN

- [ ] **FS-500** Zmienić `append_multiframe()` tak, aby dostawał pojemność
      bufora i zwracał wynik.
- [ ] **FS-501** Walidować długość START względem konkretnej komendy.
- [ ] **FS-502** Walidować DLC każdej ramki.
- [ ] **FS-503** Walidować indeks, kolejność, źródło i komendę aktywnej sesji.
- [ ] **FS-504** Odrzucać ramkę, której `offset + DLC > capacity`.
- [ ] **FS-505** Zastąpić stałe `command < 23` obliczeniem z pojemności i
      oczekiwanej długości.
- [ ] **FS-506** Dodać poprawne ACK/NACK z kodem przyczyny.
- [ ] **FS-507** Dodać timeout i reset niedokończonej sesji.
- [ ] **FS-508** Dodać testy błędnych i złośliwych sekwencji ramek.
- [ ] **FS-509** Osobny commit transportu CAN bez zmiany schematu parametrów.

Warunek zakończenia: żadna długość lub kolejność ramek nie może zapisać poza
buforem ani zastosować częściowej konfiguracji.

### Faza 6 — większe bufory i nowe wersje blobów

- [ ] **FS-600** Oddzielić pojęcie `CAPACITY` od aktualnej długości konkretnej
      wersji formatu.
- [ ] **FS-601** Zwiększyć pojemność banku do zatwierdzonej wartości.
- [ ] **FS-602** Zwiększyć pojemność globalnego tuningu do zatwierdzonej wartości.
- [ ] **FS-603** Zachować odczyt wszystkich obsługiwanych starszych wersji banku.
- [ ] **FS-604** Zachować odczyt starszych wersji tuningu.
- [ ] **FS-605** Nową wersję wysyłać wyłącznie do Canable, które rozpoznało
      możliwości firmware.
- [ ] **FS-606** Zaktualizować `ebics_config_schema.yaml`.
- [ ] **FS-607** Zaktualizować parser i testy Canable.
- [ ] **FS-608** Dodać test zgodności firmware ↔ Canable bez cichego pomijania,
      gdy wymagany artefakt integracyjny jest nieobecny.
- [ ] **FS-609** Osobny commit pojemności i wersji blobów.

Warunek zakończenia: stare ustawienia wczytują się poprawnie, nowy format ma
zadeklarowany zapas, a nieobsługiwane wersje są jawnie odrzucane.

### Faza 7 — dodawanie nowych zmiennych

Dla każdej pozycji `NEW-xxx` wykonujemy osobny podpunkt:

- [ ] **FS-700** Dodać pole do właściwego modelu, nie automatycznie do
      `MotorParams_t`.
- [ ] **FS-701** Dodać zakres, jednostkę, wartość domyślną i walidację.
- [ ] **FS-702** Dodać serializację i migrację.
- [ ] **FS-703** Dodać zapis/odczyt CAN tylko jeśli funkcja tego wymaga.
- [ ] **FS-704** Dodać test round-trip i test wartości granicznych.
- [ ] **FS-705** Dodać opis użytkownika/developera.
- [ ] **FS-706** Zmierzyć nowy rozmiar Flash, RAM i rekordu.
- [ ] **FS-707** Odhaczyć właściwe `NEW-xxx` jako WDROŻONE.

Warunek zakończenia: każde pole ma właściciela, format, test i dokumentację.

### Faza 8 — walidacja sprzętowa

- [ ] **FS-800** Aktualizacja ze starego firmware zachowuje ustawienia.
- [ ] **FS-801** Odczyt banków i tuningu po migracji jest bajtowo zgodny z
      eksportem bazowym.
- [ ] **FS-802** Kalibracja Halla pozostaje poprawna.
- [ ] **FS-803** SOC zachowuje się poprawnie i nie jest naruszany przez Config A/B.
- [ ] **FS-804** Zapis przy postoju działa, a próba zapisu podczas jazdy jest
      odroczona.
- [ ] **FS-805** Symulowany reset podczas zapisu wybiera poprzednią poprawną kopię.
- [ ] **FS-806** Uszkodzenie Config A wybiera Config B i odwrotnie.
- [ ] **FS-807** Obie uszkodzone kopie uruchamiają bezpieczne wartości domyślne
      i czytelną diagnostykę.
- [ ] **FS-808** Normalny i diagnostyczny firmware przechodzą test jazdy.
- [ ] **FS-809** Zapisać wersję, commit, SHA-256 binarki i wynik testu.

Warunek zakończenia: migracja i awarie zapisu są sprawdzone na sprzęcie, nie
tylko w modelu hostowym.

### Faza 9 — zamknięcie i przygotowanie do dalszych porządków

- [ ] **FS-900** Zaktualizować dokumentację architektury pamięci.
- [ ] **FS-901** Zaktualizować instrukcję builda i mapę linkera.
- [ ] **FS-902** Zaktualizować dokumentację protokołu CAN.
- [ ] **FS-903** Usunąć lub oznaczyć jako archiwalne opisy starego surowego
      rekordu.
- [ ] **FS-904** Dodać kontrolę rozmiaru obrazu i rekordów do CI.
- [ ] **FS-905** Potwierdzić brak nowych ostrzeżeń kompilatora/linkera.
- [ ] **FS-906** Utworzyć końcowy tag/punkt odniesienia.
- [ ] **FS-907** Oznaczyć plan jako ZAKOŃCZONY i wskazać następny plan
      porządkowania projektu.

## 7. Reguły pracy po przerwie

Każda osoba lub agent wracający do zadania ma:

1. Przeczytać „Stan wznowienia prac” i ostatni wpis dziennika.
2. Sprawdzić `git status`; nie zakładać, że wszystkie lokalne zmiany należą do
   tej fazy.
3. Realizować tylko jeden etap lub jeden bezpieczny podpunkt naraz.
4. Nie łączyć zmian formatu danych ze zmianą działania silnika.
5. Nie zaznaczać checkboxa bez dowodu: diff, test, build lub test sprzętowy.
6. Po pracy zaktualizować:
   - checkboxy;
   - bieżącą fazę;
   - ostatni zakończony krok;
   - dokładny następny krok;
   - dziennik prac.
7. Jeżeli etap nie jest zakończony, zapisać również znane ryzyko i pliki, które
   są w trakcie zmian.

## 8. Minimalny zestaw dowodów po każdej fazie

- `git status --short`;
- commit lub opis niezacommitowanego zakresu;
- wynik builda normalnego;
- wynik builda diagnostycznego;
- rozmiar ELF/BIN;
- wynik testów hostowych;
- ostrzeżenia kompilatora i linkera;
- przy fazach 4 i 8: wynik testu sprzętowego i odczyt diagnostyki pamięci;
- jednoznaczny następny checkbox.

## 9. Dziennik prac

| Data | Faza | Co zakończono | Commit/build/test | Znane ryzyko lub blokada | Następny krok |
|---|---|---|---|---|---|
| 2026-07-30 | Audyt / utworzenie planu | Zmierzono Flash, RAM, rekord parametrów i bloby; zapisano plan etapowy | HEAD `992c910`; bez zmian firmware w ramach tego planu | Worktree zawiera wcześniejsze zmiany; brak wskazanego znanego dobrego builda migracyjnego | Uzupełnić tabelę `NEW-xxx` i wykonać Fazę 0 |
| 2026-07-30 | Faza 0 | Zapisano znane przyszłe parametry; zbudowano normalny `0.0258` bez diagnostyki CAN | Flash 88 536 B; RAM `.data + .bss` 6 608 B; testy hostowe 5/5 | Brak potwierdzenia sprzętowego i pełnej listy nowych pomysłów właściciela | Test `0.0258`, eksport ustawień i dopisanie pozostałych `NEW-xxx` |
| 2026-07-30 | Faza 1/2 | Zbudowano normalny i diagnostyczny debug; wdrożono granicę aplikacji, regiony Config A/B/SOC, symbole, `ASSERT` i rozdzielone segmenty ELF | Normalny BL820 88 572 B i SHA-256 identyczny z bazą; image end `0x0801A9D8`; limit `0x0803E800`; RAM 6 600 B z heap/stack; testy 5/5 | FS-100/102/103/107 wymagają roweru i checkpointu; release zablokowany do audytu ISR | Test sprzętowy `0.0258`, eksporty, checkpoint; potem projekt rekordu |
