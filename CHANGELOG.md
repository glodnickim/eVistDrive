# Changelog

## [Unreleased] — branch cleanup/publication

### Nowe funkcje

> **Buildy FW-068…FW-073 + CB-019.** Normalny `0.0267_M820_BL820.bin`: 90 048 B, SHA-256
> `660B77B8A768D441CCBBBFAA815ACC1AE45E6CBE4D11EC65F212B07C0AE7D470`, CAN diagnostics OFF.
> Diagnostyczny `0.0268_M820_BL820.bin`: 94 576 B, SHA-256
> `93111A2D369A4DAF8E80C94171037C583C457D14ECF5A2DC4EFD3FFF28F96345`, CAN diagnostics ON.
> Canable: `dist/openbafang-cannable.exe` z tego samego drzewa.
>
> Sekcja `text` zmalała z 89 984 B (0.0265) do 89 752 B — **232 bajty mniej**, czyli usunięty
> kod końcówki wygaszania (FW-072). `_Static_assert` na wartościach trybu i silnika przeszły,
> co potwierdza, że przemianowania z FW-073 nie ruszyły niczego na drucie. W żadnym artefakcie
> (`.bin`, `.elf`) nie ma już starej nazwy — wcześniej `.elf` miał jej 6 wystąpień.
>
> Wariant diagnostyczny **nie** nazywa się `0.0267-diag`: `build_firmware.ps1` numeruje
> z trwałego licznika `.build/version.txt` i inkrementuje go przy każdym uruchomieniu.
> Numer wpisany w binarce (`EBICS_BUILD_VERSION`) zgadza się z nazwą pliku.
>
> Buildy `0.0265`/`0.0266` (FW-068/069) zostały zastąpione i **nie należy ich wgrywać** —
> nie zawierają FW-072/073 ani zabezpieczeń wartości protokołu.

> **NUMER WERSJI: minimum `0.0265`.** Obecnych źródeł **nie wolno budować jako `0.0264`** —
> zawierają FW-068/FW-069, zmieniony format obu bloków konfiguracji (banki v6, tuning v6),
> zmienioną odpowiedź na zapis konfiguracji oraz jednorazową migrację ustawień. Build z tym
> samym numerem co poprzedni uniemożliwiłby powiązanie zachowania na rowerze z zawartością
> firmware. Numer jest parametrem `-ArtifactName` skryptu `build_firmware.ps1`.

> **UWAGA — jednorazowy reset ustawień.** FW-068 i FW-069 powiększają `MotorParams_t`, więc
> zapisany rekord EEPROM przestaje pasować rozmiarem i kontrola z FW-023 go odrzuca. Po
> pierwszym uruchomieniu **wszystkie zapisane ustawienia wracają do fabrycznych**: banki
> profili, blok tuningu, kalibracja span czujnika nacisku i napięcie 100% SOC. Nie jest to
> uszkodzenie — nigdy nie dochodzi do odczytania śmieci, firmware kładzie świeży rekord
> z wartości domyślnych. Kąty Halla wracają do wkompilowanych `HALL_DEF_*`; jeśli silnik
> brzęczy zamiast płynnie ruszać, należy powtórzyć kalibrację pozycji (0x6200).

**CB-019 — czytelne dymki parametrów (wartość fabryczna, zakres, kierunek zmiany)**
- **Tylko Canable. Firmware, protokół i format banków bez zmian.**
- Mechanika była w większości gotowa i została zachowana: `fieldInput()` sam dopisuje „Factory default" i „Allowed range", w Profiles wartość fabryczna pochodzi z `PROFILE_LEVEL_PLACEHOLDER_BANKS` **dla wybranego banku i poziomu** (etykieta „Factory default for Bank 1 / SPORT+", więc nie da się jej pomylić z odczytem z kontrolera), w Dynamics z `TUNING_DEFAULTS`, checkbox czyta się On/Off, a wartości natywne przechodzą przez `fromNative` (mV → kg, gamma ×10 → dziesiętna).
- **Naprawiony dotyk.** Dymek pokazywał się tylko na `:hover` i `:focus` — to pokrywa mysz i klawiaturę, ale nie dotyk: tapnięcie `<span tabindex="0">` nie daje pewnego focusu na urządzeniach mobilnych i nie było czym dymka zamknąć. Plakietka to teraz `<button type="button">` z `aria-label`/`aria-expanded`: tap przełącza, otwarcie kolejnej zamyka poprzednią, klik obok lub Escape zamyka. `type="button"` jest konieczny, bo plakietka bywa w formularzach starych zakładek; klik robi `preventDefault()` + `stopPropagation()`, bo siedzi w `<label>`.
- Uzupełniony **kierunek zmiany** w siedmiu opisach, którym go brakowało (obie wolne rampy, czas wyłączenia, oba filtry mocy, próg przyrostu nacisku, napięcie odniesienia eMTB). Presety Aggressive / Normal / Smooth przy 11 parametrach dynamiki — wyłącznie podpowiedź w tekście, bez wpływu na wartości fabryczne i dane z roweru.
- Nowy test `tests/cb019_help_bubble_content.js`: sprawdza, że pole natywne pokazuje oba człony w kg i że surowa wartość mV nie wycieka do dymka, że checkbox czyta się „Off" bez zakresu liczbowego, oraz że brak zadeklarowanej wartości fabrycznej nie produkuje zmyślonej linii.

**FW-073 — usunięcie śladów pochodzenia z nazw i opisów**
- Produkt był już czysty: **wgrywany `.bin` nie zawierał ani jednego wystąpienia**, etykiety w Canable też nie. Ślady siedziały w warstwie deweloperskiej — 6 symboli (widocznych w `.elf`), 66 miejsc w `src`/`inc` i 195 linii w 35 plikach dokumentacji.
- **Symbole przemianowane:** `ASSIST_MODE_EMTB_TSDZ` → `ASSIST_MODE_EMTB`, `RIDE_ENGINE_TSDZ` → `RIDE_ENGINE_CORE`, `EMTB_TSDZ_*` → `EMTB_*`. **Wartości na drucie bez zmian** — tryb dalej `3`, silnik dalej `1`.
- **Zabezpieczenie tych wartości dodane dopiero teraz.** Wcześniejszy zapis w tym changelogu twierdził, że pilnują ich testy — a to było za mocne: testy Canable sprawdzały tryby 1, 2 i 6, ale **nie 3**, a `RIDE_ENGINE_CORE` miało wartość 1 wyłącznie dlatego, że było drugim elementem enum. Dołożone realne zabezpieczenia: `RIDE_ENGINE_CORE = 1` zapisane jawnie, `_Static_assert` na **wszystkich siedmiu** wartościach trybu i na wartości silnika (kompilacja padnie, jeśli ktoś je ruszy), oraz test Canable przechodzący round trip dla trybów 1–6 z osobnym sprawdzeniem eMTB. Stored bank niesie **numer, nie nazwę** — przenumerowanie sprawiłoby, że bank zapisany jako eMTB wróciłby po aktualizacji jako inny tryb, po cichu i bez błędu.
- Komentarze w firmware i Canable przepisane wg zasady: **opisujemy co robi wartość i dlaczego tak, a nie skąd pochodzi**. Nazwa silnika wszędzie jako „ride core".
- Siedem plików dokumentacji przemianowanych (`FW-027`, `FW-028`, `FW-030`, `FW-033`, `FW-039` i dwa dokumenty analityczne), wszystkie odsyłacze poprawione.
- **Pięć dokumentów porównawczych przeniesionych do `archive/`** z własnym README. Ich celem było porównanie do nazwanego projektu zewnętrznego, więc podmiana nazw pozbawiłaby je sensu; cztery z nich były już oznaczone jako archiwalne i zastąpione (wnioski dawno w kartach FW i w `ebics_config_schema.yaml`). Treść zostaje bez zmian jako materiał historyczny.
- **Zostaje celowo:** oświadczenie o pochodzeniu w `README.md` (EBiCS i firmware'y referencyjne są na GPL — to wymóg licencyjny, nie branding), katalog `.external/` z własnymi plikami `LICENSE`, oraz nazwa gałęzi git `experiment/tsdz-experiment` wraz z dwoma odwołaniami do niej w dokumentach (przepisywanie historii git to osobna operacja o innym ryzyku).

**FW-072 — wygaszanie wspomagania to jedna rampa (wycofanie FW-047)**
- FW-047 dokładało wolną „końcówkę" na ostatnich 15% poziomu, od którego zaczynało się wygaszanie. **Testy nie potwierdziły, żeby usuwała klik przekładni**, a miała skutek uboczny niewidoczny w interfejsie: `release_ms` odmierzało zejście tylko do 15%, po czym szło **jeszcze 600 ms** (250 ms przy cięciu bezpieczeństwa) do rzeczywistego zera. Ustawione 650 ms zachowywało się jak ~1250 ms — pole w Canable mówiło jedno, firmware robiło drugie.
- Teraz: **prąd z chwili zatrzymania pedałowania → 0 przez dokładnie `release_ms`**, niezależnie od tego, jak wysoki był ten prąd. Krok rampy liczony raz przy starcie wygaszania, z zaokrągleniem w górę, żeby rampa faktycznie dochodziła do zera w zadanym czasie zamiast pełzać na ostatnim ułamku kroku.
- Usunięte: `RIDE_FADE_TAIL_PCT`, `RIDE_FADE_TAIL_MS`, `RIDE_FADE_TAIL_SAFETY_MS`, `profile_release_tail_q`, `profile_release_tail_step_q`. Dodatkowo `profile_release_start_q` — po tej zmianie zapisywana w siedmiu miejscach i nieczytana ani razu; rolę znacznika „wygaszanie trwa" pełni `profile_release_step_q != 0`.
- **Zachowane bez zmian:** FW-040 (czas liczony od rzeczywistego Iq, nie od pełnej skali), `immediate_cut`, zachowanie Walk Assist, wcześniejsze zakończenie przez `coast_release`, limit `PROFILE_RELEASE_MAX_MS`, krótszy czas dla hamulca/cofania/błędu (200 ms z `ride_control`), oraz `release_ms = 0` → rampy `iq_fall_*` z poziomu.
- Bez zmian w protokole CAN i formacie banków. Opis `release_ms` w Canable mówi teraz wprost, że to **całkowity** czas zejścia do zera i że nie ma po nim żadnej końcówki. Karta `FW-047_FADE_TAIL.md` oznaczona jako wycofana, z zachowaniem treści jako zapisu próby i jej wyniku.
- Overrun celowo **nie** został dodany — to osobne, następne zadanie: `PAS STOP → opcjonalny OVERRUN → pojedyncze RELEASE → OFF`.

**FW-071 — kopiowanie sekcji zamiast trybu „apply to all levels"**
- **Tylko Canable. Firmware, protokół i format konfiguracji bez zmian** — buildy `0.0265`/`0.0266` pozostają aktualne.
- Oba przełączniki z FW-069 usunięte. Ich wada była konstrukcyjna, nie kosmetyczna: **to był tryb, który można zostawić włączony i o nim zapomnieć** — zmiana jednego pola po cichu przepisywała cztery inne poziomy, bez potwierdzenia, bez informacji o skali i bez drogi powrotnej. Dwa przełączniki dawały do tego cztery kombinacje, z których dwie znaczyły to samo.
- Ustawienia wspólne podzielone na **sekcje** (limity mocy/prądu, warunek startu, launch feel, rampy, wygładzanie i release), każda z własnym przyciskiem **„⧉ Copy to…"**. Podział nie jest dekoracją: skoro kopiowanie działa na sekcjach, jedna wielka sekcja wymuszałaby kopiowanie wszystko-albo-nic i nie obsłużyłaby przypadku, o który chodzi — te same rampy wszędzie, inna siła na poziom.
- Panel kopiowania pozwala wybrać poziomy i opcjonalnie **drugi bank**, oraz pokazuje skalę **przed** wykonaniem („Overwrites 16 values in 4 level(s)"). Kopiowanie na oba banki jest tu bezpieczne właśnie dlatego, że jest jednorazową, świadomą decyzją, a nie ambientowym trybem. Po skopiowaniu przy nagłówku sekcji pojawia się **Undo** przywracające poprzednie wartości.
- Checkbox poziomu źródłowego zostaje aktywny (oznaczony „source"): jego zablokowanie uniemożliwiałoby skopiowanie SPORT+ z banku 1 na SPORT+ w banku 2. Pomijany jest wyłącznie dokładny slot źródłowy, a nie numer poziomu.
- Wykresy ramp dostały **kolor tła edytowanego poziomu**. Należą do poziomu, tak jak podgląd silnika, ale jako jedyne karty per poziom były białe — co czytało się jako „to jest globalne", czyli odwrotnie niż jest.
- Poprawione trzy teksty pomocy, które odsyłały do nieistniejącej już zakładki Dynamics i do ramp jako globalnych.

**FW-070 — zakładka Dynamics scalona z Profiles, z jawnym rozdzieleniem zasięgu**
- **Tylko Canable. Firmware, protokół i format konfiguracji bez zmian** — buildy `0.0265`/`0.0266` pozostają aktualne.
- Po FW-069 w karcie Dynamics zostały same ustawienia **globalne** (zanik startup boostu, kroki korby do startu, zatrzask jazdy, wygładzanie RUN); rampy przeniosły się per poziom do Profiles. Zostały dwie zakładki, między którymi trzeba było skakać przy jednym zadaniu — próg nacisku per poziom w jednej, kroki korby globalnie w drugiej — a **nic w interfejsie nie mówiło, że to dwa różne zasięgi**. Łatwo było uznać, że kroki korby też są per poziom.
- Zakładka **eVistDrive Dynamics usunięta** wraz z przyciskiem nawigacji. Jej zawartość jest teraz pasmem **„Global — whole bike"** na dole zakładki Profiles.
- Rozdzielenie zasięgu zrobione trzema środkami naraz, bo pojedynczy łatwo przeoczyć przy przewijaniu: osobna płyta z grubą krawędzią i innym tłem, tekst wprost („**not** bank or level settings… applies to **both banks and all five assist levels**… changing the bank or level selector above has no effect"), oraz **własne przyciski** „Read global tuning" / „Write global (RAM)" / „Restore global". Te ostatnie odzwierciedlają rzeczywistość protokołu: to inny blok CAN (0x6023/0x6024) niż banki (0x6020/0x6021).
- Poprawiona pułapka: `renderDynamicsCharts()` miało bramkę `tabIsVisible('tab-ebics-dynamics')`, która po usunięciu zakładki nigdy nie byłaby spełniona — wykres zaniku boostu przestałby się rysować po cichu, bez błędu. Bramka wskazuje teraz `tab-ebics-profiles`; usunąć jej nie można, bo Plotly rysujący do ukrytego kontenera przyjmuje wymiar 0 i wykres wraca pusty.

**FW-068 — warunek startu asysty: konfigurowalny, z osobnym progiem dla jazdy**
- Dotąd asysta ruszała po spełnieniu dwóch warunków naraz: nacisk ≥ „Minimum pedal load" (per poziom, ustawialny) oraz obrót korby ≥ `START_MIN_STEPS` = 4 kroki kwadratury (zaszyte na stałe). Drugiego warunku nie dało się przestroić bez przebudowy firmware, a zatrzask spada natychmiast po zatrzymaniu korby (200 ms bez impulsu), więc ponowne złapanie asysty przy 25 km/h kosztowało dokładnie tyle samo nacisku co ruszanie spod świateł.
- **Kroki korby są teraz parametrem** `Crank movement to start` (Canable → Dynamics, 1–20, domyślnie 4). Zasięg globalny: to zabezpieczenie przed kiwaniem korbą, czyli własność roweru i czujnika, a nie poziomu asysty. `START_MIN_STEPS` pozostaje wartością domyślną i wartością zamrożonego monolitu Legacy.
- **Nowy próg dla jazdy** `Pedal load reduction while pedalling` (per poziom, 0–100 mV, domyślnie 0 = wyłączony) obniża „Minimum pedal load" tylko wtedy, gdy korba faktycznie się kręci. Ruszanie z zera i „Assist without crank rotation" zawsze mają pełny próg — tam nie ma ruchu korby jako zabezpieczenia. Punkt odniesienia dla nacisku może być inny przy kręcącej się korbie niż przy stojącej — na stojącym rowerze obrót korby niczego nie dowodzi (luźny łańcuch), na toczącym się dowodzi. Parametr jest jawny i per poziom, zaczepiony o `pedaling_active`, a nie o samą kadencję.
- **Druga droga do załączenia** `Engage on pressure rise` + `Pressure rise window` (per poziom, domyślnie 0 = wyłączona). Zamiast przekraczać próg bezwzględny, wystarczy że nacisk **wzrośnie** o zadaną wartość i utrzyma ten wzrost 40 ms, w oknie liczonym od chwili spełnienia warunku korby. Kolejność zdarzeń jest tu istotą zabezpieczenia: uderzenie w korzeń na wolnobiegu daje impuls, ale nie następuje po serii kroków PAS do przodu. Detektor jest odporny na dryf zera (patrz FW-058/059), bo mierzy różnicę, a nie poziom. Punkt odniesienia jest trzymany na **minimum** z okna — inaczej powolne narastanie nacisku przesuwałoby go razem ze sobą i różnica nigdy nie osiągnęłaby progu.
- **Obniżka wymaga jadącego roweru, nie samego ruchu korby.** Warunek prędkości `>= 1,0 km/h` (`RIDE_START_REDUCTION_MIN_SPEED_X100`): na stojącym rowerze można kręcić korbą do przodu przy luźnym łańcuchu i zerowym obciążeniu, czyli dokładnie w sytuacji, dla której pełny próg istnieje. Nie jest to literalne „> 0", bo pojedynczy fałszywy impuls prędkości na postoju (FW-036) nie może odblokować niższego progu. Ruszanie z zera i „Assist without crank rotation" zawsze mają pełny próg.
- Obie nowe funkcje domyślnie zerowe, więc zachowanie zaraz po wgraniu jest identyczne z dotychczasowym. Strojenie po jednej wartości naraz.
- **Odpowiedź na zapis konfiguracji mówi teraz prawdę.** Kontroler odpowiadał na każdy zakończony transfer wieloramkowy operacją 2 (`NORMAL_ACK`) bez patrzenia na wynik parsowania — blob odrzucony za złe CRC lub nieobsługiwaną wersję wyglądał jak zapis udany, więc narzędzie meldowało „written", a do flasha szły stare ustawienia. Teraz: zastosowany → `NORMAL_ACK`, odrzucony → operacja 3 (`ERROR_ACK`), transfer niekompletny → też `ERROR_ACK` zamiast ciszy i czekania narzędzia na timeout. `request-manager.js` w Canable już rozumie `ERROR_ACK` i rozwiązuje żądanie jako niepowodzenie. Stara funkcja `sendMultiframeWriteAck()` usunięta.
- **Zgodność v5→v6 zachodzi wyłącznie na drucie.** Blob v5 ze starszego Canable jest przyjmowany, nowe pola uzupełniane domyślnymi, a pierwszy „Save to Flash" zapisuje już v6. Z EEPROM v5 nie przychodzi nigdy: zmiana rozmiaru `MotorParams_t` unieważnia cały rekord przed parsowaniem, więc nie ma tam czego migrować. Canable **negocjuje wersję w dół** — v6 leci tylko do kontrolera, który sam zgłosił v6; wcześniej blok tuningu wysyłał v6 bezwarunkowo, co zablokowałoby zapis do starszej firmware.
- Blok tuningu podniesiony do v6 (32 B, `start_steps` na offsecie 22, trzy zarezerwowane u16). Kontrola długości w `tuning_config_apply_blob()` przepisana na tabelę wersji — poprzednia wyprowadzała minimalną długość z `TUNING_BLOB_LEN`, więc samo podbicie tej stałej zaczęłoby odrzucać poprawne bloby v3–v5. Przy okazji naprawiono dwie migracje z FW-053 (`hold_ms 700→1400`, `min_iq_pct 4→2`): testowały `version < TUNING_VERSION`, czyli po każdym podbiciu wersji odpalałyby się ponownie i nadpisywały wartość świadomie ustawioną z powrotem.

**FW-069 — rampy prądu per poziom i per bank**
- `Acceleration — current rise` i `Deceleration — current fall` (4 wartości) były globalne — jeden komplet na cały rower. Tymczasem `release_ms`, `power_rise_filter_ms` i `power_fall_filter_ms` **już były per poziom**, a to rampy najmocniej decydują o charakterze narastania mocy.
- Cztery wartości przeniesione do rekordu poziomu w blobie banków. Bank jest tam wymiarem tablicy, więc per poziom daje **per bank za darmo**. `assist_dynamics_apply()` nie sięga już do `tuning_config_ramp_*` — dostaje wartości przez `assist_dynamics_input_t`, tą samą drogą co `profile_release_ms`.
- **Walk Assist nie korzysta z tych ramp i nie ma korzystać.** `assist_dynamics_apply()` wychodzi z funkcji przed kodem ramp, gdy ustawione jest `walk_active`: WA ma własny kontroler prędkości (FW-060/FW-067) i sam prowadzi całą trajektorię Iq. Drugi element dynamiczny za jego regulatorem tylko pogorszyłby stabilność pętli. Pola ramp zostają na tej gałęzi celowo zerowe, a zero wybiera wkompilowane wartości awaryjne, więc żadna ścieżka nie może dostać rampy o zerowej długości.
- W Canable pola i oba wykresy (narastanie/opadanie) przeniesione z karty Dynamics do Profiles, obok pozostałych ustawień narastania mocy. Dochodzą **dwa** przełączniki kopiowania, oba działające tylko w obrębie aktywnego banku: szeroki **„Apply my edits to all levels in this bank"** (dowolne pole karty) i wąski **„Apply acceleration/deceleration ramps to all levels"** (wyłącznie cztery rampy — typowy przypadek: rampy identyczne wszędzie, reszta poziomu różna). Wąski obejmuje narastanie i opadanie, bo rampy stroi się parami. Świadomie nie ma opcji „zastosuj do obu banków": dwa banki istnieją po to, żeby się różniły.
- **Rampy zapisane starszym Canable są ignorowane.** Bajty 4–11 bloba tuningu zostają na drucie dla zgodności, ale firmware ich nie czyta. Bez tej informacji wygląda to jak usterka.
- Blob banków podniesiony do v6, rekord 35 → 46 B, blob 190 → 245 B. **Naprawione wersjonowanie rekordu:** `assist_modes_apply_bank_blob()` porównywał bajt „długość rekordu" ze stałą kompilacyjną i z niej liczył pozycję CRC, więc pierwsze powiększenie rekordu odrzuciłoby każdy zapisany bank i po cichu skasowało całą konfigurację profili. Teraz `buffer[5]` jest rzeczywistym krokiem odczytu, a pola powyżej starej długości są uzupełniane domyślnymi.
- Trzy pola FW-068 są na drucie `u8` (0–100 mV), a okno przyrostu w jednostkach 10 ms. Powód jest twardy: **protokół wieloramkowy przenosi całkowitą długość w jednym bajcie** (`rx_data_length` z `rx_data[0]`, `send_multiframe()` z `uint8_t length`), więc blob powyżej 255 B nie przechodzi w ogóle. Przy 245 B zostaje 10 B zapasu — kolejne pole per poziom będzie wymagało wcześniejszego przejścia protokołu na długość 16-bitową.
- Granice ramek dla 0x6021 (`< 23` → `< 30`) i 0x6024 (`< 2` → `< 3`) podniesione razem z blobami. Bez tego końcowe ramki są odrzucane bez śladu, a zapis kończy się błędem CRC bez wskazówki, co go spowodowało.

**FW-067 — dynamiczny próg COAST zależny od celu Walk Assist**
- `0.0263` został zastąpiony przed testem sprzętowym. Stały regulator `85/70 rpm` nie uwzględniał zmiany `Target chainring RPM`, więc przy niskim celu pozostawiał zbyt duży zapas, a przy celu bliskim maksimum działał inaczej względem wartości zadanej.
- `0.0264` wylicza próg prawdziwego `Iq=0` jako `target + 20 rpm`, a wznowienie RUN jako `target + 5 rpm`. Zachowana histereza ma zawsze 15 rpm: dla celu 20 progi wynoszą 40/25 rpm, dla 40 — 60/45, dla 50 — 70/55, a dla maksymalnych 60 — 80/65 rpm.
- Bankowy cel jest nadal walidowany do zakresu 20–60 rpm. Wartość spoza zakresu używa bezpiecznego celu domyślnego 50 rpm, dlatego wysoki lub błędny wpis nie może przepełnić obliczeń ani wyłączyć regulatora. Dodano kontrolę kompilacyjną, która nie pozwala ustawić histerezy większej lub równej offsetowi 20 rpm.
- START `80 Iq`, ograniczony RUN `5..36 Iq`, rampy `93,75/15,625/31,25 Iq/s`, sekundowe oczekiwanie bez Halla i `REACQUIRE <=24 Iq` pozostają bez zmian.
- Regresja sprawdza dynamiczne progi dla celu 20, 40, 50 i 60 rpm oraz pełne modele lekkiego i obciążonego napędu. Pełny zestaw **5/5 PASS**.
- Normalny debug `0.0264_M820_BL820.bin`: 88 844 B, SHA-256 `438CC4E68712586112C575DFC98352A3D1DF5FEB7C25550AE5BC198269B85CC7`, CAN diagnostics OFF. Diagnostyczny `0.0264-diag_M820_BL820.bin`: 93 384 B, SHA-256 `584A0BE4667C3AAEFE8901AF92F6352D62CE7F3A1E782D339DB416F60E6366DB`, CAN diagnostics ON. Oba bez segmentu RWE.
- Canable, format banków i zakres ustawienia celu nie zostały zmienione.

**FW-066 — Walk Assist: energiczny START, ograniczony RUN i coast 85/70 rpm**
- Po doprecyzowaniu odczucia docelowego `0.0262` został zastąpiony przed testem sprzętowym. Spowalniał wszystkie dodatnie rampy, także pierwszy START, oraz nadal opierał RUN na podłodze anti-stall. Właściwy model ma energicznie ruszyć tylko raz, a następnie regulować spokojnie w małym, niezerowym zakresie prądu.
- `0.0263` zachowuje jednorazowe `80 Iq`, ale przywraca osobną rampę START `93,75 Iq/s` — pełne zadanie pojawia się po około `0,85 s`. Po osiągnięciu 30% celu START kończy się do puszczenia przycisku i nigdy nie uzbraja ponownie podczas tej samej sesji.
- RUN jest ograniczony do `Iq_min=5` i `Iq_max=36`. Dodatnie zmiany mają limit `15,625 Iq/s`, normalne zmniejszanie `31,25 Iq/s`. Przy maksymalnym dodatnim błędzie model potrzebuje około `2,59 s` na przejście całego zakresu `5→36 Iq`, ponieważ również całka PI narasta wolno. Soft ceiling `36 Iq` nie może twardo obciąć prądu odziedziczonego ze START — zejście zawsze przechodzi przez rampę.
- Bankowy target 20–60 rpm jest miękkim celem. Niewielkie przekroczenie celu zmniejsza prąd tylko do `5 Iq`, nie do zera, dzięki czemu wirnik nie zatrzymuje się i nie uruchamia cyklu Hall/reacquire. Dawna krzywa anti-stall `0→48 Iq` została usunięta.
- Prawdziwe `0 Iq` podczas trzymania WA pojawia się dopiero w osobnym governorze przy `85 rpm` zębatki (`113 ERPS`). Coast pozostaje aktywny do spadku poniżej `70 rpm` (`93 ERPS`). Jeśli wolnobieg zatrzyma wirnik i Hall zniknie, firmware czeka 1 s, a następnie przechodzi do istniejącego łagodnego reacquire maks. `24 Iq`; celowy coast nie jest klasyfikowany jako jam.
- Hamulec, fault, bankowy limit prędkości koła oraz istniejące `LIMIT/STALL` pozostają nadrzędne. Przy ciężkiej blokadzie RUN nie przekracza `36 Iq`; bezpieczne zatrzymanie ma pierwszeństwo przed skokiem momentu.
- Nowy test modeluje START, miękki handover bez twardego clampu, RUN `5–36 Iq`, czas obu ramp, miękki overspeed, governor 85/70, utratę Halla podczas coast, reacquire, ciężką blokadę, stałe obciążenie i lekki napęd dochodzący do governora. Pełny zestaw **5/5 PASS**.
- Normalny debug `0.0263_M820_BL820.bin`: 88 772 B, SHA-256 `AB4CC37872E48F8C97429574A4471B4C876A680703962F504BAE9B7E23D99447`, CAN diagnostics OFF. Diagnostyczny `0.0263-diag_M820_BL820.bin`: 93 312 B, SHA-256 `BDCEC56A28C7D0A9872D8589A9144E81B9C0A538BDEC3F4D8F56C153BC637437`, CAN diagnostics ON. Oba bez segmentu RWE.
- Canable i format banków nie zostały zmienione. `Iq_min`, `Iq_max` i próg 85/70 są na tym etapie stałymi testowymi w firmware.

**FW-065 — znacznie dłuższe rampy narastania Iq w Walk Assist**
- Przed testem sprzętowym `0.0261` właściciel doprecyzował wymaganie: samo zmniejszenie docelowego Iq jest niewystarczające, ponieważ szybka rampa nadal daje nagły przyrost momentu i szybko rozpędza napęd. `0.0261` został zastąpiony przed wgraniem; nie jest buildem odrzuconym na podstawie testu sprzętowego.
- `0.0262` wydłuża wszystkie rampy dodatnie bez spowalniania zejścia: pierwszy START `250→62,5 Iq/s`, anti-stall `93,75→31,25 Iq/s`, normalne PI i `REACQUIRE` `46,875→15,625 Iq/s`. Osiągnięcie startowego `80 Iq` trwa teraz około `1,27 s` zamiast około `0,32 s`. Opadanie pozostaje `250 Iq/s`, aby nie zwiększać przestrzału.
- Maksymalne wartości z FW-064 pozostają: start `80 Iq`, handover `36 Iq`, anti-stall `48 Iq`, reacquire `24 Iq`. Zmieniono czas dochodzenia, nie twarde limity.
- Jawnie przyjęto kompromis bezpieczeństwa: przy nagłym bardzo dużym obciążeniu prąd nie może gwałtownie skoczyć tylko po to, aby za wszelką cenę utrzymać ruch. Jeśli wolna rampa nie wystarczy, zanik Halla/watchdog ma przejść do ograniczenia i zatrzymania WA; normalne obciążenia nadal muszą ruszać i stabilizować się.
- Test wymaga: przy `0,45 s` start nie może przekroczyć `30 Iq`, pełne `80 Iq` dopiero po około `1,25–1,30 s`, w pierwszych `0,6 s` reacquire nie więcej niż `10 Iq`, a przy ciężkim skoku obciążenia nie więcej niż `32 Iq` po 1 s. Pełny zestaw testów **5/5 PASS**.
- Normalny debug `0.0262_M820_BL820.bin`: 88 588 B, SHA-256 `2A4ABA23D2DF0F546C33B0ABBBAA6CF8160495CDF0A16D70185C0433A03D5477`, CAN diagnostics OFF. Diagnostyczny `0.0262-diag_M820_BL820.bin`: 93 128 B, SHA-256 `E403AB0E7722C742E093018D5A3ECB0FFD41299CCFAC12F0422B8FCB5B270E4B`, CAN diagnostics ON. Oba bez segmentu RWE.
- Canable i format ustawień nie zostały zmienione. Test wyłącznie na stojaku.

**FW-064 — łagodniejsze doganianie celu i bezpieczny cykl odzyskania WA**
- Test stojakowy `0.0260` wykazał zbyt mocne doganianie zadanych obrotów: regulator przestrzeliwał cel, przechodził w wybieg, po zaniku Halla ponownie ruszał, a po kilku cyklach przestawał działać aż do puszczenia przycisku WA. Bez logu nie potwierdzono stanu, lecz objaw i wymóg ponownego naciśnięcia odpowiadają zatrzaskowi `STALL`. `0.0260` został odrzucony jako known-good.
- Jednorazowy start z postoju `80 Iq` pozostaje bez zmian, ponieważ wcześniejszy test wykazał, że słabszy start nie rusza pewnie obciążonego napędu. Po potwierdzeniu ruchu strojenie jest łagodniejsze: `Kp 2→1`, ograniczenie błędu `18→12 ERPS`, przejęcie startu `60→36 Iq`, zasianie całki `12→8 Iq`, narastanie normalnej regulacji `93,75→46,875 Iq/s`, a opadanie `156,25→250 Iq/s`.
- Anti-stall poniżej 50% celu ma teraz maksimum `48 Iq` zamiast `60 Iq` i narasta z limitem `93,75 Iq/s` zamiast rampą startową `250 Iq/s`. Zachowuje przejście testu nagłego obciążenia, ale nie dogania celu tak energicznie jak start z postoju.
- Odzyskanie Halla ograniczono `36→24 Iq`. Próg twardej klasyfikacji utraty Halla wynosi teraz `30 Iq`, czyli więcej niż całe kontrolowane `REACQUIRE`; własna ograniczona próba nie może już zakwalifikować swojego prądu jako natychmiastowej awarii. Brak Halla przez 1,5 s nadal prowadzi do `LIMIT/STALL`.
- Test WA obejmuje cztery kolejne cykle overspeed/coast/reacquire i wymaga: braku ponownego START `80 Iq`, braku całkowania bez Halla, doganiania nie większego niż `24 Iq` oraz zachowania ochrony nagłego obciążenia. Pełny zestaw testów **5/5 PASS**.
- Normalny debug `0.0261_M820_BL820.bin`: 88 588 B, SHA-256 `1DE53EA14AC227F88D74DA34E387323A455CC00E989D4E6239EEA33F42C850EE`, CAN diagnostics OFF. Diagnostyczny `0.0261-diag_M820_BL820.bin`: 93 128 B, SHA-256 `C1006CFFED686F9BC392634DD11A4823B47F730E6EA732D7C21384160686D39D`, CAN diagnostics ON. Oba bez segmentu RWE.
- Canable i format ustawień nie zostały zmienione. `0.0261` wymaga wyłącznie testu stojakowego; test na ziemi pozostaje zablokowany.

**FW-063 — usunięcie szarpania WA po poprawce overspeed**
- Test stojakowy `0.0259` potwierdził, że regulator zaczął ograniczać narastanie obrotów, ale przy znacznie niższej prędkości pojawiło się szarpanie. Próg koła został wykluczony: przyczyną było przełączanie podłogi `5 Iq` tuż poniżej celu i `0 Iq` po jego osiągnięciu.
- `0.0260` oddaje PI cały górny zakres 50–100% celu — podłoga wynosi tam stale `0 Iq`. Anti-stall zaczyna działać dopiero poniżej 50% celu i rośnie płynnie `0→60 Iq` do 20% celu; poniżej 20% pozostaje `60 Iq`. Usuwa to przełączanie momentu przy celu, nie osłabiając ochrony przed zatrzymaniem przy dużym spadku obrotów.
- Regresja sprawdza monotoniczność krzywej i zabrania skoku większego niż `3 Iq` między sąsiednimi ERPS. Dodano model bardzo lekkiego obciążenia: PI ma utrzymać wąski zakres prędkości bez impulsów prądu i bez cyklu granicznego.
- Testy hostowe **5/5 PASS**. Normalny debug `0.0260_M820_BL820.bin`: 88 588 B, SHA-256 `56BD59EDF2CC5264090CC4CD96BEC4255A2D2EFBB3FD48A28A97A2B0E7287304`, CAN diagnostics OFF. Diagnostyczny `0.0260-diag_M820_BL820.bin`: 93 128 B, SHA-256 `F5809890A60DA3BB824423F4808709930AA25AA1D481314A6027F7F6A17E17AB`.
- Canable i format ustawień nie zostały zmienione. Późniejszy test stojakowy `0.0260` ujawnił agresywne cykle doganiania i końcowe wyłączenie, dlatego build został odrzucony i zastąpiony przez FW-064 / `0.0261`.

**FW-062 — zatrzymanie rozpędzania Walk Assist ponad celem**
- Test sprzętowy `0.0258` ujawnił, że na uniesionym kole WA stale przyspiesza aż do bankowego odcięcia prędkości. Przyczyną nie był znak Halla ani strojenie Kp: `regulate_iq_floor()` wymuszało `5 Iq` dla każdej prędkości od 50% celu wzwyż, również po przekroczeniu celu. Było to około 0,475 A dodatniego prądu fazowego, więc niemal nieobciążony napęd nie mógł przestać przyspieszać.
- `0.0259` wyłącza podłogę anti-stall po osiągnięciu celu. PI może teraz łagodnie zejść do prawdziwego `0 Iq`; istniejący `coast_requested/reacquire` nadal odpowiada za bezpieczny wybieg i powrót Halla. Podłoga `5→60 Iq` pozostaje bez zmian poniżej celu, gdzie chroni przed zatrzymaniem obciążonego napędu.
- Poprawiono błędne wymaganie testu `overspeed`: po odwinięciu całki oczekiwane jest `0 Iq`, nie `5 Iq`. Dodano regresję uniesionego koła bez oporu — dopuszcza tylko krótki, ograniczony wybieg podczas rampy zejścia i zabrania trwałego dodatniego momentu ponad celem.
- Testy hostowe **5/5 PASS**. Normalny debug `0.0259_M820_BL820.bin`: 88 588 B, SHA-256 `27CAF8B7ACE449B5942816806CDDF78F31EE206A9D111D3625C8C7E41996FC7D`, diagnostyka CAN OFF. Diagnostyczny `0.0259-diag_M820_BL820.bin`: 93 128 B, SHA-256 `E7E2C5872FD66F6DAC074BFACA29D350C2D4D9828687C63773CE756E02A915AF`; nie jest buildem do normalnej jazdy.
- Zapis ustawień banku w Canable nie został zmieniony — to osobny problem. Późniejszy test stojakowy `0.0259` wykazał szarpanie przy niższej prędkości, dlatego ten build został odrzucony i zastąpiony przez FW-063 / `0.0260`.

**FW-060 — Walk Assist utrzymujący stałe RPM zębatki**
- Nowy regulator utrzymuje bankowy cel 20–60 rpm zębatki na podstawie szybkiego pomiaru Hall ERPS (`rpm × 4/3`), a prędkość prowadzenia nadal wybiera się biegiem. Wheel speed jest wyłącznie bankowym warunkiem bezpieczeństwa.
- Jeden ciągły PI zastępuje droop oraz przełączenia `START/CLOSED_LOOP/OVERSPEED`: martwa strefa `±7 ERPS` (~`±5,25 rpm`), błąd sterujący ograniczony do `±18 ERPS`, anti-windup i ograniczona szybkość zmian Iq. Powyżej celu prąd płynnie maleje, bez zmiany stanu i bez resetu całki.
- Po pierwszym teście na rowerze start został lekko złagodzony do `72/52 Iq`, ale test `0.0252` wykazał zbyt mały moment ruszenia. `0.0254` zachowuje napinanie `18 Iq` przez 80 ms i przywraca podłogę startową `80 Iq` po ok. 0,38 s oraz przejęcie `60 Iq`. Początkowa całka nadal jest ograniczona do `12 Iq`, więc nie podtrzymuje nadmiernego rozpędzania przy lekkim obciążeniu. Dodatnia całka narasta wolno (`Ki=1`), natomiast po przekroczeniu celu odwija się cztery razy szybciej, aż do zera, aby sprawnie oddać moment po nagłym zdjęciu obciążenia.
- Stany zewnętrzne uproszczone do `OFF/REGULATE/LIMIT/STALL`. Po 1,5 s grace watchdog wykrywa brak ruchu, częściowe zakleszczenie i utratę Halla; `LIMIT` ma `15 Iq`, po 400 ms bez poprawy przechodzi w zatrzaśnięty `STALL`.
- Test `0.0251` ujawnił fałszywy `STALL`: po celowym zejściu Iq do zera wirnik zatrzymywał się, po 200 ms znikał Hall, a kod uznawał normalny wybieg za awarię. Po kilku cyklach WA pozostawał wyłączony aż do puszczenia przycisku. `0.0252` rozróżnia brak Halla pod prądem (`>=15 Iq` → safety) od wybiegu z `Iq<=2`; po wybiegu wykonuje jedną łagodną próbę odzyskania do `30 Iq` przez maks. 1 s, bez ponownego start floor i bez całkowania. Dopiero brak Halla po tej próbie prowadzi do `LIMIT/STALL`.
- Test `0.0252` ujawnił drugi wariant tego błędu: przy dużej całce Hall potrafił zniknąć, zanim ograniczona rampa zdążyła dojść do `Iq<=2`, więc „zgoda na wybieg” nie była zapisana i normalne odciążenie ponownie trafiało do `LIMIT/STALL`. `0.0254` zapamiętuje zamiar zwalniania PI już po wyjściu ponad górną granicę deadbandu, zwiększa łagodne odzyskanie do `36 Iq` przez maks. 1,5 s i nie wymaga wcześniejszego osiągnięcia zerowego prądu.
- `0.0255` eksperymentalnie utrzymuje minimum `5 Iq` podczas aktywnego `REGULATE`, więc przy trzymanym WA regulator nie schodzi normalnie do zera. Podłoga nie działa w `REACQUIRE/LIMIT/STALL`, po hamulcu ani po zakończeniu WA. Zanik Halla przy małym prądzie `<15 Iq` uruchamia teraz łagodne odzyskanie nawet bez wcześniejszego znacznika wybiegu; wcześniej ten narożny przypadek mógł pozostać w `REGULATE` bez prądu i bez wznowienia.
- `0.0256` zastępuje samo stałe minimum ochroną rzeczywistego ruchu: powyżej 50% celu podłoga wynosi `5 Iq`, między 50% a 20% płynnie rośnie do `60 Iq`, a poniżej 20% utrzymuje `60 Iq`. W obszarze anti-stall wyjście może narastać 250 Iq/s, ale nie uzbraja ponownie jednorazowego startu `80 Iq`. Test modeluje nagłe dociążenie po nieobciążonym wybiegu i wymaga, żeby wirnik nie osiągnął zera. Wszystkie limity bezpieczeństwa nadal mają pierwszeństwo.
- `walk_speed_controller.c` jest czystym modułem regulatora, a `walk_assist_motor.c` odpowiada za Hall i safety. Wspólna `assist_dynamics` nie nakłada drugiej rampy podczas WA. Na zboczu wyjścia z WA zeruje stan i natychmiast zwraca `Iq=0`, więc ostatni duży prąd WA nie jest już wykonywany przez rampę zwalniania zwykłej jazdy. Opcjonalny bankowy latch nadal świadomie utrzymuje WA po puszczeniu.
- Historyczny bankowy bajt `Walk current` pozostaje w blobie v5 dla zgodności ze starszym Canable, ale nie wpływa na FW-060. Gest zmiany banku podczas WA jest odkładany do końca zadania i zejścia Iq do zera.
- Telemetria `0x10206` raportuje teraz błąd ERPS, całkę Iq, podłogę startową i wiek Halla; `0x10205` zachowuje układ i dostaje flagi start/above-target/LIMIT.
- `tests/fw060_walk_speed_controller.js` → **PASS**. Build `.build/0.0256_M820_BL820.bin` przechodzi; test sprzętowy według `documentation/FW-060_WA_CONSTANT_RPM_CONTROLLER.md` oczekuje.

**FW-061 — poprawna klasyfikacja ruch/postój, uzbrajanie blokady i pełna diagnostyka zerowania**
- **Błąd klasyfikacji.** `bike_moving` było odczytywane dopiero w chwili zakończenia wybiegu, więc wybieg rozpoczęty w jeździe i zakończony po zatrzymaniu omijał blokadę 60 s. Do tego `MS.Speedx100` poniżej ok. 3 km/h okresowo spada do zera między impulsami koła. `main.c` **zatrzaskuje** teraz ruch na cały epizod bez pedałowania (krawędź impulsu koła **lub** `Speedx100 >= TQ_RECAL_MOVING_X100` **lub** `Speed_counter < SPEED_STOP_TICKS`), resetuje przy wznowieniu pedałowania i startuje w stanie „ruch" — stan nieznany jest traktowany jako jazda.
- **Uzbrajanie blokady.** `apply_offset_step()` zwraca rzeczywisty krok po ograniczeniu do ±5 mV; blokada uzbraja się tylko przy kroku różnym od zera. Ocena z `diff=0` raportuje `NO_CHANGE` i nie blokuje następnej potrzebnej korekty.
- **Pasmo bez zmian** (30 / 40 mV, trzy zgodne próbki), ale przekroczenie 40 mV przestało być nieme — dostaje wynik `OUT_OF_REACQUIRE_RANGE` i własny licznik. Pobieranie próbki z FW-059 nietknięte.
- **Telemetria `0x6025` v2** (24 → 56 B, pierwsze 22 bajty bez zmian): `raw_native`, zamrożony kandydat, rozrzut okna, ostatni krok ze znakiem, pozostała blokada w sekundach, flagi `coast_active`/`coast_was_moving`/`candidate_stable`, wynik ostatniej oceny jako enum oraz **kumulacyjne** liczniki okien, korekt i odrzuceń według przyczyny. W Canable doszła sekcja „Automatic zero re-calibration (coast)" w zakładce Torque.
- **Naprawiony rozjazd flag `0x6029`:** firmware pakuje bit 2 = hamulec i bit 3 = usterka czujnika momentu, a parser nazywał je `pedal_release_active` i `release_latched` i tak też pokazywał je interfejs. Poprawione na `brake_active`/`torque_fault`, etykieta zmieniona na „Brake / torque fault"; przy okazji wystawione cztery pozostałe bity (cofanie, kalibracja, utrata komunikacji, PWM). Układ bajtów bez zmian, więc wersja nie była podnoszona.
- **Znane ograniczenie:** zadana lista enumów nie ma wartości na „dryf 31–40 mV czeka na potwierdzenie", więc ten przypadek raportuje `NO_CHANGE` razem z „zero już na celu". Rozdzielenie wymagałoby wartości `REACQUIRE_PENDING`.
- **Konsekwencja klasyfikacji:** wybieg rozpoczęty w jeździe pozostaje „jazdą" także na światłach, więc nieograniczone zerowanie na postoju obejmuje już tylko epizody rozpoczęte na postoju. To wprost wymagany przypadek testowy, ale w praktyce blokada 60 s obowiązuje niemal zawsze.
- `tests/fw058_coast_rezero.js` pokrywa FW-058/059/061 wraz z portem zatrzasku ruchu → **PASS**.

**FW-059 — próbka zera ze środka wybiegu, odrzucanie niespokojnych wybiegów, mniejszy krok**
- FW-058 ograniczył, **jak często** zerowanie się odpala. Ta karta usuwa właściwą przyczynę: **skąd brana jest próbka**. Stary kod odczytywał średnią w chwili **zakończenia** wybiegu — czyli wtedy, gdy rowerzysta już naciska pedał, ale korba nie kliknęła jeszcze impulsu PAS. Do tego średnia miała stałą czasową 16 ms, więc próbka pochodziła praktycznie wyłącznie z tego najgorszego momentu i kierunkowo ciągnęła zero w stronę wstępnego nacisku.
- `coast_accumulate()` **zamraża** kandydata na zero po zamknięciu okna ustabilizowania (0,5 s), a resztę wybiegu ignoruje — narastający nacisk przed impulsem PAS nie ma już jak ruszyć próbki. Zamrożenie wypada ok. 5,5 s po zaprzestaniu pedałowania.
- Nowa **bramka spokoju**: rozrzut sygnału w oknie pomiarowym powyżej `TQ_RECAL_STABLE_MV` = 10 mV → **brak kalibracji zamiast złej kalibracji** (nierówna droga, obijający łańcuch, przestawiana stopa). Bramka stoi **po** kontroli wiarygodności bazy, więc wykrywanie usterki czujnika (Error 25) działa w dotychczasowym tempie.
- `TQ_RECAL_MAX_STEP` 20 → **5 mV** (~0,74 kg → ~0,19 kg). Pojedyncza korekta nie może już przewyższyć progu załączenia wspomagania (18 mV).
- Wynik w teście właściwości (symulowana jazda z wybiegiem co 20 s, zmienne zanieczyszczenie próbki) — najgorsza wędrówka zera w oknie 60 s: **26 mV przed FW-058 → 19 mV po samym FW-058 → 5 mV po FW-058+FW-059**. Dopiero teraz wędrówka zera jest wyraźnie mniejsza niż próg startu.
- Ryzyko: na bardzo nierównej nawierzchni bramka spokoju może odrzucać wszystkie wybiegi, a 5 mV na korektę wolniej nadąża za dryfem termicznym. Ratuje to zerowanie na postoju, które FW-058 celowo zostawił bez ograniczeń. Objaw ewentualnego problemu: wspomaganie wymagające coraz **mniejszego** nacisku w miarę nagrzewania czujnika.
- Test `tests/fw058_coast_rezero.js` pokrywa obie karty → **PASS**.

**FW-058 — rzadsze automatyczne zerowanie czujnika momentu**
- Objaw: wspomaganie dopina się w jeździe nieregularnie i za każdym razem przy innej sile nacisku, a start z miejsca jest powtarzalny. Przyczyna znaleziona w kodzie: próg załączenia to 18 mV (~0,67 kg przy 27 mV/kg), a jeden wybieg może przesunąć zero czujnika o `TQ_RECAL_MAX_STEP` = 20 mV (~0,74 kg) — czyli **więcej niż wynosi cały próg**. Zerowanie odpalało się po 1,5 s bez pedałowania, czyli praktycznie na każdym dłuższym wybiegu.
- Dlaczego tylko w jeździe: na wybiegu obie stopy leżą na pedałach, więc zapisywany „spoczynek" bywa dociążony (na postoju zwykle nie), a ruszając z miejsca naciskasz kilka–kilkanaście kg, gdzie przesunięcie o 0,74 kg jest niewyczuwalne.
- `TQ_RECAL_IDLE_TICKS` 6000 → **20000** (1,5 s → 5 s). Z istniejącym oknem ustabilizowania wybieg musi teraz trwać co najmniej **5,5 s**.
- Nowy `TQ_RECAL_MIN_PERIOD_TICKS` = 240000 (**60 s**): w ruchu zero można poprawić najwyżej raz na minutę. **Na postoju bez ograniczeń** — to ta wiarygodna kalibracja i ona nadal kompensuje dryf termiczny. Rozróżnienie po `MS.Speedx100 >= TQ_RECAL_MOVING_X100` (1,0 km/h), przekazywanym nowym argumentem do `torque_input_coast_update()`.
- Kontrola wiarygodności bazy i `cal_fault` **przed** blokadą — wykrywanie usterki czujnika (Error 25) działa w dotychczasowym tempie; blokada wstrzymuje wyłącznie ruszanie zera. Zerowanie przy starcie sterownika (`torque_input_startup_zero()`) nietknięte. Pasmo, ścieżka re-akwizycji dryfu i wielkość kroku bez zmian.
- Test hostowy `tests/fw058_coast_rezero.js` → **PASS**: blokada działa w ruchu i nie działa na postoju, korekta na postoju nie blokuje następnej w jeździe, wykrywanie usterki przechodzi przez blokadę, licznik nie schodzi poniżej zera.
- **Uczciwie o skuteczności:** to ogranicza zjawisko, nie usuwa go. W symulowanej jeździe z wybiegiem co 20 s najgorsza wędrówka zera w oknie 60 s spada z **26 mV do 19 mV** — nadal więcej niż próg 18 mV, bo `TQ_RECAL_MAX_STEP` nie był ruszany. Zmieniła się częstotliwość, nie amplituda pojedynczej korekty. Głębsza przyczyna (próbka brana na końcu wybiegu, z szybkiej średniej 16 ms, czyli już w trakcie narastania nacisku) zostaje do osobnej karty.

**FW-057 — kompensacja kadencji, włączana osobno dla każdego banku**
- Silnik oddaje mniej momentu przy wysokiej kadencji, więc wspomaganie siada przy 80–100 rpm mimo tego samego nacisku. Nowa mapa podnosi żądanie zależnie od kadencji: 100% do 70 rpm, 82% przy 80, 93% przy 100, 106% przy 110, 132% przy 120 i powyżej (trzymane, bez ekstrapolacji). Interpolacja liniowa — największy skok między sąsiednimi obrotami to **2,6%**.
- Mnożnik działa jednocześnie na `motor_power_mw` i `phase_iq_request`, w `finish_power_request()` — czyli po obliczeniu wspomagania bazowego, a **przed** limitem mocy, filtrem, przeliczeniem P/U, limitem Iq i rampą. Wszystkie trzy tryby oparte na pedałowaniu (Power, eMTB, Torque) przechodzą przez tę jedną funkcję, więc charakterystyka nie zależy od wybranego trybu.
- **132% podnosi żądanie, nie limity.** Limity mocy, prądu, prądu baterii, napięcia i temperatury działają niezmienione.
- Wyłączona dla manetki i Walk Assist (oba dokładane są poza `assist_modes_calculate()`), przy hamowaniu (`safety_cut` zeruje cel) oraz dla sztucznej kadencji zasianej przy starcie (jawny warunek `!cadence_seeded`).
- **Włącznik osobno dla każdego banku** — obejmuje wszystkie pięć poziomów w banku. Format banku v5: nagłówek 12 → 13 B, blok 189 → **190 B**, nadal w `bank_store[2][192]`, `BankBlob[192]` i limicie 24 ramek, **bez zmiany układu EEPROM**. Parser czyta v1–v5; starsze bloki nie mają tego bajtu, więc po aktualizacji kompensacja jest wyłączona i rower nie zmienia zachowania sam z siebie. Zapas w buforze topnieje do 2 B — następna flaga per bank powinna być maską bitową.
- Telemetria `0x6029` w wersji 4 (37 → 47 B): zastosowany mnożnik, moc silnika przed kompensacją, `u_abs` (nasyca się na `_U_MAX`, więc widać dobicie do limitu), napięcie paczki, kadencja bieżąca i stan przełącznika banku. Reszta listy ze schematu (`i_q_setpoint`, zmierzone `i_q`, flaga `BC_limit`) była już w bloku.
- Field weakening **świadomie poza zakresem** — najpierw pomiar samej kompensacji. Jeśli przy 120 rpm `u_abs` dobija do limitu, kompensacja nie odzyska pełnych 500 W i dopiero wtedy temat wraca.
- Testy hostowe: `tests/fw057_cadence_comp.js` (punkty charakterystyki, brak skoków, trzymanie powyżej 120 rpm, zakres 82–132%) i rozszerzony `fw056_bank_blob_roundtrip.js` (v5 = 190 B, mieści się w buforach i ramkach, flaga w obie strony, sterownik v4 nadal dostaje 189 B). Oba **PASS**.

**FW-056 — tryb wspomagania Power Curve (gamma)**
- Nowy tryb `ASSIST_MODE_POWER_CURVE = 6`: `support = support_min + (support_max - support_min) * x^gamma`, gdzie `x` to moc rowerzysty odniesiona do `reference_power_w`. Zastępuje suwak Progression jednym zrozumiałym pokrętłem — gamma 1,0 = liniowo, powyżej 1 = łagodny początek i mocna końcówka. Krzywa kształtuje **wyłącznie współczynnik wsparcia**; limit mocy, filtr, przeliczenie P/U, `max_iq_pct`, rampa Iq i FOC pozostają nietknięte.
- Bez `float` w pętli 4 kHz: tablica `inc/power_curve_lut.h` (23 wykładniki × 65 punktów + 9-punktowa podsiatka pierwszego odcinka = 3404 B Flash) z interpolacją liniową, generowana przez `tools/generate_power_curve_lut.js`.
- **Dwie gammy, po jednej na połowę okna wsparcia.** Dolna kształtuje odcinek `support_min` → środek okna (moc człowieka 0 → połowa mocy odniesienia), górna środek okna → `support_max` (połowa odniesienia → pełne odniesienie). Punkt styku jest ciągły, obie połowy liczone na pełnej rozdzielczości tablicy. Obie równe 1,0 dają dokładnie linię prostą. Górna gamma jedzie w bajtach 1–2 rekordu (`support_ratio_pct`, martwe poza POWER_LINEAR), dolna w bajcie 9 — nadal bez powiększania rekordu.
- **Podgląd w Canable liczy dokładnie ten sam algorytm co sterownik.** Generator wypuszcza tablicę również jako `ui/js/power-curve-lut.js`, a zakładka powtarza całkowitoliczbową matematykę firmware (obcinające dzielenie na promile, ta sama interpolacja, to samo okno wsparcia). Test firmware porównuje oba artefakty i wywala się, gdy moduł JS się rozjedzie z nagłówkiem C. Dotyczy to **wyłącznie kształtu krzywej** — Startup Boost, filtry, Smooth Start, rampa Iq i limity nie są modelowane na wykresie; do tego służy telemetria `0x6029`. Podgląd pokazuje teraz dwa wykresy obok siebie: Support ratio i Requested motor power.
- Zakres gamma **0,3–2,5, krok 0,1**. Poniżej 1 krzywa wygina się w drugą stronę — pełne wsparcie już przy lekkim nacisku, maksimum na długo przed mocą odniesienia. W interfejsie oznaczone jako agresywne, z zaleceniem pierwszego testu na stojaku.
- Poprawność mierzona na **dostarczonej mocy silnika**, nie na wartości krzywej: do silnika idzie `moc człowieka × wsparcie`, więc obszar największego błędu tablicy (tuż przy zerze) to jednocześnie obszar, gdzie moc człowieka jest bliska zeru. `tests/fw056_power_curve.js` → najgorszy przypadek **1,41 W** (0,17% mocy dostarczanej w tym punkcie) przy budżecie 1 W lub 1%, dla odniesienia 300 W i okna wsparcia 500 pp.
- **Format banku bez zmian: rekord 35 B, blok 189 B.** Gamma dzieli bajt 9 rekordu z `progression_pct` — pola należą do różnych trybów i nigdy nie występują razem. `bank_store[2][192]`, `BankBlob[192]` i strażniki multiframe nietknięte, żadnej migracji profili.
- `BANK_BLOB_VERSION` podniesione do 4 **przy identycznym układzie bajtów** — służy wyłącznie jako znacznik możliwości. Canable pokazuje tryb dopiero po odczytaniu banku w wersji 4 i odsyła v4 tylko do sterownika, który sam ją zgłosił; starsze firmware nigdy nie zobaczy bloku v4. Parser przyjmuje v1/v2/v3/v4.
- `calculate_support_ratio_pct()` rozbite na jawny `switch` po trybie plus wspólne `normalize_power_support_bounds()`. Dotychczasowe „jak nie Linear, to Progressive" przy trzecim trybie Power uruchamiałoby cichaczem zły algorytm. Warunek „brak wsparcia" w `calculate_power()` też jest teraz per tryb.
- Canable: `bafang-parser.js`/`canbus.js` obsługują v4 i bajt kształtu, zakładka Profiles dostaje tryb „Power Curve" z suwakiem gamma, a wykres podglądu — przełącznik **Support ratio (%) / Requested motor power (W)** wraz z linią `reference_power_w` i limitem mocy. Test `tests/fw056_bank_blob_roundtrip.js` pilnuje, że bajt 9 znaczy właściwą rzecz w obu trybach i że blok zostaje przy 189 B.
- Bez zmian w zachowaniu trybów Linear / Progressive / eMTB / Torque. Wycofanie = przełączenie poziomów z powrotem na Linear.

**FW-024 — pewne wykrywanie kierunku PAS (zatrzask cofania)**
- W trybie ride core cofanie korbami nie ucinało wspomagania od razu (moc schodziła rampą). Pomiar `0x6029`: 28 s równego kręcenia wstecz, flaga cofania (`Backwards_counter>=4`) ani razu — 0 na 111 próbek. Przyczyna: dekoder kwadraturowy nettuje kroki przód/tył z progiem 4, a jitter korby przy cofaniu wstrzykuje pojedyncze kroki „w przód", które kasują licznik przed progiem. Sam czujnik i matematyka kwadratury są dobre (Legacy, czytający ten sam dekoder, działa poprawnie — reaguje na `fwd_run`, nie na próg 4).
- Poprawka w warstwie czujnika: pierwszy krok wstecz **zatrzaskuje** `Backwards_counter` na `BACKWARD_LATCH_COUNT=8` (natychmiast ≥4 → `safety_cut` w ride core), a kroki w przód bleedują go po 1 → do skasowania trzeba ~5 czystych kroków w przód (histereza zbliżona do re-engage `fwd_run`). Odporne na jitter, nie miga.
- Zakres: wyłącznie `inc/config.h` + gałąź kroku wstecz w `src/main.c` (`reg_ADC_processing`). Bez zmian w silnikach, rampach, limitach; tablica `qd[16]` i wykrywanie stopu bez zmian. Ryzyko: krótki zanik wspomagania na fałszywy krok wstecz przy jeździe w przód — zbieżne z zachowaniem Legacy (`fwd_run`), oceniane nisko.

**0.0191 / FW-022 + FW-023 — skalibrowane wartości domyślne Halla i integralność rekordu EEPROM**
- **FW-022 — kalibracja przeżywa flashowanie.** Kalibracja `0x6200` została powtórzona, a jej wynik odczytany przez `0x6017`: `hall_order=1`, kąty `-134/-74/-12/+48/+107/+167°`, `angle_correction=+6°`. Zestaw jest wewnętrznie spójny (sześć przejść co ~60°, korekcja = dokładnie `6 × one_deg`). Wartości wpisano do `src/main.c` jako domyślne w miejsce kątów obcego silnika, a `MP.angle_correction` z `0` na `71582790`. Ponieważ flashowanie kasuje stronę EEPROM, sterownik startuje teraz na własnej kalibracji bez potrzeby jej powtarzania.
- **FW-023 — rekord EEPROM nie da się już „uszkodzić w połowie".** Dotychczas `write_virtual_eeprom()` kasowało całą stronę i przepisywało ją od nowa, a walidacja przy odczycie sprawdzała jedno słowo (`Hall_13 != 0xFFFFFFFF`). Zanik zasilania w trakcie zapisu (~10 ms okna) zostawiał rekord uznawany za ważny, w którym pozostałe kąty i całe `MP` miały `0xFFFF` — dokładnie podpis awarii `0.0185` (`voltage_min=0xFFFF`) oraz buczenia silnika bez obracania koła.
- **Stopka rekordu zapisywana na końcu:** magic `0xEB1C5001`, wersja, długość i CRC32 lądują w pamięci **po** kątach i `MP`. Rekord staje się ważny dopiero po zapisaniu ostatniego słowa, więc przerwany zapis jest odrzucany i firmware wraca do wartości skompilowanych. Ten sam wzorzec działa od dawna na stronie SOC (`soc_slot_t`), tu został przeniesiony na stronę parametrów.
- **Kontrola sensowności kątów przy odczycie:** `hall_order` musi być `±1`, a sześć posortowanych kątów musi mieć wszystkie odstępy (z zawinięciem) w przedziale `45°–75°`. Zestaw skalibrowany i stary skompilowany przechodzą; rekord po przerwanym zapisie (pięć kątów `-1`) jest odrzucany i zastępowany wartościami z kodu. To blokuje objaw buczącego silnika u źródła.
- **Brak zapisu, gdy nic się nie zmieniło:** `write_virtual_eeprom()` porównuje kąty i `MP` z zawartością pamięci i przy zgodności nie wykonuje cyklu kasowania. Wyświetlacz odsyła te same ustawienia (`0x3203`) przy każdym starcie, więc dotąd oznaczało to jedno kasowanie strony na każdą jazdę — zbędne zużycie pamięci i okno ryzyka.
- **Zmiana układu `MotorParams_t` unieważnia rekord automatycznie:** stopka leży zaraz za strukturą, więc jej przesunięcie powoduje odrzucenie starego rekordu zamiast wczytania przesuniętych śmieci. `inc/main.h` pozostał nietknięty — układ `MotorParams_t` się nie zmienia.
- **`0x6017` w wersji 2:** 37 B (było 36), dodany bajt stanu rekordu: `0` = rekord ważny, `1` = brak ważnego rekordu (wartości domyślne), `2` = rekord ważny, ale kąty odrzucone przez kontrolę.
- **Skutek jednorazowy:** stara strona nie ma stopki, więc przy pierwszym starcie `0.0191` zostanie odrzucona i ustawienia użytkownika wrócą do domyślnych. Kalibracji Halla powtarzać nie trzeba — jest w kodzie.
- **Poza zakresem:** dwie strony na przemian (A/B), które chroniłyby też przed *utratą* ustawień przy zaniku zasilania, oraz odkładanie zapisu do zatrzymania silnika. Obecna zmiana chroni przed uszkodzeniem, nie przed utratą.
- Build sprzętowy: `.build/0.0191_M820_BL820.bin`, 81912 B, SHA-256 `1F297672E7207E69FEFD23D7F30148089CB2A8537BE85018708F51D9743A131E`. Kompilacja bez błędów, ostrzeżenia wyłącznie zastane.
- **Test sprzętowy potwierdzony (2026-07-23).** `0x6001` zwraca `eVD 0.0191`; `0x6017` odpowiada w formacie 2 i zawiera dokładnie wpisane wartości kalibracji, stan rekordu `0` (ważny, kąty przeszły kontrolę). Walk Assist obraca kołem ze zdjętym i z założonym łańcuchem. **Silnik ożył i wspomaga w jeździe — zarówno na Legacy, jak i na ride core**; pierwotna awaria braku wspomagania jest usunięta. Przez cały test `state_number=0`, napięcie `36,65–36,89 V` bez zapadania. Ponowny odczyt po testach: rekord bajt w bajt identyczny.
- **Otwarte po teście:** (1) Walk Assist przestrzeliwuje cel `6,0 km/h` do `17,2 km/h` na stojaku — zabezpieczenie działa (prąd spada do zera), przyczyną jest najpewniej brak obciążenia przy pełnoprądowym starcie `WA_START_PCT=100`; do weryfikacji na ziemi przed strojeniem. (2) Ustawienia użytkownika wróciły do fabrycznych — zapowiedziany jednorazowy skutek odrzucenia starego rekordu bez stopki.

**0.0189 / 0.0190 — rebranding na eVistDrive**
- Pole informacyjne HMI (`0x6001`) zwraca `eVD <wersja>` zamiast `EBICS <wersja>`. Makro `EBICS_BUILD_VERSION` pozostaje jako identyfikator techniczny buildu.
- Build `0.0188` nie powstał — numer przepadł przy nieudanym buildzie.

**0.0187 / FW-022 — diagnostyka kalibracji Halla po potwierdzonym teście Walk Assist**
- Po naprawie `Para1` w 0.0186 Walk Assist docierał do rdzenia silnika (`i_q` oraz PWM były aktywne), ale wirnik tylko buczał i nie obracał koła. Wykluczyło to poziomy wspomagania, czujnik nacisku i silnik jazdy ride core/Legacy jako bezpośrednią przyczynę tego objawu.
- Autodetekcja pozycji `0x6200`, wykonana ze zdjętym łańcuchem, zakończyła fazę prądową i została potwierdzona testem sprzętowym: po kalibracji Walk Assist poprawnie obracał kołem. To wskazuje na kąty/kolejność Halla albo korekcję komutacji.
- 0.0187 dodaje tylko dla Canable/BESST (`source=5`) odczyt `0x6017`: 36 B z kierunkiem Halla, sześcioma kątami `q31`, `MP.angle_correction` i stanem kalibracji. Zmiana jest diagnostyczna i nie zmienia sterowania silnikiem.
- Odczyt po wgraniu 0.0187 zwrócił stare wartości skompilowane oraz `angle_correction=0`. Ponieważ przed flashowaniem skalibrowany Walk Assist działał, wynik kalibracji najprawdopodobniej nie przetrwał aktualizacji. Nie jest to jeszcze finalna poprawka: dokładne wartości trzeba odczytać bezpośrednio po kolejnej kalibracji, przed restartem lub flashowaniem.
- Build sprzętowy: `.build/0.0187_M820_BL820.bin`, 81156 B, SHA-256 `04EF08B1A5BF71FA8CFE10B9836CC51D12E920DAEAE95BBC2D7993869FED9D07`.

**0.0186 — naprawa uszkodzonego `Para1` potwierdzona odczytem CAN**
- Bezpośredni odczyt pracującego sterownika z firmware `0.0185` wykazał wspólną przyczynę braku ride core, Legacy i Walk Assist: zapisany próg podnapięciowy miał wartość `0xFFFF`. Po przeliczeniu dawało to `voltage_min=3855` ADC, więcej niż bieżące napięcie akumulatora (`37,17 V`), więc wspólny limiter zawsze zerował żądanie prądu.
- Ten sam rekord zawierał `0xFF` w napięciu systemowym/maksymalnym oraz limitach poziomów, `58 A` limitu baterii i wyliczony z `Para1[9]=254` skrajnie zawyżony limit fazowy. Samo odblokowanie undervoltage byłoby niebezpieczne.
- `parse_MOparams()` waliduje teraz cały krytyczny rdzeń parametrów po starcie. Naprawia wartości spoza bezpiecznych zakresów i zapisuje poprawiony rekord do EEPROM, zachowując dołączone później banki Ride Core, tuning, wybór silnika i kalibrację nacisku.
- `parse_DPparams()` stosuje tę samą walidację przy zapisie z Canable/HMI i nie dzieli przez zero, gdy `Para1[39]` jest zerowe.
- Limity poziomów większe niż `100%` wracają do bezpiecznego układu `20/40/60/80/100%`, a limity prędkości do `100%`.
- Poprawiono granice inicjalizacji `assist_profile`: tablica ma `5 x 6`, a stara pętla zapisywała `6 x 7` i wychodziła poza jej koniec.

**Pedał-assist / CAN — poprawki po teście 0.0177/0.0179 (brak wspomagania w ride core i Legacy)**
- **Reset `0x6101` nie zostawia już martwej konfiguracji:** stary przycisk `CalibrateTorqueSensor` wysyła `0x6101`, a w EBICS ta komenda jest historycznie resetem EEPROM. Po takim resecie brakowało domyślnych `speedLimitx100` i `wheel_cirumference`, więc po restarcie limit prędkości mógł być `0` i oba silniki jazdy widziały zerowy limit. `InitEEPROM()` zapisuje teraz `SPEEDLIMIT=2500` i `WHEEL_CIRCUMFERENCE=2218`, a `parse_MOparams()` naprawia też już zapisany/stary EEPROM z zerami.
- **Zapis `0x3203` jest odporny na zera i krótkie ramki:** jeśli HMI/BESST poda błędny limit prędkości albo obwód koła, firmware wraca do bezpiecznych defaultów zamiast zapisać konfigurację, która wycina wspomaganie.
- **Wyświetlacze 3/5/9 poziomów:** 10 slotów HMI mapuje się teraz na 5 realnych profili: `1/2 -> L1`, `3/4 -> L2`, `5/6 -> L3`, `7/8 -> L4`, `9 -> L5`. Dodano też obsługę prostych kodów numerycznych `4/5/7/8/9`; `6` zostaje Walk Assist.
- **Legacy pressure floor:** stare `TQO_threshold=3299` z EEPROM nie odwraca już mapowania nacisku do prądu. Domyślny próg startu podłogi naciskowej to `750 + TQ_GATE_MIN`.
- **Diagnostyka `0x6029` v2:** dodano flagi blokad wspólnych dla obu trybów (hamulec, PAS, wstecz, torque fault, aktywna kalibracja, watchdog CAN, PWM) oraz bieżące żądanie `i_q`, żeby następny test pokazał, gdzie dokładnie znika wspomaganie.

**Walk Assist — strojenie po teście 0.0135: start ×2, utrzymanie ÷2**
- Objaw z jazdy: pierwsza chwila za słaba, potem po ~1 s rower „gna" aż do odcięcia anty-przelotowego.
- **Start = bezwzględny % prądu fazowego** (`WA_START_PCT`=100): sufit przy 0 km/h to teraz pełny prąd fazowy (było min(200%·wa_max, 60% fazowego) = 60%; żądane ×2=120% obcięte fizycznie do 100%). Odwiązany od Walk Current z Canable/HMI — ta sama filozofia co niezależny od poziomów boost pedałowy. Wygasa liniowo do sufitu utrzymania przy 3 km/h (`WA_START_FULL_SPEED` bez zmian).
- **Utrzymanie = połowa ustawionego Walk Current** (`WA_HOLD_PCT`=50): sufit PI, sufit integratora i wygaszanie przy celu liczone z `wa_hold = fazowy·walk_assist_current·50%/100` — przy zapisanych 30% realnie 15% fazowego. Skalowanie w firmware, więc działa niezależnie od tego, co HMI/Canable ma zapisane.
- Usunięte `WA_START_BOOST_PCT` i `WA_BOOST_CEIL_PCT` (zastąpione przez `WA_START_PCT`); kick slew 180 ms i szybka rampa WA bez zmian.

**Walk Assist — strojenie po jeździe testowej 0.0132 (4 poprawki)**
- **Zwłoka załączania usunięta:** w trybie WA zewnętrzna rampa `i_q` przełącza się na szybkie tempo (0,3 s w górę / 0,14 s w dół zamiast 2,3 s / 1,0 s na postoju). WA ma własny kick 180 ms, więc podwójna rampa tylko opóźniała start; szybka rampa w dół dodatkowo przyspiesza cięcie anty-przelotowe z ~0,9 s do ~0,14 s.
- **Start boost — ruszenie 2× mocniejsze:** sufit prądu przy ruszaniu podniesiony do `WA_START_BOOST_PCT`=200% `wa_max` przy 0 km/h, wygasa liniowo do 100% przy `WA_START_FULL_SPEED`=3 km/h; twardy limit `WA_BOOST_CEIL_PCT`=60% prądu fazowego. Poniżej 3 km/h wymuszona podłoga = pełny boost (gwarantowane dopchnięcie), narastająca przez kick 180 ms. Integrator PI dalej clampowany do `wa_max` — boost nie nawija się w całkę.
- **Anty-przelot 6 km/h:** `WA_FADE_BAND` 150→250 (wygaszanie mocy od 2,5 km/h przed celem), `WA_NEAR_HOLD_PCT` 25→15 (przy celu 15% `wa_max`). Razem ze słabszym utrzymaniem i szybką rampą w dół eliminuje przelatywanie zadanej prędkości.
- **`walk_assist_current` domyślnie 30%** (było 50%): boost 2× = dokładnie 60% fazowego; słabsze utrzymanie dodatkowo ogranicza przelot. Jeśli HMI ma zapisane 50, ustawić 30 w Canable (Para1[36]).

**Pedał-assist — niższy próg startu**
- `TQ_GATE_MIN` 25→18 mV: delikatnie lżejszy nacisk uruchamia wspomaganie (próg wspólny dla wszystkich poziomów). Odczuwalna różnica poziomu E wynika ze skalowania mocy (profil E w Canable), nie z progu — jeśli E dalej za słabe przy starcie, podnieść pierwszy segment profilu E w Canable.

**CAN — przywrócony odczyt Para0 dla Canable (zakładka Full Assist)**
- Naprawa pustego ekranu Info w HMI podmieniła odpowiedź READ 0x6010 na fabryczny 4-bajtowy mini-blok `01 00 02 06` dla WSZYSTKICH pytających — Canable przestał dostawać Para0 i zakładka Full Assist była nieaktywna (Assist Light działała, bo czyta Para1/0x6011).
- Teraz odpowiedź zależy od nadawcy żądania: **source=5 (BESST/Canable) → pełny multiframe Para0**, source=3 (HMI) i inne → mini-blok jak fabryka. Handshake ekranu Info w HMI bez zmian; zapisy Para0 z Canable działały cały czas.

**Pedał-assist — boost startowy (`STARTUP_BOOST`); zastępuje `STARTUP_FLOOR`**
- Boost startowy: **mnożnik nacisku** malejący geometrycznie z kadencją — `factor(kadencja)% = STARTUP_BOOST_FACTOR × (1 − CADENCE_STEP/256)^kadencja`. Przy kadencji 0 nacisk wzmocniony o `STARTUP_BOOST_FACTOR`=200%, boost sam wygasa w miarę rozpędzania korby (`STARTUP_BOOST_CADENCE_STEP`=50 steruje tempem zaniku). Kick proporcjonalny do siły nacisku: mocno depczesz → mocny, ale kontrolowany start.
- Trzy tryby aktywacji (`STARTUP_BOOST_MODE`): 0=CADENCE (zawsze, gaśnie z kadencją), 1=SPEED (tylko od postoju, wyłączany >45 rpm), 2=AUTO (wyłączany przy małym nacisku w ruchu, próg `STARTUP_BOOST_AUTO_TQ`).
- Wzmocniony nacisk zamykany do pełnego `MP.phase_current_max` (kick niezależny od poziomu). Działa na `mapped_torque`, więc przechodzi przez te same bramki co reszta (latch: nacisk + 4 kroki do przodu) — bez wzbudzania na zjeździe.
- **`STARTUP_FLOOR` usunięty w całości** (kod + parametry): jeden mechanizm boostu zamiast dwóch nakładających się — feedback z jazdy jednoznacznie wskazuje, co stroić.
- **Fix interakcji z cadence seed:** seed publikuje sztuczne 10 rpm zanim istnieje pomiar kadencji, przez co boost liczył współczynnik przy kadencji 10 zamiast 0 i kasował sam siebie w chwili ruszenia (przy step=50 z 200% zostawało ~23%). Nowa flaga `cadence_seeded` (ustawiana przy seedzie, kasowana pierwszym realnym pomiarem lub stopem) — boost traktuje seedowaną kadencję jako 0. Dodatkowo `STARTUP_BOOST_CADENCE_STEP` 50→25 (typowe ride core): ~36% boostu przy 10 rpm, wygasa ~40 rpm.
- **Rampa i_q na postoju skrócona pod boost:** `IQ_RAMP_UP_SLOW_TICKS` 9200→2400 (2,3 s → 0,6 s) — wolna rampa rozsmarowywała kopniak boostu na 2 sekundy; 0,6 s pozwala go poczuć, nadal chroniąc napęd. Jeśli start za ostry: wrócić do 9200.

**Wspomaganie pedalowania — poprawa jakości jazdy**
- Torque EMA przeniesiony na pełną rozdzielczość kwadraturową (co 3,75° / 96×/obrót zamiast co 15°). Algorytmy widzą teraz profil siły przez cały obrót, nie tylko co 15°. Eliminuje pulsowanie on/off przy małej sile i zbyt szybkie zanikanie mocy przy zmniejszaniu nacisku.
- Cadence-gate: `torque_counter` resetuje się przy każdym kroku kwadraturowym do przodu, jeśli `torque_filtered > 0`. Silnik nie wchodzi w decay podczas martwego punktu korby dopóki korba się kręci i był jakikolwiek moment.
- `TQFILTER` domyślnie zmieniony z 4 na 6 — zachowuje tę samą stałą czasową filtra (~667 ms @ 60 RPM) przy nowej częstości aktualizacji.
- Slew limiters na prądzie `i_q` (IQ_SLEW_UP=5, IQ_SLEW_DOWN=10 mA/tyk): łagodne zaangażowanie/wyłączenie silnika, ochrona napędu przed szarpnięciem.
- Rampa `i_q`: nowy `IQ_RAMP_TIME_MODE=1` używa ułamkowego akumulatora, więc czasy są przewidywalne przy pętli 4 kHz: narastanie ok. 2,3 s / 0,3 s, opadanie ok. 1,0 s / 0,14 s zależnie od prędkości i kadencji. Hamulec, kręcenie wstecz i odcięcie termiczne nadal działają natychmiast.
- Miękkie odcięcie stopnia mocy (`SOFT_CUTOFF_ENABLE=1`): usuwa „klik" na samym końcu wspomagania. Zamiast skokowego `timer_primary_output_config(DISABLE)` po zatrzymaniu wirnika, napięcia faz zjeżdżają liniowo do wektora neutralnego (`_T/2`) przez `SOFT_CUTOFF_TICKS`=40 (≈10 ms), dopiero potem mostek jest odcinany. Taktowanie w `reg_ADC_processing` (4 kHz). Ścieżki awaryjne (hamulec/wstecz/przegrzanie) nadal tną natychmiast; start bez zmian.
- Start kadencji: `START_CADENCE_SEED_ENABLE=1` publikuje tymczasowo 10 rpm po 2 poprawnych krokach do przodu i realnym nacisku. Nie uruchamia silnika samodzielnie; `START_MIN_STEPS=4` i `TQ_GATE_MIN=25` nadal pilnują, żeby nie było wzbudzenia od ruchu przód-tył bez nacisku.

**Walk Assist — zamkniętopętlowy regulator PI prędkości**
- Zastąpiono prosty mapowanie prędkości → prąd regulatorem PI utrzymującym `walk_assist_speed`.
- Rozróżnienie kick (start z miejsca) vs resume (rozruch w ruchu): kick tylko gdy `Speedx100 < WA_KICK_SPEED`.
- Anti-windup integratora, kickstart slew 180 ms, zabezpieczenie przed przekroczeniem 6 km/h.
- `walk_assist_current` domyślnie 50% (poprzednio 30%).
- Poprawka: limit prędkości nie obcinał mocy Walk Assist (`!MS.pushassist_flag`).

**Czujnik momentu — detekcja usterek i auto-kalibracja (Error 25)**
- Detekcja sygnału poza zakresem (300–4300 mV, debounce 100 ms) → Error 25.
- Cykliczna re-kalibracja zera podczas wybiegu (korba ≥1,5 s w spoczynku): korekta offsetu max 20 mV/wybieg, pasmo ±100 mV, wymóg 3 zgodnych pomiarów dla dużych dryfów.
- Sanity-check startowy: jeśli surowy odczyt spoczynkowy poza oknem 300–1500 mV → offset zignorowany, Error 25 do pierwszego poprawnego wybiegu.

**CAN — pełna emulacja ramek oryginalnego firmware M820**
- `sendCAN_3100()`: emulacja węzła czujnika momentu (source=1), frame 10 ms z cadence i `torque_on_crank` — wymagany przez HMI do wyświetlania kadencji i momentu.
- `sendCAN_3202()`: ramka keepalive Walk Assist co ~120 ms — bez niej HMI wyłącza tryb WA po kilku sekundach.
- `sendCAN_status_broadcast()`: heartbeat 0x1200 (hamulec), 0x320F (status), 0x3000 — wymagany przez HMI do migania ikony WA.
- Refaktor timingu CAN: osobne liczniki per ramka zamiast round-robin `pollnumber`. Timings: heartbeat 480 ms, prędkość 280 ms, kadencja 1480 ms, misc 320 ms.
- `sendAcknoledge()`: odpowiedź do source ramki żądania zamiast hardcoded BESST (target=5).

**SOC / zasięg**
- Nauka zużycia Wh/km osobno dla każdego poziomu wspomagania (`wh_km_level[10]`). Zmiana poziomu nie zeruje nauki.
- Poprawka: po doładowaniu "top-up" (np. z 96% do 100%) SOC nie wracał do 100% — warunek `soc_ocv - soc_real > RECHARGE_MARGIN_PCT (5%)` nie był spełniony, licznik Coulombów startował od zapisanego ~96%, a korekcja OCV (gain=0,02/s) dobijała do ~99% dopiero po kilku minutach. Dodano osobny przypadek: jeśli napięcie ogniwa przekracza górną granicę tabeli OCV (4,07 V/ogniwo = niezbite 100%) i zapisany SOC ≥ 80%, licznik Coulombów jest natychmiast resetowany do 100% / pełnej pojemności.

**Inne**
- Wersja buildu: `EBICS_BUILD_VERSION` wstrzykiwana przez `build_firmware.ps1` do `inc/build_version.h` (gitignored) i wysyłana w polu info HMI.
- Hall autodetect: watchdog feed + keepalive CAN podczas kalibracji (procedura blokuje main loop >5 s).
- `PH_CURRENT_MAX` powiązany z `BATTERYCURRENT_MAX / CAL_I`.
- Usunięto: `statehistory[]`, `Poll_commands[]`, `pollnumber`, `soc_have_real_consumption`.

### Strojenie (Canable)

| Parametr | Pole Canable | Domyślna wartość | Uwaga |
|---|---|---|---|
| TQfilter | Ride Mode (per poziom) | 6 | Obniżyć do 4–5 dla szybszej reakcji |
| PAS timeout | Current Loading Time | 1 (=400 tyk) | Zwiększyć do 5 (=2000 tyk) przy pulsowaniu |
| Walk Assist prąd | Walk Assist Current | 30% | Boost startowy = 2× tej wartości (max 60% fazowego) |
| Walk Assist prędkość | Walk Assist Speed | 6,0 km/h | |

---

## Poprzednie commity (gałąź test/soc-temp)

- `aad8365` feat(pas): quadrature decoder (PC12+PD2) for cadence & direction
- `20dfb2f` revert(assist): restore origin/M820 pedal-assist + torque logic for clean re-analysis
- `447cb26` fix(range): send remaining range in 0.01 km units for 0x3200
- `bf8d5f5` feat(assist): real-time torque filtering + disable latched min-assist
- `544aab6` feat(walk-assist): ramp power up over ~700ms on engage
- `4ff88a9` fix(temp): set TEMP_OFFSET_C=11 to match original firmware
