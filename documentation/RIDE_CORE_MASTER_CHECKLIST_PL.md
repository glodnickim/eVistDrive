# EBICS Ride Core — główna checklista wdrożenia

Aktualizacja: 2026-07-16

Gałąź firmware: `refactor/ride-core`

Ostatni zatwierdzony commit: `f69b8e5` (`torque: hold fault 5 s and cut all assist on Error 25`)

Ostatni sprawdzony build: `0.0159_M820_BL820.bin`

Silnik uruchamiany domyślnie: **Legacy** (`RIDE_ENGINE_DEFAULT=0`)

Ten plik jest nadrzędną listą całego zadania. Statusy:

- `[x]` — kod zatwierdzony i sprawdzony buildem,
- `[~]` — rozpoczęte albo działające tylko częściowo,
- `[ ]` — jeszcze do wdrożenia,
- `TEST ROWERU` — wymaga końcowego testu sprzętowego użytkownika.

## 1. Wiążące decyzje

- [x] Zachować działający silnik Legacy jako domyślny do zakończenia prac.
- [x] Wszystkie nowe tryby kończą obliczenia na wspólnym `motor_command_t`.
- [x] Tylko `motor_core` może zapisać finalne `Iq/Id` do sterowania silnikiem.
- [x] Test roweru odbędzie się po wdrożeniu całości, nie po każdym kroku.
- [x] Canable rozwijamy na źródłach z `C:\Projekty\bafang_canable_pro` (master),
  ponieważ źródła są nowsze od binarki release v2.1.
- [x] Natywny odczyt czujnika nacisku, zakres sygnału i jego autokalibracja
  pozostają zgodne z EBICS. Nowe tryby nie mogą ustawiać własnego zera.
- [x] Punkt spoczynkowy czujnika zawsze wyznacza firmware automatycznie.
  Nie będzie ręcznego pola ani przycisku „ustaw zero”.
- [x] Wartości widoczne i edytowane przez użytkownika mają być w `kg`, nie
  w `mV` ani w surowych krokach ADC.
- [x] Skala użytkownika: brak nacisku = `0,00 kg`, pełna skala czujnika =
  `60,00 kg`.
- [x] Górny punkt kalibracji można ustawić procedurą podobną do TSDZ.
- [x] Dolny próg mapowania nacisku pozostaje regulowany, ale w `kg`.

## 2. Kontrakt czujnika nacisku

### 2.1. Co pozostaje wewnątrz firmware

- odczyt ADC i dotychczasowa natywna skala EBICS,
- automatyczne wyznaczenie zera przy uruchomieniu,
- kontrola wiarygodności zera i błąd czujnika,
- korekta dryftu podczas bezpiecznego postoju/coast,
- filtracja i dalsze obliczenia w natywnych jednostkach EBICS,
- zapis technicznego górnego punktu kalibracji.

Surowe `mV` mogą występować tylko w kodzie, protokole serwisowym i ukrytej
diagnostyce developerskiej. Nie są jednostką zwykłego interfejsu Canable.

### 2.2. Przeliczenie prezentowane użytkownikowi

```text
zero_native       = punkt wyznaczony automatycznie przez EBICS
full_scale_native = zapisany górny punkt kalibracji czujnika
delta_native      = clamp(sensor_native - zero_native, 0, span_native)
span_native       = full_scale_native - zero_native
load_kg           = delta_native * 60 kg / span_native
```

Wynik jest ograniczony do `0,00–60,00 kg`. Wartość progu podana w kg jest
przeliczana odwrotnie na natywny próg dopiero wewnątrz firmware. Dzięki temu
Power, Progressive, eMTB, Boost i Legacy nadal otrzymują sygnał zgodny z
oryginalnym sposobem obliczeń, a użytkownik pracuje wyłącznie w kg.

### 2.3. Kalibracja

- Punkt zerowy: zawsze automatyczny, bez zapisu ręcznego.
- Górny punkt: procedura „Kalibracja obciążeniem” przejmuje aktualny sygnał dla
  podanego wzorcowego obciążenia w kg i wyznacza punkt odpowiadający 60 kg.
- Dolna granica: regulowany próg rozpoczęcia mapowania nacisku, wyrażony w kg.
- Nieważne dane (pełna skala nie większa od zera, sygnał poza zakresem,
  nieudana autokalibracja) muszą zostać odrzucone przez firmware.
- Zmiana górnego punktu nie może zmieniać bieżącego automatycznego zera.

## 3. Zrobione — architektura i bezpieczeństwo

- [x] Punkt przywracania przed refaktoryzacją: tag
  `m820-before-ride-core-refactor`, commit `d6bc69c`, build `0.0136`.
- [x] Granica `motor_core` i wspólna struktura polecenia — `ea2b6cf`.
- [x] Centralizacja finalnych zapisów prądu — `3e59d0f`.
- [x] Spójny snapshot `rider_input` — `4081255`.
- [x] Osobna granica wywołania Legacy — `b031f33`.
- [x] Wydzielenie limitów napięcia, temperatury i prędkości — `998a9e7`.
- [x] Selektor Legacy / nowy Ride Core — `2c992c6`.
- [x] Wydzielenie dynamiki prądu — `553a9ca`.
- [x] Usunięcie pozostałych bezpośrednich autorów finalnego `Iq/Id` — `9e552b9`.
- [x] `ride_control` jest właścicielem polecenia silnika — `c1914cf`.
- [x] Walk ma priorytet nad zwykłym wspomaganiem w obecnym selektorze.
- [x] Natychmiastowe odcięcia bezpieczeństwa pozostają poza łagodnymi rampami.

## 4. Zrobione — tryby i charakter jazdy

- [x] Power Linear w stałoprzecinkowej ścieżce moc → prąd → `Iq` — `42650e0`,
  build `0.0148`.
- [x] Lokalna kadencja startowa nie modyfikuje `MS.cadence` ani Legacy —
  `4774de3`, build `0.0149`.
- [x] Assist without pedal rotation jako ustawienie per poziom, domyślnie OFF —
  `4774de3`.
- [x] Startup Boost TSDZ: Cadence / Speed / Auto, per poziom — `a88f2b9`,
  build `0.0150`.
- [x] Niezależny Smooth Start per poziom — `9fd2d51`, build `0.0151`.
- [x] Niezależny Release po zakończeniu pedałowania — `2be1ff9`,
  build `0.0153`.
- [x] Asymetryczny filtr wzrostu i spadku mocy — `0484123`, build `0.0154`.
- [x] Power Progressive: min/max wsparcia, moc odniesienia, progresja —
  `0f3d1c1`, build `0.0155`.
- [x] Wszystkie powyższe funkcje pozostają nieaktywne w ustawieniu domyślnym;
  Legacy nadal uruchamia się po włączeniu sterownika.
- [x] 7.1 (część): wspólny moduł `torque_input` — autozero, korekta dryftu,
  pełna skala i konwersja kg (charakterystyka fabryczna 40 mV/kg, 60 kg =
  3150 natywnie) — `c3a31e4`+`6c90cc8`+`df4f92d`, buildy `0.0156`–`0.0158`.
- [x] FW-003: wykrycie sygnału zawieszonego wysoko (>~56 kg nieprzerwanie
  ~20 s) + Error 25 zeruje CAŁĄ moc silnika (asysta i throttle) z trzymaniem
  błędu ~5 s — `ffe28b9`+`f69b8e5`, build `0.0159`. TEST ROWERU wymagany.

## 5. Zrobione — dokumentacja i przygotowanie protokołu

- [x] Dokument architektury Ride Core — `9e0351b`.
- [x] Draft źródła prawdy `protocol/ebics_config_schema.yaml` — `88c3300`.
- [x] Audyt istniejących komend firmware/HMI i bloków Para — `8e099a5`.
- [x] Zidentyfikowane obecne pola Walk, torque override i profili Legacy.
- [x] Zidentyfikowane miejsca Canable pokazujące torque w mV oraz ręczną
  kalibrację, które trzeba zastąpić interfejsem kg/autozero.

## 6. W toku

- [~] Formalne przejście publicznych pól torque z mV na kg w schemacie i
  dokumentacji.
- [~] eMTB TSDZ: rozpoczęte przygotowanie wspólnego wejścia trybów i pól profilu;
  nie ma jeszcze kompletnego wzoru, wyboru w runtime ani zatwierdzonego builda.
- [~] Audyt Canable master: repo i ekrany są rozpoznane, ale kod aplikacji nie
  został jeszcze zmieniony.

Rozpoczętego kodu eMTB nie oznaczać jako gotowy, dopóki nie przejdzie builda,
audytu wzoru i osobnego commita.

## 7. Do zrobienia — firmware, w kolejności

### 7.1. Wspólne wejście torque

- [ ] Zamknąć w jednym module konwersję: autozero → natywny sygnał EBICS → kg.
- [ ] Zachować istniejącą autokalibrację startową i bezpieczną korektę dryftu.
- [ ] Dodać górny punkt kalibracji i walidację jego zakresu.
- [ ] Dodać przeliczenie progów kg → natywna jednostka przed obliczeniami.
- [ ] Przenieść `without_rotation_threshold`, progi start/release i dolny próg
  mapowania z publicznych mV na kg.
- [ ] Zapewnić telemetrię `torque_load_kg` z rozdzielczością 0,01 kg.
- [ ] Nie udostępniać zapisu punktu zerowego.

### 7.2. eMTB

- [ ] Dokończyć wierny tryb eMTB TSDZ na przygotowanym sygnale EBICS.
- [ ] Zachować dynamiczny mianownik i opcjonalną zależność od kadencji/mocy
  zgodnie z referencją TSDZ.
- [ ] Dodać eMTB Custom z regulowaną krzywą.
- [ ] Nałożyć wspólne: Boost, filtry, limity, Smooth Start, Release i dynamikę.
- [ ] Dodać walidację przepełnień i dzielenia przez zero.
- [ ] Zbudować firmware i zatwierdzić osobnym commitem.

### 7.3. Walk Assist

- [ ] Dodać wyłącznego właściciela `CONTROL_OWNER_WALK`.
- [ ] Sterować Walk według ERPS silnika, nie według prędkości koła.
- [ ] Dodać stany: start/open-loop, Hall, blend, speed hold, stop/fault.
- [ ] Dodać łagodny regulator ERPS, rampę startu i limit prądu.
- [ ] Dodać ochronę: puszczenie przycisku, overspeed, stall i timeout.
- [ ] Zagwarantować, że Power/eMTB nie przejmą sterowania podczas Walk.
- [ ] Zachować zgodność obecnych ustawień Walk dla Legacy.

### 7.4. Konfiguracja firmware

- [ ] Zatwierdzić wersję 1 schematu YAML i wszystkie jednostki.
- [ ] Przydzielić bezkolizyjne ID po pełnym audycie komend.
- [ ] Wygenerować identyczne definicje C i JavaScript z YAML.
- [ ] Wdrożyć komendy: capabilities/schema, read saved, read runtime,
  Apply RAM, Save Flash i Revert.
- [ ] Dodać wersję, długość, CRC, walidację zakresów i migrację ustawień.
- [ ] Dodać pięć kompletnych profili ECO/TOUR/SPORT/SPORT+/BOOST.
- [ ] Zachować `Para0/Para1/Para2` jako warstwę zgodności Legacy.

## 8. Do zrobienia — Canable master

- [ ] Dodać rozpoznawanie wersji i możliwości nowego firmware EBICS.
- [ ] Dodać obsługę wersjonowanego bloku konfiguracyjnego.
- [ ] Dodać zakładki pięciu profili oraz Kopiuj/Wklej/Porównaj.
- [ ] Dodać grupy: Charakterystyka, Start i Boost, Dynamika i Release, Limity,
  Prędkość oraz osobną zakładkę Walk.
- [ ] Zastąpić `Torque Sensor Value ... mV` wartością `Nacisk ... kg`.
- [ ] Zastąpić `Torque Override Threshold (mV)` dolnym progiem w kg.
- [ ] Zmienić logger i wykres torque z mV na kg.
- [ ] Usunąć możliwość ręcznego ustawienia zera czujnika.
- [ ] Dodać procedurę kalibracji górnego punktu z obciążeniem wzorcowym w kg.
- [ ] Pozostawić raw ADC/mV tylko w ukrytej diagnostyce developerskiej.
- [ ] Dodać wykresy: moc, boost/kadencja, start/release i taper prędkości.
- [ ] Dodać `Sync`, `Apply RAM`, `Save Flash`, `Revert` i czytelne porównanie
  konfiguracji zapisanej z edytowaną.
- [ ] Nie opierać wdrożenia na starszej binarce release v2.1.

## 9. Publiczne zmienne torque do dodania w Canable

Nazwy są kontraktem funkcjonalnym; dokładne `wire_id` przydzieli generator.

| Klucz | Widok użytkownika | Jednostka | Dostęp |
|---|---|---:|---|
| `torque_load_kg` | Aktualny nacisk | kg, 0,01 | tylko odczyt |
| `torque_full_scale_kg` | Pełna skala | stałe 60,00 kg | tylko odczyt |
| `torque_calibration_reference_kg` | Obciążenie wzorcowe kalibracji | kg, 0,01 | parametr operacji kalibracji |
| `torque_lower_threshold_kg` | Dolna granica mapowania nacisku | kg, 0,01 | zapis; per poziom, zgodnie z dawnym `TQO_threshold` |
| `torque_gate_start_kg` | Próg załączenia nacisku | kg, 0,01 | zapis globalny |
| `torque_gate_release_kg` | Próg zwolnienia nacisku | kg, 0,01 | zapis globalny |
| `without_rotation_threshold_kg` | Próg pomocy bez obrotu | kg, 0,01 | zapis per poziom |
| `startup_boost_auto_threshold_kg` | Próg Auto Boost | kg, 0,01 | zapis per poziom |

Zmienne techniczne niewidoczne w zwykłym UI:

| Klucz | Znaczenie | Dostęp |
|---|---|---|
| `torque_zero_native` | bieżący automatyczny punkt zerowy | tylko diagnostyka |
| `torque_full_scale_native` | zapisany górny punkt sensora | firmware/operacja kalibracji |
| `torque_raw_native` | surowy odczyt | tylko diagnostyka developerska |

Nie dodawać `torque_zero_kg` jako ustawienia: w skali użytkownika zero zawsze
wynosi `0,00 kg`, a jego punkt techniczny zawsze wyznacza autokalibracja.

## 10. Pozostałe grupy zmiennych Canable

- Profile: `mode_type`, wsparcie linearne/min/max, moc odniesienia, progresja,
  parametr eMTB, limit mocy i limit `Iq`.
- Start: Assist without rotation, Startup Boost mode/strength/end, Smooth Start.
- Dynamika: rise/fall, Release, filtr wzrostu i spadku mocy.
- Prędkość: początek/koniec taperu i kształt krzywej.
- Walk: target ERPS, prąd bazowy/startowy, regulator, pasma, rampy i timeout.
- System: wersja schematu, capabilities, CRC i status zapisanej konfiguracji.

Pełne typy i zakresy pozostają w `protocol/ebics_config_schema.yaml`.

## 11. Końcowa weryfikacja przed przekazaniem do testu

- [ ] Czysty build M820/BL820 i zapis SHA-256 binarki.
- [ ] Audyt: finalne `Iq/Id` zapisuje tylko `motor_core`.
- [ ] Audyt: nowe tryby czytają wyłącznie spójny `rider_input`.
- [ ] Audyt: zero torque nie jest nigdzie ręcznie zapisywane.
- [ ] Audyt: zwykłe UI i logger pokazują nacisk wyłącznie w kg.
- [ ] Testy automatyczne konwersji: 0 kg, próg dolny, obciążenie pośrednie,
  60 kg, przepełnienie i błędna kalibracja.
- [ ] Testy protokołu: read/apply/save/revert, CRC, zła wersja i zły zakres.
- [ ] Przygotowanie dwóch binarek: bezpieczne Legacy oraz kandydat Ride Core.
- [ ] Przygotowanie instrukcji powrotu do taga/binarnego punktu bazowego.

## 12. Test sprzętowy użytkownika — po wdrożeniu całości

- [ ] Start bez nacisku i z lekkim/mocnym naciskiem.
- [ ] Sprawdzenie wskazania 0 kg bez nacisku po każdym uruchomieniu.
- [ ] Sprawdzenie obciążenia wzorcowego i pełnej skali 60 kg.
- [ ] Start na ciężkim biegu i podjazd bez oscylacji.
- [ ] Kadencja niska 20–30 RPM oraz wysoka 80–120 RPM.
- [ ] Brak przerw pomiędzy nogami i kontrolowany Release po zatrzymaniu.
- [ ] Natychmiastowe odcięcie przy kręceniu do tyłu.
- [ ] Power Linear, Progressive, eMTB TSDZ i eMTB Custom na każdym poziomie.
- [ ] Assist without rotation tylko po świadomym włączeniu.
- [ ] Walk: płasko, pod górę, koło w powietrzu, stall i puszczenie przycisku.
- [ ] Zapis ustawień, restart kontrolera, ponowny odczyt i Revert.
- [ ] Dłuższa jazda kontrolująca dryft zera, temperaturę i limity.

Ride Core jest ukończony dopiero po zakończeniu punktów 7–11. Punkt 12 jest
końcową akceptacją sprzętową użytkownika.
