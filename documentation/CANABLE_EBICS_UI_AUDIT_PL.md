# Audyt Canable — osobne karty i wykresy EBICS

Data audytu: 2026-07-17

Zakres:

- aplikacja: `C:\Projekty\bafang_canable_pro`, master,
- firmware: `C:\Projekty\EBICS\BAFANG_GD32F303RCT6`,
- cel: oddzielić standardowe ekrany Bafang od EBICS Legacy i EBICS Ride Core,
- ten etap jest wyłącznie audytem — kod aplikacji Canable nie został zmieniony.

## 1. Wniosek

Obecny interfejs nie powinien być dalej rozwijany przez dopisywanie niebieskich
opisów pod czarnymi opisami Bafang. W `ui/index.html` znajduje się 46 niebieskich
napisów; po odjęciu napisu w logo pozostaje 45 etykiet EBICS dołączonych do
elementów Bafang. Część tych par opisuje ten sam parametr, część zupełnie inne
znaczenie tego samego bajtu, a część napisów jest już nieaktualna.

Docelowo muszą istnieć trzy oddzielne przestrzenie:

1. **Bafang Standard** — obecne standardowe kontrolery i ich znaczenie bloków.
2. **EBICS Legacy** — stare znaczenie bajtów `6010/6011/6012` i `0x62D9`.
3. **EBICS Ride Core** — banki profili, nowe tryby, torque w kg, dynamika,
   limity i później Walk ERPS.

Nie wolno budować ekranu EBICS Ride Core na standardowym modelu danych Bafang.

## 2. Obecny układ aplikacji

Obecne karty główne:

```text
Controller | Display | Sensor | Battery | Assist (Full) | Banks |
Assist (Ltd) | Info | Debug | Firmware | Sniffer | Ride Logger | Data Backup
```

Problemy:

- `Controller` zawiera równocześnie podpisy Bafang i EBICS.
- `Assist (Full)` używa standardowych struktur P0/P1/P2, ale pokazuje pod nimi
  drugie znaczenie EBICS.
- `Assist (Ltd)` pozostaje ekranem standardowym M820.
- `Banks` jest pierwszą kartą rzeczywiście specyficzną dla Ride Core, ale ma
  jedną szeroką tabelę i pokazuje również parametry nieaktywne dla danego trybu.
- Nie ma wykrywania EBICS ani negocjacji capabilities. Karta `Banks` jest
  widoczna również dla zwykłego kontrolera Bafang.
- Wszystkie kontrolki są na końcu `enableAppControls()` ponownie włączane przez
  pozostawioną linię testową, niezależnie od stanu połączenia.

## 3. Najważniejsze ryzyka

| Priorytet | Problem | Skutek |
|---|---|---|
| Krytyczny | Brak wykrywania Bafang/EBICS | Użytkownik może otworzyć i zapisać niewłaściwy model ustawień |
| Krytyczny | `Assist (Full)` przy zapisie wysyła całe P0, P1 i P2 | Zmiana jednego pola może nadpisać inne bajty o znaczeniu EBICS |
| Krytyczny | Standardowy i EBICS parser współdzielą te same nazwy stanu | UI nie wie, czy `assist_ratio` jest procentem Bafang, czy progiem torque EBICS |
| Wysoki | Torque kg jest liczony na sztywno jako `750 mV + 40 mV/kg` | Canable nie korzysta z zapisanego górnego punktu kalibracji firmware |
| Wysoki | Przycisk `Calibrate Torque Sensor` wysyła ręczną komendę | Jest sprzeczny z zasadą automatycznego zera EBICS |
| Wysoki | Wykres Assist zakłada `motor = human_power × assist_ratio` | Jest błędny dla Torque, eMTB i części konfiguracji Progressive |
| Wysoki | Wykres Start Ramp interpretuje `Acceleration 1–8` | Nie odpowiada rampom EBICS podanym w ms i zależnym od prędkości/kadencji |
| Średni | `Banks` pokazuje wszystkie pola we wszystkich trybach | Użytkownik nie wie, które ustawienie faktycznie działa |
| Średni | Offline defaults wyglądają jak ustawienia kontrolera | Można rozpocząć edycję danych, których jeszcze nie odczytano |
| Średni | Brak Saved/Runtime/Edited i Revert | Nie wiadomo, co jest w RAM, co w flash, a co tylko w formularzu |

## 4. Co obecnie jest podwójnie opisane

### 4.1. `Controller` / blok P1

Poniższe znaczenia EBICS są potwierdzone przez `src/parser.c` i powinny zniknąć
ze standardowej karty `Controller`.

| Bajt P1 | Obecne znaczenie EBICS | Docelowa karta EBICS |
|---:|---|---|
| 0 | napięcie systemowe | Limity i system |
| 1 | maksymalny prąd baterii | Limity i system |
| 2 | próg nadnapięcia | Limity i system |
| 3–4 | próg podnapięcia pod obciążeniem | Limity i system |
| 7–8 | pojemność baterii | Limity i system / bateria |
| 9 | maksymalny prąd fazowy silnika | Limity i system |
| 10–11 | progi Limp SoC | Limity i system |
| 12 | `Cadence_exponent` Legacy | EBICS Legacy / Zaawansowane |
| 14 | Legal Flag | Limity i prędkość |
| 18 | kierunek silnika | Serwis silnika |
| 19 | przełożenie mechaniczne | Serwis silnika |
| 20 | impulsy czujnika prędkości na obrót | Limity i prędkość |
| 21 | `decay_base` Legacy | EBICS Legacy / Zaawansowane |
| 24–25 | `MagicNumber` | ukryty Serwis |
| 34–35 | początek/koniec manetki | EBICS Legacy / Manetka |
| 36 | prąd Walk w procentach | Walk Assist |
| 37 | czas Extended Boost Legacy | EBICS Legacy |
| 38 | timeout PAS Legacy | EBICS Legacy / PAS |
| 39 | `ramp_end` Legacy | EBICS Legacy / PAS |
| 41/43/45/47/48 | limit prądu poziomu 1–5 | EBICS Legacy / Poziomy |
| 50/52/54/56/57 | limit prędkości poziomu 1–5 | EBICS Legacy / Poziomy |
| 60–61 | prędkość Walk Legacy | Walk Assist |

Nie przenosić automatycznie niebieskich etykiet `not used yet`. Takie bajty
powinny zniknąć ze zwykłego UI, a pozostać jedynie w edytorze raw/Debug.

Napis `Throttle Exponent * 100` przypisany obecnie do standardowego pola
`Cadence Sensor Signals/Rotation` nie ma potwierdzenia w aktualnym
`src/parser.c`. Należy go uznać za etykietę nieaktualną i nie przenosić.

### 4.2. `Assist (Full)` / bloki P0, P1 i P2

| Obecny element standardowy | Znaczenie EBICS Legacy | Docelowo |
|---|---|---|
| P0 `Acceleration` poziomu | `TQfilter` poziomu | EBICS Legacy / Torque filter |
| P0 `Assist Ratio` | `TQO_threshold` poziomu | EBICS Legacy / dolny próg w kg |
| P1 `Battery Current Limit` | limit prądu silnika poziomu | EBICS Legacy / Poziomy |
| P1 `Speed Limit` | limit prędkości poziomu | EBICS Legacy / Poziomy |
| P2 standardowa macierz torque | `assist_profile[5][6]` | EBICS Legacy / Mapa poziomów |
| P2 `Start Pulse` | Extended Boost Strength | EBICS Legacy / Extended Boost |
| P2 `Torque Attenuation` | Extended Boost Duration | EBICS Legacy / Extended Boost |
| `Startup Angle` 0x62D9 | `TS_coeff` / multiplier | EBICS Legacy / Zaawansowane |

Extended Boost jest obecnie globalnie wyłączony w firmware. Jego pola powinny
być oznaczone jako nieaktywne albo całkowicie ukryte, a nie pokazywane jako
normalne ustawienia Ride Core.

### 4.3. `Banks` / EBICS Ride Core

Ta karta korzysta już z właściwych bloków EBICS:

| Operacja | Komenda | Stan |
|---|---:|---|
| odczyt banku | `0x6020` | dostępne |
| zastosowanie banku w RAM | `0x6021` | dostępne |
| zapis banków i tuningu w flash | `0x6022` | dostępne, wykonywane na postoju |
| odczyt tuningu | `0x6023` | dostępne |
| zastosowanie tuningu w RAM | `0x6024` | dostępne |

Do przeniesienia z obecnej karty:

- dwa banki i pięć poziomów,
- wybór Power Linear / Power Progressive / eMTB / Torque,
- Support, parametry Progressive, eMTB i Torque,
- limity mocy oraz `Iq`,
- Assist without rotation,
- Startup Boost, Smooth Start, Release,
- filtry wzrostu/opadania mocy,
- globalne rampy i cadence step.

Nie zachowywać obecnej jednej tabeli ze wszystkimi kolumnami. Pola mają być
pokazywane zależnie od wybranego trybu.

## 5. Docelowa nawigacja

Rekomendowany układ:

```text
Bafang Standard
├── Controller
├── Assist Full
└── Assist Limited

EBICS
├── Live
├── Profile jazdy
├── Czujnik nacisku
├── Start i dynamika
├── Limity i prędkość
├── Walk Assist
├── Legacy
└── System i zapis

Wspólne
├── Display
├── Battery
├── Info
├── Firmware
├── Sniffer
├── Ride Logger
└── Data Backup
```

Karty EBICS powinny być widoczne dopiero po potwierdzeniu obsługi EBICS.
`Debug` może pozostać wspólny, ale powinien być oznaczony jako developerski.

## 6. Projekt kart EBICS

### 6.1. EBICS Live

Karty informacyjne:

- aktywny bank, poziom i tryb,
- aktualny nacisk w kg,
- kadencja, prędkość i ERPS,
- moc człowieka i moc silnika,
- napięcie, prąd baterii i temperatura,
- właściciel sterowania: Ride / Walk / Safety,
- aktywny limiter: poziom, moc, prąd, napięcie, temperatura albo prędkość,
- status autozero i kalibracji pełnej skali torque.

Grafiki:

1. Mały wykres czasu: nacisk, moc człowieka, moc silnika.
2. Punkt bieżącej pracy na charakterystyce aktywnego profilu.
3. Pasek wykorzystania limitów: moc, prąd baterii, prąd fazowy, temperatura.

### 6.2. EBICS Profile jazdy

Układ:

- wybór banku 1/2,
- zakładki ECO / TOUR / SPORT / SPORT+ / BOOST,
- wybór trybu dla profilu,
- Kopiuj / Wklej / Porównaj / Przywróć edytowany poziom,
- wspólne limity poziomu w oddzielnej sekcji.

Pola zależne od trybu:

| Tryb | Pokazywane pola główne |
|---|---|
| Power Linear | Support (%) |
| Power Progressive | Support min/max, moc odniesienia, progresja |
| Torque | Torque gain, z opisem `120 = 1,0×` |
| eMTB | parametr eMTB, zależność od mocy/kadencji |
| eMTB Custom | własne punkty krzywej, gdy firmware je udostępni |

Nieaktywne pola mają być ukryte, a nie tylko wyszarzone w wielkiej tabeli.

Wykresy:

1. Power: moc człowieka → moc silnika dla pięciu poziomów.
2. Torque: nacisk 0–60 kg → żądana moc/prąd silnika.
3. eMTB: nacisk → moc dla kilku kadencji, np. 30/60/90 RPM.
4. Porównanie poziomów i porównanie Saved/Edited.

Obecny wykres `Assist Curves` można zostawić dla Bafang. Dla EBICS trzeba
wykonać osobny kalkulator używający tych samych wzorów i ograniczeń co firmware.

### 6.3. EBICS Czujnik nacisku

Karty:

- duży wskaźnik `0,00–60,00 kg`,
- status automatycznego zera: OK / trwa / błąd,
- punkt zerowy tylko jako stan diagnostyczny, bez pola zapisu,
- status zapisanej pełnej skali,
- kalibracja górnego punktu obciążeniem wzorcowym w kg,
- dolny próg mapowania per poziom,
- progi start/release i próg Assist without rotation w kg,
- błędy sygnału i ostatnia poprawna kalibracja.

Grafiki:

1. Charakterystyka czujnika: 0–60 kg z zaznaczonym bieżącym naciskiem.
2. Linie progów ECO–BOOST na jednej skali kg.
3. Podgląd działania dolnego progu i nasycenia pełnej skali.

Obecne `torqueMvToKg()` nie może być źródłem prawdy. UI musi odczytać
kalibrację z firmware albo otrzymywać gotowe `torque_load_centikg`.

Przycisk ręcznej kalibracji z karty `Controller` należy dla EBICS usunąć.
Może zostać w widoku standardowego kontrolera Bafang.

### 6.4. EBICS Start i dynamika

Sekcje:

- Assist without rotation,
- Startup Boost: enable, mode, strength, cadence end/step,
- Smooth Start,
- Acceleration Low i High,
- Deceleration Low i High,
- Release,
- filtr wzrostu i spadku mocy,
- zaawansowane progi prędkości/kadencji ramp, gdy będą w protokole.

Wykresy:

1. Boost (%) względem kadencji.
2. Start w czasie: Boost + Smooth Start + końcowa rampa `Iq`.
3. Release i opadanie mocy w czasie.
4. Mapa czasu rampy względem prędkości i kadencji.

Obecny wykres `Start Ramp` należy pozostawić tylko dla Bafang. Jego przeliczenie
`Acceleration 1–8` nie odpowiada implementacji EBICS w milisekundach.

### 6.5. EBICS Limity i prędkość

Pola:

- napięcie systemowe,
- prąd baterii i prąd fazowy,
- limity mocy i `Iq` poziomów,
- nadnapięcie i podnapięcie,
- Limp SoC,
- limity temperatury,
- prędkość, obwód koła i impulsy czujnika,
- Legal Flag,
- taper start/end/shape po udostępnieniu w protokole.

Wykresy:

1. Dostępna moc silnika względem prędkości.
2. Taper prędkości.
3. Podział aktywnych limitów — który limit obcina żądanie jako pierwszy.

### 6.6. EBICS Walk Assist

Do czasu ukończenia Walk ERPS karta pokazuje wyraźnie tryb `Legacy`:

- Walk current (%),
- Walk speed (km/h),
- stan przycisku i aktywność Walk.

Po wdrożeniu nowego Walk:

- target ERPS,
- prąd startowy/bazowy,
- strefa startu, blend i speed hold,
- regulatory, deadband, overspeed, stall i timeout.

Wykres docelowy: ERPS oraz żądany prąd względem czasu od naciśnięcia przycisku,
z zaznaczonymi przejściami stanów Walk.

### 6.7. EBICS Legacy

Ta karta jest potrzebna dla porównań i zgodności, ale nie powinna wyglądać jak
nowy Ride Core. Zawiera:

- `TS_coeff`, `Cadence_exponent`, `TQfilter`, `TQO_threshold`,
- stare limity poziomów,
- `assist_profile[5][6]`,
- PAS timeout/ramp end/decay,
- manetkę,
- Extended Boost jako `wyłączony w firmware`,
- techniczne pola Para tylko w trybie Advanced.

### 6.8. EBICS System i zapis

Jedno miejsce dla operacji:

- Sync,
- Apply RAM,
- Save Flash,
- Revert Edited,
- ponowny odczyt Runtime,
- odczyt Saved,
- wersja firmware, schematu i capabilities,
- aktywny bank,
- CRC i status ostatniego zapisu,
- eksport/import profili EBICS.

Każdy ekran EBICS powinien pokazywać trzy stany wartości:

```text
Saved Flash | Runtime RAM | Edited UI
```

## 7. Wykrywanie EBICS

Przed pokazaniem kart EBICS aplikacja powinna ustalić typ kontrolera:

```text
UNKNOWN → próba capabilities/signature → BAFANG_STANDARD albo EBICS
```

Minimalnie odpowiedź na `0x6020` z magic `EB` i poprawnym CRC potwierdza obecną
wersję Ride Core. Docelowo firmware powinien mieć osobną komendę capabilities,
zwracającą:

- identyfikator EBICS,
- wersję firmware,
- wersję schematu banku i tuningu,
- obsługiwane tryby,
- obsługiwane operacje RAM/Flash/Revert,
- dostępność torque calibration, Walk ERPS i telemetrii rozszerzonej.

Brak odpowiedzi nie może automatycznie oznaczać EBICS Legacy. W razie braku
sygnatury aplikacja pozostaje w bezpiecznym widoku Bafang/Unknown.

## 8. Dane dostępne i brakujące

| Dane | Stan obecny | Potrzeba |
|---|---|---|
| Banki profili | dostępne `0x6020/21` | przenieść do nowej karty Profile |
| Tuning ramp | dostępny `0x6023/24` | przenieść do Start i dynamika |
| Zapis flash | dostępny `0x6022` | pokazać stan oczekiwania na postój i wynik |
| Torque/cadence/speed/current/voltage/temp | dostępna podstawowa telemetria | użyć w Live i Logger |
| Torque w kg | firmware liczy `torque_load_centikg`, ale UI go nie odbiera | dodać pole telemetrii/protokołu |
| Autozero i calibration fault | istnieje w firmware | dodać read-only status protokołu |
| Górny punkt torque | istnieje `MP.torque_full_scale_native` | dodać bezpieczny odczyt i operację kalibracji |
| Aktywny tryb/owner/limiter | brak w Canable | dodać rozszerzoną telemetrię EBICS |
| Saved kontra Runtime | brak rozdzielenia | dodać oddzielne operacje/status |
| Revert RAM | brak | dodać do protokołu albo jasno opisać ograniczenie |
| Walk ERPS | jeszcze niewdrożony | karta gotowa etapami, pola ukryte do capabilities |

## 9. Zalecany podział kodu podczas późniejszego wdrożenia

To jest lista przyszłych prac, nie zmiany wykonane w tym audycie.

```text
ui/js/ebics/
├── ebics-state.js
├── ebics-capabilities.js
├── ebics-live.js
├── ebics-profiles.js
├── ebics-torque.js
├── ebics-dynamics.js
├── ebics-limits.js
├── ebics-walk.js
├── ebics-legacy.js
├── ebics-system.js
└── ebics-charts.js
```

Zasady:

- `state.ebics` oddzielone od `lastControllerP0/P1/P2`,
- osobne parsery standard Bafang i EBICS,
- wspólne mogą pozostać tylko transport CAN, połączenie i bezpieczne komponenty,
- jeden kalkulator charakterystyk EBICS używany przez wszystkie wykresy,
- żadnych stałych konwersji torque w UI bez danych z firmware,
- brak zapisu całego bloku, jeśli użytkownik niczego nie zmienił,
- zapis tylko po odczycie aktualnej wersji i walidacji CRC/schematu.

## 10. Kolejność wdrożenia

### Etap A — bezpieczeństwo i rozdzielenie

1. Dodać wykrywanie EBICS/capabilities.
2. Utworzyć osobny namespace stanu i osobny model widoku EBICS.
3. Usunąć podwójne etykiety z ekranów Bafang.
4. Zablokować standardowe zapisy P0/P1/P2 w trybie Ride Core.
5. Usunąć testowe bezwarunkowe włączanie kontrolek.

### Etap B — podstawowe karty EBICS

1. System i zapis.
2. Live.
3. Profile jazdy z polami zależnymi od trybu.
4. Legacy jako osobna karta zgodności.

### Etap C — torque i wykresy

1. Dodać protokół torque status/calibration.
2. Czujnik nacisku z autozero i 0–60 kg.
3. Osobne wykresy Power, Torque i eMTB.
4. Start, Boost, rampy i Release.

### Etap D — limity i Walk

1. Limity/prędkość i wykres taperu.
2. Walk Legacy.
3. Walk ERPS po ukończeniu firmware.

## 11. Kryteria odbioru

- Na ekranach Bafang nie ma żadnych podwójnych czarno-niebieskich etykiet.
- Karty EBICS są widoczne wyłącznie po potwierdzeniu EBICS.
- EBICS Legacy i Ride Core są osobnymi kartami.
- Wybrany tryb pokazuje tylko parametry, których rzeczywiście używa firmware.
- Torque użytkownika jest wszędzie w kg, a zero nie ma ręcznego pola zapisu.
- Wykresy EBICS korzystają z wzorów Ride Core, nie ze wzorów standard Bafang.
- `Apply RAM`, `Save Flash` i `Revert` mają rozróżnione znaczenie.
- Offline defaults są wyraźnie oznaczone jako podgląd i nie udają odczytu.
- Nie można zapisać danych z niezgodną wersją schematu lub błędnym CRC.
- Ride Logger używa kalibracji torque kontrolera, nie stałej zaszytej w UI.

## 12. Podsumowanie zakresu

Do wydzielenia jest 45 obecnych etykiet EBICS, jedna karta `Banks`, pięć pól
tuningu, stare bloki Legacy oraz przyszłe dane torque/Walk. Docelowy układ to
osiem kart EBICS, z czego sześć jest użytkowych, jedna obsługuje Legacy, a jedna
odpowiada za zapis i diagnostykę systemową.

W tym audycie nie zmieniono żadnego pliku w repozytorium Canable.
