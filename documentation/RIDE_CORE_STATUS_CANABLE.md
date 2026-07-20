# EBICS Ride Core — status wdrożenia i plan Canable

Aktualizacja: 2026-07-16

Gałąź: `refactor/ride-core`

Punkt przywracania: `m820-before-ride-core-refactor` (`d6bc69c`)

Ostatni sprawdzony build: `0.0155` M820/BL820

Nadrzędna, aktualna lista całego zadania znajduje się w
`documentation/RIDE_CORE_MASTER_CHECKLIST_PL.md`. Wiążąca decyzja dla torque:
autozero EBICS pozostaje automatyczne, pełna skala użytkownika wynosi 60 kg,
a zwykły interfejs Canable nie pokazuje ani nie przyjmuje mV.

Ten dokument rozwija techniczny status Ride Core i konfiguratora. Rozróżnia:

- **WDROŻONE** — aktywne w nowej architekturze,
- **LEGACY** — działa, ale nadal należy do starego algorytmu,
- **SZKIELET** — interfejs istnieje, funkcja docelowa jeszcze nie,
- **NIE WDROŻONE** — brak kodu wykonawczego,
- **TEST ROWERU** — kod i build są gotowe, ale zachowanie nie zostało jeszcze potwierdzone sprzętowo.

## 1. Aktualna ścieżka sterowania

```text
czujniki i dekoder PAS
        ↓
rider_input (snapshot sygnałów)
        ↓
ride_control (Legacy domyślnie; Power Linear dostępny developersko)
        ↓
legacy_assist albo assist_modes
        ↓
assist_limits
        ↓
assist_dynamics
        ↓
motor_command_t
        ↓
motor_core
        ↓
FOC / PWM
```

Ważne: `rider_input` publikuje już spójny snapshot, ale obliczenia Legacy nadal
czytają część pól bezpośrednio z `MS` i globalnych liczników. Nowe tryby mają
korzystać wyłącznie z `rider_input_t`.

## 2. Co zostało wdrożone

| Element | Status | Stan rzeczywisty |
|---|---|---|
| Punkt bazowy i tag przywracania | WDROŻONE | Tag `m820-before-ride-core-refactor`; bazowy build `0.0136` |
| `motor_command_t` | WDROŻONE | Wspólny interfejs `Iq`, `Id`, enable i emergency stop |
| Jeden zapis finalnego `Iq/Id` | WDROŻONE | Bezpośrednio zapisuje je wyłącznie `src/motor_core.c` |
| `rider_input` | WDROŻONE | Snapshot momentu, kadencji, kierunku PAS, prędkości koła i ERPS |
| `legacy_assist` | WDROŻONE/SZKIELET | Jest osobny punkt wejścia; ciało starego algorytmu nadal pozostaje monolitem |
| `assist_limits` | WDROŻONE | Kolejność Legacy: napięcie → temperatura → prawny taper prędkości |
| `assist_dynamics` | WDROŻONE | Adaptacyjne rampy `Iq`, profilowy Release, szybka ścieżka WA i natychmiastowe cięcia bezpieczeństwa |
| `assist_modes` | WDROŻONE/SZKIELET | Power Linear i Progressive działają; eMTB jest następnym trybem |
| `assist_start` | WDROŻONE | Startup Boost, niezależna obwiednia Smooth Start i Release działają |
| `ride_control` | WDROŻONE | Wybiera Legacy/TSDZ, zachowuje priorytet Walk, stosuje limity i dynamikę, wysyła `motor_command_t` |
| `protocol/ebics_config_schema.yaml` | SZKIELET | Draft v0 opisuje pola, typy, skale, zakresy i operacje; numery `wire_id` celowo nieprzydzielone |
| Audyt komend HMI/CAN | CZĘŚCIOWO | Firmware, lokalny log i źródła Canable master wstępnie sprawdzone; brakuje pełnych logów HMI/BESST |
| Test sprzętowy | TEST ROWERU | Decyzją użytkownika przeniesiony na koniec całego wdrożenia |

## 3. Co działa obecnie tylko w Legacy

Poniższe funkcje są aktywne w firmware, ale nie są jeszcze docelowymi modułami
Ride Core:

- dotychczasowe obliczanie wspomagania pedałowania,
- naciskowy startup boost w stylu TSDZ2,
- latch startu: nacisk + kolejne kroki PAS do przodu,
- istniejący Walk Assist oparty na prędkości koła `Speedx100`,
- throttle override,
- wyłączony kompilacyjnie Extended Boost,
- kalibracja kąta Halla znajdująca się w ścieżce obliczania żądania,
- zapisy i odczyty konfiguracji przez `Para0/Para1/Para2`.

Te funkcje pozostają do porównań. Nie należy dopisywać do nich nowych trybów.

## 4. Czego jeszcze nie wdrożono

| Funkcja | Status | Warunek rozpoczęcia/ukończenia |
|---|---|---|
| Power Linear TSDZ2 | WDROŻONE/DEV | Build `0.0155`; stałoprzecinkowe Power → prąd → Iq, domyślnie nadal Legacy |
| Lokalna `cadence_for_assist` | WDROŻONE/DEV | Kadencja syntetyczna istnieje tylko wewnątrz `assist_modes` i nie zmienia snapshotu ani Legacy |
| Assist without pedal rotation | WDROŻONE/DEV | Per-level, domyślnie OFF; obecny próg natywny zostanie wystawiony użytkownikowi w kg |
| Startup Boost jako ustawienie per-level | WDROŻONE/DEV | Osobny `assist_start`, tryby Cadence/Speed/Auto, krzywa przed obliczeniem Power |
| Smooth Start per-level | WDROŻONE/DEV | Obwiednia 0–100% po limitach i przed wspólną rampą; domyślnie OFF, 300 ms |
| Release niezależny od startu | WDROŻONE/DEV | Czas zejścia pełnej skali Iq po zaniku pedałowania; 0 ms zachowuje adaptacyjną rampę |
| Asymetryczny filtr mocy | WDROŻONE/DEV | Osobny czas narastania/opadania podczas aktywnego PAS; 0 ms = bypass |
| Power Progressive | WDROŻONE/DEV | Min/max wsparcia, moc odniesienia i mieszanie liniowe–kwadratowe 0–100% |
| eMTB TSDZ / Custom Curve | NIE WDROŻONE | Po progresywnym Power |
| Osobny właściciel Walk Assist | NIE WDROŻONE | `CONTROL_OWNER_WALK` + sterowanie według ERPS |
| Stany WA open-loop/Hall/blend/speed hold | NIE WDROŻONE | Po stabilnym pomiarze ERPS |
| Pięć pełnych profili poziomów | NIE WDROŻONE | Potrzebny nowy schemat protokołu |
| Kopiowanie ustawień poziomów | NIE WDROŻONE | Firmware schema + UI |
| Wykresy charakterystyk | NIE WDROŻONE | Po ustaleniu wzorów i jednostek |
| `Sync / Apply RAM / Save Flash / Revert` | NIE WDROŻONE | Nowy wersjonowany protokół konfiguracji |
| Jedno źródło prawdy YAML | SZKIELET | Draft v0 utworzony; wymaga audytu ID, domyślnych profili i generatora |

### Power Linear — stan builda 0.0155

Nowy moduł `assist_modes`:

1. liczy moc człowieka z filtrowanego sygnału momentu i kadencji,
2. mnoży ją przez współczynnik wsparcia poziomu,
3. ogranicza żądaną moc do 1500 W,
4. przelicza moc na prąd przez aktualne napięcie baterii,
5. przelicza prąd na natywne jednostki `Iq` przez `CAL_I`,
6. nakłada limit poziomu, napięcia, temperatury i legalnej prędkości,
7. przekazuje wynik przez wspólną dynamikę i `motor_core`.

Referencyjne współczynniki pięciu poziomów to obecnie
`100/200/320/420/520%`. Cztery skrajne wartości odpowiadają faktorom
TSDZ2 `50/100/160/260`; SPORT+ jest punktem pośrednim. Są to wartości
developerskie, jeszcze nie zapis profilu Canable.

Obliczenia mocy zachowują teraz precyzję miliwatów aż do przeliczenia `P/U`.
Opcjonalne `assist_without_rotation` podstawia lokalnie `cadence_for_assist=1`
po przekroczeniu progu skorygowanego momentu ponad punkt zerowy 750 mV.
Nie zapisuje tej wartości do `MS.cadence` ani `rider_input`. Funkcja pozostaje
wyłączona w każdym profilu domyślnym, ponieważ rower nie ma czujnika hamulca.

`RIDE_ENGINE_DEFAULT=0` utrzymuje Legacy. Wartość `1` włącza Power Linear
do testu developerskiego. Walk Assist zachowuje wyłączny priorytet i do czasu
wydzielenia nowego modułu korzysta ze sprawdzonej ścieżki Legacy.

### Startup Boost TSDZ — stan builda 0.0155

`assist_start` modyfikuje lokalny sygnał momentu przed obliczeniem mocy. Krzywa
ma 120 wpisów i używa stałoprzecinkowej rekurencji z referencji TSDZ2:

```text
factor[0] = startup_boost_strength_pct
factor[n] = factor[n-1] * (256 - 20) / 256
torque_for_assist *= 1 + factor[cadence] / 100
```

Obsługiwane są tryby `CADENCE`, `SPEED` i `AUTO`. `SPEED` uzbraja się na
postoju i rozbraja po przekroczeniu kadencji końca; `AUTO` wyłącza boost przy
małym nacisku podczas jazdy. Wynik jest ograniczony do fizycznego zakresu
sygnału 2550 mV ponad zero, a siła do 300%.

Developerskie profile Power używają ustawień odziedziczonych z działającego
EBICS: `enabled=true`, `CADENCE`, `strength=200%`, `end=45 RPM`. Poziom OFF ma
boost wyłączony. Stary boost Legacy pozostaje osobną ścieżką i mechanizmy nie
mogą działać jednocześnie, ponieważ `ride_control` wybiera tylko jeden silnik.

### Smooth Start — stan builda 0.0153

Smooth Start jest niezależny od siły Startup Boost. Gdy zmierzona kadencja i
ERPS silnika są równe zero, moduł uzbraja pojedynczą obwiednię startową.
Po pojawieniu się żądania `Iq` obwiednia rośnie liniowo od 0 do 100% przez
`smooth_start_ms`, a potem się rozbraja aż do następnego rzeczywistego postoju.

Obwiednia działa po limitach Power i przed wspólną adaptacyjną rampą `Iq`.
Cięcie bezpieczeństwa nadal zeruje wynik natychmiast. Implementacja nie zmienia
kadencji, ERPS, FOC ani PWM i nie blokuje startu z lokalnym
`cadence_for_assist=1`. Domyślne profile mają `enabled=false`, `300 ms`, więc
zachowanie builda pozostaje takie jak przed dodaniem funkcji.

### Release — stan builda 0.0153

`release_ms` nie podtrzymuje ostatniego żądania. Po rzeczywistym zaniku
pedałowania tryb ustawia target na zero, a wspólny `assist_dynamics` schodzi
do zera stałym slew wyliczonym jako pełna skala `Iq / release_ms`.

Release nie jest wybierany dla spadków momentu między nogami, gdy PAS nadal
jest aktywny, ani dla lokalnego startu bez obrotu. W tych przypadkach działa
zwykła adaptacyjna dynamika. `safety_cut` nadal zeruje `Iq` natychmiast.
Wartość domyślna `0` oznacza „użyj istniejącej rampy adaptacyjnej”, dzięki
czemu samo dodanie pola nie zmienia charakteru jazdy.

### Filtr wzrostu i spadku mocy — stan builda 0.0154

Power ma osobny filtr pierwszego rzędu dla żądanej mocy silnika. Czas wzrostu
wybiera `power_rise_filter_ms`, a czas spadku `power_fall_filter_ms`. Dzięki
temu ponowny nacisk może zostać obsłużony szybciej niż chwilowy spadek momentu
między nogami.

Filtr działa tylko, gdy PAS lub lokalny start bez obrotu pozostaje aktywny.
Po rzeczywistym zaniku pedałowania jego stan jest zerowany, target trybu wynosi
zero i dalsze zejście wykonuje Release/wspólna rampa. Nie ma podtrzymania
ostatniej mocy. Oba domyślne czasy wynoszą `0`, czyli filtr jest pomijany.

### Power Progressive — stan builda 0.0155

Tryb `POWER_PROGRESSIVE` stosuje stałoprzecinkowy odpowiednik wzoru:

```text
input = clamp(power / reference_power, 0, 1)
curve = (1 - progression) * input + progression * input²
support = support_min + (support_max - support_min) * curve
motor_power = power * support
```

`progression=0%` daje interpolację liniową, a `100%` pełną krzywą
kwadratową. Zakresy są ograniczone do: wsparcie 0–1000%, moc odniesienia
50–500 W i progresja 0–100%. Boost modyfikuje bazę przed krzywą, następnie
działają wspólne limity mocy, filtr, `P/U`, limity jazdy i dynamika.

Domyślne poziomy nadal mają `mode_type=POWER_LINEAR`. Ich `support_min` i
`support_max` są równe współczynnikowi liniowemu, więc samo przełączenie typu
bez strojenia pozostałych pól nie powoduje skoku charakterystyki.

## 5. Parametry już obsługiwane przez obecny Canable/protokół

Nazwy w obecnym UI bywają mylące. Poniższa tabela opisuje faktyczne użycie w
firmware.

| Lokalizacja | Zmienna firmware | Stan |
|---|---|---|
| `Para0[2,4,6,8,9]` | `assist_settings[level][2]` | TQfilter / „Ride Mode” per poziom; aktywne w Legacy |
| `Para0[12..27]` | `TQO_threshold[1..5]` | Wewnętrzny próg mapowania nacisku per poziom, 16-bit LE; UI przelicza go na kg i ogranicza do 31,2 kg dla obecnej mapy Legacy |
| `Para1[7..8]` | `battery_capacity_mah` | Aktywne |
| `Para1[9]` | `phase_current_max` | Aktywny sprzętowy limit skali prądu |
| `Para1[10]`, `[11]` | progi limp SOC | Aktywne; `0xFF` wyłącza |
| `Para1[12]` | `Cadence_exponent` | Aktywne tylko w formule Legacy |
| `Para1[14]` | `legalflag` | Aktywne |
| `Para1[20]` | `pulses_per_revolution` | Aktywne |
| `Para1[21]` | `decay_base` | Aktywne tylko w części Legacy |
| `Para1[36]` | `walk_assist_current` | Aktywne; fallback/default = **30%** |
| `Para1[37]` | `Override_Duration` | Parsowane; efekt Extended Boost jest wyłączony przez `EXTENDED_BOOST_ENABLE=0` |
| `Para1[38]` | `PAS_timeout` | Aktywne w Legacy |
| `Para1[39]` | `ramp_end` | Parsowane, obecnie nieużywane |
| `Para1[41,43,45,47,48]` | limit prądu poziomów 1–5 | Aktywne |
| `Para1[50,52,54,56,57]` | limit prędkości poziomów 1–5 | Aktywne |
| `Para1[60..61]` | `walk_assist_speed` | Aktywne w Legacy; 0 oznacza fallback 6,0 km/h |
| `Para2[0..29]` | `assist_profile[5][6]` | Aktywne profile Legacy |
| `Para2[31..35]` | `ext_boost_duration[1..5]` | Parsowane, lecz Extended Boost globalnie OFF |
| `Para2[37..41]` | `ext_boost_strength[1..5]` | Parsowane, lecz Extended Boost globalnie OFF |

## 6. Obecne parametry zaszyte w firmware — kandydaci do Canable

Te wartości działają, ale zmiana wymaga przebudowania firmware.

| Klucz docelowy | Obecna stała | Wartość `0.0148` | Docelowa grupa UI |
|---|---|---:|---|
| `iq_rise_slow_ms` | `IQ_RAMP_UP_SLOW_TICKS` | 600 ms | Dynamika |
| `iq_rise_fast_ms` | `IQ_RAMP_UP_FAST_TICKS` | 300 ms | Dynamika |
| `iq_fall_slow_ms` | `IQ_RAMP_DOWN_SLOW_TICKS` | 1000 ms | Dynamika |
| `iq_fall_fast_ms` | `IQ_RAMP_DOWN_FAST_TICKS` | 140 ms | Dynamika |
| `ramp_speed_low_kph` | `IQ_RAMP_SPEED_LO` | 4,0 km/h | Dynamika / Zaawansowane |
| `ramp_speed_high_kph` | `IQ_RAMP_SPEED_HI` | 20,0 km/h | Dynamika / Zaawansowane |
| `ramp_cadence_low_rpm` | `IQ_RAMP_CAD_LO` | 20 RPM | Dynamika / Zaawansowane |
| `ramp_cadence_high_rpm` | `IQ_RAMP_CAD_HI` | 70 RPM | Dynamika / Zaawansowane |
| `startup_boost_enabled` | `STARTUP_BOOST_ENABLE` | ON | Start i Boost |
| `startup_boost_strength_pct` | `STARTUP_BOOST_FACTOR` | 200% dodatkowego nacisku | Start i Boost |
| `startup_boost_cadence_step` | `STARTUP_BOOST_CADENCE_STEP` | 25/256 na RPM | Start i Boost / Zaawansowane |
| `startup_boost_mode` | `STARTUP_BOOST_MODE` | CADENCE | Start i Boost |
| `startup_boost_auto_threshold_kg` | `STARTUP_BOOST_AUTO_TQ` | obecnie wewnętrzne 20 mV; docelowo kg | Start i Boost |
| `smooth_start_enabled` | `SMOOTH_START_ENABLE` | OFF | Start i Boost |
| `smooth_start_ms` | `START_RAMP_TICKS` | 300 ms | Start i Boost |
| `torque_full_scale_kg` | `TQ_FULL_SCALE_MV` + kalibracja | stałe 60 kg w UI; endpoint natywny ukryty | Czujnik momentu |
| `torque_gate_start_kg` | `TQ_GATE_MIN` | obecnie wewnętrzne 18 mV; docelowo kg | Czujnik momentu |
| `torque_gate_release_kg` | `TQ_GATE_RELEASE` | obecnie wewnętrzne 5 mV; docelowo kg | Czujnik momentu |
| `assist_start_steps` | `START_MIN_STEPS` | 4 kroki | Czujnik PAS / Zaawansowane |
| `pas_stop_ms` | `PAS_STOP_TICKS` | 500 ms | Czujnik PAS |

Parametry techniczne takie jak `IQ_RAMP_Q_SHIFT`, surowe ticki 4 kHz i
wewnętrzne stany regulatorów nie powinny być pokazywane zwykłemu użytkownikowi.

## 7. Zmienne do dodania w Canable — profile poziomów

Każdy poziom `ECO`, `TOUR`, `SPORT`, `SPORT+`, `BOOST` ma mieć osobny rekord.
Poniższe pola nie mają jeszcze przydzielonych ID protokołu.

| Klucz | Etykieta UI | Typ / jednostka | Zalecany zakres | Status |
|---|---|---|---|---|
| `mode_type` | Tryb wspomagania | enum | Legacy, Power Linear, Power Progressive, eMTB TSDZ, eMTB Custom | FW Linear/Progressive; protokół + UI do podłączenia |
| `support_ratio_pct` | Współczynnik wsparcia | % | 0–1000 | FW + protokół + UI |
| `support_min_pct` | Minimalne wsparcie | % | 0–1000 | FW aktywne; protokół + UI do podłączenia |
| `support_max_pct` | Maksymalne wsparcie | % | 0–1000 | FW aktywne; protokół + UI do podłączenia |
| `reference_power_w` | Moc odniesienia | W | 50–500 | FW aktywne; protokół + UI do podłączenia |
| `progression_pct` | Progresja krzywej | % | 0–100 | FW aktywne; protokół + UI do podłączenia |
| `max_motor_power_w` | Maksymalna moc silnika | W | 0–1500 | FW + protokół + UI |
| `max_iq_pct` | Maksymalny Iq poziomu | % limitu fazowego | 0–100 | FW + protokół + UI |
| `assist_without_rotation` | Pomoc bez obrotu | bool | OFF/ON; domyślnie OFF | FW aktywne; protokół + UI do podłączenia |
| `without_rotation_threshold_kg` | Próg startu bez obrotu | kg | 0,00–7,50 | FW używa wartości natywnej 0–300 mV powyżej autozera; Canable pokazuje kg |
| `startup_boost_enabled` | Startup Boost | bool | OFF/ON | FW aktywne; protokół + UI do podłączenia |
| `startup_boost_mode` | Tryb Boost | enum | Cadence, Speed, Auto | FW aktywne; protokół + UI do podłączenia |
| `startup_boost_strength_pct` | Siła Boost | % dodatkowego nacisku | 0–300 | FW aktywne; protokół + UI do podłączenia |
| `startup_boost_end_rpm` | Koniec Boost | RPM | 0–120 | FW aktywne; protokół + UI do podłączenia |
| `smooth_start_enabled` | Smooth Start | bool | OFF/ON | FW aktywne; protokół + UI do podłączenia |
| `smooth_start_ms` | Czas Smooth Start | ms | 0–5000 | FW aktywne; protokół + UI do podłączenia |
| `iq_rise_ms` | Narastanie Iq | ms | 20–5000 | Wymaga decyzji: per-level czy globalne punkty rampy |
| `iq_fall_ms` | Opadanie Iq | ms | 20–5000 | Wymaga decyzji: per-level czy globalne punkty rampy |
| `release_ms` | Release po ustaniu PAS | ms | 0 = adaptacyjny; 1–3000 | FW aktywne; protokół + UI do podłączenia |
| `power_rise_filter_ms` | Filtr wzrostu mocy | ms | 0 = bypass; 1–3000 | FW aktywne; protokół + UI do podłączenia |
| `power_fall_filter_ms` | Filtr spadku mocy | ms | 0 = bypass; 1–3000 | FW aktywne; protokół + UI do podłączenia |
| `taper_start_kph` | Początek taperu | km/h | 0–60 | FW + protokół + UI |
| `taper_end_kph` | Koniec taperu | km/h | 0–60 | FW + protokół + UI |
| `taper_shape` | Kształt taperu | enum | Linear, Smoothstep, Map | FW + protokół + UI |

Nie ustalać jeszcze wartości domyślnych profili „na wyczucie”. Najpierw Power
Linear powinien odtworzyć jeden świadomie przetestowany profil, a potem można
kopiować go i różnicować poziomy.

## 8. Zmienne do dodania w Canable — Walk Assist

Obecne `walk_assist_current` i `walk_assist_speed` należy zachować dla
kompatybilności Legacy, ale poprawić ich opisy. Nowy regulator ERPS będzie
wymagał osobnego zestawu.

| Klucz | Etykieta UI | Jednostka / zakres | Wartość Legacy `0.0148` | Status |
|---|---|---|---:|---|
| `walk_assist_current_pct` | Walk Current | 0–100% fazowego | 30% | JUŻ JEST: `Para1[36]` |
| `walk_assist_speed_kph` | Walk Speed (Legacy) | 0,01 km/h | 6,0 km/h | JUŻ JEST: `Para1[60..61]` |
| `walk_target_erps` | Docelowe obroty silnika | ERPS | — | Nowy Walk |
| `walk_base_iq_pct` | Bazowy prąd pchania | % fazowego | — | Nowy Walk |
| `walk_start_iq_pct` | Prąd startowy | % fazowego, 0–100 | 100% hardcoded | FW + protokół + UI |
| `walk_start_end_speed` | Koniec strefy startowej | km/h | 3,0 | FW + protokół + UI |
| `walk_hold_scale_pct` | Skala utrzymania | % z Walk Current | 50% | FW + protokół + UI |
| `walk_kp` | Korekta P | wartość skalowana | 3/16 | FW + protokół + UI Advanced |
| `walk_ki_shift` | Korekta I | shift 6–16 | 11 | Legacy Advanced; nowy Walk zaczyna bez I |
| `walk_fade_band_kph` | Pasmo wygaszania | km/h | 2,5 | FW + protokół + UI |
| `walk_near_hold_pct` | Prąd przy celu | % hold | 15% | FW + protokół + UI |
| `walk_deadband_kph` | Martwa strefa | km/h | ±0,2 | FW + protokół + UI |
| `walk_overspeed_margin_kph` | Margines odcięcia | km/h | 0,5 | FW + protokół + UI |
| `walk_start_ramp_ms` | Rampa startowa | ms | 180 | FW + protokół + UI |
| `walk_timeout_s` | Limit ciągłej pracy | s | 10 | FW + protokół + UI Advanced |

## 9. Czego nie dodawać do zwykłego ekranu Canable

Poniższe elementy są wewnętrzne albo deweloperskie:

- `RIDE_ENGINE_LEGACY/TSDZ` jako normalny wybór użytkownika przed ukończeniem TSDZ,
- `control_owner`, stany Halla i stany open-loop,
- `IQ_RAMP_Q_SHIFT` i surowe wartości ticków 4 kHz,
- parametry PI FOC, Clarke/Park, SVPWM,
- bezpośredni zapis `Iq` lub `Id`,
- progi awaryjnego odcięcia bez walidacji firmware,
- `MagicNumber` i checksumy bloków,
- surowe ADC/mV torque poza ukrytą diagnostyką developerską,
- ręczne ustawianie punktu zerowego torque.

Tryb wyboru Legacy/TSDZ może istnieć wyłącznie w ukrytym panelu developerskim.

## 10. Protokół — decyzja przed pracą w Canable

Nie przydzielać nowych pól ręcznie do pozornie wolnych bajtów `Para0/1/2`.
Pełny profil ma ponad 20 parametrów, część 16-bitowych; pięć profili oraz Walk
Assist nie zmieszczą się bezpiecznie w obecnych wolnych slotach i kolidowałyby
z kompatybilnością HMI.

Utworzono pierwszy, nieaktywny draft źródła prawdy:

```text
protocol/
├── ebics_config_schema.yaml
└── generated/
    ├── ebics_config_ids.h
    └── ebics_config_schema.js
```

Plik YAML istnieje, natomiast katalog `generated` i oba pliki wynikowe powstaną
dopiero po audycie przestrzeni komend. W wersji draft v0 każde nowe pole ma
`wire_id: null`; oznacza to „nieprzydzielone”, a nie „ID zero”. Pola profili,
których wartości trzeba dobrać na rowerze, mają świadomie `default: null` i
blokują generator.

Wyniki audytu firmware, bloków Para i jednego lokalnego logu znajdują się w
`protocol/HMI_COMMAND_AUDIT.md`. Audyt potwierdził, że bloki konfiguracji to
`0x6010/0x6011/0x6012`, a `0x3200/0x3201/0x3205` są ramkami telemetrii.
Numer bazowy nowego bloku pozostaje pusty do sprawdzenia kodu Canable i pełnych
logów HMI/BESST.

Schemat musi definiować dla każdego pola: ID, typ, skalę, jednostkę, minimum,
maximum, wartość domyślną, `persistent`, `per_level`, wersję oraz obsługiwane
operacje. Z tego samego pliku mają powstawać definicje firmware i Canable.

Stare `Para0/1/2` pozostają warstwą zgodności dla Legacy. Nowe profile powinny
używać wersjonowanego bloku konfiguracyjnego lub nowych komend multiframe.

## 11. Układ Canable

```text
ECO | TOUR | SPORT | SPORT+ | BOOST
[Kopiuj] [Wklej] [Porównaj]

Charakterystyka
Start i Boost
Dynamika i Release
Limity
Prędkość
Walk Assist (globalny, osobna zakładka)

Sync | Apply RAM | Save Flash | Revert
```

Wykresy do dodania po wdrożeniu wzorów w firmware:

1. moc człowieka → moc silnika,
2. boost względem kadencji,
3. start/rampa/release względem czasu,
4. dostępne wsparcie względem prędkości.

Linia przerywana oznacza konfigurację zapisaną w kontrolerze, a ciągła —
wartości aktualnie edytowane.

## 12. Kolejność wdrażania konfiguratora

1. Po ukończeniu funkcji wykonać łączny test sprzętowy Legacy i nowych trybów.
2. Zatwierdzić draft YAML, wykonać audyt komend, przydzielić ID i dodać generator C/JavaScript.
3. Dodać minimalny profil: `mode_type`, `support_ratio`, `max_motor_power_w`, `max_iq_pct`.
4. Podłączyć Power Linear do profili protokołu; test jazdy wykonać w końcowej sesji sprzętowej.
5. Dodać start bez obrotu, Startup Boost, Smooth Start, rampy i Release.
6. Dodać progresję i eMTB.
7. Dodać osobny zestaw Walk Assist oparty na ERPS.
8. Dopiero wtedy dodać kopiowanie profili i wykresy.

Po każdym wdrożeniu aktualizować w tym dokumencie: status, commit, build,
przydzielone ID protokołu i nazwę pola Canable.
