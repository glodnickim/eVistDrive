# FW-084 — audyt implementacji i handoff dla developera

- **Data audytu:** 2026-08-06
- **Zakres:** firmware M820, bank v8/CAN, diagnostyka, Canable i nazewnictwo
- **Wynik:** **NIE ZAMYKAĆ FW-084**. Kod buduje się i obecne testy przechodzą, ale przed próbą
  obciążeniową trzeba naprawić obchodzenie limitów poziomu i jawnie rozstrzygnąć klasyfikację
  Extended Boost w limiterze prędkości.

## 1. Nazwa produktu

Jedyna nazwa nowej funkcji Ride Core:

```text
Extended Boost
```

W UI można używać pełnego nagłówka `Obstacle assist — Extended Boost`. W kodzie dozwolone są
istniejące skróty `assist_extended_boost`, `ASSIST_EXT_BOOST_*` i diagnostyka `ext_boost_*`.

Nazw `Overrun` i `Override` **nie używać dla nowej funkcji**:

- `Overrun_*`, `Override_Duration`, stare `ext_boost_*` w `MotorState_t` oraz
  `EXTENDED_BOOST_ENABLE` należą do nieaktywnego mechanizmu Legacy;
  
- `override` jest już używane także dla manetki i nie opisuje maszyny stanów FW-084;
- w dokumentacji/UI/logach stary mechanizm nazywać `Legacy overrun (inactive in Ride Core)`;
- nie włączać `EXTENDED_BOOST_ENABLE`; ewentualne usunięcie starego bloku zrobić osobnym,
  mechanicznym zadaniem po sprawdzeniu diagnostyki Legacy.

## 2. Znalezione problemy

### P0 — Extended Boost omija `Maximum motor current` i limit mocy poziomu

`ride_control.c` przekazuje modułowi globalny `ride_core_iq_limit`. Moduł ogranicza wynik tylko
do tej wartości, a potem zastępuje nią zwykły pedal-only `iq_target`. Tymczasem
`max_iq_pct` i `max_motor_power_w` są stosowane wcześniej, wewnątrz
`assist_modes.c::finish_power_request()`, wyłącznie do zwykłego wyniku trybu. Po zastąpieniu
targetu przez Extended Boost te ograniczenia nie są wykonywane ponownie.

Przykład błędu odbiorowego:

```text
globalny ride_core_iq_limit = 700
Maximum motor current poziomu = 20%
peak load = full scale, strength = 255%

oczekiwany maksymalny target poziomu: 140
obecny możliwy target Extended Boost: 700
```

`assist_limits_apply()` tego nie naprawia — ten moduł obsługuje napięcie, temperaturę i limit
prędkości, ale nie `max_iq_pct` ani `max_motor_power_w`. Pomoc Canable, która mówi, że Boost
jest ograniczony przez maksymalny prąd i moc poziomu, jest więc obecnie nieprawdziwa.

**Wymagana poprawka:** wyprowadzić jeden współdzielony limit profilu dla pedal-only targetu i
zastosować go także po Extended Boost, przed połączeniem z manetką. Musi obejmować co najmniej:

1. `ride_core_iq_limit` (łącznie z limp/SOC),
2. `level->max_iq_pct`,
3. `level->max_motor_power_w` oraz twardy limit mocy,
4. wspólne ograniczenia napięcia, temperatury, prędkości i trybu legal.

Nie ograniczać w ten sposób manetki limitem wybranego poziomu — jej osobna ścieżka i
klasyfikacja `NON_PEDAL` mają zostać zachowane.

### P0 — niejawna klasyfikacja prędkości po PAS STOP

Na zboczu PAS STOP zatrzask `assist_latched` jest zerowany. Extended Boost może wtedy wejść w
`ACTIVE`, ale późniejszy wybór źródła limitu nadal zależy wyłącznie od zatrzasku:

```c
limits_input.source = assist_latched ?
    ASSIST_LIMIT_SOURCE_PEDAL_CONFIRMED :
    ASSIST_LIMIT_SOURCE_NON_PEDAL;
```

W konsekwencji aktywny Boost jest klasyfikowany jako `NON_PEDAL`. W trybie legal oznacza to
taper 5–7 km/h i zero od 7 km/h. Może to być właściwa, konserwatywna polityka, ale karta
FW-084 jej nie definiuje, UI o niej nie mówi, a testy jej nie sprawdzają.

**Decyzja wymagana przed testem terenowym:** jedna z dwóch, zapisana w karcie i w teście:

- rekomendacja bezpieczeństwa: Extended Boost pozostaje `NON_PEDAL`; UI mówi wprost, że w
  trybie legal działa tylko w zakresie do 7 km/h;
- alternatywa produktowa: wcześniejsze, legalnie potwierdzone pedałowanie autoryzuje
  `PEDAL_CONFIRMED` przez czas ACTIVE. Tę opcję wdrażać wyłącznie po osobnym potwierdzeniu
  wymagań prawnych/produktowych; źródło musi pochodzić z `boost_output.active`, nie ze starego
  zatrzasku.

Nie zostawiać obecnego zachowania jako efektu ubocznego `assist_latched == false`.

### P1 — zapis aktywnego banku nie kasuje uzbrojenia

`assist_modes_apply_bank_blob()` może zmienić próg, siłę i czas bieżącego poziomu, po czym
wywołuje tylko `assist_modes_reset()`. Stan `assist_extended_boost` pozostaje bez zmian, jeżeli
indeks banku i poziomu się nie zmienił. Uzbrojenie utworzone pod starą konfiguracją może więc
zostać odtworzone z nową siłą/czasem.

**Poprawka:** po udanym RAM apply banku skasować `QUALIFY/ARMED/ACTIVE` w tym samym takcie.
Najprostsza bezpieczna polityka to reset po każdym udanym zapisie banku; wariant precyzyjny
resetuje tylko wtedy, gdy zapisano aktywny bank. Dodać osobny kod przyczyny diagnostycznej
`CONFIG_CHANGED` albo świadomie użyć rozszerzonego `LEVEL_OR_BANK_CHANGE`.

### P1 — test FW-084 wykonuje kopię algorytmu, a nie kod C

`tests/fw084_extended_boost.js` zawiera ręczny port klasy `Boost` i kilka kontroli regex.
Dlatego przechodzi mimo pominięcia `max_iq_pct`, limitu mocy i niejawnej klasyfikacji
`NON_PEDAL`. Komentarz, że ten sam kod jest ćwiczony na PC, nie odpowiada obecnej architekturze
testu.

**Poprawka:** dodać hostowy harness uruchamiający `src/assist_extended_boost.c`, a integrację
`ride_control -> profile limits -> source limit -> dynamics` sprawdzać osobnym modelem/harnessem.
Kontrole regex mogą zostać jako ochrona kolejności, ale nie jako dowód zachowania.

### P1 — preset v8 zaimportowany do kontrolera v7 jest cicho tracony

Canable blokuje edycję karty dla schematu starszego niż v8, lecz importer presetów korzysta z
`levelFieldDescriptors()`, które zawiera pola Extended Boost bez warunku wersji. Preset v8
może więc wpisać je do modelu banku v7, po czym serializer prawidłowo wyśle v7 i pominie te
pola. Użytkownik widzi zaimportowane, lecz niemożliwe do zapisania ustawienie.

**Poprawka:** pola/deskriptory powinny mieć `minBankSchema: 8`, a importer ma je pominąć z
jawnym komunikatem albo zablokować import tych pól. Dodać test preset v8 -> odczytany bank v7.

### P2 — statusy i dokumentacja są niespójne

- `FW-084_EXTENDED_BOOST_PLAN.md` nadal mówi, że funkcji nie ma w Ride Core ani Canable;
- `FW-091_LIMIT_SOURCE_CLASSIFICATION.md` mówi o „przyszłym overrun”, mimo istniejącej
  implementacji FW-084;
- stare dokumenty używają `Extended Boost`, `Overrun` i `Override` wymiennie dla mechanizmu
  Legacy;
- główny `CHANGELOG.md` nie ma wpisu FW-084.

Po naprawach oznaczyć kartę jako `IMPLEMENTED — BENCH/ROAD VALIDATION PENDING`, dopisać decyzję
o klasyfikacji limitu prędkości i zaktualizować dokumenty Legacy zgodnie z §1.

### P2 — diagnostyczne `remaining_ms` zaniża czas

Getter dzieli `active_ticks_left / 4`, więc na początku 200 ms pokazuje 199 ms, a przez ostatnie
1–3 takty może pokazywać 0 mimo stanu ACTIVE. Dla czytelnej diagnostyki zastosować zaokrąglenie
w górę `(active_ticks_left + 3) / 4` i dodać test graniczny.

## 3. Co jest już poprawne

- osobny moduł i maszyna stanów `IDLE/QUALIFY/ARMED/ACTIVE`;
- kwalifikacja 30 ms, histereza, świeżość uzbrojenia i brak odświeżania timera;
- wejście ACTIVE tylko na zboczu PAS STOP;
- reset dla safety, reverse, błędów czujnika, Walk, kalibracji, zmiany poziomu/banku,
  utraty ruchu i powrotu pedałowania;
- Boost działa przed manetką i nie kopiuje jej prądu;
- jedna rampa Release po zakończeniu ACTIVE;
- bank v8: 48 B/poziom, 255 B całość, CRC 253–254 i ostatnia ramka CAN 7 B;
- migracja v1–v7 do `8.0 kg / 100% / 0 ms`;
- diagnostyka `0x6029` v5 i zgodność parsera z v4;
- Canable ma pola, wartości domyślne, kartę v8, blokadę edycji starszego firmware oraz test
  round-trip v8/v7.

## 4. Minimalny zestaw testów odbiorowych po poprawce

### Limity prądu i mocy

1. `max_iq_pct = 20`, pełny peak, `strength = 255`: Boost nigdy nie przekracza 20% globalnego
   limitu poziomu.
2. `max_motor_power_w = 100`: Boost jest ograniczony tą samą metodą co zwykły pedal assist.
3. Limp/SOC, napięcie, temperatura i limit prędkości ograniczają Boost, ale nie zatrzymują ani
   nie odświeżają timera.
4. Manetka pozostaje osobno ograniczona jako `NON_PEDAL` i nigdy nie jest źródłem zapisanego
   peak load/current.

### Klasyfikacja prędkości

W trybie legal sprawdzić co najmniej 3, 6, 7 i 10 km/h. Oczekiwane wartości muszą wynikać z
jawnie wybranej decyzji z §2, a nie ze stanu zatrzasku. Powtórzyć dla offroad.

### Stan i zmiana konfiguracji

1. Uzbroić funkcję, zapisać aktywny bank z inną siłą/czasem, wykonać PAS STOP: stare
   uzbrojenie nie może uruchomić silnika.
2. Zapis banku podczas ACTIVE kończy Boost i rozpoczyna dokładnie jedną właściwą rampę Release.
3. Próg/siła/czas na każdym poziomie przechodzą round-trip firmware <-> Canable.

### Canable

1. v8: 255 B, pola na 36/37/46–47, skrajne `1.0/22.5 kg`, `0/255%`, `0/1000 ms`, poprawny CRC.
2. v7: 245 B, bajty 36–37 pozostają zerowe, żadnego wysłania v8.
3. Preset v8 do kontrolera v7: pola są jawnie pominięte/odrzucone, nigdy cicho tracone.
4. `0x6029` v4 i v5: długość, CRC, stan, peak, Iq, czas i przyczyna anulowania.

## 5. Stan weryfikacji z audytu

- wszystkie testy JavaScript firmware FW-056..FW-092: **PASS**;
- Canable `npm test`: **PASS**, łącznie z `fw084_bank_v8_roundtrip.js`;
- Canable `npm run lint`: **PASS**;
- firmware debug/normal: **BUILD PASS**;
- firmware debug/diagnostic: **BUILD PASS**;
- build ma istniejące ostrzeżenia `char*`/`uint8_t*` w `CAN_Display.c` i nieużywane `fw_ver`;
  nie są wprowadzone przez logikę FW-084.

Testy nie kasują dwóch problemów P0: obecny zestaw nie modeluje limitu prądu/mocy poziomu ani
nie asercjonuje zachowania Extended Boost przy 7+ km/h w trybie legal.
