# FW-068 — warunek startu asysty: konfigurowalny, z osobnym progiem dla jazdy

- **Data:** 2026-07-31
- **Status:** ZAIMPLEMENTOWANE, **niezbudowane i nieprzetestowane na sprzęcie**
  (build po stronie właściciela).
- **Zakres:** firmware M820 — `inc/config.h`, `inc/assist_modes.h`, `src/assist_modes.c`,
  `inc/tuning_config.h`, `src/tuning_config.c`, `src/ride_control.c`, `src/main.c`,
  `inc/main.h`, `src/CAN_Display.c`. Canable — `canbus.js`, `bafang-parser.js`,
  `ui/js/evistdrive/profiles.js`, `ui/js/evistdrive/dynamics.js`, `ui/index.html`.
- **Powiązane:** `FW-069_RAMPS_PER_LEVEL.md` — wspólny build i wspólny reset EEPROM.
  `FW-058`/`FW-059` — dryf zera, który ta karta obchodzi detektorem przyrostu.

> **Jednorazowy reset ustawień.** Patrz sekcja 6.

---

## 1. Problem

Asysta ruszała po spełnieniu **dwóch** warunków naraz:

1. nacisk ≥ „Minimum pedal load" (per poziom, ustawialny w Canable),
2. obrót korby ≥ `START_MIN_STEPS` = 4 kroki kwadratury (~15°), **zaszyte na stałe**.

Z tego wynikały dwa osobne problemy.

**Połowy progu nie dało się przestroić na rowerze.** Liczba kroków wymagała przebudowy
firmware, więc dobranie chwytu przy ruszaniu było niedostępne dla właściciela.

**Ponowne złapanie asysty w ruchu kosztowało tyle samo co start spod świateł.** Zatrzask
FW-031 spada natychmiast, gdy korba stanie (`PAS_STOP_TICKS` = 200 ms bez impulsu). Po
każdym wybiegu, zmianie biegu czy przerwie w pedałowaniu trzeba było znów przekroczyć pełny
próg — przy 25 km/h i pełnym rozpędzie, w sytuacji fizycznie zupełnie innej niż ruszanie
z zera. Do tego próg mierzy się od automatycznie ustawianego zera, które w jeździe dryfuje
(cała seria FW-058/059), więc realny wymagany nacisk dodatkowo pływał.

## 2. Dlaczego próg ma zależeć od tego, czy korba się kręci

Rozważane były dwa podejścia do „łatwiejszego łapania w ruchu".

**Podejście bez progu w ogóle.** Wspomaganie liczone jako `delta × kadencja × mnożnik`, bez
żadnej bramki: gdy delta spada do zera, moc gaśnie płynnie, a przed samozałączeniem chroni
sama kadencja, bo jest czynnikiem w mnożeniu. Elegancko, ale nie do pogodzenia z naszym
zatrzaskiem (FW-031), który powstał właśnie dlatego, że przy lekkim pedałowaniu i w martwych
punktach moc się u nas zapadała i silnik pulsował.

**Podejście z ruchomym punktem odniesienia** — i to wybraliśmy. Punkt, od którego liczy się
nacisk, może być inny, gdy korba się kręci, niż gdy stoi. Uzasadnienie jest fizyczne: na
stojącym rowerze obrót korby niczego nie dowodzi (łańcuch luźny, zerowe obciążenie), a na
toczącym się — dowodzi, bo rowerzysta faktycznie napędza. Dlatego obniżka wymaga **obu**
warunków naraz: kręcącej się korby i prędkości ≥ 1,0 km/h.

Nasza realizacja różni się od typowej w dwóch rzeczach:

- to **jawny parametr per poziom** obok „Minimum pedal load", a nie jedna wartość na cały
  rower ukryta w kodzie — Eco i Boost mogą mieć różny charakter łapania;
- zaczepiony o **`pedaling_active`** (kroki do przodu + brak wstecznego), a nie o samą
  kadencję. To sygnał, któremu w tej firmware ufamy, bo ma już wbudowaną odporność na
  kiwanie korbą.

## 3. Ustalenie, które ukształtowało projekt

**Na zjeździe przed fałszywym załączeniem chroni dziś nacisk, a nie kroki korby.** Kiwanie
pedałami przód/tył po korzeniach swobodnie przekracza 4 kroki do przodu. Ratuje to, że na
wolnobiegu łańcuch jest luźny i czujnik momentu praktycznie nic nie widzi.

Wniosek: samo obniżenie progu nacisku odsłania dokładnie tę sytuację. Dlatego druga droga do
załączenia nie jest niższym progiem, tylko **detektorem przyrostu z wymuszoną kolejnością**:
najpierw korba, potem narastanie nacisku utrzymane kilkadziesiąt milisekund. Uderzenie
w korzeń daje impuls, ale nie następuje po serii kroków PAS do przodu.

Efekt uboczny wart tyle co sama funkcja: **przyrost jest odporny na dryf zera**, bo mierzy
różnicę, a nie poziom.

## 4. Logika po zmianie

```
warunek wstępny:  fwd_run >= start_steps        (globalne, domyślnie 4)
                  i pedałowanie do przodu, brak safety_cut, poziom > 0

próg efektywny (per poziom):
  prog = Minimum pedal load
  jeśli pedaling_active ORAZ prędkość >= 1,0 km/h:
        prog -= start_load_reduction_mv                       (dół zacięty na 0)

w chwili spełnienia warunku wstępnego: baseline = aktualny nacisk (filtr 35 ms)

załącz, gdy SPEŁNIONE JEDNO Z:
  A) nacisk >= prog
  B) nacisk - baseline >= start_rise_mv
     utrzymane przez START_RISE_CONFIRM_MS (40 ms, stała w ride_control.c)
     w oknie start_rise_window_ms od zapamiętania baseline
```

Dwa szczegóły, bez których to nie działa:

- **Baseline trzymany na minimum z okna.** Gdyby śledził bieżącą wartość, powolne równomierne
  narastanie nacisku przesuwałoby go razem ze sobą, różnica nigdy nie osiągnęłaby progu
  i ścieżka B nie odpaliłaby ani razu.
- **Licznik potwierdzenia zeruje się przy spadku przyrostu poniżej progu.** Inaczej pojedyncze
  uderzenia sumowałyby się w załączenie.

### Obniżka wymaga jadącego roweru, nie samego ruchu korby

Sam obrót korby **nie wystarcza** do obniżenia progu. Na stojącym rowerze można kręcić korbą
do przodu przy luźnym łańcuchu i zerowym obciążeniu — czyli dokładnie w sytuacji, dla której
pełny próg istnieje. Obniżka służy **ponownemu złapaniu asysty na toczącym się rowerze**,
więc dochodzi warunek prędkości: `RIDE_START_REDUCTION_MIN_SPEED_X100` = 100, czyli 1,0 km/h.

Próg nie jest literalnym „> 0", bo sygnał prędkości potrafi dać pojedynczy fałszywy impuls
na postoju (patrz FW-036), a jeden taki impuls nie może odblokować niższego progu przy
stojącym rowerze. 1,0 km/h to ta sama granica „naprawdę jedzie", której używa już
`TQ_RECAL_MOVING_X100`.

Ruszanie z zera i „Assist without crank rotation" (działa przy kadencji 0) zawsze mają pełny,
nieobniżony próg.

## 5. Parametry

| Parametr | Zasięg | Zakres | Domyślnie | Gdzie |
|---|---|---|---|---|
| `Crank movement to start` | globalny | 1–20 kroków | 4 | Dynamics |
| `Pedal load reduction while pedalling` | per poziom | 0–100 mV | 0 = off | Profiles |
| `Engage on pressure rise` | per poziom | 0–100 mV | 0 = off | Profiles |
| `Pressure rise window` | per poziom | 0–2000 ms | 400 | Profiles |

Obie nowe funkcje domyślnie zerowe → **zachowanie zaraz po wgraniu identyczne z dotychczasowym**.

`start_steps` jest globalny świadomie: to zabezpieczenie przed kiwaniem korbą, czyli własność
roweru i czujnika, a nie charakter poziomu asysty. Pięć kopii jednej wartości bezpieczeństwa
łatwo rozjechać. `START_MIN_STEPS` w `config.h` zostaje jako wartość domyślna i jako wartość
zamrożonego monolitu Legacy (Walk Assist + kalibracja pozycji), który nadal jej używa wprost.

## 6. EEPROM — jednorazowy reset

Blob tuningu rośnie do v6 (32 B), a `MP.tuning_store` do 64 B; razem z FW-069 zmienia się
rozmiar `MotorParams_t`, więc zapisany rekord przestaje pasować i kontrola z FW-023
(`param_record_valid()`, porównanie długości z `FMC_OFFSET_FOOTER`) go odrzuca.

To jest **kontrolowany powrót do wartości domyślnych, nie uszkodzenie** — nigdy nie dochodzi
do odczytania śmieci. `InitEEPROM()` kładzie świeży rekord, a `read_virtual_eeprom()` woła
`hall_load_defaults()`, więc kąty Halla wracają do wkompilowanych `HALL_DEF_*`. Jeśli silnik
brzęczy zamiast płynnie ruszać, wkompilowane kąty różnią się od kalibracji tego egzemplarza —
trzeba powtórzyć kalibrację pozycji (0x6200) **przed wyjazdem**.

## 7. Zgodność wersji bloba tuningu

Kontrola długości w `tuning_config_apply_blob()` przepisana na tabelę wersji:

| wersja | body (przed CRC) | min. długość |
|---|---|---|
| 2 | 20 | 22 |
| 3/4/5 | 22 | 24 |
| 6 | 30 | 32 |

Poprzednia wersja wyprowadzała minimalną długość z `TUNING_BLOB_LEN`, więc samo podbicie tej
stałej zaczęłoby odrzucać poprawne bloby v3–v5 wysyłane przez starszy Canable.

**Przy okazji naprawione dwie migracje z FW-053** (`hold_ms 700→1400`, `min_iq_pct 4→2`):
testowały `version < TUNING_VERSION`, czyli po każdym kolejnym podbiciu wersji odpalałyby się
ponownie i nadpisywały wartość, którą użytkownik świadomie ustawił z powrotem. Teraz testują
`version < 5`, zgodnie z pierwotnym zamiarem.

`start_steps` == 0 na drucie jest czytane jako wartość domyślna 4, nie jako „brak warunku
obrotu korby" — pusty lub stary zapis nie może skasować zabezpieczenia.

Canable **negocjuje wersję w dół**: `serializeTuningBlob()` wysyła v6 tylko wtedy, gdy odczyt
z kontrolera zwrócił `tuning_schema_version >= 6`, w przeciwnym razie wysyła dawny układ v5
(24 B). Bez tego v6 trafiłby do starszej firmware, która odrzuca nieznaną wersję — i cały
zapis karty Dynamics kończyłby się niepowodzeniem. Blob banków negocjował wersję od FW-056;
blok tuningu robi to od tej karty.

### v5 → v6: co dokładnie się dzieje

| skąd przychodzi v5 | co robi firmware |
|---|---|
| **z EEPROM** | **nie przychodzi wcale.** Zmiana rozmiaru `MotorParams_t` unieważnia cały rekord (kontrola długości z FW-023), więc zapisany blob v5 nie jest nawet parsowany — obowiązują wartości domyślne builda |
| **po CAN, ze starszego Canable** | blob przyjęty, nowe pola uzupełnione domyślnymi (rekord 35 B → stride z `buffer[5]`), wartości trzymane w RAM |
| **zapis do flash (0x6022)** | serializacja zawsze w **v6** — po pierwszym „Save to Flash" w EEPROM leży już nowy format |

Czyli migracja v5→v6 zachodzi wyłącznie **na drucie**, nigdy w pamięci. Twierdzenie
o „migracji EEPROM" byłoby nieprawdziwe: tam nie ma czego migrować, bo rekord i tak przepada.

## 8. Odpowiedź na zapis konfiguracji: ACK tylko po udanym zastosowaniu

Dotąd kontroler odpowiadał na **każdy** zakończony transfer wieloramkowy operacją 2
(`NORMAL_ACK`), nie patrząc na wynik funkcji stosującej blob. Blob odrzucony za złe CRC albo
nieobsługiwaną wersję wyglądał dla Canable identycznie jak zapis udany: narzędzie meldowało
„written", użytkownik naciskał „Save to Flash" — i do flasha szły **stare** ustawienia.

Teraz:

- **zastosowany** → operacja 2 (`NORMAL_ACK`),
- **odrzucony** (złe CRC, nieznana wersja, zła długość rekordu) → operacja 3 (`ERROR_ACK`),
- **transfer niekompletny** (zgubiona lub nadmiarowa ramka, suma długości się nie zgadza) →
  również operacja 3, zamiast dotychczasowej ciszy i czekania narzędzia na timeout.

`ERROR_ACK` jest już rozumiany przez `request-manager.js` po stronie Canable i rozwiązuje
żądanie jako **niepowodzenie**, więc karta pokaże „was not written" z powodem. Stara funkcja
`sendMultiframeWriteAck()` została usunięta, żeby nie dało się jej użyć przez pomyłkę.

## 9. Testy na rowerze

Kolejność ma znaczenie: najpierw punkt odniesienia, potem **po jednej zmiennej naraz**.

1. Wszystko na 0 → zachowanie **nieodróżnialne** od dotychczasowego. Bez tego punktu nie da
   się ocenić reszty.
2. Sama obniżka (10 mV na jednym poziomie): ponowne łapanie po wybiegu przy lżejszym nacisku,
   ruszanie spod świateł **bez zmian**. Inny poziom bez obniżki = kontrola, że per poziom działa.
3. **Obniżka na stojącym rowerze — test warunku prędkości.** Rower nieruchomo, kręć korbą do
   przodu z lekkim naciskiem poniżej pełnego progu: asysta **nie może** ruszyć, mimo dużej
   obniżki. Dopiero po rozpędzeniu ten sam nacisk ma łapać.
4. Zjazd po korzeniach z samą obniżką — silnik nie może się załączyć.
5. Sam detektor przyrostu (obniżka na 0, rise 12 mV, okno 400 ms): łapanie szybsze niż w teście 1.
6. **Zjazd po korzeniach, rise 12 mV — test krytyczny.** Kiwanie pedałami przód/tył w małym
   zakresie nie może uruchomić silnika. Jeśli uruchamia: podnieść rise i/lub
   `START_RISE_CONFIRM_MS`, i **dopisać wynik do tej karty**.
7. Zjazd po korzeniach, rise 4 mV (celowo za nisko) — gdzie leży granica, żeby znać realny
   zapas przy wartości zalecanej.
8. Kroki korby: 2 → asysta łapie wyraźnie wcześniej niż przy 8; 8 → kiwanie korbą na stojąco
   nie uruchamia silnika.
9. Wszystko razem — funkcje się nie znoszą ani nie sumują w nieoczekiwany sposób.

### Testy zapisu konfiguracji (bez jazdy)

10. Zapis banku i zapis tuningu kończą się ACK, wartości wracają po Read.
11. **Restart po zapisie:** Save to Flash → wyłączyć i włączyć kontroler → Read → wartości
    trzymają, a blob wraca jako **v6** (rekord 46 B).
12. **Odrzucenie musi być widoczne:** wymusić błąd (np. starszy Canable wysyłający v6 do
    firmware bez FW-068, albo celowo uszkodzone CRC) → karta pokazuje „was not written",
    a nie ciche powodzenie. To jest test zmiany z sekcji 8.
