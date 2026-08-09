# FW-101 — Stabilny SOC: coulomb counting jako źródło prawdy i jawne wykrywanie ładowania

- **Data:** 2026-08-09
- **Status:** PLAN DLA DEVELOPERA — audyt zakończony, kod niewdrożony
- **Priorytet:** wysoki; obecny algorytm może zwiększać i utrwalać SOC bez ładowania
- **Dotyczy:** M820 / GD32F303, licznik SOC, zapis strony SOC, próg pełnego pakietu,
  auto-off oraz diagnostyka eVistDrive
- **Powiązane:** FW-018 / CB-007 (konfigurowalne napięcie pełnego pakietu).
  Ta karta konsoliduje niezrealizowane pozycje nazwane tam roboczo FW-019, FW-020
  i FW-021; numeracja projektu doszła już do FW-100, dlatego właściwym następcą
  jest FW-101.

---

## 1. Decyzje właściciela — wymagania nadrzędne

Ta sekcja jest kontraktem funkcjonalnym. Implementacja nie może zmienić tych decyzji
„dla wygody algorytmu”.

1. **Podczas normalnej pracy źródłem prawdy jest coulomb counting.**
   Jeżeli policzyliśmy, ile mAh wypłynęło z baterii, napięcie po zatrzymaniu nie ma prawa
   dopisywać energii.
2. **Usunąć ciągłą korektę OCV po 30 s postoju.**
   Zwykły postój, toczenie i relaksacja ogniw nie mogą zwiększać `soc_real` ani
   `remaining_mah`.
3. **Napięcie ma tylko trzy jawne role:**
   - jednorazowa, łagodna korekta po uruchomieniu,
   - wykrywanie ładowania przy włączonym rowerze,
   - twarde zakotwiczenie 100% przy napięciu pełnego pakietu.
4. **Stan SOC zapisywać na dedykowanej stronie flash tylko przy kontrolowanym
   wyłączeniu roweru:** przycisk, auto-off albo watchdog komunikacji.
   Nie zapisywać okresowo podczas jazdy.
5. **Ładowarka M820 nie jest traktowana jako wiarygodne źródło ujemnego prądu.**
   Ładowanie może omijać bocznik kontrolera. Algorytm ładowania musi działać z napięcia.
6. **Uwzględnić ładowanie przy stale włączonym rowerze.**
   HMI może ustawić `auto_off_minutes = 0`, więc kontroler może pozostać włączony przez
   cały cykl ładowania.
7. **Nie tworzyć drugiego parametru napięcia pełnego pakietu.**
   Istniejące `MP.soc_full_pack_10mv` pozostaje jedynym konfigurowalnym progiem 100%.

---

## 2. Wniosek z audytu — dlaczego obecny SOC „pływa”

### 2.1 Obecna korekta dodaje energię po zatrzymaniu

W `soc_update()` po 30 s z `|Battery_Current| < 500 mA` wykonywane jest co sekundę:

```c
MS.soc_real += OCV_CORR_GAIN * ((float)MS.soc_voltage - MS.soc_real);
MS.remaining_mah =
    MS.soc_real / 100.0f * (float)MP.battery_capacity_estimated_mah;
```

Przy `OCV_CORR_GAIN = 0.02` luka do wyniku napięciowego maleje co sekundę do 98%
poprzedniej wartości. Jeżeli `soc_voltage` leży o 10 punktów procentowych wyżej od
licznika, po 60 korektach `soc_real` odzyskuje około 7 punktów procentowych.

Wyświetlacz maskuje tempo limitem `SOC_DISP_MAX_STEP = 1%/min`, dlatego użytkownik widzi
powolne przybywanie procentów zamiast jednego skoku. Nie jest to wyłącznie kosmetyka:
korekta nadpisuje `remaining_mah`, a następnie może zostać zapisana do flash.

### 2.2 „Rest” nie oznacza rzeczywistego postoju

Aktualny warunek używa tylko prądu. Nie wymaga:

- `MS.Speedx100 == 0`,
- `MS.cadence == 0`,
- `MS.i_q_setpoint == 0`,
- stabilnego napięcia,
- braku toczenia.

Korekta może więc działać podczas długiego zjazdu bez napędu lub przy bardzo małym
poborze. Starszy mechanizm sprawdzał także prędkość i zadanie silnika; obecna wersja
jest pod tym względem regresją.

### 2.3 Tabela nie jest prawdziwą tabelą OCV

`calculate_SOC()` używa krzywej opisanej w kodzie jako pomiar rozładowania LG M58T
przy 3 A. Napięcie po zdjęciu obciążenia jest naturalnie wyższe niż napięcie tej samej
baterii podczas rozładowania, więc używanie tej tabeli w spoczynku ma systematyczną
skłonność do zawyżania SOC.

### 2.4 Fałszywy wzrost zostaje utrwalony

`power_off_controller()` zapisuje `soc_real` i `remaining_mah`. Jeżeli rower stał przed
wyłączeniem, korekta OCV mogła wcześniej dodać mAh, a wyłączenie zapisuje już błędny
stan jako punkt startowy następnego uruchomienia.

### 2.5 Znak prądu jest niewidoczny w zwykłej telemetrii

Wewnętrzny `MS.Battery_Current` jest typem podpisanym, lecz standardowa ramka Bafang
wysyła `abs(MS->Battery_Current)`. Z logu HMI/CSV nie można zatem stwierdzić, czy ADC
kiedykolwiek widzi prąd ujemny. Dodatkowo startowa kalibracja zera dopuszcza około
±200 kodów ADC, czyli około ±7,4 A przy obecnym przeliczniku. Jeżeli prąd ładowania
przechodziłby przez wejście, typowa ładowarka mogłaby zostać wyzerowana jako offset.

W FW-101 prąd ujemny nie będzie używany do zwiększania SOC, ale podpisany pomiar musi
zostać wystawiony diagnostycznie, aby oddzielić ograniczenie sprzętu od ograniczenia
protokołu.

---

## 3. Co daje usunięcie korekty OCV

Usunięcie korekty nie poprawia dokładności samego pomiaru mAh. Daje natomiast
deterministyczne i sprawdzalne zachowanie:

- SOC podczas jazdy i zwykłego postoju nie rośnie;
- relaksacja napięcia, temperatura i chwilowy sag nie zmieniają policzonej energii;
- `remaining_mah`, zasięg i limp mode korzystają z jednego źródła prawdy;
- postój nie może stworzyć „darmowych” mAh i zapisać ich do flash;
- strojenie pojemności i offsetu można diagnozować osobno zamiast ukrywać ich błędy
  korektą napięciową.

Cena: coulomb counting może z czasem dryfować przez błąd offsetu prądu albo błędnie
ustawioną pojemność. Ten dryft ma być zerowany w punktach, w których napięcie naprawdę
niesie mocną informację: po potwierdzonym ładowaniu i przy pełnej baterii — nie przy
każdym zatrzymaniu.

---

## 4. Nienaruszalne inwarianty nowego algorytmu

Developer ma zapisać te zasady również jako testy:

1. W stanach `DISCHARGE`, `REST_RELAX` i `REST_OBSERVE`:

   ```text
   soc_real(t + 1) <= soc_real(t)
   remaining_mah(t + 1) <= remaining_mah(t)
   ```

   Jedyny wyjątek stanowi jawna korekta startowa wykonana raz.

2. Wzrost `soc_real` jest dozwolony tylko gdy:

   - zakończyła się zatwierdzona korekta startowa,
   - stan to `CHARGING_CONFIRMED` albo `POST_CHARGE_SETTLE`,
   - lub został osiągnięty skonfigurowany próg pełnego pakietu.

3. `soc_voltage` jest obserwacją diagnostyczną, nie drugim równoległym licznikiem
   mogącym dowolnie nadpisywać `remaining_mah`.
4. Zwykły postój przez dowolnie długi czas nie może zwiększyć wyświetlanego SOC.
5. Standardowa ramka Bafang pozostaje kompatybilna. Nie zmieniamy znaczenia jej
   bezwzględnego pola prądu; podpisany prąd dodajemy do diagnostyki eVistDrive.
6. `sizeof(MotorParams_t)` nie może się zmienić. FW-101 nie dodaje parametrów trwałych.
7. Zapis strony SOC nie może nastąpić w ruchu ani okresowo w czasie jazdy.

---

## 5. Docelowy podział odpowiedzialności

### 5.1 Coulomb counting — normalna praca

Akumulator `soc_mAs_acc` ma zliczać wyłącznie rozładowanie:

```c
int32_t i_discharge_ma =
    (MS.Battery_Current > SOC_COULOMB_DEADBAND_MA)
        ? MS.Battery_Current
        : 0;

soc_mAs_acc += (float)i_discharge_ma / 4000.0f;
```

Wartość ujemna nie dodaje energii. Mały dodatni szum także nie zużywa baterii.
`SOC_COULOMB_DEADBAND_MA` nie może odziedziczyć obecnego `I_REST_MA = 500`, bo
500 mA przy 40 V to około 20 W i mogłoby usuwać realny lekki pobór. Początkowa
wartość do testów: **100 mA**, następnie strojenie z podpisanego logu na postoju.

W `soc_update()`:

```c
float dmah = soc_mAs_acc / 3600.0f;  // zawsze >= 0
MS.remaining_mah -= dmah;
```

Usunąć komentarze i logikę sugerującą `regen < 0 adds back`. Ten target nie ma
potwierdzonego pomiaru ładowania przez bocznik.

### 5.2 Napięcie podczas jazdy

`soc_voltage` nadal może być liczone i raportowane do diagnostyki. Nie może
zmieniać `soc_real` ani `remaining_mah` w normalnym stanie. IR compensation może
pozostać jako pomoc diagnostyczna, lecz nie wolno nazywać wyniku prawdziwym OCV,
dopóki tabela nie zostanie zmierzona bez obciążenia.

### 5.3 Wyświetlacz

`soc_display` może być filtrowany w dół tak jak dziś. W zwykłym stanie dodatni
`diff = soc_real - soc_display` musi zostać zablokowany:

```c
bool upward_allowed =
    soc_boot_correction_just_applied ||
    soc_charge_state == SOC_CHARGING_CONFIRMED ||
    soc_charge_state == SOC_POST_CHARGE_SETTLE ||
    soc_full_anchor;

if(diff > 0.0f && !upward_allowed) {
    diff = 0.0f;
}
```

Zapobiega to sytuacji, w której ekran jeszcze przez kilka minut rośnie, goniąc
wcześniejszy błędny `soc_real`.

---

## 6. Korekta startowa — tylko raz i bez mocnego ruchu od małej różnicy

### 6.1 Usunąć natychmiastowy reset z `soc_init()`

Obecny kod od razu po wczytaniu flash wykonuje twardą korektę, gdy:

```c
soc_ocv - saved_soc > RECHARGE_MARGIN_PCT
```

Ta decyzja jest podejmowana przed pełnym oknem stabilizacji napięcia. FW-101 ma
rozdzielić:

1. odczyt i walidację zapisanego stanu;
2. zebranie stabilnego napięcia startowego;
3. jednorazową decyzję o korekcie;
4. normalne liczenie coulombów.

### 6.2 Dane zapisane z poprzedniego wyłączenia

`soc_slot_t.last_voltage_mv` już istnieje i jest zapisywane, ale obecnie nie jest
używane po odczycie. `soc_state_load()` ma udostępnić:

- `soc_saved_real`,
- `soc_saved_remaining_mah`,
- `soc_saved_voltage_mv`,
- informację, czy slot przeszedł walidację.

Nie zmieniać rozmiaru 32-bajtowego slotu.

### 6.3 Okno stabilizacji

Początkowe stałe:

```c
#define SOC_BOOT_SETTLE_S                15U
#define SOC_BOOT_STABLE_SPAN_MV         200U
#define SOC_BOOT_RISE_DEADBAND_MV       100U
#define SOC_BOOT_ERROR_DEADBAND_PCT      3.0f
#define SOC_BOOT_FULL_GAIN_ERROR_PCT    15.0f
```

Przez pierwsze 15 s:

- silnik nie może dostać zadania specjalnie na potrzeby pomiaru;
- zbieramy minimum, maksimum i średnią napięcia;
- okno jest ważne tylko, gdy `vmax - vmin <= 200 mV`;
- korekta nie jest wykonywana, jeśli pomiar nie jest stabilny.

### 6.4 Kolejność decyzji

1. Jeżeli nie ma prawidłowego slotu SOC: pierwszy start inicjalizuje SOC z
   napięcia po stabilizacji.
2. Jeżeli istniejący `soc_full_pack_10mv` jest ważny i stabilne napięcie pakietu
   jest równe lub wyższe od progu: ustaw 100%.
3. W przeciwnym razie:

   ```text
   delta_mv  = boot_voltage_mv - saved_voltage_mv
   delta_soc = soc_voltage - saved_soc
   ```

4. Dodatnia korekta wymaga jednocześnie `delta_mv > 100 mV` i
   `delta_soc > 0`. Brak wzrostu napięcia oznacza brak ponownego „pompowania”
   SOC przy kolejnych restartach.
5. Mały błąd jest wygaszany płynnie:

   ```text
   a = abs(delta_soc)

   if a <= 3%:
       correction = 0
   else:
       gain = clamp((a - 3%) / (15% - 3%), 0, 1)
       correction = delta_soc * gain
   ```

   Przykładowo:

   | `delta_soc` | Korekta |
   |---:|---:|
   | 2% | 0% |
   | 5% | około 0,8% |
   | 10% | około 5,8% |
   | 15% | 15% |

6. Duża ujemna różnica może użyć tej samej funkcji jako korekta bezpieczeństwa
   (np. wymiana baterii na rozładowaną). Mała ujemna różnica od temperatury lub
   niedokładności krzywej pozostaje w martwej strefie.
7. Korekta wykonywana jest dokładnie raz. Wynik ustawia spójnie
   `soc_real`, `soc_display`, `MS.SOC` i `remaining_mah`.

### 6.5 Brak ratchetingu

Test obowiązkowy: pięć kolejnych cykli włącz/wyłącz bez ładowania i przy tym
samym napięciu nie może podnosić SOC. Warunek na `delta_mv` oraz zapis bieżącego
napięcia przy kontrolowanym wyłączeniu mają sprawić, że tylko pierwszy realny
wzrost napięcia może być kandydatem korekty.

---

## 7. Ładowanie przy włączonym rowerze

### 7.1 Dlaczego osobna maszyna stanów jest konieczna

`AUTO_OFF_MINUTES` ma wartość domyślną 10, ale HMI komendą `0x6303` może ustawić
`auto_off_minutes = 0`, czyli wyłączyć auto-off. Rower może więc pozostać
włączony przez wiele godzin ładowania. Sam mechanizm „sprawdź po następnym
uruchomieniu” nie pokrywa tego przypadku.

Ponieważ nie mamy wiarygodnego prądu ładowania, sam chwilowy wzrost napięcia
nie wystarcza: taki sam kierunek występuje po zdjęciu obciążenia. Potrzebny jest
długi wzorzec czasowy i jawny stan.

### 7.2 Stany

```c
typedef enum {
    SOC_CHARGE_DISCHARGE = 0,
    SOC_CHARGE_REST_RELAX,
    SOC_CHARGE_REST_OBSERVE,
    SOC_CHARGE_CANDIDATE,
    SOC_CHARGE_CONFIRMED,
    SOC_CHARGE_POST_SETTLE
} soc_charge_state_t;
```

`FULL` pozostaje osobną flagą `soc_full_anchor`, ponieważ pełna bateria może
zostać rozpoznana zarówno przy starcie, jak i w czasie ładowania online.

### 7.3 Warunek prawdziwego bezruchu

Wejście do obserwacji wymaga jednocześnie:

```text
MS.Speedx100 == 0
MS.cadence == 0
MS.i_q_setpoint == 0
brak Walk Assist
brak kalibracji
brak aktywnego sterowania mostkiem
abs(MS.Battery_Current) < SOC_CHARGE_IDLE_CURRENT_MA
```

Utrata któregokolwiek warunku podczas `REST_RELAX`, `REST_OBSERVE` albo
`CANDIDATE` wraca do `DISCHARGE` i zeruje okna detektora.

### 7.4 Relaksacja i okna napięcia

Początkowe, celowo konserwatywne wartości do trybu diagnostycznego:

```c
#define SOC_CHARGE_RELAX_GUARD_S          120U
#define SOC_CHARGE_WINDOW_S                60U
#define SOC_CHARGE_WINDOWS_REQUIRED         3U
#define SOC_CHARGE_MIN_RISE_PER_WINDOW_MV  50U
#define SOC_CHARGE_MIN_TOTAL_RISE_MV       150U
#define SOC_CHARGE_END_DROP_MV             200U
#define SOC_CHARGE_MAX_RISE_PCT_PER_MIN   0.5f
```

Algorytm:

1. `REST_RELAX`: przez pierwsze 120 s nie wolno zwiększyć SOC. Zbieramy napięcie,
   ale traktujemy wzrost jako naturalne odbicie.
2. `REST_OBSERVE`: liczymy średnią napięcia w kolejnych oknach 60 s.
3. `CANDIDATE`: kolejne średnie muszą rosnąć. Potwierdzenie wymaga trzech
   kolejnych okien, każdego wyższego o co najmniej 50 mV, oraz łącznego wzrostu
   co najmniej 150 mV.
4. Pojedynczy spadek, ruch albo zadanie silnika kasuje kandydata.
5. Progi są początkowe. Przed włączeniem wpływu na SOC detektor ma przepracować
   sprzętowe sesje w trybie shadow/diagnostic-only.

Te wartości dotyczą całego pakietu, nie pojedynczego ogniwa.

### 7.5 Tryb shadow obowiązkowy przed aktywacją

Pierwszy build FW-101 ma wykrywać i raportować stany, ale nie zmieniać SOC na
podstawie detektora online. Należy zebrać co najmniej:

- zwykły postój po mocnej jeździe bez ładowarki;
- zwykły postój po lekkiej jeździe;
- rozpoczęcie ładowania od razu po jeździe;
- rozpoczęcie ładowania po długim postoju;
- końcową fazę CV przy pełnej baterii;
- odłączenie ładowarki.

Dopiero po porównaniu nachylenia i całkowitego wzrostu napięcia należy zatwierdzić
lub zmienić progi i wyłączyć shadow mode. Nie wolno stroić ich tak, aby jeden
konkretny log „ledwo przechodził”.

### 7.6 Zachowanie po `CHARGING_CONFIRMED`

Po potwierdzeniu ładowania:

1. Ujemny prąd nadal nie jest integrowany — nie znamy jego skali ani toru.
2. Mały dodatni prąd jałowy kontrolera nie powinien obniżać SOC podczas
   potwierdzonego ładowania i pełnego bezruchu.
3. Raz na minutę wolno przesunąć `soc_real` wyłącznie w górę, w stronę
   długookresowego `soc_voltage`, maksymalnie o
   `SOC_CHARGE_MAX_RISE_PCT_PER_MIN`.
4. Wynik musi być monotoniczny w górę w stanie ładowania; chwilowy spadek
   napięcia nie zmniejsza SOC.
5. Ponieważ napięcie pod ładowarką nie jest prawdziwym OCV, jest to estymata.
   Limit tempa chroni przed natychmiastowym skokiem do zawyżonego wyniku.
6. Jeżeli napięcie osiągnie istniejący próg pełnego pakietu i spełni warunek
   stabilności, ustawić dokładnie 100% i `soc_full_anchor = 1`.

Początkowy limit 0,5%/min odpowiada maksymalnie około 4,2 A dla baterii 14 Ah.
Nie oznacza, że ładowarka ma taki prąd; jest tylko ograniczeniem bezpieczeństwa
estymatora napięciowego.

### 7.7 Koniec ładowania

Nie wolno uznać płaskiego napięcia za koniec ładowania, bo faza CV może trwać
długo. Stan `CHARGING_CONFIRMED` kończy:

- rozpoczęcie jazdy/pedałowania — natychmiastowy powrót do `DISCHARGE`;
- wyłączenie kontrolera;
- albo spadek napięcia o co najmniej `SOC_CHARGE_END_DROP_MV` od niedawnego
  maksimum, po którym następuje `POST_SETTLE`.

`POST_SETTLE` czeka na stabilne napięcie bez ładowarki i wykonuje jedną łagodną
korektę tą samą funkcją, której używa start. Jeżeli użytkownik ruszy przed
ustabilizowaniem napięcia, korekta zostaje pominięta, a jazda zaczyna się od
konserwatywnego wyniku wypracowanego podczas ładowania.

### 7.8 Wyłączenie w trakcie ładowania

Jeżeli rower zostanie wyłączony ręcznie podczas ładowania:

- zapisuje bieżący estymowany SOC i napięcie;
- ładowanie może trwać dalej przy wyłączonym kontrolerze;
- następny start wykonuje normalną korektę startową;
- pełne napięcie nadal ma pierwszeństwo i ustawia 100%.

Jeżeli auto-off jest aktywny, zachowanie jest takie samo po automatycznym
wyłączeniu. Jeżeli auto-off jest wyłączony, maszyna stanów online aktualizuje
SOC bez restartu.

---

## 8. Napięcie pełnego pakietu — istniejąca zmienna, nie nowa

### 8.1 Stan implementacji

Istnieją:

```c
uint16_t MP.soc_full_magic;
uint16_t MP.soc_full_pack_10mv;
```

- `soc_full_pack_10mv` przechowuje napięcie całego pakietu w jednostkach 10 mV;
- `4587` oznacza `45,87 V`;
- zapis przychodzi z Canable przez `0x602B`;
- odczyt wraca przez `0x6028 v2`;
- parametr jest zapisywany w rekordzie `MotorParams_t` na postoju;
- zakres walidacji firmware wynosi 20–90 V.

Nie dodawać `soc_full_voltage`, liczby ogniw ani drugiego progu.

### 8.2 Wykryta niespójność

Dokument FW-018 opisuje fallback 4,17 V/ogniwo, lecz aktualny kod
`InitEEPROM()` ustawia:

```c
MP->soc_full_magic = 0;
MP->soc_full_pack_10mv = 0;
```

Ostatni dostępny odczyt `controller_system` również raportował
`soc_full_pack_mv: 0`, czyli funkcja była nieaktywna.

FW-101 ma uznać aktualny kod za prawdę: próg jest konfigurowalny i pozostaje
nieaktywny, dopóki nie zostanie prawidłowo zapisany. Nie wolno po cichu
hardkodować wartości dla innych pakietów.

Dla obecnego roweru 11S wartością referencyjną z FW-018 i istniejących komentarzy
jest **4587 = 45,87 V**. Przed testem SOC developer/operator ma:

1. ustawić 45,87 V w Canable;
2. poczekać na zapis na pełnym postoju;
3. odczytać `0x6028 v2` i potwierdzić 4587;
4. wykonać restart i ponownie potwierdzić 4587;
5. dopiero wtedy testować zakotwiczenie 100%.

Jeżeli rzeczywisty pomiar pełnej, zbalansowanej baterii różni się od 45,87 V,
wartość zmienia się w Canable — kod pozostaje bez zmian.

### 8.3 Priorytet pełnego progu

Ważny i stabilnie przekroczony `soc_full_pack_10mv` ma pierwszeństwo przed:

- korektą startową,
- limitem tempa ładowania online,
- wynikiem tabeli napięcie→SOC.

Wtedy:

```c
remaining_mah = battery_capacity_estimated_mah;
soc_real = 100.0f;
soc_display = 100.0f;
MS.SOC = 100;
soc_full_anchor = 1;
```

Anchor zwalnia się dopiero po policzonym zużyciu
`SOC_FULL_RELEASE_FRAC` pojemności.

---

## 9. Zapis stanu SOC — wyłącznie przy kontrolowanym wyłączeniu

### 9.1 Co pozostaje

Pozostaje dedykowana strona SOC:

- adres `0x0803F800`;
- slot 32 B;
- 64 sloty;
- numer sekwencji i CRC32;
- erase dopiero po zapełnieniu strony.

To nie jest rekord parametrów `MotorParams_t`. Potocznie można mówić „EEPROM”,
ale technicznie jest to osobna, wear-levelowana strona flash.

### 9.2 Co usunąć

Z `soc_update()` usunąć:

```c
if(fabsf(MS.soc_real - soc_last_saved) >= SOC_SAVE_DELTA &&
   soc_save_seconds >= SAVE_MIN_INTERVAL_S)
    soc_state_save();
```

Usunąć nieużywane po zmianie:

- `soc_save_seconds`,
- `soc_last_saved`, jeśli nie będzie potrzebne wyłącznie diagnostycznie,
- `SOC_SAVE_DELTA`,
- `SAVE_MIN_INTERVAL_S`.

### 9.3 Jedyne dozwolone miejsce zapisu

`soc_state_save()` jest wołane przez `power_off_controller()`, zabezpieczone
`shutdown_saved`. Wszystkie kontrolowane ścieżki muszą kończyć w tej funkcji:

- długie przytrzymanie przycisku;
- auto-off;
- watchdog komunikacji przy postoju.

Nie duplikować kodu FMC w poszczególnych ścieżkach.

### 9.4 Walidacja odczytanego slotu

Poza CRC należy sprawdzić:

- `0 <= soc_real_x10 <= 1000`;
- skończone `remaining_mah`;
- `0 <= remaining_mah <= 1.05 * capacity_estimated`;
- pojemność w zakresie 1000–60000 mAh;
- napięcie w fizycznym zakresie konfiguracji.

Uszkodzony semantycznie slot ma zostać pominięty tak samo jak slot ze złym CRC.

### 9.5 Świadomy kompromis

Nagłe odłączenie baterii, zadziałanie BMS albo utrata zasilania przed
`power_off_controller()` oznacza utratę zmian SOC z bieżącej sesji. To jest
zaakceptowany koszt decyzji „zapis tylko przy wyłączeniu”. Nie wolno ukrywać go
ponownym zapisem podczas jazdy.

---

## 10. Uczenie pojemności — nie może zależeć od usuniętej korekty

Obecnie początek cyklu jest uzależniony od:

```c
MS.soc_real > 92.0f && rest_seconds >= REST_TIME_S
```

Po FW-101 `rest_seconds` nie oznacza już „można poprawić SOC z napięcia”.
Uczenie pojemności trzeba przepiąć:

1. Początek pełnego cyklu wyłącznie po potwierdzonym `soc_full_anchor`:

   ```text
   cycle_start_soc = 100.0
   cycle_discharge_mah = 0
   ```

2. `cycle_discharge_mah` sumuje tylko dodatnie `dmah`.
3. Przy zejściu poniżej obecnego dolnego progu można policzyć zmierzoną
   pojemność jak dotąd.
4. Po zmianie `battery_capacity_estimated_mah` zachować bieżący procent SOC:

   ```text
   remaining_mah = soc_real / 100 * new_capacity
   ```

   Zapobiega to skokowi SOC tylko dlatego, że zmienił się mianownik.
5. Nową pojemność zapisze najbliższe kontrolowane wyłączenie.
6. Bez potwierdzonego pełnego początku cyklu nie adaptować pojemności.

Można zachować osobny timer „true rest” dla detektora ładowania i diagnostyki,
ale nie może on sam korygować SOC.

---

## 11. Diagnostyka wymagana przed strojeniem

Nie zmieniać standardowej ramki Bafang `0x3201`, ponieważ jej pole prądu może być
zdefiniowane jako wartość bezwzględna. Do wieloramkowej diagnostyki eVistDrive
dodać, wyłącznie przez dopisanie pól i podniesienie wersji:

- podpisany `Battery_Current` przed clampem;
- surowy ADC prądu i `bat_current_offset`;
- `soc_charge_state`;
- `soc_real_x100`;
- `soc_display_x100`;
- `soc_voltage`;
- `remaining_mah`;
- zapisany SOC i zapisane napięcie;
- średnią/minimum/maksimum bieżącego okna napięcia;
- wzrost względem poprzedniego okna i całkowity wzrost;
- liczbę kolejnych rosnących okien;
- flagi: boot correction, charge candidate, charge confirmed, full anchor;
- przyczynę ostatniego przejścia stanu.

Canable/logger ma zapisywać te pola do CSV. Samo `MS.SOC` jako `uint8_t` oraz
bezwzględny prąd nie wystarczają do strojenia.

Przykładowe przyczyny przejścia:

```c
SOC_REASON_NONE = 0,
SOC_REASON_MOTION,
SOC_REASON_REST_STARTED,
SOC_REASON_RELAX_DONE,
SOC_REASON_RISE_WINDOW,
SOC_REASON_RISE_BROKEN,
SOC_REASON_CHARGE_CONFIRMED,
SOC_REASON_CHARGER_DROP,
SOC_REASON_BOOT_REANCHOR,
SOC_REASON_FULL_THRESHOLD
```

---

## 12. Zalecany podział kodu

Obecny SOC znajduje się w dużym `main.c` i nie ma testów modułowych. Preferowany
podział:

- `inc/soc_estimator.h` — typy stanu, wejście 1 Hz, wyjście i diagnostyka;
- `src/soc_estimator.c` — czysta maszyna stanów, coulomb update, korekta startowa,
  detektor ładowania i inwarianty;
- `main.c` — odczyt ADC, wywołanie modułu, kopiowanie wyniku do `MS` oraz zapis FMC;
- `CAN_Display.c` — tylko serializacja diagnostyki i istniejącego progu pełnego;
- `tests/fw101_soc_charge.js` — model referencyjny i testy strukturalne;
- `tests/host/fw101_soc_charge_host.c` — wykonanie prawdziwego modułu C na sekwencjach.

Moduł nie może wykonywać zapisu flash ani znać CAN. Otrzymuje próbkę raz na sekundę:

```c
typedef struct {
    uint32_t voltage_mv;
    int32_t battery_current_ma;
    uint16_t speed_x100;
    uint8_t cadence_rpm;
    int32_t iq_setpoint;
    bool walk_active;
    bool calibration_active;
} soc_input_t;
```

Zwraca licznik, stan i żądanie zapisu/zakotwiczenia. Decyzję o fizycznym FMC
zachowuje `power_off_controller()`.

Jeżeli ekstrakcja przekroczy bezpieczny zakres jednej karty, dopuszczalne jest
najpierw wydzielenie wyłącznie maszyny stanów napięcia. Nie wolno jednak tworzyć
drugiej kopii obliczeń SOC obok starej.

---

## 13. Kolejność wdrożenia

### Etap A — obserwowalność, bez zmiany wyniku

1. Dodać podpisaną diagnostykę prądu i pełną diagnostykę SOC.
2. Odczytywać `last_voltage_mv` ze slotu.
3. Dodać maszynę stanów online w trybie shadow.
4. Zbudować firmware i zebrać logi sprzętowe opisane w rozdziale 7.5.

### Etap B — monotoniczny SOC

5. Zmienić coulomb counting na discharge-only z małym deadbandem.
6. Usunąć ciągłą korektę OCV i przepisać warunek wyświetlacza.
7. Usunąć okresowy zapis flash.
8. Przepiąć uczenie pojemności na potwierdzony pełny anchor.
9. Uruchomić testy postoju i jazdy — jeszcze bez online charging wpływającego na SOC.

### Etap C — start i pełny próg

10. Przenieść korektę startową za okno stabilizacji.
11. Wdrożyć martwą strefę i płynne wzmocnienie.
12. Potwierdzić zapis/odczyt `soc_full_pack_10mv = 4587` na obecnym rowerze.
13. Potwierdzić pełny anchor i brak ratchetingu po restartach.

### Etap D — ładowanie online

14. Zatwierdzić progi z logów shadow.
15. Zezwolić `CHARGING_CONFIRMED` na ograniczony wzrost SOC.
16. Dodać `POST_SETTLE` i test odłączenia ładowarki.
17. Przetestować zarówno `auto_off_minutes = 10`, jak i `0`.

Każdy etap ma być osobnym, odwracalnym diffem i kończyć się buildem oraz raportem.
Nie łączyć tej karty ze zmianami charakterystyki wspomagania.

---

## 14. Testy automatyczne — obowiązkowa macierz

1. **Coulomb scale:** stałe 10 A przez 3600 s przy 14 Ah zmniejsza SOC o
   71,43%, z tolerancją wynikającą z float.
2. **Ujemny prąd:** dowolnie długi prąd ujemny poza stanem ładowania nie
   zwiększa `remaining_mah`.
3. **Deadband:** szum ±wartość graniczna nie zmienia SOC.
4. **Postój po obciążeniu:** napięcie rośnie przez 20 minut, ale bez spełnienia
   detektora ładowania SOC nie rośnie ani o 0,01%.
5. **Toczenie:** prąd zero i prędkość niezerowa nigdy nie przechodzą do obserwacji
   ładowania.
6. **Mała korekta startowa:** różnica 2% daje zero.
7. **Średnia korekta startowa:** różnica 5% daje około 0,8%, nie pełne 5%.
8. **Duża korekta startowa:** różnica 15% daje pełną korektę.
9. **Niestabilny start:** span napięcia >200 mV nie koryguje SOC.
10. **Brak wzrostu względem zapisu:** wyższy `soc_voltage` przy
    `delta_mv <= deadband` nie zwiększa SOC.
11. **Restart ratchet:** pięć restartów bez ładowania nie podnosi SOC.
12. **Full threshold:** stabilne napięcie równe progowi ustawia 100%; 10 mV
    poniżej nie ustawia.
13. **Naturalna relaksacja:** wzrost tylko w okresie `REST_RELAX` nie potwierdza
    ładowania.
14. **Ładowanie online:** trzy prawidłowe okna potwierdzają ładowanie.
15. **Przerwany kandydat:** jedno okno spadkowe kasuje kandydata.
16. **CV:** płaskie napięcie po potwierdzeniu nie kończy stanu ładowania.
17. **Odłączenie:** odpowiedni spadek przechodzi do `POST_SETTLE`.
18. **Ruch podczas ładowania:** natychmiastowy powrót do `DISCHARGE`.
19. **Limit tempa:** ładowanie online nie zwiększa SOC szybciej niż ustawiony
    limit.
20. **Zapis:** wielogodzinna jazda i zmiana >3% nie woła FMC; każde kontrolowane
    wyłączenie woła dokładnie jeden zapis.
21. **Uszkodzony slot:** poprawny CRC, ale NaN/poza zakresem jest odrzucony.
22. **Capacity learning:** cykl nie startuje od zwykłego `soc_real >92`; startuje
    po pełnym anchorze i zmiana pojemności nie powoduje skoku procentu.
23. **Display monotonic:** bez boot/charge/full `soc_display` nigdy nie rośnie.

Testy strukturalne mają również potwierdzić brak wywołania `soc_state_save()`
z `soc_update()`.

---

## 15. Testy sprzętowe

### 15.1 Przed wdrożeniem wpływu online na SOC

1. Ustawić diagnostyczny build shadow.
2. Naładować/rozładować do średniego SOC, wykonać mocną jazdę i zostawić rower
   włączony na 20 minut bez ładowarki.
3. Powtórzyć z ładowarką podłączoną od razu po zatrzymaniu.
4. Powtórzyć z ładowarką podłączoną po 10 minutach postoju.
5. Zarejestrować pełną fazę końcową i odłączenie ładowarki.
6. Porównać okna napięcia i zatwierdzić progi.

### 15.2 Odbiór właściwy

1. Jazda → postój 20 min bez ładowarki: SOC nie przybywa.
2. Jazda → wyłączenie → ponowne włączenie bez ładowania: brak istotnej korekty.
3. Częściowe ładowanie przy wyłączonym rowerze → start: łagodna korekta zgodna
   z funkcją, nie twardy skok od małej różnicy.
4. Pełne ładowanie → start: 100%.
5. `auto_off_minutes = 10`: kontroler wyłącza się, zapisuje raz; po zakończeniu
   ładowania kolejny start rozpoznaje zmianę.
6. `auto_off_minutes = 0`: kontroler pozostaje włączony, wykrywa ładowanie,
   SOC rośnie tylko po `CHARGING_CONFIRMED`, a próg pełny daje 100%.
7. Odłączenie ładowarki przy włączonym rowerze: brak skoku w dół, po stabilizacji
   jedna końcowa korekta.
8. Pięć restartów bez ładowania: brak przyrostu.
9. Odczyt `0x6028` przed i po restarcie pokazuje 4587 dla obecnego roweru.

---

## 16. Kryteria akceptacji

FW-101 jest zakończone dopiero, gdy:

- zwykły postój nie może zwiększyć `soc_real`, `remaining_mah` ani `soc_display`;
- napięcie nie nadpisuje coulomb countingu poza trzema dozwolonymi ścieżkami;
- działa ładowanie z auto-off aktywnym i wyłączonym;
- mała różnica napięciowa przy starcie wywołuje małą albo zerową korektę;
- pełny próg korzysta z istniejącego `soc_full_pack_10mv` i jest trwale odczytywany;
- nie ma zapisu SOC podczas jazdy;
- nie zmienił się `sizeof(MotorParams_t)` ani standardowe ramki Bafang;
- testy automatyczne przechodzą;
- logi sprzętowe odróżniają relaksację od ładowania na tym konkretnym pakiecie.

---

## 17. Pliki przewidziane do zmiany

**Firmware:**

- `inc/config.h` — nowe stałe, usunięcie stałych starej korekty i okresowego zapisu;
- `inc/main.h` — tylko runtime diagnostics, bez zmiany `MotorParams_t`;
- `inc/soc_estimator.h` — nowy moduł;
- `src/soc_estimator.c` — nowy moduł;
- `src/main.c` — integracja, boot settle, persistence i usunięcie starej korekty;
- `src/CAN_Display.c` — rozszerzona diagnostyka, bez zmiany ramek standardowych;
- `tests/fw101_soc_charge.js`;
- `tests/host/fw101_soc_charge_host.c`.

**Canable/eVistDrive:**

- parser rozszerzonej wersji diagnostyki;
- widok/CSV diagnostyki SOC i podpisanego prądu;
- bez nowego pola pełnego napięcia — korzystać z istniejącego `0x602B/0x6028`.

**Poza zakresem:**

- FOC i sterowanie momentem;
- profile wspomagania;
- format banków;
- zmiana rozmiaru rekordu parametrów;
- nowy sprzętowy pomiar prądu ładowarki;
- obietnica laboratoryjnej dokładności OCV bez zmierzonej krzywej spoczynkowej.

---

## 18. Uwaga dla developera

Nie „naprawiać” braku dokładnego pomiaru ładowania przez ponowne włączenie
dwukierunkowej korekty napięciowej na każdym postoju. To dokładnie mechanizm,
który ta karta usuwa.

Jeżeli detektor online okaże się nierozróżnialny od relaksacji na dostępnych
sygnałach, bezpieczny fallback brzmi:

- podczas włączenia SOC nie rośnie aż do pełnego progu;
- częściowe ładowanie zostaje rozliczone dopiero po wyłączeniu/ponownym starcie
  albo po odłączeniu ładowarki i stabilizacji;
- 100% nadal działa przez `soc_full_pack_10mv`.

Lepszy konserwatywny SOC jest poprawniejszy niż estetyczny wskaźnik, który
sam dopisuje energię po każdym zatrzymaniu.
