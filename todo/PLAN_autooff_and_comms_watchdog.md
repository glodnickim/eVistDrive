# Plan: Auto-off (wyłączanie po bezczynności) + podtrzymanie komunikacji (CAN watchdog)

## Cel (zgłoszony)
1. **Auto-off:** EBICS w ogóle **nie wyłącza się sam**. W HMI ustawia się czas (opcje **OFF … 10 min**).
   Odwzorować fabryczne zachowanie, żeby po bezczynności system się wyłączał.
2. **Podtrzymanie komunikacji (wymagane):** dopóki CAN jest — nic się nie zmienia. Gdy **CAN znika**
   (odłączony/uszkodzony kabel) → **wspomaganie natychmiast na poziom 0**.

---

## Co pokazuje log fake taxi (`log-2026-05-24-15-38-56`) — fakty

### ⚠️ Ograniczenie krótkiego logu 05-24
Log `log-2026-05-24-15-38-56` **kończy się „Stoping sniffer…"** (ręczne zatrzymanie), auto-off tam
= `0A` (10 min) → ten konkretny log nie zawiera samego wyłączenia.

### ✅ ALE auto-off JEST udokumentowany z realnego logu (potwierdzone)
`CAN_PROTOCOL_REFERENCE.md §5-6` powstał z **`log-2026-06-24-17-51-19`**, który złapał **realne
auto-off**: sesja → użytkownik zmienił **9→8 min** (17:51:54) → **rower sam się wyłączył**
18:00:04 = **8 min 10 s później** (dokładność ✓). To potwierdza:
- auto-off to działająca funkcja fabryczna, zakres **OFF…10 min** (typowo ustawione 8-9),
- nośnikiem timeoutu jest **`0x83106303`** (minuty), a **HMI odlicza własną bezczynność i odcina
  zasilanie** gdy `idle ≥ timeout`,
- `0x82F8320F` byte0 zostaje `0x01` do końca → **NIE jest** mechanizmem auto-off.

(Plik `log-2026-06-24-17-51-19` nie leży teraz na dysku — analiza w doc jest wystarczająca;
odzyskać go tylko jeśli trzeba potwierdzić dokładny bajt sygnału idle dla starego wariantu HMI.)

### Ustawienie auto-off
`0x83106303 DLC:1 Data:0A` — **10 minut**, wysyłane **ciągle** przez HMI (dekod: src=3 HMI →
tgt=2 sterownik, WRITE). Czyli HMI trzyma ustawienie (OFF…10 min) i nadaje je do sterownika.
**EBICS już to ODBIERA** → `MP->auto_off_minutes` (CAN_Display.c:392-394). Brakuje reszty mechanizmu.

### Ramki podtrzymujące (keep-alive) — obecne ciągle w logu
| ID | Dane | Rola prawdopodobna |
|---|---|---|
| `0x82FF1200` | `00` | heartbeat (najczęstszy „puls") |
| `0x82F8320F` | `01 00…` | flaga aktywności (b0=0x01 cały czas) |
| `0x82F83000` | licznik +1 co 10 s | licznik sesji/bezczynności |
| `0x83106300` | `05 0B 01 01` | status (b3=aktywny) |
| `0x83106303` | `0A` | ustawienie auto-off (min) |

To są „ramki podtrzymujące", o których mówisz. Gdy znikają → brak komunikacji.

### Mechanizm auto-off — hipoteza robocza (z `CAN_PROTOCOL_REFERENCE.md §6`)
HMI liczy **własny** czas bezczynności (brak prędkości/pedałowania/przycisków) i **sam odcina
zasilanie** gdy `idle ≥ auto_off_minutes`. Sterownik: (a) zna ustawienie z `0x6303`, (b) sygnalizuje
aktywność/bezczynność. EBICS sygnalizuje idle przez **bajt[6] ramki `0x0204`** (fix 0.0091:
`is_idle → byte[6]=0x00`). **Skoro auto-off nie działa u użytkownika → dla JEGO HMI bajt[6]
prawdopodobnie NIE jest wyzwalaczem** (albo HMI to stary wariant patrzący na inny sygnał).

---

## 🔑 KLUCZOWE: EBICS MA WŁASNE WYŁĄCZANIE ZASILANIA (self-power-off)

Sterownik nie musi czekać, aż HMI odetnie zasilanie — **sam steruje swoim zasilaniem**.
Istniejąca ścieżka (przytrzymanie przycisku on/off, main.c:635-643):
```c
if(adc_value[5]<2800) shutoffcounter++; else shutoffcounter=0;   // przycisk on/off
if(shutoffcounter>62){                     // ~2,5 s przytrzymania
    if(!shutdown_saved){ soc_state_save(); shutdown_saved=1; }   // zapis SOC
    timer_primary_output_config(TIMER0,DISABLE);                 // stop PWM
    GPIO_BC(GPIOB) = GPIO_PIN_4;   // DC/DC enable OFF  <-- odcięcie własnego zasilania
    GPIO_BC(GPIOB) = GPIO_PIN_5;   // Display OFF
}
```
**To zmienia całe podejście:** i auto-off, i wyłączenie przy zaniku CAN można zrobić tą samą
sekwencją — sterownik gasi się sam. Nie trzeba zgadywać wyzwalacza po stronie HMI.

**Refaktor:** wydzielić helper `power_off_controller()` (zapis SOC + stop PWM + `GPIO_BC(GPIOB)=PIN_4/PIN_5`)
i wołać go z 3 wyzwalaczy: (1) przycisk (jest), (2) auto-off po bezczynności, (3) utrata CAN.

### Uwaga: dwa różne „piny 4" — nie mylić
- **PA4 (port A, pin 4) = WEJŚCIE ANALOGOWE = przyciski.** `GPIO_MODE_AIN` (main.c:770), czytany
  jako `adc_value[5]` (ADC_CHANNEL_4, main.c:883). Rozpoznaje przycisk po **napięciu** na wspólnej
  linii (drabinka rezystorowa): ~4095 = nic, ~3300 = „dół", ~2400 = on/off (main.c:635).
- **PB4 (port B, pin 4) = WYJŚCIE = DC/DC enable** (self-hold zasilania, main.c:640/777).

---

## 🔎 DO ROZSTRZYGNIĘCIA: jak fabryczne HMI wyzwala wyłączenie? (test PA4)

Hipoteza (bardzo prawdopodobna, do potwierdzenia pomiarem): skoro przyciski to **wspólna linia
analogowa PA4** z HMI/klawiatury, to **HMI po upływie swojego timera bezczynności może
„wcisnąć" on/off elektronicznie** — wysterować PA4 do ~2400, jak przytrzymanie przycisku.
Sterownik czyta `adc_value[5]<2800` przez ~2,5 s → uruchamia sekwencję wyłączenia (zwalnia PB4).
Wtedy auto-off **nie potrzebuje żadnej ramki CAN** — HMI po prostu symuluje przycisk.

### Test rozstrzygający (jedno pomiar)
Przy **fabrycznym** auto-off zaloguj **napięcie/`adc_value[5]` na PA4** (bez dotykania przycisku):
- **PA4 SPADA do ~2400 tuż przed wyłączeniem** → mechanizm = „HMI symuluje on/off". Wtedy EBICS
  teoretycznie już by reagował (czyta PA4); problem byłby tylko w progu/czasie **albo w tym, że
  Twój (stary) HMI tej linii nie steruje**. Naprawa: dopilnować progu `<2800`/czasu, ewentualnie
  obniżyć próg.
- **PA4 ZOSTAJE wysoko (~4095)** → auto-off jest czysto po stronie HMI/CAN (HMI sam tnie zasilanie),
  sterownik nie dostaje sygnału na PA4. Wtedy **konieczny self-power-off** (część A2).

### Wniosek
Niezależnie od wyniku testu — **self-power-off po `s_is_idle` (A2) działa w obu przypadkach** i jest
rekomendowaną drogą dla EBICS. Test PA4 tylko potwierdza, czy dałoby się odwzorować drogę fabryczną
1:1 (HMI→linia przycisku), czy trzeba iść własnym wyłączeniem sterownika.

---

## Plan — CZĘŚĆ A: Auto-off

### KROK A1 — Wyzwalacz już znany z analizy (opcjonalne dogranie)
Mechanizm fabryczny jest udokumentowany (log 17-51-19, wyżej): **HMI odlicza bezczynność i sam
odcina zasilanie**, timeout niesiony przez `0x83106303`. Nie trzeba nic zgadywać. Dograć nowy log
(np. auto-off ustawiony na 1-2 min) **tylko** jeśli chcesz potwierdzić dokładny bajt sygnału idle
dla starego wariantu HMI:
```bash
tail -n 400 <log> | grep -vE 'Stoping|Repeated'      # ostatnie realne ramki przed ciszą
# porównaj 0x82F8320F b0, 0x83106300 b3, 0x0204 b6 tuż przed wyłączeniem
```
Dla EBICS i tak rekomendowane jest **self-power-off** (A2) — niezależne od tego, co robi HMI.

### KROK A2 — Auto-off = self-power-off po bezczynności (ZALECANE, pewne)
EBICS liczy już bezczynność (`idle_ticks`, `auto_off_thresh` z `auto_off_minutes`, main.c:576-582)
i ustawia `s_is_idle`. **Najprościej i najpewniej:** gdy `s_is_idle` utrzyma się (bezczynność
≥ `auto_off_minutes`) → wywołaj `power_off_controller()`. Sterownik gasi się sam, niezależnie od
tego, czy HMI zareaguje. To omija całą niepewność wyzwalacza po stronie HMI.
```c
if(s_is_idle && auto_off_minutes>0) power_off_controller();   // auto-off
```
(Opcjonalnie, dla zgodności/estetyki, DODATKOWO sygnalizować idle do HMI wg wyniku A1 — `0x0204`
byte[6] już jest — ale to nie jest konieczne, skoro sterownik sam się wyłącza.)

### KROK A3 — Uszanować „OFF" i zakres 0…10 min
`auto_off_minutes`: 0/0xFF → traktować jako OFF (bez auto-off), 1…10 → aktywne (main.c:451 już
sanitizuje na 8 — dostosować: 0=OFF zamiast wymuszać 8). Zakres zgodny z HMI (OFF…10).

---

## Plan — CZĘŚĆ B: Podtrzymanie komunikacji (CAN watchdog) — implementowalne OD RAZU

To jest niezależne od A i w pełni określone. Cel: **utrata CAN z HMI → assist=0**.

### KROK B1 — Znacznik ostatniej ramki z HMI
W `processCAN_Rx()` (CAN_Display.c:222), przy odbiorze **dowolnej ramki od HMI**
(np. `0x6300/0x6303/0x6304/0x3203/0xF203/0x62D9` — HMI→sterownik), ustaw globalny
`rx_last_tick = <bieżący tick>` (albo wyzeruj `comm_lost_ticks`). To są ramki, które HMI nadaje
ciągle — ich brak = brak HMI.

### KROK B2 — Dwustopniowa reakcja: assist 0 → self-power-off
W pętli (np. slow-loop @40 ms, main.c:558+), dwa progi:
```c
if(++comm_lost_ticks > COMM_CUT_TICKS){            // stopień 1: ~1-2 s bez ramki HMI
    MS.assist_level = 0;                            // wspomaganie na 0
    MS.i_q_setpoint = 0;                            // natychmiast zeruj prąd (fail-safe)
    comm_lost = 1;
}
if(comm_lost_ticks > COMM_OFF_TICKS){              // stopień 2: kilka s dalej brak HMI
    power_off_controller();                         // sterownik wyłącza się sam
}
// reset comm_lost_ticks następuje w processCAN_Rx przy każdej ramce HMI
```
- **Stopień 1 (assist 0)** — natychmiastowy fail-safe, żeby motor nie ciągnął po urwaniu kabla.
- **Stopień 2 (wyłączenie)** — po kilku sekundach dalszego braku HMI sterownik gasi się sam
  (to jest Twoje wymaganie: „przez kilka sekund nie ma komunikacji → sterownik się wyłącza").
Gdy ramki wrócą przed stopniem 2 → `comm_lost=0`; **assist NIE wraca sam** (bezpieczeństwo) —
poziom ustawiany ponownie z HMI (lub wg `comm_restore_mode`).

### KROK B3 — Stałe
- `COMM_CUT_TICKS` — np. `50` (2 s @40 ms): utrata CAN → assist 0.
- `COMM_OFF_TICKS` — np. `125` (5 s @40 ms): dalszy brak → self-power-off.
Krótko, by realnie chroniło przy urwanym kablu, ale nie fałszować przy chwilowym zaniku.
HMI ma E30 przy ~podobnym czasie — dobrać spójnie.

> Uwaga bezpieczeństwa: to jest funkcja **fail-safe**. Cięcie musi zerować `i_q_setpoint` natychmiast,
> nie tylko `assist_level` (inaczej bieżący prąd trzyma się do następnego przeliczenia).

---

## Nowe/konfigurowalne zmienne (CAN w przyszłości)
| Zmienna | Rola |
|---|---|
| `auto_off_minutes` (jest, Para) | 0=OFF … 10 min → self-power-off |
| `COMM_CUT_TICKS` | próg utraty CAN → assist 0 |
| `COMM_OFF_TICKS` | próg utraty CAN → self-power-off |
| (opcja) `comm_restore_mode` | czy assist wraca sam po powrocie CAN, czy user ustawia |

---

## Weryfikacja
### Auto-off
1. Ustaw w HMI 1-2 min, zostaw rower bezczynny → po zadanym czasie system gaśnie.
2. Log CAN: potwierdź, że sygnał z A1/A2 zmienia się przy wejściu w idle i że magistrala milknie.
3. Ustaw OFF → brak auto-off (jeździ bez wyłączania).

### Comms watchdog
1. W trakcie jazdy/postoju **odłącz kabel CAN** → wspomaganie **natychmiast 0**, motor przestaje ciągnąć.
2. Podłącz z powrotem → brak samoczynnego skoku mocy; poziom ustawiany z HMI (lub wg `comm_restore_mode`).
3. Chwilowy mikro-zanik < `COMM_TIMEOUT` → brak fałszywego cięcia.
4. Potwierdź, że `i_q_setpoint` = 0 w momencie utraty (log UART main.c:563 / CAN 0x0203).

## Ryzyka / uwagi
- **Trigger auto-off niepotwierdzony** bez logu z KROK A1 — nie implementować „na ślepo"; fallback A2-kandydat 4 (milknięcie keep-alive) jest najbezpieczniejszy jeśli brak jednoznacznego bajtu.
- Comm watchdog za krótki → fałszywe cięcia na zakłóceniach; za długi → rower ciągnie po urwaniu kabla. Stroić `COMM_TIMEOUT_TICKS`.
- Rozróżnić „brak HMI" od „HMI jest, ale cisza w danym oknie" — bazować na ramkach nadawanych CIĄGLE przez HMI (np. 0x82FF1200/0x6300), nie na rzadkich.
