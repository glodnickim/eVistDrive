# Karta zmiany FW-025 — szybsze cięcie wspomagania po zaprzestaniu pedałowania (ride core)

- **Data:** 2026-07-24
- **Status:** POTWIERDZONE JAZDĄ (2026-07-27) — zostaje jako OK na razie
  (`PAS_STOP_TICKS=800`=200 ms, `config.h:323`, obecne w każdym buildzie od `0.0195`
  po `0.0206`). Zagadka „zaleganie kilkanaście sekund" z sekcji 6b poniżej okazała
  się być OSOBNYM błędem (PI windup) — rozwiązanym przez [[FW-028_PI_WINDUP_FIX]]
  (`0.0199`), nie przez to okno. Etap 2 (F1-F4) nie był potrzebny.
- **Zakres:** reakcja toru ride core na zatrzymanie pedałowania. Miejsce poprawki
  zależne od pomiaru (warstwa czujnika PAS lub rampa) — patrz Etap 2.
- **Powiązane:** [[FW-024_PAS_DIRECTION_LATCH]] (objaw 1, kierunek), diagnostyka
  `0x6029`, [[pas-pulses-per-rev]].

---

## 1. Prostym językiem

Drugi objaw zgłoszony przez właściciela: w trybie ride core po zaprzestaniu
pedałowania moc schodzi zbyt wolno — wspomaganie „ciągnie" jeszcze chwilę po
tym, jak przestajesz kręcić. W Legacy schodzi naturalnie.

FW-024 naprawiło kierunek (cofanie). To jest osobny objaw — samo zatrzymanie,
bez cofania.

---

## 2. Tor sygnału po zatrzymaniu (co wiemy z kodu)

1. Przestajesz kręcić → brak przejść kwadratury → `pas_idle_ticks` rośnie.
2. Przez **500 ms** (`PAS_STOP_TICKS=2000` @4kHz, `config.h:314`) kadencja jest
   TRZYMANA na ostatniej wartości, a `forward_pedaling` pozostaje prawdziwe
   (`main.c:1572-1573`). W tym oknie tryb ride core dalej liczy wspomaganie.
3. Po 500 ms: `MS.cadence=0`, `fwd_run=0`. Tryb ride core zwraca `supported=false`
   (`assist_modes.c:368-373`, `cadence_for_assist==0`) → `iq_target=0`.
4. `iq_target=0` schodzi przez rampę (`assist_dynamics.c`, tryb czasowy):
   `dn_ticks` = MIN(rampa wg prędkości, rampa wg kadencji). Przy zatrzymaniu z
   ruchu prędkość jeszcze niezerowa → rampa raczej szybka; przy niskiej prędkości
   → wolna (do `IQ_RAMP_DOWN_SLOW_TICKS=4000` = 1 s).
5. `safety_cut` przy zatrzymaniu NIE jest ustawiony (to nie cofanie), więc obejście
   rampy z FW-024 tu nie działa.

Dodatkowy czynnik: wspomaganie w trybie mocy zależy od momentu (`torque × cadence`).
Jeśli zdejmiesz nacisk z pedału, moment spada i wspomaganie może zejść szybciej
niezależnie od okna 500 ms. Jeśli trzymasz nogi na pedałach (moment resztkowy),
wspomaganie ciągnie przez całe okno. To trzeba rozdzielić pomiarem.

## 3. Dlaczego Legacy schodzi lepiej

Legacy (`main.c:2838-2840`) ma moc ∝ kadencja (`cadence^helper`) ORAZ jawny
zjazd `map_rezi(..., torque_counter, ...)`. Gdy przestajesz kręcić, oba człony
ściągają moc w dół płynnie, jeszcze przed wspólną rampą. Tor ride core nie ma
odpowiednika `map_rezi` — polega na twardym oknie 500 ms + rampie.

---

## 4. Etap 1 — POMIAR (na 0.0193, ta sama sesja co test FW-024)

Cel: rozstrzygnąć, gdzie siedzi opóźnienie — w oknie 500 ms, w rampie, czy w
zwłoce momentu. Narzędzie: podgląd `0x6029` (`iq_req`, `iq_set`, `pas_idle_ms`,
`fast_pressure`, `speed`). Rower na stojaku, silnik ride core.

Dwa warianty, każdy kilka powtórzeń:
- **A: stopa zdjęta** — pedałuj z naciskiem, potem gwałtownie puść pedał i przestań
  kręcić. Zmierz czas od ostatniego ruchu korby do `iq_set≈0`.
- **B: stopa oparta** — pedałuj, potem przestań kręcić, ale zostaw nacisk resztkowy
  na pedale. Zmierz to samo.

Odczyt:
- jeśli w wariancie A moc znika szybko, a w B ciągnie ~0,5 s → dominuje okno
  `PAS_STOP_TICKS` (kadencja trzymana). Fiks w WARSTWIE CZUJNIKA.
- jeśli w obu ciągnie długo mimo `iq_req` już zero, a `iq_set` opada powoli →
  dominuje RAMPA. Fiks w warstwie silnika (poza wąskim zakresem PAS).
- jeśli `fast_pressure` (moment) opada powoli po puszczeniu pedału → zwłoka
  filtra momentu.

---

## 5. Etap 2 — POPRAWKA (wariant wg wyniku Etapu 1)

Kandydaci, do wyboru po pomiarze (nie łączyć na ślepo):

**F1 — adaptacyjne okno stopu (warstwa czujnika, faworyt jeśli dominuje 500 ms).**
Zamiast stałych 500 ms zrobić `PAS_STOP_TICKS` zależne od prędkości/kadencji, jak
w ride core (`.external/.../main.h:100-101`, `motor.c:976`): krótkie przy ruchu
(~kilkadziesiąt ms), dłuższe na postoju (żeby nie fałszować „stop" między
impulsami przy bardzo niskiej kadencji). Wspólne dla obu silników — sprawdzić
regresję Legacy przy niskiej kadencji.

**F2 — bramka spadku momentu w torze ride core (jeśli moc ciągnie mimo braku nacisku).**
Gdy `torque_on_crank` spadnie poniżej progu zwolnienia (`750+TQ_GATE_MIN`),
szybciej ściągać `iq_target` w ścieżce ride core — odpowiednik zatrzasku/zwolnienia z
Legacy, ale bez `map_rezi`.

**F3 — miękki zjazd kadencji zamiast twardego trzymania.** W oknie stopu zamiast
trzymać ostatnią kadencję do skoku na 0, liniowo obniżać ją do 0 — moc schodzi
płynnie przez naturalną zależność trybu od kadencji. Bliskie zachowaniu Legacy.

**F4 — Release per profil (`release_ms`).** Istnieje już mechanizm kontrolowanego
zjazdu (`assist_dynamics.c:61-70`), domyślnie `release_ms=0`. To narzędzie do
KSZTAŁTOWANIA zjazdu, nie do przyspieszania — użyć tylko, jeśli chcemy zjazd
płynny-ale-krótki zamiast twardego cutu.

Rekomendacja wstępna (do potwierdzenia pomiarem): **F1**, bo adresuje korzeń w
warstwie czujnika i powiela sprawdzony wzorzec ride core; ewentualnie F1+F2, jeśli
pomiar pokaże, że sam moment też zwleka.

---

## 6. Weryfikacja

1. `0x6029`: czas od ostatniego ruchu korby do `iq_set≈0` skrócony względem
   pomiaru bazowego z Etapu 1 (cel: rząd ~150–250 ms przy ruchu, bez fałszywego
   stopu na niskiej kadencji).
2. Jazda: po zaprzestaniu pedałowania wspomaganie schodzi szybko i przewidywalnie.
3. Regresja Legacy: brak fałszywego „stop" przy powolnym pedałowaniu; zachowanie
   jak dotąd.
4. Brak nawrotu objawu 1 (cofanie dalej tnie natychmiast — FW-024).

---

## 6a. Build

- `0.0195_M820_BL820.bin`, 81928 B,
  SHA-256 `91D2D7EED442D44A9A968861A29FF5043EDB539C4981FDBB33AD031AFC17CD7A`.
- Zmiana: `inc/config.h` `PAS_STOP_TICKS` 2000 → 800 (500 ms → 200 ms).
- Zawiera też FW-024 + FW-024b (kierunek, potwierdzony na 0.0194).
- Potwierdzenie skuteczności: JAZDA (odczucie), bez komputera.

## 6b. ANALIZA — zaleganie „kilkanaście sekund" (10-19 s), ride core (2026-07-24)

Właściciel zgłosił z jazdy: w ride core po zaprzestaniu pedałowania silnik napędza
przez **kilkanaście sekund**; cofnięcie korbą **w jeden obrót** ucina; objaw
**powtarzalny**. Uwaga właściciela: pamięć „było wcześniej / przed zmianami" jest
NIEPEWNA — nie traktować jako fakt.

**Log tego NIE pokazał** — połączenie USB urwało się po ruszeniu. 411 próbek w
15 min (oczek. ~2000), `iq_set` nigdy > 92 (brak realnego obciążenia), koniec =
rower nieruchomy. Dane są sprzed jazdy (biurko), realnego zdarzenia brak.

### Wykluczone (za krótkie na kilkanaście sekund)
- Rampa opadania (`assist_dynamics.c`): max ~1 s (ze strojeniem do 5 s), przy
  prędkości szybsza. `iq_req≈iq_set` w danych = brak zauważalnego lagu rampy.
- Extended boost / Overrun / `Override_Duration` — w monolicie **Legacy**
  (`main.c:2738+`), w ride core NIE wykonywany.
- Smooth start (`assist_start.c`, do 5 s) — narasta na starcie, zwraca 0 przy
  `safety_cut`/`iq_target<=0`; nie podtrzymuje po stopie.
- Okno kadencji `PAS_STOP_TICKS` (200-500 ms) — o rząd wielkości za krótkie.

### Mechanizm (hipoteza z kodu, NIEZMIERZONA)
ride core twardo bramkuje wspomaganie kadencją korby: `cadence_for_assist==0` →
`supported=false` → `iq_target=0` (`assist_modes.c:368-373`). Moc = `moment ×
kadencja`. Kadencja pochodzi WYŁĄCZNIE z obrotu korby (`MS.cadence`, zerowana
przy `pas_idle_ticks>PAS_STOP_TICKS`, `main.c:1572`). Dopóki korba się kręci,
wspomaganie trwa.

Pętla: korba się kręci → czujnik widzi kadencję → wspomaganie pcha → dokłada
rozpędu → korba się kręci. Na stojaku wolne koło + napęd + korba wirują
kilkanaście sekund po zaprzestaniu nacisku (rozpęd bez obciążenia), ewentualnie
sprzężenie silnik→korba. Cofnięcie odwraca korbę → `safety_cut` (`Backwards_counter
>=4`) → cut od razu (stąd „jeden obrót").

Konsekwencja projektowa: wspomaganie NIE wymaga aktywnego narastającego nacisku —
z nogami opartymi (moment obecny) + wirującą korbą liczy się jako jazda.

### Otwarte / do potwierdzenia
1. Ile z tego to ARTEFAKT STOJAKA (wolne koło wiruje długo). Na drodze z
   obciążeniem i wolnobiegiem może być znacznie krócej.
2. Pomiar rozstrzygający (30 s na stojaku, `0x6029`): podczas zalegania sprawdzić
   `pas_idle_ms`. Niski (korba wiruje) + wysoki prąd → potwierdza pętlę kadencji.
   Wysoki (korba stoi) + prąd nadal płynie → mechanizm inny, szukać dalej.

### Kierunek poprawki (PO potwierdzeniu, nie teraz)
Wspomaganie powinno wymagać INTENCJI jeźdźca, nie samego wirowania korby:
- odciąć/zjechać, gdy moment spadnie poniżej progu zwolnienia (noga zdjęta), albo
- wykryć „coasting" (korba napędzana rozpędem, nie naciskiem) i wygasić,
- wzorzec ride core: wymóg realnego momentu ludzkiego, nie samej rotacji.
Zmiana w warstwie mode/limits, poza wąskim wycinkiem PAS — osobna karta.

## 7. Poza zakresem

- Strojenie samej rampy (`IQ_RAMP_DOWN_*`) — tylko jeśli Etap 1 wskaże rampę jako
  dominującą; wtedy to zmiana warstwy silnika, nie czujnika.
- Objaw Walk Assist (przestrzeliwanie prędkości) — osobny temat.
