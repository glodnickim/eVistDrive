# Przewodnik jazdy RC1 — co robi każdy poziom wspomagania i na co patrzeć

```text
DOTYCZY:   wsad 0.645 (DIAG, do jazd z logiem) i 0.644 (NORMAL), tag assist-v3-rc1
PORÓWNUJ:  z 0.638 / 0.639 (baseline 25df554) — Twoje obecne oprogramowanie
ŹRÓDŁO:    wartości domyślne z kodu RC1 (src/assist_v3_config.c, src/g53_port_chain.c); wyniki z symulacji [SIM],
           nie z roweru — jazda ma je potwierdzić albo obalić
```

## 1. Najważniejsze w jednym akapicie

W RC1 **siła wspomagania na każdym poziomie jest taka sama jak w 0.638** — ten sam „mnożnik”, to samo tempo
narastania przy mocnym nacisku i ten sam limit mocy. Zmieniło się trzy rzeczy, wspólne dla wszystkich poziomów:

1. **Szybkie odpuszczanie.** Gdy zdejmujesz nacisk, a dalej kręcisz, wspomaganie schodzi szybko (ułamek sekundy),
   zamiast „ciągnąć” jeszcze 1–3 s jak w 0.638. Tempo zejścia zależy od poziomu (tabela niżej).
2. **Bez pulsowania przy wolnej kadencji.** Przy 20–30 obr/min martwy punkt korby (pedały góra–dół) nie powoduje
   już przygasania wspomagania przy każdym suwie.
3. **Carry (podtrzymanie po zatrzymaniu pedałów).** Na stromym, wolnym, mocno obciążonym podjeździe, gdy po mocnym
   nacisku nagle przestaniesz pedałować (np. korzeń, stopień), silnik może jeszcze chwilę pchać. Ma limity czasu
   i drogi i różną siłę na poziomach. W RC1 progi są wstępne — **obserwuj, nie licz na to**.

Pełne charaktery trybów (ECO z niższą mocą, TRAIL z mocniejszym wsparciem przy wolnej kadencji itd.) zaprojektowano,
ale **nie są włączone w RC1** — przyjdą po tej jeździe (Milestone E).

## 2. Poziomy

| Poziom HMI | Charakter w RC1 | Wsparcie (jak 0.638) | Narastanie przy ataku (jak 0.638) | Odpuszczanie (NOWE) | Carry (NOWE) |
|---|---|---|---|---|---|
| **1 — ECO** | spokojny | najsłabsze, liniowe (ratio 95 %) | najwolniejsze (pełna skala ok. 1,5 s) | najłagodniejsze: pełne zejście ok. 0,42 s | słabe: do 0,24 s i 0,3 m |
| **2 — TRAIL (dawniej TOUR)** | naturalny | średnie, liniowe (215 %) | ok. 1,2 s pełnej skali | ok. 0,33 s | długie: do 0,84 s i 1,0 m |
| **3 — SPORT** | szybki | mocne, liniowe (310 %) | ok. 0,9 s | szybkie: ok. 0,24 s | średnie: do 0,6 s i 0,7 m |
| **4 — SPORT+** | dynamiczny, progresywny | rośnie z siłą nacisku (AUTO z G5300): przy lekkim pedałowaniu słabsze niż SPORT, przy mocnym do 525 % | najszybsze (ok. 0,26 s pełnej skali) | najszybsze: ok. 0,2 s | najmocniejsze: do 0,96 s i 1,2 m |
| **5 — BOOST** | najmocniejszy stały | najsilniejsze, liniowe (525 %) | najszybsze (jak SPORT+) | szybkie: ok. 0,24 s | średnie: do 0,6 s i 0,7 m |

Czasy odpuszczania to zejście z pełnego wspomagania; od typowego poziomu jazdy będzie krócej. Wykrycie, że
naprawdę odpuściłeś, zajmuje dodatkowo ok. 1/16–1/4 obrotu korby [SIM: 20–90° korby].

## 3. Co ma się dziać — wspólne dla wszystkich poziomów

| Sytuacja | Oczekiwane w RC1 | W 0.638 było |
|---|---|---|
| Równe pedałowanie 60–80 obr/min | to samo wsparcie co w 0.638 (±5 %), równiejsze | lekkie falowanie |
| Wolny podjazd 20–30 obr/min | brak przygasania w martwym punkcie | przygasa co suw |
| Szybka kadencja 110–130 obr/min | równe wsparcie | równe |
| Zdejmujesz nacisk, kręcisz dalej | wspomaganie schodzi w ułamku sekundy | ciągnie 1–3 s |
| Stopniowo zmniejszasz nacisk | wspomaganie płynnie maleje razem z naciskiem | z opóźnieniem 1–4 s |
| Mocny atak (wstajesz i pchasz) | narasta jak w 0.638 | — |
| Przestajesz pedałować, stopa bez nacisku | koniec wspomagania jak w 0.638 (ok. 0,1 s) | — |
| Przestajesz pedałować, stopa dociska pedał | łagodne wygaszenie ok. 0,7–1 s, jak w 0.638 | — |
| Hamulec | odcięcie w ok. 0,2 s, jak zawsze | — |
| Pedałowanie wstecz | wygaszenie w ok. 0,03–0,06 s | podobnie |
| Ruszanie po zatrzymaniu | bez szarpnięcia i bez „dziury” | — |
| Zmiana przełożenia z odpuszczeniem | krótki spadek, bez szarpnięcia po powrocie nacisku | — |

**Carry — kiedy może zadziałać:** tylko przy prędkości ok. 1,5–10 km/h, po wyraźnie mocnym nacisku i dużym obciążeniu
silnika, gdy rower nie przyspiesza. **Nigdy:** po hamulcu, po cofnięciu korby, przy toczeniu się po płaskim, na
przełamaniu wzniesienia (gdy rower przyspiesza) ani dłużej niż limit poziomu.

## 4. Na co patrzeć na każdym poziomie (wpisuj: lepiej / tak samo / gorzej niż 0.638)

- **ECO:** czy szybkie odpuszczanie nie jest za ostre; czy brak falowania na wolnym podjeździe; carry ledwo wyczuwalne
  albo wcale.
- **TRAIL (poziom 2):** naturalność odpuszczania przy zmianie nacisku; carry na technicznym podjeździe — czy pomaga, czy zaskakuje.
- **SPORT:** start od zera (czy narasta jak w 0.638); szybkie odpuszczanie przy zmianach rytmu; poziom przy 100+ obr/min.
- **SPORT+:** lekkie pedałowanie = mało wsparcia (tak było też w 0.638), mocny atak = dużo; czy przejście jest płynne;
  carry najmocniejsze — czy nie „wypycha” po przeszkodzie.
- **BOOST:** pełna moc; czy odpuszczanie przy dużej mocy jest kontrolowane, bez szarpnięcia.

**Natychmiast przerwij i zgłoś (hamulec, poziom 0 albo wyłączenie):** wspomaganie, które nie schodzi po zdjęciu
nacisku; pchanie po hamowaniu albo cofnięciu korby; carry dłuższe niż ok. 1 s lub dalej niż ok. 1,5 m; szarpnięcia.
Powrót do znanego zachowania: wgraj 0.638.

## 5. Przed pierwszą jazdą

Wykonaj kroki stanowiskowe z `RIDE_TEST_PLAN.md` §0. Zwłaszcza krok 5: pomiar obciążenia procesora na 0.645.
**Bez spełnienia tego kroku nie jeździmy.** Krok 6: sprawdź w CANable, że silnik V3 jest aktywny po pierwszych
obrotach korby.

## 6. Notatki techniczne (dla programu, nie dla jazdy)

- Charakterystyka statyczna = G5300 (D-004): sloty HMI 1..5 → 2/4/6/8/9, ratio 95/215/310/525 (AUTO)/525,
  acceleration 4/5/6/8/8 (D7EC D28: pełna skala 1,54/1,22/0,90/0,26 s; rampa V3 = wolniejsza z D28 i BDE8 3,5 Iq/ms), moc P1 100 %.
- Odpuszczanie: Response z profilu trybu (40/60/80/90/80) → pełna skala 600 − 4,5·Response ms (0,65·P).
- Carry: siła = makro Carry (20/70/50/80/50 %), czas = makro × 12 ms, droga = makro × 0,15 dm; twarde limity
  1200 ms / 1,5 m; próg dopuszczenia 4500 CLU (D-043, kandydat).
- Max Torque / Max Power z profili (np. ECO 80 % / 350 W) nie są stosowane w RC1 (aktywacja w Milestone E).
- Wszystkie liczby pochodzą z symulacji (level 3 w pełnej macierzy, pozostałe poziomy z tego samego kodu); jazda je
  weryfikuje.
