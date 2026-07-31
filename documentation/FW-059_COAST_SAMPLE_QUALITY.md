# FW-059 — próbka zera ze środka wybiegu, odrzucanie niespokojnych wybiegów, mniejszy krok

- **Data:** 2026-07-29
- **Status:** ZAIMPLEMENTOWANE, **niezbudowane i nieprzetestowane na sprzęcie**
  (build po stronie właściciela).
- **Zakres:** firmware M820 — `inc/config.h`, `src/torque_input.c`.
  **Bez zmian w protokole, w Canable i w interfejsie modułu.**
- **Powiązane:** `FW-058_COAST_REZERO_RATE.md` — ta karta usuwa przyczynę,
  którą FW-058 tylko ograniczał częstotliwościowo.

---

## 1. Przyczyna, której FW-058 nie tknął

FW-058 zmniejszył **jak często** zerowanie się odpala. Nie ruszył tego, **skąd
bierze się próbka** — a to jest właściwy błąd.

Stary przebieg:

1. jedziesz na wybiegu, uśrednianie chodzi przez cały czas,
2. **zaczynasz naciskać** — nacisk rośnie, ale korba jeszcze nie kliknęła
   impulsu PAS (luz łańcucha, luz w przekładni),
3. warunek wybiegu nadal spełniony, więc średnia wciąga narastający nacisk,
4. pierwszy krok korby kończy wybieg → firmware zapisuje **ten zanieczyszczony
   odczyt** jako nowy spoczynek.

Do tego średnia miała stałą czasową **16 ms**, czyli pamiętała ostatnie ~50 ms —
mimo że wybieg musiał trwać ponad pół sekundy. Próbka pochodziła praktycznie
wyłącznie z tego najgorszego momentu.

Efekt jest kierunkowy, nie losowy: zero wędruje w stronę wstępnego nacisku, więc
**następnym razem trzeba docisnąć mocniej**. O ile — zależy od tego, jak szybko
zacząłeś naciskać, czyli za każdym razem inaczej.

## 2. Zmiana

| | Było | Jest |
|---|---|---|
| moment pobrania próbki | koniec wybiegu | **zamrożona po 0,5 s od początku okna** |
| kontrola spokoju próbki | brak | **rozrzut ≤ 10 mV**, inaczej brak kalibracji |
| maks. korekta na wybieg | 20 mV (~0,74 kg) | **5 mV (~0,19 kg)** |

### `src/torque_input.c`

- `coast_accumulate()` liczy średnią i śledzi min/max **tylko w oknie
  pomiarowym**; gdy `coast_ticks` przekroczy `TQ_RECAL_SETTLE_TICKS`, wartość
  zostaje **zamrożona** w `coast_candidate`, a reszta wybiegu jest ignorowana —
  narastający nacisk przed impulsem PAS nie ma już jak jej ruszyć,
- w tym samym momencie zapada `coast_candidate_stable = (max - min) <=
  TQ_RECAL_STABLE_MV`; niespokojny wybieg (nierówna droga, obijający łańcuch,
  przestawiana stopa) **nie daje kalibracji zamiast dawać złą**,
- `coast_evaluate()` używa `coast_candidate`, a bramkę spokoju stawia **po**
  kontroli wiarygodności bazy — wykrywanie usterki czujnika (Error 25) działa
  dalej w dotychczasowym tempie.

### `inc/config.h`

```c
#define TQ_RECAL_MAX_STEP   5   // było 20; jedna korekta nie może już przewyższyć progu startu (18 mV)
#define TQ_RECAL_STABLE_MV  10  // maks. rozrzut sygnału w oknie pomiarowym
```

Okno pomiarowe zaczyna się dopiero po 5 s bezczynności PAS (FW-058) i trwa 0,5 s,
więc zamrożenie następuje ok. **5,5 s** od zaprzestania pedałowania — na długo
przed tym, zanim znów naciśniesz.

## 3. Weryfikacja wykonana

`node tests/fw058_coast_rezero.js` (plik pokrywa FW-058 i FW-059) → **PASS**.
Nowe przypadki:

- nacisk w ostatnich 0,2 s wybiegu **nie rusza zera** (przed zmianą ruszał),
- wybieg z rozrzutem powyżej progu → brak korekty, policzony jako odrzucony,
- spokojny wybieg z tym samym dryfem → korekta nadal następuje (bramka spokoju
  nie wyłącza kalibracji w praktyce),
- pojedyncza korekta obcięta do 5 mV, czyli **poniżej progu startu 18 mV**.

Właściwość, dla której powstały obie karty — symulowana jazda z wybiegiem co 20 s
i zmiennym zanieczyszczeniem próbki, najgorsza wędrówka zera w oknie 60 s:

| Stan | Wędrówka zera / 60 s |
|---|---|
| przed FW-058 | **26 mV** (próg startu = 18 mV) |
| po samym FW-058 | 19 mV |
| **po FW-058 + FW-059** | **5 mV** |

Dopiero teraz wędrówka zera jest **wyraźnie mniejsza niż próg załączenia**, czyli
przestaje decydować o tym, jak mocno musisz nacisnąć.

Build: **nie uruchamiany** — kompiluje właściciel.

## 4. Ryzyko

Bramka spokoju może na bardzo nierównej nawierzchni odrzucać wszystkie wybiegi,
a przy 5 mV na korektę nadążanie za dryfem termicznym jest wolniejsze. Oba
przypadki ratuje zerowanie na postoju, które FW-058 celowo zostawił bez
ograniczeń — tam sygnał jest spokojny i korekty mogą iść jedna po drugiej.

Gdyby w praktyce okazało się, że zero przestaje nadążać (objaw: wspomaganie
wymaga coraz **mniejszego** nacisku w miarę nagrzewania się czujnika), pierwszym
pokrętłem do poluzowania jest `TQ_RECAL_STABLE_MV`, drugim `TQ_RECAL_MAX_STEP`.

## 5. Test na rowerze

Ten sam co dla FW-058: w zakładce **eVistDrive Torque** obserwować odczyt przy
całkowicie luźnych pedałach. Teraz oczekiwanie jest mocniejsze — baza ma stać
praktycznie nieruchomo w trakcie jazdy, a siła potrzebna do dopięcia wspomagania
ma przestać skakać z próby na próbę.

Jeśli mimo tego siła nadal będzie zmienna, zostaje **drugi, niezależny
mechanizm**: czekanie na impuls PAS przy dopinaniu w jeździe. Rozstrzyga go
pomiar nacisku w chwili załączenia wspomagania z bloku `0x6029` — wartość stała
≈ 18 mV wskazuje na zero, wartość skacząca na PAS.
