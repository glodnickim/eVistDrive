# FW-071 — kopiowanie sekcji zamiast trybu „apply to all levels"

- **Data:** 2026-07-31
- **Status:** ZAIMPLEMENTOWANE (tylko Canable), **nieprzetestowane w aplikacji**.
- **Zakres:** wyłącznie Canable — `ui/js/evistdrive/profiles.js`, `ui/index.html`,
  `ui/style.css`. **Firmware bez zmian**, protokół bez zmian, format konfiguracji bez zmian.
- **Powiązane:** `FW-069_RAMPS_PER_LEVEL.md` — to tam powstały usunięte tu checkboxy.

---

## 1. Co było nie tak z checkboxami

FW-069 wprowadziło dwa przełączniki: „Apply my edits to all levels in this bank" i węższy
„Apply acceleration/deceleration ramps to all levels". Obydwa usunięte.

Wada nie była kosmetyczna, tylko konstrukcyjna: **to był tryb, który można zostawić włączony
i o nim zapomnieć**. Zmiana jednego pola po cichu przepisywała cztery inne poziomy — bez
potwierdzenia, bez informacji ile wartości zostało nadpisanych i bez drogi powrotnej.
Dwa przełączniki dawały do tego cztery kombinacje, z których dwie znaczyły to samo.

## 2. Co jest teraz

**Ustawienia wspólne podzielone na sekcje**, każda z własnym przyciskiem **„⧉ Copy to…"**:

| Sekcja | Zawartość |
|---|---|
| Power and current ceiling | maks. moc, maks. prąd |
| Start condition | assist bez obrotu, minimalny nacisk, obniżka w ruchu, próg przyrostu, okno |
| Launch feel | startup boost (3 pola), smooth start (2 pola) |
| Current ramps | cztery rampy narastania i opadania |
| Power smoothing and release | release, filtr narastania, filtr opadania |

Podział nie jest dekoracją. Skoro kopiowanie działa na sekcjach, sekcja zbierająca wszystko
w jedno wymuszałaby kopiowanie na zasadzie wszystko-albo-nic i byłaby bezużyteczna dla
przypadku, który rowerzysta faktycznie ma: **te same rampy wszędzie, inna siła na poziom**.
Każda sekcja to coś, co sensownie chce się ujednolicić osobno.

Kliknięcie „Copy to…" otwiera panel przy tej sekcji:

```
Copy "Current ramps — acceleration and deceleration" (4 values) from SPORT+
Poziomy:  [x] ECO  [x] TOUR  [x] SPORT  [ ] SPORT+ (source)  [x] BOOST
Banki:    [ ] Also the other bank (same levels)
──────────────────────────────────────────────
Overwrites 16 values in 4 level(s)     [Copy]  [Cancel]
```

## 3. Dlaczego to jest lepsze, a nie tylko inne

- **Nie ma trybu do zapomnienia.** Nic się nie dzieje bez kliknięcia — to była jedyna realna
  wada checkboxów i znika całkowicie.
- **Zasięg wybierany przy każdej akcji.** Dlatego kopiowanie na **oba banki** jest tu
  bezpieczne i zostało dodane. Jako tryb byłoby najprostszą drogą do skasowania strojenia
  jednym kliknięciem; jako świadoma, jednorazowa decyzja z podglądem — nie jest.
- **Skala widoczna przed wykonaniem:** „Overwrites 16 values in 4 level(s)".
- **Cofnięcie.** Po skopiowaniu przy nagłówku sekcji pojawia się „Copied to 4 level(s) · Undo".
  Kopiowanie jest z natury ciche i niszczące, więc droga powrotna musi być pod ręką, a nie
  zakopana w logu.

## 4. Szczegół, który łatwo zrobić źle

Checkbox poziomu **źródłowego zostaje aktywny**, choć jest oznaczony „(source)". Pierwsza
wersja go blokowała — i to uniemożliwiało całkowicie sensowną operację: skopiowanie SPORT+
z banku 1 na SPORT+ w banku 2. Pomijany jest wyłącznie **dokładny slot źródłowy**
(ten bank + ten poziom), a nie numer poziomu.

## 5. Przy okazji: wykresy ramp dostały kolor poziomu

Wykresy „Acceleration — current rise" i „Deceleration — current fall" należą do
**edytowanego poziomu**, dokładnie jak podgląd silnika, ale jako jedyne karty per poziom
zostały białe — co czytało się jako „to jest globalne", czyli odwrotnie niż jest.
Doszły identyfikatory `ebicsProfileAccelerationCard` / `ebicsProfileDecelerationCard`
i obie karty trafiły do `tintProfileCards()`.

Samo płótno wykresu zostaje białe, tak jak w istniejącym podglądzie silnika — barwi się
karta, nie obszar rysowania. To zachowuje jeden język wizualny w całej zakładce.

## 6. Poprawione teksty pomocy

Trzy opisy odsyłały do „Dynamics" — zakładki, która nie istnieje od FW-070, i do ramp jako
globalnych, którymi nie są od FW-069: `startup_boost_enabled`, `release_ms`,
`power_rise_filter_ms`. Wskazują teraz na pasmo globalne na dole zakładki albo na rampy
tego poziomu.

## 7. Testy w aplikacji

1. Sekcje widoczne, każda z przyciskiem „⧉ Copy to…"; żadnego checkboxa „apply to all levels".
2. Kopiowanie ramp na 3 poziomy: licznik pokazuje „Overwrites 12 values in 3 level(s)",
   po zatwierdzeniu wartości zgadzają się na tych poziomach, a **pozostałe pola tych poziomów
   są nietknięte** — to jest sens podziału na sekcje.
3. **Undo** przywraca poprzednie wartości na wszystkich celach naraz.
4. Kopiowanie z zaznaczonym „Also the other bank" z poziomu źródłowego: SPORT+ banku 1 →
   SPORT+ banku 2 **działa**, a SPORT+ banku 1 pozostaje bez zmian.
5. „Nothing selected" blokuje przycisk Copy.
6. Zmiana poziomu w selektorze zmienia kolor tła **obu wykresów ramp**, nie tylko podglądu silnika.
7. Po kopiowaniu pasek górny sygnalizuje niezapisane zmiany w RAM (kopiowanie nie zapisuje
   niczego do kontrolera samo z siebie).
