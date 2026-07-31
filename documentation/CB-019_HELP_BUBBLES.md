# CB-019 — czytelne dymki parametrów: wartość fabryczna, zakres, kierunek zmiany

- **Data:** 2026-07-31
- **Status:** ZAIMPLEMENTOWANE (tylko Canable), **nieprzetestowane w aplikacji**.
- **Zakres:** wyłącznie Canable — `ui/js/shared.js` (`helpBadge`), `ui/style.css`,
  `ui/js/evistdrive/profiles.js` (treść opisów), `tests/cb019_help_bubble_content.js`.
  **Firmware, protokół i format banków bez zmian.**

---

## 1. Co już działało

Mechanika dymków była w większości gotowa i została zachowana bez zmian:

- `fieldInput()` w `common.js` sam dopisuje **„Factory default: …"** i **„Allowed range: …"**,
- wartość fabryczna w Profiles pochodzi z `PROFILE_LEVEL_PLACEHOLDER_BANKS` i jest brana
  **dla aktualnie wybranego banku i poziomu** — etykieta mówi wprost
  „Factory default for Bank 1 / SPORT+", więc nie da się jej pomylić z wartością odczytaną
  z kontrolera,
- w Dynamics wartości fabryczne pochodzą z `TUNING_DEFAULTS`,
- formatowanie: checkbox jako **On/Off**, jednostka doklejana do obu linii, wartości natywne
  przepuszczane przez `fromNative` (mV → kg, mV → V, gamma ×10 → dziesiętna),
- przełączenie banku lub poziomu przebudowuje pola (`renderProfileEditor` jest podpięty pod
  `change` obu selektorów), więc dymek od razu pokazuje właściwą wartość.

## 2. Co było zepsute: dotyk

Dymek pokazywał się na `:hover` i `:focus`. To pokrywa mysz i klawiaturę, ale **nie dotyk**:
tapnięcie elementu nieinteraktywnego (`<span tabindex="0">`) nie daje pewnego focusu na
urządzeniach mobilnych, a nawet gdy dało — nie było czym dymka zamknąć.

Poprawka: badge to teraz **`<button type="button">`** z `aria-label` i `aria-expanded`.

- klik/tap **przełącza** dymek (klasa `is-open`),
- otwarcie kolejnego **zamyka poprzedni** — na dotyku nie da się zebrać stosu bubbli,
  których nie ma jak zdjąć,
- klik gdziekolwiek indziej albo **Escape** zamyka,
- `:hover` i `:focus` działają jak dotąd, więc mysz i klawiatura nic nie tracą,
- `type="button"` jest konieczny: badge bywa wewnątrz formularzy na starych zakładkach,
  a domyślny `submit` przeładowałby stronę,
- klik robi `preventDefault()` + `stopPropagation()` — badge siedzi w `<label>`, więc bez
  tego kliknięcie mogłoby przy okazji przełączyć powiązane pole.

W CSS zdjęte domyślne obramowanie i padding przycisku (żeby wyglądał jak dotychczasowa
okrągła plakietka) oraz dodany `:focus-visible` — obrys widoczny dla klawiatury, niewidoczny
przy myszy.

## 3. Treść opisów

Uzupełnione o **kierunek zmiany** siedem opisów, którym go brakowało: obie wolne rampy,
czas wyłączenia, oba filtry mocy, próg przyrostu nacisku i napięcie odniesienia eMTB.
Wzorzec: *„Higher = …; lower = …"*.

Presety **Aggressive / Normal / Smooth** są przy 11 parametrach dynamiki (7 w Profiles,
4 w Dynamics). Są wyłącznie podpowiedzią w tekście — **nie zmieniają** wartości fabrycznych,
danych odczytanych z roweru, protokołu ani formatu banków.

## 4. Test

`tests/cb019_help_bubble_content.js` odtwarza składanie treści dymka z `fieldInput()`
i sprawdza rzeczy, które najłatwiej zepsuć po cichu:

- pole natywne (mV w pamięci, kg na ekranie) pokazuje **oba** człony w kg, a surowa wartość
  `18` **nie może** wyciec do dymka,
- checkbox czyta się „Off", a nie „false", i nie ogłasza zakresu liczbowego,
- pole liczbowe ma jednostkę w obu liniach,
- brak zadeklarowanej wartości fabrycznej = **brak** linii „Factory default" (żadnego
  zmyślania).

## 5. Do sprawdzenia w aplikacji

1. **Mysz:** najechanie pokazuje dymek, zjechanie chowa.
2. **Klawiatura:** Tab dochodzi do plakietki, dymek się pokazuje, Escape zamyka, obrys
   focusu widoczny.
3. **Dotyk:** tap otwiera, tap w inne miejsce zamyka, tap w drugą plakietkę przełącza —
   **to jest właściwy test tej karty**.
4. Przełączenie banku i poziomu zmienia treść „Factory default for Bank X / POZIOM".
5. Dymek przy polu w szerokiej tabeli nie jest ucinany i nie wychodzi poza ekran przy
   krawędzi okna (to działało wcześniej — sprawdzić, czy zmiana na `<button>` tego nie ruszyła).
