# FW-070 — zakładka Dynamics scalona z Profiles, z jawnym rozdzieleniem zasięgu

- **Data:** 2026-07-31
- **Status:** ZAIMPLEMENTOWANE (tylko Canable), **nieprzetestowane w aplikacji**.
- **Zakres:** wyłącznie Canable — `ui/index.html`, `ui/style.css`,
  `ui/js/evistdrive/dynamics.js`. **Firmware bez zmian**, protokół bez zmian,
  format konfiguracji bez zmian.
- **Powiązane:** `FW-069_RAMPS_PER_LEVEL.md` — to ona opróżniła kartę Dynamics z ramp
  i sprawiła, że osobna zakładka przestała mieć rację bytu.

---

## 1. Dlaczego

Po FW-069 w karcie Dynamics zostały cztery grupy ustawień, wszystkie **globalne**: zanik
startup boostu, warunek startu (kroki korby), zatrzask jazdy i wygładzanie RUN. Rampy —
jedyna rzecz, która wypełniała tę zakładkę treścią — przeniosły się per poziom do Profiles.

Zostały więc dwie zakładki, między którymi trzeba było skakać przy jednym zadaniu
(„dobierz start"): próg nacisku per poziom w Profiles, kroki korby globalnie w Dynamics.
Nic w interfejsie nie mówiło, że to dwa różne zasięgi — a to jest najważniejsza informacja
przy strojeniu. Łatwo było uznać, że kroki korby też są per poziom.

## 2. Co się zmieniło

Zakładka **eVistDrive Dynamics usunięta** razem z przyciskiem nawigacji. Jej zawartość jest
teraz **pasmem „Global — whole bike"** na dole zakładki Profiles.

Rozdzielenie zasięgu jest zrobione trzema niezależnymi środkami naraz, bo pojedynczy
łatwo przeoczyć przy przewijaniu:

1. **Osobna płyta** (`.ebics-global-band`): inne tło, ramka 2 px i gruba, 6-pikselowa
   krawędź górna — wizualna granica, nie kolejna karta w stosie.
2. **Nagłówek i tekst wprost:** „These are **not** bank or level settings. One set of values
   applies to **both banks and all five assist levels**. Changing the bank or level selector
   above has no effect on anything in this section."
3. **Własne przyciski**: „Read global tuning" / „Write global (RAM)" / „Restore global",
   fizycznie oddzielone od przycisków banku. To odzwierciedla rzeczywistość protokołu — to
   inny blok CAN (0x6023/0x6024) niż banki (0x6020/0x6021).

## 3. Pułapka, o którą trzeba było zadbać

`renderDynamicsCharts()` miało bramkę `tabIsVisible('tab-ebics-dynamics')`. Po usunięciu
zakładki ten warunek **nigdy nie byłby spełniony** i wykres zaniku boostu przestałby się
rysować — po cichu, bez błędu. Bramka wskazuje teraz `tab-ebics-profiles`.

Sama bramka musi zostać: Plotly rysujący do ukrytego kontenera przyjmuje wymiar 0 i wykres
wraca pusty po przełączeniu zakładki.

## 4. Czego NIE zmieniono

- Firmware, protokół i format bloków konfiguracji — bez najmniejszej zmiany. Buildy
  `0.0265`/`0.0266` pozostają aktualne.
- Podział na to, co globalne, a co per poziom — ustalony w FW-068/FW-069, tu tylko
  **pokazany** uczciwie w interfejsie.
- Identyfikatory elementów (`ebicsDynamics*`) i moduł `dynamics.js` — zostały, żeby zmiana
  ograniczyła się do przeniesienia i nie mieszała się z refaktorem nazw.

## 5. Testy w aplikacji

1. Zakładki „eVistDrive Dynamics" nie ma na liście, a Profiles otwiera się normalnie.
2. Na dole Profiles jest pasmo „Global — whole bike" z czterema grupami ustawień
   i trzema własnymi przyciskami.
3. **Wykres zaniku boostu rysuje się** po wejściu w Profiles i po powrocie z innej zakładki
   (to jest test poprawionej bramki widoczności — pusty wykres oznacza, że wskazuje złą zakładkę).
4. „Read global tuning" wypełnia pola pasma; „Write global (RAM)" kończy się ACK.
5. Przełączenie banku i poziomu **nie zmienia** wartości w paśmie globalnym.
6. Zapis banku (przyciski u góry) i zapis globalny (przyciski w paśmie) działają niezależnie —
   jeden nie nadpisuje drugiego.
