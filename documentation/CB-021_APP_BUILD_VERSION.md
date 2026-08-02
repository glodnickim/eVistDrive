# CB-021 — numer wersji aplikacji Canable

- **Data:** 2026-08-02
- **Status:** ZAIMPLEMENTOWANE (tylko Canable), **nieprzetestowane w aplikacji**.
- **Zakres:** wyłącznie Canable — `scripts/bump-version.js`, `package.json`,
  `ui/js/app-version.js`, `ui/index.html`, `ui/style.css`. Firmware bez zmian.

---

## 1. Problem, na którym się przewróciliśmy

Firmware ma licznik buildów od dawna (`.build/version.txt`), aplikacja nie miała nic.
`package.json` stał na `1.0.0` od zawsze, `dist/openbafang-cannable.exe` zawsze nazywał się
tak samo, a interfejs nie pokazywał żadnego numeru.

Skutek, który wystąpił w praktyce: świeżo zbudowany firmware został uruchomiony ze **starym
`.exe` sprzed kilku godzin**. Wyglądało to dokładnie tak samo jak para dopasowana — nowych
pól po prostu nie było widać. Bez numeru w interfejsie nie da się tego odróżnić od błędu
firmware'u, a szukać zaczyna się w złym miejscu.

## 2. Rozwiązanie

`scripts/bump-version.js` podbija **patch** w `package.json` i zapisuje `ui/version.json`
z numerem oraz znacznikiem czasu spakowania.

Podpięty jako `prebuild` i `prebuild:win`, więc npm uruchamia go **automatycznie** przed
`npm run build` i `npm run build:win`. Nie da się zbudować binarki bez nadania jej numeru —
gdyby to był osobny krok do zapamiętania, prędzej czy później zostałby pominięty.

Numer pokazuje się **przy tytule w nagłówku**, obok nazwy aplikacji. Dymek podaje datę
spakowania i przypomina, że wersja firmware jest w zakładce System — to dwie różne rzeczy
i mylenie ich jest łatwe.

## 3. Szczegóły, które mają znaczenie

**`ui/version.json`, nie `require('package.json')`.** Katalog `ui/` jest już w `pkg.assets`,
więc plik trafia do binarki jako zasób. Czytanie `package.json` z wnętrza spakowanego
programu bywa zawodne i zależy od tego, jak `pkg` potraktuje moduł.

**Uruchomienie ze źródeł pokazuje `dev`,** a nie pusty tekst. `npm run dev` nie generuje
`version.json`; pusta plakietka czytałaby się jako „brak wersji" zamiast „to nie jest
spakowany build". Tak samo przy błędzie odczytu.

**Nazwa pliku `.exe` się nie zmienia.** Podmiana w miejscu jest wygodniejsza niż zbieranie
katalogu ponumerowanych plików, a numer i tak jest widoczny w oknie. Kto chce archiwizować,
ma datę spakowania w dymku.

**Numer rośnie przy każdym buildzie,** także nieudanym w dalszej części — tak samo zachowuje
się licznik firmware'u. Odstęp w numeracji jest tańszy niż dwa różne pliki z tym samym numerem.

## 4. Testy w aplikacji

1. `npm run build:win` wypisuje „Canable build version: X.Y.Z", a `package.json`
   i `ui/version.json` mają ten sam numer.
2. Uruchomiony `.exe` pokazuje `vX.Y.Z` przy tytule; dymek podaje datę spakowania.
3. `npm run dev` pokazuje `dev`.
4. Kolejny build podbija patch o jeden.
