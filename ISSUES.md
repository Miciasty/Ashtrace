# Ashtrace — ISSUES

## Cel pliku

Ten plik powstał 2026-09-09 po przeglądzie wspólnych zasad Blackframe i przyjęciu [blackframe.md, rewizja 2.0](../blackframe.md). Służy do zaplanowania korekt tej biblioteki oraz przekazywania pracy między kolejnymi, niezależnymi sesjami. Nie trzeba znać historii rozmowy: poniżej są powód zadania, miejsca w kodzie, kryteria odbioru i powiązania z innymi projektami.

To lista prac i miejsce zapisu dowodów, a nie dokumentacja gotowych funkcji ani informacja, że błędy już naprawiono. Nie wszystkie pozycje są błędami wykonania: część wymaga doprecyzowania umowy z użytkownikiem lub sprawdzenia istniejących zabezpieczeń. Przegląd obejmował README, POM, workflow i wybrane źródła/testy; nie jest pełnym audytem całego kodu. Podczas przygotowania pliku nie zmieniano implementacji i nie uruchamiano testów bibliotek.

## Punkt odniesienia

- Rola projektu: Wyszukiwanie kandydatów na trafienie i łączenie zapytań z układami odniesienia oraz siatką wokseli.
- Wersja zadeklarowana w lokalnym POM: **1.0.0**. To nie jest potwierdzenie publikacji.
- Stan źródeł podczas przygotowania: **5c516fc** na gałęzi docs/blackframe-contract-v2-20260909; commit zapisuje stan sprzed zmian dokumentacji.
- Zależności: Ashcore 1.0.1, Ashgrid 1.2.0 i Ashspace 1.0.0. Nie wymaga Ashmesh ani Ashnav.
- Dokument nadrzędny: rewizja **2.0 z 2026-09-09**. Numery sekcji w zadaniach odnoszą się do tej rewizji.

## Jak rozpocząć nową sesję

1. Przeczytaj lokalne AGENTS.md/instrukcje użytkownika, [kontrakt Blackframe](../blackframe.md) i cały ten plik. Jeśli kontraktu brakuje w osobnym klonie, uzyskaj właściwą rewizję przed rozstrzyganiem wspólnych zasad.
2. Sprawdź aktualny Git i różnice względem powyższego punktu odniesienia. W tej pracy obowiązywała instrukcja użytkownika: przed zmianami utworzyć nową gałąź i zacommitować obecną wersję. Zachowaj cudze zmiany; nie resetuj repozytorium. Dla katalogu bez Git nie wymyślaj istniejącego commita.
3. Zacznij od wskazanego P1, odtwórz obserwację i sprawdź istniejące testy. Ustal kontrakt przed korektą zachowania. Wpis INSPEKCJA nie zastępuje reprodukcji.
4. Naprawiaj zadania w granicach tego projektu. Zmianę wspólnego kontraktu prowadź u właściciela niższej warstwy, a potrzebną pracę w innym repozytorium zapisz pod jego ID. Rutynowa poprawka nie wymaga edycji blackframe.md.
5. Po zmianach uruchom odpowiednie testy i końcowe clean verify. Aktualizuj statusy i dziennik poniżej: co zmieniono, rzeczywisty wynik kontroli, decyzje zgodności, pozostałe zależności i następny krok. Nie publikuj artefaktów tylko po to, aby sprawdzić kod.

Zalecana kolejność ustaleń wspólnych: Ashcore → Ashgrid → Ashspace, następnie Ashtrace i Ashnav zgodnie z ich zależnościami. Ashnav nie musi czekać na Ashtrace; niezależne zadania lokalne można podejmować wcześniej. Ashtemplate można poprawiać osobno. Ashmesh nie ma obecnie lokalnego katalogu, więc ten backlog nie zleca jego implementacji.

Maven używa zależności rozstrzygniętych z POM i repozytoriów artefaktów. Zmiana pliku w sąsiednim checkout nie podmienia ich automatycznie. Przy integracji zapisz konkretne wersje, commity i wynik rozstrzygnięcia zależności. Dla próbnego builda dolnej warstwy użyj odróżnialnej wersji roboczej lub izolowanego repozytorium testowego; nie nadpisuj istniejącego wydania inną zawartością.

## Oznaczenia

- **P1** — poprawność, publiczne gwarancje lub wymagana weryfikacja; rozstrzygnąć przed deklaracją zgodności z rewizją 2.0 i następnym wydaniem objętego zakresu.
- **P2** — porządkowanie lub pogłębiona kontrola po pilnych korektach; nie pomijać bez zapisanej decyzji.
- **INSPEKCJA** — potwierdzony zapis lub mechanizm w źródle; podany skutek może wymagać jeszcze testu wykonania.
- **AUDYT** — zakres do sprawdzenia, bez twierdzenia, że wszystkie wymienione miejsca są błędne.
- **DECYZJA** — trzeba wybrać i udokumentować wspierany kontrakt lub migrację.
- Statusy: **OTWARTE**, **W TOKU**, **ZABLOKOWANE** (z konkretną zależnością), **GOTOWE** (z dowodem spełnienia kryteriów), **NIE DOTYCZY** (z uzasadnieniem). Zachowuj identyfikatory po zamknięciu.

## Kolejka

| ID | Priorytet | Typ | Zadanie |
| --- | --- | --- | --- |
| [TRACE-001](#trace-001) | P1 | INSPEKCJA | Rozdzielić zaakceptowaną obwiednię od dokładnego trafienia |
| [TRACE-002](#trace-002) | P1 | AUDYT | Sprawdzić zasłanianie, jednostki i końce przedziałów |
| [TRACE-003](#trace-003) | P1 | AUDYT | Ustalić stabilną kolejność wyników indeksów |
| [TRACE-004](#trace-004) | P1 | INSPEKCJA | Skorygować opis kosztu sortowania i pracy indeksów |
| [TRACE-005](#trace-005) | P1 | DECYZJA | Zdefiniować wspierane implementacje i zgodność API |
| [TRACE-006](#trace-006) | P1 | INSPEKCJA | Dostosować CI, pakowanie i dowody wydania |

<a id="trace-001"></a>

## TRACE-001 — Rozdzielić zaakceptowaną obwiednię od dokładnego trafienia

**Status:** OTWARTE  
**Priorytet:** P1  
**Dowód:** INSPEKCJA  
**Kontrakt:** sekcje 3.4, 4.2, 5

**Gdzie:** [FrameBroadPhaseRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameBroadPhaseRayTracer3.java), [TraceHit3.java](src/main/java/nsk/nu/ashtrace/api/trace/model/TraceHit3.java), [FrameBroadPhaseRayTracer3ApiTest.java](src/test/java/nsk/nu/ashtrace/unit/api/trace/FrameBroadPhaseRayTracer3ApiTest.java), [README.md](README.md).

**Stan podczas przeglądu:** NarrowPhase3 zwraca boolean. firstHit i allHits budują TraceHit3 z tEnter/tExit obwiedni i worldRay.at(tEnter). Kandydaci są sortowani po tych samych parametrach obwiedni. Callback nie ma jak przekazać dokładnego parametru przecięcia właściwego kształtu.

**Znaczenie:** Pierwsze pudełko przecinane przez promień nie musi zawierać najbliższej powierzchni. Nazwa hitPoint może zostać odczytana jako dokładne miejsce trafienia.

**Praca do wykonania:** Najpierw potwierdź testem semantykę istniejącego API. Zdecyduj: zachować jawnie opisane wyszukiwanie pierwszej zaakceptowanej obwiedni albo dodać osobny, kompatybilny kontrakt zwracający dokładny wynik. Nie obiecuj dokładności przez zmianę nazwy callbacku.

**Warunki zamknięcia:**

- [ ] Scenariusz dwóch obiektów, gdzie AABB obiektu A zaczyna się wcześniej, ale jego powierzchnia leży dalej niż powierzchnia B, rozróżnia kolejność obwiedni od rzeczywistych trafień.
- [ ] Dokumentacja jednoznacznie wyjaśnia tEnter, hitPoint i firstHit. Ewentualne dokładne API wybiera i sortuje po dokładnym parametrze.
- [ ] Zmiana ma plan zgodności dla obecnego NarrowPhase3 i TraceHit3 oraz testy zaakceptowania/odrzucenia kandydatów.

**Powiązania:** [TRACE-002](../Ashtrace/ISSUES.md#trace-002) dotyczy zasłaniania; reguły primitive intersection w [CORE-004](../Ashcore/ISSUES.md#core-004). Dokładny interfejs nie wymaga wdrożenia całego silnika fizyki.

<a id="trace-002"></a>

## TRACE-002 — Sprawdzić zasłanianie, jednostki i końce przedziałów

**Status:** OTWARTE  
**Priorytet:** P1  
**Dowód:** AUDYT  
**Kontrakt:** sekcje 3.4, 4.2, 4.3, 4.5

**Gdzie:** [FrameOccludedBroadPhaseRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameOccludedBroadPhaseRayTracer3.java), [FrameGridRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameGridRayTracer3.java), [RayQueryableBroadPhase3.java](src/main/java/nsk/nu/ashtrace/api/broadphase/contracts/RayQueryableBroadPhase3.java), [FrameOccludedBroadPhaseRayTracer3ApiTest.java](src/test/java/nsk/nu/ashtrace/unit/api/trace/FrameOccludedBroadPhaseRayTracer3ApiTest.java).

**Stan podczas przeglądu:** visibleLimit przycina tMax do wejścia w pierwszy woksel. RayQueryableBroadPhase3 używa domkniętych przedziałów, a traversal Ashgrid opisuje półotwarte. Zapytania obiektów i wokseli odczytują mutable FrameGraph3. Wynik z [TRACE-001](../Ashtrace/ISSUES.md#trace-001) nadal oznacza obwiednię, jeśli API nie zmieniono.

**Znaczenie:** Obiekt na granicy ściany, promień rozpoczynający się w ścianie lub niezgodna skala siatki mogą być różnie rozumiane. Akceptacja obwiedni nie dowodzi widoczności każdej części obiektu.

**Praca do wykonania:** Zdefiniuj traktowanie równego t trafienia i ściany, tMax=0, początku w przeszkodzie i jednostkowej siatki w world. Potwierdź, jaki stan ramek/callbacków musi pozostawać stały. Odtwórz przypadek AABB przecinającego obszar przed ścianą, gdy właściwa powierzchnia jest za nią.

**Warunki zamknięcia:**

- [ ] Testy rozstrzygają równe granice, pusty odcinek, początki wewnątrz, brak zasłony i ujemne kierunki; wskazują wersję Ashgrid z poprawką [GRID-001](../Ashgrid/ISSUES.md#grid-001).
- [ ] Dokumentacja określa, czy visible dotyczy obwiedni czy dokładnego przecięcia i jak callback ma przestrzegać przyciętego przedziału.
- [ ] Jednostki t, model siatki i wymaganie stabilnych ramek są jawne; zmiana cellSize nie jest zakładana automatycznie przez tracer.

**Powiązania:** [GRID-001](../Ashgrid/ISSUES.md#grid-001) oraz [GRID-002](../Ashgrid/ISSUES.md#grid-002), [CORE-001](../Ashcore/ISSUES.md#core-001) oraz [CORE-004](../Ashcore/ISSUES.md#core-004) i [SPACE-002](../Ashspace/ISSUES.md#space-002) oraz [SPACE-004](../Ashspace/ISSUES.md#space-004). Nie maskuj defektu DDA w Ashtrace.

<a id="trace-003"></a>

## TRACE-003 — Ustalić stabilną kolejność wyników indeksów

**Status:** OTWARTE  
**Priorytet:** P1  
**Dowód:** AUDYT  
**Kontrakt:** sekcje 4.1, 4.2, 4.5

**Gdzie:** [BroadPhase3.java](src/main/java/nsk/nu/ashtrace/api/broadphase/contracts/BroadPhase3.java), [LinearAabbBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/staticindex/LinearAabbBroadPhase3.java), [BvhAabbBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/staticindex/BvhAabbBroadPhase3.java), [DynamicSpatialHashBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/dynamic/DynamicSpatialHashBroadPhase3.java), [DynamicBvhBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/dynamic/DynamicBvhBroadPhase3.java), [FrameBroadPhaseRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameBroadPhaseRayTracer3.java).

**Stan podczas przeglądu:** Pipeline rozstrzyga pełny remis tEnter/tExit według kolejności emisji broad-phase. Powtarzalność pojedynczej implementacji nie oznacza identycznej kolejności między różnymi indeksami ani po przestawieniu insertów.

**Znaczenie:** Przy dwóch obiektach w tym samym miejscu firstHit może wybrać inny obiekt po zmianie indeksu, jeżeli porządek nie jest wspólną gwarancją.

**Praca do wykonania:** Spisz gwarancje per zapytanie, w tym update/remove/reinsert i przebudowa snapshotu. Zdecyduj, czy porównanie implementacji dotyczy zbioru kandydatów, czy również porządku i remisu. Jeśli potrzebny wspólny porządek, oprzyj go na stabilnym ID, bez zakładania stabilności Object.hashCode.

**Warunki zamknięcia:**

- [ ] Testy zgodności indeksów sprawdzają deklarowane wspólne własności dla AABB/ray/sphere/nearest/sweep.
- [ ] Testy powtarzalności obejmują sekwencje mutacji, identyczne obwiednie i remisy; dowolną kolejność insertów testuje się jako gwarancję tylko po jej przyjęciu.
- [ ] Zapisano zakres thread-safety i zakaz lub model mutacji indeksu podczas callbacku.

**Powiązania:** [TRACE-001](../Ashtrace/ISSUES.md#trace-001) i ustalenia deterministyczne [CORE-002](../Ashcore/ISSUES.md#core-002); bez wymagania produkcyjnej zależności od nowych bibliotek.

<a id="trace-004"></a>

## TRACE-004 — Skorygować opis kosztu sortowania i pracy indeksów

**Status:** OTWARTE  
**Priorytet:** P1  
**Dowód:** INSPEKCJA  
**Kontrakt:** sekcje 4.4, 5.1

**Gdzie:** [README.md](README.md), [FrameBroadPhaseRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameBroadPhaseRayTracer3.java), [DynamicSpatialHashBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/dynamic/DynamicSpatialHashBroadPhase3.java), [DynamicBvhBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/dynamic/DynamicBvhBroadPhase3.java), [BroadPhaseBenchmarkMain.java](src/test/java/nsk/nu/ashtrace/benchmark/manual/BroadPhaseBenchmarkMain.java).

**Stan podczas przeglądu:** orderedCandidates zbiera i sortuje wszystkich kandydatów oraz tworzy dodatkowe listy nawet dla firstHit. README opisuje pipeline jako O(q+m), gdzie q oznacza pracę zapytania broad-phase; pomija oddzielny koszt sortowania. Koszt hash query wymaga także uwzględnienia przeglądanych wpisów, nie wyłącznie wyników.

**Znaczenie:** Zapytanie o jedno trafienie nie musi zużywać pamięci i czasu jak operacja na jednym obiekcie. Przy wielu nakładających się obwiedniach koszty mogą być znaczne.

**Praca do wykonania:** Policz czas i pamięć na podstawie faktycznych struktur. Uwzględnij C log C sortowania C kandydatów, callbacki, surowe wpisy hash, duplikaty oraz odroczoną przebudowę BVH. Optymalizację podejmij dopiero po pomiarze; poprawa tabeli nie wymaga przebudowy algorytmu.

**Warunki zamknięcia:**

- [ ] Tabela jawnie obejmuje sortowanie i pamięć kandydatów; q/m/C są zdefiniowane bez ukrywania pracy.
- [ ] Rozdzielono aktualizację, pierwsze zapytanie po mutacji i kolejne odczyty oraz wskazano przypadki najgorsze.
- [ ] Jeśli dodano optymalizację, pomiar reprezentatywnego przypadku i testy semantyki/remisów potwierdzają efekt.

**Powiązania:** [NAV-005](../Ashnav/ISSUES.md#nav-005) ma podobny temat rzetelnej złożoności, ale wymaga osobnej analizy właściwej dla grafów.

<a id="trace-005"></a>

## TRACE-005 — Zdefiniować wspierane implementacje i zgodność API

**Status:** OTWARTE  
**Priorytet:** P1  
**Dowód:** DECYZJA  
**Kontrakt:** sekcje 5, 5.1, 8

**Gdzie:** [README.md](README.md), [BvhAabbBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/staticindex/BvhAabbBroadPhase3.java), [BroadPhaseMath3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/internal/BroadPhaseMath3.java), [TraceHit3.java](src/main/java/nsk/nu/ashtrace/api/trace/model/TraceHit3.java).

**Stan podczas przeglądu:** Oficjalny quick start tworzy BvhAabbBroadPhase3 z implementation. Jednocześnie projekt ma jawny podpakiet internal. Zmiany znaczenia trafienia mogą naruszać zgodność bez zmiany sygnatur.

**Znaczenie:** Użytkownik powinien wiedzieć, które klasy są stabilnym punktem integracji i czy aktualizacja zmienia znaczenie wyniku.

**Praca do wykonania:** Opisz wspierane klasy i granicę internal. Dla [TRACE-001](../Ashtrace/ISSUES.md#trace-001) oraz [TRACE-002](../Ashtrace/ISSUES.md#trace-002) wybierz migrację, wersję i prosty przykład obwiedni oraz akceptacji. Wyjaśnij, że nearest oznacza najbliższy AABB, o ile kontrakt nie zwraca rzeczywistej powierzchni.

**Warunki zamknięcia:**

- [ ] Oba quick starty kompilują się na docelowych zależnościach.
- [ ] API/README są zgodne co do granicy candidate/exact i statusu implementacji.
- [ ] Zapisano wpływ aktualizacji Ashcore/Ashgrid/Ashspace; opublikowane artefakty nie są nadpisywane.

**Powiązania:** [CORE-006](../Ashcore/ISSUES.md#core-006), [GRID-006](../Ashgrid/ISSUES.md#grid-006), [SPACE-005](../Ashspace/ISSUES.md#space-005); brak wymogu tworzenia Ashmesh.

<a id="trace-006"></a>

## TRACE-006 — Dostosować CI, pakowanie i dowody wydania

**Status:** OTWARTE  
**Priorytet:** P1  
**Dowód:** INSPEKCJA  
**Kontrakt:** sekcje 2, 4.5, 6

**Gdzie:** [pom.xml](pom.xml), [.github/workflows/maven.yml](.github/workflows/maven.yml), [.github/workflows/publish.yml](.github/workflows/publish.yml), [README.md](README.md).

**Stan podczas przeglądu:** CI uruchamia mvn -B package, a kontrakt wymaga clean verify. POM ustawia source/target 21 bez jawnego przypięcia maven-compiler-plugin; Javadoc ma doclint=none i failOnError=false. Profil central istnieje, lecz pokazany workflow deploy nie aktywuje go i publikuje do GitHub Packages. Początkowa gałąź: main; CI filtruje master. To rozbieżność lokalnego stanu z konfiguracją: potwierdź faktyczną gałąź domyślną na serwerze przed zmianą filtrów.

**Znaczenie:** Zielony wynik obecnego CI nie jest dowodem wykonania całej bramki jakości ani obecności artefaktu w Maven Central. Brak automatyzacji Central nie dowodzi braku publikacji ręcznej.

**Praca do wykonania:** Ustaw rzeczywistą bramkę clean verify, dobierz przypięty compiler plugin i release 21, sprawdź generowanie dokumentacji oraz jednoznaczną identyfikację artefaktów. Potwierdź utrzymywane gałęzie, docelowe wersje zależności i sposób publikacji do każdej używanej destynacji. JUnit pozostaw w test scope; nie usuwaj go w imię niezależności produkcyjnej.

**Warunki zamknięcia:**

- [ ] Zapisano wynik mvn -B clean verify z wymaganymi testami oraz wersje JDK/Maven; CI obejmuje faktycznie utrzymywane gałęzie i PR-y.
- [ ] Główny JAR, sources, Javadoc i wymagane zasoby są sprawdzone. Błędny Javadoc nie jest po cichu uznawany za poprawny; nie trzeba przy tym mechanicznie włączać każdej reguły stylistycznej doclint.
- [ ] Wskazano używane cele publikacji, tag/wersję i dowody dostępności albo jawnie pozostawiono publikację jako niezweryfikowaną. Sam deploy nie służy jako test poprawek.
- [ ] Sprawdzono efektywne zależności i ich scope; test integracyjny korzysta z zamierzonej wersji dolnej warstwy, a nie przypadkowej starej kopii z lokalnego Maven.

**Powiązania:** Wspólny wzorzec: [TEMPLATE-001](../Ashtemplate/ISSUES.md#template-001) i [TEMPLATE-002](../Ashtemplate/ISSUES.md#template-002). Tę korektę można wykonać niezależnie od napraw algorytmów. Istniejącego numeru wydania nie nadpisuj innym artefaktem.

## Stan przekazania i dziennik sesji

**Na 2026-09-09:** wszystkie zadania pozostają OTWARTE. Utworzono dokumentację; nie wprowadzono korekt kodu, nie wykonano buildów bibliotek ani publikacji. Nie uznawaj samego dodania ISSUES.md za realizację żadnego zadania.

**Sugerowany start:** [TRACE-001](../Ashtrace/ISSUES.md#trace-001) i [TRACE-002](../Ashtrace/ISSUES.md#trace-002); testy kierunków ujemnych skoordynuj z [GRID-001](../Ashgrid/ISSUES.md#grid-001).

Po kolejnej sesji dopisz wiersz i uzupełnij statusy odpowiednich zadań. Zapisz także nieudane próby i ograniczenia środowiska; nie opisuj kontroli niewykonanej jako zaliczonej.

| Data / commit | ID i decyzja | Zmiana | Polecenie / test i rzeczywisty wynik | Pozostałe zależności / następny krok |
| --- | --- | --- | --- | --- |
| 2026-09-09 / punkt odniesienia powyżej | Wszystkie: OTWARTE | Utworzenie planu korekt | Inspekcja statyczna; testów bibliotek nie uruchomiono | Rozpocząć od wskazanego P1 |
