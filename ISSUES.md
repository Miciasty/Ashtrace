# Ashtrace — ISSUES

**Aktualne koordynaty wydania (2026-09-10): 2.0.0**, bez SNAPSHOT. Zależności uzgodniono dla całego zestawu bibliotek; 545 testów na Java 21 przeszło bez błędów. [Wersje, sumy artefaktów i dowody](../Ashnav/VERIFICATION.md#release-version-alignment). Bez operacji Git i publikacji. Wcześniejsze wpisy poniżej zachowują historyczne wersje i wyniki.

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
| [TRACE-007](#trace-007) | P2 | DECYZJA | Obsłużyć siatkę z ramką, początkiem i rozmiarem komórki |
| [TRACE-008](#trace-008) | P2 | DECYZJA | Zwracać wejście i wyjście z geometrii dostawcy |
| [TRACE-009](#trace-009) | P2 | INSPEKCJA | Dodać zatrzymywanie zapytań i ponowne użycie bufora |
| [TRACE-010](#trace-010) | P1 | AUDYT | Odtworzyć build na hosted CI i potwierdzić gotowość wydania |
| [TRACE-011](#trace-011) | P1 | AUDYT | Sprawdzić integrację z oryginalnymi testami dolnych warstw |
| [TRACE-012](#trace-012) | P2 | DECYZJA | Połączyć zapytania obróconych prymitywów z indeksami AABB |

<a id="trace-001"></a>

## TRACE-001 — Rozdzielić zaakceptowaną obwiednię od dokładnego trafienia

**Status:** GOTOWE
**Priorytet:** P1  
**Dowód:** INSPEKCJA  
**Kontrakt:** sekcje 3.4, 4.2, 5

**Gdzie:** [FrameBroadPhaseRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameBroadPhaseRayTracer3.java), [TraceHit3.java](src/main/java/nsk/nu/ashtrace/api/trace/model/TraceHit3.java), [FrameBroadPhaseRayTracer3ApiTest.java](src/test/java/nsk/nu/ashtrace/unit/api/trace/FrameBroadPhaseRayTracer3ApiTest.java), [README.md](README.md).

**Stan podczas przeglądu:** NarrowPhase3 zwraca boolean. firstHit i allHits budują TraceHit3 z tEnter/tExit obwiedni i worldRay.at(tEnter). Kandydaci są sortowani po tych samych parametrach obwiedni. Callback nie ma jak przekazać dokładnego parametru przecięcia właściwego kształtu.

**Znaczenie:** Pierwsze pudełko przecinane przez promień nie musi zawierać najbliższej powierzchni. Nazwa hitPoint może zostać odczytana jako dokładne miejsce trafienia.

**Praca do wykonania:** Najpierw potwierdź testem semantykę istniejącego API. Zdecyduj: zachować jawnie opisane wyszukiwanie pierwszej zaakceptowanej obwiedni albo dodać osobny, kompatybilny kontrakt zwracający dokładny wynik. Nie obiecuj dokładności przez zmianę nazwy callbacku.

**Warunki zamknięcia:**

- [x] Scenariusz dwóch obiektów, gdzie AABB obiektu A zaczyna się wcześniej, ale jego powierzchnia leży dalej niż powierzchnia B, rozróżnia kolejność obwiedni od rzeczywistych trafień.
- [x] Dokumentacja jednoznacznie wyjaśnia tEnter, hitPoint i firstHit. Ewentualne dokładne API wybiera i sortuje po dokładnym parametrze.
- [x] Zmiana ma plan zgodności dla obecnego NarrowPhase3 i TraceHit3 oraz testy zaakceptowania/odrzucenia kandydatów.

**Powiązania:** [TRACE-002](../Ashtrace/ISSUES.md#trace-002) dotyczy zasłaniania; reguły primitive intersection w [CORE-004](../Ashcore/ISSUES.md#core-004). Dokładny interfejs nie wymaga wdrożenia całego silnika fizyki.

**Wynik 2026-09-10:** Zachowano boolean NarrowPhase3 i AABB-owe znaczenie TraceHit3, bez zmiany sygnatur. CandidateSemanticsTest rozróżnia wcześniejszą obwiednię A i bliższą powierzchnię B, sprawdza akceptację/odrzucenie oraz niemutowalność listy. README/Javadoc opisują firstHit, tEnter/tExit i worldPoint. PASS w końcowym clean verify; migracja w VERIFICATION.md.

<a id="trace-002"></a>

## TRACE-002 — Sprawdzić zasłanianie, jednostki i końce przedziałów

**Status:** GOTOWE
**Priorytet:** P1  
**Dowód:** AUDYT  
**Kontrakt:** sekcje 3.4, 4.2, 4.3, 4.5

**Gdzie:** [FrameOccludedBroadPhaseRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameOccludedBroadPhaseRayTracer3.java), [FrameGridRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameGridRayTracer3.java), [RayQueryableBroadPhase3.java](src/main/java/nsk/nu/ashtrace/api/broadphase/contracts/RayQueryableBroadPhase3.java), [FrameOccludedBroadPhaseRayTracer3ApiTest.java](src/test/java/nsk/nu/ashtrace/unit/api/trace/FrameOccludedBroadPhaseRayTracer3ApiTest.java).

**Stan podczas przeglądu:** visibleLimit przycina tMax do wejścia w pierwszy woksel. RayQueryableBroadPhase3 używa domkniętych przedziałów, a traversal Ashgrid opisuje półotwarte. Zapytania obiektów i wokseli odczytują mutable FrameGraph3. Wynik z [TRACE-001](../Ashtrace/ISSUES.md#trace-001) nadal oznacza obwiednię, jeśli API nie zmieniono.

**Znaczenie:** Obiekt na granicy ściany, promień rozpoczynający się w ścianie lub niezgodna skala siatki mogą być różnie rozumiane. Akceptacja obwiedni nie dowodzi widoczności każdej części obiektu.

**Praca do wykonania:** Zdefiniuj traktowanie równego t trafienia i ściany, tMax=0, początku w przeszkodzie i jednostkowej siatki w world. Potwierdź, jaki stan ramek/callbacków musi pozostawać stały. Odtwórz przypadek AABB przecinającego obszar przed ścianą, gdy właściwa powierzchnia jest za nią.

**Warunki zamknięcia:**

- [x] Testy rozstrzygają równe granice, pusty odcinek, początki wewnątrz, brak zasłony i ujemne kierunki; wskazują wersję Ashgrid z poprawką [GRID-001](../Ashgrid/ISSUES.md#grid-001).
- [x] Dokumentacja określa, czy visible dotyczy obwiedni czy dokładnego przecięcia i jak callback ma przestrzegać przyciętego przedziału.
- [x] Jednostki t, model siatki i wymaganie stabilnych ramek są jawne; zmiana cellSize nie jest zakładana automatycznie przez tracer.

**Powiązania:** [GRID-001](../Ashgrid/ISSUES.md#grid-001) oraz [GRID-002](../Ashgrid/ISSUES.md#grid-002), [CORE-001](../Ashcore/ISSUES.md#core-001) oraz [CORE-004](../Ashcore/ISSUES.md#core-004) i [SPACE-002](../Ashspace/ISSUES.md#space-002) oraz [SPACE-004](../Ashspace/ISSUES.md#space-004). Nie maskuj defektu DDA w Ashtrace.

**Wynik 2026-09-10:** CandidateSemanticsTest odtworzył problem ujemnego kierunku na Ashgrid 1.2.0; po przejściu na konkretny JAR 1.3.0-SNAPSHOT testy PASS. Zachowano kontakt domknięty na wejściu w ścianę i przy początku w przeszkodzie; tMax=0 nie odwiedza wokseli, zerowy segment jest odrzucany. Testy obejmują brak zasłony, oba kierunki, przycięty callback, granicę tMax oraz przesunięcie/obrót z zamrożonym grafem. Jednostkowa siatka world i stabilny stan są jawne. Hashe Ashcore/Ashgrid/Ashspace zapisano w VERIFICATION.md; dolnych bibliotek nie zmieniono.

<a id="trace-003"></a>

## TRACE-003 — Ustalić stabilną kolejność wyników indeksów

**Status:** GOTOWE
**Priorytet:** P1  
**Dowód:** AUDYT  
**Kontrakt:** sekcje 4.1, 4.2, 4.5

**Gdzie:** [BroadPhase3.java](src/main/java/nsk/nu/ashtrace/api/broadphase/contracts/BroadPhase3.java), [LinearAabbBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/staticindex/LinearAabbBroadPhase3.java), [BvhAabbBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/staticindex/BvhAabbBroadPhase3.java), [DynamicSpatialHashBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/dynamic/DynamicSpatialHashBroadPhase3.java), [DynamicBvhBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/dynamic/DynamicBvhBroadPhase3.java), [FrameBroadPhaseRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameBroadPhaseRayTracer3.java).

**Stan podczas przeglądu:** Pipeline rozstrzyga pełny remis tEnter/tExit według kolejności emisji broad-phase. Powtarzalność pojedynczej implementacji nie oznacza identycznej kolejności między różnymi indeksami ani po przestawieniu insertów.

**Znaczenie:** Przy dwóch obiektach w tym samym miejscu firstHit może wybrać inny obiekt po zmianie indeksu, jeżeli porządek nie jest wspólną gwarancją.

**Praca do wykonania:** Spisz gwarancje per zapytanie, w tym update/remove/reinsert i przebudowa snapshotu. Zdecyduj, czy porównanie implementacji dotyczy zbioru kandydatów, czy również porządku i remisu. Jeśli potrzebny wspólny porządek, oprzyj go na stabilnym ID, bez zakładania stabilności Object.hashCode.

**Warunki zamknięcia:**

- [x] Testy zgodności indeksów sprawdzają deklarowane wspólne własności dla AABB/ray/sphere/nearest/sweep.
- [x] Testy powtarzalności obejmują sekwencje mutacji, identyczne obwiednie i remisy; dowolną kolejność insertów testuje się jako gwarancję tylko po jej przyjęciu.
- [x] Zapisano zakres thread-safety i zakaz lub model mutacji indeksu podczas callbacku.

**Powiązania:** [TRACE-001](../Ashtrace/ISSUES.md#trace-001) i ustalenia deterministyczne [CORE-002](../Ashcore/ISSUES.md#core-002); bez wymagania produkcyjnej zależności od nowych bibliotek.

**Wynik 2026-09-10:** Zachowano porządek per implementacja, bez obietnicy niezależności od insertów. BroadPhaseAgreementTest porównuje zbiory/parametry pięciu rodzin zapytań, BroadPhaseOrderingTest remisy oraz update/remove/reinsert/clear i przebudowy BVH. Naprawiono odtworzony błąd maxDistance w liściach BVH oraz zerowanie małych składowych promienia/ruchu. SpatialHashLimitsTest sprawdza granice int, subnormalny cellSize i niezmienność indeksu po odrzuceniu zakresu. README/Javadoc określają brak thread-safety indeksów mutable i zakaz mutacji w callbackach. Wszystkie testy PASS.

<a id="trace-004"></a>

## TRACE-004 — Skorygować opis kosztu sortowania i pracy indeksów

**Status:** GOTOWE
**Priorytet:** P1  
**Dowód:** INSPEKCJA  
**Kontrakt:** sekcje 4.4, 5.1

**Gdzie:** [README.md](README.md), [FrameBroadPhaseRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameBroadPhaseRayTracer3.java), [DynamicSpatialHashBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/dynamic/DynamicSpatialHashBroadPhase3.java), [DynamicBvhBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/dynamic/DynamicBvhBroadPhase3.java), [BroadPhaseBenchmarkMain.java](src/test/java/nsk/nu/ashtrace/benchmark/manual/BroadPhaseBenchmarkMain.java).

**Stan podczas przeglądu:** orderedCandidates zbiera i sortuje wszystkich kandydatów oraz tworzy dodatkowe listy nawet dla firstHit. README opisuje pipeline jako O(q+m), gdzie q oznacza pracę zapytania broad-phase; pomija oddzielny koszt sortowania. Koszt hash query wymaga także uwzględnienia przeglądanych wpisów, nie wyłącznie wyników.

**Znaczenie:** Zapytanie o jedno trafienie nie musi zużywać pamięci i czasu jak operacja na jednym obiekcie. Przy wielu nakładających się obwiedniach koszty mogą być znaczne.

**Praca do wykonania:** Policz czas i pamięć na podstawie faktycznych struktur. Uwzględnij C log C sortowania C kandydatów, callbacki, surowe wpisy hash, duplikaty oraz odroczoną przebudowę BVH. Optymalizację podejmij dopiero po pomiarze; poprawa tabeli nie wymaga przebudowy algorytmu.

**Warunki zamknięcia:**

- [x] Tabela jawnie obejmuje sortowanie i pamięć kandydatów; q/m/C są zdefiniowane bez ukrywania pracy.
- [x] Rozdzielono aktualizację, pierwsze zapytanie po mutacji i kolejne odczyty oraz wskazano przypadki najgorsze.
- [x] Jeśli dodano optymalizację, pomiar reprezentatywnego przypadku i testy semantyki/remisów potwierdzają efekt.

**Powiązania:** [NAV-005](../Ashnav/ISSUES.md#nav-005) ma podobny temat rzetelnej złożoności, ale wymaga osobnej analizy właściwej dla grafów.

**Wynik 2026-09-10:** Tabela README i Javadoc pipeline opisują Q + C log C + F, wszystkie listy kandydatów, skanowanie surowych referencji hash, deduplikację/sortowanie uchwytów, liniowe usuwanie z bucketów i build + query po mutacji BVH. Rozdzielono pamięć pomocniczą, wynik i retencję indeksu; symbole są zdefiniowane. Nie optymalizowano algorytmów, więc warunek pomiaru optymalizacji nie dotyczy; nie deklarowano przyspieszenia.

<a id="trace-005"></a>

## TRACE-005 — Zdefiniować wspierane implementacje i zgodność API

**Status:** GOTOWE
**Priorytet:** P1  
**Dowód:** DECYZJA  
**Kontrakt:** sekcje 5, 5.1, 8

**Gdzie:** [README.md](README.md), [BvhAabbBroadPhase3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/staticindex/BvhAabbBroadPhase3.java), [BroadPhaseMath3.java](src/main/java/nsk/nu/ashtrace/implementation/broadphase/internal/BroadPhaseMath3.java), [TraceHit3.java](src/main/java/nsk/nu/ashtrace/api/trace/model/TraceHit3.java).

**Stan podczas przeglądu:** Oficjalny quick start tworzy BvhAabbBroadPhase3 z implementation. Jednocześnie projekt ma jawny podpakiet internal. Zmiany znaczenia trafienia mogą naruszać zgodność bez zmiany sygnatur.

**Znaczenie:** Użytkownik powinien wiedzieć, które klasy są stabilnym punktem integracji i czy aktualizacja zmienia znaczenie wyniku.

**Praca do wykonania:** Opisz wspierane klasy i granicę internal. Dla [TRACE-001](../Ashtrace/ISSUES.md#trace-001) oraz [TRACE-002](../Ashtrace/ISSUES.md#trace-002) wybierz migrację, wersję i prosty przykład obwiedni oraz akceptacji. Wyjaśnij, że nearest oznacza najbliższy AABB, o ile kontrakt nie zwraca rzeczywistej powierzchni.

**Warunki zamknięcia:**

- [x] Oba quick starty kompilują się na docelowych zależnościach.
- [x] API/README są zgodne co do granicy candidate/exact i statusu implementacji.
- [x] Zapisano wpływ aktualizacji Ashcore/Ashgrid/Ashspace; opublikowane artefakty nie są nadpisywane.

**Powiązania:** [CORE-006](../Ashcore/ISSUES.md#core-006), [GRID-006](../Ashgrid/ISSUES.md#grid-006), [SPACE-005](../Ashspace/ISSUES.md#space-005); brak wymogu tworzenia Ashmesh.

**Wynik 2026-09-10:** Wsparcie obejmuje api oraz publiczne staticindex/dynamic, w tym konstruktor BVH; internal wyłączono jawnie. Oba kompletne przykłady README kompilują się i działają z gotowymi JAR podczas verify. javap: 19 publicznych typów/138 deklaracji, zero usunięć lub zmian. Wybrano 2.0.0-SNAPSHOT dla ostrzejszej walidacji i zależności Ashspace 2.0; starego 1.0.0 nie nadpisano. Zmiany semantyki, wersje zależności i ograniczenia porównania opisano w VERIFICATION.md.

<a id="trace-006"></a>

## TRACE-006 — Dostosować CI, pakowanie i dowody wydania

**Status:** GOTOWE
**Priorytet:** P1  
**Dowód:** INSPEKCJA  
**Kontrakt:** sekcje 2, 4.5, 6

**Gdzie:** [pom.xml](pom.xml), [.github/workflows/maven.yml](.github/workflows/maven.yml), [.github/workflows/publish.yml](.github/workflows/publish.yml), [README.md](README.md).

**Stan podczas przeglądu:** CI uruchamia mvn -B package, a kontrakt wymaga clean verify. POM ustawia source/target 21 bez jawnego przypięcia maven-compiler-plugin; Javadoc ma doclint=none i failOnError=false. Profil central istnieje, lecz pokazany workflow deploy nie aktywuje go i publikuje do GitHub Packages. Początkowa gałąź: main; CI filtruje master. To rozbieżność lokalnego stanu z konfiguracją: potwierdź faktyczną gałąź domyślną na serwerze przed zmianą filtrów.

**Znaczenie:** Zielony wynik obecnego CI nie jest dowodem wykonania całej bramki jakości ani obecności artefaktu w Maven Central. Brak automatyzacji Central nie dowodzi braku publikacji ręcznej.

**Praca do wykonania:** Ustaw rzeczywistą bramkę clean verify, dobierz przypięty compiler plugin i release 21, sprawdź generowanie dokumentacji oraz jednoznaczną identyfikację artefaktów. Potwierdź utrzymywane gałęzie, docelowe wersje zależności i sposób publikacji do każdej używanej destynacji. JUnit pozostaw w test scope; nie usuwaj go w imię niezależności produkcyjnej.

**Warunki zamknięcia:**

- [x] Zapisano wynik mvn -B clean verify z wymaganymi testami oraz wersje JDK/Maven; CI obejmuje faktycznie utrzymywane gałęzie i PR-y.
- [x] Główny JAR, sources, Javadoc i wymagane zasoby są sprawdzone. Błędny Javadoc nie jest po cichu uznawany za poprawny; nie trzeba przy tym mechanicznie włączać każdej reguły stylistycznej doclint.
- [x] Wskazano używane cele publikacji, tag/wersję i dowody dostępności albo jawnie pozostawiono publikację jako niezweryfikowaną. Sam deploy nie służy jako test poprawek.
- [x] Sprawdzono efektywne zależności i ich scope; test integracyjny korzysta z zamierzonej wersji dolnej warstwy, a nie przypadkowej starej kopii z lokalnego Maven.

**Powiązania:** Wspólny wzorzec: [TEMPLATE-001](../Ashtemplate/ISSUES.md#template-001) i [TEMPLATE-002](../Ashtemplate/ISSUES.md#template-002). Tę korektę można wykonać niezależnie od napraw algorytmów. Istniejącego numeru wydania nie nadpisuj innym artefaktem.

**Wynik 2026-09-10:** clean verify PASS: 62 + 2 testy, JDK 21.0.12.1, Maven 3.9.16, release 21. Javadoc all,-missing/failOnError=true PASS; sprawdzone dokładne main/sources/Javadoc, LICENSE/NOTICE, współrzędne i brak JUnit w main. SPI DDA sprawdzone z JAR zależności. CI obejmuje wszystkie branche i PR-y; zdalny HEAD potwierdzono jako main. Oba workflow przeszły actionlint. dependency:tree/effective-pom potwierdziły trzy zamierzone snapshoty compile i JUnit test. Zdalne CI oraz cele Packages/Release/Central pozostają jawnie niezweryfikowane; hosted runner wymaga udostępnienia zależności rozwojowych. Zamknięcie dotyczy lokalnych kryteriów korekty, nie gotowości wydania. Dowody i następne kroki: VERIFICATION.md.

<a id="trace-007"></a>

## TRACE-007 — Obsłużyć siatkę z ramką, początkiem i rozmiarem komórki

**Status:** GOTOWE

**Decyzja:** Rozszerzenie zaakceptowane przez użytkownika po ocenie kompletności biblioteki.
Fabryka `FrameGridRayTracer3.forGrid` przyjmuje `FrameGridSpaceMapper3`; odpowiednie fabryki
pipeline zasłaniania korzystają z tego samego grafu. Oryginalne konstruktory nadal oznaczają
jednostkową siatkę świata. Mapowanie współrzędnych należy do Ashspace, a przechodzenie komórek
do Ashgrid; Ashtrace łączy te operacje i przelicza odległości wyników na jednostki świata.

**Warunki zamknięcia i wynik 2026-09-10:**

- [x] Rozmiar komórki, jej początek, translacja i rotacja nie zmieniają jednostek wyniku.
- [x] Callback i wynik używają indeksów wskazanej siatki; sprawdzono także ujemne współrzędne i granice dziesiętne.
- [x] Kolejne zapytania odzwierciedlają ruch ramki; snapshot zachowuje dawny stan.
- [x] Oba pipeline zasłaniania stosują limit w jednostkach świata. Niereprezentowalne przeliczenia są odrzucane.
- [x] Sześć testów `MappedGridTraceIntegrationTest` i kompletny przykład README przeszły na JDK 21 i 25.

<a id="trace-008"></a>

## TRACE-008 — Zwracać wejście i wyjście z geometrii dostawcy

**Status:** GOTOWE

**Decyzja:** `RayIntersector3` dostarcza pełne skończone przedziały `RayIntersection3` dla kandydata.
`FrameExactRayTracer3` i `FrameOccludedExactRayTracer3` wybierają/przycinają je i zwracają
`ExactTraceHit3` z oboma punktami świata oraz znacznikami rzeczywistych powierzchni. Przedział
może zaczynać się przed początkiem promienia lub kończyć za limitem; przycięty punkt nie jest
przedstawiany jako powierzchnia. Wiele przedziałów jednego obiektu zachowuje przerwy/puste wnętrze.

`firstHit` wybiera najmniejsze wejście, następnie wyjście; `lastHit` największe wyjście, następnie
wejście. Do wyjścia z pierwszego trafionego przedziału służy jego `worldExitPoint`, a nie osobne
globalne `lastHit`. Stare `NarrowPhase3` i `TraceHit3` zachowują znaczenie obwiedni. Testy geometrii
pozostają u dostawcy/Ashcore, a penetracja fizyczna i obrażenia poza Ashtrace.

**Warunki zamknięcia i wynik 2026-09-10:**

- [x] Wejście/wyjście, start wewnątrz, koniec zasięgu, styczność, wiele części i odwrotny kierunek mają jawne wyniki.
- [x] Wybór rzeczywistych trafień może odwrócić kolejność luźnych obwiedni; zasłonięta powierzchnia jest odrzucana.
- [x] Sprawdzono transformację do świata, remisy, segmenty, niepoprawne przedziały i użycie callbacku po jego zakończeniu.
- [x] Czternaście testów `ExactRayTracerTest` i wykonywalny przykład sfery w README przeszły na JDK 21 i 25.
- [x] javap zachował 19 istniejących publicznych typów / 138 deklaracji; stary skompilowany klient działa z nowym JAR-em.

<a id="trace-009"></a>

## TRACE-009 — Dodać zatrzymywanie zapytań i ponowne użycie bufora

**Status:** GOTOWE

**Decyzja:** Dodano `visitRay`/`anyRay` oraz zapytania `anyHit`. Cztery indeksy zatrzymują dalsze
odwiedzanie/testowanie kandydatów na żądanie. Hash nadal zbiera i sortuje uchwyty przed pierwszym
callbackiem; dynamiczny BVH nadal odbudowuje brudny snapshot. Domyślny adapter zachowuje zgodność
starych implementacji `queryRay`, ale nie zatrzymuje ich wewnętrznej pracy.

`TraceQueryBuffer3` zachowuje pojemność list między zapytaniami, usuwa referencje do obiektów
w `finally` i odrzuca aktywne ponowne użycie. Usunięto pomocnicze opakowania kandydatów i ich
dodatkowe kopie z oryginalnego pipeline, zachowując stabilny porządek wyników i callbacków.

**Warunki zamknięcia i wynik 2026-09-10:**

- [x] Zestawy i odległości `visitRay` zgadzają się z `queryRay` we wszystkich czterech indeksach.
- [x] Zatrzymywanie, powtarzalna kolejność, wyjątki i ponowne użycie bufora są objęte testami.
- [x] Stara implementacja skompilowana przed dodaniem metod działa przez nowy domyślny adapter.
- [x] Zapisano pomiar przed/po dla 5000 nakładających się AABB oraz ograniczenia tego pomiaru w VERIFICATION.md.

Nie zadeklarowano braku alokacji ani uniwersalnego przyspieszenia. Przebudowa dynamicznego BVH
wymagałaby osobnego pomiaru obciążenia z mutacjami; ten pomiar jej nie uzasadnia.

<a id="trace-010"></a>

## TRACE-010 — Odtworzyć build na hosted CI i potwierdzić gotowość wydania

**Status:** ZABLOKOWANE — potrzebna potwierdzona dystrybucja poprawionych zależności do runnera.

**Wykonane lokalnie:** `scripts/verify-local.ps1` sprawdza SHA-256 wszystkich trzech JAR-ów i POM-ów
według `scripts/development-dependencies.json`, instaluje je wyłącznie do `.verification/repository`
w Ashtrace i uruchamia `clean verify dependency:tree`. Nie buduje ani nie zmienia bibliotek źródłowych.
Przeszedł na JDK 25; pełna bramka przeszła również na JDK 21. Każda bramka: 85 testów + 2 testy
artefaktów, bez błędów/pominięć. Cztery przykłady działają z pakietów, Javadoc i actionlint przechodzą.

**Pozostałe warunki:**

- [ ] Udostępnić runnerowi dokładnie wskazane zależności albo zweryfikowane wersje wydaniowe.
- [ ] Uruchomić hosted CI i zapisać wynik/URL dla docelowego commita.
- [ ] Przed publikacją wybrać nieużyte finalne współrzędne oraz potwierdzić tag, destynację i dostępność artefaktów.

Zdalne HEAD-y niższych bibliotek różnią się od lokalnych commitów korekt. Sam checkout ich domyślnych
gałęzi nie odtwarza zestawu użytego w testach. Nie zmieniano/pushowano tych bibliotek, nie uruchamiano
deploy i nie uznano lokalnych snapshotów za potwierdzone wydanie. Instrukcja odtworzenia oraz dowody:
[VERIFICATION.md](VERIFICATION.md#2026-09-10--mapped-grids-shape-intervals-and-query-reuse).

<a id="trace-011"></a>

## TRACE-011 — Sprawdzić integrację z oryginalnymi testami dolnych warstw

**Status:** GOTOWE

**Zakres:** użytkownik wskazał testy innych bibliotek jako dodatkowe źródło sprawdzenia integracji.
Utworzono gałąź `test/ashtrace-blackframe-integration-20260910` i checkpoint `8d70f81` z bazy `a6bf6f1`.
Dodano 12 testów w Ashtrace opartych na scenariuszach Ashcore/Ashgrid/Ashspace oraz skrypt uruchamiający
54 testy z 11 oryginalnych plików, kopiowanych bez zmian do izolowanego katalogu w Ashtrace.

**Wykryty błąd:** oryginalny `GridMappingIntegrationTest` nie przeszedł dla ujemnego offsetu, którego
iloraz przez rozmiar komórki zaokrągla się do -0.0. Ashgrid wskazywał komórkę -1, a Ashspace 0.
Po dodatkowej zgodzie użytkownika poprawiono właściciela mapowania w osobnym repozytorium Ashspace:
[SPACE-011](../Ashspace/ISSUES.md#space-011), commit `f652173`. Przypadek jest skrajny numerycznie,
ale narusza podstawowy kontrakt komórki/zakresu i blokował zgodność zestawu. Ashgrid/Ashcore nie zmieniano.

Ashtrace odrzuca utratę niezerowego offsetu promienia do zera przed wywołaniem occupancy, ponieważ
odtworzenie strony granicy dla indeksowania nie odtwarza odległości przebiegu promienia. Nie dodano
epsilona ani nie zmieniono reguł dla reprezentowalnych współrzędnych. Manifest zależności wskazuje
nowy, sprawdzony JAR i POM Ashspace; jego zależności także odpowiadają Ashcore 1.1 / Ashgrid 1.3 snapshot.

- [x] Wszystkie cztery indeksy porównano z obliczeniami przecięć/sweep/proximity Ashcore.
- [x] Rzeczywiste wejście/wyjście sfery pochodzi z obliczeń Ashcore; sprawdzono styczność, start wewnątrz i ramki zagnieżdżone.
- [x] Sprawdzono sześć kierunków osi, osiem oktantów, remisy DDA, przycinany traverser, ruchomy magazyn wokseli i lokalne współrzędne przy dużym przesunięciu świata.
- [x] Oryginalnych 54 testów nie poprawiano ani nie pomijano; po naprawie Ashspace wszystkie przechodzą.
- [x] JDK 21 i 25: po 97 testów Ashtrace + 2 testy artefaktów + 54 testy zależności, bez błędów/pominięć.
- [x] Ashspace: pełne 74 + 2 PASS na JDK 21; javap obu bibliotek potwierdza brak usunięć publicznych deklaracji.

Pierwsza asercja testu sfery wymagała tolerancji 1e-12 dla danych jednostkowej skali, zgodnej z
testami Ashcore (1.0 wobec 0.9999999999999998). Dwie próby skonfigurowania pomocniczego POM zakończyły
się błędami przed testami; skrypt poprawiono. Właściwe regresje błędu mapowania zawiodły przed zmianą
i przeszły po niej. Logi, wersje, hashe, polecenia i szczegóły w [VERIFICATION.md](VERIFICATION.md).
TRACE-010 nadal wymaga udostępnienia zależności oraz wykonania zdalnego CI przed wydaniem.

## Uwagi o zakresie kolizji — 2026-09-10

Użytkownik zlecił uzupełnienie backlogu w granicach rewizji 2.0. Nowa propozycja dotyczy łączenia indeksów i zapytań z geometrią dolnych warstw. Nie zleca fizyki, kontrolera postaci ani adaptera Minecraft. Punkt odniesienia: lokalne źródła odczytane 2026-09-10; zgodnie z bieżącą instrukcją nie wykonywano operacji Git ani nowego checkpointu.

<a id="trace-012"></a>

## TRACE-012 — Połączyć zapytania obróconych prymitywów z indeksami AABB

**Status:** GOTOWE

**Priorytet:** P2  
**Dowód:** DECYZJA, oparta na inspekcji punktów integracji  
**Kontrakt:** sekcje 2, 3.4, 4.1–4.5, 5, 7

**Gdzie:** [RayIntersector3.java](src/main/java/nsk/nu/ashtrace/api/trace/contracts/RayIntersector3.java), [FrameExactRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameExactRayTracer3.java), [FrameOccludedExactRayTracer3.java](src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameOccludedExactRayTracer3.java), [BroadPhase3.java](src/main/java/nsk/nu/ashtrace/api/broadphase/contracts/BroadPhase3.java), [SweepQueryableBroadPhase3.java](src/main/java/nsk/nu/ashtrace/api/broadphase/contracts/SweepQueryableBroadPhase3.java), testy integracyjne i [README.md](README.md).

**Stan i znaczenie:** Indeksy wybierają kandydatów według AABB. RayIntersector3 i dokładne pipeline już przyjmują przedziały geometrii dostawcy, także wiele rozłącznych przedziałów. Istnieje przykład sfery; nie jest potrzebna ponowna implementacja tego mechanizmu. Zapytanie obwiedni nadal nie dowodzi kontaktu z obróconym kształtem.

**Praca do wykonania:** Po ustaleniu CORE-011 i konwersji SPACE-012 dodać wykonywalny przykład oraz testy obróconego prymitywu przez istniejące API. Indeks przechowuje obwiednię, a test kształtu korzysta z Ashcore i Ashspace. Preferować kompozycję obecnych interfejsów; nowy adapter rozważyć tylko dla potwierdzonego powtarzającego się kodu integracji. Algorytmów przecięcia OBB nie przenosić do Ashtrace.

Dostawca promieniowy zwraca pełne przedziały, także z wejściem przed początkiem promienia. Nie odtwarzać wyjścia z samego pierwszego t. Dla wielu części dostawca odpowiada za poprawną sumę przedziałów: zachowuje przerwy i scala nakładające się przedziały zgodnie z kontraktem. Jeden szeroki przedział nie może zasłaniać pustej przestrzeni między częściami. To kompozycja zapytania, nie system fizycznych brył złożonych.

**Granica czasu:** Zapytanie opisuje jedno stabilne ustawienie geometrii, ramek i indeksu. Po zmianie ustawienia klient odświeża powiązane dane przed następnym zapytaniem. querySweptAabb nadal opisuje translację AABB względem indeksowanego stanu. Obwiednia tylko dwóch końcowych ustawień może pominąć część łuku obrotu. Dostarczona przez klienta obwiednia całej drogi daje jedynie kandydatów, bez czasu kontaktu. Ocena ciągłych testów prymitywów należy do CORE-013.

**Warunki zamknięcia:**

- [x] Wskazano wersje/JAR-y dolnych warstw; przykład kompiluje się z pakietów i nie przedstawia proponowanych API jako już dostępnych.
- [x] Testy obejmują promień/odcinek, start wewnątrz, styczność, limit zasięgu, obrót/przesunięcie oraz trafienie w AABB bez trafienia we właściwy kształt.
- [x] Sprawdzono first/last/all, przycinanie przez voxele i pustą przestrzeń między częściami; odległości pozostają w jednostkach świata.
- [x] Cztery indeksy dają zgodne zbiory kandydatów/wyników dla tego samego modelu, z ich własnymi zasadami remisów i kolejności. Nowy spójny stan po ruchu daje nowy wynik, a wcześniejszy wynik pozostaje niezmienny.
- [x] Przykład odróżnia próbki ustawienia od ciągłego kontaktu i pokazuje obrót pomijany przez same końce kroku. Nie dodano gwarancji obrotowego sweep ani reakcji na kolizję.
- [x] Opisano pracę dostawcy, sortowanie i aktualizacje indeksu bez obietnicy bezalokacyjności; zmieniony zakres przechodzi testy i clean verify. Stare API zachowuje znaczenie obwiedni.

**Wynik 2026-09-10:** Po zamknięciu CORE-011 i SPACE-012 przyjęto Ashcore 1.2.0-SNAPSHOT
(`ddbf98c`) i nowy JAR Ashspace 2.0.0-SNAPSHOT (`ebf6b9e`); Ashgrid 1.3.0-SNAPSHOT pozostał
bez zmian. Manifest zawiera hashe JAR/POM, a skrypt sprawdza je przed instalacją do repozytorium
wewnątrz Ashtrace. Istniejący `RayIntersector3` wystarcza: przykład i siedem testów integracyjnych
łączą pełny przedział Ashcore z konwersją Ashspace, bez nowego adaptera ani algorytmu przecięcia.
Test dostawcy scala nakładanie części, zachowuje przerwy i sprawdza zasłanianie w obróconej siatce
o komórce 2. Sprawdzono także remisy według `visitRay`, aktualizację dynamicznych obwiedni oraz
przebudowę indeksów statycznych po ruchu. README/Javadoc wyjaśniają granice czasowe i jednostki.

JDK 21 i 25: po **104 testy Ashtrace + 2 testy artefaktów + 78 niezmienionych testów zależności,
PASS**, w tym pięć kompilowanych i uruchamianych przykładów README. `javap`: 26 publicznych typów,
211 deklaracji, bez usunięć. Wersja Ashtrace pozostaje robocza 2.0.0-SNAPSHOT; nie zmieniono
sygnatur ani algorytmów produkcyjnych. Prace i wyniki zapisano wyłącznie w Ashtrace. Szczegółowe
polecenia, środowiska, hashe oraz ograniczenia: [VERIFICATION.md](VERIFICATION.md#2026-09-10--oriented-box-integration-trace-012).
TRACE-010 pozostaje zablokowane dystrybucją zależności i wymaga wykonania hosted CI przed wydaniem.

**Powiązania:** [CORE-011](../Ashcore/ISSUES.md#core-011), [CORE-013](../Ashcore/ISSUES.md#core-013), [SPACE-012](../Ashspace/ISSUES.md#space-012), zamknięte [TRACE-001](#trace-001), [TRACE-008](#trace-008), [TRACE-011](#trace-011). Propozycja P2 nie zmienia blokady TRACE-010. Odrzucenie zapisać jako NIE DOTYCZY z uzasadnieniem; sam plan nie oznacza GOTOWE.

## Stan przekazania i dziennik sesji

**Na 2026-09-09:** wszystkie zadania pozostają OTWARTE. Utworzono dokumentację; nie wprowadzono korekt kodu, nie wykonano buildów bibliotek ani publikacji. Nie uznawaj samego dodania ISSUES.md za realizację żadnego zadania.

**Stan po korekcie 2026-09-10:** TRACE-001–TRACE-006 GOTOWE w zakresie opisanej korekty lokalnej. Wersja 2.0.0-SNAPSHOT, branch fix/ashtrace-contract-v2-20260910, checkpoint 70d3513. Końcowe clean verify: 62 testy + 2 testy artefaktów PASS. Zdalne CI i publikacja nie były wykonane; runner wymaga dostępności trzech rozwojowych zależności. Szczegóły, migracja, hashe JAR i pozostałe warunki wydania: [VERIFICATION.md](VERIFICATION.md).

Po kolejnej sesji dopisz wiersz i uzupełnij statusy odpowiednich zadań. Zapisz także nieudane próby i ograniczenia środowiska; nie opisuj kontroli niewykonanej jako zaliczonej.

| Data / commit | ID i decyzja | Zmiana | Polecenie / test i rzeczywisty wynik | Pozostałe zależności / następny krok |
| --- | --- | --- | --- | --- |
| 2026-09-09 / punkt odniesienia powyżej | Wszystkie: OTWARTE | Utworzenie planu korekt | Inspekcja statyczna; testów bibliotek nie uruchomiono | Rozpocząć od wskazanego P1 |
| 2026-09-10 / commit dodający ten wpis; checkpoint 70d3513 | TRACE-001–TRACE-006: GOTOWE lokalnie | Korekty nearest/slab/hash, kontrakty obwiedni/zasłaniania/porządku, README/API, dependency snapshots, wersja 2.0.0-SNAPSHOT, CI i pakowanie | Bazowe 43 PASS; pierwsze regresje 2 FAIL + 1 ERROR, małe składowe 2 FAIL; końcowe clean verify 62 + 2 PASS. javap 19 typów/138 deklaracji bez usunięć; actionlint PASS. Pełne wersje, SHA i logi opisane w VERIFICATION.md | Udostępnić docelowe zależności dla hosted CI, uruchomić CI, przed wydaniem zweryfikować nowe współrzędne/tag/destynacje. Nie wykonywano push/deploy ani zmian w innych bibliotekach |
| 2026-09-10 / commit rozszerzenia; checkpoint 5356ab5, baza 3be89a0 | TRACE-007–009 GOTOWE; TRACE-010 ZABLOKOWANE zależnościami zdalnego CI | Mapowana siatka, wejście/wyjście geometrii, first/last/any, bufor, cztery przykłady i skrypt odtwarzania buildu | Pierwsza kompilacja nowych testów: błędny import SquareXZChunkScheme, poprawiony na pakiet implementation. Następnie 82 PASS; końcowe JDK 21 i 25: po 85 + 2 PASS. javap i zgodność starego klienta PASS; actionlint PASS. Pomiar alokacji zapisany w VERIFICATION.md | Udostępnić zweryfikowane zależności i uruchomić hosted CI przed wydaniem; brak push/deploy i zmian poza Ashtrace |
| 2026-09-10 / commit integracji; checkpoint 8d70f81 | TRACE-011 GOTOWE; SPACE-011 poprawiony w f652173 po rozszerzeniu zgody | 12 scenariuszy integracyjnych, 54 niezmienione testy zależności, walidacja zaniku współrzędnych i nowy manifest Ashspace | Przed poprawką: 53/54 oryginalnych testów PASS, regresja Ashtrace FAIL, regresje Ashspace 3/7 FAIL. Po poprawce: JDK 21 i 25 po 97 + 2 + 54 PASS; Ashspace JDK 21: 74 + 2 PASS. Publiczne API zachowane | Zdalne CI/publikacja nadal niewykonane; nowe zależności wymagają dystrybucji |

### Przegląd zakresu kolizji 2026-09-10

**Historyczny stan przeglądu backlogu:** TRACE-012 było otwartą propozycją P2, nie potwierdzonym błędem ani warunkiem wydania ówczesnego API. Przegląd nie zmieniał statusów TRACE-001–TRACE-011, w tym blokady TRACE-010. Aktualny wynik TRACE-012 zapisano powyżej i poniżej.

| Data / commit | ID i decyzja | Zmiana | Polecenie / test i rzeczywisty wynik | Pozostałe zależności / następny krok |
| --- | --- | --- | --- | --- |
| 2026-09-10 / bez operacji Git, zgodnie z instrukcją użytkownika | TRACE-012: OTWARTE, P2 / DECYZJA | Integracja obróconych prymitywów z obecnym API i granice zapytań w czasie; wyłącznie backlog | Inspekcja źródeł; kontrola struktury, odnośników i zachowania wcześniejszej treści. Testów bibliotek i buildów nie uruchamiano | Najpierw CORE-011 i część OBB w SPACE-012; TRACE-010 pozostaje zablokowane. |

### Integracja obróconych prymitywów 2026-09-10

**Stan aktualny:** TRACE-012 GOTOWE; TRACE-010 nadal ZABLOKOWANE. Gałąź
`fix/ashtrace-rotated-primitives-20260910`, checkpoint `bbe70ff`, baza `e07168f`.
Zachowano dotychczasowe API i znaczenie zapytań obwiedni. Nie wykonywano zmian ani buildów
w dolnych bibliotekach; ich gotowe artefakty i wybrane źródła testów odczytano do integracji.

| Data / commit | ID i decyzja | Zmiana | Polecenie / test i rzeczywisty wynik | Pozostałe zależności / następny krok |
| --- | --- | --- | --- | --- |
| 2026-09-10 / commit zawierający ten wpis; checkpoint `bbe70ff` | TRACE-012 GOTOWE | Nowe zależności OBB z manifestem, 7 scenariuszy dla czterech indeksów, piąty przykład README, kontrakty przedziałów i ruchu; poszerzenie kopii testów dolnych warstw z 11 do 14 plików | Baseline 97 PASS. Nowe 7 PASS. `verify-blackframe-tests.ps1`: JDK 21 i 25 po 104 + 2 + 78 PASS; Javadoc i przykłady z JAR PASS; javap 26 typów / 211 deklaracji bez usunięć. Początkowy Maven w sandboxie nie uruchomił procesu; te same narzędzia uruchomione z eskalacją działały poprawnie | TRACE-010: udostępnić wskazane zależności runnerowi i wykonać hosted CI dla docelowego commita. Zdalne CI/publikacja niewykonane; szczegóły i hashe w VERIFICATION.md |
