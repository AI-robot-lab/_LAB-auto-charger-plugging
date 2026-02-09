# Przewodnik współpracy (contributing guide)

Witaj w **AI Robot Lab**! 
Cieszymy się, że chcesz dołożyć swoją cegiełkę do rozwoju naszych robotów humanoidalnych. Aby współpraca przebiegała sprawnie, a nasz kod (i roboty) pozostały w jednym kawałku, prosimy o przestrzeganie poniższych zasad.

---

## Model pracy z git (Git Flow)

Stosujemy uproszczony model **Git Flow**. Wszystkie prace rozwojowe odbywają się na gałęziach tematycznych.

1. **`main`** – Kod stabilny, gotowy do demonstracji i testów hardware'owych.
2. **`develop`** – Główna gałąź integracyjna. Tu trafiają wszystkie nowe funkcjonalności.
3. **Gałęzie tematyczne:** Twórz je z `develop` według schematu:
   - `feature/nazwa-funkcji` (np. `feature/g1-hand-control`)
   - `bugfix/opis-bledu` (np. `bugfix/lidar-noise-filter`)
   - `docs/co-zmieniono` (np. `docs/api-update`)

---

## Standard "commitów"

Używamy **Conventional Commits**, aby historia zmian była czytelna dla ludzi i automatów:

- `feat:` – nowa funkcjonalność.
- `fix:` – naprawa błędu.
- `docs:` – zmiany w dokumentacji.
- `refactor:` – zmiany w kodzie, które nie zmieniają działania (np. zmiana nazw zmiennych).
- `test:` – dodanie lub poprawa testów.

*Przykład:* `feat(locomotion): add gait stabilization for uneven terrain`

---

## Standardy kodowania (Style Guide)

Nie negocjujemy jakości kodu. Przed wysłaniem PR upewnij się, że Twój kod jest sformatowany:

### Python
- Stosujemy **PEP 8**.
- Sugerowane narzędzia: `black` (formater) oraz `flake8` (linter).
- Pamiętaj o czytelnych Docstringach (rekomendowany styl Google lub NumPy).

### C++
- Stosujemy **Google C++ Style Guide**.
- Używaj `clang-format` z konfiguracją projektu.
- Pamiętaj o inteligentnych wskaźnikach (`std::unique_ptr`, `std::shared_ptr`) – unikaj surowych wskaźników tam, gdzie to możliwe.

### ROS 2
- Nazwy tematów (topics) i usług (services) piszemy w `snake_case`.
- Zawsze definiuj interfejsy (msg/srv/action) w osobnych pakietach `_msgs`.
- Każdy node powinien mieć zdefiniowane parametry w pliku `.yaml`.

---

## Zasada "Simulation First"

Zanim Twój kod trafi na fizycznego robota Unitree G1:
1. **Musi przejść kompilację** bez błędów i ostrzeżeń.
2. **Musi zostać przetestowany w symulacji** (Gazebo/MuJoCo).
3. **Musi uzyskać "Approve"** w Pull Requeście od co najmniej jednego członka zespołu.

> [!CAUTION]
> Testowanie niezweryfikowanego kodu bezpośrednio na hardware jest dopuszczalne wyłącznie pod nadzorem PI i z ręką na wyłączniku bezpieczeństwa (E-Stop).

---

## Proces Pull Request (PR)

1. Wypchnij zmiany na swoją gałąź.
2. Otwórz PR do gałęzi `develop`.
3. Wypełnij [Szablon Pull Requesta](.github/pull_request_template.md).
4. Oznacz odpowiednie osoby jako recenzentów.
5. Po uzyskaniu akceptacji i przejściu testów CI, wykonaj `Squash and merge`.

---

## Potrzebujesz pomocy?

Jeśli utkniesz:
1. Przeszukaj **Issues** (może ktoś już miał ten problem).
2. Zadaj pytanie na kanale `#dev-humanoid` lub `#dev-affective` na naszym komunikatorze.
3. Jeśli błąd dotyczy sprzętu, niezwłocznie oznacz go etykietą `hardware`.

**Dziękujemy za Twój wkład w budowanie przyszłości robotyki!**
