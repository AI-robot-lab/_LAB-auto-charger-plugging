# Instrukcja deweloperska: Unitree G1 Charging Mission

Ten dokument opisuje proces konfiguracji środowiska, budowania kodu oraz uruchamiania stacku technologicznego dla robota Unitree G1 EDU. Projekt wykorzystuje **ROS 2 (Humble/Jazzy)** i architekturę modułową opartą na **Action Servers**.

## 1. Wymagania wstępne (Prerequisites)

Przed rozpoczęciem pracy upewnij się, że Twój system spełnia poniższe wymagania:

*   **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish).
*   **ROS 2 Distro:** Humble Hawksbill (lub Jazzy Jalisco).
*   **Unitree SDK 2:** Zainstalowane i skonfigurowane w systemie (wymagane do komunikacji z `low_level_interface`).
*   **Python:** Wersja 3.10+.
*   **Build System:** `colcon`.

## 2. Konfiguracja workspace

Wykonaj poniższe kroki w terminalu, aby przygotować przestrzeń roboczą.

### Krok 1: Klonowanie repozytorium
Utwórz katalog roboczy (jeśli jeszcze go nie masz) i sklonuj kod:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/AI-robot-lab/unitree-g1-auto-charger-plugging.git
```

### Krok 2: Instalacja zależności
Użyj narzędzia `rosdep`, aby automatycznie zainstalować brakujące biblioteki systemowe zdefiniowane w plikach `package.xml`:

```bash
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y
```

### Krok 3: Budowanie projektu (Build)
Skompiluj wszystkie pakiety używając `colcon`. Flaga `--symlink-install` pozwala na edycję skryptów w Pythonie bez konieczności ponownej kompilacji.

```bash
colcon build --symlink-install
```

Jeśli kompilacja zakończy się sukcesem, zobaczysz komunikat: `Summary: X packages finished`.

### Krok 4: Source environment
Aby system widział Twoje nowe pakiety, musisz wykonać *source* pliku instalacyjnego:

```bash
source install/setup.bash
```
*(Zalecamy dodanie tej komendy do pliku `~/.bashrc`)*.

---

## 3. Architektura i pakiety (Packages Overview)

Projekt jest podzielony na niezależne pakiety. Poniżej znajduje się opis ich odpowiedzialności oraz główne `nodes`.

### `mission_control` (Mózg/ logika systemu)
Zawiera główną maszynę stanów (**State Machine**), która zarządza przebiegiem misji.
*   **Main Node:** `state_machine_action_client.py`
*   **Funkcja:** Działa jako **Action Client**. Wysyła cele (Goals) do innych podsystemów.
*   **Workflow:** `NAV_TO_STATION` $\to$ `DETECT_HANDLE` $\to$ `MANIPULATE_GRASP` $\to$ `NAV_TO_CAR`.

### `perception` (Percepcja/ wizja komputerowa)
Odpowiada za przetwarzanie obrazu z kamer RealSense/Unitree.
*   **Main Node:** `perception_action_server.py`
*   **Action Server:** Obsługuje `Detect.action`.
*   **Input:** `/camera/color/image_raw`
*   **Output:** Pozycja uchwytu `geometry_msgs/Pose`.

### `navigation` (Nawigacja/ przemieszczanie się)
Odpowiada za przemieszczanie się robota po laboratorium.
*   **Main Node:** `navigation_action_server.py`
*   **Action Server:** Obsługuje `Navigate.action`.
*   **Integracja:** Wykorzystuje Unitree SDK do wysyłania komend prędkości (`cmd_vel`).

### `manipulation` (Manipulacja/ ruchy ramion)
Odpowiada za planowanie ruchu rąk i chwytanie.
*   **Main Node:** `manipulation_action_server.py`
*   **Action Server:** Obsługuje `Manipulate.action`.
*   **Logika:** Oblicza IK (Inverse Kinematics) dla zadania `grasp_handle` lub `insert_plug`.

### `charging_interfaces`
Pakiet zawierający wyłącznie definicje wiadomości `.msg` i akcji `.action`.
*   `Navigate.action`
*   `Manipulate.action`
*   `Detect.action`

---

## 4. Uruchamianie (Running the Project)

### Tryb symulacji (Mock Mode)
Możesz uruchomić system bez fizycznego robota. W tym trybie węzły `navigation` i `manipulation` symulują działanie (czekają 2 sekundy i zwracają `success`).

1. Uruchom serwery akcji oraz klienta misji w osobnych terminalach:
   ```bash
   ros2 run navigation navigation_action_server
   ros2 run manipulation manipulation_action_server
   ros2 run perception perception_action_server
   ros2 run mission_control state_machine_action_client
   ```

2. Powinieneś zobaczyć w logach konsoli sekwencję:
   ```text
   [INFO] [state_machine_action_client]: State: NAV_TO_STATION
   [INFO] [navigation_action_server]: Received navigation goal to pose: [2.0, 0.0, 0.0]
   [INFO] [navigation_action_server]: Navigation goal succeeded
   [INFO] [state_machine_action_client]: State: DETECT_HANDLE
   ```

### Tryb rzeczywisty (Real Robot)
**UWAGA:** Wymaga podłączenia do sieci robota i trzymania E-Stop w ręku.

1. Upewnij się, że masz połączenie z robotem (ping `192.168.123.xx`).
2. Uruchom sterowniki sprzętowe:
   ```bash
   ros2 launch g1_hardware hardware_interface.launch.py
   ```
3. W osobnym terminalu uruchom logikę misji:
   ```bash
   ros2 run mission_control state_machine_action_client
   ```

---

## 5. Workflow pracy z kodem

### Dodawanie nowej funkcjonalności
1. Utwórz nową gałąź (`feature branch`):
   ```bash
   git checkout -b feature/improved-vision
   ```
2. Wprowadź zmiany w kodzie (np. w `perception/src/yolo_detector.cpp`).
3. Przed wysłaniem zmian sformatuj kod (zgodnie z `ament_lint`):
   ```bash
   colcon test --packages-select perception
   ```
4. Wyślij zmiany (Push) i utwórz **Pull Request** na GitHubie.

### Debugowanie
Użyj narzędzia `rqt_graph`, aby zweryfikować połączenia między węzłami i akcjami:
```bash
rqt_graph
```
Powinieneś widzieć `state_machine_action_client` połączony liniami akcji z węzłami `perception`, `navigation` i `manipulation`.
