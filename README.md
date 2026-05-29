# Pick & Place Autonomo con Robot TIAGO (7-DOF)

Questo repository contiene l'implementazione completa di un'applicazione di Pick & Place autonomo sequenziale di due oggetti (una lattina di Coca-Cola e un tubo di Pringles) posizionati su un tavolo, eseguito tramite il robot di servizio TIAGO dotato di un braccio manipolatore ridondante a 7 gradi di libertà (DOF).

Il progetto è sviluppato in ambiente **ROS 2 (Humble)** su sistema operativo **Ubuntu Linux 22.04 LTS** con simulazione fisica dinamica in **Gazebo** e monitoraggio visivo in **RViz**.

---

## Architettura del Progetto

Il workspace ROS 2 (`T2_G_6`) è strutturato nei seguenti package principali:

*   **`aruco_pkg`**: Gestisce il rilevamento dei marker ArUco (`DICT_6X6_250`, dimensione fisica 6 cm) per localizzare gli oggetti e le posizioni di rilascio rispetto al sistema di riferimento della base (`base_footprint`).
*   **`pick_and_place`**: Contiene la macchina a stati asincrona che coordina la manipolazione. I file principali sono:
    *   `pick_place_pringles.py`: Nodo definitivo basato su ROS 2 Action Clients e allontanamento a 3 fasi.
    *   `pick_place_cola.py` / `pick_place_action.py`: Nodi per la presa della lattina di Coca-Cola.
*   **`move_head_action`**: Implementa il server e il client per il controllo asincrono del movimento della testa del robot durante lo sweep iniziale.
*   **`my_robot_description`**: Contiene la descrizione del robot e il file URDF (`tiago_robot.urdf`) adoperato per il calcolo cinematico.
*   **`my_robot_interfaces`**: Definisce le interfacce ROS 2 personalizzate, inclusa l'Action `MoveHead.action`.
*   **`lancio_progetto`**: Contiene i file di lancio (es. `app.launch_task2.py`) per avviare i nodi in sequenza.

---

## Innovazioni Tecniche Affrontate

### 1. Percezione Visiva "On-Demand" (Spegnimento Camera)
Per minimizzare il carico computazionale e azzerare le oscillazioni di lettura dovute alle vibrazioni del braccio in movimento, è stata implementata una strategia di spegnimento controllato:
1.  **Inizializzazione**: Sottoscrizione temporanea a `/camera_info` per estrarre la matrice intrinseca $K$, disattivata subito dopo l'acquisizione.
2.  **Scansione Attiva**: Sweep angolare della testa. OpenCV individua i 4 marker stabili e calcola i vettori geometrici TF.
3.  **Spegnimento della Camera**: Rilevati stabili i 4 marker, il nodo distrugge la subscription all'immagine video per liberare la CPU (riduzione del carico di circa il 65%).
4.  **Republisher a 10Hz**: Un timer ROS asincrono ri-pubblica in loop le ultime pose valide memorizzate in cache come TF statici stabili, garantendo riferimenti immuni da oscillazioni visive.

### 2. Prevenzione delle Singolarità Cinematiche
Il braccio a 7 DOF presenta infiniti punti di singolarità in prossimità dei limiti fisici del tavolo. Per garantire la stabilità matematica dell'inversione cinematica, sono state adottate le seguenti soluzioni:
*   **Risolutore Pseudo-Inverso**: Caricamento del modello URDF tramite `robotics-toolbox-python` ed esecuzione del metodo numerico Newton-Raphson (`ik_NR`) con Jacobiano pseudo-inverso (`pinv=True`).
*   **Offset Cartesiani Calibrati**: Approccio segmentato in tre fasi cartesiane verticali sull'asse Z (posa di pre-pick a Z elevata di sicurezza, discesa verticale controllata per imboccare l'oggetto, sollevamento netto post-presa prima di avviare traslazioni orizzontali).
*   **Filtro anti-salto a 2pi**: Algoritmo software per intercettare i salti caotici di giunto dell'articolazione del polso durante il calcolo IK, normalizzando i target ed eliminando rotazioni superflue.

### 3. Svincolo Ibrido in Tre Fasi (Pringles Retreat)
Il tubo di Pringles presenta un'altezza critica (22 cm), che dimezza lo spazio utile di manovra e causa collisioni con il gomito in caso di allontanamento lineare standard. È stata implementata una manovra di retreat ibrida:
*   **Fase 1 (RETREAT 1)**: Allungamento lineare del torso (`torso_lift_joint`) fino a quota 0.35 metri tenendo fermo il braccio. Questo allontana la spalla dal tavolo e solleva l'intero braccio sopra l'altezza delle Pringles.
*   **Fase 2 (RETREAT 2)**: Allontanamento cartesiano ad anello aperto sui giunti suddiviso in **15 step temporizzati** (eseguiti in modo asincrono ogni 5.0 secondi). A ogni passo vengono applicati incrementi costanti: `arm_2_joint` += 0.02 rad, `arm_5_joint` -= 0.04 rad, `arm_6_joint` += 0.02 rad, tracciando una traiettoria parabolica di svincolo fluida ed esente da stalli cinematici.
*   **Fase 3 (RETREAT 3)**: Comando diretto in posizione per i giunti primari (`arm_1_joint` a 0.8 rad e `arm_3_joint` a -1.5 rad) per ripiegare il braccio in posa compatta prima del saluto finale.

---

## Analisi Dinamica e Fasi Temporali

Nel report dinamico (`report_giunti_filtrati.pdf`) sono chiaramente riscontrabili le quattro fasi temporali marcate cromaticamente:

1.  **Scanning ArUco (0-20s) [Azzurro]**: I giunti del braccio sono statici nella posa di home, mentre oscillano unicamente i giunti del collo per inquadrare il tavolo.
2.  **Pick & Place Cola (20-95s) [Arancio]**: Profili di posizione e velocità continui e privi di oscillazioni a testimonianza di una presa fluida.
3.  **Pick & Place Pringles (95-218s) [Viola]**: Mostra chiaramente i 15 scalini di posizione temporizzati ogni 5 secondi durante il Retreat 2 discreto. Le accelerazioni mostrano micro-impulsi controllati e smorzati.
4.  **Wave (218-240s) [Rosso]**: Movimento armonioso finale di saluto (Wave) eseguito a torso eretto (quota 0.35m) per confermare il completamento del ciclo di lavoro.

---

## Come Avviare il Progetto

Assicurarsi di aver compilato il workspace nel proprio ambiente ROS 2:

```bash
cd ~/Progetto_6_ws
colcon build --symlink-install
source install/setup.bash
```

Avviare il file di lancio del Task 2:

```bash
ros2 launch lancio_progetto app.launch_task2.py
```
