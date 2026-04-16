# Analisi dei Risultati di Simulazione: Formazione, Localizzazione e Forze Virtuali

Questo documento analizza le prestazioni di una flotta di **3 veicoli Differential Drive** impegnata in una missione di navigazione in formazione (V-shape) lungo una traiettoria sinusoidale. Il sistema utilizza un Filtro di Kalman Esteso (EKF) per la localizzazione individuale e un approccio a Campi Potenziali Artificiali per il controllo di formazione.

L'esperimento è caratterizzato da un'architettura sensoriale asimmetrica: il veicolo Master (V1) è dotato di un GPS ad alta precisione, mentre gli Slave (V2, V3) montano un GPS standard.

---

## 1. Analisi dell'Errore di Stima (Diagnostica EKF)

I grafici relativi agli errori di stima ($X$, $Y$, $\theta$) mostrano la differenza tra lo stato reale (ground truth) e lo stato stimato dall'EKF per ogni veicolo.

### 1.1 Il Transitorio Iniziale (Convergenza)
In tutti i grafici (specialmente evidenti nelle prime colonne di $X$ e $Y$), si nota un forte picco all'istante $t=0$, seguito da una rapida discesa verso lo zero entro i primi **5 secondi**. 
* **Dinamica:** Questo comportamento è la risposta del filtro all'errore di inizializzazione introdotto deliberatamente nel codice (sfalsamento di **1 m** in $X$ e **-1 m** in $Y$). 
* **Valutazione:** L'EKF dimostra un'eccellente prontezza. La matrice di covarianza iniziale $P$ e le matrici di rumore $Q$ e $R$ sono ben bilanciate, permettendo al filtro di fidarsi tempestivamente delle prime misurazioni per correggere la falsa assunzione iniziale.

### 1.2 L'Impatto del GPS Differenziato (Master vs Slave)
Osservando le righe degli errori spaziali ($X$ e $Y$), emerge chiaramente la natura eterogenea della flotta:
* **Master (V1 - Blu):** L'errore a regime oscilla in una banda molto stretta, approssimativamente tra **-0.3 m** e **+0.3 m**. Questo è il risultato diretto del parametro `sigma_gps_master = 0.2`.
* **Slaves (V2 Rosso, V3 Verde):** L'errore mostra un andamento a frequenza più bassa (una "deriva" lenta) e un'ampiezza decisamente maggiore, raggiungendo picchi di oltre **$\pm 1.0$ m**. Questo riflette l'impostazione `sigma_gps_slave = 2.0`. L'EKF degli Slave fatica maggiormente a trovare la posizione esatta e si affida di più all'integrazione cinematica degli encoder e dell'IMU, che però è soggetta a deriva (drift).

### 1.3 L'uniformità dell'Orientamento ($\theta$)
L'ultima riga di grafici (Errore $\theta$) mostra un comportamento quasi identico per tutti e tre i veicoli, con un errore racchiuso in una banda di circa **$\pm 0.3$ rad**.
* **Dinamica:** Il GPS fornisce solo misure di posizione ($X, Y$), non di orientamento. L'angolo $\theta$ viene stimato unicamente tramite la fusione dei dati dell'IMU (giroscopio e magnetometro) e degli odometri (encoder). Poiché i parametri di rumore di questi sensori (`R_imu` e `R_enc`) sono identici per tutta la flotta, la capacità dell'EKF di stimare $\theta$ è uniforme, indipendentemente dal ruolo del veicolo.

---

## 2. Analisi delle Forze Virtuali (Controllo e Sicurezza)

I grafici delle forze virtuali illustrano l'impegno di controllo richiesto per mantenere la formazione e garantire l'assenza di collisioni.

### 2.1 Sforzo di Consenso ($|F_{cons}|$)
Questo grafico rappresenta la magnitudo del vettore di correzione della velocità generato per mantenere la forma a "V".
* **Il Picco Iniziale:** All'inizio della simulazione ($t < 5$ s), la magnitudo schizza a valori altissimi (tra **15 m/s** e **20 m/s**). Poiché la flotta viene inizializzata in posizioni casuali molto distanti dalla configurazione desiderata (matrice `x0_true`), l'errore di posizione relativo $err_{ij}$ è enorme. La legge proporzionale (moltiplicata per il guadagno `K_cons = 1.0`) genera un comando di velocità violento per far "scattare" i robot nelle rispettive posizioni.
* **Fase di Regime:** Raggiunta la formazione, lo sforzo crolla a valori prossimi a **0.5 m/s**. Questo sforzo residuo non è mai perfettamente nullo a causa del rumore di stima: poiché ogni robot calcola la distanza basandosi sulla *sua* stima rumorosa della posizione altrui, i robot continuano a fare micro-correzioni continue per compensare le incertezze dei sensori (specialmente evidenti negli Slave a causa del loro GPS meno preciso).

### 2.2 Forza Repulsiva ($|F_{rep}|$)
Il grafico della forza repulsiva è la prova definitiva della sicurezza della missione.
* **Risultato:** La curva è completamente piatta a **0** per tutti i veicoli lungo l'intera simulazione.
* **Dinamica:** Questo indica che la distanza relativa tra due qualsiasi veicoli non è mai scesa al di sotto della soglia critica `R_safe = 1.5` m. Il controllo di consenso è stato sufficientemente reattivo e la traiettoria sinusoidale non è stata così aggressiva da "schiacciare" la formazione al punto da innescare la legge di repulsione. La flotta ha navigato in totale sicurezza spaziale.