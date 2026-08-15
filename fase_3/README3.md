# Fase 3: Navigazione in Ambiente Ostile e Localizzazione Collaborativa

## 1. Obiettivo
In questa fase la flotta deve navigare lungo il percorso nominale all'interno dell'ambiente precedentemente generato, affrontando le zone *GPS-denied*. Il sistema EKF viene espanso per supportare un'architettura di *Sensor Fusion* dinamica: attivazione del GPS all'aperto, transizione al sistema UWB (Ultra-Wideband) nelle zone d'ombra, e utilizzo della localizzazione collaborativa inter-veicolare.

## 2. Navigazione e Path-Following
Il veicolo Master (Veicolo 1) guida la formazione lungo il percorso specificato in `path_points`. Viene implementato un algoritmo di inseguimento del target virtuale (*Virtual Target Tracking*):
1. Il Master identifica un punto target sul percorso.
2. Il vettore di velocità di riferimento $V_{ref}$ viene calcolato dinamicamente per puntare verso il target.
3. Quando il Master si avvicina al target, l'indice avanza, guidando l'intera formazione (che lo segue tramite il protocollo di Consenso) lungo curve e diagonali.

## 3. Fusione Sensoriale Dinamica (EKF Adattivo)
L'Extended Kalman Filter è stato riscritto per accogliere un vettore di misure $z$ e una matrice Jacobiana $H$ di dimensioni variabili a runtime.

### 3.1 Transizione GPS -> UWB
Quando il veicolo entra in una zona d'ombra (distanza dal centro $\le R_{area}$):
- L'aggiornamento GPS viene disabilitato.
- Il veicolo interroga le ancore UWB fisse. Per ogni ancora visibile ($d \le r_{ancora}$), viene generata una misurazione di distanza:
  $$z_{uwb}^{(j)} = \sqrt{(x - X_{ancora}^{(j)})^2 + (y - Y_{ancora}^{(j)})^2} + \nu_{uwb}$$
  La riga corrispondente nella matrice Jacobiana $H$ è:
  $$H_{uwb}^{(j)} = \begin{bmatrix} \frac{x - X_{ancora}^{(j)}}{d} & \frac{y - Y_{ancora}^{(j)}}{d} & 0 & 0 & 0 \end{bmatrix}$$

### 3.2 Localizzazione Collaborativa
Per incrementare la resilienza, i veicoli condividono le proprie stime di stato $\hat{x}_i$ sulla rete. Ogni veicolo misura la distanza relativa $d_{ij}$ dai vicini entro un certo raggio di comunicazione.
Questa misura viene iniettata nell'EKF trattando il vicino come un'ancora UWB mobile, la cui posizione assunta è $\hat{p}_j$:
$$z_{collab}^{(j)} = ||p_i - \hat{p}_j|| + \nu_{rel}$$
Ciò crea un forte accoppiamento matematico: se un veicolo perde tutti i riferimenti assoluti (No GPS, No UWB), la sua stima non deriverà (dead-reckoning puro), ma rimarrà ancorata a quella del resto della flotta.


# Spiegazione chiara:

## Cosa significano le linee nel grafico?

- Tratteggiata Nera (k--): È il percorso nominale, la "rotaia invisibile" che il Master deve seguire.

- Linea Continua Colorata (True): Rappresenta la posizione reale del veicolo (Ground Truth). È dove il robot si trova effettivamente nel mondo fisico.

- Linea Punteggiata Colorata (Est): Rappresenta la posizione stimata dall'EKF. È dove il robot pensa di essere. Più la linea punteggiata è sovrapposta a quella continua, migliore è il sistema di localizzazione. Nelle zone rosse, potresti notare che la linea punteggiata si discosta leggermente da quella reale per poi correggersi: è l'effetto della perdita del GPS e della correzione UWB/Collaborativa.

- Triangoli Blu: Le ancore UWB fisse.

- Zone Rosse: Le aree in cui viene simulato l'oscuramento del segnale GPS.


## Come funziona l'algoritmo?

L'algoritmo gira in un loop continuo. Ad ogni istante di tempo (ogni tick del simulatore), esegue questi tre passaggi per ogni veicolo:

1) **Fase A: Misurazione e Predizione (EKF)**: Il robot legge i propri encoder (ruote) e l'IMU (bussola/giroscopio). Con questi dati, fa una predizione cieca: "Ero al punto A, ho girato le ruote a questa velocità, quindi ora dovrei essere al punto B". Questa predizione si accumula di errori (deriva) col tempo.

2) **Fase B: Sensor Fusion e Localizzazione Collaborativa (L'Aggiornamento)**: Qui entra in gioco l'intelligenza del sistema. Il robot guarda la mappa e si chiede: "Vedo i satelliti GPS?".
    - Se SÌ (fuori dalle zone rosse): Usa il GPS per correggere la sua predizione. Il GPS è rumoroso, ma non deriva nel tempo.
    - Se NO (dentro le zone rosse): Il robot passa in modalità emergenza. Cerca i triangoli blu (Ancore UWB) entro il suo raggio visivo e misura la distanza da loro. Se non ne trova abbastanza, guarda i suoi "colleghi" (gli altri veicoli) e usa il radar/radio per misurare la distanza da loro. Sfrutta le posizioni stimate dei colleghi come se fossero ancore mobili. Questa si chiama localizzazione collaborativa.

3) **Fase C: Controllo e Consenso (Il Movimento)**: Ora che ogni robot sa dove si trova, deve decidere come muoversi:
    - Il Master calcola la direzione verso il prossimo punto del percorso nero tratteggiato.
    - Gli Slave calcolano la loro distanza dal Master. Se sono troppo lontani dalla loro posizione desiderata a triangolo, accelerano. Se si avvicinano troppo a un compagno, scatta una forza "repulsiva" matematica che li allontana per evitare lo scontro.