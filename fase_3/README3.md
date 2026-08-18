# Fase 3: Navigazione in Ambiente Ostile e Localizzazione Collaborativa

## 1. Obiettivo
In questa fase la flotta deve navigare lungo il percorso nominale all'interno dell'ambiente precedentemente generato, affrontando le zone *GPS-denied*. Il sistema EKF viene espanso per supportare un'architettura di *Sensor Fusion* dinamica: attivazione del GPS all'aperto, transizione al sistema UWB (Ultra-Wideband) nelle zone d'ombra, e utilizzo della localizzazione collaborativa inter-veicolare.

## 2. Navigazione e Path-Following
Il veicolo Master (Veicolo 1) guida la formazione lungo il percorso specificato in `path_points`. Viene implementato un algoritmo di inseguimento del target virtuale (*Virtual Target Tracking*):
1. Il Master identifica un punto target sul percorso.
2. Il vettore di velocità di riferimento $V_{ref}$ viene calcolato dinamicamente per puntare verso il target.
3. Quando il Master si avvicina al target, l'indice avanza, guidando l'intera formazione (che lo segue tramite il protocollo di Consenso) lungo curve e diagonali.

## 2.1 Ordine di Esecuzione del Ciclo di Simulazione
Vale la convenzione temporale definita nel README principale (§2.4) e già adottata in Fase 2: $\hat{x}_k$ è la stima a posteriori a $t_k$, $z_k$ la misura acquisita a $t_k$, $u_k$ il comando applicato in $[t_k, t_{k+1})$ e calcolato dalla sola $\hat{x}_k$. Ogni iterazione esegue *broadcast → controllo → impianto → sensori → stima*.

Due aspetti sono specifici di questa fase:

* **L'impianto è una passata completa sulla flotta, che precede la passata dei sensori.** Le misure di range inter-veicolare a $t_{k+1}$ dipendono dalla posizione reale *dei vicini* allo stesso istante: la ground truth di tutti i veicoli deve quindi essere già stata propagata prima che uno qualsiasi generi le proprie letture.
* **La disponibilità del GPS è valutata sulla posizione reale $x_{k+1}^{true}$**, non sulla stima. L'oscuramento del segnale è una proprietà fisica dell'ambiente: un veicolo non decide di perdere il GPS in base a dove *crede* di trovarsi.

## 3. Fusione Sensoriale Dinamica (EKF Adattivo)
L'Extended Kalman Filter è stato riscritto per accogliere un vettore di misure $z$ e una matrice Jacobiana $C$ di dimensioni variabili a runtime.

Poiché la composizione di $z$ cambia ad ogni passo, insieme al vettore delle misure viene costruita una **maschera logica** che marca le componenti angolari. Il wrapping dell'innovazione in $[-\pi, \pi]$ è applicato attraverso questa maschera anziché per posizione nel vettore: l'operazione resta così corretta qualunque sia l'ordine con cui le misure vengono accodate, mentre un indice calcolato (del tipo `length(z)-3`) sarebbe valido solo finché l'ordinamento non cambia e si romperebbe silenziosamente al primo rimaneggiamento.

### 3.1 Transizione GPS -> UWB
Quando il veicolo entra in una zona d'ombra (distanza dal centro $\le r_{area}$):
- L'aggiornamento GPS viene disabilitato.
- Il veicolo interroga le ancore UWB fisse. Per ogni ancora visibile ($d \le r_{ancora}$), viene generata una misurazione di distanza:
  $$z_{uwb}^{(j)} = \sqrt{(x - X_{ancora}^{(j)})^2 + (y - Y_{ancora}^{(j)})^2} + \nu_{uwb}$$
  La riga corrispondente nella matrice Jacobiana $C$ è:
  $$C_{uwb}^{(j)} = \begin{bmatrix} \frac{x - X_{ancora}^{(j)}}{d} & \frac{y - Y_{ancora}^{(j)}}{d} & 0 & 0 & 0 \end{bmatrix}$$

### 3.2 Localizzazione Collaborativa
Per incrementare la resilienza, i veicoli condividono le proprie stime di stato $\hat{x}_i$ sulla rete. Ogni veicolo misura la distanza relativa $d_{ij}$ dai vicini entro il raggio di comunicazione $r_{collab}$.
Questa misura viene iniettata nell'EKF trattando il vicino come un'**ancora UWB mobile**, la cui posizione assunta è la stima ricevuta $\hat{p}_j$:
$$z_{collab}^{(j)} = ||p_i - \hat{p}_j|| + \nu_{rel}$$

**Sincronizzazione dell'ancora mobile.** L'ancora usata non è $\hat{p}_j$ al tempo $t_k$, ma la sua **propagazione di un passo** con il modello di moto del vicino:
$$\hat{p}_{j, k+1|k} = \begin{bmatrix} \hat{x}_{j,k} + \hat{v}_{j,k}\cos(\hat{\theta}_{j,k}) T_s \\ \hat{y}_{j,k} + \hat{v}_{j,k}\sin(\hat{\theta}_{j,k}) T_s \end{bmatrix}$$
La scelta risponde a due esigenze distinte:
1. **Causalità e assenza di loop algebrico.** Usare la stima aggiornata $\hat{x}_{j,k+1}$ renderebbe il filtro di $i$ dipendente dal filtro di $j$ al medesimo istante — e viceversa, dato che $j$ fa lo stesso con $i$. Il ritardo di un passo rompe la circolarità e corrisponde a ciò che un canale reale rende effettivamente disponibile: l'ultimo pacchetto ricevuto. In Fase 4 questo ritardo diventerà esplicito e variabile.
2. **Coerenza temporale.** La misura fisica $d_{ij}$ è acquisita a $t_{k+1}$. Confrontarla con una posizione riferita a $t_k$ introdurrebbe un bias sistematico di $|v_j| T_s$, dello stesso ordine del rumore del sensore ($\sigma_{collab} = 0.6$ m a 2.5 m/s).

Ciò crea un accoppiamento matematico fra gli agenti: se un veicolo perde tutti i riferimenti assoluti (no GPS, no UWB), la sua stima non degrada come nel dead-reckoning puro, ma resta agganciata a quella del resto della flotta.

### 3.3 Limiti Noti dell'Implementazione Attuale
Due limiti sono documentati esplicitamente perché condizionano l'interpretazione dei risultati e definiscono il lavoro successivo.

**a) Il filtro è ottimista sulla misura collaborativa.** La matrice $R$ associata a $z_{collab}$ contiene il solo rumore del sensore, $\sigma_{collab}^2$. L'incertezza della stima del vicino — la sua matrice $\Sigma_j$ — viene ignorata, come se $\hat{p}_j$ fosse un'ancora fissa nota esattamente. Il filtro sottostima quindi la propria covarianza. La correzione minima consiste nel proiettare la covarianza del vicino sulla direzione della congiungente:
$$R_{eff} = \sigma_{collab}^2 + u^T \Sigma_j^{(1:2,1:2)} u, \qquad u = \frac{\hat{p}_i - \hat{p}_j}{||\hat{p}_i - \hat{p}_j||}$$
Richiede che il vicino trasmetta, oltre alla posizione, il blocco $2 \times 2$ della propria covarianza. Questo mitiga la sovra-confidenza ma **non** risolve la correlazione: poiché $i$ e $j$ si scambiano informazione ciclicamente, le stime diventano correlate in modo ignoto (*data rumination*), ed è per questo che l'architettura prevede la Covariance Intersection.

**b) La scala della formazione determina l'esistenza stessa dello scenario collaborativo.** Le zone GPS-denied hanno raggio 65 m. Se la formazione è larga pochi metri, i tre veicoli condividono sempre la stessa condizione di copertura e lo scenario di riferimento — "un veicolo perde il GPS ma un vicino lo mantiene e lo àncora" — non può verificarsi. Misura su run completo, prima e dopo la ritaratura della formazione:

| Condizione | Formazione 6 m | Formazione 36-40 m |
|---|---|---|
| Tutti e tre coperti da GPS | 60.2% | 48.6% |
| Copertura **mista** (almeno uno coperto, almeno uno al buio) | 5.4% | **23.3%** |
| Tutti e tre al buio | 34.4% | 28.1% |

*(Confronto a parità di mappa. Con la mappa attuale — zone d'ombra a raggio variabile 45-85 m — i valori sono 67.3% / 19.3% / 13.4%.)*

Con la formazione stretta lo scenario utile occupava il 5.4% dei campioni, e unicamente come transitorio di attraversamento del bordo di una zona: la localizzazione collaborativa era implementata ma non aveva modo di dimostrare alcun beneficio. Portando le distanze reciproche a 36-40 m — valore che corrisponde peraltro all'impiego reale dei mezzi battipista, che lavorano su fronti ampi — la copertura mista sale al 23.3% del tempo.

Resta il 28.1% di campioni in cui **nessun** veicolo dispone di riferimenti assoluti diversi dalle ancore UWB fisse. In quelle condizioni il ranging inter-veicolare non può correggere la posizione assoluta della flotta: una traslazione rigida dell'intero gruppo lascia tutte le distanze relative invariate, quindi la direzione di traslazione comune appartiene al nucleo della matrice di osservabilità collettiva. È una proprietà strutturale, non un difetto di taratura, e va analizzata e riportata come risultato — non nascosta.

**c) Effetto dell'infrastruttura UWB sulla qualità della stima.** Il passaggio da 2 a 5 ancore, con raggio di visibilità portato da 100 m a 150 m su base hardware reale (§2.2 del README principale), produce l'effetto più marcato dell'intera fase. Errore medio di posizione, run completo:

| Veicolo | MAE con GPS | MAE in zona GPS-denied (2 ancore, r=100 m) | MAE in zona GPS-denied (5 ancore, r=150 m) |
|---|---|---|---|
| V1 (Master, GPS RTK) | 0.17 m | 3.36 m | **0.22 m** |
| V2 (Slave, GPS standard) | 0.58 m | 2.19 m | **0.23 m** |
| V3 (Slave, GPS standard) | 0.60 m | 2.53 m | **0.22 m** |

Due letture meritano di essere riportate. La prima: con un'infrastruttura UWB adeguata l'attraversamento delle zone cieche cessa di essere un degrado e diventa quasi trasparente, con un miglioramento di oltre un ordine di grandezza. La seconda, meno attesa: **per gli Slave la stima in zona GPS-denied è più accurata che a cielo aperto** (0.23 m contro 0.58 m). Non è un paradosso — il ranging UWB a $\sigma = 0.5$ m da ancore geometricamente ben distribuite porta più informazione di un GPS standard a $\sigma = 2.0$ m. Il risultato suggerisce che, in un'area attrezzata con ancore, converrebbe fondere UWB e GPS *simultaneamente* anziché commutare fra i due: l'architettura a $C$ di dimensione variabile lo consente già senza modifiche strutturali.

**d) Errore di inseguimento a regime della formazione.** Le distanze $d_{12}$ e $d_{13}$ si assestano a circa 39.9 m contro un target di 36.1 m, mentre $d_{23}$ resta esatta (40.0 m). Non è rumore: è l'errore a regime di un controllo puramente proporzionale che insegue un riferimento in movimento. Solo il Master riceve il termine di velocità $V_{rif}$ del path following; gli Slave si muovono unicamente per effetto del consenso, e devono quindi mantenere un errore di formazione non nullo per generare la velocità necessaria a stare al passo. L'errore è di modo comune lungo la direzione del moto, ed è per questo che la distanza fra i due Slave — simmetrici rispetto al Master — resta corretta. La correzione naturale è un termine di feedforward: propagare $V_{rif}$ a tutta la flotta (come già avviene in Fase 2, dove infatti il fenomeno non si presenta) oppure introdurre un'azione integrale nella legge di consenso.


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