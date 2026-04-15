# Fase 2: La Flotta e il Controllo di Formazione Distribuito

## 1. Obiettivo
Questa fase estende l'architettura a un sistema multi-agente composto da $N=3$ veicoli. L'obiettivo primario è implementare un controllo di formazione decentralizzato basato su protocolli di consenso, garantendo l'evitamento delle collisioni. A scopo accademico, il veicolo 1 (Master) è equipaggiato con un ricevitore GPS ad alta precisione, mentre i veicoli 2 e 3 (Slave) montano sensori standard. La rete di comunicazione è assunta ideale (full-mesh, zero latenza).

## 2. Architettura Decentralizzata e Closed-Loop
A differenza della Fase 1 in cui il veicolo seguiva ciecamente una traiettoria imposta, il sistema ora opera in *closed-loop*:
1. Ogni veicolo aggiorna il proprio stato reale (Ground Truth).
2. I sensori generano letture rumorose.
3. Il filtro EKF locale stima lo stato $\hat{x}_i$.
4. I veicoli si scambiano i propri stati stimati (Canale Ideale).
5. Il controllore locale calcola gli ingressi $(v, \omega)$ basandosi sull'errore di formazione e li applica al veicolo.

## 3. Controllo di Formazione e Feedback Linearization

### 3.1 Linearizzazione Esatta via Feedback
Il modello dell'uniciclo è anolonomo, il che rende difficile l'applicazione diretta del consenso su coordinate cartesiane. Per aggirare il problema, si controlla un punto $p_i$ situato a una distanza $b$ lungo l'asse longitudinale del veicolo:
$$p_i = \begin{bmatrix} p_{xi} \\ p_{yi} \end{bmatrix} = \begin{bmatrix} X_i + b \cos(\theta_i) \\ Y_i + b \sin(\theta_i) \end{bmatrix}$$
Derivando rispetto al tempo, si ottiene una relazione lineare invertibile tra la velocità del punto $\dot{p}_i$ e gli ingressi fisici del veicolo $(v_i, \omega_i)$:
$$\begin{bmatrix} \dot{p}_{xi} \\ \dot{p}_{yi} \end{bmatrix} = \begin{bmatrix} \cos(\theta_i) & -b \sin(\theta_i) \\ \sin(\theta_i) & b \cos(\theta_i) \end{bmatrix} \begin{bmatrix} v_i \\ \omega_i \end{bmatrix} \implies u_i = R^{-1}(\theta_i, b) \dot{p}_{cmd, i}$$

### 3.2 Legge di Consenso Integrata
Definita una traiettoria nominale descritta da una velocità di riferimento $V_{ref}$, e una matrice di posizioni relative desiderate $\Delta_{ij}$, la legge di controllo per il punto $p_i$ è:
$$\dot{p}_{cmd, i} = V_{ref} - K_c \sum_{j \in \mathcal{N}_i} A_{ij} \left( (p_i - p_j) - \Delta_{ij} \right) + F_{rep, i}$$
Dove:
- $K_c$ è il guadagno del consenso.
- $A_{ij}$ sono i pesi della matrice di adiacenza (tutti 1 per rete full-mesh, tranne la diagonale).
- $\Delta_{ij}$ è l'offset desiderato del veicolo $i$ rispetto al veicolo $j$.

### 3.3 Artificial Potential Fields (Evitamento Collisioni)
Per l'evitamento delle collisioni inter-veicolari, si aggiunge una componente repulsiva $F_{rep, i}$ che si attiva solo se la distanza stimata $d_{ij} = ||\hat{p}_i - \hat{p}_j||$ scende sotto una soglia di sicurezza $R_{safe}$:
$$F_{rep, i} = \sum_{j \neq i} k_{rep} \left( \frac{1}{d_{ij}} - \frac{1}{R_{safe}} \right) \frac{1}{d_{ij}^2} \nabla d_{ij}$$
Questa "forza virtuale" devia la traiettoria garantendo che i veicoli non si scontrino durante le fasi transitorie di recupero dell'errore.