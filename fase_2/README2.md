# Fase 2: La Flotta e il Controllo di Formazione Distribuito

## 1. Obiettivo
Questa fase estende l'architettura a un sistema multi-agente composto da $N=3$ veicoli. L'obiettivo primario è implementare un controllo di formazione decentralizzato basato su protocolli di consenso, garantendo l'evitamento delle collisioni. A scopo accademico, il veicolo 1 (Master) è equipaggiato con un ricevitore GPS ad alta precisione, mentre i veicoli 2 e 3 (Slave) montano sensori standard. La rete di comunicazione è assunta ideale (full-mesh, zero latenza).

## 2. Architettura Decentralizzata e Closed-Loop
A differenza della Fase 1, in cui il veicolo seguiva ciecamente una traiettoria imposta in anello aperto, il sistema opera ora in *closed-loop*: la stima alimenta il controllo, e il controllo modifica lo stato che verrà stimato al passo successivo. Questo rende l'ordine di esecuzione dei blocchi una scelta di modellazione, non un dettaglio implementativo.

Detto $t_k = (k-1)T_s$, si adotta la convenzione temporale definita nel README principale (§2.4): $\hat{x}_k$ è la stima **a posteriori** a $t_k$, $z_k$ è la misura **acquisita** a $t_k$, e $u_k$ è il comando **applicato** nell'intervallo $[t_k, t_{k+1})$.

Ogni iterazione esegue:

1. **Broadcast** — ogni veicolo pubblica la propria stima $\hat{x}_k$ (canale ideale: full-mesh, latenza nulla).
2. **Controllo** — il controllore locale calcola $u_k = (v_k, \omega_k)$ dall'errore di formazione, valutato sulle stime a $t_k$ proprie e dei vicini.
3. **Impianto** — la ground truth avanza: $x_{k+1}^{true} = f(x_k^{true}, u_k)$.
4. **Sensori** — le letture rumorose $z_{k+1}$ sono generate da $x_{k+1}^{true}$.
5. **Stima** — l'EKF locale predice da $\hat{x}_k$ e corregge con $z_{k+1}$, ottenendo $\hat{x}_{k+1}$.

Due proprietà vanno rispettate e sono il motivo di quest'ordine preciso:

* **Il controllo è causale.** $u_k$ dipende esclusivamente da $\hat{x}_k$. Retroazionare $\hat{x}_{k+1}$ — cioè la stima prodotta *dopo* che il comando ha già agito — equivarrebbe a concedere al controllore la conoscenza del futuro.
* **Misura e predizione sono sincrone.** L'innovazione $z_{k+1} - h(\hat{x}_{k+1|k})$ ha senso statistico solo se entrambi i termini si riferiscono allo stesso istante. Un disallineamento di un solo passo produce un errore *deterministico* di $|v|T_s$ sulla posizione e $|\omega|T_s$ sull'heading: con $\omega$ in saturazione a 0.6 rad/s e $T_s = 0.1$ s si tratta di 0.06 rad, un ordine di grandezza sopra il rumore del magnetometro ($\sigma_\theta = 0.05$ rad). Trattandosi di un bias e non di rumore, non comparirebbe nella matrice $\Sigma$: il filtro resterebbe convinto di essere accurato mentre è sistematicamente in ritardo, rendendo insensata l'analisi di consistenza prevista in Fase 6.

### 2.1 Modello di attuazione
La ground truth avanza con la velocità reale già presente nello stato a $t_k$; il comando appena calcolato diventa effettivo a $t_{k+1}$. Si modella così un attuatore ZOH con ritardo di un passo, ipotesi più realistica del tracking istantaneo e coerente con il modello random-walk che l'EKF assume su $v$ e $\omega$. Questo punto del codice è l'innesto previsto per il modello di slittamento: attualmente $v_{k+1}^{true} = v_{cmd}$ (tracking ideale dei motori), in seguito $v_{k+1}^{true} = f_{slip}(v_{cmd}, x_k^{true}, \text{terreno})$.

## 3. Controllo di Formazione e Feedback Linearization

### 3.1 Linearizzazione Esatta via Feedback
Il modello dell'uniciclo è anolonomo, il che rende difficile l'applicazione diretta del consenso su coordinate cartesiane. Per aggirare il problema, si controlla un punto $p_i$ situato a una distanza $b$ lungo l'asse longitudinale del veicolo:
$$p_i = \begin{bmatrix} p_{xi} \\ p_{yi} \end{bmatrix} = \begin{bmatrix} X_i + b \cos(\theta_i) \\ Y_i + b \sin(\theta_i) \end{bmatrix}$$
Derivando rispetto al tempo, si ottiene una relazione lineare invertibile tra la velocità del punto $\dot{p}_i$ e gli ingressi fisici del veicolo $(v_i, \omega_i)$:
$$\begin{bmatrix} \dot{p}_{xi} \\ \dot{p}_{yi} \end{bmatrix} = \begin{bmatrix} \cos(\theta_i) & -b \sin(\theta_i) \\ \sin(\theta_i) & b \cos(\theta_i) \end{bmatrix} \begin{bmatrix} v_i \\ \omega_i \end{bmatrix} \implies u_i = T_{fl}^{-1}(\theta_i, b)\, \dot{p}_{cmd, i}$$

### 3.2 Legge di Consenso Integrata
Definita una traiettoria nominale descritta da una velocità di riferimento $V_{ref}$, e una matrice di posizioni relative desiderate $\Delta_{ij}$, la legge di controllo per il punto $p_i$ è:
$$\dot{p}_{cmd, i} = V_{ref} - K_c \sum_{j} a_{ij} \left( (p_i - p_j) - \Delta_{ij} \right) + F_{rep, i}$$
Dove:
- $K_c$ è il guadagno del consenso.
- $a_{ij}$ sono gli elementi della **matrice di adiacenza** del grafo di comunicazione, costruita da `common/costruisci_grafo.m`. In questa fase il canale è ideale ($R_c = \infty$), quindi il grafo è il completo $K_3$ e $a_{ij} = 1$ per ogni $i \ne j$.
- $\Delta_{ij}$ è l'offset desiderato del veicolo $i$ rispetto al veicolo $j$.

**Il termine di consenso è il protocollo lineare del Cap. 17.** Introducendo la variabile traslata $\tilde p_i = p_i - p_i^{des}$, l'errore di formazione diventa $(p_i - p_j) - \Delta_{ij} = \tilde p_i - \tilde p_j$, e la sommatoria si riscrive in forma matriciale come

$$u = -K_c\,(L \otimes I_2)\,\tilde p$$

con $L = D - A$ il Laplaciano. La dinamica dell'errore è quindi $\dot e = -K_c L e$, che decade come $e^{-K_c\lambda_2(L) t}$: la costante di tempo vale $\tau = 1/(K_c\lambda_2(L))$. Per il grafo completo $K_3$ si ha $\lambda_2(L) = 3$, da cui $\tau = 1/(3 \cdot 0.15) = 2.22$ s — valore verificato in simulazione.

Il codice registra a ogni passo la connettività algebrica $\lambda_2(L)$, l'intero spettro $\lambda_i(Q)$ dei pesi di Metropolis-Hastings — ordinati per modulo, da cui $\rho_2 = \lvert\lambda_2(Q)\rvert$ e la molteplicità $\mathrm{mol}_{\lambda_1}(Q)$ — e il numero di archi attivi, riportati nella figura `5_grafo_comunicazione.png`. La convenzione sui simboli è in [TEORIA_consenso_su_grafi.md §5](../theory/TEORIA_consenso_su_grafi.md).

> Trattazione completa del consenso lineare su grafi: [TEORIA_consenso_su_grafi.md](../theory/TEORIA_consenso_su_grafi.md).

Nel codice la matrice di inversione $T_{fl}^{-1}$ è `T_fl_inv`: il nome evita la collisione con $R$, che in tutto il progetto indica esclusivamente la covarianza del rumore di misura (vedi la tabella di notazione nel README principale, §2.5).

### 3.3 Artificial Potential Fields (Evitamento Collisioni)
Per l'evitamento delle collisioni inter-veicolari si aggiunge una componente repulsiva $F_{rep,i}$, ottenuta come antigradiente della funzione **FIRAS** di Khatib (1986), attiva solo se la distanza stimata $d_{ij} = \|\hat{p}_i - \hat{p}_j\|$ scende sotto la soglia $d_{safe}$:

$$U_{rep} = \tfrac{1}{2}k_{rep}\left(\frac{1}{d_{ij}}-\frac{1}{d_{safe}}\right)^2 \;\Longrightarrow\; F_{rep,i} = \sum_{j\ne i} k_{rep}\left(\frac{1}{d_{ij}}-\frac{1}{d_{safe}}\right)\frac{1}{d_{ij}^2}\,\hat{u}_{j\to i}$$

con $\hat{u}_{j\to i} = \nabla_{p_i} d_{ij} = (\hat p_i - \hat p_j)/d_{ij}$ versore della congiungente.

Il consenso di §3.2 e questa repulsione **non sono due controllori distinti**: sono i termini attrattivo e repulsivo di un unico campo potenziale, di cui la legge di controllo è l'antigradiente. Il termine $\left(\frac{1}{d}-\frac{1}{d_{safe}}\right)$ è costruito per annullarsi esattamente in $d = d_{safe}$, garantendo la continuità della velocità comandata attraverso la soglia.

Poiché il gradiente scala come $1/d^3$, il guadagno $k_{rep}$ **non è trasferibile fra geometrie di scala diversa**: nel codice non è un numero fissato, ma viene ricavato invertendo il requisito di progetto "a $d = d_{safe}/2$ la repulsione vale $v_{rep,ref}$".

> **Trattazione completa** — origine storica del metodo, derivazione passo per passo del gradiente, verifica dei segni, natura cinematica e non dinamica delle "forze", analisi di convergenza alla Lyapunov, limiti noti (minimi locali, GNRON, saturazione, effetto dell'errore di stima) e confronto con le Control Barrier Functions: vedi **[TEORIA_campi_potenziali.md](../theory/TEORIA_campi_potenziali.md)**.

## 4. Figure Prodotte

L'esecuzione di `main2.m` genera la cartella `fase_2/risultati/`:

| File | Contenuto |
|---|---|
| `1_animazione_flotta.png` | fotogramma finale delle traiettorie reali e stimate |
| `animazione_flotta.mp4` | animazione completa della convergenza in formazione |
| `2_errore_posizione_2d.png` | errore di posizione scalare $\lVert e_{pos}\rVert$, un pannello per veicolo |
| `3_diagnostica_ekf.png` | errori separati su $X$, $Y$, $\theta$ |
| `4_forze_virtuali.png` | magnitudo dei termini di consenso e di repulsione |
| `5_grafo_comunicazione.png` | $\lambda_2(L)$, spettro $\lambda_i(Q)$ e numero di archi attivi nel tempo |

Il commento quantitativo alle figure è in [risultati/GRAPH_DISCUSSION.md](risultati/GRAPH_DISCUSSION.md).

Definendo `MODO_BATCH = true` nel workspace prima di lanciare lo script, animazione e figure vengono disattivate: è la modalità usata dalle campagne Monte Carlo di `common/verifica_consistenza.m`.
