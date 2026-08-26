# Consenso Lineare su Grafi e Controllo di Formazione

Riferimento: *Intelligent Distributed Systems*, **Capitolo 17**. Note collegate: [TEORIA_rumore_di_processo.md](TEORIA_rumore_di_processo.md), [TEORIA_campi_potenziali.md](TEORIA_campi_potenziali.md), [TEORIA_osservabilita_e_filtro.md](TEORIA_osservabilita_e_filtro.md).



## 1. Il Contesto: Cos'è e Perché Serve

Nelle simulazioni di cooperazione tra veicoli autonomi non esiste un server centrale che impartisce comandi. Ogni veicolo deve decidere come muoversi e come aggiornare le proprie stime usando unicamente:
* **Misure locali**: sensori di bordo, ranging UWB.
* **Messaggi radio**: inviati e ricevuti dai soli veicoli direttamente connessi (vicini nel grafo di comunicazione).

Il **consenso lineare** è lo strumento matematico che permette alla flotta di raggiungere un comportamento collettivo coerente (mantenere una formazione e condividere le stime della posizione) in modo completamente **distribuito**.

---

## 2. La Dualità Fondamentale: Dati in Memoria vs. Movimento Fisico

La teoria del consenso si articola in due formulazioni matematiche ben distinte per evitare di confondere la **gestione dell'informazione** con la **dinamica fisica dei veicoli**.

| Proprietà | Forma Stocastica ($Q$) | Forma Laplaciana ($L$) |
| :--- | :--- | :--- |
| **Equazione di base** | $x(k+1) = Q \, x(k)$ | $\dot{x} = -K_{cons} \, L \, x$ |
| **Somma delle righe** | $\sum_j q_{ij} = 1$ | $\sum_j L_{ij} = 0$ |
| **Dominio di applicazione** | **Variabili di memoria**: stime dei filtri, masse informative (Cap. 18). | **Grandezze fisiche**: velocità comandate ai motori del veicolo. |
| **Azione dell'agente** | **Sostituisce** il proprio stato con la media pesata tra sé e i vicini. | Genera una **velocità** proporzionale all'errore di posizionamento relativo. |
| **Invariante e Target** | Matrice doppiamente stocastica $\rightarrow$ convergenza alla **media aritmetica esatta** ($\frac{1}{n} \sum x_j(0)$). | Il comando si **annulla** quando l'errore tra i veicoli si azzera. |

> **Metropolis-Hastings**: È la regola usata per calcolare i pesi della matrice $Q$. Permette a ogni robot di stabilire quanto fidarsi dei vicini basandosi solo sul loro numero di connessioni dirette (grado locale $d_i$), senza che nessuno debba conoscere la mappa globale della rete.
> $$q_{ij} = \frac{1}{\max(d_i, d_j) + 1} \quad (i \neq j), \qquad q_{ii} = 1 - \sum_{k \neq i} q_{ik}$$

> **Le due forme non sono in contraddizione**: discretizzando $\dot{x} = -K_{cons} L x$ con Eulero in avanti e passo $T_s$ si ottiene $x(k+1) = (I - \epsilon L)\,x(k)$ con $\epsilon = K_{cons} T_s$, cioè una $Q$ con elementi $\epsilon\,a_{ij}$ fuori diagonale e $1 - \epsilon\,d_i$ sulla diagonale: è la regola dei **pesi a massimo grado**. La forma stocastica è la discretizzazione di quella laplaciana, e Metropolis ne è il raffinamento che adatta il passo al grado locale invece di usarne uno globale.

---

## 3. Dimostrazione: Il Controllo di Formazione *è* un Consenso

Il robot non vuole semplicemente raggiungere lo stesso punto degli altri, ma vuole mantenere una **geometria rigida** definita dagli offset $\Delta_{ij} = p_i^{des} - p_j^{des}$.

La legge di controllo applicata nel codice per il singolo veicolo $i$ è:

$$u_i = -K_{cons} \sum_{j} a_{ij} \Big[ (p_i - p_j) - \Delta_{ij} \Big]$$

### Riscrittura Matriciale Passaggio per Passaggio

1. **Introduzione dello stato di errore ($\tilde{p}_i$)**:
   Si definisce lo scostamento dalla posizione teorica desiderata $\tilde{p}_i = p_i - p_i^{des}$. Sostituendo $\Delta_{ij}$:
   $$(p_i - p_j) - (p_i^{des} - p_j^{des}) = (p_i - p_i^{des}) - (p_j - p_j^{des}) = \tilde{p}_i - \tilde{p}_j$$
   *Il problema di mantenere la formazione equivale a fare consenso sullo scostamento $\tilde{p}$.*

2. **Scomposizione della sommatoria**:
   $$u_i = -K_{cons} \sum_{j} a_{ij} (\tilde{p}_i - \tilde{p}_j) = -K_{cons} \left[ \tilde{p}_i \sum_j a_{ij} - \sum_j a_{ij} \tilde{p}_j \right]$$

3. **Emergenza del Laplaciano ($L = D - A$)**:
   Il termine $\sum_j a_{ij}$ è il grado $d_i$ del nodo $i$ (matrice diagonale $D$). Il termine $\sum_j a_{ij} \tilde{p}_j$ rappresenta la riga $i$-esima del prodotto tra la matrice di adiacenza $A$ e il vettore degli errori:
   $$u_i = -K_{cons} \Big[ (D \tilde{p})_i - (A \tilde{p})_i \Big] = -K_{cons} \Big[ (D - A) \tilde{p} \Big]_i$$

4. **Forma globale vettoriale**:
   Estendendo la relazione alle due dimensioni cartesiane $X$ e $Y$ tramite il prodotto di Kronecker ($\otimes I_2$):

$$\boxed{u = -K_{cons} (L \otimes I_2) \tilde{p}}$$

---

## 4. Implementazione Locale vs. Analisi Globale

Esiste una precisa ragione architetturale per cui la matrice $L$ **non viene mai assemblata a bordo dei veicoli**:

* **Algoritmo a bordo (Forma per componenti)**: Il veicolo $i$ calcola $u_i$ sommando i contributi dei soli nodi da cui riceve un pacchetto radio. Non gli serve conoscere la topologia dell'intera rete.
* **Analisi del progettista (Forma matriciale)**: La matrice $L$ viene costruita nel simulatore per analizzare la stabilità globale, verificare se la rete è connessa e prevedere i tempi di risposta della flotta.

---

## 5. Indicatori Spettrali di Prestazione ($\lambda_2$ e $\rho_2$)

Per valutare lo stato della rete e la velocità della simulazione si monitorano due parametri spettrali:

### 1. Connettività Algebrica ($\lambda_2$ di $L$)
* **Condizione di esistenza** (grafi non orientati, come quello del progetto): Se $\lambda_2 > 0$, esiste almeno uno *spanning tree* e la flotta può mantenere la formazione. Se $\lambda_2 = 0$, il grafo si è spezzato e la flotta si frammenta. Su grafo orientato $L$ non è simmetrica e il test equivalente è la molteplicità algebrica unitaria dell'autovalore nullo.
* **Velocità di convergenza fisica**: L'errore di formazione decade esponenzialmente con una costante di tempo:

$$\tau = \frac{1}{K_{cons} \, \lambda_2}$$

### 2. Raggio Spettrale Essenziale ($\rho_2$ di $Q$)
* Determina la velocità con cui gli algoritmi di stima distribuita (D-WLS, DKF del Cap. 18) mediano la **coppia informativa** $(F_i, a_i) = (H_i^T R_i^{-1} H_i, \; H_i^T R_i^{-1} z_i)$. Si lavora nel dominio dell'informazione, e non su stime e misure, perché le informazioni si **sommano**: è questo che rende lecito sostituirle con una media.
* **Proprietà del Grafo Completo ($K_3$)**: Con $N = 3$ veicoli tutti connessi tra loro, la regola di Metropolis rende $Q = \frac{1}{3} \mathbf{1}\mathbf{1}^T$. In questo caso **$\rho_2 = 0$**, il che garantisce la convergenza alla media aritmetica esatta in **1 singolo passo di comunicazione**.

---

## 6. Vincoli Fisici, Canale Radio e Safety

L'evitamento delle collisioni è gestito da un termine repulsivo $F_{rep}$, calcolato quando due veicoli scendono sotto la distanza di sicurezza $d_{safe} = 15 \text{ m}$.

Sebbene la repulsione richieda sia la distanza sia la direzione (ottenibile solo via pacchetto radio e non da un semplice sensore di ranging), la sicurezza è garantita dalla gerarchia dei raggi d'azione:

$$d_{safe} \; (15 \text{ m}) \;\ll\; R_c \; (120 \text{ m, portata UWB})$$

Poiché il raggio di comunicazione $R_c$ è nettamente più ampio della distanza di sicurezza $d_{safe}$, il canale dati è **sempre presente e attivo** ben prima che due veicoli entrino in rotta di collisione.

---

## 7. Sintesi delle Proprietà Verificate nel Simulatore

Sui dati di simulazione (`common/verifica_grafo.m`), il modello teorico è stato validato con i seguenti risultati numerici:

* **Invarianza alle traslazioni**: $L \mathbf{1} = 0$ con errore numerico nullo ($< 10^{-16}$).
* **Accuratezza teorica**: Su grafo completo $K_3$, $\lambda_2 = 3.0000$ e $\tau = 2.22 \text{ s}$.
* **Equivalenza software**: Lo scarto tra il comando calcolato ciclando sui vicini e quello calcolato con la matrice $L \otimes I_2$ è inferiore alla precisione di macchina ($6.7 \cdot 10^{-16} \text{ m/s}$).
* **Predittività di $\rho_2$**: su grafo a catena 1–2–3, il tasso di decadimento misurato dell'errore di consenso vale 0.6667 contro lo 0.6667 previsto (scarto $2.3 \cdot 10^{-15}$). Il caso $K_3$ non lo mostrerebbe, avendo $\rho_2 = 0$.
* **Diagnosticità degli indicatori**: su grafo sconnesso si ottiene $\lambda_2 = 0$ e $\rho_2 = 1$, cioè entrambi segnalano correttamente la mancata convergenza.

---

## Riferimenti

* *Intelligent Distributed Systems*, Cap. 17 — consenso lineare, matrici stocastiche, Laplaciano, progettazione dei pesi.
* *Intelligent Distributed Systems*, Cap. 18 — D-WLS e DKF, connettività congiunta.
* L. Xiao, S. Boyd, *"Fast linear iterations for distributed averaging"*, Systems & Control Letters 53, 2004 — progettazione ottima dei pesi.