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

## 5. Leggere gli Spettri: $L$ e $Q$ sono Due Matrici Diverse

Il progetto calcola **due** matrici sullo stesso grafo. I loro autovalori vanno letti separatamente, e per questo ogni simbolo porta sempre l'indicazione della matrice da cui proviene: $\lambda_i(L)$, $\lambda_i(Q)$, $\mathrm{mol}_{\lambda_1}(Q)$.

| | $L = D - A$ | $Q$ (Metropolis) |
|---|---|---|
| **Pesa gli archi con** | $a_{ij} \in \{0,1\}$ | $q_{ij} = 1/(\max(d_i,d_j)+1)$ |
| **Serve a** | controllo di formazione | D-WLS |
| **Dinamica** | $\dot x = -K L x$, tempo continuo | $x(k{+}1) = Qx(k)$, tempo discreto |
| **Somma delle righe** | $0$ | $1$ |
| **Spettro** | $0 = \lambda_1 \le \lambda_2 \le \dots$, tutti reali $\ge 0$ | $1 = \lambda_1 \ge \lambda_2 \ge \dots > -1$ |
| **Rete ben collegata** | $\lambda_2(L)$ **grande** (fino a $n$ su $K_n$) | $\rho_2$ **piccolo** (fino a 0 su $K_n$) |

**Le due grandezze si muovono in verso opposto.** Sulla topologia della Fase 4 si legge $\lambda_2(L) = 0.697$ e $\rho_2 = 0.826$ sullo *stesso* grafo. Dire "$\lambda_2 = 0$ significa grafo completo" scambia i due ruoli: $\lambda_2(L) = 0$ significa grafo **sconnesso**.

### 5.1 L'equazione caratteristica e l'ordinamento

Gli autovalori si ottengono da

$$\det(Q - \lambda I) = 0$$

**Gli autovalori di $Q$ si ordinano per modulo decrescente**, $|\lambda_1| \ge |\lambda_2| \ge \dots \ge |\lambda_n|$. 

$$\rho_2 = |\lambda_2(Q)|$$

> **Il prezzo dell'ordinamento per modulo.** Con questa convenzione $\lambda_n(Q)$ è l'autovalore più vicino a **zero**, non il più negativo, e il test sulle oscillazioni non può quindi essere scritto su di lui. Va scritto su $\lambda_{min}(Q) = \min_i \lambda_i(Q)$, che non dipende dalla posizione. Su $K_{3,3}$ lo spettro per modulo è $[1,\ -0.5,\ 0.25,\ 0.25,\ 0.25,\ 0.25]$: guardando $\lambda_n = +0.25$ si mancherebbe completamente il $-0.5$. Sulle topologie del progetto i due ordinamenti coincidono, ma la definizione deve essere quella giusta.

### 5.2 Valore contro molteplicità

È la distinzione che rende leggibile tutto il resto. $\lambda_1(Q) = 1$ e $\lambda_1(L) = 0$ valgono **sempre**, per costruzione: sono garanzie strutturali, non condizioni da verificare. L'informazione sulla connettività non sta nel loro *valore* ma nella loro **molteplicità**:

$$\mathrm{mol}_{\lambda_1}(Q) \;=\; \mathrm{mol}_{\lambda_1}(L) \;=\; \text{numero di componenti connesse}$$

Vale 1 su rete connessa; vale $k$ se la flotta si è spezzata in $k$ gruppi, ciascuno dei quali converge alla propria media anziché a quella globale. È il test di connettività, e si può leggere indifferentemente su $Q$ o su $L$ — `common/verifica_grafo.m` verifica che i due coincidano su tutte le topologie di prova.

### 5.3 Le tre letture dello spettro di $Q$

$Q$ è simmetrica per costruzione, quindi ha autovalori reali. Ordinati in modo decrescente:

**$\lambda_1(Q) = 1$ — equilibrio.** Garantito sempre, perché $Q$ è stocastica e $Q\mathbf{1} = \mathbf{1}$: il consenso ammette uno stato stazionario e non diverge.

**$\rho_2 = |\lambda_2(Q)|$ — velocità.** Il fattore di convergenza asintotico: l'errore di consenso decade come $\rho_2^q$. Con $\rho_2 \to 0$ la rete scambia informazione rapidamente; con $\rho_2 \to 1$ c'è un collo di bottiglia. Vale $\rho_2 < 1$ se e solo se il grafo è connesso. Verificato su $K_{3,3}$: $\rho_2 = 0.5$ e il tasso di decadimento misurato vale 0.5.

**$\lambda_{min}(Q)$ — oscillazioni.** Se tendesse a $-1$ il consenso oscillerebbe fra due configurazioni anziché convergere: $-1$ è autovalore di una catena periodica, cioè di un grafo **bipartito** percorso senza mai restare fermi. Come detto al §5.1, con l'ordinamento per modulo va cercato esplicitamente e non coincide con $\lambda_n(Q)$.

**Con i pesi di Metropolis non può accadere.** La diagonale è sempre strettamente positiva:

$$q_{ii} = 1 - \sum_{j \in \mathcal{N}_i} \frac{1}{\max(d_i,d_j)+1} \;\ge\; 1 - \frac{d_i}{d_i+1} = \frac{1}{d_i+1} > 0$$

perché $\max(d_i,d_j) \ge d_i$ su ogni arco. Un peso proprio non nullo rende la catena aperiodica ed esclude $-1$. Il controllo resta significativo per altre scelte di pesi: sulla catena bipartita 1–2–3 il random walk semplice $D^{-1}A$ dà $\lambda_{min}(Q) = -1$ e oscilla, mentre Metropolis dà $0$.

### 5.4 Le tre letture dello spettro di $L$

**$\lambda_1(L) = 0$ — equilibrio.** Garantito sempre, perché $L\mathbf{1} = 0$: qualunque configurazione traslata rigidamente è di equilibrio.

**$\lambda_2(L)$ — rigidità della formazione.** L'errore decade come $e^{-K_{cons}\lambda_2(L)\,t}$, quindi

$$\tau = \frac{1}{K_{cons}\,\lambda_2(L)}$$

$\lambda_2(L)$ alto dà una risposta rigida e veloce, $\lambda_2(L) \to 0$ una risposta elastica e lenta. È il modo **più lento** del sistema, e quindi quello che detta il tempo di riassetto.

**Oscillazioni: escluse in tempo continuo, da verificare in tempo discreto.** Su grafo non orientato $L$ è simmetrica e semidefinita positiva, quindi tutti i modi di $\dot x = -KLx$ decadono senza oscillare. La simulazione però discretizza con Eulero,

$$x(k+1) = (I - K_{cons}T_s L)\,x(k)$$

e questa iterazione oscilla o diverge se $K_{cons}T_s\lambda_{max}(L) > 2$. La condizione va verificata, non data per scontata: vale 0.045 in Fase 3 e 0.278 in Fase 4, entrambe ampiamente sotto la soglia. Su grafo **orientato** cadrebbe anche la premessa, perché $L$ non sarebbe simmetrica e gli autovalori potrebbero essere complessi.

### 5.5 Quando le due letture coincidono

$I - Q$ è **sempre** un Laplaciano pesato (righe a somma nulla). Quando tutti i pesi fuori diagonale risultano uguali a un valore $\epsilon$ — cosa che accade **se e solo se** $\max(d_i,d_j)$ è lo stesso su ogni arco — Metropolis coincide con i pesi a massimo grado e vale

$$Q = I - \epsilon L, \qquad \epsilon = \frac{1}{d_{max}+1}, \qquad \rho_2 = 1 - \epsilon\,\lambda_2(L)$$

In quel caso $\rho_2$ e $\lambda_2(L)$ sono lo stesso numero letto due volte. **Entrambe le topologie del progetto ricadono in questo caso**, ed è una fortuna e non una regola:

| | $d_{max}$ | $\epsilon$ | $\lambda_2(L)$ | $\rho_2 = 1 - \epsilon\lambda_2(L)$ |
|---|---|---|---|---|
| Fase 3, $K_3$ | 2 | $1/3$ | 3.000 | 0.000 |
| Fase 4, 5 nodi | 3 | $1/4$ | 0.697 | 0.826 |

Basta però un grafo con un nodo di grado massimo non adiacente a tutti — un hub con una coda, per esempio — perché i pesi si differenzino e la relazione cada. Per questo il codice calcola le due matrici **separatamente** e verifica la relazione a runtime anziché assumerla.

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
* **Accuratezza teorica**: Su grafo completo $K_3$, $\lambda_2(L) = 3.0000$ e $\tau = 2.22 \text{ s}$.
* **Equivalenza software**: Lo scarto tra il comando calcolato ciclando sui vicini e quello calcolato con la matrice $L \otimes I_2$ è inferiore alla precisione di macchina ($6.7 \cdot 10^{-16} \text{ m/s}$).
* **Predittività di $\rho_2$**: su grafo a catena 1–2–3, il tasso di decadimento misurato dell'errore di consenso vale 0.6667 contro lo 0.6667 previsto (scarto $2.3 \cdot 10^{-15}$). Il caso $K_3$ non lo mostrerebbe, avendo $\rho_2 = 0$. Su $K_{3,3}$ la verifica coglie anche la distinzione fra $\lambda_{min}(Q) = -0.5$ e $\lambda_n(Q) = +0.25$, che con l'ordinamento per modulo non coincidono.
* **Diagnosticità degli indicatori**: su grafo sconnesso si ottiene $\lambda_2(L) = 0$, $\rho_2 = 1$ e $\mathrm{mol}_{\lambda_1}(Q) = \mathrm{mol}_{\lambda_1}(L) = 2$: tutti e tre segnalano la mancata convergenza, e la molteplicità dice anche in quanti gruppi la rete si è spezzata.

---

## Riferimenti

* *Intelligent Distributed Systems*, Cap. 17 — consenso lineare, matrici stocastiche, Laplaciano, progettazione dei pesi.
* *Intelligent Distributed Systems*, Cap. 18 — D-WLS e DKF, connettività congiunta. Trattazione dedicata in [TEORIA_stima_distribuita.md](TEORIA_stima_distribuita.md).
* L. Xiao, S. Boyd, *"Fast linear iterations for distributed averaging"*, Systems & Control Letters 53, 2004 — progettazione ottima dei pesi.