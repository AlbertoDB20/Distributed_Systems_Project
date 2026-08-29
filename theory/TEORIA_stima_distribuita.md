# Stima Distribuita di un Parametro Costante: D-WLS

Riferimento: *Intelligent Distributed Systems*, **Capitolo 18, §18.1**. Note collegate: [TEORIA_consenso_su_grafi.md](TEORIA_consenso_su_grafi.md), [TEORIA_osservabilita_e_filtro.md](TEORIA_osservabilita_e_filtro.md), [TEORIA_rumore_di_processo.md](TEORIA_rumore_di_processo.md).

---

## 1. Il Problema

Una grandezza incognita è misurata da $n$ nodi, ciascuno con il proprio sensore e la propria qualità di misura. La soluzione ovvia è raccogliere tutto in un nodo centrale e risolvere lì il problema ai minimi quadrati. In un sistema distribuito quel nodo non esiste, e replicarlo via *flooding* — ogni nodo ritrasmette tutti i dati finché ciascuno possiede l'intero insieme — costa un numero di messaggi che cresce molto più in fretta della rete.

Il **D-WLS** ottiene lo stesso risultato numerico del calcolo centralizzato facendo scambiare a ogni nodo soltanto due oggetti di dimensione fissa con i propri vicini diretti.

Il metodo si applica quando il parametro è **costante e non stocastico**. Se la grandezza possiede una dinamica propria serve il Filtro di Kalman Distribuito (DKF, §18.2), che è la stessa architettura con in più il passo di predizione.

---

## 2. La Soluzione Centralizzata

Raccolte le misure di tutti i nodi in un unico istante,

$$z_i = C_i\,x + \varepsilon_i, \qquad i = 1,\dots,n$$

e impilate in $z$, $C$, con $\varepsilon$ di covarianza $R$, la stima ottima ai minimi quadrati pesati e la covarianza del suo errore valgono

$$\hat x_{LS} = \left(C^T R^{-1} C\right)^{-1} C^T R^{-1} z, \qquad P = \left(C^T R^{-1} C\right)^{-1}$$

Il pesaggio con $R^{-1}$ è ciò che distingue il WLS dai minimi quadrati ordinari: le misure entrano in proporzione alla loro **precisione**, non tutte allo stesso modo.

---

## 3. Perché la Soluzione si Decompone

I rumori dei diversi nodi sono scorrelati fra loro, quindi $R$ è **diagonale a blocchi**. È questa singola proprietà a rendere possibile tutto il resto: l'inversa di una diagonale a blocchi è diagonale a blocchi, e i due prodotti si spezzano in somme di termini puramente locali:

$$\hat x_{LS} = \left(\sum_{i=1}^{n} C_i^T R_i^{-1} C_i\right)^{-1} \left(\sum_{i=1}^{n} C_i^T R_i^{-1} z_i\right), \qquad P = \left(\sum_{i=1}^{n} C_i^T R_i^{-1} C_i\right)^{-1}$$

Si definiscono allora la **matrice di informazione locale** e lo **stato di informazione locale**:

$$F_i(0) = C_i^T R_i^{-1} C_i, \qquad a_i(0) = C_i^T R_i^{-1} z_i$$

da cui

$$\hat x_{LS} = \left(\sum_i F_i(0)\right)^{-1} \sum_i a_i(0)$$

Nessun nodo deve possedere la matrice globale: è sufficiente che ciascuno sappia calcolare le due **somme**. La ragione per cui si lavora nel dominio dell'informazione anziché su stime e misure è esattamente questa: le informazioni si sommano, ed è ciò che rende lecito sostituire una somma con una media.

---

## 4. L'Algoritmo in Tre Fasi

**Fase 1 — Inizializzazione locale.** Ogni nodo misura e calcola i propri $F_i(0)$ e $a_i(0)$. Se $F_i(0)$ è invertibile può già produrre una stima locale $\hat x_i = F_i(0)^{-1}a_i(0)$, ma è la stima ottenuta con la sola informazione propria.

**Fase 2 — $q$ cicli di consenso sulla media.** I nodi si scambiano le coppie informative con i soli vicini diretti, applicando pesi doppiamente stocastici (Metropolis-Hastings, oppure pesi a massimo grado):

$$F_i(k+1) = F_i(k) + \sum_{j} q_{ij}\big(F_j(k) - F_i(k)\big) = q_{ii}F_i(k) + \sum_{j \in \mathcal{N}_i} q_{ij} F_j(k)$$

e identicamente per $a_i$. Le due scritture sono la stessa cosa: la prima mostra che l'aggiornamento è una **correzione verso i vicini**, la seconda che è una **media pesata**. Su grafo connesso entrambe convergono alla media aritmetica dei valori iniziali:

$$\lim_{k\to\infty} F_i(k) = \frac{1}{n}\sum_{l} F_l(0), \qquad \lim_{k\to\infty} a_i(k) = \frac{1}{n}\sum_{l} a_l(0)$$

**Fase 3 — Ricostruzione locale.** Ogni nodo calcola

$$\hat x_i = F_i(q)^{-1} a_i(q)$$

ottenendo la stessa soluzione che avrebbe prodotto il calcolo centralizzato, senza aver mai trasmesso una misura grezza né conosciuto la topologia della rete.

La doppia stocasticità di $Q$ non è un dettaglio realizzativo: una $Q$ soltanto stocastica farebbe convergere a una combinazione pesata arbitraria degli stati iniziali, e la stima ricostruita risulterebbe polarizzata.

### 4.1 Quanti cicli servono

Il residuo di consenso decade geometricamente con ragione $\rho_2$, quindi per scendere sotto una tolleranza $\epsilon$ occorrono

$$q \;\ge\; \max\left(\frac{\log\epsilon}{\log\rho_2},\; \mathrm{diam}(\mathcal{G})\right)$$

Il secondo termine è di natura diversa dal primo e non va trascurato: sotto il **diametro** del grafo l'informazione non ha materialmente attraversato la rete, e due nodi a distanza 3 non sanno nulla l'uno dell'altro finché non sono trascorsi 3 scambi. Con un $q$ inferiore il risultato non è impreciso, è privo di senso — e nulla lo segnala, perché le matrici restano invertibili e i numeri prodotti sembrano ragionevoli.

Il valore va infine confrontato con quanto il canale radio sostiene in un passo di campionamento. Se il budget non basta il fatto va **dichiarato**: è la differenza fra un algoritmo che ammette di non essere arrivato a convergenza e uno che restituisce in silenzio un numero sbagliato.

Il dimensionamento è un calcolo del **progettista**, non dell'agente: $\rho_2$ e il diametro sono proprietà globali del grafo, esattamente come $\lambda_2(L)$ (vedi [TEORIA_consenso_su_grafi.md](TEORIA_consenso_su_grafi.md) §4). A bordo il numero di cicli è un parametro di configurazione, non una decisione presa in tempo reale.

Su grafo completo $\rho_2 = 0$ e il diametro vale 1: la formula restituisce $q = 1$ per qualunque tolleranza. Non è una scorciatoia scritta a mano, è il valore che il dimensionamento produce da solo.

### 4.2 Su quale scala dei tempi

I $q$ cicli **non** occupano passi di campionamento. Il consenso vive sulla scala dei tempi della radio: uno scambio TW-TOF su DW1000 dura circa 1 ms, contro i 100 ms del passo di controllo. Fra due istanti di campionamento successivi il canale sostiene quindi decine di cicli, e l'intero round di stima si esaurisce prima che il controllo debba agire di nuovo.

Ne segue che il D-WLS può essere eseguito **a ogni passo** anziché periodicamente, senza alcun costo in termini di dinamica. Quel che cambia non è l'accuratezza — che dipende dall'informazione accumulata, non dalla frequenza con cui la si interroga — ma la **latenza** con cui il risultato è disponibile. L'ipotesi andrà rivista in Fase 4, dove le latenze di rete diventano esplicite.

---

## 5. Il Fattore $1/n$ si Cancella (ma non sempre)

Nella ricostruzione il fattore di normalizzazione compare a numeratore e a denominatore:

$$\hat x_i = \left(\tfrac{1}{n}\textstyle\sum_l F_l\right)^{-1}\left(\tfrac{1}{n}\textstyle\sum_l a_l\right) = \left(\textstyle\sum_l F_l\right)^{-1}\textstyle\sum_l a_l = \hat x_{LS}$$

**Il D-WLS non richiede quindi che i nodi conoscano la cardinalità della rete.** È una differenza sostanziale rispetto al DKF (§18.2), dove la somma globale va ricostruita esplicitamente moltiplicando per $n$ e ogni nodo deve conoscerlo a priori.

La cancellazione riguarda però la **sola stima**. Per dichiarare la propria incertezza il nodo deve ricostruire

$$P = \left(n\,F_i(q)\right)^{-1}$$

e qui il fattore non si semplifica, perché non c'è alcun rapporto fra quantità entrambe scalate. Un nodo che voglia accompagnare la stima con la sua covarianza deve dunque conoscere $n$ anche nel D-WLS.

---

## 6. Perché Non si Genera Doppio Conteggio

Scambiare ripetutamente informazione su un grafo con cicli normalmente produce *data rumination*: la stessa informazione rientra da percorsi diversi e viene contata più volte, rendendo i filtri artificialmente sicuri di sé (README §2.3). Nel consenso questo non accade, e la ragione è algebrica. Poiché $Q$ è doppiamente stocastica le sue **colonne** sommano a uno, quindi

$$\sum_i F_i(k+1) = \sum_i \sum_j q_{ij}F_j(k) = \sum_j \Big(\sum_i q_{ij}\Big) F_j(k) = \sum_j F_j(k)$$

La somma totale dell'informazione è un **invariante**: il consenso la ridistribuisce fra i nodi senza mai crearne. La differenza rispetto alla fusione delle pose è che là la stima è ricorsiva nel tempo e rientra nel proprio filtro, mentre qui il consenso opera su un insieme congelato di contributi iniziali.

---

## 7. Topologia Tempo-Variante

Il risultato non richiede che il grafo resti fisso. È sufficiente che i grafi che si presentano infinite volte siano **congiuntamente connessi**, cioè che la loro unione sia connessa (Definizione 96 e Teorema 17 del testo): la rete può essere disconnessa a ogni singolo istante e sostenere ugualmente il consenso, purché nel tempo i collegamenti coprano l'intera flotta. È la proprietà che rende l'algoritmo utilizzabile in presenza di perdite di pacchetto e di raggio radio finito, cioè nelle condizioni delle Fasi 4 e 6.

---

## 8. Applicazione al Progetto: il Parametro di Terreno

La resistenza specifica al moto di un cingolato su neve dipende dallo stato del manto, che è proprietà del comprensorio e non del singolo mezzo: **un parametro costante e non stocastico**, esattamente il caso del D-WLS.

$$\frac{F_{traz}}{W} = \mu_{terr} + c_{terr}\,v^2 + \varepsilon \qquad\Longrightarrow\qquad x_{terr} = \begin{bmatrix}\mu_{terr}\\ c_{terr}\end{bmatrix}, \quad C_i = \begin{bmatrix}1 & v_i^2\end{bmatrix}$$

con $\mu_{terr}$ coefficiente di resistenza al rotolamento del manto e $c_{terr}$ il termine dipendente dalla velocità (compattazione della neve, resistenza della fresa). Lo sforzo specifico è misurato da un **torsiometro** sull'albero di trasmissione, normalizzando sul peso del mezzo: è una **lettura** del terreno, non un'azione su di esso, quindi l'impianto simulato resta invariato. Il modello di slittamento della Fase 5 userà questi parametri per generare la dinamica; qui vengono soltanto identificati.

La covarianza del sensore segue la convenzione degli altri: `R_traz_master` $= 0.010^2$ per il Master, con trasmissione strumentata, e `R_traz_slave` $= 0.025^2$ per gli Slave, con sensoristica di serie. È la stessa asimmetria adottata per il GPS, ed è ciò che rende effettiva la pesatura: con $R_i$ tutte uguali il termine $R^{-1}$ diventerebbe uno scalare comune e sparirebbe dalla soluzione, riducendo il WLS ai minimi quadrati ordinari. Il rapporto di pesi informativi è 6.25 a favore del Master.

---

## 9. Osservabilità: Perché Serve la Rete

Il modello ha **due** parametri e ogni veicolo produce **una** misura scalare per passo. La sua matrice di informazione locale

$$F_i = C_i^T R_i^{-1} C_i = \frac{1}{R_i}\begin{bmatrix} 1 & v_i^2 \\ v_i^2 & v_i^4\end{bmatrix}$$

è un prodotto esterno, e ha quindi **rango 1 su 2**. Nessun mezzo, da solo e a velocità costante, può separare l'intercetta dalla pendenza: il problema è singolare, non semplicemente impreciso. Diventa risolvibile solo unendo misure prese a velocità **diverse**.

È lo stesso meccanismo della GDOP nel ranging UWB (README §2.2): quel che conta non è quante misure si raccolgono, ma quanto sono geometricamente diverse fra loro. Qui la diversità è fornita dalla formazione — su percorso curvo le ali della V percorrono archi di raggio diverso dal Master — e dall'accumulo temporale lungo la missione.

> **La cooperazione non accelera la stima: la rende possibile.** È il caso segnalato dal testo del corso con l'inciso *"providing that $F_i(t)$ is invertible"*, che nel progetto non è una precauzione formale ma la condizione operativa normale.

---

## 10. Risultati Numerici

Due configurazioni, identiche in tutto tranne che nella **topologia della rete**. È il confronto che mostra la teoria in funzione: cambiando solo il raggio radio e il numero di mezzi, tutte le grandezze spettrali cambiano di conseguenza e l'algoritmo si adatta da sé.

| | Fase 3 | Fase 4 |
|---|---|---|
| Veicoli, raggio radio | 3, 120 m | 5, 55 m |
| Archi attivi | 3 su 3 — **completo** | 5 su 10 |
| $\lambda_2(L)$ — connettività algebrica | 3.000 | 0.697 |
| $\mathrm{mol}_{\lambda_1}(Q)$ — componenti connesse | 1 | 1 |
| $\rho_2 = \lvert\lambda_2(Q)\rvert$ — velocità | 0.000 | 0.826 |
| $\lambda_{min}(Q)$ — oscillazioni (mai $-1$) | 0.000 | $-0.076$ |
| Diametro del grafo | 1 | 3 |
| Cicli $q$ richiesti | **1** | **49** |
| Scarto dal WLS centralizzato | $1.5\cdot10^{-15}$ | $5.2\cdot10^{-7}$ |
| Disaccordo fra i nodi | $8.4\cdot10^{-16}$ | $9.2\cdot10^{-9}$ |
| Invarianza di $\sum_i F_i$ | $2.8\cdot10^{-16}$ | $1.0\cdot10^{-15}$ |

**Due spettri distinti, da non confondere.** Nel progetto convivono due matrici e due famiglie di autovalori, ed è un errore ricorrente scambiarle:

* **$L$, il Laplaciano.** Autovalori $0 = \lambda_1(L) \le \lambda_2(L) \le \dots$ Il secondo è la **connettività algebrica**: vale zero **se e solo se il grafo è sconnesso**, e cresce con quanto la rete è ben collegata, fino a $\lambda_2(L) = n$ sul grafo completo $K_n$. Governa la dinamica del controllo di formazione, $\tau = 1/(K_{cons}\lambda_2(L))$.
* **$Q$, la matrice di Metropolis.** È stocastica, quindi il suo autovalore massimo vale **sempre** $\lambda_1(Q) = 1$, con autovettore $\mathbf{1}$: è la garanzia che il consenso ammette uno stato di equilibrio e non diverge. Ordinando gli autovalori per modulo decrescente, il secondo è $\rho_2 = |\lambda_2(Q)|$, il **fattore di convergenza**: vale zero sul grafo completo (consenso esatto in un passo) e tende a uno quando la rete è mal collegata.

I due si muovono in **verso opposto**: rete ben collegata significa $\lambda_2(L)$ grande e $\rho_2$ piccolo. Dire "$\lambda_2 = 0$ significa grafo completo" scambia i due ruoli e afferma il contrario del vero — la Fase 4 lo rende evidente, con $\lambda_2(L) = 0.697$ e $\rho_2 = 0.826$ sullo **stesso** grafo.

La connettività, in entrambe le letture, sta nella **molteplicità** dell'autovalore di consenso e non nel suo valore: $\mathrm{mol}_{\lambda_1}(Q) = \mathrm{mol}_{\lambda_1}(L)$ è il numero di componenti connesse. Trattazione completa in [TEORIA_consenso_su_grafi.md §5](TEORIA_consenso_su_grafi.md).

**Che cosa mostra la tabella.**

Il **rango 1 su 2** di $F_i(0)$ vale in entrambe le configurazioni: nessun mezzo, da solo e a velocità costante, può separare intercetta e pendenza.

Sul **grafo completo** la regola di Metropolis dà $Q = \frac{1}{n}\mathbf{1}\mathbf{1}^T$ e quindi $\rho_2 = 0$: un solo ciclo rende la media esatta, e lo scarto dalla soluzione centralizzata scende alla precisione di macchina. Il costo di comunicazione è di un singolo scambio.

Sul **grafo sparso** servono 49 cicli su un budget radio di 50, cioè il 98% della banda allocata: la comunicazione smette di essere gratuita e diventa un vincolo di progetto. Lo scarto dal centralizzato risale a $5\cdot10^{-7}$ perché il consenso è ora troncato — convergente ma non esatto, che è la condizione normale fuori dal caso completo.

**La previsione basata su $\rho_2$ è conservativa.** In Fase 4 la formula chiede 49 cicli e ne bastano 34 nei fatti, perché $\rho_2$ governa il decadimento *asintotico* e trascura la costante moltiplicativa. Sovrastimare è il verso giusto in cui sbagliare.

**Guadagno della cooperazione.** Misurato sull'incertezza di $c_{terr}$, il parametro difficile: da **7× a 15×** per gli Slave in Fase 4, contro 8× in Fase 3. Per il Master il guadagno è invece unitario, perché possiede sia il sensore migliore sia la maggiore escursione di velocità. La rete non produce un miglioramento uniforme, ma **distribuisce a tutti la qualità del membro meglio strumentato** — la stessa struttura del GPS RTK montato sul solo Master. Dettaglio per veicolo in [fase_4/README4.md](../fase_4/README4.md).

Validazione algoritmica su topologie note, incluso il dimensionamento automatico di $q$ e il caso di budget radio insufficiente: `common/verifica_dwls.m`.

---

## 11. Limiti Noti

**Regressore incerto (*errors-in-variables*).** La riga $C_i = [1,\ v_i^2]$ è costruita sulla velocità **stimata**, l'unica disponibile a bordo, e un regressore rumoroso attenua la pendenza verso lo zero. La covarianza dell'errore di stima non modella questo effetto e risulta quindi **ottimista** — lo stesso tipo di limite già documentato per la misura collaborativa, che ignora $\Sigma_j$ (README §2.3).

L'entità dipende però da quanto è ben condizionato il problema. Con la formazione stretta della Fase 3 l'attenuazione portava $c_{terr}$ da 0.00430 a 0.00386 contro un valore vero di 0.00500, cioè $-2.06$ deviazioni standard. Con la formazione larga della Fase 4 lo scarto scende a $-0.44$, e il riferimento a velocità esatta cade a $+0.03$: l'effetto non è sparito, è diventato piccolo rispetto all'informazione disponibile. Crescerà di nuovo in Fase 5, quando lo slittamento renderà il comando diverso dalla velocità effettiva.

**Eccitazione limitata dal controllo.** La flotta viaggia molto più lenta dei 2.5 m/s di progetto — 0.83 m/s con tre mezzi, 0.50 m/s con cinque — per l'errore di inseguimento a regime del consenso del primo ordine: il Master è trattenuto dai vicini, e più vicini ha, più è trattenuto. Aggiungere veicoli **rallenta** la flotta.

Ciò che conta per la stima non è però la velocità media ma la **dispersione** di $v^2$, e quella migliora: le ali esterne della V stanno a 40 m dall'asse contro i 20 m delle interne, quindi in curva la loro velocità si scosta dal Master il doppio. È il motivo per cui la Fase 4 stima meglio pur andando più piano. Il passaggio al consenso del secondo ordine, che rimuove il ritardo di formazione, agirebbe su entrambi i fronti.



---

## Riferimenti

* *Intelligent Distributed Systems*, Cap. 18 §18.1 — D-WLS, coppia informativa, connettività congiunta (Teorema 17).
* *Intelligent Distributed Systems*, Cap. 18 §18.2 — DKF, per confronto sul ruolo di $n$.
* *Intelligent Distributed Systems*, Cap. 13 §13.1.1 — soluzione WLS centralizzata e sua covarianza.
* *Intelligent Distributed Systems*, Cap. 17 — pesi di Metropolis-Hastings, $\rho_2$, doppia stocasticità.
