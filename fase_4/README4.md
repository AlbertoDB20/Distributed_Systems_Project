# Fase 3: Navigazione in Ambiente Ostile e Localizzazione Collaborativa

## 1. Obiettivo
In questa fase la flotta deve navigare lungo il percorso nominale all'interno dell'ambiente precedentemente generato, affrontando le zone *GPS-denied*. Il sistema EKF viene espanso per supportare un'architettura di *Sensor Fusion* dinamica: attivazione del GPS all'aperto, transizione al sistema UWB (Ultra-Wideband) nelle zone d'ombra, e utilizzo della localizzazione collaborativa inter-veicolare.

Su questa base si innesta un secondo problema di stima, di natura diversa dal primo: l'identificazione collaborativa di un **parametro di terreno**, costante e comune a tutta la flotta, risolta con i Minimi Quadrati Pesati Distribuiti (§4.6). È il primo algoritmo del progetto in cui la matrice di consenso di Metropolis entra effettivamente in funzione anziché servire da sola diagnostica.

## 2. Navigazione e Path-Following
Il veicolo Master (Veicolo 1) guida la formazione lungo il percorso specificato in `path_points`. Viene implementato un algoritmo di inseguimento del target virtuale (*Virtual Target Tracking*):
1. Il Master identifica un punto target sul percorso.
2. Il vettore di velocità di riferimento $V_{ref}$ viene calcolato dinamicamente per puntare verso il target.
3. Quando il Master si avvicina al target, l'indice avanza, guidando l'intera formazione (che lo segue tramite il protocollo di Consenso) lungo curve e diagonali.

## 2.1 Ordine di Esecuzione del Ciclo di Simulazione

> Lo schema temporale dettagliato di un passo — le due scale dei tempi, chi legge cosa e a quale istante, lo sfalsamento fra iterazione del codice e istante fisico — è in [TEORIA_ciclo_temporale.md](../theory/TEORIA_ciclo_temporale.md).
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
Richiede che il vicino trasmetta, oltre alla posizione, il blocco $2 \times 2$ della propria covarianza — cioè che il **contenuto del pacchetto scambiato** passi da 2 a 5 numeri. Questo mitiga la sovra-confidenza ma **non** risolve la correlazione: poiché $i$ e $j$ si scambiano informazione ciclicamente, le stime diventano correlate in modo ignoto (*data rumination*), ed è per questo che l'architettura prevede la Covariance Intersection.

> **Programmato per la Fase 5** (README principale, §4, punto 4). Entrambi gli interventi — estensione del pacchetto con $\Sigma_j$ e aggiornamento in forma CI — sono raccolti lì, perché lo slittamento rende l'incertezza dei vicini non solo maggiore ma **variabile nel tempo**, ed è il momento in cui trattare la stima ricevuta come esatta diventa insostenibile.

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

Errore medio di posizione su run completo, nelle tre configurazioni successive:

| Veicolo | 2 ancore r=100 m, $Q$ diagonale | 5 ancore r=150 m, $Q$ diagonale | 5 ancore r=150 m, $Q$ CWNA |
|---|---|---|---|
| | con GPS / al buio | con GPS / al buio | con GPS / al buio |
| V1 (Master, GPS RTK) | 0.17 / 3.36 m | 0.17 / 0.22 m | **0.061 / 0.072 m** |
| V2 (Slave, GPS standard) | 0.58 / 2.19 m | 0.58 / 0.23 m | **0.194 / 0.077 m** |
| V3 (Slave, GPS standard) | 0.60 / 2.53 m | 0.60 / 0.22 m | **0.195 / 0.071 m** |

Tre letture meritano di essere riportate.

*Infrastruttura.* Con un numero adeguato di ancore l'attraversamento delle zone cieche cessa di essere un degrado e diventa quasi trasparente: da 3.36 m a 0.22 m per il Master, oltre un ordine di grandezza a parità di filtro.

*Rumore di processo.* La riformulazione di $Q$ in forma CWNA (README principale, §2.5) porta un ulteriore fattore 3 su tutti i veicoli, senza toccare né sensori né infrastruttura. È informazione che era già disponibile nel modello di moto e che la $Q$ diagonale costringeva il filtro a scartare.

*Un risultato controintuitivo.* **Per gli Slave la stima in zona GPS-denied è più accurata che a cielo aperto** (0.077 m contro 0.194 m). Non è un paradosso: il ranging UWB a $\sigma = 0.5$ m da ancore geometricamente ben distribuite porta più informazione di un GPS standard a $\sigma = 2.0$ m. Il risultato suggerisce che, in un'area attrezzata con ancore, converrebbe fondere UWB e GPS *simultaneamente* anziché commutare fra i due — l'architettura a $C$ di dimensione variabile lo consente già senza modifiche strutturali.

**d) Errore di inseguimento a regime della formazione.** Le distanze $d_{12}$ e $d_{13}$ si assestano a circa 39.9 m contro un target di 36.1 m, mentre $d_{23}$ resta esatta (40.0 m). Non è rumore: è l'errore a regime di un controllo puramente proporzionale che insegue un riferimento in movimento. Solo il Master riceve il termine di velocità $V_{rif}$ del path following; gli Slave si muovono unicamente per effetto del consenso, e devono quindi mantenere un errore di formazione non nullo per generare la velocità necessaria a stare al passo. L'errore è di modo comune lungo la direzione del moto, ed è per questo che la distanza fra i due Slave — simmetrici rispetto al Master — resta corretta. La correzione naturale è un termine di feedforward: propagare $V_{rif}$ a tutta la flotta (come già avviene in Fase 2, dove infatti il fenomeno non si presenta) oppure introdurre un'azione integrale nella legge di consenso.


## 4. Figure Prodotte

L'esecuzione di `main3.m` genera la cartella `fase_3/risultati/` con otto figure. Le prime quattro replicano quelle di Fase 2, con nomenclatura coerente; le successive sono specifiche di questa fase.

| File | Contenuto |
|---|---|
| `1_mappa_navigazione.png` | mappa completa: percorso nominale, zone GPS-denied, ancore UWB, traiettorie reali e stimate |
| `2_errore_posizione_2d.png` | errore di posizione scalare $\lVert e_{pos}\rVert$, con le fasce GPS-denied ombreggiate |
| `3_diagnostica_ekf.png` | errori separati su $X$, $Y$, $\theta$ |
| `4_forze_virtuali.png` | magnitudo dei termini di consenso e di repulsione |
| `5_copertura_e_covarianza.png` | riferimenti assoluti disponibili e traccia di $\Sigma_{pos}$ nel tempo |
| `6_bound_3sigma.png` | errore di stima confrontato con l'inviluppo a $\pm 3\sigma$ dichiarato dal filtro |
| `7_grafo_comunicazione.png` | $\lambda_2(L)$, $\rho_2(Q)$ e distanze inter-veicolari contro il raggio radio |
| `8_stima_terreno_dwls.png` | stima distribuita del parametro di terreno: parametri, errore, guadagno informativo |

### 4.1 Lettura della mappa (figura 1)
- **Tratteggio nero**: percorso nominale, inseguito dal solo Master.
- **Linea continua colorata**: posizione reale del veicolo (ground truth).
- **Linea punteggiata colorata**: posizione stimata dall'EKF. La sovrapposizione fra le due misura la qualità della localizzazione.
- **Triangoli blu**: ancore UWB fisse, collocate per minimizzazione della GDOP.
- **Aree rosse**: zone di oscuramento del segnale GPS.

### 4.2 Copertura sensoriale e covarianza (figura 5)
È la figura che sintetizza il risultato della fase. Il pannello superiore riporta il numero di **riferimenti assoluti** disponibili a ogni istante: vale 1 quando il GPS è attivo, e sale al numero di ancore UWB in vista quando il GPS è negato. Il pannello inferiore riporta $\text{tr}(\Sigma_{pos}) = \Sigma_{11} + \Sigma_{22}$, in scala logaritmica perché la grandezza copre tre decadi fra il transitorio iniziale e il regime.

I riferimenti sono distinti in **assoluti** (GPS e ancore fisse, che vincolano la posizione nel riferimento mappa) e **relativi** (vicini, che vincolano soltanto la geometria della formazione). La distinzione non è formale: come mostrato in [TEORIA_osservabilita_e_filtro.md](../theory/TEORIA_osservabilita_e_filtro.md) §1.4, Caso D, il ranging inter-veicolare da solo lascia non osservabile la traslazione comune della flotta.

Valori medi di $\text{tr}(\Sigma_{pos})$ misurati (seed 7, transitorio escluso):

| Veicolo | con GPS | in zona GPS-denied | rapporto |
|---|---|---|---|
| V1 (Master, GPS RTK) | 0.0089 m² | 0.0130 m² | 1.5× peggiore |
| V2 (Slave, GPS standard) | 0.0968 m² | 0.0132 m² | **7.3× migliore** |
| V3 (Slave, GPS standard) | 0.0977 m² | 0.0134 m² | **7.3× migliore** |

Il Master, che dispone di GPS RTK, subisce un lieve peggioramento entrando in zona cieca. Gli Slave, che montano GPS standard, registrano invece un **miglioramento di oltre sette volte**: con cinque ancore ben distribuite il ranging UWB a $\sigma = 0.5$ m porta più informazione di un GPS a $\sigma = 2.0$ m. È la quantificazione, sul piano della covarianza, del risultato già riportato al §3.3 in termini di errore.

### 4.3 Grafo di comunicazione (figura 7)
La legge di consenso è pesata dalla matrice di **adiacenza** del grafo, e non più applicata indiscriminatamente a tutti i veicoli. Il raggio di comunicazione coincide con `r_collab` = 120 m, la portata del ranging inter-veicolare: è la stessa radio UWB a fornire sia la misura di distanza sia il canale dati, quindi non avrebbe senso che il consenso raggiungesse un vicino con cui il ranging è impossibile.

La versione precedente iterava il consenso su tutti i veicoli senza controllo di portata, assumendo implicitamente connettività totale. Con $d_{ij} = 36\text{–}40$ m ed $R_c = 120$ m il grafo resta comunque completo e i risultati numerici non cambiano, ma la struttura ora è corretta ed è il presupposto del §19.2.6 del corso, dove il raggio diventa il parametro che frammenta la topologia.

| Grandezza | Valore misurato |
|---|---|
| Connettività algebrica $\lambda_2(L)$ | 3.0000 (= $K_3$ completo) |
| Essential spectral radius $\rho_2(Q)$ | 0.0000 |
| Costante di tempo $\tau = 1/(K_{cons}\lambda_2)$ | 2.22 s |
| Distanza inter-veicolare massima | 41.7 m, ossia il **35%** del raggio disponibile |
| Grafo connesso per l'intera missione | sì |

Il terzo pannello riporta le distanze reciproche contro $R_c$: il margine del 65% spiega perché i primi due pannelli risultino costanti. La topologia diventerà tempo-variante nelle fasi successive, con latenze e perdite di pacchetto.

> Trattazione completa: [TEORIA_consenso_su_grafi.md](../theory/TEORIA_consenso_su_grafi.md).

### 4.4 Consistenza (figura 6)
Confronto fra l'errore di stima effettivo e l'inviluppo $\pm 3\sigma$ estratto da `Sigma_hist`. La frazione di campioni fuori banda risulta compresa fra 0.0% e 0.3% contro un valore atteso di 0.3% per un filtro esattamente calibrato: il filtro è consistente e leggermente conservativo. La verifica è condotta su singolo run a scopo diagnostico; la validazione statistica con campagna Monte Carlo e test NEES è prevista in Fase 6.

### 4.5 Nota sulla figura 4
In questa fase la formazione viene inizializzata già nella configurazione desiderata, quindi non esiste il transitorio di riavvicinamento presente in Fase 2. Di conseguenza la forza repulsiva risulta **identicamente nulla** per l'intera missione, e lo sforzo di consenso si mantiene pressoché costante. Il valore non nullo a regime (1.67 m/s per il Master, 0.83 m/s per gli Slave) corrisponde all'errore di inseguimento descritto al §3.3, punto d.

### 4.6 Stima distribuita del parametro di terreno (figura 8)

Alla stima della posa, dinamica e locale a ciascun mezzo, si affianca un secondo problema di natura diversa: identificare una proprietà del **terreno**, uguale per tutta la flotta e costante nel tempo. Trattandosi di un parametro costante e non stocastico il Cap. 18 prescrive il **D-WLS**; il DKF servirebbe se la grandezza avesse una dinamica propria. Il modello è la resistenza specifica al moto,

$$\frac{F_{traz}}{W} = \mu_{terr} + c_{terr}\,v^2 + \varepsilon, \qquad C_i = \begin{bmatrix}1 & v_i^2\end{bmatrix}$$

misurata da un **torsiometro** sull'albero di trasmissione, normalizzando sul peso del mezzo. È una lettura del terreno e non un'azione su di esso: l'impianto simulato resta quello delle sezioni precedenti e i risultati già riportati non cambiano. La covarianza segue la convenzione degli altri sensori: `R_traz_master` $= 0.010^2$, `R_traz_slave` $= 0.025^2$.

**Il consenso viaggia sulla scala dei tempi della radio, non del controllo.** Uno scambio TW-TOF dura circa 1 ms contro i 100 ms del passo di campionamento, quindi fra due istanti di controllo il canale sostiene decine di cicli. Un round completo di D-WLS è eseguito **a ogni passo** — 6926 round sui 692 s di missione — con metà del passo riservata a questo traffico e l'altra metà a ranging e broadcast delle pose, per un budget di $q_{max} = 50$ cicli. La stima del terreno è così disponibile a 10 Hz, alla stessa cadenza di quella di posa.

Il consenso opera su **copie** degli accumulatori locali, che crescono per tutta la missione senza mai essere toccati: l'informazione di ogni veicolo resta quella genuinamente prodotta dai suoi sensori e non viene ricontata al round successivo.

**Correttezza dell'algoritmo.** Il grafo è il completo $K_3$, dove la regola di Metropolis dà $\rho_2 = 0$ e diametro 1: il dimensionamento automatico restituisce $q = 1$, e un solo ciclo rende la media esatta. Costa il 2% del budget radio disponibile.

| Proprietà | Valore misurato |
|---|---|
| Cicli dimensionati / osservati | 1 / 1 — residuo di consenso $3.3\cdot10^{-16}$ |
| Scarto dal WLS centralizzato | $1.5\cdot10^{-15}$ (massimo su tutti i round) |
| Accordo fra i tre veicoli | $8.4\cdot10^{-16}$ — la stessa stima per tutti |
| Invarianza di $\sum_i F_i(k)$ | $2.8\cdot10^{-16}$ — nessun doppio conteggio |
| Ricostruzione di $P$ da $(n F_i)^{-1}$ | $1.0\cdot10^{-14}$ |

Nei primi due pannelli le tre curve D-WLS coincidono a meno della precisione di macchina, e sono disegnate con spessore decrescente per renderle distinguibili. Le curve punteggiate mostrano cosa otterrebbe ciascun veicolo **senza cooperare**: V2 si assesta attorno al 14% di errore relativo, contro l'1.5% della soluzione distribuita.

Nel terzo pannello la curva solo-locale di V3 scende sotto quella del D-WLS negli ultimi secondi (0.79% contro 1.53%). È un caso fortuito e non una prestazione: l'incertezza dichiarata da V3 in isolamento è 8.3 volte più larga, quindi la sua stima ha semplicemente attraversato il valore vero. Confrontare le realizzazioni singole anziché le covarianze è precisamente l'errore che la tabella seguente evita.

**Quanto vale la cooperazione.** Il confronto corretto non è il numero di condizionamento — aggiungere i due Slave lo peggiora leggermente, pur aggiungendo informazione — ma la covarianza $P = (\sum_i F_i)^{-1}$, che per l'ordinamento di Loewner può solo ridursi:

| Veicolo | $\sigma(c_{terr})$ da solo | in rete | guadagno |
|---|---|---|---|
| V1 (Master) | $5.62\cdot10^{-4}$ | $5.53\cdot10^{-4}$ | 1.0× |
| V2 (Slave) | $4.49\cdot10^{-3}$ | $5.53\cdot10^{-4}$ | **8.1×** |
| V3 (Slave) | $4.60\cdot10^{-3}$ | $5.53\cdot10^{-4}$ | **8.3×** |

Il Master da solo arriverebbe quasi dove arriva la rete: possiede il sensore migliore ($R_{traz} = 0.010^2$ contro $0.025^2$) e la maggiore escursione di velocità ($\mathrm{std}(v^2) = 0.211$ contro 0.041 e 0.039). La rete non produce un miglioramento uniforme, ma distribuisce a tutti la qualità del membro meglio strumentato — la stessa struttura del GPS RTK montato sul solo Master.

**Accuratezza raggiunta e sue due limitazioni.** La stima finale vale $\mu_{terr} = 0.0908$ e $c_{terr} = 0.00386$ contro valori veri di 0.0900 e 0.00500, cioè $+1.93\sigma$ e $-2.06\sigma$ rispetto all'incertezza dichiarata. Eseguire il round a ogni passo anziché ogni 5 s non cambia questi numeri, perché la stima dipende dall'informazione accumulata e non dalla frequenza con cui la si interroga: quel che cambia è la **latenza** con cui il risultato è disponibile. Lo scarto ha due cause distinte e misurabili separatamente:

- **Regressore incerto.** La riga $C_i = [1,\ v_i^2]$ usa la velocità *stimata*, l'unica disponibile a bordo; un regressore rumoroso attenua la pendenza verso lo zero. Rifacendo il calcolo con la velocità vera si ottiene $c_{terr} = 0.00430$, cioè $-1.24\sigma$: circa metà dello scarto viene da qui. La covarianza $P$ non modella questo effetto e risulta quindi ottimista — lo stesso tipo di limite già documentato al §3.3 per la misura collaborativa, che ignora $\Sigma_j$.
- **Eccitazione insufficiente.** La flotta viaggia a 0.83 m/s di media contro i 2.5 m/s di progetto, per l'errore di inseguimento a regime descritto al §3.3 punto d. Il termine $c_{terr}v^2$ vale quindi circa 0.004, sotto il rumore del sensore di trazione: $c_{terr}$ è debolmente osservabile. Il passaggio al consenso del secondo ordine, che rimuove il ritardo di formazione, agisce direttamente su questa limitazione.

> Trattazione completa e validazione algoritmica su topologie note (`common/verifica_dwls.m`): [TEORIA_stima_distribuita.md](../theory/TEORIA_stima_distribuita.md).
