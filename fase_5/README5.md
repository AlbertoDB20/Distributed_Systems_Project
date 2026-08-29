# Fase 4: Realismo dei Sistemi Distribuiti

## 1. Obiettivo
La Fase 4 rimuove le assunzioni ideali rimaste nell'architettura, una alla volta. Lo scenario di partenza è quello della Fase 3 — navigazione in zone GPS-denied con ranging UWB, localizzazione collaborativa e stima distribuita del terreno — ma la rete smette di essere una comodità e diventa un vincolo.

| Passo | Contenuto | Stato |
|---|---|---|
| **4.0** | Flotta a $N = 5$ con **grafo non completo**: alcuni mezzi non si vedono | **fatto** |
| **4.1** | Sensori alle **frequenze reali**: il GNSS diventa più lento del passo | **fatto** |
| **4.2** | Canale con **latenza e perdita di pacchetti** | **fatto** |
| 4.3 | GPS di qualità uniforme per tutti i veicoli | da fare |

### 1.1 Che cosa cambia col passo 4.0

In Fase 3 il raggio radio (120 m) superava di tre volte l'estensione della formazione: il grafo era **completo per costruzione**, $\rho_2 = 0$, e un solo ciclo di consenso bastava. Tutta l'analisi spettrale del Cap. 17 era corretta ma inerte — due rette orizzontali in un grafico.

Qui il raggio scende a 55 m e i mezzi diventano cinque su due file d'ala. La conseguenza è che **cinque coppie su dieci non si vedono**, e da lì cambia tutto:

| Grandezza | Fase 3 | Fase 4 |
|---|---|---|
| Veicoli / raggio radio | 3 / 120 m | 5 / 55 m |
| Archi attivi | 3 su 3 (completo) | **5 su 10** |
| Connettività algebrica $\lambda_2(L)$ | 3.000 | **0.697** |
| Fattore di convergenza $\rho_2 = \lvert\lambda_2(Q)\rvert$ | 0.000 | **0.826** |
| Diametro del grafo | 1 salto | **3 salti** |
| Cicli di consenso richiesti | 1 | **49** su 50 disponibili |
| Guadagno $K_{cons}$ | 0.15 | **0.646** |

La riduzione del raggio non è un artificio per rendere interessante il grafo: è la conseguenza coerente di una fisica già argomentata nel README principale (§2.2). Le antenne dei veicoli sono più basse di quelle su palo delle ancore, e l'effetto della piattaforma metallica è presente su **entrambi** i capi del link anziché su uno solo. Quella stessa fisica che giustifica $\sigma_{collab} > \sigma_{uwb}$ implica anche una portata inferiore; il documento sosteneva finora metà dell'argomento e ignorava l'altra metà.

### 1.2 Geometria della formazione e robustezza della topologia

La "V" a cinque mezzi ha un apice, due ali interne a $\pm 20$ m dall'asse e due ali esterne a $\pm 40$ m:

```
                    V1  (0, 40)      Master
          V2 (-20, 10)      V3 (20, 10)
   V4 (-40, -20)                  V5 (40, -20)
```

Le distanze nominali si separano in **due gruppi netti**: cinque coppie a 36–40 m e cinque a 67–80 m, con un vuoto di 27 m in mezzo. Qualunque raggio fra 45 e 65 m produce quindi la stessa identica topologia, e la scelta di 55 m lascia **+35% di margine sui link attivi e −17% su quelli assenti**: l'errore di formazione non li fa sfarfallare. La topologia resta deterministica per l'intera missione, e la variabilità temporale verrà introdotta deliberatamente ai passi 4.2 e in Fase 6, non subita per un margine mal scelto.

La struttura risultante è una catena con due nodi foglia: `V4 — V2 — V1 — V3 — V5`, più l'arco `V2 — V3`. V2 e V3 sono **punti di articolazione**: se uno dei due cade, un intero ramo si stacca dalla rete. È un difetto di robustezza reale di questa geometria, e va tenuto presente leggendo i risultati.

### 1.3 Ritaratura del guadagno di consenso

La costante di tempo dell'errore di formazione vale $\tau = 1/(K_{cons}\lambda_2(L))$. Passando da $\lambda_2(L) = 3.000$ a $\lambda_2(L) = 0.697$, a parità di guadagno $\tau$ salirebbe da 2.22 a **9.56 s**: la formazione diventerebbe quattro volte più molle, con un ritardo di inseguimento tale da mettere a rischio i link stessi.

$K_{cons}$ è quindi ricalcolato per tenere $\tau$ invariato:

$$K_{cons} = \frac{1}{\tau\,\lambda_2(L)} = \frac{1}{2.22 \cdot 0.697} = 0.646$$

con margine di discretizzazione $K_{cons} T_s \lambda_{max} = 0.28$, ben sotto il limite di stabilità pari a 2. In simulazione $\tau$ risulta effettivamente 2.22 s.

**È il punto in cui $\lambda_2(L)$ smette di essere un indicatore e diventa un parametro di progetto.** Fino alla Fase 3 la connettività algebrica veniva calcolata, graficata e commentata; qui viene *usata* per dimensionare un guadagno.

### 1.4 Passo 4.1 — Sensori alle frequenze reali

Il passo di simulazione resta **10 Hz**. Ogni sensore viene però trattato per quello che è: chi lavora più in fretta del passo si legge a 10 Hz senza penalità, chi lavora più piano fornisce una misura solo ogni $N$ passi.

| Sensore | Modello | Frequenza nativa | Trattamento a 10 Hz |
|---|---|---|---|
| AHRS (heading, giroscopio) | **Xsens MTi-3** | 100 Hz interni | letto a 10 Hz, $\sigma$ di targa invariata |
| Velocità cingoli | sensore Hall sul pignone di trazione | conteggio impulsi | finestra di 100 ms = il passo |
| Ranging | **Qorvo DW1000** | ~1 ms per scambio TW-TOF | 9 scambi = 9 ms, sta nel passo |
| GNSS Master | **u-blox ZED-F9P** (RTK) | fino a 20 Hz, usato a **5 Hz** | 1 fix ogni **2** passi |
| GNSS Slave | **u-blox NEO-M8N** | **1 Hz** nominale | 1 fix ogni **10** passi |

Due scelte vanno motivate, perché la tentazione di fare il contrario è forte.

**L'AHRS non va riscalato.** Un MTi-3 filtra internamente a 100 Hz e restituisce un assetto già elaborato: la $\sigma$ di targa (2° RMS su yaw) vale alla frequenza a cui lo si interroga. Dividere $R$ per 10, come se si mediassero dieci campioni grezzi indipendenti, sarebbe sbagliato due volte — la media è già stata fatta a bordo del sensore, e l'heading nel frattempo ruota, quindi mediarlo introdurrebbe un bias.

**Gli encoder non hanno una frequenza, hanno una finestra.** Un sensore a conteggio di impulsi integra su un intervallo: 100 ms è esattamente ciò che il passo di simulazione già rappresenta.

**Resta il solo GNSS più lento del passo, e va decimato.** Campionarlo a 10 Hz significherebbe dargli fino a dieci volte le misure che produce, cioè dichiarare un ricevitore migliore di quello montato: con un NEO-M8N a 1 Hz l'equivalente sarebbe $\sigma_{eff} = 2.0/\sqrt{10} = 0.63$ m anziché 2.0 m.

Nel codice la distinzione fra "il segnale esiste" e "il ricevitore ha prodotto una soluzione" è tenuta separata:

```matlab
fix_gps = ~in_denied && mod(k, fleet(i).passi_gps) == 0;
```

`in_denied` resta una proprietà della mappa, e continua ad alimentare le statistiche di copertura e le fasce ombreggiate nei grafici. La conseguenza operativa è che uno Slave a cielo aperto passa il **90% dei passi in sola predizione**, sostenuto da AHRS ed encoder.

### 1.5 Passo 4.2 — Canale con latenza e perdite

Il pacchetto di posa scambiato a 10 Hz non arriva né subito né sempre.

| Parametro | Valore |
|---|---|
| Ritardo | gaussiano, $\mathcal{N}(100\text{ ms},\ (33\text{ ms})^2)$ troncato a $[0, 200]$ ms |
| Perdita di pacchetti | 0.5% |

**Sulla forma della distribuzione.** Le latenze di una rete reale sono asimmetriche a destra: c'è un pavimento fisico dato dal tempo di trasmissione, la moda sta vicino a quel pavimento, e la coda lunga viene da ritrasmissioni e collisioni. Una gaussiana simmetrica non è fedele, ma è **conservativa** — a parità di massimo ha media più alta, quindi stressa di più il sistema — e a 10 Hz il ritardo si quantizza comunque su tre soli valori (0, 1 o 2 passi), il che rende la forma in gran parte irrilevante. I 200 ms di massimo sono pessimistici per UWB, dove uno scambio dura ~1 ms, ma sotto un passo di campionamento il ritardo non sarebbe rappresentabile.

**Non serve il timestamping**, e non è una scorciatoia: è che il progetto non ha il problema che il timestamping risolve. Vanno distinti due ritardi.

- **Posa dell'ancora vecchia.** Il pacchetto del vicino arriva vecchio, ma la misura di distanza la fa la propria radio *adesso*: è fresca. Stale è solo la posizione usata per predirla, e si rimedia propagandola in avanti col modello di moto — esattamente ciò che la Fase 3 già faceva per un passo.
- **Misura fuori sequenza (OOSM).** Una misura *del proprio stato*, presa nel passato, che arriva adesso. Lì servirebbe tornare indietro nel buffer, applicarla al momento giusto e ri-propagare.

Il caso del progetto è il primo. Nessun veicolo trasmette misure *altrui*: trasmette la propria posa, che il ricevente usa come ancora. Niente buffer di $(\hat x, \Sigma)$, niente retrodizione.

E il buffer, di fatto, esiste già: `fleet(j).x_est` è l'intera storia delle stime. Ricevere con ritardo significa semplicemente **leggerla più indietro**:

```matlab
k_rx(i,j) = max(k_rx(i,j), max(1, k - round(d_s/Ts)));
```

`k_rx(i,j)` è l'indice del pacchetto più fresco che $i$ possiede di $j$. Il `max` esterno impedisce che un pacchetto tardivo sostituisca un dato più recente già ricevuto. Su un pacchetto perso l'indice non avanza e il dato invecchia da solo: **ritardo e perdita si compongono in un'unica grandezza**, l'*età* del dato usato, che è l'unica cosa che il filtro subisce.

Il consenso usa il dato ricevuto **senza compensare** il ritardo — è la condizione in cui vale il limite di stabilità teorico — mentre l'ancora mobile viene propagata fino a $t_{k+1}$ per tutta la sua età.

Il modello vale per il broadcast delle pose a 10 Hz. I cicli di consenso del D-WLS, che vivono sulla scala del millisecondo, restano ideali: perdite su quel canale sono materia della Fase 6, dove entra la connettività congiunta.

### 1.6 Risultati dei passi 4.1 e 4.2

**Il canale si comporta come richiesto.**

| Grandezza | Valore misurato |
|---|---|
| Pacchetti persi | 0.48% (richiesto 0.5%) |
| Ritardo quantizzato 0 / 1 / 2 passi | 7% / 87% / 7% |
| Età del dato usato | **100 ms in media, 400 ms al massimo** |
| Margine di stabilità consumato | 18% in media, 71% nel caso peggiore (limite 565 ms) |
| Errore dell'ancora mobile all'età media | 0.002 m, contro $\sigma_{collab} = 0.6$ m |

L'età massima di 400 ms nasce da due passi di ritardo più due passi di perdite consecutive — un evento raro (probabilità $2.5\cdot10^{-5}$ per coppia e per passo) ma che su 240 000 tentativi si presenta qualche volta. È il caso peggiore, e mangia il **71%** del margine di stabilità.

**Sull'accuratezza il GNSS lento domina, il ritardo no.**

| Veicolo | MAE con GPS | MAE in zona cieca | $\mathrm{tr}(\Sigma_{pos})$ con GPS / cieca |
|---|---|---|---|
| V1 (Master, ZED-F9P a 5 Hz) | 0.070 m | 0.078 m | 0.0140 / 0.0129 m² |
| V2 (Slave, NEO-M8N a 1 Hz) | 0.347 m | 0.086 m | 0.3321 / 0.0124 m² |
| V3 | 0.332 m | 0.073 m | 0.3341 / 0.0131 m² |
| V4 | 0.311 m | 0.084 m | 0.3323 / 0.0163 m² |
| V5 | 0.389 m | 0.075 m | 0.3330 / 0.0143 m² |

Il Master perde poco (0.061 → 0.070 m): a 5 Hz il fix RTK arriva ancora abbastanza spesso. Gli Slave passano da 0.19–0.21 a **0.31–0.39 m**, quasi il doppio, perché fra un fix e l'altro dead-reckonano per un secondo intero.

**In zona cieca invece non cambia quasi nulla** (0.073–0.089 m, era 0.074–0.089): là il riferimento è il ranging UWB, che gira a 10 Hz e non è stato toccato.

Il risultato già osservato al passo 4.0 ne esce **molto rafforzato**: gli Slave ora stimano **quattro volte meglio al buio che a cielo aperto**, e la covarianza dichiarata concorda con un fattore 27 ($0.332$ contro $0.0124$ m²). Non è un paradosso, è la conseguenza diretta di confrontare un NEO-M8N a 1 Hz con cinque ancore UWB a 10 Hz e $\sigma = 0.5$ m.

**Il ritardo costa velocità, non accuratezza.** La missione passa da 1078.5 a **1218.1 secondi**, il 13% più lenta, a parità di tutto il resto. Il consenso agisce su errori di formazione vecchi di 100 ms, il che equivale a ridurre il guadagno d'anello: la flotta resta stabile — si consuma il 18% del margine — ma reagisce più pigramente. È l'unico effetto del passo 4.2 che si vede sui numeri, e va attribuito al ritardo e non alle perdite.

**Lo 0.5% di perdita è di fatto invisibile.** Con quella probabilità l'età media del dato cresce di 0.005 passi, cioè mezzo millisecondo: sotto ogni soglia di rilevabilità. Per vedere un effetto servirebbe il 5–10%, che è materia della Fase 6.

### 1.7 Conseguenza da registrare: l'architettura a commutazione ora costa

Il ramo UWB scatta solo dentro le zone cieche. Uno Slave a cielo aperto, nel 90% dei passi in cui non ha un fix GNSS, **non usa le ancore anche quando le ha in portata**: procede in sola predizione.

Era una semplificazione accettabile con il GPS a 10 Hz. Con il GNSS a 1 Hz costa 0.35 m di errore là dove le ancore ne darebbero 0.08. La correzione è una riga — sostituire il ramo `elseif in_denied` con una condizione sulla visibilità delle ancore — ed è ora sostenuta dai dati, non da un'intuizione. È il candidato naturale al prossimo intervento.

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
1. **Causalità e assenza di loop algebrico.** Usare la stima aggiornata $\hat{x}_{j,k+1}$ renderebbe il filtro di $i$ dipendente dal filtro di $j$ al medesimo istante — e viceversa, dato che $j$ fa lo stesso con $i$. Il ritardo rompe la circolarità e corrisponde a ciò che un canale reale rende effettivamente disponibile: l'ultimo pacchetto ricevuto. Qui vale un passo solo in assenza di ritardo di canale: con il modello del §1.5 la propagazione copre l'intera età del pacchetto.
2. **Coerenza temporale.** La misura fisica $d_{ij}$ è acquisita a $t_{k+1}$. Confrontarla con una posizione riferita a $t_k$ introdurrebbe un bias sistematico di $|v_j| T_s$, dello stesso ordine del rumore del sensore ($\sigma_{collab} = 0.6$ m a 2.5 m/s).

Ciò crea un accoppiamento matematico fra gli agenti: se un veicolo perde tutti i riferimenti assoluti (no GPS, no UWB), la sua stima non degrada come nel dead-reckoning puro, ma resta agganciata a quella del resto della flotta.

### 3.3 Limiti Noti dell'Implementazione Attuale
Due limiti sono documentati esplicitamente perché condizionano l'interpretazione dei risultati e definiscono il lavoro successivo.

**a) Il filtro è ottimista sulla misura collaborativa.** La matrice $R$ associata a $z_{collab}$ contiene il solo rumore del sensore, $\sigma_{collab}^2$. L'incertezza della stima del vicino — la sua matrice $\Sigma_j$ — viene ignorata, come se $\hat{p}_j$ fosse un'ancora fissa nota esattamente. Il filtro sottostima quindi la propria covarianza.

Il pacchetto scambiato contiene infatti la sola posizione stimata: $\Sigma_j$ **non viene trasmessa**, perché finché non la si usa per la Covariance Intersection non porterebbe alcun beneficio e occuperebbe banda per nulla. Quando la CI verrà introdotta, la covarianza del vicino entrerà proiettata sulla direzione della congiungente:
$$R_{eff} = \sigma_{collab}^2 + u^T \Sigma_j^{(1:2,1:2)} u, \qquad u = \frac{\hat{p}_i - \hat{p}_j}{||\hat{p}_i - \hat{p}_j||}$$
Richiede che il vicino trasmetta, oltre alla posizione, il blocco $2 \times 2$ della propria covarianza — cioè che il **contenuto del pacchetto scambiato** passi da 2 a 5 numeri. Questo mitiga la sovra-confidenza ma **non** risolve la correlazione: poiché $i$ e $j$ si scambiano informazione ciclicamente, le stime diventano correlate in modo ignoto (*data rumination*), ed è per questo che l'architettura prevede la Covariance Intersection.

> **Programmato per la Fase 5** (README principale, §4, punto 4). I due interventi — estensione del pacchetto con $\Sigma_j$ e aggiornamento in forma CI — vanno introdotti **insieme**: usare $R_{eff}$ con il guadagno di Kalman standard tratterebbe l'incertezza del vicino come rumore indipendente, mentre la propria stima contiene già informazione arrivata da lui, e sarebbe il doppio conteggio del Cap. 15 travestito da correzione. La collocazione in Fase 5 è dettata dallo slittamento, che rende l'incertezza dei vicini non solo maggiore ma **variabile nel tempo**.

**b) La scala della formazione determina l'esistenza stessa dello scenario collaborativo.** Le zone GPS-denied hanno raggio 45–85 m. Se la formazione è larga pochi metri i mezzi condividono sempre la stessa condizione di copertura, e lo scenario di riferimento — "un veicolo perde il GPS ma un vicino lo mantiene e lo àncora" — non può verificarsi. La formazione a cinque, profonda 60 m contro i 30 m della Fase 3, migliora ulteriormente questa statistica:

| Condizione | Fase 3 (3 mezzi, 30 m) | Fase 4 (5 mezzi, 60 m) |
|---|---|---|
| Tutti coperti da GPS | 48.6% | 52.8% |
| Copertura **mista** (almeno uno coperto, almeno uno al buio) | 23.3% | **37.8%** |
| Tutti al buio | 28.1% | **9.4%** |

La copertura mista è la condizione in cui la localizzazione collaborativa serve davvero, e passa dal 23.3% al 37.8% del tempo di missione. Simmetricamente, i campioni in cui l'intera flotta è cieca crollano dal 28.1% al 9.4%: con cinque mezzi distribuiti su un fronte più ampio è molto meno probabile che tutti si trovino contemporaneamente dentro la stessa zona d'ombra.

Resta il 9.4% di campioni in cui **nessun** veicolo dispone di riferimenti assoluti diversi dalle ancore UWB fisse. In quelle condizioni il ranging inter-veicolare non può correggere la posizione assoluta della flotta: una traslazione rigida dell'intero gruppo lascia tutte le distanze relative invariate, quindi la direzione di traslazione comune appartiene al nucleo della matrice di osservabilità collettiva. È una proprietà strutturale, non un difetto di taratura.

**c) Accuratezza della stima di posa.** I valori correnti, comprensivi dei passi 4.1 e 4.2, sono nella tabella del §1.6. Con tutti i sensori a 10 Hz e canale ideale, cioè al solo passo 4.0, valevano:

| Veicolo | MAE con GPS | MAE in zona cieca |
|---|---|---|
| V1 (Master, GPS RTK) | 0.061 m | 0.077 m |
| V2–V5 (Slave, GPS standard) | 0.190–0.213 m | 0.074–0.089 m |

**Per gli Slave la stima in zona GPS-denied è più accurata che a cielo aperto**, e la covarianza dichiarata concorda. Il divario, già presente qui, si allarga a un fattore 4 con il GNSS alla sua frequenza reale. Non è un paradosso: il ranging UWB a $\sigma = 0.5$ m da cinque ancore geometricamente ben distribuite porta più informazione di un GPS standard a $\sigma = 2.0$ m. Il risultato suggerisce che in un'area attrezzata con ancore converrebbe fondere UWB e GPS *simultaneamente* anziché commutare fra i due; l'architettura a $C$ di dimensione variabile lo consente già senza modifiche strutturali.

Il fatto che i quattro Slave siano fra loro indistinguibili, comprese le due ali esterne che hanno un solo vicino ciascuna, indica che il grafo sparso non degrada la localizzazione: le ancore fisse restano il riferimento dominante in zona cieca, e il ranging collaborativo è un contributo aggiuntivo, non il sostegno principale.

**d) Errore di inseguimento a regime, e il suo costo sulla velocità della flotta.** Solo il Master riceve il termine di velocità $V_{rif}$ del path following; gli Slave si muovono unicamente per effetto del consenso, e devono quindi mantenere un errore di formazione non nullo per generare la velocità necessaria a stare al passo. È l'errore a regime di un controllo puramente proporzionale che insegue un riferimento in movimento.

C'è una legge esatta dietro. A regime la formazione è rigida e si muove tutta alla stessa velocità $v_f$, quindi $v_f = V_{rif,i} - K_{cons}(L\tilde p)_i$ per ogni veicolo. Sommando su tutti gli $i$ e sfruttando $\mathbf{1}^T L = 0$, il termine di consenso **sparisce**:

$$n\,v_f = \sum_i V_{rif,i} \qquad\Longrightarrow\qquad v_f = \frac{v_{cruise}}{N}$$

perché il solo Master riceve $V_{rif}$. Il consenso è una forza *interna* e non può spostare il baricentro: l'unica spinta esterna viene dal Master e si divide fra tutti. Verifica: $2.5/3 = 0.83$ m/s con tre mezzi (misurato 0.83), $2.5/5 = 0.50$ con cinque (misurato 0.50).

Aggiungere veicoli a una formazione del primo ordine guidata da un Master la **rallenta** quindi in proporzione diretta. La missione passa da 692 s (Fase 3) a 1079 s (passo 4.0), e a **1218 s** con il ritardo di canale del passo 4.2, che toglie un ulteriore 13% agendo su errori di formazione vecchi di 100 ms.

La correzione è **una riga**: propagare $V_{rif}$ a tutta la flotta in feedforward, così che $\sum_i V_{rif,i} = N v_{cruise}$ e $v_f = v_{cruise}$. È esattamente ciò che fa la Fase 2, dove infatti il fenomeno non si presenta. L'alternativa strutturale è il consenso del secondo ordine, in cui ogni agente ha una propria posizione desiderata e il Master smette di essere un punto singolo di guasto.


## 4. Figure Prodotte

L'esecuzione di `main4.m` genera la cartella `fase_4/risultati/` con otto figure, con la stessa nomenclatura della Fase 3 per consentire il confronto diretto fra le due configurazioni.

| File | Contenuto |
|---|---|
| `1_mappa_navigazione.png` | mappa completa: percorso nominale, zone GPS-denied, ancore UWB, traiettorie reali e stimate |
| `2_errore_posizione_2d.png` | errore di posizione scalare $\lVert e_{pos}\rVert$, con le fasce GPS-denied ombreggiate |
| `3_diagnostica_ekf.png` | errori separati su $X$, $Y$, $\theta$ |
| `4_forze_virtuali.png` | magnitudo dei termini di consenso e di repulsione |
| `5_copertura_e_covarianza.png` | riferimenti assoluti disponibili e traccia di $\Sigma_{pos}$ nel tempo |
| `6_bound_3sigma.png` | errore di stima confrontato con l'inviluppo a $\pm 3\sigma$ dichiarato dal filtro |
| `7_grafo_comunicazione.png` | $\lambda_2(L)$, spettro $\lambda_i(Q)$ e distanze inter-veicolari contro il raggio radio |
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

Il Master, che dispone di GNSS RTK a 5 Hz, resta pressoché indifferente all'ingresso in zona cieca. Gli Slave, con NEO-M8N a 1 Hz, registrano invece un **miglioramento di 27 volte** sulla traccia della covarianza (0.332 contro 0.0124 m²): con cinque ancore ben distribuite il ranging UWB a $\sigma = 0.5$ m e 10 Hz porta molta più informazione di un GNSS a $\sigma = 2.0$ m e 1 Hz. È la quantificazione, sul piano della covarianza, del risultato riportato al §1.6 in termini di errore.

### 4.3 Grafo di comunicazione (figura 7)
La legge di consenso è pesata dalla matrice di **adiacenza** del grafo. Il raggio di comunicazione coincide con `r_collab` = **55 m**, la portata del ranging inter-veicolare: è la stessa radio UWB a fornire sia la misura di distanza sia il canale dati, quindi non avrebbe senso che il consenso raggiungesse un vicino con cui il ranging è impossibile. La riduzione da 120 a 55 m rispetto alla Fase 3 è motivata al §1.1.

| Grandezza | Valore misurato |
|---|---|
| Archi attivi | 5 su 10 coppie possibili |
| Connettività algebrica $\lambda_2(L)$ | 0.6972 ($K_5$ completo darebbe 5) |
| Componenti connesse $\mathrm{mol}_{\lambda_1}(L) = \mathrm{mol}_{\lambda_1}(Q)$ | 1 |
| Spettro $\lambda_i(Q)$ (per modulo) | $[+1.000,\ +0.826,\ +0.654,\ +0.096,\ -0.076]$ |
| Essential spectral radius $\rho_2 = \lvert\lambda_2(Q)\rvert$ | 0.8257 |
| Autovalore minimo $\lambda_{min}(Q)$ | $-0.0757$ (lontano da $-1$: nessuna oscillazione) |
| Margine di discretizzazione $K_{cons}T_s\lambda_{max}(L)$ | 0.278 (limite: 2) |
| Diametro del grafo | 3 salti |
| Costante di tempo $\tau = 1/(K_{cons}\lambda_2(L))$ | 2.22 s |
| Grafo connesso per l'intera missione | sì |
| Margine sui link attivi / assenti | +35% / −17% |

Il terzo pannello separa graficamente i due gruppi di coppie: cinque curve continue fra 36 e 41 m, ben sotto il raggio radio, e cinque punteggiate fra 66 e 81 m, altrettanto nettamente sopra. Il vuoto fra i due gruppi è ciò che rende la topologia deterministica malgrado l'errore di formazione.

I primi due pannelli restano costanti perché nessuna coppia attraversa mai la soglia. La topologia diventerà tempo-variante ai passi 4.2 e in Fase 6, con latenze e perdite di pacchetto — deliberatamente, non per effetto di un margine mal scelto.

**Attenzione a non confondere i due spettri.** $\lambda_2(L) = 0.697$ è il secondo autovalore del **Laplaciano**, e misura quanto la rete è ben collegata: vale zero se e solo se il grafo è sconnesso. $\rho_2 = 0.826$ viene dallo spettro della **matrice di Metropolis** $Q$, e misura quanto lentamente il consenso converge: vale zero sul grafo completo. I due si muovono in verso opposto, e questa fase lo mostra sullo stesso grafo. La convenzione completa sui simboli è in [TEORIA_consenso_su_grafi.md §5](../theory/TEORIA_consenso_su_grafi.md).

> Trattazione completa: [TEORIA_consenso_su_grafi.md](../theory/TEORIA_consenso_su_grafi.md).

### 4.4 Consistenza (figura 6)
Confronto fra l'errore di stima effettivo e l'inviluppo $\pm 3\sigma$ estratto da `Sigma_hist`. La frazione di campioni fuori banda risulta compresa fra 0.0% e 0.3% contro un valore atteso di 0.3% per un filtro esattamente calibrato: il filtro è consistente e leggermente conservativo. La verifica è condotta su singolo run a scopo diagnostico; la validazione statistica con campagna Monte Carlo e test NEES è prevista in Fase 6.

### 4.5 Nota sulla figura 4
La formazione viene inizializzata già nella configurazione desiderata, quindi non esiste il transitorio di riavvicinamento presente in Fase 2 e la forza repulsiva risulta **identicamente nulla** per l'intera missione. Lo sforzo di consenso si mantiene pressoché costante, e il suo valore non nullo a regime corrisponde all'errore di inseguimento descritto al §3.3, punto d.

### 4.6 Stima distribuita del parametro di terreno (figura 8)

Alla stima della posa, dinamica e locale a ciascun mezzo, si affianca un secondo problema di natura diversa: identificare una proprietà del **terreno**, uguale per tutta la flotta e costante nel tempo. Trattandosi di un parametro costante e non stocastico il Cap. 18 prescrive il **D-WLS**; il DKF servirebbe se la grandezza avesse una dinamica propria. Il modello è la resistenza specifica al moto,

$$\frac{F_{traz}}{W} = \mu_{terr} + c_{terr}\,v^2 + \varepsilon, \qquad C_i = \begin{bmatrix}1 & v_i^2\end{bmatrix}$$

misurata da un **torsiometro** sull'albero di trasmissione, normalizzando sul peso del mezzo. È una lettura del terreno e non un'azione su di esso: l'impianto simulato resta quello delle sezioni precedenti e i risultati già riportati non cambiano. La covarianza segue la convenzione degli altri sensori: `R_traz_master` $= 0.010^2$, `R_traz_slave` $= 0.025^2$.

**Il consenso viaggia sulla scala dei tempi della radio, non del controllo.** Uno scambio TW-TOF dura circa 1 ms contro i 100 ms del passo di campionamento, quindi fra due istanti di controllo il canale sostiene decine di cicli. Un round completo di D-WLS è eseguito **a ogni passo** — 10 786 round sui 1079 s di missione — con metà del passo riservata a questo traffico e l'altra metà a ranging e broadcast delle pose, per un budget di $q_{max} = 50$ cicli. La stima del terreno è così disponibile a 10 Hz, alla stessa cadenza di quella di posa.

Il consenso opera su **copie** degli accumulatori locali, che crescono per tutta la missione senza mai essere toccati: l'informazione di ogni veicolo resta quella genuinamente prodotta dai suoi sensori e non viene ricontata al round successivo.

**È qui che il costo della comunicazione smette di essere trascurabile.** Con $\rho_2 = 0.826$ e diametro 3, il dimensionamento automatico chiede **49 cicli su 50 disponibili**: il 98% della banda allocata al D-WLS. In Fase 3, sul grafo completo, ne bastava uno. Il consenso passa da voce marginale a vincolo di progetto, e la scelta del raggio radio diventa un compromesso fra qualità del link e costo del consenso.

La previsione è conservativa: ne bastano 34 nei fatti, perché $\rho_2$ governa il decadimento *asintotico* e trascura la costante moltiplicativa. Sovrastimare è il verso giusto in cui sbagliare, ma qui lo si paga in banda.

| Proprietà | Fase 3 ($K_3$) | Fase 4 (5 nodi, 5 archi) |
|---|---|---|
| Cicli dimensionati / osservati | 1 / 1 | **49 / 34** |
| Residuo di consenso | $3.3\cdot10^{-16}$ | $5.5\cdot10^{-6}$ |
| Scarto dal WLS centralizzato | $1.5\cdot10^{-15}$ | $5.2\cdot10^{-7}$ |
| Disaccordo fra i veicoli | $8.4\cdot10^{-16}$ | $9.2\cdot10^{-9}$ |
| Invarianza di $\sum_i F_i(k)$ | $2.8\cdot10^{-16}$ | $1.0\cdot10^{-15}$ |
| Ricostruzione della covarianza da $(n F_i)^{-1}$ | $1.0\cdot10^{-14}$ | $1.9\cdot10^{-9}$ |

Lo scarto dal centralizzato risale di otto ordini di grandezza, da precisione di macchina a $5\cdot10^{-7}$: con $q$ troncato il consenso è **convergente ma non esatto**, la condizione normale fuori dal caso completo. L'unica riga che resta a precisione di macchina è l'invarianza della somma, perché non dipende da quanti cicli si eseguono ma solo dalla doppia stocasticità di $Q$.

Nei primi due pannelli le curve D-WLS dei cinque veicoli coincidono, e sono disegnate con spessore decrescente per renderle distinguibili. Le punteggiate mostrano cosa otterrebbe ciascun mezzo **senza cooperare**: le due ali esterne si assestano attorno all'8% di errore relativo, contro lo 0.31% della soluzione distribuita.

**Quanto vale la cooperazione.** Il confronto corretto non è il numero di condizionamento — aggiungere Slave lo peggiora leggermente, pur aggiungendo informazione — ma la covarianza $(\sum_i F_i)^{-1}$, che per l'ordinamento di Loewner può solo ridursi:

| Veicolo | $\mathrm{dev}(c_{terr})$ da solo | in rete | guadagno | $\mathrm{std}(v^2)$ |
|---|---|---|---|---|
| V1 (Master) | $4.55\cdot10^{-4}$ | $4.47\cdot10^{-4}$ | 1.0× | 0.215 |
| V2 (ala interna) | $5.44\cdot10^{-3}$ | $4.47\cdot10^{-4}$ | **12.2×** | 0.034 |
| V3 (ala interna) | $3.13\cdot10^{-3}$ | $4.47\cdot10^{-4}$ | **7.0×** | 0.070 |
| V4 (ala esterna) | $6.74\cdot10^{-3}$ | $4.47\cdot10^{-4}$ | **15.1×** | 0.017 |
| V5 (ala esterna) | $6.33\cdot10^{-3}$ | $4.47\cdot10^{-4}$ | **14.2×** | 0.020 |

L'ultima colonna spiega la terza: il guadagno di ciascun mezzo è tanto maggiore quanto meno quel mezzo, da solo, esplora velocità diverse. Il Master, che ha sia il sensore migliore ($R_{traz} = 0.010^2$ contro $0.025^2$) sia la massima escursione, da solo arriverebbe già dove arriva la rete. La rete non produce un miglioramento uniforme: **distribuisce a tutti la qualità del membro meglio strumentato**, la stessa struttura del GPS RTK montato sul solo Master.

**Accuratezza raggiunta.** La stima finale vale $\mu_{terr} = 0.0902$ e $c_{terr} = 0.00480$ contro valori veri di 0.0900 e 0.00500, cioè $+1.44$ e $-0.44$ deviazioni standard. In Fase 3, con la formazione stretta, lo scarto su $c_{terr}$ era di $-2.06$ deviazioni e l'errore relativo dell'1.53%; qui scende allo **0.31%**.

**Il miglioramento non viene dall'andare più veloci, ma dall'andare a velocità più diverse.** La flotta a cinque è più lenta di quella a tre — 0.50 contro 0.83 m/s di media — perché il Master trascina più vicini. Ma le ali esterne stanno a 40 m dall'asse contro i 20 m delle interne, quindi in curva la loro velocità si scosta dal Master il doppio, e la dispersione di $v^2$ che condiziona il problema di stima cresce. È la stessa lezione della GDOP: conta la diversità geometrica delle misure, non il loro numero né la loro entità.

**Limite residuo: il regressore incerto.** La riga $C_i = [1,\ v_i^2]$ usa la velocità *stimata*, l'unica disponibile a bordo, e un regressore rumoroso attenua la pendenza verso lo zero. Rifacendo il calcolo con la velocità vera si ottiene $c_{terr} = 0.005015$, cioè $+0.03$ deviazioni: quasi esatto. L'effetto non è sparito, è diventato piccolo rispetto all'informazione ora disponibile, e tornerà a contare in Fase 5 quando lo slittamento renderà il comando diverso dalla velocità effettiva. La covarianza dichiarata non lo modella e risulta quindi ottimista, come già la misura collaborativa che ignora $\Sigma_j$ (§3.3).

> Trattazione completa e validazione algoritmica su topologie note (`common/verifica_dwls.m`): [TEORIA_stima_distribuita.md](../theory/TEORIA_stima_distribuita.md).
