# Fase 5: Fusione Consistente con Covariance Intersection

## 1. Obiettivo

La Fase 5 rimuove l'ultima ipotesi ottimistica rimasta nell'architettura di stima: che la posa ricevuta da un vicino sia **esatta**. Lo scenario fisico è identico a quello della Fase 4 — stesso ambiente, stessa flotta, stesso canale — e questo è voluto: l'unica variabile che cambia è la regola di fusione, così l'effetto è isolabile.

| Passo | Contenuto | Stato |
|---|---|---|
| **5.1** | **Covariance Intersection** sull'aggiornamento collaborativo, pacchetto esteso a $(\hat p_j, \Sigma_j)$ | **fatto** |
| 5.2 | Impianto: la ground truth passa da $v^{true} = v_{cmd}$ a una legge di slittamento | da fare |
| 5.3 | Modello di misura degli encoder in presenza di slittamento | da fare |
| 5.4 | Rumore di processo: taratura di $q_a$, $q_\alpha$ e attivazione di $k_{terreno}$ | da fare |

Il passo 5.1 viene per primo pur essendo indipendente dal modello di slittamento: modificare il formato del pacchetto una volta sola costa meno che farlo due volte, e la CI è il prerequisito perché l'incertezza variabile introdotta dallo slittamento venga poi trattata correttamente.

### 1.1 Che cosa si eredita dalla Fase 4

Nulla di quanto segue viene toccato in questa fase. La documentazione completa è in [fase_4/README4.md](../fase_4/README4.md); qui se ne riportano solo i valori operativi.

| Grandezza | Valore | Origine |
|---|---|---|
| Veicoli / raggio radio | 5 / 55 m | passo 4.0 |
| Archi attivi | 5 su 10 possibili | passo 4.0 |
| $\lambda_2(L)$ / $\rho_2 = \lvert\lambda_2(Q)\rvert$ / diametro | 0.697 / 0.826 / 3 salti | passo 4.0 |
| $K_{cons}$ | 0.646 ($\tau = 2.22$ s) | passo 4.0 |
| GNSS Master / Slave | ZED-F9P a 5 Hz / NEO-M8N a 1 Hz | passo 4.1 |
| Ritardo / perdite sul broadcast | $[0, 200]$ ms gaussiano troncato / 0.5% | passo 4.2 |
| Cicli di consenso D-WLS | 49 su 50 disponibili | passo 4.0 |

L'ambiente è **lo stesso file** della Fase 4: `ambiente_fase5.mat` è byte per byte identico ad `ambiente_fase4.mat`. Mappa, zone cieche e posizione delle ancore coincidono, quindi il confronto fra le due fasi è diretto.

### 1.2 Il problema: l'ultimo filtro ottimista

Fino alla Fase 4 l'aggiornamento collaborativo trattava la posa del vicino come un punto noto: nella matrice $R$ entrava il solo rumore del sensore di ranging, $\sigma_{collab}^2$. Due cose erano sbagliate, e in versi opposti.

**L'incertezza del vicino non veniva contata affatto.** Il vicino non è un'ancora fissa rilevata una volta per tutte: è un veicolo che sta stimando la propria posa, con un'incertezza $\Sigma_j$ dello stesso ordine di grandezza della propria. Ignorarla equivale a dichiarare un riferimento migliore di quello che è.

**La correlazione fra le due stime non veniva contata nemmeno.** Ed è il problema più serio, perché non si risolve gonfiando $R$. In una rete con cicli — e il grafo della Fase 4 ne ha — l'informazione che il veicolo $i$ trasmette torna indietro dopo pochi salti dentro la stima di $j$, e rientra nel filtro di $i$ come se fosse nuova. Il termine di covarianza incrociata $P_{ij} = E[\tilde x_i \tilde x_j^T]$ è reale e non nullo, ma tracciarlo esattamente richiederebbe che ogni agente conosca l'intera topologia, la storia degli scambi e i modelli di rumore di tutti gli altri: costo $O(N^3)$, comunicazione continua, e impossibile in presenza di perdite. Imporre $P_{ij} = 0$ è la causa riconosciuta della divergenza nella localizzazione cooperativa decentralizzata — il *data rumination*.

> **Perché non si può correggere solo $R$.** Usare $\Sigma_j$ per gonfiare $R$ lasciando il guadagno di Kalman standard sarebbe **peggiore** che ignorarla, non migliore: tratterebbe l'incertezza del vicino come rumore *indipendente*, cioè affermerebbe esplicitamente $P_{ij} = 0$ proprio nel punto in cui la correlazione nasce. È il doppio conteggio del Cap. 15 travestito da rigore. Estensione del pacchetto e CI vanno introdotte **insieme**, ed è la ragione per cui la Fase 4 non trasmetteva $\Sigma_j$.

### 1.3 L'algoritmo

**Il pacchetto si estende.** Da $\hat p_j$ soli a $(\hat p_j, \Sigma_j^{(1:2,1:2)})$: da 2 a 5 numeri, essendo la covarianza simmetrica. A 8 byte per numero sono 40 B a pacchetto, cioè **800 B/s ricevuti** da ciascun veicolo a 10 Hz con due vicini in media. Il dato viaggia sullo stesso canale del broadcast della posa e subisce lo stesso ritardo e le stesse perdite.

$\Sigma_j$ **non viene propagata in avanti** come invece si fa con la posizione. Su un'età massima di 400 ms il modello di processo aggiungerebbe circa $1.3\cdot10^{-4}$ m², cioè lo 0.3% di una varianza di posizione tipica di 0.04–0.16 m². Propagarla richiederebbe le Jacobiane del vicino per un contributo che si perde nell'arrotondamento.

**L'aggiornamento si spezza in due.** Non tutte le misure sono correlate con la stima a priori, e applicare la CI a tutte costerebbe ottimalità senza alcun guadagno:

| Blocco | Misure | Regola | Motivo |
|---|---|---|---|
| **1** | AHRS, encoder, GNSS, ancore fisse UWB | Kalman standard | sensori propri del veicolo e riferimenti a posizione nota: nessuna informazione proveniente dalla rete |
| **2** | ranging verso i vicini | **Covariance Intersection** | la posa del vicino contiene informazione già passata da questo filtro |

È l'architettura nota come **Split Covariance Intersection**. Il blocco 2 linearizza attorno al risultato del blocco 1, che è l'a priori del secondo aggiornamento.

**L'incertezza del vicino entra proiettata** (Carrillo-Arce et al., IROS 2013). La CI classica fonde due stime della *stessa* grandezza, mentre qui il vicino stima il *proprio* stato e una misura di sola distanza non produce una stima puntuale della propria posizione. Della sua ellisse di incertezza conta soltanto l'estensione **lungo la congiungente**, l'unica direzione che il ranging legge:

$$R_{eff} = \sigma_{collab}^2 + u_{ij}^T\,\Sigma_j^{(1:2,1:2)}\,u_{ij}, \qquad u_{ij} = \frac{\hat p_i - \hat p_j}{\lVert \hat p_i - \hat p_j \rVert}$$

La proiezione riduce una matrice a uno scalare omogeneo a $\sigma_{collab}^2$, e la somma è lecita perché il rumore del sensore è indipendente dall'errore di $j$. Il vettore $u_{ij}$ è già la prima riga del Jacobiano $C$: non è un calcolo aggiuntivo.

**La CI è un aggiornamento di Kalman con a priori e misura sgonfiati.** Posto $\Sigma_\gamma = \bar\Sigma/\gamma$ e $R_\gamma = R_{eff}/(1-\gamma)$, la definizione

$$\Sigma^{-1} = \gamma\,\bar\Sigma^{-1} + (1-\gamma)\,C^T R_{eff}^{-1} C$$

diventa $\Sigma^{-1} = \Sigma_\gamma^{-1} + C^T R_\gamma^{-1} C$, cioè la forma informativa dell'aggiornamento di Kalman. Le equazioni restano quelle di sempre, applicate a due matrici riscalate. È anche il motivo per cui la CI si innesta su un EKF senza riscriverne l'architettura: la nonlinearità resta confinata in $C$ e in $h(\cdot)$.

**Il peso $\gamma$ è scelto minimizzando la traccia del blocco di posizione.** Il problema è convesso e scalare su $[0,1]$, quindi si risolve con `fminbnd` (sezione aurea con interpolazione parabolica, il metodo indicato nella teoria). Si minimizza la traccia della sola posizione e non dell'intera $\Sigma$ perché lo stato mescola metri, radianti e velocità: sommarne le varianze darebbe un costo dimensionalmente incoerente, dominato dall'unità di misura più grande.

> **Notazione.** Il documento di teoria [theory/TEORIA_CI.pdf](../theory/TEORIA_CI.pdf) chiama $\omega$ questo peso. Nel progetto si usa $\gamma$, come già in [README.md §2.1](../README.md) e §2.3, per non collidere con la velocità angolare $\omega$ del modello uniciclo. Nel codice è `gamma_ci`, perché `gamma` è una funzione predefinita di MATLAB.

Il caso $\gamma = 1$ — non fondere, e tenersi l'a priori — appartiene all'insieme ammissibile ma cade sull'estremo che la ricerca non raggiunge, perché lì $1/(1-\gamma)$ diverge. Viene quindi confrontato esplicitamente con l'ottimo trovato: senza quel confronto resterebbe a ogni passo un gonfiamento residuo dello 0.04%, irrilevante su una fusione sola ma non su un filtro che fonde 16 000 volte.

### 1.4 Risultati

**La stima non peggiora, la covarianza dichiarata sale.**

| Veicolo | MAE con GPS | MAE in zona cieca | $\mathrm{tr}(\Sigma_{pos})$ cieca, Fase 4 | Fase 5 | NEES posizione |
|---|---|---|---|---|---|
| V1 (Master) | 0.070 m | 0.077 m | 0.0129 m² | 0.0146 m² | 1.08 |
| V2 (Slave) | 0.354 m | 0.080 m | 0.0124 m² | 0.0156 m² | 1.29 |
| V3 | 0.331 m | 0.081 m | 0.0131 m² | 0.0149 m² | 1.36 |
| V4 | 0.324 m | 0.087 m | 0.0163 m² | 0.0184 m² | 1.49 |
| V5 | 0.369 m | 0.078 m | 0.0143 m² | 0.0148 m² | 1.31 |

L'errore in zona cieca resta nella stessa fascia della Fase 4 (0.073–0.086 m), mentre la traccia della covarianza cresce mediamente del **14%**. È esattamente il comportamento atteso: la CI non migliora la stima, **toglie una confidenza che non era giustificata**. La missione dura 1218.3 s contro i 1218.1 della Fase 4, cioè non cambia.

**Il NEES conferma la consistenza.** L'indice $(\tilde p^T \Sigma_{pos}^{-1}\tilde p)$ ha valore atteso 2 per una stima bidimensionale: sopra 2 il filtro è ottimista, sotto è conservativo. Tutti e cinque i veicoli stanno fra **1.08 e 1.49**, cioè dalla parte sicura, che è la sola garanzia che la CI promette.

### 1.5 Il risultato che conta: la CI rifiuta la misura collaborativa

Il dato più informativo dell'intera fase è che **il peso $\gamma$ torna pari a 1 in tutte e 16 218 le fusioni**. La Covariance Intersection, messa nella condizione di decidere quanto fidarsi del vicino, decide di **non usarlo affatto**.

| Grandezza | Valore |
|---|---|
| Fusioni in forma CI | 16 218 su 16 218 passi in zona cieca |
| Di cui con misura accolta ($\gamma < 1$) | **0** |
| Gonfiamento di $R$, $R_{eff}/\sigma_{collab}^2$ | 1.14× in media, 1.80× al massimo |
| Incertezza sulla congiungente: a priori | **0.09 m** |
| Incertezza sulla congiungente: misura | **0.62 m** |
| Ancore fisse viste in zona cieca | 4.5 in media, minimo 3 |

Non è un ottimizzatore bloccato: è una soglia esatta. Con a priori isotropo di varianza $s$ e misura scalare di varianza $R$ lungo una direzione, la traccia risultante vale $f(\gamma) = 1/(\gamma/s + (1-\gamma)/R) + s/\gamma$ e la sua derivata in $\gamma = 1$ è $s^2/R - 2s$. Il minimo lascia il bordo — cioè la misura viene usata — solo se $s > 2R$, ovvero

$$\mathrm{dev}(\text{a priori}) > \sqrt{2}\;\mathrm{dev}(\text{misura})$$

Qui il rapporto vale **0.09/0.62 = 0.15**, dieci volte sotto la soglia. La ragione sta nell'ultima riga della tabella: in zona cieca il veicolo vede in media 4.5 ancore fisse a $\sigma = 0.5$ m e non ne vede mai meno di 3. Un a priori così vincolato non ha nulla da guadagnare da un singolo range verso un veicolo che è a sua volta incerto.

**Che cosa se ne ricava.** La Fase 4 aveva già osservato che «le ancore fisse restano il riferimento dominante in zona cieca, e il ranging collaborativo è un contributo aggiuntivo». La CI trasforma quell'osservazione qualitativa in un verdetto quantitativo: in questa configurazione il ranging collaborativo non è un contributo aggiuntivo, è **rumore travestito da informazione**, e il guadagno di accuratezza che sembrava portare in Fase 4 era interamente confidenza spuria. Il 14% di covarianza in più misurato al §1.4 è la restituzione di quel prestito.

Il risultato è condizionato alla geometria, non generale. La CI userebbe il vicino nel momento in cui l'a priori si degradasse oltre la soglia: un veicolo con **una o zero ancore in vista**, o l'ingresso del modello di slittamento dei passi 5.2–5.4, che allarga $\Sigma$ proprio in modo variabile nel tempo. È la condizione in cui la localizzazione collaborativa serve davvero, e la Fase 5 lascia l'infrastruttura pronta a riconoscerla da sola.

### 1.6 Validazione

`common/verifica_ci.m` verifica sei proprietà indipendenti dalla simulazione.

| Test | Verifica | Esito |
|---|---|---|
| 1 | La forma di Kalman sgonfiata coincide con $\Sigma^{-1} = \gamma\bar\Sigma^{-1} + (1-\gamma)C^TR^{-1}C$ | errore 0 |
| 2 | Il peso si muove: misura debole $\to \gamma = 1$, misura forte $\to \gamma = 0.0007$ | ✓ |
| 3 | La fusione non peggiora mai: $\mathrm{tr}(\Sigma_{pos}) \le \mathrm{tr}(\bar\Sigma_{pos})$ | 0 violazioni su 500 |
| 4 | **Consistenza sotto correlazione ignota**, 20 000 prove Monte Carlo | ✓ |
| 5 | La geometria della Fase 5 porta a $\gamma = 1$ | ✓ |
| 6 | La soglia di accoglimento è $\sqrt 2$: 1.35 scartata, 1.45 accolta | ✓ |

Il TEST 4 è quello che giustifica l'esistenza dell'algoritmo nel progetto. Si costruiscono due stime della stessa posizione i cui errori condividono una componente comune, con correlazione reale 0.74 ignota a entrambi gli stimatori, e si confrontano le due regole di fusione:

| Regola | NEES | Covarianza dichiarata | Covarianza reale |
|---|---|---|---|
| Kalman standard ($P_{ij} = 0$ imposto) | **3.46** | 0.170 m² | 0.297 m² |
| Covariance Intersection | **1.81** | 0.340 m² | 0.310 m² |

Il guadagno di Kalman dichiara **1.75 volte meno incertezza** di quanta ne abbia davvero; la CI ne dichiara un po' di più del necessario. Il caso è deliberatamente lineare, così il test isola la regola di fusione dall'errore di linearizzazione.

### 1.7 Conseguenza da registrare: l'architettura a commutazione ora costa il doppio

Il ramo UWB scatta solo dentro le zone cieche. Uno Slave a cielo aperto, nel 90% dei passi in cui non ha un fix GNSS, **non usa le ancore anche quando le ha in portata**: procede in sola predizione, con 0.35 m di errore là dove le ancore ne darebbero 0.08.

La Fase 5 aggiunge una seconda ragione alla stessa correzione. Quella commutazione è anche ciò che tiene l'a priori in zona cieca sempre così stretto da rendere inutile il ranging collaborativo: le due condizioni — «poche ancore» e «vicini utili» — non si presentano mai insieme, per costruzione dell'architettura e non per proprietà dell'ambiente. Sostituire `elseif in_denied` con una condizione sulla visibilità delle ancore resta un intervento di una riga, ed è ora sostenuto da due misure indipendenti.

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
1. **Causalità e assenza di loop algebrico.** Usare la stima aggiornata $\hat{x}_{j,k+1}$ renderebbe il filtro di $i$ dipendente dal filtro di $j$ al medesimo istante — e viceversa, dato che $j$ fa lo stesso con $i$. Il ritardo rompe la circolarità e corrisponde a ciò che un canale reale rende effettivamente disponibile: l'ultimo pacchetto ricevuto. Qui vale un passo solo in assenza di ritardo di canale: con il canale non ideale ereditato dalla Fase 4 la propagazione copre l'intera età del pacchetto.
2. **Coerenza temporale.** La misura fisica $d_{ij}$ è acquisita a $t_{k+1}$. Confrontarla con una posizione riferita a $t_k$ introdurrebbe un bias sistematico di $|v_j| T_s$, dello stesso ordine del rumore del sensore ($\sigma_{collab} = 0.6$ m a 2.5 m/s).

Ciò crea un accoppiamento matematico fra gli agenti: se un veicolo perde tutti i riferimenti assoluti (no GPS, no UWB), la sua stima non degrada come nel dead-reckoning puro, ma resta agganciata a quella del resto della flotta.

**È proprio quell'accoppiamento a rendere la misura diversa da tutte le altre.** L'informazione che $i$ trasmette torna indietro dentro la stima di $j$ e rientra nel filtro di $i$ come se fosse nuova: l'ipotesi di scorrelazione su cui poggia il guadagno di Kalman non regge. Dalla Fase 5 questa singola riga di misura è quindi estratta dal blocco comune e aggiornata in forma **Covariance Intersection** (§1.3), mentre AHRS, encoder, GNSS e ancore fisse restano su guadagno di Kalman standard. Il pacchetto ricevuto porta con sé $\Sigma_j$, che entra proiettata sulla congiungente.

### 3.3 Limiti Noti dell'Implementazione Attuale
Due limiti sono documentati esplicitamente perché condizionano l'interpretazione dei risultati e definiscono il lavoro successivo.

**a) Il filtro era ottimista sulla misura collaborativa — risolto in questa fase.** Fino alla Fase 4 la matrice $R$ associata a $z_{collab}$ conteneva il solo rumore del sensore, $\sigma_{collab}^2$: l'incertezza $\Sigma_j$ del vicino veniva ignorata, come se $\hat{p}_j$ fosse un'ancora fissa nota esattamente, e la correlazione fra i due filtri nemmeno considerata.

Il passo 5.1 chiude entrambi i punti insieme, come richiesto: il pacchetto passa da 2 a 5 numeri per trasportare il blocco $2\times 2$ della covarianza, questa entra proiettata sulla direzione della congiungente,
$$R_{eff} = \sigma_{collab}^2 + u^T \Sigma_j^{(1:2,1:2)} u, \qquad u = \frac{\hat{p}_i - \hat{p}_j}{||\hat{p}_i - \hat{p}_j||}$$
e l'aggiornamento viene eseguito in forma CI anziché con il guadagno di Kalman. Derivazione, risultati e validazione ai §1.2–1.6.

**Il verdetto della CI su questa configurazione è netto: la misura collaborativa viene scartata in tutte le fusioni.** L'a priori, già vincolato da 4.5 ancore fisse in media, è 7.3 volte migliore della misura, mentre la soglia di accoglimento è $\sqrt2$. Quanto scritto al punto c) — "il ranging collaborativo è un contributo aggiuntivo, non il sostegno principale" — ne esce quantificato: in zona cieca non è nemmeno un contributo aggiuntivo.

**b) La scala della formazione determina l'esistenza stessa dello scenario collaborativo.** Le zone GPS-denied hanno raggio 45–85 m. Se la formazione è larga pochi metri i mezzi condividono sempre la stessa condizione di copertura, e lo scenario di riferimento — "un veicolo perde il GPS ma un vicino lo mantiene e lo àncora" — non può verificarsi. La formazione a cinque, profonda 60 m contro i 30 m della Fase 3, migliora ulteriormente questa statistica:

| Condizione | Fase 3 (3 mezzi, 30 m) | Fase 4 (5 mezzi, 60 m) |
|---|---|---|
| Tutti coperti da GPS | 48.6% | 52.8% |
| Copertura **mista** (almeno uno coperto, almeno uno al buio) | 23.3% | **37.8%** |
| Tutti al buio | 28.1% | **9.4%** |

La copertura mista è la condizione in cui la localizzazione collaborativa serve davvero, e passa dal 23.3% al 37.8% del tempo di missione. Simmetricamente, i campioni in cui l'intera flotta è cieca crollano dal 28.1% al 9.4%: con cinque mezzi distribuiti su un fronte più ampio è molto meno probabile che tutti si trovino contemporaneamente dentro la stessa zona d'ombra.

Resta il 9.4% di campioni in cui **nessun** veicolo dispone di riferimenti assoluti diversi dalle ancore UWB fisse. In quelle condizioni il ranging inter-veicolare non può correggere la posizione assoluta della flotta: una traslazione rigida dell'intero gruppo lascia tutte le distanze relative invariate, quindi la direzione di traslazione comune appartiene al nucleo della matrice di osservabilità collettiva. È una proprietà strutturale, non un difetto di taratura.

**c) Accuratezza della stima di posa.** I valori correnti sono nella tabella del §1.4. Con tutti i sensori a 10 Hz e canale ideale, cioè al solo passo 4.0, valevano:

| Veicolo | MAE con GPS | MAE in zona cieca |
|---|---|---|
| V1 (Master, GPS RTK) | 0.061 m | 0.077 m |
| V2–V5 (Slave, GPS standard) | 0.190–0.213 m | 0.074–0.089 m |

**Per gli Slave la stima in zona GPS-denied è più accurata che a cielo aperto**, e la covarianza dichiarata concorda. Il divario, già presente qui, si allarga a un fattore 4 con il GNSS alla sua frequenza reale. Non è un paradosso: il ranging UWB a $\sigma = 0.5$ m da cinque ancore geometricamente ben distribuite porta più informazione di un GPS standard a $\sigma = 2.0$ m. Il risultato suggerisce che in un'area attrezzata con ancore converrebbe fondere UWB e GPS *simultaneamente* anziché commutare fra i due; l'architettura a $C$ di dimensione variabile lo consente già senza modifiche strutturali.

Il fatto che i quattro Slave siano fra loro indistinguibili, comprese le due ali esterne che hanno un solo vicino ciascuna, indica che il grafo sparso non degrada la localizzazione: le ancore fisse restano il riferimento dominante in zona cieca. La Fase 5 porta l'osservazione alle sue conseguenze — vedi §1.5.

**d) Errore di inseguimento a regime, e il suo costo sulla velocità della flotta.** Solo il Master riceve il termine di velocità $V_{rif}$ del path following; gli Slave si muovono unicamente per effetto del consenso, e devono quindi mantenere un errore di formazione non nullo per generare la velocità necessaria a stare al passo. È l'errore a regime di un controllo puramente proporzionale che insegue un riferimento in movimento.

C'è una legge esatta dietro. A regime la formazione è rigida e si muove tutta alla stessa velocità $v_f$, quindi $v_f = V_{rif,i} - K_{cons}(L\tilde p)_i$ per ogni veicolo. Sommando su tutti gli $i$ e sfruttando $\mathbf{1}^T L = 0$, il termine di consenso **sparisce**:

$$n\,v_f = \sum_i V_{rif,i} \qquad\Longrightarrow\qquad v_f = \frac{v_{cruise}}{N}$$

perché il solo Master riceve $V_{rif}$. Il consenso è una forza *interna* e non può spostare il baricentro: l'unica spinta esterna viene dal Master e si divide fra tutti. Verifica: $2.5/3 = 0.83$ m/s con tre mezzi (misurato 0.83), $2.5/5 = 0.50$ con cinque (misurato 0.50).

Aggiungere veicoli a una formazione del primo ordine guidata da un Master la **rallenta** quindi in proporzione diretta. La missione passa da 692 s (Fase 3) a 1079 s (passo 4.0), e a **1218 s** con il ritardo di canale del passo 4.2, che toglie un ulteriore 13% agendo su errori di formazione vecchi di 100 ms. Il passo 5.1 non la cambia: la CI agisce sulla stima, non sul controllo.

La correzione è **una riga**: propagare $V_{rif}$ a tutta la flotta in feedforward, così che $\sum_i V_{rif,i} = N v_{cruise}$ e $v_f = v_{cruise}$. È esattamente ciò che fa la Fase 2, dove infatti il fenomeno non si presenta. L'alternativa strutturale è il consenso del secondo ordine, in cui ogni agente ha una propria posizione desiderata e il Master smette di essere un punto singolo di guasto.


## 4. Figure Prodotte

L'esecuzione di `main5.m` genera la cartella `fase_5/risultati/` con otto figure, con la stessa nomenclatura delle Fasi 3 e 4 per consentire il confronto diretto fra le configurazioni.

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
La legge di consenso è pesata dalla matrice di **adiacenza** del grafo. Il raggio di comunicazione coincide con `r_collab` = **55 m**, la portata del ranging inter-veicolare: è la stessa radio UWB a fornire sia la misura di distanza sia il canale dati, quindi non avrebbe senso che il consenso raggiungesse un vicino con cui il ranging è impossibile. La riduzione da 120 a 55 m rispetto alla Fase 3 è motivata in [fase_4/README4.md §1.1](../fase_4/README4.md).

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
