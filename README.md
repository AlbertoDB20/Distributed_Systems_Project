# Progetto di Intelligient Distributed Systems
## Localizzazione Collaborativa e Controllo di una Flotta di Snow Groomer Off-road in Ambienti Ostili

**Corso:** Intelligent Distributed Systems  
**Obiettivo del Progetto:** Sviluppo, simulazione e validazione di un'architettura decentralizzata per la stima della posa e il controllo di formazione di una flotta di $N$ veicoli terrestri. Il sistema deve operare in modo resiliente in presenza di terreni a bassa aderenza (slittamenti dinamici), zone di negazione del segnale GPS (GPS-denied) e vincoli di comunicazione tipici delle reti reali.

---

## 0. Struttura del Repository

```
README.md                    questo documento: architettura, notazione, roadmap
common/                      funzioni condivise fra le fasi e script di validazione
  calcola_Q_cwna.m             rumore di processo in forma CWNA (§2.5)
  costruisci_grafo.m           adiacenza, grado, Laplaciano, lambda_2 (§3.1)
  pesi_metropolis.m            matrice di consenso doppiamente stocastica (§3.1)
  consenso_dwls.m              minimi quadrati pesati distribuiti (§2.7)
  verifica_Q_cwna.m            validazione della Q contro il metodo di Van Loan
  verifica_grafo.m             validazione delle proprieta' spettrali del grafo
  verifica_dwls.m              validazione del D-WLS contro il WLS centralizzato
  verifica_consistenza.m       campagna Monte Carlo con test NEES sulla Fase 2
theory/                      note teoriche di approfondimento
  TEORIA_rumore_di_processo.md      ruolo di Q, modello CWNA, canale laterale
  TEORIA_campi_potenziali.md        Khatib, funzione FIRAS, limiti del metodo
  TEORIA_osservabilita_e_filtro.md  osservabilita', scelta EKF contro UKF
  TEORIA_consenso_su_grafi.md       consenso lineare, Laplaciano, pesi Metropolis
  TEORIA_stima_distribuita.md       D-WLS, coppia informativa, ruolo di n
  TEORIA_ciclo_temporale.md         scansione dei tempi dentro un passo T_s
fase_1/  fase_2/  fase_3/    una cartella per fase, ciascuna con:
  mainN.m                      script di simulazione
  READMEN.md                   documentazione della fase
  risultati/                   figure generate dallo script
exam/                        regole d'esame e template del report
```

Tutti gli script ancorano i propri percorsi alla posizione del file tramite
`mfilename('fullpath')`, e funzionano quindi indipendentemente dalla directory
di lavoro corrente.

---

## 1. Modellazione del Sistema e Variabili di Stato

Il cuore matematico del progetto si basa su una modellazione accurata del singolo agente. Per poter gestire in modo coerente le letture dei sensori propriocettivi (odometria, IMU) ed esterocettivi (GPS, UWB), si è scelto di adottare un modello cinematico di tipo **Uniciclo**, estendendo però il vettore di stato per includere le derivate prime.

Il vettore di stato per l'i-esimo veicolo è definito come:
$$x_i = [x_i, y_i, \theta_i, v_i, \omega_i]^T$$

L'inclusione della velocità lineare $v_i$ e della velocità angolare $\omega_i$ all'interno dello stato non è un vezzo matematico, ma una necessità ingegneristica. Operando su terreni fangosi o nevosi, i comandi di trazione inviati ai motori non corrispondono alla velocità effettiva del baricentro a causa dei continui slittamenti. Stimando le velocità direttamente nel filtro, è possibile svincolarsi parzialmente dall'errore odometrico. 

Inoltre, lo slittamento non viene modellato come un semplice rumore bianco costante. Viene introdotto un approccio stocastico dinamico: la matrice di covarianza del rumore di processo $Q$ viene aggiornata in tempo reale in funzione della velocità del veicolo e di un fattore caratteristico del terreno. Più il veicolo accelera su un terreno avverso, maggiore sarà l'incertezza iniettata nel modello di predizione, rendendo il filtro intrinsecamente più conservativo.

### 1.1 Parametri Fisici di Riferimento
I parametri sono tarati su un mezzo battipista di classe reale (PistenBully 600, Prinoth Bison): ingombro di circa 5 m sui cingoli e 9 m con fresa e lama.

| Grandezza | Simbolo | Valore | Note |
|---|---|---|---|
| Raggio ruota motrice | $r$ | 0.5 m | |
| Carreggiata (interasse cingoli) | $L$ | 3.5 m | |
| Punto di controllo (feedback lin.) | $b$ | 2.0 m | $\approx$ semilunghezza del mezzo |
| Velocità massima | $v_{max}$ | 5.0 m/s | $\approx$ 18 km/h, trasferimento |
| Velocità di lavoro | $v_{cruise}$ | 2.5 m/s | $\approx$ 9 km/h, battitura |
| Velocità angolare massima | $\omega_{max}$ | 0.6 rad/s | raggio di sterzata minimo $\approx$ 4 m |
| Frequenza di campionamento | $f_s$ | 10 Hz | uniforme fino alla Fase 4 |
| Distanze inter-veicolari nominali | $d_{ij}$ | 36-40 m | formazione a "V" |
| Soglia di sicurezza | $d_{safe}$ | 15 m | ingombro fisico + margine |
| Raggio di comunicazione inter-veicolare | $r_{collab}$ | 120 m | link UWB in vista ottica |
| Raggio di visibilità ancora UWB | $r_{ancora}$ | 150 m | portata DW1000 derata (vedi §2.2) |
| Numero di ancore UWB | $n_{ancore}$ | 5 | posizionate via ottimizzazione GDOP |
| $\sigma$ GPS RTK / standard | | 0.2 / 2.0 m | Master / Slave |
| $\sigma$ magnetometro / giroscopio | | 0.05 rad / 0.02 rad/s | |
| $\sigma$ encoder | | 0.1 rad/s | |
| $\sigma$ ranging UWB (ancora fissa / veicolo) | | 0.5 / 0.6 m | vedi §2.2 |

**La scala della formazione non è una scelta estetica.** Le zone GPS-denied hanno raggio 65 m: con distanze inter-veicolari di pochi metri l'intera flotta condivide sempre la stessa condizione di copertura, e lo scenario centrale del progetto — un veicolo cieco ancorato da un vicino che vede ancora i satelliti — non si verifica mai. Portando la formazione alla scala reale dei mezzi la condizione di copertura mista passa dal 5.4% al 23.3% del tempo di missione. Analogamente, i guadagni di controllo non sono trasferibili fra scale diverse: il gradiente del potenziale repulsivo scala come $1/d^3$, quindi $k_{rep}$ viene ricavato per inversione da un requisito di progetto anziché fissato a un numero.

---

## 2. Architettura di Stima e Infrastruttura

### 2.1 Extended Kalman Filter (EKF) e Osservabilità
La stima dello stato locale di ogni veicolo è affidata a un **Extended Kalman Filter**. La scelta rispetto a varianti derivative-free come l'UKF riposa su cinque argomenti, il primo dei quali è quantitativo e decisivo.

1. **La non linearità è debole sulla scala dell'incertezza.** È questo il criterio che conta: non quanto una funzione curva in assoluto, ma quanto curva entro $\pm 3\sigma$. Nel modello di moto l'errore di linearizzazione vale $\tfrac{1}{2}vT_s\sigma_\theta^2 \approx 8\cdot10^{-6}$ m per passo, contro un rumore di processo di $4.5\cdot10^{-2}$ m: **tre ordini di grandezza sotto**. Per le misure di distanza UWB il rapporto è analogo — $4\cdot10^{-4}$ m contro $\sigma_{uwb} = 0.5$ m — perché il veicolo opera a decine o centinaia di metri dalle ancore con incertezza decimetrica. EKF e UKF darebbero qui risultati indistinguibili, e l'UKF pagherebbe 11 propagazioni più una fattorizzazione di Cholesky per passo.
2. **Gli Jacobiani servono comunque altrove:** per il Gramiano di osservabilità e per il calcolo del peso $\gamma$ nella Covariance Intersection.
3. **Costo computazionale in un contesto distribuito.** L'architettura deve scalare a $N$ agenti e alle frequenze reali della Fase 4, dove un'IMU può operare a 100–200 Hz. Il rapporto di costo fra le due soluzioni è di circa 5–10×.
4. **Misure a dimensione variabile.** La matrice $C$ cambia dimensione a runtime — GPS disponibile o negato, numero variabile di ancore e di vicini in portata. Con l'EKF si accodano righe; con l'UKF andrebbe rieseguita la trasformata unscented sulla funzione di misura a ogni cambio di configurazione.
5. **Nessun parametro di taratura.** L'UKF richiede di scegliere e giustificare $\alpha$, $\beta$, $\kappa$; l'EKF non ha manopole.

> **La scelta è condizionata, non assoluta.** L'argomento 1 dipende da $\sigma_\theta$, e l'errore di linearizzazione cresce **quadraticamente** con esso: se il modello di slittamento della Fase 5 degradasse sensibilmente la stima di heading, il conto va rifatto.

**Osservabilità.** Indipendentemente dalla scelta del filtro, gli Jacobiani calcolati a ogni istante di campionamento permettono di valutare il rango della matrice di osservabilità $\mathcal{O}$, e quindi di dimostrare matematicamente come la perdita del GPS la degradi — portando alla deriva della posa assoluta — e come la fusione con le misure UWB permetta di recuperarla.

> La trattazione teorica completa — definizione e criteri di osservabilità, analisi dei quattro regimi attraversati dal sistema, non osservabilità collettiva della flotta, legame fra GDOP e grado di osservabilità, e giustificazione quantitativa della scelta EKF contro UKF — è in [theory/TEORIA_osservabilita_e_filtro.md](theory/TEORIA_osservabilita_e_filtro.md).

### 2.2 Infrastruttura UWB per Ambienti GPS-Denied
Nella realtà operativa, il segnale GPS è soggetto ad attenuazioni e multipath, specialmente in ambienti forestali, urbani o indoor. Per ovviare a questo problema in modo economicamente sostenibile, il progetto prevede l'installazione di moduli Ultra-Wideband (UWB). Questa tecnologia, caratterizzata da un'accuratezza centimetrica, viene utilizzata per due scopi:
1.  **Ranging con ancore fisse:** Misurare la distanza da alcune antenne UWB dislocate strategicamente sulla mappa, specialmente in prossimità delle zone cieche per il GPS.
2.  **Ranging inter-veicolare:** Misurare la distanza relativa tra i membri della flotta.

Il posizionamento delle $n_{ancore} = 5$ ancore fisse non è casuale: viene ottimizzato minimizzando la Geometric Dilution of Precision (GDOP) media sui tratti di percorso che ricadono nelle zone cieche. Vale la pena notare che la matrice geometrica da cui si calcola la GDOP **è la stessa matrice $C$** delle righe di ranging UWB nell'EKF: la GDOP non è un criterio esterno, è una misura del condizionamento del problema di stima che il filtro dovrà risolvere in quel punto.

**Taratura su hardware reale.** I parametri UWB sono riferiti a dispositivi effettivamente in commercio e compatibili con l'impiego outdoor veicolare:

| Dispositivo | Portata dichiarata | Accuratezza ranging | Fascia |
|---|---|---|---|
| Qorvo (ex Decawave) DW1000 / DWM1001C | fino a 290 m @ 110 kbps, 10% PER, LOS | ~10 cm in condizioni favorevoli | economica |
| Humatics (ex Time Domain) PulsON P440 | oltre 600 m, 3.1–4.8 GHz, TW-TOF | ~2 cm | professionale, all-weather |

Si adotta $r_{ancora} = 150$ m: la portata nominale del DW1000 derata di circa un fattore 2 per attenuazione da precipitazione nevosa, ostruzioni parziali del terreno (NLOS) e margine sul Packet Error Rate. Le ancore si assumono montate su palo (3–4 m), poiché la letteratura sperimentale mostra che l'errore di ranging cresce marcatamente al ridursi dell'altezza d'antenna.

I due valori di rumore di ranging riflettono questa asimmetria: $\sigma_{uwb} = 0.5$ m verso ancora fissa (posizione rilevata una volta per tutte, antenna elevata, buona vista ottica) contro $\sigma_{collab} = 0.6$ m fra veicoli. La differenza **non** è dovuta al moto — lo spostamento durante uno scambio TW-TOF di circa 1 ms a 2.5 m/s vale millimetri — ma all'antenna più bassa montata sul mezzo e all'effetto della piattaforma metallica, che nel link inter-veicolare è presente su *entrambi* i terminali anziché su uno solo.

### 2.3 Fusione Decentralizzata e Covariance Intersection
Essendo un progetto di Sistemi Distribuiti, non esiste un'unità di calcolo centrale. I veicoli comunicano tra loro scambiandosi le proprie stime di posa. Se un veicolo perde il GPS ma un suo vicino lo mantiene, il sistema sfrutta la distanza UWB e lo scambio dati per correggere la traiettoria del veicolo "cieco".
Tuttavia, lo scambio continuo di stime in una rete chiusa genera il fenomeno del *Data Rumination*: le informazioni diventano circolari e i filtri di Kalman iniziano a sottostimare la propria covarianza (diventano troppo "ottimisti"). La soluzione prevista è la **Covariance Intersection (CI)**, che garantisce stime statisticamente consistenti anche in presenza di correlazioni ignote fra gli agenti: fonde due stime pesandone le inverse delle covarianze con $\gamma$ e $1-\gamma$, e per *qualunque* correlazione incrociata restituisce un maggiorante della vera covarianza d'errore.

> **Stato di implementazione.** La CI **non è ancora implementata**, ed è programmata per la **Fase 5** (§4). Allo stato attuale i veicoli si scambiano la sola posizione stimata, e la $R$ della misura collaborativa contiene unicamente il rumore del sensore: il filtro ignora l'incertezza $\Sigma_j$ del vicino e risulta quindi **ottimista**. È un limite noto e documentato, non un'omissione — vedi [fase_3/README3.md §3.3](fase_3/README3.md) per la quantificazione e [theory/TEORIA_osservabilita_e_filtro.md §1.4](theory/TEORIA_osservabilita_e_filtro.md), Caso E, per il legame con l'*observability mismatch*.

Va notato che la CI **classica non si applica direttamente** a questo caso: fonde due stime della *stessa* grandezza, mentre il vicino trasmette una stima del *proprio* stato, e da una misura di sola distanza non si ricava una stima puntuale della propria posizione ma una circonferenza. La formulazione corretta è quella di Carrillo-Arce et al. (IROS 2013): l'incertezza del vicino entra come rumore di misura aggiuntivo, $R_{eff} = \sigma_{collab}^2 + u^T\Sigma_j^{(1:2,1:2)}u$, e poiché quel rumore è correlato in modo ignoto con il proprio prior, l'aggiornamento va eseguito in forma CI anziché con il guadagno di Kalman standard.

### 2.4 Architettura del Ciclo di Simulazione e Convenzione Temporale
In un simulatore closed-loop, stima e controllo si inseguono a vicenda: è quindi necessario fissare in modo non ambiguo a quale istante si riferisce ogni grandezza, e garantire che nessun blocco utilizzi informazione non ancora disponibile. Detto $t_k = (k-1)T_s$, si adotta la convenzione:

| Simbolo | Significato |
|---|---|
| $x_k^{true}$ | stato reale all'istante $t_k$ |
| $\hat{x}_k$ | stima **a posteriori** a $t_k$, condizionata a tutte le misure fino a $t_k$ incluso |
| $z_k$ | misura **acquisita** a $t_k$, quindi funzione di $x_k^{true}$ |
| $u_k$ | comando **applicato** nell'intervallo $[t_k, t_{k+1})$, calcolato dalla sola $\hat{x}_k$ |

Ogni iterazione esegue cinque blocchi in quest'ordine:

1. **Broadcast** — ogni agente pubblica la propria stima $\hat{x}_k$ sulla rete.
2. **Controllo** — $u_k = g(\hat{x}_k, \{\hat{x}_k^{(j)}\}_{j \in \mathcal{N}_i})$.
3. **Impianto** — $x_{k+1}^{true} = f(x_k^{true}, u_k)$, per **tutti** gli agenti.
4. **Sensori** — $z_{k+1} = h(x_{k+1}^{true}) + \nu_{k+1}$.
5. **Stima** — predizione da $\hat{x}_k$ e correzione con $z_{k+1}$, ottenendo $\hat{x}_{k+1}$.

Due vincoli strutturali giustificano quest'ordine:

* **Causalità del controllo.** Il comando che agisce durante $[t_k, t_{k+1})$ può dipendere solo da $\hat{x}_k$. Usare $\hat{x}_{k+1}$ significherebbe retroazionare informazione futura.
* **Coerenza temporale dell'innovazione.** L'innovazione $z_{k+1} - h(\hat{x}_{k+1|k})$ è statisticamente significativa solo se misura e predizione si riferiscono al **medesimo** istante. Correggere la predizione a $t_{k+1}$ con una misura acquisita a $t_k$ introduce un bias sistematico dell'ordine di $|v|T_s$ sulla posizione e $|\omega|T_s$ sull'heading — un errore *deterministico*, che per costruzione non è rappresentato nella matrice di covarianza $\Sigma$ e che quindi invaliderebbe l'analisi di consistenza (3-sigma bounds, NEES) della Fase 6.

I blocchi 2-3 e 4-5 sono implementati come due passate distinte sull'intera flotta: la ground truth di tutti gli agenti deve esistere a $t_{k+1}$ prima che uno qualsiasi generi le proprie misure, poiché dalla Fase 3 in avanti le misure di range inter-veicolare dipendono dalla posizione reale dei vicini.

> **Schema temporale completo** — che cosa accade dentro un intervallo $T_s$, le due scale dei tempi (controllo a 10 Hz, radio a ~1 kHz), la mappa fra iterazione del codice e istante fisico, e il budget degli scambi radio: [theory/TEORIA_ciclo_temporale.md](theory/TEORIA_ciclo_temporale.md).

**Ritardo di un passo sulle stime scambiate.** La posizione di un vicino usata come ancora mobile è la sua stima $\hat{x}_k^{(j)}$ — l'ultimo pacchetto ricevuto — propagata di un passo con il modello di moto, ottenendo $\hat{x}_{k+1|k}^{(j)}$. La scelta risponde a due esigenze: rompe il loop algebrico fra filtri che altrimenti dipenderebbero l'uno dalla stima aggiornata dell'altro nel medesimo istante, e rispecchia ciò che un canale di comunicazione reale rende effettivamente disponibile. La propagazione è necessaria perché confrontare una misura acquisita a $t_{k+1}$ con una posizione riferita a $t_k$ reintrodurrebbe lo stesso bias di $|v_j| T_s$ descritto sopra.

### 2.5 Rumore di Processo: formulazione CWNA

La matrice $Q$ non è una diagonale costante ma viene ricostruita a ogni passo di predizione secondo il modello **CWNA** (*Continuous White Noise Acceleration*). Due ragioni, entrambe sostanziali.

**Struttura fisica.** Nel modello uniciclo la posizione non possiede dinamica propria: cambia soltanto perché $v$ e $\theta$ sono incerti. Una $Q$ diagonale inietta rumore direttamente su $x$ e $y$, cioè afferma che il veicolo si sposta lateralmente anche da fermo. Nel modello CWNA il rumore entra sulle **accelerazioni** — dove agisce fisicamente lo slittamento — e si propaga alla posizione attraverso il modello:

$$\dot x = f(x) + \Gamma\, w(t), \qquad w \sim \mathcal{N}(0, Q_c), \qquad Q_d = \int_0^{T_s}\! e^{A\tau}\,\Gamma Q_c\Gamma^T e^{A^T\tau}\,d\tau$$

Congelando $\theta$ sull'intervallo di campionamento — lecito, poiché con $T_s = 0.1$ s e $\omega \le 0.6$ rad/s l'angolo varia al più di 3.4° — l'integrale si risolve in forma chiusa e restituisce i blocchi canonici del modello a velocità costante, $q\begin{bmatrix} T_s^3/3 & T_s^2/2 \\ T_s^2/2 & T_s\end{bmatrix}$ (Bar-Shalom et al., cap. 6), proiettati sulla direzione di marcia. Il risultato **non è diagonale**: contiene le correlazioni posizione–velocità, che sono informazione fisica ("se sovrastimo la velocità, siamo anche troppo avanti") che una diagonale scarta.

**Scalatura su $T_s$.** $Q_d$ è proporzionale a $T_s$. La formulazione precedente sommava una costante a ogni passo, quindi cambiare la frequenza di campionamento ri-tarava silenziosamente il filtro. È il prerequisito per il multi-rate della Fase 4.

Tre canali di rumore, ciascuno con significato fisico:

| Parametro | Unità | Significato | Effetto dopo 1 s di sola predizione |
|---|---|---|---|
| $q_a = 0.10$ | m²/s³ | accelerazione longitudinale — slittamento in trazione | $\sigma_v = 0.32$ m/s |
| $q_\alpha = 0.01$ | rad²/s³ | accelerazione angolare — slittamento in sterzata | $\sigma_\omega = 0.10$ rad/s |
| $q_{lat} = 0.02$ | m²/s | deriva laterale su pendio innevato | 0.14 m di scarto laterale |
| $k_{terreno} = 0$ | 1/s | $Q$ adattiva: $q_a \leftarrow q_a + k_{terreno}v^2$ | inattiva fino alla Fase 5 |

Il canale laterale non è un termine di comodo: il modello uniciclo è anolonomo e **non può** rappresentare la traslazione laterale, quindi senza di esso $Q_d$ risulta **singolare** (rango 4 su 5) e l'incertezza perpendicolare alla direzione di marcia non cresce mai, per quanto a lungo il filtro resti privo di misure assolute. Rappresenta esattamente ciò che il modello non sa descrivere, ed è il primo candidato a essere sostituito da un modello esplicito in Fase 5.

**Validazione.** La forma chiusa è verificata numericamente contro la discretizzazione esatta ottenuta con il metodo di **Van Loan** (`common/verifica_Q_cwna.m`): a $T_s = 0.1$ s l'errore relativo è di $4\cdot10^{-4}$, e cresce come $T_s^2$ (a $T_s = 1$ s raggiunge l'11%). Se in Fase 4 un ramo di predizione dovesse operare a passi molto più lunghi, converrà passare direttamente a Van Loan.

**Effetto misurato** (Fase 2, 15 run Monte Carlo, regime):

| | NEES | RMSE posizione | RMSE $v$ | RMSE $\theta$ |
|---|---|---|---|---|
| $Q$ diagonale costante | 3.95 | 0.376 m | 0.0350 m/s | 0.0420 rad |
| $Q$ in forma CWNA | 4.03 | **0.200 m** | 0.0318 m/s | **0.0082 rad** |

La **consistenza non cambia** — entrambe le formulazioni sono ugualmente calibrate, conservative di circa 1.25× rispetto al valore atteso $E[\text{NEES}] = n = 5$ — mentre l'**accuratezza migliora di 1.9× in posizione e 5.1× in heading**. È il risultato atteso: il NEES misura il *rapporto* fra errore reale e covarianza dichiarata, e una $Q$ sovradimensionata gonfia entrambi lasciando il rapporto invariato. Ciò che il CWNA corregge non è la calibrazione ma la quantità di informazione che il modello di processo mette a disposizione del filtro.

> **Avvertenza di taratura, da dichiarare nel report.** La ground truth attuale ha rumore di processo *nullo* (tracking ideale degli attuatori). I valori di $q_a$ e $q_\alpha$ sono quindi deliberatamente conservativi rispetto all'impianto simulato — il filtro risulta pessimista, non ottimista, che è la condizione sicura — e parte del guadagno di accuratezza misurato deriva dal fatto che una $Q$ più piccola è più vicina a un impianto che di rumore non ne ha affatto. La calibrazione onesta sarà possibile solo in Fase 5, contro uno slittamento realmente simulato. I benefici *strutturali* — scalatura su $T_s$, correlazioni posizione-velocità, non singolarità, parametrizzazione fisica — sono invece incondizionati.

### 2.6 Notazione Adottata

Si adotta integralmente la notazione di Thrun, Burgard e Fox, *Probabilistic Robotics*, mantenuta identica in tutte le fasi del progetto e nel codice MATLAB.

| Simbolo | Significato | Dimensione | Nome nel codice |
|---|---|---|---|
| $A_t$ | Jacobiano del modello di moto, $\partial f/\partial x$ | 5×5 | `A_k` |
| $C_t$ | Jacobiano del modello di misura, $\partial h/\partial x$ | m×5 | `C_k` |
| $Q_t$ | Covarianza del rumore di **processo** | 5×5 | `Q` |
| $R_t$ | Covarianza del rumore di **misura** | m×m | `R`, `R_k` |
| $\Sigma_t$ | Covarianza della **stima** | 5×5 | `Sigma` |
| $\bar\Sigma_t$ | Covarianza **predetta** (a priori) | 5×5 | `Sigma_bar` |
| $K_t$ | Guadagno di Kalman | 5×m | `K` |
| $\mu_t$ | Stima a posteriori | 5×1 | `x_est` |
| $\bar\mu_t$ | Stima a priori (predetta) | 5×1 | `x_pred` |

A queste si aggiunge $S_t = C_t\bar\Sigma_t C_t^T + R_t$, la **covarianza dell'innovazione**, che nel testo compare solo come parentesi interna dell'espressione di $K_t$ e che nel codice è resa esplicita come `S` per chiarezza.

**Assenza del termine $B_t u_t$.** Il modello di riferimento include un ingresso di controllo nella predizione, $\bar\mu_t = A_t\mu_{t-1} + B_t u_t$. Nella nostra formulazione **questo termine non compare**, ed è una scelta deliberata: $v$ e $\omega$ non sono ingressi ma **stati**, osservati dagli encoder, e la predizione è un random walk su di essi. La ragione è fisica — su terreno a bassa aderenza il comando inviato ai motori *non coincide* con la velocità effettiva del baricentro, quindi usarlo come ingresso deterministico introdurrebbe una correlazione fra rumore di processo e ingresso, violando le ipotesi di indipendenza del filtro. È la stessa ragione che motiva l'estensione del vettore di stato descritta in §1.

**Attenzione a una convenzione non universale:** in questo testo $Q$ è il rumore di processo e $R$ quello di misura. Parte della letteratura di controllo adotta la convenzione opposta.

### 2.7 Stima Distribuita di un Parametro Costante (D-WLS)

Accanto alla stima della **posa**, che è dinamica e locale a ciascun mezzo, la flotta risolve un secondo problema di natura diversa: identificare una proprietà del **terreno**, uguale per tutti e costante nel tempo. È il caso in cui il Cap. 18 prescrive i **Minimi Quadrati Pesati Distribuiti** (D-WLS); il Filtro di Kalman Distribuito servirebbe se la grandezza avesse una dinamica propria.

Il modello è la resistenza specifica al moto di un cingolato su neve:

$$\frac{F_{traz}}{W} = \mu_{terr} + c_{terr}\,v^2 + \varepsilon \qquad\Longrightarrow\qquad x_{terr} = \begin{bmatrix}\mu_{terr}\\ c_{terr}\end{bmatrix}, \quad C_i = \begin{bmatrix}1 & v_i^2\end{bmatrix}$$

Lo sforzo specifico è misurato da un **torsiometro** sull'albero di trasmissione, normalizzando sul peso del mezzo. È una **lettura** del terreno e non un'azione su di esso, quindi l'impianto simulato resta invariato; il modello di slittamento della Fase 5 userà questi parametri per generare la dinamica, qui vengono soltanto identificati. La covarianza del sensore segue la convenzione degli altri: `R_traz_master` = $0.010^2$ e `R_traz_slave` = $0.025^2$, con la stessa asimmetria adottata per il GPS. Serve a rendere effettiva la pesatura del WLS: con $R$ uguali per tutti il termine $R^{-1}$ diventa uno scalare comune e sparisce dalla soluzione.

**Il consenso vive sulla scala dei tempi della radio, non del controllo.** Uno scambio TW-TOF su DW1000 dura circa 1 ms contro i 100 ms del passo di campionamento: fra due istanti di controllo il canale sostiene decine di cicli di consenso. Un round completo di D-WLS viene quindi eseguito **a ogni passo**, con metà del passo riservata a questo traffico e l'altra metà a ranging e broadcast delle pose, per un budget di $q_{max} = 50$ cicli. La stima del terreno è così disponibile aggiornata a 10 Hz, alla stessa cadenza della stima di posa.

**Il numero di cicli non è una costante scritta a mano.** Viene dimensionato dal raggio spettrale essenziale, $q \ge \log\epsilon/\log\rho_2$, imponendo almeno il **diametro** del grafo — sotto quella soglia l'informazione non ha materialmente attraversato la rete, e due nodi a distanza 3 non sanno nulla l'uno dell'altro — e poi troncato al budget radio. Su $K_3$ il dimensionamento restituisce $q = 1$ da solo, perché $\rho_2 = 0$; su topologia frammentata cresce automaticamente, e un apposito indicatore segnala quando il canale non basta più anziché produrre in silenzio un numero sbagliato. È il calcolo del **progettista**: $\rho_2$ e il diametro sono proprietà globali del grafo, come $\lambda_2$ e $L$.

**Struttura dell'algoritmo.** Poiché i rumori dei diversi veicoli sono scorrelati, la matrice $R$ globale è diagonale a blocchi e la soluzione WLS centralizzata si decompone in somme di contributi puramente locali. Ogni veicolo costruisce la propria **coppia informativa**

$$F_i = C_i^T R_i^{-1} C_i, \qquad a_i = C_i^T R_i^{-1} z_i$$

la media con i vicini tramite il consenso a pesi di Metropolis, e ricostruisce $\hat x_i = F_i^{-1}a_i$. È qui che la matrice $Q$ costruita al §3.1 **entra in un algoritmo** anziché servire da sola diagnostica: la doppia stocasticità è la condizione perché la media coincida con la somma globale divisa per $n$, e quindi perché la stima ricostruita non risulti polarizzata.

**Nessun veicolo può risolvere il problema da solo.** I parametri sono due e ogni mezzo produce una misura scalare per passo, quindi la sua $F_i$ è un prodotto esterno di **rango 1 su 2**: il problema locale è singolare, non semplicemente impreciso. Diventa risolvibile solo unendo misure prese a velocità diverse, fornite dalla formazione — su percorso curvo le ali della V percorrono archi di raggio diverso dal Master — e dall'accumulo lungo la missione. È lo stesso meccanismo della GDOP (§2.2): conta la diversità geometrica delle misure, non il loro numero.

**Il fattore $1/n$ si cancella nella stima, non nella covarianza.** A convergenza il nodo possiede $F_i(q) = \frac{1}{n}\sum_l F_l$, e nel rapporto $F_i^{-1}a_i$ la normalizzazione sparisce: il D-WLS **non richiede di conoscere la cardinalità della rete**, a differenza del DKF. Per dichiarare l'incertezza serve però $P = (n F_i(q))^{-1}$, dove il fattore non si semplifica.

**Nessun doppio conteggio, malgrado i cicli del grafo.** Le colonne di una matrice doppiamente stocastica sommano a uno, quindi $\sum_i F_i(k)$ è un **invariante** del consenso: l'informazione viene ridistribuita, mai duplicata. È la differenza strutturale rispetto alla fusione delle pose, dove la stima è ricorsiva e rientra nel proprio filtro — ed è la ragione per cui il D-WLS non anticipa in alcun modo la Covariance Intersection di §2.3. I due canali restano distinti: il pacchetto D-WLS trasporta la coppia informativa del **parametro di terreno**, non la covarianza della posa $\Sigma_i$, che continua a non essere condivisa fino alla Fase 5.

| Proprietà verificata | Risultato |
|---|---|
| Rango di $F_i(0)$ da una singola misura | 1 su 2 — il nodo isolato non può risolvere |
| Autovalore $\lambda_1 = 1$  | $\lambda_1 = 1$ garantisce lo STATO di EQUILIBRIO, quindi il grafo, dopo q-cicli di iterazioni, raggiungierà certamente uno stato di consenso. |
| Autovalore $\lambda_2 = 0$  | $\lambda_2 $ dà informazioni sulla velocità della rete e sulla presenza di bottleneck. $\lambda_2 = 0 $ significa che il grafo è All-to-All, con velocità massima: il consenso lo raggiunge in un ciclo solo (q=1). |
| D-WLS contro WLS centralizzato | scarto $1.5\cdot10^{-15} $(quindi le due soluzioni coincidino) |
| Guadagno su $\sigma(c_{terr})$ per gli Slave | **8.1×** e **8.3×** |

**Un solo ciclo di consenso è sufficiente**, perché su $K_3$ la regola di Metropolis dà $\rho_2 = 0$: il risultato spettrale del §3.1 si traduce in un costo di comunicazione di un singolo scambio per round, cioè il 2% del budget disponibile. Il guadagno della cooperazione non è uniforme — il Master, che ha sia il sensore migliore sia la maggiore escursione di velocità, da solo arriverebbe quasi dove arriva la rete. Ciò che la rete produce è la **distribuzione a tutti della qualità del membro meglio strumentato**, la stessa struttura del GPS RTK montato sul solo Master.

> Trattazione completa — derivazione passo per passo secondo il Cap. 18, ruolo della doppia stocasticità, connettività congiunta su topologia tempo-variante, limiti noti (regressore incerto, eccitazione insufficiente, assenza di dimenticanza): [theory/TEORIA_stima_distribuita.md](theory/TEORIA_stima_distribuita.md). Risultati e figura: [fase_3/README3.md §4.6](fase_3/README3.md).

---

## 3. Controllo di Formazione (Consenso)

La flotta, composta inizialmente da $N=3$ veicoli, deve navigare lungo la mappa mantenendo una specifica formazione (ad esempio, muovendosi affiancati). Invece di adottare un approccio centralizzato, si utilizza un protocollo basato sul **Consenso lineare su grafi**.

### 3.1 Formalizzazione algebrica

La rete di comunicazione è modellata come grafo $\mathcal{G} = (\mathcal{N},\mathcal{E})$, con matrice di **adiacenza** $A$, matrice di **grado** $D$ e **Laplaciano** $L = D - A$. La legge di controllo per l'$i$-esimo veicolo è

$$u_i = -K_{cons}\sum_{j} a_{ij}\Big[(p_i - p_j) - \Delta_{ij}\Big]$$

Introducendo la variabile traslata $\tilde p_i = p_i - p_i^{des}$, l'errore di formazione diventa $\tilde p_i - \tilde p_j$ e la legge si riscrive in forma matriciale come

$$u = -K_{cons}\,(L \otimes I_2)\,\tilde p$$

cioè **esattamente il protocollo di consenso lineare**. Il mantenimento della formazione non è quindi un problema distinto dal consenso: è consenso su coordinate traslate. Ne segue che l'intera teoria del Cap. 17 si applica senza adattamenti — condizione di spanning tree, connettività algebrica, costante di tempo.

| Grandezza | Significato | Valore nel progetto |
|---|---|---|
| $\lambda_2(L)$ | connettività algebrica: $>0$ ⟺ grafo connesso, e ne quantifica il grado | 3.0000 ($K_3$ completo) |
| $\rho_2(Q)$ | essential spectral radius: fattore di convergenza asintotico | 0.0000 |
| $\tau = 1/(K_{cons}\lambda_2)$ | costante di tempo dell'errore di formazione | 2.22 s |

I pesi della matrice di consenso $Q$ sono costruiti con la regola di **Metropolis-Hastings**, $q_{ij} = 1/(\max(d_i,d_j)+1)$, che ogni nodo calcola conoscendo soltanto il proprio grado e quello dei vicini diretti — nessuna conoscenza della topologia globale. La regola produce una $Q$ simmetrica e quindi **doppiamente stocastica**, condizione necessaria perché il consenso converga alla media aritmetica esatta (*average consensus*) anziché a una combinazione pesata arbitraria.

**Risultato notevole.** Con grafo completo la regola di Metropolis dà $q_{ij} = 1/n$ per ogni coppia, quindi $Q = \frac{1}{n}\mathbf{1}\mathbf{1}^T$ e $\rho_2 = 0$: il consenso medio converge in **un solo passo**. Con $N=3$ in rete full-mesh gli algoritmi di stima distribuita del Cap. 18 risulterebbero quindi esatti già con una sola iterazione di consenso.

**Adiacenza nel controllo, pesi stocastici nella stima.** I coefficienti $a_{ij}$ e $q_{ij}$ non sono tarature alternative dello stesso peso, ma appartengono a due formulazioni distinte: $a_{ij}$ è un **indicatore di accoppiamento**, $q_{ij}$ un **peso di sostituzione**. La legge di controllo genera una velocità a partire da una differenza, e per una differenza l'invarianza $L\mathbf{1} = 0$ vale automaticamente per qualunque peso non negativo: non esiste alcun $a_{ii}$, non c'è nulla da normalizzare, e la stabilità è garantita da $K_{cons} > 0$ su grafo connesso. Sostituire $q_{ij}$ ad $a_{ij}$ significherebbe usare il solo blocco fuori diagonale di $Q$ — scartando proprio $q_{ii}$, cioè l'elemento che la rende stocastica — e legare il guadagno effettivo alla topologia: su $K_3$ si otterrebbe $L_w = \frac{1}{3}L$, quindi $\tau = 6.67$ s anziché 2.22 s, con un fattore che cambierebbe a ogni collegamento perduto. Metropolis pesa inoltre in base al **grado**, difesa sensata contro il doppio conteggio quando si media informazione, priva di significato su una geometria rigida in cui $\Delta_{ij}$ è un vincolo e non un'opinione. La matrice $Q$ è quindi costruita e monitorata ma **non entra nella legge di controllo**: diventerà operativa nella stima distribuita del Cap. 18, dove l'agente sostituisce davvero la propria coppia informativa con una media dei vicini e la doppia stocasticità è la condizione di correttezza del risultato.

### 3.2 Dalla teoria all'implementazione

Il passaggio dalla forma matriciale al codice comporta una scelta architetturale precisa: **$L$ non viene mai assemblata a bordo dei veicoli**. L'agente $i$ esegue la forma per componenti, sommando i contributi dei soli nodi da cui ha ricevuto un pacchetto, e non ha bisogno di conoscere la topologia della rete. La matrice globale esiste unicamente nel simulatore, come strumento del progettista per l'analisi di stabilità e la previsione dei tempi di risposta. L'equivalenza fra le due forme è verificata numericamente: lo scarto fra il comando calcolato ciclando sui vicini e quello ottenuto da $(L\otimes I_2)\tilde p$ vale $6.7\cdot10^{-16}$ m/s.

Le proprietà spettrali sono validate in `common/verifica_grafo.m`: invarianza $L\mathbf{1} = 0$ a precisione di macchina, $\lambda_2 = n$ sul grafo completo $K_n$, doppia stocasticità della $Q$ di Metropolis, e **predittività di $\rho_2$** — su grafo a catena il tasso di decadimento misurato dell'errore di consenso coincide con quello previsto entro $2.3\cdot10^{-15}$. Su grafo sconnesso entrambi gli indicatori segnalano correttamente la mancata convergenza ($\lambda_2 = 0$, $\rho_2 = 1$).

Le Fasi 2 e 3 registrano a ogni passo $\lambda_2(L)$, $\rho_2(Q)$ e il numero di archi attivi, riportati nelle figure `fase_2/risultati/5_grafo_comunicazione.png` e `fase_3/risultati/7_grafo_comunicazione.png`. La topologia diventa così una grandezza osservabile della simulazione anziché un'ipotesi implicita, e la costante di tempo prevista $\tau = 2.22$ s trova riscontro nel transitorio di formazione di Fase 2.

### 3.3 Vincolo di portata radio

Il grafo è vincolato dal raggio di comunicazione: in Fase 2 il canale è ideale ($R_c = \infty$, grafo completo per costruzione), in Fase 3 $R_c$ coincide con la portata UWB `r_collab` = 120 m, poiché è la stessa radio a fornire sia la misura di distanza sia il canale dati. Le distanze inter-veicolari raggiungono al massimo 41.7 m, il 35% del raggio disponibile: il grafo resta connesso per l'intera missione. Il margine diventa critico nelle fasi successive, quando latenze e perdite di pacchetto renderanno la topologia tempo-variante.

L'adiacenza pesa il **solo** termine di consenso; la repulsione anti-collisione non è pesata dal grafo. La distinzione è priva di conseguenze operative, ed è un requisito di progetto che lo sia: la repulsione richiede la **direzione** della congiungente e non la sola distanza, e la direzione si ricava unicamente dal pacchetto radio — un sensore di ranging da solo non la fornisce. In assenza di collegamento non sarebbe quindi calcolabile in alcun modo. La sicurezza è garantita dalla gerarchia dei raggi d'azione, $d_{safe} = 15$ m $\ll R_c = 120$ m: il canale dati è attivo ben prima che due mezzi entrino in rotta di collisione. La condizione è un invariante di progetto, imposto da un `assert` in fase di inizializzazione in entrambe le fasi.

> Trattazione completa — matrici stocastiche e doppiamente stocastiche, proprietà spettrali del Laplaciano, teorema dello spanning tree, progettazione dei pesi e validazione numerica: [theory/TEORIA_consenso_su_grafi.md](theory/TEORIA_consenso_su_grafi.md).

### 3.4 Evitamento delle collisioni
Al livello di controllo cinematico viene sovrapposto un algoritmo di evitamento collisioni basato sui **Campi Potenziali Artificiali** (Khatib, 1986): se la distanza inter-veicolare scende sotto la soglia $d_{safe}$, viene generata una velocità repulsiva virtuale che devia temporaneamente la traiettoria dei mezzi.

Consenso e repulsione non sono due controllori distinti, ma i due termini di un unico campo potenziale $U = U_{cons} + U_{rep}$, di cui la legge di controllo è l'antigradiente: il consenso è il potenziale **attrattivo**, il cui minimo coincide con la formazione desiderata, e la funzione **FIRAS** di Khatib fornisce quello **repulsivo**. Un requisito di progetto lega i due termini: deve valere $d_{safe} < \min_{i\ne j}\|\Delta_{ij}\|$, altrimenti la repulsione non si annulla mai nella configurazione desiderata e la formazione risulta irraggiungibile (condizione *GNRON*). La trattazione completa — derivazione del gradiente, taratura del guadagno, limiti del metodo e confronto con le Control Barrier Functions — è in [theory/TEORIA_campi_potenziali.md](theory/TEORIA_campi_potenziali.md).

---

## 4. Roadmap di Sviluppo e Validazione

Per garantire la solidità dell'impianto teorico, lo sviluppo in ambiente MATLAB seguirà un paradigma incrementale, partendo da un caso ideale per arrivare a un sistema distribuito quanto più vicino alla realtà.

* **Fase 1: Il Core (Veicolo Singolo Ideale).** Si implementa la dinamica dell'uniciclo e l'EKF per un singolo veicolo. In questa fase, tutti i sensori (GPS, Encoder, IMU) operano alla stessa frequenza di campionamento. Il GPS copre l'intera mappa. L'obiettivo è tarare i parametri di rumore e validare il calcolo degli Jacobiani per l'analisi di osservabilità locale.
* **Fase 2: La Flotta e la Formazione.** Si istanziano $N=3$ veicoli comunicanti attraverso un canale di rete ideale (latenza nulla, zero perdite). Viene integrato il controllo tramite consenso e l'algoritmo per l'evitamento delle collisioni. A scopo puramente analitico e accademico, in questa fase si attribuisce al veicolo "Master" un sensore GPS di qualità superiore, per osservare come la sua stima influenzi il comportamento della formazione rispetto agli "Slave".
* **Fase 3: L'Ambiente Ostile.** Si introduce il realismo ambientale. Vengono create zone *GPS-denied* casuali lungo il percorso e si posizionano le ancore UWB calcolando la GDOP ottimale. Si testa la resilienza della flotta: il sistema deve dimostrare di poter attraversare le zone cieche mantenendo la formazione, affidandosi alla triangolazione UWB e alla localizzazione collaborativa. In questa fase si innesta anche il **D-WLS** per l'identificazione collaborativa del parametro di terreno (§2.7), primo algoritmo del progetto in cui la matrice di consenso di Metropolis entra in funzione anziché servire da diagnostica.
* **Fase 4: Realismo dei Sistemi Distribuiti (Multi-rate e Latenze).** Si rimuovono le assunzioni ideali. I sensori operano ora alle loro frequenze reali (es. IMU molto veloce, GPS lento). Il canale di comunicazione introduce latenze. Viene implementato il *timestamping* delle misurazioni: le letture in ritardo provenienti dagli altri veicoli vengono retroattivate correttamente nel buffer storico dell'EKF per non destabilizzare il filtro. Si uniforma la qualità del GPS per tutti i veicoli, rendendo le zone di perdita del segnale dipendenti esclusivamente dalla posizione spaziale.
* **Fase 5: Realismo Off-Highway (Modellazione dello Slittamento).** Si rimuove l'assunzione di aderenza ideale, finora implicita nel fatto che la velocità reale del veicolo coincide istante per istante con quella comandata. Viene introdotto un modello di slittamento rappresentativo delle condizioni operative di un mezzo cingolato su fondo nevoso, basato su uno studio già disponibile in letteratura sulla trazione di veicoli cingolati su terreni a bassa aderenza, sviluppato presso l'Università degli Studi di Trento. La fase comporta tre interventi sull'architettura:
  1. **Impianto.** La generazione della ground truth passa da `x_true(4,k+1) = v_cmd` a una legge $v^{true} = f_{slip}(v_{cmd}, x^{true}, \text{terreno})$. Il punto di innesto è già predisposto e marcato nel codice di Fase 2 e Fase 3.
  2. **Modello di misura degli encoder.** È la conseguenza più critica. Il modello attuale $z_{enc} = \left[\frac{v}{r}+\frac{L\omega}{2r},\; \frac{v}{r}-\frac{L\omega}{2r}\right]^T$ presuppone rotolamento puro. In presenza di slittamento gli encoder misurano la velocità dei *cingoli*, non quella del veicolo: la relazione fra le due si carica di un errore **sistematico**, non di un rumore bianco. Poiché gli encoder sono l'osservatore primario di $v$ e $\omega$, un modello di misura polarizzato degraderebbe l'intero filtro. Occorre decidere se assorbire lo slittamento gonfiando $R_{enc}$ (soluzione minima ma statisticamente scorretta, un bias non è rumore), oppure introdurlo esplicitamente in $h(\cdot)$ tramite parametri di slittamento stimati.
  3. **Rumore di processo.** Si completa la formulazione CWNA di $Q$ (§2.5) calibrando $\sigma_a$ e $\sigma_\alpha$ sullo slittamento effettivamente simulato, e si attiva la legge adattiva $\sigma_a^2(v) = \sigma_{a0}^2 + k_{terreno}\,v^2$ promessa in §1. Il parametro di terreno identificato dal D-WLS (§2.7) è l'ingresso naturale di questa taratura: la stessa struttura $\mu + c\,v^2$ che descrive la resistenza al moto governa la dipendenza dello slittamento dalla velocità, e la flotta la stima già in modo collaborativo.
  4. **Contenuto dei pacchetti scambiati e Covariance Intersection.** Lo slittamento rende l'incertezza dei vicini più grande e soprattutto **più variabile nel tempo**: un veicolo su fondo cedevole degrada rapidamente, uno su fondo compatto no. Trattare la stima ricevuta come esatta — ciò che il codice fa oggi — diventa insostenibile. Il pacchetto scambiato fra veicoli va quindi esteso dalla sola posizione $\hat p_j$ alla coppia $(\hat p_j,\ \Sigma_j^{(1:2,1:2)})$: **da 2 a 5 numeri**, cioè da 16 a 40 byte, pari a 800 B/s per veicolo a 10 Hz con due vicini. Su questa base si implementano, nell'ordine:
     * la correzione minima $R_{eff} = \sigma_{collab}^2 + u^T\Sigma_j^{(1:2,1:2)}u$, che proietta la covarianza del vicino sulla congiungente e rimuove la sovra-confidenza più grossolana;
     * la **Covariance Intersection** vera e propria nella formulazione di Carrillo-Arce et al., che affronta la correlazione ignota fra le stime (§2.3). Solo l'aggiornamento collaborativo la richiede: GPS, IMU ed encoder sono genuinamente indipendenti dalle stime dei vicini e restano su guadagno di Kalman standard — architettura nota come *Split Covariance Intersection*.

  Il punto 4 è indipendente dal modello di slittamento e potrebbe essere anticipato; è collocato qui perché la Fase 4 ridefinisce comunque il protocollo di comunicazione introducendo latenze e timestamp, e conviene modificare il formato del pacchetto una volta sola.
* **Fase 6: Validazione.** Il modello viene sottoposto a perdite stocastiche di pacchetti di rete (*packet loss*). La validazione finale del progetto includerà la creazione di grafici, in particolare l'analisi della consistenza del filtro tramite i *3-sigma bounds*, confrontando l'errore di stima reale rispetto alla covarianza teorica calcolata dal sistema distribuito.