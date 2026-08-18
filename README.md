# Progetto di Intelligient Distributed Systems
## Localizzazione Collaborativa e Controllo di una Flotta di Snow Groomer Off-road in Ambienti Ostili

**Corso:** Intelligent Distributed Systems  
**Obiettivo del Progetto:** Sviluppo, simulazione e validazione di un'architettura decentralizzata per la stima della posa e il controllo di formazione di una flotta di $N$ veicoli terrestri. Il sistema deve operare in modo resiliente in presenza di terreni a bassa aderenza (slittamenti dinamici), zone di negazione del segnale GPS (GPS-denied) e vincoli di comunicazione tipici delle reti reali.

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
La stima dello stato locale di ogni veicolo è affidata a un Extended Kalman Filter (EKF). La scelta dell'EKF rispetto ad altre varianti (come l'UKF) è dettata dall'esigenza di condurre una analisi di osservabilità. Calcolando gli Jacobiani del sistema ad ogni istante di campionamento, è possibile valutare il rango della matrice di osservabilità $\mathcal{O}$. Questo permetterà di dimostrare matematicamente come la perdita del GPS degradi l'osservabilità (portando a una deriva della posa assoluta) e come la fusione con le misurazioni UWB permetta di recuperarla.

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
Tuttavia, lo scambio continuo di stime in una rete chiusa genera il fenomeno del *Data Rumination*: le informazioni diventano circolari e i filtri di Kalman iniziano a sottostimare la propria covarianza (diventano troppo "ottimisti"). Per risolvere questo problema, la fusione dei dati provenienti dagli altri veicoli avviene tramite l'algoritmo di **Covariance Intersection (CI)**, che garantisce stime statisticamente consistenti anche in presenza di correlazioni ignote tra gli agenti.

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

**Ritardo di un passo sulle stime scambiate.** La posizione di un vicino usata come ancora mobile è la sua stima $\hat{x}_k^{(j)}$ — l'ultimo pacchetto ricevuto — propagata di un passo con il modello di moto, ottenendo $\hat{x}_{k+1|k}^{(j)}$. La scelta risponde a due esigenze: rompe il loop algebrico fra filtri che altrimenti dipenderebbero l'uno dalla stima aggiornata dell'altro nel medesimo istante, e rispecchia ciò che un canale di comunicazione reale rende effettivamente disponibile. La propagazione è necessaria perché confrontare una misura acquisita a $t_{k+1}$ con una posizione riferita a $t_k$ reintrodurrebbe lo stesso bias di $|v_j| T_s$ descritto sopra.

### 2.5 Notazione Adottata

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

---

## 3. Controllo di Formazione (Consenso)

La flotta, composta inizialmente da $N=3$ veicoli, deve navigare lungo la mappa mantenendo una specifica formazione (ad esempio, muovendosi affiancati). 
Invece di adottare un approccio centralizzato, si utilizza un protocollo basato sul **Consenso**. Ogni veicolo agisce in base all'errore calcolato tra la propria posizione e le posizioni stimate dei propri vicini, regolando la propria velocità e sterzatura per convergere alla distanza relativa desiderata.
Per garantire la sicurezza operativa, al livello di controllo cinematico viene sovrapposto un algoritmo di evitamento collisioni basato sui **Campi Potenziali Artificiali** (Khatib, 1986): se la distanza inter-veicolare scende sotto la soglia $d_{safe}$, viene generata una velocità repulsiva virtuale che devia temporaneamente la traiettoria dei mezzi.

Consenso e repulsione non sono due controllori distinti, ma i due termini di un unico campo potenziale $U = U_{cons} + U_{rep}$, di cui la legge di controllo è l'antigradiente: il consenso è il potenziale **attrattivo**, il cui minimo coincide con la formazione desiderata, e la funzione **FIRAS** di Khatib fornisce quello **repulsivo**. Un requisito di progetto lega i due termini: deve valere $d_{safe} < \min_{i\ne j}\|\Delta_{ij}\|$, altrimenti la repulsione non si annulla mai nella configurazione desiderata e la formazione risulta irraggiungibile (condizione *GNRON*). La trattazione completa — derivazione del gradiente, taratura del guadagno, limiti del metodo e confronto con le Control Barrier Functions — è in [fase_2/TEORIA_campi_potenziali.md](fase_2/TEORIA_campi_potenziali.md).

---

## 4. Roadmap di Sviluppo e Validazione

Per garantire la solidità dell'impianto teorico, lo sviluppo in ambiente MATLAB seguirà un paradigma incrementale, partendo da un caso ideale per arrivare a un sistema distribuito quanto più vicino alla realtà.

* **Fase 1: Il Core (Veicolo Singolo Ideale).** Si implementa la dinamica dell'uniciclo e l'EKF per un singolo veicolo. In questa fase, tutti i sensori (GPS, Encoder, IMU) operano alla stessa frequenza di campionamento. Il GPS copre l'intera mappa. L'obiettivo è tarare i parametri di rumore e validare il calcolo degli Jacobiani per l'analisi di osservabilità locale.
* **Fase 2: La Flotta e la Formazione.** Si istanziano $N=3$ veicoli comunicanti attraverso un canale di rete ideale (latenza nulla, zero perdite). Viene integrato il controllo tramite consenso e l'algoritmo per l'evitamento delle collisioni. A scopo puramente analitico e accademico, in questa fase si attribuisce al veicolo "Master" un sensore GPS di qualità superiore, per osservare come la sua stima influenzi il comportamento della formazione rispetto agli "Slave".
* **Fase 3: L'Ambiente Ostile.** Si introduce il realismo ambientale. Vengono create zone *GPS-denied* casuali lungo il percorso e si posizionano le ancore UWB calcolando la GDOP ottimale. Si testa la resilienza della flotta: il sistema deve dimostrare di poter attraversare le zone cieche mantenendo la formazione, affidandosi alla triangolazione UWB e alla localizzazione collaborativa.
* **Fase 4: Realismo dei Sistemi Distribuiti (Multi-rate e Latenze).** Si rimuovono le assunzioni ideali. I sensori operano ora alle loro frequenze reali (es. IMU molto veloce, GPS lento). Il canale di comunicazione introduce latenze. Viene implementato il *timestamping* delle misurazioni: le letture in ritardo provenienti dagli altri veicoli vengono retroattivate correttamente nel buffer storico dell'EKF per non destabilizzare il filtro. Si uniforma la qualità del GPS per tutti i veicoli, rendendo le zone di perdita del segnale dipendenti esclusivamente dalla posizione spaziale.
* **Fase 5: Realismo Off-Highway (Modellazione dello Slittamento).** Si rimuove l'assunzione di aderenza ideale, finora implicita nel fatto che la velocità reale del veicolo coincide istante per istante con quella comandata. Viene introdotto un modello di slittamento rappresentativo delle condizioni operative di un mezzo cingolato su fondo nevoso, basato su uno studio già disponibile in letteratura sulla trazione di veicoli cingolati su terreni a bassa aderenza, sviluppato presso l'Università degli Studi di Trento. La fase comporta tre interventi sull'architettura:
  1. **Impianto.** La generazione della ground truth passa da `x_true(4,k+1) = v_cmd` a una legge $v^{true} = f_{slip}(v_{cmd}, x^{true}, \text{terreno})$. Il punto di innesto è già predisposto e marcato nel codice di Fase 2 e Fase 3.
  2. **Modello di misura degli encoder.** È la conseguenza più critica. Il modello attuale $z_{enc} = \left[\frac{v}{r}+\frac{L\omega}{2r},\; \frac{v}{r}-\frac{L\omega}{2r}\right]^T$ presuppone rotolamento puro. In presenza di slittamento gli encoder misurano la velocità dei *cingoli*, non quella del veicolo: la relazione fra le due si carica di un errore **sistematico**, non di un rumore bianco. Poiché gli encoder sono l'osservatore primario di $v$ e $\omega$, un modello di misura polarizzato degraderebbe l'intero filtro. Occorre decidere se assorbire lo slittamento gonfiando $R_{enc}$ (soluzione minima ma statisticamente scorretta, un bias non è rumore), oppure introdurlo esplicitamente in $h(\cdot)$ tramite parametri di slittamento stimati.
  3. **Rumore di processo.** Si completa la formulazione CWNA di $Q$ (§2.5) calibrando $\sigma_a$ e $\sigma_\alpha$ sullo slittamento effettivamente simulato, e si attiva la legge adattiva $\sigma_a^2(v) = \sigma_{a0}^2 + k_{terreno}\,v^2$ promessa in §1.
* **Fase 6: Validazione.** Il modello viene sottoposto a perdite stocastiche di pacchetti di rete (*packet loss*). La validazione finale del progetto includerà la creazione di grafici, in particolare l'analisi della consistenza del filtro tramite i *3-sigma bounds*, confrontando l'errore di stima reale rispetto alla covarianza teorica calcolata dal sistema distribuito.