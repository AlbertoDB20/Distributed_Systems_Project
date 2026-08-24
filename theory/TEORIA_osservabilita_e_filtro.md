# Osservabilità e Scelta del Filtro
### Nota teorica di approfondimento

Terza nota della serie, insieme a [TEORIA_rumore_di_processo.md](TEORIA_rumore_di_processo.md) e [TEORIA_campi_potenziali.md](TEORIA_campi_potenziali.md). La trattazione procede dalla teoria generale all'applicazione al sistema in esame.

---

# 1. Osservabilità

## 1.1 Cosa significa, in parole semplici

Un sistema è **osservabile** se, guardando solo le uscite (le misure) per un intervallo di tempo finito, è possibile **risalire univocamente allo stato interno**.

Una formulazione equivalente e più intuitiva: un sistema *non* è osservabile se esistono **due stati diversi che producono esattamente le stesse misure**. Se esistono, nessuna quantità di dati potrà mai distinguerli: non per un limite dell'algoritmo, ma perché **l'informazione non è presente nelle misure**.

Un'analogia: un osservatore chiuso in una stanza buia, dotato di bussola e contachilometri, conosce il proprio orientamento e la distanza percorsa. Se durante la notte l'intera stanza venisse traslata di 100 metri verso nord, **tutti gli strumenti fornirebbero le medesime letture**. La posizione assoluta è non osservabile.

## 1.2 Perché conta

Perché **uno stimatore può stimare solo ciò che è osservabile**. Non si tratta di una questione di scelta fra filtro di Kalman, particle filter o approcci data-driven: è una proprietà **strutturale della coppia (modello, sensori)**, indipendente dall'algoritmo.

Le conseguenze pratiche sono tre:

**a) L'incertezza cresce senza limite.** Nelle direzioni non osservabili $\Sigma$ diverge. È il comportamento corretto: il filtro segnala la propria ignoranza, e i bound a $3\sigma$ si allargano di conseguenza.

**b) Il caso pericoloso è quando il filtro *non* se ne accorge.** Se per un errore di modellazione il sistema linearizzato *sembra* osservabile mentre quello vero non lo è, il filtro genera informazione dal nulla, $\Sigma$ resta piccola, e diventa **sovra-confidente**. È la condizione che si verifica nel filtro decentralizzato adottato in questo progetto, analizzata al Caso E.

**c) Serve a progettare, non solo a diagnosticare.** L'analisi di osservabilità indica *quali sensori servono* e *dove collocarli*. Nel caso in esame: quante ancore UWB e con quale geometria.

## 1.3 Come si verifica

### Metodo 1 — Matrice di osservabilità (criterio di Kalman)

$$\mathcal{O} = \begin{bmatrix} C \\ CA \\ CA^2 \\ \vdots \\ CA^{n-1}\end{bmatrix} \qquad\qquad \text{osservabile} \iff \text{rank}(\mathcal{O}) = n$$

**Interpretazione.** L'uscita misurata è $y = Cx$. Derivando: $\dot y = C\dot x = CAx$, e ancora $\ddot y = CA^2x$. Da $y$ e dalle sue derivate si ottiene quindi il sistema $[C; CA; CA^2; \dots]\,x$: se tale matrice ha rango pieno, il sistema è invertibile e $x$ è ricostruibile. Il teorema di Cayley-Hamilton garantisce che oltre $n-1$ derivate non ottieni nulla di nuovo.

**Il nucleo di $\mathcal{O}$ è il sottospazio non osservabile**: sono le direzioni lungo cui lo stato può essere spostato **senza alterare alcuna uscita**. Sono precisamente gli "spostamenti della stanza al buio".

### Metodo 2 — Gramiano di osservabilità

$$W_o(t_0,t_1) = \int_{t_0}^{t_1}\Phi^T(\tau,t_0)\,C^T C\,\Phi(\tau,t_0)\,d\tau \qquad \text{osservabile} \iff W_o \text{ non singolare}$$

Il Gramiano è preferibile al test di rango per tre ragioni, tutte rilevanti in questo progetto:

- Il rango è **binario**: osservabile o no. Ma un sistema "osservabile appena appena" è praticamente indistinguibile da uno non osservabile, quando c'è rumore. Il Gramiano fornisce un **grado** di osservabilità: $\sigma_{\min}(W_o)$ quantifica quanto è debole la direzione peggiore.
- È applicabile a **sistemi tempo-varianti**, come quello in esame: la disponibilità dei sensori varia e $C$ cambia a ogni passo.
- Cattura il fatto che **l'informazione si accumula nel tempo**. Un veicolo che curva raccoglie informazione che in nessun singolo istante possiede.

### Metodo 3 — PBH (Popov-Belevitch-Hautus)

$\text{rank}\begin{bmatrix}A - \lambda I \\ C\end{bmatrix} = n$ per ogni autovalore $\lambda$. Utile per identificare **quale modo** è non osservabile, non solo se ce n'è uno.

### Per sistemi non lineari

Non esiste una matrice $\mathcal{O}$ globale. Si usa la **condizione di rango di osservabilità** con le derivate di Lie:

$$\mathcal{O} = \text{span}\{dh,\; dL_f h,\; dL_f^2 h,\; \dots\}$$

e si parla di **osservabilità locale debole**. In pratica ingegneristica si valuta l'osservabilità del sistema **linearizzato** nel punto di lavoro corrente — che è esattamente ciò che l'EKF calcola già a ogni passo.

### Un concetto vicino da non confondere: la rilevabilità

Un sistema è **rilevabile** (*detectable*) se i modi non osservabili sono **stabili**. È più debole dell'osservabilità ma sufficiente per avere un osservatore convergente: se non riesci a vedere un modo ma quel modo si spegne da solo, l'errore converge comunque. Nel sistema in esame la posizione in dead-reckoning **non** è un modo stabile: si tratta di un integratore puro, il cui errore cresce indefinitamente. Il sistema non è quindi nemmeno rilevabile in assenza di riferimenti assoluti.

---

## 1.4 Applicazione al sistema in esame

Nel corso di una singola missione il sistema attraversa **quattro regimi di osservabilità distinti**.

### Caso A — GPS disponibile

$$C = \begin{bmatrix} 1&0&0&0&0 \\ 0&1&0&0&0 \\ 0&0&1&0&0 \\ 0&0&0&0&1 \\ 0&0&0&1/r&L/2r \\ 0&0&0&1/r&-L/2r \end{bmatrix} \begin{matrix}\leftarrow \text{GPS}\\ \\ \leftarrow \text{IMU}\\ \\ \leftarrow \text{encoder}\\ \end{matrix}$$

$\text{rank}(C) = 5$ **già senza derivate**. Ogni stato è misurato direttamente o attraverso una mappa lineare invertibile (gli encoder danno $(v,\omega)$ tramite una matrice $2\times2$ invertibile).

In questa configurazione l'analisi di osservabilità è **immediata**: il risultato, riportato in README1 §3, non richiede alcuna elaborazione. Il contenuto sostanziale è nei tre casi successivi.

### Caso B — GPS negato, nessuna ancora, nessun vicino (dead reckoning puro)

Restano IMU ed encoder: $\text{rank}(C) = 3$, ossia $\theta$, $v$ e $\omega$ sono osservabili, $x$ e $y$ no.

Le derivate non aggiungono informazione, e la dimostrazione è immediata. Nella struttura di $A$, nel modello uniciclo *nessuna equazione dipende da $x$ e $y$* — la posizione non entra in nessuna equazione, entra solo la sua derivata. Quindi **le colonne 1 e 2 di $A$ sono identicamente nulle**. Anche le colonne 1 e 2 di $C$ sono nulle. Di conseguenza $CA$, $CA^2$, … hanno tutte le colonne 1 e 2 nulle.

$$\text{rank}(\mathcal{O}) = 3, \qquad \ker(\mathcal{O}) = \text{span}\{e_1, e_2\}$$

**La posizione è completamente non osservabile: si tratta di una proprietà esatta, non di un'approssimazione.** È la spiegazione formale della deriva osservata nelle zone cieche.

### Caso C — Ranging UWB verso ancore fisse

Ogni ancora visibile aggiunge una riga:

$$C_{uwb}^{(j)} = \left[\tfrac{x-X_j}{d}\;\;\tfrac{y-Y_j}{d}\;\;0\;\;0\;\;0\right]$$

che è un **versore diretto lungo la congiungente** veicolo-ancora, nel piano $(x,y)$.

| Ancore visibili | Rango del blocco posizione | Cosa sai |
|---|---|---|
| 0 | 0 | niente (caso B) |
| 1 | 1 | il veicolo giace su una **circonferenza**; la direzione tangenziale resta non osservabile |
| 2 non collineari | 2 | **osservabile** |
| 2 collineari con il veicolo | 1 | i due versori sono paralleli: geometria degenere, equivalente a una sola ancora |

**È a questo punto che la GDOP si collega all'osservabilità:**

$$\text{GDOP} = \sqrt{\text{tr}\left[(C_{geom}^T C_{geom})^{-1}\right]}$$

dove $C_{geom}$ è la pila dei versori di vista. **Ma quella è esattamente la sottomatrice di $C$ delle righe UWB.** La GDOP non è un criterio geometrico esterno importato dal GNSS: è una **misura del grado di osservabilità** del problema di stima nel punto considerato. Quando la geometria degenera, $C_{geom}^TC_{geom}$ diventa quasi singolare, la GDOP esplode e il Gramiano è mal condizionato — sono tre modi di dire la stessa cosa.

È la ragione per cui nel codice quella matrice è denominata `C_geom`: **ottimizzare la GDOP equivale a ottimizzare l'osservabilità**.

### Caso D — La flotta: osservabilità collettiva

È il risultato più significativo prodotto dall'analisi, ed è ciò che colloca il lavoro nell'ambito dei *sistemi distribuiti* anziché in quello della localizzazione di un singolo agente.

Considera lo stato impilato dei tre veicoli, $X = [x_1; x_2; x_3] \in \mathbb{R}^{15}$. Supponi che le uniche misure siano IMU, encoder e **distanze inter-veicolari** $d_{ij}$.

Ora trasla **l'intera flotta** di un vettore costante $\delta \in \mathbb{R}^2$:

- tutte le $d_{ij}$ restano identiche (la geometria relativa non cambia)
- tutti i $\theta_i$ restano identici
- tutti i $v_i, \omega_i$ restano identici

**L'uscita è identica.** Quindi le direzioni

$$n_1 = [1,0,0,0,0\,|\,1,0,0,0,0\,|\,1,0,0,0,0]^T \qquad n_2 = [0,1,\dots\,|\,0,1,\dots\,|\,0,1,\dots]^T$$

appartengono al nucleo: **deficienza di rango pari a 2**.

**Conclusione, da enunciare così:** *il ranging inter-veicolare da solo non può mai ancorare la posizione assoluta della flotta. Serve almeno un riferimento assoluto — GPS o ancora fissa — su almeno un agente.*

Va aggiunta una precisazione: la **rotazione** della flotta *è* osservabile, perché il magnetometro dà l'heading assoluto. Disponendo del solo giroscopio (misura relativa), anche la rotazione comune risulterebbe non osservabile e la deficienza salirebbe a **3**.

Il risultato giustifica a posteriori l'intera architettura, e spiega perché la formazione stretta adottata inizialmente (6 m) rendesse impossibile dimostrare il beneficio della localizzazione collaborativa: con tutti e tre i veicoli simultaneamente privi di riferimenti assoluti, il sistema ricade esattamente in questa condizione.

### Caso E — Il filtro decentralizzato maschera la non osservabilità

I due argomenti precedenti convergono in questo punto.

Nel Caso D la deficienza di rango esiste perché la Jacobiana della misura $d_{ij}$ rispetto allo stato **congiunto** contiene sia il blocco per $i$ che quello per $j$, e i due si cancellano applicati alla traslazione comune:

$$\frac{x_i-x_j}{d}\cdot 1 + \frac{x_j-x_i}{d}\cdot 1 = 0$$

Nel filtro **decentralizzato** adottato, tuttavia, il veicolo $i$ tratta la posizione del vicino come **nota esattamente**: la riga di $C$ ha derivate solo rispetto a $x_i$, il blocco rispetto a $x_j$ non esiste.

**La cancellazione non avviene più.** Il filtro locale *vede* una direzione osservabile dove il sistema reale non ne ha. Genera informazione dal nulla e diventa sovra-confidente.

Si tratta **dello stesso difetto** già rilevato a proposito della $R$ della misura collaborativa, che contiene solo $\sigma_{collab}^2$ e ignora $\Sigma_j$. Non sono due problemi: sono lo stesso problema visto da due angolazioni — la Jacobiana mancante e la covarianza mancante sono la stessa omissione. Ed è la ragione per cui l'architettura prevede la Covariance Intersection.

> **Stato e collocazione.** La CI **non è implementata** allo stato attuale, ed è programmata per la **Fase 5** (README principale, §4, punto 4) insieme all'estensione del pacchetto scambiato fra veicoli, che deve passare dalla sola $\hat p_j$ alla coppia $(\hat p_j, \Sigma_j^{(1:2,1:2)})$. Senza quella covarianza la CI non è nemmeno formulabile: i pesi $\gamma$ e $1-\gamma$ si applicano proprio alle inverse delle covarianze.

Il fenomeno è noto in letteratura come *observability mismatch*, ed è studiato principalmente nell'ambito dell'EKF-SLAM (Huang & Roumeliotis).

### Strumento diagnostico non ancora implementato

Il test di rango in un singolo istante è fuorviante per un sistema tempo-variante. Il deliverable giusto è il **Gramiano su finestra scorrevole**:

$$W_o(k, k{+}N) = \sum_{i=k}^{k+N}\Phi^T(i,k)\,C_i^T R_i^{-1} C_i\,\Phi(i,k)$$

con $\sigma_{\min}(W_o)$ o $\text{cond}(W_o)$ tracciati nel tempo, sullo sfondo delle fasce GPS-denied. È il grafico che sintetizza l'intero argomento teorico del progetto.

Esiste inoltre una via più diretta: **l'inversa della covarianza $\Sigma^{-1}$ è la matrice di informazione**, e accumula proprio i termini $C^TR^{-1}C$. Non coincide con il Gramiano, poiché $Q$ erode progressivamente l'informazione accumulata, ma la crescita di $\Sigma$ nelle direzioni deboli **costituisce già la firma empirica della non osservabilità**. Lo storico `Sigma_hist` è disponibile in Fase 2 e Fase 3, e la traccia del blocco posizione è tracciata nella figura `5_copertura_e_covarianza.png` della Fase 3.

---

# 2. Perché l'EKF, e non l'UKF

## 2.1 Il problema generale

Il filtro di Kalman è **ottimo** (minima varianza) per sistemi lineari con rumore gaussiano. Per sistemi non lineari serve un'approssimazione, e la domanda vera è una sola:

> **come propaghi una gaussiana attraverso una funzione non lineare?**

Il problema è che una gaussiana che attraversa una funzione curva **non esce gaussiana**. Bisogna decidere quale gaussiana usare come approssimazione dell'uscita.

### L'approccio EKF: linearizza la funzione

Si sostituisce $f(x)$ con il suo sviluppo di Taylor al primo ordine attorno alla stima corrente, applicando poi le formule del KF lineare.

- La **media** viene propagata attraverso la $f$ vera, non lineare — questo è buono.
- La **covarianza** viene propagata attraverso la Jacobiana $A$ — approssimazione al primo ordine.

L'errore è del **secondo ordine e superiori**: quello che si perde è la **curvatura**.

### L'approccio UKF: non linearizzare, campionare

La *trasformata unscented* seleziona $2n+1$ punti deterministici (**sigma points**) che riproducono **esattamente** media e covarianza della distribuzione a priori. Ciascun punto viene propagato attraverso la funzione non lineare *vera*, e da quelli trasformati si ricalcolano media e covarianza.

- Accurata al **secondo ordine** (terzo, per priori gaussiane con tuning appropriato) per qualsiasi non linearità
- **Nessuna Jacobiana**: funziona con modelli non derivabili o a scatola chiusa
- Costo: $2n+1$ valutazioni invece di 1 + Jacobiana

Il motto di Julier e Uhlmann riassume tutto: *"è più facile approssimare una distribuzione di probabilità che una funzione non lineare arbitraria."*

### Quando la differenza conta davvero

L'EKF degrada quando:

1. **la non linearità è forte sulla scala dell'incertezza** — cioè la funzione curva sensibilmente entro $\pm3\sigma$
2. **l'incertezza è grande**
3. le Jacobiane sono difficili, costose o impossibili da calcolare

Il criterio è **relativo**, non assoluto: non rileva la curvatura in sé, ma la curvatura **nella regione coperta dall'incertezza**. Se l'incertezza è piccola rispetto alla scala di curvatura, la linearizzazione è eccellente e l'EKF vale quanto l'UKF a una frazione del costo.

---

## 2.2 Valutazione quantitativa sul sistema in esame

Le non linearità presenti sono due, poiché GPS, IMU ed encoder hanno modello di misura **lineare**:

### Non linearità 1 — modello di moto, $x \mathrel{+}= v\cos\theta\,T_s$

L'errore di linearizzazione è dell'ordine del termine del secondo ordine:

$$\tfrac{1}{2}\left|\tfrac{\partial^2 f}{\partial\theta^2}\right|\sigma_\theta^2 = \tfrac{1}{2}\,v\,T_s\,\sigma_\theta^2$$

Con i valori operativi ($v = 2.5$ m/s, $T_s = 0.1$ s, $\sigma_\theta = 0.008$ rad):

$$\tfrac{1}{2}\cdot 0.25\cdot(0.008)^2 = 8\cdot10^{-6}\ \text{m} = \textbf{8 micrometri per passo}$$

Da confrontare con il rumore di processo per passo, $\sqrt{Q_{11}} \approx 0.045$ m.

> **L'errore di linearizzazione è circa 5000 volte più piccolo del rumore di processo.**

Anche con il precedente $\sigma_\theta = 0.043$ rad il rapporto restava di 200 a 1.

### Non linearità 2 — misura di distanza, $d = \|p - p_a\|$

La funzione distanza curva sulla scala di $d$ stesso. L'errore di linearizzazione è dell'ordine di $\tfrac{1}{2}\sigma_\perp^2/d$, con $\sigma_\perp$ l'incertezza perpendicolare alla congiungente. Con $\sigma_\perp \approx 0.2$ m e $d \approx 50$–150 m:

$$\tfrac{1}{2}\cdot\frac{0.04}{50} = 4\cdot10^{-4}\ \text{m}$$

contro $\sigma_{uwb} = 0.5$ m. **Circa 1000 volte sotto il rumore di misura.**

Il ranging diventa fortemente non lineare quando la distanza dall'ancora è **confrontabile con l'incertezza di posizione**. Qui le due grandezze differiscono di tre ordini di grandezza: decine o centinaia di metri contro un'incertezza decimetrica.

### Conclusione

**Per questo sistema, EKF e UKF darebbero risultati praticamente identici.** L'UKF spenderebbe 11 valutazioni per passo (con $n=5$) più una fattorizzazione di Cholesky di $\Sigma$ a ogni passo, per recuperare una correzione tre ordini di grandezza sotto il rumore.

---

## 2.3 Motivazioni della scelta

Va anzitutto escluso un argomento che non regge: la scelta dell'EKF **non** può essere motivata dalla necessità di condurre l'analisi di osservabilità. Quell'analisi si esegue sul modello linearizzato in ogni caso, indipendentemente dal filtro utilizzato per la stima, e gli Jacobiani esistono a prescindere dal fatto che il filtro li impieghi. Nulla impedirebbe di calcolare gli Jacobiani per la sola analisi e di stimare con un UKF.

Le motivazioni effettive, in ordine di rilevanza, sono le seguenti. Sono riportate in forma sintetica nel README principale, §2.1.

**1. Argomento quantitativo: la non linearità è debole, ed è misurata.** I termini del secondo ordine risultano tre ordini di grandezza sotto il pavimento di rumore (§2.2). È l'argomento decisivo, perché poggia su una misura e non su una valutazione qualitativa.

**2. Gli Jacobiani sono necessari altrove.** Servono per il Gramiano di osservabilità e per il calcolo del peso $\gamma$ nella **Covariance Intersection**. Disponendone già, l'EKF risulta la scelta naturale. Si tratta di un argomento di opportunità, non di necessità.

**3. Costo computazionale in contesto distribuito.** Con 3 veicoli a 10 Hz il costo è irrilevante, ma l'architettura deve scalare a $N$ agenti e alle frequenze reali della Fase 4, dove un'IMU può operare a 100–200 Hz. L'EKF richiede una predizione e una Jacobiana; l'UKF undici propagazioni e una fattorizzazione di Cholesky, con un rapporto di circa 5–10×.

**4. Misure a dimensione variabile.** La matrice $C$ cambia dimensione a runtime in funzione della disponibilità del GPS e del numero di ancore e vicini in portata. Con l'EKF è sufficiente accodare righe; con l'UKF andrebbe rieseguita la trasformata unscented sulla funzione di misura a ogni cambio di configurazione, il che è realizzabile ma appesantisce il codice.

**5. Assenza di parametri di taratura.** L'UKF richiede di fissare e giustificare $\alpha$, $\beta$, $\kappa$; l'EKF non ne ha.

---

## 2.4 Condizioni in cui la scelta andrebbe rivista

La scelta dell'EKF è **condizionata** alle caratteristiche del sistema, non assoluta. L'UKF risulterebbe preferibile nelle seguenti condizioni:

- presenza di sensori **bearing-only** o di visione, fortemente non lineari a corto raggio;
- **incertezza iniziale elevata**, come in un'inizializzazione globale con $\sigma$ dell'ordine delle decine di metri, dove $\cos\theta$ curva sensibilmente entro $\pm3\sigma$;
- adozione di un modello **dinamico** con forze di contatto cingolo-terreno, in cui gli Jacobiani sono onerosi e la non linearità marcata;
- **crescita di $\sigma_\theta$.** Se in **Fase 5** il modello di slittamento degradasse sensibilmente la stima di heading, il calcolo del §2.2 va ripetuto: l'errore di linearizzazione cresce **quadraticamente** con $\sigma_\theta$. Portando $\sigma_\theta$ a 0.3 rad l'errore aumenterebbe di un fattore 1400, raggiungendo 0.01 m per passo, non più trascurabile. La verifica va quindi ripetuta dopo la Fase 5, e non può considerarsi acquisita.

---

## Riferimenti

- R. E. Kalman, *"On the General Theory of Control Systems"*, 1960 — criterio di osservabilità.
- S. J. Julier, J. K. Uhlmann, *"Unscented Filtering and Nonlinear Estimation"*, Proc. IEEE 92(3), 2004.
- G. P. Huang, S. I. Roumeliotis, *"Analysis and Improvement of the Consistency of Extended Kalman Filter Based SLAM"*, ICRA 2008 — *observability mismatch*.
- A. Barrau, S. Bonnabel, *"The Invariant Extended Kalman Filter as a Stable Observer"*, IEEE T-AC 62(4), 2017.
