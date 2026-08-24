# Osservabilità e Scelta del Filtro
### Nota teorica di approfondimento

Terza nota della serie, insieme a [TEORIA_rumore_di_processo.md](TEORIA_rumore_di_processo.md) e [TEORIA_campi_potenziali.md](../fase_2/TEORIA_campi_potenziali.md). Teoria generale prima, applicazione al progetto poi.

---

# 1. Osservabilità

## 1.1 Cosa significa, in parole semplici

Un sistema è **osservabile** se, guardando solo le uscite (le misure) per un intervallo di tempo finito, riesci a **risalire univocamente allo stato interno**.

La formulazione equivalente che aiuta di più a capire: un sistema *non* è osservabile se esistono **due stati diversi che producono esattamente le stesse misure**. Se esistono, nessuna quantità di dati potrà mai distinguerli — non perché il tuo algoritmo è scarso, ma perché **l'informazione non c'è**.

L'immagine: sei in una stanza buia con una bussola e un contachilometri. Sai in che direzione sei girato e quanta strada hai fatto. Ma se qualcuno di notte spostasse l'intera stanza di 100 metri a nord, **tutti i tuoi strumenti direbbero esattamente le stesse cose**. La tua posizione assoluta è non osservabile.

## 1.2 Perché conta

Perché **uno stimatore può stimare solo ciò che è osservabile**. Questo va detto con forza: non è un problema di Kalman contro particle filter contro rete neurale. È una proprietà **strutturale della coppia (modello, sensori)**, indipendente dall'algoritmo.

Le conseguenze pratiche sono tre:

**a) L'incertezza cresce senza limite.** Nelle direzioni non osservabili $\Sigma$ diverge. In un certo senso è la cosa *buona*: il filtro sa di non sapere, e i bound a $3\sigma$ si allargano onestamente.

**b) Il caso pericoloso è quando il filtro *non* se ne accorge.** Se per un errore di modellazione il sistema linearizzato *sembra* osservabile mentre quello vero non lo è, il filtro genera informazione dal nulla, $\Sigma$ resta piccola, e diventa **sovra-confidente**. Torneremo su questo, perché nel tuo progetto succede esattamente.

**c) Serve a progettare, non solo a diagnosticare.** L'analisi di osservabilità ti dice *quali sensori servono* e *dove metterli*. Nel tuo caso: quante ancore UWB, e in che geometria.

## 1.3 Come si verifica

### Metodo 1 — Matrice di osservabilità (criterio di Kalman)

$$\mathcal{O} = \begin{bmatrix} C \\ CA \\ CA^2 \\ \vdots \\ CA^{n-1}\end{bmatrix} \qquad\qquad \text{osservabile} \iff \text{rank}(\mathcal{O}) = n$$

**Da dove viene, intuitivamente.** Misuri $y = Cx$. Derivi: $\dot y = C\dot x = CAx$. Derivi ancora: $\ddot y = CA^2x$. Quindi da $y$ e dalle sue derivate ottieni il sistema $[C; CA; CA^2; \dots]\,x$. Se quella pila ha rango pieno puoi invertire e ricavare $x$. Il teorema di Cayley-Hamilton garantisce che oltre $n-1$ derivate non ottieni nulla di nuovo.

**Il nucleo di $\mathcal{O}$ è il sottospazio non osservabile**: sono le direzioni lungo cui puoi muovere lo stato **senza cambiare nessuna uscita**. Sono precisamente gli "spostamenti della stanza al buio".

### Metodo 2 — Gramiano di osservabilità

$$W_o(t_0,t_1) = \int_{t_0}^{t_1}\Phi^T(\tau,t_0)\,C^T C\,\Phi(\tau,t_0)\,d\tau \qquad \text{osservabile} \iff W_o \text{ non singolare}$$

**Perché è superiore al test di rango**, e perché nel tuo caso è lo strumento giusto:

- Il rango è **binario**: osservabile o no. Ma un sistema "osservabile appena appena" è praticamente indistinguibile da uno non osservabile, quando c'è rumore. Il Gramiano dà un **grado** di osservabilità: $\sigma_{\min}(W_o)$ ti dice quanto è debole la direzione peggiore.
- Funziona per **sistemi tempo-varianti**, che è il tuo caso: i sensori vanno e vengono, e $C$ cambia a ogni passo.
- Cattura il fatto che **l'informazione si accumula nel tempo**. Un veicolo che curva raccoglie informazione che in nessun singolo istante possiede.

### Metodo 3 — PBH (Popov-Belevitch-Hautus)

$\text{rank}\begin{bmatrix}A - \lambda I \\ C\end{bmatrix} = n$ per ogni autovalore $\lambda$. Utile per identificare **quale modo** è non osservabile, non solo se ce n'è uno.

### Per sistemi non lineari

Non esiste una matrice $\mathcal{O}$ globale. Si usa la **condizione di rango di osservabilità** con le derivate di Lie:

$$\mathcal{O} = \text{span}\{dh,\; dL_f h,\; dL_f^2 h,\; \dots\}$$

e si parla di **osservabilità locale debole**. In pratica ingegneristica si valuta l'osservabilità del sistema **linearizzato** nel punto di lavoro corrente — che è esattamente ciò che l'EKF calcola già.

### Un concetto vicino da non confondere: la rilevabilità

Un sistema è **rilevabile** (*detectable*) se i modi non osservabili sono **stabili**. È più debole dell'osservabilità ma sufficiente per avere un osservatore convergente: se non riesci a vedere un modo ma quel modo si spegne da solo, l'errore converge comunque. Nel tuo caso la posizione in dead-reckoning **non** è un modo stabile — è un integratore puro, l'errore cresce — quindi il sistema non è nemmeno rilevabile senza riferimenti assoluti.

---

## 1.4 Applicato al tuo progetto

Qui la cosa diventa interessante, perché il tuo sistema attraversa **quattro regimi di osservabilità diversi** nella stessa missione.

### Caso A — GPS disponibile

$$C = \begin{bmatrix} 1&0&0&0&0 \\ 0&1&0&0&0 \\ 0&0&1&0&0 \\ 0&0&0&0&1 \\ 0&0&0&1/r&L/2r \\ 0&0&0&1/r&-L/2r \end{bmatrix} \begin{matrix}\leftarrow \text{GPS}\\ \\ \leftarrow \text{IMU}\\ \\ \leftarrow \text{encoder}\\ \end{matrix}$$

$\text{rank}(C) = 5$ **già senza derivate**. Ogni stato è misurato direttamente o attraverso una mappa lineare invertibile (gli encoder danno $(v,\omega)$ tramite una matrice $2\times2$ invertibile).

Va detto con onestà: **in questa configurazione l'analisi di osservabilità è quasi banale**. Il README1 §3 lo afferma correttamente, ma non è un risultato che vale punti. La sostanza sta negli altri tre casi.

### Caso B — GPS negato, nessuna ancora, nessun vicino (dead reckoning puro)

Restano IMU ed encoder: $\text{rank}(C) = 3$ — vedi $\theta$, $v$, $\omega$, ma non $x,y$.

Aggiungono qualcosa le derivate? **No, e si dimostra in due righe.** Guarda la struttura di $A$: nel modello uniciclo *niente dipende da $x$ e $y$* — la posizione non entra in nessuna equazione, entra solo la sua derivata. Quindi **le colonne 1 e 2 di $A$ sono identicamente nulle**. Anche le colonne 1 e 2 di $C$ sono nulle. Di conseguenza $CA$, $CA^2$, … hanno tutte le colonne 1 e 2 nulle.

$$\text{rank}(\mathcal{O}) = 3, \qquad \ker(\mathcal{O}) = \text{span}\{e_1, e_2\}$$

**La posizione è completamente non osservabile, ed è una proprietà esatta, non approssimata.** È la deriva che vedi nelle zone cieche, e ora hai il motivo formale.

### Caso C — Ranging UWB verso ancore fisse

Ogni ancora visibile aggiunge una riga:

$$C_{uwb}^{(j)} = \left[\tfrac{x-X_j}{d}\;\;\tfrac{y-Y_j}{d}\;\;0\;\;0\;\;0\right]$$

che è un **versore diretto lungo la congiungente** veicolo-ancora, nel piano $(x,y)$.

| Ancore visibili | Rango del blocco posizione | Cosa sai |
|---|---|---|
| 0 | 0 | niente (caso B) |
| 1 | 1 | sei su una **circonferenza**; la direzione tangenziale resta non osservabile |
| 2 non collineari | 2 | **osservabile** |
| 2 collineari con te | 1 | i due versori sono paralleli → degenere, come averne una sola |

**Ed è qui che la GDOP si aggancia all'osservabilità** — questo è il punto più elegante che il tuo progetto contiene:

$$\text{GDOP} = \sqrt{\text{tr}\left[(C_{geom}^T C_{geom})^{-1}\right]}$$

dove $C_{geom}$ è la pila dei versori di vista. **Ma quella è esattamente la sottomatrice di $C$ delle righe UWB.** La GDOP non è un criterio geometrico esterno importato dal GNSS: è una **misura del grado di osservabilità** del problema di stima nel punto in cui ti trovi. Quando la geometria degenera, $C_{geom}^TC_{geom}$ diventa quasi singolare, la GDOP esplode e il Gramiano è mal condizionato — sono tre modi di dire la stessa cosa.

È il motivo per cui, nel rename, quella matrice si chiama `C_geom` e non `H_mat`. **Ottimizzare la GDOP significa ottimizzare l'osservabilità.** Dillo così all'orale.

### Caso D — La flotta: osservabilità collettiva

Questo è il risultato più forte che il tuo progetto può produrre, ed è quello che lo rende un progetto di *sistemi distribuiti* e non di localizzazione singola.

Considera lo stato impilato dei tre veicoli, $X = [x_1; x_2; x_3] \in \mathbb{R}^{15}$. Supponi che le uniche misure siano IMU, encoder e **distanze inter-veicolari** $d_{ij}$.

Ora trasla **l'intera flotta** di un vettore costante $\delta \in \mathbb{R}^2$:

- tutte le $d_{ij}$ restano identiche (la geometria relativa non cambia)
- tutti i $\theta_i$ restano identici
- tutti i $v_i, \omega_i$ restano identici

**L'uscita è identica.** Quindi le direzioni

$$n_1 = [1,0,0,0,0\,|\,1,0,0,0,0\,|\,1,0,0,0,0]^T \qquad n_2 = [0,1,\dots\,|\,0,1,\dots\,|\,0,1,\dots]^T$$

appartengono al nucleo: **deficienza di rango pari a 2**.

**Conclusione, da enunciare così:** *il ranging inter-veicolare da solo non può mai ancorare la posizione assoluta della flotta. Serve almeno un riferimento assoluto — GPS o ancora fissa — su almeno un agente.*

Una sfumatura che vale la pena aggiungere: la **rotazione** della flotta *è* osservabile, perché il magnetometro dà l'heading assoluto. Se avessi solo un giroscopio (misura relativa), anche la rotazione comune sarebbe non osservabile e la deficienza salirebbe a **3**.

Questo risultato giustifica a posteriori tutta l'architettura, ed è anche il motivo per cui, all'inizio, la formazione da 6 m rendeva impossibile dimostrare la localizzazione collaborativa: se tutti e tre sono ciechi contemporaneamente, sei esattamente in questo caso.

### Caso E — Il punto delicato: il tuo filtro locale *nasconde* la non osservabilità

E qui i due discorsi si saldano, in un modo che vale la pena capire bene.

Nel Caso D la deficienza di rango esiste perché la Jacobiana della misura $d_{ij}$ rispetto allo stato **congiunto** contiene sia il blocco per $i$ che quello per $j$, e i due si cancellano applicati alla traslazione comune:

$$\frac{x_i-x_j}{d}\cdot 1 + \frac{x_j-x_i}{d}\cdot 1 = 0$$

Ma nel tuo filtro **decentralizzato**, il veicolo $i$ tratta la posizione del vicino come **nota esattamente**: la riga di $C$ ha derivate solo rispetto a $x_i$, il blocco rispetto a $x_j$ non esiste.

**La cancellazione non avviene più.** Il filtro locale *vede* una direzione osservabile dove il sistema reale non ne ha. Genera informazione dal nulla e diventa sovra-confidente.

E nota una cosa: **è esattamente lo stesso difetto** segnalato a proposito della $R$ della misura collaborativa, che contiene solo $\sigma_{collab}^2$ e ignora $\Sigma_j$. Non sono due problemi: sono lo stesso problema visto da due angolazioni — la Jacobiana mancante e la covarianza mancante sono la stessa omissione. Ed è la ragione per cui l'architettura prevede la Covariance Intersection.

> **Stato e collocazione.** La CI **non è implementata** allo stato attuale, ed è programmata per la **Fase 5** (README principale, §4, punto 4) insieme all'estensione del pacchetto scambiato fra veicoli, che deve passare dalla sola $\hat p_j$ alla coppia $(\hat p_j, \Sigma_j^{(1:2,1:2)})$. Senza quella covarianza la CI non è nemmeno formulabile: i pesi $\omega$ e $1-\omega$ si applicano proprio alle inverse delle covarianze.

Questo è materiale da esame di alto livello. Il fenomeno ha un nome in letteratura — *observability mismatch* — ed è studiato soprattutto nell'EKF-SLAM (Huang & Roumeliotis).

### Lo strumento pratico che manca

Il test di rango in un singolo istante è fuorviante per un sistema tempo-variante. Il deliverable giusto è il **Gramiano su finestra scorrevole**:

$$W_o(k, k{+}N) = \sum_{i=k}^{k+N}\Phi^T(i,k)\,C_i^T R_i^{-1} C_i\,\Phi(i,k)$$

con $\sigma_{\min}(W_o)$ o $\text{cond}(W_o)$ plottati nel tempo, sullo sfondo delle fasce GPS-denied. È **il** grafico che porta tutto l'argomento teorico del progetto.

E una scorciatoia utile: **l'inversa della covarianza $\Sigma^{-1}$ è la matrice di informazione**, e accumula proprio i termini $C^TR^{-1}C$. Non è identica al Gramiano — $Q$ nel frattempo erode l'informazione accumulata — ma la crescita di $\Sigma$ nelle direzioni deboli **è già la firma empirica della non osservabilità**. `Sigma_hist` è già salvato: gli autovalori del blocco posizione nel tempo sono un primo grafico che costa zero.

---

# 2. Perché l'EKF, e non l'UKF

## 2.1 Il problema generale

Il filtro di Kalman è **ottimo** (minima varianza) per sistemi lineari con rumore gaussiano. Per sistemi non lineari serve un'approssimazione, e la domanda vera è una sola:

> **come propaghi una gaussiana attraverso una funzione non lineare?**

Il problema è che una gaussiana che attraversa una funzione curva **non esce gaussiana**. Bisogna decidere quale gaussiana usare come approssimazione dell'uscita.

### L'approccio EKF: linearizza la funzione

Sostituisci $f(x)$ con il suo sviluppo di Taylor al primo ordine attorno alla stima corrente, poi applichi le formule del KF lineare.

- La **media** viene propagata attraverso la $f$ vera, non lineare — questo è buono.
- La **covarianza** viene propagata attraverso la Jacobiana $A$ — approssimazione al primo ordine.

L'errore è del **secondo ordine e superiori**: quello che si perde è la **curvatura**.

### L'approccio UKF: non linearizzare, campionare

La *trasformata unscented*: scegli $2n+1$ punti deterministici (**sigma points**) che riproducono **esattamente** media e covarianza della distribuzione a priori. Passi **ciascuno** attraverso la funzione non lineare *vera*. Ricalcoli media e covarianza dai punti trasformati.

- Accurata al **secondo ordine** (terzo, per priori gaussiane con tuning appropriato) per qualsiasi non linearità
- **Nessuna Jacobiana**: funziona con modelli non derivabili o a scatola chiusa
- Costo: $2n+1$ valutazioni invece di 1 + Jacobiana

Il motto di Julier e Uhlmann riassume tutto: *"è più facile approssimare una distribuzione di probabilità che una funzione non lineare arbitraria."*

### Quando la differenza conta davvero

L'EKF degrada quando:

1. **la non linearità è forte sulla scala dell'incertezza** — cioè la funzione curva sensibilmente entro $\pm3\sigma$
2. **l'incertezza è grande**
3. le Jacobiane sono difficili, costose o impossibili da calcolare

Il criterio è **relativo**, non assoluto: non conta quanto la funzione è curva, conta quanto curva **nella regione in cui sei incerto**. Se l'incertezza è piccola rispetto alla scala di curvatura, la linearizzazione è eccellente e l'EKF vale quanto l'UKF a una frazione del costo.

---

## 2.2 Applicato al tuo progetto: facciamo il conto

Le non linearità presenti sono due, perché GPS, IMU ed encoder sono **lineari**:

### Non linearità 1 — modello di moto, $x \mathrel{+}= v\cos\theta\,T_s$

L'errore di linearizzazione è dell'ordine del termine del secondo ordine:

$$\tfrac{1}{2}\left|\tfrac{\partial^2 f}{\partial\theta^2}\right|\sigma_\theta^2 = \tfrac{1}{2}\,v\,T_s\,\sigma_\theta^2$$

Con i numeri attuali ($v = 2.5$ m/s, $T_s = 0.1$ s, $\sigma_\theta = 0.008$ rad):

$$\tfrac{1}{2}\cdot 0.25\cdot(0.008)^2 = 8\cdot10^{-6}\ \text{m} = \textbf{8 micrometri per passo}$$

Da confrontare con il rumore di processo per passo, $\sqrt{Q_{11}} \approx 0.045$ m.

> **L'errore di linearizzazione è circa 5000 volte più piccolo del rumore di processo.**

Anche con il vecchio $\sigma_\theta = 0.043$ rad restava 200 volte sotto.

### Non linearità 2 — misura di distanza, $d = \|p - p_a\|$

La funzione distanza curva sulla scala di $d$ stesso. L'errore di linearizzazione è dell'ordine di $\tfrac{1}{2}\sigma_\perp^2/d$, con $\sigma_\perp$ l'incertezza perpendicolare alla congiungente. Con $\sigma_\perp \approx 0.2$ m e $d \approx 50$–150 m:

$$\tfrac{1}{2}\cdot\frac{0.04}{50} = 4\cdot10^{-4}\ \text{m}$$

contro $\sigma_{uwb} = 0.5$ m. **Circa 1000 volte sotto il rumore di misura.**

Il ranging diventa fortemente non lineare quando sei **vicino** all'ancora rispetto alla tua incertezza. Tu sei lontanissimo: decine o centinaia di metri, con incertezza decimetrica.

### Conclusione

**Per questo sistema, EKF e UKF darebbero risultati praticamente identici.** L'UKF spenderebbe 11 valutazioni per passo (con $n=5$) più una fattorizzazione di Cholesky di $\Sigma$ a ogni passo, per recuperare una correzione tre ordini di grandezza sotto il rumore.

---

## 2.3 Come giustificarlo bene (e come il README lo giustifica male)

Il README §2.1 attualmente dice:

> *"La scelta dell'EKF rispetto ad altre varianti (come l'UKF) è dettata dall'esigenza di condurre una analisi di osservabilità."*

**Questa motivazione è debole e va cambiata.** Il motivo: l'analisi di osservabilità si fa sul modello linearizzato *comunque*, indipendentemente da quale filtro esegui. Le Jacobiane esistono matematicamente anche se il tuo filtro non le usa. Un esaminatore attento lo vede subito, e ti chiede perché non hai semplicemente calcolato le Jacobiane per l'analisi e usato l'UKF per stimare.

Ecco le giustificazioni solide, in ordine di forza:

**1. Quantitativa — la non linearità è debole, ed è misurata.** I termini del secondo ordine sono 3 ordini di grandezza sotto il pavimento di rumore. È l'argomento più forte perché è un numero, non un'opinione.

**2. Le Jacobiane servono comunque, altrove.** Per il Gramiano di osservabilità, e soprattutto per il calcolo del peso $\omega$ nella **Covariance Intersection**. Avendole già disponibili, l'EKF è la scelta naturale. *(Onestamente: è un argomento "visto che ci sono", non un "devo per forza".)*

**3. Costo computazionale in un contesto distribuito.** Oggi 3 veicoli a 10 Hz sono nulla, ma l'architettura deve scalare a $N$ agenti e alle frequenze reali della Fase 4 — un'IMU può girare a 100-200 Hz. L'EKF costa 1 predizione + 1 Jacobiana; l'UKF costa 11 propagazioni + una Cholesky. Circa 5-10×.

**4. Misure a dimensione variabile.** La tua $C$ cambia dimensione a runtime — GPS acceso/spento, numero variabile di ancore e vicini visibili. Con l'EKF accodi righe e basta. Con l'UKF devi rieseguire la trasformata unscented sulla funzione di misura ogni volta: fattibile, ma il codice si complica.

**5. L'UKF ha parametri di tuning** ($\alpha, \beta, \kappa$) da scegliere e giustificare. L'EKF non ne ha. In un progetto in cui vuoi difendere ogni parametro, meno manopole è meglio.

---

## 2.4 Quando avrei scelto diversamente

Sarebbe onesto — e fa fare bella figura — dire che la scelta è **condizionata**, non assoluta. Avrei scelto l'UKF se:

- avessi un sensore **bearing-only** o visione (fortemente non lineare a corto raggio);
- l'incertezza iniziale fosse **grande**: un'inizializzazione globale con $\sigma$ di decine di metri farebbe curvare $\cos\theta$ sensibilmente entro $\pm3\sigma$;
- avessi un modello **dinamico** con forze cingolo-terreno, dove le Jacobiane sono dolorose e la non linearità è forte;
- **$\sigma_\theta$ diventasse grande.** E qui c'è un avvertimento concreto: se in **Fase 5** il modello di slittamento degradasse sensibilmente la stima di heading — poniamo $\sigma_\theta \to 0.3$ rad — il conto del §2.2 va rifatto, perché l'errore di linearizzazione cresce **quadraticamente** con $\sigma_\theta$. Con $\sigma_\theta = 0.3$ salirebbe di un fattore 1400, arrivando a 0.01 m per passo: non più trascurabile. **È una verifica da rifare dopo la Fase 5**, non un risultato acquisito per sempre.

## 2.5 Cosa considererei davvero, al posto dell'UKF

**Iterated EKF, solo per le misure di distanza.** L'update linearizza attorno a $\bar\mu$; se la predizione è sbagliata, la Jacobiana (la direzione di vista) è calcolata nel punto sbagliato. Iterare — ricalcolare la Jacobiana nel punto aggiornato e rifare l'update 2-3 volte — costa pochissimo e colpisce **esattamente** l'unica misura genuinamente non lineare che hai. È un intervento mirato invece di un cambio di filtro all'ingrosso.

**Invariant EKF (InEKF).** La risposta moderna. La posa dell'uniciclo vive su $SE(2)$, un gruppo di Lie. L'InEKF definisce l'errore **sul gruppo** anziché in coordinate euclidee, e per una classe di sistemi (*group affine*) la dinamica dell'errore diventa **indipendente dallo stato** — la linearizzazione smette di essere un'approssimazione nel senso rilevante, e il filtro acquista proprietà di convergenza e consistenza **dimostrabili**. Risolve in particolare proprio l'*observability mismatch* del Caso E. Riferimento: Barrau & Bonnabel. Citarlo, anche senza implementarlo, mostra che sai dove sta la frontiera.

**Un esperimento consigliato per il report:** implementare l'UKF **come confronto** e mostrare che i due coincidono. Sono ~40 righe, e trasformano un'affermazione ("la non linearità è debole") in un **fatto misurato** ("verificato empiricamente contro un filtro derivative-free"). È il tipo di verifica che sposta il voto sulla voce *quality of the results*, ed è coerente con il metodo già seguito per la validazione Van Loan della $Q$.

---

## Riferimenti

- R. E. Kalman, *"On the General Theory of Control Systems"*, 1960 — criterio di osservabilità.
- S. J. Julier, J. K. Uhlmann, *"Unscented Filtering and Nonlinear Estimation"*, Proc. IEEE 92(3), 2004.
- G. P. Huang, S. I. Roumeliotis, *"Analysis and Improvement of the Consistency of Extended Kalman Filter Based SLAM"*, ICRA 2008 — *observability mismatch*.
- A. Barrau, S. Bonnabel, *"The Invariant Extended Kalman Filter as a Stable Observer"*, IEEE T-AC 62(4), 2017.
