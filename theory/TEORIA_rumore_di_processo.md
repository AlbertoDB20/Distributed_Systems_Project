# Il Rumore di Processo e la Formulazione CWNA
### Nota teorica di approfondimento — come e perché si costruisce $Q$

Accompagna `calcola_Q_cwna.m`. Costruita dai concetti generali fino alle specificità di questo progetto.

---

## 1. Cos'è $Q$, in parole semplici

Un filtro di Kalman è una **trattativa fra due fonti di informazione**:

1. *Quello che il modello prevede* — "andavo a 2.5 m/s verso nord, un decimo di secondo fa; quindi ora dovrei essere 25 cm più avanti".
2. *Quello che i sensori misurano* — "il GPS dice che sono qui".

Nessuna delle due è affidabile. Il filtro deve decidere **a chi credere e quanto**, e questo lo decidono due matrici:

| | Risponde a | Se è grande… |
|---|---|---|
| $R$ | *Quanto sbagliano i miei sensori?* | il filtro si fida poco delle misure |
| $Q$ | *Quanto sbaglia il mio modello?* | il filtro si fida poco di sé stesso |

$Q$ **non è rumore in senso fisico**: è la dichiarazione formale della propria ignoranza. Dice: *"in un intervallo di tempo, ecco di quanto la realtà può discostarsi dalle mie equazioni"*.

### Perché $Q$ non può essere zero

Questo è il punto che di solito chiarisce tutto. Guarda il ciclo:

$$\underbrace{\bar\Sigma_k = A\,\Sigma_{k-1}A^T + Q}_{\text{predizione: l'incertezza CRESCE}} \qquad\qquad \underbrace{\Sigma_k = (I - K C)\,\bar\Sigma_k}_{\text{correzione: l'incertezza CALA}}$$

Con $Q = 0$ l'incertezza cala e basta. $\Sigma \to 0$, quindi $K = \Sigma C^T S^{-1} \to 0$, quindi il filtro **smette di ascoltare i sensori**. Diventa sordo: convinto di sapere già tutto, ignora le misure e va alla deriva per sempre.

$Q$ è il meccanismo che tiene viva l'incertezza. È l'ammissione, ripetuta a ogni passo, che il tempo passa e il mondo si allontana dalle equazioni.

Il punto di equilibrio fra le due frecce determina $K$, cioè **quanta parte di ogni misura entra nella stima**:

- $Q$ troppo grande → il filtro diffida del proprio modello → si appoggia troppo a sensori rumorosi → stima nervosa, l'informazione del modello viene sprecata.
- $Q$ troppo piccola → il filtro si fida troppo del modello → ignora le misure → deriva, e nei casi estremi diverge.

---

## 2. Cosa c'era prima, e i due errori distinti

```matlab
Q = diag([0.01, 0.01, 0.01, 0.25, 0.04]);   % sommata a ogni passo, a 10 Hz
```

### Errore A — non scala con $T_s$

$Q$ rappresenta rumore **accumulato in un intervallo di tempo**. Raddoppiando l'intervallo dovrebbe raddoppiare la varianza accumulata. Qui invece è un numero fisso sommato a ogni passo, quindi:

- a 10 Hz inietti quel numero 10 volte al secondo;
- a 100 Hz lo inietti 100 volte al secondo.

**Stessa fisica, filtro completamente diverso, e senza che nessuno se ne accorga.**

L'analogia: è come dire *"perdo 1 cm di precisione a ogni misura"* invece che *"al secondo"*. Se misuri più spesso non peggiori — ma questa formulazione afferma di sì.

Ed è un problema bloccante per la Fase 4, che è multi-rate per definizione.

### Errore B — la struttura diagonale afferma una cosa falsa

Una $Q$ diagonale dichiara: *gli errori sulle cinque componenti di stato crescono in modo indipendente*. Ma guarda il modello:

$$x_{k+1} = x_k + v_k\cos(\theta_k)\,T_s$$

**La posizione non ha dinamica propria.** Cambia soltanto perché $v$ e $\theta$ sono incerti. Ne seguono due conseguenze:

1. Iniettare rumore direttamente su $x$ e $y$ significa affermare che il veicolo **si sposta lateralmente da fermo**. Fisicamente non accade.
2. L'errore su $x$ e l'errore su $v$ **non sono indipendenti**: se sovrastimo la velocità, un istante dopo abbiamo anche sovrastimato la posizione. Sono legati per costruzione.

E attenzione: mettere zero fuori diagonale **non è una scelta neutra**. Non è "non lo so", è "**affermo che la correlazione è nulla**". È una dichiarazione, ed è falsa. Vedremo fra poco che la correlazione vera vale 0.87.

---

## 3. L'idea del CWNA: mettere il rumore dove sta davvero l'ignoranza

La domanda giusta da porsi è: **di questo veicolo, cosa non so davvero?**

Non la posizione — quella discende dalla cinematica. Nemmeno la velocità in sé. Quello che non so è **come la velocità cambierà**, perché su neve il cingolo slitta e il mezzo accelera diversamente da come è stato comandato.

Quindi il rumore va messo sull'**accelerazione**. È lì la sede fisica dell'ignoranza — ed è esattamente lì che agisce lo slittamento.

Da qui il nome: **CWNA — Continuous White Noise Acceleration**.
- *Continuous*: il rumore è un processo a tempo continuo, non una spintarella una volta per passo.
- *White*: scorrelato nel tempo, ogni istante è indipendente dal precedente.
- *Acceleration*: agisce sull'accelerazione.

---

## 4. Come si propaga: dall'accelerazione alla posizione

Questo è il cuore della costruzione, e si capisce con la cinematica del liceo.

Immagina che durante un intervallo $T_s$ agisca un'accelerazione ignota $a$. Cosa succede?

$$\Delta v = a\,T_s \qquad\qquad \Delta p = \tfrac{1}{2}a\,T_s^2$$

**Una sola causa ignota, due effetti — e sono legati fra loro.** Non sono due errori separati: sono lo *stesso* errore visto in due modi. Ecco perché posizione e velocità non possono avere errori indipendenti.

### Il calcolo esatto

Se $a$ è rumore bianco continuo con densità spettrale $q$, cioè $E[w(t)w(\tau)] = q\,\delta(t-\tau)$:

$$\Delta v(T_s) = \int_0^{T_s}\! w(\tau)\,d\tau \;\Longrightarrow\; \text{var}(\Delta v) = q\,T_s$$

$$\Delta p(T_s) = \int_0^{T_s}\!\!\int_0^{s}\! w(\tau)\,d\tau\,ds = \int_0^{T_s}\!(T_s-\tau)\,w(\tau)\,d\tau \;\Longrightarrow\; \text{var}(\Delta p) = \int_0^{T_s}\!(T_s-\tau)^2 q\,d\tau = \frac{q\,T_s^3}{3}$$

$$\text{cov}(\Delta p, \Delta v) = \int_0^{T_s}\!(T_s-\tau)\,q\,d\tau = \frac{q\,T_s^2}{2}$$

Da cui il **blocco canonico a velocità costante**, che si ritrova identico in ogni testo di tracking:

$$\boxed{\;Q_{1D} = q\begin{bmatrix} T_s^3/3 & T_s^2/2 \\[0.3ex] T_s^2/2 & T_s \end{bmatrix}\;}$$

(Bar-Shalom, Li, Kirubarajan, *Estimation with Applications to Tracking and Navigation*, cap. 6.)

### Quanto vale davvero quella correlazione

$$\rho = \frac{\text{cov}(\Delta p,\Delta v)}{\sigma_p\,\sigma_v} = \frac{q T_s^2/2}{\sqrt{q T_s^3/3}\cdot\sqrt{q T_s}} = \frac{T_s^2/2}{T_s^2/\sqrt3} = \frac{\sqrt3}{2} \approx 0.866$$

**Gli errori di posizione e velocità sono correlati all'87%.** La $Q$ diagonale precedente affermava che quella correlazione fosse zero. Non è un dettaglio: è informazione fisica gratuita che il filtro veniva costretto a buttare via.

---

## 5. Dal caso 1D al nostro veicolo

Il veicolo non si muove su una retta, ma c'è una semplificazione naturale: **la velocità $v$ è diretta lungo l'heading**. Quindi la coppia (*posizione lungo la direzione di marcia*, $v$) è esattamente il blocco 1D appena ricavato.

Resta solo da **proiettare** la posizione lungo-marcia sugli assi $x$ e $y$, tramite il versore $[\cos\theta,\ \sin\theta]$. Proiettare una varianza scalare $\sigma^2$ lungo una direzione dà:

$$\sigma^2\begin{bmatrix}\cos^2\theta & \cos\theta\sin\theta \\ \cos\theta\sin\theta & \sin^2\theta\end{bmatrix}$$

una matrice di **rango 1**: un'ellisse degenere, schiacciata sulla sola direzione di marcia.

Ed ecco spiegate le righe del codice:

```matlab
Q_d(1,1) = q_a * T3 * c*c;    % varianza su x        <- proiezione di q_a*T3
Q_d(1,2) = q_a * T3 * c*s;    % covarianza x-y       <- la stessa, incrociata
Q_d(2,2) = q_a * T3 * s*s;    % varianza su y
Q_d(1,4) = q_a * T2 * c;      % CORRELAZIONE x-v     <- il termine che prima mancava
Q_d(2,4) = q_a * T2 * s;      % CORRELAZIONE y-v
Q_d(4,4) = q_a * T1;          % varianza su v        <- nessuna proiezione: v e' scalare
```

### Il canale angolare: identico

Stessa struttura, con $(\theta, \omega)$ al posto di (posizione, $v$): $\theta$ fa da "posizione" e $\omega$ da sua derivata. Nessuna proiezione, perché $\theta$ è già scalare.

```matlab
Q_d(3,3) = par.q_alpha * T3;   % varianza su theta
Q_d(3,5) = par.q_alpha * T2;   % correlazione theta-omega
Q_d(5,5) = par.q_alpha * T1;   % varianza su omega
```

---

## 6. Il canale laterale: perché serve davvero

Qui c'è la parte più sottile, e vale la pena capirla bene perché è il tipo di dettaglio su cui un esaminatore si ferma.

Con i due soli canali visti sopra, **$Q_d$ è singolare**. Perché?

Il canale longitudinale alimenta la posizione soltanto **lungo** la direzione di marcia. Nulla alimenta la posizione **perpendicolare**. Il filtro sta quindi affermando: *"posso essere incerto su quanto sono avanzato, ma so con precisione assoluta la mia posizione laterale."*

**Per un uniciclo ideale questo è vero.** L'uniciclo non può traslare di lato — è il vincolo anolonomo. L'errore laterale non può crescere per rumore di processo, perché il modello non prevede alcun moto laterale.

**Per un battipista su un pendio innevato è falso.** Il mezzo *scivola di traverso*, eccome. È un moto reale che il modello uniciclo, strutturalmente, non sa descrivere.

Quindi `q_lat` **non è un aggiustamento numerico**. È la dichiarazione onesta: *"esiste una quantità di moto che il mio modello non può rappresentare, ed ecco quanto vale."*

Entra come rumore di **velocità** e non di accelerazione — non esiste uno stato "velocità laterale" a cui agganciarlo — quindi contribuisce con $q_{lat}T_s$ direttamente sulla posizione, lungo il versore perpendicolare $[-\sin\theta,\ \cos\theta]$.

### La conseguenza numerica, verificata

| | autovalore minimo di $Q_d$ | rango |
|---|---|---|
| senza canale laterale | $2.6\cdot10^{-21}$ | **4** su 5 |
| con canale laterale | $8.3\cdot10^{-7}$ | **5** su 5 |

Senza quel canale, se il veicolo procedesse in rettilineo abbastanza a lungo l'incertezza laterale potrebbe solo diminuire, mai crescere: $\Sigma$ diventa mal condizionata e il filtro numericamente fragile.

---

## 7. Perché va ricalcolata a ogni passo

Perché **dipende da $\theta$**.

L'immagine da tenere a mente: la $Q$ del CWNA è un'**ellisse di incertezza allungata nella direzione di marcia**, che ruota insieme al veicolo. Un rettangolo fisso — la diagonale costante — non può fare questo, per quanto lo si tari bene.

```
       direzione di marcia
              ↗
         .-'''''-.          l'incertezza cresce molto lungo la marcia
       (           )        (non so bene quanto sono avanzato)
        '-.......-'         e poco di traverso (l'uniciclo non slitta)
```

Se attivi `k_terreno`, dipende anche da $v$, e va ricalcolata a maggior ragione.

---

## 8. I numeri di questo progetto

I parametri sono densità spettrali, e si leggono così: **dopo un tempo $T$ di sola predizione, l'incertezza accumulata vale $\sqrt{q\,T}$**.

| Parametro | Valore | Unità | Significato fisico | Dopo 1 s di sola predizione |
|---|---|---|---|---|
| `q_a` | 0.10 | m²/s³ | accel. longitudinale — slittamento in trazione | $\sigma_v = 0.32$ m/s |
| `q_alpha` | 0.01 | rad²/s³ | accel. angolare — slittamento in sterzata | $\sigma_\omega = 0.10$ rad/s |
| `q_lat` | 0.02 | m²/s | deriva laterale su pendio | 0.14 m di scarto |
| `k_terreno` | 0 | 1/s | $q_a \leftarrow q_a + k\,v^2$ | inattivo fino alla Fase 5 |

### Confronto con la formulazione precedente, a $T_s = 0.1$ s

| Elemento | Prima | Ora (CWNA) | Rapporto |
|---|---|---|---|
| $Q_{33}$ (heading) | $10^{-2}$ | $q_\alpha T_s^3/3 = 3.3\cdot10^{-6}$ | **3000×** |
| $Q_{55}$ ($\omega$) | $4\cdot10^{-2}$ | $q_\alpha T_s = 10^{-3}$ | 40× |
| $Q_{44}$ ($v$) | $0.25$ | $q_a T_s = 10^{-2}$ | 25× |
| $Q_{11}$ (posizione) | $10^{-2}$ | $\le 2\cdot10^{-3}$, dipende da $\theta$ | ~5× |
| $Q_{14}$ (correl. pos-vel) | **0** | $5\cdot10^{-4}\cos\theta$ | *non esisteva* |

Il fattore 3000 su $Q_{33}$ spiega da solo il miglioramento sull'heading: un rumore di processo tremila volte superiore al necessario obbligava il filtro a scartare quasi del tutto il proprio modello di moto e ad affidarsi al solo magnetometro.

### La legge adattiva

$$q_a(v) = q_{a0} + k_{terreno}\,v^2$$

Più il mezzo corre su fondo cedevole, più incertezza viene iniettata. È la promessa fatta in §1 del README principale, che **solo la formulazione CWNA rende esprimibile**: con una $Q$ diagonale non c'era nessun posto fisicamente sensato in cui infilare la dipendenza dalla velocità. Ora c'è, ed è una riga.

Resta a zero finché l'impianto non simula uno slittamento reale contro cui calibrarla (Fase 5).

---

## 9. La validazione: metodo di Van Loan

Nella derivazione abbiamo fatto **una sola approssimazione**: abbiamo congelato $\theta$ sull'intervallo di campionamento. Va verificato che sia benigna.

Il metodo di **Van Loan (1978)** calcola la discretizzazione **esatta**, senza congelare nulla, tramite un singolo esponenziale di matrice su un sistema aumentato $10\times10$:

$$\Psi = \begin{bmatrix} -A & \Gamma Q_c\Gamma^T \\ 0 & A^T\end{bmatrix}T_s, \qquad \Phi = e^{\Psi}, \qquad A_d = \Phi_{22}^T, \qquad Q_d = A_d\,\Phi_{12}$$

Confronto (`verifica_Q_cwna.m`):

| $T_s$ | errore relativo |
|---|---|
| **0.1 s** (operativo) | $4\cdot10^{-4}$ |
| 0.5 s | $1\cdot10^{-2}$ |
| 1.0 s | $1.1\cdot10^{-1}$ |

L'errore cresce come $T_s^2$, coerente con un'approssimazione del secondo ordine. Alla frequenza operativa è irrilevante. **Nota per la Fase 4:** se un ramo di predizione dovesse operare a passi molto più lunghi, converrà passare direttamente a Van Loan.

---

## 10. Cosa è effettivamente cambiato

Campagna Monte Carlo su Fase 2, 15 run, a regime:

| | NEES | RMSE posizione | RMSE $v$ | RMSE $\theta$ |
|---|---|---|---|---|
| $Q$ diagonale costante | 3.95 | 0.376 m | 0.0350 m/s | 0.0420 rad |
| $Q$ in forma CWNA | 4.03 | **0.200 m** | 0.0318 m/s | **0.0082 rad** |

**La consistenza non cambia.** Entrambe le formulazioni sono ugualmente calibrate, conservative di circa 1.25× rispetto a $E[\text{NEES}] = n = 5$.

Questo risultato è controintuitivo ma ha una spiegazione precisa: il NEES misura il **rapporto** fra errore reale e covarianza dichiarata. Una $Q$ sovradimensionata gonfia **entrambi** — il filtro sbaglia di più *e* dichiara di sbagliare di più — quindi il rapporto resta lo stesso.

Ciò che il CWNA corregge non è la calibrazione, è **quanta informazione il modello di processo mette a disposizione del filtro**. Accuratezza 1.9× in posizione, 5.1× in heading.

Se all'esame arriva la domanda *"quindi la $Q$ vecchia era inconsistente?"* — la risposta onesta è **no, era inefficiente**.

> **Avvertenza di taratura.** La ground truth attuale ha rumore di processo *nullo* (tracking ideale degli attuatori). I valori di $q_a$ e $q_\alpha$ sono deliberatamente conservativi rispetto all'impianto — il filtro risulta pessimista, non ottimista, che è la condizione sicura — e parte del guadagno misurato deriva dal fatto che una $Q$ più piccola è semplicemente più vicina a un impianto che di rumore non ne ha affatto. La calibrazione onesta arriverà in Fase 5. I benefici *strutturali* (scalatura su $T_s$, correlazioni, non singolarità, parametrizzazione fisica) sono invece incondizionati.

---

## 11. Domande probabili all'orale

**"Perché $Q$ non è diagonale?"**
Perché nel modello uniciclo la posizione non ha dinamica propria: cambia solo attraverso $v$ e $\theta$. Un'unica accelerazione ignota produce simultaneamente un errore di velocità e uno di posizione, legati fra loro. La correlazione vale $\sqrt3/2 \approx 0.87$; una diagonale afferma che sia zero.

**"Perché $Q$ dipende da $\theta$?"**
Perché l'incertezza è un'ellisse allungata lungo la direzione di marcia, e ruota con il veicolo. La velocità è diretta lungo l'heading, quindi l'incertezza che genera si proietta su $x$ e $y$ tramite $\cos\theta$ e $\sin\theta$.

**"Perché $T_s^3/3$ e non $T_s^4/4$?"**
Perché $T_s^4/4$ è il risultato del modello **DWNA** (*Discrete White Noise Acceleration*), che assume un'accelerazione casuale **costante** entro ciascun intervallo. Il CWNA assume rumore bianco **continuo**, e integrando propriamente si ottiene $T_s^3/3$. Cambiano anche le unità: $\sigma_a^2$ è in m²/s⁴ nel DWNA, $q$ è in m²/s³ nel CWNA. Il CWNA è preferibile qui perché la frequenza di campionamento è una scelta dell'estimatore, non una proprietà fisica del disturbo — e in Fase 4 quella frequenza cambierà.

**"Cosa succede se sbaglio $Q$?"**
Troppo grande: il filtro diffida del proprio modello, si appoggia a sensori rumorosi, stima nervosa e informazione sprecata. Troppo piccola: si fida troppo del modello, ignora le misure, deriva e nei casi estremi diverge. Con $Q = 0$ esattamente, $\Sigma \to 0$, $K \to 0$ e il filtro diventa sordo.

**"A cosa serve il canale laterale?"**
A rappresentare ciò che il modello non sa rappresentare. L'uniciclo è anolonomo e non prevede traslazione laterale, quindi senza quel canale $Q_d$ ha rango 4 su 5 e l'incertezza perpendicolare alla marcia non cresce mai. È anche il primo candidato a essere sostituito da un modello esplicito di slittamento in Fase 5.

**"Come hai verificato che la formula sia giusta?"**
Contro il metodo di Van Loan, che dà la discretizzazione esatta senza congelare $\theta$. A $T_s = 0.1$ s l'errore relativo è $4\cdot10^{-4}$ e cresce come $T_s^2$.

---

## Riferimenti

- Y. Bar-Shalom, X.-R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley 2001 — cap. 6, modelli CWNA e DWNA.
- C. F. Van Loan, *"Computing Integrals Involving the Matrix Exponential"*, IEEE T-AC 23(3), 1978 — discretizzazione esatta.
- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press 2005 — notazione adottata nel progetto.
