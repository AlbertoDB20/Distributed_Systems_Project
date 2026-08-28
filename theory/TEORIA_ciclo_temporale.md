# Scansione Temporale di un Passo di Campionamento

Che cosa accade, e in quale ordine, dentro un intervallo $T_s = 100$ ms. Note collegate: [TEORIA_consenso_su_grafi.md](TEORIA_consenso_su_grafi.md), [TEORIA_stima_distribuita.md](TEORIA_stima_distribuita.md), [TEORIA_osservabilita_e_filtro.md](TEORIA_osservabilita_e_filtro.md).

---

## 1. Due Scale dei Tempi, non Una

L'errore di lettura più comune è immaginare che tutto avvenga alla stessa cadenza. Non è così: il sistema vive su **due scale separate da due ordini di grandezza**.

| | Cadenza | Periodo | Chi ci vive |
|---|---|---|---|
| **Controllo** | 10 Hz | 100 ms | EKF, legge di formazione, evoluzione dell'impianto |
| **Radio** | ~1 kHz | ~1 ms | broadcast delle pose, ranging UWB, cicli di consenso |

Uno scambio TW-TOF su DW1000 dura circa 1 ms. In un solo passo di controllo il canale sostiene quindi **un centinaio di scambi**, ed è questa disparità che rende possibile eseguire un round completo di stima distribuita a ogni campione anziché ogni tanto.

---

## 2. Che Cosa Accade nell'Istante $t_k$

```
   CONTROLLO          t_k                                              t_(k+1)
     10 Hz             |<------------- Ts = 100 ms ------------------->|
                       |                                               |
                       |   IMPIANTO: x_true evolve con u_k tenuto      |
                       |   costante (attuatore ZOH)                    |
                       |   x_true(k) ============================>  x_true(k+1)
                       |                                               |
                       v                                               v
        +-----------------------------------+              +--------------------+
        |  tutto quanto segue accade "in    |              |  si ripete in      |
        |  t_k", entro pochi millisecondi   |              |  t_(k+1)           |
        +-----------------------------------+              +--------------------+
                       |
                       |   RADIO ~1 kHz, ~1 ms per scambio
                       |
   (4) SENSORI         |--> z_k = h(x_true(k)) + rumore
                       |    IMU, encoder, GPS oppure ancore UWB e vicini
                       |
   (5) STIMA           |--> EKF: predice da x_est(k-1), corregge con z_k
                       |    ==> x_est(k), Sigma(k)
                       |
   (4b) TORSIOMETRO    |--> z_traz(k), poi F_loc += C'R^-1 C
                       |                     a_loc += C'R^-1 z
                       |
   (1) BROADCAST       |--> pubblica p_ctrl(k) e la posa propagata      ~3 scambi
                       |    costruisce il grafo G(k) dalla portata radio
                       |
   (6) D-WLS           |--> q cicli di consenso su (F_loc, a_loc)       q <= 50 scambi
                       |    ==> stima del terreno, uguale per tutti
                       |
   (2) CONTROLLO       |--> u_k = g( x_est(k), stime dei vicini )
                       |    consenso + repulsione + riferimento, poi T_fl^-1
                       |
                       v
                     u_k applicato sull'intervallo [t_k, t_(k+1))
```

**Il punto che il disegno rende evidente:** i blocchi da (4) a (2) sono tutti *istantanei* rispetto alla scala del controllo. L'unica cosa che occupa davvero i 100 ms è l'evoluzione fisica dell'impianto.

---

## 3. Chi Legge Che Cosa, e a Quale Istante

| Blocco | Legge | Scrive | Riferito a |
|---|---|---|---|
| (4) Sensori | $x^{true}_k$ | $z_k$ | $t_k$ |
| (5) Stima | $\hat x_{k-1}$, $\Sigma_{k-1}$, $z_k$ | $\hat x_k$, $\Sigma_k$ | $t_k$ |
| (4b) Torsiometro | $x^{true}_k$, $\hat v_k$ | $F_i$, $a_i$ | $t_k$ |
| (1) Broadcast | $\hat x_k$ | `p_ctrl`, `p_ancora_mobile`, $G$ | $t_k$ |
| (6) D-WLS | $F_i$, $a_i$, $G$ | $\hat x_{terr}$ | $t_k$ |
| (2) Controllo | $\hat x_k$, stime dei vicini | $u_k$ | $t_k$ |
| (3) Impianto | $x^{true}_k$, $u_k$ | $x^{true}_{k+1}$ | $[t_k, t_{k+1})$ |

Nessun blocco legge una grandezza riferita a un istante successivo al proprio. È la proprietà che rende la simulazione causale.

---

## 4. Come l'Iterazione del Codice si Mappa sull'Istante

Il ciclo `for k = 1:N_steps-1` di `main3.m` **non coincide** con l'istante $t_k$: è sfalsato di mezzo passo, e questa è la sola vera insidia di lettura del codice.

```
   iterazione k-1                iterazione k                 iterazione k+1
 ...--------------|============================|-------------------------...
                  |                            |
                  |   t_k                      |   t_(k+1)
                  |                            |
   chiude a t_k:  |  apre a t_k:               |  chiude a t_(k+1):
   (4) sensori    |  (1) broadcast             |  (4) sensori
   (5) stima      |  (2) controllo             |  (5) stima
   (4b) torsiom.  |  (3) impianto -> t_(k+1)   |  ...
   (6) D-WLS      |                            |
```

L'istante $t_k$ è quindi servito da **due metà di iterazioni diverse**: la coda dell'iterazione $k-1$ produce $\hat x_k$, la testa dell'iterazione $k$ lo consuma per il broadcast e il controllo. È solo una conseguenza di come è indicizzato il ciclo, non un ritardo aggiuntivo.

Una verifica immediata: nel codice il broadcast dell'iterazione $k$ legge `fleet(i).x_est(:, k)`, che è stato scritto in fondo all'iterazione $k-1$.

---

## 5. Perché Quest'Ordine e non un Altro

Due vincoli lo determinano, entrambi già verificati sui numeri.

**Causalità del controllo.** Il comando che agisce su $[t_k, t_{k+1})$ può dipendere solo da $\hat x_k$. Retroazionare $\hat x_{k+1}$ — la stima prodotta *dopo* che il comando ha già agito — significherebbe concedere al controllore la conoscenza del futuro.

**Coerenza temporale dell'innovazione.** L'innovazione $z_{k+1} - h(\hat x_{k+1|k})$ ha senso statistico solo se misura e predizione si riferiscono al medesimo istante. Un disallineamento di un solo passo produce un errore **deterministico** di $|v|T_s$ sulla posizione e $|\omega|T_s$ sull'heading: con $\omega$ in saturazione e $T_s = 0.1$ s vale 0.06 rad, un ordine di grandezza sopra il rumore del magnetometro. Trattandosi di un bias e non di rumore non comparirebbe in $\Sigma$, e il filtro resterebbe convinto di essere accurato mentre è sistematicamente in ritardo.

La correzione di questo ordinamento, all'inizio del progetto, ha portato il MAE sull'heading da 0.117 a 0.034 rad.

**Una conseguenza sulla struttura del codice.** I blocchi (2)-(3) e (4)-(5) sono due passate distinte sull'intera flotta: la ground truth di tutti i mezzi deve esistere a $t_{k+1}$ prima che uno qualsiasi generi le proprie misure, perché i range inter-veicolari dipendono dalla posizione reale dei vicini.

---

## 6. Il Budget Radio

Metà del passo è riservata al D-WLS, l'altra metà a tutto il resto.

| Traffico | Scambi | Tempo |
|---|---|---|
| Broadcast delle pose | ~3 | ~3 ms |
| Ranging verso le ancore | fino a 5 | ~5 ms |
| Ranging inter-veicolare | 2 | ~2 ms |
| Cicli di consenso D-WLS | $q \le 50$ | $\le 50$ ms |
| **Margine** | | **~40 ms** |

Il numero di cicli $q$ non è fissato a mano ma dimensionato da $\rho_2$ e dal diametro del grafo, e troncato a questo budget. Sul grafo completo $K_3$ vale $\rho_2 = 0$ e il dimensionamento restituisce $q = 1$: **un solo scambio, il 2% del budget**. Su topologia frammentata cresce, e un indicatore segnala quando il canale non basta più. Vedi [TEORIA_stima_distribuita.md](TEORIA_stima_distribuita.md) §4.1.

---

## 7. Il Ritardo di un Passo sulle Stime Scambiate

La posizione di un vicino usata come ancora mobile è la sua stima $\hat x_k^{(j)}$ — l'ultimo pacchetto ricevuto — **propagata di un passo** con il modello di moto:

$$\hat p_{k+1|k}^{(j)} = \hat p_k^{(j)} + \hat v_k^{(j)} \begin{bmatrix}\cos\hat\theta_k^{(j)} \\ \sin\hat\theta_k^{(j)}\end{bmatrix} T_s$$

Due ragioni. La prima è strutturale: rompe il **loop algebrico** fra filtri che altrimenti dipenderebbero l'uno dalla stima aggiornata dell'altro nel medesimo istante. La seconda è di realismo: è ciò che un canale reale rende effettivamente disponibile.

La propagazione è necessaria perché confrontare una misura acquisita a $t_{k+1}$ con una posizione riferita a $t_k$ reintrodurrebbe lo stesso bias di $|v_j|T_s$ descritto al §5.

---

## 8. Che Cosa Cambia nelle Fasi Successive

Lo schema qui descritto vale finché reggono due ipotesi, entrambe destinate a cadere.

**Sincronismo dei sensori.** Oggi IMU, encoder, GPS e UWB campionano tutti a 10 Hz. In **Fase 4** operano alle frequenze reali — un'IMU può arrivare a 100–200 Hz, un GPS sta sotto i 10 Hz — e il blocco (4) si sfalda in più sotto-blocchi a cadenze diverse. È il motivo per cui la $Q$ è stata scritta in forma CWNA, proporzionale a $T_s$: cambiare cadenza non deve ritarare il filtro in silenzio.

**Latenza nulla del canale.** Oggi il traffico radio è istantaneo sulla scala del controllo. In **Fase 4** la latenza diventa esplicita e variabile, e serve il *timestamping* delle misure: una lettura in ritardo va reinserita nel punto giusto della storia del filtro. In **Fase 6** si aggiungono le perdite di pacchetto, che rendono la topologia tempo-variante e obbligano a rileggere il consenso in termini di connettività congiunta.

---

## Riferimenti

* README.md §2.4 — convenzione temporale e ordine di esecuzione.
* fase_3/README3.md §2.1 — applicazione alla Fase 3.
* *Intelligent Distributed Systems*, Cap. 17 e 18 — consenso lineare e stima distribuita.
