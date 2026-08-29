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
   (1) BROADCAST       |--> pubblica p_ctrl(k); i vicini lo ricevono con     ~N scambi
                       |    ritardo, e a volte non lo ricevono affatto
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
| (1) Broadcast | $\hat x_k$ | `p_ctrl`, viste ritardate dei vicini, $G$ | $t_k$ |
| (6) D-WLS | $F_i$, $a_i$, $G$ | $\hat x_{terr}$ | $t_k$ |
| (2) Controllo | $\hat x_k$, stime dei vicini | $u_k$ | $t_k$ |
| (3) Impianto | $x^{true}_k$, $u_k$ | $x^{true}_{k+1}$ | $[t_k, t_{k+1})$ |

Nessun blocco legge una grandezza riferita a un istante successivo al proprio. È la proprietà che rende la simulazione causale.

---

## 4. Come l'Iterazione del Codice si Mappa sull'Istante

Il ciclo `for k = 1:N_steps-1` degli script di simulazione **non coincide** con l'istante $t_k$: è sfalsato di mezzo passo, e questa è la sola vera insidia di lettura del codice.

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
| Broadcast delle pose | ~$N$ | ~5 ms |
| Ranging verso le ancore | fino a 5 | ~5 ms |
| Ranging inter-veicolare | 2 | ~2 ms |
| Cicli di consenso D-WLS | $q \le 50$ | $\le 50$ ms |
| **Margine** | | **~40 ms** |

Il numero di cicli $q$ non è fissato a mano ma dimensionato da $\rho_2$ e dal diametro del grafo, e troncato a questo budget. Sul grafo completo $K_3$ vale $\rho_2 = 0$ e il dimensionamento restituisce $q = 1$: **un solo scambio, il 2% del budget**. Su topologia frammentata cresce, e un indicatore segnala quando il canale non basta più. Vedi [TEORIA_stima_distribuita.md](TEORIA_stima_distribuita.md) §4.1.

---

## 7. L'Età del Dato Scambiato

La posizione di un vicino usata come ancora mobile è la sua stima $\hat x^{(j)}$ contenuta nell'**ultimo pacchetto ricevuto**, propagata in avanti con il modello di moto fino a $t_{k+1}$:

$$\hat p_{k+1}^{(j)} = \hat p_{k-\eta}^{(j)} + \hat v_{k-\eta}^{(j)} \begin{bmatrix}\cos\hat\theta_{k-\eta}^{(j)} \\ \sin\hat\theta_{k-\eta}^{(j)}\end{bmatrix} (\eta+1)\,T_s$$

dove $\eta$ è l'**età** del pacchetto in passi. Fino alla Fase 3 vale $\eta = 0$, cioè un passo di propagazione; in Fase 4 diventa variabile, perché somma il ritardo di consegna e i passi trascorsi dall'ultimo pacchetto arrivato.

Due ragioni per propagare. La prima è strutturale: rompe il **loop algebrico** fra filtri che altrimenti dipenderebbero l'uno dalla stima aggiornata dell'altro nel medesimo istante. La seconda è di coerenza: confrontare una misura acquisita a $t_{k+1}$ con una posizione riferita a un istante precedente reintrodurrebbe lo stesso bias di $|v_j|T_s$ per passo descritto al §5.

**Il consenso di formazione non propaga.** Usa il dato ricevuto così com'è, ed è una scelta: è la condizione in cui vale il limite di stabilità classico per il consenso con ritardo,

$$\tau_d < \frac{\pi}{2\,K_{cons}\,\lambda_{max}(L)}$$

che per la topologia della Fase 4 vale 565 ms. Con un'età media di 100 ms se ne consuma il 18%, con il picco di 400 ms il 71%.

---

## 8. Che Cosa Cambia nelle Fasi Successive

Lo schema qui descritto vale finché reggono due ipotesi, entrambe destinate a cadere.

**Sincronismo dei sensori** — caduto in Fase 4. Fino alla Fase 3 AHRS, encoder, GNSS e UWB campionano tutti a 10 Hz. Dalla Fase 4 ciascuno opera alla propria cadenza, e si scopre che **un solo sensore è davvero più lento del passo**: il GNSS. L'AHRS filtra internamente a 100 Hz e restituisce un assetto già elaborato, gli encoder integrano su una finestra che coincide col passo, il ranging chiude in 9 ms. Il blocco (4) non si sfalda quindi in sotto-blocchi a cadenze diverse: acquisisce sempre tutto, tranne il fix GNSS che arriva un passo su 2 (Master) o su 10 (Slave).

**Latenza nulla del canale** — caduta in Fase 4. Il ritardo di consegna diventa variabile in $[0, 200]$ ms e si aggiunge lo 0.5% di pacchetti persi; le due cose si compongono nell'**età** del dato usato.

> **Non serve il timestamping**, e non è una semplificazione. Vanno distinti due ritardi. Il primo è la *posa dell'ancora vecchia*: la misura di distanza la fa la propria radio adesso ed è fresca, stale è solo la posizione del vicino usata per predirla — si rimedia propagandola in avanti col modello di moto, che è ciò che il §7 già fa per un passo. Il secondo è la *misura fuori sequenza*, una misura del **proprio** stato presa nel passato che arriva adesso: lì servirebbe tornare indietro nel buffer, applicarla al momento giusto e ri-propagare. Il progetto ha solo il primo caso, perché nessun veicolo trasmette misure altrui. E il buffer esiste già: la storia delle stime è in `fleet(j).x_est`, e ricevere con ritardo significa leggerla più indietro.

**Perdite di pacchetto sul consenso** — resta per la Fase 6. Lo 0.5% della Fase 4 riguarda il solo broadcast delle pose a 10 Hz; i cicli di consenso del D-WLS, che vivono sulla scala del millisecondo, sono ancora ideali. In Fase 6 le perdite si estendono a quel canale e frammentano la topologia, obbligando a rileggere il consenso in termini di **connettività congiunta**.

---

## Riferimenti

* README.md §2.4 — convenzione temporale e ordine di esecuzione.
* fase_3/README3.md §2.1 — applicazione alla Fase 3.
* *Intelligent Distributed Systems*, Cap. 17 e 18 — consenso lineare e stima distribuita.
