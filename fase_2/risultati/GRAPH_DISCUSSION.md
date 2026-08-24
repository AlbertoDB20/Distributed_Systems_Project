# Analisi dei Risultati di Simulazione: Formazione, Localizzazione e Forze Virtuali

Questo documento analizza le prestazioni di una flotta di **3 veicoli Differential Drive** impegnata in una missione di navigazione in formazione (V-shape) lungo una traiettoria sinusoidale. Il sistema utilizza un Filtro di Kalman Esteso (EKF) per la localizzazione individuale e un approccio a Campi Potenziali Artificiali per il controllo di formazione.

L'esperimento è caratterizzato da un'architettura sensoriale asimmetrica: il veicolo Master (V1) è dotato di un GPS ad alta precisione ($\sigma = 0.2$ m), mentre gli Slave (V2, V3) montano un GPS standard ($\sigma = 2.0$ m).

> **Nota metodologica.** I valori numerici riportati si riferiscono a un singolo run con seed `rng(7)`, sulla versione del simulatore successiva a (a) la correzione dell'ordine temporale del ciclo (vedi [README2.md](README2.md) §2) e (b) la ritaratura dei parametri sulla scala fisica reale di un mezzo battipista (formazione di 36-40 m, $L = 3.5$ m, $b = 2.0$ m, $\omega_{max} = 0.6$ rad/s) e (c) la riformulazione della matrice $Q$ in forma CWNA (README principale, §2.5). Le statistiche "a regime" sono calcolate su $t > 30$ s, escludendo il transitorio di convergenza della formazione. Trattandosi di un run singolo, questi numeri hanno valore **indicativo**: la validazione statistica vera e propria richiede una campagna Monte Carlo con test NEES/NIS, prevista nella fase di validazione finale.

---

## 1. Analisi dell'Errore di Stima (Diagnostica EKF)

I grafici relativi agli errori di stima ($X$, $Y$, $\theta$) mostrano la differenza tra lo stato reale (ground truth) e lo stato stimato dall'EKF per ogni veicolo.

### 1.1 Il Transitorio Iniziale (Convergenza)
In tutti i grafici si nota un picco all'istante $t=0$, seguito da una rapida discesa verso lo zero.

* **Dinamica:** è la risposta del filtro all'errore di inizializzazione introdotto deliberatamente nel codice (sfalsamento di $+1$ m in $X$, $-1$ m in $Y$, $+0.2$ rad in $\theta$), a fronte di una covarianza iniziale $\Sigma_0$ volutamente larga.
* **Velocità di convergenza (errore di posizione sotto 0.5 m):**

  | Veicolo | $\sigma_{GPS}$ | Tempo di convergenza |
  |---|---|---|
  | V1 (Master) | 0.2 m | 0.1 s (un solo passo) |
  | V2 (Slave)  | 2.0 m | 3.7 s |
  | V3 (Slave)  | 2.0 m | 0.6 s |

* **Interpretazione:** la differenza non è casuale, è il guadagno di Kalman al primo aggiornamento. Con $\Sigma_0 = 5$ m² e $R = \sigma_{GPS}^2$, il Master ottiene $K \approx 5/(5+0.04) \approx 0.99$ e cancella quasi integralmente l'errore iniziale in un singolo passo; gli Slave ottengono $K \approx 5/(5+4) \approx 0.55$ e necessitano di più aggiornamenti successivi. La differenza fra V2 e V3 a parità di sensore dipende dalla particolare realizzazione del rumore in questo run.

### 1.2 L'Impatto del GPS Differenziato (Master vs Slave)
Le righe degli errori spaziali ($X$ e $Y$) mostrano chiaramente la natura eterogenea della flotta. Deviazione standard dell'errore a regime:

| Veicolo | $\text{sd}(e_X)$ | $\max\|e_X\|$ | $\text{sd}(e_Y)$ | Copertura $3\sigma$ su $X$ |
|---|---|---|---|---|
| V1 (Master) | 0.066 m | 0.243 m | 0.030 m | 100% |
| V2 (Slave)  | 0.215 m | 0.595 m | 0.069 m | 100% |
| V3 (Slave)  | 0.210 m | 0.574 m | 0.074 m | 100% |

* **Master (V1 — Blu):** l'errore resta confinato in una banda stretta, conseguenza diretta di `sigma_gps_master = 0.2`. Si noti che $\text{sd}(e_X) = 0.066$ m è **inferiore** al rumore del sensore: è l'effetto della fusione, il filtro estrae dalla sequenza di misure un'informazione più accurata della singola lettura.
* **Slave (V2 Rosso, V3 Verde):** l'errore mostra un andamento a frequenza più bassa e ampiezza maggiore. Con `sigma_gps_slave = 2.0` il filtro attribuisce meno peso al GPS e si affida maggiormente all'integrazione cinematica di encoder e IMU, che è accurata sul breve periodo ma soggetta a deriva.

### 1.3 L'uniformità dell'Orientamento ($\theta$)
L'ultima riga di grafici mostra un comportamento **quasi identico** per tutti e tre i veicoli:

| Veicolo | $\text{sd}(e_\theta)$ | $\max\|e_\theta\|$ |
|---|---|---|
| V1 | 0.0079 rad (0.45°) | 0.0254 rad |
| V2 | 0.0077 rad (0.44°) | 0.0281 rad |
| V3 | 0.0079 rad (0.45°) | 0.0264 rad |

* **Perché è uniforme:** il GPS fornisce solo misure di posizione, non di orientamento. L'angolo $\theta$ è osservato dal magnetometro e dal giroscopio dell'IMU, e vincolato indirettamente dagli encoder. Poiché `R_imu` e `R_enc` sono identiche per tutta la flotta, la capacità di stimare $\theta$ è indipendente dal ruolo del veicolo.
* **Verifica quantitativa:** per un random walk scalare con varianza di processo $q = Q_{33}$ e varianza di misura $r = \sigma_\theta^2 = 0.0025$, la soluzione stazionaria dell'equazione di Riccati $\Sigma = (\Sigma+q)r/(\Sigma+q+r)$ fornisce $\sigma_\theta^\infty = \sqrt{\Sigma_\infty}$. Con la formulazione CWNA $Q_{33} = q_\alpha T_s^3/3 = 3.3\cdot10^{-6}$, da cui $\sigma_\theta^\infty \approx 9.5\cdot10^{-3}$ rad. Il valore misurato (0.0079 rad) è coerente con la predizione analitica: **il filtro si comporta come la teoria prevede**, ed è un controllo di correttezza dell'implementazione, non un'osservazione qualitativa.

  Con la $Q$ diagonale precedente si aveva $Q_{33} = 0.01$, cioè $\sigma_\theta^\infty \approx 0.045$ rad — anch'esso confermato dalla misura (0.043 rad). La differenza fra i due casi è tutta nel modello: un rumore di processo su $\theta$ tremila volte superiore al necessario obbligava il filtro a scartare quasi del tutto l'informazione del proprio modello di moto e ad affidarsi al solo magnetometro.

> **Storico — errore di ordine temporale.** Una versione precedente di questo documento riportava per $\theta$ una banda di errore di circa $\pm 0.3$ rad, attribuendola alle caratteristiche della fusione sensoriale. Quella lettura era errata: $\pm 0.3$ rad corrisponde a oltre $6\sigma$ rispetto al rumore del magnetometro e non era spiegabile statisticamente. La causa reale era il disallineamento temporale di un passo fra misura e predizione nel ciclo di simulazione, che produceva un bias sistematico $\omega T_s \approx 0.2$ rad in condizioni di saturazione della velocità angolare. Corretto l'ordine di esecuzione del ciclo, l'errore medio assoluto su $\theta$ è passato da 0.117 rad a 0.034 rad (fattore 3.5) a parità di seed.

---

## 2. Analisi delle Forze Virtuali (Controllo e Sicurezza)

I grafici delle forze virtuali illustrano l'impegno di controllo richiesto per mantenere la formazione e garantire l'assenza di collisioni.

### 2.1 Sforzo di Consenso ($|F_{cons}|$)
Rappresenta la magnitudo del vettore di correzione della velocità generato per mantenere la forma a "V".

* **Il picco iniziale:** nei primi secondi la magnitudo raggiunge 12.4, 23.1 e 10.9 m/s rispettivamente per V1, V2, V3. La flotta è inizializzata in posizioni distanti decine di metri dalla configurazione desiderata (matrice `x0_true`), quindi l'errore di formazione $err_{ij}$ è grande e la legge proporzionale genera un comando di velocità elevato. La formazione rientra entro 1 m dalle distanze nominali a $t = 13.2$ s.
* **Attenzione — limite noto:** questi valori superano la saturazione degli attuatori ($v_{max} = 5$ m/s, $\omega_{max} = 0.6$ rad/s), che risulta attiva per circa il 3% del run — esclusivamente durante il transitorio. In quella fase il comando effettivamente applicato è *tagliato* e la feedback linearization non è più esatta: la velocità realizzata del punto di controllo differisce in direzione, non solo in modulo, da quella comandata. Il sistema converge comunque, ma la convergenza nel transitorio non è garantita dall'analisi lineare del consenso. Una legge di consenso saturata a monte (per esempio normalizzando $F_{cons}$ oltre una soglia) renderebbe il comportamento coerente con la teoria per costruzione.
* **Fase di regime:** raggiunta la formazione, lo sforzo si assesta a 0.031 m/s (V1), 0.042 m/s (V2), 0.044 m/s (V3). Non è mai esattamente nullo: ogni veicolo calcola l'errore di formazione sulla *propria stima rumorosa* della posizione altrui, quindi continua a effettuare micro-correzioni. È il costo di controllo dell'incertezza di stima, ed è coerentemente più alto per gli Slave, che dispongono di stime meno accurate. Il valore è oltre dieci volte inferiore a quello della taratura originaria, per due effetti sommati: la riduzione di `K_cons` da 1.0 a 0.15 e il miglioramento della stima portato dalla $Q$ in forma CWNA. Il secondo è il più istruttivo — **lo sforzo di controllo a regime è una misura indiretta della qualità della stima**: ogni veicolo insegue la posizione *stimata* dei vicini, quindi il rumore di stima si traduce direttamente in comando sprecato.

### 2.1.1 Nota sulla consistenza del filtro
La copertura dei bound a $3\sigma$ risulta del 100% su tutti e tre i veicoli, e il test NEES su 30 run Monte Carlo (`common/verifica_consistenza.m`) fornisce un valore medio di **4.03** contro il valore atteso $E[\text{NEES}] = n = 5$. Il filtro è quindi **conservativo di circa 1.25×**: dichiara un'incertezza leggermente superiore all'errore che effettivamente commette. È la condizione desiderabile per un estimatore di sicurezza — l'alternativa, un filtro ottimista, produrrebbe bound a $3\sigma$ che non contengono l'errore reale.

Va detto che una copertura del 100% è di per sé un indizio di conservativismo: per un filtro perfettamente calibrato ci si attenderebbe circa il 99.7%. Il margine residuo è coerente con il fatto che l'impianto simulato non ha ancora rumore di processo, mentre $Q > 0$: la calibrazione definitiva sarà possibile solo dopo l'introduzione dello slittamento in Fase 5.

### 2.2 Forza Repulsiva ($|F_{rep}|$)
* **Risultato:** con la taratura sulla scala fisica reale il meccanismo **si attiva**. V2 e V3 registrano un picco di 1.79 m/s durante il transitorio, quando le traiettorie di rientro in formazione si incrociano. V1 non lo attiva mai. A regime la curva è nulla per tutti.
* **Dinamica:** `d_safe = 15` m rappresenta l'ingombro fisico reale dei mezzi (~5 m sui cingoli, ~9 m con fresa e lama) più un margine. Restando ampiamente sotto la distanza nominale di formazione (36 m), la repulsione non interferisce con il consenso a regime, ma interviene quando il transitorio comprime la geometria. È il comportamento desiderato: un vincolo di sicurezza deve essere inattivo finché non serve.
* **Nota sulla taratura di $k_{rep}$:** il gradiente del potenziale FIRAS scala come $1/d^3$, quindi il guadagno **non è trasferibile fra scale diverse**. Nel codice non è più un numero fisso ma viene ricavato per inversione da un requisito di progetto ("a $d = d_{safe}/2$ la repulsione vale $v_{rep,ref} = 3$ m/s"), così resta automaticamente coerente se $d_{safe}$ cambia. Con la taratura precedente ($k_{rep} = 2$, valore sensato per $d_{safe} = 1.5$ m) applicata a distanze di decine di metri, la forza repulsiva sarebbe risultata dell'ordine di $10^{-4}$ m/s: numericamente presente, fisicamente inesistente.
* **Limite residuo:** la repulsione agisce sulle posizioni **stimate**. La sicurezza è quindi garantita solo a meno dell'errore di localizzazione, e un margine rigoroso richiederebbe di gonfiare $d_{safe}$ in funzione dell'incertezza $3\sigma$ sulla posizione relativa — tanto più rilevante nelle zone GPS-denied della Fase 3, dove quell'incertezza cresce di un ordine di grandezza.
