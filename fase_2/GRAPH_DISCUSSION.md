# Analisi dei Risultati di Simulazione: Formazione, Localizzazione e Forze Virtuali

Questo documento analizza le prestazioni di una flotta di **3 veicoli Differential Drive** impegnata in una missione di navigazione in formazione (V-shape) lungo una traiettoria sinusoidale. Il sistema utilizza un Filtro di Kalman Esteso (EKF) per la localizzazione individuale e un approccio a Campi Potenziali Artificiali per il controllo di formazione.

L'esperimento è caratterizzato da un'architettura sensoriale asimmetrica: il veicolo Master (V1) è dotato di un GPS ad alta precisione ($\sigma = 0.2$ m), mentre gli Slave (V2, V3) montano un GPS standard ($\sigma = 2.0$ m).

> **Nota metodologica.** I valori numerici riportati si riferiscono a un singolo run con seed `rng(7)`, sulla versione del simulatore successiva a (a) la correzione dell'ordine temporale del ciclo (vedi [README2.md](README2.md) §2) e (b) la ritaratura dei parametri sulla scala fisica reale di un mezzo battipista (formazione di 36-40 m, $L = 3.5$ m, $b = 2.0$ m, $\omega_{max} = 0.6$ rad/s). Le statistiche "a regime" sono calcolate su $t > 30$ s, escludendo il transitorio di convergenza della formazione. Trattandosi di un run singolo, questi numeri hanno valore **indicativo**: la validazione statistica vera e propria richiede una campagna Monte Carlo con test NEES/NIS, prevista nella fase di validazione finale. I valori andranno inoltre rigenerati dopo la riformulazione della matrice $Q$ in forma CWNA.

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

| Veicolo | $\text{sd}(e_X)$ | $\max\|e_X\|$ | $\text{sd}(e_Y)$ |
|---|---|---|---|
| V1 (Master) | 0.098 m | 0.351 m | 0.099 m |
| V2 (Slave)  | 0.335 m | 1.005 m | 0.294 m |
| V3 (Slave)  | 0.321 m | 1.043 m | 0.292 m |

* **Master (V1 — Blu):** l'errore resta confinato in una banda stretta, conseguenza diretta di `sigma_gps_master = 0.2`. Si noti che $\text{sd}(e_X) = 0.098$ m è **inferiore** al rumore del sensore: è l'effetto della fusione, il filtro estrae dalla sequenza di misure un'informazione più accurata della singola lettura.
* **Slave (V2 Rosso, V3 Verde):** l'errore mostra un andamento a frequenza più bassa e ampiezza maggiore. Con `sigma_gps_slave = 2.0` il filtro attribuisce meno peso al GPS e si affida maggiormente all'integrazione cinematica di encoder e IMU, che è accurata sul breve periodo ma soggetta a deriva.

### 1.3 L'uniformità dell'Orientamento ($\theta$)
L'ultima riga di grafici mostra un comportamento **quasi identico** per tutti e tre i veicoli:

| Veicolo | $\text{sd}(e_\theta)$ | $\max\|e_\theta\|$ |
|---|---|---|
| V1 | 0.0423 rad (2.4°) | 0.141 rad |
| V2 | 0.0429 rad (2.5°) | 0.151 rad |
| V3 | 0.0421 rad (2.4°) | 0.141 rad |

* **Perché è uniforme:** il GPS fornisce solo misure di posizione, non di orientamento. L'angolo $\theta$ è osservato dal magnetometro e dal giroscopio dell'IMU, e vincolato indirettamente dagli encoder. Poiché `R_imu` e `R_enc` sono identiche per tutta la flotta, la capacità di stimare $\theta$ è indipendente dal ruolo del veicolo.
* **Verifica quantitativa:** per un random walk scalare con varianza di processo $q = Q_{33} = 0.01$ e varianza di misura $r = \sigma_\theta^2 = 0.0025$, la soluzione stazionaria dell'equazione di Riccati $\Sigma = (\Sigma+q)r/(\Sigma+q+r)$ fornisce $\Sigma_\infty \approx 2.1 \cdot 10^{-3}$, cioè $\sigma_\theta^\infty \approx 0.045$ rad. Il valore misurato in simulazione (0.043 rad) coincide con la predizione analitica: **il filtro si comporta come la teoria prevede**, e questo è un controllo di correttezza dell'implementazione, non solo un'osservazione qualitativa.

> **Storico.** Una versione precedente di questo documento riportava per $\theta$ una banda di errore di circa $\pm 0.3$ rad, attribuendola alle caratteristiche della fusione sensoriale. Quella lettura era errata: $\pm 0.3$ rad corrisponde a oltre $6\sigma$ rispetto al rumore del magnetometro e non era spiegabile statisticamente. La causa reale era il disallineamento temporale di un passo fra misura e predizione nel ciclo di simulazione, che produceva un bias sistematico $\omega T_s \approx 0.2$ rad in condizioni di saturazione della velocità angolare. Corretto l'ordine di esecuzione del ciclo, l'errore medio assoluto su $\theta$ è passato da 0.117 rad a 0.034 rad (fattore 3.5) a parità di seed.

---

## 2. Analisi delle Forze Virtuali (Controllo e Sicurezza)

I grafici delle forze virtuali illustrano l'impegno di controllo richiesto per mantenere la formazione e garantire l'assenza di collisioni.

### 2.1 Sforzo di Consenso ($|F_{cons}|$)
Rappresenta la magnitudo del vettore di correzione della velocità generato per mantenere la forma a "V".

* **Il picco iniziale:** nei primi secondi la magnitudo raggiunge 12.4, 23.1 e 10.9 m/s rispettivamente per V1, V2, V3. La flotta è inizializzata in posizioni distanti decine di metri dalla configurazione desiderata (matrice `x0_true`), quindi l'errore di formazione $err_{ij}$ è grande e la legge proporzionale genera un comando di velocità elevato. La formazione rientra entro 1 m dalle distanze nominali a $t = 14.7$ s.
* **Attenzione — limite noto:** questi valori superano la saturazione degli attuatori ($v_{max} = 5$ m/s, $\omega_{max} = 0.6$ rad/s), che risulta attiva per circa il 3% del run — esclusivamente durante il transitorio. In quella fase il comando effettivamente applicato è *tagliato* e la feedback linearization non è più esatta: la velocità realizzata del punto di controllo differisce in direzione, non solo in modulo, da quella comandata. Il sistema converge comunque, ma la convergenza nel transitorio non è garantita dall'analisi lineare del consenso. Una legge di consenso saturata a monte (per esempio normalizzando $F_{cons}$ oltre una soglia) renderebbe il comportamento coerente con la teoria per costruzione.
* **Fase di regime:** raggiunta la formazione, lo sforzo si assesta a 0.081 m/s (V1), 0.112 m/s (V2), 0.113 m/s (V3). Non è mai esattamente nullo: ogni veicolo calcola l'errore di formazione sulla *propria stima rumorosa* della posizione altrui, quindi continua a effettuare micro-correzioni. È il costo di controllo dell'incertezza di stima, ed è coerentemente più alto per gli Slave, che dispongono di stime meno accurate. Il valore è circa quattro volte inferiore a quello registrato con la taratura precedente, per effetto della riduzione di `K_cons` da 1.0 a 0.15: un guadagno alto non migliora la formazione a regime, amplifica soltanto il rumore di stima trasformandolo in comando.

### 2.2 Forza Repulsiva ($|F_{rep}|$)
* **Risultato:** con la taratura sulla scala fisica reale il meccanismo **si attiva**. V2 e V3 registrano un picco di 1.50 m/s a $t = 4.4$ s, durante il transitorio in cui le traiettorie di rientro in formazione si incrociano. V1 non lo attiva mai. A regime la curva è nulla per tutti.
* **Dinamica:** `d_safe = 15` m rappresenta l'ingombro fisico reale dei mezzi (~5 m sui cingoli, ~9 m con fresa e lama) più un margine. Restando ampiamente sotto la distanza nominale di formazione (36 m), la repulsione non interferisce con il consenso a regime, ma interviene quando il transitorio comprime la geometria. È il comportamento desiderato: un vincolo di sicurezza deve essere inattivo finché non serve.
* **Nota sulla taratura di $k_{rep}$:** il gradiente del potenziale FIRAS scala come $1/d^3$, quindi il guadagno **non è trasferibile fra scale diverse**. Nel codice non è più un numero fisso ma viene ricavato per inversione da un requisito di progetto ("a $d = d_{safe}/2$ la repulsione vale $v_{rep,ref} = 3$ m/s"), così resta automaticamente coerente se $d_{safe}$ cambia. Con la taratura precedente ($k_{rep} = 2$, valore sensato per $d_{safe} = 1.5$ m) applicata a distanze di decine di metri, la forza repulsiva sarebbe risultata dell'ordine di $10^{-4}$ m/s: numericamente presente, fisicamente inesistente.
* **Limite residuo:** la repulsione agisce sulle posizioni **stimate**. La sicurezza è quindi garantita solo a meno dell'errore di localizzazione, e un margine rigoroso richiederebbe di gonfiare $d_{safe}$ in funzione dell'incertezza $3\sigma$ sulla posizione relativa — tanto più rilevante nelle zone GPS-denied della Fase 3, dove quell'incertezza cresce di un ordine di grandezza.
