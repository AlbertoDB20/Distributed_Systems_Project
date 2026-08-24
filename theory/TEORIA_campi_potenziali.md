# Campi Potenziali Artificiali e Funzione FIRAS
### Nota teorica di approfondimento — evitamento collisioni inter-veicolare

Questo documento raccoglie il quadro teorico completo dietro le poche righe di codice che generano la forza repulsiva fra i mezzi. È pensato per essere esposto oralmente.

---

## 1. Da dove viene: Khatib, 1986

Il metodo dei **Campi Potenziali Artificiali** (*Artificial Potential Fields*, APF) è stato introdotto da **Oussama Khatib** a Stanford, in *"Real-Time Obstacle Avoidance for Manipulators and Mobile Robots"* (International Journal of Robotics Research, 1986; una prima versione a ICRA 1985).

Il problema che voleva risolvere era di natura pratica: negli anni '80 la pianificazione del moto era un problema **geometrico globale**, risolto offline costruendo una rappresentazione dello spazio libero. Costoso e inutilizzabile per reagire a un ostacolo che compare all'improvviso.

L'intuizione di Khatib è di ribaltare il problema: invece di *pianificare un percorso*, si costruisce un **campo di forze** e si lascia che il robot lo segua istante per istante. Il robot diventa una particella che si muove in un paesaggio energetico: l'obiettivo è una valle che attrae, gli ostacoli sono colline che respingono. Nessuna pianificazione, solo la valutazione locale di un gradiente. È quindi un metodo **reattivo**, eseguibile in tempo reale, ma — come vedremo — **non completo**.

L'acronimo **FIRAS** viene dal francese *Force Inductrice de Répulsion Artificielle de Surface*: "forza che induce una repulsione artificiale dalla superficie". Khatib è di formazione francese (dottorato a Tolosa) e nel paper originale usa questo nome per la specifica funzione potenziale repulsiva che propone.

---

## 2. La formulazione generale

Si definisce un **potenziale scalare** $U(q)$ sullo spazio delle configurazioni, somma di due contributi:

$$U(q) = U_{att}(q) + U_{rep}(q)$$

e il comando si ottiene come **antigradiente**:

$$F(q) = -\nabla U(q) = -\nabla U_{att}(q) - \nabla U_{rep}(q)$$

Il segno meno è la definizione stessa di forza conservativa: si scende lungo la pendenza dell'energia potenziale. Come una biglia in una scodella.

### 2.1 Il termine attrattivo — nel nostro progetto è il consenso

La scelta classica è un potenziale **quadratico** nell'errore rispetto all'obiettivo:

$$U_{att} = \tfrac{1}{2}k_{att}\,\|q - q_{goal}\|^2 \quad\Longrightarrow\quad F_{att} = -k_{att}(q - q_{goal})$$

cioè una **legge di Hooke**: una molla lineare che tira verso il bersaglio.

Nel nostro sistema **l'obiettivo non è un punto, è una geometria relativa**. Il potenziale attrattivo diventa quindi la somma degli errori di formazione rispetto a tutti i vicini:

$$U_{cons} = \tfrac{1}{2}K_{cons}\sum_{j\in\mathcal{N}_i}\left\|(p_i - p_j) - \Delta_{ij}\right\|^2$$

$$F_{cons} = -\nabla_{p_i}U_{cons} = -K_{cons}\sum_{j\in\mathcal{N}_i}\left[(p_i - p_j) - \Delta_{ij}\right]$$

che è **esattamente** la legge di consenso implementata:

```matlab
err_ij = (p_est(:, i) - p_est(:, j)) - Delta(:, i, j);
F_cons = F_cons - K_cons * err_ij;
```

> **Punto importante da dire all'orale:** il consenso e i campi potenziali non sono due tecniche diverse incollate insieme. Il consenso *è* il termine attrattivo di un campo potenziale, dove il minimo dell'energia coincide con la formazione desiderata. Il sistema completo è un unico campo, con un termine di attrazione e uno di repulsione.

### 2.2 Il termine repulsivo — cosa deve garantire

Il potenziale repulsivo deve soddisfare tre requisiti:

1. **Divergere** quando la distanza tende a zero: l'ostacolo dev'essere una barriera invalicabile.
2. **Annullarsi** oltre una distanza di influenza $d_0$: altrimenti un ostacolo lontanissimo continuerebbe a deviare il robot, e la somma di tutti gli ostacoli del mondo dominerebbe l'attrazione.
3. **Raccordarsi con continuità** al bordo $d_0$: se la forza saltasse bruscamente a zero, il controllo entrerebbe in *chattering* attorno alla soglia.

La funzione FIRAS proposta da Khatib soddisfa tutti e tre:

$$U_{rep}(d) = \begin{cases} \dfrac{1}{2}k_{rep}\left(\dfrac{1}{d} - \dfrac{1}{d_0}\right)^2 & d \le d_0 \\[2ex] 0 & d > d_0 \end{cases}$$

dove $d$ è la distanza dall'ostacolo e $d_0$ la distanza di influenza (nel codice `d_safe`).

Il punto chiave del design è la struttura $\left(\frac{1}{d} - \frac{1}{d_0}\right)$: **è costruita per annullarsi in $d = d_0$**. È questa la risposta alla domanda "perché non usi semplicemente $1/d^2$?".

---

## 3. Derivazione del gradiente (la parte da saper fare alla lavagna)

Vogliamo $F_{rep} = -\nabla_{p_i}U_{rep}$, dove $d = \|p_i - p_j\|$.

**Passo 1 — regola della catena.** $U_{rep}$ dipende da $p_i$ solo attraverso $d$:

$$\nabla_{p_i}U_{rep} = \frac{\partial U_{rep}}{\partial d}\cdot\nabla_{p_i}d$$

**Passo 2 — derivata rispetto alla distanza.**

$$\frac{\partial U_{rep}}{\partial d} = k_{rep}\left(\frac{1}{d}-\frac{1}{d_0}\right)\cdot\frac{\partial}{\partial d}\left(\frac{1}{d}-\frac{1}{d_0}\right) = k_{rep}\left(\frac{1}{d}-\frac{1}{d_0}\right)\cdot\left(-\frac{1}{d^2}\right)$$

**Passo 3 — gradiente della distanza.** È il versore che va da $j$ verso $i$:

$$\nabla_{p_i}d = \nabla_{p_i}\|p_i - p_j\| = \frac{p_i - p_j}{\|p_i - p_j\|} =: \hat{u}_{j\to i}$$

**Passo 4 — ricomposizione.** I due segni meno si cancellano:

$$\boxed{\;F_{rep} = k_{rep}\left(\frac{1}{d}-\frac{1}{d_0}\right)\frac{1}{d^2}\,\hat{u}_{j\to i}\;}$$

E questo è, riga per riga, il codice:

```matlab
dist    = norm(p_est(:, i) - p_est(:, j));           % d
if dist < d_safe && dist > 0.1
    grad_d  = (p_est(:, i) - p_est(:, j)) / dist;    % u_hat = grad_p d
    rep_mag = k_rep * (1/dist - 1/d_safe) * (1/dist^2);
    F_rep   = F_rep + rep_mag * grad_d;
end
```

### 3.1 Verifica di correttezza

**Segno.** Per $d < d_0$ si ha $\frac{1}{d} > \frac{1}{d_0}$, quindi il termine in parentesi è **positivo**, `rep_mag > 0`, e la forza punta lungo $\hat{u}_{j\to i}$, cioè **allontana $i$ da $j$**. Corretto.

**Comportamento al bordo.** In $d = d_0$: $\left(\frac{1}{d_0}-\frac{1}{d_0}\right)=0$, quindi $F_{rep} = 0$ **esattamente**. La forza è continua ($C^0$) attraverso la soglia.

> **Precisazione onesta, utile se l'esaminatore insiste:** la forza è continua ma la sua *derivata* non lo è. Infatti
> $$\frac{dF}{dd} = k_{rep}\left(-\frac{3}{d^4}+\frac{2}{d_0 d^3}\right) \quad\Longrightarrow\quad \left.\frac{dF}{dd}\right|_{d=d_0} = -\frac{k_{rep}}{d_0^4} \ne 0$$
> Il campo è quindi $C^0$ ma non $C^1$ in $d_0$. Basta a evitare il chattering grossolano, non a garantire un comando derivabile.

**Asintoto.** Per $d\to 0$, $F_{rep}\sim k_{rep}/d^3 \to \infty$: barriera teoricamente invalicabile.

---

## 4. Una precisazione che vale punti: non sono forze

Nel codice le variabili si chiamano `F_cons` e `F_rep`, ma **non sono forze**: hanno dimensione **[m/s]**, sono velocità.

Khatib formulava il metodo a livello **dinamico**: $F = ma$, il potenziale genera una forza che agisce su una massa. Noi lavoriamo a livello **cinematico**: il campo potenziale produce direttamente la **velocità desiderata** del punto di controllo,

$$\dot{p}_{cmd,i} = \underbrace{V_{ref}}_{\text{missione}} + \underbrace{F_{cons}}_{\text{formazione}} + \underbrace{F_{rep}}_{\text{sicurezza}}$$

che viene poi invertita nella feedback linearization per ottenere i comandi fisici $(v,\omega)$.

È una semplificazione standard (*first-order* o *kinematic APF*), legittima perché a 2.5 m/s la dinamica di un battipista è lenta rispetto al ciclo di controllo a 10 Hz. Ma **va dichiarata**: se l'esaminatore chiede "dov'è la massa?", la risposta è che non c'è, e questo significa che il modello non cattura l'inerzia — un mezzo da 9 tonnellate non cambia velocità istantaneamente.

Di conseguenza $k_{rep}$ non è in newton: ha unità **[m³/s]**, come si verifica dimensionalmente:
$$[F] = [k_{rep}]\cdot\left[\tfrac{1}{m}\right]\cdot\left[\tfrac{1}{m^2}\right] = \tfrac{m}{s} \;\Longrightarrow\; [k_{rep}] = \tfrac{m^3}{s}$$

---

## 5. Taratura del guadagno e legge di scala

Poiché $F_{rep}\sim k_{rep}/d^3$, **il guadagno non è trasferibile fra scale diverse**: raddoppiare $d_0$ richiede di moltiplicare $k_{rep}$ per circa 8.

Questo è esattamente l'errore che il progetto conteneva: con `k_rep = 2` tarato su $d_0 = 1.5$ m, applicato a distanze di decine di metri la repulsione valeva $\sim 10^{-4}$ m/s — numericamente presente, fisicamente inesistente.

La soluzione adottata è **non fissare il guadagno, ma un requisito di progetto** e ricavarlo per inversione:

> *"A metà del raggio di sicurezza, la repulsione deve valere $v_{rep,ref} = 3$ m/s"*

```matlab
d_ref  = d_safe / 2;
k_rep  = v_rep_ref / ((1/d_ref - 1/d_safe) * (1/d_ref^2));
```

Con $d_{safe} = 15$ m si ottiene $k_{rep} = 2531.25$ m³/s. Il numero sembra assurdo ma è solo un fattore di scala: rimane coerente in automatico se $d_{safe}$ cambia.

### 5.1 Profilo numerico nel progetto

| $d$ [m] | $\|F_{rep}\|$ [m/s] | Commento |
|---|---|---|
| 36.1 | 0 | distanza nominale di formazione — repulsione spenta |
| 15.0 | 0 | soglia $d_{safe}$ — attivazione, con continuità |
| 13.0 | 0.15 | intervento appena percettibile |
| 11.0 | 0.51 | correzione dolce |
| 10.0 | 0.84 | |
| 9.0 | 1.39 | |
| 7.5 | 3.00 | requisito di progetto, $d_{safe}/2$ |
| 6.0 | 7.03 | oltre $v_{max}$: comando saturato |
| 5.0 | 13.5 | |
| 4.0 | 29.0 | barriera |

Nella simulazione di Fase 2 la repulsione si attiva realmente durante il transitorio, con picco di **1.50 m/s a $t = 4.4$ s** su V2 e V3 (le traiettorie di rientro in formazione si incrociano), poi resta identicamente nulla a regime. È il comportamento desiderato: un vincolo di sicurezza deve essere invisibile finché non serve.

---

## 6. Convergenza: cosa si può e cosa non si può dimostrare

Considerando il solo campo potenziale (senza $V_{ref}$), la dinamica è un **flusso di gradiente**: $\dot{p} = -\nabla U$. Allora

$$\dot{U} = (\nabla U)^T\dot{p} = -\|\nabla U\|^2 \le 0$$

quindi $U$ è una **funzione di Lyapunov**: non cresce mai, e per il principio di invarianza di LaSalle il sistema converge all'insieme dei **punti critici** di $U$.

**Qui sta il limite fondamentale del metodo:** i punti critici includono il minimo desiderato, ma **non solo quello**. Nulla garantisce che il punto di arresto sia l'obiettivo.

> **Osservazione specifica del nostro sistema, da dire se ci si vuole distinguere:** con $V_{ref}\ne 0$ il sistema **non è più un puro flusso di gradiente**, ma un flusso di gradiente più una deriva costante. L'argomento di Lyapunov sopra non si applica più tale e quale, e infatti in Fase 3 si osserva un **errore di inseguimento a regime**: le distanze $d_{12}$ e $d_{13}$ si assestano a 39.9 m contro un target di 36.1 m. Non è un bug: è l'errore a regime di un controllo proporzionale che insegue un riferimento in movimento. Gli Slave, non ricevendo $V_{ref}$, devono *mantenere* un errore di formazione per generare la velocità che serve a stare al passo del Master. La correzione è un termine di feedforward o un'azione integrale.

---

## 7. Limiti noti (la parte su cui l'esaminatore preme)

**1. Minimi locali.** Il difetto storico dell'APF. L'attrazione e la repulsione possono bilanciarsi in un punto che non è l'obiettivo, e il robot si ferma lì. Il metodo è reattivo, non completo. Rimedi noti: *navigation functions* di Rimon-Koditschek (dimostrabilmente prive di minimi locali su *sphere worlds*), perturbazione casuale per uscire dalla stallo, oppure un pianificatore globale sovrapposto.

**2. GNRON — *Goal Non-Reachable with Obstacles Nearby*.** Se l'obiettivo cade dentro la distanza di influenza di un ostacolo, la repulsione non si annulla mai nell'obiettivo, che quindi non è più un punto di equilibrio: il robot gli orbita attorno senza raggiungerlo.

> **Nel nostro progetto questo diventa un vincolo di progetto esplicito:**
> $$d_{safe} < \min_{i\ne j}\|\Delta_{ij}\|$$
> cioè $15 < 36.1$. Se la soglia di sicurezza superasse la distanza nominale di formazione, la formazione sarebbe **matematicamente irraggiungibile**. È il tipo di disuguaglianza che conviene saper enunciare.

**3. Saturazione degli attuatori.** Sotto i 6 m la repulsione richiede più di $v_{max}$. Il comando viene tagliato, e nessuna proprietà del campo potenziale sopravvive alla saturazione: la sicurezza non è garantita in quel regime.

**4. Il campo agisce su stime, non sulla verità.** `p_est` viene dall'EKF. La collisione è evitata a meno dell'errore di localizzazione. Un margine rigoroso richiederebbe di gonfiare la soglia:
$$d_{safe}^{eff} = d_{safe} + 3\sigma_{rel}$$
con $\sigma_{rel}$ deviazione standard dell'errore di posizione **relativa**. In Fase 3, nelle zone GPS-denied, quell'incertezza cresce di un ordine di grandezza: è lì che il margine conta davvero.

**5. Nonolonomia e geometria del mezzo.** Il campo produce un vettore velocità in direzione arbitraria, ma l'uniciclo non trasla lateralmente. Il problema è risolto controllando il punto $P$ posto a $b = 2$ m dal baricentro — ma questo significa che **è $P$ a evitare la collisione, non il corpo del veicolo**. Su un mezzo lungo 5 m (9 m con fresa e lama) la differenza non è trascurabile, e $d_{safe}$ deve assorbirla.

---

## 8. L'alternativa moderna: Control Barrier Functions

Se l'esaminatore chiede *"come lo faresti meglio oggi?"*, la risposta è **CBF**.

Invece di **aggiungere** un termine repulsivo e sperare, si definisce l'insieme sicuro come $\mathcal{C} = \{x : h(x)\ge 0\}$ (per esempio $h = \|p_i-p_j\|^2 - d_{safe}^2$) e si impone la condizione

$$\dot{h}(x,u) \ge -\alpha\big(h(x)\big)$$

come **vincolo rigido** dentro un problema di programmazione quadratica che modifica **minimamente** il comando nominale del consenso:

$$u^* = \arg\min_u \|u - u_{nom}\|^2 \quad \text{s.t.} \quad \dot h \ge -\alpha(h),\; u \in \mathcal{U}$$

Vantaggi rispetto all'APF: garantisce l'**invarianza in avanti** dell'insieme sicuro (dimostrabile, non sperata), non distorce il comando quando si è lontani dal vincolo, e i limiti di attuazione $\mathcal{U}$ entrano **dentro** il problema di ottimizzazione invece di essere applicati a valle come saturazione. Riferimento: Ames et al., *"Control Barrier Functions: Theory and Applications"* (ECC 2019).

---

## 9. Domande probabili all'orale, con risposta breve

**"Perché la funzione FIRAS ha quella forma e non $1/d^2$?"**
Perché $\left(\frac1d-\frac1{d_0}\right)$ si annulla esattamente in $d_0$. Con $1/d^2$ la forza salterebbe da un valore finito a zero attraversando la soglia, generando chattering.

**"Le tue forze sono in newton?"**
No, in m/s. È controllo cinematico: il campo produce la velocità desiderata del punto di controllo, non una forza. Non c'è massa nel modello.

**"Cosa succede se un veicolo si blocca?"**
Minimo locale. L'APF è reattivo e incompleto: converge a un punto critico del potenziale, non necessariamente all'obiettivo.

**"Mi garantisci che non si scontrino?"**
No, e sarebbe scorretto dirlo. L'APF non fornisce garanzie formali in presenza di saturazione degli attuatori e di errore di stima. Per una garanzia servirebbe una CBF con QP.

**"Cosa succede se aumento $d_{safe}$ oltre la distanza di formazione?"**
GNRON: la formazione diventa irraggiungibile perché la repulsione non si annulla mai nella configurazione desiderata. Serve $d_{safe} < \min\|\Delta_{ij}\|$.

**"Perché $k_{rep}=2531$?"**
Non è una costante fisica: è un fattore di scala in m³/s ricavato invertendo un requisito di progetto. Poiché il gradiente scala come $1/d^3$, il guadagno scala come $d_{safe}^3$ e non è trasferibile fra geometrie diverse.

**"Il consenso e il campo potenziale sono due controllori distinti?"**
No, sono i due termini dello stesso campo: il consenso è il potenziale attrattivo, la repulsione quello repulsivo. La legge di controllo è l'antigradiente della loro somma.

---

## Riferimenti

- O. Khatib, *"Real-Time Obstacle Avoidance for Manipulators and Mobile Robots"*, IJRR 5(1), 1986.
- E. Rimon, D. E. Koditschek, *"Exact Robot Navigation Using Artificial Potential Functions"*, IEEE T-RO 8(5), 1992. — costruzione priva di minimi locali.
- A. D. Ames et al., *"Control Barrier Functions: Theory and Applications"*, ECC 2019. — alternativa con garanzie formali.
