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

---

## 2. Architettura di Stima e Infrastruttura

### 2.1 Extended Kalman Filter (EKF) e Osservabilità
La stima dello stato locale di ogni veicolo è affidata a un Extended Kalman Filter (EKF). La scelta dell'EKF rispetto ad altre varianti (come l'UKF) è dettata dall'esigenza di condurre una analisi di osservabilità. Calcolando gli Jacobiani del sistema ad ogni istante di campionamento, è possibile valutare il rango della matrice di osservabilità $\mathcal{O}$. Questo permetterà di dimostrare matematicamente come la perdita del GPS degradi l'osservabilità (portando a una deriva della posa assoluta) e come la fusione con le misurazioni UWB permetta di recuperarla.

### 2.2 Infrastruttura UWB per Ambienti GPS-Denied
Nella realtà operativa, il segnale GPS è soggetto ad attenuazioni e multipath, specialmente in ambienti forestali, urbani o indoor. Per ovviare a questo problema in modo economicamente sostenibile, il progetto prevede l'installazione di moduli Ultra-Wideband (UWB). Questa tecnologia, caratterizzata da un'accuratezza centimetrica, viene utilizzata per due scopi:
1.  **Ranging con ancore fisse:** Misurare la distanza da alcune antenne UWB dislocate strategicamente sulla mappa, specialmente in prossimità delle zone cieche per il GPS.
2.  **Ranging inter-veicolare:** Misurare la distanza relativa tra i membri della flotta.

Il posizionamento delle ancore fisse non è casuale: verrà ottimizzato basandosi sul criterio della Geometric Dilution of Precision (GDOP), minimizzando così l'errore di triangolazione nelle aree critiche della mappa.

### 2.3 Fusione Decentralizzata e Covariance Intersection
Essendo un progetto di Sistemi Distribuiti, non esiste un'unità di calcolo centrale. I veicoli comunicano tra loro scambiandosi le proprie stime di posa. Se un veicolo perde il GPS ma un suo vicino lo mantiene, il sistema sfrutta la distanza UWB e lo scambio dati per correggere la traiettoria del veicolo "cieco".
Tuttavia, lo scambio continuo di stime in una rete chiusa genera il fenomeno del *Data Rumination*: le informazioni diventano circolari e i filtri di Kalman iniziano a sottostimare la propria covarianza (diventano troppo "ottimisti"). Per risolvere questo problema, la fusione dei dati provenienti dagli altri veicoli avviene tramite l'algoritmo di **Covariance Intersection (CI)**, che garantisce stime statisticamente consistenti anche in presenza di correlazioni ignote tra gli agenti.

---

## 3. Controllo di Formazione (Consenso)

La flotta, composta inizialmente da $N=3$ veicoli, deve navigare lungo la mappa mantenendo una specifica formazione (ad esempio, muovendosi affiancati). 
Invece di adottare un approccio centralizzato, si utilizza un protocollo basato sul **Consenso**. Ogni veicolo agisce in base all'errore calcolato tra la propria posizione e le posizioni stimate dei propri vicini, regolando la propria velocità e sterzatura per convergere alla distanza relativa desiderata.
Per garantire la sicurezza operativa, al livello di controllo cinematico viene sovrapposto un algoritmo di evitamento collisioni basato sui **Campi Potenziali Artificiali**: se la distanza inter-veicolare scende sotto una soglia di sicurezza, viene generata una forza repulsiva virtuale che devia temporaneamente la traiettoria dei mezzi.

---

## 4. Roadmap di Sviluppo e Validazione

Per garantire la solidità dell'impianto teorico, lo sviluppo in ambiente MATLAB seguirà un paradigma incrementale, partendo da un caso ideale per arrivare a un sistema distribuito quanto più vicino alla realtà.

* **Fase 1: Il Core (Veicolo Singolo Ideale).** Si implementa la dinamica dell'uniciclo e l'EKF per un singolo veicolo. In questa fase, tutti i sensori (GPS, Encoder, IMU) operano alla stessa frequenza di campionamento. Il GPS copre l'intera mappa. L'obiettivo è tarare i parametri di rumore e validare il calcolo degli Jacobiani per l'analisi di osservabilità locale.
* **Fase 2: La Flotta e la Formazione.** Si istanziano $N=3$ veicoli comunicanti attraverso un canale di rete ideale (latenza nulla, zero perdite). Viene integrato il controllo tramite consenso e l'algoritmo per l'evitamento delle collisioni. A scopo puramente analitico e accademico, in questa fase si attribuisce al veicolo "Master" un sensore GPS di qualità superiore, per osservare come la sua stima influenzi il comportamento della formazione rispetto agli "Slave".
* **Fase 3: L'Ambiente Ostile.** Si introduce il realismo ambientale. Vengono create zone *GPS-denied* casuali lungo il percorso e si posizionano le ancore UWB calcolando la GDOP ottimale. Si testa la resilienza della flotta: il sistema deve dimostrare di poter attraversare le zone cieche mantenendo la formazione, affidandosi alla triangolazione UWB e alla localizzazione collaborativa.
* **Fase 4: Realismo dei Sistemi Distribuiti (Multi-rate e Latenze).** Si rimuovono le assunzioni ideali. I sensori operano ora alle loro frequenze reali (es. IMU molto veloce, GPS lento). Il canale di comunicazione introduce latenze. Viene implementato il *timestamping* delle misurazioni: le letture in ritardo provenienti dagli altri veicoli vengono retroattivate correttamente nel buffer storico dell'EKF per non destabilizzare il filtro. Si uniforma la qualità del GPS per tutti i veicoli, rendendo le zone di perdita del segnale dipendenti esclusivamente dalla posizione spaziale.
* **Fase 5: Validazione.** Il modello viene sottoposto a perdite stocastiche di pacchetti di rete (*packet loss*). La validazione finale del progetto includerà la creazione di grafici, in particolare l'analisi della consistenza del filtro tramite i *3-sigma bounds*, confrontando l'errore di stima reale rispetto alla covarianza teorica calcolata dal sistema distribuito.