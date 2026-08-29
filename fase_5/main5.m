% =========================================================================
% FASE 5: Fusione Consistente con Covariance Intersection
%
% Stessa flotta e stesso ambiente della Fase 4: N = 5 battipista, grafo non completo (r_collab = 55 m), sensori a frequenze reali, canale con latenza e perdite. Cade l'ultima ipotesi ideale: la stima del vicino non e' piu' trattata come esatta. Il pacchetto trasporta anche Sigma_j e l'aggiornamento collaborativo passa alla Covariance Intersection. Documentazione: README5.md.
% =========================================================================
clear; clc; close all;

addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'common'));

% NOTAZIONE (Thrun, "Probabilistic Robotics"), identica in tutte le fasi
%   x_t = f(x_{t-1}) + eps_t   cov(eps) = Q    z_t = h(x_t) + delta_t   cov(delta) = R
%   A_k, C_k   Jacobiane df/dx e dh/dx        Sigma, Sigma_bar   covarianza stimata e predetta
%   K, S       guadagno e covarianza dell'innovazione
%   x_est, x_pred  stima a posteriori e a priori (mu_t, mu_bar_t nel testo)
% Tabella completa: README.md §2.6.
%
% ASSENZA DEL TERMINE B_t*u_t
% v e omega sono stati stimati e non ingressi: la predizione e' un random walk
% su di essi. Su terreno scivoloso il comando non coincide con la velocita'
% reale, quindi usarlo come ingresso correlerebbe rumore di processo e
% ingresso, violando le ipotesi del filtro.

%% 1. CARICAMENTO AMBIENTE
% Percorso ancorato allo script e non alla directory corrente: funziona sia
% lanciando il file dall'IDE sia dalla radice del progetto.
try
    load(fullfile(fileparts(mfilename('fullpath')), 'ambiente_fase5.mat'));
    disp('Ambiente caricato con successo.');
catch
    error('File ambiente_fase5.mat non trovato. Esegui prima genera_ambiente.m');
end

%% 2. PARAMETRI DI SISTEMA
% Taratura su battipista reale (PistenBully 600 / Prinoth Bison): 5 m sui
% cingoli, 9 m con fresa e lama. Dettaglio in README.md §1.1.
param.r  = 0.5;        % [m] raggio ruota motrice del cingolo
param.L  = 3.5;        % [m] carreggiata (interasse cingoli)
param.b  = 2.0;        % [m] punto di controllo per feedback linearization
v_max    = 5.0;        % [m/s] ~18 km/h
w_max    = 0.6;        % [rad/s] raggio di sterzata minimo ~4 m
v_cruise = 2.5;        % [m/s] ~9 km/h, velocita' di lavoro del Master
f_s      = 10;         % [Hz]
Ts       = 1/f_s;      % [s] passo di campionamento
N_veh    = 5;          % 1 Master + 4 Slave

% ORIZZONTE DI SIMULAZIONE
% stima larga: il loop termina da solo quando il Master raggiunge l'ultimo
% waypoint, e il margine serve solo a non troncare la missione.
lunghezza_path = sum(vecnorm(diff(path_points)', 2, 1));
num_punti_path = size(path_points, 1);
t_end   = lunghezza_path / 0.5 + 500;
t       = 0:Ts:t_end;
N_steps = length(t);

%% 3. PARAMETRI DEL FILTRO E DEI SENSORI

% RUMORE DI PROCESSO IN FORMA CWNA
% Q e' ricostruita a ogni predizione da calcola_Q_cwna(): il rumore entra
% sulle accelerazioni, dove agisce lo slittamento, e si propaga alla posizione
% attraverso il modello. Derivazione: theory/TEORIA_rumore_di_processo.md.
% I valori sono densita' spettrali: dopo 1 s di sola predizione l'incertezza
% accumulata vale sqrt(q * 1s).
par_Q.q_a       = 0.10;   % [m^2/s^3]   accel. longitudinale -> sigma_v +0.32 m/s in 1 s
par_Q.q_alpha   = 0.01;   % [rad^2/s^3] accel. angolare      -> sigma_w +0.10 rad/s in 1 s
par_Q.q_lat     = 0.02;   % [m^2/s]     deriva laterale      -> 0.14 m in 1 s
par_Q.k_terreno = 0.0;    % [1/s]       Q adattiva q_a += k_terreno*v^2, attiva in Fase 5

% Il canale laterale non e' opzionale: l'uniciclo e' anolonomo e non puo'
% descrivere traslazione laterale, quindi senza q_lat la Q_d risulta singolare
% (rango 4 su 5) e l'incertezza perpendicolare alla marcia non cresce mai.
%
% TARATURA CONSERVATIVA
% l'impianto ha oggi rumore di processo nullo, quindi questi valori rendono il
% filtro pessimista e non ottimista: e' la condizione sicura. La calibrazione
% contro uno slittamento vero sara' possibile solo in Fase 5.

% SENSORI: MODELLI REALI E FREQUENZE NATIVE
% Il passo di simulazione resta 10 Hz. Ogni sensore viene pero' trattato per
% quello che e': chi lavora piu' in fretta del passo si legge a 10 Hz senza
% penalita', chi lavora piu' piano fornisce una misura solo ogni N passi.
%
%   AHRS Xsens MTi-3            100 Hz interni, letto a 10 Hz
%     filtra internamente e restituisce un assetto gia' elaborato, quindi la
%     sigma di targa vale alla frequenza di lettura e non va riscalata.
%   Sensore di velocita' a effetto Hall sul pignone di trazione
%     conta impulsi sulla finestra di campionamento: 100 ms e' la finestra
%     naturale, ne' lenta ne' veloce rispetto al passo.
%   Ranging UWB Qorvo DW1000    ~1 ms per scambio TW-TOF
%     con 5 ancore e 4 vicini servono 9 scambi, cioe' 9 ms su 100: sta nel passo.
%   GNSS u-blox ZED-F9P (RTK)   fino a 20 Hz, qui usato a 5 Hz
%   GNSS u-blox NEO-M8N         1 Hz nominale
%
% E' il GNSS l'unico sensore piu' lento del passo, e va decimato: campionarlo
% a 10 Hz significherebbe dargli fino a dieci volte le misure che produce,
% cioe' dichiarare un ricevitore migliore di quello montato. Con un NEO-M8N a
% 1 Hz l'equivalente sarebbe sigma = 0.63 m anziche' 2.0 m.
R_gps_master  = diag([0.2^2, 0.2^2]);    % [x, y] ZED-F9P in RTK su neve, valore conservativo
R_gps_slave   = diag([2.0^2, 2.0^2]);    % [x, y] NEO-M8N, 2.5 m CEP da datasheet
R_imu         = diag([0.05^2, 0.02^2]);  % [theta, omega] MTi-3: 2 deg RMS su yaw
R_enc         = diag([0.1^2, 0.1^2]);    % [w_destro, w_sinistro]
R_traz_master = 0.010^2;                 % [-]^2 torsiometro, trasmissione strumentata
R_traz_slave  = 0.025^2;                 % [-]^2 torsiometro, sensoristica di serie

% Periodo di aggiornamento del GNSS, espresso in passi di simulazione
f_gps_master  = 5;                       % [Hz] ZED-F9P, conservativo sui 20 di targa
f_gps_slave   = 1;                       % [Hz] NEO-M8N, frequenza nominale
passi_gps_master = round(f_s / f_gps_master);   % 1 fix ogni 2 passi
passi_gps_slave  = round(f_s / f_gps_slave);    % 1 fix ogni 10 passi

% Il torsiometro misura lo sforzo di trazione specifico F_traz/W ed e' l'unico
% sensore che non serve alla stima della posa: alimenta il D-WLS della §5.
% L'asimmetria Master/Slave, la stessa adottata per il GPS, e' cio' che rende
% effettiva la pesatura del WLS: con R uguali per tutti il termine R^-1 diventa
% uno scalare comune e sparisce dalla soluzione.

% RANGING UWB
% stessa tecnologia verso ancora fissa e fra veicoli, qualita' diversa. Il link
% inter-veicolare e' peggiore per l'antenna piu' bassa e per l'effetto della
% piattaforma metallica presente su entrambi i capi anziche' su uno solo. I
% valori sono conservativi rispetto ai centimetri nominali dell'UWB perche'
% tengono conto di neve, ostruzioni parziali e multipath. Vedi README.md §2.2.
sigma_uwb    = 0.5;   % [m] verso ancora fissa (antenna su palo, 3-4 m)
sigma_collab = 0.6;   % [m] fra veicoli

% RAGGIO DI COMUNICAZIONE INTER-VEICOLARE
% ridotto rispetto alla Fase 3, dove valeva 120 m e rendeva il grafo completo
% per costruzione. La stessa fisica che giustifica sigma_collab > sigma_uwb —
% antenne piu' basse ed effetto piattaforma su entrambi i capi del link —
% implica anche una portata inferiore verso un veicolo che verso un'ancora su
% palo. Con questo valore le ali esterne della V non si vedono fra loro e il
% grafo smette di essere completo: e' la condizione in cui rho_2 > 0 e il
% numero di cicli di consenso diventa una grandezza da dimensionare.
%
% Le distanze nominali si separano in due gruppi netti, 36-40 m e 67-80 m, con
% un vuoto di 27 m in mezzo: qualunque valore fra 45 e 65 produce la stessa
% topologia. Il margine e' +38% sui link presenti e -18% su quelli assenti,
% quindi l'errore di formazione non li fa sfarfallare.
r_collab = 55;   % [m]

%% 4. PARAMETRI DEL CONTROLLO DI FORMAZIONE

% GEOMETRIA DELLA FORMAZIONE
% "V" a cinque mezzi su due file d'ala. Distanze nominali: 36.1 m fra apice e
% ali interne, 40.0 m fra le due interne, 36.1 m fra ciascuna interna e la
% esterna dalla stessa parte; 67-80 m fra tutte le altre coppie.
%
% Le ali esterne stanno a 40 m dall'asse contro i 20 m delle interne: in curva
% la loro velocita' si scosta dal Master il doppio, e questo amplia la
% dispersione di v^2 che condiziona la stima del terreno (§5).
pos_des = [  0.0,  40.0;    % V1 (Master), apice
           -20.0,  10.0;    % V2, ala interna sinistra
            20.0,  10.0;    % V3, ala interna destra
           -40.0, -20.0;    % V4, ala esterna sinistra
            40.0, -20.0];   % V5, ala esterna destra

% Offset desiderati fra ogni coppia: Delta_ij = p_i^des - p_j^des
Delta = zeros(2, N_veh, N_veh);
for i = 1:N_veh
    for j = 1:N_veh
        Delta(:, i, j) = pos_des(i,:)' - pos_des(j,:)';
    end
end

% CONSENSO E GRAFO DI COMUNICAZIONE
% Il consenso e' pesato dalla matrice di adiacenza e coincide con il protocollo
% lineare u = -K_cons*(L kron I_2)*p_tilde sulla variabile traslata
% p_tilde_i = p_i - pos_des_i. Vedi theory/TEORIA_consenso_su_grafi.md.
%
% RITARATURA DEL GUADAGNO
% la costante di tempo dell'errore di formazione vale tau = 1/(K_cons*lambda_2(L)).
% Passando da K_3 completo (lambda_2 = 3.000) alla topologia a cinque mezzi con
% due nodi foglia (lambda_2 = 0.697), a parita' di guadagno tau salirebbe da
% 2.22 a 9.56 s. K_cons e' quindi ricalcolato per tenere tau invariato: e'
% lambda_2(L) usato come parametro di progetto e non come indicatore.
% Margine di discretizzazione K_cons*Ts*lambda_max = 0.28, ben sotto il limite 2.
K_cons   = 0.646;       % 1/(tau*lambda_2(L)) con tau = 2.22 s, lambda_2(L) = 0.697
R_c_comm = r_collab;    % [m] raggio del grafo = portata UWB inter-veicolare

% REPULSIONE ANTI-COLLISIONE (campi potenziali, funzione FIRAS)
% k_rep non e' un numero fissato ma ricavato invertendo il requisito "a meta'
% della distanza di sicurezza la repulsione vale v_rep_ref": il gradiente scala
% come 1/d^3 e il guadagno non e' trasferibile fra geometrie di scala diversa.
% Vedi theory/TEORIA_campi_potenziali.md.
d_safe    = 15.0;       % [m] ingombro fisico dei mezzi + margine
v_rep_ref = 3.0;        % [m/s] intensita' repulsiva desiderata a d = d_safe/2
d_ref     = d_safe / 2;
k_rep     = v_rep_ref / ((1/d_ref - 1/d_safe) * (1/d_ref^2));

% VINCOLO DI PROGETTO d_safe < R_c_comm
% la repulsione richiede la direzione verso il vicino, non la sola distanza, e
% la direzione arriva solo dal pacchetto radio: il ranging UWB da' d_ij ma non
% il bearing. Con d_safe < R_c il link e' attivo ogni volta che la repulsione
% serve, quindi la situazione non puo' presentarsi.
assert(d_safe < R_c_comm, ['Vincolo di progetto violato: d_safe = %.1f m deve ' ...
    'essere minore del raggio di comunicazione R_c = %.1f m, altrimenti ' ...
    'esisterebbero configurazioni in cui la repulsione e'' necessaria ma la ' ...
    'posizione del vicino non e'' disponibile.'], d_safe, R_c_comm);


%% 5. STIMA DISTRIBUITA DEL PARAMETRO DI TERRENO (D-WLS)

% PROBLEMA
% la resistenza specifica al moto di un cingolato su neve dipende dallo stato
% del manto, che e' proprieta' del comprensorio e non del singolo mezzo: un
% parametro costante e non stocastico, il caso per cui il Cap. 18 prescrive il
% D-WLS. Il DKF servirebbe se la grandezza avesse una dinamica propria.
%
% MODELLO
% sforzo di trazione specifico necessario a tenere la velocita':
%     F_traz/W = mu_terr + c_terr*v^2 + eps
% lineare nei parametri, con x_terr = [mu_terr; c_terr] e riga di regressione
% C_traz = [1, v^2]. Misurato dal torsiometro, e' una lettura del terreno e non
% un'azione su di esso: l'impianto resta invariato.
%
% PERCHE' DEVE ESSERE DISTRIBUITA
% due parametri e una sola misura scalare per passo rendono la matrice di
% informazione locale un prodotto esterno, di rango 1 su 2: nessun mezzo puo'
% risolvere il problema da solo. Serve unire misure prese a velocita' diverse,
% e in curva le ali della V viaggiano piu' veloci o piu' lente del Master.
%
% COSA VIENE SCAMBIATO
% la sola coppia informativa (F_i, a_i) del parametro di terreno, 5 numeri.
% NON la covarianza della posa Sigma_i, che resta non condivisa.
%
% Trattazione completa: theory/TEORIA_stima_distribuita.md.
x_terr_true = [0.09; 0.005];   % [-] e [s^2/m^2] verita' di terreno, ignota ai veicoli

% BUDGET RADIO E SCALA DEI TEMPI DEL CONSENSO
% il consenso vive sulla scala dei tempi della radio, non del controllo: uno
% scambio TW-TOF dura ~1 ms contro i 100 ms del passo, quindi in un passo il
% canale sostiene decine di cicli. Il round di D-WLS e' percio' eseguito a ogni
% passo. Meta' del passo e' riservata al D-WLS, l'altra meta' al ranging e al
% broadcast delle pose. Schema temporale: theory/TEORIA_ciclo_temporale.md.
T_scambio  = 1e-3;                               % [s] durata di uno scambio TW-TOF
quota_dwls = 0.5;                                % frazione del passo riservata al D-WLS
q_max_dwls = floor(quota_dwls * Ts / T_scambio); % cicli sostenibili in un passo
toll_dwls  = 1e-4;                               % tolleranza sul residuo di consenso

% TOLLERANZA
% il residuo di consenso deve essere trascurabile rispetto all'incertezza della
% stima, non piccolo in assoluto: con dev.std(c_terr)/c_terr dell'ordine del
% 10%, un disaccordo residuo di 1e-4 e' gia' tre ordini di grandezza sotto cio'
% che conta. Chiedere di piu' costerebbe cicli senza cambiare il risultato.
%
% CANALE DI COMUNICAZIONE: LATENZA E PERDITA DI PACCHETTI
% Il pacchetto di posa scambiato a 10 Hz non arriva ne' subito ne' sempre.
%
% RITARDO: gaussiano centrato a meta' dell'intervallo, troncato agli estremi.
% La latenza di una rete reale e' in verita' asimmetrica a destra (pavimento
% fisico, coda da ritrasmissioni), ma la gaussiana simmetrica ha media piu'
% alta a parita' di massimo, quindi stressa DI PIU' il sistema: e' il verso
% sicuro. A 10 Hz il ritardo si quantizza comunque su tre soli valori (0, 1 o
% 2 passi), e la forma della distribuzione conta poco.
%
% I 200 ms di massimo sono pessimistici per UWB, dove uno scambio TW-TOF dura
% circa 1 ms, ma sotto un passo di campionamento il ritardo non sarebbe
% rappresentabile. Sono anche il 35% del margine di stabilita' del consenso,
% che per questa topologia vale pi/(2*K_cons*lambda_max(L)) = 0.565 s.
ritardo_min = 0.0;                              % [s]
ritardo_max = 0.2;                              % [s]
ritardo_med = (ritardo_min + ritardo_max) / 2;  % [s] centro della gaussiana
ritardo_dev = (ritardo_max - ritardo_min) / 6;  % [s] 3 sigma = mezzo intervallo
perc_loss   = 0.5;                              % [%] pacchetti persi del tutto

% Il modello vale per il broadcast delle pose a 10 Hz. I cicli di consenso del
% D-WLS, che vivono sulla scala del millisecondo, restano ideali: perdite su
% quel canale sono materia della Fase 6, dove entra la connettivita' congiunta.

% NUMERO DI CICLI q
% non e' una costante scritta a mano: consenso_dwls lo dimensiona come
% q >= log(toll)/log(rho_2), imponendo almeno il diametro del grafo e troncando
% a q_max_dwls. Su K_3 la regola di Metropolis da' rho_2 = 0 e il
% dimensionamento restituisce q = 1 da solo; su topologia frammentata cresce, e
% info.budget_ok segnala quando il canale radio non basta piu'.

%% 6. INIZIALIZZAZIONE DELLA FLOTTA

% RIFERIMENTO DELLA FORMAZIONE
% la "V" e' definita nel riferimento globale e non viene mai ruotata, ne' qui
% ne' nella legge di consenso: mantiene quindi orientamento fisso rispetto alla
% mappa. Approssimazione accettabile perche' il percorso si sviluppa lungo +Y.
%
% Il punto di partenza e' arretrato lungo il percorso quanto basta perche' i
% veicoli di coda non nascano fuori dalla mappa: la formazione e' profonda 60 m
% e il percorso parte da y = 0.
margine_start = (pos_des(1,2) - min(pos_des(:,2))) + 10;
idx_start = find(path_points(:,2) >= margine_start, 1);
if isempty(idx_start) || idx_start >= num_punti_path
    idx_start = 1;
end

% Il Master nasce su un waypoint, gli altri attorno al centroide della formazione
origine_form = path_points(idx_start, :)' - pos_des(1,:)';
dir_iniziale = atan2(path_points(idx_start+1,2) - path_points(idx_start,2), ...
                     path_points(idx_start+1,1) - path_points(idx_start,1));

% Struttura preallocata: evita il ridimensionamento dinamico dentro al loop.
% u_hist non alimenta alcuna figura di questa fase ed e' registrato in vista
% della Fase 5, dove il confronto fra comando e velocita' reale misura lo slittamento.
fleet = repmat(struct( ...
    'x_true',         zeros(5, N_steps), ...
    'x_est',          zeros(5, N_steps), ...
    'Sigma',          diag([2, 2, 0.1, 1, 1]), ...
    'Sigma_hist',     zeros(5, 5, N_steps), ...   % per bound 3-sigma e NEES
    'u_hist',         zeros(2, N_steps), ...      % comandi [v; w] applicati
    'u_cons_hist',    zeros(2, N_steps), ...      % termine di consenso (figura 4)
    'u_rep_hist',     zeros(2, N_steps), ...      % termine repulsivo (figura 4)
    'in_denied_hist', false(1, N_steps), ...      % copertura GPS
    'n_uwb_hist',     zeros(1, N_steps), ...      % ancore fisse in vista (figura 5)
    'R_gps',          [], ...
    'passi_gps',      1, ...                     % periodo del fix GNSS, in passi
    'n_fix_gps',      0, ...                     % fix effettivamente ricevuti
    'R_traz',         0, ...
    'F_loc',          zeros(2), ...               % matrice di informazione locale
    'a_loc',          zeros(2,1), ...             % stato di informazione locale
    'target_idx',     0), N_veh, 1);

for i = 1:N_veh
    pos_iniziale = origine_form + pos_des(i,:)';
    fleet(i).x_true(:,1) = [pos_iniziale; dir_iniziale; 0; 0];

    % Errore iniziale di posa entro +/- 1 m, per non partire da stima esatta
    fleet(i).x_est(:,1) = fleet(i).x_true(:,1) + [(rand(2,1)-0.5)*2; 0; 0; 0];
    fleet(i).Sigma_hist(:,:,1) = fleet(i).Sigma;

    if i == 1
        fleet(i).R_gps     = R_gps_master;
        fleet(i).R_traz    = R_traz_master;
        fleet(i).passi_gps = passi_gps_master;
    else
        fleet(i).R_gps     = R_gps_slave;
        fleet(i).R_traz    = R_traz_slave;
        fleet(i).passi_gps = passi_gps_slave;
    end

    fleet(i).target_idx = idx_start + 1;
end

%% 7. CICLO DI SIMULAZIONE

% ORDINE DI ESECUZIONE E CONVENZIONE TEMPORALE
% ogni iterazione esegue broadcast, controllo, impianto, sensori, stima. Detto
% t_k = (k-1)*Ts: x_est(:,k) e' la stima a posteriori a t_k, il comando u_k
% agisce su [t_k, t_{k+1}) e dipende solo da x_est(:,k), e le misure che
% correggono la predizione a t_{k+1} sono generate dalla ground truth a
% t_{k+1}. Nessun blocco usa informazione futura.
% Schema temporale completo: theory/TEORIA_ciclo_temporale.md.
%
% SPECIFICITA' EREDITATE DALLE FASI PRECEDENTI
% a) la copertura GPS e' valutata sulla posizione REALE a t_{k+1}: e' una
%    proprieta' dell'ambiente, non della stima del veicolo;
% b) i range inter-veicolari dipendono dalla ground truth di tutti i mezzi,
%    quindi l'impianto e' una passata completa che precede quella dei sensori;
% c) l'ancora mobile e' l'ultima stima ricevuta dal vicino, propagata in avanti
%    per tutta l'eta' del pacchetto. Rompe il loop algebrico fra i filtri ed e'
%    cio' che un canale reale rende disponibile; senza propagazione si
%    confronterebbe una misura presa a t_{k+1} con una posizione riferita al
%    passo di ricezione, con bias pari a |v_j|*eta*Ts.
%
% SPECIFICITA' DELLA FASE 5
% d) l'aggiornamento e' spezzato in due: le misure indipendenti dalle stime
%    altrui restano su guadagno di Kalman, quelle collaborative passano alla
%    Covariance Intersection. Vedi il blocco (4)-(5) piu' sotto.
disp('Simulazione in corso...');
N_end = N_steps;    % ultimo campione valido, aggiornato all'arrivo

% Fattori di Cholesky dei rumori costanti, calcolati una volta sola
L_imu = chol(R_imu)';
L_enc = chol(R_enc)';

% Diagnostica del grafo di comunicazione, un campione per passo
% SPETTRI DEL GRAFO: DUE MATRICI, DUE LETTURE
% L = D - A pesa gli archi con l'adiacenza ed e' l'oggetto del CONTROLLO DI
% FORMAZIONE (tempo continuo): lambda_1(L) = 0 sempre, mol_lambda_1(L) e' il
% numero di componenti, lambda_2(L) da' tau = 1/(K_cons*lambda_2(L)).
% Q pesa gli archi con Metropolis ed e' l'oggetto del D-WLS (tempo discreto):
% lambda_1(Q) = 1 sempre, mol_lambda_1(Q) e' il numero di componenti, e
% rho_2 = |lambda_2(Q)| e' il fattore di convergenza. Gli autovalori di Q sono
% ordinati per MODULO decrescente, quindi il test sulle oscillazioni va fatto su
% lambda_min(Q) e non su lambda_n(Q), che con questo ordinamento e' quello piu'
% vicino a zero.
% Le due letture si muovono in verso opposto: rete ben collegata significa
% lambda_2(L) grande e rho_2 piccolo. Vedi theory/TEORIA_consenso_su_grafi.md.
lambda1_L_hist     = zeros(1, N_steps);      % lambda_1(L), sempre nullo
lambda2_L_hist     = zeros(1, N_steps);      % lambda_2(L), connettivita' algebrica
lambda_max_L_hist  = zeros(1, N_steps);      % lambda_max(L), per il margine di discretizzazione
mol_lambda1_L_hist = zeros(1, N_steps);      % mol_lambda_1(L) = componenti connesse
lambda_Q_hist      = zeros(N_veh, N_steps);  % spettro di Q, per MODULO decrescente
lambda_min_Q_hist  = zeros(1, N_steps);      % autovalore piu' negativo di Q
mol_lambda1_Q_hist = zeros(1, N_steps);      % mol_lambda_1(Q) = componenti connesse
rho2_hist          = zeros(1, N_steps);      % rho_2 = max_{i>=2} |lambda_i(Q)|
n_archi_hist       = zeros(1, N_steps);      % archi attivi

% Storici della stima distribuita del terreno, un round per passo
dwls.t         = nan(1, N_steps);
dwls.X         = nan(2, N_veh, N_steps);   % stima D-WLS di ciascun veicolo
dwls.X_loc     = nan(2, N_veh, N_steps);   % stima con la sola informazione locale
dwls.x_centr   = nan(2, N_steps);          % WLS centralizzato, riferimento
dwls.scarto    = nan(1, N_steps);          % D-WLS contro centralizzato
dwls.inv_somma = nan(1, N_steps);          % invariante della somma
dwls.cond_loc  = nan(N_veh, N_steps);      % condizionamento locale
dwls.cond_rete = nan(1, N_steps);          % condizionamento di rete
dwls.dev_std   = nan(2, N_steps);          % sqrt(diag(Sigma_terr))
dwls.dev_loc   = nan(2, N_veh, N_steps);   % incertezza del singolo veicolo da solo
dwls.scarto_S  = nan(1, N_steps);          % Sigma_terr di rete contro ricostruita dal nodo
dwls.q_eff     = nan(1, N_steps);          % cicli di consenso eseguiti
dwls.q_mis     = nan(1, N_steps);          % cicli osservati per scendere sotto toll
dwls.residuo   = nan(1, N_steps);          % disaccordo residuo fra i nodi
dwls.diam      = nan(1, N_steps);          % diametro del grafo
dwls.budget_ok = true(1, N_steps);         % il canale radio e' bastato
n_round        = 0;

% RIFERIMENTO LATO SIMULATORE
% accumulatore centralizzato costruito con la velocita' VERA al posto di quella
% stimata nella riga di regressione. Non e' disponibile ad alcun veicolo: serve
% solo a isolare l'effetto del regressore incerto (errors-in-variables).
F_ideale    = zeros(2);
a_ideale    = zeros(2,1);
dev_v_media = zeros(1, N_veh);   % sigma(v) medio dichiarato dall'EKF

% STATO DEL CANALE
% k_rx(i,j) e' l'indice del pacchetto piu' fresco che il veicolo i possiede del
% vicino j. Non serve alcun buffer dedicato: la storia delle stime e' gia' in
% fleet(j).x_est, e ricevere con ritardo significa semplicemente leggerla piu'
% indietro. Su un pacchetto perduto l'indice non avanza e il dato invecchia.
k_rx = ones(N_veh);
eta_pacchetto = zeros(N_veh, N_veh, N_steps);   % eta' del dato usato, in passi
n_persi = 0; n_inviati = 0;

% DIAGNOSTICA DELLA COVARIANCE INTERSECTION
% NaN dove la CI non e' intervenuta, cosi' le statistiche si scrivono con le
% funzioni "nan*" senza dover mascherare a mano i passi a cielo aperto.
gamma_hist   = nan(N_veh, N_steps);   % peso ottimo, uno per fusione
infl_hist    = nan(N_veh, N_steps);   % max_j R_eff/sigma_collab^2, gonfiamento di R
dev_pri_hist = nan(N_veh, N_steps);   % sqrt(u'*Sigma*u), incertezza a priori sulla congiungente
dev_mis_hist = nan(N_veh, N_steps);   % sqrt(R_eff), incertezza della misura sulla stessa direzione
n_ci_hist    = zeros(1, N_veh);       % fusioni eseguite da ciascun veicolo

for k = 1:N_steps-1

    %% (1) BROADCAST SU CANALE NON IDEALE
    % Ogni veicolo pubblica il proprio punto di controllo, ricavato dalla
    % stima a t_k. p_ctrl e' il dato VERO trasmesso; quello che i vicini
    % ricevono e' in ritardo, e a volte non arriva affatto.
    %
    % CONTENUTO DEL PACCHETTO, ESTESO IN FASE 5
    % fino alla Fase 4 il pacchetto conteneva la sola posizione stimata. Ora
    % trasporta anche il blocco di posizione della covarianza, cioe' la coppia
    % (p_j, Sigma_j(1:2,1:2)): da 2 a 5 numeri, essendo Sigma simmetrica.
    % Estensione e Covariance Intersection vanno introdotte insieme: usare
    % Sigma_j per gonfiare R lasciando il guadagno di Kalman standard
    % tratterebbe l'incertezza del vicino come rumore INDIPENDENTE, che e' il
    % doppio conteggio del Cap. 15 travestito da correzione.
    p_ctrl = zeros(2, N_veh);
    for i = 1:N_veh
        xe = fleet(i).x_est(:, k);
        p_ctrl(:, i) = xe(1:2) + param.b * [cos(xe(3)); sin(xe(3))];
    end

    % Consegna dei pacchetti: per ogni coppia ordinata (ricevente, mittente) si
    % estrae una perdita e, se il pacchetto passa, un ritardo. L'indice non
    % puo' arretrare, cosi' un pacchetto tardivo non sostituisce un dato piu'
    % fresco gia' ricevuto.
    for i = 1:N_veh
        for j = 1:N_veh
            if i == j
                k_rx(i,j) = k;                  % il proprio dato non transita
                continue;
            end
            n_inviati = n_inviati + 1;
            if rand()*100 < perc_loss
                n_persi = n_persi + 1;          % perso: il dato invecchia
                continue;
            end
            d_s = min(max(ritardo_med + ritardo_dev*randn(), ritardo_min), ritardo_max);
            k_rx(i,j) = max(k_rx(i,j), max(1, k - round(d_s/Ts)));
        end
    end
    eta_pacchetto(:,:,k) = k - k_rx;

    % Cosa ciascun veicolo CREDE degli altri, ricostruito dal pacchetto in suo
    % possesso. Il consenso usa il dato cosi' com'e', senza compensare il
    % ritardo: e' la condizione in cui vale il limite di stabilita' teorico.
    % L'ancora mobile viene invece propagata fino a t_{k+1}, come gia' in Fase 3,
    % ma ora per l'intera eta' del pacchetto e non per un solo passo.
    % Sigma_rx e' la covarianza di posizione dichiarata dal vicino nel pacchetto
    % ricevuto. NON viene propagata in avanti come la posizione: su un'eta'
    % massima di 400 ms il modello di processo aggiunge circa 1.3e-4 m^2, cioe'
    % lo 0.3% di una varianza di posizione tipica di 0.04-0.16 m^2. Propagarla
    % costerebbe le Jacobiane del vicino per un contributo che si perde
    % nell'arrotondamento.
    p_ctrl_rx = zeros(2, N_veh, N_veh);
    p_anc_rx  = zeros(2, N_veh, N_veh);
    Sigma_rx  = zeros(2, 2, N_veh, N_veh);
    for i = 1:N_veh
        for j = 1:N_veh
            xj  = fleet(j).x_est(:, k_rx(i,j));
            dir = [cos(xj(3)); sin(xj(3))];
            p_ctrl_rx(:,i,j) = xj(1:2) + param.b * dir;
            p_anc_rx(:,i,j)  = xj(1:2) + xj(4) * dir * (k + 1 - k_rx(i,j)) * Ts;
            Sigma_rx(:,:,i,j) = fleet(j).Sigma_hist(1:2, 1:2, k_rx(i,j));
        end
    end

    % Grafo di comunicazione a t_k, vincolato dalla portata radio. Si costruisce
    % sulle posizioni CORRENTI perche' essere in portata e' un fatto fisico:
    % il ritardo riguarda il contenuto del pacchetto, non la sua esistenza.
    % Di pesi_metropolis serve qui il solo rho2, come indicatore di velocita'
    % di convergenza: la legge di controllo e' in forma laplaciana e usa G.A.
    % La matrice Q entra invece nel D-WLS del blocco (6).
    G = costruisci_grafo(p_ctrl, R_c_comm);
    [~, rho2, lambda_Q, mol_lambda1_Q, lambda_min_Q] = pesi_metropolis(G.A);
    lambda1_L_hist(k)     = G.lambda1_L;
    lambda2_L_hist(k)     = G.lambda2_L;
    lambda_max_L_hist(k)  = G.lambda_L(end);
    mol_lambda1_L_hist(k) = G.mol_lambda1_L;
    lambda_Q_hist(:,k)  = lambda_Q;
    mol_lambda1_Q_hist(k) = mol_lambda1_Q;
    lambda_min_Q_hist(k)  = lambda_min_Q;
    rho2_hist(k)       = rho2;
    n_archi_hist(k)    = G.n_archi;

    %% (2) CONTROLLO e (3) IMPIANTO
    for i = 1:N_veh
        u_cons  = [0; 0];
        u_rep   = [0; 0];
        V_rif_i = [0; 0];

        % PATH FOLLOWING: lo insegue il solo Master, gli Slave seguono per consenso
        if i == 1
            idx = fleet(i).target_idx;
            target_pt = path_points(idx, :)';

            % Il target virtuale avanza quando il Master gli e' abbastanza
            % vicino. La soglia e' la distanza di lookahead: 10 m, il doppio
            % della lunghezza del mezzo, evita di inseguire un punto quasi
            % coincidente con la propria posizione, che renderebbe la direzione
            % di riferimento numericamente instabile.
            if norm(p_ctrl(:, i) - target_pt) < 10.0 && idx < num_punti_path
                fleet(i).target_idx = idx + 1;
                target_pt = path_points(fleet(i).target_idx, :)';
            end

            d_target = target_pt - p_ctrl(:, i);
            V_rif_i  = v_cruise * d_target / max(norm(d_target), 1e-6);
        end

        for j = 1:N_veh
            % CONSENSO pesato dall'adiacenza: il vicino j contribuisce solo se
            % l'arco esiste, cioe' solo se e' in portata radio.
            if G.A(i,j) > 0
                err_ij = (p_ctrl(:, i) - p_ctrl_rx(:, i, j)) - Delta(:, i, j);
                u_cons = u_cons - K_cons * G.A(i,j) * err_ij;
            end

            % REPULSIONE con guard "i ~= j" anziche' "G.A(i,j) > 0": i due sono
            % equivalenti sotto il vincolo d_safe < R_c verificato in §4, e la
            % forma piu' larga e' conservativa perche' non spegnerebbe la
            % repulsione se il vincolo venisse violato.
            if i ~= j
                dist = norm(p_ctrl(:, i) - p_ctrl_rx(:, i, j));
                if dist < d_safe && dist > 0.1
                    grad_d  = (p_ctrl(:, i) - p_ctrl_rx(:, i, j)) / dist;
                    rep_mag = k_rep * (1/dist - 1/d_safe) * (1/dist^2);
                    u_rep   = u_rep + rep_mag * grad_d;
                end
            end
        end

        fleet(i).u_cons_hist(:, k) = u_cons;
        fleet(i).u_rep_hist(:, k)  = u_rep;

        % Feedback linearization sul punto P a distanza b, valutata sulla
        % stima a t_k. T_fl_inv converte la velocita' cartesiana comandata nei
        % comandi fisici dell'uniciclo. Il nome evita la collisione con R, che
        % nel progetto indica solo la covarianza del rumore di misura.
        p_dot_cmd = V_rif_i + u_cons + u_rep;
        th_est    = fleet(i).x_est(3, k);
        T_fl_inv  = [ cos(th_est),          sin(th_est);
                     -sin(th_est)/param.b,  cos(th_est)/param.b ];

        vw_cmd = T_fl_inv * p_dot_cmd;
        v_cmd  = max(min(vw_cmd(1), v_max), -v_max);
        w_cmd  = max(min(vw_cmd(2), w_max), -w_max);
        fleet(i).u_hist(:, k) = [v_cmd; w_cmd];

        % IMPIANTO: la ground truth avanza con la velocita' gia' presente nello
        % stato a t_k, e il comando appena calcolato diventa effettivo a t_{k+1}.
        % Modello di attuatore ZOH con un passo di ritardo.
        %
        % >>> INNESTO DEL MODELLO DI SLITTAMENTO (Fase 5) <<<
        % oggi x_true(4,k+1) = v_cmd, cioe' tracking ideale dei motori;
        % domani x_true(4,k+1) = f_slip(v_cmd, x_true(:,k), terreno).
        xt = fleet(i).x_true(:, k);
        fleet(i).x_true(1, k+1) = xt(1) + xt(4) * cos(xt(3)) * Ts;
        fleet(i).x_true(2, k+1) = xt(2) + xt(4) * sin(xt(3)) * Ts;
        fleet(i).x_true(3, k+1) = wrapToPi(xt(3) + xt(5) * Ts);
        fleet(i).x_true(4, k+1) = v_cmd;
        fleet(i).x_true(5, k+1) = w_cmd;
    end

    %% (4) SENSORI e (5) STIMA
    for i = 1:N_veh
        xt_next = fleet(i).x_true(:, k+1);      % stato reale a t_{k+1}

        [x_pred, Sigma_bar] = ekf_predict(fleet(i).x_est(:, k), fleet(i).Sigma, Ts, par_Q);

        % BLOCCO BASE: IMU ed encoder, sempre disponibili
        z_imu = xt_next([3,5]) + L_imu * randn(2,1);
        z_enc = [(xt_next(4) + (param.L/2)*xt_next(5))/param.r;
                 (xt_next(4) - (param.L/2)*xt_next(5))/param.r] + L_enc * randn(2,1);

        z      = [z_imu; z_enc];
        z_pred = [ x_pred(3);
                   x_pred(5);
                  (x_pred(4) + (param.L/2)*x_pred(5))/param.r;
                  (x_pred(4) - (param.L/2)*x_pred(5))/param.r ];
        C_k    = [0 0 1 0 0;
                  0 0 0 0 1;
                  0 0 0 1/param.r  param.L/(2*param.r);
                  0 0 0 1/param.r -param.L/(2*param.r)];
        R_k    = blkdiag(R_imu, R_enc);

        % Maschera delle componenti angolari di z: l'innovazione va wrappata
        % solo dove ha senso. Cresce insieme a z, quindi resta corretta
        % qualunque sia l'ordine con cui le misure vengono accodate.
        is_angle = [true; false; false; false];     % solo theta dell'IMU

        % DIMENSIONE VARIABILE DELLE MISURE
        % z, C_k, R_k crescono per accodamento perche' il numero di misure
        % cambia a runtime: GPS presente o negato, ancore e vicini in vista in
        % numero variabile. E' uno dei motivi della scelta dell'EKF (README.md
        % §2.1, punto 4). I marcatori AGROW dichiarano che la crescita e'
        % voluta; il vettore non supera comunque le 11 righe.

        % COPERTURA GPS: valutata sulla posizione reale, e' una proprieta'
        % dell'ambiente e non della stima del veicolo.
        in_denied = false;
        for idx_zona = 1:length(gps_denied_zones)
            centro = [gps_denied_zones(idx_zona).xc; gps_denied_zones(idx_zona).yc];
            if norm(xt_next(1:2) - centro) <= gps_denied_zones(idx_zona).raggio
                in_denied = true;
                break;
            end
        end
        fleet(i).in_denied_hist(k+1) = in_denied;

        % Il fix arriva solo ai multipli del periodo del ricevitore. Sono due
        % condizioni distinte: in_denied dice se il segnale ESISTE in quel
        % punto, il resto dice se il ricevitore ha prodotto una soluzione a
        % questo passo. Fra un fix e l'altro il veicolo procede in sola
        % predizione, sostenuto da AHRS ed encoder.
        fix_gps = ~in_denied && mod(k, fleet(i).passi_gps) == 0;

        if fix_gps
            % GPS attivo: misura diretta di posizione assoluta
            z_gps    = xt_next(1:2) + chol(fleet(i).R_gps)' * randn(2,1);
            z        = [z; z_gps];                          %#ok<AGROW>
            z_pred   = [z_pred; x_pred(1:2)];               %#ok<AGROW>
            C_k      = [C_k; 1 0 0 0 0; 0 1 0 0 0];         %#ok<AGROW>
            R_k      = blkdiag(R_k, fleet(i).R_gps);
            is_angle = [is_angle; false; false];            %#ok<AGROW>
            fleet(i).n_fix_gps = fleet(i).n_fix_gps + 1;
        elseif in_denied
            % RANGING UWB VERSO LE ANCORE FISSE
            % sono riferimenti ASSOLUTI: ancorano la posizione nel riferimento
            % mappa. Il contatore alimenta la figura 5, dove spiega l'andamento
            % della covarianza in zona cieca.
            n_uwb_k = 0;
            for idx_anc = 1:size(uwb_opt, 1)
                p_anc  = uwb_opt(idx_anc, :)';
                d_true = norm(xt_next(1:2) - p_anc);
                if d_true <= r_ancora
                    d_est = max(norm(x_pred(1:2) - p_anc), 0.1);   % previene div/0

                    z        = [z; d_true + sigma_uwb * randn()];   %#ok<AGROW>
                    z_pred   = [z_pred; d_est];                     %#ok<AGROW>
                    C_k      = [C_k; (x_pred(1)-p_anc(1))/d_est, ...
                                     (x_pred(2)-p_anc(2))/d_est, 0, 0, 0];  %#ok<AGROW>
                    R_k      = blkdiag(R_k, sigma_uwb^2);
                    is_angle = [is_angle; false];                   %#ok<AGROW>
                    n_uwb_k  = n_uwb_k + 1;
                end
            end
            fleet(i).n_uwb_hist(k+1) = n_uwb_k;
        end

        % STIMA 1: MISURE INDIPENDENTI, GUADAGNO DI KALMAN STANDARD
        % AHRS, encoder, GPS e ancore fisse nascono da sensori propri del
        % veicolo e da riferimenti la cui posizione e' nota a priori: nessuna
        % di queste misure contiene informazione proveniente dalla rete, quindi
        % l'ipotesi di scorrelazione dall'a priori regge e la CI sarebbe solo
        % una perdita di ottimalita' gratuita. E' l'architettura nota come
        % Split Covariance Intersection.
        [x_upd, Sigma_upd] = ...
            ekf_update(x_pred, Sigma_bar, z, z_pred, C_k, R_k, is_angle);

        % STIMA 2: MISURE COLLABORATIVE, COVARIANCE INTERSECTION
        % LOCALIZZAZIONE COLLABORATIVA: i vicini come ancore mobili. Sono
        % riferimenti RELATIVI: vincolano la geometria della formazione ma non
        % la posizione assoluta della flotta.
        %
        % Il blocco e' separato dal precedente perche' qui la correlazione con
        % l'a priori esiste ed e' ignota: la posa del vicino contiene gia'
        % informazione partita da questo stesso veicolo e tornata indietro
        % lungo i cicli del grafo. La linearizzazione avviene attorno a x_upd e
        % non a x_pred, perche' quello e' l'a priori di QUESTO aggiornamento.
        if in_denied
            z_ci = []; z_pred_ci = []; C_ci = []; R_ci = [];
            s_pri = 0; s_mis = 0; n_vic = 0;
            for j = 1:N_veh
                if i == j, continue; end

                % La misura e' fisica, fra le posizioni reali a t_{k+1}; la sua
                % predizione usa invece l'ultimo pacchetto ricevuto da j,
                % propagato in avanti per tutta la sua eta'. E' l'unica cosa
                % che il veicolo i puo' conoscere.
                d_true = norm(xt_next(1:2) - fleet(j).x_true(1:2, k+1));
                if d_true <= r_collab
                    p_j   = p_anc_rx(:, i, j);
                    d_est = max(norm(x_upd(1:2) - p_j), 0.1);
                    u_ij  = [(x_upd(1)-p_j(1))/d_est, (x_upd(2)-p_j(2))/d_est];

                    % R EFFICACE (Carrillo-Arce et al., IROS 2013)
                    % l'incertezza del vicino non e' isotropa: della sua
                    % ellisse conta solo l'estensione LUNGO la congiungente,
                    % perche' e' l'unica direzione che la misura di distanza
                    % legge. La proiezione u'*Sigma_j*u la riduce a uno scalare
                    % omogeneo a sigma_collab^2 e la somma e' lecita, essendo
                    % il rumore del sensore indipendente dall'errore di j.
                    R_eff = sigma_collab^2 + u_ij * Sigma_rx(:,:,i,j) * u_ij';

                    z_ci      = [z_ci;      d_true + sigma_collab * randn()]; %#ok<AGROW>
                    z_pred_ci = [z_pred_ci; d_est];                           %#ok<AGROW>
                    C_ci      = [C_ci;      u_ij, 0, 0, 0];                   %#ok<AGROW>
                    R_ci      = blkdiag(R_ci, R_eff);

                    infl_hist(i, k+1) = max(infl_hist(i, k+1), R_eff / sigma_collab^2);

                    % CONFRONTO CHE DECIDE gamma
                    % la CI mette a confronto due incertezze sulla STESSA
                    % direzione, quella della congiungente: quella dell'a
                    % priori, sqrt(u'*Sigma*u), e quella della misura,
                    % sqrt(R_eff). Il rapporto fra le due spiega da solo il
                    % peso che l'ottimizzazione restituisce.
                    s_pri = s_pri + sqrt(u_ij * Sigma_upd(1:2,1:2) * u_ij');
                    s_mis = s_mis + sqrt(R_eff);
                    n_vic = n_vic + 1;
                end
            end
            if n_vic > 0
                dev_pri_hist(i, k+1) = s_pri / n_vic;
                dev_mis_hist(i, k+1) = s_mis / n_vic;
            end

            % I contributi dei vicini vengono fusi in un solo aggiornamento con
            % un unico peso, e non uno alla volta: la CI sequenziale sgonfia
            % l'a priori a ogni passaggio e risulterebbe piu' conservativa
            % senza ragione, dato che i rumori dei ranging sono fra loro
            % indipendenti e R_ci e' gia' diagonale a blocchi.
            if ~isempty(z_ci)
                [x_upd, Sigma_upd, g_ci] = ...
                    aggiorna_ci(x_upd, Sigma_upd, z_ci, z_pred_ci, C_ci, R_ci);
                gamma_hist(i, k+1) = g_ci;
                n_ci_hist(i)       = n_ci_hist(i) + 1;
            end
        end

        fleet(i).x_est(:, k+1)     = x_upd;
        fleet(i).Sigma             = Sigma_upd;
        fleet(i).Sigma_hist(:,:,k+1) = fleet(i).Sigma;

        % TORSIOMETRO E ACCUMULO INFORMATIVO LOCALE (fase 1 del D-WLS)
        % la misura dipende dalla velocita' reale, la riga di regressione da
        % quella stimata, che e' l'unica disponibile a bordo. Il regressore
        % incerto attenua la pendenza verso lo zero: l'effetto e' quantificato
        % dall'accumulatore di riferimento piu' sotto.
        z_traz = x_terr_true(1) + x_terr_true(2) * xt_next(4)^2 ...
                 + sqrt(fleet(i).R_traz) * randn();
        C_traz = [1, fleet(i).x_est(4, k+1)^2];
        R_traz = fleet(i).R_traz;

        % I contributi informativi si sommano, quindi accumulare nel tempo
        % equivale a impilare le righe di C e i campioni di z. E' cio' che
        % migliora progressivamente il condizionamento del problema locale, man
        % mano che il veicolo attraversa velocita' diverse.
        fleet(i).F_loc = fleet(i).F_loc + C_traz' * (C_traz / R_traz);
        fleet(i).a_loc = fleet(i).a_loc + C_traz' * (z_traz / R_traz);

        C_ideale       = [1, xt_next(4)^2];
        F_ideale       = F_ideale + C_ideale' * (C_ideale / R_traz);
        a_ideale       = a_ideale + C_ideale' * (z_traz  / R_traz);
        dev_v_media(i) = dev_v_media(i) + sqrt(fleet(i).Sigma(4,4));
    end

    %% (6) ROUND DI STIMA DISTRIBUITA DEL TERRENO
    % Le fasi 2 e 3 del Cap. 18, cicli di consenso e ricostruzione locale, sono
    % in consenso_dwls. Il round si esaurisce fra t_k e t_{k+1} sulla scala dei
    % tempi della radio, quindi la stima e' disponibile a ogni campione.
    %
    % Il grafo e' quello valutato al broadcast di t_k: la topologia usata per
    % scambiare i pacchetti e' l'ultima nota, coerentemente con la convenzione
    % di ritardo adottata per le stime condivise.
    F_stack = zeros(2, 2, N_veh);
    a_stack = zeros(2, N_veh);
    for i = 1:N_veh
        F_stack(:,:,i) = fleet(i).F_loc;
        a_stack(:,i)   = fleet(i).a_loc;
    end
    [X_dwls, info_dwls] = consenso_dwls(F_stack, a_stack, G.A, q_max_dwls, toll_dwls);

    n_round = n_round + 1;
    dwls.t(n_round)           = t(k+1);
    dwls.X(:,:,n_round)       = X_dwls;
    dwls.X_loc(:,:,n_round)   = info_dwls.X_loc;
    dwls.x_centr(:,n_round)   = info_dwls.x_centr;
    dwls.scarto(n_round)      = info_dwls.scarto;
    dwls.inv_somma(n_round)   = info_dwls.inv_somma;
    dwls.cond_loc(:,n_round)  = info_dwls.cond_loc(:);
    dwls.cond_rete(n_round)   = info_dwls.cond_rete;
    dwls.dev_std(:,n_round)   = info_dwls.dev_std;
    dwls.dev_loc(:,:,n_round) = info_dwls.dev_loc;
    dwls.q_eff(n_round)       = info_dwls.q_eff;
    dwls.q_mis(n_round)       = info_dwls.q_misurato;
    dwls.residuo(n_round)     = info_dwls.residuo;
    dwls.diam(n_round)        = info_dwls.diametro;
    dwls.budget_ok(n_round)   = info_dwls.budget_ok;
    dwls.scarto_S(n_round)    = norm(info_dwls.Sigma_nodo - info_dwls.Sigma_terr, 'fro') / ...
                                max(norm(info_dwls.Sigma_terr, 'fro'), eps);

    % TERMINAZIONE ANTICIPATA
    % il Master ha raggiunto l'ultimo waypoint. Senza questo controllo la
    % simulazione proseguirebbe con la flotta ferma, falsando ogni statistica.
    if fleet(1).target_idx >= num_punti_path && ...
       norm(fleet(1).x_true(1:2, k+1) - path_points(end, :)') < 15.0
        N_end = k + 1;
        fprintf('Percorso completato a t = %.1f s (campione %d di %d).\n', ...
                t(N_end), N_end, N_steps);
        break;
    end
end

if N_end == N_steps
    warning('Il Master non ha completato il percorso entro t_end = %.0f s.', t_end);
end
disp('Simulazione completata.');

%% 8. FIGURE E RIEPILOGO
% Percorso ancorato allo script: le figure finiscono in fase_5/risultati/ sia
% lanciando il file dall'IDE sia dalla radice del progetto.
cartella_fase   = fileparts(mfilename('fullpath'));
cartella_output = fullfile(cartella_fase, 'risultati');
if ~exist(cartella_output, 'dir'), mkdir(cartella_output); end

colors  = ['b', 'r', 'g', 'm', 'c'];   % un colore per veicolo
t_plot  = t(1:N_end);

% DECIMAZIONE DEL DISEGNO
% con decine di migliaia di campioni il tracciamento delle bande riempite
% diventa piu' lento della simulazione. Si disegnano al piu' 4000 punti, che a
% schermo sono indistinguibili dal dato completo. Le STATISTICHE restano
% calcolate su tutti i campioni: si decima solo cio' che va sullo schermo.
passo_plot = max(1, ceil(N_end / 4000));
kp = 1:passo_plot:N_end;

% Fasce di oscuramento del Master, usate come sfondo nei grafici temporali:
% senza, l'andamento della covarianza sembrerebbe casuale.
mask_denied = fleet(1).in_denied_hist(1:N_end);

% FIGURA 1: mappa di navigazione
fig1 = figure('Name', 'Fase 5: Navigazione e Sensor Fusion', 'Color', 'w', 'Position', [100 100 800 800]);
hold on; grid on; axis equal; axis([0 W_MAP 0 H_MAP]);

th_c = linspace(0, 2*pi, 100);
for idx_zona = 1:length(gps_denied_zones)
    x_c = gps_denied_zones(idx_zona).xc + gps_denied_zones(idx_zona).raggio * cos(th_c);
    y_c = gps_denied_zones(idx_zona).yc + gps_denied_zones(idx_zona).raggio * sin(th_c);
    patch(x_c, y_c, 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end

plot(path_points(:,1), path_points(:,2), 'k--', 'LineWidth', 1, 'DisplayName', 'Path Nominale');

if ~isempty(uwb_opt)
    plot(uwb_opt(:,1), uwb_opt(:,2), 'b^', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Ancore UWB');
end

for i = 1:N_veh
    plot(fleet(i).x_true(1, 1:N_end), fleet(i).x_true(2, 1:N_end), [colors(i) '-'], 'LineWidth', 1.5, 'DisplayName', sprintf('True V%d', i));
    plot(fleet(i).x_est(1, 1:N_end),  fleet(i).x_est(2, 1:N_end),  [colors(i) ':'], 'LineWidth', 1.5, 'DisplayName', sprintf('Est V%d', i));
    plot(fleet(i).x_true(1, N_end), fleet(i).x_true(2, N_end), [colors(i) 'o'], 'MarkerFaceColor', colors(i), 'HandleVisibility', 'off');
end
title('Fase 3: Navigazione in Zone GPS-Denied'); xlabel('X [m]'); ylabel('Y [m]'); legend('Location', 'best');
exportgraphics(fig1, fullfile(cartella_output, '1_mappa_navigazione.png'), 'Resolution', 300);

% FIGURA 2: errore assoluto di posizione 2D 
fig2 = figure('Name','Errore Assoluto di Posizione 2D','Color','w', ...
              'Position', [100, 100, 800, 200*N_veh]);
for i = 1:N_veh
    subplot(N_veh, 1, i);
    err_x   = fleet(i).x_true(1,1:N_end) - fleet(i).x_est(1,1:N_end);
    err_y   = fleet(i).x_true(2,1:N_end) - fleet(i).x_est(2,1:N_end);
    err_pos = sqrt(err_x.^2 + err_y.^2);        % norma dell'errore sul piano [m]

    ombreggia_denied(t_plot, fleet(i).in_denied_hist(1:N_end), [0, max(err_pos)*1.1]);
    plot(t_plot(kp), err_pos(kp), colors(i), 'LineWidth', 1.2); grid on;
    title(sprintf('V%d: Errore di Posizione Scalare ||e_{pos}||', i));
    ylabel('Errore [m]');
end
xlabel('Tempo [s]');
exportgraphics(fig2, fullfile(cartella_output, '2_errore_posizione_2d.png'), 'Resolution', 300);

% FIGURA 3: diagnostica EKF (errori X, Y, theta)
fig3 = figure('Name','Diagnostica EKF: Errore di Stima','Color','w', ...
              'Position', [100, 100, 330*N_veh, 600]);
for i = 1:N_veh
    err_x  = fleet(i).x_true(1,1:N_end) - fleet(i).x_est(1,1:N_end);
    err_y  = fleet(i).x_true(2,1:N_end) - fleet(i).x_est(2,1:N_end);
    err_th = wrapToPi(fleet(i).x_true(3,1:N_end) - fleet(i).x_est(3,1:N_end));

    subplot(3, N_veh, i);
    plot(t_plot(kp), err_x(kp), colors(i), 'LineWidth', 1); grid on; hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    title(sprintf('V%d: Errore X', i));  if i==1; ylabel('[m]'); end

    subplot(3, N_veh, i + N_veh);
    plot(t_plot(kp), err_y(kp), colors(i), 'LineWidth', 1); grid on; hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    title(sprintf('V%d: Errore Y', i));  if i==1; ylabel('[m]'); end

    subplot(3, N_veh, i + 2*N_veh);
    plot(t_plot(kp), err_th(kp), colors(i), 'LineWidth', 1); grid on; hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    title(sprintf('V%d: Errore \\theta', i));
    xlabel('Tempo [s]');  if i==1; ylabel('[rad]'); end
end
exportgraphics(fig3, fullfile(cartella_output, '3_diagnostica_ekf.png'), 'Resolution', 300);

% FIGURA 4: forze virtuali (consenso e repulsione)
fig4 = figure('Name','Analisi delle Forze Virtuali nel Tempo','Color','w', ...
              'Position', [150, 150, 330*N_veh, 500]);
for i = 1:N_veh
    mag_cons = vecnorm(fleet(i).u_cons_hist(:, 1:N_end));
    mag_rep  = vecnorm(fleet(i).u_rep_hist(:,  1:N_end));

    subplot(2, N_veh, i);
    plot(t_plot(kp), mag_cons(kp), colors(i), 'LineWidth', 1.5); grid on;
    title(sprintf('V%d: Sforzo Consenso (|F_{cons}|)', i));
    xlabel('Tempo [s]');  if i==1; ylabel('Magnitudo [m/s]'); end
    ylim([0, max(0.1, max(mag_cons)*1.2)]);

    subplot(2, N_veh, i + N_veh);
    plot(t_plot(kp), mag_rep(kp), 'k', 'LineWidth', 1.5); grid on;
    title(sprintf('V%d: Forza Repulsiva (|F_{rep}|)', i));
    xlabel('Tempo [s]');  if i==1; ylabel('Magnitudo [m/s]'); end
    ylim([0, max(0.1, max(mag_rep)*1.2)]);
end
exportgraphics(fig4, fullfile(cartella_output, '4_forze_virtuali.png'), 'Resolution', 300);

% FIGURA 5: copertura sensoriale e covarianza
% E' il grafico che dimostra la tesi della fase: la covarianza di posizione
% cresce quando mancano riferimenti assoluti e rientra non appena tornano il
% GPS o le ancore UWB. Il pannello superiore rende leggibile quello inferiore.
fig5 = figure('Name','Copertura Sensoriale e Covarianza','Color','w', 'Position', [100, 100, 1000, 700]);

subplot(2,1,1); hold on; grid on;
for i = 1:N_veh
    % Riferimenti assoluti: 1 se il GPS e' attivo, altrimenti il numero di
    % ancore UWB in vista. I vicini non contano, perche' vincolano la geometria
    % della formazione ma non la posizione assoluta della flotta.
    n_ass = double(~fleet(i).in_denied_hist(1:N_end)) + fleet(i).n_uwb_hist(1:N_end);
    plot(t_plot(kp), n_ass(kp), colors(i), 'LineWidth', 1.2, 'DisplayName', sprintf('V%d', i));
end
ylabel('N. riferimenti assoluti'); title('Riferimenti assoluti disponibili (GPS oppure ancore UWB in vista)');
legend('Location','best'); ylim([-0.2, max(2, size(uwb_opt,1)) + 0.5]);

% Asse logaritmico: la traccia copre tre decadi fra il transitorio iniziale
% (~4 m^2) e il regime con GPS attivo (~5e-3 m^2), e su scala lineare il
% transitorio schiaccerebbe tutto il resto.
subplot(2,1,2); hold on; grid on; set(gca, 'YScale', 'log');
tr = zeros(N_veh, N_end);
for i = 1:N_veh
    tr(i,:) = squeeze(fleet(i).Sigma_hist(1,1,1:N_end) + fleet(i).Sigma_hist(2,2,1:N_end))';
end
lim_lo = 10^floor(log10(min(tr(:))));
lim_hi = 10^ceil(log10(max(tr(:))));
ombreggia_denied(t_plot, mask_denied, [lim_lo, lim_hi]);
for i = 1:N_veh
    plot(t_plot(kp), tr(i,kp), colors(i), 'LineWidth', 1.2, 'DisplayName', sprintf('V%d', i));
end
xlabel('Tempo [s]'); ylabel('tr(\Sigma_{pos})  [m^2]');
title('Traccia del blocco posizione della covarianza (sfondo: Master in zona GPS-denied)');
legend('Location','best');
exportgraphics(fig5, fullfile(cartella_output, '5_copertura_e_covarianza.png'), 'Resolution', 300);

% FIGURA 6: consistenza, errore contro inviluppo a 3 sigma
% Verifica che la covarianza dichiarata dal filtro contenga l'errore
% effettivamente commesso. Anticipa la validazione di Fase 6: qui su singolo
% run e a scopo diagnostico, li' su campagna Monte Carlo.
fig6 = figure('Name','Consistenza: errore e bound 3-sigma','Color','w', ...
              'Position', [100, 100, 330*N_veh, 600]);
etichette = {'X [m]', 'Y [m]', '\theta [rad]'};
lim_asse  = zeros(1, 3);
k0 = min(round(10/Ts), N_end);   % transitorio escluso dal solo calcolo dei
                                 % limiti d'asse, non dai dati
for c = 1:3
    lim_c = 0;
    for i = 1:N_veh
        s3 = 3 * sqrt(squeeze(fleet(i).Sigma_hist(c,c,k0:N_end)))';
        e  = fleet(i).x_true(c,k0:N_end) - fleet(i).x_est(c,k0:N_end);
        if c == 3, e = wrapToPi(e); end
        lim_c = max([lim_c, max(s3), max(abs(e))]);
    end
    lim_asse(c) = lim_c * 1.2;
end

for i = 1:N_veh
    for c = 1:3
        subplot(3, N_veh, (c-1)*N_veh + i); hold on; grid on;
        e = fleet(i).x_true(c,1:N_end) - fleet(i).x_est(c,1:N_end);
        if c == 3, e = wrapToPi(e); end
        s3 = 3 * sqrt(squeeze(fleet(i).Sigma_hist(c,c,1:N_end)))';

        fill([t_plot(kp), fliplr(t_plot(kp))], [s3(kp), fliplr(-s3(kp))], [0.85 0.85 0.85], ...
             'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(t_plot(kp), e(kp), colors(i), 'LineWidth', 0.8);
        ylim([-lim_asse(c), lim_asse(c)]);
        fuori = 100 * mean(abs(e) > s3);
        title(sprintf('V%d: %s  (fuori 3\\sigma: %.1f%%)', i, etichette{c}, fuori));
        if c == 3, xlabel('Tempo [s]'); end
    end
end
exportgraphics(fig6, fullfile(cartella_output, '6_bound_3sigma.png'), 'Resolution', 300);

% FIGURA 7: diagnostica del grafo di comunicazione
kk  = 1:(N_end-1);          % statistiche su tutti i campioni
kkp = kk(1:passo_plot:end);  % sottoinsieme disegnato
fig7 = figure('Name','Grafo di Comunicazione: connettivita e convergenza','Color','w', ...
              'Position', [200, 200, 1000, 600]);

subplot(3,1,1);
plot(t(kkp), lambda2_L_hist(kkp), 'b', 'LineWidth', 1.5); grid on; hold on;
yline(N_veh, 'k--', 'LineWidth', 1, 'DisplayName', 'K_N completo');
ylabel('\lambda_2(L)'); ylim([0, N_veh*1.3]);
title(sprintf('Connettivita algebrica (misurata %.3f; K_%d completo darebbe %d)', ...
      mean(lambda2_L_hist(kk)), N_veh, N_veh));

% Spettro di Q con le tre letture: lambda_1(Q) = 1 (equilibrio garantito),
% rho_2 = |lambda_2(Q)| (velocita'), lambda_min(Q) (oscillazioni, che con
% Metropolis non si presentano mai perche' la diagonale e' strettamente
% positiva). Autovalori ordinati per modulo decrescente.
subplot(3,1,2); hold on; grid on;
for r_idx = 1:N_veh
    plot(t(kkp), lambda_Q_hist(r_idx,kkp), 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8, ...
         'HandleVisibility', 'off');
end
plot(t(kkp), lambda_Q_hist(1,kkp),  'k', 'LineWidth', 1.5, 'DisplayName', '\lambda_1(Q) = 1');
plot(t(kkp), rho2_hist(kkp),       'r', 'LineWidth', 1.5, 'DisplayName', '\rho_2 = |\lambda_2(Q)|');
plot(t(kkp), lambda_min_Q_hist(kkp),'b', 'LineWidth', 1.5, 'DisplayName', '\lambda_{min}(Q)');
yline(1, 'k:', 'HandleVisibility','off');
yline(-1, 'k:', 'DisplayName', 'soglia di oscillazione');
ylabel('\lambda_i(Q)'); ylim([-1.1, 1.1]); legend('Location','best','FontSize',7);
title('Spettro di Q (Metropolis): equilibrio, velocita, oscillazioni');

% Le coppie si dividono in due gruppi: quelle che restano sempre in portata e
% quelle che non lo sono mai. Il margine di ciascun gruppo dal raggio radio
% dice quanto la topologia e' robusta all'errore di formazione.
subplot(3,1,3); hold on; grid on;
d_max = 0; d_max_attivi = 0; d_min_assenti = Inf; n_coppie_attive = 0;
for i = 1:N_veh
    for j = i+1:N_veh
        d_ij = vecnorm(fleet(i).x_true(1:2,kk) - fleet(j).x_true(1:2,kk));
        sempre_in_portata = all(d_ij <= R_c_comm);
        if sempre_in_portata
            stile = '-';  n_coppie_attive = n_coppie_attive + 1;
            d_max_attivi = max(d_max_attivi, max(d_ij));
        else
            stile = ':';
            d_min_assenti = min(d_min_assenti, min(d_ij));
        end
        plot(t(kkp), d_ij(1:passo_plot:end), stile, 'LineWidth', 1.2, ...
             'DisplayName', sprintf('d_{%d%d}', i, j));
        d_max = max(d_max, max(d_ij));
    end
end
yline(R_c_comm, 'k--', 'LineWidth', 1.5, 'DisplayName', 'R_c (portata radio)');
xlabel('Tempo [s]'); ylabel('Distanza [m]'); legend('Location','best','FontSize',7);
ylim([0, max(d_max, R_c_comm)*1.15]);
title('Distanze inter-veicolari contro il raggio radio (continue: link attivi)');
exportgraphics(fig7, fullfile(cartella_output, '7_grafo_comunicazione.png'), 'Resolution', 300);

% RIEPILOGO A CONSOLE: accuratezza della stima di posa
% l'errore e' separato fra copertura GPS e zona cieca: e' il confronto che
% dimostra se il ranging UWB e la localizzazione collaborativa reggono.
fprintf('\n--- ACCURATEZZA DELLA STIMA DI POSA ---\n');
fprintf('Durata missione           : %.1f s (%d campioni)\n', t(N_end), N_end);
cop_tutti = true(1, N_end); cop_nessuno = true(1, N_end);
for i = 1:N_veh
    cop_tutti   = cop_tutti   & ~fleet(i).in_denied_hist(1:N_end);
    cop_nessuno = cop_nessuno &  fleet(i).in_denied_hist(1:N_end);
end
fprintf('Copertura GPS (segnale disponibile): tutti %.1f%%, mista %.1f%%, nessuno %.1f%%\n', ...
        100*mean(cop_tutti), 100*mean(~cop_tutti & ~cop_nessuno), 100*mean(cop_nessuno));
fprintf('Fix GNSS effettivi        : V1 %.1f Hz (ZED-F9P), V2-V%d %.1f Hz (NEO-M8N)\n', ...
        fleet(1).n_fix_gps/t(N_end), N_veh, fleet(2).n_fix_gps/t(N_end));
fprintf('   in sola predizione fra un fix e l altro: %.0f%% dei passi a cielo aperto\n', ...
        100*(1 - 1/passi_gps_slave));
fprintf('   veicolo    MAE con GPS   MAE in zona cieca   tr(Sigma) con GPS / cieca\n');
ruoli    = repmat({'(Slave) '}, 1, N_veh);
ruoli{1} = '(Master)';
for i = 1:N_veh
    e_pos = vecnorm(fleet(i).x_true(1:2,1:N_end) - fleet(i).x_est(1:2,1:N_end));
    tr_i  = squeeze(fleet(i).Sigma_hist(1,1,1:N_end) + fleet(i).Sigma_hist(2,2,1:N_end))';
    cieco = fleet(i).in_denied_hist(1:N_end);
    fprintf('   V%d %s %11.3f m %13.3f m %14.4f / %.4f m^2\n', i, ...
            ruoli{i}, ...
            mean(e_pos(~cieco)), mean(e_pos(cieco)), ...
            mean(tr_i(~cieco)), mean(tr_i(cieco)));
end

% RIEPILOGO A CONSOLE: canale di comunicazione
% L'eta' del pacchetto e' cio' che conta davvero: somma il ritardo di consegna
% e i passi trascorsi dall'ultimo pacchetto arrivato. E' l'unica grandezza che
% il filtro subisce.
eta = eta_pacchetto(:,:,kk);
eta = eta(~repmat(logical(eye(N_veh)), 1, 1, numel(kk)));   % esclude la diagonale
tau_med = mean(eta)*Ts;
tau_max = max(eta)*Ts;
tau_lim = pi / (2*K_cons*mean(lambda_max_L_hist(kk)));      % Olfati-Saber & Murray

fprintf('\n--- CANALE DI COMUNICAZIONE ---\n');
fprintf('%-34s %s\n', 'Grandezza', 'Valore');
fprintf('%-34s %.1f%% richiesto, %.2f%% misurato\n', 'Pacchetti persi', ...
        perc_loss, 100*n_persi/max(n_inviati,1));
fprintf('%-34s %.0f-%.0f ms, media %.0f ms\n', 'Ritardo di consegna', ...
        1e3*ritardo_min, 1e3*ritardo_max, 1e3*ritardo_med);
fprintf('%-34s %.0f%% / %.0f%% / %.0f%% dei pacchetti\n', 'Ritardo quantizzato 0/1/2 passi', ...
        100*mean(eta==0), 100*mean(eta==1), 100*mean(eta>=2));
fprintf('%-34s %.0f ms in media, %.0f ms al massimo\n', 'Eta del dato usato', ...
        1e3*tau_med, 1e3*tau_max);
fprintf('%-34s %.0f ms (%.0f%% del limite, %.0f ms)\n', 'Margine di stabilita consumato', ...
        1e3*tau_med, 100*tau_med/tau_lim, 1e3*tau_lim);
fprintf('%-34s %.3f m su sigma_collab = %.1f m\n', 'Errore ancora mobile a eta media', ...
        0.5*0.3*tau_med^2, sigma_collab);

% RIEPILOGO A CONSOLE: COVARIANCE INTERSECTION
% Le righe rispondono a tre domande distinte: quanto spesso la CI interviene,
% quanto pesa l'a priori nella fusione, e se il risultato e' consistente.
% L'ultima e' la sola che dice se la fase ha raggiunto il proprio scopo.
%
% NEES DI POSIZIONE: e' (p_true - p_est)' * Sigma_pos^-1 * (p_true - p_est),
% cioe' il rapporto fra errore reale e covarianza dichiarata. Con due gradi di
% liberta' il valore atteso e' 2. Sopra 2 il filtro e' OTTIMISTA, dichiara meno
% incertezza di quanta ne abbia davvero ed e' la condizione da evitare; sotto 2
% e' conservativo, che e' il verso sicuro e quello in cui la CI spinge.
nees_cieco = nan(1, N_veh);
for i = 1:N_veh
    idx_cieco = find(fleet(i).in_denied_hist(1:N_end));
    val = zeros(1, numel(idx_cieco));
    for c = 1:numel(idx_cieco)
        kc     = idx_cieco(c);
        e_pos2 = fleet(i).x_true(1:2,kc) - fleet(i).x_est(1:2,kc);
        val(c) = e_pos2' * (fleet(i).Sigma_hist(1:2,1:2,kc) \ e_pos2);
    end
    if ~isempty(val), nees_cieco(i) = mean(val); end
end

n_ci_tot  = sum(n_ci_hist);
passi_cie = sum(arrayfun(@(s) sum(s.in_denied_hist(1:N_end)), fleet));
grado_med = 2*mean(n_archi_hist(kk)) / N_veh;

fprintf('\n--- COVARIANCE INTERSECTION ---\n');
fprintf('%-34s %s\n', 'Grandezza', 'Valore');
fprintf('%-34s %d su %d passi in zona cieca\n', 'Fusioni in forma CI', ...
        n_ci_tot, passi_cie);
fprintf('%-34s media %.3f, intervallo [%.3f, %.3f]\n', 'Peso gamma sull a priori', ...
        mean(gamma_hist(:), 'omitnan'), min(gamma_hist(:)), max(gamma_hist(:)));
fprintf('%-34s %d su %d (%.1f%%): gamma < 1\n', '   di cui misura accolta', ...
        nnz(gamma_hist(:) < 1), n_ci_tot, 100*nnz(gamma_hist(:) < 1)/max(n_ci_tot,1));
fprintf('%-34s media %.2fx, massimo %.2fx\n', 'Gonfiamento di R (R_eff/sigma^2)', ...
        mean(infl_hist(:), 'omitnan'), max(infl_hist(:)));
fprintf('%-34s a priori %.2f m contro misura %.2f m\n', 'Incertezza sulla congiungente', ...
        mean(dev_pri_hist(:), 'omitnan'), mean(dev_mis_hist(:), 'omitnan'));
fprintf('%-34s %.1fx a favore dell a priori (soglia sqrt(2))\n', '   rapporto', ...
        mean(dev_mis_hist(:), 'omitnan') / mean(dev_pri_hist(:), 'omitnan'));

% Da dove viene un a priori cosi' forte: in zona cieca il ramo delle ancore
% fisse e' sempre attivo, e cinque ancore a sigma = 0.5 m ben distribuite
% lasciano poco da aggiungere a un singolo range fra veicoli a sigma = 0.6 m.
n_anc = [];
for i = 1:N_veh
    cieco = fleet(i).in_denied_hist(1:N_end);
    n_anc = [n_anc, fleet(i).n_uwb_hist(cieco)];   %#ok<AGROW>
end
fprintf('%-34s %.1f in media, minimo %d\n', 'Ancore fisse viste in zona cieca', ...
        mean(n_anc), min(n_anc));
fprintf('%-34s 5 numeri = %d B, %.0f B/s ricevuti\n', 'Pacchetto scambiato', ...
        5*8, 5*8*f_s*grado_med);
fprintf('%-34s atteso 2.00 (>2 ottimista, <2 conservativo)\n', 'NEES di posizione in zona cieca');
for i = 1:N_veh
    fprintf('   V%d %s %25.2f\n', i, ruoli{i}, nees_cieco(i));
end

% RIEPILOGO A CONSOLE: grafo di comunicazione
n_coppie = N_veh*(N_veh-1)/2;
fprintf('\n--- GRAFO DI COMUNICAZIONE ---\n');
fprintf('Raggio di comunicazione   : %.0f m\n', R_c_comm);
fprintf('Archi attivi              : %.0f su %d coppie possibili\n', ...
        mean(n_archi_hist(kk)), n_coppie);
fprintf('SPETTRO DI Q (Metropolis, tempo discreto, D-WLS)\n');
fprintf('   lambda_i(Q)     = [%s]  (per modulo decrescente)\n', ...
        sprintf('%+.4f ', mean(lambda_Q_hist(:,kk),2)));
fprintf('   lambda_1(Q)     = %.4f    equilibrio: garantito sempre, Q e stocastica\n', ...
        mean(lambda_Q_hist(1,kk)));
fprintf('   mol_lambda_1(Q) = %d         componenti connesse (1 = rete unica)\n', ...
        round(mean(mol_lambda1_Q_hist(kk))));
fprintf('   rho_2           = %.4f    velocita: |lambda_2(Q)|, errore ~ rho_2^q\n', ...
        mean(rho2_hist(kk)));
fprintf('   lambda_min(Q)   = %+.4f   oscillazioni: lontano da -1 perche diag(Q) > 0\n', ...
        mean(lambda_min_Q_hist(kk)));
fprintf('Diametro del grafo        : %d salti\n', max(dwls.diam(1:n_round)));
fprintf('SPETTRO DI L (Laplaciano, tempo continuo, formazione)\n');
fprintf('   lambda_1(L)     = %.4f    equilibrio: garantito sempre, L*1 = 0\n', ...
        mean(lambda1_L_hist(kk)));
fprintf('   mol_lambda_1(L) = %d         componenti connesse\n', ...
        round(mean(mol_lambda1_L_hist(kk))));
fprintf('   lambda_2(L)     = %.4f    rigidita della formazione (K_%d completo darebbe %d)\n', ...
        mean(lambda2_L_hist(kk)), N_veh, N_veh);
fprintf('   tau = 1/(K_cons*lambda_2(L)) = %.2f s\n', 1/(K_cons*mean(lambda2_L_hist(kk))));
fprintf('   oscillazioni: escluse in tempo continuo (lambda_i(L) reali e >= 0);\n');
fprintf('      in tempo discreto serve K_cons*Ts*lambda_max(L) < 2, qui vale %.3f\n', ...
        K_cons*Ts*mean(lambda_max_L_hist(kk)));
fprintf('Grafo connesso per tutta la missione: %s\n', string(all(lambda2_L_hist(kk) > 1e-9)));
if isfinite(d_min_assenti)
    fprintf('Margini: link attivi fino a %.1f m (+%.0f%%), coppie fuori portata da %.1f m (-%.0f%%)\n', ...
            d_max_attivi, 100*(R_c_comm-d_max_attivi)/d_max_attivi, ...
            d_min_assenti, 100*(d_min_assenti-R_c_comm)/d_min_assenti);
else
    fprintf('Grafo completo: tutte le coppie in portata, d_max = %.1f m contro R_c = %.0f m\n', ...
            d_max, R_c_comm);
end

% FIGURA 8: stima distribuita del parametro di terreno
rr  = 1:n_round;             % statistiche su tutti i round
rrp = rr(1:passo_plot:end);  % sottoinsieme disegnato
if n_round > 0
    nomi_par = {'\mu_{terr} [-]', 'c_{terr} [s^2/m^2]'};
    fig8 = figure('Name','Stima distribuita del terreno (D-WLS)','Color','w', ...
                  'Position', [250, 250, 1100, 700]);

    % PANNELLI 1 e 2: le due componenti del parametro
    % le curve D-WLS dei tre veicoli sono sovrapposte, ed e' il risultato e non
    % un errore di disegno: con rho_2 = 0 il consenso e' esatto in un passo.
    % Le punteggiate mostrano cosa otterrebbe ciascun veicolo senza cooperare.
    for c = 1:2
        subplot(2,2,c); hold on; grid on;
        % Banda a 3 sigma dalla covarianza dell'errore di stima (sum_i F_i)^-1.
        % E' il metro con cui leggere lo scarto dal valore vero: la stima e'
        % buona quanto l'informazione raccolta consente, non quanto si vorrebbe.
        mu_c = squeeze(dwls.X(c,1,rr))';
        s3_c = 3 * dwls.dev_std(c,rr);
        fill([dwls.t(rrp), fliplr(dwls.t(rrp))], [mu_c(1:passo_plot:end) + s3_c(1:passo_plot:end), fliplr(mu_c(1:passo_plot:end) - s3_c(1:passo_plot:end))], ...
             [0.85 0.85 0.85], 'EdgeColor','none', 'DisplayName','D-WLS \pm 3\sigma');
        for i = 1:N_veh
            plot(dwls.t(rrp), squeeze(dwls.X_loc(c,i,rrp)), [colors(i) ':'], ...
                 'LineWidth', 1.0, 'DisplayName', sprintf('V%d solo locale', i));
        end
        % Spessore decrescente: a consenso convergente le curve dei veicoli
        % coincidono, e restano distinguibili solo sovrapponendole in ordine.
        spess = linspace(3.5, 0.8, N_veh);
        for i = 1:N_veh
            plot(dwls.t(rrp), squeeze(dwls.X(c,i,rrp)), [colors(i) '-'], ...
                 'LineWidth', spess(i), 'DisplayName', sprintf('V%d D-WLS', i));
        end
        yline(x_terr_true(c), 'k--', 'LineWidth', 1.5, 'DisplayName', 'valore vero');
        xlabel('Tempo [s]'); ylabel(nomi_par{c});
        title(sprintf('Stima di %s  (le curve D-WLS coincidono)', nomi_par{c}));

        % Scala fissata sulla banda a 3 sigma, escluso il transitorio dei primi
        % round dove la covarianza schiaccerebbe il grafico. Le stime
        % solo-locali possono uscire dal riquadro: e' esso stesso il risultato.
        sel   = rr(max(1, round(0.1*n_round)):end);
        banda = [mu_c(sel) + s3_c(sel), mu_c(sel) - s3_c(sel), x_terr_true(c)];
        banda = banda(isfinite(banda));
        centro = (max(banda) + min(banda)) / 2;
        semi   = max((max(banda) - min(banda))/2, abs(x_terr_true(c))*0.02);
        ylim(centro + [-3, 3]*semi);
        if c == 1, legend('Location','southwest','FontSize',7); end
    end

    % PANNELLO 3: errore relativo, con e senza cooperazione
    subplot(2,2,3); hold on; grid on;
    err_dwls = nan(1, n_round); err_loc = nan(N_veh, n_round);
    for r = rr
        err_dwls(r) = norm(dwls.X(:,1,r) - x_terr_true) / norm(x_terr_true);
        for i = 1:N_veh
            err_loc(i,r) = norm(dwls.X_loc(:,i,r) - x_terr_true) / norm(x_terr_true);
        end
    end
    for i = 1:N_veh
        semilogy(dwls.t(rrp), err_loc(i,rrp), [colors(i) ':'], 'LineWidth', 1.0, ...
                 'DisplayName', sprintf('V%d solo locale', i));
    end
    semilogy(dwls.t(rrp), err_dwls(rrp), 'k-', 'LineWidth', 2, 'DisplayName', 'D-WLS');
    set(gca, 'YScale', 'log');
    xlabel('Tempo [s]'); ylabel('||x - x_{vero}|| / ||x_{vero}||');
    title('Errore relativo di stima'); legend('Location','best','FontSize',7);

    % PANNELLO 4: guadagno informativo della cooperazione
    % confronta l'incertezza che ogni veicolo dichiarerebbe da solo con quella
    % ottenuta in rete. Il parametro mostrato e' c_terr, il piu' difficile:
    % mu_terr e' visibile da fermo, c_terr richiede velocita' diverse. Il
    % confronto e' monotono per costruzione, quindi la curva di rete sta sempre
    % sotto a tutte le altre.
    subplot(2,2,4); hold on; grid on;
    for i = 1:N_veh
        semilogy(dwls.t(rrp), squeeze(dwls.dev_loc(2,i,rrp)), [colors(i) ':'], ...
                 'LineWidth', 1.0, 'DisplayName', sprintf('V%d da solo', i));
    end
    semilogy(dwls.t(rrp), dwls.dev_std(2,rrp), 'k-', 'LineWidth', 2, ...
             'DisplayName', 'in rete (D-WLS)');
    set(gca, 'YScale', 'log');
    xlabel('Tempo [s]'); ylabel('\sigma(c_{terr}) [s^2/m^2]');
    title('Incertezza su c_{terr}: da solo contro in rete');
    legend('Location','best','FontSize',7);

    exportgraphics(fig8, fullfile(cartella_output, '8_stima_terreno_dwls.png'), 'Resolution', 300);

    % RIEPILOGO A CONSOLE: stima distribuita del terreno
    fprintf('\n--- STIMA DISTRIBUITA DEL PARAMETRO DI TERRENO ---\n');
    fprintf('Round di D-WLS eseguiti   : %d (uno per passo, a %.0f Hz)\n', n_round, 1/Ts);
    fprintf('Budget radio              : q_max = %d cicli/passo (%.0f%% di Ts a %.0f ms/scambio)\n', ...
            q_max_dwls, 100*quota_dwls, 1e3*T_scambio);
    fprintf('Cicli di consenso q       : dimensionati = %d, osservati = %s (rho_2 = %.4f, diametro = %d)\n', ...
            max(dwls.q_eff(rr)), string(max(dwls.q_mis(rr))), ...
            mean(rho2_hist(1:N_end-1)), max(dwls.diam(rr)));
    fprintf('Budget sufficiente        : %s   residuo di consenso max = %.2e\n', ...
            string(all(dwls.budget_ok(rr))), max(dwls.residuo(rr)));
    fprintf('Parametro vero            : mu_terr = %.5f   c_terr = %.6f\n', x_terr_true);
    fprintf('Stima finale D-WLS (V1)   : mu_terr = %.5f   c_terr = %.6f\n', dwls.X(:,1,n_round));
    fprintf('WLS centralizzato         : mu_terr = %.5f   c_terr = %.6f\n', dwls.x_centr(:,n_round));
    % Il disaccordo residuo fra i veicoli va confrontato con la tolleranza
    % richiesta, non con la precisione di macchina: con q troncato il consenso
    % e' convergente ma non esatto, ed e' la condizione normale su grafo non
    % completo.
    disaccordo = max(max(abs(dwls.X(:,:,n_round) - dwls.X(:,1,n_round))));
    fprintf('Accordo fra i veicoli     : disaccordo max = %.2e  (entro tolleranza: %s)\n', ...
            disaccordo, string(disaccordo < toll_dwls));
    fprintf('D-WLS contro centralizzato: %.2e  (max su tutti i round)\n', max(dwls.scarto(rr)));
    fprintf('Invariante della somma    : %.2e  (max su tutti i round)\n', max(dwls.inv_somma(rr)));
    fprintf('Condizionamento (ultimo)  : locale = [%s]  di rete = %.2e\n', ...
            sprintf('%.1e ', dwls.cond_loc(:,n_round)), dwls.cond_rete(n_round));
    % Il guadagno va letto per veicolo: la rete non migliora tutti allo stesso
    % modo, distribuisce a tutti la qualita' del membro meglio strumentato.
    fprintf('dev.std(c_terr) in rete   : %.2e\n', dwls.dev_std(2,n_round));
    for i = 1:N_veh
        fprintf('   V%d da solo: %.2e   -> guadagno %.1fx\n', i, ...
                dwls.dev_loc(2,i,n_round), ...
                dwls.dev_loc(2,i,n_round) / dwls.dev_std(2,n_round));
    end
    fprintf('Errore relativo finale    : D-WLS %.2f%%   solo locale [%s]%%\n', ...
            100*err_dwls(n_round), sprintf('%.2f ', 100*err_loc(:,n_round)));
    % Lo scarto dal valore vero va letto contro l'incertezza dichiarata: e'
    % l'unico modo di distinguere una stima sbagliata da una poco informata.
    dev  = dwls.dev_std(:,n_round);
    scrt = dwls.X(:,1,n_round) - x_terr_true;
    fprintf('Incertezza dichiarata     : dev(mu) = %.5f   dev(c) = %.6f\n', dev);
    fprintf('Scarto dal vero           : mu_terr %+.2f dev   c_terr %+.2f dev\n', ...
            scrt ./ dev);
    fprintf('Covarianza dal nodo       : scarto da (sum F)^-1 = %.2e  (richiede n)\n', ...
            max(dwls.scarto_S(rr)));

    % Riferimento con regressore esatto: isola l'effetto errors-in-variables,
    % cioe' il fatto che a bordo v^2 e' noto solo attraverso la stima dell'EKF.
    x_ideale   = F_ideale \ a_ideale;
    dev_ideale = sqrt(diag(inv(F_ideale)));
    fprintf('WLS con v vera (riferim.) : mu_terr = %.5f   c_terr = %.6f\n', x_ideale);
    fprintf('   scarto                 : mu_terr %+.2f dev   c_terr %+.2f dev\n', ...
            (x_ideale - x_terr_true) ./ dev_ideale);
    fprintf('dev.std(v) media dell EKF : [%s] m/s\n', ...
            sprintf('%.3f ', dev_v_media/(N_end-1)));

    % Escursione di velocita': e' cio' che condiziona il problema di stima
    v_all = zeros(N_veh, N_end);
    for i = 1:N_veh, v_all(i,:) = fleet(i).x_true(4, 1:N_end); end
    fprintf('Velocita della flotta     : media %.2f m/s, escursione [%.2f, %.2f] m/s\n', ...
            mean(v_all(:)), min(v_all(:)), max(v_all(:)));
    for i = 1:N_veh
        fprintf('   V%d: media %.2f m/s, escursione [%.2f, %.2f], std(v^2) = %.3f\n', ...
                i, mean(v_all(i,:)), min(v_all(i,:)), max(v_all(i,:)), std(v_all(i,:).^2));
    end
    fprintf('   -> e'' la dispersione di v^2 a determinare quanto e'' osservabile c_terr\n');
end

fprintf('Figure salvate in %s\n', cartella_output);

%% FUNZIONI LOCALI EKF

function [x_pred, Sigma_bar] = ekf_predict(x_old, Sigma_old, Ts, par_Q)
    v_est = x_old(4); th_est = x_old(3); w_est = x_old(5);
    
    x_pred = x_old;
    x_pred(1) = x_pred(1) + v_est * cos(th_est) * Ts;
    x_pred(2) = x_pred(2) + v_est * sin(th_est) * Ts;
    x_pred(3) = wrapToPi(x_pred(3) + w_est * Ts);
    
    A_k = eye(5);
    A_k(1, 3) = -v_est * sin(th_est) * Ts;
    A_k(1, 4) = cos(th_est) * Ts;
    A_k(2, 3) = v_est * cos(th_est) * Ts;
    A_k(2, 4) = sin(th_est) * Ts;
    A_k(3, 5) = Ts;
    
    % Q ricalcolata a ogni passo: dipende da theta (e da v se k_terreno > 0)
    Q_k = calcola_Q_cwna(th_est, v_est, Ts, par_Q);
    Sigma_bar = A_k * Sigma_old * A_k' + Q_k;
end

function [x_new, Sigma_new] = ekf_update(x_pred, Sigma_bar, z, z_pred, C_k, R, is_angle)
    S = C_k * Sigma_bar * C_k' + R;
    K = Sigma_bar * C_k' / S;

    y_innov = z - z_pred;

    % Wrap dell'innovazione limitato alle sole componenti angolari, indicate
    % dalla maschera logica costruita insieme a z. Sostituisce il calcolo per
    % posizione (idx = length(z)-3), che era corretto solo finche' le misure
    % venivano accodate in un ordine preciso e si sarebbe rotto in silenzio
    % al primo cambio di ordinamento.
    y_innov(is_angle) = wrapToPi(y_innov(is_angle));

    x_new = x_pred + K * y_innov;
    x_new(3) = wrapToPi(x_new(3));

    Sigma_new = (eye(5) - K * C_k) * Sigma_bar;
end

function ombreggia_denied(t, mask, y_lim)
    % Disegna in grigio le fasce temporali in cui il veicolo e' privo di GPS.
    % y_lim = [y_min, y_max] esplicito: la patch deve coprire tutta l'altezza
    % dell'asse, e con asse logaritmico y_min deve essere strettamente positivo.
    % Va chiamata PRIMA dei plot dei dati, cosi' le patch restano sullo sfondo.
    d      = diff([false, logical(mask(:)'), false]);
    inizio = find(d ==  1);
    fine   = find(d == -1) - 1;
    for b = 1:numel(inizio)
        x1 = t(inizio(b)); x2 = t(fine(b));
        patch([x1 x2 x2 x1], [y_lim(1) y_lim(1) y_lim(2) y_lim(2)], [0.9 0.9 0.9], ...
              'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    ylim(y_lim);
end
