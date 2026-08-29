% =========================================================================
% FASE 3: Navigazione in Ambiente Ostile e Localizzazione Collaborativa
%
% Flotta di N = 3 battipista che attraversa zone GPS-denied mantenendo la
% formazione, con ranging UWB, localizzazione collaborativa e stima
% distribuita del terreno. Documentazione: README3.md.
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
    load(fullfile(fileparts(mfilename('fullpath')), 'ambiente_fase3.mat'));
    disp('Ambiente caricato con successo.');
catch
    error('File ambiente_fase3.mat non trovato. Esegui prima genera_ambiente.m');
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
N_veh    = 3;          % 1 Master + 2 Slave

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

% COVARIANZE DEI SENSORI
R_gps_master  = diag([0.2^2, 0.2^2]);    % [x, y] ricevitore RTK
R_gps_slave   = diag([2.0^2, 2.0^2]);    % [x, y] ricevitore standard
R_imu         = diag([0.05^2, 0.02^2]);  % [theta, omega]
R_enc         = diag([0.1^2, 0.1^2]);    % [w_destro, w_sinistro]
R_traz_master = 0.010^2;                 % [-]^2 torsiometro, trasmissione strumentata
R_traz_slave  = 0.025^2;                 % [-]^2 torsiometro, sensoristica di serie

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
% deve superare ampiamente l'estensione della formazione (36-40 m), altrimenti
% i vicini escono di portata proprio quando la localizzazione collaborativa
% serve. 120 m e' compatibile con un link UWB in vista ottica su neve aperta.
r_collab = 120;  % [m]

%% 4. PARAMETRI DEL CONTROLLO DI FORMAZIONE

% GEOMETRIA DELLA FORMAZIONE
% "V" su fronte ampio, coerente con l'impiego reale dei battipista: d12 = d13 =
% 36.1 m, d23 = 40.0 m. La scala e' anche un requisito funzionale, perche' con
% pochi metri di apertura i tre mezzi condividono sempre la stessa condizione
% di copertura GPS (le zone d'ombra hanno raggio 65 m) e la localizzazione
% collaborativa non avrebbe modo di mostrare alcun beneficio. Vedi README3.md §3.3.
pos_des = [  0.0,  20.0;    % V1 (Master), in testa
           -20.0, -10.0;    % V2, ala sinistra
            20.0, -10.0];   % V3, ala destra

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
% p_tilde_i = p_i - pos_des_i. La costante di tempo vale tau = 1/(K_cons*lambda_2),
% cioe' 2.22 s sul grafo completo K_3. Vedi theory/TEORIA_consenso_su_grafi.md.
%
% A differenza della Fase 2 il grafo e' qui vincolato dalla portata radio, e il
% raggio coincide con r_collab: e' la stessa radio UWB a fornire sia la misura
% di distanza sia il canale dati, quindi il consenso non puo' raggiungere un
% vicino con cui il ranging non e' possibile.
K_cons   = 0.15;
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
toll_dwls  = 1e-9;                               % tolleranza sul residuo di consenso

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
% veicoli di coda non nascano fuori dalla mappa: la formazione e' profonda 30 m
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
        fleet(i).R_gps  = R_gps_master;
        fleet(i).R_traz = R_traz_master;
    else
        fleet(i).R_gps  = R_gps_slave;
        fleet(i).R_traz = R_traz_slave;
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
% SPECIFICITA' DELLA FASE 3
% a) la copertura GPS e' valutata sulla posizione REALE a t_{k+1}: e' una
%    proprieta' dell'ambiente, non della stima del veicolo;
% b) i range inter-veicolari dipendono dalla ground truth di tutti i mezzi,
%    quindi l'impianto e' una passata completa che precede quella dei sensori;
% c) l'ancora mobile e' la stima del vicino a t_k propagata di un passo. Rompe
%    il loop algebrico fra i filtri ed e' cio' che un canale reale rende
%    disponibile; senza propagazione si confronterebbe una misura presa a
%    t_{k+1} con una posizione riferita a t_k, con bias pari a |v_j|*Ts.
disp('Simulazione in corso...');
N_end = N_steps;    % ultimo campione valido, aggiornato all'arrivo

% Fattori di Cholesky dei rumori costanti, calcolati una volta sola
L_imu = chol(R_imu)';
L_enc = chol(R_enc)';

% Diagnostica del grafo di comunicazione, un campione per passo
lambda2_hist = zeros(1, N_steps);   % connettivita' algebrica
rho2_hist    = zeros(1, N_steps);   % essential spectral radius dei pesi Metropolis
n_archi_hist = zeros(1, N_steps);   % archi attivi

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

for k = 1:N_steps-1

    %% (1) BROADCAST
    % Ogni veicolo pubblica il proprio punto di controllo, ricavato dalla
    % stima a t_k. E' il solo contenuto del pacchetto usato dal consenso.
    p_ctrl = zeros(2, N_veh);
    for i = 1:N_veh
        xe = fleet(i).x_est(:, k);
        p_ctrl(:, i) = xe(1:2) + param.b * [cos(xe(3)); sin(xe(3))];
    end

    % Grafo di comunicazione a t_k, vincolato dalla portata radio.
    % Di pesi_metropolis serve qui il solo rho2, come indicatore di velocita'
    % di convergenza: la legge di controllo e' in forma laplaciana e usa G.A.
    % La matrice Q entra invece nel D-WLS del blocco (6).
    G = costruisci_grafo(p_ctrl, R_c_comm);
    [~, rho2] = pesi_metropolis(G.A);
    lambda2_hist(k) = G.lambda2;
    rho2_hist(k)    = rho2;
    n_archi_hist(k) = G.n_archi;

    % Stime dei vicini propagate a t_{k+1}: sono le ancore mobili della
    % localizzazione collaborativa (nota (c) in testa al ciclo).
    p_ancora_mobile = zeros(2, N_veh);
    for j = 1:N_veh
        xj = fleet(j).x_est(:, k);
        p_ancora_mobile(:, j) = xj(1:2) + xj(4) * [cos(xj(3)); sin(xj(3))] * Ts;
    end

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
                err_ij = (p_ctrl(:, i) - p_ctrl(:, j)) - Delta(:, i, j);
                u_cons = u_cons - K_cons * G.A(i,j) * err_ij;
            end

            % REPULSIONE con guard "i ~= j" anziche' "G.A(i,j) > 0": i due sono
            % equivalenti sotto il vincolo d_safe < R_c verificato in §4, e la
            % forma piu' larga e' conservativa perche' non spegnerebbe la
            % repulsione se il vincolo venisse violato.
            if i ~= j
                dist = norm(p_ctrl(:, i) - p_ctrl(:, j));
                if dist < d_safe && dist > 0.1
                    grad_d  = (p_ctrl(:, i) - p_ctrl(:, j)) / dist;
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

        if ~in_denied
            % GPS attivo: misura diretta di posizione assoluta
            z_gps    = xt_next(1:2) + chol(fleet(i).R_gps)' * randn(2,1);
            z        = [z; z_gps];                          %#ok<AGROW>
            z_pred   = [z_pred; x_pred(1:2)];               %#ok<AGROW>
            C_k      = [C_k; 1 0 0 0 0; 0 1 0 0 0];         %#ok<AGROW>
            R_k      = blkdiag(R_k, fleet(i).R_gps);
            is_angle = [is_angle; false; false];            %#ok<AGROW>
        else
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

            % LOCALIZZAZIONE COLLABORATIVA: i vicini come ancore mobili
            % sono riferimenti RELATIVI: vincolano la geometria della
            % formazione ma non la posizione assoluta della flotta.
            for j = 1:N_veh
                if i == j, continue; end

                % La misura e' fisica, fra le posizioni reali a t_{k+1}; la sua
                % predizione usa invece la stima condivisa del vicino, che e'
                % l'unica cosa che il veicolo i puo' conoscere.
                d_true = norm(xt_next(1:2) - fleet(j).x_true(1:2, k+1));
                if d_true <= r_collab
                    p_j   = p_ancora_mobile(:, j);
                    d_est = max(norm(x_pred(1:2) - p_j), 0.1);

                    z        = [z; d_true + sigma_collab * randn()];  %#ok<AGROW>
                    z_pred   = [z_pred; d_est];                       %#ok<AGROW>
                    C_k      = [C_k; (x_pred(1)-p_j(1))/d_est, ...
                                     (x_pred(2)-p_j(2))/d_est, 0, 0, 0];  %#ok<AGROW>
                    R_k      = blkdiag(R_k, sigma_collab^2);
                    is_angle = [is_angle; false];                     %#ok<AGROW>
                end
            end
        end

        [fleet(i).x_est(:, k+1), fleet(i).Sigma] = ...
            ekf_update(x_pred, Sigma_bar, z, z_pred, C_k, R_k, is_angle);
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
% Percorso ancorato allo script: le figure finiscono in fase_3/risultati/ sia
% lanciando il file dall'IDE sia dalla radice del progetto.
cartella_fase   = fileparts(mfilename('fullpath'));
cartella_output = fullfile(cartella_fase, 'risultati');
if ~exist(cartella_output, 'dir'), mkdir(cartella_output); end

colors  = ['b', 'r', 'g'];
t_plot  = t(1:N_end);

% Fasce di oscuramento del Master, usate come sfondo nei grafici temporali:
% senza, l'andamento della covarianza sembrerebbe casuale.
mask_denied = fleet(1).in_denied_hist(1:N_end);

% FIGURA 1: mappa di navigazione
fig1 = figure('Name', 'Fase 3: Navigazione e Sensor Fusion', 'Color', 'w', 'Position', [100 100 800 800]);
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
fig2 = figure('Name','Errore Assoluto di Posizione 2D','Color','w');
for i = 1:N_veh
    subplot(3, 1, i);
    err_x   = fleet(i).x_true(1,1:N_end) - fleet(i).x_est(1,1:N_end);
    err_y   = fleet(i).x_true(2,1:N_end) - fleet(i).x_est(2,1:N_end);
    err_pos = sqrt(err_x.^2 + err_y.^2);        % norma dell'errore sul piano [m]

    ombreggia_denied(t_plot, fleet(i).in_denied_hist(1:N_end), [0, max(err_pos)*1.1]);
    plot(t_plot, err_pos, colors(i), 'LineWidth', 1.2); grid on;
    title(sprintf('V%d: Errore di Posizione Scalare ||e_{pos}||', i));
    ylabel('Errore [m]');
end
xlabel('Tempo [s]');
exportgraphics(fig2, fullfile(cartella_output, '2_errore_posizione_2d.png'), 'Resolution', 300);

% FIGURA 3: diagnostica EKF (errori X, Y, theta)
fig3 = figure('Name','Diagnostica EKF: Errore di Stima','Color','w', 'Position', [100, 100, 1000, 600]);
for i = 1:N_veh
    err_x  = fleet(i).x_true(1,1:N_end) - fleet(i).x_est(1,1:N_end);
    err_y  = fleet(i).x_true(2,1:N_end) - fleet(i).x_est(2,1:N_end);
    err_th = wrapToPi(fleet(i).x_true(3,1:N_end) - fleet(i).x_est(3,1:N_end));

    subplot(3, N_veh, i);
    plot(t_plot, err_x, colors(i), 'LineWidth', 1); grid on; hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    title(sprintf('V%d: Errore X', i));  if i==1; ylabel('[m]'); end

    subplot(3, N_veh, i + N_veh);
    plot(t_plot, err_y, colors(i), 'LineWidth', 1); grid on; hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    title(sprintf('V%d: Errore Y', i));  if i==1; ylabel('[m]'); end

    subplot(3, N_veh, i + 2*N_veh);
    plot(t_plot, err_th, colors(i), 'LineWidth', 1); grid on; hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    title(sprintf('V%d: Errore \\theta', i));
    xlabel('Tempo [s]');  if i==1; ylabel('[rad]'); end
end
exportgraphics(fig3, fullfile(cartella_output, '3_diagnostica_ekf.png'), 'Resolution', 300);

% FIGURA 4: forze virtuali (consenso e repulsione)
fig4 = figure('Name','Analisi delle Forze Virtuali nel Tempo','Color','w', 'Position', [150, 150, 1000, 500]);
for i = 1:N_veh
    mag_cons = vecnorm(fleet(i).u_cons_hist(:, 1:N_end));
    mag_rep  = vecnorm(fleet(i).u_rep_hist(:,  1:N_end));

    subplot(2, N_veh, i);
    plot(t_plot, mag_cons, colors(i), 'LineWidth', 1.5); grid on;
    title(sprintf('V%d: Sforzo Consenso (|F_{cons}|)', i));
    xlabel('Tempo [s]');  if i==1; ylabel('Magnitudo [m/s]'); end
    ylim([0, max(0.1, max(mag_cons)*1.2)]);

    subplot(2, N_veh, i + N_veh);
    plot(t_plot, mag_rep, 'k', 'LineWidth', 1.5); grid on;
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
    plot(t_plot, n_ass, colors(i), 'LineWidth', 1.2, 'DisplayName', sprintf('V%d', i));
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
    plot(t_plot, tr(i,:), colors(i), 'LineWidth', 1.2, 'DisplayName', sprintf('V%d', i));
end
xlabel('Tempo [s]'); ylabel('tr(\Sigma_{pos})  [m^2]');
title('Traccia del blocco posizione della covarianza (sfondo: Master in zona GPS-denied)');
legend('Location','best');
exportgraphics(fig5, fullfile(cartella_output, '5_copertura_e_covarianza.png'), 'Resolution', 300);

% FIGURA 6: consistenza, errore contro inviluppo a 3 sigma
% Verifica che la covarianza dichiarata dal filtro contenga l'errore
% effettivamente commesso. Anticipa la validazione di Fase 6: qui su singolo
% run e a scopo diagnostico, li' su campagna Monte Carlo.
fig6 = figure('Name','Consistenza: errore e bound 3-sigma','Color','w', 'Position', [100, 100, 1000, 600]);
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

        fill([t_plot, fliplr(t_plot)], [s3, fliplr(-s3)], [0.85 0.85 0.85], ...
             'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(t_plot, e, colors(i), 'LineWidth', 0.8);
        ylim([-lim_asse(c), lim_asse(c)]);
        fuori = 100 * mean(abs(e) > s3);
        title(sprintf('V%d: %s  (fuori 3\\sigma: %.1f%%)', i, etichette{c}, fuori));
        if c == 3, xlabel('Tempo [s]'); end
    end
end
exportgraphics(fig6, fullfile(cartella_output, '6_bound_3sigma.png'), 'Resolution', 300);

% FIGURA 7: diagnostica del grafo di comunicazione
kk = 1:(N_end-1);
fig7 = figure('Name','Grafo di Comunicazione: connettivita e convergenza','Color','w', ...
              'Position', [200, 200, 1000, 600]);

subplot(3,1,1);
plot(t(kk), lambda2_hist(kk), 'b', 'LineWidth', 1.5); grid on; hold on;
yline(N_veh, 'k--', 'LineWidth', 1);
ylabel('\lambda_2(L)'); ylim([0, N_veh*1.3]);
title(sprintf('Connettivita algebrica (K_%d completo: \\lambda_2 = %d)', N_veh, N_veh));

subplot(3,1,2);
plot(t(kk), rho2_hist(kk), 'r', 'LineWidth', 1.5); grid on;
ylabel('\rho_2(Q)'); ylim([-0.05, 1.05]);
title('Essential spectral radius dei pesi di Metropolis');

subplot(3,1,3); hold on; grid on;
d_max = 0;
for i = 1:N_veh
    for j = i+1:N_veh
        d_ij = vecnorm(fleet(i).x_true(1:2,kk) - fleet(j).x_true(1:2,kk));
        plot(t(kk), d_ij, 'LineWidth', 1.2, 'DisplayName', sprintf('d_{%d%d}', i, j));
        d_max = max(d_max, max(d_ij));
    end
end
yline(R_c_comm, 'k--', 'LineWidth', 1.5, 'DisplayName', 'R_c (portata radio)');
xlabel('Tempo [s]'); ylabel('Distanza [m]'); legend('Location','best');
ylim([0, max(d_max, R_c_comm)*1.15]);
title('Distanze inter-veicolari contro il raggio di comunicazione');
exportgraphics(fig7, fullfile(cartella_output, '7_grafo_comunicazione.png'), 'Resolution', 300);

% RIEPILOGO A CONSOLE: grafo di comunicazione
fprintf('\n--- GRAFO DI COMUNICAZIONE ---\n');
fprintf('Raggio di comunicazione   : %.0f m\n', R_c_comm);
fprintf('Connettivita algebrica    : lambda_2 = %.4f  (K_%d completo: %d)\n', ...
        mean(lambda2_hist(kk)), N_veh, N_veh);
fprintf('Essential spectral radius : rho_2   = %.4f\n', mean(rho2_hist(kk)));
fprintf('Costante di tempo prevista: tau = 1/(K_cons*lambda_2) = %.2f s\n', ...
        1/(K_cons*mean(lambda2_hist(kk))));
fprintf('Grafo connesso per tutta la missione: %s\n', string(all(lambda2_hist(kk) > 1e-9)));
fprintf('Margine di portata: d_max = %.1f m contro R_c = %.0f m (%.0f%% del raggio)\n', ...
        d_max, R_c_comm, 100*d_max/R_c_comm);

% FIGURA 8: stima distribuita del parametro di terreno
rr = 1:n_round;
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
        fill([dwls.t(rr), fliplr(dwls.t(rr))], [mu_c + s3_c, fliplr(mu_c - s3_c)], ...
             [0.85 0.85 0.85], 'EdgeColor','none', 'DisplayName','D-WLS \pm 3\sigma');
        for i = 1:N_veh
            plot(dwls.t(rr), squeeze(dwls.X_loc(c,i,rr)), [colors(i) ':'], ...
                 'LineWidth', 1.0, 'DisplayName', sprintf('V%d solo locale', i));
        end
        % Spessore decrescente: le tre curve coincidono a meno della precisione
        % di macchina, e restano distinguibili solo cosi'.
        for i = 1:N_veh
            plot(dwls.t(rr), squeeze(dwls.X(c,i,rr)), [colors(i) '-'], ...
                 'LineWidth', 4-i, 'DisplayName', sprintf('V%d D-WLS', i));
        end
        yline(x_terr_true(c), 'k--', 'LineWidth', 1.5, 'DisplayName', 'valore vero');
        xlabel('Tempo [s]'); ylabel(nomi_par{c});
        title(sprintf('Stima di %s  (le tre curve D-WLS coincidono)', nomi_par{c}));

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
        semilogy(dwls.t(rr), err_loc(i,rr), [colors(i) ':'], 'LineWidth', 1.0, ...
                 'DisplayName', sprintf('V%d solo locale', i));
    end
    semilogy(dwls.t(rr), err_dwls(rr), 'k-', 'LineWidth', 2, 'DisplayName', 'D-WLS');
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
        semilogy(dwls.t(rr), squeeze(dwls.dev_loc(2,i,rr)), [colors(i) ':'], ...
                 'LineWidth', 1.0, 'DisplayName', sprintf('V%d da solo', i));
    end
    semilogy(dwls.t(rr), dwls.dev_std(2,rr), 'k-', 'LineWidth', 2, ...
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
    fprintf('Accordo fra i veicoli     : scarto max = %.2e  (consenso esatto: %s)\n', ...
            max(max(abs(dwls.X(:,:,n_round) - dwls.X(:,1,n_round)))), ...
            string(max(max(abs(dwls.X(:,:,n_round) - dwls.X(:,1,n_round)))) < 1e-12));
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
