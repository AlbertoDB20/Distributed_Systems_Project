% =========================================================================
% FASE 3: Navigazione in Ambiente Ostile e Localizzazione Collaborativa
% =========================================================================
clear; clc; close all;

% Funzioni condivise fra le fasi (calcola_Q_cwna, ...)
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'common'));

% -------------------------------------------------------------------------
% NOTAZIONE (Thrun, "Probabilistic Robotics") — identica in tutte le fasi
%
%   Modello:   x_t = f(x_{t-1}, u_t) + eps_t     cov(eps) = Q   [processo]
%              z_t = h(x_t)          + delta_t   cov(delta) = R [misura]
%
%   A_k      Jacobiano df/dx        (nel testo A_t)
%   C_k      Jacobiano dh/dx        (nel testo C_t)
%   Q        covarianza rumore di PROCESSO
%   R        covarianza rumore di MISURA
%   Sigma    covarianza della stima (nel testo Sigma_t)
%   K        guadagno di Kalman
%   S        covarianza dell'innovazione, S = C*Sigma_bar*C' + R
%            (nel testo compare solo come parentesi interna di K_t)
%
%   Corrispondenza per i VETTORI (nel codice tenuti espliciti per leggibilita'):
%   x_est  <-> mu_t        (stima a posteriori)
%   x_pred <-> mu_bar_t    (stima a priori / predetta)
%   Sigma_bar <-> Sigma_bar_t (covarianza predetta)
%
%   NOTA — Assenza del termine B_t*u_t: nel modello di riferimento la
%   predizione usa l'ingresso comandato. Qui NO: v e omega sono STATI stimati,
%   osservati dagli encoder, e la predizione e' un random walk su di essi.
%   Scelta deliberata: su terreno scivoloso il comando NON coincide con la
%   velocita' reale, quindi usarlo come ingresso correlerebbe il rumore di
%   processo con l'ingresso stesso, violando le ipotesi del filtro.
% -------------------------------------------------------------------------

%% 1. CARICAMENTO AMBIENTE
try
    load('ambiente_fase3.mat');
    disp('Ambiente caricato con successo.');
catch
    error('File ambiente_fase3.mat non trovato. Esegui prima lo script di generazione.');
end

%% 2. PARAMETRI DI SISTEMA
% Taratura su mezzo battipista reale (classe PistenBully 600 / Prinoth Bison):
% ingombro ~5 m sui cingoli, ~9 m con fresa e lama. Vedi Fase 2 per il dettaglio.
param.r = 0.5;         % [m] raggio ruota motrice del cingolo
param.L = 3.5;         % [m] carreggiata (interasse cingoli)
param.b = 2.0;         % [m] punto di controllo per feedback linearization
v_max = 5.0;           % [m/s] ~18 km/h
w_max = 0.6;           % [rad/s] ~34 deg/s, coerente con un cingolato di 5 m
v_cruise = 2.5;        % [m/s] ~9 km/h, velocita' di lavoro del Master
f_s = 10;              % [Hz] frequenza
Ts = 1/f_s;            % [s]

% Calcolo del tempo di simulazione approssimativo in base al percorso
lunghezza_path = sum(vecnorm(diff(path_points)', 2, 1));
num_punti_path = size(path_points, 1);
t_end = (lunghezza_path / 2.0) + 1000; % Assumiamo velocità media di 2 m/s + margine
t = 0:Ts:t_end;
N_steps = length(t);
N_veh = 3;             % 1 Master, 2 Slaves

%% 3. PARAMETRI RUMORE SENSORI
% -------------------------------------------------------------------------
% RUMORE DI PROCESSO — modello CWNA (Continuous White Noise Acceleration)
%
% Q non e' piu' una matrice diagonale costante ma viene ricostruita a ogni
% passo di predizione da calcola_Q_cwna(). Due ragioni, entrambe sostanziali:
%
% 1) STRUTTURA FISICA. Nel modello uniciclo la posizione non ha dinamica
%    propria: cambia solo perche' v e theta sono incerti. Una Q diagonale
%    inietta rumore direttamente su x e y, cioe' afferma che il veicolo si
%    sposta lateralmente anche da fermo. Nel modello CWNA il rumore entra
%    sulle ACCELERAZIONI — dove agisce fisicamente lo slittamento — e si
%    propaga alla posizione attraverso il modello, generando anche le
%    CORRELAZIONI posizione-velocita' che una diagonale butta via.
%
% 2) SCALATURA SU Ts. Q_d e' proporzionale a Ts (e a Ts^2/2, Ts^3/3 nei
%    termini propagati). La formulazione precedente sommava una costante a
%    ogni passo: cambiare la frequenza di campionamento ri-tarava
%    silenziosamente il filtro. E' il prerequisito per il multi-rate di Fase 4.
%
% I valori sono espressi come densita' spettrali e si leggono cosi': dopo 1 s
% di sola predizione, l'incertezza accumulata vale sqrt(q * 1s).
par_Q.q_a       = 0.10;   % [m^2/s^3]   accel. longitudinale (slittam. in trazione)
                          %             -> sigma_v cresce di 0.32 m/s in 1 s
par_Q.q_alpha   = 0.01;   % [rad^2/s^3] accel. angolare (slittam. in sterzata)
                          %             -> sigma_omega cresce di 0.10 rad/s in 1 s
par_Q.q_lat     = 0.02;   % [m^2/s]     deriva laterale su pendio innevato
                          %             -> 0.14 m di scarto laterale in 1 s.
                          %             Canale indispensabile: senza, Q_d e'
                          %             SINGOLARE (rango 4/5) perche' il modello
                          %             uniciclo non puo' descrivere traslazione
                          %             laterale, e l'incertezza perpendicolare
                          %             alla marcia non crescerebbe mai.
par_Q.k_terreno = 0.0;    % [1/s]       Q adattiva: q_a += k_terreno*v^2.
                          %             Inattiva finche' l'impianto non simula
                          %             uno slittamento reale (Fase 5).
%
% NOTA DI TARATURA. La ground truth attuale ha rumore di processo NULLO
% (tracking ideale degli attuatori), quindi questi valori sono deliberatamente
% conservativi rispetto all'impianto simulato: il filtro risulta pessimista,
% non ottimista. E' la condizione sicura. La calibrazione onesta di q_a e
% q_alpha sara' possibile solo in Fase 5, contro uno slittamento vero.
% -------------------------------------------------------------------------

% Rumori di misura
R_gps_master = diag([0.2^2, 0.2^2]);
R_gps_slave  = diag([2.0^2, 2.0^2]);
R_imu        = diag([0.05^2, 0.02^2]);   % [th, w]
R_enc        = diag([0.1^2, 0.1^2]);     % [wR, wL]
% RANGING UWB — Il ranging verso ancora fissa e quello inter-veicolare usano la
% stessa tecnologia, ma non hanno la stessa qualita'. In condizioni LOS ideali
% l'UWB e' accurato al centimetro; i valori qui sono volutamente conservativi
% perche' tengono conto di neve, ostruzioni parziali del terreno e multipath.
sigma_uwb    = 0.5;   % [m] verso ANCORA FISSA: posizione rilevata una volta per
                      %     tutte, antenna su palo (3-4 m), quindi buona LOS.
sigma_collab = 0.6;   % [m] INTER-VEICOLARE: peggiore, ma non per il moto.
                      %     Lo spostamento durante lo scambio TW-TOF (~1 ms a
                      %     2.5 m/s) vale millimetri ed e' trascurabile. Le
                      %     ragioni vere sono due:
                      %     1) ANTENNE PIU' BASSE - montate sul mezzo e non su
                      %        palo. La letteratura sperimentale mostra che
                      %        l'errore di ranging cresce marcatamente al
                      %        ridursi dell'altezza d'antenna (piu' multipath
                      %        da riflessione al suolo, piu' occlusione).
                      %     2) EFFETTO PIATTAFORMA SU ENTRAMBI I TERMINALI -
                      %        la massa metallica del veicolo distorce il
                      %        diagramma d'antenna e introduce un bias di
                      %        ranging. Verso un'ancora fissa l'effetto e'
                      %        presente su un solo capo del link, fra due
                      %        veicoli su entrambi.

% RAGGIO DI COMUNICAZIONE — Deve essere ampiamente superiore all'estensione
% della formazione (36-40 m), altrimenti i vicini escono dalla portata radio e
% la localizzazione collaborativa si spegne proprio quando serve. Il valore
% precedente (40 m) era al limite esatto della nuova formazione.
% 120 m e' compatibile con un link UWB in vista ottica su neve aperta.
r_collab = 120;  % [m]

%% 4. PARAMETRI FORMAZIONE
% Formazione a "V" su fronte ampio, coerente con l'impiego reale dei mezzi
% battipista. Distanze reciproche: d12 = d13 = 36.1 m, d23 = 40.0 m.
% E' anche un requisito funzionale: con una formazione di pochi metri i tre
% veicoli condividono sempre la stessa condizione di copertura GPS (le zone
% d'ombra hanno raggio 65 m) e la localizzazione collaborativa non ha modo di
% dimostrare alcun beneficio. Vedi README3.md §3.3.
pos_des = [  0.0,  20.0;  % V1 (Master), in testa
           -20.0, -10.0;  % V2, ala sinistra
            20.0, -10.0]; % V3, ala destra
Delta = zeros(2, N_veh, N_veh);
for i = 1:N_veh
    for j = 1:N_veh
        Delta(:, i, j) = pos_des(i,:)' - pos_des(j,:)';
    end
end

% Guadagni riscalati sulla nuova geometria (vedi Fase 2 per la derivazione).
K_cons    = 0.15;       % tau = 1/(3*K_cons) ~ 2.2 s sul grafo completo K3
d_safe    = 15.0;       % [m] ingombro fisico dei mezzi + margine
v_rep_ref = 3.0;        % [m/s] intensita' repulsiva desiderata a d = d_safe/2
d_ref     = d_safe / 2;
k_rep     = v_rep_ref / ((1/d_ref - 1/d_safe) * (1/d_ref^2));

%% 5. INIZIALIZZAZIONE STRUTTURA FLOTTA
% La formazione e' definita nel riferimento GLOBALE: la legge di consenso usa
% Delta_ij senza mai ruotarlo, quindi la "V" mantiene orientamento fisso
% rispetto alla mappa. E' un'approssimazione accettabile perche' il percorso si
% sviluppa prevalentemente lungo +Y.
%
% Le posizioni iniziali seguono ORA lo stesso criterio, senza rotazione. La
% versione precedente applicava rot_mat a pos_des solo qui e non nel consenso:
% con dir_iniziale ~ pi/2 questo ruotava la formazione di 90 gradi, facendo
% nascere i veicoli in una configurazione diversa da quella poi inseguita dal
% controllo. Con offset di 3 m l'effetto era invisibile; con offset di 20 m
% genererebbe un errore iniziale spurio di decine di metri.
%
% Il punto di partenza e' inoltre arretrato lungo il percorso quanto basta
% perche' i veicoli di coda non nascano fuori dalla mappa: la formazione e'
% profonda 30 m e il percorso parte da y = 0.
margine_start = (pos_des(1,2) - min(pos_des(:,2))) + 10;
idx_start = find(path_points(:,2) >= margine_start, 1);
if isempty(idx_start); idx_start = 1; end

% Il Master nasce esattamente su un waypoint; gli altri si dispongono attorno
% al centroide virtuale della formazione.
p_master0    = path_points(idx_start, :)';
origine_form = p_master0 - pos_des(1,:)';
dir_iniziale = atan2(path_points(idx_start+1,2) - path_points(idx_start,2), ...
                     path_points(idx_start+1,1) - path_points(idx_start,1));

for i = 1:N_veh
    fleet(i).x_true = zeros(5, N_steps);
    fleet(i).x_est  = zeros(5, N_steps);
    fleet(i).Sigma      = diag([2, 2, 0.1, 1, 1]);
    fleet(i).u_hist         = zeros(2, N_steps);    % comandi [v; w] applicati
    fleet(i).in_denied_hist = false(1, N_steps);    % storico copertura GPS
    fleet(i).Sigma_hist     = zeros(5, 5, N_steps); % storico covarianza, per
                                                    % 3-sigma bounds e test NEES

    % Offset geometrico iniziale per la formazione (riferimento globale)
    pos_iniziale = origine_form + pos_des(i,:)';

    fleet(i).x_true(:,1) = [pos_iniziale(1); pos_iniziale(2); dir_iniziale; 0; 0];
    fleet(i).Sigma_hist(:,:,1) = fleet(i).Sigma;
    fleet(i).x_est(:,1)  = fleet(i).x_true(:,1) + [(rand(2,1)-0.5)*2; 0; 0; 0]; % Piccolo errore iniziale
    
    if i == 1
        fleet(i).R_gps = R_gps_master;
    else
        fleet(i).R_gps = R_gps_slave;
    end
    
    fleet(i).target_idx = idx_start + 1; % Indice del percorso da inseguire
end

%% 6. MAIN SIMULATION LOOP
% -------------------------------------------------------------------------
% CONVENZIONE TEMPORALE E ORDINE DI ESECUZIONE: identici alla Fase 2.
%   (1) BROADCAST  (2) CONTROLLO  (3) IMPIANTO  (4) SENSORI  (5) STIMA
% Nessuno step usa informazione futura: il controllo in [t_k, t_{k+1}) dipende
% solo da x_est(:,k), e le misure che correggono la predizione a t_{k+1} sono
% generate dalla ground truth a t_{k+1}.
%
% SPECIFICITA' DELLA FASE 3
% a) La disponibilita' del GPS e' valutata sulla posizione REALE a t_{k+1}:
%    e' una proprieta' fisica dell'ambiente, non della stima del veicolo.
% b) Le misure di range inter-veicolare a t_{k+1} dipendono dalla ground truth
%    di TUTTI i veicoli. Per questo l'impianto (3) e' una passata completa su
%    tutta la flotta che PRECEDE la passata dei sensori (4).
% c) L'ancora mobile usata nella localizzazione collaborativa e' la stima del
%    vicino a t_k (ultimo pacchetto ricevuto) PROPAGATA di un passo con il suo
%    modello di moto. Due ragioni:
%      - rompe il loop algebrico fra i filtri, che altrimenti dipenderebbero
%        l'uno dalla stima aggiornata dell'altro nello stesso istante;
%      - e' cio' che un canale reale rende disponibile (in Fase 4 il ritardo
%        diventera' esplicito e variabile).
%    Senza la propagazione si confronterebbe una misura presa a t_{k+1} con una
%    posizione riferita a t_k: bias sistematico pari a |v_j|*Ts.
% -------------------------------------------------------------------------
disp('Simulazione in corso...');
N_end = N_steps;    % indice dell'ultimo campione valido (aggiornato all'arrivo)

for k = 1:N_steps-1

    % --- (1) BROADCAST ---------------------------------------------------
    % Punti di controllo dalle stime a t_k, usati dal consenso.
    p_ctrl = zeros(2, N_veh);
    for i = 1:N_veh
        xe = fleet(i).x_est(:, k);
        p_ctrl(1, i) = xe(1) + param.b * cos(xe(3));
        p_ctrl(2, i) = xe(2) + param.b * sin(xe(3));
    end

    % Stime dei vicini propagate a t_{k+1}: ancore mobili per la
    % localizzazione collaborativa (vedi nota (c) in testa al loop).
    p_ancora_mobile = zeros(2, N_veh);
    for j = 1:N_veh
        xj = fleet(j).x_est(:, k);
        p_ancora_mobile(1, j) = xj(1) + xj(4) * cos(xj(3)) * Ts;
        p_ancora_mobile(2, j) = xj(2) + xj(4) * sin(xj(3)) * Ts;
    end

    % --- (2) CONTROLLO + (3) IMPIANTO ------------------------------------
    for i = 1:N_veh
        u_cons  = [0; 0];
        F_rep   = [0; 0];
        V_rif_i = [0; 0];

        % Path following: solo il Master insegue il percorso nominale
        if i == 1
            idx = fleet(i).target_idx;
            target_pt = path_points(idx, :)';

            % Avanza il target virtuale quando il Master lo ha raggiunto.
            % La soglia e' la distanza di lookahead del pure-pursuit: 10 m e'
            % il doppio della lunghezza del mezzo, sufficiente a non inseguire
            % un punto praticamente coincidente con la propria posizione (che
            % renderebbe la direzione di riferimento numericamente instabile).
            if norm(p_ctrl(:, i) - target_pt) < 10.0 && idx < num_punti_path
                fleet(i).target_idx = idx + 1;
                target_pt = path_points(fleet(i).target_idx, :)';
            end

            v_dir   = (target_pt - p_ctrl(:, i)) / norm(target_pt - p_ctrl(:, i) + 1e-6);
            V_rif_i = v_cruise * v_dir;
        end

        % Consenso e repulsione, sulle stime condivise a t_k
        for j = 1:N_veh
            if i ~= j
                err_ij = (p_ctrl(:, i) - p_ctrl(:, j)) - Delta(:, i, j);
                u_cons = u_cons - K_cons * err_ij;

                dist = norm(p_ctrl(:, i) - p_ctrl(:, j));
                if dist < d_safe && dist > 0.1
                    grad_d  = (p_ctrl(:, i) - p_ctrl(:, j)) / dist;
                    rep_mag = k_rep * (1/dist - 1/d_safe) * (1/dist^2);
                    F_rep   = F_rep + rep_mag * grad_d;
                end
            end
        end

        p_dot_cmd = V_rif_i + u_cons + F_rep;

        % Feedback linearization valutata sulla STIMA a t_k
        th_est = fleet(i).x_est(3, k);
        T_fl_inv  = [ cos(th_est),           sin(th_est);
                  -sin(th_est)/param.b,   cos(th_est)/param.b ];

        vw_cmd = T_fl_inv * p_dot_cmd;
        v_cmd  = max(min(vw_cmd(1), v_max), -v_max);
        w_cmd  = max(min(vw_cmd(2), w_max), -w_max);
        fleet(i).u_hist(:, k) = [v_cmd; w_cmd];

        % (3) IMPIANTO
        % >>> PUNTO DI INNESTO DEL MODELLO DI SLITTAMENTO <<<
        % Oggi: tracking ideale dei motori, x_true(4,k+1) = v_cmd.
        % Domani: x_true(4,k+1) = f_slip(v_cmd, x_true(:,k), terreno, ...).
        xt = fleet(i).x_true(:, k);
        fleet(i).x_true(1, k+1) = xt(1) + xt(4) * cos(xt(3)) * Ts;
        fleet(i).x_true(2, k+1) = xt(2) + xt(4) * sin(xt(3)) * Ts;
        fleet(i).x_true(3, k+1) = wrapToPi(xt(3) + xt(5) * Ts);
        fleet(i).x_true(4, k+1) = v_cmd;
        fleet(i).x_true(5, k+1) = w_cmd;
    end

    % --- (4) SENSORI + (5) STIMA -----------------------------------------
    for i = 1:N_veh
        xt_next = fleet(i).x_true(:, k+1);      % stato reale a t_{k+1}

        % Predizione da t_k a t_{k+1}
        [x_pred, Sigma_bar] = ekf_predict(fleet(i).x_est(:, k), fleet(i).Sigma, Ts, par_Q);

        % --- Blocco base: sempre disponibile (IMU + encoder) ---
        z_imu = xt_next([3,5]) + chol(R_imu)' * randn(2,1);
        z_enc = [(xt_next(4) + (param.L/2)*xt_next(5))/param.r;
                 (xt_next(4) - (param.L/2)*xt_next(5))/param.r] + chol(R_enc)' * randn(2,1);

        z      = [z_imu; z_enc];
        z_pred = [ x_pred(3);
                   x_pred(5);
                  (x_pred(4) + (param.L/2)*x_pred(5))/param.r;
                  (x_pred(4) - (param.L/2)*x_pred(5))/param.r ];
        C_k      = [0 0 1 0 0;
                  0 0 0 0 1;
                  0 0 0 1/param.r  param.L/(2*param.r);
                  0 0 0 1/param.r -param.L/(2*param.r)];
        R_k  = blkdiag(R_imu, R_enc);

        % Maschera delle componenti ANGOLARI di z: l'innovazione va wrappata
        % solo dove ha senso. Cresce insieme a z, quindi resta corretta
        % qualunque sia l'ordine con cui le misure vengono accodate.
        is_angle = [true; false; false; false];     % solo theta dell'IMU

        % --- Disponibilita' GPS: proprieta' fisica, valutata su xt_next ---
        in_denied = false;
        for z_idx = 1:length(gps_denied_zones)
            centro = [gps_denied_zones(z_idx).xc; gps_denied_zones(z_idx).yc];
            if norm(xt_next(1:2) - centro) <= gps_denied_zones(z_idx).raggio
                in_denied = true;
                break;
            end
        end
        fleet(i).in_denied_hist(k+1) = in_denied;

        if ~in_denied
            % --- GPS attivo: misura diretta di posizione assoluta ---
            z_gps    = xt_next(1:2) + chol(fleet(i).R_gps)' * randn(2,1);
            z        = [z; z_gps];
            z_pred   = [z_pred; x_pred(1:2)];
            C_k        = [C_k; 1 0 0 0 0; 0 1 0 0 0];
            R_k    = blkdiag(R_k, fleet(i).R_gps);
            is_angle = [is_angle; false; false];
        else
            % --- GPS negato: ranging UWB verso le ancore FISSE ---
            for a_idx = 1:size(uwb_opt, 1)
                p_anc  = uwb_opt(a_idx, :)';
                d_true = norm(xt_next(1:2) - p_anc);
                if d_true <= r_ancora
                    d_est = max(norm(x_pred(1:2) - p_anc), 0.1);   % previene div/0

                    z        = [z; d_true + sigma_uwb * randn()];
                    z_pred   = [z_pred; d_est];
                    C_k        = [C_k; (x_pred(1)-p_anc(1))/d_est, (x_pred(2)-p_anc(2))/d_est, 0, 0, 0];
                    R_k    = blkdiag(R_k, sigma_uwb^2);
                    is_angle = [is_angle; false];
                end
            end

            % --- Localizzazione collaborativa: vicini come ancore MOBILI ---
            for j = 1:N_veh
                if i ~= j
                    % Misura FISICA: fra le posizioni reali a t_{k+1}
                    d_true = norm(xt_next(1:2) - fleet(j).x_true(1:2, k+1));
                    if d_true <= r_collab
                        % Predizione della misura: usa la stima CONDIVISA del
                        % vicino (propagata), non la sua posizione reale, che
                        % il veicolo i non puo' conoscere.
                        p_j   = p_ancora_mobile(:, j);
                        d_est = max(norm(x_pred(1:2) - p_j), 0.1);

                        z        = [z; d_true + sigma_collab * randn()];
                        z_pred   = [z_pred; d_est];
                        C_k        = [C_k; (x_pred(1)-p_j(1))/d_est, (x_pred(2)-p_j(2))/d_est, 0, 0, 0];
                        % LIMITE NOTO: R contiene solo il rumore del sensore.
                        % L'incertezza Sigma_j della stima del vicino e' ignorata,
                        % quindi il filtro risulta OTTIMISTA.
                        %
                        % CORREZIONE PROGRAMMATA IN FASE 5 (README §4, punto 4):
                        %   R_eff = sigma_collab^2 + u' * Sigma_j(1:2,1:2) * u
                        % con u versore della congiungente. Richiede di estendere
                        % il PACCHETTO SCAMBIATO fra veicoli: oggi contiene la sola
                        % posizione stimata (vedi p_ancora_mobile sopra), dovra'
                        % contenere anche il blocco 2x2 della covarianza del
                        % mittente. Da 2 a 5 numeri, da 16 a 40 byte.
                        % Sopra a questo si innesta la Covariance Intersection,
                        % che affronta la correlazione ignota fra le stime.
                        R_k    = blkdiag(R_k, sigma_collab^2);
                        is_angle = [is_angle; false];
                    end
                end
            end
        end

        % (5) Aggiornamento
        [fleet(i).x_est(:, k+1), fleet(i).Sigma] = ...
            ekf_update(x_pred, Sigma_bar, z, z_pred, C_k, R_k, is_angle);
        fleet(i).Sigma_hist(:,:,k+1) = fleet(i).Sigma;
    end

    % --- Terminazione: il Master ha completato il percorso ----------------
    % Evita di simulare centinaia di secondi con la flotta ferma sull'ultimo
    % waypoint, che falserebbero qualsiasi statistica calcolata sul run.
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

%% 7. PLOT RISULTATI
figure('Name', 'Fase 3: Navigazione e Sensor Fusion', 'Color', 'w', 'Position', [100 100 800 800]);
hold on; grid on; axis equal; axis([0 W_MAP 0 H_MAP]);

% Disegna Zone GPS-Denied
for z_idx = 1:length(gps_denied_zones)
    th_c = linspace(0, 2*pi, 100);
    x_c = gps_denied_zones(z_idx).xc + gps_denied_zones(z_idx).raggio * cos(th_c);
    y_c = gps_denied_zones(z_idx).yc + gps_denied_zones(z_idx).raggio * sin(th_c);
    patch(x_c, y_c, 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end

% Disegna Percorso
plot(path_points(:,1), path_points(:,2), 'k--', 'LineWidth', 1, 'DisplayName', 'Path Nominale');

% Disegna Ancore UWB
if ~isempty(uwb_opt)
    plot(uwb_opt(:,1), uwb_opt(:,2), 'b^', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Ancore UWB');
end

% Disegna Traiettorie Veicoli
colors = ['b', 'r', 'g'];
% N_end e' l'ultimo campione valido, restituito dal loop alla fine del percorso.
% Sostituisce la ricerca euristica del primo zero in x_true, che confondeva un
% campione non simulato con un veicolo realmente transitato per x = 0.
for i = 1:N_veh
    plot(fleet(i).x_true(1, 1:N_end), fleet(i).x_true(2, 1:N_end), [colors(i) '-'], 'LineWidth', 1.5, 'DisplayName', sprintf('True V%d', i));
    plot(fleet(i).x_est(1, 1:N_end),  fleet(i).x_est(2, 1:N_end),  [colors(i) ':'], 'LineWidth', 1.5, 'DisplayName', sprintf('Est V%d', i));

    % Marker finale
    plot(fleet(i).x_true(1, N_end), fleet(i).x_true(2, N_end), [colors(i) 'o'], 'MarkerFaceColor', colors(i));
end

title('Fase 3: Navigazione in Zone GPS-Denied'); xlabel('X [m]'); ylabel('Y [m]'); legend('Location', 'best');

%% ========================================================================
% FUNZIONI LOCALI EKF
% =========================================================================

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
