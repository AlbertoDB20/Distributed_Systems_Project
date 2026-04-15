% =========================================================================
% FASE 2: Flotta N=3, Consenso e GPS Differenziato
% =========================================================================
clear; clc; close all;

lin_traj = false; 

%% 1. PARAMETRI DI SISTEMA
param.r = 0.5;         % [m] raggio ruote
param.L = 1.0;         % [m] distanza tra le ruote (carreggiata)
param.b = 0.3;         % [m] distanza punto di controllo P per feedback linearization --> siccome il Differential Drive non può traslare lateralmente, definiamo un "punto virtuale" di controllo che si trova 0.3 m in avanti rispetto al centro del veicolo. Questo punto è quello che cercheremo di far seguire alla formazione, dato che può muoversi in ogni direzione in modo fluido.
v_max = 7.0;           % [m/s]
f_s = 10;              % [Hz] frequenza 
Ts = 1/f_s;            % [s]
t_end = 40;            % [s]
t = 0:Ts:t_end;
N_steps = length(t);
N_veh = 3;             % Numero di veicoli

%% 2. PARAMETRI DEL FILTRO E SENSORI
sigma_v_proc = 0.5;   
sigma_w_proc = 0.2;   
Q = diag([0.01, 0.01, 0.01, sigma_v_proc^2, sigma_w_proc^2]);       % Covarianza del rumore di PROCESSO (modello dinamico)

% GPS: Master (1) ha qualità eccellente, Slave (2,3) standard
sigma_gps_master = 0.2;  % [m]
sigma_gps_slave  = 2.0;  % [m]

sigma_imu_th = 0.05;     % [rad]
sigma_imu_w  = 0.02;     % [rad/s]
R_imu = diag([sigma_imu_th^2, sigma_imu_w^2]);      % Covarianza del rumore di MISURA (sensore IMU)

sigma_enc_wRL = 0.1;     % [rad/s]
R_enc = diag([sigma_enc_wRL^2, sigma_enc_wRL^2]);   % Covarianza del rumore di MISURA (sensore Encoders)

%% 3. PARAMETRI CONTROLLO DI FORMAZIONE
% Vogliamo una formazione a triangolo ("V" shape) che si muove in diagonale
if lin_traj == true
    V_ref = [1.5; 1.5]; % [m/s] Velocità globale di avanzamento della flotta
else 
    % Parametri Traiettoria Sinusoidale
    v_x_ref = 1.0;      % [m/s] Velocità di avanzamento costante lungo X
    A_sin = 2.0;        % [m] Ampiezza dell'oscillazione lungo Y (spostamento laterale)
    omega_sin = 0.5;    % [rad/s] Frequenza dell'oscillazione
end

% pos_des indica dove deve stare ogni robot rispetto al "baricentro virtuale" del gruppo.
% Veh 1 (Master) al centro/avanti, Veh 2 a sx, Veh 3 a dx
pos_des = [ 0.0,  2.0;  % V1 (X, Y) Master
           -2.0, -2.0;  % V2 (X, Y)
            2.0, -2.0]; % V3 (X, Y)

% Calcolo matrice delle distanze relative desiderate Delta_ij --> necessaria perchè i veicoli devono mantenere una certa distanza reciproca per formare la "V", non semplicemente una distanza di raggio r (altrimenti potrebbero essere in ogni posizione nella circonferenza di raggio r).)
Delta = zeros(2, N_veh, N_veh);
for i = 1:N_veh
    for j = 1:N_veh
        Delta(:, i, j) = pos_des(i,:)' - pos_des(j,:)';
    end
end

% K_cons e k_rep sono la rigidità delle "molle" invisibili che collegano i veicoli. Il Consenso li tira verso la posizione giusta, la Repulsione li allontana se la distanza scende sotto il raggio di sicurezza R_safe.
K_cons = 1.0;       % Guadagno Consenso
R_safe = 1.5;       % [m] Raggio attivazione repulsione
k_rep  = 2.0;       % Guadagno forza repulsiva

%% 4. INIZIALIZZAZIONE STRUTTURA FLOTTA
% Inizializziamo i veicoli con posizioni casuali per vedere il transitorio --> questa posizione iniziale non la conosciamo nella realtà, in simultazione è solo per testare la capacità del sistema di convergere alla formazione desiderata partendo da una configurazione disordinata.
x0_true = [ -5, -4, 0, 0, 0; 
             4, -5, 0, 0, 0; 
            -1, -8, 0, 0, 0 ]';         % (X, Y, theta, V, W) per ogni veicolo

for i = 1:N_veh
    fleet(i).x_true = zeros(5, N_steps);        % 5 stati: X, Y, theta, V, W
    fleet(i).x_est  = zeros(5, N_steps);
    fleet(i).P      = diag([5, 5, 0.5, 1, 1]);      % Covarianza iniziale --> grande incertezza sulla posizione iniziale (+- 5 m), un po' meno su theta (+- 0.5 rad = 28 gradi), e ancora meno su V e W (+- 1 m/s) siccome quando accendo la macchina so di essere fermo.
    fleet(i).z_hist = zeros(6, N_steps);        % 6 misure: GPS(2), IMU(2), Enc(2)
    
    fleet(i).x_true(:,1) = x0_true(:,i);
    % L'EKF parte con una stima leggermente sbagliata --> questa stima iniziale non è perfetta. sto mentendo al EKF per vedere se riesce a correggersi e convergere alla stima corretta. Se partisse già con la stima perfetta, non vedremmo il processo di correzione e convergenza.
    fleet(i).x_est(:,1)  = x0_true(:,i) + [1; -1; 0.2; 0; 0];    % questa è la stima iniziale:+1 m in X, -1 m in Y, +0.2 rad in theta (circa 11 gradi), stessa stima per V e W
    
    % Assegna R_gps (Covarianza del rumore di MISURA GPS) specifica in base al ruolo
    if i == 1
        fleet(i).R_gps = diag([sigma_gps_master^2, sigma_gps_master^2]);
    else
        fleet(i).R_gps = diag([sigma_gps_slave^2, sigma_gps_slave^2]);
    end
end

%% 5. MAIN SIMULATION LOOP
for k = 1:N_steps-1
    if lin_traj == false
        t_k = t(k);
        v_y_ref = A_sin * omega_sin * cos(omega_sin * t_k);
        V_ref = [v_x_ref; v_y_ref]; % Vettore velocità desiderata al tempo k
    end

    % --- A. Lettura Sensori e Aggiornamento EKF per tutti i veicoli ---
    for i = 1:N_veh
        % 1. Genera Misure Reali sporcate
        xt = fleet(i).x_true(:, k);         % --> ground truth al tempo k per il veicolo i (non deve essere noto al EKF!)
        z_gps = xt(1:2) + sqrt(fleet(i).R_gps) * randn(2,1);    % la misura GPS è data dalla posizione grond truth più un rumore gaussiano con deviazione standard specifica per quel veicolo (master o slave) --> avro (x_gps, y_gps) che è la simulazione di ciò che il vero GPS legge, quindi una posizione rumorosa intorno alla posizione reale del veicolo.
        z_imu = xt([3,5]) + sqrt(R_imu) * randn(2,1);       % prendo il 3° elemento (l'angolo θ, misurato dal magnetometro/bussola) e il 5° elemento (la velocità angolare ω, misurata dal giroscopio) e ci aggiungo un rumore gaussiano con deviazione standard specifica per l'IMU. Quindi z_imu è la simulazione di ciò che l'IMU legge, ovvero una stima rumorosa dell'angolo e della velocità angolare del veicolo. 
        wR_t = (xt(4) + (param.L/2)*xt(5)) / param.r;       % simulo encoder (sfrutto cinematica inversa per ottenere la velocità angolare della ruota destra wR a partire dalla velocità lineare v (xt(4)) e dalla velocità angolare w (xt(5)) del veicolo, considerando la geometria del Differential Drive.
        wL_t = (xt(4) - (param.L/2)*xt(5)) / param.r;
        z_enc = [wR_t; wL_t] + sqrt(R_enc) * randn(2,1);
        
        z = [z_gps; z_imu; z_enc];      % vettore che simula le misure!
        fleet(i).z_hist(:, k+1) = z; % Salviamo k+1 per coerenza con l'aggiornamento
        
        % 2. Esegui step EKF
        R_tot = blkdiag(fleet(i).R_gps, R_imu, R_enc);   % matrice di covarianza totale (6x6) --> assumiamo che i sensori siano indipendenti, quindi avremo una matrice diagonale.
        [fleet(i).x_est(:, k+1), fleet(i).P] = ekf_step(fleet(i).x_est(:, k), fleet(i).P, z, Ts, Q, R_tot, param); % EKF step: prendo la stima precedente, la covarianza, la nuova misura, e aggiorno la stima e la covarianza. La stima aggiornata x_est(:, k+1) è quella che useremo per il controllo.
    end
    
    % --- B. Scambio Informazioni (Canale Ideale) e Controllo Consenso ---
    % Feedback Linearization: Estraiamo le posizioni STIMATE e le mappiamo sul punto di controllo p_i --> avanti di 0.3 m rispetto al centro del veicolo.
    p_est = zeros(2, N_veh);
    for i = 1:N_veh
        xe = fleet(i).x_est(:, k+1);
        p_est(1, i) = xe(1) + param.b * cos(xe(3));
        p_est(2, i) = xe(2) + param.b * sin(xe(3));
    end
    
    % Calcolo del controllo per ogni veicolo tramite CAMPI POTENZIALI ARTIFICIALI
    for i = 1:N_veh
        F_cons = [0; 0];            % N.B. IMP: "Forza" di attrazione e repulsione. Sono in [m/s]!!! 
        F_rep = [0; 0];             

        for j = 1:N_veh
            if i ~= j           % il consenso non lo faccio con me stesso, solo con gli altri ...
                % 1. Consenso: ATTRAZIONE verso la formazione (LINEARE)
                err_ij = (p_est(:, i) - p_est(:, j)) - Delta(:, i, j);      % distanza stimata dei due veicoli (p_est(:, i) - p_est(:, j)) MENO la distanza desiderata Delta(:, i, j) --> se troppo vicini, err_ij sarà negativo, se troppo lontani sarà positivo. Il consenso cerca di annullare questo errore.
                F_cons = F_cons - K_cons * err_ij;      % Legge di Hooke: F = - K * x
                
                % 2. Evitamento Collisioni: REPULSIONE (NON LINEARE, condizione if)
                dist = norm(p_est(:, i) - p_est(:, j));
                if dist < R_safe && dist > 0.1 
                    grad_d = (p_est(:, i) - p_est(:, j)) / dist;        % vettore unitario che punta da j verso i (direzione di repulsione)
                    rep_mag = k_rep * (1/dist - 1/R_safe) * (1/dist^2);    % maggiore è la vicinanza (dist più piccolo), maggiore è la forza repulsiva, con un termine di scaling che annulla la forza a R_safe.
                    F_rep = F_rep + rep_mag * grad_d;          % la forza totale di repulsione è la somma delle forze repulsive da tutti gli altri veicoli che sono troppo vicini.
                end
            end
        end
        
        % Velocità desiderata del punto p_i
        p_dot_cmd = V_ref + F_cons + F_rep;         % vettore velocità (v_x, v_y) finale del nostro punto virtuale. È la somma di tre intenti: "vai avanti insieme agli altri" (V_ref), "stai in formazione" (F_cons) e "non collidere" (F_rep).
        
        % Inversione (Feedback Linearization) per ottenere v, w
        theta_est = fleet(i).x_est(3, k+1);
        R_inv = [ cos(theta_est),     sin(theta_est); 
                 -sin(theta_est)/param.b, cos(theta_est)/param.b ];
             
        vw_cmd = R_inv * p_dot_cmd;
        
        % Saturazione dei comandi (realismo)
        v_cmd = max(min(vw_cmd(1), v_max), -v_max);
        w_cmd = max(min(vw_cmd(2), 2.0), -2.0);
        
        % --- C. Aggiornamento Dinamica Reale (Ground Truth) ---
        xt = fleet(i).x_true(:, k);
        fleet(i).x_true(1, k+1) = xt(1) + xt(4) * cos(xt(3)) * Ts;
        fleet(i).x_true(2, k+1) = xt(2) + xt(4) * sin(xt(3)) * Ts;
        fleet(i).x_true(3, k+1) = wrapToPi(xt(3) + xt(5) * Ts);
        fleet(i).x_true(4, k+1) = v_cmd; % Assumiamo tracking ideale dei motori
        fleet(i).x_true(5, k+1) = w_cmd;
    end
end

%% 6. PLOT RISULTATI
figure('Name','Traiettoria Flotta e Formazione','Color','w'); hold on; grid on; axis equal;
colors = ['b', 'r', 'g'];
for i = 1:N_veh
    % Plot Ground Truth
    plot(fleet(i).x_true(1,:), fleet(i).x_true(2,:), [colors(i) '--'], 'LineWidth', 1.5, 'DisplayName', sprintf('True V%d', i));
    % Plot Stima EKF
    plot(fleet(i).x_est(1,:), fleet(i).x_est(2,:), [colors(i) '-'], 'LineWidth', 2, 'DisplayName', sprintf('Est V%d', i));
    % Plot punto finale
    plot(fleet(i).x_est(1,end), fleet(i).x_est(2,end), [colors(i) 'o'], 'MarkerFaceColor', colors(i), 'MarkerSize', 8);
end
title('Fase 2: Controllo di Formazione Distribuito'); xlabel('X [m]'); ylabel('Y [m]'); legend('Location','best');

% Grafico degli Errori di Stima GPS
figure('Name','Confronto Errori Stima X','Color','w');
for i = 1:N_veh
    subplot(3, 1, i);
    err_x = fleet(i).x_true(1,:) - fleet(i).x_est(1,:);
    plot(t, err_x, colors(i)); grid on;
    if i == 1
        title('Errore Posizione X - MASTER (GPS Alta Precisione)');
    else
        title(sprintf('Errore Posizione X - SLAVE %d (GPS Standard)', i));
    end
    ylabel('Errore [m]');
end
xlabel('Tempo [s]');

%% ========================================================================
% FUNZIONI LOCALI
% =========================================================================

function [x_new, P_new] = ekf_step(x_old, P_old, z, Ts, Q, R, param)
    % 1. PREDIZIONE
    v_est = x_old(4); w_est = x_old(5); th_est = x_old(3);
    
    x_pred = x_old;
    x_pred(1) = x_pred(1) + v_est * cos(th_est) * Ts;
    x_pred(2) = x_pred(2) + v_est * sin(th_est) * Ts;
    x_pred(3) = x_pred(3) + w_est * Ts;
    
    A_k = eye(5);
    A_k(1, 3) = -v_est * sin(th_est) * Ts;
    A_k(1, 4) = cos(th_est) * Ts;
    A_k(2, 3) = v_est * cos(th_est) * Ts;
    A_k(2, 4) = sin(th_est) * Ts;
    A_k(3, 5) = Ts;
    
    P_pred = A_k * P_old * A_k' + Q;
    
    % 2. UPDATE
    z_pred_gps = [x_pred(1); x_pred(2)];
    z_pred_imu = [x_pred(3); x_pred(5)];
    z_pred_enc = [(x_pred(4) + (param.L/2)*x_pred(5)) / param.r; 
                  (x_pred(4) - (param.L/2)*x_pred(5)) / param.r];
    z_pred = [z_pred_gps; z_pred_imu; z_pred_enc];
    
    C_gps = [1 0 0 0 0; 0 1 0 0 0];
    C_imu = [0 0 1 0 0; 0 0 0 0 1];
    C_enc = [0 0 0, 1/param.r,  param.L/(2*param.r);
             0 0 0, 1/param.r, -param.L/(2*param.r)];
    C_k = [C_gps; C_imu; C_enc];
    
    S = C_k * P_pred * C_k' + R;
    K = P_pred * C_k' / S;
    
    y_innov = z - z_pred;
    y_innov(3) = wrapToPi(y_innov(3));
    
    x_new = x_pred + K * y_innov;
    x_new(3) = wrapToPi(x_new(3));
    
    P_new = (eye(5) - K * C_k) * P_pred;
end