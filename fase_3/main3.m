% =========================================================================
% FASE 3: Navigazione in Ambiente Ostile e Localizzazione Collaborativa
% =========================================================================
clear; clc; close all;

%% 1. CARICAMENTO AMBIENTE
try
    load('ambiente_fase3.mat');
    disp('Ambiente caricato con successo.');
catch
    error('File ambiente_fase3.mat non trovato. Esegui prima lo script di generazione.');
end

%% 2. PARAMETRI DI SISTEMA
param.r = 0.5;         % [m] raggio ruote
param.L = 1.0;         % [m] carreggiata
param.b = 0.3;         % [m] punto feedback linearization
v_max = 5.0;           % [m/s]
w_max = 2.0;           % [rad/s]
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
Q = diag([0.05, 0.05, 0.01, 0.5^2, 0.2^2]); % Rumore di processo

% Rumori di misura
R_gps_master = diag([0.2^2, 0.2^2]);
R_gps_slave  = diag([2.0^2, 2.0^2]);
R_imu        = diag([0.05^2, 0.02^2]);   % [th, w]
R_enc        = diag([0.1^2, 0.1^2]);     % [wR, wL]
sigma_uwb    = 0.5;                      % [m] Rumore distanza ancore UWB
sigma_collab = 1.0;                      % [m] Rumore distanza inter-veicolare

r_collab = 40; % [m] Raggio massimo per usare un vicino come "ancora collaborativa"

%% 4. PARAMETRI FORMAZIONE
pos_des = [ 0.0,  3.0;  % V1 (Master)
           -3.0, -3.0;  % V2
            3.0, -3.0]; % V3
Delta = zeros(2, N_veh, N_veh);
for i = 1:N_veh
    for j = 1:N_veh
        Delta(:, i, j) = pos_des(i,:)' - pos_des(j,:)';
    end
end
K_cons = 1.2;       
R_safe = 2.0;       
k_rep  = 3.0;       

%% 5. INIZIALIZZAZIONE STRUTTURA FLOTTA
% Posizioniamo i veicoli vicino al punto di partenza del percorso
start_pt = path_points(1, :)';
dir_iniziale = atan2(path_points(2,2)-path_points(1,2), path_points(2,1)-path_points(1,1));

for i = 1:N_veh
    fleet(i).x_true = zeros(5, N_steps);
    fleet(i).x_est  = zeros(5, N_steps);
    fleet(i).P      = diag([2, 2, 0.1, 1, 1]);
    
    % Offset geometrico iniziale per la formazione
    rot_mat = [cos(dir_iniziale), -sin(dir_iniziale); sin(dir_iniziale), cos(dir_iniziale)];
    pos_iniziale = start_pt + rot_mat * pos_des(i,:)';
    
    fleet(i).x_true(:,1) = [pos_iniziale(1); pos_iniziale(2); dir_iniziale; 0; 0];
    fleet(i).x_est(:,1)  = fleet(i).x_true(:,1) + [(rand(2,1)-0.5)*2; 0; 0; 0]; % Piccolo errore iniziale
    
    if i == 1
        fleet(i).R_gps = R_gps_master;
    else
        fleet(i).R_gps = R_gps_slave;
    end
    
    fleet(i).target_idx = 2; % Indice del percorso da inseguire
end

%% 6. MAIN SIMULATION LOOP
disp('Simulazione in corso...');
for k = 1:N_steps-1
    
    % Estraiamo le stime condivise k-esime per la localizzazione collaborativa
    p_est_condivise = zeros(2, N_veh);
    for i = 1:N_veh
        p_est_condivise(:, i) = fleet(i).x_est(1:2, k);
    end
    
    % --- A. Lettura Sensori e EKF (Sensor Fusion Dinamica) ---
    for i = 1:N_veh
        xt = fleet(i).x_true(:, k);
        xe_old = fleet(i).x_est(:, k);
        P_old = fleet(i).P;
        
        % 1. Verifica se siamo in zona GPS-denied
        in_denied = false;
        for z_idx = 1:length(gps_denied_zones)
            if norm(xt(1:2) - [gps_denied_zones(z_idx).xc; gps_denied_zones(z_idx).yc]) <= gps_denied_zones(z_idx).R
                in_denied = true;
                break;
            end
        end
        
        % 2. Generazione Misure Base (Sempre presenti: IMU + ENC)
        z_imu = xt([3,5]) + sqrt(R_imu) * randn(2,1);
        z_enc = [(xt(4) + (param.L/2)*xt(5))/param.r; 
                 (xt(4) - (param.L/2)*xt(5))/param.r] + sqrt(R_enc) * randn(2,1);
        
        z = [z_imu; z_enc];
        R_dinamica = blkdiag(R_imu, R_enc);
        
        % 3. Predizione Base
        [x_pred, P_pred] = ekf_predict(xe_old, P_old, Ts, Q);
        
        % Costruzione dinamica di z_pred e H
        z_pred = [x_pred(3); x_pred(5); 
                 (x_pred(4) + (param.L/2)*x_pred(5))/param.r; 
                 (x_pred(4) - (param.L/2)*x_pred(5))/param.r];
             
        H = [0 0 1 0 0; 
             0 0 0 0 1; 
             0 0 0 1/param.r  param.L/(2*param.r); 
             0 0 0 1/param.r -param.L/(2*param.r)];
             
        % 4. Logica GPS vs UWB vs Collaborativa
        if ~in_denied
            % GPS Attivo
            z_gps = xt(1:2) + sqrt(fleet(i).R_gps) * randn(2,1);
            z = [z_gps; z];
            z_pred = [x_pred(1:2); z_pred];
            H = [[1 0 0 0 0; 0 1 0 0 0]; H];
            R_dinamica = blkdiag(fleet(i).R_gps, R_dinamica);
        else
            % GPS Negato -> Usa UWB
            if ~isempty(uwb_opt)
                for a_idx = 1:size(uwb_opt, 1)
                    dist_true = norm(xt(1:2) - uwb_opt(a_idx, :)');
                    if dist_true <= r_ancora
                        z_uwb = dist_true + sigma_uwb * randn();
                        dist_est = norm(x_pred(1:2) - uwb_opt(a_idx, :)');
                        dist_est = max(dist_est, 0.1); % Previeni div/0
                        
                        z = [z_uwb; z];
                        z_pred = [dist_est; z_pred];
                        H_row = [(x_pred(1) - uwb_opt(a_idx, 1))/dist_est, (x_pred(2) - uwb_opt(a_idx, 2))/dist_est, 0, 0, 0];
                        H = [H_row; H];
                        R_dinamica = blkdiag(sigma_uwb^2, R_dinamica);
                    end
                end
            end
            
            % Localizzazione Collaborativa: Usa i vicini come ancore
            for j = 1:N_veh
                if i ~= j
                    dist_true = norm(xt(1:2) - fleet(j).x_true(1:2, k));
                    if dist_true <= r_collab
                        z_collab = dist_true + sigma_collab * randn();
                        % Calcolo distanza stimata usando la posizione stimata e condivisa del vicino j
                        dist_est = norm(x_pred(1:2) - p_est_condivise(:, j));
                        dist_est = max(dist_est, 0.1);
                        
                        z = [z_collab; z];
                        z_pred = [dist_est; z_pred];
                        H_row = [(x_pred(1) - p_est_condivise(1, j))/dist_est, (x_pred(2) - p_est_condivise(2, j))/dist_est, 0, 0, 0];
                        H = [H_row; H];
                        R_dinamica = blkdiag(sigma_collab^2, R_dinamica);
                    end
                end
            end
        end
        
        % 5. Aggiornamento EKF
        [fleet(i).x_est(:, k+1), fleet(i).P] = ekf_update(x_pred, P_pred, z, z_pred, H, R_dinamica);
    end
    
    % --- B. Controllo Distribuito e Path Following ---
    p_cntrl = zeros(2, N_veh);
    for i = 1:N_veh
        xe = fleet(i).x_est(:, k+1);
        p_cntrl(1, i) = xe(1) + param.b * cos(xe(3));
        p_cntrl(2, i) = xe(2) + param.b * sin(xe(3));
    end
    
    for i = 1:N_veh
        u_cons = [0; 0];
        F_rep = [0; 0];
        V_rif_i = [0; 0];
        
        % Path Following solo per il Master
        if i == 1
            idx = fleet(i).target_idx;
            target_pt = path_points(idx, :)';
            dist_to_target = norm(p_cntrl(:, i) - target_pt);
            
            % Avanza indice se vicino al target
            if dist_to_target < 3.0 && idx < num_punti_path
                fleet(i).target_idx = idx + 1;
                target_pt = path_points(fleet(i).target_idx, :)';
            end
            
            % Vettore di riferimento verso il target
            v_dir = (target_pt - p_cntrl(:, i)) / norm(target_pt - p_cntrl(:, i) + 1e-6);
            V_rif_i = 2.5 * v_dir; % Velocità crociera Master: 2.5 m/s
        end
        
        % Consenso e Repulsione
        for j = 1:N_veh
            if i ~= j
                err_ij = (p_cntrl(:, i) - p_cntrl(:, j)) - Delta(:, i, j);
                u_cons = u_cons - K_cons * err_ij;
                
                dist = norm(p_cntrl(:, i) - p_cntrl(:, j));
                if dist < R_safe && dist > 0.1
                    grad_d = (p_cntrl(:, i) - p_cntrl(:, j)) / dist;
                    rep_mag = k_rep * (1/dist - 1/R_safe) * (1/dist^2);
                    F_rep = F_rep + rep_mag * grad_d;
                end
            end
        end
        
        p_dot_cmd = V_rif_i + u_cons + F_rep;
        
        % Feedback Linearization
        th_est = fleet(i).x_est(3, k+1);
        R_inv = [ cos(th_est),             sin(th_est); 
                 -sin(th_est)/param.b, cos(th_est)/param.b ];
             
        vw_cmd = R_inv * p_dot_cmd;
        v_cmd = max(min(vw_cmd(1), v_max), -v_max);
        w_cmd = max(min(vw_cmd(2), w_max), -w_max);
        
        % Dinamica Reale
        xt = fleet(i).x_true(:, k);
        fleet(i).x_true(1, k+1) = xt(1) + xt(4) * cos(xt(3)) * Ts;
        fleet(i).x_true(2, k+1) = xt(2) + xt(4) * sin(xt(3)) * Ts;
        fleet(i).x_true(3, k+1) = wrapToPi(xt(3) + xt(5) * Ts);
        fleet(i).x_true(4, k+1) = v_cmd;
        fleet(i).x_true(5, k+1) = w_cmd;
    end
end
disp('Simulazione completata.');

%% 7. PLOT RISULTATI
figure('Name', 'Fase 3: Navigazione e Sensor Fusion', 'Color', 'w', 'Position', [100 100 800 800]);
hold on; grid on; axis equal; axis([0 W_MAP 0 H_MAP]);

% Disegna Zone GPS-Denied
for z_idx = 1:length(gps_denied_zones)
    th_c = linspace(0, 2*pi, 100);
    x_c = gps_denied_zones(z_idx).xc + gps_denied_zones(z_idx).R * cos(th_c);
    y_c = gps_denied_zones(z_idx).yc + gps_denied_zones(z_idx).R * sin(th_c);
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
for i = 1:N_veh
    % Trova un punto in cui finisce la simulazione (escludiamo zeri se t_end eccessivo)
    idx_end = find(fleet(i).x_true(1,:) == 0, 1) - 1;
    if isempty(idx_end); idx_end = N_steps; end
    
    plot(fleet(i).x_true(1, 1:idx_end), fleet(i).x_true(2, 1:idx_end), [colors(i) '-'], 'LineWidth', 1.5, 'DisplayName', sprintf('True V%d', i));
    plot(fleet(i).x_est(1, 1:idx_end), fleet(i).x_est(2, 1:idx_end), [colors(i) ':'], 'LineWidth', 1.5, 'DisplayName', sprintf('Est V%d', i));
    
    % Marker finale
    plot(fleet(i).x_true(1, idx_end), fleet(i).x_true(2, idx_end), [colors(i) 'o'], 'MarkerFaceColor', colors(i));
end

title('Fase 3: Navigazione in Zone GPS-Denied'); xlabel('X [m]'); ylabel('Y [m]'); legend('Location', 'best');

%% ========================================================================
% FUNZIONI LOCALI EKF
% =========================================================================

function [x_pred, P_pred] = ekf_predict(x_old, P_old, Ts, Q)
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
    
    P_pred = A_k * P_old * A_k' + Q;
end

function [x_new, P_new] = ekf_update(x_pred, P_pred, z, z_pred, H, R)
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    
    y_innov = z - z_pred;
    
    % Wrap dell'innovazione per gli angoli. 
    % L'angolo theta proviene dall'IMU. L'IMU e l'Encoder sono sempre
    % aggiunti per ultimi nel vettore z. Theta dell'IMU è il terzultimo elemento.
    idx_theta = length(z) - 3; 
    y_innov(idx_theta) = wrapToPi(y_innov(idx_theta));
    
    x_new = x_pred + K * y_innov;
    x_new(3) = wrapToPi(x_new(3));
    
    P_new = (eye(5) - K * H) * P_pred;
end