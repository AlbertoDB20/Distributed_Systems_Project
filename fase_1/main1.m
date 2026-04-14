% =========================================================================
% FASE 1: Veicolo Singolo Ideale (EKF Sincrono a 10 Hz)
% =========================================================================
clear; clc; close all;

%% 1. PARAMETRI DI SISTEMA
r_veicolo = 0.5;      % [m] raggio ruote
L_veicolo = 1.0;      % [m] distanza tra le ruote
v_max = 7.0;          % [m/s] velocità massima
f_s = 10;             % [Hz] frequenza di campionamento
Ts = 1/f_s;           % [s] tempo di campionamento
t_end = 60;           % [s] durata simulazione
t = 0:Ts:t_end;
N_steps = length(t);

%% 2. PARAMETRI DEL FILTRO
% Q: Covarianza del rumore di processo (incertezza del modello / slittamento)
% Varianze: X, Y, theta (piccole, dipendono da v e w), V, W (più alte)
sigma_v_proc = 0.5;   % Incertezza sull'accelerazione/slittamento
sigma_w_proc = 0.2;   
Q = diag([0.01, 0.01, 0.01, sigma_v_proc^2, sigma_w_proc^2]);

% R: Covarianza dei sensori
sigma_gps_xy = 1.5;   % [m] rumore GPS 
R_gps = diag([sigma_gps_xy^2, sigma_gps_xy^2]);

sigma_imu_th = 0.05;  % [rad] rumore magnetometro
sigma_imu_w  = 0.02;  % [rad/s] rumore giroscopio
R_imu = diag([sigma_imu_th^2, sigma_imu_w^2]);

sigma_enc_wRL = 0.1;  % [rad/s] rumore misura ruote
R_enc = diag([sigma_enc_wRL^2, sigma_enc_wRL^2]);

% Matrice R globale (Blocchi diagonali)
R = blkdiag(R_gps, R_imu, R_enc);

%% 3. GENERAZIONE DEL GROUND TRUTH (Realtà Simulata)
x_true = zeros(5, N_steps);
% Condizioni iniziali
x_true(:,1) = [0; 0; 0; 0; 0]; 

% Generiamo comandi di V e W fittizi per fare un percorso a "8"
v_cmd = 3 * ones(1, N_steps); % Velocità costante a 3 m/s
w_cmd = 0.5 * sin(2*pi*0.05*t); % Sterzata sinusoidale

for k = 1:N_steps-1
    % Modello dinamico vero (senza rumore, la nostra ground truth)
    x_true(1, k+1) = x_true(1, k) + x_true(4, k) * cos(x_true(3, k)) * Ts;
    x_true(2, k+1) = x_true(2, k) + x_true(4, k) * sin(x_true(3, k)) * Ts;
    x_true(3, k+1) = x_true(3, k) + x_true(5, k) * Ts;
    x_true(4, k+1) = v_cmd(k);    % Comando di velocità (modello a derivata zero)
    x_true(5, k+1) = w_cmd(k);
end

%% 4. GENERAZIONE DELLE MISURE SENSORIALI
z_history = zeros(6, N_steps);

for k = 1:N_steps
    % Lettura GPS
    z_gps = [x_true(1, k); x_true(2, k)] + sigma_gps_xy * randn(2,1);     % Rumore GPS
    
    % Lettura IMU
    z_imu = [x_true(3, k); x_true(5, k)] + [sigma_imu_th; sigma_imu_w] .* randn(2,1);
    
    % Lettura Encoder (Modello Inverso + Rumore)
    v_true = x_true(4, k);
    w_true = x_true(5, k);
    wR_true = (v_true + (L_veicolo/2)*w_true) / r_veicolo;
    wL_true = (v_true - (L_veicolo/2)*w_true) / r_veicolo;
    z_enc = [wR_true; wL_true] + sigma_enc_wRL * randn(2,1);
    
    % Stack misure
    z_history(:, k) = [z_gps; z_imu; z_enc];
end

%% 5. INIZIALIZZAZIONE EKF
x_est = zeros(5, N_steps);
% Inizializziamo con piccolo errore
x_est(:,1) = x_true(:,1) + [1; -1; 0.1; 0; 0]; 
P = diag([5, 5, 0.5, 1, 1]); % Covarianza iniziale

%% 6. LOOP EKF
for k = 1:N_steps-1
    % --- 6.1 PREDIZIONE ---
    v_est = x_est(4, k);
    w_est = x_est(5, k);
    th_est = x_est(3, k);
    
    % Stato predetto
    x_pred = x_est(:, k);
    x_pred(1) = x_pred(1) + v_est * cos(th_est) * Ts;
    x_pred(2) = x_pred(2) + v_est * sin(th_est) * Ts;
    x_pred(3) = x_pred(3) + w_est * Ts;
    % v e w assumiamo modello a derivata zero (Random Walk) nel predittore
    
    % Jacobiano A_k
    A_k = eye(5);
    A_k(1, 3) = -v_est * sin(th_est) * Ts;
    A_k(1, 4) = cos(th_est) * Ts;
    A_k(2, 3) = v_est * cos(th_est) * Ts;
    A_k(2, 4) = sin(th_est) * Ts;
    A_k(3, 5) = Ts;
    
    % Aggiornamento Covarianza
    P_pred = A_k * P * A_k' + Q;
    
    % --- 6.2 AGGIORNAMENTO (UPDATE) ---
    % Modello di misura predetto h(x_pred)
    z_pred_gps = [x_pred(1); x_pred(2)];
    z_pred_imu = [x_pred(3); x_pred(5)];
    
    v_p = x_pred(4); w_p = x_pred(5);
    z_pred_enc = [(v_p + (L_veicolo/2)*w_p) / r_veicolo; 
                  (v_p - (L_veicolo/2)*w_p) / r_veicolo];
              
    z_pred = [z_pred_gps; z_pred_imu; z_pred_enc];
    
    % Jacobiano C_k
    C_gps = [1 0 0 0 0; 
             0 1 0 0 0];
    C_imu = [0 0 1 0 0; 
             0 0 0 0 1];
    C_enc = [0 0 0, 1/r_veicolo,  L_veicolo/(2*r_veicolo);
             0 0 0, 1/r_veicolo, -L_veicolo/(2*r_veicolo)];
         
    C_k = [C_gps; C_imu; C_enc];
    
    % Calcolo Guadagno di Kalman K
    S = C_k * P_pred * C_k' + R;
    K = P_pred * C_k' / S;
    
    % Correzione dello stato
    y_innov = z_history(:, k+1) - z_pred;
    % Normalizzazione angolo per evitare salti di 2*pi nel calcolo dell'innovazione
    y_innov(3) = wrapToPi(y_innov(3)); 
    
    x_est(:, k+1) = x_pred + K * y_innov;
    x_est(3, k+1) = wrapToPi(x_est(3, k+1)); % Mantieni theta in [-pi, pi]
    
    % Aggiornamento Covarianza stimata
    P = (eye(5) - K * C_k) * P_pred;
end

%% 7. PLOT RISULTATI
figure('Name','Traiettoria (Ground Truth vs Stima)','Color','w');
hold on; grid on; axis equal;
plot(x_true(1,:), x_true(2,:), 'k--', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(z_history(1,:), z_history(2,:), 'r.', 'MarkerSize', 5, 'DisplayName', 'Misure GPS');
plot(x_est(1,:), x_est(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Stima EKF');
xlabel('X [m]'); ylabel('Y [m]'); title('Fase 1: Localizzazione EKF Veicolo Singolo');
legend('Location','best');

figure('Name','Errori di Stima nel Tempo','Color','w');
subplot(3,1,1);
plot(t, x_true(1,:) - x_est(1,:), 'b'); title('Errore Posizione X'); ylabel('[m]'); grid on;
subplot(3,1,2);
plot(t, x_true(2,:) - x_est(2,:), 'r'); title('Errore Posizione Y'); ylabel('[m]'); grid on;
subplot(3,1,3);
err_th = wrapToPi(x_true(3,:) - x_est(3,:));
plot(t, rad2deg(err_th), 'k'); title('Errore Heading \theta'); ylabel('[deg]'); xlabel('Tempo [s]'); grid on;