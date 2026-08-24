% =========================================================================
% FASE 1: Veicolo Singolo Ideale (EKF Sincrono a 10 Hz)
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

% Generiamo comandi di V e W fittizi per fare un percorso sinusoidale
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
Sigma = diag([5, 5, 0.5, 1, 1]); % Covarianza iniziale

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
    % Q ricalcolata a ogni passo: dipende da theta (e da v se k_terreno > 0)
    Q_k = calcola_Q_cwna(th_est, v_est, Ts, par_Q);
    Sigma_bar = A_k * Sigma * A_k' + Q_k;
    
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
    S = C_k * Sigma_bar * C_k' + R;
    K = Sigma_bar * C_k' / S;
    
    % Correzione dello stato
    y_innov = z_history(:, k+1) - z_pred;
    % Normalizzazione angolo per evitare salti di 2*pi nel calcolo dell'innovazione
    y_innov(3) = wrapToPi(y_innov(3)); 
    
    x_est(:, k+1) = x_pred + K * y_innov;
    x_est(3, k+1) = wrapToPi(x_est(3, k+1)); % Mantieni theta in [-pi, pi]
    
    % Aggiornamento Covarianza stimata
    Sigma = (eye(5) - K * C_k) * Sigma_bar;
end

%% 7. PLOT E SALVATAGGIO RISULTATI

cartella_output = fullfile('fase_1', 'risultati'); % Cartella 'risultati' nella directory corrente
if ~exist(cartella_output, 'dir')
    mkdir(cartella_output); % Crea la cartella se non esiste
end

fig1 = figure('Name','Traiettoria (Ground Truth vs Stima)','Color','w');
hold on; grid on; axis equal;
plot(x_true(1,:), x_true(2,:), 'k--', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(z_history(1,:), z_history(2,:), 'r.', 'MarkerSize', 5, 'DisplayName', 'Misure GPS');
plot(x_est(1,:), x_est(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Stima EKF');
xlabel('X [m]'); ylabel('Y [m]'); title('Fase 1: Localizzazione EKF Veicolo Singolo');
legend('Location','best');
percorso_fig1 = fullfile(cartella_output, 'fase1_traiettoria.png');
exportgraphics(fig1, percorso_fig1, 'Resolution', 300);

fig2 = figure('Name','Errori di Stima nel Tempo','Color','w');
subplot(3,1,1);
plot(t, x_true(1,:) - x_est(1,:), 'b'); title('Errore Posizione X'); ylabel('[m]'); grid on;
subplot(3,1,2);
plot(t, x_true(2,:) - x_est(2,:), 'r'); title('Errore Posizione Y'); ylabel('[m]'); grid on;
subplot(3,1,3);
err_th = wrapToPi(x_true(3,:) - x_est(3,:));
plot(t, rad2deg(err_th), 'k'); title('Errore Heading \theta'); ylabel('[deg]'); xlabel('Tempo [s]'); grid on;
percorso_fig2 = fullfile(cartella_output, 'fase1_errori.png');
exportgraphics(fig2, percorso_fig2, 'Resolution', 300);
