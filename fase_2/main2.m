% =========================================================================
% FASE 2: Flotta N=3, Consenso e GPS Differenziato
% =========================================================================
clear; clc; close all;

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

lin_traj = false; 

%% 1. PARAMETRI DI SISTEMA
% -------------------------------------------------------------------------
% TARATURA SU MEZZO BATTIPISTA REALE (classe PistenBully 600 / Prinoth Bison)
% Ingombro ~5 m di larghezza sui cingoli, ~9 m con fresa e lama.
% -------------------------------------------------------------------------
param.r = 0.5;         % [m] raggio ruota motrice del cingolo
param.L = 3.5;         % [m] carreggiata: interasse fra i due cingoli
param.b = 2.0;         % [m] distanza del punto di controllo P per la feedback
                       %     linearization. Il differential drive non puo'
                       %     traslare lateralmente: si controlla un punto
                       %     virtuale posto in avanti rispetto al baricentro,
                       %     che invece puo' muoversi in ogni direzione.
                       %     b ~ semilunghezza del mezzo (punto sul frontale).
                       %     NB: l'inversione amplifica la componente laterale
                       %     del comando di un fattore 1/b, quindi b piccolo
                       %     su un mezzo lento e' fisicamente insensato.

v_max = 5.0;           % [m/s] ~18 km/h, velocita' massima di trasferimento
w_max = 0.6;           % [rad/s] ~34 deg/s: un mezzo cingolato di 5 m non ruota
                       %         su se stesso a 2 rad/s (raggio di sterzata
                       %         minimo v/w ~ 4 m alla velocita' di lavoro)

f_s = 10;              % [Hz] frequenza di campionamento
Ts = 1/f_s;            % [s]
t_end = 200;           % [s] esteso: con formazione di decine di metri e mezzi
                       %     lenti il transitorio di consenso dura ~1 minuto
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
    V_ref = [1.0; 2.3]; % [m/s] Velocità globale di avanzamento della flotta
else
    % Parametri Traiettoria Sinusoidale
    v_y_ref   = 2.5;    % [m/s] ~9 km/h, velocita' di lavoro tipica in battitura
    A_sin     = 20.0;   % [m] Ampiezza dell'oscillazione laterale
    omega_sin = 0.05;   % [rad/s] Frequenza: v_x di picco = A*omega = 1.0 m/s,
                        %         compatibile con w_max su una formazione larga
end

% pos_des indica dove deve stare ogni veicolo rispetto al "baricentro virtuale"
% del gruppo. V1 (Master) in testa, V2 a sinistra e V3 a destra, arretrati.
%
% SCALA DELLA FORMAZIONE — I mezzi battipista lavorano su fronti ampi, con
% distanze reciproche dell'ordine delle decine di metri, non dei metri.
% Oltre al realismo questo e' un requisito FUNZIONALE del progetto: con una
% formazione di pochi metri tutti i veicoli si trovano sempre nella stessa
% condizione di copertura GPS (le zone d'ombra sono ampie decine di metri),
% e lo scenario "un veicolo perde il GPS, il vicino che ce l'ha lo ancora"
% non puo' mai verificarsi. La localizzazione collaborativa diventa
% dimostrabile solo se la formazione e' comparabile alla scala dell'ambiente.
%
% Distanze reciproche risultanti: d12 = d13 = 36.1 m, d23 = 40.0 m.
pos_des = [  0.0,  20.0;  % V1 (X, Y) Master, in testa
           -20.0, -10.0;  % V2 (X, Y) ala sinistra
            20.0, -10.0]; % V3 (X, Y) ala destra

% Calcolo matrice delle distanze relative desiderate Delta_ij --> necessaria perchè i veicoli devono mantenere una certa distanza reciproca per formare la "V", non semplicemente una distanza di raggio r (altrimenti potrebbero essere in ogni posizione nella circonferenza di raggio r).)
Delta = zeros(2, N_veh, N_veh);
for i = 1:N_veh
    for j = 1:N_veh
        Delta(:, i, j) = pos_des(i,:)' - pos_des(j,:)';
    end
end

% K_cons e k_rep sono la rigidita' delle "molle" invisibili che collegano i
% veicoli: il Consenso li tira verso la posizione desiderata, la Repulsione li
% allontana se la distanza scende sotto il raggio di sicurezza d_safe.
%
% GUADAGNO DI CONSENSO — La dinamica dell'errore di formazione e' e_dot = -K*L*e,
% con L Laplaciano del grafo. Per il grafo completo K3 (rete full-mesh) gli
% autovalori non nulli valgono 3, quindi la costante di tempo di convergenza e'
% tau = 1/(3*K_cons). Con K_cons = 0.15 si ottiene tau ~ 2.2 s.
% Il valore precedente (1.0) era tarato su una formazione di 2 m: applicato a
% errori di decine di metri comanderebbe velocita' di ~100 m/s, saturando gli
% attuatori per tutto il transitorio e invalidando la feedback linearization.
K_cons = 0.15;

% RAGGIO DI SICUREZZA — Deve rappresentare l'ingombro fisico reale: due mezzi
% da ~5 m (9 m con fresa e lama) non devono avvicinarsi oltre questa soglia.
% Resta ampiamente sotto la distanza nominale di formazione (36 m), quindi la
% repulsione non interferisce con il consenso a regime.
d_safe = 15.0;      % [m]

% GUADAGNO REPULSIVO — Il potenziale FIRAS ha gradiente che scala come 1/d^3:
% il guadagno NON e' trasferibile fra scale diverse e va riderivato ogni volta
% che cambia d_safe. Lo si specifica quindi tramite un requisito di progetto
% ("a meta' del raggio di sicurezza la repulsione vale v_rep_ref") e lo si
% ricava per inversione, cosi' resta coerente in automatico.
v_rep_ref = 3.0;    % [m/s] intensita' repulsiva desiderata a d = d_safe/2
d_ref  = d_safe / 2;
k_rep  = v_rep_ref / ((1/d_ref - 1/d_safe) * (1/d_ref^2));

%% 4. INIZIALIZZAZIONE STRUTTURA FLOTTA
% Inizializziamo i veicoli con posizioni casuali per vedere il transitorio --> questa posizione iniziale non la conosciamo nella realtà, in simultazione è solo per testare la capacità del sistema di convergere alla formazione desiderata partendo da una configurazione disordinata.
% Scala coerente con la nuova formazione (decine di metri): scostamenti
% iniziali dell'ordine di 20-30 m dalla configurazione desiderata.
x0_true = [ -30, -20, 0, 0, 0;
             25, -35, 0, 0, 0;
             -5, -55, 0, 0, 0 ]';       % (X, Y, theta, V, W) per ogni veicolo

for i = 1:N_veh
    fleet(i).x_true = zeros(5, N_steps);        % 5 stati: X, Y, theta, V, W
    fleet(i).x_est  = zeros(5, N_steps);
    fleet(i).Sigma      = diag([5, 5, 0.5, 1, 1]);      % Covarianza iniziale --> grande incertezza sulla posizione iniziale (+- 5 m), un po' meno su theta (+- 0.5 rad = 28 gradi), e ancora meno su V e W (+- 1 m/s) siccome quando accendo la macchina so di essere fermo.
    fleet(i).z_hist = zeros(6, N_steps);        % 6 misure: GPS(2), IMU(2), Enc(2)
    
    fleet(i).x_true(:,1) = x0_true(:,i);
    % L'EKF parte con una stima leggermente sbagliata --> questa stima iniziale non è perfetta. sto mentendo al EKF per vedere se riesce a correggersi e convergere alla stima corretta. Se partisse già con la stima perfetta, non vedremmo il processo di correzione e convergenza.
    fleet(i).x_est(:,1)  = x0_true(:,i) + [1; -1; 0.2; 0; 0];    % questa è la stima iniziale:+1 m in X, -1 m in Y, +0.2 rad in theta (circa 11 gradi), stessa stima per V e W

    fleet(i).F_cons_hist = zeros(2, N_steps);
    fleet(i).F_rep_hist  = zeros(2, N_steps);
    fleet(i).u_hist      = zeros(2, N_steps);   % comandi [v; w] applicati
    
    % Assegna R_gps (Covarianza del rumore di MISURA GPS) specifica in base al ruolo
    if i == 1
        fleet(i).R_gps = diag([sigma_gps_master^2, sigma_gps_master^2]);
    else
        fleet(i).R_gps = diag([sigma_gps_slave^2, sigma_gps_slave^2]);
    end
end

%% 5. MAIN SIMULATION LOOP
% -------------------------------------------------------------------------
% CONVENZIONE TEMPORALE  (indice k  <->  istante t_k = (k-1)*Ts)
%   x_true(:,k) : stato REALE all'istante t_k
%   x_est(:,k)  : stima A POSTERIORI a t_k (usa tutte le misure fino a t_k INCLUSO)
%   z_hist(:,k) : misura ACQUISITA a t_k, quindi funzione di x_true(:,k)
%   u_hist(:,k) : comando APPLICATO in [t_k, t_{k+1}), calcolato SOLO da x_est(:,k)
%
% ORDINE DI ESECUZIONE (causale: nessuno step usa informazione futura)
%   (1) BROADCAST : ogni veicolo pubblica la propria stima a t_k
%   (2) CONTROLLO : u(:,k) = g( x_est(:,k), stime dei vicini a t_k )
%   (3) IMPIANTO  : x_true(:,k+1) = f( x_true(:,k), u(:,k) )
%   (4) SENSORI   : z(:,k+1) = h( x_true(:,k+1) ) + rumore
%   (5) STIMA     : EKF predict da x_est(:,k), update con z(:,k+1) -> x_est(:,k+1)
%
% I blocchi (2)-(3) e (4)-(5) sono DUE PASSATE SEPARATE su tutti i veicoli:
% la ground truth di TUTTI i veicoli deve esistere a t_{k+1} prima che uno
% qualsiasi generi le proprie misure. Qui non e' strettamente necessario, ma
% lo diventa in Fase 3 dove le misure dipendono anche dalla posizione reale
% dei vicini: la struttura e' la stessa nelle due fasi per coerenza.
% -------------------------------------------------------------------------

% Misura all'istante iniziale: registrata per completezza dello storico.
% Non viene usata dal filtro, perche' x_est(:,1) e' assunto come prior a t_1.
for i = 1:N_veh
    fleet(i).z_hist(:,1) = genera_misure(fleet(i).x_true(:,1), fleet(i).R_gps, R_imu, R_enc, param);
end

for k = 1:N_steps-1
    % Riferimento di velocita' della flotta, valutato a t_k
    if lin_traj == false
        v_x_ref = A_sin * omega_sin * cos(omega_sin * t(k));
        V_ref = [v_x_ref; v_y_ref];
    end

    % --- (1) BROADCAST ---------------------------------------------------
    % Canale ideale: ogni veicolo riceve istantaneamente le stime dei vicini
    % riferite a t_k. Il punto di controllo p_i e' calcolato dalla STIMA e non
    % dalla ground truth: e' l'unica informazione realmente disponibile a bordo.
    p_est = zeros(2, N_veh);
    for i = 1:N_veh
        xe = fleet(i).x_est(:, k);
        p_est(1, i) = xe(1) + param.b * cos(xe(3));
        p_est(2, i) = xe(2) + param.b * sin(xe(3));
    end

    % --- (2) CONTROLLO + (3) IMPIANTO ------------------------------------
    for i = 1:N_veh
        F_cons = [0; 0];    % N.B. "Forze" omogenee a velocita': sono in [m/s]
        F_rep  = [0; 0];

        for j = 1:N_veh
            if i ~= j       % il consenso e' solo con gli altri, non con se stessi
                % 1. Consenso: ATTRAZIONE verso la formazione (LINEARE)
                % Errore fra la distanza relativa stimata e quella desiderata.
                err_ij = (p_est(:, i) - p_est(:, j)) - Delta(:, i, j);
                F_cons = F_cons - K_cons * err_ij;      % legge di Hooke: F = -K*x

                % 2. Evitamento collisioni: REPULSIONE (NON LINEARE, attiva a soglia)
                dist = norm(p_est(:, i) - p_est(:, j));
                if dist < d_safe && dist > 0.1
                    grad_d  = (p_est(:, i) - p_est(:, j)) / dist;   % versore da j verso i
                    rep_mag = k_rep * (1/dist - 1/d_safe) * (1/dist^2);
                    F_rep   = F_rep + rep_mag * grad_d;
                end
            end
        end

        fleet(i).F_cons_hist(:, k) = F_cons;
        fleet(i).F_rep_hist(:, k)  = F_rep;

        % Velocita' desiderata del punto di controllo p_i: somma di tre intenti
        % ("avanza con la flotta", "resta in formazione", "non collidere").
        p_dot_cmd = V_ref + F_cons + F_rep;

        % Inversione (feedback linearization) valutata sulla STIMA a t_k
        theta_est = fleet(i).x_est(3, k);
        T_fl_inv = [ cos(theta_est),           sin(theta_est);
                 -sin(theta_est)/param.b,   cos(theta_est)/param.b ];

        vw_cmd = T_fl_inv * p_dot_cmd;

        % Saturazione dei comandi (realismo attuatori)
        v_cmd = max(min(vw_cmd(1), v_max), -v_max);
        w_cmd = max(min(vw_cmd(2), w_max), -w_max);
        fleet(i).u_hist(:, k) = [v_cmd; w_cmd];

        % (3) IMPIANTO: la ground truth avanza con la velocita' REALE gia'
        % presente nello stato a t_k; il comando appena calcolato diventa
        % effettivo a t_{k+1} (attuatore ZOH con ritardo di un passo).
        %
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
        % Le misure sono funzione dello stato reale a t_{k+1}: lo STESSO istante
        % a cui si riferisce la predizione che andranno a correggere.
        z = genera_misure(fleet(i).x_true(:, k+1), fleet(i).R_gps, R_imu, R_enc, param);
        fleet(i).z_hist(:, k+1) = z;

        % Sensori indipendenti -> R totale diagonale a blocchi (6x6)
        R_k = blkdiag(fleet(i).R_gps, R_imu, R_enc);
        [fleet(i).x_est(:, k+1), fleet(i).Sigma] = ...
            ekf_step(fleet(i).x_est(:, k), fleet(i).Sigma, z, Ts, Q, R_k, param);
    end
end

%% 6. PLOT RISULTATI E ANIMAZIONE REAL-TIME
figure('Name','Animazione Flotta e Formazione','Color','w'); 
hold on; grid on; axis equal;
title('Animazione della Traiettoria (Reale vs Stimata)'); 
xlabel('X [m]'); ylabel('Y [m]');

colors = ['b', 'r', 'g'];
h_true_trail = zeros(1, N_veh);
h_est_trail = zeros(1, N_veh);
h_true_pos = zeros(1, N_veh);
h_est_pos = zeros(1, N_veh);

for i = 1:N_veh
    % Linee per le traiettorie
    h_true_trail(i) = plot(fleet(i).x_true(1,1), fleet(i).x_true(2,1), [colors(i) '--'], 'LineWidth', 1);
    h_est_trail(i) = plot(fleet(i).x_est(1,1), fleet(i).x_est(2,1), [colors(i) '-'], 'LineWidth', 1.5);
    
    % Pallini per la posizione attuale
    h_true_pos(i) = plot(fleet(i).x_true(1,1), fleet(i).x_true(2,1), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);
    h_est_pos(i) = plot(fleet(i).x_est(1,1), fleet(i).x_est(2,1), [colors(i) 'o'], 'MarkerFaceColor', colors(i), 'MarkerSize', 8);
end
legend([h_est_pos(1), h_est_pos(2), h_est_pos(3)], {'Master (V1)', 'Slave (V2)', 'Slave (V3)'}, 'Location', 'best');

% Loop di Animazione
step_animazione = 2; % Salta un frame per velocizzare l'animazione (cambia a 1 per fluidità massima)
for k = 1:step_animazione:N_steps
    for i = 1:N_veh
        % Aggiorna le linee (storia)
        set(h_true_trail(i), 'XData', fleet(i).x_true(1, 1:k), 'YData', fleet(i).x_true(2, 1:k));
        set(h_est_trail(i), 'XData', fleet(i).x_est(1, 1:k), 'YData', fleet(i).x_est(2, 1:k));
        
        % Aggiorna i pallini (posizione corrente)
        set(h_true_pos(i), 'XData', fleet(i).x_true(1, k), 'YData', fleet(i).x_true(2, k));
        set(h_est_pos(i), 'XData', fleet(i).x_est(1, k), 'YData', fleet(i).x_est(2, k));
    end
    drawnow; % Forza MATLAB a disegnare il frame istantaneamente
    pause(0.05);
end


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

% --- DASHBOARD SALUTE EKF (Errori X, Y, Theta) ---
figure('Name','Diagnostica EKF: Errore di Stima','Color','w', 'Position', [100, 100, 1000, 600]);
for i = 1:N_veh
    % Calcolo degli errori per tutte e tre le variabili spaziali
    err_x = fleet(i).x_true(1,:) - fleet(i).x_est(1,:);
    err_y = fleet(i).x_true(2,:) - fleet(i).x_est(2,:);
    err_th = wrapToPi(fleet(i).x_true(3,:) - fleet(i).x_est(3,:)); % wrapToPi per evitare salti a 360 gradi
    
    % Colonna per ogni veicolo, riga per ogni variabile
    % Plot Errore X
    subplot(3, N_veh, i);
    ylim([-2.5, 2.5]); 
    plot(t, err_x, colors(i), 'LineWidth', 1); grid on; hold on;
    yline(0, 'k--', 'LineWidth', 1.5); % Linea dello zero
    title(sprintf('V%d: Errore X', i)); 
    if i==1; ylabel('[m]'); end
    
    % Plot Errore Y
    subplot(3, N_veh, i + N_veh);
    ylim([-2.5, 2.5]);
    plot(t, err_y, colors(i), 'LineWidth', 1); grid on; hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    title(sprintf('V%d: Errore Y', i)); 
    if i==1; ylabel('[m]'); end
    
    % Plot Errore Theta
    subplot(3, N_veh, i + 2*N_veh);
    ylim([-0.5, 0.5])
    plot(t, err_th, colors(i), 'LineWidth', 1); grid on; hold on;
    yline(0, 'k--', 'LineWidth', 1.5);
    title(sprintf('V%d: Errore \\theta', i)); 
    xlabel('Tempo [s]'); 
    if i==1; ylabel('[rad]'); end
end


%% --- DASHBOARD FORZE VIRTUALI (Consenso e Repulsione) ---
figure('Name','Analisi delle Forze Virtuali nel Tempo','Color','w', 'Position', [150, 150, 1000, 500]);

for i = 1:N_veh
    % Calcolo della magnitudo (norma) dei vettori forza istante per istante
    mag_F_cons = sqrt(fleet(i).F_cons_hist(1,:).^2 + fleet(i).F_cons_hist(2,:).^2);
    mag_F_rep  = sqrt(fleet(i).F_rep_hist(1,:).^2  + fleet(i).F_rep_hist(2,:).^2);
    
    % 1. Plot Sforzo di Consenso (Attrazione)
    subplot(2, N_veh, i);
    plot(t, mag_F_cons, colors(i), 'LineWidth', 1.5); grid on; hold on;
    title(sprintf('V%d: Sforzo Consenso (|F_{cons}|)', i));
    xlabel('Tempo [s]'); 
    if i==1; ylabel('Magnitudo [m/s]'); end
    
    % 2. Plot Forza Repulsiva (Evitamento collisioni)
    subplot(2, N_veh, i + N_veh);
    plot(t, mag_F_rep, 'k', 'LineWidth', 1.5); grid on; hold on;
    title(sprintf('V%d: Forza Repulsiva (|F_{rep}|)', i));
    xlabel('Tempo [s]'); 
    if i==1; ylabel('Magnitudo [m/s]'); end
    
    % Allineamento visivo degli assi Y per facilitare il confronto
    subplot(2, N_veh, i);       ylim([0, max(0.1, max(mag_F_cons)*1.2)]);
    subplot(2, N_veh, i+N_veh); ylim([0, max(0.1, max(mag_F_rep)*1.2)]);
end

%% ========================================================================
% FUNZIONI LOCALI
% =========================================================================

function z = genera_misure(x_true, R_gps, R_imu, R_enc, param)
    % Simula il vettore di misure z = [GPS(2); IMU(2); ENC(2)] a partire dallo
    % stato REALE, sporcandolo con rumore gaussiano a media nulla.
    %
    % Fattorizzazione di Cholesky invece di sqrt(R): sqrt() opera elemento per
    % elemento e da' il risultato giusto solo se R e' diagonale. chol(R)' e' la
    % radice matriciale corretta e resta valida se in futuro si introducono
    % correlazioni fra sensori (es. GPS con errori accoppiati su X e Y).
    z_gps = x_true(1:2)   + chol(R_gps)' * randn(2,1);
    z_imu = x_true([3,5]) + chol(R_imu)' * randn(2,1);   % [theta (magnetom.); w (giroscopio)]

    % Encoder: cinematica inversa del differential drive, da (v,w) a (wR,wL)
    wR = (x_true(4) + (param.L/2)*x_true(5)) / param.r;
    wL = (x_true(4) - (param.L/2)*x_true(5)) / param.r;
    z_enc = [wR; wL] + chol(R_enc)' * randn(2,1);

    z = [z_gps; z_imu; z_enc];
end

function [x_new, Sigma_new] = ekf_step(x_old, Sigma_old, z, Ts, Q, R, param)
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
    
    Sigma_bar = A_k * Sigma_old * A_k' + Q;
    
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
    
    S = C_k * Sigma_bar * C_k' + R;
    K = Sigma_bar * C_k' / S;
    
    y_innov = z - z_pred;
    y_innov(3) = wrapToPi(y_innov(3));
    
    x_new = x_pred + K * y_innov;
    x_new(3) = wrapToPi(x_new(3));
    
    Sigma_new = (eye(5) - K * C_k) * Sigma_bar;
end
