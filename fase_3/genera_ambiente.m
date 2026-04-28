% =========================================================================
% FASE 3 (Sotto-step 1 & 2): Generazione Mappa e Ottimizzazione UWB (Aree Cieche)
% =========================================================================
clear; clc; close all;

%% 1. PARAMETRI DELLA MAPPA E DELLE ZONE
W = 500; % [m] Larghezza mappa (X)
H = 500; % [m] Altezza mappa (Y)

% Parametri GPS-denied zones
n_area = 9;      % Numero di zone buie
r_area = 50;     % [m] Raggio delle zone

% Parametri Percorso
tipo_percorso = 'sinusoide'; 
num_punti_path = 500; 

% Parametri UWB
n_ancore = 2;      % Numero di ancore installabili (aumentato per coprire le aree)
r_ancora = 100;    % [m] Raggio di visibilità dell'ancora UWB

%% 2. GENERAZIONE DEL PERCORSO NOMINALE
y_path = linspace(0, H, num_punti_path);
switch tipo_percorso
    case 'sinusoide'
        ampiezza = 100;
        frequenza = 2 * pi / H; 
        x_path = W/2 + ampiezza * sin(frequenza * y_path);
    case 'diagonale'
        x_path = linspace(0, W, num_punti_path);
    case 'arco'
        R_arco = 200;
        theta = linspace(pi, pi/2, num_punti_path);
        x_path = W - R_arco + R_arco * cos(theta);
        y_path = R_arco * sin(theta);
end
path_points = [x_path', y_path'];

%% 3. GENERAZIONE ZONE GPS-DENIED
rng(42); % Manteniamo il seed fisso per fare in modo che la mappa sia riproducibile a ogni avvio. (Puoi rimuoverlo o cambiarlo se vuoi mappe sempre diverse)

gps_denied_zones = struct('xc', {}, 'yc', {}, 'R', {});
for i = 1:n_area
    % Generazione casuale delle coordinate X e Y all'interno della mappa
    % Usiamo un margine pari a r_area per evitare che il centro della zona
    % venga generato esattamente sul bordo e "esca" dalla mappa.
    gps_denied_zones(i).xc = r_area + rand() * (W - 2*r_area); 
    gps_denied_zones(i).yc = r_area + rand() * (H - 2*r_area);
    gps_denied_zones(i).R  = r_area;
end

%% 4. ESTRAZIONE PUNTI CRITICI (Solo il percorso dentro le zone buie)
punti_critici = [];
for k = 1:num_punti_path
    p = path_points(k, :);
    in_denied = false;
    for i = 1:n_area
        dist = norm(p - [gps_denied_zones(i).xc, gps_denied_zones(i).yc]);
        if dist <= gps_denied_zones(i).R
            in_denied = true;
            break; % Se è in una zona, inutile controllare le altre
        end
    end
    if in_denied
        punti_critici = [punti_critici; p];
    end
end

%% 5. OTTIMIZZAZIONE ANCORE UWB (Minimizzazione GDOP sul percorso cieco)
disp('Avvio ottimizzazione posizioni UWB lungo il percorso cieco...');

if isempty(punti_critici)
    warning('Il percorso non attraversa le zone GPS-denied. Nessuna ottimizzazione necessaria.');
    uwb_opt = [];
else
    cost_func = @(p_uwb_vec) eval_mean_gdop(p_uwb_vec, punti_critici, r_ancora);
    
    % Initial guess intelligente: distribuiamo le ancore lungo i segmenti di percorso cieco
    p_uwb_init = zeros(1, 2*n_ancore);
    step_idx = max(1, floor(size(punti_critici, 1) / n_ancore));
    
    for idx = 1:n_ancore
        p_idx = min((idx-1)*step_idx + 1, size(punti_critici, 1));
        % Posizioniamo l'ancora iniziale vicino al percorso, ma con un leggero offset 
        % (15m) per evitare che partano esattamente collineari al percorso,
        % cosa che farebbe esplodere la GDOP al primo step.
        offset_dir = (-1)^idx; % Le alterniamo a destra e a sinistra
        p_uwb_init(2*idx-1) = punti_critici(p_idx, 1) + offset_dir * 15;
        p_uwb_init(2*idx)   = punti_critici(p_idx, 2) + offset_dir * 15;
    end
    
    % Ottimizzazione
    options = optimset('Display','iter', 'MaxFunEvals', 4000, 'MaxIter', 2000);
    p_uwb_ottimo = fminsearch(cost_func, p_uwb_init, options);
    
    uwb_opt = reshape(p_uwb_ottimo, 2, n_ancore)';
end

%% 6. PLOT DELL'AMBIENTE
figure('Name', 'Mappa Ambiente e Ottimizzazione UWB (Aree Cieche)', 'Color', 'w');
hold on; grid on; axis equal;
axis([0 W 0 H]);

% Disegna Zone GPS-Denied (Cerchi rossi semi-trasparenti)
for i = 1:n_area
    th = linspace(0, 2*pi, 100);
    x_c = gps_denied_zones(i).xc + gps_denied_zones(i).R * cos(th);
    y_c = gps_denied_zones(i).yc + gps_denied_zones(i).R * sin(th);
    patch(x_c, y_c, 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'r', 'LineStyle', '--', 'DisplayName', 'GPS-Denied Zone');
end

% Disegna Griglia Punti Critici
if ~isempty(punti_critici)
    plot(punti_critici(:,1), punti_critici(:,2), 'r.', 'MarkerSize', 4, 'DisplayName', 'Area da Ottimizzare');
end

% Disegna Percorso
plot(x_path, y_path, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Percorso Nominale');

% Disegna Ancore UWB e loro raggio di copertura
if ~isempty(uwb_opt)
    plot(uwb_opt(:,1), uwb_opt(:,2), 'b^', 'MarkerFaceColor', 'b', 'MarkerSize', 10, 'DisplayName', 'Ancore UWB');
    for i = 1:n_ancore
        th = linspace(0, 2*pi, 100);
        x_c = uwb_opt(i,1) + r_ancora * cos(th);
        y_c = uwb_opt(i,2) + r_ancora * sin(th);
        plot(x_c, y_c, 'b-', 'LineWidth', 0.5, 'HandleVisibility', 'off'); 
    end
end

title('Fase 3: Ottimizzazione UWB sulle Aree Cieche');
xlabel('X [m]'); ylabel('Y [m]');
[~, obj_h] = legend('Location', 'best');

%% 7. SALVATAGGIO DATI
save('ambiente_fase3.mat', 'W', 'H', 'gps_denied_zones', 'tipo_percorso', 'path_points', 'uwb_opt', 'r_ancora');
disp('Ambiente generato e salvato in "ambiente_fase3.mat".');

%% ========================================================================
% FUNZIONI LOCALI
% =========================================================================
function mean_gdop = eval_mean_gdop(uwb_vec, punti, r_ancora)
    anchors = reshape(uwb_vec, 2, [])';
    N_anchors = size(anchors, 1);
    N_punti = size(punti, 1);
    gdop_sum = 0;
    
    for k = 1:N_punti
        p = punti(k, :);
        H_mat = [];
        
        for i = 1:N_anchors
            dist = norm(p - anchors(i, :));
            % L'ancora è visibile solo se entro il suo raggio
            if dist <= r_ancora
                dist = max(dist, 0.1); 
                H_mat = [H_mat; (p(1) - anchors(i, 1))/dist, (p(2) - anchors(i, 2))/dist];
            end
        end
        
        % Penalità progressiva: se non abbiamo almeno 2 ancore, impossibile calcolare posa 2D
        visibili = size(H_mat, 1);
        if visibili < 2
            gdop_sum = gdop_sum + 1e5 + (2 - visibili) * 50000;
        else
            G = H_mat' * H_mat;
            if rcond(G) < 1e-8
                gdop_sum = gdop_sum + 2000; % Matrice quasi singolare
            else
                gdop_sum = gdop_sum + sqrt(trace(inv(G)));
            end
        end
    end
    mean_gdop = gdop_sum / N_punti;
end