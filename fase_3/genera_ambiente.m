% =========================================================================
% FASE 3 (Sotto-step 1 & 2): Generazione Mappa e Ottimizzazione UWB (Aree Cieche)
% =========================================================================
clear; clc; close all;

%% 1. PARAMETRI DELLA MAPPA E DELLE ZONE
W_MAP = 500; % [m] Larghezza mappa (X)
H_MAP = 500; % [m] Altezza mappa (Y)

% Parametri GPS-denied zones
% Le zone d'ombra reali (fondovalle, pareti rocciose, fasce boschive) non hanno
% tutte la stessa estensione. Il raggio viene estratto uniformemente in
% [r_area_min, r_area_max], con media 65 m: lo stesso valore che prima era
% imposto identico a tutte le zone.
n_area     = 8;      % Numero di zone buie
r_area_min = 45;     % [m] Raggio minimo della zona d'ombra
r_area_max = 85;     % [m] Raggio massimo

% Parametri Percorso
tipo_percorso = 'sinusoide'; 
num_punti_path = 500; 

% -------------------------------------------------------------------------
% PARAMETRI UWB — tarati su hardware realmente in commercio
%
% Riferimenti di mercato compatibili con questo caso d'uso (outdoor, veicolare,
% all-weather, ranging Two-Way Time-of-Flight):
%
%   Qorvo (ex Decawave) DW1000 / modulo DWM1001C
%     - portata dichiarata fino a 290 m @ 110 kbps, 10% PER, LOS outdoor
%     - fascia economica, adatta ad ancore a basso costo installabili in numero
%
%   Humatics (ex Time Domain) PulsON P440
%     - 3.1-4.8 GHz, TW-TOF, risoluzione ~2 cm fino a 600 m e oltre
%     - progettato per impiego outdoor all-weather e veicolare
%     - fascia professionale, adatto alle ancore fisse su palo
%
% SCELTA: r_ancora = 150 m. E' la portata nominale del DW1000 (290 m) derata di
% circa un fattore 2 per tenere conto di attenuazione da precipitazione nevosa,
% ostruzioni parziali del terreno (NLOS) e margine sul Packet Error Rate.
% Con hardware di classe P440 si potrebbe arrivare oltre i 300 m, ma 150 m
% mantiene il problema di posizionamento non banale e il risultato conservativo.
%
% ALTEZZA D'ANTENNA: la letteratura sperimentale mostra che l'errore di ranging
% cresce sensibilmente quando le antenne sono vicine al suolo. Si assume quindi
% che le ancore siano montate su palo (3-4 m), coerentemente con l'ipotesi di
% vista ottica su neve aperta.
% -------------------------------------------------------------------------
n_ancore = 5;      % Numero di ancore installabili
r_ancora = 150;    % [m] Raggio di visibilita' dell'ancora UWB

%% 2. GENERAZIONE DEL PERCORSO NOMINALE
y_path = linspace(0, H_MAP, num_punti_path);
switch tipo_percorso
    case 'sinusoide'
        ampiezza = 100;
        frequenza = 2 * pi / H_MAP; 
        x_path = W_MAP/2 + ampiezza * sin(frequenza * y_path);
    case 'diagonale'
        x_path = linspace(0, W_MAP, num_punti_path);
    case 'arco'
        raggio_arco = 200;
        theta = linspace(pi, pi/2, num_punti_path);
        x_path = W_MAP - raggio_arco + raggio_arco * cos(theta);
        y_path = raggio_arco * sin(theta);
end
path_points = [x_path', y_path'];

%% 3. GENERAZIONE ZONE GPS-DENIED
rng(42); % Manteniamo il seed fisso per fare in modo che la mappa sia riproducibile a ogni avvio. (Puoi rimuoverlo o cambiarlo se vuoi mappe sempre diverse)

gps_denied_zones = struct('xc', {}, 'yc', {}, 'raggio', {});
for i = 1:n_area
    % Raggio estratto per ogni zona: zone d'ombra di estensione disomogenea
    r_i = r_area_min + rand() * (r_area_max - r_area_min);

    % Coordinate del centro. Il margine pari a r_i evita che la zona venga
    % generata a cavallo del bordo mappa ed "esca" dall'area simulata.
    gps_denied_zones(i).xc     = r_i + rand() * (W_MAP - 2*r_i);
    gps_denied_zones(i).yc     = r_i + rand() * (H_MAP - 2*r_i);
    gps_denied_zones(i).raggio = r_i;
end

%% 4. ESTRAZIONE PUNTI CRITICI (Solo il percorso dentro le zone buie)
punti_critici = [];
for k = 1:num_punti_path
    p = path_points(k, :);
    in_denied = false;
    for i = 1:n_area
        dist = norm(p - [gps_denied_zones(i).xc, gps_denied_zones(i).yc]);
        if dist <= gps_denied_zones(i).raggio
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
    cost_func = @(p_uwb_vec) eval_mean_gdop(p_uwb_vec, punti_critici, r_ancora, W_MAP, H_MAP);
    
    % Initial guess intelligente: distribuiamo le ancore lungo i segmenti di percorso cieco
    p_uwb_init = zeros(1, 2*n_ancore);
    step_idx = max(1, floor(size(punti_critici, 1) / n_ancore));
    
    for idx = 1:n_ancore
        p_idx = min((idx-1)*step_idx + 1, size(punti_critici, 1));
        % Posizioniamo l'ancora iniziale vicino al percorso, ma con un offset
        % laterale per evitare che parta esattamente collineare al percorso:
        % ancore collineari danno una geometria degenere e fanno esplodere la
        % GDOP al primo passo. L'offset e' una frazione del raggio di
        % visibilita', cosi' resta sensato se r_ancora cambia.
        offset_dir = (-1)^idx;              % le alterniamo a destra e a sinistra
        offset_mag = 0.3 * r_ancora;
        p_uwb_init(2*idx-1) = punti_critici(p_idx, 1) + offset_dir * offset_mag;
        p_uwb_init(2*idx)   = punti_critici(p_idx, 2) + offset_dir * offset_mag;
    end
    
    % Ottimizzazione.
    % NOTA sul metodo: fminsearch implementa Nelder-Mead, che e' derivative-free
    % e non vincolato. Con n_ancore = 5 lo spazio di ricerca ha 10 dimensioni,
    % al limite superiore di affidabilita' del metodo: il risultato e' un minimo
    % LOCALE dipendente dall'initial guess, non l'ottimo globale. E' accettabile
    % perche' l'initial guess e' informato (ancore distribuite lungo il percorso
    % cieco), ma va dichiarato. Alternative piu' solide: multi-start, oppure
    % fmincon con vincoli di scatola espliciti se disponibile la Optimization
    % Toolbox.
    options = optimset('Display','iter', 'MaxFunEvals', 8000, 'MaxIter', 4000);
    p_uwb_ottimo = fminsearch(cost_func, p_uwb_init, options);
    
    uwb_opt = reshape(p_uwb_ottimo, 2, n_ancore)';
end

%% 6. PLOT DELL'AMBIENTE
figure('Name', 'Mappa Ambiente e Ottimizzazione UWB (Aree Cieche)', 'Color', 'w');
hold on; grid on; axis equal;
axis([0 W_MAP 0 H_MAP]);

% Disegna Zone GPS-Denied (Cerchi rossi semi-trasparenti)
for i = 1:n_area
    th = linspace(0, 2*pi, 100);
    x_c = gps_denied_zones(i).xc + gps_denied_zones(i).raggio * cos(th);
    y_c = gps_denied_zones(i).yc + gps_denied_zones(i).raggio * sin(th);
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
% Il file viene sempre scritto nella RADICE del progetto, indipendentemente
% dalla cartella corrente: eseguendo lo script dall'IDE la working directory
% diventa fase_3/, e senza questo accorgimento si creerebbe una seconda copia
% dell'ambiente che puo' divergere silenziosamente da quella usata da main3.
root_progetto = fileparts(fileparts(mfilename('fullpath')));
file_out = fullfile(root_progetto, 'ambiente_fase3.mat');
save(file_out, 'W_MAP', 'H_MAP', 'gps_denied_zones', 'tipo_percorso', ...
     'path_points', 'uwb_opt', 'r_ancora');
fprintf('Ambiente generato e salvato in "%s".\n', file_out);

%% ========================================================================
% FUNZIONI LOCALI
% =========================================================================
function mean_gdop = eval_mean_gdop(uwb_vec, punti, r_ancora, W_MAP, H_MAP)
    anchors = reshape(uwb_vec, 2, [])';
    N_anchors = size(anchors, 1);
    N_punti = size(punti, 1);
    gdop_sum = 0;
    
    for k = 1:N_punti
        p = punti(k, :);
        C_geom = [];
        
        for i = 1:N_anchors
            dist = norm(p - anchors(i, :));
            % L'ancora è visibile solo se entro il suo raggio
            if dist <= r_ancora
                dist = max(dist, 0.1); 
                C_geom = [C_geom; (p(1) - anchors(i, 1))/dist, (p(2) - anchors(i, 2))/dist];
            end
        end
        
        % Penalità progressiva: se non abbiamo almeno 2 ancore, impossibile calcolare posa 2D
        visibili = size(C_geom, 1);
        if visibili < 2
            gdop_sum = gdop_sum + 1e5 + (2 - visibili) * 50000;
        else
            G = C_geom' * C_geom;
            if rcond(G) < 1e-8
                gdop_sum = gdop_sum + 2000; % Matrice quasi singolare
            else
                gdop_sum = gdop_sum + sqrt(trace(inv(G)));
            end
        end
    end
    mean_gdop = gdop_sum / N_punti;

    % VINCOLO DI INSTALLABILITA': le ancore devono cadere dentro i confini
    % dell'area di lavoro. fminsearch e' un ottimizzatore NON vincolato, quindi
    % il vincolo va imposto come penalita' additiva sulla funzione di costo,
    % altrimenti l'ottimizzatore e' libero di collocare un'ancora fuori mappa
    % (soluzione matematicamente valida ma fisicamente non realizzabile).
    viol = sum(max(0, -anchors(:,1)) + max(0, anchors(:,1) - W_MAP)) + ...
           sum(max(0, -anchors(:,2)) + max(0, anchors(:,2) - H_MAP));
    mean_gdop = mean_gdop + 10 * viol;
end