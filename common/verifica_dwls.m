%VERIFICA_DWLS Validazione dell'algoritmo di stima distribuita D-WLS.
%
% Controlla, su topologie note e con dati sintetici, che consenso_dwls.m
% riproduca le proprieta' enunciate nel Cap. 18:
%   - equivalenza con il WLS centralizzato a consenso convergente
%   - invarianza della somma dell'informazione (assenza di doppio conteggio)
%   - singolarita' del problema locale: nessun nodo puo' risolverlo da solo
%   - decadimento dell'errore governato da rho2 su topologie non complete
%   - fallimento diagnosticabile su grafo sconnesso
%   - pesatura effettiva delle misure secondo la loro qualita' (WLS, non LS)
%   - dimensionamento automatico del numero di cicli q da rho2 e dal diametro
clear; clc;
addpath(fileparts(mfilename('fullpath')));

fprintf('============ VALIDAZIONE D-WLS DISTRIBUITO (Cap. 18) ============\n\n');

% -------------------------------------------------------------------------
% Problema di prova: modello di resistenza specifica al moto su neve
%     z(i) = mu_terr + c_terr * v_i^2 + eps(i)
% cioe' C_i = [1, v_i^2] con parametro incognito x = [mu_terr; c_terr].
% E' lo stesso problema risolto in Fase 3 (vedi fase_3/README3.md).
% -------------------------------------------------------------------------
rng(7);                                  % ripetibilita' del test
x_vero  = [0.09; 0.005];                 % parametro costante non stocastico
v       = [1.8, 2.5, 3.4];               % velocita' dei tre nodi [m/s]
sigma_traz = [0.010, 0.025, 0.025];      % Master meglio strumentato degli Slave
n       = numel(v);
m       = numel(x_vero);

C_i = cell(1,n); R_i = zeros(1,n); z_i = zeros(1,n);
F0  = zeros(m,m,n); a0 = zeros(m,n);
for i = 1:n
    C_i{i} = [1, v(i)^2];
    R_i(i) = sigma_traz(i)^2;
    z_i(i) = C_i{i}*x_vero + sigma_traz(i)*randn();
    F0(:,:,i) = C_i{i}' * (C_i{i} / R_i(i));     % F_i(0) = C_i' R_i^-1 C_i
    a0(:,i)   = C_i{i}' * (z_i(i)  / R_i(i));    % a_i(0) = C_i' R_i^-1 z_i
end

% =========================================================================
fprintf('--- TEST 1: il singolo nodo NON puo'' risolvere il problema\n');
% Con m = 2 parametri e una sola misura scalare, F_i(0) = C_i' R_i^-1 C_i e'
% il prodotto esterno di un vettore per se stesso: ha rango 1 su 2.
for i = 1:n
    fprintf('    nodo %d: rango F_i(0) = %d su %d, cond = %.2e\n', ...
            i, rank(F0(:,:,i)), m, cond(F0(:,:,i)));
end
F_rete = sum(F0,3);
fprintf('    rete  : rango sum F_i  = %d su %d, cond = %.2e\n', ...
        rank(F_rete), m, cond(F_rete));
fprintf('    -> la cooperazione non accelera la stima: la rende POSSIBILE\n\n');

% =========================================================================
fprintf('--- TEST 2: equivalenza con il WLS centralizzato (K3 completo)\n');
G  = costruisci_grafo([0 20; -20 -10; 20 -10]', 120);   % formazione a V
[X, info] = consenso_dwls(F0, a0, G.A, 1, 0);           % q = 1 ciclo imposto

% WLS centralizzato costruito per impilamento, secondo la formula del corso
C_stack = cell2mat(C_i');
R_stack = diag(R_i);
x_wls   = (C_stack' / R_stack * C_stack) \ (C_stack' / R_stack * z_i');

fprintf('    rho2(Q) sul grafo completo = %.4f\n', info.rho2);
fprintf('    x_LS centralizzato = [%.6f, %.6f]\n', x_wls);
for i = 1:n
    fprintf('    nodo %d dopo q=1     = [%.6f, %.6f]   scarto = %.2e\n', ...
            i, X(:,i), norm(X(:,i) - x_wls));
end
fprintf('    -> con rho2 = 0 un solo ciclo basta: %s\n\n', ...
        string(max(vecnorm(X - x_wls, 2, 1)) < 1e-12));

% =========================================================================
fprintf('--- TEST 3: invarianza della somma (niente doppio conteggio)\n');
% La doppia stocasticita' di Q rende sum_i F_i(k) costante lungo il consenso:
% l'informazione viene ridistribuita, mai duplicata.
for q = [1 2 5 20]
    [~, inf_q] = consenso_dwls(F0, a0, G.A, q, 0);
    fprintf('    q = %2d : ||sum F_i(q) - sum F_i(0)||/||sum F_i(0)|| = %.2e\n', ...
            q, inf_q.inv_somma);
end
fprintf('\n');

% =========================================================================
fprintf('--- TEST 4: topologia a catena, il consenso richiede piu'' cicli\n');
% Su grafo non completo rho2 > 0 e l'errore rispetto alla soluzione
% centralizzata decade geometricamente con ragione rho2.
G_cat = costruisci_grafo([0 0; 40 0; 80 0]', 45);   % catena 1-2-3
[~, inf_cat] = consenso_dwls(F0, a0, G_cat.A, 1, 0);
fprintf('    rho2(Q) sulla catena = %.4f\n', inf_cat.rho2);
err_prec = NaN;
for q = 1:8
    [Xq, ~] = consenso_dwls(F0, a0, G_cat.A, q, 0);
    err = max(vecnorm(Xq - x_wls, 2, 1));
    if isnan(err_prec)
        fprintf('    q = %d : errore = %.3e\n', q, err);
    else
        fprintf('    q = %d : errore = %.3e   rapporto = %.4f\n', ...
                q, err, err/err_prec);
    end
    err_prec = err;
end
fprintf('    -> il rapporto tende a rho2 = %.4f\n\n', inf_cat.rho2);

% =========================================================================
fprintf('--- TEST 5: grafo SCONNESSO, il fallimento e'' diagnosticabile\n');
% Nodi 1-2 collegati, nodo 3 isolato. Il consenso converge alla media della
% propria componente, quindi i due gruppi stimano parametri diversi e il nodo
% isolato resta con un problema singolare.
G_scon = costruisci_grafo([0 0; 20 0; 200 0]', 30);
[X_s, inf_s] = consenso_dwls(F0, a0, G_scon.A, 30, 0);
fprintf('    lambda2 = %.4f, rho2 = %.4f  (grafo connesso: %s)\n', ...
        G_scon.lambda2_L, inf_s.rho2, string(G_scon.connesso));
for i = 1:n
    if any(isnan(X_s(:,i)))
        fprintf('    nodo %d: F_i singolare -> nessuna stima ricostruibile\n', i);
    else
        fprintf('    nodo %d: [%.6f, %.6f]  scarto da x_LS = %.3e\n', ...
                i, X_s(:,i), norm(X_s(:,i) - x_wls));
    end
end
fprintf('    -> lambda2 = 0 e rho2 = 1 segnalano il problema PRIMA di usarne\n');
fprintf('       il risultato: sono gli indicatori del Cap. 17 usati come guardia\n\n');

% =========================================================================
fprintf('--- TEST 6: e'' davvero PESATO (WLS e non LS ordinario)\n');
% Il nodo 1 e' strumentato meglio (sigma_traz = 0.010 contro 0.025): la sua misura
% deve contare di piu'. Il confronto e' con i minimi quadrati ORDINARI, che
% trattano le tre misure come equivalenti.
%
% Il confronto va fatto in media e non su una singola realizzazione: il WLS
% minimizza la covarianza dell'errore, non l'errore del singolo esperimento,
% e su una realizzazione isolata puo' benissimo risultare peggiore. Si esegue
% quindi una campagna Monte Carlo.
peso = 1./R_i; peso = peso / sum(peso);
fprintf('    pesi informativi relativi = [%.3f %.3f %.3f]\n', peso);

N_mc = 20000;
err_wls = zeros(N_mc, m); err_ord = zeros(N_mc, m);
for mc = 1:N_mc
    zz = C_stack*x_vero + sqrt(R_i(:)).*randn(n,1);
    Fmc = zeros(m,m,n); amc = zeros(m,n);
    for i = 1:n
        Fmc(:,:,i) = C_i{i}' * (C_i{i} / R_i(i));
        amc(:,i)   = C_i{i}' * (zz(i)  / R_i(i));
    end
    Xmc = consenso_dwls(Fmc, amc, G.A, 1, 0);
    err_wls(mc,:) = (Xmc(:,1) - x_vero)';
    err_ord(mc,:) = (((C_stack'*C_stack) \ (C_stack'*zz)) - x_vero)';
end
rms_wls = sqrt(mean(err_wls.^2, 1));
rms_ord = sqrt(mean(err_ord.^2, 1));
S_teo   = inv(sum(F0,3));   % covarianza teorica del WLS, (sum F_i)^-1

fprintf('    Monte Carlo su %d realizzazioni:\n', N_mc);
fprintf('                        mu_terr        c_terr\n');
fprintf('    RMSE LS ordinario   %.3e     %.3e\n', rms_ord);
fprintf('    RMSE WLS (D-WLS)    %.3e     %.3e\n', rms_wls);
fprintf('    sqrt(diag(P)) teor. %.3e     %.3e   <- corrisponde al WLS\n', ...
        sqrt(diag(S_teo)));
fprintf('    guadagno del WLS    %.2fx         %.2fx\n', rms_ord./rms_wls);
fprintf('    bias del WLS        %.2e     %.2e   (deve essere ~0)\n\n', ...
        mean(err_wls, 1));

% =========================================================================
fprintf('--- TEST 7: accumulo temporale, la stima migliora con le misure\n');
% Ogni nodo somma nel tempo i propri contributi informativi. La covarianza
% dell'errore di stima e' (sum_i F_i)^-1 e decresce come 1/N_campioni.
N_camp = [1 10 50 200 1000];
fprintf('    N_camp   sigma(mu_terr)   sigma(c_terr)   errore ||x-x_vero||\n');
for N = N_camp
    Facc = zeros(m,m,n); aacc = zeros(m,n);
    for i = 1:n
        for s = 1:N
            % Velocita' leggermente variabile: e' cio' che accade in formazione
            % su percorso curvo, ed e' l'unica cosa che condiziona il problema.
            vs = v(i) + 0.3*randn();
            Cs = [1, vs^2];
            zs = Cs*x_vero + sigma_traz(i)*randn();
            Facc(:,:,i) = Facc(:,:,i) + Cs' * (Cs / R_i(i));
            aacc(:,i)   = aacc(:,i)   + Cs' * (zs / R_i(i));
        end
    end
    [XN, ~] = consenso_dwls(Facc, aacc, G.A, 1, 0);
    S = inv(sum(Facc,3));
    fprintf('    %6d   %13.3e   %13.3e   %.3e\n', ...
            N, sqrt(S(1,1)), sqrt(S(2,2)), norm(XN(:,1) - x_vero));
end

% =========================================================================
fprintf('\n--- TEST 8: dimensionamento automatico di q (rho2, diametro, budget)\n');
% Con toll > 0 il numero di cicli non e' piu' imposto: viene ricavato da
% q >= log(toll)/log(rho2), con almeno il diametro del grafo, e troncato al
% budget radio q_max. Il campo q_misurato dice a quale ciclo il residuo e'
% davvero sceso sotto toll, e verifica la previsione.
toll = 1e-9;
casi_q = struct('nome', {}, 'A', {}, 'qmax', {});
casi_q(1).nome = 'K3 completo, budget 50';       casi_q(1).A = G.A;      casi_q(1).qmax = 50;
casi_q(2).nome = 'catena 1-2-3, budget 50';      casi_q(2).A = G_cat.A;  casi_q(2).qmax = 50;
casi_q(3).nome = 'catena 1-2-3, budget 200';     casi_q(3).A = G_cat.A;  casi_q(3).qmax = 200;
casi_q(4).nome = 'grafo sconnesso, budget 200';  casi_q(4).A = G_scon.A; casi_q(4).qmax = 200;

fprintf('    %-28s %6s %5s %6s %6s %8s %10s\n', ...
        'topologia', 'rho2', 'diam', 'q_teor', 'q_eff', 'q_miss', 'budget_ok');
for c = 1:numel(casi_q)
    [~, iq] = consenso_dwls(F0, a0, casi_q(c).A, casi_q(c).qmax, toll);
    if isnan(iq.q_misurato), q_mis = "mai"; else, q_mis = string(iq.q_misurato); end
    fprintf('    %-28s %6.4f %5d %6s %6d %8s %10s\n', casi_q(c).nome, iq.rho2, ...
            iq.diametro, string(iq.q_teorico), iq.q_eff, q_mis, string(iq.budget_ok));
end
fprintf('    -> su K3 il dimensionamento restituisce q = 1 da solo: rho2 = 0\n');
fprintf('    -> sulla catena servono 52 cicli, quindi un budget di 50 NON basta\n');
fprintf('       e budget_ok lo segnala invece di produrre in silenzio un numero sbagliato\n');
fprintf('    -> su grafo sconnesso q_teorico = Inf: nessun budget puo'' bastare\n');
fprintf('    -> q_teorico e'' conservativo di qualche ciclo rispetto a q_misurato,\n');
fprintf('       perche'' rho2 governa il decadimento ASINTOTICO e trascura la costante\n');

fprintf('\n=================================================================\n');
