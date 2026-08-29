%VERIFICA_GRAFO Validazione delle proprieta' spettrali del grafo e dei pesi.
%
% Controlla, su topologie note, che costruisci_grafo.m e pesi_metropolis.m
% riproducano le proprieta' enunciate nel Cap. 17:
%   - L*1 = 0 (lo zero e' sempre autovalore)
%   - lambda2 > 0 se e solo se il grafo e' connesso
%   - lambda2 = n per il grafo completo K_n
%   - Q doppiamente stocastica (righe E colonne a somma unitaria)
%   - rho2 < 1 per grafi connessi
clear; clc;
addpath(fileparts(mfilename('fullpath')));

fprintf('=============== VALIDAZIONE GRAFO E PESI (Cap. 17) ===============\n\n');

casi = struct('nome', {}, 'p', {}, 'Rc', {});
casi(1).nome = 'K3 completo (formazione a V, R_c = 120 m)';
casi(1).p = [0 20; -20 -10; 20 -10]';   casi(1).Rc = 120;
casi(2).nome = 'Catena 1-2-3 (R_c ridotto a 45 m)';
casi(2).p = [0 0; 40 0; 80 0]';         casi(2).Rc = 45;
casi(3).nome = 'Grafo SCONNESSO (R_c = 30 m)';
casi(3).p = [0 0; 20 0; 200 0]';        casi(3).Rc = 30;
casi(4).nome = 'K5 completo';
casi(4).p = [0 0; 1 0; 0 1; 1 1; 0.5 0.5]';  casi(4).Rc = Inf;

for c = 1:numel(casi)
    G = costruisci_grafo(casi(c).p, casi(c).Rc);
    [Q, rho2] = pesi_metropolis(G.A);
    n = size(casi(c).p, 2);

    fprintf('--- %s\n', casi(c).nome);
    fprintf('    archi = %d   gradi = [%s]\n', G.n_archi, num2str(diag(G.D)'));
    fprintf('    L*1 = 0                    : %s (residuo %.2e)\n', ...
            string(norm(G.L*ones(n,1)) < 1e-12), norm(G.L*ones(n,1)));
    fprintf('    lambda_2(L) = %.4f, mol_lambda_1(L) = %d  -> connesso: %s\n', ...
            G.lambda2_L, G.mol_lambda1_L, string(G.connesso));
    fprintf('    Q righe a somma 1          : %s\n', string(all(abs(sum(Q,2)-1) < 1e-12)));
    fprintf('    Q colonne a somma 1        : %s  <- doppiamente stocastica\n', ...
            string(all(abs(sum(Q,1)-1) < 1e-12)));
    fprintf('    Q elementi non negativi    : %s\n', string(all(Q(:) >= -1e-12)));
    fprintf('    rho_2 = %.4f  (< 1: %s)\n\n', rho2, string(rho2 < 1));
end

% --- Verifica analitica: per K_n gli autovalori di L sono {0, n, ..., n} ---
fprintf('--- Confronto con il risultato analitico per il grafo completo K_n\n');
for n = [3 4 5 6]
    G = costruisci_grafo(rand(2,n), Inf);   % Inf -> sempre completo
    fprintf('    K%d: lambda_2(L) misurato = %.4f, atteso = %d  -> %s\n', ...
            n, G.lambda2_L, n, string(abs(G.lambda2_L - n) < 1e-9));
end

% --- Convergenza empirica del consenso contro la previsione di rho2 ---
% Si confronta il decadimento misurato dell'errore di consenso con il fattore
% asintotico rho2 previsto dalla teoria, su due topologie con rho2 diverso.
fprintf('\n--- Convergenza empirica del consenso medio contro la previsione rho2\n');

topol = struct('nome', {'K3 completo (formazione)', 'Catena 1-2-3'}, ...
               'p',    {[0 20; -20 -10; 20 -10]', [0 0; 40 0; 80 0]'}, ...
               'Rc',   {120, 45});

for c = 1:numel(topol)
    G = costruisci_grafo(topol(c).p, topol(c).Rc);
    [Q, rho2] = pesi_metropolis(G.A);
    x0 = [10; -4; 1];  media = mean(x0);
    x = x0;  err = zeros(1,31);
    for k = 1:31
        err(k) = norm(x - media);
        x = Q * x;
    end

    fprintf('\n  %s:  rho2 = %.4f\n', topol(c).nome, rho2);
    if rho2 < 1e-12
        % CASO NOTEVOLE: con grafo completo la regola di Metropolis produce
        % q_ij = 1/(max(d_i,d_j)+1) = 1/n per OGNI coppia, quindi Q = (1/n)*1*1',
        % che e' l'operatore di media esatta. Il consenso converge in UN passo e
        % rho2 = 0. Non e' un caso degenere: e' la configurazione ottima.
        fprintf('    convergenza in UN passo (Q = (1/n)*1*1'', media esatta)\n');
        fprintf('    errore dopo 1 passo = %.2e\n', err(2));
        fprintf('    esito: %s\n', string(err(2) < 1e-12));
    else
        tasso = (err(26)/err(6))^(1/20);
        fprintf('    tasso di decadimento misurato = %.4f\n', tasso);
        fprintf('    coincide con rho2: %s (scarto %.2e)\n', ...
                string(abs(tasso - rho2) < 1e-6), abs(tasso - rho2));
    end
    fprintf('    valore finale = %.6f  (media iniziale = %.6f)\n', x(1), media);
end

% --- Equivalenza fra forma per componenti e forma matriciale ---------------
% Il codice dei main implementa la forma PER COMPONENTI, l'unica realizzabile
% a bordo: l'agente i somma sui soli vicini diretti e non assembla mai il
% Laplaciano globale, che non potrebbe conoscere. La forma matriciale con L
% serve all'ANALISI (lambda_2, costante di tempo, condizione di spanning
% tree), non al calcolo. Qui si verifica che le due producano gli stessi
% numeri, cosi' che l'equivalenza sia dimostrata e non soltanto asserita.
fprintf('\n--- Equivalenza: forma per componenti (nel codice) vs forma con L\n');
rng(3);
p_des = [0 20; -20 -10; 20 -10]';       % formazione desiderata (2 x 3)
p_att = p_des + randn(2,3)*4;           % posizioni perturbate
K     = 0.15;
G     = costruisci_grafo(p_att, 120);

% (a) forma per componenti, identica a quella dei main
u_comp = zeros(2,3);
for i = 1:3
    for j = 1:3
        if G.A(i,j) > 0
            err_ij = (p_att(:,i) - p_att(:,j)) - (p_des(:,i) - p_des(:,j));
            u_comp(:,i) = u_comp(:,i) - K * G.A(i,j) * err_ij;
        end
    end
end

% (b) forma matriciale u = -K*(L kron I_2)*p_tilde
% p_att(:) impila per colonne: [x1;y1;x2;y2;x3;y3], ordinamento coerente con kron.
p_tilde = p_att(:) - p_des(:);
u_mat   = reshape(-K * kron(G.L, eye(2)) * p_tilde, 2, 3);

scarto = max(abs(u_comp(:) - u_mat(:)));
fprintf('    scarto massimo fra le due forme = %.2e\n', scarto);
fprintf('    coincidono: %s\n', string(scarto < 1e-12));
fprintf('    (la forma per componenti usa solo A ed e'' distribuita;\n');
fprintf('     quella con L e'' equivalente ma richiede la topologia globale)\n');

fprintf('\n  NOTA per questo progetto: con N = 3 veicoli e rete full-mesh il\n');
fprintf('  consenso medio converge ESATTAMENTE in una sola iterazione. Gli\n');
fprintf('  algoritmi di stima distribuita del Cap. 18 risulterebbero quindi\n');
fprintf('  esatti gia'' con q = 1, e non approssimati.\n');


% =========================================================================
fprintf('\n--- SPETTRO DI Q: le tre letture ---\n');
% Q e' simmetrica, quindi autovalori reali, ordinati in modo decrescente da
% pesi_metropolis. Si leggono lambda_1 (equilibrio), rho_2 (velocita') e
% lambda_n (oscillazioni).
fprintf('   %-34s %10s %8s %12s %9s\n', 'topologia', 'lambda_1', 'rho_2', 'lambda_min', 'min diag');
for c = 1:numel(casi)
    G = costruisci_grafo(casi(c).p, casi(c).Rc);
    [Q, rho2, ~, ~, lambda_min_Q] = pesi_metropolis(G.A);
    fprintf('   %-34s %10.4f %8.4f %+12.4f %9.4f\n', casi(c).nome, ...
            1, rho2, lambda_min_Q, min(diag(Q)));
end

fprintf('\n--- mol_lambda_1(Q) = mol_lambda_1(L) = numero di componenti connesse\n');
% La connettivita' si legge sulla MOLTEPLICITA' dell'autovalore di consenso, non
% sul suo valore: lambda_1(Q) = 1 e lambda_1(L) = 0 valgono sempre.
for c = 1:numel(casi)
    G = costruisci_grafo(casi(c).p, casi(c).Rc);
    [~, ~, ~, mol_Q] = pesi_metropolis(G.A);
    fprintf('   %-34s mol_lambda_1(Q) = %d, mol_lambda_1(L) = %d  -> %s\n', ...
            casi(c).nome, mol_Q, G.mol_lambda1_L, string(mol_Q == G.mol_lambda1_L));
end

fprintf('\n--- Metropolis non puo oscillare: diag(Q) > 0 sempre\n');
% q_ii = 1 - sum_j 1/(max(d_i,d_j)+1) >= 1 - d_i/(d_i+1) = 1/(d_i+1) > 0,
% perche' max(d_i,d_j) >= d_i su ogni arco. Un peso proprio non nullo rende la
% catena aperiodica ed esclude l'autovalore -1. Controprova su grafo bipartito.
A_cat = [0 1 0; 1 0 1; 0 1 0];                  % catena 1-2-3, bipartita
d_cat = sum(A_cat, 2);
Q_rw  = A_cat ./ d_cat;                          % random walk semplice D^-1 A
[Q_mh, ~, lambda_Q_mh, ~, lambda_min_mh] = pesi_metropolis(A_cat);
fprintf('   random walk D^-1*A : lambda_i(Q) = [%s] -> lambda_min(Q) = %.4f, oscilla\n', ...
        sprintf('%+.4f ', sort(real(eig(Q_rw)), 'descend')), min(real(eig(Q_rw))));
fprintf('   Metropolis         : lambda_i(Q) = [%s] -> lambda_min(Q) = %+.4f, diag = [%s]\n', ...
        sprintf('%+.4f ', lambda_Q_mh), lambda_min_mh, sprintf('%.4f ', diag(Q_mh)));

fprintf('\n--- ordinamento per MODULO: rho_2 = |lambda_2(Q)|, ma lambda_n non e'' il minimo\n');
% Ordinando per modulo decrescente rho_2 coincide per costruzione con
% |lambda_2(Q)|. In cambio l'ultimo elemento non e' piu' l'autovalore piu'
% negativo, e il test sulle oscillazioni va fatto su min_i lambda_i(Q).
% K_{3,3} lo mostra: per modulo lo spettro e' [1, -0.5, 0.25, 0.25, 0.25, 0.25].
m_bip = 3;  A_bip = zeros(2*m_bip);
A_bip(1:m_bip, m_bip+1:end) = 1;  A_bip(m_bip+1:end, 1:m_bip) = 1;
[Q_bip, rho2_bip, lambda_Q_bip, ~, lambda_min_bip] = pesi_metropolis(A_bip);
x = randn(2*m_bip,1); x = x - mean(x); tasso = NaN;
for it = 1:30
    x_new = Q_bip*x;
    if it > 5, tasso = norm(x_new - mean(x_new)) / norm(x - mean(x)); end
    x = x_new;
end
fprintf('   K_{3,3}: lambda_i(Q) per modulo = [%s]\n', sprintf('%+.4f ', lambda_Q_bip));
fprintf('   rho_2 = |lambda_2(Q)| = %.4f   tasso MISURATO = %.4f  -> coincide: %s\n', ...
        rho2_bip, tasso, string(abs(tasso - rho2_bip) < 1e-6));
fprintf('   lambda_n(Q) = %+.4f (ultimo per modulo)  ma lambda_min(Q) = %+.4f\n', ...
        lambda_Q_bip(end), lambda_min_bip);
fprintf('   -> il test sulle oscillazioni usa lambda_min, non lambda_n\n');

fprintf('\n--- Quando Q = I - eps*L (Metropolis = pesi a massimo grado)\n');
% Accade se e solo se max(d_i,d_j) e' lo stesso su ogni arco. In quel caso
% rho_2 e lambda_2(L) sono lo stesso numero: rho_2 = 1 - eps*lambda_2(L).
for c = 1:numel(casi)
    G = costruisci_grafo(casi(c).p, casi(c).Rc);
    [Q, rho2] = pesi_metropolis(G.A);
    fuori = Q(~eye(size(Q)) & G.A > 0);
    if isempty(fuori) || ~all(abs(fuori - fuori(1)) < 1e-12)
        fprintf('   %-34s pesi NON uniformi -> nessuna relazione di scala\n', casi(c).nome);
    else
        eps_w = fuori(1);
        ok    = norm(Q - (eye(size(Q)) - eps_w*G.L), 'fro') < 1e-12;
        fprintf('   %-34s eps = %.4f, Q = I - eps*L: %s, rho_2 previsto %.4f (misurato %.4f)\n', ...
                casi(c).nome, eps_w, string(ok), 1 - eps_w*G.lambda2_L, rho2);
    end
end

fprintf('\n=================================================================\n');
