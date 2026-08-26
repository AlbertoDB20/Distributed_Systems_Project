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
    fprintf('    lambda2 = %.4f  -> connesso: %s\n', G.lambda2, string(G.connesso));
    fprintf('    Q righe a somma 1          : %s\n', string(all(abs(sum(Q,2)-1) < 1e-12)));
    fprintf('    Q colonne a somma 1        : %s  <- doppiamente stocastica\n', ...
            string(all(abs(sum(Q,1)-1) < 1e-12)));
    fprintf('    Q elementi non negativi    : %s\n', string(all(Q(:) >= -1e-12)));
    fprintf('    rho2 = %.4f  (< 1: %s)\n\n', rho2, string(rho2 < 1));
end

% --- Verifica analitica: per K_n gli autovalori di L sono {0, n, ..., n} ---
fprintf('--- Confronto con il risultato analitico per il grafo completo K_n\n');
for n = [3 4 5 6]
    G = costruisci_grafo(rand(2,n), Inf);   % Inf -> sempre completo
    fprintf('    K%d: lambda2 misurato = %.4f, atteso = %d  -> %s\n', ...
            n, G.lambda2, n, string(abs(G.lambda2 - n) < 1e-9));
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
