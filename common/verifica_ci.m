% VERIFICA_CI  Validazione dell'aggiornamento in Covariance Intersection.
%
% Sei test che coprono le proprieta' su cui la Fase 5 si appoggia. Il quarto e'
% quello che conta davvero: dimostra su campagna Monte Carlo che la CI resta
% consistente dove il guadagno di Kalman standard diventa ottimista.
%
% I test 5 e 6 rispondono alla domanda che il risultato di Fase 5 solleva: il
% peso gamma torna quasi sempre pari a 1, cioe' la misura collaborativa viene
% scartata. E' il comportamento corretto per la geometria di quella fase, non
% un ottimizzatore bloccato, e qui lo si dimostra esibendo la soglia esatta a
% cui il comportamento si inverte.

clear; clc;
addpath(fileparts(mfilename('fullpath')));
fprintf('=== VALIDAZIONE DELLA COVARIANCE INTERSECTION ===\n\n');

% Stato di comodo a cinque componenti, come nel progetto. I test lavorano sul
% blocco di posizione: le altre tre componenti restano indifferenti.
C_pos = [eye(2), zeros(2,3)];
x0    = zeros(5,1);

%% TEST 1: equivalenza con la forma informativa
% La funzione implementa la CI come Kalman con prior e misura sgonfiati. Deve
% restituire esattamente cio' che la definizione prescrive nello spazio
% dell'informazione, Sigma^-1 = gamma*Sigma_bar^-1 + (1-gamma)*C'*R^-1*C.
Sigma_bar = diag([0.40, 0.25, 0.05, 0.02, 0.01]);
R_test    = diag([0.30, 0.60]);
z_test    = [0.5; -0.3];

[~, Sigma_ci, g1] = aggiorna_ci(x0, Sigma_bar, z_test, C_pos*x0, C_pos, R_test);
Sigma_att = inv(g1*inv(Sigma_bar) + (1-g1)*(C_pos'/R_test)*C_pos);   %#ok<MINV>
err_forma = norm(Sigma_ci - Sigma_att) / norm(Sigma_att);

fprintf('TEST 1  Forma informativa\n');
fprintf('   Sigma^-1 = gamma*Sigma_bar^-1 + (1-gamma)*C R^-1 C : errore %.2e\n', err_forma);
fprintf('   esito: %s\n\n', string(err_forma < 1e-10));

%% TEST 2: il peso si muove
% Se gamma tornasse sempre 1 la ricerca sarebbe inerte. Si sottopone la stessa
% funzione a due regimi opposti: misura molto peggiore dell'a priori, e misura
% molto migliore. Il peso deve spostarsi da un estremo all'altro.
S_stretta = diag([0.01, 0.01, 0.05, 0.02, 0.01]);   % a priori buono
S_larga   = diag([4.00, 4.00, 0.05, 0.02, 0.01]);   % a priori pessimo
R_debole  = 1.00 * eye(2);
R_forte   = 0.01 * eye(2);

[~, ~, g_scarta] = aggiorna_ci(x0, S_stretta, z_test, C_pos*x0, C_pos, R_debole);
[~, ~, g_accoglie] = aggiorna_ci(x0, S_larga, z_test, C_pos*x0, C_pos, R_forte);

fprintf('TEST 2  Escursione del peso\n');
fprintf('   a priori 0.10 m contro misura 1.00 m : gamma = %.4f (misura scartata)\n', g_scarta);
fprintf('   a priori 2.00 m contro misura 0.10 m : gamma = %.4f (misura accolta)\n', g_accoglie);
fprintf('   esito: %s\n\n', string(g_scarta > 0.99 && g_accoglie < 0.10));

%% TEST 3: la fusione non peggiora mai
% gamma = 1 corrisponde a ignorare la misura, quindi l'ottimo non puo' fare
% peggio del non fondere. La ricerca scalare non raggiunge pero' gamma = 1
% esatto, perche' li' il fattore 1/(1-gamma) diverge, e senza il confronto
% esplicito che aggiorna_ci fa con quel caso resterebbe un gonfiamento residuo
% dello 0.04% a ogni passo. Il test verifica che il confronto ci sia.
n_prove  = 500;
peggiora = 0;
ecc_max  = 0;
n_scarti = 0;
rng(1);
for p = 1:n_prove
    Sp = diag([rand*2+0.01, rand*2+0.01, 0.05, 0.02, 0.01]);
    Rp = diag([rand*2+0.01, rand*2+0.01]);
    [~, Sc, g_p] = aggiorna_ci(x0, Sp, randn(2,1), C_pos*x0, C_pos, Rp);
    ecc_max  = max(ecc_max, trace(Sc(1:2,1:2))/trace(Sp(1:2,1:2)) - 1);
    n_scarti = n_scarti + (g_p == 1);
    if trace(Sc(1:2,1:2)) > trace(Sp(1:2,1:2)) + 1e-12
        peggiora = peggiora + 1;
    end
end
fprintf('TEST 3  Monotonia rispetto al non fondere\n');
fprintf('   tr(Sigma_pos) mai superiore all a priori : %d violazioni su %d\n', peggiora, n_prove);
fprintf('   misure scartate (gamma = 1 esatto) : %d su %d\n', n_scarti, n_prove);
fprintf('   eccesso massimo residuo : %.2e\n', max(ecc_max, 0));
fprintf('   esito: %s\n\n', string(peggiora == 0));

%% TEST 4: consistenza sotto correlazione ignota
% E' l'unico test che giustifica l'esistenza della CI nel progetto.
%
% Si costruiscono due stime della STESSA posizione i cui errori condividono una
% componente comune: e_1 = a*w_1 + c*w_c, e_2 = b*w_2 + c*w_c. La covarianza
% incrociata vale c^2 ed e' reale, ma nessuno dei due stimatori la conosce.
% Ciascuno dichiara onestamente la propria covarianza, che e' corretta presa da
% sola. Il caso e' deliberatamente LINEARE, cosi' il test isola la regola di
% fusione dall'errore di linearizzazione.
%
% Si confrontano poi due fusioni: guadagno di Kalman standard, che assume
% implicitamente P_12 = 0, e Covariance Intersection. Il giudizio e' il NEES,
% (e' * Sigma^-1 * e) mediato sulle prove: con due gradi di liberta' il valore
% atteso e' 2, e superarlo significa dichiarare meno incertezza di quanta se ne
% abbia.
M   = 20000;
a_c = 0.30; b_c = 0.30; c_c = 0.50;
P1  = (a_c^2 + c_c^2) * eye(2);
P2  = (b_c^2 + c_c^2) * eye(2);
rho = c_c^2 / (a_c^2 + c_c^2);

Sigma_1 = blkdiag(P1, diag([0.05, 0.02, 0.01]));
nees_kf = zeros(1, M);
nees_ci = zeros(1, M);
err_kf  = zeros(2, M);
err_ci  = zeros(2, M);

rng(2);
for m = 1:M
    w_c = c_c * randn(2,1);
    e_1 = a_c * randn(2,1) + w_c;
    e_2 = b_c * randn(2,1) + w_c;

    x_1 = [e_1; zeros(3,1)];    % stima 1: verita' + errore
    z_2 = e_2;                  % stima 2, vista come misura di posizione

    % Fusione ingenua: Kalman con R = P2, cioe' P_12 = 0 imposto
    S_kf = C_pos * Sigma_1 * C_pos' + P2;
    K_kf = Sigma_1 * C_pos' / S_kf;
    x_kf = x_1 + K_kf * (z_2 - C_pos * x_1);
    P_kf = (eye(5) - K_kf * C_pos) * Sigma_1;

    % Fusione robusta
    [x_ci, P_ci] = aggiorna_ci(x_1, Sigma_1, z_2, C_pos * x_1, C_pos, P2);

    err_kf(:,m) = x_kf(1:2);
    err_ci(:,m) = x_ci(1:2);
    nees_kf(m)  = x_kf(1:2)' * (P_kf(1:2,1:2) \ x_kf(1:2));
    nees_ci(m)  = x_ci(1:2)' * (P_ci(1:2,1:2) \ x_ci(1:2));
end

emp_kf = cov(err_kf');
emp_ci = cov(err_ci');
fprintf('TEST 4  Consistenza sotto correlazione ignota (%d prove)\n', M);
fprintf('   correlazione reale fra le due stime : %.2f (ignota agli stimatori)\n', rho);
fprintf('   Kalman standard : NEES %.2f   dichiarato %.4f m^2, reale %.4f m^2\n', ...
        mean(nees_kf), P_kf(1,1), emp_kf(1,1));
fprintf('   Cov. Intersect. : NEES %.2f   dichiarato %.4f m^2, reale %.4f m^2\n', ...
        mean(nees_ci), P_ci(1,1), emp_ci(1,1));
fprintf('   il primo sottostima l incertezza, il secondo la maggiora\n');
fprintf('   esito: %s\n\n', string(mean(nees_kf) > 2 && mean(nees_ci) <= 2));

%% TEST 5: il caso della Fase 5
% In zona cieca l'a priori del veicolo e' gia' corretto da cinque ancore fisse
% a sigma = 0.5 m: sulla congiungente vale circa 0.09 m, contro i 0.62 m della
% misura collaborativa. La CI deve scartarla.
dev_pri = 0.09;
dev_mis = 0.62;
S_f5    = blkdiag(dev_pri^2 * eye(2), diag([0.05, 0.02, 0.01]));
[~, S_out, g_f5] = aggiorna_ci(x0, S_f5, 0.4, 0, [1 0 0 0 0], dev_mis^2);

fprintf('TEST 5  Geometria della Fase 5\n');
fprintf('   a priori %.2f m contro misura %.2f m : gamma = %.4f\n', dev_pri, dev_mis, g_f5);
fprintf('   tr(Sigma_pos) da %.5f a %.5f m^2 (variazione %+.2f%%)\n', ...
        trace(S_f5(1:2,1:2)), trace(S_out(1:2,1:2)), ...
        100*(trace(S_out(1:2,1:2))/trace(S_f5(1:2,1:2)) - 1));
fprintf('   esito: %s (la misura non porta informazione utile)\n\n', string(g_f5 > 0.99));

%% TEST 6: soglia analitica di accoglimento
% Con a priori isotropo di varianza s e misura scalare di varianza R lungo una
% direzione, la traccia vale f(gamma) = 1/(gamma/s + (1-gamma)/R) + s/gamma e
% la sua derivata in gamma = 1 e' s^2/R - 2s. Il minimo esce dal bordo, cioe'
% la misura viene accolta, solo per s > 2R:
%
%       dev(a priori) > sqrt(2) * dev(misura)
%
% La misura collaborativa deve quindi essere piu' di 1.41 volte MIGLIORE
% dell'a priori per meritare di essere usata. E' il prezzo della robustezza:
% la CI accetta di perdere informazione debole pur di non sbagliare mai.
soglia_teorica = sqrt(2);
rapporti = [0.5, 1.0, 1.35, 1.45, 2.0, 4.0];
fprintf('TEST 6  Soglia di accoglimento, dev(a priori)/dev(misura)\n');
fprintf('   soglia teorica sqrt(2) = %.4f\n', soglia_teorica);
ok_soglia = true;
esiti     = {'scartata', 'accolta'};
for rr = rapporti
    S_r = blkdiag((rr*0.5)^2 * eye(2), diag([0.05, 0.02, 0.01]));
    [~, ~, g_r] = aggiorna_ci(x0, S_r, 0.1, 0, [1 0 0 0 0], 0.5^2);
    accolta = g_r < 0.99;
    fprintf('   rapporto %.2f -> gamma = %.4f  %s\n', rr, g_r, esiti{accolta + 1});
    ok_soglia = ok_soglia && (accolta == (rr > soglia_teorica));
end
fprintf('   esito: %s\n\n', string(ok_soglia));

fprintf('=== FINE VALIDAZIONE ===\n');
