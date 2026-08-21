%VERIFICA_Q_CWNA Validazione della forma chiusa contro il metodo di Van Loan.
%
% La Q_d implementata in calcola_Q_cwna.m e' ottenuta risolvendo l'integrale
% di discretizzazione dopo aver CONGELATO theta sull'intervallo di
% campionamento. Questo script quantifica l'errore di quell'approssimazione
% confrontandola con la discretizzazione ESATTA ottenuta con il metodo di
% Van Loan (1978), che valuta l'integrale tramite un singolo esponenziale di
% matrice su un sistema aumentato 10x10:
%
%     Psi = [ -A   Gamma*Q_c*Gamma' ;  0   A' ] * Ts
%     Phi = expm(Psi)
%     A_d = Phi(n+1:2n, n+1:2n)'
%     Q_d = A_d * Phi(1:n, n+1:2n)
%
% Il confronto e' fatto sui soli canali di accelerazione: il canale laterale
% q_lat non fa parte del modello uniciclo (rappresenta proprio cio' che il
% modello NON puo' descrivere) e non ha quindi un riferimento Van Loan.
clear; clc;
addpath(fileparts(mfilename('fullpath')));

par.q_a = 0.10;  par.q_alpha = 0.01;  par.q_lat = 0.0;  par.k_terreno = 0.0;

fprintf('Confronto forma chiusa (theta congelato) vs Van Loan (esatto)\n');
fprintf('%-8s %-8s %-8s   %-14s %-14s %-10s\n', 'Ts[s]','v[m/s]','th[rad]', ...
        'max|diff|', 'max|Qd|', 'errore rel.');
fprintf('%s\n', repmat('-',1,72));

for Ts = [0.1, 0.5, 1.0]
  for v = [2.5, 5.0]
    for th = [0.3, 1.2]
        Q_chiusa = calcola_Q_cwna(th, v, Ts, par);

        % --- Van Loan ---
        A_c = [0 0 -v*sin(th) cos(th) 0;
               0 0  v*cos(th) sin(th) 0;
               0 0  0         0       1;
               0 0  0         0       0;
               0 0  0         0       0];
        M = diag([0 0 0 par.q_a par.q_alpha]);   % Gamma*Q_c*Gamma'
        Psi = [-A_c, M; zeros(5), A_c'] * Ts;
        Phi = expm(Psi);
        A_d = Phi(6:10, 6:10)';
        Q_vl = A_d * Phi(1:5, 6:10);
        Q_vl = (Q_vl + Q_vl')/2;                 % simmetrizzazione numerica

        d = max(abs(Q_chiusa(:) - Q_vl(:)));
        m = max(abs(Q_vl(:)));
        fprintf('%-8.2f %-8.1f %-8.1f   %-14.3e %-14.3e %-10.2e\n', Ts, v, th, d, m, d/m);
    end
  end
end

% --- Controllo di definita positivita' con il canale laterale attivo --------
fprintf('\nDefinita positivita'' di Q_d (con q_lat attivo):\n');
par.q_lat = 0.02;
for th = [0, 0.7, pi/2, 2.9]
    Q = calcola_Q_cwna(th, 2.5, 0.1, par);
    e = eig((Q+Q')/2);
    fprintf('  theta = %5.2f rad -> autovalore minimo = %.3e  (%s)\n', ...
            th, min(e), string(min(e) > 0));
end
fprintf('\nSenza canale laterale (q_lat = 0) Q_d sarebbe singolare:\n');
par.q_lat = 0.0;
Q = calcola_Q_cwna(0.7, 2.5, 0.1, par);
fprintf('  autovalore minimo = %.3e, rango = %d/5\n', min(eig((Q+Q')/2)), rank(Q, 1e-12));
