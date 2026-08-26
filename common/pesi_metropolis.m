function [Q, rho2] = pesi_metropolis(A)
%   Matrice di consenso doppiamente stocastica (Metropolis-Hastings).
%
%   [Q, rho2] = PESI_METROPOLIS(A) costruisce la matrice di transizione Q del
%   protocollo di consenso lineare x(k+1) = Q x(k) a partire dalla matrice di
%   adiacenza A, secondo la regola di Metropolis-Hastings.
%
%   REGOLA DEI PESI
%       q_ij = 1 / (max(d_i, d_j) + 1)        se (j,i) e' un arco, i ~= j
%       q_ii = 1 - sum_{k ~= i} q_ik
%       q_ij = 0                              altrimenti
%
%   dove d_i e' il grado del nodo i. Ogni nodo calcola i propri pesi conoscendo
%   soltanto il proprio grado e quello dei vicini diretti: nessuna conoscenza
%   della topologia globale e' richiesta, il che rende la regola effettivamente
%   distribuita.
%
%   PERCHE' DOPPIAMENTE STOCASTICA
%   Una matrice Q e' stocastica se ha elementi non negativi e righe a somma
%   unitaria (Q*1 = 1); e' DOPPIAMENTE stocastica se anche le colonne sommano a
%   uno (1'*Q = 1'). La distinzione determina il valore di convergenza:
%     - Q solo stocastica  -> gli agenti convergono a una combinazione pesata
%                             arbitraria degli stati iniziali;
%     - Q doppiamente st.  -> convergono esattamente alla MEDIA ARITMETICA,
%                             ossia si ottiene l'AVERAGE CONSENSUS.
%   L'average consensus e' il prerequisito degli algoritmi di stima distribuita
%   del Cap. 18 (D-WLS e DKF), dove la media dei contributi informativi locali
%   deve ricostruire la somma globale a meno del fattore 1/n.
%   La regola di Metropolis produce una Q simmetrica, e quindi automaticamente
%   doppiamente stocastica.
%
%   VELOCITA' DI CONVERGENZA
%   rho2 e' l'ESSENTIAL SPECTRAL RADIUS, cioe' il modulo del secondo autovalore
%   piu' grande di Q. Il fattore di convergenza asintotico del consenso vale
%   esattamente rho2: piu' e' piccolo, piu' rapida e' la convergenza. Vale
%   sempre rho2 < 1 per grafi connessi.
%
%   La regola di Metropolis converge sensibilmente piu' in fretta dei pesi a
%   massimo grado (q = 1/n_d con n_d > max_i d_i), perche' sfrutta il grado
%   locale dei vicini anziche' un limite superiore valido per tutta la rete.

    n = size(A, 1);
    d = sum(A, 2);          % grado di ciascun nodo

    Q = zeros(n);
    for i = 1:n
        for j = 1:n
            if i ~= j && A(i,j) > 0
                Q(i,j) = 1 / (max(d(i), d(j)) + 1);
            end
        end
        Q(i,i) = 1 - sum(Q(i, [1:i-1, i+1:n]));
    end

    % Essential spectral radius: secondo autovalore in modulo.
    % Il primo vale sempre 1, con autovettore il vettore di soli uni.
    autov = sort(abs(eig(Q)), 'descend');
    if n > 1
        rho2 = autov(2);
    else
        rho2 = 0;
    end
end
