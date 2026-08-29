function [Q, rho2, lambda_Q, mol_lambda1_Q, lambda_min_Q] = pesi_metropolis(A)
%   Matrice di consenso doppiamente stocastica (Metropolis-Hastings).
%
%   [Q, rho2, lambda_Q, mol_lambda1_Q, lambda_min_Q] = PESI_METROPOLIS(A)
%   costruisce la matrice di transizione Q del
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
%   SPETTRO DI Q E SUA LETTURA
%   lambda_Q contiene gli autovalori di Q ottenuti da det(Q - lambda*I) = 0.
%   Attenzione a non scrivere det(I - lambda*Q) = 0, che ne restituisce i
%   RECIPROCI. Q e' simmetrica, quindi sono tutti reali.
%
%   ORDINAMENTO: PER MODULO DECRESCENTE
%   |lambda_1| >= |lambda_2| >= ... >= |lambda_n|. E' la convenzione standard
%   nella letteratura sul consenso, e ha il vantaggio di rendere
%       rho2 = |lambda_2(Q)|
%   vero per costruzione, senza dover cercare il massimo modulo fra tutti gli
%   autovalori diversi dal primo.
%
%   Ha pero' una conseguenza da tenere presente: con questo ordinamento
%   lambda_n(Q) e' l'autovalore piu' vicino a ZERO, non il piu' negativo. Il
%   test sulle oscillazioni va quindi scritto su lambda_min_Q = min_i lambda_i,
%   che non dipende dalla posizione nell'ordinamento. Su K_{3,3} con pesi di
%   Metropolis lo spettro per modulo e' [1, -0.5, 0.25, 0.25, 0.25, 0.25]:
%   lambda_n vale +0.25 e mancherebbe completamente il -0.5.
%
%   Se ne leggono tre cose:
%
%     lambda_1(Q) = 1   Q e' stocastica, quindi 1 e' SEMPRE autovalore, con
%                    autovettore il vettore di soli uni. Non e' una condizione
%                    da verificare ma una GARANZIA strutturale: il consenso
%                    ammette uno stato di equilibrio e non diverge mai.
%
%     mol_lambda1_Q  molteplicita' di lambda_1(Q), pari al numero di COMPONENTI
%                    CONNESSE: vale 1 su grafo connesso, k se la rete e'
%                    spezzata in k gruppi che convergono ciascuno alla propria
%                    media. E' qui che sta l'informazione sulla connettivita',
%                    non nel valore dell'autovalore.
%
%     rho2           ESSENTIAL SPECTRAL RADIUS, pari a |lambda_2(Q)| con
%                    l'ordinamento per modulo adottato qui. L'errore di consenso
%                    decade come rho2^q: rho2 -> 0 rete velocissima (zero esatto
%                    sul grafo completo), rho2 -> 1 collo di bottiglia. Vale
%                    rho2 < 1 se e solo se il grafo e' connesso.
%
%     lambda_min_Q   l'autovalore piu' NEGATIVO, min_i lambda_i(Q). Con
%                    l'ordinamento per modulo non coincide con lambda_n(Q) e va
%                    cercato esplicitamente. Se tendesse a -1 il consenso
%                    oscillerebbe,
%                    perche' -1 e' autovalore di una catena periodica, cioe' di
%                    un grafo bipartito percorso senza mai restare fermi.
%                    CON I PESI DI METROPOLIS NON PUO' ACCADERE: la diagonale e'
%                    sempre strettamente positiva,
%                        q_ii = 1 - sum_j 1/(max(d_i,d_j)+1) >= 1/(d_i+1) > 0
%                    perche' max(d_i,d_j) >= d_i su ogni arco. Un peso proprio
%                    non nullo rende la catena aperiodica ed esclude -1. Il
%                    controllo resta utile per altre scelte di pesi: il random
%                    walk semplice D^-1*A su grafo bipartito da' lambda_n = -1.
%
%   RELAZIONE CON IL LAPLACIANO
%   I - Q e' sempre un Laplaciano PESATO (righe a somma nulla). Quando tutti i
%   pesi fuori diagonale risultano uguali a un valore eps, cosa che accade se e
%   solo se max(d_i,d_j) e' lo stesso su ogni arco, Metropolis coincide con i
%   pesi a massimo grado e vale l'identita'
%       Q = I - eps*L,   eps = 1/(d_max+1),   rho2 = 1 - eps*lambda_2(L)
%   In quel caso rho2 e lambda_2(L) sono lo stesso numero letto due volte. In
%   generale NO: bastano due nodi di grado diverso non adiacenti a un nodo di
%   grado massimo perche' i pesi si differenzino e la relazione cada.
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

    % Spettro di Q. Simmetrica per costruzione, quindi autovalori reali.
    % Ordinati per MODULO decrescente: lambda_Q(1) = 1 e rho2 = |lambda_Q(2)|.
    autov = real(eig(Q));
    [~, ordine] = sort(abs(autov), 'descend');
    lambda_Q    = autov(ordine);

    mol_lambda1_Q = sum(abs(lambda_Q - 1) < 1e-9);

    % Con l'ordinamento per modulo, rho2 e' per costruzione il secondo elemento.
    if n > 1
        rho2 = abs(lambda_Q(2));
    else
        rho2 = 0;
    end

    % Autovalore piu' negativo, cercato esplicitamente: con l'ordinamento per
    % modulo puo' trovarsi in qualunque posizione, non necessariamente in fondo.
    lambda_min_Q = min(lambda_Q);
end
