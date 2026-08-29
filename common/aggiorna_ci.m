function [x_new, Sigma_new, gamma_ci] = aggiorna_ci(x_pred, Sigma_bar, z, z_pred, C_k, R_eff)
%   Aggiornamento EKF in forma Covariance Intersection.
%
%   [x_new, Sigma_new, gamma_ci] = AGGIORNA_CI(x_pred, Sigma_bar, z, z_pred, C_k, R_eff)
%   fonde la stima a priori (x_pred, Sigma_bar) con un blocco di misure
%   (z, C_k, R_eff) la cui correlazione con la stima a priori e' SCONOSCIUTA,
%   restituendo una stima garantita consistente per qualunque valore di quella
%   correlazione. Sostituisce ekf_update sul solo aggiornamento collaborativo.
%
%   PERCHE' SERVE
%   Il guadagno di Kalman standard presuppone che l'innovazione sia scorrelata
%   dalla stima a priori. Nella localizzazione collaborativa non lo e': la
%   posa del vicino usata come ancora mobile contiene informazione che quel
%   vicino ha a sua volta ricevuto, e che e' gia' dentro il proprio filtro. La
%   covarianza incrociata P_ij e' reale ma non tracciabile in modo distribuito,
%   e ignorarla (P_ij = 0) produce una Sigma artificialmente piccola: il filtro
%   diventa OTTIMISTA e degenera nel data rumination.
%
%   FORMULAZIONE (Julier e Uhlmann, 1997)
%   Date due stime (x_1, P_1) e (x_2, P_2) della stessa grandezza, la CI le
%   combina nello spazio dell'informazione con un peso convesso:
%       P_cc^-1 = gamma*P_1^-1 + (1-gamma)*P_2^-1
%       x_cc    = P_cc * [gamma*P_1^-1*x_1 + (1-gamma)*P_2^-1*x_2]
%   Qui la prima stima e' il proprio a priori e la seconda e' l'informazione
%   portata dalla misura, C'*R_eff^-1*C, per cui
%       Sigma^-1 = gamma*Sigma_bar^-1 + (1-gamma)*C'*R_eff^-1*C
%
%   LA CI E' UN KALMAN CON PRIOR E MISURA SGONFIATI
%   Ponendo Sigma_g = Sigma_bar/gamma e R_g = R_eff/(1-gamma), l'espressione
%   sopra diventa Sigma^-1 = Sigma_g^-1 + C'*R_g^-1*C, cioe' esattamente la
%   forma informativa dell'aggiornamento di Kalman. Le equazioni sono quindi
%   quelle di sempre, applicate a un a priori e a un rumore di misura entrambi
%   gonfiati. E' anche il motivo per cui la CI si innesta su un EKF senza
%   riscriverne l'architettura: la nonlinearita' resta confinata in C_k e in
%   z_pred, come nell'aggiornamento standard.
%
%   SCELTA DEL PESO gamma
%   gamma e' scelto a ogni fusione minimizzando la "dimensione" della Sigma
%   risultante. Il problema e' convesso e scalare su [0, 1], quindi si risolve
%   con una ricerca monodimensionale (fminbnd: sezione aurea con interpolazione
%   parabolica). Si minimizza la TRACCIA DEL BLOCCO DI POSIZIONE e non quella
%   dell'intera Sigma: lo stato mescola metri, radianti e velocita', e sommarne
%   le varianze darebbe un costo dimensionalmente incoerente, dominato
%   dall'unita' di misura piu' grande. La misura collaborativa e' inoltre una
%   misura di posizione, quindi e' li' che il peso va deciso.
%
%   DUE PROPRIETA' CHE NE DISCENDONO
%   a) per gamma = 1 la misura viene ignorata e Sigma = Sigma_bar. Il caso e'
%      confrontato esplicitamente con l'ottimo trovato, quindi la fusione non
%      degrada mai la stima rispetto al non fondere.
%   b) per gamma -> 0 si butta via l'a priori e Sigma esplode nelle direzioni
%      che la misura non vincola. Una misura di sola distanza ne vincola una
%      sola, quindi il costo diverge e l'ottimo resta interno.
%   La minimizzazione e' su un criterio, non sulla consistenza: quest'ultima
%   vale per QUALUNQUE gamma in [0, 1], ed e' il senso del teorema di Julier.
%
%   SOGLIA DI ACCOGLIMENTO
%   Con a priori isotropo di varianza s e misura scalare di varianza R lungo
%   una direzione, la derivata del costo in gamma = 1 vale s^2/R - 2s: l'ottimo
%   lascia il bordo, cioe' la misura viene effettivamente usata, solo se
%       dev(a priori) > sqrt(2) * dev(misura)
%   Una misura appena peggiore dell'a priori viene quindi scartata del tutto e
%   non semplicemente pesata poco. E' il prezzo della robustezza: la CI
%   rinuncia all'informazione debole pur di non sbagliare mai. La soglia e'
%   verificata numericamente in verifica_ci.m, TEST 6.
%
%   IL PREZZO
%   La CI e' conservativa per costruzione: Sigma sovrastima quasi sempre la
%   covarianza reale dell'errore. Rinuncia all'ottimalita' in cambio della
%   garanzia di non divergere, che in una rete ad hoc con perdite e ritardi
%   variabili e' l'unica proprieta' realmente difendibile.
%
%   INGRESSI
%       x_pred     [n x 1] stima a priori (o posteriori del blocco indipendente)
%       Sigma_bar  [n x n] covarianza associata
%       z          [m x 1] misure correlate in modo ignoto con x_pred
%       z_pred     [m x 1] misure predette, h(x_pred)
%       C_k        [m x n] Jacobiano dh/dx valutato in x_pred
%       R_eff      [m x m] covarianza di misura, gia' comprensiva
%                          dell'incertezza proiettata del vicino
%
%   USCITE
%       x_new      [n x 1] stima fusa
%       Sigma_new  [n x n] covarianza fusa, simmetrizzata
%       gamma_ci   peso ottimo trovato: vicino a 1 significa misura poco
%                  informativa o vicino molto incerto, vicino a 0 il contrario
%
%   Teoria completa: theory/TEORIA_CI.pdf e README.md §2.3.

n = numel(x_pred);

% Estremi della ricerca. Non si arriva a 0 e 1 esatti perche' li' i due fattori
% di scala divergono; il troncamento a 1e-4 lascia comunque quattro ordini di
% grandezza di escursione, ben oltre quanto il minimo richiede.
gamma_min = 1e-4;
gamma_max = 1 - 1e-4;

% Tolleranza lasca di proposito: gamma entra come fattore di scala, e uno
% scarto di 1e-3 sul peso e' irrilevante rispetto all'incertezza in gioco. Si
% risparmiano meta' delle valutazioni della funzione di costo.
opt = optimset('TolX', 1e-3);
gamma_ci = fminbnd(@costo, gamma_min, gamma_max, opt);

[x_new, Sigma_new] = fondi(gamma_ci);

% gamma = 1 ESATTO, CHE LA RICERCA NON PUO' RAGGIUNGERE
% e' il caso "non fondere": la misura viene ignorata e la stima resta l'a
% priori. Appartiene all'insieme ammissibile ma cade sull'estremo escluso,
% quindi la ricerca vi si avvicina senza toccarlo e lascia un gonfiamento
% residuo dell'ordine di 1e-4. Su una fusione sola e' irrilevante, ma il filtro
% fonde a ogni passo e il residuo si accumulerebbe. Il confronto esplicito con
% il costo di non fondere rende esatta la proprieta' "la fusione non peggiora
% mai", che altrimenti varrebbe solo a meno della tolleranza di ricerca.
%
% IL CONFRONTO E' STRETTO, E LA DIFFERENZA CONTA
% quando le due informazioni si equivalgono il costo e' PIATTO in gamma: la
% covarianza risultante e' la stessa per ogni peso, ma la stima no, perche' e'
% la media convessa delle due. Scartare in caso di pareggio butterebbe via un
% miglioramento reale dell'errore senza guadagnare nulla sulla covarianza
% dichiarata. La soglia relativa separa il pareggio (scarto nullo a meno
% dell'arrotondamento) dal troncamento del bordo, che e' quattro ordini di
% grandezza piu' grande.
tr_bar = Sigma_bar(1,1) + Sigma_bar(2,2);
if costo(gamma_ci) > tr_bar * (1 + 1e-9)
    gamma_ci  = 1;
    x_new     = x_pred;
    Sigma_new = Sigma_bar;
end

    function c = costo(g)
        [~, Sg] = fondi(g);
        c = Sg(1,1) + Sg(2,2);       % traccia del blocco di posizione
    end

    function [xf, Sf] = fondi(g)
        Sigma_g = Sigma_bar / g;
        R_g     = R_eff / (1 - g);

        S  = C_k * Sigma_g * C_k' + R_g;
        K  = Sigma_g * C_k' / S;

        xf = x_pred + K * (z - z_pred);
        xf(3) = wrapToPi(xf(3));

        % Simmetrizzazione: la divisione per gamma amplifica anche l'asimmetria
        % numerica accumulata, che senza questo passaggio si propaga di passo in
        % passo fino a rendere Sigma non definita positiva.
        Sf = (eye(n) - K * C_k) * Sigma_g;
        Sf = (Sf + Sf') / 2;
    end
end
