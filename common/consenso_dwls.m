function [X_est, info] = consenso_dwls(F_loc, a_loc, A, q_max, toll)
%   Minimi Quadrati Pesati Distribuiti (D-WLS) via consenso sulla media.
%
%   [X_est, info] = CONSENSO_DWLS(F_loc, a_loc, A, q_max, toll) risolve in modo
%   distribuito il problema di stima di un parametro COSTANTE e NON STOCASTICO
%   x appartenente a R^m, a partire dai contributi informativi locali dei nodi
%   e dalla sola matrice di adiacenza del grafo di comunicazione (Cap. 18).
%
%   ARGOMENTI
%     F_loc   m x m x n   matrice di informazione locale di ciascun nodo,
%                         F_i(0) = C_i' * R_i^-1 * C_i
%     a_loc   m x n       stato di informazione locale di ciascun nodo,
%                         a_i(0) = C_i' * R_i^-1 * z_i
%     A       n x n       matrice di adiacenza del grafo
%     q_max   scalare     numero MASSIMO di cicli di consenso consentiti dal
%                         budget radio disponibile nel passo di campionamento
%     toll    scalare     tolleranza sul residuo di consenso. Il numero di
%                         cicli viene dimensionato da rho2 per raggiungerla e
%                         poi troncato a q_max. Con toll <= 0 (o argomento
%                         omesso) vengono eseguiti esattamente q_max cicli.
%
%   RESTITUISCE
%     X_est   m x n       stima ricostruita localmente da ciascun nodo
%     info    struct      diagnostica (vedi in fondo)
%
%   -----------------------------------------------------------------------
%   NOTAZIONE. Il testo del corso scrive il modello di misura come
%   z(i) = H(i) x + eps(i) con covarianza C{eps(i)}. Questo progetto adotta
%   integralmente la notazione di Thrun (vedi README §2.6), in cui la matrice
%   di regressione si chiama C e la covarianza del rumore di misura R:
%
%       H(i)        ->  C_i     matrice di regressione locale
%       C{eps(i)}   ->  R_i     covarianza del rumore di misura locale
%
%   ATTENZIONE: nel testo del corso C{.} e' l'OPERATORE di covarianza, qui C
%   e' la matrice di regressione. I due simboli si scambiano di ruolo.
%   -----------------------------------------------------------------------
%
%   PROBLEMA
%   Ogni nodo i acquisisce z_i = C_i x + eps_i, con eps_i a media nulla e
%   covarianza R_i, rumori scorrelati fra nodi diversi. La soluzione WLS
%   centralizzata e la covarianza dell'errore di stima valgono
%
%       x_LS = (C' R^-1 C)^-1 C' R^-1 z ,     P = (C' R^-1 C)^-1
%
%   con C e z impilati su tutti i nodi. Poiche' R e' diagonale a blocchi (i
%   rumori sono scorrelati), entrambe si decompongono in SOMME di contributi
%   puramente locali:
%
%       x_LS = ( sum_i C_i' R_i^-1 C_i )^-1  ( sum_i C_i' R_i^-1 z_i )
%            = ( sum_i F_i(0) )^-1 ( sum_i a_i(0) )
%
%   Nessun nodo deve quindi accumulare la matrice globale: basta che ciascuno
%   sappia calcolare le due SOMME. Ed e' esattamente cio' che l'average
%   consensus sa fare senza coordinamento centrale.
%
%   ALGORITMO IN TRE FASI
%     FASE 1 - Inizializzazione locale. Ogni nodo calcola F_i(0) e a_i(0)
%              dalla propria misura. Sono forniti in ingresso a questa
%              funzione.
%     FASE 2 - q cicli di consenso sulla media, con pesi doppiamente
%              stocastici di Metropolis-Hastings:
%                  F_i(k+1) = q_ii F_i(k) + sum_{j in N_i} q_ij F_j(k)
%                  a_i(k+1) = q_ii a_i(k) + sum_{j in N_i} q_ij a_j(k)
%              Su grafo connesso entrambe convergono alla media aritmetica
%              dei valori iniziali, cioe' alla somma globale divisa per n.
%     FASE 3 - Ricostruzione locale: x_i = F_i(q)^-1 a_i(q).
%
%   DIMENSIONAMENTO DEL NUMERO DI CICLI q
%   Il residuo di consenso decade geometricamente con ragione rho2, il raggio
%   spettrale essenziale di Q (Cap. 17). Per scendere sotto una tolleranza
%   assegnata servono quindi
%
%       q >= log(toll) / log(rho2)
%
%   cicli, e almeno tanti quanti il DIAMETRO del grafo: sotto quella soglia
%   l'informazione non ha materialmente attraversato la rete, e due nodi a
%   distanza 3 non sanno nulla l'uno dell'altro finche' non sono trascorsi 3
%   scambi. Il valore ottenuto viene troncato a q_max, cioe' al numero di
%   scambi che il canale radio sostiene in un passo di campionamento.
%
%   Il dimensionamento e' un calcolo del PROGETTISTA, non dell'agente: rho2 e
%   il diametro sono proprieta' globali del grafo, come lambda_2 e L. A bordo
%   il numero di cicli e' un parametro di configurazione, non una decisione
%   presa in tempo reale. Il campo info.q_misurato riporta invece il ciclo in
%   cui il residuo e' EFFETTIVAMENTE sceso sotto toll, e serve a verificare
%   che la previsione basata su rho2 regga.
%
%   Su grafo completo la regola di Metropolis da' Q = (1/n)*ones(n) e quindi
%   rho2 = 0: un solo ciclo rende la media esatta, e q vale 1 per qualunque
%   tolleranza. Non e' una scorciatoia scritta a mano, e' il valore che il
%   dimensionamento restituisce da solo.
%
%   PERCHE' NON SERVE CONOSCERE n
%   A convergenza F_i(q) -> (1/n) sum_l F_l(0) e a_i(q) -> (1/n) sum_l a_l(0).
%   Il fattore 1/n compare a numeratore e denominatore e SI CANCELLA:
%
%       x_i = F_i(q)^-1 a_i(q) = ( (1/n) sum F )^-1 ( (1/n) sum a ) = x_LS
%
%   E' una differenza sostanziale rispetto al Filtro di Kalman Distribuito
%   (DKF, Cap. 18 §2), dove la somma globale va ricostruita esplicitamente
%   moltiplicando per n, e ogni nodo deve quindi conoscere a priori la
%   CARDINALITA' della rete. Il D-WLS non ha questo prerequisito.
%
%   PERCHE' NON C'E' DATA RUMINATION
%   Scambiare ripetutamente informazione su un grafo con cicli normalmente
%   produce doppio conteggio (§2.3 del README). Qui non accade, e il motivo e'
%   algebrico: Q e' doppiamente stocastica, quindi le sue COLONNE sommano a
%   uno e la somma totale dell'informazione e' un INVARIANTE del consenso,
%
%       sum_i F_i(k+1) = sum_i sum_j q_ij F_j(k) = sum_j (sum_i q_ij) F_j(k)
%                      = sum_j F_j(k)
%
%   Il consenso ridistribuisce l'informazione fra i nodi senza mai crearne di
%   nuova. La stessa proprieta' e' verificata numericamente in info.inv_somma.
%   La differenza rispetto alla fusione delle pose e' che li' la stima e'
%   ricorsiva nel tempo e rientra nel proprio filtro, mentre qui il consenso
%   riparte a ogni chiamata da un insieme di contributi locali congelati.

    if nargin < 5, toll = 0; end
    [m, ~, n] = size(F_loc);

    % Pesi di consenso: e' qui che la Q di Metropolis entra in un algoritmo, e
    % non piu' come sola diagnostica. La doppia stocasticita' non e' un
    % dettaglio: senza di essa si convergerebbe a una combinazione pesata
    % arbitraria e non alla media, e la stima ricostruita sarebbe polarizzata.
    [Q, rho2] = pesi_metropolis(A);

    % --- Dimensionamento di q (calcolo del progettista) ------------------
    diam = diametro_grafo(A);
    if toll <= 0
        q_teorico = q_max;                  % nessun dimensionamento: q imposto
    elseif rho2 <= 0
        q_teorico = max(1, diam);           % grafo completo: un ciclo e' esatto
    elseif rho2 >= 1
        q_teorico = Inf;                    % grafo sconnesso: non converge mai
    else
        q_teorico = max(ceil(log(toll)/log(rho2)), diam);
    end
    q_eff = max(1, min(q_teorico, q_max));

    % --- Riferimento centralizzato ---------------------------------------
    % Calcolato SOLO per validazione: e' la grandezza che nessun agente puo'
    % costruire a bordo, e serve a misurare quanto il risultato distribuito le
    % si avvicina. Non entra in alcun modo nell'algoritmo.
    F_glob  = sum(F_loc, 3);
    a_glob  = sum(a_loc, 2);
    x_centr = risolvi_sicuro(F_glob, a_glob, m);

    % --- FASE 2: q cicli di consenso sulla media -------------------------
    F = F_loc;
    a = a_loc;
    q_misurato = NaN;                       % primo ciclo con residuo < toll
    residuo    = Inf;
    % Il ciclo di consenso F_i <- sum_j q_ij F_j e' una contrazione sull'indice
    % di nodo, quindi si scrive come un solo prodotto matriciale appiattendo
    % F in (m*m) x n. Equivalente al doppio ciclo su i e j, molto piu' rapido
    % quando q e' dell'ordine delle decine.
    Qt = Q.';
    for it = 1:q_eff
        F_new = reshape(reshape(F, m*m, n) * Qt, m, m, n);
        a_new = a * Qt;
        % Residuo di consenso: quanto i nodi sono ancora in disaccordo fra
        % loro, misurato sullo stato di informazione. E' la grandezza che
        % decade come rho2^q e si annulla quando tutti raggiungono la media.
        residuo = max(vecnorm(a_new - mean(a_new, 2), 2, 1)) / ...
                  max(norm(mean(a_new, 2)), eps);
        F = F_new;
        a = a_new;
        if isnan(q_misurato) && toll > 0 && residuo < toll
            q_misurato = it;
        end
    end

    % --- FASE 3: ricostruzione locale della stima ottima -----------------
    X_est = zeros(m, n);
    X_loc = zeros(m, n);
    cond_loc  = zeros(1, n);
    dev_loc = nan(m, n);
    for i = 1:n
        X_est(:,i) = risolvi_sicuro(F(:,:,i),     a(:,i),     m);
        % Stima che il nodo otterrebbe con la SOLA informazione propria, senza
        % cooperare. E' il termine di paragone che quantifica cosa porta la rete.
        X_loc(:,i) = risolvi_sicuro(F_loc(:,:,i), a_loc(:,i), m);
        cond_loc(i) = numero_condizionamento(F_loc(:,:,i));
        % Incertezza che il nodo dichiarerebbe da solo, F_i(0)^-1. Il confronto
        % con quella di rete misura il guadagno della cooperazione, e a
        % differenza del numero di condizionamento e' monotono: F e' una somma
        % di termini semidefiniti positivi, quindi aggiungere nodi puo' solo
        % farla crescere e ridurre la covarianza (ordinamento di Loewner).
        if rcond(F_loc(:,:,i)) >= 1e-12
            dev_loc(:,i) = sqrt(diag(inv(F_loc(:,:,i))));
        end
    end

    % --- Diagnostica -----------------------------------------------------
    info.x_centr    = x_centr;                      % WLS centralizzato (riferimento)
    info.X_loc      = X_loc;                        % stima senza cooperazione
    info.rho2       = rho2;                         % fattore di convergenza
    info.diametro   = diam;                         % minimo numero di scambi utile
    info.q_teorico  = q_teorico;                    % cicli richiesti da rho2 e diametro
    info.q_eff      = q_eff;                        % cicli effettivamente eseguiti
    info.q_misurato = q_misurato;                   % cicli osservati per scendere sotto toll
    info.residuo    = residuo;                      % disaccordo residuo fra i nodi
    info.budget_ok  = (q_teorico <= q_max);         % il canale radio e' bastato
    info.cond_loc   = cond_loc;                     % condizionamento locale
    info.cond_rete  = numero_condizionamento(F_glob);% condizionamento di rete
    info.dev_loc    = dev_loc;                      % incertezza del nodo da solo
    info.Q          = Q;

    % --- Covarianza dell'errore di stima, Sigma_terr = (sum_i F_i)^-1 ----
    % E' la seconda meta' del risultato del Cap. 18: dice quanto vale la stima
    % appena calcolata. Il testo del corso la chiama P; qui si usa Sigma, che
    % nella notazione di Thrun adottata dal progetto indica la covarianza di
    % una stima (README.md §2.6).
    %
    % A differenza della stima la covarianza NON si ottiene gratis. Il nodo
    % possiede F_i(q) = (1/n) sum_l F_l, quindi deve ricostruirla come
    % (n * F_i(q))^-1: qui il fattore n non si cancella, perche' non c'e' un
    % rapporto fra due quantita' entrambe scalate. Chi voglia dichiarare
    % l'incertezza della propria stima deve dunque conoscere la cardinalita'
    % della rete, esattamente come nel DKF. Il campo Sigma_nodo verifica
    % numericamente questa ricostruzione.
    if rcond(F_glob) < 1e-12
        info.Sigma_terr = NaN(m);
        info.Sigma_nodo = NaN(m);
    else
        info.Sigma_terr = inv(F_glob);
        info.Sigma_nodo = inv(n * F(:,:,1));   % ricostruita dal solo nodo 1
    end
    info.dev_std = sqrt(diag(info.Sigma_terr));   % deviazioni standard dei parametri

    % Scarto fra la stima distribuita e quella centralizzata: e' zero (a meno
    % della precisione di macchina) quando il consenso e' arrivato a regime.
    if any(isnan(x_centr))
        info.scarto = NaN;
    else
        info.scarto = max(vecnorm(X_est - x_centr, 2, 1));
    end
    % Invariante della somma: verifica numerica del fatto che il consenso non
    % crea ne' distrugge informazione.
    info.inv_somma = norm(sum(F, 3) - F_glob, 'fro') / max(norm(F_glob, 'fro'), eps);
end

% -------------------------------------------------------------------------
function x = risolvi_sicuro(F, a, m)
    % La ricostruzione x = F^-1 a richiede F invertibile. Con m parametri e una
    % sola misura scalare per nodo, F_i(0) = C_i' R_i^-1 C_i ha rango 1: il
    % singolo nodo NON puo' risolvere il problema. E' il caso segnalato dal
    % testo del corso ("providing that F_i(t) is invertible") e diventa qui la
    % motivazione fisica dell'algoritmo distribuito, non un caso patologico.
    if rcond(F) < 1e-12 || any(~isfinite(F(:)))
        x = NaN(m, 1);
    else
        x = F \ a;
    end
end

function c = numero_condizionamento(F)
    if any(~isfinite(F(:)))
        c = Inf;
    else
        c = cond(F);
    end
end

function d = diametro_grafo(A)
    % Massima distanza in numero di archi fra due nodi qualsiasi. Fissa il
    % numero MINIMO di cicli di consenso perche' l'informazione di ogni nodo
    % raggiunga tutti gli altri: sotto quella soglia il risultato e' privo di
    % senso, non semplicemente impreciso. Su grafo sconnesso si restituisce n,
    % che il chiamante interpreta insieme a rho2 = 1.
    n = size(A, 1);
    D = inf(n);
    D(logical(eye(n))) = 0;
    D(A > 0) = 1;
    for k = 1:n                             % Floyd-Warshall
        D = min(D, D(:,k) + D(k,:));
    end
    d = max(D(:));
    if ~isfinite(d), d = n; end
end
