function G = costruisci_grafo(p, R_c)

%   G = COSTRUISCI_GRAFO(p, R_c) costruisce le matrici associate al grafo di
%   comunicazione a partire dalle posizioni degli agenti p (2 x n) e dal raggio
%   di comunicazione R_c. Con R_c = Inf si ottiene il grafo completo (rete
%   full-mesh ideale).
%
%   CAMPI RESTITUITI
%     G.A       matrice di ADIACENZA (n x n). A(i,j) = 1 se l'agente j
%               trasmette all'agente i, 0 altrimenti. Diagonale nulla.
%               Qui il grafo e' NON ORIENTATO (A simmetrica): il ranging UWB
%               e' bidirezionale, se i sente j allora j sente i.
%     G.D       matrice di GRADO, diag(d_i) con d_i = sum_j A(i,j).
%     G.L       matrice LAPLACIANA, L = D - A.
%     G.lambda_L        autovalori di L, ordinati in modo CRESCENTE.
%     G.lambda1_L       il primo, sempre nullo: L*1 = 0.
%     G.lambda2_L       CONNETTIVITA' ALGEBRICA, secondo autovalore.
%     G.mol_lambda1_L   molteplicita' di lambda_1(L) = numero di componenti
%                       connesse. Vale 1 se e solo se il grafo e' connesso.
%     G.connesso        true se G.mol_lambda1_L == 1, cioe' se lambda_2(L) > 0.
%     G.n_archi         numero di archi non orientati attivi.
%
%   CONVENZIONE SUGLI AUTOVALORI
%   Il progetto calcola due matrici sullo stesso grafo e ne legge gli spettri
%   separatamente, quindi ogni autovalore porta sempre l'indicazione della
%   matrice da cui proviene:
%     lambda_i(L)       autovalori del Laplaciano, crescenti, tutti >= 0.
%                       Governano la dinamica del CONTROLLO DI FORMAZIONE
%                       (tempo continuo), con tau = 1/(K_cons*lambda_2(L)).
%     lambda_i(Q)       autovalori dei pesi di Metropolis, decrescenti, in
%                       (-1, 1]. Governano il D-WLS (tempo discreto).
%   Le due letture si muovono in verso opposto: rete ben collegata significa
%   lambda_2(L) grande e rho_2 piccolo. Vedi theory/TEORIA_consenso_su_grafi.md.

    n = size(p, 2);

    % --- Matrice di adiacenza da vincolo di portata radio ---
    A = zeros(n);
    for i = 1:n
        for j = 1:n
            if i ~= j && norm(p(:,i) - p(:,j)) <= R_c
                A(i,j) = 1;
            end
        end
    end

    D = diag(sum(A, 2));    % grado in ingresso di ciascun nodo
    L = D - A;              % Laplaciano

    % Per grafo non orientato L e' simmetrica e semidefinita positiva, quindi
    % gli autovalori sono reali e non negativi. La parte reale rende comunque
    % robusto il calcolo agli errori di arrotondamento.
    lambda_L = sort(real(eig(L)));
    % L e' semidefinita positiva per costruzione: eventuali valori negativi
    % sono solo arrotondamento, e si azzerano per non stampare "-0.0000".
    lambda_L(abs(lambda_L) < 1e-12) = 0;

    G.A              = A;
    G.D              = D;
    G.L              = L;
    G.lambda_L       = lambda_L;
    G.lambda1_L      = lambda_L(1);
    G.lambda2_L      = lambda_L(min(2, n));
    G.mol_lambda1_L  = sum(abs(lambda_L) < 1e-9);
    G.connesso       = G.mol_lambda1_L == 1;
    G.n_archi        = nnz(A) / 2;
end
