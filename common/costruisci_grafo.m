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
%     G.lambda  autovalori di L, ordinati in modo crescente.
%     G.lambda2 CONNETTIVITA' ALGEBRICA (secondo autovalore piu' piccolo).
%     G.connesso true se il grafo e' connesso.
%     G.n_archi  numero di archi non orientati attivi.

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

    % Autovalori ordinati. Per grafo non orientato L e' simmetrica e
    % semidefinita positiva, quindi gli autovalori sono reali e non negativi;
    % la parte reale rende comunque robusto il calcolo a errori numerici.
    lambda = sort(real(eig(L)));

    G.A        = A;
    G.D        = D;
    G.L        = L;
    G.lambda   = lambda;
    G.lambda2  = lambda(min(2, n));
    G.connesso = G.lambda2 > 1e-9;
    G.n_archi  = nnz(A) / 2;
end
