function Q_d = calcola_Q_cwna(theta, v, Ts, par)
%CALCOLA_Q_CWNA Rumore di processo discreto per l'uniciclo a stato esteso.
%
%   Q_d = CALCOLA_Q_CWNA(theta, v, Ts, par) restituisce la matrice 5x5 di
%   covarianza del rumore di processo per lo stato [x; y; theta; v; omega],
%   discretizzata su un passo Ts secondo il modello CWNA (Continuous White
%   Noise Acceleration).
%
%   PERCHE' NON UNA DIAGONALE COSTANTE
%   Nel modello uniciclo la posizione non ha dinamica propria: cambia soltanto
%   perche' v e theta sono incerti. Iniettare rumore direttamente su x e y
%   equivale ad affermare che il veicolo si teletrasporta lateralmente a
%   velocita' nulla. Il rumore deve entrare dove sta l'incertezza fisica reale
%   — sulle ACCELERAZIONI, che e' esattamente dove agisce lo slittamento — e
%   propagarsi alla posizione attraverso il modello. Il risultato NON e'
%   diagonale: contiene le correlazioni posizione-velocita', che sono
%   informazione fisica ("se sovrastimo la velocita', siamo anche troppo
%   avanti") che una Q diagonale butta via.
%
%   FORMULAZIONE
%   Modello continuo:  xdot = f(x) + Gamma*w(t),   w ~ N(0, Q_c)
%   con Gamma che seleziona i soli canali di accelerazione. La discretizzazione
%   esatta e'
%       Q_d = int_0^Ts  expm(A*tau) * Gamma*Q_c*Gamma' * expm(A'*tau)  d(tau)
%   Congelando theta sull'intervallo — lecito, perche' con Ts = 0.1 s e
%   omega <= 0.6 rad/s l'angolo varia al massimo di 3.4 gradi — l'integrale si
%   risolve in forma chiusa e restituisce i blocchi canonici del modello a
%   velocita' costante:
%       q * [ Ts^3/3   Ts^2/2 ;
%             Ts^2/2   Ts     ]
%   (Bar-Shalom, Li, Kirubarajan, "Estimation with Applications to Tracking and
%   Navigation", cap. 6). La correttezza dell'approssimazione e' verificata
%   numericamente contro il metodo di Van Loan in verifica_Q_cwna.m.
%
%   TRE CANALI DI RUMORE, CIASCUNO CON SIGNIFICATO FISICO
%     par.q_a       [m^2/s^3]   accelerazione LONGITUDINALE. Slittamento in
%                               trazione: il cingolo gira ma il mezzo non
%                               avanza di conseguenza. Alimenta la posizione
%                               lungo la direzione di marcia e la velocita' v.
%     par.q_alpha   [rad^2/s^3] accelerazione ANGOLARE. Slittamento in
%                               sterzata. Alimenta theta e omega.
%     par.q_lat     [m^2/s]     velocita' LATERALE. Deriva di traverso, tipica
%                               su pendio innevato. Serve un canale separato
%                               perche' il modello uniciclo e' anolonomo e non
%                               PUO' rappresentare la traslazione laterale:
%                               senza questo termine Q_d sarebbe SINGOLARE e
%                               l'incertezza perpendicolare alla marcia non
%                               crescerebbe mai, per quanto a lungo il filtro
%                               resti senza misure assolute.
%     par.k_terreno [1/s]       dipendenza dalla velocita': q_a viene
%                               maggiorato di k_terreno*v^2. Piu' il mezzo
%                               corre su fondo cedevole, piu' incertezza viene
%                               iniettata. Con k_terreno = 0 il termine e'
%                               inattivo; viene calibrato in Fase 5, quando
%                               l'impianto simulera' uno slittamento reale
%                               contro cui tararlo.
%
%   NOTA IMPORTANTE: Q_d dipende da theta (e, se k_terreno > 0, da v). Va
%   quindi ricalcolata a OGNI passo di predizione, non una volta sola in fase
%   di setup. E' inoltre proporzionale a Ts: cambiare la frequenza di
%   campionamento non altera piu' silenziosamente la taratura del filtro, che
%   e' il prerequisito per il funzionamento multi-rate della Fase 4.

    T1 = Ts;
    T2 = Ts^2 / 2;
    T3 = Ts^3 / 3;

    c = cos(theta);
    s = sin(theta);

    % Densita' spettrale longitudinale, adattiva sulla velocita'
    q_a = par.q_a + par.k_terreno * v^2;

    Q_d = zeros(5,5);

    % --- Canale LONGITUDINALE: accelerazione -> (posizione di marcia, v) -----
    % Blocco a velocita' costante [T3 T2; T2 T1], proiettato sulla direzione
    % di marcia [cos(theta); sin(theta)].
    Q_d(1,1) = q_a * T3 * c*c;
    Q_d(1,2) = q_a * T3 * c*s;
    Q_d(2,2) = q_a * T3 * s*s;
    Q_d(1,4) = q_a * T2 * c;      % <-- correlazione posizione/velocita'
    Q_d(2,4) = q_a * T2 * s;
    Q_d(4,4) = q_a * T1;

    % --- Canale LATERALE: deriva -> posizione perpendicolare alla marcia -----
    % Versore perpendicolare: [-sin(theta); cos(theta)].
    Q_d(1,1) = Q_d(1,1) + par.q_lat * T1 * s*s;
    Q_d(1,2) = Q_d(1,2) - par.q_lat * T1 * c*s;
    Q_d(2,2) = Q_d(2,2) + par.q_lat * T1 * c*c;

    % --- Canale ANGOLARE: accelerazione angolare -> (theta, omega) -----------
    Q_d(3,3) = par.q_alpha * T3;
    Q_d(3,5) = par.q_alpha * T2;
    Q_d(5,5) = par.q_alpha * T1;

    % Simmetrizzazione (finora e' stato riempito il solo triangolo superiore)
    Q_d(2,1) = Q_d(1,2);
    Q_d(4,1) = Q_d(1,4);
    Q_d(4,2) = Q_d(2,4);
    Q_d(5,3) = Q_d(3,5);
end
