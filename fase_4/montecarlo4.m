function montecarlo4(N_run)
%MONTECARLO4  Campagna Monte Carlo di validazione sulla Fase 4.
%
%   MONTECARLO4()      esegue 50 ripetizioni.
%   MONTECARLO4(n)     ne esegue n.
%
%   PERCHE' SERVE
%   Tutti i risultati riportati finora vengono da una singola esecuzione. Un
%   run solo non distingue una proprieta' del sistema da una fluttuazione del
%   rumore: la stima del terreno, per esempio, cambia in modo vistoso da un run
%   all'altro. La campagna ripete la stessa configurazione con semi diversi e
%   riporta media e dispersione, che e' l'unica forma in cui i numeri reggono
%   un confronto.
%
%   COME FUNZIONA
%   Non duplica la simulazione: lancia main4.m in modalita' batch. Lo script,
%   se trova MODO_BATCH = true, salta la generazione delle figure e non azzera
%   il workspace del chiamante, lasciando quindi tutte le sue variabili
%   leggibili da qui. La sorgente della simulazione resta una sola, e la
%   campagna non puo' divergere dal codice che produce le figure.
%
%   Il seme del generatore e' fissato al numero della ripetizione, quindi
%   l'intera campagna e' riproducibile.
%
%   COSA PRODUCE
%     montecarlo_fase4.mat        struttura completa, un elemento per run, piu'
%                                 le statistiche aggregate e i metadati
%     risultati/9_montecarlo_convergenza.png
%     risultati/10_montecarlo_consistenza.png
%     un riepilogo a console in forma di tabelle, gia' impaginabili nel report
%
%   Il file .mat e' pensato come riferimento riutilizzabile: le stesse
%   grandezze estratte qui saranno confrontate con quelle della Fase 5, dove
%   l'aggiornamento collaborativo passa alla Covariance Intersection.

%#ok<*USENS>   le variabili lette qui sotto sono definite da main4.m, che viene
%              eseguito come script dentro questa funzione: l'analizzatore
%              statico non puo' vederne le assegnazioni.

if nargin < 1, N_run = 50; end

MC_cartella = fileparts(mfilename('fullpath'));
MC_uscita   = fullfile(MC_cartella, 'risultati');
if ~exist(MC_uscita, 'dir'), mkdir(MC_uscita); end

% GRIGLIA DI TEMPO NORMALIZZATO
% le missioni non durano tutte uguale, perche' la durata dipende dall'errore di
% inseguimento e quindi dal rumore. Le serie temporali vengono percio' riportate
% su un asse normalizzato in [0, 1] prima di essere mediate fra i run.
MC_ngriglia = 400;
MC_tau      = linspace(0, 1, MC_ngriglia);

MC_ris = struct([]);
MC_t0  = tic;

fprintf('=== CAMPAGNA MONTE CARLO, FASE 4 ===\n');
fprintf('Ripetizioni richieste: %d\n\n', N_run);

for MC_r = 1:N_run

    rng(MC_r);                     % riproducibilita' della singola ripetizione
    MODO_BATCH = true;             %#ok<NASGU>  letta da main4.m
    main4;                         % simulazione completa, senza figure

    %% ESTRAZIONE DELLE GRANDEZZE DI INTERESSE
    kk = 1:(N_end-1);              % stessa finestra usata dal riepilogo di main4

    % COPERTURA SATELLITARE
    % tre condizioni mutuamente esclusive: tutti coperti, copertura mista,
    % nessuno coperto. La fascia mista e' quella in cui la cooperazione conta.
    cop_tutti   = true(1, N_end);
    cop_nessuno = true(1, N_end);
    for i = 1:N_veh
        cop_tutti   = cop_tutti   & ~fleet(i).in_denied_hist(1:N_end);
        cop_nessuno = cop_nessuno &  fleet(i).in_denied_hist(1:N_end);
    end
    MC_ris(MC_r).cop_tutti   = 100*mean(cop_tutti);
    MC_ris(MC_r).cop_mista   = 100*mean(~cop_tutti & ~cop_nessuno);
    MC_ris(MC_r).cop_nessuno = 100*mean(cop_nessuno);

    % ACCURATEZZA E CONSISTENZA, VEICOLO PER VEICOLO
    err_pos_run = zeros(N_veh, N_end);
    tr_pos_run  = zeros(N_veh, N_end);
    nees_run    = zeros(N_veh, N_end);
    for i = 1:N_veh
        e_xy = fleet(i).x_true(1:2, 1:N_end) - fleet(i).x_est(1:2, 1:N_end);
        err_pos_run(i,:) = vecnorm(e_xy);
        S11 = squeeze(fleet(i).Sigma_hist(1,1,1:N_end))';
        S22 = squeeze(fleet(i).Sigma_hist(2,2,1:N_end))';
        tr_pos_run(i,:)  = S11 + S22;

        % NEES DI POSIZIONE: e' * Sigma_pos^-1 * e, due gradi di liberta'.
        % Il valore atteso e' 2. Sopra 2 il filtro dichiara meno incertezza di
        % quanta ne abbia (ottimista), sotto e' conservativo.
        for k = 1:N_end
            Sp = fleet(i).Sigma_hist(1:2,1:2,k);
            nees_run(i,k) = e_xy(:,k)' * (Sp \ e_xy(:,k));
        end

        cieco = fleet(i).in_denied_hist(1:N_end);
        MC_ris(MC_r).mae_gps(i)    = mean(err_pos_run(i, ~cieco));
        MC_ris(MC_r).mae_cieca(i)  = mean(err_pos_run(i,  cieco));
        MC_ris(MC_r).rmse_tot(i)   = sqrt(mean(err_pos_run(i,:).^2));
        MC_ris(MC_r).trS_gps(i)    = mean(tr_pos_run(i, ~cieco));
        MC_ris(MC_r).trS_cieca(i)  = mean(tr_pos_run(i,  cieco));
        MC_ris(MC_r).nees_medio(i) = mean(nees_run(i,:));

        % NEES SEPARATO PER CONDIZIONE DI COPERTURA
        % le due condizioni sollecitano rami diversi del filtro: a cielo aperto
        % correggono GNSS, AHRS ed encoder, tutti indipendenti fra loro; in zona
        % cieca entra anche il ranging verso i vicini, la cui incertezza non e'
        % modellata. Separare il NEES e' l'unico modo per vedere se il secondo
        % ramo costa in consistenza.
        MC_ris(MC_r).nees_gps(i)   = mean(nees_run(i, ~cieco));
        MC_ris(MC_r).nees_cieca(i) = mean(nees_run(i,  cieco));

        % FRAZIONE FUORI BANDA A 3 SIGMA, sulle due componenti di posizione.
        % Per un filtro esattamente calibrato ci si attende lo 0.27%.
        fuori = abs(e_xy(1,:)) > 3*sqrt(S11) | abs(e_xy(2,:)) > 3*sqrt(S22);
        MC_ris(MC_r).fuori_3s(i) = 100*mean(fuori);
    end

    % CANALE DI COMUNICAZIONE
    eta = eta_pacchetto(:,:,kk);
    eta = eta(~repmat(logical(eye(N_veh)), 1, 1, numel(kk)));   % esclude la diagonale
    tau_lim = pi / (2*K_cons*mean(lambda_max_L_hist(kk)));      % Olfati-Saber e Murray
    MC_ris(MC_r).persi_perc  = 100*n_persi/max(n_inviati,1);
    MC_ris(MC_r).eta_media   = 1e3*mean(eta)*Ts;               % [ms]
    MC_ris(MC_r).eta_max     = 1e3*max(eta)*Ts;                % [ms]
    MC_ris(MC_r).margine_med = 100*mean(eta)*Ts/tau_lim;       % [%] del limite
    MC_ris(MC_r).margine_max = 100*max(eta)*Ts/tau_lim;        % [%] del limite
    MC_ris(MC_r).tau_lim     = 1e3*tau_lim;                    % [ms]

    % GRAFO DI COMUNICAZIONE
    MC_ris(MC_r).lambda2_L = mean(lambda2_L_hist(kk));
    MC_ris(MC_r).rho2      = mean(rho2_hist(kk));
    MC_ris(MC_r).n_archi   = mean(n_archi_hist(kk));
    MC_ris(MC_r).connesso  = all(lambda2_L_hist(kk) > 1e-9);
    MC_ris(MC_r).diametro  = max(dwls.diam(1:n_round), [], 'omitnan');
    MC_ris(MC_r).q_dimens  = max(dwls.q_eff(1:n_round), [], 'omitnan');
    MC_ris(MC_r).q_osserv  = max(dwls.q_mis(1:n_round), [], 'omitnan');

    % STIMA DISTRIBUITA DEL TERRENO
    % L'errore relativo e' calcolato sul VETTORE dei due parametri: preso da
    % solo, c_terr ha un errore relativo molto piu' grande, perche' e' il
    % parametro meno eccitato dai dati.
    x_fin = dwls.X(:, 1, n_round);
    MC_ris(MC_r).mu_terr   = x_fin(1);
    MC_ris(MC_r).c_terr    = x_fin(2);
    MC_ris(MC_r).err_mu    = 100*abs(x_fin(1) - x_terr_true(1))/abs(x_terr_true(1));
    MC_ris(MC_r).err_c     = 100*abs(x_fin(2) - x_terr_true(2))/abs(x_terr_true(2));
    MC_ris(MC_r).err_vett  = 100*norm(x_fin - x_terr_true(:))/norm(x_terr_true);
    MC_ris(MC_r).dev_mu    = dwls.dev_std(1, n_round);
    MC_ris(MC_r).dev_c     = dwls.dev_std(2, n_round);
    MC_ris(MC_r).scarto_c  = dwls.scarto(n_round);          % D-WLS contro centralizzato
    MC_ris(MC_r).residuo   = max(dwls.residuo(1:n_round), [], 'omitnan');
    MC_ris(MC_r).invar_F   = max(dwls.inv_somma(1:n_round), [], 'omitnan');
    for i = 1:N_veh
        MC_ris(MC_r).dev_c_solo(i) = dwls.dev_loc(2, i, n_round);
        MC_ris(MC_r).guadagno(i)   = dwls.dev_loc(2, i, n_round) / dwls.dev_std(2, n_round);
    end

    % ECCITAZIONE DEL REGRESSORE
    % La riga di regressione del D-WLS e' [1, v^2]: se v^2 resta quasi costante
    % le due colonne diventano collineari e il problema si mal condiziona. La
    % dispersione di v^2 e' quindi la grandezza che spiega, run per run, quanto
    % bene c_terr risulti identificabile, e viene registrata per poterla
    % correlare con l'errore commesso.
    for i = 1:N_veh
        MC_ris(MC_r).std_v2(i) = std(fleet(i).x_true(4, 1:N_end).^2);
    end
    MC_ris(MC_r).cond_rete = dwls.cond_rete(n_round);

    % MISSIONE
    MC_ris(MC_r).durata = t(N_end);
    v_med = zeros(1, N_veh);
    for i = 1:N_veh, v_med(i) = mean(fleet(i).x_true(4, 1:N_end)); end
    MC_ris(MC_r).v_flotta = mean(v_med);

    % SERIE TEMPORALI SU ASSE NORMALIZZATO, per le medie fra run
    tau_run = linspace(0, 1, N_end);
    for i = 1:N_veh
        MC_ris(MC_r).curva_err(i,:)  = interp1(tau_run, err_pos_run(i,:), MC_tau);
        MC_ris(MC_r).curva_trS(i,:)  = interp1(tau_run, tr_pos_run(i,:),  MC_tau);
        MC_ris(MC_r).curva_nees(i,:) = interp1(tau_run, nees_run(i,:),    MC_tau);
    end
    err_dwls_run = nan(1, n_round);
    for k = 1:n_round
        err_dwls_run(k) = 100*norm(dwls.X(:,1,k) - x_terr_true(:))/norm(x_terr_true);
    end
    MC_ris(MC_r).curva_dwls = interp1(linspace(0,1,n_round), err_dwls_run, MC_tau);
    mask_den = false(N_veh, N_end);
    for i = 1:N_veh, mask_den(i,:) = fleet(i).in_denied_hist(1:N_end); end
    MC_ris(MC_r).curva_denied = interp1(tau_run, mean(mask_den, 1), MC_tau);

    % AVANZAMENTO
    MC_trascorso = toc(MC_t0);
    fprintf(['[%2d/%2d] %5.1f s | missione %6.1f s | MAE cieca %.3f m | ' ...
             'NEES %.2f | c_terr %.5f | ETA %s\n'], ...
            MC_r, N_run, MC_trascorso, MC_ris(MC_r).durata, ...
            mean(MC_ris(MC_r).mae_cieca), mean(MC_ris(MC_r).nees_medio), ...
            MC_ris(MC_r).c_terr, ...
            durata_leggibile(MC_trascorso/MC_r*(N_run-MC_r)));

    % SALVATAGGIO PROGRESSIVO: una campagna da mezz'ora non deve andare persa
    % per un'interruzione all'ultima ripetizione.
    if mod(MC_r, 10) == 0 || MC_r == N_run
        save(fullfile(MC_cartella, 'montecarlo_fase4.mat'), 'MC_ris', 'MC_tau', 'MC_r');
    end
end

%% AGGREGAZIONE E RIEPILOGO
MC_stat = aggrega(MC_ris, N_veh);
MC_meta = struct('data', datetime('now'), 'n_run', N_run, 'semi', 1:N_run, ...
                 'N_veh', N_veh, 'r_collab', R_c_comm, 'K_cons', K_cons, ...
                 'Ts', Ts, 'x_terr_true', x_terr_true(:), ...
                 'fase', 'Fase 4 (nessuna Covariance Intersection)');
save(fullfile(MC_cartella, 'montecarlo_fase4.mat'), ...
     'MC_ris', 'MC_stat', 'MC_meta', 'MC_tau');

stampa_riepilogo(MC_ris, MC_stat, N_run, N_veh, x_terr_true);
disegna_figure(MC_ris, MC_tau, N_run, N_veh, MC_uscita, x_terr_true(2));

fprintf('\nDati salvati in %s\n', fullfile(MC_cartella, 'montecarlo_fase4.mat'));
fprintf('Durata totale della campagna: %s\n', durata_leggibile(toc(MC_t0)));
end

%=========================================================================
% FUNZIONI LOCALI
%=========================================================================

function S = aggrega(R, N_veh)
% Media e deviazione standard campionaria di ogni grandezza scalare o
% vettoriale, calcolate fra le ripetizioni.
campi = fieldnames(R);
for c = 1:numel(campi)
    nome = campi{c};
    if startsWith(nome, 'curva'), continue; end        % le serie si mediano a parte
    M = cat(1, R.(nome));
    if ~isnumeric(M) && ~islogical(M), continue; end
    S.([nome '_med']) = mean(double(M), 1);
    S.([nome '_dev']) = std(double(M), 0, 1);
    S.([nome '_min']) = min(double(M), [], 1);
    S.([nome '_max']) = max(double(M), [], 1);
end
% Curve medie fra i run, con la dispersione a un sigma
for nome = {'curva_err', 'curva_trS', 'curva_nees'}
    A = cat(3, R.(nome{1}));                            % N_veh x griglia x run
    S.([nome{1} '_med']) = mean(A, 3);
    S.([nome{1} '_dev']) = std(A, 0, 3);
end
for nome = {'curva_dwls', 'curva_denied'}
    A = cat(1, R.(nome{1}));
    S.([nome{1} '_med']) = mean(A, 1);
    S.([nome{1} '_dev']) = std(A, 0, 1);
end
S.N_veh = N_veh;
end

%-------------------------------------------------------------------------
function [lo, hi] = bande_nees(M, dof)
% Bande di accettazione a due code al 95% per il NEES mediato su M campagne
% indipendenti: M*NEES segue una chi quadro con M*dof gradi di liberta'.
% I quantili sono ottenuti con l'approssimazione di Wilson-Hilferty, che evita
% la dipendenza dallo Statistics Toolbox ed e' accurata a meno di 1e-3 per
% gradi di liberta' superiori a qualche decina.
k  = M * dof;
z  = 1.959963984540054;                 % quantile normale a 0.975
lo = k * (1 - 2/(9*k) - z*sqrt(2/(9*k)))^3 / M;
hi = k * (1 - 2/(9*k) + z*sqrt(2/(9*k)))^3 / M;
end

%-------------------------------------------------------------------------
function r = corr_pearson(x, y)
% Coefficiente di correlazione lineare, senza dipendere dallo Statistics Toolbox.
x = x(:) - mean(x(:));
y = y(:) - mean(y(:));
r = (x'*y) / max(sqrt((x'*x)*(y'*y)), eps);
end

%-------------------------------------------------------------------------
function s = durata_leggibile(secondi)
if secondi < 60
    s = sprintf('%.0f s', secondi);
else
    s = sprintf('%d min %02d s', floor(secondi/60), round(mod(secondi,60)));
end
end

%-------------------------------------------------------------------------
function stampa_riepilogo(R, S, N_run, N_veh, x_terr_true)
% Tabelle a console nella stessa forma in cui vanno nel report: valore medio
% seguito dalla deviazione standard fra le ripetizioni.

ruoli = repmat({'Slave '}, 1, N_veh);
ruoli{1} = 'Master';

fprintf('\n\n========================================================\n');
fprintf('  RIEPILOGO DELLA CAMPAGNA (%d ripetizioni)\n', N_run);
fprintf('========================================================\n');

fprintf('\n--- ACCURATEZZA DELLA STIMA DI POSA ---\n');
fprintf('%-16s %16s %16s %16s\n', 'Veicolo', 'MAE con GPS [m]', ...
        'MAE cieca [m]', 'RMSE totale [m]');
for i = 1:N_veh
    fprintf('V%d (%s)     %8.3f +- %.3f %8.3f +- %.3f %8.3f +- %.3f\n', i, ruoli{i}, ...
            S.mae_gps_med(i),   S.mae_gps_dev(i), ...
            S.mae_cieca_med(i), S.mae_cieca_dev(i), ...
            S.rmse_tot_med(i),  S.rmse_tot_dev(i));
end
fprintf('Rapporto MAE (cielo aperto / zona cieca), medio sugli Slave: %.1fx\n', ...
        mean(S.mae_gps_med(2:end)) / mean(S.mae_cieca_med(2:end)));

fprintf('\n--- COVARIANZA DICHIARATA: tr(Sigma_pos) [m^2] ---\n');
fprintf('%-16s %20s %20s %10s\n', 'Veicolo', 'con GPS', 'in zona cieca', 'rapporto');
for i = 1:N_veh
    fprintf('V%d (%s)   %10.4f +- %.4f %10.4f +- %.4f %8.1fx\n', i, ruoli{i}, ...
            S.trS_gps_med(i),   S.trS_gps_dev(i), ...
            S.trS_cieca_med(i), S.trS_cieca_dev(i), ...
            S.trS_gps_med(i)/S.trS_cieca_med(i));
end

fprintf('\n--- CONSISTENZA DEL FILTRO ---\n');
[lo, hi] = bande_nees(N_run, 2);
fprintf('NEES di posizione, atteso 2.00, banda 95%% su %d run: [%.2f, %.2f]\n', ...
        N_run, lo, hi);
for i = 1:N_veh
    esito = 'conservativo';
    if S.nees_medio_med(i) > hi, esito = 'OTTIMISTA'; end
    if S.nees_medio_med(i) >= lo && S.nees_medio_med(i) <= hi, esito = 'calibrato'; end
    fprintf('   V%d (%s)  NEES = %.2f +- %.2f   -> %s\n', i, ruoli{i}, ...
            S.nees_medio_med(i), S.nees_medio_dev(i), esito);
end
fprintf('NEES separato per condizione di copertura:\n');
for i = 1:N_veh
    fprintf('   V%d (%s)  con GPS %.2f +- %.2f   in zona cieca %.2f +- %.2f\n', ...
            i, ruoli{i}, S.nees_gps_med(i), S.nees_gps_dev(i), ...
            S.nees_cieca_med(i), S.nees_cieca_dev(i));
end
fprintf('   media sulla flotta: con GPS %.2f, in zona cieca %.2f\n', ...
        mean(S.nees_gps_med), mean(S.nees_cieca_med));
fprintf('Campioni fuori dalla banda 3 sigma: %.2f%% in media (atteso 0.27%%)\n', ...
        mean(S.fuori_3s_med));

fprintf('\n--- COPERTURA SATELLITARE ---\n');
fprintf('%-28s %.1f%% +- %.1f\n', 'Tutti i veicoli coperti', S.cop_tutti_med, S.cop_tutti_dev);
fprintf('%-28s %.1f%% +- %.1f\n', 'Copertura mista', S.cop_mista_med, S.cop_mista_dev);
fprintf('%-28s %.1f%% +- %.1f\n', 'Nessun veicolo coperto', S.cop_nessuno_med, S.cop_nessuno_dev);

fprintf('\n--- CANALE DI COMUNICAZIONE ---\n');
fprintf('%-32s %.2f%% +- %.2f\n', 'Pacchetti persi', S.persi_perc_med, S.persi_perc_dev);
fprintf('%-32s %.0f +- %.0f ms\n', 'Eta media del dato', S.eta_media_med, S.eta_media_dev);
fprintf('%-32s %.0f +- %.0f ms  (max osservato %.0f)\n', 'Eta massima del dato', ...
        S.eta_max_med, S.eta_max_dev, S.eta_max_max);
fprintf('%-32s %.0f%% medio, %.0f%% nel caso peggiore\n', 'Margine di stabilita consumato', ...
        S.margine_med_med, S.margine_max_max);
fprintf('%-32s %.0f ms\n', 'Limite teorico sul ritardo', S.tau_lim_med);

fprintf('\n--- GRAFO DI COMUNICAZIONE ---\n');
fprintf('%-32s %.4f +- %.4f\n', 'Connettivita lambda_2(L)', S.lambda2_L_med, S.lambda2_L_dev);
fprintf('%-32s %.4f +- %.4f\n', 'Raggio spettrale rho_2', S.rho2_med, S.rho2_dev);
fprintf('%-32s %.2f +- %.2f\n', 'Archi attivi', S.n_archi_med, S.n_archi_dev);
fprintf('%-32s %d\n', 'Diametro (massimo sui run)', round(S.diametro_max));
fprintf('%-32s %d dimensionati, %d osservati\n', 'Cicli di consenso q', ...
        round(S.q_dimens_med), round(S.q_osserv_med));
fprintf('%-32s %s\n', 'Grafo connesso in tutti i run', ...
        string(all(cat(1, R.connesso))));

fprintf('\n--- STIMA DISTRIBUITA DEL TERRENO ---\n');
fprintf('Valori veri: mu_terr = %.5f, c_terr = %.6f\n', x_terr_true(1), x_terr_true(2));
fprintf('%-24s %.5f +- %.5f   (errore %.2f%% +- %.2f)\n', 'mu_terr stimato', ...
        S.mu_terr_med, S.mu_terr_dev, S.err_mu_med, S.err_mu_dev);
fprintf('%-24s %.6f +- %.6f   (errore %.1f%% +- %.1f)\n', 'c_terr stimato', ...
        S.c_terr_med, S.c_terr_dev, S.err_c_med, S.err_c_dev);
fprintf('%-24s %.2f%% +- %.2f\n', 'Errore sul vettore', S.err_vett_med, S.err_vett_dev);
fprintf('%-24s dichiarata %.6f, osservata %.6f  -> rapporto %.2f\n', 'Dev.std su c_terr', ...
        S.dev_c_med, S.c_terr_dev, S.c_terr_dev/S.dev_c_med);
fprintf('%-24s %.2e\n', 'Scarto dal centralizzato', S.scarto_c_med);
fprintf('%-24s %.2e\n', 'Disaccordo fra i nodi', S.residuo_med);
fprintf('%-24s %.2e\n', 'Invarianza di sum(F_i)', S.invar_F_med);
fprintf('%-24s %.1f +- %.1f  (da %.0f a %.0f)\n', 'Condizionamento di rete', ...
        S.cond_rete_med, S.cond_rete_dev, S.cond_rete_min, S.cond_rete_max);
fprintf('Eccitazione del regressore, std(v^2) per veicolo:\n');
for i = 1:N_veh
    fprintf('   V%d (%s)  %.3f +- %.3f  (da %.3f a %.3f)\n', i, ruoli{i}, ...
            S.std_v2_med(i), S.std_v2_dev(i), S.std_v2_min(i), S.std_v2_max(i));
end
rho_sp = corr_pearson(cat(1, R.std_v2)*[1;0;0;0;0], cat(1, R.err_c));
fprintf('Correlazione fra std(v^2) del Master e errore su c_terr: %+.2f\n', rho_sp);
fprintf('Guadagno della cooperazione su dev(c_terr):\n');
for i = 1:N_veh
    fprintf('   V%d (%s)  da solo %.2e   in rete %.2e   -> %.1fx +- %.1f\n', ...
            i, ruoli{i}, S.dev_c_solo_med(i), S.dev_c_med, ...
            S.guadagno_med(i), S.guadagno_dev(i));
end

fprintf('\n--- MISSIONE ---\n');
fprintf('%-24s %.1f +- %.1f s  (da %.1f a %.1f)\n', 'Durata', ...
        S.durata_med, S.durata_dev, S.durata_min, S.durata_max);
fprintf('%-24s %.3f +- %.3f m/s\n', 'Velocita media flotta', ...
        S.v_flotta_med, S.v_flotta_dev);
end

%-------------------------------------------------------------------------
function disegna_figure(R, tau, N_run, N_veh, cartella, c_vero)
% Due figure pensate per il report: la prima mostra che il comportamento medio
% non e' un caso fortunato del singolo run, la seconda che il filtro e'
% consistente su tutta la campagna.

colori = lines(N_veh);
S = aggrega(R, N_veh);

% FIGURA 9: convergenza media su tempo normalizzato
fig9 = figure('Name','Monte Carlo: convergenza','Color','w', ...
              'Position',[100 100 1000 700]);

subplot(3,1,1); hold on; grid on;
for i = 1:N_veh
    m = S.curva_trS_med(i,:); d = S.curva_trS_dev(i,:);
    fill([tau fliplr(tau)], [max(m-d,1e-4) fliplr(m+d)], colori(i,:), ...
         'FaceAlpha', 0.12, 'EdgeColor','none', 'HandleVisibility','off');
    plot(tau, m, 'Color', colori(i,:), 'LineWidth', 1.4, ...
         'DisplayName', sprintf('V%d', i));
end
set(gca,'YScale','log'); ylabel('tr(\Sigma_{pos}) [m^2]');
title(sprintf('Media su %d ripetizioni, banda a 1 sigma', N_run));
legend('Location','best','FontSize',7);

subplot(3,1,2); hold on; grid on;
for i = 1:N_veh
    plot(tau, S.curva_err_med(i,:), 'Color', colori(i,:), 'LineWidth', 1.4, ...
         'DisplayName', sprintf('V%d', i));
end
ylabel('Errore di posizione [m]'); legend('Location','best','FontSize',7);
title('Errore medio di posizione');

subplot(3,1,3); hold on; grid on;
m = S.curva_dwls_med; d = S.curva_dwls_dev;
fill([tau fliplr(tau)], [max(m-d,1e-3) fliplr(m+d)], [0.2 0.4 0.8], ...
     'FaceAlpha', 0.15, 'EdgeColor','none');
plot(tau, m, 'Color', [0.1 0.2 0.6], 'LineWidth', 1.6);
set(gca,'YScale','log'); xlabel('Tempo normalizzato'); ylabel('Errore rel. [%]');
title('Stima distribuita del terreno: errore relativo sul vettore dei parametri');
exportgraphics(fig9, fullfile(cartella, '9_montecarlo_convergenza.png'), 'Resolution', 200);

% FIGURA 10: consistenza
fig10 = figure('Name','Monte Carlo: consistenza','Color','w', ...
               'Position',[150 150 1300 400]);

subplot(1,3,1); hold on; grid on;
[lo, hi] = bande_nees(N_run, 2);
% Sfondo grigio dove almeno un veicolo e' in zona cieca: serve a leggere le
% escursioni del NEES insieme alla condizione che le produce.
den = S.curva_denied_med;
area(tau, 5*(den > 0), 'FaceColor', [0.88 0.88 0.88], 'EdgeColor', 'none', ...
     'HandleVisibility', 'off');
for i = 1:N_veh
    plot(tau, S.curva_nees_med(i,:), 'Color', colori(i,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('V%d', i));
end
yline(2,  'k-',  'LineWidth', 1.5, 'DisplayName', 'atteso = 2');
yline(lo, 'r--', 'LineWidth', 1.2, 'DisplayName', 'banda 95%');
yline(hi, 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xlabel('Tempo normalizzato'); ylabel('NEES di posizione');
title('NEES mediato sulle ripetizioni'); legend('Location','best','FontSize',7);

subplot(1,3,2);
istogramma = cat(1, R.c_terr);
histogram(istogramma, max(8, round(N_run/5)), 'FaceColor', [0.3 0.5 0.8]); hold on; grid on;
xline(mean(istogramma), 'b-', 'LineWidth', 1.6, 'DisplayName', 'media stimata');
xline(c_vero, 'r--', 'LineWidth', 1.8, 'DisplayName', 'valore vero');
xlabel('c_{terr} stimato'); ylabel('Ripetizioni');
title('Dispersione della stima del parametro di terreno');
legend('Location','best','FontSize',7);

% Lo scatter mostra che la qualita' della stima del terreno non e' casuale:
% dipende da quanto il Master ha esplorato velocita' diverse in quel run.
subplot(1,3,3); grid on; hold on;
sv2 = cat(1, R.std_v2); sv2 = sv2(:,1);
scatter(sv2, cat(1, R.err_c), 28, [0.2 0.4 0.8], 'filled');
xlabel('std(v^2) del Master'); ylabel('Errore su c_{terr} [%]');
title(sprintf('Eccitazione contro errore (\\rho = %+.2f)', ...
      corr_pearson(sv2, cat(1, R.err_c))));
exportgraphics(fig10, fullfile(cartella, '10_montecarlo_consistenza.png'), 'Resolution', 200);

fprintf('\nFigure salvate in %s\n', cartella);
end
