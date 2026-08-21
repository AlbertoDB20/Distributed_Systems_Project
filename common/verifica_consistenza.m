%VERIFICA_CONSISTENZA Test NEES Monte Carlo sul filtro di Fase 2.
%
% Il NEES (Normalized Estimation Error Squared) verifica se la covarianza che il
% filtro DICHIARA e' coerente con l'errore che effettivamente commette:
%
%     NEES_k = (x_true_k - x_est_k)' * inv(Sigma_k) * (x_true_k - x_est_k)
%
% Per un filtro consistente con n stati vale E[NEES] = n = 5.
%   NEES >> n  ->  filtro OTTIMISTA: sottostima la propria incertezza. E' il caso
%                  pericoloso, i 3-sigma bounds non contengono l'errore reale.
%   NEES << n  ->  filtro CONSERVATIVO: sovrastima. Sicuro ma poco informativo.
%
% Mediando su N campioni indipendenti, N*NEES_medio segue una chi-quadro con
% N*n gradi di liberta'. I limiti al 95% sono calcolati con l'approssimazione di
% Wilson-Hilferty, per non dipendere dalla Statistics Toolbox.
%
% AVVERTENZA METODOLOGICA. I campioni consecutivi nel tempo sono fortemente
% correlati, quindi l'intervallo di confidenza calcolato su tutti i campioni e'
% ottimistico: va letto come indicazione dell'ordine di grandezza, non come test
% statistico rigoroso. La validazione formale della Fase 6 dovra' usare un
% campione per run, o decimare temporalmente.
clear; clc;
set(0,'DefaultFigureVisible','off');

M      = 30;    % run Monte Carlo
n      = 5;     % dimensione dello stato
t_skip = 40;    % [s] transitorio di formazione escluso

qui = fileparts(mfilename('fullpath'));
addpath(qui, fullfile(fileparts(qui), 'fase_2'));
cartella_iniziale = pwd;
cd(fullfile(fileparts(qui), 'fase_2'));

nees_tot = [];
fprintf('Campagna Monte Carlo: %d run\n', M);
for m = 1:M
    MODO_BATCH = true;
    rng(1000 + m);
    main2;

    k0 = round(t_skip/Ts);
    for i = 1:N_veh
        for k = k0:N_steps
            e = fleet(i).x_true(:,k) - fleet(i).x_est(:,k);
            e(3) = wrapToPi(e(3));
            nees_tot(end+1) = e' * (fleet(i).Sigma_hist(:,:,k) \ e); %#ok<SAGROW>
        end
    end
    if mod(m,10)==0, fprintf('  ...%d/%d\n', m, M); end
    clearvars -except M n t_skip qui cartella_iniziale nees_tot m
end
cd(cartella_iniziale);

nees_medio = mean(nees_tot);
N          = numel(nees_tot);
gdl        = N * n;
wh         = @(k,z) k*(1 - 2/(9*k) + z*sqrt(2/(9*k)))^3;
lim_inf    = wh(gdl, -1.96) / N;
lim_sup    = wh(gdl, +1.96) / N;

fprintf('\n================= TEST DI CONSISTENZA NEES (Fase 2) =================\n');
fprintf('Run Monte Carlo       : %d\n', M);
fprintf('Campioni NEES         : %d   (escluso il transitorio t < %d s)\n', N, t_skip);
fprintf('Valore atteso E[NEES] : %d   (dimensione dello stato)\n', n);
fprintf('Intervallo 95%%        : [%.3f , %.3f]\n', lim_inf, lim_sup);
fprintf('NEES medio misurato   : %.3f\n', nees_medio);
if nees_medio > lim_sup
    fprintf('ESITO                 : OTTIMISTA — Sigma sottostima l''errore di %.1fx\n', nees_medio/n);
elseif nees_medio < lim_inf
    fprintf('ESITO                 : CONSERVATIVO — Sigma sovrastima l''errore di %.1fx\n', n/nees_medio);
else
    fprintf('ESITO                 : CONSISTENTE\n');
end
fprintf('=====================================================================\n');
