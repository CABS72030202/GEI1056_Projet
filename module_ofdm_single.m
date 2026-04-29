% module_ofdm_single.m
% Script principal du systeme de communication OFDM sur un seul bladeRF.
% Ce script orchestre la chaine complete de transmission et de reception en duplex
% sur une meme radio, permettant l'emission et la reception simultanees:
% 1) Charger la configuration des parametres RF et OFDM.
% 2) Ouvrir et configurer la radio bladeRF unique pour TX et RX.
% 3) Construire la trame TX et synchroniser les horloges TX/RX pour capturer le signal.
% 4) Traiter les echantillons recus (synchronisation, egalisation, demodulation).
% 5) Fermer proprement la radio.

clear; clc; close all;

%% Preparation de l'environnement
% Charger la configuration et ajouter le dossier des pilotes bladeRF au path.
config = default_ofdm_config();
addpath(config.bladeRFMatlabDir);
if ~contains(lower(getenv('PATH')), lower(config.bladeRFRoot))
    setenv('PATH', [config.bladeRFRoot ';' getenv('PATH')]);
end

%% Ouverture de la radio bladeRF
% Une erreur a l'ouverture (radio absente ou driver manquant) n'interrompt pas
% le script: les etapes suivantes fonctionneront en mode degrade ou simule.
try
    b = bladeRF();
    fprintf('bladeRF ouvert avec succes.\n');
    fprintf('  Serial : %s\n', strtrim(b.info.serial));
catch ME
    fprintf(2, 'Erreur a l''ouverture du bladeRF:\n%s\n', ME.message);
    b = [];
end

%% Execution de la chaine OFDM complete
% Pipeline principal:
% - module_tx    : genere les bits, construit et module la trame OFDM.
% - synchronize_tx_rx : emet la trame TX et capture les echantillons RX
%                       en utilisant les timestamps hardware pour aligner
%                       les deux flux sur la meme horloge.
% - module_rx    : synchronise, egalize et demodule les echantillons recus.
txContext     = module_tx(config);
rawRxSamples  = synchronize_tx_rx(b, txContext, config);
rxContext     = module_rx(rawRxSamples, config, txContext); %#ok<NASGU>

%% Fermeture de la radio bladeRF
if ~isempty(b)
    try
        delete(b);
        clear b;
        fprintf('bladeRF ferme avec succes.\n');
    catch MEclose
        fprintf(2, 'Erreur a la fermeture du bladeRF:\n%s\n', MEclose.message);
    end
end

%% -----------------------------------------------------------------------

function config = default_ofdm_config()
    % Centraliser tous les parametres du systeme OFDM dans une seule structure.
    % Modifier ces valeurs pour adapter le systeme (bande, modulation, etc.).
    config.bladeRFRoot      = 'C:\Program Files\bladeRF';
    config.bladeRFMatlabDir = fullfile(config.bladeRFRoot, 'matlab');

    % --- Parametres RF ---
    % Frequence centrale, taux d'echantillonnage, bande passante et gains TX/RX.
    config.centerFrequencyHz = 2.45e9;
    config.sampleRateHz      = 2.5e6;
    config.bandwidthHz       = 2.5e6;
    config.txGainDb          = 40;
    config.rxGainDb          = 40;
    config.syncLeadTimeMs    = 250;  % Delai avant l'emission TX pour laisser le temps au RX de demarrer (ms)
    config.rxPrerollMs       = 10;   % Debut de la capture RX avant l'emission TX, pour ne pas rater le debut de trame (ms)
    config.rxPostrollMs      = 10;   % Prolongation de la capture RX apres la fin de la trame TX (ms)
    config.streamTimeoutMs   = 5000;

    % --- Parametres OFDM ---
    % N_FFT  : taille de la FFT (nombre total de sous-porteuses)
    % N_VC   : sous-porteuses de garde (virtual carriers), laissees a zero
    % N_CP   : longueur du prefixe cyclique (doit etre >= delai max du canal)
    % N_USED : sous-porteuses actives portant des donnees (N_FFT - N_VC)
    % N_SYM  : nombre de symboles OFDM dans le payload
    % N_PREA : nombre de repetitions du symbole de preambule
    % M_ORDER: ordre de la modulation QAM (ex: 16 pour 16-QAM)
    config.N_FFT   = 64;
    config.N_VC    = config.N_FFT / 4;
    config.N_CP    = config.N_FFT / 4;
    config.N_USED  = config.N_FFT - config.N_VC;
    config.N_SYM   = 12;
    config.N_PREA  = 4;
    config.M_ORDER = 16;
    config.txZeroPadMs = 5;

    K = config.N_USED / 2;
    config.data_bins_shift = [(-K):-1 1:K];
end


function rawSamples = synchronize_tx_rx(radio, txContext, config)
% Configurer la radio, puis planifier l'emission TX et la capture RX
% sur la meme horloge hardware (timestamps) pour garantir leur alignement.
% La capture RX demarre quelques echantillons avant l'emission TX (preroll)
% afin de ne pas manquer le debut de la trame en cas de latence variable.

    rawSamples = [];
    if isempty(radio)
        warning('module_ofdm: aucun bladeRF fourni.');
        return;
    end

    fs = config.sampleRateHz;

    % --- Configuration RF identique pour TX et RX ---
    radio.rx.frequency  = config.centerFrequencyHz;
    radio.tx.frequency  = config.centerFrequencyHz;
    radio.rx.samplerate = fs;
    radio.tx.samplerate = fs;
    radio.rx.bandwidth  = config.bandwidthHz;
    radio.tx.bandwidth  = config.bandwidthHz;
    radio.rx.config.timeout_ms = config.streamTimeoutMs;
    radio.tx.config.timeout_ms = config.streamTimeoutMs;
    radio.tx.gain = config.txGainDb;

    % --- Calcul des longueurs de capture et des timestamps hardware ---
    % Le timestamp TX est fixe dans le futur pour laisser au RX le temps de demarrer.
    % Le timestamp RX est avance de 'preroll' echantillons avant le debut TX.
    tx_len   = numel(txContext.txBuffer);
    preroll  = round(config.rxPrerollMs  * fs / 1000);
    postroll = round(config.rxPostrollMs * fs / 1000);
    rx_len   = tx_len + preroll + postroll;

    radio.tx.start();
    tx_ts = uint64(double(radio.rx.timestamp) + round(config.syncLeadTimeMs * fs / 1000));
    rx_ts = uint64(max(0, double(tx_ts) - preroll));

    fprintf('module_ofdm: synchro TX/RX (t0=%u, RX=%u, %d echantillons).\n', tx_ts, rx_ts, rx_len);

    % --- Lancer la capture RX dans un worker parallele ---
    % L'utilisation de parfeval permet de demarrer le recepteur en parallele
    % de l'emission, ce qui est indispensable pour respecter les timestamps.
    rx_future = [];
    try
        pool = gcp('nocreate');
        if isempty(pool), pool = parpool('local', 1); end
        rx_future = parfeval(pool, @bladerf_parallel_rx_worker, 4, ...
            strtrim(radio.info.serial), config, rx_ts, rx_len);
        fprintf('module_ofdm: worker RX parallele lance (%d echantillons).\n', rx_len);
    catch ME
        warning('module_ofdm:noParallel', 'Worker RX indisponible: %s', ME.message);
    end

    % --- Emettre le burst TX ---
    radio.transmit(double(txContext.txBuffer(:)), config.streamTimeoutMs, tx_ts, true, true);
    fprintf('module_ofdm: burst TX envoye (%d echantillons).\n', tx_len);

    % --- Recuperer les echantillons RX ---
    if ~isempty(rx_future)
        % Cas nominal: lecture des echantillons du worker parallele.
        [samples, ts_out, actual_count, overrun] = fetchOutputs(rx_future);
        cancel(rx_future);
    else
        % Fallback sequentiel si le worker n'a pas pu demarrer
        radio.rx.start();
        [samples, ts_out, actual_count, overrun] = radio.receive(rx_len, config.streamTimeoutMs, rx_ts);
        radio.rx.stop();
    end

    rawSamples = samples(:);
    if actual_count > 0 && actual_count < numel(rawSamples)
        rawSamples = rawSamples(1:actual_count);
    end

    fprintf('module_ofdm: %d echantillons captures (ts=%u).\n', numel(rawSamples), ts_out);
    if overrun
        warning('module_ofdm:rxOverrun', 'Overrun detecte pendant la capture RX.');
    end

    radio.tx.stop();
end
