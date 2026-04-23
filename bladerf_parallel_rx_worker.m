function [samples, timestamp_out, actual_count, overrun] = bladerf_parallel_rx_worker(deviceSerial, config, rxTimestamp, rxLength)
%BLADERF_PARALLEL_RX_WORKER Ouvrir un bladeRF sur un worker et capturer RX.
%   Appelee via parfeval depuis module_ofdm.
% But: preparer la capture RX en parallele pour fiabiliser la synchro.

    % Preparer l'environnement bladeRF dans le worker
    % Chaque worker MATLAB est un processus separe: reconfigurer PATH/addpath.
    addpath(config.bladeRFMatlabDir);
    if ~contains(lower(getenv('PATH')), lower(config.bladeRFRoot))
        setenv('PATH', [config.bladeRFRoot ';' getenv('PATH')]);
    end

    b = bladeRF(['*:serial=' char(deviceSerial)]);
    % Fermer automatiquement le peripherique meme en cas d'erreur.
    cleanupObj = onCleanup(@() delete(b)); %#ok<NASGU>

    % Configurer et demarrer RX
    b.rx.frequency         = config.centerFrequencyHz;
    b.rx.samplerate        = config.sampleRateHz;
    b.rx.bandwidth         = config.bandwidthHz;
    b.rx.config.timeout_ms = config.streamTimeoutMs;
    b.rx.start();

    % Capture horodatee demandee a rxTimestamp.
    [samples, timestamp_out, actual_count, overrun] = b.receive(rxLength, config.streamTimeoutMs, rxTimestamp);
    samples = samples(:);
end
