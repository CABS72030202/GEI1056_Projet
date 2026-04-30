%% OFDM TRANSMISSION - BLADE RF
%% TX MODULE 

clear; clc; close all;

%% Preparation de l'environnement
config = default_ofdm_config();
addpath(config.bladeRFMatlabDir);
if ~contains(lower(getenv('PATH')), lower(config.bladeRFRoot))
    setenv('PATH', [config.bladeRFRoot ';' getenv('PATH')]);
end
rng(1,'twister');

%% Ouverture du bladeRF
b = bladeRF();
fprintf('bladeRF ouvert avec succes.\n');
fprintf('  Serial : %s\n', strtrim(b.info.serial));


%% ============================================================
%  Configuration
%% ============================================================
N_FFT   = 64;
N_CP    = N_FFT/4;
N_VC    = N_FFT/4;
N_USED  = N_FFT - N_VC;
N_SYM   = 12;
M_ORDER = 16;
sampleRateHz = 2.5e6;
centerFrequencyHz = 2.45e9;
bandwidthHz = 2.5e6;
txGainDb = 40;
rxGainDb = 40;
streamTimeoutMs = 5000;

b = configure_bladerf(b, sampleRateHz, centerFrequencyHz, bandwidthHz, txGainDb, rxGainDb, streamTimeoutMs);

K = N_USED/2;
data_bins_shift = [-K:-1 1:K];

% Indices MATLAB des sous-porteuses utilisees
used_bins = mod(data_bins_shift, N_FFT) + 1;


%% ============================================================
%  Génération de bit
%% ============================================================
bits_per_sym = log2(M_ORDER);
n_data_symbols = N_USED * N_SYM;
total_bits = n_data_symbols * bits_per_sym;
txBits = randi([0 1], total_bits, 1);


%% ============================================================
%  Modulation
%% ============================================================
bitGroups = reshape(txBits, bits_per_sym, []).';
symbol_indices = bi2de(bitGroups, 'left-msb');
txSymbols = qammod(symbol_indices, M_ORDER,'gray', 'InputType', 'integer', 'UnitAveragePower', true);


%% ============================================================
%  Conversion série/paralelle
%% ============================================================
parallelSymbols = reshape(txSymbols, N_USED, N_SYM);
frequencyGridShift = complex(zeros(N_FFT, N_SYM));
frequencyGridShift(used_bins, :) = parallelSymbols;


%% ============================================================
%  IFFT
%% ============================================================
X_ifft_in = ifftshift(frequencyGridShift, 1);
x_time = ifft(X_ifft_in, N_FFT, 1);


%% ============================================================
%  CP
%% ============================================================
x_cp = [x_time(end-N_CP+1:end, :); x_time];
payload = x_cp(:);


%% ============================================================
%  Préambule
%% ============================================================
N_PREA  = 8;  %Répétition du preambule
preamble = [];

preamble_freq_shift = complex(zeros(N_FFT, 1));
alt = (-1).^(0:numel(used_bins)-1).';
preamble_freq_shift(used_bins) = complex(alt, 0);
preamble_time = ifft(ifftshift(preamble_freq_shift, 1), N_FFT, 1);
preamble_sym = [preamble_time(end-N_CP+1:end); preamble_time];
sign_pattern = repmat([1, -1], 1, ceil(N_PREA/2));
sign_pattern = sign_pattern(1:N_PREA);
    %Alternance de signe pour un premabule répété
for k = 1:N_PREA
    preamble = [preamble; preamble_sym * sign_pattern(k)];
end 


%% ============================================================
%  Zéro Padding
%% ============================================================
pad = complex(zeros(round(0.005 * sampleRateHz), 1)); % 5 ms de silence


%% ============================================================
%  Création du signal
%% ============================================================
%N_REP = 20; %répition du signal pour faire comme lab04
timeSignal = [preamble; payload]; 
txFrame = [pad; timeSignal; pad];


%% ============================================================
%  Normalisation
%% ============================================================
peak_abs = max(abs(txFrame));
if peak_abs <= 0
    txBuffer = complex(single(txFrame));
else
    scale = 0.8 / peak_abs;
    txBuffer = complex(single(txFrame * scale));
end



%% ============================================================
%  ENVOIE BLADE RF
%% ============================================================
N_REP = 20;   % nombre de répétitions de la trame

txStream = repmat(txBuffer(:), N_REP, 1);

tx_len = numel(txBuffer);
tx_stream_len = numel(txStream);

fprintf('TX: longueur dune trame = %d echantillons.\n', tx_len);
fprintf('TX: nombre de repetitions = %d.\n', N_REP);
fprintf('TX: longueur totale envoyee = %d echantillons.\n', tx_stream_len);
fprintf('TX: duree approx dune trame = %.3f ms.\n', 1000 * tx_len / sampleRateHz);
fprintf('TX: duree approx totale = %.3f ms.\n', 1000 * tx_stream_len / sampleRateHz);

b.tx.start();

fprintf('TX: transmission repetee en cours...\n');
b.transmit(double(txStream(:)), streamTimeoutMs);
fprintf('TX: transmission terminee.\n');

b.tx.stop();


%% CONFIG ENVIRONNEMENT 
function config = default_ofdm_config()
    % Parametres globaux centralises (RF + OFDM + synchronisation).
    config.bladeRFRoot      = 'C:\Users\yoyol\OneDrive\UQTR-Yohan\1-Systèmes de télécommunications\Projet';
    config.bladeRFMatlabDir = fullfile(config.bladeRFRoot, 'matlab');

    % Parametres RF
    config.centerFrequencyHz = 2.45e9;
    config.sampleRateHz      = 2.5e6;
    config.bandwidthHz       = 2.5e6;
    config.txGainDb          = 30;
    config.syncLeadTimeMs    = 1000;  % Marge avant le burst TX (ms)
    config.rxPrerollMs       = 10;   % Capture avant le burst (ms)
    config.rxPostrollMs      = 10;   % Capture apres le burst (ms)
    config.streamTimeoutMs   = 5000;

    % Parametres OFDM
    config.N_FFT   = 64;
    config.N_VC    = 16;
    config.N_USED  = config.N_FFT - config.N_VC;
    config.N_CP    = config.N_FFT / 4;
    config.N_SYM   = 12;
    config.N_PREA  = 8;
    config.M_ORDER = 16;

    K = config.N_USED / 2;
    config.data_bins_shift = [(-K):-1 1:K];
end

%% CONFIG BLADE
function b = configure_bladerf(b, sampleRateHz, centerFrequencyHz, bandwidthHz, txGainDb, rxGainDb, streamTimeoutMs)

    b.rx.frequency  = centerFrequencyHz;
    b.tx.frequency  = centerFrequencyHz;

    b.rx.samplerate = sampleRateHz;
    b.tx.samplerate = sampleRateHz;

    b.rx.bandwidth  = bandwidthHz;
    b.tx.bandwidth  = bandwidthHz;

    b.tx.gain = txGainDb;
    b.rx.gain = rxGainDb;

    b.rx.config.timeout_ms = streamTimeoutMs;
    b.tx.config.timeout_ms = streamTimeoutMs;

    fprintf('BladeRF configure:\n');
    fprintf('  Frequence centrale : %.3f GHz\n', centerFrequencyHz/1e9);
    fprintf('  Sample rate        : %.3f MS/s\n', sampleRateHz/1e6);
    fprintf('  Bande passante     : %.3f MHz\n', bandwidthHz/1e6);
    fprintf('  Gain TX            : %.1f dB\n', txGainDb);
end
