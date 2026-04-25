%OFM YOHAN Lineaire and no poopoo no use

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
% Chaine TX OFDM complete sans fonction
% bits -> QAM -> grille OFDM -> IFFT -> CP -> preambule -> txBuffer
%% ============================================================

% 1) Configuration
N_FFT   = 64;
N_CP    = N_FFT/4;
N_VC    = N_FFT/4;
N_USED  = N_FFT - N_VC;
N_SYM   = 12;
N_PREA  = 4;
M_ORDER = 16;
sampleRateHz = 2.5e6;
centerFrequencyHz = 2.45e9;
bandwidthHz = 2.5e6;
txGainDb = 30;
streamTimeoutMs = 5000;

b = configure_bladerf(b, sampleRateHz, centerFrequencyHz, bandwidthHz, txGainDb, streamTimeoutMs);


bits_per_sym = log2(M_ORDER);

K = N_USED/2;
data_bins_shift = [-K:-1 1:K];

% Indices MATLAB des sous-porteuses utilisees
used_bins = mod(data_bins_shift, N_FFT) + 1;

% 2) Generation des bits
n_data_symbols = N_USED * N_SYM;
total_bits = n_data_symbols * bits_per_sym;

txBits = randi([0 1], total_bits, 1);

fprintf('TX: %d bits generes pour %d symboles QAM.\n', total_bits, n_data_symbols);


% 3) Modulation QAM
bitGroups = reshape(txBits, bits_per_sym, []).';

symbol_indices = bi2de(bitGroups, 'left-msb');

txSymbols = qammod(symbol_indices, M_ORDER,'gray', 'InputType', 'integer', 'UnitAveragePower', true);

fprintf('TX: modulation %d-QAM terminee.\n', M_ORDER);


% 4) Conversion serie-parallele
parallelSymbols = reshape(txSymbols, N_USED, N_SYM);

frequencyGridShift = complex(zeros(N_FFT, N_SYM));

frequencyGridShift(used_bins, :) = parallelSymbols;

fprintf('TX: grille OFDM construite [%d x %d].\n',size(frequencyGridShift,1), size(frequencyGridShift,2));


% 5) IFFT : domaine frequentiel -> domaine temporel
X_ifft_in = ifftshift(frequencyGridShift, 1);

x_time = ifft(X_ifft_in, N_FFT, 1);

fprintf('TX: IFFT terminee.\n');


% 6) Ajout du CP
x_cp = [x_time(end-N_CP+1:end, :); x_time];

payload = x_cp(:);

fprintf('TX: CP ajoute. Payload = %d echantillons.\n', numel(payload));


% 7) Construction du preambule
preamble_freq_shift = complex(zeros(N_FFT, 1));

alt = (-1).^(0:numel(used_bins)-1).';
preamble_freq_shift(used_bins) = complex(alt, 0);

preamble_time = ifft(ifftshift(preamble_freq_shift, 1), N_FFT, 1);

preamble_sym = [preamble_time(end-N_CP+1:end); preamble_time];

sign_pattern = repmat([1, -1], 1, ceil(N_PREA/2));
sign_pattern = sign_pattern(1:N_PREA);

preamble = [];

for k = 1:N_PREA
    preamble = [preamble; preamble_sym * sign_pattern(k)];
end

fprintf('TX: preambule construit (%d repetitions, %d echantillons).\n',N_PREA, numel(preamble));


% 8) Trame complete
timeSignal = [preamble; payload];

fprintf('TX: trame complete = %d echantillons.\n', numel(timeSignal));


% 9) Normalisation du buffer TX
peak_abs = max(abs(timeSignal));

if peak_abs <= 0
    txBuffer = complex(single(timeSignal));
else
    scale = 0.8 / peak_abs;
    txBuffer = complex(single(timeSignal * scale));
end

fprintf('TX: buffer normalise. Peak final = %.3f.\n', max(abs(txBuffer)));

% 10) Visualisation TX
n_plot = min(numel(txBuffer), 2000);
t = (0:n_plot-1).' / sampleRateHz;
s = double(txBuffer(1:n_plot));

figure('Name', 'TX Signal Visualization', 'NumberTitle', 'off');

tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t, real(s), 'b', t, imag(s), 'r');
grid on;
xlabel('Temps (s)');
ylabel('Amplitude');
legend('I','Q');
title('Signal TX I/Q');

nexttile;
plot(t, abs(s), 'k');
grid on;
xlabel('Temps (s)');
ylabel('|s(t)|');
title('Enveloppe TX');

nexttile([2 1]);
scatter(real(txSymbols), imag(txSymbols), 10, 'filled', ...
    'MarkerFaceAlpha', 0.35);
grid on;
axis equal;
xlim([-1 1]);
ylim([-1 1]);
xlabel('In-phase (I)');
ylabel('Quadrature (Q)');
title('Constellation TX');




%% ============================================================
% TRANSMISSION + CAPTURE BLADE RF
%% ============================================================

tx_len = numel(txBuffer);

preroll  = round(config.rxPrerollMs  * sampleRateHz / 1000);
postroll = round(config.rxPostrollMs * sampleRateHz / 1000);

rx_len = tx_len + preroll + postroll;

fprintf('\nBladeRF: tx_len = %d echantillons.\n', tx_len);
fprintf('BladeRF: rx_len = %d echantillons.\n', rx_len);

% Demarrer les modules TX/RX
b.rx.start();
b.tx.start();

% Planifier TX dans le futur
current_ts = double(b.rx.timestamp);
tx_ts = uint64(current_ts + round(config.syncLeadTimeMs * sampleRateHz / 1000));

% Planifier RX avant TX
rx_ts = uint64(double(tx_ts) - preroll);

fprintf('BladeRF: TX timestamp = %u.\n', tx_ts);
fprintf('BladeRF: RX timestamp = %u.\n', rx_ts);

% Envoyer le burst avec timestamp
fprintf('BladeRF: transmission TX programmee...\n');
b.transmit(double(txBuffer(:)), streamTimeoutMs, tx_ts, true, true);

% Capturer autour du burst
fprintf('BladeRF: capture RX...\n');
[rawSamples, ts_out, actual_count, overrun] = ...
    b.receive(rx_len, streamTimeoutMs, rx_ts);

rawSamples = rawSamples(:);

if actual_count > 0 && actual_count < numel(rawSamples)
    rawSamples = rawSamples(1:actual_count);
end

fprintf('BladeRF: %d echantillons captures.\n', numel(rawSamples));
fprintf('BladeRF: timestamp RX retourne = %u.\n', ts_out);

if overrun
    warning('BladeRF: overrun detecte pendant la capture RX.');
end

% Arret propre des modules
b.rx.stop();
b.tx.stop();


%% ============================================================
% RX OFDM LINEAIRE - SANS FONCTION
% Reception diagnostic + detection preambule/STO
%% ============================================================


% 1) Recuperer les echantillons bruts
rawSamples = rawSamples(:);

if isempty(rawSamples)
    rawSamples = complex(zeros(N_FFT + N_CP, 1));
    fprintf('RX: echantillons absents, buffer de secours cree (%d echantillons).\n', ...
        numel(rawSamples));
else
    fprintf('RX: %d echantillons fournis pour traitement.\n', numel(rawSamples));
end

rxSamples = rawSamples;

% 2) Analyse simple du signal RX
absBuffer = abs(double(rxSamples(:)));

meanAbs = mean(absBuffer);
maxAbs  = max(absBuffer);
rmsAbs  = rms(absBuffer);

fprintf('RX: mean|s| = %.4f, rms|s| = %.4f, max|s| = %.4f.\n', ...
    meanAbs, rmsAbs, maxAbs);

if maxAbs < 0.05
    warning(['RX: signal tres faible. La capture ressemble peut-etre au bruit. ' ...
             'Verifier le cablage, le gain ou la synchro TX/RX.']);
end


%% STO
% 3) Detection du preambule par correlation
if isempty(preamble)
    error('RX: preambule vide. Impossible de faire la detection STO.');
end

rxSamplesDouble = double(rxSamples(:));
preambleDouble  = double(preamble(:));

if numel(rxSamplesDouble) < numel(preambleDouble)
    error('RX: la capture est plus courte que le preambule.');
end

% Filtre adapte du preambule
preambleConj = conj(preambleDouble);
preambleMatched = flipud(preambleConj);

% Correlation glissante
corrComplex = conv(rxSamplesDouble, preambleMatched, 'valid');

% Magnitude de correlation
corrMetric = abs(corrComplex);

% Detection du pic
[peakValue, startIndex] = max(corrMetric);

estimatedSTO = startIndex - 1;

fprintf('RX: preambule detecte a l''echantillon %d.\n', startIndex);
fprintf('RX: STO estime = %d echantillons.\n', estimatedSTO);
fprintf('RX: pic de correlation = %.4e.\n', peakValue);

% 4) Correction STO : aligner la trame sur le preambule
alignedFrame = rxSamples(startIndex:end);

fprintf('RX: trame alignee = %d echantillons.\n', numel(alignedFrame));

% 5) Visualisation de la correlation

figure('Name', 'RX - Detection preambule', 'NumberTitle', 'off');
plot(corrMetric, 'k');
grid on;
xlabel('Indice de correlation');
ylabel('|Correlation|');
title('Detection du preambule par correlation');


%% 6) Comparaison RX avant et apres alignement STO

fs = sampleRateHz;

frameLength = length(preamble) + length(payload);

rxBefore = rawSamples(:);
rxAfter  = alignedFrame(:);

% Meme longueur affichee pour comparer proprement
nBefore = min(length(rxBefore), frameLength + startIndex - 1);
nAfter  = min(length(rxAfter), frameLength);

sBefore = double(rxBefore(1:nBefore));
sAfter  = double(rxAfter(1:nAfter));

tBefore = (0:nBefore-1).' / fs;
tAfter  = (0:nAfter-1).' / fs;

figure('Name', 'Comparaison RX avant/apres alignement STO', ...
       'NumberTitle', 'off', ...
       'Position', [100, 80, 900, 700]);

tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% I/Q avant
nexttile(tl);
plot(tBefore, real(sBefore), 'b', tBefore, imag(sBefore), 'r');
grid on;
xlabel('Temps (s)');
ylabel('Amplitude');
title('Avant alignement - I/Q');
legend('I','Q');
ylim([-1 1]);
xline((startIndex-1)/fs, '--k', 'Début détecté');

% I/Q apres
nexttile(tl);
plot(tAfter, real(sAfter), 'b', tAfter, imag(sAfter), 'r');
grid on;
xlabel('Temps depuis alignement (s)');
ylabel('Amplitude');
title('Après alignement - I/Q');
legend('I','Q');
ylim([-1 1]);

% Enveloppe avant
nexttile(tl);
plot(tBefore, abs(sBefore), 'k');
grid on;
xlabel('Temps (s)');
ylabel('|s(t)|');
title('Avant alignement - enveloppe');
ylim([0 1]);
xline((startIndex-1)/fs, '--r', 'Début détecté');

% Enveloppe apres
nexttile(tl);
plot(tAfter, abs(sAfter), 'k');
grid on;
xlabel('Temps depuis alignement (s)');
ylabel('|s(t)|');
title('Après alignement - enveloppe');
ylim([0 1]);

% Constellation brute apres
nexttile(tl);
scatter(real(sAfter), imag(sAfter), 8, 'filled', ...
    'MarkerFaceAlpha', 0.25);
grid on;
axis equal;
xlim([-1 1]);
ylim([-1 1]);
xlabel('I');
ylabel('Q');
title('Après alignement - constellation brute');

fprintf('RX: comparaison avant/apres alignement affichee.\n');


%% CFO
n_cfo = (0:length(alignedFrame)-1).';
Lpre = length(preamble_sym);

r1 = alignedFrame(1:Lpre);
r2 = -alignedFrame(Lpre+1:2*Lpre);

phaseProduct = conj(r1).*r2;
phaseSum = sum(phaseProduct);
phaseDiff = angle(phaseSum);

CFO_est_Hz = phaseDiff / (2*pi*Lpre/sampleRateHz);

alignedFrameCFOcorr = alignedFrame .* exp(-1j*2*pi*CFO_est_Hz*n_cfo/sampleRateHz);

fprintf('RX: CFO estime = %.2f Hz\n', CFO_est_Hz);

%Affichage
fs = sampleRateHz;

frameLength = length(preamble) + length(payload);

rxAfterCFO = alignedFrameCFOcorr(:);

nAfterCFO = min(length(rxAfterCFO), frameLength);

sAfterCFO = double(rxAfterCFO(1:nAfterCFO));

tAfterCFO = (0:nAfterCFO-1).' / fs;

figure('Name', 'RX apres correction CFO', ...
       'NumberTitle', 'off', ...
       'Position', [650, 80, 500, 900]);

tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl);
plot(tAfterCFO, real(sAfterCFO), 'b', ...
     tAfterCFO, imag(sAfterCFO), 'r');
grid on;
xlabel('Temps depuis alignement (s)');
ylabel('Amplitude');
legend('I', 'Q');
title('RX apres correction CFO - I/Q');
ylim([-1 1]);

nexttile(tl);
plot(tAfterCFO, abs(sAfterCFO), 'k');
grid on;
xlabel('Temps depuis alignement (s)');
ylabel('|s(t)|');
title('RX apres correction CFO - enveloppe');
ylim([0 1]);

nexttile(tl);
scatter(real(sAfterCFO), imag(sAfterCFO), 8, 'filled', ...
    'MarkerFaceAlpha', 0.25);
grid on;
axis equal;
xlim([-1 1]);
ylim([-1 1]);
xlabel('I');
ylabel('Q');
title('Constellation brute apres correction CFO');

fprintf('RX: visualisation apres correction CFO terminee.\n');





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
    config.N_PREA  = 4;
    config.M_ORDER = 16;

    K = config.N_USED / 2;
    config.data_bins_shift = [(-K):-1 1:K];
end

%% CONFIG BLADE
function b = configure_bladerf(b, sampleRateHz, centerFrequencyHz, bandwidthHz, txGainDb, streamTimeoutMs)

    b.rx.frequency  = centerFrequencyHz;
    b.tx.frequency  = centerFrequencyHz;

    b.rx.samplerate = sampleRateHz;
    b.tx.samplerate = sampleRateHz;

    b.rx.bandwidth  = bandwidthHz;
    b.tx.bandwidth  = bandwidthHz;

    b.tx.gain = txGainDb;

    b.rx.config.timeout_ms = streamTimeoutMs;
    b.tx.config.timeout_ms = streamTimeoutMs;

    fprintf('BladeRF configure:\n');
    fprintf('  Frequence centrale : %.3f GHz\n', centerFrequencyHz/1e9);
    fprintf('  Sample rate        : %.3f MS/s\n', sampleRateHz/1e6);
    fprintf('  Bande passante     : %.3f MHz\n', bandwidthHz/1e6);
    fprintf('  Gain TX            : %.1f dB\n', txGainDb);
end

%% PERMET LE TRAVAIL PARRALEL RX/TX
function [samples, ts_out, actual_count, overrun] = rx_worker_bladerf(serialNumber, centerFrequencyHz, sampleRateHz, bandwidthHz, streamTimeoutMs, rx_ts, rx_len)

    b_rx = bladeRF(['*:serial=' serialNumber]);

    b_rx.rx.frequency  = centerFrequencyHz;
    b_rx.rx.samplerate = sampleRateHz;
    b_rx.rx.bandwidth  = bandwidthHz;
    b_rx.rx.config.timeout_ms = streamTimeoutMs;

    b_rx.rx.start();

    [samples, ts_out, actual_count, overrun] = b_rx.receive(rx_len, streamTimeoutMs, rx_ts);

    b_rx.rx.stop();
end