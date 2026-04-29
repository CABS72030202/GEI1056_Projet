%% OFDM TRANSMISSION - BLADE RF
%% RX MODULE 


%% Preparation de l'environnement
config = default_ofdm_config();
addpath(config.bladeRFMatlabDir);
if ~contains(lower(getenv('PATH')), lower(config.bladeRFRoot))
    setenv('PATH', [config.bladeRFRoot ';' getenv('PATH')]);
end

rng(1,'twister'); %Randomness seed

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
streamTimeoutMs = 5000

b = configure_bladerf(b, sampleRateHz, centerFrequencyHz, bandwidthHz, txGainDb, rxGainDb, streamTimeoutMs);
K = N_USED/2;
data_bins_shift = [-K:-1 1:K];
% Indices MATLAB des sous-porteuses utilisees
used_bins = mod(data_bins_shift, N_FFT) + 1;

% 2) Generation des bits (SERA UTILISÉ SEULEMENT POUR LE BER)
bits_per_sym = log2(M_ORDER);
n_data_symbols = N_USED * N_SYM;
total_bits = n_data_symbols * bits_per_sym;
txBits = randi([0 1], total_bits, 1);
% 3) Modulation QAM
bitGroups = reshape(txBits, bits_per_sym, []).';
symbol_indices = bi2de(bitGroups, 'left-msb');
txSymbols = qammod(symbol_indices, M_ORDER,'gray', 'InputType', 'integer', 'UnitAveragePower', true);
% 4) Conversion serie-parallele (SERA UTILISÉ SEULEMENT POUR LE BER)
parallelSymbols = reshape(txSymbols, N_USED, N_SYM);
frequencyGridShift = complex(zeros(N_FFT, N_SYM));
frequencyGridShift(used_bins, :) = parallelSymbols;


% PREAMBULE IDENTIQUE SUR RX/TX
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
%  CAPTURE BLADE RF
%% ============================================================

% Capture
captureTimeSec = 1;   
rx_len = round(captureTimeSec * sampleRateHz);

fprintf('\nRX: capture de %d echantillons (%.3f ms)\n', rx_len, captureTimeSec);

% Acquisition des données
b.rx.start();
[rawSamples, ts_out, actual_count, overrun] = b.receive(rx_len, streamTimeoutMs);
b.rx.stop();

rawSamples = rawSamples(:);

fprintf('RX: %d echantillons captures.\n', numel(rawSamples));
fprintf('RX: timestamp retourne = %u.\n', ts_out);

if overrun
    warning('RX: overrun detecte pendant la capture.');
end

if isempty(rawSamples)
    rawSamples = complex(zeros(N_FFT + N_CP, 1));
    fprintf('RX: echantillons absents, buffer de secours cree (%d echantillons).\n',numel(rawSamples));
end
rxSamples = rawSamples;




%% ============================================================
%  Synchronisation et STO
%% ============================================================

% 1) Detection du preambule par correlation
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

% Correlation glissante(convolution)
corrComplex = conv(rxSamplesDouble, preambleMatched, 'valid');

% Magnitude de correlation
corrMetric = abs(corrComplex);

% Detection du pic
[peakValue, startIndex] = max(corrMetric);

estimatedSTO = startIndex - 1;

% 4) Correction STO : aligner la trame sur le preambule
alignedFrame = rxSamples(startIndex:end);

% 5) Extraction propre des données
Lpre = length(preamble);
payload_start = startIndex + Lpre;
payload_end   = payload_start + payload_len_expected - 1;
if payload_end > length(rxSamples)
    error('Pas assez d''echantillons apres synchro');
end
rxPreamble = rxSamples(startIndex : startIndex + Lpre - 1);
rxPayload  = rxSamples(payload_start : payload_end);


%% ============================================================
%  CFO
%% ============================================================

sym_len = N_FFT + N_CP;
payload_len_expected = N_SYM * sym_len;
rxPayload_forCFO = rxPayload(1:payload_len_expected);
rxSymsCP = reshape(rxPayload_forCFO, sym_len, N_SYM);

% Estimation du CFO normalisé avec le CP
eps_list = zeros(N_SYM, 1);
for k = 1:N_SYM
    r = rxSymsCP(:, k);
    cp_part   = r(1:N_CP);
    tail_part = r(N_FFT+1:N_FFT+N_CP);
    phase_val = sum(conj(cp_part) .* tail_part);
    eps_list(k) = angle(phase_val) / (2*pi*N_FFT);
end
eps_hat = mean(eps_list);
CFO_Hz = eps_hat * sampleRateHz;
fprintf('RX: CFO estime par CP = %.2f Hz\n', CFO_Hz);

% Correction CFO
n_cfopreamble = (0:length(rxPreamble)-1).';
n_cfopayload  = (0:length(rxPayload)-1).';
rxPreambleCFO = rxPreamble.* exp(-1j*2*pi*eps_hat*n_cfopreamble);
rxPayloadCFO  = rxPayload .* exp(-1j*2*pi*eps_hat*n_cfopayload);


%% ============================================================
%  Estimation du canal
%% ============================================================
% Reformer les repetitions du preambule
rxPreBlk = reshape(rxPreambleCFO, sym_len, N_PREA);

% Corriger l'alternance de signe du preambule : + - + - 
for k = 1:N_PREA
    rxPreBlk(:, k) = rxPreBlk(:, k) * sign_pattern(k);
end

% Reformer les symboles OFDM du payload
rxPaySyms = reshape(rxPayloadCFO, sym_len, N_SYM);

% Reference frequentielle connue du preambule
XtrainUsed = preamble_freq_shift(used_bins);

bestMetric = -inf;
bestOffset = 0;
bestHUsed = [];
metrics = -inf(N_CP, 1);

for off = 0:N_CP-1

    H_list = [];
    % Estimer le canal avec chaque repetition du preambule
    for k = 1:N_PREA
        pre_k = rxPreBlk(off+1 : off+N_FFT, k);
        Yk = fft(pre_k, N_FFT);
        Yk_shift = fftshift(Yk);
        Yk_used = Yk_shift(used_bins);
        Hk = Yk_used ./ (XtrainUsed + 1e-12);
        H_list = [H_list, Hk];
    end

    % Canal moyen sur les repetitions du preambule
    H_used = mean(H_list, 2);

    % Tester le meme offset sur le payload
    x_win = rxPaySyms(off+1 : off+N_FFT, :);
    Y = fft(x_win, N_FFT, 1);
    Y_shift = fftshift(Y, 1);
    Y_used = Y_shift(used_bins, :);

    % Egalisation temporaire pour evaluer la qualite de l'offset
    Yeq = Y_used ./ (H_used + 1e-12);
    metric = mean(abs(mean(Yeq, 1)));
    metrics(off+1) = metric;

    if metric > bestMetric
        bestMetric = metric;
        bestOffset = off;
        bestHUsed = H_used;
    end
end

H_used_final = bestHUsed;   %Estimation final du canal



%% ============================================================
% EGALISATION
%% ============================================================
% Reformer les symboles OFDM du payload apres correction CFO
rxPayloadSyms = reshape(rxPayloadCFO, sym_len, N_SYM);

% Correction CFO residuelle par symbole avec le CP
rxPayloadSymsCFO = zeros(size(rxPayloadSyms));
eps_sym = zeros(N_SYM, 1);

for m = 1:N_SYM

    sym80 = rxPayloadSyms(:, m);
    cp_part   = sym80(1:N_CP);
    tail_part = sym80(N_FFT+1:N_FFT+N_CP);
    val = sum(conj(cp_part) .* tail_part);
    eps_m = angle(val) / (2*pi*N_FFT);
    eps_sym(m) = eps_m;
    n_sym = (0:sym_len-1).';
    rxPayloadSymsCFO(:, m) = sym80 .* exp(-1j*2*pi*eps_m*n_sym);

end

fprintf('RX: CFO residuel moyen par symbole = %.2f Hz\n',mean(eps_sym) * sampleRateHz);

% Retrait du CP avec le meilleur offset trouve
x_no_cp = rxPayloadSymsCFO(bestOffset+1 : bestOffset+N_FFT, :);

% FFT finale
Y_fft = fft(x_no_cp, N_FFT, 1);
Y_fft_shift = fftshift(Y_fft, 1);

% Extraction des sous-porteuses utiles
Y_used = Y_fft_shift(used_bins, :);

% Egalisation avec le canal estime par preambule
rxEq = Y_used ./ (H_used_final + 1e-12);


%% ============================================================
% CPE CORRECTION
%% ============================================================

% rxEq est [N_USED x N_SYM]
Z = rxEq;

% Reference TX dans la meme forme
Xref = parallelSymbols;   % [N_USED x N_SYM]

% Estimation du CPE par symbole OFDM
theta_hat = angle(sum(Z .* conj(Xref), 1) + 1e-12);

% Correction CPE
Z_cpe = Z .* exp(-1j * theta_hat);

% Sauvegarde du signal corrige
rxEqCPE = Z_cpe;

fprintf('RX: CPE par symbole (rad):\n');
disp(round(theta_hat, 3));

% Pour la suite, utiliser rxEqCPE au lieu de rxEq
rxQAM = rxEqCPE(:);


%% ============================================================
% DÉMODULATION
%% ============================================================
rxInts = qamdemod(rxQAM, M_ORDER,'gray','OutputType', 'integer','UnitAveragePower', true);

%Conversion entiers -> bits
rxBitGroups = de2bi(rxInts, bits_per_sym, 'left-msb');
rxBits = reshape(rxBitGroups.', [], 1);


%% ============================================================
% CALCUL DU BER
%% ============================================================
bitComparison = (rxBits ~= txBits);
bitErrors = sum(bitComparison);
BER = bitErrors / total_bits;

fprintf('\n===== RESULTATS BER =====\n');
fprintf('Bits transmis = %d\n', total_bits);
fprintf('Erreurs bits  = %d\n', bitErrors);
fprintf('BER           = %.3e\n', BER);



%% ============================================================
% AFFICHAGE
%% ============================================================

%IQ
figure(1);
scatter(real(rxQAM), imag(rxQAM), 10, 'filled','MarkerFaceAlpha', 0.35);
grid on;
axis equal;
xlabel('I');
ylabel('Q');
title('Apres egalisation');
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);


% Canal 
figure('Name', 'Canal estime H[k]', 'NumberTitle', 'off');

subplot(2,1,1);
plot(abs(H_used_final), 'k', 'LineWidth', 1.2);
grid on;
xlabel('Indice sous-porteuse utile');
ylabel('|H[k]|');
title('Amplitude du canal estime');

subplot(2,1,2);
plot(angle(H_used_final), 'k', 'LineWidth', 1.2);
grid on;
xlabel('Indice sous-porteuse utile');
ylabel('Phase H[k] (rad)');
title('Phase du canal estime');










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
