% module_ofdm_double.m
% Script principal OFDM pour architecture a deux ordinateurs / deux bladeRF.
% Chaque machine execute ce meme script, avec un role fixe:
% - role = 'TX' : emission uniquement
% - role = 'RX' : reception + traitement uniquement

clear; clc; close all;

%% Preparation de l'environnement
config = default_ofdm_config();
addpath(config.bladeRFMatlabDir);
if ~contains(lower(getenv('PATH')), lower(config.bladeRFRoot))
	setenv('PATH', [config.bladeRFRoot ';' getenv('PATH')]);
end

%% Choix du role (constante)
% Changer UNIQUEMENT cette constante selon la machine:
%   'TX' sur le PC emetteur, 'RX' sur le PC recepteur.
nodeRole = upper(strtrim(config.nodeRole));
if ~ismember(nodeRole, {'TX', 'RX'})
	error('module_ofdm_double: nodeRole invalide (%s). Utiliser TX ou RX.', nodeRole);
end

%% Ouverture de la radio pour le role choisi
radio = open_one_radio_by_role(config, nodeRole);
cleanupRadio = onCleanup(@() close_one_radio(radio, nodeRole)); %#ok<NASGU>

if isempty(radio)
	error('module_ofdm_double: impossible d''ouvrir la radio pour le role %s.', nodeRole);
end

%% Execution OFDM selon le role
switch nodeRole
	case 'TX'
		% Fixer la graine avant generation des bits: les deux PCs utilisent
		% la meme sequence PRBS sans echange de donnees (pratique standard BER).
		rng(config.dataSeed);
		txContext = module_tx(config);
		transmit_only(radio, txContext, config);

	case 'RX'
		% Capturer les echantillons bruts — RX ne connait pas encore les donnees.
		rawRxSamples = receive_only(radio, config);

		% Correction SFO: utilise uniquement le preambule deterministe (config),
		% aucune connaissance des bits TX n'est necessaire ici.
		correctedSamples = estimate_and_correct_sfo(rawRxSamples, config);

		% Reconstruire la reference TX avec la meme graine PRBS que le TX.
		% Le RX ne recoit pas ces bits: il les regenere independamment
		% grace a la graine commune, comme tout testeur de BER.
		rng(config.dataSeed);
		txContext = module_tx(config);
		rxContext = module_rx(correctedSamples, config, txContext); %#ok<NASGU>
end

%% -----------------------------------------------------------------------

function config = default_ofdm_config()
	% Centraliser les parametres du systeme OFDM et de l'interface bladeRF.
	config.bladeRFRoot      = 'C:\Program Files\bladeRF';
	config.bladeRFMatlabDir = fullfile(config.bladeRFRoot, 'matlab');

	% Constante de role pour cette machine: 'TX' ou 'RX'.
	config.nodeRole = 'TX';

	% Graine PRBS commune aux deux PCs pour la generation des bits de test.
	% Modifier cette valeur pour changer la sequence, mais elle doit etre
	% identique sur les deux machines.
	config.dataSeed = 42;

	config.txSerial = '';
	config.rxSerial = '';

	% --- Parametres RF ---
	config.centerFrequencyHz = 2.45e9;
	config.sampleRateHz      = 2.5e6;
	config.bandwidthHz       = 2.5e6;
	config.txGainDb          = 40;
	config.rxGainDb          = 40;
	config.syncLeadTimeMs    = 250;
	config.rxPrerollMs       = 10;
	config.rxPostrollMs      = 10;
	config.streamTimeoutMs   = 5000;

	% --- Parametres de synchro manuelle inter-PC ---
	% TX: envoyer plusieurs bursts pour tolerer l'incertitude de lancement.
	config.txBurstCount     = 20;
	config.txBurstPeriodMs  = 80;
	% RX: surcapture longue couvrant toute la rafale TX.
	config.rxCaptureDurationMs = 4000;

	% --- Parametres OFDM ---
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

function radio = open_one_radio_by_role(config, roleName)
% Ouvrir la radio correspondant au role de cette machine.

	radio = [];
	try
		if strcmpi(roleName, 'TX')
			serial = config.txSerial;
		else
			serial = config.rxSerial;
		end

		if isempty(strtrim(serial))
			radio = bladeRF();
		else
			radio = bladeRF(['*:serial=' char(strtrim(serial))]);
		end
		fprintf('bladeRF %s ouvert avec succes (serial=%s).\n', roleName, strtrim(radio.info.serial));
	catch ME
		fprintf(2, 'Erreur ouverture bladeRF %s:\n%s\n', roleName, ME.message);
	end
end


function transmit_only(radio, txContext, config)
% Emission seule (role TX).

	if isempty(radio)
		warning('module_ofdm_double:txMissing', 'Radio TX absente.');
		return;
	end

	fs = config.sampleRateHz;
	radio.tx.frequency         = config.centerFrequencyHz;
	radio.tx.samplerate        = fs;
	radio.tx.bandwidth         = config.bandwidthHz;
	radio.tx.config.timeout_ms = config.streamTimeoutMs;
	radio.tx.gain              = config.txGainDb;

	tx_len = numel(txContext.txBuffer);
	tx_len_samples = numel(txContext.txBuffer);
	leadSamples = round(config.syncLeadTimeMs * fs / 1000);
	tx_ts0 = uint64(double(radio.tx.timestamp) + leadSamples);

	burst_count = max(1, round(double(config.txBurstCount)));
	period_samples_cfg = round(double(config.txBurstPeriodMs) * fs / 1000);
	period_samples = max(period_samples_cfg, tx_len_samples + round(0.001 * fs));

	fprintf(['module_ofdm_double[TX]: %d bursts programmes, periode=%d echantillons, ' ...
		'longueur burst=%d.\n'], burst_count, period_samples, tx_len);
	radio.tx.start();
	for k = 0:(burst_count - 1)
		tx_tsk = uint64(double(tx_ts0) + double(k) * double(period_samples));
		radio.transmit(double(txContext.txBuffer(:)), config.streamTimeoutMs, tx_tsk, true, true);
	end
	radio.tx.stop();
	fprintf('module_ofdm_double[TX]: sequence de bursts envoyee.\n');
end


function rawSamples = receive_only(radio, config)
% Reception seule (role RX), puis retour des echantillons bruts.
% Ne depend d'aucune donnee TX: longueur de capture basee uniquement
% sur les parametres OFDM et la duree de surcapture configuree.

	rawSamples = [];
	if isempty(radio)
		warning('module_ofdm_double:rxMissing', 'Radio RX absente.');
		return;
	end

	fs = config.sampleRateHz;

	radio.rx.frequency         = config.centerFrequencyHz;
	radio.rx.samplerate        = fs;
	radio.rx.bandwidth         = config.bandwidthHz;
	radio.rx.config.timeout_ms = config.streamTimeoutMs;

	% Longueur de capture derivee des parametres OFDM (pas du buffer TX).
	sym_len         = config.N_FFT + config.N_CP;
	one_burst_len   = config.N_PREA * sym_len + config.N_SYM * sym_len + ...
	                  2 * round(double(config.txZeroPadMs) * fs / 1000);
	raw_capture_len = round(double(config.rxCaptureDurationMs) * fs / 1000);
	rx_len          = max(raw_capture_len, one_burst_len);

	leadSamples = round(config.syncLeadTimeMs * fs / 1000);
	rx_ts = uint64(max(0, double(radio.rx.timestamp) + leadSamples));

	fprintf('module_ofdm_double[RX]: surcapture a ts=%u (%d echantillons).\n', rx_ts, rx_len);
	radio.rx.start();
	[samples, ts_out, actual_count, overrun] = radio.receive(rx_len, config.streamTimeoutMs, rx_ts);
	radio.rx.stop();

	rawSamples = samples(:);
	if actual_count > 0 && actual_count < numel(rawSamples)
		rawSamples = rawSamples(1:actual_count);
	end

	fprintf('module_ofdm_double[RX]: %d echantillons captures (ts=%u).\n', numel(rawSamples), ts_out);
	if overrun
		warning('module_ofdm_double:rxOverrun', 'Overrun detecte pendant la capture RX.');
	end
end


function correctedSamples = estimate_and_correct_sfo(rawSamples, config)
% Estimer et corriger le SFO (Sample Frequency Offset) entre les deux bladeRF.
%
% Le preambule de reference est reconstruit ici depuis config uniquement:
% sa structure est entierement deterministe (pas de bits aleatoires),
% donc RX peut le generer independamment sans connaitre les donnees TX.
%
% Principe:
%   1) Correlater le preambule de reference sur la surcapture pour trouver
%      les pics correspondant aux differents bursts TX.
%   2) Comparer l'espacement mesure entre deux pics avec l'espacement nominal
%      attendu (burst_period_samples) -> donne epsilon en ppm.
%   3) Reechantillonner le flux RX avec resample(x, p, q) pour compenser.
%      ratio = 1/(1+epsilon), approxime en fraction rationnelle p/q.
%   4) Le flux corrige a la bonne cadence d'echantillonnage nominale;
%      module_rx peut alors s'en servir normalement.

	correctedSamples = rawSamples;  % par defaut, pas de correction

	if isempty(rawSamples)
		warning('module_ofdm_double:sfo', 'Donnees insuffisantes pour estimer le SFO.');
		return;
	end

	% --- 1) Correlation preambule pour detecter les pics de bursts ---
	% Le preambule est deterministe: RX peut le reconstruire sans connaitre
	% les bits TX. Aucune information de la couche donnees n'est utilisee ici.
	pre = build_preamble_ref(config);
	raw = rawSamples(:);
	corr = abs(conv(raw, conj(flipud(pre)), 'valid'));

	% Normaliser pour avoir un seuil independant du niveau de puissance.
	corr_norm = corr / (max(corr) + eps);

	% L'espacement attendu entre bursts en echantillons (d'apres la config TX).
	fs = config.sampleRateHz;
	sym_len       = config.N_FFT + config.N_CP;
	one_burst_len = config.N_PREA * sym_len + config.N_SYM * sym_len + ...
	                2 * round(double(config.txZeroPadMs) * fs / 1000);
	period_samples = max( ...
		round(double(config.txBurstPeriodMs) * fs / 1000), ...
		one_burst_len + round(0.001 * fs));

	% Trouver les pics avec une zone d'exclusion = period/2 autour de chaque pic.
	exclude = max(1, round(period_samples / 2));
	peaks_idx = [];
	corr_copy = corr_norm;
	threshold = 0.3;
	for attempt = 1:config.txBurstCount
		[peak_val, peak_pos] = max(corr_copy);
		if peak_val < threshold, break; end
		peaks_idx(end+1) = peak_pos; %#ok<AGROW>
		% Exclure le voisinage de ce pic pour le prochain tour.
		i_lo = max(1, peak_pos - exclude);
		i_hi = min(numel(corr_copy), peak_pos + exclude);
		corr_copy(i_lo:i_hi) = 0;
	end

	if numel(peaks_idx) < 2
		warning('module_ofdm_double:sfo', ...
			'Moins de 2 pics detectes (%d): SFO non corrige.', numel(peaks_idx));
		return;
	end

	% --- 2) Estimation de l'epsilon SFO ---
	% Utiliser le premier et le dernier pic pour maximiser la resolution.
	peaks_sorted = sort(peaks_idx);
	pos1 = peaks_sorted(1);
	pos2 = peaks_sorted(end);
	n_gaps = peaks_sorted(end) - peaks_sorted(1);	% peut couvrir plusieurs periodes
	n_periods = round(n_gaps / period_samples);

	if n_periods < 1
		warning('module_ofdm_double:sfo', 'Pics trop proches pour estimer le SFO.');
		return;
	end

	measured_period = (pos2 - pos1) / n_periods;
	epsilon = (measured_period - period_samples) / period_samples;
	epsilon_ppm = epsilon * 1e6;

	fprintf(['module_ofdm_double[RX]: SFO estime = %.2f ppm ' ...
		'(pos1=%d, pos2=%d, n_periods=%d).\n'], epsilon_ppm, pos1, pos2, n_periods);

	if abs(epsilon_ppm) < 0.1
		fprintf('module_ofdm_double[RX]: SFO negligeable, pas de resampling.\n');
		return;
	end

	% --- 3) Resampling pour compenser la derive d'horloge ---
	% On veut que le RX ait la meme cadence que le TX:
	%   ratio = f_s_TX / f_s_RX = 1 / (1 + epsilon)
	% resample(x, p, q) produit p/q echantillons par echantillon d'entree.
	N_rat = 10000;
	p_rat = N_rat;
	q_rat = round(N_rat * (1 + epsilon));
	if q_rat < 1, q_rat = 1; end

	correctedSamples = resample(raw, p_rat, q_rat);
	fprintf('module_ofdm_double[RX]: resampling %d/%d applique.\n', p_rat, q_rat);
end


function preamble = build_preamble_ref(config)
% Reconstruire le preambule de reference depuis la configuration uniquement.
% Identique a build_tx_preamble dans module_tx, mais disponible cote RX
% sans avoir besoin du contexte TX ni des bits de donnees.

	preamble_freq_shift = complex(zeros(config.N_FFT, 1));
	used_bins = mod(config.data_bins_shift, config.N_FFT) + 1;
	alt = (-1).^(0:numel(used_bins)-1).';
	preamble_freq_shift(used_bins) = complex(alt, 0);

	preamble_time = ifft(ifftshift(preamble_freq_shift, 1), config.N_FFT, 1);
	preamble_sym  = [preamble_time(end - config.N_CP + 1:end); preamble_time];

	sign_pattern = repmat([1, -1], 1, ceil(config.N_PREA / 2));
	sign_pattern = sign_pattern(1:config.N_PREA);
	preamble = [];
	for k = 1:config.N_PREA
		preamble = [preamble; preamble_sym * sign_pattern(k)]; %#ok<AGROW>
	end
end


function close_one_radio(radio, roleName)
% Fermer proprement la radio de ce noeud.

	if isempty(radio)
		return;
	end

	try
		delete(radio);
		fprintf('bladeRF %s ferme avec succes.\n', roleName);
	catch ME
		fprintf(2, 'Erreur fermeture bladeRF %s:\n%s\n', roleName, ME.message);
	end
end
