% module_ofdm_double.m
% Script principal OFDM pour architecture a deux ordinateurs / deux bladeRF.
% Chaque machine execute ce meme script, avec un role fixe:
% - role = 'TX' : emission uniquement
% - role = 'RX' : reception + traitement uniquement
%
% NOTE: la synchronisation inter-PC fine sera ajoutee plus tard.
% Ici, la synchro manuelle est rendue robuste via:
% - TX en bursts repetes
% - RX en surcapture longue

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
		txContext = module_tx(config);
		transmit_only(radio, txContext, config);

	case 'RX'
		% On reconstruit localement le contexte TX attendu pour reutiliser
		% les fonctions de synchro/egalisation/decode deja existantes.
		txContext = module_tx(config);
		rawRxSamples = receive_only(radio, txContext, config);
		rxContext = module_rx(rawRxSamples, config, txContext); %#ok<NASGU>
end

%% -----------------------------------------------------------------------

function config = default_ofdm_config()
	% Centraliser les parametres du systeme OFDM et de l'interface bladeRF.
	config.bladeRFRoot      = 'C:\Program Files\bladeRF';
	config.bladeRFMatlabDir = fullfile(config.bladeRFRoot, 'matlab');

	% Constante de role pour cette machine: 'TX' ou 'RX'.
	config.nodeRole = 'TX';

	% Optionnel: serials dedies par role.
	% Laisser vide ('') pour ouverture automatique.
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


function rawSamples = receive_only(radio, txContext, config)
% Reception seule (role RX), puis retour des echantillons bruts.

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

	tx_len   = numel(txContext.txBuffer);
	raw_capture_len = round(double(config.rxCaptureDurationMs) * fs / 1000);
	based_capture_len = tx_len + round(config.rxPrerollMs  * fs / 1000) + round(config.rxPostrollMs * fs / 1000);
	rx_len   = max(raw_capture_len, based_capture_len);

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
