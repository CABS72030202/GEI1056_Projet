function txContext = module_tx(config)
%MODULE_TX Structure de la chaine d'emission OFDM.
% Chaque etape est isolee pour faciliter la lecture et les tests.
% Pipeline TX:
% bits -> symboles QAM -> grille OFDM -> IFFT + CP -> preambule -> buffer radio

	% Conserver les informations utiles pour le suivi de la trame TX.
	txContext = struct();
	txContext.config = config;
	txContext.bits = [];
	txContext.symbols = [];
	txContext.parallelSymbols = [];
	txContext.frequencyGridShift = [];
    txContext.preamble = [];
	txContext.timeSignal = [];
	txContext.txBuffer = [];

	% 1) Generer les bits a transmettre.
	txContext.bits = generate_tx_bits(config);

	% 2) Mapper les bits vers la constellation choisie.
	txContext.symbols = map_bits_to_symbols(txContext.bits, config);

	% 3) Construire le payload OFDM et le preambule.
	txContext = build_ofdm_frame(txContext, config);
    txContext.preamble = build_tx_preamble(config);

	% 4) Passer en domaine temporel.
	txContext.timeSignal = compose_time_domain_signal(txContext, config);

	% 5) Mettre le signal au format attendu par le bladeRF.
	txContext.txBuffer = prepare_tx_buffer(txContext.timeSignal, config);

	% 6) Visualiser le signal prepare + constellation.
	visualize_tx_signal(txContext.txBuffer, txContext.symbols, config);
end


function bits = generate_tx_bits(config)
% Generer une sequence de bits aleatoires pour la transmission.
% Sortie: vecteur colonne de bits (0/1).

    bits_per_sym = log2(config.M_ORDER);
    n_data_symbols = config.N_USED * config.N_SYM;
    total_bits = n_data_symbols * bits_per_sym;

    bits = randi([0 1], total_bits, 1);

    fprintf('module_tx: %d bits generes pour transmission (%d symboles).\n', numel(bits), n_data_symbols);
end


function symbols = map_bits_to_symbols(bits, config)
% Mapper les bits vers la modulation choisie.
% Mapping QAM Gray (necessite Communications Toolbox).

	bits_per_sym = log2(config.M_ORDER);

	% Regrouper les bits par symbole: [bits_per_sym] bits -> 1 symbole.
	b_reshaped = reshape(bits, bits_per_sym, []).';

	% Bits -> indices entiers -> symboles QAM normalises.
	symbol_indices = bi2de(b_reshaped, 'left-msb');
	symbols = qammod(symbol_indices, config.M_ORDER, 'gray', ...
		'InputType', 'integer', 'UnitAveragePower', true);

	fprintf('module_tx: [%d x %d] symbols generes.\n', size(symbols, 1), size(symbols, 2));
end


function txContext = build_ofdm_frame(txContext, config)
% Construire la grille OFDM en domaine frequentiel.
% Serie -> parallele: colonnes = symboles OFDM, lignes = sous-porteuses.

	txContext.parallelSymbols = reshape(txContext.symbols, config.N_USED, config.N_SYM);

	freq_grid_shift = complex(zeros(config.N_FFT, config.N_SYM));
	used_bins = mod(config.data_bins_shift, config.N_FFT) + 1;
	% Seules les sous-porteuses utilisees portent des donnees.
	freq_grid_shift(used_bins, :) = txContext.parallelSymbols;

	txContext.frequencyGridShift = freq_grid_shift;

	fprintf('module_tx: conversion serie-parallele [%d x %d] et mapping sous-porteuses termine.\n', ...
		size(txContext.parallelSymbols, 1), size(txContext.parallelSymbols, 2));

end


function timeSignal = compose_time_domain_signal(txContext, config)
% Appliquer IFFT, insertion CP, puis inserer un preambule en tete de trame.
% CP (Cyclic Prefix): reduit l'impact du multi-trajet.

	X_ifft_in = ifftshift(txContext.frequencyGridShift, 1);
	x_time = ifft(X_ifft_in, config.N_FFT, 1);

	x_cp = [x_time(end - config.N_CP + 1:end, :); x_time];
	payload = x_cp(:);

	timeSignal = [txContext.preamble; payload];

	fprintf('module_tx: IFFT + CP termines (%d echantillons payload).\n', numel(payload));
	fprintf('module_tx: preambule insere (%d echantillons). Trame TX totale: %d echantillons.\n', ...
		numel(txContext.preamble), numel(timeSignal));
end


function txBuffer = prepare_tx_buffer(timeSignal, ~)
% Normaliser le signal pour limiter les saturations TX.
% Le facteur 0.8 garde une marge anti-ecretage.

	if isempty(timeSignal)
		error('Signal temporel vide: impossible de preparer le buffer TX.');
	end

	peak_abs = max(abs(timeSignal));
	if peak_abs <= 0
		txBuffer = complex(single(timeSignal));
		return;
	end

	scale = 0.8 / peak_abs;
	txBuffer = complex(single(timeSignal .* scale));

	fprintf('module_tx: buffer TX prepare (normalisation %.3f).\n', scale);
end

function preamble = build_tx_preamble(config)
% Preambule OFDM: repetitions du meme symbole avec pattern +/- alterne.
% Sert de reference connue pour la synchronisation et l'estimation de canal.

	% Generer le preambule de base: BPSK +/- alterne sur les porteuses utilisees
	preamble_freq_shift = complex(zeros(config.N_FFT, 1));
	used_bins = mod(config.data_bins_shift, config.N_FFT) + 1;
	
	% Pattern BPSK alterne
	alt = (-1).^(0:numel(used_bins)-1).';
	preamble_freq_shift(used_bins) = complex(alt, 0);

	% IFFT + CP pour un symbole
	preamble_time = ifft(ifftshift(preamble_freq_shift, 1), config.N_FFT, 1);
	preamble_sym = [preamble_time(end - config.N_CP + 1:end); preamble_time];

	% Repetitions avec alternance de signe pour un motif plus robuste.
	sign_pattern = repmat([1, -1], 1, ceil(config.N_PREA / 2));
	sign_pattern = sign_pattern(1:config.N_PREA);
	preamble = [];
	for k = 1:config.N_PREA
		preamble = [preamble; preamble_sym * sign_pattern(k)];
	end
end


function visualize_tx_signal(txBuffer, symbols, config)
% Visualisation simple du signal TX (I/Q + module + constellation).
% Outil de diagnostic visuel rapide.

	if isempty(txBuffer)
		warning('module_tx: aucun echantillon TX a visualiser.');
		return;
	end

	if isfield(config, 'sampleRateHz') && config.sampleRateHz > 0
		fs = double(config.sampleRateHz);
	else
		fs = 1;
	end

	n_plot = min(numel(txBuffer), 2000);
	t = (0:n_plot-1).' / fs;
	s = double(txBuffer(1:n_plot));

	figure('Name', 'TX Signal Visualization', 'NumberTitle', 'off', ...
		'Position', [10, 80, 500, 900]);
	tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

	nexttile(tl);
	plot(t, real(s), 'b', t, imag(s), 'r');
	grid on;
	xlabel('Temps (s)');
	ylabel('Amplitude');
	legend('I', 'Q');
	title('Signal TX (I/Q)');

	nexttile(tl);
	plot(t, abs(s), 'k');
	grid on;
	xlabel('Temps (s)');
	ylabel('|s(t)|');
	title('Enveloppe du signal TX');

	ax_const = nexttile(tl, [2 1]);
	n_const = min(numel(symbols), 3000);
	scatter(real(symbols(1:n_const)), imag(symbols(1:n_const)), 10, 'filled', ...
		'MarkerFaceAlpha', 0.35);
	title('Constellation TX');
	grid on;
	axis equal;
	xlabel('In-phase (I)');
	ylabel('Quadrature (Q)');
    xlim([-1 1]);
    ylim([-1 1]);

	fprintf('module_tx: visualisation TX affichee (%d echantillons).\n', n_plot);
end
