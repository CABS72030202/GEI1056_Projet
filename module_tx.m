function txContext = module_tx(config)
%MODULE_TX Chaine d'emission OFDM complete.
% Construit la trame a transmettre a partir de la configuration et retourne
% une structure txContext contenant tous les signaux et parametres intermediaires.
% Pipeline TX:
% bits aleatoires -> symboles QAM -> grille OFDM -> IFFT + prefixe cyclique
%                -> preambule -> zero padding -> normalisation -> buffer radio

	% Initialiser la structure de contexte TX avec des champs vides.
	% Cette structure sera completee au fil des etapes du pipeline.
	txContext = struct();
	txContext.config = config;
	txContext.bits = [];
	txContext.symbols = [];
	txContext.parallelSymbols = [];
	txContext.frequencyGridShift = [];
    txContext.zeroPad = [];
    txContext.preamble = [];
	txContext.timeSignal = [];
	txContext.txBuffer = [];

	% 1) Generer les bits d'information a transmettre.
	txContext.bits = generate_tx_bits(config);

	% 2) Mapper les bits vers les symboles de la constellation QAM choisie.
	txContext.symbols = map_bits_to_symbols(txContext.bits, config);

	% 3) Construire la grille frequentielle OFDM et configurer le zero padding.
	txContext = build_ofdm_frame(txContext, config);
    txContext.preamble = build_tx_preamble(config);

	% 4) Convertir la grille OFDM en signal temporel (IFFT + prefixe cyclique).
	txContext.timeSignal = compose_time_domain_signal(txContext, config);

	% 5) Normaliser le signal et le convertir au format attendu par la radio.
	txContext.txBuffer = prepare_tx_buffer(txContext.timeSignal, config);

	% 6) Afficher le signal genere et la constellation TX.
	visualize_tx_signal(txContext.txBuffer, txContext.symbols, config);
end


function bits = generate_tx_bits(config)
% Generer une sequence de bits aleatoires uniformes a transmettre.
% Le nombre de bits est calcule pour remplir exactement la grille OFDM:
% N_USED sous-porteuses * N_SYM symboles * log2(M_ORDER) bits par symbole.

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

	% Convertir les indices entiers en symboles QAM complexes normalises.
	symbol_indices = bi2de(b_reshaped, 'left-msb');
	symbols = qammod(symbol_indices, config.M_ORDER, 'gray','InputType', 'integer', 'UnitAveragePower', true);

	fprintf('module_tx: [%d x %d] symbols generes.\n', size(symbols, 1), size(symbols, 2));
end


function txContext = build_ofdm_frame(txContext, config)
% Construire la grille OFDM en domaine frequentiel (apres fftshift).
% La conversion serie -> parallele repartit les symboles sur les colonnes
% (une colonne = un symbole OFDM, une ligne = une sous-porteuse).
% Les sous-porteuses de garde restent a zero (N_VC sous-porteuses inutilisees).

	txContext.parallelSymbols = reshape(txContext.symbols, config.N_USED, config.N_SYM);

	freq_grid_shift = complex(zeros(config.N_FFT, config.N_SYM));
	used_bins = mod(config.data_bins_shift, config.N_FFT) + 1;
	% Seules les sous-porteuses actives portent des donnees; les autres restent nulles.
	freq_grid_shift(used_bins, :) = txContext.parallelSymbols;

	txContext.frequencyGridShift = freq_grid_shift;

	% Zero padding: silence avant et apres la trame pour faciliter la synchronisation RX.
	pad_len = round((double(config.txZeroPadMs) / 1000) * double(config.sampleRateHz));
	txContext.zeroPad = complex(zeros(pad_len, 1));

	fprintf('module_tx: conversion serie-parallele [%d x %d] et mapping sous-porteuses termine.\n', ...
		size(txContext.parallelSymbols, 1), size(txContext.parallelSymbols, 2));
	fprintf('module_tx: zero padding configure (%d echantillons de chaque cote).\n', pad_len);

end


function timeSignal = compose_time_domain_signal(txContext, config)
% Convertir la grille OFDM frequentielle en signal temporel.
% Etapes: ifftshift pour recentrer le spectre, IFFT colonne par colonne,
% insertion du prefixe cyclique (CP) qui copie les N_CP derniers echantillons
% en tete de chaque symbole pour absorber l'effet du multitrajet,
% puis assemblage: zero padding | preambule | payload | zero padding.

	X_ifft_in = ifftshift(txContext.frequencyGridShift, 1);
	x_time = ifft(X_ifft_in, config.N_FFT, 1);

	x_cp = [x_time(end - config.N_CP + 1:end, :); x_time];
	payload = x_cp(:);

	timeSignal = [txContext.zeroPad; txContext.preamble; payload; txContext.zeroPad];

	fprintf('module_tx: IFFT + CP termines (%d echantillons payload).\n', numel(payload));
	fprintf('module_tx: preambule insere (%d echantillons) + zero padding (%d + %d). Trame TX totale: %d echantillons.\n', ...
		numel(txContext.preamble), numel(txContext.zeroPad), numel(txContext.zeroPad), numel(timeSignal));
end


function txBuffer = prepare_tx_buffer(timeSignal, ~)
% Normaliser le signal temporel pour eviter la saturation de l'amplificateur TX.
% Le facteur 0.8 conserve une marge de 20% sous le plein echelle,
% ce qui reduit le risque d'ecrerage sans trop reduire la puissance utile.

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
% Generer le preambule OFDM place en tete de chaque trame.
% Le preambule est constitue de N_PREA repetitions d'un meme symbole OFDM,
% avec un pattern de signe +/- alternant entre chaque repetition.
% Les sous-porteuses actives sont modulees en BPSK avec un motif alterne (+1/-1),
% ce qui produit un signal periodique facilement identifiable en reception.
% Ce preambule sert a la fois a la synchronisation temporelle (STO)
% et a l'estimation de la reponse du canal (H[k]).

	% Generer le symbole de base du preambule: BPSK alterne sur les sous-porteuses actives.
	preamble_freq_shift = complex(zeros(config.N_FFT, 1));
	used_bins = mod(config.data_bins_shift, config.N_FFT) + 1;
	
	% Pattern BPSK alterne
	alt = (-1).^(0:numel(used_bins)-1).';
	preamble_freq_shift(used_bins) = complex(alt, 0);

	% IFFT + CP pour un symbole
	preamble_time = ifft(ifftshift(preamble_freq_shift, 1), config.N_FFT, 1);
	preamble_sym = [preamble_time(end - config.N_CP + 1:end); preamble_time];

	% Pattern d'alternance: +1 pour les repetitions impaires, -1 pour les paires.
	sign_pattern = repmat([1, -1], 1, ceil(config.N_PREA / 2));
	sign_pattern = sign_pattern(1:config.N_PREA);
	preamble = [];
	for k = 1:config.N_PREA
		preamble = [preamble; preamble_sym * sign_pattern(k)];
	end
end


function visualize_tx_signal(txBuffer, symbols, config)
% Afficher le signal TX genere: composantes I/Q, enveloppe et constellation.
% La fenetre d'affichage exclut les zones de zero padding pour ne montrer
% que la partie utile contenant le preambule et le payload.

	if isempty(txBuffer)
		warning('module_tx: aucun echantillon TX a visualiser.');
		return;
	end

	if isfield(config, 'sampleRateHz') && config.sampleRateHz > 0
		fs = double(config.sampleRateHz);
	else
		fs = 1;
	end

	% Calculer les indices de la fenetre utile (entre les deux zones de zero padding).
	pad_len = round((double(config.txZeroPadMs) / 1000) * fs);
	sym_len = double(config.N_FFT + config.N_CP);
	useful_len = double(config.N_PREA) * sym_len + double(config.N_SYM) * sym_len;

	idx_start = max(1, pad_len + 1);
	idx_end = min(numel(txBuffer), pad_len + useful_len);

	if idx_end < idx_start
		idx_start = 1;
		idx_end = numel(txBuffer);
	end

	s = double(txBuffer(idx_start:idx_end));
	n_plot = numel(s);
	t = (0:n_plot-1).' / fs;

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

	fprintf('module_tx: visualisation TX affichee entre padding (%d echantillons, indices %d:%d).\n', ...
		n_plot, idx_start, idx_end);
end
