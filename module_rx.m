function rxContext = module_rx(rawSamples, config, txContext)
%MODULE_RX Chaine de reception OFDM complete.
% Prend en entree les echantillons bruts captures par la radio et retourne
% une structure rxContext contenant tous les resultats intermediaires.
% Pipeline RX:
% 1) Normalisation + metriques
% 2) Synchronisation STO sur preambule
% 3) Extraction preambule/payload
% 4) Estimation et correction CFO
% 5) Estimation canal + offset CP optimal
% 6) Egalisation + correction CPE
% 7) Demodulation, BER, EVM et SNR

	rxContext = struct();
	rxContext.config = config;
	rxContext.rawSamples = [];
	rxContext.displaySamples = [];
	rxContext.signalMetrics = struct();
	rxContext.sync = struct();
	rxContext.sections = struct();
	rxContext.cfo = struct();
	rxContext.channel = struct();
	rxContext.equalization = struct();
	rxContext.cpe = struct();
	rxContext.demod = struct();
	rxContext.alignedFrame = [];

	% 1) Echantillons RX et metriques
	rxContext.rawSamples = normalize_rx_samples(rawSamples, config);
	rxContext.displaySamples = rxContext.rawSamples;
	rxContext.signalMetrics = analyze_rx_signal(rxContext.displaySamples);

	% 2) Synchronisation STO sur le preambule TX connu
	preamble = extract_tx_preamble(txContext);
	rxContext.sync = detect_preamble_sto(rxContext.rawSamples, preamble);
	rxContext.alignedFrame = align_frame_on_sync(rxContext.rawSamples, rxContext.sync.startIndex);

	% 3) Extraction preambule + payload selon la synchro detectee
	rxContext.sections = extract_rx_sections(rxContext.rawSamples, rxContext.sync, preamble, config);
	if ~rxContext.sections.isValid
		warning('module_rx:sections', 'Extraction RX incomplete: pipeline de demodulation interrompu.');
		return;
	end

	% Visualisation de la correlation de detection et de l'alignement STO
	visualize_sto_results(rxContext.rawSamples, rxContext.alignedFrame, rxContext.sync, ...
		numel(preamble) + rxContext.sections.payloadExpectedLen, config);

	% 4) CFO (global) + correction preambule/payload
	rxContext.cfo = estimate_and_correct_cfo(rxContext.sections.rxPreamble, rxContext.sections.rxPayload, config);

	% Visualisation du signal apres correction du CFO
	alignedFrameCFO = [rxContext.cfo.rxPreambleCFO; rxContext.cfo.rxPayloadCFO];
	visualize_cfo_results(alignedFrameCFO, numel(preamble) + rxContext.sections.payloadExpectedLen, config);

	% 5) Estimation de canal sur le preambule et offset CP optimal
	rxContext.channel = estimate_channel_and_offset(rxContext.cfo.rxPreambleCFO, rxContext.cfo.rxPayloadCFO, config);
	if ~rxContext.channel.isValid
		warning('module_rx:channel', 'Estimation canal impossible: pipeline de demodulation interrompu.');
		return;
	end

	% 6) Egalisation frequencielle + CFO residuel symbole + CPE
	rxContext.equalization = equalize_payload(rxContext.cfo.rxPayloadCFO, rxContext.channel, config);
	rxContext.cpe = apply_cpe_correction(rxContext.equalization.rxEq, txContext);

	% 7) Demodulation QAM et BER
	rxContext.demod = demodulate_and_compute_ber(rxContext.cpe.rxQAM, txContext, config);

	% Visualisations de fin de chaine
	visualize_rx_post_equalization(rxContext.equalization.Y_used, rxContext.cpe.rxQAM, rxContext.channel.H_used_final);
end


function normalizedSamples = normalize_rx_samples(rawSamples, config)
% Convertir les echantillons RX en vecteur colonne complexe utilisable.
% Si aucun echantillon n'est fourni, un vecteur nul de secours est cree
% afin d'eviter les erreurs en aval de la chaine.

	if nargin < 1 || isempty(rawSamples)
		normalizedSamples = complex(zeros(config.N_FFT + config.N_CP, 1));
		fprintf('module_rx: echantillons RX absents, buffer de secours (%d echantillons).\n', ...
			numel(normalizedSamples));
		return;
	end

	normalizedSamples = rawSamples(:);
	fprintf('module_rx: %d echantillons fournis pour traitement RX.\n', numel(normalizedSamples));
end


function signalMetrics = analyze_rx_signal(rxBuffer)
% Calculer des indicateurs de puissance du signal recu.
% La moyenne, la valeur RMS et le pic de |s| permettent de verifier
% rapidement si le signal capture contient un burst utile ou du bruit.

	abs_buffer = abs(double(rxBuffer(:)));
	signalMetrics = struct();
	if isempty(abs_buffer)
		signalMetrics.meanAbs = NaN;
		signalMetrics.maxAbs = NaN;
		signalMetrics.rmsAbs = NaN;
		warning('module_rx:metrics', 'Buffer RX vide: metriques non disponibles.');
		return;
	end

	signalMetrics.meanAbs = mean(abs_buffer);
	signalMetrics.maxAbs = max(abs_buffer);
	signalMetrics.rmsAbs = rms(abs_buffer);

	fprintf('module_rx: mean|s|=%.4f, rms|s|=%.4f, max|s|=%.4f.\n', ...
		signalMetrics.meanAbs, signalMetrics.rmsAbs, signalMetrics.maxAbs);

	if signalMetrics.maxAbs < 0.05
		warning('module_rx:weakSignal', ...
			['Signal RX tres faible: la capture ressemble au bruit. ' ...
			 'Verifier le loopback/cablage RF ou la synchro TX/RX.']);
	end
end


function preamble = extract_tx_preamble(txContext)
% Extraire le preambule de la structure de contexte TX.
% Le preambule est utilise comme reference pour la synchronisation
% et l'estimation du canal en reception.

	preamble = [];
	if nargin < 1 || ~isstruct(txContext)
		return;
	end

	if isfield(txContext, 'preamble') && ~isempty(txContext.preamble)
		preamble = txContext.preamble(:);
	end
end


function alignedFrame = align_frame_on_sync(rxSamples, startIndex)
% Decouper la capture RX a partir de l'indice de synchronisation detecte.
% Le vecteur retourne commence au debut de la trame OFDM estimee.

	alignedFrame = [];
	if isempty(rxSamples)
		return;
	end

	idx = max(1, round(double(startIndex)));
	if idx > numel(rxSamples)
		warning('module_rx:syncRange', 'startIndex hors plage (%d > %d).', idx, numel(rxSamples));
		return;
	end

	alignedFrame = rxSamples(idx:end);
end


function sections = extract_rx_sections(rxSamples, syncContext, preamble, config)
% Decouper la capture RX en deux blocs: le preambule et le payload.
% L'indice de synchronisation indique le debut du preambule; le payload
% suit immediatement apres, sur N_SYM * (N_FFT + N_CP) echantillons.

	sections = struct();
	sections.isValid = false;
	sections.startIndex = 1;
	sections.rxPreamble = [];
	sections.rxPayload = [];
	sections.payloadStart = 1;
	sections.payloadEnd = 0;
	sections.payloadExpectedLen = 0;

	if isempty(rxSamples) || isempty(preamble)
		warning('module_rx:sections', 'rxSamples ou preambule vide.');
		return;
	end

	if ~isfield(syncContext, 'startIndex') || isempty(syncContext.startIndex)
		warning('module_rx:sections', 'syncContext.startIndex absent.');
		return;
	end

	startIndex = max(1, round(double(syncContext.startIndex)));
	Lpre = numel(preamble);
	sym_len = config.N_FFT + config.N_CP;
	payloadExpectedLen = config.N_SYM * sym_len;
	payloadStart = startIndex + Lpre;
	payloadEnd = payloadStart + payloadExpectedLen - 1;

	if startIndex + Lpre - 1 > numel(rxSamples)
		warning('module_rx:sections', 'Pas assez d''echantillons pour extraire le preambule RX.');
		return;
	end

	if payloadEnd > numel(rxSamples)
		warning('module_rx:sections', ...
			'Pas assez d''echantillons apres synchro (%d requis, %d disponibles).', payloadEnd, numel(rxSamples));
		return;
	end

	sections.startIndex = startIndex;
	sections.rxPreamble = rxSamples(startIndex : startIndex + Lpre - 1);
	sections.rxPayload = rxSamples(payloadStart : payloadEnd);
	sections.payloadStart = payloadStart;
	sections.payloadEnd = payloadEnd;
	sections.payloadExpectedLen = payloadExpectedLen;
	sections.isValid = true;

	fprintf('module_rx: extraction OK (preambule %d echantillons, payload %d echantillons).\n', ...
		numel(sections.rxPreamble), numel(sections.rxPayload));
end


function cfoContext = estimate_and_correct_cfo(rxPreamble, rxPayload, config)
% Estimer le decalage de frequence porteuse (CFO) et corriger le signal.
% Methode CP (Moose): le prefixe cyclique est une copie exacte de la fin
% du symbole OFDM. Leur correlation donne un angle = 2*pi*eps*N_FFT,
% d'ou eps_hat = angle(...) / (2*pi*N_FFT), avec eps = CFO / fs.
% La correction est appliquee en multipliant par exp(-j*2*pi*eps_hat*n).

	cfoContext = struct();
	cfoContext.eps_list = [];
	cfoContext.eps_hat = 0;
	cfoContext.CFO_Hz = 0;
	cfoContext.rxPreambleCFO = rxPreamble;
	cfoContext.rxPayloadCFO = rxPayload;
	cfoContext.nSymUsed = 0;
	cfoContext.isValid = false;

	if isempty(rxPayload)
		warning('module_rx:cfo', 'Payload RX vide: estimation CFO impossible.');
		return;
	end

	sym_len = config.N_FFT + config.N_CP;
	n_sym = floor(numel(rxPayload) / sym_len);
	if n_sym < 1
		warning('module_rx:cfo', 'Payload trop court pour estimer le CFO.');
		return;
	end

	rxPayload_forCFO = rxPayload(1:n_sym * sym_len);
	rxSymsCP = reshape(rxPayload_forCFO, sym_len, n_sym);

	eps_list = zeros(n_sym, 1);
	for k = 1:n_sym
		r = rxSymsCP(:, k);
		cp_part = r(1:config.N_CP);
		tail_part = r(config.N_FFT+1 : config.N_FFT+config.N_CP);
		phase_val = sum(conj(cp_part) .* tail_part);
		eps_list(k) = angle(phase_val) / (2*pi*config.N_FFT);
	end

	eps_hat = mean(eps_list);
	if isfield(config, 'sampleRateHz') && config.sampleRateHz > 0
		CFO_Hz = eps_hat * config.sampleRateHz;
	else
		CFO_Hz = eps_hat;
	end

	n_cfopreamble = (0:numel(rxPreamble)-1).';
	n_cfopayload = (0:numel(rxPayload)-1).';
	rxPreambleCFO = rxPreamble .* exp(-1j*2*pi*eps_hat*n_cfopreamble);
	rxPayloadCFO = rxPayload .* exp(-1j*2*pi*eps_hat*n_cfopayload);

	cfoContext.eps_list = eps_list;
	cfoContext.eps_hat = eps_hat;
	cfoContext.CFO_Hz = CFO_Hz;
	cfoContext.rxPreambleCFO = rxPreambleCFO;
	cfoContext.rxPayloadCFO = rxPayloadCFO;
	cfoContext.nSymUsed = n_sym;
	cfoContext.isValid = true;

	fprintf('module_rx: CFO estime par CP = %.2f Hz.\n', CFO_Hz);
end


function channelContext = estimate_channel_and_offset(rxPreambleCFO, rxPayloadCFO, config)
% Estimer la reponse frequentielle du canal H[k] et trouver l'offset CP optimal.
% Estimation LS: H[k] = Y[k] / X[k], avec X[k] le preambule connu et Y[k]
% la DFT du signal recu sur les sous-porteuses utiles.
% L'offset CP optimal est recherche par balayage de 0 a N_CP-1: on retient
% l'offset qui maximise mean(|mean(Yeq, symboles)|), indicateur de coherence
% inter-symbole sur le payload equalise.

	channelContext = struct();
	channelContext.bestOffset = 0;
	channelContext.bestMetric = -inf;
	channelContext.H_used_final = [];
	channelContext.metrics = [];
	channelContext.isValid = false;

	sym_len = config.N_FFT + config.N_CP;
	nPrea = floor(numel(rxPreambleCFO) / sym_len);
	nSym = floor(numel(rxPayloadCFO) / sym_len);

	if nPrea < 1 || nSym < 1
		warning('module_rx:channel', 'Donnees insuffisantes pour estimer le canal.');
		return;
	end

	rxPreambleCFO = rxPreambleCFO(1:nPrea * sym_len);
	rxPayloadCFO = rxPayloadCFO(1:nSym * sym_len);
	rxPreBlk = reshape(rxPreambleCFO, sym_len, nPrea);
	rxPaySyms = reshape(rxPayloadCFO, sym_len, nSym);

	[preamble_freq_shift, sign_pattern] = build_preamble_reference(config, nPrea);
	used_bins = get_used_bins(config);
	XtrainUsed = preamble_freq_shift(used_bins);

	for k = 1:nPrea
		rxPreBlk(:, k) = rxPreBlk(:, k) * sign_pattern(k);
	end

	bestMetric = -inf;
	bestOffset = 0;
	bestHUsed = [];
	metrics = -inf(config.N_CP, 1);

	for off = 0:config.N_CP-1
		H_list = [];
		for k = 1:nPrea
			pre_k = rxPreBlk(off+1 : off+config.N_FFT, k);
			Yk = fft(pre_k, config.N_FFT);
			Yk_shift = fftshift(Yk);
			Yk_used = Yk_shift(used_bins);
			Hk = Yk_used ./ (XtrainUsed + 1e-12);
			H_list = [H_list, Hk]; %#ok<AGROW>
		end

		H_used = mean(H_list, 2);

		x_win = rxPaySyms(off+1 : off+config.N_FFT, :);
		Y = fft(x_win, config.N_FFT, 1);
		Y_shift = fftshift(Y, 1);
		Y_used = Y_shift(used_bins, :);
		Yeq = Y_used ./ repmat(H_used + 1e-12, 1, size(Y_used, 2));
		metric = mean(abs(mean(Yeq, 1)));
		metrics(off+1) = metric;

		if metric > bestMetric
			bestMetric = metric;
			bestOffset = off;
			bestHUsed = H_used;
		end
	end

	channelContext.bestOffset = bestOffset;
	channelContext.bestMetric = bestMetric;
	channelContext.H_used_final = bestHUsed;
	channelContext.metrics = metrics;
	channelContext.isValid = ~isempty(bestHUsed);

	fprintf('module_rx: meilleur offset CP = %d (metric %.4f).\n', bestOffset, bestMetric);
end


function eqContext = equalize_payload(rxPayloadCFO, channelContext, config)
% Effectuer l'egalisation frequentielle du payload OFDM.
% Pour chaque symbole OFDM:
%   1) Estimation du CFO residuel symbole par symbole via le CP (Moose)
%   2) Correction en temps: x_corr[n] = x[n] * exp(-j*2*pi*eps_m*n)
%   3) Retrait du CP selon l'offset optimal, FFT, fftshift
%   4) Egalisation un-tap: Yeq[k] = Y[k] / H[k] (valide si le canal
%      est invariant dans la fenetre d'un symbole OFDM)

	eqContext = struct();
	eqContext.eps_sym = [];
	eqContext.rxPayloadSymsCFO = [];
	eqContext.Y_used = [];
	eqContext.rxEq = [];
	eqContext.isValid = false;

	if isempty(rxPayloadCFO) || ~channelContext.isValid
		warning('module_rx:eq', 'Egalisation impossible: donnees d''entree invalides.');
		return;
	end

	sym_len = config.N_FFT + config.N_CP;
	nSym = floor(numel(rxPayloadCFO) / sym_len);
	if nSym < 1
		warning('module_rx:eq', 'Payload trop court pour l''egalisation.');
		return;
	end

	rxPayloadCFO = rxPayloadCFO(1:nSym * sym_len);
	rxPayloadSyms = reshape(rxPayloadCFO, sym_len, nSym);

	rxPayloadSymsCFO = zeros(size(rxPayloadSyms));
	eps_sym = zeros(nSym, 1);

	for m = 1:nSym
		sym80 = rxPayloadSyms(:, m);
		cp_part = sym80(1:config.N_CP);
		tail_part = sym80(config.N_FFT+1 : config.N_FFT+config.N_CP);
		val = sum(conj(cp_part) .* tail_part);
		eps_m = angle(val) / (2*pi*config.N_FFT);
		eps_sym(m) = eps_m;
		n_sym = (0:sym_len-1).';
		rxPayloadSymsCFO(:, m) = sym80 .* exp(-1j*2*pi*eps_m*n_sym);
	end

	if isfield(config, 'sampleRateHz') && config.sampleRateHz > 0
		fprintf('module_rx: CFO residuel moyen par symbole = %.2f Hz.\n', mean(eps_sym) * config.sampleRateHz);
	end

	x_no_cp = rxPayloadSymsCFO(channelContext.bestOffset+1 : channelContext.bestOffset+config.N_FFT, :);
	Y_fft = fft(x_no_cp, config.N_FFT, 1);
	Y_fft_shift = fftshift(Y_fft, 1);
	used_bins = get_used_bins(config);
	Y_used = Y_fft_shift(used_bins, :);

	rxEq = Y_used ./ repmat(channelContext.H_used_final + 1e-12, 1, size(Y_used, 2));

	eqContext.eps_sym = eps_sym;
	eqContext.rxPayloadSymsCFO = rxPayloadSymsCFO;
	eqContext.Y_used = Y_used;
	eqContext.rxEq = rxEq;
	eqContext.isValid = true;
end


function cpeContext = apply_cpe_correction(rxEq, txContext)
% Corriger la derive de phase commune (CPE - Common Phase Error) symbole par symbole.
% La CPE est une rotation de phase identique sur toutes les sous-porteuses
% d'un meme symbole OFDM, causee par du bruit de phase ou un CFO residuel.
% Estimation: theta[m] = angle(sum(Z[:,m] .* conj(Xref[:,m])))
% Correction:  Z_corr[:,m] = Z[:,m] .* exp(-j*theta[m])

	cpeContext = struct();
	cpeContext.theta_hat = [];
	cpeContext.rxEqCPE = rxEq;
	cpeContext.rxQAM = rxEq(:);
	cpeContext.isValid = ~isempty(rxEq);

	if isempty(rxEq)
		warning('module_rx:cpe', 'Signal egalise vide: correction CPE sautee.');
		return;
	end

	if ~isfield(txContext, 'parallelSymbols') || isempty(txContext.parallelSymbols)
		warning('module_rx:cpe', 'Reference TX absente: CPE non corrigee.');
		return;
	end

	Z = rxEq;
	Xref = txContext.parallelSymbols;
	nSym = min(size(Z, 2), size(Xref, 2));
	nUsed = min(size(Z, 1), size(Xref, 1));

	Z = Z(1:nUsed, 1:nSym);
	Xref = Xref(1:nUsed, 1:nSym);

	theta_hat = angle(sum(Z .* conj(Xref), 1) + 1e-12);
	Z_cpe = Z .* repmat(exp(-1j * theta_hat), size(Z, 1), 1);

	cpeContext.theta_hat = theta_hat;
	cpeContext.rxEqCPE = Z_cpe;
	cpeContext.rxQAM = Z_cpe(:);
	cpeContext.isValid = true;

	fprintf('module_rx: CPE estime sur %d symboles OFDM.\n', nSym);
end


function demodContext = demodulate_and_compute_ber(rxQAM, txContext, config)
% Demoduler les symboles QAM recus et calculer BER, EVM et SNR.

	demodContext = struct();
	demodContext.rxInts = [];
	demodContext.rxBits = [];
	demodContext.totalBitsCompared = 0;
	demodContext.bitErrors = NaN;
	demodContext.BER = NaN;
	demodContext.hasReference = false;
	demodContext.evmRms = NaN;
	demodContext.evmPercent = NaN;
	demodContext.evmDb = NaN;
	demodContext.snrDb = NaN;
	demodContext.nSymbolsCompared = 0;
	demodContext.alpha = NaN;

	if isempty(rxQAM)
		warning('module_rx:demod', 'Signal RX vide: demodulation impossible.');
		return;
	end

	bits_per_sym = log2(config.M_ORDER);
	rxInts = qamdemod(rxQAM, config.M_ORDER, 'gray', 'OutputType', 'integer', 'UnitAveragePower', true);
	rxBitGroups = de2bi(rxInts, bits_per_sym, 'left-msb');
	rxBits = reshape(rxBitGroups.', [], 1);

	demodContext.rxInts = rxInts;
	demodContext.rxBits = rxBits;

	if isfield(txContext, 'bits') && ~isempty(txContext.bits)
		txBits = txContext.bits(:);
		nCompare = min(numel(txBits), numel(rxBits));
		if nCompare > 0
			bitErrors = sum(rxBits(1:nCompare) ~= txBits(1:nCompare));
			BER = bitErrors / nCompare;

			demodContext.totalBitsCompared = nCompare;
			demodContext.bitErrors = bitErrors;
			demodContext.BER = BER;
			demodContext.hasReference = true;

			fprintf('\n===== RESULTATS BER =====\n');
			fprintf('Bits compares = %d\n', nCompare);
			fprintf('Erreurs bits  = %d\n', bitErrors);
			fprintf('BER           = %.3e\n', BER);
		end
	else
		warning('module_rx:ber', 'Bits TX indisponibles: BER non calcule.');
	end

	% EVM et SNR: alignement par alpha avant mesure de l'erreur vectorielle.
	if isfield(txContext, 'symbols') && ~isempty(txContext.symbols)
		txSyms = txContext.symbols(:);
		rxSyms = rxQAM(:);
		nSymCompare = min(numel(txSyms), numel(rxSyms));

		if nSymCompare > 0
			txRef = txSyms(1:nSymCompare);
			rxRef = rxSyms(1:nSymCompare);

			alpha = sum(conj(txRef) .* rxRef) / (sum(abs(txRef).^2) + 1e-12);
			rxAligned = rxRef / (alpha + 1e-12);
			err = rxAligned - txRef;

			sigPow = mean(abs(txRef).^2);
			errPow = mean(abs(err).^2);

			evmRms = sqrt(errPow / (sigPow + 1e-12));
			evmDb = 20 * log10(evmRms + 1e-12);
			snrDb = 10 * log10((sigPow + 1e-12) / (errPow + 1e-12));

			demodContext.evmRms = evmRms;
			demodContext.evmPercent = 100 * evmRms;
			demodContext.evmDb = evmDb;
			demodContext.snrDb = snrDb;
			demodContext.nSymbolsCompared = nSymCompare;
			demodContext.alpha = alpha;

			fprintf('\n===== RESULTATS EVM / SNR =====\n');
			fprintf('Symboles compares = %d\n', nSymCompare);
			fprintf('alpha             = %.4f%+.4fj\n', real(alpha), imag(alpha));
			fprintf('EVM_rms           = %.4f\n', evmRms);
			fprintf('EVM               = %.2f %%\n', 100 * evmRms);
			fprintf('EVM(dB)           = %.2f dB\n', evmDb);
			fprintf('SNR estime        = %.2f dB\n', snrDb);
		end
	else
		warning('module_rx:evm', 'Symboles TX indisponibles: EVM/SNR non calcules.');
	end
end

function visualize_rx_post_equalization(Y_used, rxQAM, H_used_final)
% Afficher les constellations avant et apres egalisation, ainsi que le canal estime.
% Ces figures permettent d'evaluer visuellement la qualite de l'egalisation:
% les points de la constellation doivent se regrouper apres egalisation.

	if isempty(Y_used) || isempty(rxQAM) || isempty(H_used_final)
		return;
	end

	figure('Name', 'Constellation RX avant/apres egalisation', 'NumberTitle', 'off');

	subplot(1, 2, 1);
	scatter(real(Y_used(:)), imag(Y_used(:)), 10, 'filled', 'MarkerFaceAlpha', 0.35);
	grid on;
	axis equal;
	xlabel('I');
	ylabel('Q');
	title('Avant egalisation');
	xlim([-3 3]);
	ylim([-3 3]);

	subplot(1, 2, 2);
	scatter(real(rxQAM), imag(rxQAM), 10, 'filled', 'MarkerFaceAlpha', 0.35);
	grid on;
	axis equal;
	xlabel('I');
	ylabel('Q');
	title('Apres egalisation');
	xlim([-1.5 1.5]);
	ylim([-1.5 1.5]);

	figure('Name', 'Canal estime H[k]', 'NumberTitle', 'off');

	subplot(2, 1, 1);
	plot(abs(H_used_final), 'k', 'LineWidth', 1.2);
	grid on;
	xlabel('Indice sous-porteuse utile');
	ylabel('|H[k]|');
	title('Amplitude du canal estime');

	subplot(2, 1, 2);
	plot(angle(H_used_final), 'k', 'LineWidth', 1.2);
	grid on;
	xlabel('Indice sous-porteuse utile');
	ylabel('Phase H[k] (rad)');
	title('Phase du canal estime');
end


function visualize_sto_results(rxSamples, alignedFrame, syncContext, frameLength, config)
% Afficher les resultats de la synchronisation temporelle (STO).
% Deux figures: le profil de correlation ayant permis de detecter le debut
% de trame, et une comparaison du signal avant/apres l'alignement temporel.

	if isempty(rxSamples) || isempty(alignedFrame)
		return;
	end

	if nargin < 4 || isempty(frameLength) || frameLength <= 0
		frameLength = numel(alignedFrame);
	end

	if isfield(config, 'sampleRateHz') && config.sampleRateHz > 0
		fs = double(config.sampleRateHz);
	else
		fs = 1;
	end

	startIndex = 1;
	if isfield(syncContext, 'startIndex') && ~isempty(syncContext.startIndex)
		startIndex = max(1, round(double(syncContext.startIndex)));
	end

	if isfield(syncContext, 'corrMetric') && ~isempty(syncContext.corrMetric)
		figure('Name', 'RX - Detection preambule', 'NumberTitle', 'off');
		plot(syncContext.corrMetric, 'k');
		grid on;
		xlabel('Indice de correlation');
		ylabel('|Correlation|');
		title('Detection du preambule par correlation');
	end

	rxBefore = rxSamples(:);
	rxAfter = alignedFrame(:);
	nBefore = min(numel(rxBefore), frameLength + startIndex - 1);
	nAfter = min(numel(rxAfter), frameLength);

	if nBefore < 1 || nAfter < 1
		return;
	end

	sBefore = double(rxBefore(1:nBefore));
	sAfter = double(rxAfter(1:nAfter));
	tBefore = (0:nBefore-1).' / fs;
	tAfter = (0:nAfter-1).' / fs;

	figure('Name', 'Comparaison RX avant/apres alignement STO', ...
		'NumberTitle', 'off', ...
		'Position', [100, 80, 900, 700]);

	tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

	nexttile(tl);
	plot(tBefore, real(sBefore), 'b', tBefore, imag(sBefore), 'r');
	xline((startIndex-1)/fs, '--k', 'Debut detecte');
	grid on;
	xlabel('Temps (s)');
	ylabel('Amplitude');
	title('Avant alignement - I/Q');
	legend('I', 'Q', 'Debut de la trame', 'Location', 'northwest');
	ylim([-1 1]);

	nexttile(tl);
	plot(tAfter, real(sAfter), 'b', tAfter, imag(sAfter), 'r');
	grid on;
	xlabel('Temps depuis alignement (s)');
	ylabel('Amplitude');
	title('Apres alignement - I/Q');
	legend('I', 'Q');
	ylim([-1 1]);

	nexttile(tl);
	plot(tBefore, abs(sBefore), 'k');
	grid on;
	xlabel('Temps (s)');
	ylabel('|s(t)|');
	title('Avant alignement - enveloppe');
	ylim([0 1]);
	xline((startIndex-1)/fs, '--r', 'Debut detecte');

	nexttile(tl);
	plot(tAfter, abs(sAfter), 'k');
	grid on;
	xlabel('Temps depuis alignement (s)');
	ylabel('|s(t)|');
	title('Apres alignement - enveloppe');
	ylim([0 1]);

	fprintf('module_rx: comparaison avant/apres alignement affichee.\n');
end


function visualize_cfo_results(alignedFrameCFOcorr, frameLength, config)
% Afficher le signal apres correction du decalage de frequence porteuse (CFO).
% Permet de verifier que la correction CFO a bien stabilise la phase du signal.

	if isempty(alignedFrameCFOcorr)
		return;
	end

	if nargin < 2 || isempty(frameLength) || frameLength <= 0
		frameLength = numel(alignedFrameCFOcorr);
	end

	if isfield(config, 'sampleRateHz') && config.sampleRateHz > 0
		fs = double(config.sampleRateHz);
	else
		fs = 1;
	end

	rxAfterCFO = alignedFrameCFOcorr(:);
	nAfterCFO = min(numel(rxAfterCFO), frameLength);
	if nAfterCFO < 1
		return;
	end

	sAfterCFO = double(rxAfterCFO(1:nAfterCFO));
	tAfterCFO = (0:nAfterCFO-1).' / fs;

	figure('Name', 'RX apres correction CFO', ...
		'NumberTitle', 'off', ...
		'Position', [650, 80, 500, 900]);

	tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

	nexttile(tl);
	plot(tAfterCFO, real(sAfterCFO), 'b', tAfterCFO, imag(sAfterCFO), 'r');
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
	scatter(real(sAfterCFO), imag(sAfterCFO), 8, 'filled', 'MarkerFaceAlpha', 0.25);
	grid on;
	axis equal;
	xlim([-1 1]);
	ylim([-1 1]);
	xlabel('I');
	ylabel('Q');
	title('Constellation brute apres correction CFO');

	fprintf('module_rx: visualisation apres correction CFO terminee.\n');
end


function syncContext = detect_preamble_sto(rxSamples, preamble)
% Detecter le debut de la trame OFDM par correlation avec le preambule transmis.
% Un filtre adapte est construit a partir du preambule conjugue renverse;
% la convolution avec le signal recu produit un pic au moment ou les deux
% signaux sont alignes, ce qui donne l'estimation du STO.

    syncContext = struct();
    syncContext.startIndex = 1;
    syncContext.estimatedSTO = 0;
    syncContext.corrMetric = [];
    syncContext.peakValue = 0;

    if isempty(rxSamples) || isempty(preamble)
        warning('module_rx:sync', 'Impossible de detecter le preambule: rxSamples ou preambule vide.');
        return;
    end

    rxSamples = double(rxSamples(:));
    preamble  = double(preamble(:));

    if numel(rxSamples) < numel(preamble)
        warning('module_rx:sync','Capture RX plus courte que le preambule: detection impossible.');
        return;
    end

    % Construire le filtre adapte: h[n] = conj(preamble[-n])
    preambleConj = conj(preamble);
    preambleMatched = flipud(preambleConj);

    % Correlation glissante entre le signal RX et le filtre adapte
    corrComplex = conv(rxSamples, preambleMatched, 'valid');

    % Calculer la magnitude de la correlation (metrique de detection)
    corrMetric = abs(corrComplex);

    % L'indice du pic correspond au debut estime de la trame (STO)
    [peakValue, startIndex] = max(corrMetric);

    % Sauvegarde des resultats
    syncContext.startIndex = startIndex;
    syncContext.estimatedSTO = startIndex - 1;
    syncContext.corrMetric = corrMetric;
    syncContext.peakValue = peakValue;

    fprintf('module_rx: preambule detecte a l''echantillon %d.\n', startIndex);
    fprintf('module_rx: STO estime = %d echantillons.\n', syncContext.estimatedSTO);
    fprintf('module_rx: pic de correlation = %.4e.\n', peakValue);
end


function used_bins = get_used_bins(config)
% Convertir les indices de sous-porteuses en convention centree (DC=0)
% vers les indices MATLAB (base 1) correspondant a la sortie de fftshift.
% Exemple: indice -1 avec N_FFT=64 -> mod(-1,64)+1 = 64.

	used_bins = mod(config.data_bins_shift, config.N_FFT) + 1;
end


function [preamble_freq_shift, sign_pattern] = build_preamble_reference(config, nPrea)
% Reconstruire la reference frequentielle du preambule cote RX.
% Le preambule TX est un symbole BPSK a alternance de signe (+1/-1) sur
% les sous-porteuses utiles; chaque repetition est multipliee par +1 ou -1
% alternativement. Cette fonction retourne le vecteur fftshift correspondant
% et le vecteur de signes, necessaires pour l''estimation LS du canal.

	preamble_freq_shift = complex(zeros(config.N_FFT, 1));
	used_bins = get_used_bins(config);
	alt = (-1).^(0:numel(used_bins)-1).';
	preamble_freq_shift(used_bins) = complex(alt, 0);

	sign_pattern = repmat([1, -1], 1, ceil(nPrea / 2));
	sign_pattern = sign_pattern(1:nPrea);
end

