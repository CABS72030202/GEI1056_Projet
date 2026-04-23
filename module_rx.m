function rxContext = module_rx(rawSamples, config, txContext) %#ok<INUSD>
%MODULE_RX Structure de la chaine de reception OFDM.
% Version actuelle: reception de diagnostic (normalisation + metriques + affichage).
% Les etapes de synchro fine/demodulation pourront etre ajoutees ici.

	rxContext = struct();
	rxContext.config = config;
	rxContext.rawSamples = [];
	rxContext.displaySamples = [];

	% 1) Recuperer les echantillons bruts deja recus par module_ofdm.
	rxContext.rawSamples = normalize_rx_samples(rawSamples, config);
	rxContext.displaySamples = rxContext.rawSamples;
	rxContext.signalMetrics = analyze_rx_signal(rxContext.displaySamples);

	% 2) Affichage de diagnostic RX (identique au style TX).
	visualize_rx_signal(rxContext.displaySamples, config);
end


function normalizedSamples = normalize_rx_samples(rawSamples, config)
% Normaliser l'entree RX pour garantir un vecteur complexe exploitable.
% Si l'entree est vide, un buffer de secours est genere.

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
% Calculer quelques indicateurs simples pour distinguer bruit et burst utile.
% Metriques de debug: moyenne, RMS et pic de |s|.

	abs_buffer = abs(double(rxBuffer(:)));
	signalMetrics = struct();
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


function visualize_rx_signal(rxBuffer, config)
% Visualisation simple du signal RX (I/Q + module + constellation).
% La fenetre d'affichage est centree automatiquement sur le burst d'energie maximale.
% Cela evite d'afficher surtout du silence sur des captures longues.

	if isempty(rxBuffer)
		warning('module_rx: aucun echantillon RX a visualiser.');
		return;
	end

	if isfield(config, 'sampleRateHz') && config.sampleRateHz > 0
		fs = double(config.sampleRateHz);
	else
		fs = 1;
	end

	n_total = numel(rxBuffer);
	n_plot = min(n_total, 2000);

	% Localiser la zone utile via une energie glissante.
	win = min(256, floor(n_total / 4));
	energy = filter(ones(win, 1) / win, 1, abs(double(rxBuffer(:))).^2);
	[~, peak_idx] = max(energy);
	burst_center = max(1, peak_idx - win);
	plot_start = max(1, burst_center - floor(n_plot / 4));
	plot_end   = min(n_total, plot_start + n_plot - 1);
	plot_start = max(1, plot_end - n_plot + 1);

	fprintf('module_rx: affichage centre sur le burst (echantillons %d a %d).\n', plot_start, plot_end);

	t = (plot_start-1 : plot_end-1).' / fs;
	s = double(rxBuffer(plot_start:plot_end));

	figure('Name', 'RX Signal Visualization', 'NumberTitle', 'off', ...
		'Position', [530, 80, 500, 900]);
	tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

	nexttile(tl);
	plot(t, real(s), 'b', t, imag(s), 'r');
	grid on;
	xlabel('Temps (s)');
	ylabel('Amplitude');
	legend('I', 'Q');
	title('Signal RX (I/Q)');
    ylim([-1 1]);

	nexttile(tl);
	plot(t, abs(s), 'k');
	grid on;
	xlabel('Temps (s)');
	ylabel('|s(t)|');
	title('Enveloppe du signal RX');
    ylim([0 1]);

	ax_const = nexttile(tl, [2 1]); %#ok<NASGU>
	const_start = max(1, burst_center);
	const_end   = min(n_total, const_start + 2999);
	scatter(real(rxBuffer(const_start:const_end)), imag(rxBuffer(const_start:const_end)), 10, 'filled', ...
		'MarkerFaceAlpha', 0.35);
	title('Constellation RX');
	grid on;
	axis equal;
	xlabel('In-phase (I)');
	ylabel('Quadrature (Q)');
	xlim([-1 1]);
	ylim([-1 1]);

	fprintf('module_rx: visualisation RX affichee (%d echantillons autour du burst).\n', plot_end - plot_start + 1);
end
