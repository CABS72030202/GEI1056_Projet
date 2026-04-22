% setup_bladerf_test.m
% Script minimal pour ouvrir puis fermer un bladeRF proprement,
% meme si ce script est execute hors du dossier d'installation bladeRF.

% Chemins de l'installation bladeRF (a adapter si besoin)
bladeRFRoot = 'C:\Program Files\bladeRF';
bladeRFMatlabDir = fullfile(bladeRFRoot, 'matlab');

if ~isfolder(bladeRFMatlabDir)
	error('Dossier MATLAB bladeRF introuvable: %s', bladeRFMatlabDir);
end

% Rendre le wrapper MATLAB visible
addpath(bladeRFMatlabDir);

% Aider Windows a trouver bladeRF.dll et ses dependances
currentPath = getenv('PATH');
if isempty(strfind(lower(currentPath), lower(bladeRFRoot)))
	setenv('PATH', [bladeRFRoot ';' currentPath]);
end

b = [];

try
	% Ouvre le premier bladeRF detecte
	b = bladeRF();
	fprintf('bladeRF ouvert avec succes.\n');
	fprintf('  Serial : %s\n', strtrim(b.info.serial));

catch ME
	fprintf(2, 'Erreur a l''ouverture du bladeRF:\n%s\n', ME.message);
end

% Fermeture propre (si ouvert)
if ~isempty(b)
	try
		delete(b);
		clear b;
		fprintf('bladeRF ferme proprement.\n');
	catch MEclose
		fprintf(2, 'Erreur a la fermeture du bladeRF:\n%s\n', MEclose.message);
	end
end

