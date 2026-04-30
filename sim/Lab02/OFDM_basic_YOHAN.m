%% =========================================================================
% OFDM_basic.m
% MIMO-OFDM Wireless Communications with MATLAB
% Yong Soo Cho, Jaekwon Kim, Won Young Yang and Chung G. Kang
% 2010 John Wiley & Sons (Asia) Pte Ltd
%
% Simulation OFDM SISO :
% - Canal AWGN (Ch = 0)
% - Canal multipath Rayleigh (Ch = 1)
% - Calcul BER vs Eb/N0

%NOTE: Le code a été modifié à des fins de compréhension
%% =========================================================================

clear all

% =========================================================================
% 1) CONFIGURATION GÉNÉRALE : TYPE DE GI ET TYPE DE CANAL
% =========================================================================

NgType=1; % NgType=1/2 pour cyclic prefix (CP) / zero padding (ZP)

if NgType==1
    nt='CP';
elseif NgType==2
    nt='ZP';
end

Ch=1;  % Ch=0/1 pour AWGN / multipath

if Ch==0
    chType='AWGN';
    Target_neb=100;      % Seuil d'erreurs pour arrêter la simulation
else
    chType='CH';
    Target_neb=500;
end

figure(Ch+1), clf

% =========================================================================
% 2) PROFIL DE CANAL MULTIPATH (VERSION ORIGINALE DU PROGRAMME)
% =========================================================================

PowerdB=[0 -8 -17 -21 -25]; % Puissance des taps en dB
Delay=[0 3 5 6 8];          % Délais des taps en nombre d’échantillons

Power=10.^(PowerdB/10);     % Conversion en échelle linéaire
Ntap=length(PowerdB);       % Nombre de trajets
Lch=Delay(end)+1;           % Longueur du canal discret

% =========================================================================
% 3) PARAMÈTRES OFDM ET MODULATION
% =========================================================================

Nbps=4;                     % Bits par symbole (16-QAM ici)
M=2^Nbps;                   % Ordre de modulation

Nfft=64;                    % Taille FFT

Ng=3;                       % (sera écrasé juste après)
Ng=Nfft/4;                  % Longueur du Guard Interval

Nsym=Nfft+Ng;               % Durée totale d’un symbole OFDM

Nvc=Nfft/4;                 % Nombre de sous-porteuses virtuelles
Nused=Nfft-Nvc;             % Nombre de sous-porteuses utiles

% =========================================================================
% 4) PARAMÈTRES AJOUTÉS POUR RESPECTER L'ÉNONCÉ (ITU-R + DOPPLER)
% =========================================================================

Ts = 100e-9;                % Période d’échantillonnage 100 ns
fc = 2.4e9;                 % Fréquence porteuse
vVehA = 100/3.6;                % Vitesse véhicule (m/s)
vPedA = 3/3.6;                % Vitesse véhicule (m/s)
c = 3e8;                    % Constante

fDPedA = (vPedA/c)*fc;
Tsym = Nsym * Ts;           % Durée d’un symbole OFDM

% =========================================================================
% 5) SÉLECTION MODÈLE DE CANAL ITU-R (TABLE 2.3 DU LIVRE)
% =========================================================================

chanModel = 'PedA';  % 'PedA' ou 'VehA'

switch chanModel
    case 'PedA'
        Delay_ns  = [0 110 190 410];
        PowerdB   = [0 -9.7 -19.2 -22.8];
        fD = (vPedA/c)*fc; 
    case 'VehA'
        Delay_ns  = [0 310 710 1090 1730 2510];
        PowerdB   = [0 -1.0 -9.0 -10.0 -15.0 -20.0];
        fD = (vVehA/c)*fc; 
end

td = Delay_ns*1e-9;      
tapIdx = round(td/Ts);      % Quantification des délais (Fig 2.19 du livre)
Power = 10.^(PowerdB/10);        % puissances linéaires (ITU-R)
Power = Power / sum(Power);      % normalisation
Ntap  = length(PowerdB);         % nb de tap
Lch   = max(tapIdx) + 1;         % longueur canal en échantillons

% =========================================================================
% 6) PARAMÈTRES DE SIMULATION BER
% =========================================================================

EbN0=[0:5:20];              % Gamme Eb/N0
N_iter=1e5;                 % Nombre max d’itérations par Eb/N0
Nframe=3;                   % Symboles OFDM par frame
sigPow=0;                   % Initialisation puissance signal

file_name = ['OFDM_BER_' chType '_' nt '_' chanModel '_TV.dat'];
fid = fopen(file_name, 'w+');

norms=[1 sqrt(2) 0 sqrt(10) 0 sqrt(42)];  % Facteurs normalisation constellation

% =========================================================================
% 7) BOUCLE PRINCIPALE SUR Eb/N0
% =========================================================================

for i=0:length(EbN0)

   randn('state',0);
   rand('state',0);

   Neb=0; Ntb=0;   % Initialisation erreurs / bits totaux

   for m=1:N_iter

      % ------------------------------------------------------------------
      % 7.1 ÉMETTEUR
      % ------------------------------------------------------------------

      X= randi(1,Nused*Nframe,M);     % Génération symboles
      Xmod= qammod(X,M,'gray')/norms(Nbps);

      if NgType~=2
          x_GI=zeros(1,Nframe*Nsym);
      elseif NgType==2
          x_GI= zeros(1,Nframe*Nsym+Ng);
      end

      kk1=[1:Nused/2]; 
      kk2=[Nused/2+1:Nused]; 
      kk3=1:Nfft; 
      kk4=1:Nsym;
      

      for k=1:Nframe

         if Nvc~=0
             X_shift= [0 Xmod(kk2) zeros(1,Nvc-1) Xmod(kk1)];
         else
             X_shift= [Xmod(kk2) Xmod(kk1)];
         end

         x= ifft(X_shift);

         x_GI(kk4)= guard_interval(Ng,Nfft,NgType,x);

         kk1=kk1+Nused; 
         kk2= kk2+Nused; 
         kk3=kk3+Nfft; 
         kk4=kk4+Nsym;
      end
      
        % ------------------------------------------------------------------
        % 7.2 CANAL
        % ------------------------------------------------------------------
        if Ch==0
            y = x_GI;   % AWGN
        else
            % -------------------------------------------------
            % Génération des coefficients Jakes (g) :
            % g(ksym, ell) = coefficient complexe du tap ell au symbole ksym
            % -------------------------------------------------
            g = zeros(Nframe, Ntap);
        
            for ell = 1:Ntap
                [h_temp, ~] = Jakes_Flat(fD, Tsym, Nframe, 0, Power(ell));
                g(:, ell) = h_temp.';  % colonne = tap ell
            end
        
            % -------------------------------------------------
            % Canal multipath variant par symbole OFDM
            % -------------------------------------------------
            y = zeros(1, Nframe*Nsym + Lch - 1);
            h_store = zeros(Nframe, Lch);   % stocker h_k pour égalisation parfaite
            

            for ksym = 1:Nframe
        
                % Construire h_k pour le symbole courant
                h_k = zeros(1, Lch);
                h_k(tapIdx + 1) = g(ksym, :);
        
                h_store(ksym, :) = h_k;
        
                % Extraire symbole temporel (avec GI)
                x_sym = x_GI((ksym-1)*Nsym + (1:Nsym));
        
                % Convolution
                y_sym = conv(x_sym, h_k);
        
                % Assemblage dans y (overlap-add)
                idx = (ksym-1)*Nsym + (1:(Nsym + Lch - 1));
                y(idx) = y(idx) + y_sym;
            end
        end

      % ------------------------------------------------------------------
      % 7.3 ESTIMATION PUISSANCE (i=0 seulement)
      % ------------------------------------------------------------------

      if i==0
        y1=y(1:Nframe*Nsym); 
        sigPow = sigPow + y1*y1';
        continue;
      end

      % ------------------------------------------------------------------
      % 7.4 AJOUT BRUIT AWGN
      % ------------------------------------------------------------------

      snr = EbN0(i)+10*log10(Nbps*(Nused/Nfft));
      noise_mag = sqrt((10.^(-snr/10))*sigPow/2);

      y_GI = y + noise_mag*(randn(size(y))+j*randn(size(y)));

      % ------------------------------------------------------------------
      % 7.5 RÉCEPTEUR
      % ------------------------------------------------------------------

      kk1=(NgType==2)*Ng+[1:Nsym]; 
      kk2=1:Nfft;
      kk3=1:Nused; 
      kk4=Nused/2+Nvc+1:Nfft; 
      kk5=(Nvc~=0)+[1:Nused/2];
      HH = zeros(1, Nfft*Nframe);   % Grand buffer pour stocker H par symbole

    for k=1:Nframe
    
        % FFT du symbole OFDM reçu (après retrait du GI)
        Y(kk2)= fft(remove_GI(Ng,Nsym,NgType,y_GI(kk1)));
    
        % Extraction + réordonnancement des sous-porteuses utiles
        Y_shift=[Y(kk4) Y(kk5)];
    
        % Si canal multipath (variant), calculer la réponse fréquentielle pour CE symbole
        if Ch==1
            h_k = h_store(k, :);
            H = fft([h_k zeros(1, Nfft-Lch)]);   % H local (1..Nfft)
            HH(kk2) = H;                         % stocker H dans un buffer global (comme Y)
            H_shift(kk3) = [HH(kk4) HH(kk5)];    % indices kk4/kk5 valides car HH est grand
        end
    
        % Égalisation / compensation canal
        if Ch==0
            Xmod_r(kk3) = Y_shift;              % AWGN
        else
            Xmod_r(kk3) = Y_shift./H_shift(kk3);     % Multipath : égalisation ZF
        end
    
        % Mise à jour des indices (logique originale du programme)
        kk1=kk1+Nsym; 
        kk2=kk2+Nfft; 
        kk3=kk3+Nused; 
        kk4=kk4+Nfft; 
        kk5=kk5+Nfft;
    
    end

      % ------------------------------------------------------------------
      % 7.6 DÉMODULATION + BER
      % ------------------------------------------------------------------

      X_r=qamdemod(Xmod_r*norms(Nbps),M,'gray');

      Neb=Neb+sum(sum(de2bi(X_r',Nbps)~=de2bi(X(:,1),Nbps)));

      Ntb=Ntb+Nused*Nframe*Nbps;

      if Neb>Target_neb
          break;
      end
   end

   % ---------------------------------------------------------------------
   % 7.7 AFFICHAGE DES RÉSULTATS
   % ---------------------------------------------------------------------

   if i==0

     sigPow= sigPow/Nsym/Nframe/N_iter;

     fprintf('Signal power= %11.3e\n', sigPow);
     fprintf(fid,'%%Signal power= %11.3e\n%%EbN0[dB]       BER\n', sigPow);

    else

     Ber = Neb/Ntb;     

     fprintf('EbN0=%3d[dB], BER=%4d/%8d =%11.3e\n', EbN0(i), Neb,Ntb,Ber)
     fprintf(fid, '%d\t%11.3e\n', EbN0(i), Ber);

     if Ber<1e-6
         break;
     end
   end
end

% =========================================================================
% 8) FIN DE SIMULATION
% =========================================================================

if (fid~=0)
    fclose(fid);
end

disp('Simulation is finished');


PedA = load('OFDM_BER_CH_CP_PedA_TV.dat');
VehA = load('OFDM_BER_CH_CP_VehA_TV.dat');

figure;
semilogy(PedA(:,1), PedA(:,2), '-s', 'LineWidth', 1.5); hold on;
semilogy(VehA(:,1), VehA(:,2), '-o', 'LineWidth', 1.5); grid on;
xlabel('EbN0[dB]'); ylabel('BER'); title('BER: ITU-R Channels');
legend('PedA','VehA','Location','southwest');
ylim([1e-3 1]);
plot_ber(file_name, Nbps);
