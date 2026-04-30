# GEI1056 – Projet de session
**Implémentation en temps réel d'un transceiver SISO-OFDM sur BladeRF avec MATLAB**

Cours GEI1056 – Systèmes de télécommunications | UQTR | Hiver 2026

---

## Description

Ce projet implémente une chaîne complète de transmission **SISO-OFDM** en temps réel sur la plateforme SDR [Nuand BladeRF](https://www.nuand.com/), en utilisant MATLAB et le [Communications Toolbox Support Package for BladeRF 2.0.

La chaîne inclut :
- Génération de bits et modulation QAM (16-QAM par défaut)
- Construction de la trame OFDM (IFFT, préfixe cyclique, préambule)
- Synchronisation trame (STO), estimation et compensation du CFO
- Estimation du canal par pilotes, égalisation, démodulation et calcul du BER

**Paramètres OFDM par défaut**

| Paramètre | Valeur |
|---|---|
| Taille FFT (`N_FFT`) | 64 |
| Préfixe cyclique (`N_CP`) | 16 (N_FFT/4) |
| Sous-porteuses utilisées (`N_USED`) | 48 (N_FFT − N_VC) |
| Symboles OFDM par trame (`N_SYM`) | 12 |
| Modulation | 16-QAM |
| Fréquence d'échantillonnage | 2,5 MHz |
| Fréquence centrale | 2,45 GHz |

---

## Structure du dépôt

```
GEI1056_Projet/
├── double_bladeRF/          # Test RF câblé avec deux BladeRF (TX sur un PC, RX sur l'autre)
│   ├── demo.mp4             # Vidéo de démonstration du lien TX/RX entre deux BladeRF
│   ├── txModule.m           # Émetteur : génère et transmet la trame OFDM via BladeRF
│   └── rxModule.m           # Récepteur : capture et décode la trame OFDM via BladeRF
│
├── single_bladeRF/          # Test en loopback avec un seul BladeRF (TX et RX sur le même appareil)
│   ├── module_ofdm_single.m # Point d'entrée principal – orchestre TX, synchro et RX
│   ├── module_tx.m          # Module émetteur (bits → trame OFDM)
│   ├── module_rx.m          # Module récepteur (synchro → égalisation → BER)
│   └── bladerf_parallel_rx_worker.m  # Worker de réception parallèle
│
├── sim/                     # Simulation MATLAB bout-en-bout (sans matériel)
│   ├── sim.m                # Simulation OFDM complète sur canal AWGN (référence BER)
│   └── Lab02/ Lab03/        # Fonctions auxiliaires des laboratoires (CP, GI, CFO, STO…)
│
└── test/                    # Scripts de débogage et d'exploration
    ├── lab04_MATLAB.m       # Test de la chaîne TX/RX avec BladeRF (débogage)
    ├── Lineaire_OFDM.m      # Test linéarité de la chaîne OFDM
    └── setup_bladerf_test.m # Configuration et vérification de la connexion BladeRF
```

---

## Prérequis

- **MATLAB** R2019b
- **Communications Toolbox**
- **[Communications Toolbox Support Package for BladeRF 2.0](https://www.mathworks.com/matlabcentral/fileexchange/74591-communications-toolbox-support-package-for-bladerf-2-0?s_tid=srchtitle_site_search_2_Nuand%2520bladeRF)**
- Drivers BladeRF installés ([guide Windows](https://github.com/Nuand/bladeRF/wiki/Getting-Started%3A-Windows))
- Fonction `default_ofdm_config()` configurée avec les chemins locaux vers les drivers BladeRF (C:\Program Files\bladeRF par défaut).

---

## Utilisation

### 1. Loopback – un seul BladeRF (`single_bladeRF/`)

Connecter un BladeRF en loopback RF (câble SMA entre TX et RX avec atténuateur de 30 dB).  
Lancer dans MATLAB :

```matlab
cd single_bladeRF
module_ofdm_single
```

Le script ouvre la radio, émet la trame OFDM, capture les échantillons RX, puis affiche les métriques (constellation, BER, synchronisation).

---

### 2. Liaison RF câblée – deux BladeRF (`double_bladeRF/`)

Connecter les deux BladeRF par câble RF SMA avec atténuateur de 30 dB.

**PC 1 – Émetteur :**
```matlab
cd double_bladeRF
txModule
```

**PC 2 – Récepteur :**
```matlab
cd double_bladeRF
rxModule
```

> Lancer `txModule` avant `rxModule` : l'émetteur envoie plusieurs bursts sur une longue période, tandis que le récepteur ne capture que 2-3 trames complètes. Démarrer RX pendant que TX est en cours de transmission.

---

### 3. Simulation sans matériel (`sim/`)

```matlab
cd sim
sim
```

Génère les courbes BER vs SNR de référence en canal AWGN, sans BladeRF.

---

## Démonstration vidéo

Une vidéo de démonstration du fonctionnement de la transmission entre deux BladeRF est disponible ici :

[double_bladeRF/demo.mp4](double_bladeRF/demo.mp4)

---

## Montage RF câblé

La transmission RF est réalisée en **liaison câblée** entre deux BladeRF :

- Utiliser un **atténuateur RF** de 30 dB pour éviter la saturation du récepteur
- Ajuster le gain TX (`txGainDb`) et le gain RX (`rxGainDb`) selon l'atténuation
- Ne pas émettre en OTA (over-the-air) sans autorisation réglementaire