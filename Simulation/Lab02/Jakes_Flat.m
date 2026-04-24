function [h,t,phi_N] = Jakes_Flat(fd, Ts, Ns, t0, E0, phi_N)
%JAKES_FLAT  Flat (single-tap) Rayleigh fading process using Jakes/Clarke model
%
%   [h,t,phi_N] = Jakes_Flat(fd, Ts, Ns, t0, E0, phi_N)
%
% Inputs:
%   fd    : maximum Doppler frequency [Hz]
%   Ts    : sampling time [s]
%   Ns    : number of samples
%   t0    : initial time [s] (optional, default 0)
%   E0    : average channel power (optional, default 1)
%   phi_N : initial phase of the maximum Doppler sinusoid [rad] (optional, random)
%
% Outputs:
%   h     : complex fading vector (1 x Ns)
%   t     : time vector (1 x Ns)
%   phi_N : phase used (returned so you can continue the process in segments)
%
% Notes:
% - This implements a classical Jakes spectrum approximation using a sum-of-sinusoids.
% - The resulting h has approximately Rayleigh envelope and autocorrelation ~ J0(2*pi*fd*tau).
%
% Reference (conceptually): "MIMO-OFDM Wireless Communications with MATLAB", Cho et al.

    % -----------------------
    % Defaults / input checks
    % -----------------------
    if nargin < 3
        error('Jakes_Flat requires at least fd, Ts, Ns.');
    end
    if nargin < 4 || isempty(t0)
        t0 = 0;
    end
    if nargin < 5 || isempty(E0)
        E0 = 1;
    end
    if nargin < 6 || isempty(phi_N)
        phi_N = 2*pi*rand();  % random initial phase
    end

    if fd < 0 || Ts <= 0 || Ns <= 0
        error('Invalid parameters: fd>=0, Ts>0, Ns>0 required.');
    end

    % -----------------------
    % Time axis
    % -----------------------
    t = t0 + (0:Ns-1)*Ts;

    % -----------------------
    % Jakes parameters
    % -----------------------
    % "As suggested by Jakes": choose N0 large enough for accuracy
    N0 = 8;
    N  = 4*N0 + 2;                  % number of oscillators (for approximation)
    wd = 2*pi*fd;                   % max Doppler angular frequency [rad/s]

    % Angles for the oscillators
    n  = 1:N0;
    beta_n = pi*n/(N0+1);           % Jakes angles
    wn = wd*cos(2*pi*n/N);          % projected Doppler frequencies (one common variant)

    % Carrier-like term and phase shaping (variant used in many textbook codes)
    wc = wd*cos(pi/(2*N0+1));       % "center" term (helps symmetry)

    % -----------------------
    % Generate fading process
    % -----------------------
    % Real and imaginary parts built from cosine sums with quadrature
    % Using the structure often shown in textbook codes
    cosc = cos(wc*t + phi_N);       % 1 x Ns

    % Build sum over N0 oscillators
    % Each oscillator contributes to I and Q with different phase offsets
    % Phases phi_n are deterministic here (can be randomized, but this matches classic Jakes style)
    phi_n = pi*n/(N0+1);            % 1 x N0

    % Expand for vectorized computation: (N0 x Ns)
    tt = repmat(t, N0, 1);
    W  = repmat(wn(:), 1, Ns);
    PH = repmat(phi_n(:), 1, Ns);

    % In-phase and quadrature sums
    I = 2*sum(cos(W.*tt).*cos(PH), 1) + sqrt(2)*cos(wd*t); % 1 x Ns
    Q = 2*sum(sin(W.*tt).*sin(PH), 1) + sqrt(2)*sin(wd*t); % 1 x Ns

    % Complex fading, scaled to average power E0
    h = sqrt(E0/(2*(2*N0+1))) * (I + 1j*Q) .* cosc;

    % Ensure row vectors
    h = reshape(h, 1, []);
    t = reshape(t, 1, []);
end
