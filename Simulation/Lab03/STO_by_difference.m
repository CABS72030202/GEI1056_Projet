function [STO_est,Mag]=STO_by_difference(y,Nfft,Ng,com_delay)
% STO estimation by minimizing the difference between CP and rear part 
% of OFDM symbol estimates STO by minimizing the difference between CP 
% (cyclic prefix) and rear part of OFDM symbol
% Input:  y          = Received OFDM signal including CP
%         Ng         = Number of samples in CP (Guard Interval)
%         com_delay  = Common delay
% Output: STO_est    = STO estimate|STO
%         Mag        = Correlation function trajectory varying with time


N_ofdm=Nfft+Ng; minimum=100; STO_est=0;
if nargin<4, com_delay = N_ofdm/2; end
for n=1:N_ofdm
   nn = n+com_delay+[0:Ng-1]; 
   tmp0 = abs(y(nn))-abs(y(nn+Nfft));
   Mag(n) = tmp0*tmp0'; % Eq.(5.11) is strong against CFO
   if (Mag(n)<minimum)
     minimum=Mag(n);  STO_est = N_ofdm-com_delay -(n-1); 
   end
end
