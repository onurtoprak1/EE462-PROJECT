% ==== PLL Design
f0 = 50;           % rated frequency (Hz)
w0 = 2*pi*f0;  
A0 = 1;            % rated magnitude of input signal

zeta = 1/sqrt(2)              % damping ratio (0.5 - 1)
ts   = 0.1;                   % settling time (s)
tau  = ts/4.6;                % response time constant
wn   = 1/(tau*zeta)           % natural frequency
KP   = 2*zeta*wn;
Ti   = 2*zeta/wn;
Kp   = 2/A0 * KP              % P gain
Ki   = Kp/Ti                  % I gain

TP   = (pi^2/16)*(w0^2/(zeta*wn^3))

T      = 1/f0;
Tover4 = T/4;
wf     = 2*zeta*w0;

% ==== SOGI Parameters

ts_SOGI = 0.02;               % settling time of SOGI-QSG
k       = 9.2/w0/ts_SOGI      % gain of SOGI-QSG

td = 0;          % deactivation delay
dw = 2*pi*2.5;    % 2.5 Hz frequency disturbance
tw = td + 0.5;    % at tw

dphi = pi/4;      % pi/4 rad phase disturbance
tphi = td + 0.25; % at tphi

dA = 0.5;        % 0.5 pu magniture disturbance
tA = td + 0.75;  % at tA0

wh = w0 * 5;
Ah = A0/5;
th = td + 1;

