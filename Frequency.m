load 'flight data.mat'

flight = find(a.y.Time > 0.25 & a.y.Time < 3);
xvals = a.y.Time(flight); yvals = a.y.Data(flight);
plot(xvals, yvals);
figure(2);
Fs = 1/mean(diff(xvals));
T = 1/Fs;
L = length(yvals);
t = a.y.Time;

Y = fft(a.y.Data);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
loglogpl(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')