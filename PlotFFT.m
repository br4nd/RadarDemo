function PlotFFT(s)
persistent h

dt = 61e-12;
Fs = 1/dt;

%Fs = 1/61e-12;
%dt = 1/Fs;

N = length(s);
%t = (0:dt:dt*N-dt);

S = fftshift(fft(s));

dF = Fs / N;
%f = -Fs/2:dF:Fs/2-dF;
f = [0:dF:Fs/2-dF]/1e9;

if isempty(h)  % On first call set up the plot structure

  h = plot(f,abs(S(end/2:end-1))/N);
  xlabel('Frequency (GHz)','FontSize',20,'FontWeight','bold')
  title('FFT','FontSize',30,'FontWeight','bold')
  grid on;
  axis([1,7,-.01,.1,])
  set(gca, 'FontSize',16,'FontWeight','bold')

else  
  set(h,'Ydata',abs(S(end/2:end-1))/N);
end