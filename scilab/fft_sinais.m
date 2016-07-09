% Rotina para calcular a FFT e o decremento logarítmico de um sinal
% temporal

sig = disp2;

L = length(disp);           % tamanho do sinal
T = t(2) - t(1);            % período de amostragem
Fs = 1/T;                   % frequência de amostragem

NFFT = 2 ^ nextpow2(L);
SIG = fft(sig,NFFT) / L;
displaySIG = 2 * abs(DISP(1:NFFT/2+1));

f = Fs / 2 * linspace(0,1,NFFT/2+1);

% Plotagem do da FFT
plot(f,displaySIG);
title('Espectro do sinal');
xlabel('Frequência [Hz]');
ylabel('Amplitude');
set(gca,'XLim',[0,10]);

% Encontra a frequência de maior amplitude
maxFreIndex = find(displaySIG == max(displaySIG));
maxFre = f(maxFreIndex);
fprintf('Frequência de pico: %6.2f Hz.\n',maxFre);

% Encontra o primeiro máximo
firstMax = find(sig == max(sig));
firstMaxTime = t(firstMax);

% Encontra o segundo máximo
foundSecondMax = false;

currentIndex = firstMax;
while ( ~foundSecondMax && currentIndex < L-2 )
    currentIndex = currentIndex + 1;
    if (sig(currentIndex) > sig(currentIndex-1) && ...
        sig(currentIndex) > sig(currentIndex+1))
        foundSecondMax = true;
    end
end

secondMax = sig(currentIndex);
secondMaxTime = t(currentIndex);
percievedPeriod = secondMaxTime - firstMaxTime;

% Calcula o decremento logarítmico com base nos dois primeiros máximos
logarithmicDecrement = log ( firstMax / secondMax);

% Calcula o coeficiente de amortecimento a partir do decremento logarítmico
zeta = 1 / sqrt ( 1 + ( 2 * pi / logarithmicDecrement ) ^ 2 );
fprintf('Coeficiente de amortecimento: %2.6f\n',zeta);

fprintf('Frequência natural corrigida: %6.2f Hz\n', 2*pi/percievedPeriod...
    / (1-zeta^2) / ( 2 * pi));