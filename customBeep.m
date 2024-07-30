%from https://www.mathworks.com/matlabcentral/answers/1597399-create-rapid-beeping-tone-in-matlab

function customBeep(beepSpacing, toneFreq)
    spaceduration = beepSpacing;
    tonefreq = toneFreq
    fs = 8192; %sample rate
    toneduration = 0.1;
    %spaceduration = 0.05;
    %tonefreq = 800;
    nbeeps = 2;
    t = linspace(0,toneduration,round(toneduration*fs));
    y = 0.8*sin(2*pi*tonefreq*t); % tone
    ys = zeros(1,round(spaceduration*fs)); % space
    Y = [repmat([y ys],[1 nbeeps-1]) y]; % the whole signal
    sound(Y,fs);
end