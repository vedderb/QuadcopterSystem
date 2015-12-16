GEN_LEN = 200;
PULSE_LEN = GEN_LEN;
SAMP_SPEED = 200000;
F_ULTRA = 40000;
TIM_PERIOD = (((168000000 / 2 / SAMP_SPEED) - 1));

sinc_tmp =  [-GEN_LEN/2:GEN_LEN/2] * pi * 2.0 / (GEN_LEN/2);
x = [-PULSE_LEN/2:PULSE_LEN/2] * pi * PULSE_LEN * (F_ULTRA / SAMP_SPEED) / (PULSE_LEN/2);

y = sin(sinc_tmp) ./ sinc_tmp;
y(find(isnan(y) == 1)) = 1;

%s = [y y -y -y];
s = y;

%pulse = (s .* sin(x) + 0.5) .* TIM_PERIOD * 0.5;
%pulse = (s .* sin(1:length(s)) + 0.5) .* TIM_PERIOD * 0.5;
pulse = (s .* sin(1:length(s)) + 1.0) .* TIM_PERIOD * 0.5;

%pulse = pulse ./ max(pulse);

%pulse = pulse ./ 210.0 - 0.5;

r = randn(1,2e4)*0.1;
r(500:500+length(pulse)-1) = r(500:500+length(pulse)-1) + pulse;

rr = xcorr(r, y.*sin(1:length(y)));
plot(rr)
find(rr == max(rr))


%a = sinc(0:0.01:1*pi);
%y = [fliplr(a) a];
%x = randn(1,1e4)*0.1;
%x(2000:2000+length(a)-1) = x(2000:2000+length(a)-1) + a;
%plot(xcorr(x,y))
