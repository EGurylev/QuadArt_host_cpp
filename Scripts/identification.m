pkg load control

%% Tune ARX model and compare it with analytical model
% for pitch control channel

% Read log file_in_loadpath
filename = "Log_Mon Sep 26 19:00:24 2016.csv";

data = csvread(filename);
num_rec = size(data, 1) - 2;
% Read fieldnames
ret = textread(filename, "%s", 2);
fieldnames = strsplit(ret{2}, ",");

time = data(3:end, find(strcmp(fieldnames, "time"))) / 1e6;% ms
time_cf = data(3:end, find(strcmp(fieldnames, "time_cf"))) / 1e3;
pitch_cf = data(3:end, find(strcmp(fieldnames, "pitch_cf")));
roll_cf = data(3:end, find(strcmp(fieldnames, "roll_cf")));
pitch_set = data(3:end, find(strcmp(fieldnames, "pitch_set")));
roll_set = data(3:end, find(strcmp(fieldnames, "roll_set")));

% Synchronize time from PC with time from CF
% Find moment in time when cf is connected and send real data
for i = 1:num_rec
    if roll_cf(i) != 0 && roll_cf(i - 1) == 0
        start_time = time(i);
        start_time_cf = time_cf(i);
    end
end

time_diff = start_time_cf - start_time;
time_cf -= time_diff;
mask = time_cf >= 0;
time_cf = time_cf(mask);
roll_cf = roll_cf(mask);
pitch_cf = pitch_cf(mask);

% Interpolate data
freq = 500;%Hz
dt = 1 / freq;
time_i = 0:dt:time(end);

[time_cf, i_before, i_after] = unique(time_cf);
pitch_cf = pitch_cf(i_before);

pitch_cf_i = interp1(time_cf, pitch_cf, time_i)';
pitch_set_i = interp1(time, pitch_set, time_i)';
pitch_cf_i(isnan(pitch_cf_i)) = 0;% zero padding
pitch_set_i(isnan(pitch_set_i)) = 0;% zero padding
%plot(time_i, pitch_set_i), hold on, grid on
%plot(time_i, pitch_cf_i, 'color', 'g', "LineWidth", 3)

% Tune 2-nd order ARX model
%pitch_cf_dat = iddata(pitch_cf_i, pitch_set_i, dt);
%[Tm, x0] = arx(pitch_cf_dat, 2);
%step(Tm / dcgain(Tm), 'color', 'r'), hold on
%[y, t, x] = lsim(Tm, pitch_set_i, time_i);
%plot(t, y, 'color', 'black', "LineWidth", 3)

mass = 0.029;
l = 0.045;
g = 9.81;
# Lookup tables for thrust
pwm_table = 650 * [0,6.25,12.5,18.75,25,31.25,37.5,43.25,50,56.25, ...
    62.5,68.75,75,81.25,87.5,93.75]; # in cf's thrust control range
rpm_table = [0,4485,7570,9374,10885,12277,13522,14691,15924,17174, ...
    18179,19397,20539,21692,22598,23882]; # revolutions per minute
thrust_table = [0,1.6,4.8,7.9,10.9,13.9,17.3,21.0,24.4,28.6,32.8, ...
    37.3,41.7,46.0,51.9,57.9] / 1e3; # kg

# Calc. rpm and pwm output for equilibrium point   
rpm_eq = interp1(thrust_table, rpm_table, mass);
pwm_eq = interp1(rpm_table, pwm_table, rpm_eq);

k1 = rpm_eq / pwm_eq;
k2 = mass * g / rpm_eq;
k3 = 2 * l;
k4 = 180 / pi;
K = k1 * k2 * k3 * k4;

% Cascade pid controllers
% Settings for pitch rate pid
pid_pitch_rate_kp = 250;
pid_pitch_rate_ki = 0;
pid_pitch_rate_kd = 0;
% Settings for pitch pid
pid_pitch_kp = 20;
pid_pitch_ki = 0;
pid_pitch_kd = 0;

% Complementary (Mahony) filter
Kp = 0.4;
dtf = 1 / 250;
% Coefficient for accelaration imu data
alpha = Kp * dtf;
% Low-pass filter (CF firmware) for accelaration
attenuation = freq / (2 * pi * 4);
attenuation = int32(bitshift(1, 8) / attenuation + 0.5);
tau = dt * ((1 - alpha) / alpha);
lpfc = tf(alpha, [tau 1]);
imu = parallel(lpfc, 1 - alpha);
imu = 1; % filter dynamics should be neglected

% First inner loop (pitch rate)
I = 1.4e-5;
sys = tf(K / I, [1 0]);
C1 = pid(pid_pitch_rate_kp, pid_pitch_rate_ki / dt, pid_pitch_rate_kd * dt);
OL1 = sys * C1;
CL1 = feedback(OL1);

% Second inner loop (pitch)
C2 = pid(pid_pitch_kp, pid_pitch_ki / dt, pid_pitch_kd * dt);
OL2 = imu * C2 * CL1 * tf(1, [1 0]);
CL2 = feedback(OL2);
%[y1, t, x] = lsim (CL2, pitch_set_i, time_i);
%step(Tcl)
%plot(time_i, y1, 'color', 'r', "LineWidth", 3)
%legend('Pitch set', 'Pitch cf', 'Pitch arx', 'Pitch analytic')
%xlabel('time, sec')
%ylabel('Pitch, grad')

% Outer loop (position x)
dt_host = 0.01;
pid_x_kp = 1;
pid_x_ki = 0.2;
pid_x_kd = 1.5;
alpha = 0.05;
tau = dt * ((1 - alpha) / alpha);
lpfc3 = tf(1, [tau 1]);

C3 = pid(pid_x_kp, pid_x_ki / dt_host, pid_x_kd * dt_host) * lpfc3;
OL3 = CL2 * C3;
CL3 = feedback(OL3);
step(CL3)

%x_real = data(3:end, find(strcmp(fieldnames, "x")));
%x_set = data(3:end, find(strcmp(fieldnames, "x_set")));
%[x_sim, t, x_] = lsim(T3, x_set, time);
%plot(time, x_real), hold on
%plot(time, x_set, 'g')
%plot(time, x_sim, 'color', 'r', "LineWidth", 3)