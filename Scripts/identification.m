pkg load control

%% Tune ARX model and compare it with analytical model
% for pitch control channel

% Read log file_in_loadpath
filename = "sine_pitch1.csv";

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
x_set = data(3:end, find(strcmp(fieldnames, "x_set")));
x = data(3:end, find(strcmp(fieldnames, "x")));
z = data(3:end, find(strcmp(fieldnames, "z")));

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
dt_cf = 1 / freq;
dt_host = 0.01;
time_i = 0:dt_host:time(end);

[time_cf, i_before, i_after] = unique(time_cf);
pitch_cf = pitch_cf(i_before);

pitch_cf_i = interp1(time_cf, pitch_cf, time_i)';
pitch_set_i = interp1(time, pitch_set, time_i)';
x_i = interp1(time, x, time_i)';
x_i(isnan(x_i)) = 0;% zero padding
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


%[y1, t, x] = lsim (CL2, pitch_set_i, time_i);
%step(Tcl)
%plot(time_i, y1, 'color', 'r', "LineWidth", 3)
%legend('Pitch set', 'Pitch cf', 'Pitch arx', 'Pitch analytic')
%xlabel('time, sec')
%ylabel('Pitch, grad')



%x_real = data(3:end, find(strcmp(fieldnames, "x")));
%x_set = data(3:end, find(strcmp(fieldnames, "x_set")));
%[x_sim, t, x_] = lsim(T3, x_set, time);
%plot(time, x_real), hold on
%plot(time, x_set, 'g')
%plot(time, x_sim, 'color', 'r', "LineWidth", 3)

%%% Analytical model from "DESIGN OF A TRAJECTORY TRACKING CONTROLLER FOR A NANOQUADCOPTER" 2016
%I = 1.4e-05;% Moment of inertia
%Ct = 3.1582e-10;% thrust coefficient
%d = 39.73e-3;% Arm length
%g = 9.81;
%mass = 0.029; % kg
%we = sqrt(mass * g / (4 * Ct)); %equilibrium rpm
%k1 = 0.2685; %PWM to PRM
%k2 = 180 / pi;
%K = k1 * k2 * we * 2 * sqrt(2) * d * Ct / I;
%sys1 = tf(K, [1 0]);
%
%% First inner loop (pitch rate)
%C1 = pid(pid_pitch_rate_kp, pid_pitch_rate_ki / dt, pid_pitch_rate_kd * dt);
%OL1 = sys1 * C1;
%CL1 = feedback(OL1);
%
%% Second inner loop (pitch)
%C2 = pid(pid_pitch_kp, pid_pitch_ki / dt, pid_pitch_kd * dt);
%OL2 = C2 * CL1 * tf(1, [1 0]);
%CL2 = feedback(OL2);


%step(CL3)

% Tune 2-nd order ARX model;


%time_i(idx) = [];
%pitch_cf_i(idx) = [];
%z_i(isnan(z_i)) = 0;% zero padding
%
%x_trig = x_i != 0;
%x_trig_d = diff(x_trig);
%x0 = x_i(find(x_trig_d, 1) + 1);
%
%x_i -= x0;
%idx = pitch_cf_i == 0;
%x_i(idx) = 0;


%



% Inner loops: pitch rate and pitch

% Cascade pid controllers
% Settings for pitch rate pid
pid_pitch_rate_kp = 250;
pid_pitch_rate_ki = 0;
pid_pitch_rate_kd = 0;
% Settings for pitch pid
pid_pitch_kp = 5;
pid_pitch_ki = 0;
pid_pitch_kd = 0;

pitch_dat = iddata(pitch_cf_i, pitch_set_i, dt_host);
[Tp, x0] = arx(pitch_dat, 2);
Tpc = tf(d2c(Tp));

[pitch_sim, t, i] = lsim(Tpc, pitch_set_i, time_i);
% Normilize dc gain to 1
Tpc /= dcgain(Tpc);
% Remove neglegible zero from transfer function
[num,den] = tfdata(Tpc);
num{1}(1) = [];
Tpc = tf(num, den);

% Calculate system model's dc gain
K_sys = num{1} / (pid_pitch_rate_kp * pid_pitch_kp);

% System model
sys = tf(K_sys, [1 0]);
%First inner loop (pitch rate)
C1 = pid(pid_pitch_rate_kp, pid_pitch_rate_ki / dt_cf, pid_pitch_rate_kd * dt_cf);
CL1 = feedback(sys * C1);
% Second inner loop (pitch)
C2 = pid(pid_pitch_kp, pid_pitch_ki / dt_cf, pid_pitch_kd * dt_cf);
CL2 = feedback(CL1 * C2 * tf(1, [1 0]));

a1 = subplot(2,1,1);
plot(time_i, pitch_set_i), hold on
plot(time_i, pitch_cf_i, 'g')
plot(time_i, pitch_sim, 'r')


% Outer loop (position x)
%pid_x_kp = 0.35;
%pid_x_ki = 0.5;
%pid_x_kd = 0.40;
%alpha = 0.25;
%tau = dt_host * ((1 - alpha) / alpha);
%lpfc3 = tf(1, [tau 1]);
%%lpfc3 = 1;
%C3 = pid(pid_x_kp, pid_x_ki / dt_host, pid_x_kd * dt_host) * lpfc3;
%
%sys2 = tf(500, [6 20 0]);
%
%OL3 = C3 * sys2;
%CL3 = feedback(OL3);


g = 9.81;
sys_x = -tf((pi / 180) * g * 100, [1 1 0]) * CL2;
[x_sim, t, i] = lsim(sys_x, pitch_set_i, time_i);

a2 = subplot(2,1,2);
plot(time, x), hold on
plot(time_i, x_sim, 'r')

%% Kalman filter for x position
% Convert model do discrete state space form
model_x = ss(c2d(sys_x, dt_host));
A = model_x.a;
B = model_x.b;
C = model_x.c;
L = 1e-3 * ones(4, 1);

% State vector
N = numel(time_i);
X = zeros(4, N);
for n = 2:N
    Y = C * X(:, n - 1);
    X(:, n) = A * X(:, n - 1) + L * (x_i(n - 1) - Y) + B * pitch_set_i(n - 1);
end

plot(time_i, X(4, :), 'g')

%[m, p, z, e] = dlqe(a, g, c, q, r)