pkg load control

%% Identify model for pitch/roll dynamics

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
    if pitch_cf(i) != 0 && pitch_cf(i - 1) == 0
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
dt_cf = 1 / freq; % sampling rate on quadcopter
dt_host = 0.01; % sampling rate on host PC
time_i = 0:dt_host:time(end);

[time_cf, i_before, i_after] = unique(time_cf);
pitch_cf = pitch_cf(i_before);

pitch_cf_i = interp1(time_cf, pitch_cf, time_i)';
pitch_set_i = interp1(time, pitch_set, time_i)';
x_i = interp1(time, x, time_i)';
x_i(isnan(x_i)) = 0;% zero padding
pitch_cf_i(isnan(pitch_cf_i)) = 0;% zero padding
pitch_set_i(isnan(pitch_set_i)) = 0;% zero padding


% Inner loop for pitch control.
% It consits of two cascade pid controllers
% Settings for pitch rate pid
pid_pitch_rate_kp = 250;
pid_pitch_rate_ki = 0;
pid_pitch_rate_kd = 0;
% Settings for pitch pid
pid_pitch_kp = 5;
pid_pitch_ki = 0;
pid_pitch_kd = 0;

% Identify 2-nd order ARX model from experimental data
pitch_dat = iddata(pitch_cf_i, pitch_set_i, dt_host);
[model_pitch, x0] = arx(pitch_dat, 2);
model_pitch_c = tf(d2c(model_pitch));

[pitch_sim, t, i] = lsim(model_pitch_c, pitch_set_i, time_i);
% Normilize dc gain to 1
model_pitch_c /= dcgain(model_pitch_c);
% Remove neglegible zero from transfer function
[num,den] = tfdata(model_pitch_c);
num{1}(1) = [];
model_pitch_c = tf(num, den);

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
xlabel('time, sec'), ylabel('Angle, deg'), grid on
legend('Pitch set', 'Pitch measured', 'Pitch model')

% Outer loop (position x)
g = 9.81;
sys_x = -tf((pi / 180) * g * 100, [1 1 0]) * CL2;
[x_sim, t, i] = lsim(sys_x, pitch_set_i, time_i);

a2 = subplot(2,1,2);
plot(time, x), hold on
plot(time_i, x_sim, 'r')
linkaxes([a1 a2], 'x')

%% Kalman filter for x position
% Convert model b discrete state space form
model_x = ss(c2d(sys_x, dt_host));
A = model_x.a;
B = model_x.b;
C = model_x.c;
[est, L, x] = kalman(model_x, 2, 0.1e-1);

N = numel(time_i);
% State vector
X = zeros(4, N);
for n = 2:N
    Y = C * X(:, n - 1);
    X(:, n) = A * X(:, n - 1) + L * (x_i(n - 1) - Y) + B * pitch_set_i(n - 1);
end

plot(time_i, X(4, :), 'g')
xlabel('time, sec'), ylabel('x position, cm'), grid on
legend('x measured', 'x model', 'x with Kalman filter')