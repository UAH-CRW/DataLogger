result = [];
for n = 0:23
    vals = csvread(sprintf('LOG%i.CSV', n));
    result = [result; vals];
end

% Convert accel to G
result(:, 2:4) = result(:, 2:4) * 16 / 2^15; %+- 16 G on a 16 bit signed ADC
%Gyro to deg/s
result(:, 5:7) = result(:, 5:7) * 2000 / 2^15; % +- 2000 deg/s on a 16 signed ADC
%Mag to uT
result(:, 8:10) = result(:, 8:10) * 4800 / 2^15; %+- 4800 uT on a 16 bit signed ADC
% Convert time from ms to s
result(:, 1) = result(:, 1) / 1000;

%4.5 to 4.8 * 10^3 s looks like flight
trelevant = find(result(:, 1) >= 4.5e3 & result(:, 1) <= 4.8e3);
data = result(trelevant, :);
data(:, 1) = data(:, 1) - data(1, 1); % Set zero to beginning of file
%Even out time (low resolution on logged time makes things look
%inconsistent)
data(:, 1) = linspace(data(1, 1), data(end, 1), length(data(:, 1)));

%Drogue deploy is taken at the moment abs(axial accel) > 5 G (near the
%drogue deploy). Same for main.
% Drogue deploy (Raven): 18.0713 s
% Main deploy (Raven): 76.4887 s
% Drogue deploy (IMU): 82.8488 s
% Main deploy (IMU): 115.9569 s

tratio = (76.4887 - 18.0713) / (115.9569 - 82.8488);
data(:, 1) = data(:, 1) * tratio; % Correct for apparent sampling rate error
data(:, 1) = data(:, 1) - 128.1; %Align launch to approximately T-0

csvwrite('flight_data.csv', data);
csvwrite('Consolidated_and_converted_data.csv', result);

%% Make timeseries and .mat file
t = data(:, 1);
a.x = timeseries(data(:, 2), t);
a.x.DataInfo.Units = 'G';
a.x.Name = 'Acceleration (X-axis)';
a.x.TimeInfo.units = 's';

a.y = timeseries(data(:, 3), t);
a.y.DataInfo.Units = 'G';
a.y.Name = 'Acceleration (Y-axis)';
a.y.TimeInfo.units = 's';

a.z = timeseries(data(:, 4), t);
a.z.DataInfo.Units = 'G';
a.z.Name = 'Acceleration (Z-axis)';
a.z.TimeInfo.units = 's';

g.x = timeseries(data(:, 5), t);
g.x.DataInfo.Units = 'deg/s';
g.x.Name = 'Angular Velocity (X-axis)';
g.x.TimeInfo.units = 's';

g.y = timeseries(data(:, 6), t);
g.y.DataInfo.Units = 'deg/s';
g.y.Name = 'Angular Velocity (Y-axis)';
g.y.TimeInfo.units = 's';

g.z = timeseries(data(:, 7), t);
g.z.DataInfo.Units = 'deg/s';
g.z.Name = 'Angular Velocity (Z-axis)';
g.z.TimeInfo.units = 's';

m.x = timeseries(data(:, 8), t);
m.x.DataInfo.Units = 'uT';
m.x.Name = 'Magnetic Field (X-axis)';
m.x.TimeInfo.units = 's';

m.y = timeseries(data(:, 9), t);
m.y.DataInfo.Units = 'uT';
m.y.Name = 'Magnetic Field (Y-axis)';
m.y.TimeInfo.units = 's';

m.z = timeseries(data(:, 10), t);
m.z.DataInfo.Units = 'uT';
m.z.Name = 'Magnetic Field (Z-axis)';
m.z.TimeInfo.units = 's';

save('Flight Data.mat', 'a', 'g', 'm');

%% Grab Pressure
pdata = csvread('Raven_pressure.csv');
p = timeseries(int32(pdata(:, 2) * 101325), pdata(:, 1));
p.DataInfo.Units = 'Pa';
p.Name = 'Barometric Pressure';
p.TimeInfo.Units = 's';

rdata = csvread('Raven_accel.csv');
raven.axial = timeseries(rdata(:, 2), rdata(:, 1));
raven.axial.DataInfo.Units = 'G';
raven.axial.Name = 'Raven Axial Acceleration';
raven.axial.TimeInfo.Units = 's';

save('Flight Data.mat', 'p', 'raven', '-append');