instrreset;
s = serial('COM10');
s.baudrate = 9600;
fopen(s);

tstep = 0.1;
position = [0, 0, 0];
positions = [position];
fig = figure();
while ishandle(fig)
    % Format is w, x, y, z, X, Y, Z
    % Where w, x, y, z are the components of the quaternion
    % And X, Y, Z are the accelerations along the x, y, and z axes
    line = sscanf(fgets(s), '%g,', [14, 1]).';
    %disp(line);
    if length(line) < 14 || all(line(11:14) == 0) 
        disp('Incomplete line');
        continue
    end
    

    
    quat = line(11:14); % ./ 2^14;
    %accel = line(5:7) ./ 100;
    
    disp(quat);
    
%     if(max(abs(accel) > 300))
%         accel = [0 0 0];
%     end
    
    %rot_accel = quatrotate(quatinv(quat), accel);
%     this_vel = rot_accel * tstep;
%     this_pos = this_vel * tstep;
%     positions = [positions; positions(end, :) + this_pos];
%     [rows, ~] = size(positions);
%     plot(positions(:, 1), positions(:, 2));
    axis vis3d
    drawnow;
    pause(0.001);
end

fclose(s);
delete(s);
clear s;