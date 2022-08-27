% Starter-code of P1 author: Jose Guivant

% Edited and completed by: z5260252, Gajendra Jayasekera
% Date: 20 / 04 / 2022
% Purpose: This file is the main program for solving Project 2, Part C (Alternative)
% Course: MTRN4010.T1.2022

% =========================================================================

function PartC(file)
    load(file); % load dataset
    cost = @(bias) Cost(bias, data); % cost definition
    estimated_bias = fminsearch(cost, 0); % search for the bias in the system, taking initial guess as 0
    estimated_bias = rad2deg(estimated_bias);
    fprintf('Estimated bias is %.2f deg/sec\n',estimated_bias);
end

function cost = Cost(bias, data)
    
    X = data.pose0; % platform's initial pose; [x0 (m); y0 (m); heading0 (rads)]
    table = data.table; % table of events.
    event = table(:,1); % first event.
    ne = data.n; % number of events
    t0 = 0.0001 * double(event(1)); % initial time
    vw = [0;0]; % To keep last [speed; heading rate] measurement.

    cost = 0;
    GT = data.verify.poseL; % Ground truth

    for i = 1:ne
        event = table(:,i);
        sensorID = event(3);
        t = 0.0001 * double(event(1));
        dt = t - t0; % dt since last event (needed for predictions steps). 
        t0 = t;
        here = event(2); % where to read it, from that sensor recorder.
        
        X = VehiclePose(X,vw,dt); % P1 Part A - Function call

        switch sensorID % measurement from
            case 1 % LiDAR
                X_GT = GT(:,here); % Get GT when lidar is available
                cost = cost + norm(X_GT(1:2) - X(1:2)); % sum of cost
            continue;
            
            case 2 % Speed encoder + gyro
                vw = data.vw(:,here); % speed and gyroZ, last updated copy.
                vw(2) = vw(2) - bias; % account for current predicted bias
            continue;
            
            otherwise % Unknown sensors 
                %fprintf('unknown sensor, type[%d], at t=[%d]\n',sensorID, t);         
            continue;
        end
    end
end

function X = VehiclePose(pose_curr, vw, dt)
    h = pose_curr(3);
    dpose = [vw(1)*cos(h); vw(1)*sin(h); vw(2)];
    X = pose_curr + dt*dpose;
end