function finalRad= ExampleControlProgram(serPort)

    % Set constants for this program
    maxDuration= 1200;  % Max time to allow the program to run (s)
    maxFwdVel= 0.5;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)   
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    h_front = ReadSonarMultiple(serPort,2);
    h_right = ReadSonarMultiple(serPort,1);
    h_left = ReadSonarMultiple(serPort,3);
    [fvel wvel] = getRobotvel(serPort);
    SetDriveWheelsCreate(serPort,0.1,0.1);
    samples=[3 ; 3; 3; 0.5; 0.5]
    % Enter main loop
    while toc(tStart) < maxDuration
        % Check for and react to bump sensor readings
        % 1 - right, 2 - front, 3 - left, 4 - back
            front = ReadSonarMultiple(serPort,2);
            right = ReadSonarMultiple(serPort,1);
            left = ReadSonarMultiple(serPort,3);
            [h_fvel h_wvel] = getRobotvel(serPort);
        %if length([right; front; left])==3 && (abs(left-h_left) > 0.1) || (abs(right-h_right) > 0.1) || (abs(front-h_front) > 0.1)
        if length([right; front; left])==3 && (h_fvel ~= fvel) || (h_wvel ~= wvel)
            h_front = front;
            h_right = right;
            h_left =  left;
            fvel = h_fvel; 
            wvel = h_wvel;
            wheelRight= fvel+wvel*0.258/2;
            wheelLeft= fvel-wvel*0.258/2;
            
            inst_sample = [h_right ;h_front ;h_left ;wheelRight ;wheelLeft]
            samples=[samples inst_sample]
        end
        [fvel wvel] = getRobotvel(serPort);
        if (fvel==0) && (wvel==0)
            save('test_set','samples');
            disp('savedd')
        end
        pause(0.2)
    end
    save('test_set','samples')
    disp('finish---------')
    % Specify output parameter
    finalRad= 0;

    SetFwdVelAngVelCreate(serPort,0,0)
end