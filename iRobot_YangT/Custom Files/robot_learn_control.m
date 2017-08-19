function finalRad= ExampleControlProgram(serPort)

    
    % Set constants for this program
    maxDuration= 1200;  % Max time to allow the program to run (s)
    maxFwdVel= 0.5;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)   
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    load('weights')
    % Enter main loop
    while toc(tStart) < maxDuration
        % Check for and react to bump sensor readings
        % 1 - right, 2 - front, 3 - left, 4 - back
        front = ReadSonarMultiple(serPort,2);
        right = ReadSonarMultiple(serPort,1);
        left = ReadSonarMultiple(serPort,3);
        if length([right; front; left])==3
            Y = runMLP([right; front; left],Wx,Wy);
            Y = matdemap(Y);

            SetDriveWheelsCreate(serPort,Y(1,1,:),Y(2,1,:));
        else
            SetDriveWheelsCreate(serPort,-0.5,0.5);
        end
        [x y th]= OverheadLocalizationCreate(serPort);
        %if(rem(toc(tStart),2)==0)
        plot(x,y,'r*');
        %end
        pause(0.1)
    end
    
    % Specify output parameter
    finalRad= 0;

    SetFwdVelAngVelCreate(serPort,0,0)
end