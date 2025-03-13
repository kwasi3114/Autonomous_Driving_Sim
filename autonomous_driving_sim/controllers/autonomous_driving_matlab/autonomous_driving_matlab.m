function autonomous_driving_matlab
    TIME_STEP = 64;
    
    %pyenv('Version', "C:\Users\kwasi\.pyenv\pyenv-win\versions\3.11.7\python.exe");

    % Start Simulink model
    %modelName = 'vehicle_controller';
    %load_system(modelName);
    %set_param(modelName, 'SimulationCommand', 'start');  
    
% Specify the full path to the headers
    carHeaderPath = 'C:\Program Files\Webots\include\controller\c\webots\vehicle\car.h';
    driverHeaderPath = 'C:\Program Files\Webots\include\controller\c\webots\vehicle\driver.h';
    webotsIncludePath = 'C:\Program Files\Webots\include\controller\c\webots';  % Directory where 'types.h' is located
    
    % Check if the library is already loaded
    if libisloaded('libController')
        % Unload the library (MATLAB requires this to add new headers)
        unloadlibrary('libController');
        
        loadlibrary('Controller', '', 'alias', 'libController', 'addheader', carHeaderPath, 'includepath', webotsIncludePath);
        loadlibrary('Controller', '', 'alias', 'libController', 'addheader', driverHeaderPath, 'includepath', webotsIncludePath);
        %loadlibrary('Controller', '', 'alias', 'libController', 'addheader', driverHeaderPath);
        
        % Reload with additional headers (specify paths)
        %loadlibrary('Controller', carHeaderPath, 'alias', 'libController', ...
        %            'addheader', driverHeaderPath, ...
        %            'addinclude', webotsIncludePath);  % Add the Webots include directory
        
        % Display message
        disp('car.h and driver.h have been added.');
    end


    % Initialize Webots camera
    camera = wb_robot_get_device('camera');
    wb_camera_enable(camera, TIME_STEP);
    %wb_driver_init();
    %wbu_car_init();
    %wbu_driver_init();

    % Get image dimensions
    image_width = wb_camera_get_width(camera);
    image_height = wb_camera_get_height(camera);
    

    while wb_robot_step(TIME_STEP) ~= -1
        % Get the raw image
        %raw_image = wb_camera_get_image(camera);
        
        raw_image = wb_camera_get_image(camera);
        success = wb_camera_save_image(camera, 'image/camera_image.jpeg', 50);
        pause(0.01); % Small delay to allow the file system to update
        %disp(success)
        
        %pyrunfile("car_driver.py")
        
        % Ensure image is valid before assigning
        %if ~isempty(raw_image) && any(raw_image(:) ~= 0)
        %    assignin('base', 'webotsImage', im2double(raw_image));  % Convert to double for Simulink
        %else
        %    disp('Webots image is empty or all zeros.');
        %end
        %set_param(modelName, 'SimulationCommand', 'start');
        % Step the Simulink simulation forward
        %if strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
        %  set_param(modelName, 'SimulationCommand', 'start');
        %end
        %set_param(modelName,'SimulationCommand','update')
        %set_param(modelName, 'SimulationCommand', 'step');
        
        
        %assignin('base', 'webotsImage', uint8(raw_image));
        %sim(modelName);
        %set_param(modelName, 'SimulationCommand', 'start');
        
        %if strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
        %  set_param(modelName, 'SimulationCommand', 'start');
        %end
        
        %set_param(modelName, 'SimulationCommand', 'step');

        % Convert Webots single-column image to a 3D RGB array
        %rgb_image = reshape(raw_image, [3, image_width, image_height]);
        %rgb_image = permute(rgb_image, [3, 2, 1]); % Convert to HxWx3 format

        % Write the image to Simulink Data Store
        %disp(size(raw_image));  % Display the size of the image array
        %disp(image_width);
        %disp(image_height);
        %assignin('base', 'webotsImage', raw_image);

        % Step Simulink simulation
        %set_param(modelName, 'SimulationCommand', 'step');
    end

    % Stop and close Simulink model when Webots ends
    set_param(modelName, 'SimulationCommand', 'stop');
    close_system(modelName, 0);
end
