% Make Pipeline object to manage streaming
pipe = realsense.pipeline();

% Configure the pipeline to stream color data at 1920x1080 resolution
config = realsense.config();
config.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30); % 30 FPS

% Start streaming with the configured settings
profile = pipe.start(config);

% Define camera intrinsics
focalLength = [607.34521484, 607.34686279];
principalPoint = [313.57650757, 260.03515625];
imageSize = [480, 640];
Intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Create a UDP object for communication
arduinoIP = '192.168.139.99'; % Replace with the IP address of your Arduino
arduinoPort = 9999;
u = udp(arduinoIP, 'RemotePort', arduinoPort);

% Open the UDP connection
fopen(u);
r1 = zeros(1,100000);
r2 = zeros(1,100000);
y=1;
% Create a figure for displaying the video
figure;

while true
    tic; % Start timer
    
    % Wait for the next set of frames
    fs = pipe.wait_for_frames();
    
    % Get the color frame
    color_frame = fs.get_color_frame();
    
    % Get actual data and convert into a format imshow can use
    data = color_frame.get_data();
    img = permute(reshape(data', [3, color_frame.get_width(), color_frame.get_height()]), [3 2 1]);
    
    % Detect AprilTags in the image
    [id, loc, pos] = readAprilTag(img, 'tag36h11', Intrinsics, 0.141); % Tag size is 0.141 meters
    
    % Display the image
    imshow(img);
    %hold on;
    
    % Calculate the centroids of the detected tags
    centroids = zeros(length(id), 2);
    for i = 1:length(id)
        centroids(i, :) = mean(loc(:,:,i), 1);
    end
    
    % Find the coordinates of ID 1, ID 2, and ID 3
    idx1 = find(id == 1);
    %idx2 = find(id == 3);
    %idx3 = find(id == 4);
    idx4 = find(id ~=1);%&& id ~=3 && id~=4)
    
    % if ~isempty(idx1) && ~isempty(idx2) && ~isempty(idx3)
    %     % Calculate the heading angle
    %     deltaX = centroids(idx4, 1) - centroids(idx3, 1);
    %     deltaY = centroids(idx4, 2) - centroids(idx1, 2); 
        z=pos(1,idx4).R
        b=z(2,1);
        r2(y) = b;
        headingAngle = asin(b); %%Find the zero heading direction before we begin by using the poseestim.m file
        
        % Send heading angle to Arduino
        fwrite(u, headingAngle, 'double');
        r1(y) = headingAngle;
    
    
    %hold off;
    
    elapsedTime = toc; % End timer
    y=y+1;
    %pause(max(0, 0.033 - elapsedTime)); % Ensure loop runs at ~30 Hz
end

% Close the UDP connection when done
fclose(u);
delete(u);
clear u;
