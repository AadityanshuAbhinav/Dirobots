% Make Pipeline object to manage streaming
pipe = realsense.pipeline();

% Configure the pipeline to stream color data at 1920x1080 resolution
config = realsense.config();
config.enable_stream(realsense.stream.color, 1920, 1080, realsense.format.rgb8, 30); % 30 FPS

% Start streaming with the configured settings
profile = pipe.start(config);

% Define camera intrinsics
focalLength = [966.176, 966.176];
principalPoint = [972.006, 503.704];
imageSize = [1080, 1920];
Intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Create a UDP object for communication
arduinoIP = '192.168.75.195'; % Replace with the IP address of your Arduino
arduinoPort = 8888;
u = udp(arduinoIP, 'RemotePort', arduinoPort);

% Open the UDP connection
fopen(u);

% Create a figure for displaying the video
figure;

while true
    % Wait for the next set of frames
    fs = pipe.wait_for_frames();
    
    % Get the color frame
    color_frame = fs.get_color_frame();
    
    % Get actual data and convert into a format imshow can use
    data = color_frame.get_data();
    img = permute(reshape(data', [3, color_frame.get_width(), color_frame.get_height()]), [3 2 1]);
    
    % Detect AprilTags in the image
    [id, loc, ~] = readAprilTag(img, 'tag36h11', Intrinsics, 0.141); % Tag size is 0.141 meters
    
    % Display the image
    imshow(img);
    hold on;
    
    % Calculate the centroids of the detected tags
    centroids = zeros(length(id), 2);
    for i = 1:length(id)
        centroids(i, :) = mean(loc(:,:,i), 1);
    end
    
    % Find the coordinates of ID 0, ID 4, and ID 3
    idx0 = find(id == 0);
    idx4 = find(id == 4);
    idx3 = find(id == 3);
    
    if ~isempty(idx0) && ~isempty(idx4) && ~isempty(idx3)
        % Calculate x0 as the distance between ID 0 and ID 4 with scaling factors
        x0 = sqrt(((centroids(idx0,1) - centroids(idx4,1)) / 5.6316)^2 + ((centroids(idx0,2) - centroids(idx4,2)) / 5.597073519)^2);
        
        % Calculate y0 as the distance between ID 4 and ID 3 with scaling factors
        y0 = sqrt(((centroids(idx4,1) - centroids(idx3,1)) / 5.6316)^2 + ((centroids(idx4,2) - centroids(idx3,2)) / 5.597073519)^2);
        
        % Plot detected AprilTags and their centroids
        for i = 1:length(id)
            % Draw the tag boundary
            plot(loc(:,1,i), loc(:,2,i), 'g-', 'LineWidth', 2);
            
            % Calculate the coordinates in the new system
            if id(i) == 0
                coord = [0, 0];
            elseif id(i) == 4
                coord = [x0, 0];
            elseif id(i) == 3
                coord = [x0, y0];
            else
                % Calculate coordinates relative to the new system
                coord = [(centroids(i,1) - centroids(idx0,1)) / 5.6316, (centroids(i,2) - centroids(idx0,2)) / 5.597073519];
            end
            
            % Display the tag ID and coordinates
            text(loc(1,1,i), loc(1,2,i), sprintf('ID: %d\nX: %.2f\nY: %.2f', id(i), coord(1), coord(2)), 'Color', 'yellow', 'FontSize', 12);
            
            % Plot the centroid
            plot(centroids(i, 1), centroids(i, 2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            
            % Send the x-coordinate to the Arduino if it's a new marker
            if id(i) ~= 0 && id(i) ~= 4 && id(i) ~= 3
                fwrite(u, num2str(coord(1)));
            end
        end
    end
    
    hold off;
    
    % Pause briefly to allow the image to update
    pause(0.01);
end

% Close the UDP connection when done
fclose(u);
delete(u);
clear u;

% Stop streaming when the figure is closed
pipe.stop();
