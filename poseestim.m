% Make Pipeline object to manage streaming
pipe = realsense.pipeline();

config.enable_stream(realsense.stream.color, 1920, 1080, realsense.format.rgb8, 30); % 30 FPS

% Start streaming with the configured settings
profile = pipe.start(config);

% Define camera intrinsics
focalLength = [966.176, 966.176];
principalPoint = [972.006, 503.704];
imageSize = [1080, 1920];
Intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Specify the tag size in meters
tagSize = 0.141; % Adjust based on your tag size

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
    
    % Detect AprilTags in the image and estimate their poses
    [id, loc, pose] = readAprilTag(img, 'tag36h11', Intrinsics, tagSize);
    
    % Display the image
    imshow(img);
    hold on;
    
    pos = pose.R;
    % Annotate the image with detected tag IDs and poses
    for i = 1:length(id)
        % Display the ID and pose of each detected tag
        disp(['Detected Tag ID: ', num2str(id(i))]);
        disp(['Pose: ', mat2str(pos(:,:,1))]);
        
        % Draw the tag corners
        plot(loc(:,1,i), loc(:,2,i), 'r-', 'LineWidth', 2);
        
        % Draw the tag ID at the center of the tag
        text(mean(loc(:,1,i)), mean(loc(:,2,i)), num2str(id(i)), 'Color', 'yellow', 'FontSize', 12);
    end
    
    hold off;
    drawnow;
end

% Stop the pipeline when done
pipe.stop();
