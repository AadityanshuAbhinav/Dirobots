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
    [id, loc, pose] = readAprilTag(img, 'tag36h11', Intrinsics, 0.141); % Tag size is 0.141 meters

    % Display the image
    imshow(img);
    hold on;

    % Find marker 0
    idx0 = find(id == 0);

    if ~isempty(idx0)
        % Get the direction of marker 0 in the global coordinate system
        R0 = pose(idx0).Rotation;
        angle_z0 = atan2d(R0(2,1), R0(1,1));

        % Plot detected AprilTags and their new poses
        for i = 1:length(id)
            % Get the current tag's rotation matrix and translation
            R = pose(i).Rotation;
            t_global = pose(i).Translation;

            % Calculate the local position relative to marker 0
            t_local = t_global - pose(idx0).Translation;

            % Calculate the rotation angle about the z-axis relative to marker 0
            angle_z = atan2d(R(2,1), R(1,1)) - angle_z0;

            % Draw the tag boundary
            plot(loc(:,1,i), loc(:,2,i), 'g-', 'LineWidth', 2);

            % Draw the centroid
            center = mean(loc(:,:,i), 1);
            plot(center(1), center(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

            % Display the new position and rotation angle
            text(center(1), center(2), sprintf('ID: %d\nX: %.2f\nY: %.2f\nZ: %.2f\nAngle: %.2f°', ...
                                               id(i), t_local(1), t_local(2), t_local(3), angle_z), 'Color', 'yellow', 'FontSize', 12);
        end
    end

    hold off;

    % Pause briefly to allow the image to update
    pause(0.01);
end

% Stop streaming when the figure is closed
pipe.stop();
