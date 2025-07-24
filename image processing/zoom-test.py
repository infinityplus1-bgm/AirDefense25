import cv2

zoom_factor = 3

pipeline = f"v4l2src device=/dev/video2 ! image/jpeg,width=1280,height=720,framerate=60/1 ! jpegdec ! videoconvert ! appsink"

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Define grid spacing
grid_spacing = 20 # pixels

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        break

    original_height, original_width = frame.shape[:2]

    # Use INTER_LINEAR or INTER_CUBIC for upscaling for better quality
    resized_frame = cv2.resize(frame, (original_width * zoom_factor, original_height * zoom_factor), interpolation=cv2.INTER_LINEAR)

    resized_height, resized_width = resized_frame.shape[:2]

    # Calculate start and end points for the middle section of the resized frame
    # This effectively crops the center 'original_width' x 'original_height' portion
    start_x_resized = (resized_width // 2) - (original_width // 2)
    end_x_resized = (resized_width // 2) + (original_width // 2)
    
    start_y_resized = (resized_height // 2) - (original_height // 2)
    end_y_resized = (resized_height // 2) + (original_height // 2)

    # Crop the middle section from the resized frame
    final_cropped_frame = resized_frame[start_y_resized:end_y_resized, start_x_resized:end_x_resized]

    # Get dimensions of the final cropped frame for drawing the grid
    frame_height, frame_width = final_cropped_frame.shape[:2]

    # Draw horizontal grid lines
    for y in range(0, frame_height, grid_spacing):
        cv2.line(final_cropped_frame, (0, y), (frame_width, y), (0, 255, 0), 1) # Green lines, thickness 1

    # Draw vertical grid lines
    for x in range(0, frame_width, grid_spacing):
        cv2.line(final_cropped_frame, (x, 0), (x, frame_height), (0, 255, 0), 1) # Green lines, thickness 1

    cv2.imshow('Camera View with Grid', final_cropped_frame)

    if cv2.waitKey(1) == ord('q'):
        break
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
