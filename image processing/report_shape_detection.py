import cv2
import numpy as np

def detect_red_shapes(image_path, min_area=100):
    """
    Detects red color regions and then identifies basic shapes within those regions,
    only considering shapes with an area above a certain threshold.

    Args:
        image_path (str): The path to the input image.
        min_area (int): The minimum area threshold for detected shapes. Contours
                        with an area smaller than this will be ignored.

    Returns:
        None: Displays the original image with detected red shapes outlined.
    """
    # Read the image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not open or find the image at {image_path}")
        return

    # Create a copy of the original image to draw on
    img_with_shapes = img.copy()

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the range for red color in HSV
    # Red color wraps around the hue spectrum, so we need two ranges
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for the red color ranges
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # Combine the masks
    red_mask = mask1 + mask2

    # Optional: Perform morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the red mask
    contours, _ = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over the contours found in the red mask
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)

        # Filter contours based on the minimum area
        if area > min_area:
            # Calculate the perimeter of the contour
            perimeter = cv2.arcLength(contour, True)

            # Approximate the contour to get the vertices of the shape
            # Use a higher epsilon value for approximation as contours from mask can be noisy
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

            # Get the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Aspect ratio for potential square detection
            aspect_ratio = float(w) / h

            # Determine the shape based on the number of vertices
            shape = "Unknown" # Default shape label

            if len(approx) == 3:
                shape = "Red Triangle"
                # Draw contour on the original image copy
                cv2.drawContours(img_with_shapes, [contour], -1, (0, 255, 0), 2) # Green color for triangle outline
            elif len(approx) == 4:
                # Check if it's likely a square based on aspect ratio
                if 0.95 <= aspect_ratio <= 1.05:
                    shape = "Red Square"
                    # Draw contour on the original image copy
                    cv2.drawContours(img_with_shapes, [contour], -1, (0, 0, 255), 2) # Blue color for square outline
                else:
                    shape = "Red Rectangle"
                    # Draw contour on the original image copy
                    cv2.drawContours(img_with_shapes, [contour], -1, (255, 0, 0), 2) # Red color for rectangle outline
            elif len(approx) >= 5:
                 # Approximate to check for circularity
                if perimeter > 0: # Avoid division by zero
                    (x_c, y_c), radius = cv2.minEnclosingCircle(contour)
                    circularity = (4 * np.pi * area) / (perimeter * perimeter)
                    if circularity > 0.6:  # Adjust threshold as needed for mask contours
                        shape = "Red Circle"
                        # Draw contour on the original image copy
                        cv2.drawContours(img_with_shapes, [contour], -1, (255, 255, 0), 2) # Yellow color for circle outline
                    else:
                        shape = "Red Polygon"
                        # Draw contour on the original image copy
                        cv2.drawContours(img_with_shapes, [contour], -1, (0, 255, 255), 2) # Cyan color for polygon outline


            # Put the shape name on the detected object on the original image copy
            if shape != "Unknown":
                 cv2.putText(img_with_shapes, shape, (x - 25, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


    cv2.imshow("Red Shape Detection", img_with_shapes)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Example usage: Replace 'frame.png' with the path to your image
    # You can also specify a minimum area, e.g., detect_red_shapes('frame.png', min_area=500)
    image_file = 'frame.png'
    min_shape_area = 100 # Set your desired minimum area here

    detect_red_shapes(image_file, min_area=min_shape_area)