import cv2
import os


def save_frames(video_path, output_paths, frame_offset=0, skip=10):
    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_offset)
    success, image = cap.read()
    
    if not success:
        print("Error: Unable to read the video.")
        return
    
    for i, output_path in enumerate(output_paths):
        cv2.imwrite(output_path, image)
        # print(image)
        print(f"Frame {i+1} saved as {output_path}")
        
        # Skip frames
        for _ in range(skip):  # Skip 10 frames to save the next frame
            success, image = cap.read()
            if not success:
                break

    cap.release()

# Example usage:
base_folder = "/home/user/Projects/robot_localization_ws/of_poc/videos"

video_name = "iris_yaw.mp4"
video_path = os.path.join(base_folder, video_name)

dir_name = os.path.join(base_folder, os.path.splitext(video_name)[0])
if not os.path.exists(dir_name):
    os.mkdir(dir_name)
output_paths = [f"frame{i}.jpg" for i in range(1,4)]  # Paths to save frames
output_paths = [os.path.join(base_folder, os.path.splitext(video_name)[0], i) for i in output_paths]
save_frames(video_path, output_paths, 10, 80)