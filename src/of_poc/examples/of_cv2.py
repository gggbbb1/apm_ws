import math
import time
from typing import Tuple
import cv2 as cv
import numpy as np
import os
import sys
from of_poc.examples.circular_path_integration import circular_path_integration, check_circularity

ARROW_VIS_FACTOR = 15
DEFAULT_VIDEO_FOLDER = "/home/user/Projects/robot_localization_ws/of_poc/videos"
FILTER_FACTOR = 0.8

class OF():
    def __init__(self, video_name: str, videos_folder: str) -> None:
        self.full_path = os.path.join(videos_folder, video_name)
        self.cap = None
        self.last_frame_greyscale = None
        self.filtered_x = 0
        self.filtered_y = 0
        self.filtered_curl = 0
        self.helper_mat = None
        self.first_setup()

    def first_setup(self) -> bool:
        self.cap = cv.VideoCapture(self.full_path)
        ret, first_frame = self.cap.read()
        self.last_frame_greyscale = cv.cvtColor(first_frame, cv.COLOR_BGR2GRAY)
        mask = np.zeros_like(first_frame)
        mask[..., 1] = 255
        self.height, self.width, _ = first_frame.shape
        self.num_frames = 0
        self.start_time = time.time()
        self.sum_curl = 0
        return True

    def calculate_full_flow_farneback(self, grey_scale_frame: np.ndarray) -> np.ndarray:
        # Calculates dense optical flow by Farneback method
        flow = cv.calcOpticalFlowFarneback(
            self.last_frame_greyscale, grey_scale_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        # print(flow)
        return flow

    def get_flow_avg_vector(self, flow: np.ndarray) -> np.ndarray:
        after_mean = np.mean(flow, axis=0)
        after_mean = np.mean(after_mean, axis=0)
        return after_mean

    def draw_flow_representation_on_frame(self, frame, flow_x, flow_y, vis_factor=1.0, start_point: Tuple[int] = None, title: str = "") -> None:
        if not start_point:
            height, width = frame.shape[:2]
            center_x = int(width / 2)
            center_y = int(height / 2)
            start_point = (center_x, center_y)
        print(f"{flow_x, flow_y=}")
        end_point = (int(start_point[0] + flow_x*vis_factor), int(
            start_point[1] + flow_y * vis_factor))
        color = (0, 0, 255)  # Red color in BGR
        thickness = 2
        tipLength = 0.1  # Length of the arrow tip relative to the arrow length
        title_point = (start_point[0], start_point[1] - 20)
        cv.circle(frame, start_point, 4, (0, 0, 0), -1)

        cv.arrowedLine(frame, start_point, end_point, color,
                       thickness, tipLength=tipLength)
        cv.putText(frame, title, title_point, 1, 2, (255, 0, 0), 2)

    def guess_camera_move_type(self, flow):
        # ret = circular_path_integration(flow, (self.width//2, self.height//2), 50)
        ret = check_circularity(flow, self.helper_mat)
        return ret

    def init_helper_mat(self, matrix):
        height, width, _ = matrix.shape
        print(matrix.shape)
        self.helper_mat = np.zeros_like(matrix)
        print(self.helper_mat.shape)
        center_point = (width // 2, height // 2)
        for y in range(height):
            for x in range(width):
                point_relative_to_center = (x - center_point[0], y - center_point[1])
                radius = np.linalg.norm(point_relative_to_center)
                if radius == 0.0:
                    radius = 100000000
                # if radius > min(height, width)//2:
                #     self.helper_mat[y][x][0] = 0
                #     self.helper_mat[y][x][1] = 0
                # else:
                self.helper_mat[y][x][0] = -point_relative_to_center[1] / radius**2
                self.helper_mat[y][x][1] = point_relative_to_center[0] / radius**2
        print(self.helper_mat)
        
    def translation_to_speed(self, tx, ty, curr_height, fovx_deg, fovy_deg, dtime, image_width, image_height) -> [float, float]:
        half_x_meters = curr_height * math.tan(math.radians(fovx_deg/2))
        half_y_meters = curr_height * math.tan(math.radians(fovy_deg/2))
        vx = (half_x_meters * tx / image_width) / dtime
        vy = (half_y_meters * ty / image_height) / dtime
        return vx, vy
    
    def run(self):
        while (self.cap.isOpened()):
            # time.sleep(0.3)
            self.num_frames += 1

            ret, frame = self.cap.read()
            
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            flow = self.calculate_full_flow_farneback(gray)
            if self.helper_mat is None:
                self.init_helper_mat(flow)
            # fps calculating
            elapsed_time = time.time() - self.start_time
            self.avg_fps = self.num_frames/elapsed_time
            print(f"{self.avg_fps=}")

            (flow_x, flow_y) = self.get_flow_avg_vector(flow)
            self.filtered_x = self.filtered_x * \
                FILTER_FACTOR + flow_x * (1 - FILTER_FACTOR)
            self.filtered_y = self.filtered_y * \
                FILTER_FACTOR + flow_y * (1 - FILTER_FACTOR)
            print(f"{flow_x, flow_y=}")
            vx, vy = self.translation_to_speed(flow_x, flow_y, 80, 50, 40, 1/self.avg_fps, frame.shape[0], frame.shape[1])
            print(f"{vx, vy=}")
            # print("filtered", self.filtered_x, self.filtered_y)
            self.draw_flow_representation_on_frame(
                frame, flow_x, flow_y, vis_factor=ARROW_VIS_FACTOR, title="average flow")
            self.draw_flow_representation_on_frame(
                frame, self.filtered_x, self.filtered_y, vis_factor=ARROW_VIS_FACTOR, start_point=(self.width//3, self.height//3), title="filtered flow")
            # flow = np.ndarray(shape=(512, 640, 2))
            # flow.fill(1)
            curl = self.guess_camera_move_type(flow)
            self.draw_flow_representation_on_frame(
                frame, curl, 0, vis_factor=0.5, start_point=(2*self.width//3, 2*self.height//3), title="curl")
            print(f"{curl=}")

            self.filtered_curl = self.filtered_curl * \
                FILTER_FACTOR + curl * (1 - FILTER_FACTOR)
            self.draw_flow_representation_on_frame(
                frame, self.filtered_curl, 0, vis_factor=0.5, start_point=(int(2.5*self.width)//3, int(2.5*self.height)//3), title="filtered curl")
            self.sum_curl = self.sum_curl + self.filtered_curl
            # print(flow.shape)
            cv.imshow("dense optical flow", frame)

            self.last_frame_greyscale = gray
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

    def close(self):
        print("angular path: ", ((self.sum_curl * self.avg_fps))*180/np.pi, " degrees")
        self.cap.release()
        cv.destroyAllWindows()

# The following frees up resources and
# closes all windows


if __name__ == "__main__":
    if len(sys.argv) <= 2:
        video_path = DEFAULT_VIDEO_FOLDER
    else:
        video_path = sys.argv[2]
    video_name = sys.argv[1]
    of = OF(video_name=video_name, videos_folder=video_path)
    of.run()
    of.close()
