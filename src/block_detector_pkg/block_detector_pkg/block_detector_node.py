import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix


FREQ = 2.0 # Hz
MIN_CONTOUR_AREA = 50 # Pixels
MAX_CONTOUR_AREA = 50000



ROTATION_RIGHT = np

class CameraIntrinsics:
    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

class BlockDetector(Node):
    def __init__(self):
        super().__init__('block_detector')
        
        self.left_img_subscriber = self.create_subscription(Image, '/stereo/left/image_rect', self.left_img_cb, 10)
        self.right_img_subscriber = self.create_subscription(Image, '/stereo/right/image_rect', self.right_img_cb, 10)

        self.bridge = CvBridge()
        
        self.left_contours : list | None = None
        self.right_contours : list | None = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.left_transform = None
        self.right_transform = None
        
        self.left_intrinsics = CameraIntrinsics(
            1161.151855, 
            1172.578003,
            335.189156,
            227.486429
        )
        
        self.right_intrinsics = CameraIntrinsics(
            1160.628540,
            1169.930298,
            318.783033,
            236.077347
        )
        
        self.timer = self.create_timer(1.0 / FREQ, self.timer_cb)
        
    def timer_cb(self):
        try: # see if the april tag is currenty visible, and if so, get the transform
            left_transform = self.tf_buffer.lookup_transform(
                'left_frame',
                'left_camera',
                rclpy.time.Time()
            )
            self.left_transform = left_transform
            right_transform = self.tf_buffer.lookup_transform(
                'right_frame',
                'right_camera',
                rclpy.time.Time()
            )
            self.right_transform = right_transform
        except:
            pass # Probably should add something here
        
        # don't do anything if the left or right camera images have not been processed yet, or if the april tag has not been seen yet.
        if self.left_contours is None or self.right_contours is None or self.left_transform is None or self.right_transform is None:
            self.get_logger().info("Haven't recieved both left and right images yet.")
            return
        
        # Log the number of contours
        self.get_logger().info(
            f"Left contours: {len(self.left_contours)}, "
            f"Right contours: {len(self.right_contours)}"
        )
        
        # Might move this to the callback since it doesn't need to be recalculated on every timer cb
        left_centers = [self.get_contour_center(contour) for contour in self.left_contours]
        right_centers = [self.get_contour_center(contour) for contour in self.right_contours]
        
        # find corresponding points between camera 1 and camera 2
        corresponding_points = self.find_corresponding_points(left_centers, right_centers)
        
        id = 0
        
        # triangulate the positions between the corresponding points and assign an id value to them
        for point1, point2 in corresponding_points:
            u1, v1 = point1
            u2, v2 = point2
            
            x, y, z = self.triangulate(u1, v1, u2, v2)
            
            # log the info
            
            self.get_logger().info(
                f"ID: {id}"
                f"X:  {x:.3f}, Y:  {y:.3f}, Z:  {z:.3f}"
                # f"TX: {tx:.2f}, TY: {ty:.2f}, TZ: {tz:.2f}\n"
            )

            id += 1
            
    def transform_point(self, x, y, z, transform):
        t = transform.transform.translation # translation matrix
        q = transform.transform.rotation # Quaternion for rotation
        
        rotation_matrix = quaternion_matrix([ # convert quaternion to the 4x4 matrix
            q.x,
            q.y,
            q.z,
            q.w
        ])
        
        # add translation to the 4x4 matrix
        rotation_matrix[0, 3] = t.x
        rotation_matrix[1, 3] = t.y
        rotation_matrix[2, 3] = t.z
        
        point = np.array([x, y, z, 1.0]) # current xyz point
        
        transformation = rotation_matrix @ point # multiply both matrices
        
        return (
            transformation[0],
            transformation[1],
            transformation[2]
        ) # return the transformed x y z points
    
    def get_contour_center(self, contour):
        M = cv2.moments(contour)
        
        point_count = M['m00']
        u_summation = M['m10']
        v_summation = M['m01']
        
        if point_count == 0 : return None
        
        return [u_summation / point_count, v_summation / point_count]
    
    def get_angle(self, u, v, intrinsics : CameraIntrinsics):
        cx = intrinsics.cx
        cy = intrinsics.cy
        fx = intrinsics.fx
        fy = intrinsics.fy
        theta_x = np.arctan2(u - cx, fx)
        theta_y = np.arctan2(v - cy, fy)
        
        return theta_x, theta_y
        
    def get_contours(self, image : np.ndarray) -> list:
        # grayscale the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Convert everthing lower than 60 to 255, and anything above that to 0.
        _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
    

        # Find contours with opencv magic (find contour, take ALL contourright_img_cbs (used to take only outside but calib broke that), then simplify contour)
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Remove contours smaller than MIN_CONTOUR_AREA and bigger than MAX_CONTOUR_AREA
        good_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= MIN_CONTOUR_AREA and area <= MAX_CONTOUR_AREA:
                good_contours.append(contour)

        return good_contours
    
    def triangulate(self, u1, v1, u2, v2):
        
        x1, y1 = self.get_angle(u1, v1, self.left_intrinsics)
        x2, y2 = self.get_angle(u2, v2, self.right_intrinsics)
        
        left_q = self.left_transform.transform.rotation # Quaternion for rotation
        right_q = self.right_transform.transform.rotation # Quaternion for rotation
        left_t = self.left_transform.transform.translation # translation matrix
        right_t = self.right_transform.transform.translation # translation matrix
        
        lt = np.array([left_t.x, left_t.y, left_t.z])
        rt = np.array([right_t.x, right_t.y, right_t.z])
        
        

        left_rotation_matrix = quaternion_matrix([ # convert quaternion to the 4x4 matrix
            left_q.x,
            left_q.y,
            left_q.z,
            left_q.w
        ])[0:3, 0:3]
        right_rotation_matrix = quaternion_matrix([ # convert quaternion to the 4x4 matrix
            right_q.x,
            right_q.y,
            right_q.z,
            right_q.w
        ])[0:3, 0:3]
        
        # I only need this if the transform ends up being frame in terms of camera, which it hopefully isnt
        
        # left_camera_position = -left_rotation_matrix.T @ lt
        # right_camera_position = -right_rotation_matrix.T @ rt
        # left_rotation_matrix = left_rotation_matrix.T
        # right_rotation_matrix = right_rotation_matrix.T
        left_camera_position = lt
        right_camera_position = rt
        
        ld = np.array([np.tan(x1), np.tan(y1), 1.0])
        rd = np.array([np.tan(x2), np.tan(y2), 1.0])
        ld /= np.linalg.norm(ld)
        rd /= np.linalg.norm(rd)
        ld = left_rotation_matrix @ ld
        rd = right_rotation_matrix @ rd
                
        A = np.column_stack((ld, -rd))
        b = right_camera_position - left_camera_position

        st, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

        s = st[0]
        t = st[1]
        
        p_left = left_camera_position + s * ld
        p_right = right_camera_position + t * rd
        
        point = (p_left + p_right) / 2
        
        x, y, z = point
        
        return x, y, z
    
    def find_corresponding_points(self, points1, points2):
        # this assumes V is the relative same on both.
        MAX_V_DIFF = 20
        num_points = min(len(points1), len(points2))
        if num_points == 0:
            return []
        
        remaining1 = points1.copy()
        remaining2 = points2.copy()
        
        points = []
        for _ in range(num_points):
            best_pair = []
            best_diff = float('inf')
            for pair1 in remaining1:
                for pair2 in remaining2:
                    diff = abs(pair1[1] - pair2[1])
                    if diff < best_diff:
                        best_pair = [pair1, pair2]
                        best_diff = diff
            if best_diff > MAX_V_DIFF:
                break
            pair1, pair2 = best_pair
            remaining1.remove(pair1)
            remaining2.remove(pair2)
            points.append([pair1, pair2])
        
        return points
            
    def left_img_cb(self, msg : Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.left_contours = self.get_contours(image)
    
    def right_img_cb(self, msg : Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.right_contours = self.get_contours(image)
        
        
        
def main(args=None):
    rclpy.init(args=args)

    node = BlockDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()