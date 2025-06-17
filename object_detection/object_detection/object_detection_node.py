import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import pcl
import numpy as np
import tf2_ros
from tf2_ros import TransformException, ConnectivityException
from custom_msgs.msg import DetectedSurfaces, DetectedObjects
from typing import List, Tuple, Union

class ObjectDetection(Node):
    def __init__(self) -> None:
        super().__init__('object_detection_node')
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/wrist_rgbd_depth_sensor/points',
            self.callback,
            10)
        self.surface_pub = self.create_publisher(
            MarkerArray,
            'bench_markers',
            10)
        self.surface_detected_pub = self.create_publisher(
            DetectedSurfaces,
            'surface_detected',
            10)
        self.objects_pub = self.create_publisher(
            MarkerArray,
            'object_markers',
            10)
        self.object_detected_pub = self.create_publisher(
            DetectedObjects,
            'object_detected',
            10)
        self.marker_id = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def callback(self, msg: PointCloud2) -> None:
        try:
            cloud = self.from_ros_msg(msg)
            filtered_cloud_plane = self.filter_cloud(cloud, max_x_dist=3, min_height=-0.05, max_height=0.1)
            filtered_cloud_objects = self.filter_cloud(cloud, max_x_dist=3, min_height=0.0, max_height=1.0)
            plane_indices, plane_coefficients, plane_cloud = self.extract_plane(filtered_cloud_plane)
            bench_clusters, surface_centroids, surface_dimensions = self.extract_clusters(plane_cloud, "Bench Surface")
            object_clusters, object_centroids, object_dimensions = self.extract_clusters(filtered_cloud_objects, "Object")

            self.pub_surface_marker(surface_centroids, surface_dimensions)
            self.pub_object_marker(object_centroids, object_dimensions)
            self.pub_surface_detected(surface_centroids, surface_dimensions)
            self.pub_object_detected(object_centroids, object_dimensions)

        except (TransformException, ConnectivityException) as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")

    def from_ros_msg(self, msg: PointCloud2) -> Union[pcl.PointCloud, None]:
        try:
            transform = self.tf_buffer.lookup_transform('base_link',
                                                        msg.header.frame_id,
                                                        rclpy.time.Time(),
                                                        timeout=rclpy.time.Duration(seconds=1.0))
            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])
            rotation_quaternion = np.array([transform.transform.rotation.x,
                                            transform.transform.rotation.y,
                                            transform.transform.rotation.z,
                                            transform.transform.rotation.w])
            rotation_matrix = self.quaternion_to_rotation_matrix(rotation_quaternion)

            point_step = msg.point_step
            num_points = len(msg.data) // point_step
            points = []
            for i in range(num_points):
                start_index = i * point_step
                x = np.frombuffer(msg.data[start_index:start_index + 4], dtype=np.float32)[0]
                y = np.frombuffer(msg.data[start_index + 4:start_index + 8], dtype=np.float32)[0]
                z = np.frombuffer(msg.data[start_index + 8:start_index + 12], dtype=np.float32)[0]
                point = np.array([x, y, z])
                rotated_point = np.dot(rotation_matrix, point)
                relative_point = rotated_point + translation
                points.append(relative_point)

            data = np.array(points, dtype=np.float32)
            assert data.shape[1] == 3, "Number of fields must be 3"
            cloud = pcl.PointCloud()
            cloud.from_array(data)
            return cloud

        except (TransformException, ConnectivityException) as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in from_ros_msg: {e}")
            return None

    def quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        x, y, z, w = q
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,       1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
        ])

    def filter_cloud(self, cloud: pcl.PointCloud, max_x_dist: float, min_height: float, max_height: float) -> Union[pcl.PointCloud, None]:
        try:
            indices = [i for i in range(cloud.size) if (cloud[i][0] <= max_x_dist and min_height <= cloud[i][2] <= max_height)]
            return cloud.extract(indices)
        except Exception as e:
            self.get_logger().error(f"Error in filter_cloud: {e}")
            return None

    def extract_plane(self, cloud: pcl.PointCloud) -> Tuple[np.ndarray, np.ndarray, pcl.PointCloud]:
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)
        indices, coefficients = seg.segment()
        plane_cloud = cloud.extract(indices)
        self.get_logger().info(f"Number of inliers: {len(indices)}")
        self.get_logger().info(f"Plane coefficients: {coefficients}")
        return indices, coefficients, plane_cloud

    def extract_clusters(self, cloud: pcl.PointCloud, cluster_type: str) -> Tuple[List[pcl.PointCloud], List[List[float]], List[List[float]]]:
        tree = cloud.make_kdtree()
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.02)
        ec.set_MinClusterSize(100)
        ec.set_MaxClusterSize(80000)
        ec.set_SearchMethod(tree)

        cluster_indices = ec.Extract()
        bench_clusters = []
        cluster_centroids = []
        cluster_dimensions = []

        for idx, indices in enumerate(cluster_indices):
            self.get_logger().info(f"Processing {cluster_type} cluster {idx + 1}...")
            cluster = cloud.extract(indices)
            centroid = np.mean(cluster, axis=0)
            min_coords = np.min(cluster, axis=0)
            max_coords = np.max(cluster, axis=0)
            dimensions = max_coords - min_coords

            bench_clusters.append(cluster)
            cluster_centroids.append(centroid.tolist())
            cluster_dimensions.append(dimensions.tolist())

            num_points = len(indices)
            self.get_logger().info(f"{cluster_type} cluster {idx + 1} has {num_points} points.")
            self.get_logger().info(f"Centroid: {centroid}")
            self.get_logger().info(f"Dimensions: {dimensions}")

        if not bench_clusters:
            self.get_logger().warning(f"No {cluster_type} clusters extracted...")

        return bench_clusters, cluster_centroids, cluster_dimensions

    def pub_surface_marker(self, surface_centroids: List[List[float]], surface_dimensions: List[List[float]]) -> None:
        marker_array = MarkerArray()
        surface_thickness = 0.05

        def clamp(val: float, min_val: float = -0.01) -> float:
            return val if val > 0 else min_val

        for idx, (centroid, dimensions) in enumerate(zip(surface_centroids, surface_dimensions)):
            length = clamp(float(dimensions[0]))
            width = clamp(float(dimensions[1]))

            cube_marker = Marker()
            cube_marker.header.frame_id = "base_link"
            cube_marker.id = idx
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
            cube_marker.pose.position.x = centroid[0]
            cube_marker.pose.position.y = centroid[1]
            cube_marker.pose.position.z = centroid[2] - surface_thickness /2.6
            cube_marker.pose.orientation.w = 1.0

            cube_marker.scale.x = length
            cube_marker.scale.y = width
            cube_marker.scale.z = surface_thickness

            cube_marker.color.r = 0.0
            cube_marker.color.g = 1.0
            cube_marker.color.b = 0.0
            cube_marker.color.a = 0.5
            marker_array.markers.append(cube_marker)

        if marker_array.markers:
            self.get_logger().info(f"Published {len(marker_array.markers)} bench plane markers")
            self.surface_pub.publish(marker_array)
        else:
            self.get_logger().warning("No bench plane markers to publish.")

    def pub_surface_detected(self, centroids: List[List[float]], dimensions: List[List[float]]) -> None:
        for idx, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
            surface_msg = DetectedSurfaces()
            surface_msg.surface_id = idx
            surface_msg.position.x = centroid[0]
            surface_msg.position.y = centroid[1]
            surface_msg.position.z = centroid[2]
            surface_msg.height = dimension[0]
            surface_msg.width = dimension[1]
            self.surface_detected_pub.publish(surface_msg)

    def pub_object_marker(self, object_centroids: List[List[float]], object_dimensions: List[List[float]]) -> None:
        marker_array = MarkerArray()
        offset_x = 0.05

        def clamp(val: float, min_val: float =-0.01) -> float:
            return val if val > 0 else min_val

        for idx, (centroid, dimensions) in enumerate(zip(object_centroids, object_dimensions)):
            length = clamp(dimensions[0])
            width = clamp(dimensions[1])
            height = clamp(dimensions[2])

            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(centroid[0]) 
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2]) 
            marker.pose.orientation.w = 1.0

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = height

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker_array.markers.append(marker)

        if marker_array.markers:
            self.get_logger().info(f"Published {len(marker_array.markers)} objects on flat surface markers!")
            self.objects_pub.publish(marker_array)
        else:
            self.get_logger().warning("No objects on flat surface markers to publish.")

    def pub_object_detected(self, centroids: List[List[float]], dimensions: List[List[float]]) -> None:
        for idx, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
            object_msg = DetectedObjects()
            object_msg.object_id = idx
            object_msg.position.x = centroid[0]
            object_msg.position.y = centroid[1]
            object_msg.position.z = centroid[2] 
            object_msg.height = dimension[0]
            object_msg.width = dimension[1]
            object_msg.thickness = dimension[2]
            self.object_detected_pub.publish(object_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    rclpy.spin(object_detection)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
