import velodyne_decoder as vd
import numpy as np
from datetime import datetime


def rotation_matrix(theta1, theta2, theta3, order='xyz'):
    c1 = np.cos(theta1 * np.pi / 180)
    s1 = np.sin(theta1 * np.pi / 180)
    c2 = np.cos(theta2 * np.pi / 180)
    s2 = np.sin(theta2 * np.pi / 180)
    c3 = np.cos(theta3 * np.pi / 180)
    s3 = np.sin(theta3 * np.pi / 180)
    matrix = np.array([[c2 * c3, -c2 * s3, s2],
                       [c1 * s3 + c3 * s1 * s2, c1 * c3 - s1 * s2 * s3, -c2 * s1],
                       [s1 * s3 - c1 * c3 * s2, c3 * s1 + c1 * s2 * s3, c1 * c2]])
    return matrix


'''
Extract LiDAR data from pcap files
'''

config = vd.Config(model='VLP-32C', rpm=600)
pcap_file = '../LiDAR_data.pcap'

data_length = 2000000
video_length = 60 # unit in second


cloud_arrays = np.zeros((data_length, 6))
point_cloud_data = np.zeros((data_length, 2, video_length)) # only get x,y data

data_index = 0
counter = 0 # there are 30 collected data in one second
second = 0

for stamp, points in vd.read_pcap(pcap_file, config):
    points = np.array(points)

    if counter != 30:
        counter = counter + 1
        print("Real time: ", datetime.fromtimestamp(stamp))
        print("Point.shape: ", points.shape)
        cloud_arrays[data_index:data_index + points.shape[0]] = points
        data_index = data_index + points.shape[0]
        print("data_index: ", data_index)
    if counter == 30:
        counter = 0
        print("======= Processing the data at %d second =======" % second)
        data_xyz = cloud_arrays[0:data_index, 0:3]
        print(data_xyz.shape)
        data_xyz = rotation_matrix(0, 0, 60).dot(data_xyz.T).T
        data_xyz[:, 1] = -data_xyz[:, 1]
        for i in range(data_xyz.shape[0]):
            point_cloud_data[i, :, second] = data_xyz[i, 0:2]
        cloud_arrays = np.zeros((data_length, 6))
        data_index = 0
        second = second + 1

    if second == 60: # Cuz the length of the video is 5 minutes = 300 seconds
        break




print(point_cloud_data.shape)
np.save('point_cloud_match_to_video_with_second.npy', point_cloud_data)



# center = [None] * len(data_xyz_rotation_projection)
# for i in range(len(data_xyz_rotation_projection)):
#     center[i] = (int(130 + 20 * data_xyz_rotation_projection[i, 0]), int(734 + 20 * data_xyz_rotation_projection[i, 1]))





# for i in range(len(data_xyz_rotation_projection)):
#     synchronization_image = cv2.circle(video_image, center[i], radius=0, color=(0, 0, 255), thickness=-1)
# cv2.imshow('image', synchronization_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(data_xyz_rotation)
# # pcd.rotate(pcd.get_rotation_matrix_from_xyz((0, 0, 1.03)))
# vis = o3d.visualization.Visualizer()
# vis.create_window()
# vis.add_geometry(pcd)
# # o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 0.015)
# # o3d.visualization.ViewControl.translate(vis.get_view_control(), x=-1000, y=1000, xo=0, yo=0)
# vis.run()
# # vis.capture_screen_image('point cloud.png')