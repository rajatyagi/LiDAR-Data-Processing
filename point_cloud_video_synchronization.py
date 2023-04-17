import numpy as np
import cv2

point_cloud_data = np.load('point_cloud_match_to_video_with_second.npy')

print(point_cloud_data.shape)



# image = np.zeros([832, 832, 3], np.uint8) + 255

# out = cv2.VideoWriter('point_cloud.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 15, (832, 832))

# center = [None] * point_cloud_data.shape[0]
#
# for second in range(10):
#     for i in range(point_cloud_data.shape[0]):
#         center[i] = (int(130 + 20 * point_cloud_data[i, 0, second]), int(734 + 20 * point_cloud_data[i, 1, second]))
#
#     for i in range(len(center)):
#         synchronization_image = cv2.circle(image, center[i], radius=0, color=(0, 0, 255), thickness=-1)

    # out.write(synchronization_image)

# out.release()



'''
capture the video data
'''
cap = cv2.VideoCapture('output1640b_transformed.mp4')

second = 0
counter = 0

out = cv2.VideoWriter('synchronization.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 15, (832, 832))

while (cap.isOpened()):

    ret, frame = cap.read()
    print(frame)
    fps = cap.get(cv2.CAP_PROP_FPS)  # fps = 15

    if counter == 0:
        print('======= Processing the data at %d second =======' % second)
        center = [None] * point_cloud_data.shape[0]
        for i in range(point_cloud_data.shape[0]):
            center[i] = (int(130 + 20 * point_cloud_data[i, 0, second]), int(734 + 20 * point_cloud_data[i, 1, second]))
            # 130 and 734 here are for translating the coordinate, 20 here is for scaling the resolution of the data to match the video resolution

    for i in range(len(center)):
        synchronization_image = cv2.circle(frame, center[i], radius=0, color=(0, 0, 255), thickness=-1) # circle function is for ploting the point cloud data on the picture frame

    out.write(synchronization_image)

    counter = counter + 1
    if counter == 15:
        second = second + 1
        counter = 0

out.release()

