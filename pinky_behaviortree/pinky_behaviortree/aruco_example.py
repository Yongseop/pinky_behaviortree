import cv2
import numpy as np
import cv2.aruco as aruco

# 웹캠 초기화
cap = cv2.VideoCapture(0)

# 카메라 파라미터
camera_matrix = np.array([[800, 0, 320],
                        [0, 800, 240],
                        [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5,1), dtype=np.float32)

# ArUco 설정
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

# 마커 크기
marker_size = 0.05  # 5cm = 0.05m

while True:
   ret, frame = cap.read()
   if not ret:
       break
   
   gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
   
   if ids is not None:
       aruco.drawDetectedMarkers(frame, corners, ids)
       rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
       
       for i in range(len(ids)):
           # 좌표축 그리기
           cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.03)
           
           # 회전 행렬 계산
           rot_matrix, _ = cv2.Rodrigues(rvecs[i])
           euler_angles = np.zeros(3)

           # Roll (x-axis rotation)
           euler_angles[0] = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
           # Pitch (y-axis rotation)
           euler_angles[1] = np.arctan2(-rot_matrix[2, 0], np.sqrt(rot_matrix[2, 1]**2 + rot_matrix[2, 2]**2))
           # Yaw (z-axis rotation)
           euler_angles[2] = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])

           # 각도를 degree로 변환
           euler_angles = np.degrees(euler_angles)
           
           # 거리 계산
           distance = np.sqrt(tvecs[i][0][0]**2 + tvecs[i][0][1]**2 + tvecs[i][0][2]**2)
           
           # 정보 표시
           text_position = (10, 30 * (i + 1))
           cv2.putText(frame, f'Marker {ids[i][0]}:', (10, 30 * (i + 1)), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
           cv2.putText(frame, f'Distance: {distance:.2f}m', (10, 30 * (i + 2)), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
           cv2.putText(frame, f'Roll: {euler_angles[0]:.1f}deg', (10, 30 * (i + 3)), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
           cv2.putText(frame, f'Pitch: {euler_angles[1]:.1f}deg', (10, 30 * (i + 4)), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
           cv2.putText(frame, f'Yaw: {euler_angles[2]:.1f}deg', (10, 30 * (i + 5)), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
           
   cv2.imshow('ArUco Detection', frame)
   
   if cv2.waitKey(1) & 0xFF == ord('q'):
       break

cap.release()
cv2.destroyAllWindows()
