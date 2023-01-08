import cv2
import face_recognition
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# import rospkg

# rospkg = rospkg.RosPack()

# self.path_to_package = rospkg.get_path

image_path = "/home/toi/Pictures/thao.png"

standing_person_image = face_recognition.load_image_file(image_path)
standing_person_face_encoding = face_recognition.face_encodings(standing_person_image)[0]

face_locations = []
face_encodings = []
face_names = []
process_this_frame = True

# cap = cv2.VideoCapture(0)
# ret, frame = cap.read()
frame = cv2.imread("/home/toi/Pictures/k.png")
small_frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)

if process_this_frame:
	face_locations = face_recognition.face_locations(small_frame)
	face_encodings = face_recognition.face_encodings(small_frame, face_locations)

	face_names = []
	for face_encoding in face_encodings:
		match = face_recognition.compare_faces([standing_person_face_encoding], face_encoding)
		name = "du"

		if match[0]:
			print("MATCH")
			name = "thao"
		
		face_names.append(name)
	

process_this_frame = not process_this_frame


for (top,right, bottom, left), name in zip(face_locations, face_names):
	top *= 2
	right *= 2
	bottom *= 2
	left *= 2

	cv2.rectangle(frame, (left,top), (right,bottom),(0,0,255),2)

	cv2.rectangle(frame, (left, bottom - 35), (right,bottom),(0,0,255))
	font = cv2.FONT_HERSHEY_DUPLEX
	cv2.putText(frame, name, (left + 6, bottom - 6),font,1.0,(255,255,255),1)

cv2.imshow("img", frame)
cv2.waitKey(0)