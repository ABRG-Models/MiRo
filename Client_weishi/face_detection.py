import cv2

# face detection

def face_detection(self, image):

    self.detected_faces = image

    face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    if len(faces) > 0:
        print(faces)
        for (x, y, w, h) in faces:
            cv2.rectangle(self.detected_faces, (x, y), (x + w, y + h), (255, 0, 0), 2)
            face_x_coord = x + w / 2;
            face_y_coord = y + h / 2;
            roi_gray = gray[y:y + h, x:x + w]
            self.roi_color = image[y:y + h, x:x + w]

    return self.detected_faces

def face_recognition(self, image):

