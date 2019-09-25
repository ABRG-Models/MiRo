import cv2
import boto3
import io
import os
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

# face detection
class detect_primary_user:

    def face_detection (self, image):

        detected_faces = image
        roi_color = None
        face_x_coord = None
        face_y_coord = None
        dic_faces = {}
        
        face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        if len(faces) > 0:
            for (x, y, w, h) in faces:
                # index of face
                face = 0

                cv2.rectangle(detected_faces, (x, y), (x + w, y + h), (255, 0, 0), 2)
                # face_x_coord = x + w / 2
                # face_y_coord = y + h / 2
                roi_color = image[y:y + h, x:x + w]
                print('======roi_color', roi_color)
                dic_faces[face] = [roi_color, [x, y, w, h]]
                face += 1

        return detected_faces, dic_faces


    def face_collection(self):
        username = 'MOM'
        path = self.path_train
        rekognition = self.rekognition
        collectionId = self.collectionId
        for r, d, f in os.walk(path):
            for file in f:
                if file != '.DS_Store':
                    sourceFile = os.path.join(r, file)
                    imageSource = open(sourceFile, 'rb')
                    # adding faces to a Collection
                    rekognition.index_faces(
                            Image={'Bytes': imageSource.read()}, 
                            ExternalImageId=username, 
                            CollectionId=collectionId)


    def face_recognition(self, dic_faces):
        primary = False
        face_user = []

        for key, value in dic_faces.items():
            face = value[0]
            print('------', face)

            array = cv2.cvtColor(np.array(face), cv2.COLOR_RGB2BGR)
            image = Image.fromarray(array)
            #image convert to binary
            stream = io.BytesIO()
            image.save(stream, format="JPEG")
            image_binary = stream.getvalue()
            #use image search faces in the Collection
            try:
                response = self.rekognition.search_faces_by_image(
                            CollectionId=self.collectionId,
                            Image={'Bytes': image_binary}
                        )

                if len(response['FaceMatches']) > 0:
                    primary = True
                    face_user = value[1]
                        # x_face = (x1 + x2) / 2.0
                        # y_face = (y1 + y2) / 2.0
                    # print('=====FACE MATCH=====')
                        # plt.imshow(image)
                        # plt.text(x_ff, y_ff, 'PRIMARY USER')
                        # plt.axis('off')
                        # mngr = plt.get_current_fig_manager()
                        # mngr.window.wm_geometry('+380+310')
                        # plt.pause(2)
                        # plt.ioff
                        # plt.clf()
                        # plt.close()

                        # points = ((x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, y1))
                        # draw = ImageDraw.Draw(image)
                        # draw.line(points, fill='#00d400', width=2)
                        # draw.text((x_face, y_face), response['FaceMatches'][0]['Face']['ExternalImageId'], fill=(255, 255, 0))
                #image.show()
            except:
                pass

        return primary, face_user

    def __init__ (self):
        self.rekognition = boto3.client('rekognition', region_name='us-east-2')
        self.collectionId = 'primary_user'
        # create a collection
        # rekognition.create_collection(CollectionId=collectionId)
        # self.path of the training pics library of the primary user
        self.path_train = '/home/miro/jodie/MiRo/lib/fr_lib/train'






