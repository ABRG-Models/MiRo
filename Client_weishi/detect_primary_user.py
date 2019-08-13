import cv2
import boto3
import io
import os
from PIL import Image, ImageDraw, ImageFont

# face detection

class detect_primary_user:

    def face_detection (self, image):

        detected_faces = image
        roi_color = None


        face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                cv2.rectangle(detected_faces, (x, y), (x + w, y + h), (255, 0, 0), 2)
                face_x_coord = x + w / 2;
                face_y_coord = y + h / 2;
                roi_gray = gray[y:y + h, x:x + w]
                roi_color = image[y:y + h, x:x + w]

        return detected_faces, roi_color

    #save face
    def save_face(self, face):
        cv2.imwrite(self.path_test, face)


    def face_recognition(self, face):
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
                    response = rekognition.index_faces(Image={'Bytes': imageSource.read()}, ExternalImageId=username, CollectionId=collectionId)

        # face search
        imageSource=open(self.path_test,'rb')
        resp = rekognition.detect_faces(Image={'Bytes': imageSource.read()})
        all_faces = resp['FaceDetails']
        len(all_faces)


        image = Image.open(self.path_test)
        image_width, image_height = image.size

        for face in all_faces:

            box = face['BoundingBox']
            x1 = box['Left'] * image_width
            y1 = box['Top'] * image_height
            x2 = x1 + box['Width'] * image_width
            y2 = y1 + box['Height'] * image_height

        
            #get only face     
            image_crop = image.crop((x1, y1, x2, y2))

            #image convert to binary
            stream = io.BytesIO()
            image.save(stream, format="JPEG")
            image_crop_binary = stream.getvalue()
        
            #use image search faces in the Collection     
            response = rekognition.search_faces_by_image(
                    CollectionId=collectionId,
                    Image={'Bytes': image_crop_binary}
                    )

            if len(response['FaceMatches']) > 0:
                draw = ImageDraw.Draw(image)
                points = (
                            (x1, y1),
                            (x2, y1),
                            (x2, y2),
                            (x1, y2),
                            (x1, y1)
                        )
                draw.line(points, fill='#00d400', width=2)
                draw.text((x1, 0), response['FaceMatches'][0]['Face']['ExternalImageId'], fill=(255, 255, 0))
        image.show()

    def __init__ (self):
        self.rekognition = boto3.client('rekognition', region_name='us-east-2')
        self.collectionId = 'primary_user'
        # create a collection
        # rekognition.create_collection(CollectionId=collectionId)
        # self.path of the training pics library of the primary user
        self.path_train = '/home/miro/jodie/MiRo/lib/fr_lib/train'
        self.path_test = './user_pic.png'






