import node
import boto3
import io
import os
import cv2
import numpy as np
from PIL import Image

class NodeFaceRecognition(node.Node):

    def __init__(self, sys):
        node.Node.__init__(self, sys, "face_recognition")

        self.rekognition = boto3.client('rekognition', region_name='us-east-2')
        self.collectionId = 'primary_user'
        try:
            # create a collection
            self.rekognition.create_collection(CollectionId=self.collectionId)
        except:
            pass

        # add training faces into the collection
        self.path_train = '/home/miro/jodie/MiRo/lib/fr_lib/train'
        username = 'USER'
        for r, d, f in os.walk(self.path_train):
            for file in f:
                if file != '.DS_Store':
                    sourceFile = os.path.join(r, file)
                    imageSource = open(sourceFile, 'rb')
                    # adding faces to a Collection
                    self.rekognition.index_faces(
                            Image={'Bytes': imageSource.read()},
                            ExternalImageId=username,
                            CollectionId=self.collectionId)

    def tick_camera(self, stream_index):
        # faces is the array of detected faces from node_detect_face
        faces = self.state.detect_face[stream_index]
        img = self.state.frame_raw[stream_index]

        for face in faces:
            (x, y, w, h, conf) = face
            face = img[x:x+w, y:y+h]

            array = cv2.cvtColor(np.array(face), cv2.COLOR_RGB2BGR)
            image = Image.fromarray(array)

            # image convert to binary
            stream = io.BytesIO()
            image.save(stream, format="JPEG")
            image_binary = stream.getvalue()

            # use image search faces in the Collection
            try:
                response = self.rekognition.search_faces_by_image(
                    CollectionId=self.collectionId,
                    Image={'Bytes': image_binary}
                )

                if len(response['FaceMatches']) > 0:
                    # store
                    self.state.primary_user = True
                    print('=====FACE MATCH=====')
            except:
                pass
