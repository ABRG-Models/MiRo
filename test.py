import boto3
import io
from PIL import Image, ImageDraw, ImageFont
import os

rekognition = boto3.client('rekognition', region_name='us-east-2')
collectionId = 'primary_user'
# create a collection

#rekognition.create_collection(CollectionId=collectionId)
path = '/home/miro/jodie/MiRo/lib/fr_lib/train'
for r, d, f in os.walk(path):
    for file in f:
        if file != '.DS_Store':
            sourceFile = os.path.join(r, file)
            imageSource = open(sourceFile, 'rb')
            # adding faces to a Collection
            response = rekognition.index_faces(Image={'Bytes': imageSource.read()}, ExternalImageId='jodie', CollectionId=collectionId)
rekognition.describe_collection(CollectionId=collectionId)

# test function: index_faces
# for faceRecord in response['FaceRecords']:
#     print('  Face ID:  ' + faceRecord['Face']['FaceId'])
#     print('  Location: {}'.format(faceRecord['Face']['BoundingBox']))

# face detection
imageSource=open('/home/miro/jodie/MiRo/lib/fr_lib/test/pic.jpg','rb')
resp = rekognition.detect_faces(Image={'Bytes': imageSource.read()})
all_faces = resp['FaceDetails']
len(all_faces)


image = Image.open("/home/miro/jodie/MiRo/lib/fr_lib/test/pic.jpg")
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
        # fnt = ImageFont.truetype('/Library/Fonts/Arial.ttf', 15)
        # draw.text((x1, y2), response['FaceMatches'][0]['Face']['ExternalImageId'], font=fnt, fill=(255, 255, 0))
        draw.text((x1, y2), response['FaceMatches'][0]['Face']['ExternalImageId'], fill=(255, 255, 0))
image.show()
