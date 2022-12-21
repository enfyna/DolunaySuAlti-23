import cv2 as cv
import numpy as np
from wandb import Classes

# modelin okunmasi
net = cv.dnn.readNet("yolov4-tiny-custom_best.weights",
                     "yolov4-tiny-custom.cfg")


#obje isimlerinin okunmasi
clasees = []
with open("obj.names", 'r') as f:
    classes = [line.strip() for line in f.readlines()]

layer_name = net.getLayerNames()
output_layer = [layer_name[i - 1] for i in net.getUnconnectedOutLayers()]


#cember renkleri
colors = np.random.uniform(0, 255, 0)
coloryesil= (0,255,0)
colorsiyah= (255,255,255)
colorrec = (255,255,255)


#goruntunun alınması 0 gomulu 1 harici
cap = cv.VideoCapture(0)


#objenin nerede oldugunu donduren kod
def whereObject(objX,objY,capX,capY):
    whereX= objX-capX
    if(whereX<0):
        return "left"
    elif(whereX>0):
        colorrec=coloryesil
        return "right"
    else : 
        colorrec=coloryesil
        return "middle"



#camera acıksa goruntunun islenmesi
while(cap.isOpened()):
    _,frame=cap.read()

    # frame in alıması
    img =frame
    img = cv.resize(img, None, fx=1, fy=1)#boyutunun degistirilmesi
    height, width, channel = img.shape#frame in ozelliklerinin alinmasi

    # Detect Objects https://pyimagesearch.com/2017/11/06/deep-learning-opencvs-blobfromimage-works/
    blob = cv.dnn.blobFromImage(
        img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layer)

    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:#dogruluk
                # Object detection
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                # Reactangle Cordinate
                x = int(center_x - w/2)
                y = int(center_y - h/2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

# print(len(boxes))
# number_object_detection = len(boxes)
    indexes = cv.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    print(indexes)
    recW =(((int)(width/2))-30, ((int)(height/2))-30)
    recH =(((int)(width/2))+30, ((int)(height/2))+30)
    cv.rectangle(img, recW,recH, coloryesil, 1)
    
    font = cv.FONT_HERSHEY_PLAIN
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            
            whereObj= whereObject(x+(int)(w/2),y+(int)(h/2),(int)(width/2),(int)(height/2))
            label = str(classes[class_ids[i]])
            # print(label)
            #color = colors[i]
            cv.line(img,(x + int(w/2), y + int(h/2)),((int)(width/2),(int)(height/2)),0x001300,3)
            cv.rectangle(img, (x, y), (x + w, y + h), coloryesil, 2)
            cv.putText(img, label, (x, y + 30), font, 3, 0xFFF, 3)
            cv.putText(img, whereObj, (x, y +50), font, 3, 0x121212, 3)

    cv.imshow("IMG", img)
    cv.waitKey(1)
    if cv.waitKey(33) == ord('q'):
        break
cap.release()
cv.destroyAllWindows()
