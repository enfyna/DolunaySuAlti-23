import numpy as np
import cv2

class Gorev:

    def __init__(self,baglanti_modu,cap):
        # Gorevde kullanılacaklari ayarla
            # Kamerayi al ve cozunurlugu ayarla
        if baglanti_modu == "USB":
            self.cap = cap
            self.cap_x , self.cap_y = int(cap.get(3)) , int(cap.get(4))
            self.cap_hx , self.cap_hy = int(self.cap_x/2) , int(self.cap_y/2) 
        elif baglanti_modu == "SITL":
            self.cap_x , self.cap_y , self.cap_hx , self.cap_hy = 500 , 500 , 250 , 250
            
            # Renk aralıgı
        self.lower_cyan = np.array([50, 60, 170])
        self.upper_cyan = np.array([100, 255, 210])
            # Erode-Dilate kernel
        self.kernel = np.ones((5, 5), np.float32)
        pass

    def calistir(self,baglanti_modu):

        if str(baglanti_modu) == "USB":
            _, frame = cap.read()
        else: #SITL
            q = int(baglanti_modu)
            yol = "/home/oem/Documents/Kod/Python/pixhawk/Gorev1/v3/"
            dosya = str(yol)+((str(q)+".png").zfill(8))
            try:
                frame = cv2.imread(dosya)#cap.read() #Kare al
                if frame is None:
                    frame = 1/0
                q+=1
            except:
                q = 200
                dosya = str(yol)+((str(q)+".png").zfill(8))
                frame = cv2.imread(dosya)#cap.read() #Kare al

        # Bulanıklastır
        frame_blur = cv2.GaussianBlur(frame, (5, 5), cv2.BORDER_DEFAULT)
        #cv2.imwrite(yol+"blur.png", frame_blur)

        # Renkleri tersine cevir
        frame_inv = cv2.bitwise_not(frame_blur)
        #cv2.imwrite(yol+'inv.png', frame_inv)

        # HSV'ye donustur
        frame_hsv = cv2.cvtColor(frame_inv, cv2.COLOR_BGR2HSV)

        # inRange ile camgobegi rengini bul
        mask = cv2.inRange(frame_hsv, self.lower_cyan, self.upper_cyan)
        #cv2.imwrite(yol+'mask.png', mask)
        #cv2.imshow("mask",mask)

        # Erode-dilate yap
        #mask = cv2.erode(mask, kernel, iterations=2)
        #mask = cv2.dilate(mask, kernel, iterations=1)
        #cv2.imwrite(yol+'erode-dilate.png', mask)
        #cv2.imshow("ed",mask)

        # contour listesini al
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cx , cy = None , None

        for c in contours:
            approx = cv2.approxPolyDP(
                c, 0.01 * cv2.arcLength(c, True), True)       
            
            if len(approx) < 8:
                continue
            else:
                # finding center point of shape
                M = cv2.moments(c)
                if M['m00'] != 0.0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                #self.cizgiciz(self,cap_hx,cap_hy,cx,cy)
                frame = self.cizgiciz(frame, c, cx, cy)
        
        cv2.imshow("Renk Tespit", frame)
        #cv2.imwrite(yol+'son.png', frame)
        
        if cv2.waitKey(1) == ord("q"):
            pass

        if cx is not None:
            cx = cx - self.cap_hx
        if cy is not None:
            cy = cy - self.cap_hy
        return cx , cy

    def cizgiciz(self,frame,c,cx,cy):
        s = ""
        if cx >= self.cap_hx:
            s = "Sag-"
        elif cx < self.cap_hx:
            s = "Sol-"
        if cy <= self.cap_hy:
            s += "Ust"
        elif cy > self.cap_hy:
            s += "Alt"
        cv2.putText(frame, s,(self.cap_hx, self.cap_hy),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
        cv2.line(frame, (int(cx), int(cy)), (self.cap_hx, self.cap_hy), (0, 255, 0), 2)
        cv2.drawContours(frame, [c], 0, (0, 255 ,0 ), 2)
        return frame

    def cap_yoket(self):
        self.cap.release()
