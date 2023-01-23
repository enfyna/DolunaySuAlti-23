import cv2
import numpy as np
import cProfile

def cizgiciz(cap_hx,cap_hy,cx,cy):
    s = ""
    if cx >= cap_hx:
        s = "Sag-"
    elif cx < cap_hx:
        s = "Sol-"
    if cy <= cap_hy:
        s += "Ust"
    elif cy > cap_hy:
        s += "Alt"
    cv2.putText(frame, s, (cap_hx, cap_hy),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
    cv2.line(frame, (int(cx), int(cy)), (cap_hx, cap_hy), (0, 255, 0), 2)
    cv2.drawContours(frame, [c], 0, (0, 255 ,0 ), 2)

if __name__ == '__main__':

    profiler = cProfile.Profile()
    profiler.enable()

    cap = cv2.VideoCapture(0)

    video_x, video_y = 500, 500

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_x)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_y)

    cap_hx , cap_hy = int(cap.get(3)/2) , int(cap.get(4)/2)
    #cap_hx, cap_hy = int(video_x/2), int(video_y/2)

    # Renk araligi
    lower_cyan = np.array([50, 60, 190])
    upper_cyan = np.array([90, 90, 210])

    # Erode-Dilate kernel
    kernel = np.ones((5, 5), np.float32)

    while True:
    #for _ in range(1):

        _, frame = cap.read()
        yol = "/home/oem/Documents/Kod/Python/pixhawk/Gorev1/ktr/"
        #frame = cv2.imread(yol+"frame.png")

        # Bulanıklastır
        frame_blur = cv2.GaussianBlur(frame, (5, 5), cv2.BORDER_DEFAULT)
        #cv2.imwrite(yol+"blur.png", frame_blur)

        # Renkleri tersine cevir
        frame_inv = cv2.bitwise_not(frame_blur)
        #cv2.imwrite(yol+'inv.png', frame_inv)

        # HSV'ye donustur
        frame_hsv = cv2.cvtColor(frame_inv, cv2.COLOR_BGR2HSV)

        # inRange ile camgobegi rengini bul
        mask = cv2.inRange(frame_hsv, lower_cyan, upper_cyan)
        #cv2.imwrite(yol+'mask.png', mask)
        cv2.imshow("mask",mask)

        # Erode-dilate yap
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=1)
        #cv2.imwrite(yol+'erode-dilate.png', mask)
        cv2.imshow("ed",mask)

        # contour listesini al
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            approx = cv2.approxPolyDP(
                c, 0.01 * cv2.arcLength(c, True), True)       
            
            if len(approx) < 6:
                continue
            else:
                # finding center point of shape
                M = cv2.moments(c)
                if M['m00'] != 0.0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                cizgiciz(cap_hx,cap_hy,cx,cy)
        
        cv2.imshow("Renk Tespit", frame)
        #cv2.imwrite(yol+'son.png', frame)

        if cv2.waitKey(1) == ord("q"):
            break
    
    cap.release()
    cv2.destroyAllWindows()

    profiler.disable()
    profiler.print_stats("tottime")

