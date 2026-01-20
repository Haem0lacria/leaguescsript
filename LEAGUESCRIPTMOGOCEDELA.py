import dxcam 
import cv2
import numpy as np
import pyautogui
import time
import keyboard 

cam =dxcam.create()

#FILTER ZA KROG RANGE TWITCHA
lower = np.array([85, 100, 100])
upper = np.array([100, 255, 255])

#HP MINIONOV 
lower_dark = np.array([0, 0, 0])
upper_dark = np.array([180, 255, 80])

prev = None

while True:
    frame = cam.grab()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    contours_range, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)# OD TU DO ELSE JE CENTER IN RADIJ KROGA OD MAX RANGE TWITCHA

    if contours_range:
        c_range = max(contours_range, key=cv2.contourArea)
        (x_range, y_range), radius = cv2.minEnclosingCircle(c_range)
        center_x = int(x_range)
        center_y = int(y_range)
        radius = int(radius)
    else:
        center_x, center_y, radius = None, None, None

    mask_low_hp = cv2.inRange(hsv, lower_dark, upper_dark)
    contours_low_hp, _ = cv2.findContours(mask_low_hp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    low_hp_targets = []
    for c_hp in contours_low_hp:
        xh, yh, wh, hh, = cv2.boundingRect(c_hp)

        #OBILIKA HP BARA
        if wh < 20 or hh > 10:
            continue

        #IZREŽE HP BAR
        hp_bar = hsv[yh:yh+hh, xh:xh+wh]

        #MASKA ZA RDEČI DEL KOLIKO MAJO SE ŽIVLJENJA
        lower_red = np.array([160, 100, 100])
        upper_red = np.array([180, 255, 255])
        mask_red = cv2.inRange(hp_bar, lower_red, upper_red)

        #ŠTEVILO RDEČIH PIXOLOV
        red_pixels = cv2.countNonZero(mask_red)

        #VSI PIXLI HP BARA
        total_pixels = wh * hh

        #RATIOOOOO
        ratio = red_pixels / total_pixels

        #KDAJ NAPADATTT
        if ratio < 0.1: #10% 10cs/MIN
            minion_cx = xh + wh // 2
            minion_cy = yh + hh // 2
            low_hp_targets.append((minion_cx, minion_cy))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    if prev is None:
        prev = gray
        continue

    diff = cv2.absdiff(prev, gray) #razlika med prejšnim in totim frame


    _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

    contours, _ =cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        prev = gray
        continue

    if contours:
        c = max (contours, key=cv2.contourArea) #KJE JE NAJVEC GIBANJA
        x, y, w, h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        if area < 500: #MAJHNO GIBANJE
            prev = gray
            continue #IGNORIRA MAJHNE PREMIKE

        #CENTER GIBANJA
        cx = x + w // 2
        cy = y + h // 2

        #IGRNORIRANJE ZUNAJ RANGE
        if center_x is not None:
            dist_sq = (cx - center_x)**2 + (cy - center_y)**2
            if dist_sq > radius**2:
                prev = gray 
                continue
            

        # PREMIK MIŠKE NA GIBANJE
        pyautogui.moveTo(cx, cy)

        #MINIONI
        if center_x is not None:
            for (mx, my) in low_hp_targets:
                dist_sq = (mx - center_x)**2 + (my - center_y)**2
                if dist_sq < radius**2: #CE JE V RANGE MAX RANGE
                    pyautogui.keyDown('x')
                    pyautogui.keyUp('x')
                    pyautogui.click(button='right')
                    break
        if center_x is not None:
            dist_sq = (cx-center_x)**2 + (cy-center_y)**2
            if dist_sq < radius**2: #CE JE V RANGE MAX RANGE
                pyautogui.keyDown('x')
                pyautogui.keyUp('x')
                pyautogui.click(button='right')            
        if keyboard.is_pressed("esc"):
            break

prev = gray
