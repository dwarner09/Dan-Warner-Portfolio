import math
import time
import cv2 as cv
import random
import numpy as np

para_open = 0
img1 = cv.imread('newmarszoom2.jpg')
H, W = img1.shape[:2]
video = cv.VideoWriter('edl.avi', cv.VideoWriter_fourcc('M','J','P','G'), 4, (W, H))


def edl_simulator():
    y = random.randint(500, 1000)
    corrected_land_x = 0
    predict_land_x = 0
    g = 3.711  # m/s^2
    landing_target_x = 1150*1000 + 500000
    # Spacecraft Dimensional Parameters (Fixed)
    mass = 2000  # kg
    area_sc = 75  # m^2
    area_para = 0  # (750 when deployed) # m^2
    Cd_sc = 0.8
    Cd_para = 1.75
    status_3 = " "
    deployed_status = 0
    # randomizer for entry window in x (lateral) axis
    xo = random.randint(480000, 600000)
    x = xo
    # randomizer for entry velocity
    vx = random.randint(5700, 5900)
    # randomizer for altitude of entry
    zo = random.randint(124000, 127000)
    z = zo
    # Z (vertical) velocity starts at 0(perpendicular to ground))
    vzo = 0
    vz = vzo
    # Temperature Randomizer (outputs percentage multiplier between 92-108) changes every 20 s. Impacts air density
    temp_rand = random.randint(92, 108)/100

    start_time = int(time.time())
    clock_time = start_time
    temp_time = start_time

    while z > 1:
        global para_open
        if time.time() - clock_time > 1:
            t = time.time() - clock_time
            clock_time = time.time()
            alt = z
            p = 0.699 * math.exp(-0.00009 * alt)
            p_status = "OFF"
            if clock_time - temp_time  > 20:
                temp_rand = random.randint(92, 108)/100
                temp_time = time.time()
            if alt > 60000:
                T = -156 * temp_rand
            elif alt > 7000:
                T = (-23.4 - (0.00222 * alt)) * temp_rand
            else:
                T = (-31 - (0.000998 * alt)) * temp_rand

            ro = p / (0.1921 * (T + 273.1))

            if ro > 0:
                pass
            else:
                ro = 0.000001

            if z < 3100: # variable from computer
                area_para = 0
                p_status = "Ejected"
            elif z < 15000 + para_open: # variable from computer
                area_para = 750
                p_status = "Deployed"
                deployed_status += 1

            if z < 900: # variable from computer
                z_thrust = (g + vz**2*.8/alt)*mass  # Force Newtons
                if vz > -1.1:
                    z_thrust = g*mass
            else:
                z_thrust = 0

            if 600 < z < 3000:
                x_thrust = ((landing_target_x - x - (vx * (z / (-vz)))) * 2 * mass) / (z / vz) ** 2
                if x_thrust > 6000:
                    x_thrust = 6000
                if x_thrust < -6000:
                    x_thrust = -6000
                if vx > 200:
                    x_thrust = 0
                if vx < -200:
                    x_thrust = 0
            elif z < 600:
                x_thrust = mass * (-vx) / (z / -vz)
                if x_thrust > 6000:
                    x_thrust = 6000
                if x_thrust < -6000:
                    x_thrust = -6000
            else:
                x_thrust = 0

            thrust = math.sqrt(x_thrust**2 + z_thrust**2)

            if thrust > 0:
                t_status = "ON"
            else:
                t_status = "OFF"

            sim_x = x + vx*t - (0.25*((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vx**2 * t**2)/mass + \
                0.5*x_thrust/mass*t**2
            vx = vx - (0.5*((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vx**2 * t)/mass + x_thrust/mass*t

            z = z + vz*t - (0.25*((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vz**2 * t**2)/mass - 0.5*g*t**2 + \
                0.5*z_thrust/mass*t**2
            vz = vz - g*t + z_thrust/mass*t + (0.5*((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vz**2 * t)/mass
            Vt_z = math.sqrt((2 * mass * g)/(((Cd_sc * area_sc) + (Cd_para * area_para))*ro))
            if abs(vz) > Vt_z:
                vz = -Vt_z

            status_1 = "alt = " + str(z) + "m" + " vz = " + str(vz) + "m/s " + "x = " + str(x/1000) + \
                       "km" + " vx = " + str(vx) + "m/s"
            # print(status_1)
            status_2 = "Parachute: " + str(p_status) + " Thrust: " + str(t_status) + " Thrust = " + \
                       str(thrust/1000) + "kN"
            # print(status_2)

            x = terrain_relative_nav(sim_x, status_1, status_2, corrected_land_x, predict_land_x, y, status_3)
            predict_land_x, status_3 = spacecraft_computer(z, vz, x, vx, deployed_status)
            corrected_land_x = corrected_landing(z, vz, x, vx, para_open)


def spacecraft_computer(z, vz, x, vx, deployed_status):
    global para_open
   # estimated entry parameters = downrange distance 1261 km
    landing_target_x = 1150*1000 + 500000
    mass = 2000      # kg
    g = 3.711        # m/s^2
    area_sc = 75    # m^2
    area_para = 0 # m^2
    Cd_sc = 0.8
    Cd_para = 1.75

    t = 1

    while z > 1:
        alt = z
        p = 0.699 * math.exp(-0.00009 * alt)

        if alt > 60000:
            T = -156
        elif alt > 7000:
            T = -23.4 - (0.00222 * alt)
        else:
            T = -31 - (0.000998 * alt)

        ro = p / (0.1921 * (T + 273.1))

        if ro > 0:
            pass
        else:
            ro = 0.000001

        if z < 3100:
            area_para = 0
        elif z < 15000:
            area_para = 750

        if z < 900:
            thrust = (g + vz**2*.8/alt)*mass  # Force Newtons
            if vz > -1.1:
                thrust = g*mass
            thrust_vector = 0
            z_thrust = math.cos(thrust_vector)*thrust
            x_thrust = math.sin(thrust_vector)*thrust
        else:
            thrust = 0
            thrust_vector = 0
            z_thrust = math.cos(thrust_vector)*thrust
            x_thrust = math.sin(thrust_vector)*thrust

        x = x + vx * t - (0.25 * ((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vx ** 2 * t ** 2) / mass + \
            0.5 * x_thrust / mass * t ** 2
        vx = vx - (0.5 * ((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vx ** 2 * t) / mass + x_thrust / mass * t

        z = z + vz*t - (0.25*((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vz**2 * t**2)/mass - 0.5*g*t**2 + \
            0.5*z_thrust/mass*t**2
        vz = vz - g*t + z_thrust/mass*t +(0.5*((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vz**2 * t)/mass
        Vt_z = math.sqrt((2 * mass * g)/(((Cd_sc * area_sc) + (Cd_para * area_para))*ro))
        if abs(vz) > Vt_z:
            vz = -Vt_z

    predict_land_x = x

    # print("Predicted Landing Point:", predict_land_x)
    if deployed_status == 0:
        if predict_land_x > landing_target_x:
            para_open = (predict_land_x - landing_target_x)*.4
            if para_open > 20000:
                para_open = 20000
            status_3 = "Parachute Open: " + str(para_open) + "m early"
            # print("Parachute Open:", para_open, "m early")
        else:
            para_open = (predict_land_x - landing_target_x)*.4
            if para_open < -12000:
                para_open = -12000
            status_3 = "Parachute Open: " + str(-para_open) + "m delayed"
            # print("Parachute Open:", -para_open, "m delay")
    else:
        status_3 = "Parachute Opened"
    predict_land_x = predict_land_x/1000

    return predict_land_x, status_3


def corrected_landing(z, vz, x, vx, para_open):

    # estimated entry parameters = downrange distance 1261 km
    landing_target_x = 1150*1000 + 500000
    mass = 2000  # kg
    g = 3.711  # m/s^2
    area_sc = 75  # m^2
    area_para = 0  # m^2
    Cd_sc = 0.8
    Cd_para = 1.75

    t = 1

    while z > 1:
        alt = z
        p = 0.699 * math.exp(-0.00009 * alt)

        if alt > 60000:
            T = -156
        elif alt > 7000:
            T = -23.4 - (0.00222 * alt)
        else:
            T = -31 - (0.000998 * alt)

        ro = p / (0.1921 * (T + 273.1))

        if ro > 0:
            pass
        else:
            ro = 0.000001

        if z < 3100:
            area_para = 0
        elif z < (15000 + para_open):
            area_para = 750

        if z < 900:
            z_thrust = (g + vz ** 2 * .8 / alt) * mass  # Force Newtons
            if vz > -1.1:
                z_thrust = g * mass
        else:
            z_thrust = 0

        if 600 < z < 3000:
            x_thrust = ((landing_target_x - x - (vx * (z / (-vz)))) * 2 * mass) / (z / vz)**2
            if x_thrust > 6000:
                x_thrust = 6000
            if x_thrust < -6000:
                x_thrust = -6000
            if vx > 200:
                x_thrust = 0
            if vx < -200:
                x_thrust = 0
        elif z < 600:
            x_thrust = mass * (-vx)/(z / -vz)
            if x_thrust > 6000:
                x_thrust = 6000
            if x_thrust < -6000:
                x_thrust = -6000
        else:
            x_thrust = 0

        x = x + vx * t - (0.25 * ((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vx ** 2 * t ** 2) / mass + \
            0.5 * x_thrust / mass * t ** 2
        vx = vx - (0.5 * ((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vx ** 2 * t) / mass + x_thrust / mass * t

        z = z + vz * t - (0.25 * (
                    (Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vz ** 2 * t ** 2) / mass - 0.5 * g * t ** 2 + \
            0.5 * z_thrust / mass * t ** 2
        vz = vz - g * t + z_thrust / mass * t + (
                    0.5 * ((Cd_sc * area_sc) + (Cd_para * area_para)) * ro * vz ** 2 * t) / mass
        Vt_z = math.sqrt((2 * mass * g) / (((Cd_sc * area_sc) + (Cd_para * area_para)) * ro))
        if abs(vz) > Vt_z:
            vz = -Vt_z

    corrected_land_x = x / 1000
    # print("Corrected Landing Point:", corrected_land_x)

    return corrected_land_x


def terrain_relative_nav(sim_x, status_1, status_2, corrected_land_x, predict_land_x, y, status_3):
    global video

    img1 = cv.imread('newmarszoom2.jpg')  # queryImage
    width = int(400)
    height = int(300)
    x = int(sim_x/1000)
    y = y
    landing_target_x = (1150*1000 + 500000)/1000

    ylow = int(y - (height / 2))
    yhigh = int(y + (height / 2))
    xlow = int(x - (width / 2))
    xhigh = int(x + (width / 2))

    #print(ylow, yhigh, xlow, xhigh)
    img2 = img1[ylow:yhigh, xlow:xhigh]

    orb = cv.ORB_create(nfeatures=50000)
    # find the keypoints and descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    # create BFMatcher object
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    # Match descriptors.
    matches = bf.match(des1, des2)
    # Sort them in the order of their distance.
    matches = sorted(matches, key=lambda x: x.distance)
    high_matches = matches[:10]
    # Draw first 10 matches.
    #img3 = cv.drawMatches(img1, kp1, img2, kp2, high_matches, None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    #cv.imshow('matches', img3)
    # Initialize lists
    list_kp1 = []
    list_kp2 = []

    # For each match...
    for mat in high_matches:
        # Get the matching keypoints for each of the images
        img1_idx = mat.queryIdx
        img2_idx = mat.trainIdx

        # x - columns
        # y - rows
        # Get the coordinates
        (x1, y1) = kp1[img1_idx].pt
        (x2, y2) = kp2[img2_idx].pt

        # Append to each list
        list_kp1.append((x1, y1))
        list_kp2.append((x2, y2))

    list_kp1 = np.array(list_kp1)
    list_kp1_avg = np.average(list_kp1, axis=0)
    x1 = list_kp1_avg[0]
    y1 = list_kp1_avg[1]

    list_kp2 = np.array(list_kp2)
    list_kp2_avg = np.average(list_kp2, axis=0)
    x2 = list_kp2_avg[0]
    y2 = list_kp2_avg[1]

    # cv.drawMarker(img1, (int(x1), int(y1)), [255, 0, 0])
    # cv.drawMarker(img2, (int(x2), int(y2)), [255, 0, 0])

    h, w = img2.shape[:2]
    H, W = img1.shape[:2]

    DX = int(x1) - int(x2)
    DY = int(y1) - int(y2)
    top_left = (DX, DY)
    bottom_right = (DX + w, DY + h)
    center_point = ((top_left[0] + bottom_right[0]) / 2, (top_left[1] + bottom_right[1]) / 2)
    x = center_point[0]*1000

    font = cv.FONT_HERSHEY_SIMPLEX
    cv.putText(img1, status_1, (100, 100), font, 1, [255, 255, 255], thickness=2)
    cv.putText(img1, status_2, (100, 150), font, 1, [255, 255, 255], thickness=2)
    cv.putText(img1, status_3, (100, 200), font, 1, [255, 255, 255], thickness=2)


    #cv.rectangle(img1, top_left, bottom_right, 255, 2)
    cv.drawMarker(img1, (int(landing_target_x), y), [255, 150, 0], markerType= 3,thickness=5)     # target
    cv.circle(img1, (int(landing_target_x), y), 50, [255, 150, 0], thickness=2)                   # target
    cv.drawMarker(img1, (int(predict_land_x), y), [255, 255, 255],markerType=1, thickness=5)    # predicted
    cv.drawMarker(img1, (int(corrected_land_x), y), [0, 125, 255], thickness=5)                 # corrected
    cv.drawMarker(img1, (int(center_point[0]), int(center_point[1])), [0, 255, 0], thickness=4) # s/c position

    #legend
    cv.drawMarker(img1, (100, 250), [255, 150, 0], markerType=3, thickness=4) # target
    cv.circle(img1, (100, 250), 20, [255, 150, 0], thickness=2)               # target
    cv.drawMarker(img1, (100, 300), [255, 255, 255], thickness=4)           # prediction
    cv.drawMarker(img1, (100, 350), [0, 125, 255], thickness=4)             # corrected
    cv.drawMarker(img1, (100, 400), [0, 255, 0], thickness=4)               # s/c position
    cv.putText(img1, "Landing Target", (150, 250), font, 1, [255, 255, 255], thickness=2)
    cv.putText(img1, "Computer Predicted Landing", (150, 300), font, 1, [255, 255, 255], thickness=2)
    cv.putText(img1, "Guidance Corrected Landing", (150, 350), font, 1, [255, 255, 255], thickness=2)
    cv.putText(img1, "Spacecraft Position", (150, 400), font, 1, [255, 255, 255], thickness=2)

    cv.imshow('map', img1)
    cv.imshow('img2', img2)

    video.write(img1)

    cv.waitKey(1)

    return x

edl_simulator()
video.release()
