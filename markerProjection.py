import PhotoScan, cv2, numpy


def cross(a, b):
    result = PhotoScan.Vector([a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y *b.x])
    return result


def findIndex(in_photo):
    global cam_num
    cams = chunk.cameras
    cam_index = 0
    for item in cams:
        if item.label == in_photo:
            cam_num = cam_index
        cam_index += 1


def run_script(points, img_name):
    global chunk
    photo_name = img_name.split('/')[-1]
    chunk = PhotoScan.app.document.chunk # set the chunk
    model = chunk.model
    vertices = chunk.model.vertices

    print(chunk)

    if chunk.transform.matrix:
        T0 = chunk.transform.matrix
    else:
        T0 = PhotoScan.Matrix().diag([1, 1, 1, 1])

    for point in points:
        x = point[0]
        y = point[1]
        marker_2D = (x, y) # pixel coordinates on the image for marker
        findIndex(photo_name)
        camera = chunk.cameras[cam_num] # sets the camera to create marker on
        marker = chunk.addMarker() # creates a marker on the chunk
        marker.projections[camera] = marker_2D # moves that marker to appropriate location on image

        point_2D = marker.projections[camera].coord
        vect = camera.sensor.calibration.unproject(point_2D)
        vect = camera.transform.mulv(vect)
        center = camera.center


        # estimating ray and surface intersection
        for face in model.faces:

            v = face.vertices

            E1 = PhotoScan.Vector(vertices[v[1]].coord - vertices[v[0]].coord)
            E2 = PhotoScan.Vector(vertices[v[2]].coord - vertices[v[0]].coord)
            D = PhotoScan.Vector(vect)
            T = PhotoScan.Vector(center - vertices[v[0]].coord)
            P = cross(D, E2)
            Q = cross(T, E1)
            result = PhotoScan.Vector([Q * E2, P * T, Q * D]) / (P * E1)

            if (0 < result[1]) and (0 < result[2]) and (result[1] + result[2] <= 1):
                t = (1 - result[1] - result[2]) * vertices[v[0]].coord
                u = result[1] * vertices[v[1]].coord
                v_ = result[2] * vertices[v[2]].coord

                point_3D = T0.mulp(u + v_ + t)
                point_3D = chunk.crs.project(point_3D)
                break

        point = chunk.crs.unproject(point_3D)
        point = T0.inv().mulp(point)

        for cur_camera in chunk.cameras:

            if (cur_camera == camera) or not cur_camera.transform:
                continue
            cur_proj = cur_camera.project(point)

            if (0 <= cur_proj[0] < camera.sensor.width) and (0 <= cur_proj[1] < camera.sensor.height):
                marker.projections[cur_camera] = cur_proj


def canny_edges():
    global image_name
    while True:
        lower_canny = PhotoScan.app.getInt(label='Lower Canny Threshold', value=50)
        upper_canny = PhotoScan.app.getInt(label='Upper Canny Threshold', value=150)
        imge = cv2.imread(image_name, 0)
        imgc = cv2.imread(image_name)
        height, width, channels = imgc.shape
        edges = cv2.Canny(imge, lower_canny, upper_canny)
        try:
            lines = cv2.HoughLinesP(edges, 1, numpy.pi / 180, 100, 100, 10)
            line_image = numpy.zeros((height, width, 1), numpy.uint8)  # create a blank image to draw lines on
            for x in range(0, len(lines)):
                for x1, y1, x2, y2 in lines[x]:
                    cv2.line(imgc, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.line(line_image, (x1, y1), (x2, y2), (255), 1)
            break
        except:
            print('Try less restrictive thresholds')
            break

    return (line_image, imgc)


def find_nearest_white(IMG, TARGET):
    nonzero = cv2.findNonZero(IMG)
    distances = numpy.sqrt((nonzero[:, :, 0] - TARGET[0]) ** 2 + (nonzero[:, :, 1] - TARGET[1]) ** 2)
    nearest_index = numpy.argmin(distances)
    return nonzero[nearest_index]


def click(event, x, y, flags, param):
    global target
    global display_imgs
    global white_point
    global link_list
    if event == cv2.EVENT_LBUTTONDOWN:
        target = (x, y)
        white_point = find_nearest_white(display_imgs[0], target)
        find_connected(display_imgs[0], white_point)
        for item in link_list:
            cv2.circle(display_imgs[1], (item[0], item[1]), 3, (0, 0, 255), 1)
        cv2.imshow('Image', display_imgs[1])


def find_connected(img, in_coords):
    global link_list
    in_x = int(in_coords[0][0])
    in_y = int(in_coords[0][1])
    x = 0
    while x < 500:
        iter_list = [(in_x + 1, in_y), (in_x + 1, in_y - 1), (in_x, in_y - 1), (in_x - 1, in_y - 1), (in_x - 1, in_y),
                     (in_x - 1, in_y + 1), (in_x, in_y + 1), (in_x + 1, in_y + 1)]
        check_count = len(link_list)
        for item in iter_list:
            px = int(img[item[1], item[0]])
            if px > 0:
                in_x = item[0]
                in_y = item[1]
                link_list.append((in_x, in_y))
            else:
                pass
        x += 1

    return link_list


def image_processing():
    global display_imgs
    global link_list
    display_imgs = canny_edges()
    cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Image', click)

    while True:
        cv2.imshow('Image', display_imgs[1])
        cv2.resizeWindow('Image', 1600, 1200)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    link_list = list(set(link_list)) # removes duplicates from list if they arise
    print(link_list)
    run_script(link_list, image_name)

link_list = []
target = ()
white_point = []
cam_num = 0
image_name = PhotoScan.app.getOpenFileName()


image_processing()
#run_script(link_list, image_name)