import cv2
import numpy as np

# Hệ số khoảng cách
distance_gain_x = 0
distance_gain_y = 0

def find_center(mask: np.ndarray):
    """
    Tìm tâm của mặt nạ.
    Args:
        mask (numpy.ndarray): mặt nạ
    Returns:
        tuple: tọa độ tâm (x, y)
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        M = cv2.moments(contours[0])
        if M['m00'] != 0:
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])
            return cX, cY
    return None

def display_object(image: np.ndarray, object: dict, color: tuple = (188, 223, 235), color_center: tuple = (255, 0, 0)):
    """
    Hiển thị tên và tâm của đối tượng trên ảnh.
    Args:
        image (numpy.ndarray): ảnh gốc
        object (dict): đối tượng
        color (tuple): màu chữ
        color_center (tuple): màu tâm
    Returns:
        None
    """
    if object.get('center') is None or object.get('name') is None:
        return print('[display_object] object not found')
    x_center, y_center = object['center']
    # hiện tên
    cv2.putText(image, object['name'], (x_center - 50, y_center - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    # vẽ tâm
    cv2.circle(image, (x_center, y_center), 5, color_center, -1)

def display_objects(image: np.ndarray, objects: list[dict], color: tuple = (188, 223, 235), color_center: tuple = (255, 0, 0)):
    """
    Hiển thị tên và tâm của các đối tượng trên ảnh.
    Args:
        image (numpy.ndarray): ảnh gốc
        objects (list[dict]): danh sách đối tượng
        color (tuple): màu chữ
        color_center (tuple): màu tâm
    Returns:
        None
    """
    for object in objects:
        display_object(image, object, color)

def draw_frame(image: np.ndarray, objects: list[object], show=False):
    """
    Vẽ đường thẳng nối Goc và Den_doc và Den_ngang và tính hệ số khoảng cách.
    Args:
        image (numpy.ndarray): ảnh gốc
        objects (list[dict]): danh sách đối tượng
        show (bool): có hiển thị không
    Returns:
        None
    """
    index = { 'Goc': -1, 'Den_doc': -1, 'Den_ngang': -1 }
    for i, object in enumerate(objects):
        if index.get(object['name']) is not None:
            index[object['name']] = i
    for idx in index.items():
        if idx[1] == -1:
            return print('[draw_frame] 3 objects not found')
        
    # vẽ đường thẳng nối Goc và Den_doc và Den_ngang
    if show:
        cv2.line(image, objects[index['Goc']]['center'], objects[index['Den_doc']]['center'], (0, 255, 0), 2)
        cv2.line(image, objects[index['Goc']]['center'], objects[index['Den_ngang']]['center'], (0, 0, 255), 2)

    # tính khoảng cách
    dy = calc_distance(objects[index['Goc']]['center'], objects[index['Den_doc']]['center'])
    dx = calc_distance(objects[index['Goc']]['center'], objects[index['Den_ngang']]['center'])

    global distance_gain_x, distance_gain_y
    distance_gain_x = 950 / dx
    distance_gain_y = 950 / dy

    d1 = calc_distance(objects[index['Goc']]['center'], objects[index['Den_doc']]['center'], output='mm')
    d2 = calc_distance(objects[index['Goc']]['center'], objects[index['Den_ngang']]['center'], output='mm')

    # hiển thị khoảng cách    
    if show:
        cv2.putText(image, f'D: {d1:.1f}mm', (int((objects[index['Goc']]['center'][0] + objects[index['Den_doc']]['center'][0]) / 2),
                    int((objects[index['Goc']]['center'][1] + objects[index['Den_doc']]['center'][1]) / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.putText(image, f'D: {d2:.1f}mm', (int((objects[index['Goc']]['center'][0] + objects[index['Den_ngang']]['center'][0]) / 2),
                    int((objects[index['Goc']]['center'][1] + objects[index['Den_ngang']]['center'][1]) / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)


def get_object(objects: list[dict], name: str):
    """
    Tìm đối tượng trong danh sách đối tượng theo tên.
    Args:
        objects (list[dict]): danh sách đối tượng
        name (str): tên đối tượng cần tìm
    Returns:
        dict: đối tượng tìm thấy
    """
    for object in objects:
        if object.get('name') == name:
            return object
    return None


def get_coordinate(goc: dict, object: dict, image = None):
    """
    lấy tọa độ của object so với Goc
    Args:
        goc (dict): đối tượng Goc
        object (dict): đối tượng cần lấy tọa độ
        image (numpy.ndarray): ảnh gốc
    Returns:
        tuple: tọa độ của object so với Goc
    """
    if goc.get('center') is None or object.get('center') is None:
        return print('[get_coordinate] goc or object not found')
    gocx, gocy = goc['center']
    objx, objy = object['center']
    dx = (objx - gocx) * distance_gain_x
    dy = (objy - gocy) * distance_gain_y
    if image is not None:
        # hiện tọa độ
        cv2.putText(image, f'x={int(dx)}, y={int(dy)}', (int(objx - 50), int(objy + 50)),
                                                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    return dx, dy


def mask_offset(mask: np.ndarray, offset: tuple):
    """
    Dịch chuyển mặt nạ theo offset.
    Args:
        mask (numpy.ndarray): mặt nạ
        offset (tuple): offset (x, y)
    Returns:
        numpy.ndarray: mặt nạ đã dịch chuyển
    """
    offset_x, offset_y = offset
    mask = np.roll(mask, offset_x, axis=1)
    mask = np.roll(mask, offset_y, axis=0)
    return mask


def draw_contours(image: np.ndarray, mask: np.ndarray, color=(0, 255, 0), fill=False, opacity=0.5):
    """
    Vẽ viền của mặt nạ lên ảnh.
    Args:
        image (numpy.ndarray): ảnh gốc
        mask (numpy.ndarray): mặt nạ
        color (tuple): màu viền
        fill (bool): có tô màu bên trong không
        opacity (float): độ trong suốt của màu tô
    Returns:
        None
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, 0, color, 2)
    if fill:
        overlay = image.copy()
        cv2.fillPoly(overlay, contours, color)
        cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)


def calc_distance(pt1: tuple, pt2: tuple, output='px'):
    """
    Tính khoảng cách giữa 2 điểm.
    Args:
        pt1 (tuple): tọa độ điểm 1
        pt2 (tuple): tọa độ điểm 2
        output (str): đơn vị đầu ra, 'px' hoặc 'mm'
    Returns:
        float: khoảng cách giữa 2 điểm
    """
    x1, y1 = pt1
    x2, y2 = pt2
    dx = x2 - x1
    dy = y2 - y1
    if output == 'mm':
        dx *= distance_gain_x
        dy *= distance_gain_y
    return np.sqrt(dx ** 2 + dy ** 2)
