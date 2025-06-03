# msg/VisionResult.msg

# Standardowy nagłówek ROS 2 (timestamp + frame_id)
std_msgs/Header header

# Numer klatki i fps
int32 frame_number
float32 fps

# Status procesu (np. "Tryb: Wyszukiwanie twarzy", "MarkerFound" itp.)
string status

# Imię / ID rozpoznanej osoby
string person_name

# Miara pewności rozpoznania twarzy
float32 confidence

# Bounding box twarzy – zapiszemy jako pojedynczy Point, gdzie
#   x = lewy górny róg (w pikselach),
#   y = górny górny róg (w pikselach),
#   z = szerokość prostokąta (w pikselach),
#   w = wysokość prostokąta (jeśli wolisz, ale jeżeli Point ma tylko x,y,z,
#         to możesz użyć z=width, w=height w innym komunikacie).
geometry_msgs/Point face_rect

# Współrzędne środka markera ArUco w przestrzeni obrazu (piksele)
geometry_msgs/Point marker_center
