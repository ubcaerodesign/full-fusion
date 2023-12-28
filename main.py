import attitude_visualization
import serial_read
import filter
from math import pi

from data_plotter import plot_in_one_figure
import matplotlib.pyplot as plt
import folium
import webbrowser

# use the same data stamps 
# assuming that each data line is sent with the same items (we "reuse" gps data) 

reader = serial_read.Read("test_data\\imu_gps.csv")
indices = {}
rad_or_deg = {"filter": "rad", "gps_update": "deg", "att_update": "rad", "map": "deg", "graph": "deg"} # do not change filter or map units  

def convert_angle(input_type, output_type):
    if rad_or_deg[input_type] == "rad" and rad_or_deg[output_type] == "rad":
        return 1
    elif rad_or_deg[input_type] == "rad" and rad_or_deg[output_type] == "deg":
        return 180 / pi
    elif rad_or_deg[input_type] == "deg" and rad_or_deg[output_type] == "rad":
        return pi / 180
    else: 
        return 1

headers = reader.getData()
i = 0
for head in headers:
    name = head.split()[0]
    indices[name] = i
    try:
        unit = head.split()[1]
        if "rad" in unit:
            rad_or_deg[name] = "rad"
        elif "deg" in unit:
            rad_or_deg[name] = "deg"
    except:
        pass
    i += 1 

firstLine = list(map(float, reader.getData())) # list of data (string parsed and split in reader)

kf = filter.KalmanFilter([convert_angle("gps_lat", "filter") * firstLine[indices["gps_lat"]], \
                          convert_angle("gps_lon", "filter") * firstLine[indices["gps_lon"]], firstLine[indices["gps_alt"]]], \
                         [firstLine[indices["gps_vN"]], firstLine[indices["gps_vE"]], firstLine[indices["gps_vD"]]], \
                         [firstLine[indices["q1"]], firstLine[indices["q2"]], firstLine[indices["q3"]], firstLine[indices["q0"]]]) # check quaternion order

drawer = attitude_visualization.Display()

algo_data = {"p": [], "v": [], "q": []}
time = []
algo_points = []
ref_points = []

i = 1
while True: 
    print(firstLine[indices["time"]])
    time.append(firstLine[indices["time"]])
    q = kf.getq()
    p = kf.getp()
    drawer.draw(q[0], q[1], q[2], q[3], p[2])
    algo_points.append((convert_angle("filter", "map") * p[0], convert_angle("filter", "map") * p[1]))
    ref_points.append((convert_angle("ref_pos_lat", "map") * firstLine[indices["ref_pos_lat"]], convert_angle("ref_pos_lon", "map") * firstLine[indices["ref_pos_lon"]]))
    v = kf.getv()
    algo_data["p"].append([convert_angle("filter", "graph") * p[0], convert_angle("filter", "graph") * p[1], p[2]])
    algo_data["v"].append(v)
    algo_data["q"].append(q)
    try:
        nextLine = list(map(float, reader.getData()))
    except:
        break
    T_s = nextLine[indices["time"]] - firstLine[indices["time"]]
    kf.predict([firstLine[indices["accel_x"]], firstLine[indices["accel_y"]], firstLine[indices["accel_z"]]], \
               [convert_angle("gyro_x", "filter") * firstLine[indices["gyro_x"]], convert_angle("gyro_y", "filter") * firstLine[indices["gyro_y"]], \
                convert_angle("gyro_z", "filter") * firstLine[indices["gyro_z"]]], T_s)

    if i % 10 == 0: # change based on ratio of gps sample rate to imu sample rate
        kf.update([convert_angle("gps_lat", "gps_update") * nextLine[indices["gps_lat"]], \
                   convert_angle("gps_lon", "gps_update") * nextLine[indices["gps_lon"]], nextLine[indices["gps_alt"]], \
                   nextLine[indices["gps_vN"]], nextLine[indices["gps_vE"]], nextLine[indices["gps_vD"]]], \
                  [convert_angle("ref_Yaw", "att_update") * nextLine[indices["ref_Yaw"]], convert_angle("ref_Pitch", "att_update") * nextLine[indices["ref_Pitch"]], \
                   convert_angle("ref_Roll", "att_update") * nextLine[indices["ref_Roll"]]]) 

    firstLine = nextLine 
    i += 1


plot_in_one_figure(time, algo_data["p"], title="Position")
plot_in_one_figure(time, algo_data["v"], title="Velocity (NED)", legend=("vN", "vE", "vD"))
plot_in_one_figure(time, algo_data["q"], title="Attitude")

m = folium.Map(location=algo_points[0], zoom_start=18)
tile = folium.TileLayer(
        tiles = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr = 'Esri',
        name = 'Esri Satellite',
        overlay = False,
        control = True
       ).add_to(m)
folium.PolyLine(algo_points, tooltip="Coast").add_to(m)
folium.PolyLine(ref_points, tooltip="Coast", color="red").add_to(m)
m.save("map\\index.html")

webbrowser.open("map\\index.html")

plt.show()