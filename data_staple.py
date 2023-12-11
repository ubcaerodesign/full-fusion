path = "C:\\Users\\Ethan Gray\\Downloads\\gnss-ins-sim-2.2.0\\gnss-ins-sim-2.2.0\\demo_saved_data\\"
run = "2023-12-02-16-46-34"
files = ["\\gps-0.csv", "\\accel-0.csv", "\\gyro-0.csv", "\\mag-0.csv", "\\time.csv", "\\ref_att_quat.csv", "\\ref_pos.csv"]
all_data = []
for file in files:
    with open(path + run + file, 'r') as f: 
        all_data.append(f.readlines())
lines = []
with open("test_data\\imu_gps.csv", 'w') as f:
    line = ""
    line_num = 0
    while True:
        try:
            for data in all_data:
                line += data[line_num].strip() + ","
            lines.append(line[:-1] + "\n")
            line = ""
            line_num += 1
        except:
            break
    f.writelines(lines)


