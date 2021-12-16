import json


# map_dict: 'map_id’: [[location.x, location.y, location.z], [yaw, pitch, roll]]
params_dict = {
    'Town04' : [[201.5, -296.2, 0.5], [1.58, 0, 0]],
    'Town03' : [[201.5, -296.2, 0.5], [1.58, 0, 0]]
    # 'xcm_url': 'udpm://239.255.76.67:7667?ttl=1' # 车上的工控机
}
with open("./sim_params_test.json","w") as f:
    f.write("{\n")
    for i,item in enumerate(params_dict.items()):
        if i == len(params_dict) - 1:
            f.write("\""+item[0]+"\""+': '+str(item[1])+'\n')
            break
        f.write("\""+item[0]+"\""+': '+str(item[1])+',\n')
    f.write("\n}")

with open("./sim_params_test.json",'r') as load_f:
    spawn_origin = json.load(load_f)
    print(spawn_origin)

map = 'Town04'
print(spawn_origin[map][0][1])