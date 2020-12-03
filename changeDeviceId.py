import json
import sys

if len(sys.argv) >1:
    with open('initial.ini', 'r') as f:
        write_json = f.read()
    writedata = json.loads(write_json)  # 将json转换为字典
    try:
        for a in writedata:
            if type(writedata[a]) is dict:
                if 'device_name' in writedata[a].keys():
                    writedata[a]['device_name']=sys.argv[1]
        print("修改device_id成功")
    except Exception as e:
        print("修改device_id失败，请重试")
    write_json = json.dumps(writedata)
    with open('initial.ini', 'w') as f:
        f.write(write_json)
else:
    print("未输入参数,请输入device_id,格式为python3 changeDeviceId device_id")