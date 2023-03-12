import json
import os
import shutil
import open3d as o3d
import time

# 所有涉及json文件读写的操作,只能写在util中
# 所有涉及json文件读写的操作,只能写在util中
# 所有涉及json文件读写的操作,只能写在util中

# 如果是直接使用config中的值,就写成Configs类的静态变量
class Configs:

    with open(file="./ANS/config.json", mode='r') as j:
        config:dict = json.load(j)

    gui:bool = config["gui"]
    workspace:str = config["workspace"]
    sensor_config:dict = config["sensor_config"]
    byhand:bool = config["byhand"]
    distance:float = config["distance"]
    write_color_and_depth:bool = config["write_color_and_depth"]
    fps:int = config["fps"]
    counts:int = config["counts"]

    class Filter:

        class preprocess:
            with open(file="./ANS/config.json", mode='r') as j:
                config:dict = json.load(j)["Filter"]["preprocess"]

            radius:float = config["KDTreeSearchParamHybrid"]["radius"]
            max_nn:float = config["KDTreeSearchParamHybrid"]["max_nn"]
            nb_neighbors:int = config["remove_statistical_outlier"]["nb_neighbors"]
            std_ratio:float = config["remove_statistical_outlier"]["std_ratio"]
            distance_threshold:float = config["segment_plane"]["distance_threshold"]
            ransac_n:int = config["segment_plane"]["ransac_n"]
            num_iterations:int = config["segment_plane"]["num_iterations"]
        
        class get_object:
            with open(file="./ANS/config.json", mode='r') as j:
                config:dict = json.load(j)["Filter"]["get_object"]

            nb_neighbors:int = config["remove_statistical_outlier"]["nb_neighbors"]
            std_ratio:float = config["remove_statistical_outlier"]["std_ratio"]
            nb_points:int = config["remove_radius_outlier"]["nb_points"]
            radius:float = config["remove_radius_outlier"]["radius"]
            esp:float = config["cluster_dbscan"]["esp"]
            min_points:int = config["cluster_dbscan"]["min_points"]

# ---------------------------手-动-分-割-线---------------------------
# source_ply: 原始点云
# temp_ply: 经过滤波、去除背景、粗配准的预处理点云
# object_ply: 聚类获取目标主体的点云
# ans: 配准后的点云, 测量数据

class Workspace:

    @staticmethod
    def init_workspace(workspace:str = Configs.workspace):
        if not os.path.isdir(workspace): os.mkdir(workspace)
        for path in ("source_ply", "temp_ply", "object_ply", "video", "ans", "colors", "depths", "IRs", "depot"):
            if not os.path.isdir( os.path.join(workspace, path) ):
                os.mkdir(os.path.join(workspace, path))

    @staticmethod
    def clean_workspace(workspace:str = Configs.workspace, pseudo:bool = True):
        if not os.path.isdir(workspace): return
        for path in ("source_ply", "temp_ply", "object_ply", "video", "ans", "colors", "depths", "IRs"):
            if not os.path.isdir( os.path.join(workspace, path) ): continue
            __class__.clean_dir(os.path.join(workspace, path), pseudo)
            
    @staticmethod
    def clean_dir(path:str, pseudo:bool=True, workspace:str = Configs.workspace):
        if not os.path.isdir(path): return
        if pseudo:
            dir = time.strftime('%H%M%S%y%m%d', time.localtime())
            if not os.path.isdir(os.path.join(workspace, "depot/{}".format(dir))): 
                os.mkdir(os.path.join(workspace, "depot/{}".format(dir)))
            shutil.move(path, os.path.join(workspace, "depot/{}".format(dir)))
        else:
            shutil.rmtree(path)
        os.mkdir(path)


class Pointcloud:

    @staticmethod
    def load_sensor_config(sensor_config:dict = Configs.sensor_config) -> o3d.io.AzureKinectSensorConfig:
        return o3d.io.AzureKinectSensorConfig(sensor_config)

    @staticmethod
    def read_pointcloud(file:str) -> o3d.geometry.PointCloud: 
        return o3d.io.read_point_cloud(file)


class Json:

    @staticmethod
    def append_write_json(file:str, data:dict):
        with open(file, mode='r', encoding="utf-8") as f:
            length = len(f.read())
        with open(file, mode='r',encoding="utf-8") as f:
            if length > 0:
                old_data = json.load(f)
            else:
                old_data = {}
        old_data.update(data)
        with open(file, mode='w', encoding="utf-8") as f:
            json.dump(old_data, f)

    @staticmethod
    def write_json(file:str, data:dict):
        if os.path.isfile(file): raise ValueError("文件{}已存在".format(file))
        with open(file, mode='w', encoding="utf-8") as f:
            json.dump(data, f)

    @staticmethod
    def load_json_dict(file:str) -> dict:
        with open(file, mode='r', encoding="utf-8") as f:
            d = json.load(f)
        if isinstance(d, dict):
            return d
        else:
            raise TypeError("只能读取存有dict的json文件")

    @staticmethod
    def load_json_list(file:str) -> list:
        with open(file, mode='r', encoding="utf-8") as f:
            l = json.load(f)
        if isinstance(l, list):
            return l
        else:
            raise TypeError("只能读取存有list的json文件")


class Pos:

    @staticmethod
    def get_pos() -> list:
        while True:
            try:
                pos_str = input("输入位置参数(a,b,dx,dy)：").split(",")
                flag = input("确认参数：输入Y继续，输入N重新输入。")
                if flag == ('y' or 'Y'):
                    break
            except:
                pass
        pos = []
        for i in pos_str:
            pos.append(eval(i)*1.0)
        return pos

    @staticmethod
    def write_pos(pos:list, count:int):
        if not os.path.exists(os.path.join(Configs.workspace, "pointcloud_pos.json")):
            Json.write_json(os.path.join(Configs.workspace, "pointcloud_pos.json"), {count:pos})
        else:
            Json.append_write_json(os.path.join(Configs.workspace, "pointcloud_pos.json"), {count:pos})