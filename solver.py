import util
import open3d as o3d
import numpy as np
import os
from shapely.geometry import Polygon
import multiprocessing as mp
import csv
import cv2 as cv


class Solver:

    def __init__(self) -> None:
        self.__cpus:int = mp.cpu_count()

    def load_pointcloud(self, file:str = os.path.join(util.Configs.workspace, "ans/ans.ply")):
        self.__pointcloud:o3d.geometry.PointCloud = o3d.io.read_point_cloud(file)
        self.__pointcloud.estimate_normals(
                            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30), 
                            fast_normal_computation=False)
        # self.__pointcloud.estimate_normals()
        distances:np.ndarray = self.__pointcloud.compute_nearest_neighbor_distance()
        avg_dist:float = np.mean(distances)
        radius:float = 1.5 * avg_dist   

        # self.__trianglemesh:o3d.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        #                                 self.__pointcloud, o3d.utility.DoubleVector([radius, radius * 2]))
        self.__trianglemesh:o3d.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                                        self.__pointcloud, o3d.utility.DoubleVector([0.005, 0.01, 0.02, 0.04]))
        self.__trianglemesh.compute_triangle_normals()
        self.__xyz:np.ndarray = np.asarray(self.__pointcloud.points) * 1000 # 点云的点转为Numpy数组,并把单位从米转为毫米
        self.__rgb:np.ndarray = np.asarray(self.__pointcloud.colors) # 点云的颜色转为Numpy数组

    def get_mesh_volume(self) -> float:
        hull, _ = self.__pointcloud.compute_convex_hull()
        hull.compute_triangle_normals()
        return hull.get_volume() * 1000000

    @property
    def pointcloud(self) -> o3d.geometry.PointCloud:
        return self.__pointcloud

    @property
    def high(self) -> float: # 计算株高,单位是毫米
        y:np.ndarray = self.__xyz[:, 1]  # 获取点云的Y轴（高度）数组
        return (y[np.argmax(y)] - y[np.argmin(y)])  # 获取点云的Y轴（高度）数组的极值差作为目标的高度

    @property
    def color(self) -> float: # 统计绿色占比
        sumcolor:int = self.__rgb.shape[0]  + 1
        green:np.ndarray = self.__rgb[( (self.__rgb[:, 1]-self.__rgb[:, 0] > 0.1176470588235294) &
                        (self.__rgb[:, 1]-self.__rgb[:, 2] > 0.1176470588235294) )]
        sumgreen:int = green.shape[0]
        return sumgreen/sumcolor * 100

    @property
    def area(self) -> float: # 计算表面积,单位是平方厘米
        return self.__trianglemesh.get_surface_area() * 10000

    @staticmethod
    def boundary_points_sort(points: np.ndarray) -> np.ndarray:
        center_x, center_y = (np.nanmax(points, axis=0) +
                            np.nanmin(points, axis=0))/2
        # 在第三列添加向量角余弦值,第四列添加向量的长度
        points_with_flag = np.hstack((points, np.hstack((((points[:, 0]-center_x)/np.sqrt(np.square(points[:, 0]-center_x)+np.square(
            points[:, 1]-center_y))).reshape(points.shape[0], 1), np.sqrt(np.square(points[:, 0]-center_x)+np.square(points[:, 1]-center_y)).reshape(points.shape[0],1)))))
        points_part_a, points_part_b = points_with_flag[points_with_flag[:,1]>=center_y], points_with_flag[points_with_flag[:,1]<center_y]  
        points_part_a = points_part_a[ np.lexsort(( points_part_a[:,3], points_part_a[:,2] ))[::-1] ] # 中心点以上的点
        points_part_b = points_part_b[ np.lexsort((points_part_b[:,3],))[::-1]] # 中心点以下的点
        points_part_b = points_part_b[ np.lexsort(( points_part_b[:,2],)) ]
        return np.vstack((points_part_a[:,:2], points_part_b[:,:2]))

    @staticmethod
    def count_area(points:np.ndarray) -> float: # 计算多边形面积,单位是平方厘米
        return Polygon(Solver.boundary_points_sort(points)).area / 100

    @staticmethod
    def get_volume(point_list:list, ans_list:list) -> float: # 计算体积,单位是立方厘米(毫升)
        for points in point_list:
            point = Solver.boundary_points_sort(points)
            ans_list.append(Solver.count_area(point) * 0.1 ) # 0.1 cm

    @property
    def volume(self) -> float: # 计算体积
        xz:np.ndarray = np.hstack((self.__xyz[:,0].reshape(self.__xyz.shape[0], 1), self.__xyz[:,2].reshape(self.__xyz.shape[0], 1)))
        y:np.ndarray = self.__xyz[:,1]
        point_list:list = list()
        for i in np.sort(np.unique(y)):
            point_list.append(xz[np.where(y == i)])
        step:int = round(len(point_list)/self.__cpus)
        with mp.Manager() as manager:
            ans_list = manager.list()
            pool = mp.Pool(self.__cpus)
            for i in range(self.__cpus-1):
                pool.apply_async(func=Solver.get_volume, args=(point_list[i*step:(i+1)*step], ans_list))
            pool.apply_async(func=Solver.get_volume, args=(point_list[(self.__cpus-2)*step:], ans_list))
            pool.close()
            pool.join()
        
            return sum(ans_list)

    @staticmethod
    def spilt_0(input_file:str, output_file:str):
        img = cv.imread(input_file) 
        bgra = cv.cvtColor(img, cv.COLOR_BGR2BGRA)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        gray = cv.bitwise_not(gray)
        mask=np.zeros(gray.shape, np.uint8) # 原图大小的纯黑mask图像
        ret, thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV+cv.THRESH_OTSU)

        # 噪声去除 
        kernel = np.ones((3,3), np.uint8) 
        opening = cv.morphologyEx(thresh,cv.MORPH_OPEN,kernel, iterations = 2) 
        # 确定背景区域
        sure_bg = cv.dilate(opening,kernel,iterations=3) 
        # 寻找前景区域 
        dist_transform = cv.distanceTransform(opening, cv.DIST_L2,5) 
        ret, sure_fg = cv.threshold(dist_transform, dist_transform.mean(), 255, cv.THRESH_BINARY) 
        # 找到未知区域 
        sure_fg = np.uint8(sure_fg)
        unknown = cv.subtract(sure_bg,sure_fg)

        # 类别标记 
        ret, markers = cv.connectedComponents(sure_fg) 
        # 为所有的标记加1，保证背景是0而不是1 
        markers = markers+1 
        # 现在让所有的未知区域为0 
        markers[unknown==255] = 0

        # print(ret)
        markers = cv.watershed(img,markers)
        x, y = markers.shape
        markers[0] = 0
        markers[x-1] = 0
        markers[1:x, 0] = 0
        markers[1:x, y-1] = 0 

        mask[markers == 1] = 1
        mask[markers == -1] = 1

        masked = cv.bitwise_and(bgra,bgra,mask=mask)
        cv.imwrite(output_file, masked)

    @staticmethod
    def split_1(input_path:str = os.path.join(util.Configs.workspace, "IRs")):
        for dir in os.listdir(input_path):
            for file in os.listdir(os.path.join(input_path, dir)):
                Solver.spilt_0(os.path.join(input_path, dir+'/'+file), os.path.join(input_path, dir+'/'+file.replace(".", "_o.")))
        
    def solve(self, output_file:str = os.path.join(util.Configs.workspace, "ans/ans.csv")) -> dict:

        ans = {"high":f"{self.high}mm",
                "area":f"{self.area}cm^2",
                "color":f"{self.color}%",
                "volume":f"{self.volume}mL"}

        with open(output_file, mode='w', encoding="utf-8", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(("Event","Value"))
            for item in ans.items():
                writer.writerow(item)

        Solver.split_1()
        return ans      