import util
import os
import open3d as o3d
import numpy as np
import multiprocessing as mp
import math as m


class Filter():
    def __init__(self, sor_path=os.path.join(util.Configs.workspace, "source_ply"),
                 temp_path=os.path.join(util.Configs.workspace, "temp_ply"),
                 obj_path=os.path.join(util.Configs.workspace, "object_ply"),
                 ans_path=os.path.join(util.Configs.workspace, "ans")) -> None:
        self.sor_path: str = sor_path
        self.temp_path: str = temp_path
        self.obj_path: str = obj_path
        self.ans_path: str = ans_path
        self.cpus: int = mp.cpu_count()

    @staticmethod
    def preprocess_0(file: str, pos: list):  # 滤波并粗配准

        ply:o3d.geometry.PointCloud = o3d.io.read_point_cloud(file)  # 读取点云文件

        # print("Downsample the point cloud with a voxel of 0.003")
        # downply = ply.voxel_down_sample(voxel_size=0.003)  # 体素化，size = 3 mm
        # print(downply)
        downply:o3d.geometry.PointCloud = ply
        print("Recompute the normal of the downsampled point cloud")
        downply.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=util.Configs.Filter.preprocess.radius,
                max_nn=util.Configs.Filter.preprocess.max_nn))

        print("Statistical oulier removal")
        cl, _ = downply.remove_statistical_outlier(
            nb_neighbors=util.Configs.Filter.preprocess.nb_neighbors,
            std_ratio=util.Configs.Filter.preprocess.std_ratio)  # 去除离群点
        downply_inlier_cloud:o3d.geometry.PointCloud = cl
        print(downply_inlier_cloud)

        plane_model, inliers = downply_inlier_cloud.segment_plane(
            distance_threshold=util.Configs.Filter.preprocess.distance_threshold,
            ransac_n=util.Configs.Filter.preprocess.ransac_n,
            num_iterations=util.Configs.Filter.preprocess.num_iterations)
        a, b, c, d = plane_model  # 平面分割
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        outlier_cloud:o3d.geometry.PointCloud = downply_inlier_cloud.select_by_index(
            inliers, invert=True)

        # ------------------------------------------------------
        a, b, dx, dz = pos  # pos=(俯仰角度、旋转角度、横坐标、纵坐标)
        a, b = m.radians(a), m.radians(b)  # 坐标变换矩阵
        mat_x = np.asarray([[1., 0., 0., 0.], [0., m.cos(a), m.sin(a), 0.], [
            0., -m.sin(a), m.cos(a), 0.], [0., 0., 0., 1.]])
        mat_y = np.asarray([[m.cos(b), 0., -m.sin(b), 0.], [0., 1., 0., 0.],
                            [m.sin(b), 0., m.cos(b), 0.], [0., 0., 0., 1.]])
        mat_m = np.asarray([[1., 0., 0., 0.], [0., 1., 0., 0.], [
            0., 0., 1., 0.], [-dx, 0., -dz, 1.]])
        trans_mat = np.matmul(np.matmul(mat_x, mat_y), mat_m)
        # ------------------------------------------------------
        outlier_cloud.transform(trans_mat)

        return outlier_cloud

    def preprocess_1(self, file_list: list, pos: list, count:int):  # 批量preprocess
        for name in file_list:
            print('------------------------------------')
            ply = __class__.preprocess_0(os.path.join(self.sor_path, name), pos)
            print("预处理点云...")
            o3d.io.write_point_cloud(
                self.temp_path+f"/{count}/temp_"+name, ply)
            print('------------------------------------')

    def preprocess_2(self, pos: list):  # 多进程并行批量preprocess
        pool = mp.Pool(self.cpus)
        files:list = os.listdir(self.sor_path)
        file_nums:int = len(files)
        step:int = int(file_nums/self.cpus)+1
        count:int = len(os.listdir(self.temp_path))
        os.mkdir(os.path.join(self.temp_path, str(count)))
        if file_nums <= self.cpus:
            for file in files:
                pool.apply_async(self.preprocess_1, args=([file], pos, count))
        else:
            for i in range(self.cpus):
                if (i+1)*step >= file_nums:
                    files_list = files[i*step:]
                    flag = True
                else:
                    files_list = files[i*step:(i+1)*step]
                    flag = False
                pool.apply_async(self.preprocess_1, args=(files_list, pos, count))
                if flag:
                    break
        print("开始处理点云...")
        pool.close()
        pool.join()
        print("点云处理完毕...")
        util.Workspace.clean_dir(self.sor_path)

    @staticmethod
    def get_object_0(ply: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        cl, ind = ply.remove_statistical_outlier(
            nb_neighbors=util.Configs.Filter.get_object.nb_neighbors,
            std_ratio=util.Configs.Filter.get_object.std_ratio,
            print_progress=True)  # 去除离群点
        ply:o3d.geometry.PointCloud = cl
        cl, ind = ply.remove_radius_outlier(
            nb_points=util.Configs.Filter.get_object.nb_points,
            radius=util.Configs.Filter.get_object.radius,
            print_progress=True)  # 半径滤波
        ply:o3d.geometry.PointCloud = cl
        print(ply)
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(ply.cluster_dbscan(
                util.Configs.Filter.get_object.esp,
                util.Configs.Filter.get_object.min_points,
                True))

        max_label:int = labels.max()
        count, points_num = 0, 0
        for i in range(max_label + 1):  # 分析聚类结果
            ind = np.where(labels == i)[0]
            colors = np.asarray(ply.select_by_index(ind).colors)
            sumcolor = colors.shape[0] + 1
            green = colors[((colors[:, 1]-colors[:, 0] > 0.1176470588235294) &
                            (colors[:, 1]-colors[:, 2] > 0.1176470588235294))]
            sumgreen = green.shape[0]

            if sumgreen/sumcolor >= count and np.asarray(ply.select_by_index(ind).points).shape[0] > points_num:
                count = sumgreen/sumcolor
                points_num = np.asarray(
                    ply.select_by_index(ind).points).shape[0]
                object_ply = ply.select_by_index(ind)

        return object_ply

    def get_object_1(self, count: int):  # 批量get_object
        files = os.listdir(os.path.join(self.temp_path, str(count)))
        ply:o3d.geometry.PointCloud = o3d.io.read_point_cloud(os.path.join(self.temp_path+f"/{count}", files[0]))
        object_ply:o3d.geometry.PointCloud = Filter.get_object_0(ply)
        for file in files[1:]:
            ply = o3d.io.read_point_cloud(
                os.path.join(self.temp_path+f"/{count}", file))
            ply = __class__.get_object_0(ply)
            object_ply += ply

        o3d.io.write_point_cloud(os.path.join(
            self.obj_path, f"object{count}.ply"), object_ply)

    @staticmethod
    def load_point_clouds(path: str):
        plys = list()
        files:list = os.listdir(path)
        for file in files:
            ply:o3d.geometry.PointCloud = o3d.io.read_point_cloud(os.path.join(path, file))
            ply.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(
                knn=40))  # 计算法线，只考虑邻域内的40个点
            plys.append(ply)
        return plys

    @staticmethod
    def pairwise_registration(source, target, max_correspondence_distance_coarse, max_correspondence_distance_fine):
        print("Apply point-to-plane ICP")
        icp_coarse = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_coarse, np.identity(4))
        icp_fine = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_fine, np.identity(4))
        transformation_icp = icp_fine.transformation
        information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            source, target, max_correspondence_distance_fine,
            icp_fine.transformation)
        return transformation_icp, information_icp

    @staticmethod
    def full_registration(plys, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine):
        pose_graph = o3d.pipelines.registration.PoseGraph()
        odometry = np.identity(4)
        pose_graph.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(odometry))
        n_plys = len(plys)
        for source_id in range(n_plys):
            for target_id in range(source_id + 1, n_plys):
                transformation_icp, information_icp = Filter.pairwise_registration(
                    plys[source_id], plys[target_id], max_correspondence_distance_coarse, max_correspondence_distance_fine)
                print("Buildo3d.pipelines.registration.PoseGraph")
                if target_id == source_id + 1:  # odometry case
                    odometry = np.dot(transformation_icp, odometry)
                    pose_graph.nodes.append(
                        o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=False))
                else:  # loop closure case
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=True))
        return pose_graph

    @staticmethod
    def registration(input_path: str = os.path.join(util.Configs.workspace, "object_ply"), voxel_size: float = 0.1, output_path: str = os.path.join(util.Configs.workspace, "ans/ans.ply")):
        plys_down:o3d.geometry.PointCloud = Filter.load_point_clouds(input_path)
        print("Full registration ...")
        max_correspondence_distance_coarse:float = voxel_size * 15
        max_correspondence_distance_fine:float = voxel_size * 1.5
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            pose_graph = __class__.full_registration(plys_down,
                                                  max_correspondence_distance_coarse,
                                                  max_correspondence_distance_fine)

        print("Optimizing PoseGraph ...")
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=max_correspondence_distance_fine,
            edge_prune_threshold=0.25,
            reference_node=0)
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            o3d.pipelines.registration.global_optimization(
                pose_graph, o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
                o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(), option)

        print("Transform points and display")
        ans_ply:o3d.geometry.PointCloud = o3d.geometry.PointCloud()
        for point_id in range(len(plys_down)):
            print(pose_graph.nodes[point_id].pose)
            plys_down[point_id].transform(pose_graph.nodes[point_id].pose)
            ans_ply += plys_down[point_id]

        cl, _ = ans_ply.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=1.0)  # 去除离群点
        downply_inlier_cloud:o3d.geometry.PointCloud = cl

        cl, _ = downply_inlier_cloud.remove_radius_outlier(
            nb_points=20, radius=4, print_progress=True)  # 半径滤波
        ans_ply:o3d.geometry.PointCloud = cl

        o3d.io.write_point_cloud(output_path, ans_ply)
        # o3d.visualization.draw_geometries([plys_down])
