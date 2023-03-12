import open3d as o3d
import numpy as np
import keyboard
import cv2
import util
import os
# import pyk4a


class Recorder():
    def __init__(self, config:o3d.io.AzureKinectSensorConfig, device=0, align_depth_to_color=True) -> None:
        self.config:o3d.io.AzureKinectSensorConfig = config
        self.sensor = o3d.io.AzureKinectSensor(config)
        self.device:int = device
        self.align_depth_to_color:bool = align_depth_to_color
        self.recorder = o3d.io.AzureKinectRecorder(config, device)
        self.__rgbdimages:list = list()
        self.__pointclouds:list = list()
        self.__colors:list = list()
        self.__depths:list = list()
        self.__flag:bool = True
        keyboard.add_hotkey("q", self.stop_record_by_hand)

    def record(self, vedio_path: str, fps=60) -> None:
        self.__pointclouds.clear()
        self.recorder.init_sensor()
        self.recorder.open_record(vedio_path)
        for i in range(fps):
            temp:o3d.geometry.RGBDImage = self.recorder.record_frame(enable_record=True,
                                              enable_align_depth_to_color=self.align_depth_to_color)
            if temp != None:
                self.__rgbdimages.append(temp)
        self.recorder.close_record()
        return None

    def record_by_hand(self, vedio_path: str, distance:float, write_color_and_depth=True, colors_path='', depths_path='') -> None:
        self.__pointclouds.clear()
        self.recorder.init_sensor()
        self.recorder.open_record(vedio_path)
        self.__flag = True
        i:int = 0
        while self.__flag:
            temp:o3d.geometry.RGBDImage = self.recorder.record_frame(enable_record=True,
                                              enable_align_depth_to_color=self.align_depth_to_color)
            if temp != None:
                self.__rgbdimages.append(temp)
                if write_color_and_depth:  # 控制是否将RGB图和深度图保存在磁盘中
                    i += 1
                    print("获取colors和depths...")

                    source_colors:np.ndarray = np.asarray(temp.color)
                    source_depths:np.ndarray = np.asarray(temp.depth)

                    filtered_depths:np.ndarray = np.zeros(
                        source_depths.shape, dtype="float32")
                    filtered_depths[(source_depths[:, :] > 0) & (source_depths[:, :] < distance)] = source_depths[(
                        source_depths[:, :] > 0) & (source_depths[:, :] < distance)]
                    filtered_colors:np.ndarray = np.zeros(
                        source_colors.shape, dtype="uint8")
                    filtered_colors[(source_depths[:, :] > 0) & (source_depths[:, :] < distance)] = source_colors[(
                        source_depths[:, :] > 0) & (source_depths[:, :] < distance)]
                    filtered_colors = filtered_colors[:, :, ::-1]

                    cv2.imwrite(
                        "{}/color_{}.png".format(colors_path, i), filtered_colors)
                    cv2.imwrite(
                        "{}/depth_{}.tif".format(depths_path, i), filtered_depths)

        print("I stopped !")
        self.recorder.close_record()
        return None

    def stop_record_by_hand(self) -> None:
        self.__flag = False
        return None

    @property
    def rgbdimages(self) -> list:
        return self.__rgbdimages

    def build_point_cloud(self, ply_path: str) -> None:
        i:int = 0
        for rgbd in self.__rgbdimages:
            print("Building ...")
            ply:o3d.geometry.PointCloud = o3d.geometry.PointCloud.create_from_rgbd_image(
                o3d.geometry.RGBDImage.create_from_color_and_depth(color= rgbd.color, depth= rgbd.depth, convert_rgb_to_intensity= False),
                o3d.camera.PinholeCameraIntrinsic(1280, 720, 607.270142, 607.071411, 635.536255, 370.521240))
            ply = ply.remove_non_finite_points()
            self.__pointclouds.append(ply)
            o3d.io.write_point_cloud(ply_path+"/{}.ply".format(i), ply)
            print("Build PointCloud: {}.ply".format(i))
            i += 1
        return None

    def get_colors_and_depths(self, distance:float) -> None:
        for rgbd in self.__rgbdimages:
            print("获取colors和depths...")
            source_colors:np.ndarray = np.asarray(rgbd.color)
            source_depths:np.ndarray = np.asarray(rgbd.depth)

            filtered_depths:np.ndarray = np.zeros(source_depths.shape, dtype="float32")
            filtered_depths[(source_depths[:, :] > 0) & (source_depths[:, :] < distance)] = source_depths[(
                source_depths[:, :] > 0) & (source_depths[:, :] < distance)]
            filtered_colors:np.ndarray = np.zeros(source_colors.shape, dtype="uint8")
            filtered_colors[(source_depths[:, :] > 0) & (source_depths[:, :] < distance)] = source_colors[(
                source_depths[:, :] > 0) & (source_depths[:, :] < distance)]
            filtered_colors = filtered_colors[:, :, ::-1]

            self.__colors.append(filtered_colors)
            self.__depths.append(filtered_depths)
        return None

    def write_colors_and_depth(self, colors_path: str, depths_path: str) -> None:
        i = 0
        for colors_img in self.__colors:
            cv2.imwrite("{}/color_{:0>3d}.png".format(colors_path, i), colors_img)
            i += 1
        i = 0
        for depths_img in self.__depths:
            cv2.imwrite("{}/depth_{:0>3d}.tif".format(depths_path, i), depths_img)
            i += 1
        return None
    
    # @staticmethod
    # def get_ir(output_path:str = os.path.join(util.Configs.workspace,"IRs")) -> pyk4a.capture:
    #     k4a = pyk4a.PyK4A()
    #     k4a.start()
    #     capture = k4a.get_capture()
    #     cv2.imwrite("{}/color_{}.png".format(output_path, len(os.listdir(output_path))), capture.transformed_ir)
    #     return capture.transformed_ir

    def get_ir(self, output_path:str = os.path.join(util.Configs.workspace,"IRs"), video_path:str=os.path.join(util.Configs.workspace, "video")) -> None:
        output_path = os.path.join(output_path, "{}".format(len(os.listdir(video_path))-1))
        video_path = os.path.join(video_path, "ir.mkv")
        config:o3d.io.AzureKinectSensorConfig = o3d.io.AzureKinectSensorConfig({
                                                            "camera_fps": "K4A_FRAMES_PER_SECOND_5",
                                                            "color_format": "K4A_IMAGE_FORMAT_IR16",
                                                            "color_resolution": "K4A_COLOR_RESOLUTION_OFF",
                                                            "depth_delay_off_color_usec": "0",
                                                            "depth_mode": "K4A_DEPTH_MODE_PASSIVE_IR",
                                                            "disable_streaming_indicator": "false",
                                                            "subordinate_delay_off_master_usec": "0",
                                                            "synchronized_images_only": "false",
                                                            "wired_sync_mode": "K4A_WIRED_SYNC_MODE_STANDALONE"})
        self.sensor = o3d.io.AzureKinectSensor(config)
        self.recorder = o3d.io.AzureKinectRecorder(config, self.device)
        self.recorder.init_sensor()
        self.recorder.open_record(video_path)
        for i in range(30):
            self.recorder.record_frame(enable_record=True,
                                              enable_align_depth_to_color=False)
        self.recorder.close_record()
        os.system("ffmpeg -i {} -map 0:0 -vsync 0 {}/IR_%03d.png".format(video_path, output_path))
        self.sensor = o3d.io.AzureKinectSensor(self.config)
        self.recorder = o3d.io.AzureKinectRecorder(self.config, self.device)
        return None

    @property
    def colors(self) -> list:
        return self.__colors

    @property
    def depths(self) -> list:
        return self.__depths

    def clean(self) -> None:
        self.__colors.clear()
        self.__depths.clear()
        self.__rgbdimages.clear()
        return None


class JsonRecorder:
    def __init__(self):
        util.Workspace.init_workspace()
        self.__recorder:Recorder = Recorder(util.Pointcloud.load_sensor_config())

    def record(self, byhand:bool = util.Configs.byhand):
        count:int = len(os.listdir(os.path.join(util.Configs.workspace, "video")))
        for path in ("colors", "depths", "IRs"):
            os.mkdir(os.path.join(util.Configs.workspace, "{}/{}".format(path, count)))
        if byhand:
            keyboard.add_hotkey("q", self.recorder.stop_record_by_hand)
            self.__recorder.record_by_hand(vedio_path= os.path.join(util.Configs.workspace, "video/{}.mkv".format(count)), 
                                        distance= util.Configs.distance, 
                                        write_color_and_depth= util.Configs.write_color_and_depth,
                                        colors_path= os.path.join(util.Configs.workspace, "colors/{}".format(count)),
                                        depths_path= os.path.join(util.Configs.workspace, "depths/{}".format(count)))
        else:
            self.__recorder.record(vedio_path= os.path.join(util.Configs.workspace, "video/{}.mkv".format(count)), fps= util.Configs.fps)
            self.__recorder.get_colors_and_depths(distance= util.Configs.distance)
            if util.Configs.write_color_and_depth:
                self.__recorder.write_colors_and_depth(colors_path= os.path.join(util.Configs.workspace, "colors/{}".format(count)),
                                                    depths_path= os.path.join(util.Configs.workspace, "depths/{}".format(count)))
        self.__recorder.build_point_cloud(os.path.join(util.Configs.workspace, "source_ply"))
        self.__recorder.clean()

    # @staticmethod
    # def get_ir(output_path:str = os.path.join(util.Configs.workspace,"IRs")) -> pyk4a.capture:
    #     return Recorder.get_ir(output_path)

    # @staticmethod
    # def get_ir(video_path:str=os.path.join(util.Configs.workspace, "video"), output_path:str=os.path.join(util.Configs.workspace,"IRs")):
    #     output_path = os.path.join(output_path, "{}".format(len(os.listdir(video_path))-1))
    #     video_path = os.path.join(video_path, "{}.mkv".format(len(os.listdir(video_path))-1))
    #     os.system("ffmpeg -i {} -map 0:2 -vsync 0 {}/IR_%04d.png".format(video_path, output_path))

    def get_ir(self, video_path:str=os.path.join(util.Configs.workspace, "video"), output_path:str=os.path.join(util.Configs.workspace,"IRs")):
        self.__recorder.get_ir(output_path, video_path)

    @property
    def recorder(self):
        return self.__recorder