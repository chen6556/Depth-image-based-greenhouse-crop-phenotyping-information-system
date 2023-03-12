import util
from recorder import JsonRecorder
from filter import Filter
from solver import Solver
import os
import time
import multiprocessing.shared_memory as mpsm


if __name__ == "__main__":
    myrecorder = JsonRecorder()
    myfilter = Filter()
    mysolver = Solver()
    util.Workspace.init_workspace()
    if util.Configs.gui:
        os.system('python {}/ANS/ui.pyw'.format(os.getcwd().replace('\\', '/')))
        info = mpsm.ShareableList(name="info")
        while not info[1]:
            time.sleep(0.1)
        counts = info[0]
        info[1] = False
    else:
        counts = util.Configs.counts
        
    for i in range(counts):
        # 录制点云
        if util.Configs.gui:
            while not info[1]:
                time.sleep(0.1)
            info[1] = False
        else:
            input("{} 按Enter键开始录制.".format(i))
        print("正在录制...")
        myrecorder.record()
        
        print("请输入位置参数:")
        myrecorder.get_ir()  # 处理点云并获取数据
        if util.Configs.gui:
            while not info[6]:
                time.sleep(0.1)
            info[6] = False
            pos = [info[2], info[3], info[4], info[5]]
        else:
            pos = util.Pos.get_pos()
        print("已获取位置参数.")
        util.Pos.write_pos(pos, i)
        myfilter.preprocess_2(pos)

    for i in range(counts):
        myfilter.get_object_1(i)
    myfilter.registration()
    mysolver.load_pointcloud()
    mysolver.solve()