#!/usr/bin/env python3
"""
使用 MuJoCo 可视化 URDF，所有关节角度固定为 0
"""

import mujoco
import mujoco.viewer
import numpy as np
import os
import tempfile
import shutil

def main():
    # 获取绝对路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    meshes_dir = os.path.join(parent_dir, "meshes")
    urdf_path = os.path.join(script_dir, "装配体1 - 副本.urdf")

    print(f"URDF 目录: {script_dir}")
    print(f"Meshes 目录: {meshes_dir}")
    print(f"正在读取 URDF 文件...")

    # 读取 URDF 内容
    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_content = f.read()

    # 简化方法：创建临时工作目录，复制所有文件
    print(f"准备工作目录...")
    with tempfile.TemporaryDirectory() as temp_dir:
        # 创建临时 meshes 目录
        temp_meshes = os.path.join(temp_dir, "meshes")
        os.makedirs(temp_meshes, exist_ok=True)

        # 复制所有 STL 文件
        for stl_file in os.listdir(meshes_dir):
            if stl_file.endswith('.STL'):
                src = os.path.join(meshes_dir, stl_file)
                dst = os.path.join(temp_meshes, stl_file)
                shutil.copy2(src, dst)

        # 修改 URDF 内容，使用相对路径
        package_name = "装配体1 - 副本"
        pattern = f'package://{package_name}/meshes/'
        urdf_content = urdf_content.replace(pattern, 'meshes/')

        # 保存临时 URDF 文件
        temp_urdf = os.path.join(temp_dir, "robot.urdf")
        with open(temp_urdf, 'w', encoding='utf-8') as f:
            f.write(urdf_content)

        print(f"临时目录内容:")
        print(f"  根目录: {os.listdir(temp_dir)}")
        print(f"  meshes 目录: {os.listdir(temp_meshes)}")

        # 检查第一个mesh文件路径
        import re
        mesh_paths = re.findall(r'filename="([^"]+\.STL)"', urdf_content, re.IGNORECASE)
        if mesh_paths:
            print(f"  URDF 中的第一个 mesh 路径: {mesh_paths[0]}")

        # 保存原始工作目录
        original_dir = os.getcwd()

        try:
            # 切换到临时目录
            os.chdir(temp_dir)

            print(f"当前工作目录: {os.getcwd()}")
            print(f"正在加载 URDF 模型...")

            # 使用 from_xml_string 并提供 assets 字典
            assets = {}
            for stl_file in os.listdir(temp_meshes):
                if stl_file.endswith('.STL'):
                    stl_path = os.path.join(temp_meshes, stl_file)
                    with open(stl_path, 'rb') as f:
                        stl_data = f.read()
                        # 只注册 meshes/xxx.STL 路径
                        assets[f'meshes/{stl_file}'] = stl_data

            print(f"注册了 {len(assets)} 个资源文件")

            model = mujoco.MjModel.from_xml_string(urdf_content, assets)
            data = mujoco.MjData(model)

            print(f"✓ 成功加载模型！")
            print(f"  关节数量: {model.nu}")
            print(f"  自由度数量: {model.nv}")
            print(f"  位置变量数量: {model.nq}")

            # 打印所有关节名称
            print(f"\n关节列表:")
            for i in range(model.njnt):
                joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                joint_type = model.jnt_type[i]
                type_names = ['free', 'ball', 'slide', 'hinge']
                type_name = type_names[joint_type] if joint_type < len(type_names) else 'unknown'
                print(f"  {i+1}. {joint_name} (类型: {type_name})")

            # 将所有关节角度设置为 0
            print(f"\n将所有关节角度设置为 0...")
            data.qpos[:] = 0.0
            data.qvel[:] = 0.0

            # 前向运动学计算
            mujoco.mj_forward(model, data)

            print(f"当前关节位置: {data.qpos}")
            print(f"\n正在启动 MuJoCo 可视化窗口...")
            print(f"\n操作提示:")
            print(f"  • 鼠标左键拖动: 旋转视角")
            print(f"  • 鼠标右键拖动: 平移视角")
            print(f"  • 鼠标滚轮: 缩放")
            print(f"  • 空格键: 暂停/继续")
            print(f"  • 按 Esc 退出")
            print(f"  • 起始姿态为 0 度，可自由运动\n")

            # 使用 MuJoCo passive viewer 可视化
            with mujoco.viewer.launch_passive(model, data) as viewer:
                # 设置相机位置以获得更好的视角
                viewer.cam.distance = 1.5
                viewer.cam.azimuth = 45
                viewer.cam.elevation = -20

                # 保持窗口打开并强制所有关节保持 0 角度/速度
                while viewer.is_running():
                    data.qpos[:] = 0.0
                    data.qvel[:] = 0.0
                    mujoco.mj_forward(model, data)
                    viewer.sync()

        finally:
            # 恢复原始工作目录
            os.chdir(original_dir)

        print(f"\n可视化已关闭。")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()
