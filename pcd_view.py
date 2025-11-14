import open3d as o3d
import glob

pcd_files = sorted(glob.glob("./*.pcd"))
index = 0

def load_and_show(idx):
    pcd = o3d.io.read_point_cloud(pcd_files[idx])
    vis.clear_geometries()
    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    print(f"Viewing {idx+1}/{len(pcd_files)}: {pcd_files[idx]}")

def next_frame(vis):
    global index
    index = (index + 1) % len(pcd_files)
    load_and_show(index)
    return False  # 必须返回值！

def prev_frame(vis):
    global index
    index = (index - 1) % len(pcd_files)
    load_and_show(index)
    return False

def close_window(vis):
    vis.close()
    return False

vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window()
vis.register_key_callback(ord("D"), next_frame)      # D键 下一帧
vis.register_key_callback(ord("A"), prev_frame)      # A键 上一帧
vis.register_key_callback(256, close_window)         # Esc键关闭

load_and_show(index)
vis.run()
vis.destroy_window()
