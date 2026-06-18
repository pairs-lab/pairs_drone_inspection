import time
import numpy as np
import mujoco
import mujoco.viewer
from src import config

def get_bin_coords(bin_id):
    if bin_id == "HOME": return config.HOME_POS
    col_map = {'A': 0.0, 'B': 1.2, 'C': 2.4}
    x = col_map[bin_id[0]]
    z = (int(bin_id[1]) - 1) * 0.8 + 0.25 
    return np.array([x, -0.8, z])

def fly_to_target(model, data, mocap_id, target_pos, viewer, renderer, ui_camera_callback):
    start_pos = data.mocap_pos[mocap_id].copy()
    distance  = np.linalg.norm(target_pos - start_pos)
    duration  = max(distance / 2.0, 0.5)
    steps     = int(duration * 100)
    
    render_skip = 2
    for step in range(steps):
        if not config.app_running: break
        t = step / steps
        alpha = t * t * (3 - 2 * t)
        data.mocap_pos[mocap_id] = (1 - alpha) * start_pos + alpha * target_pos
        mujoco.mj_step(model, data)
        viewer.sync()
        
        if step % render_skip == 0:
            renderer.update_scene(data, camera="fpv_cam")
            pixels = renderer.render()
            ui_camera_callback(pixels)
            
        time.sleep(0.01)

def change_bin_color(model, bin_id, color_rgba):
    if bin_id and bin_id != "HOME":
        geom_name = f"geom_{bin_id}"
        geom_idx  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        if geom_idx != -1:
            model.geom_rgba[geom_idx] = color_rgba

def mujoco_thread(ui_update_cb, ui_result_cb, ui_camera_cb):
    model = mujoco.MjModel.from_xml_path('/home/admin1/pairs_drone_inspection/inspection_sim/mujoco_sim/inspection_mujoco_demo/models/scene.xml')
    data  = mujoco.MjData(model)
    mocap_id = model.body('drone_body').mocapid[0]
    renderer = mujoco.Renderer(model, height=360, width=480) 

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 7.5
        viewer.cam.elevation = -18
        viewer.cam.azimuth = 145
        viewer.cam.lookat = [1.2, -0.7, 2.2]

        while viewer.is_running() and config.app_running:
            if config.current_target_id is not None and not config.is_flying:
                config.is_flying = True
                target = config.current_target_id
                target_coords = get_bin_coords(target)
                current_drone_pos = data.mocap_pos[mocap_id].copy()

                if target != "HOME":
                    change_bin_color(model, target, [1.0, 0.5, 0.0, 1.0])
                    ui_update_cb(target, "active")
                    viewer.sync()

                fly_to_target(model, data, mocap_id, np.array([current_drone_pos[0], -0.9, current_drone_pos[2]]), viewer, renderer, ui_camera_cb)
                fly_to_target(model, data, mocap_id, np.array([target_coords[0], -0.9, target_coords[2]]), viewer, renderer, ui_camera_cb)

                if target != "HOME":
                    fly_to_target(model, data, mocap_id, target_coords, viewer, renderer, ui_camera_cb)
                    
                    start_scan = time.time()
                    while time.time() - start_scan < 2.0:
                        if not config.app_running: break
                        mujoco.mj_step(model, data) 
                        viewer.sync()
                        renderer.update_scene(data, camera="fpv_cam")
                        ui_camera_cb(renderer.render())
                        time.sleep(0.05) 
                    
                    if target in config.ERROR_BINS:
                        for i in range(6): 
                            if not config.app_running: break
                            if i % 2 == 0:
                                change_bin_color(model, target, [0.9, 0.1, 0.1, 1.0]) 
                            else:
                                change_bin_color(model, target, [1.0, 0.5, 0.0, 1.0]) 
                            
                            blink_start = time.time()
                            while time.time() - blink_start < 0.2:
                                mujoco.mj_step(model, data)
                                viewer.sync()
                                renderer.update_scene(data, camera="fpv_cam")
                                ui_camera_cb(renderer.render())
                                time.sleep(0.05)

                        change_bin_color(model, target, [0.9, 0.1, 0.1, 1.0]) 
                        ui_result_cb(target, "error")
                    else:
                        change_bin_color(model, target, [0.1, 0.9, 0.2, 1.0]) 
                        ui_result_cb(target, "completed")
                        
                    config.inspected_bins.add(target)
                    viewer.sync()
                else:
                    fly_to_target(model, data, mocap_id, config.HOME_POS, viewer, renderer, ui_camera_cb)
                    ui_camera_cb(None) 
                    
                config.is_flying = False
                config.current_target_id = None
            else:
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.01)