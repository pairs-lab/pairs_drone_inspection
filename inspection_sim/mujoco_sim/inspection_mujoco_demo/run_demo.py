import threading
from src.environment import generate_warehouse_xml
from src.simulation import mujoco_thread
from src.ui import OperatorConsole

def main():
    # Generate XML and Assets first
    generate_warehouse_xml()
    
    # Initialize the UI (runs on main thread)
    app = OperatorConsole()

    # Start MuJoCo Simulation in a background thread
    sim_thread = threading.Thread(
        target=mujoco_thread, 
        args=(app.set_card_state, app.handle_inspection_result, app.update_camera_ui)
    )
    sim_thread.start()

    # Start the UI event loop
    app.run()

if __name__ == "__main__":
    main()