import numpy as np

# Simulation Config
HOME_POS = np.array([-3.0, -0.9, 0.22])
ERROR_BINS = {"A6", "C2"}

# Generate Bin Data
bin_data = {}
for c_idx, col in enumerate(["A", "B", "C"]):
    for lvl in range(1, 7):
        bid = f"{col}{lvl}"
        qty = 10 + c_idx * 6 + lvl
        bin_data[bid] = {"part": f"PN-{col}0{lvl}", "qty": qty}

# Shared Application State
app_running = True
is_flying = False
current_target_id = None
inspected_bins = set()

# UI Colors
BG_COLOR = "#0b1121"         
CARD_BG = "#172033"          
CARD_BORDER = "#2e3c54"      
TEXT_MAIN = "#60a5fa"        
TEXT_SUB = "#64748b"         
TEXT_QTY = "#f8fafc"         
ACTIVE_BORDER = "#f97316"    
COMPLETED_BG = "#064e3b"     
COMPLETED_TEXT = "#34d399"   
ERROR_BG = "#7f1d1d"
ERROR_TEXT = "#fca5a5"
ERROR_BORDER = "#ef4444"