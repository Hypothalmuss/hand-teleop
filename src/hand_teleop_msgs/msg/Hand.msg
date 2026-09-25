bool present
float64[63] landmarks        # 21 x (x, y, z), MediaPipe normalized image coords, mirrored
float64 palm_scale           # wrist (0) to middle_mcp (9) distance, normalized units
float64 pinch                # thumb_tip(4)-index_tip(8) distance / palm_scale
bool is_open
bool is_fist
