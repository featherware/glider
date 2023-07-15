def box_wing_xml(size: list = [15, 0.3, 30], density=23) -> str:
    return f"""<geom name="main_wing" type="box" size="{" ".join([str(dim) for dim in size])}" density="{density}" rgba="{WING_RGBA}" pos="0 0 0"/>"""

def delta_wing_coords(wing_span_m=20, sweep_back_deg=20, wing_chord_m=3) -> tuple:

    sweep_back_rad = sweep_back_deg * 2 * np.pi / 360
    # tan(sweep_back) = wing_trail / (wing_span / 2)
    wing_tip_trail_m = np.tan(sweep_back_rad) * (wing_span_m / 2)

    leading_point = np.array([0, 0, 0])

    wing_tip_left = np.array([
        wing_tip_trail_m,
        0,
        -wing_span_m / 2
    ])

    wing_tip_right = np.array([
        wing_tip_trail_m,
        0,
        wing_span_m / 2
    ])

    trailing_point = np.array([
        wing_chord_m,
        0,
        0
    ])

    return (leading_point, wing_tip_left, trailing_point, wing_tip_right)

def delta_wing_xml(wing_span_m=20, sweep_back_deg=20, wing_chord_m=3)
    
    (
        leading_point, 
        wing_tip_left, 
        trailing_point, 
        wing_tip_right
     ) = delta_wing_coords(
        wing_span_m=20, 
        sweep_back_deg=20, 
        wing_chord_m=3
    )

    asset = """"""
    xml = """<geom name="delta-wing" type="mesh">"""

    # TODO