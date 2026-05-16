import matplotlib.pyplot as plt

from calculate_paths import PathCalculator

DEFAULT_FONT = 'Arial'
DEFAULT_FONT_SIZE = 40            # mm

SAFE_Z = 0.0
CUT_Z = -3.0
FEED_RATE = 1000  # mm/min
LIFT_RATE = 300  # mm/min


# TODO GEORGI Tape roll diameter adjustment


def contours_to_gcode(contours: list) -> list[str]:
    gcode = [
        "G21",  # mm mode
        "G90"  # absolute positioning
    ]

    for contour in contours:
        x0, y0 = contour[0]
        gcode.append(f"G0 Z{SAFE_Z}")
        gcode.append(f"G0 X{x0:.2f} Y{y0:.2f}")
        gcode.append(f"G1 Z{CUT_Z} F{FEED_RATE}")

        for x, y in contour[1:]:
            gcode.append(f"G1 X{x:.2f} Y{y:.2f}")

        gcode.append(f"G0 Z{SAFE_Z}")

    gcode.append(f"G0 Y0 Z{SAFE_Z}")
    return gcode


def matplot_preview_paths(paths: list) -> None:
    plt.figure(figsize=(12, 3))

    contours_count = 0
    for contour in paths:
        plt.plot(contour[:, 0], contour[:, 1], 'k-')
        contours_count += len(contour)

    plt.axis('equal')
    plt.show()
    print(f'Plotted {contours_count} contours')


def make_text_gcode(text: str, font: str | None, font_size: int | str | None, preview: bool) -> list[str]:
    font_size = int(font_size or DEFAULT_FONT_SIZE)

    calc = PathCalculator(font or DEFAULT_FONT, font_size)

    all_paths = calc.generate_paths(text)
    #print(f'Generated {len(all_paths)} paths')

    if preview:
        matplot_preview_paths(all_paths)

    return contours_to_gcode(all_paths)



