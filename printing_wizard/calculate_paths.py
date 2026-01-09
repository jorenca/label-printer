from typing import Union, Any

import freetype
import numpy as np
from matplotlib import font_manager


# Thank you, ChatGPT, although a bit of a mess


def find_font_path(family="Arial", weight="regular"):
    prop = font_manager.FontProperties(
        family=family,
        weight=weight
    )
    font_path = font_manager.findfont(prop, fallback_to_default=False)
    return font_path


def normalize_contour(points, tags):
    on = lambda i: (tags[i] & 1) != 0

    pts = []
    n = len(points)

    for i in range(n):
        pts.append((points[i], on(i)))
        nxt = (i + 1) % n
        if not on(i) and not on(nxt):
            # insert implied on-curve midpoint
            mid = (points[i] + points[nxt]) / 2
            pts.append((mid, True))

    # ensure starts on-curve
    if not pts[0][1]:
        mid = (pts[-1][0] + pts[0][0]) / 2
        pts.insert(0, (mid, True))

    return pts


def contour_to_segments(points, tags):
    on = lambda t: (t & 1) != 0

    # --- normalize contour (insert implied on-curve points) ---
    norm = []
    n = len(points)

    for i in range(n):
        norm.append((points[i], on(tags[i])))
        j = (i + 1) % n
        if not on(tags[i]) and not on(tags[j]):
            mid = (points[i] + points[j]) / 2
            norm.append((mid, True))

    # ensure starts on-curve
    if not norm[0][1]:
        mid = (norm[-1][0] + norm[0][0]) / 2
        norm.insert(0, (mid, True))

    # --- explicitly close contour ---
    norm.append(norm[0])

    # --- build segments ---
    segments = []
    i = 0
    m = len(norm)

    while i < m - 1:
        p0, on0 = norm[i]
        p1, on1 = norm[i + 1]

        if on0 and on1:
            segments.append(("line", p0, p1))
            i += 1

        elif on0 and not on1:
            p2, _ = norm[i + 2]
            segments.append(("quad", p0, p1, p2))
            i += 2

        else:
            raise RuntimeError("Invalid contour topology")

    return segments


def flatten_quadratic_adaptive(p0, p1, p2, tol):
    """Recursive flattening of quadratic Bézier"""
    mid_line = (p0 + p2) / 2
    mid_curve = 0.25*p0 + 0.5*p1 + 0.25*p2
    d = np.linalg.norm(mid_curve - mid_line)
    if d <= tol:
        return [p0, p2]
    # Subdivide
    p01 = (p0 + p1) / 2
    p12 = (p1 + p2) / 2
    p012 = (p01 + p12) / 2
    left = flatten_quadratic_adaptive(p0, p01, p012, tol)
    right = flatten_quadratic_adaptive(p012, p12, p2, tol)
    return left[:-1] + right  # avoid duplicate midpoints


def segments_to_polyline(segments, tolerance=0.01):
    poly = []
    for seg in segments:
        if seg[0] == "line":
            poly.append(seg[1])
            poly.append(seg[2])
        else:
            pts = flatten_quadratic_adaptive(*seg[1:], tol=tolerance)
            poly.extend(pts[:-1])
    return np.array(poly)


class PathCalculator:
    def __init__(self, font_name: str, font_size: int):

        font_path = find_font_path(font_name)

        self.face = freetype.Face(font_path)
        self.face.set_char_size(int(font_size * 64))
        #face.set_pixel_sizes(0, int(FONT_SIZE))

        self.ascender = self.face.size.ascender / 64.0

    def get_kerning(self, left_char, right_char):
        if not self.face.has_kerning:
            return 0.0

        return self.face.get_kerning(
            ord(left_char),
            ord(right_char),
            freetype.FT_KERNING_DEFAULT
        ).x / 64.0

    def glyph_to_paths(self, char: str, x_offset: float, y_offset: float) -> tuple[list[np.ndarray], Union[float, Any]]:
        paths = []
        self.face.load_char(char)
        glyph = self.face.glyph
        outline = glyph.outline

        tags = outline.tags
        points = np.array(outline.points, dtype=float)
        if len(points) > 0:
            points /= 64
            points[:, 0] += x_offset
            points[:, 1] += y_offset

        start = 0
        for end in outline.contours:
            contour_pts = points[start:end + 1]
            contour_tags = tags[start:end + 1]

            segments = contour_to_segments(contour_pts, contour_tags)
            polyline = segments_to_polyline(segments)

            polyline[:, 0] += x_offset
            polyline[:, 1] += self.ascender

            paths.append(polyline)
            start = end + 1

        advance = glyph.advance.x / 64.0
        return paths, advance

    def generate_paths(self, text: str):
        all_paths = []

        x_cursor = 0.0
        y_cursor = -self.ascender

        prev_char = None

        for char in text:
            if prev_char:
                x_cursor += self.get_kerning(prev_char, char)

            paths, advance = self.glyph_to_paths(char, x_cursor, y_cursor)
            all_paths.extend(paths)

            x_cursor += advance/2
            prev_char = char

        return all_paths

