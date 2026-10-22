"""RGB-D red-target localization without learned models or OpenCV."""

from __future__ import annotations

from collections import deque
from numbers import Integral

import numpy as np


def red_mask(rgb):
    """Return an HSV threshold mask, including both sides of hue's red wrap.

    Hue is measured in degrees. Saturation >= 0.45 and value >= 0.25 reject
    low-color and nearly black pixels. This is a synthetic-scene baseline,
    not a general object detector or a claim about real-camera robustness.
    """
    image = np.asarray(rgb)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("rgb must be an H-by-W-by-3 uint8 array")
    if image.shape[0] == 0 or image.shape[1] == 0:
        raise ValueError("rgb must have nonzero height and width")
    channels = image.astype(float) / 255.0
    red, green, blue = np.moveaxis(channels, -1, 0)
    highest = channels.max(axis=2)
    lowest = channels.min(axis=2)
    spread = highest - lowest
    colored = spread > 0
    saturation = np.divide(spread, highest, out=np.zeros_like(spread), where=highest > 0)
    hue = np.zeros_like(highest)
    red_high = colored & (highest == red)
    green_high = colored & ~red_high & (highest == green)
    blue_high = colored & ~red_high & ~green_high
    hue[red_high] = ((green[red_high] - blue[red_high]) / spread[red_high]) % 6
    hue[green_high] = (blue[green_high] - red[green_high]) / spread[green_high] + 2
    hue[blue_high] = (red[blue_high] - green[blue_high]) / spread[blue_high] + 4
    hue *= 60.0
    return ((hue <= 20.0) | (hue >= 340.0)) & (saturation >= 0.45) & (highest >= 0.25)


def _matrix(values, name):
    try:
        original = np.asarray(values)
        if original.dtype.kind not in "iuf":
            raise ValueError("non-real numeric dtype")
        array = original.astype(float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain 16 finite matrix values") from exc
    # Flat PyBullet matrices are OpenGL column-major. Explicit 4-by-4 arrays
    # are already mathematical matrices and do not need another transpose.
    if array.shape == (16,):
        array = array.reshape(4, 4, order="F")
    elif array.shape != (4, 4):
        raise ValueError(f"{name} must be a flat length-16 or 4-by-4 matrix")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def _largest_component(mask):
    height, width = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    largest = []
    # Four-neighbor connectivity keeps isolated diagonal noise separate.
    for v, u in zip(*np.nonzero(mask)):
        if seen[v, u]:
            continue
        seen[v, u] = True
        queue = deque([(int(v), int(u))])
        component = []
        while queue:
            y, x = queue.popleft()
            component.append((y, x))
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < height and 0 <= nx < width and mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    queue.append((ny, nx))
        if len(component) > len(largest):
            largest = component
    return largest


def detect_red(rgb, depth_buffer, view_matrix, projection_matrix, min_area=15):
    """Locate the largest red component and unproject its visible surface.

    Returns ``None`` if no component reaches ``min_area`` or if it has no
    foreground depth. Returned pixel coordinates are image indices [u, v],
    possibly fractional. The mapping uses pixel centers: x=2*(u+0.5)/W-1 and
    y=1-2*(v+0.5)/H. OpenGL depth d maps to clip z=2*d-1.

    The valid-component centroid and median depth are unprojected through
    inverse(P @ V). The result is a visible *surface* point, not an object's
    center. A simulation may use its known synthetic geometry to compensate.
    """
    if isinstance(min_area, (bool, np.bool_)) or not isinstance(min_area, Integral) or min_area < 1:
        raise ValueError("min_area must be a positive integer")
    mask = red_mask(rgb)
    try:
        original_depth = np.asarray(depth_buffer)
        if original_depth.dtype.kind not in "iuf":
            raise ValueError("non-real numeric dtype")
        depth = original_depth.astype(float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("depth_buffer must be a finite H-by-W array in [0, 1]") from exc
    if depth.shape != mask.shape or not np.all(np.isfinite(depth)):
        raise ValueError("depth_buffer must match the RGB shape and contain finite values")
    if np.any(depth < 0) or np.any(depth > 1):
        raise ValueError("depth_buffer values must be in [0, 1]")
    view = _matrix(view_matrix, "view_matrix")
    projection = _matrix(projection_matrix, "projection_matrix")
    try:
        transform = np.linalg.inv(projection @ view)
    except np.linalg.LinAlgError as exc:
        raise ValueError("projection @ view must be invertible") from exc
    if not np.all(np.isfinite(transform)):
        raise ValueError("camera transform must be finite")
    component = _largest_component(mask)
    if len(component) < min_area:
        return None
    coordinates = np.asarray(component, dtype=int)
    ys, xs = coordinates[:, 0], coordinates[:, 1]
    component_depth = depth[ys, xs]
    valid = (component_depth > 0) & (component_depth < 1)
    if not np.any(valid):
        return None
    u, v = float(xs[valid].mean()), float(ys[valid].mean())
    d = float(np.median(component_depth[valid]))
    height, width = mask.shape
    clip = np.array([2 * (u + 0.5) / width - 1, 1 - 2 * (v + 0.5) / height, 2 * d - 1, 1])
    world = transform @ clip
    if not np.all(np.isfinite(world)) or abs(float(world[3])) < 1e-12:
        raise ValueError("unprojected surface point is not finite")
    surface = world[:3] / world[3]
    if not np.all(np.isfinite(surface)):
        raise ValueError("unprojected surface point is not finite")
    return {"pixel": [u, v], "surface_point": surface.tolist(), "area": len(component)}
