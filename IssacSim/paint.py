"""Paint the Franzi robot and put the FMC3-Robotics logo on its chest.

The URDF ships with every visual material set to plain white (SolidWorks
export default), so the converted USD renders as an all-white robot. The
scheme here follows the FMC3 reference photos: gloss white body panels,
near-black visor head, dark joint connectors, black hands and sensors,
rubber-black tyres.

Materials are bound to each link's ``visuals`` scope with
``strongerThanDescendants`` so they override whatever the importer authored
on the meshes below. Binding on ``visuals`` - not on the link itself - keeps
the logo quad (a sibling of ``visuals``) in charge of its own texture.
"""

import re
from pathlib import Path

TEXTURES = Path(__file__).resolve().parent / "usd" / "textures"

# First regex that matches the link name wins; (rgb, roughness, metallic).
WHITE = ((0.85, 0.86, 0.88), 0.30, 0.0)
DARK = ((0.09, 0.095, 0.105), 0.50, 0.0)
VISOR = ((0.02, 0.02, 0.025), 0.12, 0.0)
TIRE = ((0.04, 0.04, 0.045), 0.90, 0.0)
PALETTE = [
    (r"head_pitch", VISOR),
    (r"finger|d435|d405|MID360|lidar", DARK),
    (r"wheel_Link", TIRE),
    (r"waist|head_yaw|shoulder_pitch|elbow_pitch|wrist_roll|steering", DARK),
    (r".", WHITE),
]


def _style(link_name):
    return next(style for pattern, style in PALETTE if re.search(pattern, link_name))


def _material(stage, path, rgb, roughness, metallic):
    from pxr import Gf, Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def paint_robot(stage, root="/World/Robot", looks="/World/Looks"):
    """Bind the palette to every link under ``root``. Idempotent."""
    from pxr import UsdShade

    materials = {}
    for link in stage.GetPrimAtPath(root).GetChildren():
        visuals = stage.GetPrimAtPath(f"{link.GetPath()}/visuals")
        if not visuals.IsValid():
            continue
        style = _style(link.GetName())
        if style not in materials:
            materials[style] = _material(
                stage, f"{looks}/franzi_{len(materials)}", *style
            )
        api = UsdShade.MaterialBindingAPI.Apply(visuals)
        api.Bind(
            materials[style], bindingStrength=UsdShade.Tokens.strongerThanDescendants
        )


def write_logo(path=TEXTURES / "fmc3_logo.png", side=1024):
    """Render the FMC3-Robotics chest decal after the official wordmark:
    geometric 'FMC' (detached F top bar, socket-style C with a square in its
    mouth), superscript 3, 'ROBOTICS' letter-spaced beneath. Dark on
    body-white; drawn from primitives because the brand face is custom."""
    from PIL import Image, ImageDraw, ImageFont

    path = Path(path)
    bold = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    ink, body_white = (30, 32, 38), (238, 239, 241)
    image = Image.new("RGB", (side, side), body_white)
    draw = ImageDraw.Draw(image)

    height = int(side * 0.21)  # capital height: the mark is wide and squat
    stroke = int(height * 0.26)
    radius = stroke // 2
    top = int(side * 0.36)
    bottom = top + height
    gap = int(height * 0.11)
    x = 0  # laid out from 0, centred by shifting at the end

    def bar(x0, y0, x1, y1):
        draw.rounded_rectangle((x0, y0, x1, y1), radius=radius, fill=ink)

    # F - the top bar floats clear of the stem, middle bar cut short.
    f_width = int(height * 0.88)
    bar(x, top, x + f_width, top + stroke)
    bar(x, top + stroke + gap, x + stroke, bottom)
    bar(x, top + int(height * 0.52), x + int(f_width * 0.62), top + int(height * 0.52) + stroke)
    x += f_width + int(height * 0.16)

    # M - square shoulders, a narrow centre stub dropping past half.
    m_width = int(height * 1.05)
    bar(x, top, x + stroke, bottom)
    bar(x + m_width - stroke, top, x + m_width, bottom)
    bar(x, top, x + m_width, top + stroke)
    mid = x + m_width // 2
    stub = int(stroke * 0.8)
    bar(mid - stub // 2, top, mid + stub // 2, top + int(height * 0.58))
    x += m_width + int(height * 0.16)

    # C - a socket: closed shell open wide to the right, square in the mouth.
    c_width = int(height * 1.08)
    mouth = int(height * 0.24)
    draw.rounded_rectangle((x, top, x + c_width, bottom), radius=int(height * 0.20), fill=ink)
    draw.rounded_rectangle(
        (x + stroke, top + stroke, x + c_width - stroke, bottom - stroke),
        radius=int(height * 0.08),
        fill=body_white,
    )
    draw.rectangle(
        (x + c_width - stroke, top + mouth, x + c_width, bottom - mouth),
        fill=body_white,
    )
    square = int(height * 0.30)
    centre_y = (top + bottom) // 2
    draw.rectangle(
        (
            x + c_width - stroke - gap - square,
            centre_y - square // 2,
            x + c_width - stroke - gap,
            centre_y + square // 2,
        ),
        fill=ink,
    )
    x += c_width + int(height * 0.12)

    # Superscript 3.
    sup = ImageFont.truetype(bold, int(height * 0.55))
    draw.text((x, top - int(height * 0.18)), "3", font=sup, fill=ink)
    sup_box = draw.textbbox((x, 0), "3", font=sup)
    width = sup_box[2]

    # Centre the wordmark, then set ROBOTICS under it.
    shift = (side - width) // 2
    wordmark = image.crop((0, 0, width, bottom + 1))
    image.paste(body_white, (0, 0, side, bottom + 1))
    image.paste(wordmark, (shift, 0))

    small = ImageFont.truetype(bold, int(side * 0.062))
    word, spacing = "ROBOTICS", int(side * 0.030)
    letters = [draw.textbbox((0, 0), c, font=small) for c in word]
    total = sum(b[2] - b[0] for b in letters) + spacing * (len(word) - 1)
    x, y = (side - total) // 2, bottom + int(height * 0.42)
    for char, box in zip(word, letters):
        draw.text((x, y), char, font=small, fill=ink)
        x += (box[2] - box[0]) + spacing
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return path


def spawn_logo(stage, torso="/World/Robot/torso_Link", size=0.16, height=0.60):
    """A decal quad just proud of the chest, riding the torso link.

    The chest plane is found from the torso's own bounding box (in the link
    frame), so nothing here depends on where the mesh was modelled. ``height``
    places the decal centre as a fraction of the torso's vertical extent.
    """
    from pxr import Usd, UsdGeom

    from render_cell import spawn_textured_quad

    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    box = cache.ComputeUntransformedBound(
        stage.GetPrimAtPath(f"{torso}/visuals")
    ).ComputeAlignedBox()
    low, high = box.GetMin(), box.GetMax()
    position = (
        high[0] + 0.002,
        (low[1] + high[1]) / 2.0,
        low[2] + height * (high[2] - low[2]),
    )
    quad = spawn_textured_quad(
        stage, f"{torso}/logo", size, write_logo(), position, yaw_degrees=90.0
    )
    # The quad is authored facing +z in the xy plane; the extra x-rotation
    # (applied first, before the yaw) stands it upright facing the link's +x
    # with the text unmirrored to someone standing in front of the robot.
    UsdGeom.Xformable(quad).AddRotateXOp().Set(90.0)
    print(f"logo decal at {tuple(round(v, 3) for v in position)} on {torso}")
    return quad
