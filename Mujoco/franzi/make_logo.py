"""Draw assets/logo.png - the panel texture for the torso and chassis.

A mostly-light square (the box faces stay near-white) with the robot mark
centred: orange rounded plate, 汇川 wordmark, FRANZI ROBOTICS beneath.

Run:  python3 make_logo.py    (needs Pillow and the Noto CJK fonts)
"""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
OUT = HERE / "assets" / "logo.png"
SIZE = 512

FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
]


def cjk_font(size):
    for path in FONT_CANDIDATES:
        if not Path(path).exists():
            continue
        # Pick the collection face that actually has simplified 汇.
        for index in range(6):
            try:
                font = ImageFont.truetype(path, size, index=index)
            except OSError:
                break
            if font.getmask("汇").getbbox() is not None:
                return font
    raise SystemExit("no CJK font found - install fonts-noto-cjk")


image = Image.new("RGB", (SIZE, SIZE), (225, 227, 230))
draw = ImageDraw.Draw(image)

# Brushed-panel hint: faint horizontal lines.
for y in range(0, SIZE, 8):
    draw.line([(0, y), (SIZE, y)], fill=(218, 220, 224), width=1)
draw.rectangle([0, 0, SIZE - 1, SIZE - 1], outline=(190, 193, 198), width=4)

# Orange plate with the wordmark.
plate = (96, 150, SIZE - 96, 300)
draw.rounded_rectangle(plate, radius=24, fill=(226, 88, 18))
word = "汇川"
font_big = cjk_font(110)
box = draw.textbbox((0, 0), word, font=font_big)
draw.text(
    ((SIZE - box[2] + box[0]) / 2, (plate[1] + plate[3]) / 2 - (box[3] + box[1]) / 2),
    word,
    font=font_big,
    fill=(255, 255, 255),
)

sub = "FRANZI ROBOTICS"
font_small = cjk_font(34)
box = draw.textbbox((0, 0), sub, font=font_small)
draw.text(
    ((SIZE - box[2] + box[0]) / 2, 330),
    sub,
    font=font_small,
    fill=(90, 94, 100),
)

OUT.parent.mkdir(exist_ok=True)
# MuJoCo's cube mapping rotates the image a quarter turn on the side faces;
# pre-rotate so the wordmark reads horizontally there.
image.rotate(90).save(OUT)
print(f"wrote {OUT}")

# The chest decal, same artwork Isaac puts on the robot (IssacSim/paint.py).
FMC3 = HERE.parents[1] / "IssacSim" / "usd" / "textures" / "fmc3_logo.png"
if not FMC3.exists():
    import sys

    sys.path.insert(0, str(HERE.parents[1] / "IssacSim"))
    from paint import write_logo

    write_logo(FMC3)
decal = Image.open(FMC3).rotate(90)
decal.save(OUT.parent / "fmc3_logo.png")
print(f"wrote {OUT.parent / 'fmc3_logo.png'}")
