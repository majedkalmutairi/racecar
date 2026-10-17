"""Generate tracks/oval.png: white = drivable, black = wall."""
from pathlib import Path

from PIL import Image, ImageDraw

W, H = 1000, 700
OUTER = (30, 30, 970, 670)    # left, top, right, bottom in pixels
INNER = (170, 170, 830, 530)  # 140 px = 1.4 m lane at 100 px/m

out = Path(__file__).resolve().parent.parent / "tracks" / "oval.png"

img = Image.new("RGB", (W, H), "black")
draw = ImageDraw.Draw(img)
draw.ellipse(OUTER, fill="white")
draw.ellipse(INNER, fill="black")
img.save(out)
print(f"saved {out}")
