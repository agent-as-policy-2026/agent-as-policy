# tool: contact_sheet.py
# category: process
# purpose: Build a labeled thumbnail contact sheet from a directory of PNG images.
# usage: python3 knowledge/tools/contact_sheet.py <input_dir> <output_image>
# inputs/outputs: Reads PNG files in input_dir; writes a JPEG or PNG contact sheet to output_image.
# assumptions: Pillow is installed; image inputs use the .png extension.
# verified: used successfully in the session that wrote it
from pathlib import Path
from PIL import Image, ImageDraw
import sys

paths = sorted(Path(sys.argv[1]).glob('*.png'))
thumb_w, thumb_h, cols = 384, 216, 5
rows = (len(paths) + cols - 1) // cols
out = Image.new('RGB', (cols * thumb_w, rows * (thumb_h + 22)), 'white')
for i, path in enumerate(paths):
    im = Image.open(path).convert('RGB')
    im.thumbnail((thumb_w, thumb_h))
    x, y = (i % cols) * thumb_w, (i // cols) * (thumb_h + 22)
    out.paste(im, (x, y))
    ImageDraw.Draw(out).text((x + 4, y + thumb_h), path.stem, fill='black')
out.save(sys.argv[2])
