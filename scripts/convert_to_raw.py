import argparse
from PIL import Image


def parse_data(im, out_file):
    px = im.load()
    data = bytearray()
    for y in range(im.height):
        byte = 0
        idx = 7
        for x in range(im.width):
            byte |= (px[x, y] > 0) << idx
            idx -= 1
            if idx < 0:
                data.append(byte)
                byte = 0
                idx = 7
        if idx < 7:
            data.append(byte)
    return data


def main():
    parser = argparse.ArgumentParser(prog="Convert image to raw image format")
    parser.add_argument("in_file", type=str, help="File to convert")
    args = parser.parse_args()

    out_file = args.in_file.split(".")[0]
    out_file += ".raw"
    with Image.open(args.in_file) as im:
        im: Image = im.convert("1")

    print(f"{im.width}x{im.height}")
    data = parse_data(im, out_file)
    with open(out_file, "wb") as fd:
        fd.write(data)
    print(f"Output file at: {out_file}, width: {im.width}")


if __name__ == "__main__":
    main()
