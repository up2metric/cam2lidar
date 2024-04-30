import os
from PIL import Image

class SubImage(object):
    def __init__(self, pil_image, coordinates):
        # PIL image object
        self.img = pil_image
        # this is a list with coordinates
        # used to crop from the original image;
        # these coordinates must be used as
        # DIAGONAL in order to crop or put back in place
        self.coords = coordinates


def generate_sections(x_dim, y_dim, cut_x, cut_y):
    """ sections from 0 to X and 0 to Y with step by step
        step is the cut size
    """
    yy = []
    for y in range(0, x_dim, cut_x):
        yy.append(y)
    yy.append(x_dim)

    xx = []
    for x in range(0, y_dim, cut_y):
        xx.append(x)
    xx.append(y_dim)

    # lists of int tuples
    return xx, yy


def generate_crop_coordinates(xx, yy):
    """ every combination of pair with
        the values from above function
    """
    coords = []
    for x in xx:
        rows = []
        for y in yy:
            rows.append((x, y))
        coords.append(rows)
    return coords


def generate_subimages(img, coords: list):
    subimages = []
    for i in range(len(coords) - 1):
        row0 = coords[i]
        row1 = coords[i + 1]

        for ii in range(len(row0) - 1):
            x_pair, y_pair = row0[ii], row1[ii + 1]

            cropped = img.crop((x_pair[1], x_pair[0], y_pair[1], y_pair[0]))
            cropped_coords = [
                (x_pair[1], x_pair[0]),
                (y_pair[1], y_pair[0])
            ]
            subimg = SubImage(cropped, cropped_coords)
            subimages.append(subimg)

    # array of PIL Images
    return subimages


def get_dimensions(subimages: list):
    """ we need this for reconstruction
        because we dont know the original img size
        we only have the array of subimages
    """
    max_X = 0
    max_Y = 0
    for subimage in subimages:
        for coords in subimage.coords:
            # coords is a tuple
            max_X = coords[0] if coords[0] > max_X else max_X
            max_Y = coords[1] if coords[1] > max_Y else max_Y

    # max x and y are the size of image
    return max_X, max_Y


def reconstruct_image(subimages: list, folder: str):
    y, x = get_dimensions(subimages)
    new_image = Image.new("RGBA", (y, x))

    for subimage in subimages:
        new_image.paste(subimage.img, subimage.coords[0])

    # saves locally
    new_image.save(os.path.join(folder, "reconstructed.png"))
    return new_image

def isIn(firstCorner=(0,0),secondCorner=(0,0),point=(0,0)):
   x1,y1=firstCorner[0],firstCorner[1]
   x2,y2=secondCorner[0],secondCorner[1]
   x,y=point[0],point[1]

   if (x >= x1 and x <= x2 and y >= y1 and y <= y2) :
       return True
   elif(x >= x2 and x <= x1 and y >= y2 and y <= y1):
       return True
   else:
       return False


if __name__ == '__main__':
    # this is what you provide as user
    cut_size = (768, 432, 3)

    # and you provide the image, ofc
    original_path = "/tmp/0.jpg"
    img = Image.open(original_path)
    img_size = img.size

    if cut_size[0] > img_size[0] or cut_size[1] > img_size[1]:
        raise ValueError("image size smaller than cut size.")

    xx, yy = generate_sections(*img_size, cut_size[0], cut_size[1])

    coords = generate_crop_coordinates(xx, yy)

    subimages = generate_subimages(img, coords)

    for index, subimage in enumerate(subimages, start=1):
        # -> if you want to save the pieces and see them
        subimage.img.save(f"{index}.png")
        # print(subimage.coords)

    folder = "."
    reconstruct_image(subimages, folder)