import osmnx as ox
import json
import numpy as np
import rasterio.features
from affine import Affine
import matplotlib.pyplot as plt
from pyproj import Transformer

center_point = (40.4448, -79.9421)
radius_meters = 1000
cell_size = 0.5

pgh = 26917
world = 4326

walking_tags = {
    "foot": True,
    "highway": ["footway", "path"],
}

biking_tags = {
    "highway": [
        "cycleway",
        "path",
        "residential",
        "tertiary",
        "living_street"
    ],
    "bicycle": True,
    "cycleway": True
}

walking = ox.features_from_point(center_point, walking_tags, radius_meters)
walking_utm = walking.to_crs(epsg=pgh)

biking = ox.features_from_point(center_point, biking_tags, radius_meters)
biking_utm = biking.to_crs(epsg=pgh)

minx, miny, maxx, maxy = walking_utm.total_bounds
width = int((maxx - minx) / cell_size)
height = int((maxy - miny) / cell_size)
transform = Affine(cell_size, 0, minx, 0, -cell_size, maxy)

transformer = Transformer.from_crs("EPSG:" + str(pgh), "EPSG:" + str(world), always_xy=True)
topleft_lon, topleft_lat = transformer.transform(minx, maxy)
bottomright_lon, bottomright_lat = transformer.transform(maxx, miny)

walking_shapes = ((geom, 0) for geom in walking_utm.geometry if geom is not None)
walking_grid = rasterio.features.rasterize(
    shapes=walking_shapes,
    out_shape=(height, width),
    transform=transform,
    fill=5,
    dtype=np.uint8
)

biking_shapes = ((geom, 0) for geom in biking_utm.geometry if geom is not None)
biking_grid = rasterio.features.rasterize(
    shapes=biking_shapes,
    out_shape=(height, width),
    transform=transform,
    fill=5,
    dtype=np.uint8
)


plt.imshow(biking_grid, cmap="gray", origin="upper")
plt.axis("off")
plt.show()

export = {
    "bounds": ((topleft_lon, topleft_lat), (bottomright_lon, bottomright_lat)),
    "shape": (height, width),
    "walking": walking_grid.tolist(),
    "biking": biking_grid.tolist()
}

with open("map.json", "w") as file:
    file.write(json.dumps(export))