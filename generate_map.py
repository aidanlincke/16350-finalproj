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

tags = {
    "footway": True,
    "highway": ["footway", "path"],
}

sidewalks = ox.features_from_point(center_point, tags, radius_meters)
sidewalks_utm = sidewalks.to_crs(epsg=pgh)
minx, miny, maxx, maxy = sidewalks_utm.total_bounds
width = int((maxx - minx) / cell_size)
height = int((maxy - miny) / cell_size)
transform = Affine(cell_size, 0, minx, 0, -cell_size, maxy)

transformer = Transformer.from_crs("EPSG:" + pgh, "EPSG:" + world, always_xy=True)
topleft_lon, topleft_lat = transformer.transform(minx, maxy)
bottomright_lon, bottomright_lat = transformer.transform(maxx, miny)

shapes = ((geom, 1) for geom in sidewalks_utm.geometry if geom is not None)
grid = rasterio.features.rasterize(
    shapes=shapes,
    out_shape=(height, width),
    transform=transform,
    fill=0,
    dtype=np.uint8
)


plt.imshow(grid, cmap="gray", origin="upper")
plt.axis("off")
plt.show()

export = {
    "bounds": ((topleft_lon, topleft_lat), (bottomright_lon, bottomright_lat)),
    "shape": (height, width),
    "data": grid.tolist(),
}

with open("map.json", "w") as file:
    file.write(json.dumps(export))