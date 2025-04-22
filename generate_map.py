import osmnx as ox
import json
import numpy as np
import rasterio.features
from affine import Affine
import matplotlib.pyplot as plt
from pyproj import Transformer

# Configuration
center_point = (40.444165, -79.942852) # Walking to the Sky
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
bike_racks_latlon = [[40.444132, -79.941917], # UC Gym
                    [40.442845, -79.942399], # Merson Courtyard
                    [40.442105, -79.938684], # Maggie Mo
                    [40.442489, -79.945821], # Wean
                    [40.441666, -79.947262], # Scaife
                    [40.441341, -79.943816], # Hunt
                    [40.444083, -79.944599], # Gates
                    [40.442431, -79.943734]] # Doherty


pgh_to_world = Transformer.from_crs("EPSG:" + str(pgh), "EPSG:" + str(world), always_xy=True)
world_to_pgh = Transformer.from_crs("EPSG:" + str(world), "EPSG:" + str(pgh), always_xy=True)

walking = ox.features_from_point(center_point, walking_tags, radius_meters)
walking_utm = walking.to_crs(epsg=pgh)

biking = ox.features_from_point(center_point, biking_tags, radius_meters)
biking_utm = biking.to_crs(epsg=pgh)

minx, miny, maxx, maxy = walking_utm.total_bounds
width = int((maxx - minx) / cell_size)
height = int((maxy - miny) / cell_size)
transform = Affine(cell_size, 0, minx, 0, -cell_size, maxy)
topleft_lon, topleft_lat = pgh_to_world.transform(minx, maxy)
bottomright_lon, bottomright_lat = pgh_to_world.transform(maxx, miny)

bike_racks_rowcol = []
for lat, lon in bike_racks_latlon:
    x, y = world_to_pgh.transform(lon, lat)
    col, row = ~transform * (x, y)
    col, row = int(col), int(row)
    if 0 <= row < height and 0 <= col < width:
        bike_racks_rowcol.append((row, col))


walking_shapes = ((geom, 0) for geom in walking_utm.geometry if geom is not None)
walking_grid = rasterio.features.rasterize(
    shapes=walking_shapes,
    out_shape=(height, width),
    transform=transform,
    fill=255,
    dtype=np.uint8
)

biking_shapes = ((geom, 0) for geom in biking_utm.geometry if geom is not None)
biking_grid = rasterio.features.rasterize(
    shapes=biking_shapes,
    out_shape=(height, width),
    transform=transform,
    fill=255,
    dtype=np.uint8
)


plt.imshow(biking_grid, cmap="gray", origin="upper")
plt.axis("off")
plt.show()

export = {
    "topleft_latlon": (topleft_lat, topleft_lon),
    "bottomright_latlon": (bottomright_lat, bottomright_lon),
    "width": width,
    "height": height,
    "bike_racks": bike_racks_rowcol,
    "walking": walking_grid.tolist(),
    "biking": biking_grid.tolist()
}

with open("map.json", "w") as file:
    file.write(json.dumps(export))