import osmnx as ox
import json
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
    "foot": True,
    "highway": [
        "footway",
        "cycleway",
        "path",
        "residential",
        "tertiary",
        "living_street"
    ],
    "bicycle": True,
    "cycleway": True
}
bike_racks_latlon = [[40.444231214978565, -79.94195625185966], # UC Gym
                    [40.442877844449434, -79.94237735867502], # Merson Courtyard
                    [40.442124599275054, -79.93868261575699], # Maggie Mo
                    [40.44250530554684, -79.94570598006248], # Wean
                    [40.441729601480795, -79.94718521833421], # Scaife
                    [40.44130806380692, -79.94378283619882], # Hunt
                    [40.44405668682533, -79.94456604123117], # Gates
                    [40.442404, -79.943669], # Doherty
                    [40.44726037997291, -79.94610026478767]] # 5th and Clyde


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

walking_shapes = ((geom, 0) for geom in walking_utm.geometry if geom is not None)
walking_grid = rasterio.features.rasterize(
    shapes=walking_shapes,
    out_shape=(height, width),
    transform=transform,
    fill=1
)

biking_shapes = ((geom, 0) for geom in biking_utm.geometry if geom is not None)
biking_grid = rasterio.features.rasterize(
    shapes=biking_shapes,
    out_shape=(height, width),
    transform=transform,
    fill=1
)

bike_racks_rowcol = []
for lat, lon in bike_racks_latlon:
    x, y = world_to_pgh.transform(lon, lat)
    col, row = ~transform * (x, y)
    col, row = int(col), int(row)
    if 0 <= row < height and 0 <= col < width and walking_grid[row][col] == 0 and biking_grid[row][col] == 0:
        bike_racks_rowcol.append((row, col))
    else:
        print(f"Warning: The bike rack at {lat, lon} was invalid!")

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