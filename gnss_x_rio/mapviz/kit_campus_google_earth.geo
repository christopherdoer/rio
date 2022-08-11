image_path: "kit_campus_google_earth"        # The relative path to the map tiles
image_width: 2224     # The full pixel width of the map
image_height: 1316    # The full pixel height of the map
tile_size: 2224         # The pixel size of the individual tiles

datum: "2022"         # Datum is currently ignored
projection: "utm"      # (utm|wgs84)

                       # At least 2 tie points are required for 
                       # scale, and 3 for orientation.
tiepoints:             #   [pixel x, pixel y, geo x, geo y]
 - point: [631, 272, 457003.744, 5429071.476]
 - point: [580, 1078, 456987.295, 5428845.481]
 - point: [1616, 1094, 457279.252, 5428838.779]
 - point: [1794, 228, 457331.139, 5429081.512]

# 321 67 49.013036°N, 8.412003°E  457003.744 5429071.476
# 224 1200 49.011002°N, 8.411802°E   456987.295 5428845.481
# 1707 1224  49.010962°N, 8.415795°E 457279.252 5428838.779
# 1937, 7 49.013149°N,   8.416479°E  457331.139 5429081.512

## How To:
## Export satelite image with GoogleEarth
## Select 3 distinct points
## Get image coordinates
## Get LLH coordinates with GoogleEarth and convert to UTM e.g. using: http://www.movable-type.co.uk/scripts/latlong-utm-mgrs.html
 
