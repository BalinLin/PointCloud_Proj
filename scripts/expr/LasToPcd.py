from laspy.file import File
import numpy as np
from pypcd import pypcd
import twd97
import time

np.set_printoptions(suppress=True)

# inFile = File("/media/yoyo/harddisk/project/ncsist/WGS84.las", mode='r')
inFile = File("/media/yoyo/harddisk/project/v2/NCSIST/WGS84.las", mode='r')

# v1 (270804.94158672367, 2746110.115385094, 301.93) to (270800, 2746100, 300)
# v2 (304432.11792123807, 2767367.268923381, 32.4) to (304430, 2767360, 30)
# map_offset = [270800, 2746100, 300]
map_offset = [304430, 2767360, 30]

las_offset = inFile.header.offset
las_scale = inFile.header.scale
pc_len = len(inFile.points)
laspc = np.ndarray([pc_len, 4], dtype=np.float32)

t1 = time.time()

for idx in range(pc_len):
    longitude = inFile.X[idx] * las_scale[0] + las_offset[0]
    latitude  = inFile.Y[idx] * las_scale[1] + las_offset[1]
    waterLevel= inFile.Z[idx] * las_scale[2] + las_offset[2]
    twd97Pos = twd97.fromwgs84(latitude, longitude)

    laspc[idx, 0] = twd97Pos[0] - map_offset[0]
    laspc[idx, 1] = twd97Pos[1] - map_offset[1]
    laspc[idx, 2] = waterLevel  - map_offset[2]
    laspc[idx, 3] = inFile.intensity[idx]

    if idx % 100000 == 0:
        print("{0:8d}/{1:} time:{2}".format(idx, pc_len, time.time() - t1))

md = { 'version': .7,
       'fields': ['x', 'y', 'z', 'intensity'],
        'size': [4, 4, 4, 4],
        'count': [1, 1, 1, 1],
        'width': 0,
        'height': 1,
        'viewpoint': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        'points': 0,
        'type': ["F", "F", "F", "F"],
        'data': 'ascii'}

md['width'] = pc_len
md['points'] = pc_len

pc = pypcd.PointCloud(md, laspc)
pypcd.save_point_cloud_bin(pc, "./twAll_bin.pcd")
# pypcd.save_point_cloud(pc, "./twAll_ascii.pcd")
