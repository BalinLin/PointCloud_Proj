from laspy.file import File
import numpy as np
from pypcd import pypcd
import twd97

inFile = File("./WGS84.las", mode='r')

# pointformat = inFile.point_format
# for spec in inFile.point_format:
#     print(spec.name)
# print("==========")

print(len(inFile.points))
# print(inFile.points[0])
# print(inFile.X[0])
# print(inFile.Y[0])
# print(inFile.Z[0])
# print(inFile.intensity[0])
# print(inFile.flag_byte[0])
# print(inFile.raw_classification[0])
# print(inFile.scan_angle_rank[0])
# print(inFile.user_data[0])
# print(inFile.pt_src_id[0])
# print(inFile.gps_time[0])
# print(inFile.red[0])
# print(inFile.green[0])
# print(inFile.blue[0])
# print(inFile.get_edge_flight_line()[0])
# print("==========")

# headerformat = inFile.header.header_format
# for spec in headerformat:
#     print(spec.name)
print(inFile.header.offset)
print(inFile.header.scale)
print("==========")


print(inFile.header.min)

longitude = inFile.X[0] * inFile.header.scale[0] + inFile.header.offset[0]
latitude  = inFile.Y[0] * inFile.header.scale[1] + inFile.header.offset[1]
waterLevel= inFile.Z[0] * inFile.header.scale[2] + inFile.header.offset[2]
print("Origin(X, Y, Z):", longitude, latitude, waterLevel)
# input(N, E) output(X, Y)
print("TW97(X, Y, Z):", twd97.fromwgs84(latitude, longitude), waterLevel)
# input(X, Y) output(N, E)
print("WGS84(N, E, Z):", twd97.towgs84(longitude, latitude), waterLevel)

# v1 (270804.94158672367, 2746110.115385094, 301.93) to (270800, 2746100, 300)
# v2 (304432.11792123807, 2767367.268923381, 32.4) to (304430, 2767360, 30)
