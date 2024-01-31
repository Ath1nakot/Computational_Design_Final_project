from typing import Dict, List, NamedTuple

POINT_DISTANCES = [-1, 0, 1]


class Point3D(NamedTuple):
    x: int
    y: int
    z: int


class Mesh3D:
    def __init__(
        self,
        width: int,
        depth: int,
        height: int,
    ):
        self.width = width
        self.depth = depth
        self.height = height

    def get_adjacency_dict(self) -> Dict[Point3D, List[Point3D]]:
        adjacency_dict = {}
        point_list = self._get_point_list()

        for point in point_list:
            adjacent_point_list = self._get_adjacent_point_list(point)
            adjacency_dict[point] = adjacent_point_list

        return adjacency_dict

    def _get_point_list(self) -> List[Point3D]:
        point_list = []

        for x in range(self.width):
            for y in range(self.depth):
                for z in range(self.height):
                    point = Point3D(x=x, y=y, z=z)
                    point_list.append(point)

        return point_list

    def _get_adjacent_point_list(
        self,
        point: Point3D,
    ) -> List[Point3D]:
        return [
            Point3D(x=point.x + dx, y=point.y + dy, z=point.z + dz)
            for dx in POINT_DISTANCES
            for dy in POINT_DISTANCES
            for dz in POINT_DISTANCES
            if self._is_valid(point, dx, dy, dz)
        ]

    def _is_valid(
        self,
        point: Point3D,
        dx: int,
        dy: int,
        dz: int,
    ) -> bool:
        return (
            (0 <= point.x + dx <= self.width)
            and (0 <= point.y + dy <= self.depth)
            and (0 <= point.z + dz <= self.height)
        ) and not (dx == 0 and dy == 0 and dz == 0)
