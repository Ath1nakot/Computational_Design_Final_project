import rhino3dm as rg

class BaseSystem:
    def __init__(self):
        pass

    def find_neighbors(self, agent, system, distance):
        neighbors = []
        for neighbor in system:
            i_distance = rg.Point3d.DistanceTo(agent, neighbor)
            if i_distance < distance:
                neighbors.append(neighbor)
        return neighbors


class SpringSystem(BaseSystem):
    def __init__(self, spring_system):
        self.springs = []
        for i_spring in spring_system:
            spring = Spring(i_spring)
            self.springs.append(spring)

    def update(self):
        for spring in self.springs:
            spring.update()


class RopeSystem(BaseSystem):
    def __init__(self, rope_system):
        self.ropes = []
        for i_rope in rope_system:
            rope = Spring(i_rope)
            self.ropes.append(rope)

    def update(self):
        for rope in self.ropes:
            rope.update()


class Spring:
    def __init__(self):
        pass

    def update(self):
        pass


class Rope:
    def __init__(self):
        pass

    def update(self):
        pass
