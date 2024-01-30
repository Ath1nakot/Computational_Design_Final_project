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
    def __init__(self,spring_system):
        self.springs = []
        for spring in spring_system:
            spring =

    def update(self):
        for spring in self.springs:
            spring.update()


class RopeSystem(BaseSystem):
    def __init__(self):
        self.ropes = []

    def update(self):
        for rope in self.ropes:
            rope.update()

    def find_neighbors(self, agent, system, distance):
        neighbors = []
        for neighbor in system:
            i_distance = rg.Point3d.DistanceTo(agent, neighbor)
            if i_distance < distance:
                neighbors.append(neighbor)
        return neighbors


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
