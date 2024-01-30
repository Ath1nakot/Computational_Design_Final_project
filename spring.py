import rhino3dm as rg


class SystemBase:
    def __init__(self, spring_system, rope_system):
        self.spring_system = spring_system
        self.rope_system = rope_system

    def find_neighbors(self, agent, system, distance):
        neighbors = []
        for neighbor in system:
            i_distance = rg.Point3d.DistanceTo(agent, neighbor)
            if i_distance < distance:
                neighbors.append(neighbor)
        return neighbors


class SpringSystem:
    def __init__(self):
        self.springs = []

    def update(self):
        for spring in self.springs:
            spring.update()


class RopeSystem:
    def __init__(self):
        self.ropes = []

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
