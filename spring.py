import rhino3dm as rg


# ================================================================
# Basic System
# ================================================================
class BaseSystem:
    def __init__(self):
        pass

    # ##########Calculate distance between given agent and all items in the given system########## #
    def find_neighbors(self, agent, system, distance):
        neighbors = []
        for neighbor in system:
            i_distance = rg.Point3d.DistanceTo(agent, neighbor)
            if i_distance < distance:
                neighbors.append(neighbor)
        return neighbors


# ================================================================
# Spring System
# ================================================================
class SpringSystem(BaseSystem):
    def __init__(self):
        self.springs = []
        self.cen_pts = []
        self.pts_dict = {}
        self.steers = []

    # ##########INPUT: mesh; OUTPUT: dictionary, key as central point, value as all surrounding points########## #
    def prepare_mesh(self, imesh, size):
        imesh.Faces.ConvertQuadsToTriangles()

        # get vertices
        vertices = imesh.Vertices.ToPoint3dArray()
        points = []
        for i in range(len(vertices)):
            point = rg.Point3d(vertices[i].X, vertices[i].Y, vertices[i].Z)
            points.append(point)

        # calculate the surrounding points
        self.pts_dict = {}
        for i in points:
            distances = []
            for j in points:
                if i != j:
                    distance = j.DistanceTo(i)
                    distances.append((j, distance))
            distances.sort(key=lambda x: x[1])
            closest_points = []

            for d in distances[:size]:
                closest_points.append(d[0])
            self.pts_dict[i] = closest_points

    # ##########INPUT: Dictionary; OUTPUT: springs list########## #
    def convert_spring(self, force_list):
        self.cen_pts = self.pts_dict.keys()
        for i in range(len(self.cen_pts)):
            spring = Spring(self.cen_pts[i], self.pts_dict[self.cen_pts[i]], force_list[i])
            self.springs.append(spring)

    # ##########Add force to each spring########## #
    def add_force(self):
        for spring in self.springs:
            steer = spring.add_force()
            self.steers.append(steer)

    # ##########INPUT: spring; OUTPUT: spring########## #
    def update(self):
        for i in range(len(self.springs)):
            self.springs[i].update(self.steers[i])

    # ##########Return mesh based on all the central points########## #
    def show(self):
        o_pts = []
        for spring in self.springs:
            cen_pt = spring.show()
            o_pts.append(cen_pt)
        return o_pts


# ================================================================
# Spring
# ================================================================
class Spring:
    # ##########Initialize a spring########## #
    def __init__(self, cen_pt, clo_pts, inflate_vector):
        self.cen_pt = cen_pt
        self.clo_pts = clo_pts
        self.start_velocity = inflate_vector
        self.o_lens = []

        for i in range(len(self.clo_pts)):
            self.o_lens.append(self.cen_pt.DistanceTo(self.clo_pts[i]))

    # ##########Add all the force to cen_pts########## #
    def add_force(self):
        # Calculate spring force
        sprint_force = rg.Vector3d(0,0,0)
        for i in range(len(self.clo_pts)):
            n_length = self.cen_pt.DistanceTo(self.clo_pts[i])
            delta = n_length - self.o_lens[i]
            dir = rg.Vector3d(self.clo_pts[i]) - rg.Vector3d(self.cen_pt)
            dir.Unitize()

            if delta > 0:
                spring_single = dir * 0.02 * delta
            else:
                spring_single = dir * 0.02 * delta

            sprint_force += spring_single

        # Calculate rope force
        rope_force = rg.Vector3d(0,0,0)

        #o_X = sprint_force.X + rope_force.X + self.start_velocity.X
        #o_Y = sprint_force.X + rope_force.Y + self.start_velocity.Y
        #o_Z = sprint_force.X + rope_force.Z + self.start_velocity.Z
        #steer = rg.Vector3d(o_X, o_Y, o_Z)
        steer = sprint_force + rope_force + self.start_velocity
        return steer

    # ##########Update the velocity of each cen_pts########## #
    def update(self, steer):
        self.cen_pt += self.start_velocity

    # ##########Show the cen_pts########## #
    def show(self):
        return self.cen_pt


# ================================================================
# Rope System   -------------------------------    TO BE CONTINUE
# ================================================================
class RopeSystem(BaseSystem):
    def __init__(self):
        self.ropes = []

    def update(self):
        for rope in self.ropes:
            rope.update()


# ================================================================
# Rope   ---------------------------------------    TO BE CONTINUE
# ================================================================
class Rope:
    def __init__(self):
        pass

    def update(self):
        pass


# ================================================================
# MAIN   ----------------------------------------------    TO TEST
# ================================================================
imesh = rg.Mesh()
force_list = []
iReset = True

if iReset:
    my_spring_system = SpringSystem()
    my_spring_system.prepare_mesh(imesh)
    my_spring_system.convert_spring(force_list)
    my_spring_system.add_force()
else:
    my_spring_system.update()
    oGeo = my_spring_system.show()