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
    def prepare_mesh(self, mesh):
        # get topology vertex indices from GetMeshVertices method
        ivertex = mesh.TopologyVertices

        d = {}  # empty dictionary for vertex indices
        dpoints = {}  # empty dictionary for related points

        for i in range(ivertex.Count):
            connected_vertices_idx = ivertex.ConnectedTopologyVertices(i)
            connected_vertices = []
            for idx in range(len(connected_vertices_idx)):
                clo_pt = rg.Point3d(mesh.Vertices[connected_vertices_idx[idx]])
                connected_vertices.append(clo_pt)

                cen_pt = rg.Point3d(mesh.Vertices[i])
                d[cen_pt] = connected_vertices
                dpoints[i] = connected_vertices_idx
        print(cen_pt)
        self.pts_dict = d

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
        o_mesh = rg.Mesh()
        for spring in self.springs:
            cen_pt = spring.show()
            o_mesh.Vertices.Add(cen_pt)

        return o_mesh


# ================================================================
# Spring
# ================================================================
class Spring:
    # ##########Initialize a spring########## #
    def __init__(self, cen_pt, clo_pts, inflate_vector):
        self.cen_pt = cen_pt
        self.clo_pts = clo_pts
        self.start_velocity = inflate_vector * 0.75
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

        # Calculate start velocity
        self.start_velocity *= 0.95

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