
import Rhino.Geometry as rg
import math

#get the points from an input mesh:
def GetParticles(mesh):
    # quadrangulate mesh
    #mesh.QuadRemesh(rg.QuadRemeshParameters())
    #triangulate mesh:
    mesh.Faces.ConvertQuadsToTriangles()
    
    #get vertices
    vertices = mesh.Vertices.ToPoint3dArray()
    points = []
    for i in range(len(vertices)):
        point = rg.Point3d( vertices[i].X, vertices[i].Y, vertices[i].Z)
        
        points.append(point) 

    return points

#establish a relationship to convert to springs:
def GetMeshVertices(mesh):

    indices = mesh.TopologyVertices
    indicesCount = len(indices)

    distances = []

    for i in range(1, indicesCount):
        d = indices[i-1].DistanceTo(indices[i])
        distances.append(d)

    
    points = []
    for i in range(indicesCount):
        point = rg.Point3d(indices[i].X, indices[i].Y, indices[i].Z,)
        points.append(point)
    

    return points

def GetVerticesDict(mesh):

    ivertex = mesh.TopologyVertices #get topology vertex indices from GetMeshVertices method 
    
    d = {} #empty dictionary for vertex indices
    dpoints = {} #empty dictionary for related points
    
    for i in range(ivertex.Count):
        connected_vertices_idx = ivertex.ConnectedTopologyVertices(i)
        connected_vertices = []
        
        
        for idx in range(len(connected_vertices_idx)):
            connected_vertices.append(mesh.Vertices[connected_vertices_idx[idx]])
            
            d[mesh.Vertices[i]] = connected_vertices
            dpoints[i] = connected_vertices_idx
        
        
        
        
    #for ivertex, connected_vertex in d.items():
    
        #vertices = []
    
        #for ivertex in connected_vertices:
            #vpoint = rg.Point3d(mesh.Vertices[ivertex].X, mesh.Vertices[ivertex].Y, mesh.Vertices[ivertex].Z)
            #vertices.append(vpoint)
        
    #dpoints[ivertex] = vertices
        
    return d
    

a = GetVerticesDict(iMesh)

#print a.keys()