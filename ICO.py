import open3d as o3d
import numpy as np
import math
import time
import scipy
from scipy.integrate import solve_ivp
import copy

N = 12
class params:
  g = 9.8
  R = 8.31 * 10.0
  T = 273.0
  P = 1
  k = 0.01
  mass = 0.1

def count_P_V_S(mesh):
  V = mesh.get_volume()
  P = params.R * params.T / V
  S = []
  vertices = np.asarray(mesh.vertices)
  triangles = np.asarray(mesh.triangles)
  for triangle in triangles:
    surf = np.linalg.norm(np.cross(
      vertices[triangle[0]] - vertices[triangle[1]], 
      vertices[triangle[1]] - vertices[triangle[2]]))
    S.append(surf)
  S = np.asarray(S)
  return P, V, S

def Spring_ICO(mesh):
  vertices =  np.asarray(mesh.vertices)
  triangles = np.asarray(mesh.triangles)
  Fm = np.zeros((N, 3))
  for triangle in triangles:
    edge_0 = vertices[triangle[0]] - vertices[triangle[1]]
    edge_1 = vertices[triangle[1]] - vertices[triangle[2]]
    edge_2 = vertices[triangle[2]] - vertices[triangle[0]]
    Fm[triangle[0], :] += params.k * edge_2 - params.k * edge_0
    Fm[triangle[1], :] += params.k * edge_0 - params.k * edge_1
    Fm[triangle[2], :] += params.k * edge_1 - params.k * edge_2
  Fm = Fm / (2 * params.mass)
  return Fm

def PV_ICO(mesh):
  vertices =  np.asarray(mesh.vertices)
  triangles = np.asarray(mesh.triangles)
  Fm = np.zeros((N, 3))
  P, V, S = count_P_V_S(mesh)
  dp = P - params.P
  normals = np.zeros((np.shape(triangles)[0], 3))
  for i in range(np.shape(triangles)[0]):
    normals[i] = np.cross(
      vertices[triangles[i, 0]] - vertices[triangles[i, 1]],
      vertices[triangles[i, 1]] - vertices[triangles[i, 2]])
    normals[i] = normals[i] / np.linalg.norm(normals[i])
  center = np.asarray(mesh.get_center())
  for i in range(np.shape(normals)[0]):
    if (np.dot(normals[i], triangles[i, 0] - center) < 0):
      normals[i] = -normals[i]
  for i in range(np.shape(triangles)[0]):
    Fm[triangles[i, 0], :] += S[i] * normals[i] * dp / (3 * params.mass)
    Fm[triangles[i, 1], :] += S[i] * normals[i] * dp / (3 * params.mass)
    Fm[triangles[i, 2], :] += S[i] * normals[i] * dp / (3 * params.mass)
  
  return Fm



def right_side(t, vect_in):
  # vect_in = np.reshape(vect_in, (6, int(vect_in.size / 6)))
  Vel = (vect_in[N * 3 :]) #r_dot
  # mesh2 = copy.deepcopy(mesh)
  Acc = Spring_ICO(mesh) + PV_ICO(mesh)
  # Acc = Spring_ICO(mesh)
  # print(np.shape(Spring_ICO(mesh2)))
  Acc = np.reshape(Acc, (3 * N,)) #v_dot
  return np.concatenate((Vel, Acc), axis = 0)


R = 1.0

mesh = o3d.geometry.TriangleMesh.create_icosahedron(radius=0.5)
mesh.compute_vertex_normals()

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh)
vis.update_geometry(mesh)
vis.poll_events()
tau = 6.5
dt = 0.01
t = np.linspace(0,tau,int(tau/dt))
Vels = np.zeros((3*N))
for i in range(t.size - 1):
  vertices =  np.asarray(mesh.vertices)
  # print((vertices.size))
  triangles = np.asarray(mesh.triangles)
  print(vertices)
  print((np.reshape(vertices, (N * 3,))))
  break
  y_0 = np.concatenate((np.reshape(vertices, (N * 3,)), Vels), axis = 0)
  # print(mesh.is_watertight())
  # print(np.asarray(mesh.vertices))
  # print(count_P_V_S(mesh))
  try:
    sol = scipy.integrate.solve_ivp(right_side, (t[i], t[i + 1]), y_0, first_step = dt)
  except:
    pass
  Vels = sol.y[N * 3 :, 1]
  # print((sol))
  vertices = np.reshape(sol.y[:N * 3, 1], (N, 3))
  mesh.vertices = o3d.utility.Vector3dVector(vertices)
  print(mesh.is_watertight())
  # time.sleep(20.1)
  vis.update_geometry(mesh)
  vis.poll_events()
  vis.update_renderer()
  

  # print(count_P_V_S(mesh))
  # time.sleep(20.1)


o3d.visualization.draw_geometries([mesh])
vertices =  np.asarray(mesh.vertices)
  # print((vertices.size))
triangles = np.asarray(mesh.triangles)

normals = np.zeros((np.shape(triangles)[0], 3))
for i in range(np.shape(triangles)[0]):
  normals[i] = np.cross(
    vertices[triangles[i, 0]] - vertices[triangles[i, 1]],
    vertices[triangles[i, 1]] - vertices[triangles[i, 2]])
  normals[i] = normals[i] / np.linalg.norm(normals[i])
center = np.asarray(mesh.get_center())
for i in range(np.shape(normals)[0]):
  if (np.dot(normals[i], triangles[i, 0] - center) < 0):
    normals[i] = -normals[i]