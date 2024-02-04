import open3d as o3d
import numpy as np
import math
import time
import scipy
from scipy.integrate import solve_ivp
import copy
import matplotlib
import matplotlib.pyplot as plt

N = 12
class params:
    g = 9.8
    R = 8.31 * 10.0
    T = 273.0
    #   P = 1 * 1e5
    P = 10398.474625227915
    k = 1.0
    d = 0.1
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
        S.append(surf / 2)
    S = np.asarray(S)
    return P, V, S


def Spring_ICO(mesh):
    vertices = np.asarray(mesh.vertices)
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


def Damping_ICO(mesh, Vel):
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    Fm = np.zeros((N, 3))
    for triangle in triangles:
        edge_0 = vertices[triangle[0]] - vertices[triangle[1]]
        edge_1 = vertices[triangle[1]] - vertices[triangle[2]]
        edge_2 = vertices[triangle[2]] - vertices[triangle[0]]
        v_0 = Vel[triangle[0]]
        v_1 = Vel[triangle[1]]
        v_2 = Vel[triangle[2]]
        # if(np.linalg.norm(v_0) != 0 and np.linalg.norm(edge_0) != 0 and np.linalg.norm(edge_2)):
        #   Fm[triangle[0], :] += params.d * v_0 * np.dot(v_0, edge_0)/(
        #         np.linalg.norm(v_0) * np.linalg.norm(edge_0)) - params.d * v_0 * np.dot(v_0, edge_2)/(
        #         np.linalg.norm(v_0) * np.linalg.norm(edge_2))
        # if(np.linalg.norm(v_1) != 0 and np.linalg.norm(edge_1)!=0 and np.linalg.norm(edge_0) != 0):
        #   Fm[triangle[1], :] += params.d * v_0 * np.dot(v_1, edge_1)/(
        #         np.linalg.norm(v_1) * np.linalg.norm(edge_1)) - params.d * v_1 * np.dot(v_1, edge_0)/(
        #         np.linalg.norm(v_1) * np.linalg.norm(edge_0))
        # if(np.linalg.norm(v_2) != 0 and np.linalg.norm(edge_2)!= 0 and np.linalg.norm(edge_1)!=0):
        #   Fm[triangle[0], :] += params.d * v_2 * np.dot(v_2, edge_2)/(
        #         np.linalg.norm(v_2) * np.linalg.norm(edge_2)) - params.d * v_2 * np.dot(v_2, edge_1)/(
        #         np.linalg.norm(v_2) * np.linalg.norm(edge_1))
        Fm[triangle[0], :] += -params.d * (v_0 - v_1) * edge_0 / np.linalg.norm(edge_0) + params.d * (
                    v_0 - v_2) * edge_2 / np.linalg.norm(edge_2)
        Fm[triangle[1], :] += params.d * (v_1 - v_0) * edge_0 / np.linalg.norm(edge_0) - params.d * (
                    v_1 - v_2) * edge_1 / np.linalg.norm(edge_1)
        Fm[triangle[2], :] += -params.d * (v_2 - v_0) * edge_2 / np.linalg.norm(edge_2) + params.d * (
                    v_2 - v_1) * edge_1 / np.linalg.norm(edge_1)
    Fm = Fm / (2 * params.mass)
    return Fm


def PV_ICO(mesh):
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    Fm = np.zeros((N, 3))
    P, V, S = count_P_V_S(mesh)
    dp = P
    normals = np.zeros((np.shape(triangles)[0], 3))
    centers = np.zeros((np.shape(triangles)))
    for i in range(np.shape(triangles)[0]):
        normals[i] = np.cross(
            vertices[triangles[i, 0]] - vertices[triangles[i, 1]],
            vertices[triangles[i, 1]] - vertices[triangles[i, 2]])
        normals[i] = normals[i] / np.linalg.norm(normals[i])
        centers[i] = (vertices[triangles[i, 0]] + vertices[triangles[i, 1]] + vertices[triangles[i, 2]]) / 3
    center = np.asarray(mesh.get_center())
    #   print(center)
    for i in range(np.shape(normals)[0]):
        if (np.dot(normals[i], centers[i] - center) < 0):
            normals[i] = -normals[i]
        #   print("reversed", i, "normal")
    for i in range(np.shape(triangles)[0]):
        Fm[triangles[i, 0], :] += S[i] * normals[i] * dp / (3 * params.mass)
        Fm[triangles[i, 1], :] += S[i] * normals[i] * dp / (3 * params.mass)
        Fm[triangles[i, 2], :] += S[i] * normals[i] * dp / (3 * params.mass)

    return Fm


def right_side(t, vect_in):
    # vect_in = np.reshape(vect_in, (6, int(vect_in.size / 6)))
    Vel = (vect_in[:, 3:])  # r_dot
    #   print(np.shape(Vel))
    # mesh2 = copy.deepcopy(mesh)
    Acc = np.zeros((N, 3))  # + Spring_ICO(mesh) + PV_ICO(mesh) + Damping_ICO(mesh, Vel)
    # Acc = Spring_ICO(mesh)
    # print(np.shape(Spring_ICO(mesh2)))
    #   Acc = np.reshape(Acc, (3 * N,)) #v_dot
    return np.concatenate((Vel, Acc), axis=1)


def RK4(t, right_side, vect_in):
    k_1 = right_side(t, vect_in)
    # print(np.shape(k_1))
    k_2 = right_side(t + dt / 2, vect_in + dt / 2 * k_1)
    # print(np.shape(k_2))
    k_3 = right_side(t + dt / 2, vect_in + dt / 2 * k_2)
    # print(np.shape(k_3))
    k_4 = right_side(t + dt, vect_in + dt * k_3)
    # print(np.shape(k_4))

    return vect_in + dt / 6 * (k_1 + 2 * k_2 + 2 * k_3 + k_4)


R = 1.0

mesh = o3d.geometry.TriangleMesh.create_icosahedron(radius=0.5)
mesh.compute_vertex_normals()

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh)
vis.update_geometry(mesh)
vis.poll_events()
tau = 100.5
dt = 0.01
t = np.linspace(0, tau, int(tau / dt))
Vels = np.zeros((N, 3))
Vert_modules = np.zeros((N, t.size))

for i in range(t.size - 1):
    vertices = np.asarray(mesh.vertices)
    P, V, S = count_P_V_S(mesh)
    dp = P - params.P
    #   print(dp, P, params.P, V)
    # print((vertices.size))
    triangles = np.asarray(mesh.triangles)
    vector_in = np.concatenate((vertices, Vels), axis=1)
    vector_out = RK4(t, right_side, vector_in)
    vertices = vector_out[:, :3]
    Vels = vector_out[:, 3:]
    for j in range(N):
        Vert_modules[j, i] = np.linalg.norm(vertices[j, :])
    #   print(np.shape(vector_in))
    #   print(Vert_modules[:,i])
    #   print((np.reshape(vertices, (N * 3,))))
    #   break
    #   y_0 = np.concatenate((np.reshape(vertices, (N * 3,)), Vels), axis = 0)
    # print(mesh.is_watertight())
    # print(np.asarray(mesh.vertices))
    # print(count_P_V_S(mesh))
    #   try:
    #     sol = scipy.integrate.solve_ivp(right_side, (t[i], t[i + 1]), y_0, first_step = dt)
    #   except:
    #     pass
    #   Vels = sol.y[N * 3 :, 1]
    # print((sol))
    #   vertices = np.reshape(sol.y[:N * 3, 1], (N, 3))
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    #   print(mesh.is_watertight())
    if (mesh.is_watertight() == False):
        print(t[i])
        # print(vertices)
        # Vert_modules = np.asarray(Vert_modules)
        # stop = 0
        for j in range(i + 1):
            for k in range(N):
                # print(np.isnan(Vert_modules[0, j]))
                if (np.isnan(Vert_modules[k, j])):
                    stop = j
                    print(stop)
                    break
        # print(stop)
        print("not Watertight")
        # print(Vert_modules[0, stop - 5: stop + 1])
        # plt.plot(t[:stop + 1], Vert_modules[0, :stop + 1])
        # plt.show()
        break
    # print(Vert_modules[0, :])
    # plt.plot(t[:], Vert_modules[0, :])
    # plt.show()
    #   time.sleep(2.1)
    vis.update_geometry(mesh)
    vis.poll_events()
    vis.update_renderer()

    # print(count_P_V_S(mesh))
    # time.sleep(20.1)
print(Vert_modules[0, :])
plt.plot(t[:], Vert_modules[0, :])
plt.show()

o3d.visualization.draw_geometries([mesh])
vertices = np.asarray(mesh.vertices)
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
