import A_star_logic
import airsim
import airsimneurips
import numpy as np


def get_path(curPos, gatePos, subDiv):
    """
    Returns an array of tuples, each with 3 elements

    Arguments:
    curPos - drone's current position (Airsim pose object)
    P1 - next gate postion (numpy.ndarray with 3 position entries)
    V1 - drone's current velocity (numpy.ndarray with 3 velocity entries)
    vel_bias - effect of inital velocity (float)
    spacing_const - increasing this number will increase how many points are in each path (float) // generally keep this number pretty low

    """
    minX_at_drone = 0
    minY_at_drone = 0
    minZ_at_drone = 0

    minX = min(gatePos.x_val, curPos.x_val)
    minY = min(gatePos.y_val, curPos.y_val)
    minZ = min(gatePos.z_val, curPos.z_val)

    if (min(gatePos.x_val, curPos.x_val) == curPos.x_val):
        minX_at_drone = 1
    if (min(gatePos.y_val, curPos.y_val) == curPos.y_val):
        minY_at_drone = 1
    if (min(gatePos.z_val, curPos.z_val) == curPos.z_val):
        minZ_at_drone = 1

    maxX = max(gatePos.x_val, curPos.x_val)
    maxY = max(gatePos.y_val, curPos.y_val)
    maxZ = max(gatePos.z_val, curPos.z_val)

    # make grid with subdivision size of 0.25
    gridX = int((maxX-minX)//subDiv)
    gridY = int((maxY-minY)//subDiv)
    gridZ = int((maxZ-minZ)//subDiv)

    # really scuffed fix to grid generation issue
    if gridX == 0:
        gridX = 1
    if gridY == 0:
        gridY = 1
    if gridZ == 0:
        gridZ = 1

    a_star_grid = np.zeros((gridX, gridY, gridZ), dtype=int)

    # first run A_star on simplified grid, then get actual positions based on generate points
    # here is my scuffed way of getting start and end positions
    start = (0 if minX_at_drone else gridX-1, 0 if minY_at_drone else gridY-1, 0 if minZ_at_drone else gridZ-1)
    end = (0 if not minX_at_drone else gridX-1, 0 if not minY_at_drone else gridY-1, 0 if not minZ_at_drone else gridZ-1)
    # print(start, end)
    # print()

    a_star_path = A_star_logic.a_star_3d(a_star_grid, start, end)
    # print(a_star_path)

    actual_path = []

    # in case a path isn't generated, so the run doesn't end
    if a_star_path == None:
        return None

    # convert back to airsim gridspace
    for i in a_star_path:
        actual_path.append((minX+i[0]*subDiv, minY+i[1]*subDiv, minZ+i[2]*subDiv))
    # print(actual_path)

    return actual_path