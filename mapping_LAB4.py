#!/usr/bin/env python3

"""
    # {Erik Molitor}
    # {9611043879}
    # {emolitor@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

        

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()
        """
        Fill in your solution here
        """

        #inital values 
        angle_minOwn = scan.angle_min
        angle_max = scan.angle_max
        angle_incrementOwn = scan.angle_increment
        range_minOwn = scan.range_min
        range_maxOwn = scan.range_max
        rangesOwn = scan.ranges
        x_turtle = pose.pose.position.x
        y_turtle = pose.pose.position.y
        angle = angle_minOwn - angle_incrementOwn

        origin_x = origin.position.x
        origin_y = origin.position.y

        x_map = []
        y_map = []


        for r in rangesOwn:
            angle = angle + angle_incrementOwn
            if r <= range_minOwn or r >= range_maxOwn:
                continue

            # x and y to object seen from turtlebot
            x_obj_robot_frame = r*cos(angle)
            y_obj_robot_frame = r*sin(angle)
            
            #x,y of obj in real world 
            x_obj_world_frame = x_turtle + (x_obj_robot_frame*cos(robot_yaw) - y_obj_robot_frame*sin(robot_yaw))
            y_obj_world_frame = y_turtle + (x_obj_robot_frame*sin(robot_yaw) + y_obj_robot_frame*cos(robot_yaw))
            
            # list of object coordinates
            x_map.append(x_obj_world_frame)
            y_map.append(y_obj_world_frame)
            
        x_min = 100
        x_max = 0
        y_min = 100
        y_max = 0

        # add object to map and occupy space
        for idx in range(len(x_map)):

            x_idx = int((x_map[idx]-origin_x)/resolution)
            y_idx = int((y_map[idx]-origin_y)/resolution)

            if self.is_in_bounds(grid_map, x_idx, y_idx) == False:
                continue

            self.add_to_map(grid_map, x_idx, y_idx, self.occupied_space)

            if x_idx < x_min:
                x_min = x_idx
            
            if x_idx > x_max:
                x_max = x_idx
            
            if y_idx < y_min:
                y_min = y_idx
            
            if y_idx > y_min:
                y_min = y_idx    


        """
        For C only!
        Fill in the update correctly below.
        """ 
          # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = x_min
        # The minimum y index in 'grid_map' that has been updated
        update.y = y_min
        # Maximum x index - minimum x index + 1
        update.width = (x_max-x_min+1)
        # Maximum y index - minimum y index + 1
        update.height = (y_max-y_min+1)
        # The map data inside the rectangle, in row-major order.
        update.data = []

        for y in range(update.y, update.y+update.height):
            for x in range(update.x, update.x+update.width):
                update.data.append(grid_map[x, y])

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
#!/usr/bin/env python3

"""
    # {Erik Molitor}
    # {9611043879}
    # {emolitor@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

        

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()
        """
        Fill in your solution here
        """

        #inital values 
        angle_minOwn = scan.angle_min
        angle_max = scan.angle_max
        angle_incrementOwn = scan.angle_increment
        range_minOwn = scan.range_min
        range_maxOwn = scan.range_max
        rangesOwn = scan.ranges
        x_turtle = pose.pose.position.x
        y_turtle = pose.pose.position.y
        angle = angle_minOwn

        origin_x = origin.position.x
        origin_y = origin.position.y

        x_map = []
        y_map = []


        for r in rangesOwn:
            if r > range_minOwn and r < range_maxOwn:

                # x and y to object seen from turtlebot
                x_obj_robot_frame = r*cos(angle)
                y_obj_robot_frame = r*sin(angle)
            
                #x,y of obj in real world (rotation matrix)
                x_obj_world_frame = x_turtle + (x_obj_robot_frame*cos(robot_yaw) - y_obj_robot_frame*sin(robot_yaw))
                y_obj_world_frame = y_turtle + (x_obj_robot_frame*sin(robot_yaw) + y_obj_robot_frame*cos(robot_yaw))
            
                # list of object coordinates
                x_map.append(x_obj_world_frame)
                y_map.append(y_obj_world_frame)

                # start and end point of trace
                start = int((x_turtle - origin_x)/resolution), int((y_turtle - origin_x)/resolution)
                end = int((x_obj_world_frame - origin_x)/resolution),int((y_obj_world_frame - origin_y)/resolution)

                #traveresed cells
                trav = self.raytrace(start, end)
                # add free space to all cells inbetween robot and object
                for cell in trav:
                    self.add_to_map(grid_map, cell[0], cell[1], self.free_space)

            angle = angle + angle_incrementOwn
   
        x_min = 500
        x_max = -500
        y_min = 500
        y_max = -500

        # add object to map and occupy space
        for idx in range(len(x_map)):

            x_idx = int((x_map[idx]-origin_x)/resolution)
            y_idx = int((y_map[idx]-origin_y)/resolution)
            
            if self.is_in_bounds(grid_map, x_idx, y_idx) == False:
                continue
            self.add_to_map(grid_map, x_idx, y_idx, self.occupied_space)

            if x_idx < x_min:
                x_min = x_idx
            
            if x_idx > x_max:
                x_max = x_idx
            
            if y_idx < y_min:
                y_min = y_idx
            
            if y_idx > y_max:
                y_max = y_idx    


        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = x_min
        # The minimum y index in 'grid_map' that has been updated
        update.y = y_min
        # Maximum x index - minimum x index + 1
        update.width = (x_max-x_min+1)
        # Maximum y index - minimum y index + 1
        update.height = (y_max-y_min+1)
        # The map data inside the rectangle, in row-major order.
        update.data = []

        for y in range(update.y, update.y+update.height):
            for x in range(update.x, update.x+update.width):
                update.data.append(grid_map[x, y])

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """

        """
        Fill in your solution here
        """
        c_r = self.radius
        height = grid_map.get_height()
        width = grid_map.get_width()

        for y in range(height):
            for x in range(width):
                cell = grid_map[x, y]
                if cell == self.occupied_space:
                    xlb = int(x-c_r)
                    xub = int(x+c_r)
                    ylb = int(y-c_r)
                    yub = int(y+c_r)

                    for y_t in range(ylb, yub):
                        for x_t in range(xlb, xub):
                            if self.is_in_bounds(grid_map, x_t, y_t) == True:
                                cell_t = grid_map[x_t, y_t]
                                if cell_t == self.occupied_space:
                                    continue
                                elif (x-x_t)**2 + (y-y_t)**2 <= c_r**2:
                                    self.add_to_map(grid_map, x_t, y_t, self.c_space)

        # Return the inflated map
        return grid_map

        # Return the inflated map
        return grid_map
