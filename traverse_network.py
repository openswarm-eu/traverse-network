import math
from vector2d import Vector2D

# Network traversal parameters (repulsion)
TARGET_DISTANCE = 0.1
EXPONENT = 2
GAIN = 1

WEIGHT_ATTRACTION = 10
WEIGHT_REPULSION = 1

TARGET_SEPARATION = 0.05 # meters


def get_attraction_vector(neighbors):

    result_vector = Vector2D(0,0)

    # Find the node with the smallest hop count to the target location
    min_hop_count_node = (Vector2D(0,0), float('inf'))
    for robot in neighbors:
        hop_count = robot[1]
        if hop_count < min_hop_count_node[1]:
            min_hop_count_node = robot

    # Calculate the attraction vector towards the side of the target node
    direction = min_hop_count_node[0]

    margin = direction.rotate(math.pi/2) # Move to a point perpendicular from the node
    margin = margin.normalize() # Get unit vector
    margin *= TARGET_SEPARATION # separation away from the node

    result_vector = direction + margin

    return result_vector


def repulsion(distance):
    f_norm_dist_exp = pow(TARGET_DISTANCE / distance, EXPONENT)
    return -GAIN / distance * (f_norm_dist_exp * f_norm_dist_exp)


def get_repulsion_vector(neighbors):

    result_vector = Vector2D(0,0)

    for robot in neighbors:
        position = robot[0]
        distance = abs(position)
        angle = math.atan2(position.y, position.x)

        # Calculate repulsion force
        lj_force = repulsion(distance)

        # Calculate the repulsion vector
        x = math.cos(angle) * lj_force
        y = math.sin(angle) * lj_force
        repultion_vector = Vector2D(x, y)

        result_vector += repultion_vector       

    if len(neighbors) > 0:
        result_vector /= len(neighbors)

    return result_vector


def traverse_network(neighbors):
    
    attraction_vector = get_attraction_vector(neighbors)
    repulsion_vector = get_repulsion_vector(neighbors)
    
    sum_force = WEIGHT_ATTRACTION * attraction_vector + WEIGHT_REPULSION * repulsion_vector

    return sum_force


if __name__ == "__main__":

    # Example usage of the traverse_network function

    ### Inputs ###
    # List of nodes in the network
    #   1. position (x,y) relative to the robot
    #   2. the node's hop count to the target location to travel towards
    neighbors = [
        (Vector2D( 0.1, 0.1), 2),
        (Vector2D(-0.1, 0.1), 3),
        (Vector2D(-0.1, -0.1), 3),
    ]

    vector = traverse_network(neighbors)
    print(vector)

    # Sample output
    # (1.06311,0.936887)
    # 
    # The output vector represents the direction to move towards the target location
    # The magnitude of the vector can be tuned by the parameters defined at the top of the file
