import carla
import math
import argparse

def calculate_sides(hypotenuse, angle):


  # Convert the angle to radians
  angle_radians = math.radians(angle)

  # Calculate the opposite side using the sine function
  opposite_side = hypotenuse * math.sin(angle_radians)

  # Calculate the adjacent side using the cosine function
  adjacent_side = hypotenuse * math.cos(angle_radians)

  return opposite_side, adjacent_side

def main():
  
  argparser = argparse.ArgumentParser(
      description=__doc__)
  argparser.add_argument(
      '--host',
      metavar='H',
      default='127.0.0.1',
      help='IP of the host server (default: 127.0.0.1)')
  argparser.add_argument(
      '-p', '--port',
      metavar='P',
      default=2000,
      type=int,
      help='TCP port to listen to (default: 2000)')
  
  args = argparser.parse_args()

  client = carla.Client(args.host, args.port)

  world = client.get_world()
  spawn_points = world.get_map().get_spawn_points()

  actor_list = world.get_actors()

  vehicle = actor_list.filter('*volkswagen.t2_2021*')

  vehicle = vehicle[0]


  vehicle_transform = vehicle.get_transform()

    
  metres_distance = 5

  while(1):

    spectator = world.get_spectator()
    spectator_pos = carla.Transform(vehicle_transform.location + carla.Location(x=20,y=10,z=4),
                                    carla.Rotation(yaw = vehicle_transform.rotation.yaw -155))
    #spectator.set_transform(spectator_pos)

    vehicle_transform = vehicle.get_transform()
    y,x = calculate_sides(metres_distance, vehicle_transform.rotation.yaw )

    spectator_pos = carla.Transform(vehicle_transform.location + carla.Location(x=-x,y=-y,z=5 ),
                                            carla.Rotation( yaw = vehicle_transform.rotation.yaw,pitch = -25))
    spectator.set_transform(spectator_pos)  

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')