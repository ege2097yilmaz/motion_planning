import carla
import time

def main():
    try:
        # Connect to the CARLA server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Load a specific world
        world = client.load_world('Town03')  # Replace 'Town01' with the name of the desired map

        # Get the blueprint library
        blueprint_library = world.get_blueprint_library()

        # Select a vehicle blueprint (e.g., Tesla Model 3)
        vehicle_bp = blueprint_library.filter('model3')[0]

        # Get a valid spawn point from the map
        spawn_point = world.get_map().get_spawn_points()[0]

        # Spawn the vehicle
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)

        print(f'Spawned vehicle {vehicle.type_id} at {spawn_point.location}')

        # Set the vehicle to drive in autopilot mode
        vehicle.set_autopilot(True)

        # Run the simulation for a few seconds
        time.sleep(10)

    finally:
        # Clean up and destroy all actors
        print('Destroying actors...')
        vehicle.destroy()
        print('All actors destroyed.')

if __name__ == '__main__':
    main()
