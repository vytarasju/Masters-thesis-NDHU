import numpy as np
import math

class DefineMeasurements:
    def __init__(self,terrain, height_width = 1, total_cost = 0, motion_cost = np.array([])):
        # Distance measured in meters
        self.terrain = []
        self.terrain_scale = height_width
        self.air_density = 1.225 #kg/m^3

        # Non-Measured distance variables
        self.terrain_nm = terrain
        self.total_cost_nm = total_cost
        self.motion_cost_nm = motion_cost.tolist()

        self.initUAV()

    # If the init was called before motion and total distance results
    def initDistance(self, total_cost = 0, motion_cost = []):
        self.total_cost_nm = total_cost
        self.motion_cost_nm = motion_cost.tolist()
    
    # Main UAV parameters
    def initUAV(self):
        # Added from other sources (DJI T10)
        self.battery_capacity = 9500 #mAh
        self.battery_voltage = 52.22 #V
        self.max_wind_tolerance = 6 #m/s

        # Simulated Values From Reference
        self.UAV_weight = 100 #Newton
        self.rotor_radius = 0.5 #meters
        self.blade_angular_velocity = 400 #radians/second
        self.blade_number = 4 #quadcopter
        self.blade_chord_length = 0.0196 #meters?
        self.fuselage_flat_plane_area = 0.0118 #meters^2
        self.profile_drag_coefficient = 0.012
        self.induced_power_correction_factor = 0.1

        self.rotor_disc_area = math.pi * self.rotor_radius**2
        self.blade_tip_speed = self.blade_angular_velocity * self.rotor_disc_area
        self.rotor_solidity = (self.blade_number * self.blade_chord_length) / (math.pi * self.rotor_radius)
        self.fuselage_drag_ratio = self.fuselage_flat_plane_area / (self.rotor_solidity * self.rotor_disc_area)
        self.rotor_mean_induced_velocity = math.sqrt(self.UAV_weight / (2 * self.air_density * self.rotor_disc_area))

    """ Takes speed meters/second and returns power required
     to achieve that specific distance in meters per seconds
     Considering power consumption a constant (depending on speed)
     The answer is then Power consumtion * seconds of operation"""
    # Air Density still a constant here
    def getPropulsionPowerConsumtion(self, speed):
        # Calculating power constants individually
        blade_profile_power = (self.profile_drag_coefficient / 8) * self.air_density * self.rotor_solidity * \
                                    self.rotor_disc_area * self.blade_angular_velocity**3 * self.rotor_radius**3

        induced_power = (1 + self.induced_power_correction_factor) * (self.UAV_weight**(3/2) / \
                                             math.sqrt(2 * self.air_density * self.rotor_disc_area))
        
        # Calculating separate parts of propulsion power
        blade_profile = blade_profile_power * (1 + ((3 * speed**2) / (self.blade_tip_speed**2)))
        induced = induced_power * (math.sqrt(1 + (speed**4 / (4 * self.rotor_mean_induced_velocity**4))) - \
                                   (speed**2 / (2 * self.rotor_mean_induced_velocity**2)))**(1/2)
        parasitic = 0.5 * self.fuselage_drag_ratio * self.air_density * self.rotor_solidity * \
                    self.rotor_disc_area * speed
        
        propulsion_power_consumtion = blade_profile + induced + parasitic

        return propulsion_power_consumtion

    # Function converting total distance and motion distance to:
    # meters (XYZ terrain is already defined in meters), or seconds, or milliamphours
    def convertDistancetoMeasurements(self, type, speed = 0):
        is_aco, is_motion = False, False
        distance_measured = 0
        motion_measured = []

        if self.total_cost_nm != 0: is_aco = True
        if len(self.motion_cost_nm) != 0: is_motion = True

        match type:
            case 'meters':
                if is_aco: distance_measured = self.total_cost_nm * self.terrain_scale

                if is_motion: 
                    for iteration, row in enumerate(self.motion_cost_nm):
                        motion_measured.append([])
                        for col in row:
                            distance_meters = col * self.terrain_scale
                            motion_measured[iteration].append(distance_meters)

            case 'seconds':
                if speed == 0:
                    print('ERROR: Specified type requires UAV speed')
                    return
                
                if is_aco: 
                    distance_meters = self.total_cost_nm * self.terrain_scale
                    distance_measured = distance_meters / speed

                if is_motion: 
                    for iteration, row in enumerate(self.motion_cost_nm):
                        motion_measured.append([])
                        for col in row:
                            distance_meters = col * self.terrain_scale
                            distance_seconds = distance_meters / speed
                            motion_measured[iteration].append(distance_seconds)

            case 'milliamphours':
                if speed == 0:
                    print('ERROR: Specified type requires UAV speed')
                    return
                
                power_consumtion_moving = self.getPropulsionPowerConsumtion(speed)
                if is_aco: 
                    distance_meters = self.total_cost_nm * self.terrain_scale
                    distance_seconds = distance_meters / speed
                    distance_watthour = power_consumtion_moving  * (distance_seconds / 3600)
                    distance_measured = distance_watthour / self.battery_voltage * 1000

                if is_motion: 
                    for iteration, row in enumerate(self.motion_cost_nm):
                        motion_measured.append([])
                        for col in row:
                            distance_meters = col * self.terrain_scale
                            distance_seconds = distance_meters / speed
                            distance_watthour = power_consumtion_moving * (distance_seconds / 3600)
                            distance_milliamphour = distance_watthour / self.battery_voltage * 1000
                            motion_measured[iteration].append(distance_milliamphour)
            case _:
                print('ERROR: Speficied type not defined')
        
        if is_aco and is_motion: return distance_measured, motion_measured
        elif not is_aco and is_motion: return motion_measured
        elif is_aco and not is_motion: return distance_measured
        else:
            print("ERROR: Neither total_cost, nor motion_cost were provided initially")
            return 0