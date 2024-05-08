import numpy as np
import math

class DefineMeasurements:
    def __init__(self, height_width, terrain, distance = 0, motion_distance = []):
        # Distance measured in meters
        self.terrain = []
        self.terrain_scale = height_width
        # defined it for itteration, did not use it
        # self.terrain_ratio_100 = height_width / 100 # Meters per 0.01 scale for both x and y
        self.air_density = 1.225 #kg/m^3

        # Non-Measured distance variables
        self.terrain_nm = terrain
        self.distance_nm = distance
        self.motion_distance_nm = motion_distance.tolist()

        self.initUAV()
    
    def initUAV(self):
        # Simulated Values From Reference
        self.UAV_weight = 100 #Newton
        self.rotor_radius = 0.5 #meters
        self.blade_angular_velocity = 400 #radians/second
        self.blade_number = 4 #quadcopter
        self.blade_chord_length = 0.0196 #meters?
        self.fuselarge_flat_plane_area = 0.0118 #meters^2
        self.profile_drag_coefficient = 0.012
        self.induced_power_correction_factor = 0.1

        self.rotor_disc_area = math.pi * self.rotor_radius**2
        self.blade_tip_speed = self.blade_angular_velocity * self.rotor_disc_area
        self.rotor_solidity = (self.blade_number * self.blade_chord_length) / (math.pi * self.rotor_radius)
        self.fuselarge_drag_ratio = self.fuselarge_flat_plane_area / (self.rotor_solidity * self.rotor_disc_area)
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
        parasitic = 0.5 * self.fuselarge_drag_ratio * self.air_density * self.rotor_solidity * \
                    self.rotor_disc_area * speed
        
        propulsion_power_consumtion = blade_profile + induced + parasitic

        return propulsion_power_consumtion

    def getMeasurements(self, type, speed = 0):
        is_aco, is_motion = False, False
        distance_measured = 0
        motion_measured = []

        # Preparing for different types conversions
        if (type == 'seconds' or type == 'watts') and speed == 0: 
            print('ERROR: Specified type requires UAV speed')
            return

        power_consumtion_moving = 0
        if type == 'watts': power_consumtion_moving = self.getPropulsionPowerConsumtion(speed)

        # Converting distance
        if self.distance_nm != 0:
            is_aco = True
            distance_meters = self.distance_nm * self.terrain_scale
            
            if speed != 0: distance_seconds = distance_meters / speed

            if type == 'meters': distance_measured = distance_meters
            if type == 'seconds': distance_measured = distance_seconds
            if type == 'watts':
                distance_wats = power_consumtion_moving * distance_seconds
                distance_measured = distance_wats


        # Converting motion
        if len(self.motion_distance_nm) != 0:
            is_motion = True
            # Calculate motion iteratively
            for iteration, row in enumerate(self.motion_distance_nm):
                motion_measured.append([])
                for col in row:
                    distance_meters = col * self.terrain_scale

                    if speed != 0: distance_seconds = distance_meters / speed

                    if type == 'meters': motion_measured[iteration].append(distance_meters)
                    if type == 'seconds': motion_measured[iteration].append(distance_seconds)
                    if type == 'watts':
                        distance_wats = power_consumtion_moving * distance_seconds
                        motion_measured[iteration].append(distance_wats)

        
        if is_aco and is_motion: return distance_measured, motion_measured
        elif not is_aco and is_motion: return motion_measured
        elif is_aco and not is_motion: return distance_measured
        else:
            print("ERROR: Neither aco_distance, nor motion_distance were provided initially")
            return 0