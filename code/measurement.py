import numpy as np
import math

class UAV:
    def __init__(self,terrain = [], height_width = 1, total_cost = 0, motion_cost = np.array([])):
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
        self.UAV_max_speed = 30 #m/s
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

        # Gets maximum operation time
        crusing_speed_consumption = self.getPropulsionPowerConsumtion(self.UAV_max_speed)
        # Battery used for a second of set speed operation
        crusing_persecond_watthour = crusing_speed_consumption / 3600
        crusing_persecond_charge = crusing_persecond_watthour / self.battery_voltage * 1000
        self.maximum_operation_time = math.floor(self.battery_capacity / crusing_persecond_charge)

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
                            distance_charge = distance_watthour / self.battery_voltage * 1000
                            motion_measured[iteration].append(distance_charge)
            case _:
                print('ERROR: Speficied type not defined')
        
        if is_aco and is_motion: return distance_measured, motion_measured
        elif not is_aco and is_motion: return motion_measured
        elif is_aco and not is_motion: return distance_measured
        else:
            print("ERROR: Neither total_cost, nor motion_cost were provided initially")
            return 0

class WPT:
    def __init__(self):
        # Specs taken from PowerCast TX91501B

        self.power_transmitter = 2000 #Watts
        self.supply_voltage = 5 #V
        # self.diameter_at_meter = 5 #meters 

        """Initializse individual parameters"""
        self.gain_transmitter = 8 #dBi
        self.gain_receiver = 2 #dBi
        self.wavelength = 0.33
        # Original Reference: 0.125; from PowerCast PCC110: 0.75
        self.rectifier_efficiency = 0.85 #RF to DC conversion
        # Original Reference: 3.16105
        self.polarization_loss = 3.16105
        self.pathloss_coefficient = 2
        self.friis_adjustable_var = 0.2316

        self.charging_efficiency_constants = ((self.gain_transmitter * self.gain_receiver * self.rectifier_efficiency) / self.polarization_loss) \
                                             * ((self.wavelength / (4 * math.pi))**self.pathloss_coefficient)

        """References are providing only the result of constants"""
        # self.charging_efficiency_constants = 0.000432
    
    # Gives how much time will WPT need to charge particular sensor
    # Takes in distance between sensor and WPT and how much to charge (mAh)
    def chargeTime(self, distance, charge_needed):
        # Helper function: how much power will IOT receiver get from specific distance
        def powerReceivedBySensor(distance):
            charging_efficiency = self.charging_efficiency_constants / ((distance + self.friis_adjustable_var)**self.pathloss_coefficient)
            power_received = charging_efficiency * self.power_transmitter
            return power_received
        charging_power_watt = powerReceivedBySensor(distance)
        charging_power_milliamp = charging_power_watt / self.supply_voltage
        charge_time = ((charge_needed / 1000) / charging_power_milliamp) * 3600
        # charge_time = ((charge_needed / 1000) / charging_power_watt) * 3600
        return charge_time
    
    # Gives how much battery capacity WPT would use when operating for a specified time
    def chargeConsumptionGivenTime(self, time):
        hours = time / 3600
        watthours = self.power_transmitter * hours
        charge_used = (watthours / self.supply_voltage) * 1000
        return charge_used


class IoTDevice:
    def __init__(self):
        self.sensor_operation_freq = 60 #s
        self.mc_operation_freq = 300 #s
        #equals to a singular AAA battery
        self.battery_capacity = 1200 #mAh
        
        # Specs from DHT22
        #Sensor
        self.sensor_voltage = 3.3 #V
        self.sensor_amp_operating = 0.0015 #A
        self.sensor_amp_idle = 0.00005 #A
        self.sensor_operation_period = 2 #s

        # Specs from Particle Argon 
        #Microcontroller
        self.mc_voltage = 3.3 #V
        self.mc_amp_operating = 0.0317 #A
        self.mc_amp_idle  = 0.00352 #A
        self.mc_operation_period = 10 #s

        #Power Variables
        self.mc_power_operating = self.mc_voltage * self.mc_amp_operating
        self.mc_power_idle = self.mc_voltage * self.mc_amp_idle

    # Returns how much battery IOT device would consume given: start and end time points
    def batteryConsumtionGivenTime(self, time_start, time_end):
        # Helper function: returns how much time decive has been operating and idling
        def countDeviceRunTime(operation_freq, operation_period):
            cycles_past = math.floor(time_start / operation_freq)
            current_cycle_time = time_start - (cycles_past * operation_freq)

            giventime_cycles = []
            reference_time = time_start - current_cycle_time
            iteration = 1
            while reference_time != time_end:
                start_cycle, end_cycle = 0, operation_freq
                if iteration == 1: start_cycle += current_cycle_time
                # If reference time is not enough for more than 1 cycle anymore
                if (reference_time + operation_freq) >= time_end:
                    end_cycle = time_end - reference_time
                    giventime_cycles.append([start_cycle, end_cycle])
                    reference_time = time_end
                # If reference time can fit more than 1 cycle
                else:
                    reference_time += operation_freq
                    giventime_cycles.append([start_cycle, end_cycle])
                iteration += 1

            # Check how much operation time and idle time each cycle has
            total_operation_time, total_idle_time = 0, 0
            max_idle_time = operation_freq - operation_period
            for cycle in giventime_cycles:
                operation_time = cycle[1] - max_idle_time
                # If cycle time does not have operation time included, it's 0
                if operation_time <= 0: operation_time = 0
                idle_time = (cycle[1] - cycle[0]) - operation_time
                # If cycle time does not have idle time included, it's 0
                if idle_time <= 0: idle_time = 0
                total_operation_time += operation_time
                total_idle_time += idle_time
            return total_operation_time, total_idle_time
        
        # Helper function: returns how much charge (mAh) given idling and operating time consumes
        def chargeConsumptionGivenRuntime(idle_time, operation_time, voltage, operation_amp, idle_amp):
            # Helper function: returns how much charge (mAh) given time and properties the device consumes
            def chargeGivenTime(time, amp):
                power = voltage * amp
                hours = time / 3600
                watthours = power * hours
                charge = (watthours / voltage) * 1000
                return charge
            idle_charge = chargeGivenTime(idle_time, idle_amp)
            operation_charge = chargeGivenTime(operation_time, operation_amp)
            return idle_charge + operation_charge


        mc_operation_runtime, mc_idle_runtime = countDeviceRunTime(self.mc_operation_freq, self.mc_operation_period)
        mc_charge = chargeConsumptionGivenRuntime(mc_idle_runtime, mc_operation_runtime, self.mc_voltage, self.mc_amp_operating, self.mc_amp_idle)

        sensor_operation_runtime, sensor_idle_runtime = countDeviceRunTime(self.sensor_operation_freq, self.sensor_operation_period)
        sensor_charge = chargeConsumptionGivenRuntime(sensor_operation_runtime, sensor_idle_runtime, self.sensor_voltage, self.sensor_amp_operating, self.sensor_amp_idle)
        
        return mc_charge + sensor_charge

"""Testing BEGIN"""
# drone = UAV()
# device = IoTDevice()
# wpt = WPT()

# angle = 120
# cluster_list = [[1.47, 1.47], [1.35, 1.35], [3.2, 3.2]]
# print(f'absolute maximum operation time possible with drone: {(drone.maximum_operation_time / 60):.2f} minutes')
# charge_used = device.batteryConsumtionGivenTime(0,drone.maximum_operation_time)
# time_sum = 0

# for cluster in cluster_list:
#     max_distance = max(dis for dis in cluster) / 2
#     print(f'center between 2 points: {max_distance}')
#     cluster_hover_height = max_distance / math.tan(math.radians(angle / 2))
#     if cluster_hover_height < 2: cluster_hover_height = 2
#     # print(f'tan {math.tan(math.radians(angle / 2))} hover height {cluster_hover_height}')
#     time_to_charge = wpt.chargeTime(cluster_hover_height, charge_used)
#     print(f'{(time_to_charge / 60):.2f} minutes to charge at {cluster_hover_height} meters distance for max drone op time')
#     # print(f'WPT would use {wpt.chargeConsumptionGivenTime(time_to_charge):.2f} mAh to complete this charging')
#     time_sum += time_to_charge

# print(f'Total time taken {time_sum}')
"""Testing END"""