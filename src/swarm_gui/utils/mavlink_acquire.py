from pymavlink import mavutil

class MavlinkAcquire:

    def __init__(self, ip):

        """
        This GCS port will receive messages from all UAVs and
        distinguishing between them by their system ID
        """
        self.master = mavutil.mavlink_connection('udp:' + ip + ':14550')  
    
    def get_drone_id(self,msg):
        """
        Get the PX4 system ID
        """ 
        raw_drone_id=msg.get_srcSystem()

        # Distinguish Visbot OWL2 and OWL3 based on customized PX4 configuration
        # OWL2: 20-23
        # OWL3: 30-37
        drone_gen=int(raw_drone_id) // 10

        if drone_gen == 3:
            drone_id=int(raw_drone_id) % 10

        if drone_gen == 2:
            drone_id=int(raw_drone_id) % 10 + 8
        
        return drone_id

    # ########################
    #  Acquire Drone State
    # ########################
    def acquire_drone_info(self):
        """
        Acquire the drone's information
        """
        msg = self.master.recv_match(type='HEARTBEAT', blocking=False)

        if not msg:
            return None
        
        # extract info from the heartbeat message
        state = self.parse_heartbeat(msg)
        return state
    
    def parse_heartbeat(self,msg):
        """
        Parse the heartbeat message and extract the UAV's state
        """
        base_mode = msg.base_mode
        custom_mode = msg.custom_mode
        system_status = msg.system_status

        # extract the flight mode
        flight_mode = mavutil.mode_string_v10(msg)

        # extract the armed status
        armed = base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        # assemble the state information
        state = {
            'drone_id': self.get_drone_id(msg),
            'base_mode': base_mode,
            'custom_mode': custom_mode,
            'system_status': system_status,
            'flight_mode': flight_mode,
            'armed': armed,
        }

        return state

    # ########################
    #  Acquire Battery Status
    # ########################
    def acquire_drone_battery(self):
        """
        Acquire the battery level from BATTERY_STATUS message
        """
        msg = self.master.recv_match(type='BATTERY_STATUS', blocking=False)

        if not msg:
            return None
        
        battery_remaining = msg.battery_remaining
        voltages = msg.voltages[0] / 1000.0  # Convert millivolts to volts
        current_battery = msg.current_battery / 100.0  # Convert to amperes
        battery_info = {
            'drone_id': self.get_drone_id(msg),
            "battery_remaining": battery_remaining,
            "voltage": voltages,
            "current": current_battery
        }
        return battery_info
    

    # ########################
    #  Acquire Body Height (depracated)
    # height is now acquired from ROS
    # ########################
    def acquire_drone_odom(self):
        """
        Acquire the body height from LOCAL_POSITION_NED message
        """
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        
        if not msg:
            return None
        
        # TODO: More info like velocity, acceleration can be extracted from this message if needed
        # Note the sign of y and z axis in NED frame is different from the World frame (North-West-Up)
        height = -round(msg.z, 6)

        height_info = {
            'drone_id': self.get_drone_id(msg),
            'height': height
        }
        return height_info


    # def acquire_drone_PV(self):
    #     PV_msg=self.master.recv_match('LOCAL_POSITION_NED',blocking=True)
        
    #     PV={
    #         "velocity_norm": PV_msg.
    #     }
    
if __name__ == '__main__':
    ip='0.0.0.0'
    get_mavlink = MavlinkAcquire(ip)
    get_mavlink.main()