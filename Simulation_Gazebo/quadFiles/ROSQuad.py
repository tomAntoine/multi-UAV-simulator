"""
Drone delivery: 
we are going to build a mission in mission planner, upload the mission to the drone.
The script will connect with the vehicle and check if a new mission has been uploaded. 
As soon as a valid mission is available, we takeoff in GUIDED mode and then we switch
to AUTO.
When the mission is completed we command to co back to home and land
"""


import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import numpy as np
import math


#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------
#-- Define arm and takeoff
def arm_and_takeoff(vehicle,altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 0.1:
          print("Target altitude reached")
          break
      time.sleep(1)

def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.commands.upload()


    # After clearing the mission you MUST re-download the mission from the vehicle
    # before vehicle.commands can be used again
    # (see https://github.com/dronekit/dronekit-python/issues/230)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

def clear_all_mission(vehicle):
    n_WP, _ = get_current_mission(vehicle)

    for i in range(n_WP):
        clear_mission(vehicle)



def download_mission(vehicle):
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    

def get_current_mission(vehicle):
    """
    Downloads the mission and returns the wp list and number of WP 
    
    Input: 
        vehicle
        
    Return:
        n_wp, wpList
    """

    print ("Downloading mission")
    download_mission(vehicle)
    missionList = []
    n_WP        = 0
    for wp in vehicle.commands:
        missionList.append(wp)
        n_WP += 1 
        
    return n_WP, missionList
    

def add_last_waypoint_to_mission(                                       #--- Adds a last waypoint on the current mission file
        vehicle,            #--- vehicle object
        wp_Last_Latitude,   #--- [deg]  Target Latitude
        wp_Last_Longitude,  #--- [deg]  Target Longitude
        wp_Last_Altitude):  #--- [m]    Target Altitude
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
    # Get the set of commands from the vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()


    # Save the vehicle commands to a list
    missionlist=[]

    for cmd in cmds:
        missionlist.append(cmd)



    # Modify the mission as needed. For example, here we change the


    #mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT

    wpLastObject = Command( 0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)


    # Clear the current mission (command is sent when we call upload())
    print('clear')
    cmds.clear()

    #Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()

    return (cmds.count)    

def ChangeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            print(mode)
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True


def get_location_metres(original_location, dNorth, dEast, altitude):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    newalt = original_location.alt + altitude
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,newalt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,newalt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;

def reverse_get_location_metres(original_location, location):

    lat = location.lat
    lon = location.lon
    alt = location.alt

    earth_radius = 6378137.0 

    dLat = (lat - original_location.lat)*math.pi/180
    dLon = (lon - original_location.lon)*math.pi/180

    dNorth = dLat*earth_radius
    dEast  = dLon*(earth_radius*math.cos(math.pi*original_location.lat/180))

    return(dNorth,dEast,alt)





def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


if __name__ == '__main__':

    while True:

        if mode == 'GROUND':
            #--- Wait until a valid mission has been uploaded
            n_WP, missionList = get_current_mission(vehicle)
            time.sleep(2)


            arm_and_takeoff(10)


            if n_WP > 0:
                print ("A valid mission has been uploaded: takeoff!")
                mode = 'TAKEOFF'


        elif mode == 'TAKEOFF':
           
            #-- Add a fake waypoint at the end of the mission


            original_location = vehicle.location.global_relative_frame


            print(vehicle.location.global_relative_frame.lat)
            print(vehicle.location.global_relative_frame.lon)
            print(vehicle.location.global_relative_frame.alt)

            wp1 = get_location_metres(original_location,0,0,15)

            print(wp1)
            wp2 = get_location_metres(original_location,2,0,10)
            print(wp2)
            wp3 = get_location_metres(original_location,20,20,10)
            wp4 = get_location_metres(original_location,20,10,10)
            wp5 = get_location_metres(original_location,0,0,3)
            wp6 = get_location_metres(original_location,-5,-20,10)



            add_last_waypoint_to_mission(vehicle, wp1.lat, wp1.lon, wp1.alt)
            add_last_waypoint_to_mission(vehicle, wp2.lat, wp2.lon, wp2.alt)
            add_last_waypoint_to_mission(vehicle, wp3.lat, wp3.lon, wp3.alt)
            add_last_waypoint_to_mission(vehicle, wp4.lat, wp4.lon, wp1.alt)
            add_last_waypoint_to_mission(vehicle, wp5.lat, wp5.lon, wp1.alt)
            add_last_waypoint_to_mission(vehicle, wp6.lat, wp6.lon, wp1.alt)


            print("Home waypoint added to the mission")
            time.sleep(1)
            #-- Takeoff
            arm_and_takeoff(10)
            
            #-- Change the UAV mode to AUTO
            print("Changing to AUTO")
            ChangeMode(vehicle,"AUTO")
            
            #-- Change mode, set the ground speed
            vehicle.groundspeed = gnd_speed
            mode = 'MISSION'
            print ("Switch mode to MISSION")
            
        elif mode == 'MISSION':
            #-- Here we just monitor the mission status. Once the mission is completed we go back
            #-- vehicle.commands.cout is the total number of waypoints
            #-- vehicle.commands.next is the waypoint the vehicle is going to
            #-- once next == cout, we just go home
            
            print ("Current WP: %d of %d "%(vehicle.commands.next, vehicle.commands.count))
            if vehicle.commands.next == vehicle.commands.count:
                print ("Final waypoint reached: go back home")
                #-- First we clear the flight mission
                clear_mission(vehicle)
                print ("Mission deleted")
                
                #-- We go back home
                ChangeMode(vehicle,"RTL")
                mode = "BACK"
                
        elif mode == "BACK":
            if vehicle.location.global_relative_frame.alt < 1:
                print ("Switch to GROUND mode, waiting for new missions")
                mode = 'GROUND'
                ChangeMode(vehicle,"STABILIZE")
        
        
        time.sleep(0.5)