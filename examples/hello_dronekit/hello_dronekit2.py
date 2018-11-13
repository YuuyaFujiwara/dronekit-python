# coding: utf-8
# dronekitの練習
# 養成塾でのサンプル hello_dronekit.pyではdronekit-sitlに接続していたが、
# sim_vehicle.pyでのsitlに接続する。
#
# 1) cygwinコンソールから、sim_vehicle.pyを実行してSITLを起動する。
# 2) SITLで起動したMAVProxyコンソールにてポートを追加する。
#    ※デフォルトポートはMissionPlannerとの接続で使用するため。
#    $output add 127.0.0.1:14552
# 3) このコードを実行する。
#
print "Start simulator (SITL)"
#import dronekit_sitl
#sitl = dronekit_sitl.start_default()
#connection_string = sitl.connection_string()
#connection_string = "tcp:127.0.0.1:5760"    # tcp ...fail
connection_string = "127.0.0.1:14552"       #udp
# Import DroneKit-Python
from dronekit import connect, VehicleMode
# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)
# Get some vehicle attributes (state)
print "Get some vehicle attribute values:"
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name # settable
# Close vehicle object before exiting script
vehicle.close()
# Shut down simulator
# sitl.stop()
print("Completed")