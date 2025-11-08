# Spawn ROV in Gazebo
ign service -s /world/pool/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 1000  \
  --req "sdf_filename: '/home/jhsrobo/corews/src/rov_sim/urdf/rov_sim.sdf' name: 'rov'"