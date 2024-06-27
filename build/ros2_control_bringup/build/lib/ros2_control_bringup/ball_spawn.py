import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetModelState

import random
class BallSpawner(Node):
    def __init__(self):
        super().__init__('ball_spawner')
        self.client = self.create_client(SpawnEntity, 'spawn_entity')

        p_x = random.uniform(*random.choice([(-3, -1), (1, 3)]))
        p_y = random.uniform(*random.choice([(-3, -1), (1, 3)]))
        p_z = random.uniform(1,3)

        if(p_x<0.0):
          v_x = random.uniform(0,3)
        elif(p_x>0.0):
          v_x = random.uniform(-3,0)
        else:
          v_x = 0.0
        
        if(p_x<0.0):
          v_y = random.uniform(0,3)
        elif(p_x>0.0):
          v_y = random.uniform(-3,0)
        else:
          v_y = 0.0

        v_z = random.uniform(0,3)

        self.ball_sdf = """
<sdf version='1.7'>
  <model name='table_tennis_ball'>
    <pose>0 0 0.04 0 0 0</pose>
    <static>false</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.02</radius>
          </sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.3</mu>
              <mu2>0.3</mu2>
              <fdir1>1 0 0</fdir1>
              <slip1>0.1</slip1>
              <slip2>0.1</slip2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.9</restitution_coefficient>
            <threshold>0.0001</threshold>
          </bounce>
          <contact>
            <collide_without_contact>0</collide_without_contact>
            <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
            <collide_bitmask>1</collide_bitmask>
            <poissons_ratio>0.35</poissons_ratio>
            <elastic_modulus>5e8</elastic_modulus>
            <ode>
              <soft_cfm>0.0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e10</kp>
              <kd>1.0</kd>
              <max_vel>10.0</max_vel>
              <min_depth>1e5</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <sphere>
            <radius>0.02</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Orange</name>
          </script>
        </material>
      </visual>
      <inertial>
        <mass>0.026</mass>
        <inertia>
          <ixx>8.66666666667e-06</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>8.66666666667e-06</iyy>
          <iyz>0</iyz>
          <izz>8.66666666667e-06</izz>
        </inertia>
      </inertial>
    </link>
    <plugin name="initial_velocity" filename="libInitialVelocityPlugin.so">
      <linear>""" + f"{0} {0} {0}" + """</linear>
      <angular>0 0 0</angular>
    </plugin>
  </model>
</sdf>
        """
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()
        self.req.name = 'ball'
        self.req.xml = self.ball_sdf
        self.req.initial_pose.position.x = 1.0
        self.req.initial_pose.position.y = 0.0
        self.req.initial_pose.position.z = 3.0

    def send_request(self):
        self.req.robot_namespace = "my_namespace"
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    ball_spawner = BallSpawner()
    ball_spawner.send_request()

    while rclpy.ok():
        rclpy.spin_once(ball_spawner)
        if ball_spawner.future.done():
            try:
                response = ball_spawner.future.result()
            except Exception as e:
                ball_spawner.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                ball_spawner.get_logger().info(
                    'Ball Spawned with name: %s' % ball_spawner.req.name)
            break
    ball_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    