'''hw5p1.py

   This is skeleton code for HW5 Problem 1.  Please EDIT.

   This should re-use the trajectory construction from HW4 P4.  It
   also has proper handling to shut down when the trajectory runs out
   of segments.  And provides the time step (dt) to the evaluation, in
   preparation for the coming inverse kinematics.

   Node: /generator Publish: /joint_states sensor_msgs/JointState

'''

import rclpy
import numpy as np

from asyncio            import Future
from rclpy.node         import Node
from sensor_msgs.msg    import JointState

from hw3code.Segments   import Hold, Stay, GotoCubic, SplineCubic
# from hw4code.hw4p3      import fkin, Jac

from hw6code.GeneratorNode     import GeneratorNode
from hw6code.KinematicChain    import KinematicChain

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self,node):
        # Pick the target.
        self.chain = KinematicChain(node, 'dummy_link', 'link_ten', self.jointnames())
        self.q = np.radians(np.array([0, 90, -90, 0, 0, 0, 0, 0, 0, 0]).reshape((-1,1)))

        q1 = np.radians(np.array([0, 90, -90, 0, 0, 0, 0, 0, 0, 0]).reshape((-1,1)))
        q2 = np.radians(np.array([0, -90, 90, 90, -90, 90, -90, 90, -90, 90]).reshape((-1,1)))

        self.chain.setjoints(self.q)
        self.segments = [Hold(self.q, 1.0),GotoCubic(q1, q2, 1.0),GotoCubic(q2, q1, 1.0)]
        self.t0 = 0
        self.cyclic = True

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        return ['rod_one_joint', 'rod_three_joint', 'rod_four_joint','rod_five_joint',
            'rod_six_joint','rod_seven_joint','rod_eight_joint','rod_nine_joint','rod_ten_joint','rod_eleven_joint']

    # Evaluate at the given time.
    def evaluate(self, tabsolute, dt):
        # Make sure we have a segment.
        if len(self.segments) == 0:
            return None

        # Also check whether the current segment is done.
        if self.segments[0].completed(tabsolute - self.t0):
            # If the current segment is done, shift to the next
            self.t0 = self.t0 + self.segments[0].duration()
            seg = self.segments.pop(0)
            if self.cyclic:
                self.segments.append(seg)

            # Make sure we still have something to do.
            if len(self.segments) == 0:
                return None

        # Compute the positions/velocities as a function of time.
        (q, qdot) = self.segments[0].evaluate(tabsolute - self.t0)

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#   Generator Node Class
#
# class Generator(Node):
#     # Initialization.
#     def __init__(self, future):
#         # Initialize the node, naming it 'generator'
#         super().__init__('generator')

#         # Add a publisher to send the joint commands.
#         self.pub = self.create_publisher(JointState, '/joint_states', 10)

#         # Wait for a connection to happen.  This isn't necessary, but
#         self.get_logger().info("Waiting for a subscriber...")
#         while(not self.count_subscribers('/joint_states')):
#             pass

#         # Set up a trajectory.
#         self.trajectory = Trajectory()
#         self.jointnames = self.trajectory.jointnames()

#         # Save the future to signal when the trajectory end, i.e. no
#         # longer returns useful data.
#         self.future = future

#         # Create a timer to keep calculating/sending commands.
#         self.starttime = self.get_clock().now()
#         rate           = 100
#         self.timer     = self.create_timer(1/float(rate), self.update)
#         self.dt        = self.timer.timer_period_ns * 1e-9
#         self.get_logger().info("Running with dt of %f seconds (%fHz)" %
#                                (self.dt, rate))

#     # Shutdown
#     def shutdown(self):
#         # Destroy the timer, then shut down the node.
#         self.timer.destroy()
#         self.destroy_node()


#     # Update - send a new joint command every time step.
#     def update(self):
#         # Grab the current time.
#         now = self.get_clock().now()
#         t   = (now - self.starttime).nanoseconds * 1e-9

#         # Compute the desired joint positions and velocities for this time.
#         desired = self.trajectory.evaluate(t, self.dt)
#         if desired is None:
#             self.future.set_result("Trajectory has ended")
#             return
#         (q, qdot) = desired

#         # Build up a command message and publish.
#         cmdmsg = JointState()
#         cmdmsg.header.stamp = now.to_msg()      # Current time
#         cmdmsg.name         = self.jointnames   # List of joint names
#         cmdmsg.position     = q                 # List of joint positions
#         cmdmsg.velocity     = qdot              # List of joint velocities
#         self.pub.publish(cmdmsg)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the generator node (100Hz) for the Trajectory.
    rclpy.init(args=args)
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()