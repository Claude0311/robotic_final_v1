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
from std_msgs.msg import Int32MultiArray

from hw3code.Segments   import Hold, Stay, GotoCubic, SplineCubic
# from hw4code.hw4p3      import fkin, Jac

from final_v1.GeneratorNode     import GeneratorNode
from final_v1.KinematicChain    import KinematicChain
from hw5code.TransformHelpers  import *

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self,node):
        # Pick the target.
        self.chain = KinematicChain(node, 'world', 'link_41', self.jointnames())
        self.q = np.radians(np.array([0]*41).reshape((-1,1)))
        self.q[0,0] = np.pi/2
        self.q_nom = np.radians(np.array([0]*41).reshape((-1,1)))
        self.q_nom[0,0] = np.pi/2

        #q1 = np.radians(np.array([0]*39).reshape((-1,1)))
        #q2 = np.radians(np.array([10]*39).reshape((-1,1)))
        self.publisher_ = node.create_publisher(Int32MultiArray, 'collision', 10)

        self.chain.setjoints(self.q)
        #self.segments = [Hold(self.q, 1.0),GotoCubic(q1, q2, 1.0),GotoCubic(q2, q1, 1.0)]
        self.t0 = 0
        self.cyclic = True
        self.goal_list = [[-1.35,1,0],[-1.35,-1,0]]

        self.lam = 10
        self.collision_count = 0
        self.colliding = False

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        j = []
        for ind in range(1,42):
            j.append('joint%i'%ind)
        return j

    def set_goal(self, pos):
        self.goal_list = []
        for p in pos:
            self.goal_list.append([p.x,p.y,p.z])

        #print(self.goal_list)
    
    # Evaluate at the given time.
    def evaluate(self, tabsolute, dt):

        qdot = np.zeros((41,1))

        pd_all = np.empty((0,1))
        p_tips_all = np.empty((0,1))
        J_all = np.empty((0,41))
        eRR = np.empty((0,1))

        collision_count = 0
        collision_threshold = 0.072
        collision_arr = Int32MultiArray()

        for goal in self.goal_list:
            pd = np.array(goal).reshape((3,1))

            p_links = []
            for dof in range(self.chain.dofs):
                p_links.append(p_from_T(self.chain.data.T[dof]).reshape((3,1)))

            # Calulating the #dof of tip
            p_links = np.array(p_links).squeeze()
            p_links = p_links.T
            tip_dof = np.argmin(np.linalg.norm(pd-p_links,axis=0))

            tip_dis = np.min(np.linalg.norm(pd-p_links,axis=0))
            if tip_dis<collision_threshold:# and tip_dof>=10:
                collision_count +=1
                collision_arr.data.append(1)
            else:
                collision_arr.data.append(0)

            #Stack the primary task quantities
            # pd_all = np.append(pd_all,pd[:2,:],axis=0)

            # p_tips_all = np.append(p_tips_all, p_from_T(self.chain.data.T[tip_dof]).reshape((3,1))[:2,:],axis=0)

            pd_tip = p_from_T(self.chain.data.T[tip_dof]).reshape((3,1))

            if pd[2][0]<pd_tip[2][0]:
                continue

            eR1 = pd[:2,:]
            eR2 = pd_tip[:2,:]
            eR12 = ep(eR2,eR1)
            eRn = np.linalg.norm(eR12)
            eR12 /= eRn**2
            eRR = np.append(eRR, 0.3*eR12, axis=0)
            J_all = np.append(J_all,self.chain.Jv_tip(tip_dof)[:2,:],axis=0)

            # gradient_threshold = 1
            # if eRn<gradient_threshold:
            #     eR12 /= eRn
            #     eR12 *= np.sin(eRn*np.pi/gradient_threshold)
            #     eRR = np.append(eRR, 0.3*eR12, axis=0)
            #     J_all = np.append(J_all,self.chain.Jv_tip(tip_dof)[:2,:],axis=0)
            # else:
            #     continue

        if collision_count>0 and not self.colliding: 
            print('collision detected: #',collision_count)
            self.collision_count += collision_count
            self.colliding = True
        if collision_count>0 and self.colliding:
            self.colliding = False
        collision_arr.data.insert(0,self.collision_count)
        self.publisher_.publish(collision_arr)
            
        Jinv = np.linalg.pinv(J_all,rcond=0.1)
        
        qdot = Jinv @ (self.lam*eRR) + (np.eye(J_all.shape[1])- Jinv @ J_all) @ (self.lam*(self.q_nom - self.q))

        q = self.q + qdot*dt
        self.q = q
        self.chain.setjoints(self.q)

        return (q.flatten().tolist(), qdot.flatten().tolist())


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
