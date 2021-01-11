
import math
import matplotlib.pyplot as plt

## define a path segment class for P2P motion
class PathP2P:
    def __init__(self,qi_vec,qf_vec,qdmax_vec,qddmax_vec):
        # final point
        self.qf_vec=qf_vec

        # initial point
        self.qi_vec=qi_vec

        # max velocity
        self.qdmax_vec=qdmax_vec

        # max acceleration
        self.qddmax_vec=qddmax_vec

        # get trajectory using trapezoidal method
        self.tc_vec,self.qd_vec,self.tf=self.__get_traj()


    # calculate parabolic segment time
    def __get_tc(self,tf):
        if tf:
            tc_vec = [0.5 * tf - 0.5 * math.sqrt(abs((tf * tf * qdd - 4 * (qf - qi)) / qdd)) for qdd,qf,qi in zip(self.qddmax_vec,self.qf_vec,self.qi_vec)]
        else:
            tc_vec = [qd/qdd if qd*qd/qdd<=(qf-qi) else math.sqrt((qf-qi)/qdd) for qd,qdd,qf,qi in zip(self.qdmax_vec,self.qddmax_vec,self.qf_vec,self.qi_vec)]


        return tc_vec

    # calculates final time for this p2p motion
    def __get_tf(self):
        tc_vec = self.__get_tc([])
        tf_vec=[(qdd*tc*tc+qf-qi)/(qdd*tc+1e-3) for tc,qd, qdd, qf, qi in zip(tc_vec,self.qdmax_vec, self.qddmax_vec, self.qf_vec, self.qi_vec)]
        return tf_vec

    def __get_traj(self):
        # all joints are synchronized with the slowest joint
        tf_vec = self.__get_tf()
        tf = max(tf_vec)
        # keep max accel fixed and adjust max velocity to synchronize all joints
        tc_vec = self.__get_tc(tf)
        qd_vec=[tc*qdd for tc,qdd in zip(tc_vec,self.qddmax_vec)]
        return tc_vec,qd_vec,tf

    def query(self,t):
        q_vec=[0]* len(self.qi_vec)


        for i in range(len(self.qi_vec)):
            if 0 <= t <= self.tc_vec[i]:
                q_vec[i]=self.qi_vec[i]+0.5*self.qddmax_vec[i]*t*t
            else:
                if self.tc_vec[i] < t <= self.tf-self.tc_vec[i]:
                    q_vec[i] = self.qi_vec[i]+self.qddmax_vec[i]*self.tc_vec[i]*(t-0.5*self.tc_vec[i])
                else:
                    q_vec[i] = self.qf_vec[i]-0.5*self.qddmax_vec[i]*(self.tf-t)*(self.tf-t)

        return q_vec

# class for the compound P2P path
# points are passed as a 2D list
class MultiSegPath:
    def __init__(self,q_mat,qdmax_vec,qddmax_vec):
        self.path_vec=[]
        for i in range(len(q_mat)-1):
            self.path_vec.append(PathP2P(q_mat[i],q_mat[i+1],qdmax_vec,qddmax_vec))

        self.seg_num=len(self.path_vec)
        self.tf_total_vec=self.get_total_time()

    def get_total_time(self):
        tf_vec=[path.tf for path in self.path_vec]
        tf_total_vec=tf_vec

        for i in range(1,len(tf_vec)):
            tf_total_vec[i]=tf_vec[i]+tf_total_vec[i-1]
        return tf_total_vec

    # path query
    def query(self,t):

        # find and call the corresponding query
        if t<self.tf_total_vec[0]:
            return self.path_vec[0].query(t)

        for i in range(1,len(self.tf_total_vec)):
            if t < self.tf_total_vec[i]:
                return self.path_vec[i].query(t-self.tf_total_vec[i-1])




if __name__=='__main__':
    # Waypoints: [[0.0, 0.0], [0.0, 1.0], [1.0, 2.0]]
    # Velocity
    # limits: [1.0, 2.0]
    # Acceleration
    # limits: [2.5, 3.5]

    # mypath=PathP2P([0.0, 0.0], [1.0, 2.0],[1.0, 2.0],[2.5, 3.5])
    # print('final time: %f sec' %mypath.tf)
    #
    # q_vec=[]
    # t_vec=[]
    # t=0
    # for i in range(20):
    #     t_vec.append(t)
    #     q_vec.append(mypath.query(t))
    #     t+=mypath.tf/19
    #
    # # fig,ax=plt.subplots(2)
    # plt.plot(t_vec,q_vec)
    # plt.show()

    # create a multi segment path object
    myPath = MultiSegPath([[0.0, 0.0], [0.0, 1.0], [1.0, 2.0]],[1.0, 2.0],[2.5, 3.5])
    print('final time for all segments in sec:',myPath.tf_total_vec)
    total_time=myPath.tf_total_vec[-1]

    # plot trajectory
    q_vec=[]
    t_vec=[]
    t=0
    N=40
    for i in range(N):
        t_vec.append(t)
        q_vec.append(myPath.query(t))
        t+=total_time/N
        print(q_vec[i])
    plt.plot(t_vec,q_vec)
    plt.show()


