"""
Author: lz 20220201
Location: HKCLR 17W
"""
import codecs
import numpy as np
import matplotlib.pyplot as plt
class Draw_plot():
    def __init__(self):
        self.f = codecs.open('/data/BuyCode/Cheetah_Software_24Nm_selfIMU/build/log_jump.txt', mode='r', encoding='utf-8')
        # self.f = codecs.open('/home/baozi/log_jump.txt', mode='r', encoding='utf-8')
        # self.data={}
        self.iter=0
        self.iter_time=[]
        #Leg0
        self.leg_0_q0=[]
        self.leg_0_q1=[]
        self.leg_0_q2=[]
        self.leg_0_qd0=[]
        self.leg_0_qd1=[]
        self.leg_0_qd2=[]
        self.leg_0_tau0=[]
        self.leg_0_tau1=[]
        self.leg_0_tau2=[]
        self.leg_0_p0=[]
        self.leg_0_p1=[]
        self.leg_0_p2=[]
        #Leg3
        self.leg_3_q0=[]
        self.leg_3_q1=[]
        self.leg_3_q2=[]
        self.leg_3_qd0=[]
        self.leg_3_qd1=[]
        self.leg_3_qd2=[]
        self.leg_3_tau0=[]
        self.leg_3_tau1=[]
        self.leg_3_tau2=[]
        self.leg_3_p0=[]
        self.leg_3_p1=[]
        self.leg_3_p2=[]
        #leg1
        self.leg_1_q0=[]
        self.leg_1_q1=[]
        self.leg_1_q2=[]
        self.leg_1_qd0=[]
        self.leg_1_qd1=[]
        self.leg_1_qd2=[]
        self.leg_1_tau0=[]
        self.leg_1_tau1=[]
        self.leg_1_tau2=[]
        self.leg_1_p0=[]
        self.leg_1_p1=[]
        self.leg_1_p2=[]
        #Leg2
        self.leg_2_q0=[]
        self.leg_2_q1=[]
        self.leg_2_q2=[]
        self.leg_2_qd0=[]
        self.leg_2_qd1=[]
        self.leg_2_qd2=[]
        self.leg_2_tau0=[]
        self.leg_2_tau1=[]
        self.leg_2_tau2=[]
        self.leg_2_p0=[] #in body
        self.leg_2_p1=[]
        self.leg_2_p2=[]
        #Leg0 desire
        self.leg_0_qcmd0=[]
        self.leg_0_qcmd1=[]
        self.leg_0_qcmd2=[]
        #Leg1 desire
        self.leg_1_qcmd0=[]
        self.leg_1_qcmd1=[]
        self.leg_1_qcmd2=[]
        #Leg2 desire
        self.leg_2_qcmd0=[]
        self.leg_2_qcmd1=[]
        self.leg_2_qcmd2=[]
        #Leg3 Desire
        self.leg_3_qcmd0=[]
        self.leg_3_qcmd1=[]
        self.leg_3_qcmd2=[]
        #body pos and orien
        self.body_v_x_w=[]
        self.body_v_y_w=[]
        self.body_v_z_w=[]
        self.body_omega_x_w=[]
        self.body_omega_y_w=[]
        self.body_omega_z_w=[]

        self.body_p_x_w=[]
        self.body_p_y_w=[]
        self.body_p_z_w=[]
        self.body_roll_x_w=[]
        self.body_pitch_y_w=[]
        self.body_yaw_z_w=[]
        #in world frame
        self.leg_0_p0w=[]
        self.leg_0_p1w=[]
        self.leg_0_p2w=[]
        self.leg_1_p0w=[]
        self.leg_1_p1w=[]
        self.leg_1_p2w=[]
        #in body frame ref
        self.leg_0_pref0b=[]
        self.leg_0_pref1b=[]
        self.leg_0_pref2b=[]
        self.leg_1_pref0b=[]
        self.leg_1_pref1b=[]
        self.leg_1_pref2b=[]
        #in body frame pfoot vdes
        self.leg_0_vref0b=[]
        self.leg_0_vref1b=[]
        self.leg_0_vref2b=[]
        self.leg_3_vref0b=[]
        self.leg_3_vref1b=[]
        self.leg_3_vref2b=[]
        #velocity in real body
        self.leg_0_v0w=[]
        self.leg_0_v1w=[]
        self.leg_0_v2w=[]
        self.leg_3_v0w=[]
        self.leg_3_v1w=[]
        self.leg_3_v2w=[]
        #velocity in qdes
        self.leg_0_qdcmd0=[]
        self.leg_0_qdcmd1=[]
        self.leg_0_qdcmd2=[]
        self.leg_1_qdcmd0=[]
        self.leg_1_qdcmd1=[]
        self.leg_1_qdcmd2=[]
        self.leg_2_qdcmd0=[]
        self.leg_2_qdcmd1=[]
        self.leg_2_qdcmd2=[]
        self.leg_3_qdcmd0=[]
        self.leg_3_qdcmd1=[]
        self.leg_3_qdcmd2=[]
        self.read_data_from_file()

    def read_data_from_file(self):
        line = self.f.readline()
        while line:
            newline = line.split()
            line = self.f.readline()
            iter_list=[float(item) for item in newline]
            self.iter_time.append(iter_list[0])
            self.leg_0_q0.append(iter_list[1])
            self.leg_0_q1.append(iter_list[2])
            self.leg_0_q2.append(iter_list[3])
            self.leg_0_qd0.append(iter_list[4])
            self.leg_0_qd1.append(iter_list[5])
            self.leg_0_qd2.append(iter_list[6])
            self.leg_0_tau0.append(iter_list[7])
            self.leg_0_tau1.append(iter_list[8])
            self.leg_0_tau2.append(iter_list[9])
            self.leg_0_p0.append(iter_list[10])
            self.leg_0_p1.append(iter_list[11])
            self.leg_0_p2.append(iter_list[12])
            #leg3
            self.leg_3_q0.append(iter_list[13])
            self.leg_3_q1.append(iter_list[14])
            self.leg_3_q2.append(iter_list[15])
            self.leg_3_qd0.append(iter_list[16])
            self.leg_3_qd1.append(iter_list[17])
            self.leg_3_qd2.append(iter_list[18])
            self.leg_3_tau0.append(iter_list[19])
            self.leg_3_tau1.append(iter_list[20])
            self.leg_3_tau2.append(iter_list[21])
            self.leg_3_p0.append(iter_list[22])
            self.leg_3_p1.append(iter_list[23])
            self.leg_3_p2.append(iter_list[24])

            #leg1
            self.leg_1_q0.append(iter_list[25])
            self.leg_1_q1.append(iter_list[26])
            self.leg_1_q2.append(iter_list[27])
            self.leg_1_qd0.append(iter_list[28])
            self.leg_1_qd1.append(iter_list[29])
            self.leg_1_qd2.append(iter_list[30])
            self.leg_1_tau0.append(iter_list[31])
            self.leg_1_tau1.append(iter_list[32])
            self.leg_1_tau2.append(iter_list[33])
            self.leg_1_p0.append(iter_list[34])
            self.leg_1_p1.append(iter_list[35])
            self.leg_1_p2.append(iter_list[36])
            #leg2
            self.leg_2_q0.append(iter_list[37])
            self.leg_2_q1.append(iter_list[38])
            self.leg_2_q2.append(iter_list[39])
            self.leg_2_qd0.append(iter_list[40])
            self.leg_2_qd1.append(iter_list[41])
            self.leg_2_qd2.append(iter_list[42])
            self.leg_2_tau0.append(iter_list[43])
            self.leg_2_tau1.append(iter_list[44])
            self.leg_2_tau2.append(iter_list[45])
            self.leg_2_p0.append(iter_list[46])
            self.leg_2_p1.append(iter_list[47])
            self.leg_2_p2.append(iter_list[48])
            self.leg_0_qcmd0.append(iter_list[49])
            self.leg_0_qcmd1.append(iter_list[50])
            self.leg_0_qcmd2.append(iter_list[51])
            #Leg3 Desire
            self.leg_3_qcmd0.append(iter_list[52])
            self.leg_3_qcmd1.append(iter_list[53])
            self.leg_3_qcmd2.append(iter_list[54])
            #body pos and orien

            self.body_p_x_w.append(iter_list[55])
            self.body_p_y_w.append(iter_list[56])
            self.body_p_z_w.append(iter_list[57])
            self.body_roll_x_w.append(iter_list[58])
            self.body_pitch_y_w.append(iter_list[59])
            self.body_yaw_z_w.append(iter_list[60])
            self.body_omega_x_w.append(iter_list[61])
            self.body_omega_y_w.append(iter_list[62])
            self.body_omega_z_w.append(iter_list[63])
            self.body_v_x_w.append(iter_list[64])
            self.body_v_y_w.append(iter_list[65])
            self.body_v_z_w.append(iter_list[66])

            # self.leg_0_p0w.append(iter_list[67])
            # self.leg_0_p1w.append(iter_list[68])
            # self.leg_0_p2w.append(iter_list[69])
            # self.leg_1_p0w.append(iter_list[70])
            # self.leg_1_p1w.append(iter_list[71])
            # self.leg_1_p2w.append(iter_list[72])

            self.leg_0_pref0b.append(iter_list[67])
            self.leg_0_pref1b.append(iter_list[68])
            self.leg_0_pref2b.append(iter_list[69])
            self.leg_1_pref0b.append(iter_list[70])
            self.leg_1_pref1b.append(iter_list[71])
            self.leg_1_pref2b.append(iter_list[72])
            self.leg_0_vref0b.append(iter_list[73])
            self.leg_0_vref1b.append(iter_list[74])
            self.leg_0_vref2b.append(iter_list[75])
            self.leg_3_vref0b.append(iter_list[76])
            self.leg_3_vref1b.append(iter_list[77])
            self.leg_3_vref2b.append(iter_list[78])
            #velocity in real
            self.leg_0_v0w.append(iter_list[79])
            self.leg_0_v1w.append(iter_list[80])
            self.leg_0_v2w.append(iter_list[81])
            self.leg_3_v0w.append(iter_list[82])
            self.leg_3_v1w.append(iter_list[83])
            self.leg_3_v2w.append(iter_list[84])
            self.leg_1_qcmd0.append(iter_list[85])
            self.leg_1_qcmd1.append(iter_list[86])
            self.leg_1_qcmd2.append(iter_list[87])
            #Leg2 desire
            self.leg_2_qcmd0.append(iter_list[88])
            self.leg_2_qcmd1.append(iter_list[89])
            self.leg_2_qcmd2.append(iter_list[90])
            #velocity qdes
            self.leg_0_qdcmd0.append(iter_list[91])
            self.leg_0_qdcmd1.append(iter_list[92])
            self.leg_0_qdcmd2.append(iter_list[93])
            self.leg_1_qdcmd0.append(iter_list[94])
            self.leg_1_qdcmd1.append(iter_list[95])
            self.leg_1_qdcmd2.append(iter_list[96])
            self.leg_2_qdcmd0.append(iter_list[97])
            self.leg_2_qdcmd1.append(iter_list[98])
            self.leg_2_qdcmd2.append(iter_list[99])
            self.leg_3_qdcmd0.append(iter_list[100])
            self.leg_3_qdcmd1.append(iter_list[101])
            self.leg_3_qdcmd2.append(iter_list[102])
            # self.data.update({self.iter: np.array(iter_list)})
            # print(self.iter,newline)
            self.iter+=1
        self.f.close()
    def plot_data(self):

        fig = plt.figure(1)
        ax = fig.add_subplot(221)
        ax.plot(self.iter_time,self.body_p_x_w,color="blue",lw=2,label="body pos x")
        ax.plot(self.iter_time,self.body_p_y_w,color="red",lw=2,label="body pos y")
        ax.plot(self.iter_time,self.body_p_z_w,color="black",lw=2,label="body pos z")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,1], title='Body Pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(222)
        ax.plot(self.iter_time,self.body_v_x_w,color="blue",lw=2,label="body vx")
        ax.plot(self.iter_time,self.body_v_y_w,color="red",lw=2,label="body vy")
        ax.plot(self.iter_time,self.body_v_z_w,color="black",lw=2,label="body vz")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Body velocity',ylabel='Y-m/s', xlabel='X-time(s)')
        ax = fig.add_subplot(223)
        ax.plot(self.iter_time,self.body_roll_x_w,color="blue",lw=2,label="body roll")
        ax.plot(self.iter_time,self.body_pitch_y_w,color="red",lw=2,label="body pitch")
        ax.plot(self.iter_time,self.body_yaw_z_w,color="black",lw=2,label="body yaw")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Body ori',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(224)
        ax.plot(self.iter_time,self.body_omega_x_w,color="blue",lw=2,label="body omega x")
        ax.plot(self.iter_time,self.body_omega_y_w,color="red",lw=2,label="body omega y")
        ax.plot(self.iter_time,self.body_omega_z_w,color="black",lw=2,label="body omega z")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Body omega',ylabel='Y-rad/s', xlabel='X-time(s)')
        # plt.show()

        fig = plt.figure(2)
        fig.subplots_adjust(hspace=.5)
        ax = fig.add_subplot(331)
        ax.plot(self.iter_time,self.leg_0_q0,color="black",lw=2,label="leg 0 q0")
        ax.plot(self.iter_time,self.leg_0_qcmd0,color="red",lw=2,label="leg 0 q0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg0 q0 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(332)
        ax.plot(self.iter_time,self.leg_0_q1,color="black",lw=2,label="leg0 q1")
        ax.plot(self.iter_time,self.leg_0_qcmd1,color="red",lw=2,label="leg0 q1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,0.3], title='Leg0 q1 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(333)
        ax.plot(self.iter_time,self.leg_0_q2,color="black",lw=2,label="leg 0 q2")
        ax.plot(self.iter_time,self.leg_0_qcmd2,color="red",lw=2,label="leg 0 q2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,3], title='Leg0 q2 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(334)
        ax.plot(self.iter_time,self.leg_0_p0,color="black",lw=2,label="Leg0 px")
        ax.plot(self.iter_time,self.leg_0_p0,color="red",lw=2,label="Leg0 prefx",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg0 Pfootx pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(335)
        ax.plot(self.iter_time,self.leg_0_p1,color="black",lw=2,label="Leg0 prefx")
        ax.plot(self.iter_time,self.leg_0_p1,color="red",lw=2,label="Leg0 prefy",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg0 Pfooty pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(336)
        ax.plot(self.iter_time,self.leg_0_p2,color="black",lw=2,label="Leg0 prefz")
        ax.plot(self.iter_time,self.leg_0_p2,color="red",lw=2,label="Leg0 prefz",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,0], title='Leg0 Pfootz pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(337)
        ax.plot(self.iter_time,self.leg_0_tau0,color="black",lw=2,label="Leg_0 tau0")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-25,25], title='Leg0 tau0',ylabel='Y-N/s', xlabel='X-time(s)')
        ax = fig.add_subplot(338)
        ax.plot(self.iter_time,self.leg_0_tau1,color="red",lw=2,label="Leg_0 tau1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-25,25], title='Leg0 tau1',ylabel='Y-N/s', xlabel='X-time(s)')
        ax = fig.add_subplot(339)
        ax.plot(self.iter_time,self.leg_0_tau2,color="red",lw=2,label="Leg_0 tau2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-22,22], title='Leg0 tau2',ylabel='Y-N/s', xlabel='X-time(s)')
        fig = plt.figure(3)
        fig.subplots_adjust(hspace=.5)
        ax = fig.add_subplot(331)
        ax.plot(self.iter_time,self.leg_1_q0,color="black",lw=2,label="leg 1 q0")
        ax.plot(self.iter_time,self.leg_1_qcmd0,color="red",lw=2,label="leg 1 q0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg1 q0 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(332)
        ax.plot(self.iter_time,self.leg_1_q1,color="black",lw=2,label="leg1 q1")
        ax.plot(self.iter_time,self.leg_1_qcmd1,color="red",lw=2,label="leg1 q1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,0.3], title='Leg1 q1 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(333)
        ax.plot(self.iter_time,self.leg_1_q2,color="black",lw=2,label="leg 1 q2")
        ax.plot(self.iter_time,self.leg_1_qcmd2,color="red",lw=2,label="leg 1 q2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,3], title='Leg1 q2 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(334)
        ax.plot(self.iter_time,self.leg_1_p0,color="black",lw=2,label="Leg1 px")
        ax.plot(self.iter_time,self.leg_1_p0,color="red",lw=2,label="Leg1 prefx",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg1 Pfootx pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(335)
        ax.plot(self.iter_time,self.leg_1_p1,color="black",lw=2,label="Leg1 prefx")
        ax.plot(self.iter_time,self.leg_1_p1,color="red",lw=2,label="Leg1 prefy",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg1 Pfooty pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(336)
        ax.plot(self.iter_time,self.leg_1_p2,color="black",lw=2,label="Leg1 prefz")
        ax.plot(self.iter_time,self.leg_1_p2,color="red",lw=2,label="Leg1 prefz",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,0], title='Leg1 Pfootz pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(337)
        ax.plot(self.iter_time,self.leg_1_tau0,color="black",lw=2,label="Leg_1 tau0")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-25,25], title='Leg1 tau0',ylabel='Y-N/s', xlabel='X-time(s)')
        ax = fig.add_subplot(338)
        ax.plot(self.iter_time,self.leg_1_tau1,color="red",lw=2,label="Leg_1 tau1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-25,25], title='Leg1 tau1',ylabel='Y-N/s', xlabel='X-time(s)')
        ax = fig.add_subplot(339)
        ax.plot(self.iter_time,self.leg_1_tau2,color="red",lw=2,label="Leg_1 tau2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-22,22], title='Leg1 tau2',ylabel='Y-N/s', xlabel='X-time(s)')
        fig = plt.figure(4)
        fig.subplots_adjust(hspace=.5)
        ax = fig.add_subplot(331)
        ax.plot(self.iter_time,self.leg_2_q0,color="black",lw=2,label="leg 2 q0")
        ax.plot(self.iter_time,self.leg_2_qcmd0,color="red",lw=2,label="leg 2 q0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg2 q0 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(332)
        ax.plot(self.iter_time,self.leg_2_q1,color="black",lw=2,label="leg2 q1")
        ax.plot(self.iter_time,self.leg_2_qcmd1,color="red",lw=2,label="leg2 q1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,0.3], title='Leg2 q1 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(333)
        ax.plot(self.iter_time,self.leg_2_q2,color="black",lw=2,label="leg 2 q2")
        ax.plot(self.iter_time,self.leg_2_qcmd2,color="red",lw=2,label="leg 2 q2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,3], title='Leg2 q2 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(334)
        ax.plot(self.iter_time,self.leg_2_p0,color="black",lw=2,label="Leg2 px")
        ax.plot(self.iter_time,self.leg_2_p0,color="red",lw=2,label="Leg2 prefx",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg2 Pfootx pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(335)
        ax.plot(self.iter_time,self.leg_2_p1,color="black",lw=2,label="Leg2 prefx")
        ax.plot(self.iter_time,self.leg_2_p1,color="red",lw=2,label="Leg2 prefy",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg2 Pfooty pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(336)
        ax.plot(self.iter_time,self.leg_2_p2,color="black",lw=2,label="Leg2 prefz")
        ax.plot(self.iter_time,self.leg_2_p2,color="red",lw=2,label="Leg2 prefz",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,0], title='Leg2 Pfootz pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(337)
        ax.plot(self.iter_time,self.leg_2_tau0,color="black",lw=2,label="Leg_2 tau0")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-25,25], title='Leg2 tau0',ylabel='Y-N/s', xlabel='X-time(s)')
        ax = fig.add_subplot(338)
        ax.plot(self.iter_time,self.leg_2_tau1,color="red",lw=2,label="Leg_2 tau1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-25,25], title='Leg2 tau1',ylabel='Y-N/s', xlabel='X-time(s)')
        ax = fig.add_subplot(339)
        ax.plot(self.iter_time,self.leg_2_tau2,color="red",lw=2,label="Leg_2 tau2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-22,22], title='Leg2 tau2',ylabel='Y-N/s', xlabel='X-time(s)')
        fig = plt.figure(5)
        fig.subplots_adjust(hspace=.5)
        ax = fig.add_subplot(331)
        ax.plot(self.iter_time,self.leg_3_q0,color="black",lw=2,label="leg_3 q0")
        ax.plot(self.iter_time,self.leg_3_qcmd0,color="red",lw=2,label="leg_3 q0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg_3 q0 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(332)
        ax.plot(self.iter_time,self.leg_3_q1,color="black",lw=2,label="leg_3 q1")
        ax.plot(self.iter_time,self.leg_3_qcmd1,color="red",lw=2,label="leg_3 qref1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,0.3], title='Leg_3 q1 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(333)
        ax.plot(self.iter_time,self.leg_3_q2,color="black",lw=2,label="leg_1 q2")
        ax.plot(self.iter_time,self.leg_3_qcmd2,color="red",lw=2,label="leg_1 qref2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,3], title='Leg_1 q2 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(334)
        ax.plot(self.iter_time,self.leg_3_p0,color="black",lw=2,label="Leg0 px")
        ax.plot(self.iter_time,self.leg_3_p0,color="red",lw=2,label="Leg0 prefx",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.4,0.4], title='Leg_3 Pfootx pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(335)
        ax.plot(self.iter_time,self.leg_3_p1,color="black",lw=2,label="Leg_3 py")
        ax.plot(self.iter_time,self.leg_3_p1,color="red",lw=2,label="Leg_3 prefy",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.4,0.4], title='Leg_3 Pfooty pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(336)
        ax.plot(self.iter_time,self.leg_3_p2,color="black",lw=2,label="Leg_3 pz")
        ax.plot(self.iter_time,self.leg_3_p2,color="red",lw=2,label="Leg_3 prefz",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,0], title='Leg_3 Pfootz pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(337)
        ax.plot(self.iter_time,self.leg_3_tau0,color="black",lw=2,label="Leg_3 tau0")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-25,25], title='Leg_3 tau0',ylabel='Y-N/s', xlabel='X-time(s)')
        ax = fig.add_subplot(338)
        ax.plot(self.iter_time,self.leg_3_tau1,color="red",lw=2,label="Leg_3 tau1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-25,25], title='Leg_3 tau1',ylabel='Y-N/s', xlabel='X-time(s)')
        ax = fig.add_subplot(339)
        ax.plot(self.iter_time,self.leg_3_tau2,color="red",lw=2,label="Leg_3 tau2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-22,22], title='Leg_3 tau2',ylabel='Y-N/s', xlabel='X-time(s)')
        fig = plt.figure(6)
        fig.subplots_adjust(hspace=.5)
        ax = fig.add_subplot(231)
        ax.plot(self.iter_time,self.leg_0_p0,color="blue",lw=2,label="Leg0 px in body")
        ax.plot(self.iter_time,self.leg_0_pref0b,color="red",lw=2,label="Leg0 prefx in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,2], title='Leg_0 Pfootb x pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(232)
        ax.plot(self.iter_time,self.leg_0_p1,color="blue",lw=2,label="Leg0 py in body")
        ax.plot(self.iter_time,self.leg_0_pref1b,color="red",lw=2,label="Leg0 prefy in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,2], title='Leg_0 Pfootb y pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(233)
        ax.plot(self.iter_time,self.leg_0_p2,color="blue",lw=2,label="Leg0 pz in body")
        ax.plot(self.iter_time,self.leg_0_pref2b,color="red",lw=2,label="Leg0 prefz in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,0.1], title='Leg_0 Pfootb z pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(234)
        ax.plot(self.iter_time,self.leg_1_p0,color="blue",lw=2,label="Leg_1 px in body")
        ax.plot(self.iter_time,self.leg_1_pref0b,color="red",lw=2,label="Leg_1 prefx in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.4,0.4], title='Leg_1 Pfootb x pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(235)
        ax.plot(self.iter_time,self.leg_1_p1,color="blue",lw=2,label="Leg_1 py in body")
        ax.plot(self.iter_time,self.leg_1_pref1b,color="red",lw=2,label="Leg_1 prefy in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.4,0.4], title='Leg_1 Pfootb y pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(236)
        ax.plot(self.iter_time,self.leg_1_p2,color="blue",lw=2,label="Leg_1 pz in body")
        ax.plot(self.iter_time,self.leg_1_pref2b,color="red",lw=2,label="Leg_1 prefz in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,0.], title='Leg_1 Pfootb z pos',ylabel='Y-m', xlabel='X-time(s)')
        fig = plt.figure(7)
        fig.subplots_adjust(hspace=.5)
        ax = fig.add_subplot(231)
        ax.plot(self.iter_time,self.leg_0_v0w,color="blue",lw=2,label="Leg0 pdot-x in body")
        ax.plot(self.iter_time,self.leg_0_vref0b,color="red",lw=2,label="Leg0 pdot-refx in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,2], title='Leg_0 Pfootb dot x pos',ylabel='Y-m/s', xlabel='X-time(s)')
        ax = fig.add_subplot(232)
        ax.plot(self.iter_time,self.leg_0_v1w,color="blue",lw=2,label="Leg0 pdot-y in body")
        ax.plot(self.iter_time,self.leg_0_vref1b,color="red",lw=2,label="Leg0 pdot-refy in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,2], title='Leg_0 Pfootb dot y pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(233)
        ax.plot(self.iter_time,self.leg_0_v2w,color="blue",lw=2,label="Leg0 pdot-z in body")
        ax.plot(self.iter_time,self.leg_0_vref2b,color="red",lw=2,label="Leg0 pdot-refz in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1.5,1.5], title='Leg_0 Pfootb dot z pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(234)
        ax.plot(self.iter_time,self.leg_3_v0w,color="blue",lw=2,label="Leg_3 pdot-x in body")
        ax.plot(self.iter_time,self.leg_3_vref0b,color="red",lw=2,label="Leg_3 pdot-refx in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,2], title='Leg_3 Pfootb dot x pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(235)
        ax.plot(self.iter_time,self.leg_3_v1w,color="blue",lw=2,label="Leg_3 pdot-y in body")
        ax.plot(self.iter_time,self.leg_3_vref1b,color="red",lw=2,label="Leg_3 pdot-refy in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,2], title='Leg_3 Pfootb dot y pos',ylabel='Y-m', xlabel='X-time(s)')
        ax = fig.add_subplot(236)
        ax.plot(self.iter_time,self.leg_3_v2w,color="blue",lw=2,label="Leg_3 pdot-z in body")
        ax.plot(self.iter_time,self.leg_3_vref2b,color="red",lw=2,label="Leg_3 pdot-refz in body",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1.5,1.5], title='Leg_3 Pfootb dot z pos',ylabel='Y-m', xlabel='X-time(s)')
        fig = plt.figure(8)
        fig.subplots_adjust(hspace=.5)
        ax = fig.add_subplot(431)
        ax.plot(self.iter_time,self.leg_0_qd0,color="black",lw=2,label="leg_0 qd0")
        ax.plot(self.iter_time,self.leg_0_qdcmd0,color="red",lw=2,label="leg_0 qdcmd0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg0 q0 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(432)
        ax.plot(self.iter_time,self.leg_0_qd1,color="black",lw=2,label="leg0 qd1")
        ax.plot(self.iter_time,self.leg_0_qdcmd1,color="red",lw=2,label="leg0 qdcmd1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg0 q1 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(433)
        ax.plot(self.iter_time,self.leg_0_qd2,color="black",lw=2,label="leg_0 qd2")
        ax.plot(self.iter_time,self.leg_0_qdcmd2,color="red",lw=2,label="leg_0 qdcmd2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg0 q2 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(434)
        ax.plot(self.iter_time,self.leg_1_qd0,color="black",lw=2,label="leg_1 qd0")
        ax.plot(self.iter_time,self.leg_1_qdcmd0,color="red",lw=2,label="leg_1 qdcmd0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg1 q0 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(435)
        ax.plot(self.iter_time,self.leg_1_qd1,color="black",lw=2,label="leg1 qd1")
        ax.plot(self.iter_time,self.leg_1_qdcmd1,color="red",lw=2,label="leg1 qdcmd1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg1 q1 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(436)
        ax.plot(self.iter_time,self.leg_1_qd2,color="black",lw=2,label="leg 1 qd2")
        ax.plot(self.iter_time,self.leg_1_qdcmd2,color="red",lw=2,label="leg 1 qdcmd2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg1 q2 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(437)
        ax.plot(self.iter_time,self.leg_2_qd0,color="black",lw=2,label="leg 2 qd0")
        ax.plot(self.iter_time,self.leg_2_qdcmd0,color="red",lw=2,label="leg 2 qdcmd0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg2 q0 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(438)
        ax.plot(self.iter_time,self.leg_2_qd1,color="black",lw=2,label="leg2 qd1")
        ax.plot(self.iter_time,self.leg_2_qdcmd1,color="red",lw=2,label="leg2 qdcmd1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg2 q1 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(439)
        ax.plot(self.iter_time,self.leg_2_qd2,color="black",lw=2,label="leg 2 qd2")
        ax.plot(self.iter_time,self.leg_2_qdcmd2,color="red",lw=2,label="leg 2 qdcmd2",linestyle="--")

        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg2 q2 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(4,3,10)
        ax.plot(self.iter_time,self.leg_3_qd0,color="black",lw=2,label="leg_3 qd0")
        ax.plot(self.iter_time,self.leg_3_qdcmd0,color="red",lw=2,label="leg_3 q0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg_3 q0 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(4,3,11)
        ax.plot(self.iter_time,self.leg_3_qd1,color="black",lw=2,label="leg_3 qd1")
        ax.plot(self.iter_time,self.leg_3_qdcmd1,color="red",lw=2,label="leg_3 qref1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg_3 q1 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')
        ax = fig.add_subplot(4,3,12)
        ax.plot(self.iter_time,self.leg_3_qd2,color="black",lw=2,label="leg_3 qd2")
        ax.plot(self.iter_time,self.leg_3_qdcmd2,color="red",lw=2,label="leg_3 qdcmd2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-46,46], title='Leg_3 q2 joint velocity',ylabel='Y-rad/s', xlabel='X-time(s)')


        fig = plt.figure(9)
        fig.subplots_adjust(hspace=.5)
        ax = fig.add_subplot(431)
        ax.plot(self.iter_time,self.leg_0_q0,color="black",lw=2,label="leg_0 q0")
        ax.plot(self.iter_time,self.leg_0_qcmd0,color="red",lw=2,label="leg_0 qcmd0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg0 q0 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(432)
        ax.plot(self.iter_time,self.leg_0_q1,color="black",lw=2,label="leg_0 q1")
        ax.plot(self.iter_time,self.leg_0_qcmd1,color="red",lw=2,label="leg_0 qcmd1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,0.3], title='Leg0 q1 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(433)
        ax.plot(self.iter_time,self.leg_0_q2,color="black",lw=2,label="leg_0 q2")
        ax.plot(self.iter_time,self.leg_0_qcmd2,color="red",lw=2,label="leg_0 qcmd2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,3], title='Leg0 q2 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(434)
        ax.plot(self.iter_time,self.leg_1_q0,color="black",lw=2,label="leg_1 q0")
        ax.plot(self.iter_time,self.leg_1_qcmd0,color="red",lw=2,label="leg_1 qcmd0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg1 q0 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(435)
        ax.plot(self.iter_time,self.leg_1_q1,color="black",lw=2,label="leg_1 q1")
        ax.plot(self.iter_time,self.leg_1_qcmd1,color="red",lw=2,label="leg_1 qcmd1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,0.3], title='Leg1 q1 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(436)
        ax.plot(self.iter_time,self.leg_1_q2,color="black",lw=2,label="leg_1 q2")
        ax.plot(self.iter_time,self.leg_1_qcmd2,color="red",lw=2,label="leg_1 qcmd2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,3], title='Leg1 q2 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(437)
        ax.plot(self.iter_time,self.leg_2_q0,color="black",lw=2,label="leg_2 q0")
        ax.plot(self.iter_time,self.leg_2_qcmd0,color="red",lw=2,label="leg_2 qcmd0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg2 q0 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(438)
        ax.plot(self.iter_time,self.leg_2_q1,color="black",lw=2,label="leg_2 q1")
        ax.plot(self.iter_time,self.leg_2_qcmd1,color="red",lw=2,label="leg_2 qcmd1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,0.3], title='Leg2 q1 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(439)
        ax.plot(self.iter_time,self.leg_2_q2,color="black",lw=2,label="leg_2 q2")
        ax.plot(self.iter_time,self.leg_2_qcmd2,color="red",lw=2,label="leg_2 qcmd2",linestyle="--")

        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,3], title='Leg2 q2 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(4,3,10)
        ax.plot(self.iter_time,self.leg_3_q0,color="black",lw=2,label="leg_3 q0")
        ax.plot(self.iter_time,self.leg_3_qcmd0,color="red",lw=2,label="leg_3 qcmd0",linestyle=":")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-1,1], title='Leg_3 q0 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(4,3,11)
        ax.plot(self.iter_time,self.leg_3_q1,color="black",lw=2,label="leg_3 q1")
        ax.plot(self.iter_time,self.leg_3_qcmd1,color="red",lw=2,label="leg_3 qcmd1",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-2,0.3], title='Leg_3 q1 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        ax = fig.add_subplot(4,3,12)
        ax.plot(self.iter_time,self.leg_3_q2,color="black",lw=2,label="leg_1 q2")
        ax.plot(self.iter_time,self.leg_3_qcmd2,color="red",lw=2,label="leg_1 qcmd2",linestyle="--")
        ax.legend(loc='upper right',borderaxespad=0.0)
        ax.set(xlim=[0,5], ylim=[-0.5,3], title='Leg_1 q2 joint angular',ylabel='Y-rad', xlabel='X-time(s)')
        plt.show()
def main():
    dp=Draw_plot()
    dp.plot_data()
    # dp.read_data_from_file()
    # print(dp.data)
if __name__ == '__main__':
    main()

