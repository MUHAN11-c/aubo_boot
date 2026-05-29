"""TF对齐标签页"""
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
DT=0.05

class TfPersistent:
    def __init__(self,base="base_link",ee="tool_tcp"):
        self.base=base;self.ee=ee;self._node=None;self._buffer=None;self._listener=None;self._start()
    def _start(self):
        import rclpy;from tf2_ros import Buffer,TransformListener
        if not rclpy.ok():rclpy.init(args=[])
        self._node=rclpy.create_node("_latte_gui_tf");self._buffer=Buffer(node=self._node);self._listener=TransformListener(self._buffer,self._node)
    def lookup(self,timeout=0.3):
        import rclpy;from geometry_msgs.msg import Pose,Point,Quaternion
        try:
            rclpy.spin_once(self._node,timeout_sec=0.05)
            tfs=self._buffer.lookup_transform(self.base,self.ee,rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=timeout))
            t=tfs.transform.translation;r=tfs.transform.rotation
            return Pose(position=Point(x=t.x,y=t.y,z=t.z),orientation=Quaternion(x=r.x,y=r.y,z=r.z,w=r.w))
        except: return None
    def shutdown(self):
        if self._node is not None:self._node.destroy_node();self._node=None

class TabTF(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent);self.gui=parent
        l=QVBoxLayout(self);l.setContentsMargins(0,0,0,0)
        bar=QHBoxLayout()
        self.lbl_status=QLabel("未获取");self.lbl_status.setStyleSheet("color:gray;font-weight:bold")
        bar.addWidget(QLabel("TF:"));bar.addWidget(self.lbl_status)
        self.btn_fetch=QPushButton("获取末端位姿");bar.addWidget(self.btn_fetch)
        self.btn_retarget=QPushButton("轨迹对齐");self.btn_retarget.setEnabled(False);bar.addWidget(self.btn_retarget)
        bar.addStretch();self.lbl_pose=QLabel("");bar.addWidget(self.lbl_pose)
        l.addLayout(bar)
        self.fig=Figure(figsize=(16,5.5),dpi=100);self.canvas=FigureCanvas(self.fig);l.addWidget(self.canvas)
        self._tf=None;self.tf_pose=None;self.retargeted_traj=None
    def start_tf(self):
        if self._tf is None:self._tf=TfPersistent()
    def poll_tf(self):
        self.start_tf();pose=self._tf.lookup(timeout=0.3)
        if pose is not None:
            self.tf_pose=pose;self.lbl_status.setText("已连接");self.lbl_status.setStyleSheet("color:green;font-weight:bold")
            self.lbl_pose.setText(f"({pose.position.x:.3f},{pose.position.y:.3f},{pose.position.z:.3f})");self.btn_retarget.setEnabled(True)
        else:self.lbl_status.setText("等待TF...");self.lbl_status.setStyleSheet("color:orange")
    def retarget(self,form_raw):
        if self.tf_pose is None or form_raw is None: return
        from latte_imitation.trajectory_transform import retarget_trajectory
        from latte_imitation.trajectory import CartesianTrajectory
        from latte_imitation.latte_art.bridge import euler_deg_to_quat
        Tf=len(form_raw);quat=euler_deg_to_quat(45,0,0)
        cart=CartesianTrajectory(positions=form_raw.astype(np.float32),orientations=np.tile(quat.astype(np.float32),(Tf,1)),timestamps=np.arange(Tf,dtype=np.float32)*DT,dt=DT,episode_idx=-1,frame_id="base_link")
        self.retargeted_traj=retarget_trajectory(cart,self.tf_pose).positions
    def draw(self,orig_traj):
        if self.retargeted_traj is None or orig_traj is None: return
        self.fig.clear();gs=self.fig.add_gridspec(1,3,wspace=0.3);rt=self.retargeted_traj
        ax=self.fig.add_subplot(gs[0,0],projection='3d')
        ax.plot(orig_traj[:,0],orig_traj[:,1],orig_traj[:,2],'gray',lw=1.0,alpha=0.5);ax.plot(rt[:,0],rt[:,1],rt[:,2],'g-',lw=1.5)
        if self.tf_pose:ax.scatter([self.tf_pose.position.x],[self.tf_pose.position.y],[self.tf_pose.position.z],c='red',s=80,marker='*')
        ax.set_title('SE(3)重定目标')
        for pos,(d1,d2,l1,l2) in enumerate([(0,1,'X','Y'),(0,2,'X','Z')],1):
            ax_p=self.fig.add_subplot(gs[0,pos])
            ax_p.plot(orig_traj[:,d1],orig_traj[:,d2],'gray',lw=0.8,alpha=0.5);ax_p.plot(rt[:,d1],rt[:,d2],'g-',lw=1.2)
            if self.tf_pose:p=self.tf_pose.position;ax_p.scatter([p.x],[p.y if d2==1 else p.z],c='red',s=60,marker='*')
            ax_p.set_xlabel(l1);ax_p.set_ylabel(l2);ax_p.set_title(f'{l1}{l2}');ax_p.grid(alpha=0.3);ax_p.set_aspect('equal')
        self.canvas.draw_idle()
