import numpy as np

class BipedalGaitPlanner:
    num_feet = 2

    def __init__(
        self,
        dt,
        frequencies=1.5,
        phase_offset=0.5,
        stance_ratio=0.6,
    ):
        self.dt = dt  # simulation的一帧是多少s
        self.frequencies = float(frequencies)  # 1s经历几个gait周期
        self.phase_offset = float(phase_offset)  # 左脚比右脚领先多少个gait周期
        self.stance_ratio = float(stance_ratio)  # stance期占整个周期比例
        self.stance_middle_point = 0.3  # 初始周期位置

        # 状态变量（单个env）
        self.gait_index = self.stance_middle_point
        self.foot_indices = np.zeros(self.num_feet, dtype=np.float32)  # 左右脚 gait phase
        self.clock_inputs = np.zeros(self.num_feet, dtype=np.float32)  # 正弦值

    def update_gait_phase(self, stop: bool = False):
        # 更新 gait 相位
        self.gait_index = (self.gait_index + self.dt * self.frequencies) % 1.0

        # 左脚：加偏移，右脚：无偏移
        self.foot_indices[0] = (self.gait_index + self.phase_offset) % 1.0
        self.foot_indices[1] = self.gait_index

        if stop:
            self.gait_index = self.stance_middle_point
            self.foot_indices[:] = self.stance_middle_point

        for i in range(self.num_feet):
            idx = self.foot_indices[i]
            if idx < self.stance_ratio:
                self.foot_indices[i] = 0.5 * idx / self.stance_ratio
            else:
                self.foot_indices[i] = 0.5 + 0.5 * (idx - self.stance_ratio) / (1 - self.stance_ratio)

            self.clock_inputs[i] = np.sin(2 * np.pi * self.foot_indices[i])

