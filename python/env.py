import gymnasium as gym
import numpy as np
import mujoco

class PenduinEnv(gym.Env):

    def __init__(self, xml_path, mode="quad"):
        super().__init__()

        # model from mujoco
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # constants
        freq = 100
        self.frame_skip = round(1/(self.model.opt.timestep * freq))
        # self.max_steps = 1000
        self.dt = self.frame_skip * self.model.opt.timestep
        self.max_accel = 200
        self.max_speed = self.model.actuator_ctrlrange[0, 1]
        self.min_speed = 0.1
        self.th_limit = np.pi/2
        self.a_dot_denom = 20
        self.mode = mode
                
        # action space: u = [ theta_ddot ]
        self.action_space = gym.spaces.Box(
            low=-1, high=1, shape=(1,), dtype=np.float32
        ) # multiply with max_accel in step()
        
        # observation space: y = [sin(alpha), cos(alpha), theta, alpha_dot, theta_dot] 
        # (a->pendulum / theta->arm)
        obs_limit = np.array([1, 1, np.inf, np.inf, 1], dtype=np.float32)
        self.observation_space = gym.spaces.Box(
            low=-obs_limit, high=obs_limit, shape=(5,), dtype=np.float32
        )
        self.a_prev = 0
        self.speed = 0
        # self.cnt = 0

    def _read_alpha(self):
        a = self.data.qpos[1]
        a = a % (2*np.pi) # make it a in [0, 360)
        encoder = round(a*4096/(2*np.pi))
        a = encoder*2*np.pi/4096
        return a


    def _get_obs(self):
        # alpha (rad)
        a = self._read_alpha()
        sina = np.sin(a)
        cosa = np.cos(a)

        # alpha dot (normalized) - empirically, might changed denominator later
        a_dot = ((a - self.a_prev + np.pi)%(2*np.pi) - np.pi)/self.dt
        a_dot /= self.a_dot_denom
        self.a_prev = a

        # theta (normalized)
        th = self.data.qpos[0] / self.th_limit
        
        # theta_dot (normilized)
        th_dot = self.speed / self.max_speed

        return np.array([sina, cosa, th, a_dot, th_dot], dtype=np.float32)
 

    def _reward(self, obs, action): # [sina, cosa, th, a_dot, th_dot] (normalized)

        sina, cosa, th, a_dot, th_dot = obs
        a = np.arctan2(sina, cosa)
        a_dot *= self.a_dot_denom
        th *= self.th_limit
        th_dot *= self.max_speed
        
        # positive quadratic (LQR inspired)
        if self.mode == "quad":
            Q, R = 0.05, 0.01
            reward = (1+cosa)/2 - R*action**2 - Q*th**2

        
        # Gaussian/Exponential (PILCO: A Model-Based and Data-Efficient Approach to Policy Search) 
        if self.mode == "exp":
            Q, R = 0.05, 0.01
            cost = ( a**2 + R*action**2 + Q*th**2 )
            reward = np.exp(-cost)

        '''
        # l1 penalty 
        Q, R = 0.05, 0.01
        penalty = ( abs(a) + R*action**2 + Q*th**2 )
        reward = 1 - penalty
        # quanser qube
        Q, R = 0.05, 0.01
        penalty = ( a**2 + R*action**2 + Q*th**2 + R*(action-self.action_prev)**2)
        reward = 1 - penalty
        '''
        
        return float(reward)
        
        
        
    def step(self, action):
        # action
        act = np.clip(action[0], a_min=-1, a_max=1)
        accel = self.max_accel * act
        self.speed = self.speed + accel*self.dt
        self.speed = np.clip(self.speed, a_min=-self.max_speed, a_max=self.max_speed)
        # simulation
        self.data.ctrl[0] = self.speed if abs(self.speed) >= self.min_speed else 0.0
        mujoco.mj_step(self.model, self.data, nstep=self.frame_skip)
        # self.cnt += 1
        # afremath
        obs = self._get_obs() # [sina, cosa, th, a_dot, th_dot]
        reward = self._reward(obs, act)
        terminated = bool(np.abs(obs[2]) > 1) # theta > 90 deg
        if terminated: reward -= 100
        truncated = False # [bool(self.cnt >= self.max_steps)] in train.py

        return obs, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # mujoco reset
        mujoco.mj_resetData(self.model, self.data)
        # self.cnt = 0
        
        alpha0 = np.pi        
        if options is not None:
            alpha0 = options.get("alpha0", np.pi)
        # starting state
        self.data.qpos[0] = 0 # theta
        self.data.qpos[1] = alpha0 + self.np_random.uniform(low=-0.1, high=0.1) # alpha
        self.data.qvel[:] = 0.0
        self.a_prev = self._read_alpha()
        self.speed = 0
        mujoco.mj_forward(self.model, self.data)


        return self._get_obs(), {}
