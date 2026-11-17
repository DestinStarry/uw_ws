"""MuJoCo plant adapter, independent of ROS and control algorithms."""
import math
import numpy as np
import mujoco


class Simulation:
    def __init__(self, model_path, linear_drag, angular_drag, *,
                 gravity=9.80665, water_density=1000.0, water_viscosity=0.001,
                 displaced_volume=0.0, buoyancy_offset=(0.0, 0.0, 0.02),
                 water_current=(0.0, 0.0, 0.0), actuator_model=None):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.actuator_model = actuator_model
        self.body = self.model.body('base').id
        self.origin_z = float(self.data.xpos[self.body, 2])
        # 全浸没静力模型：真实重力由 MuJoCo 施加，浮力在固定机体浮心处施加。
        # 默认按初始质量计算中性排水体积；浮心偏移是待实测标定的参数。
        for value in (gravity, water_density):
            if not np.isfinite(value) or value <= 0:
                raise ValueError('Gravity and water density must be finite and positive')
        if not np.isfinite(water_viscosity) or water_viscosity < 0:
            raise ValueError('Water viscosity must be finite and nonnegative')
        if not np.isfinite(displaced_volume) or displaced_volume < 0:
            raise ValueError('Displaced volume must be finite and nonnegative')
        self.water_current = np.asarray(water_current, dtype=float)
        offset = np.asarray(buoyancy_offset, dtype=float)
        for vector in (self.water_current, offset):
            if vector.shape != (3,) or not np.all(np.isfinite(vector)):
                raise ValueError('Water current and buoyancy offset need three finite values')
        rotation = self.data.xmat[self.body].reshape(3, 3)
        initial_com = rotation.T @ (self.data.subtree_com[self.body] - self.data.xpos[self.body])
        self.buoyancy_center = initial_com + offset
        mass = float(self.model.body_subtreemass[self.body])
        self.displaced_volume = float(displaced_volume) if displaced_volume > 0 else mass / water_density
        self.buoyancy_force = np.array([0.0, 0.0, water_density * self.displaced_volume * gravity])
        self.model.opt.gravity[:] = (0.0, 0.0, -gravity)
        self.model.opt.density = water_density
        self.model.opt.viscosity = water_viscosity
        self.model.opt.wind[:] = self.water_current
        # 模型质量配置改变后，水阻代理仍应位于初始总质心，减少虚假的偏置水动力矩。
        proxy = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'hull_fluid_proxy')
        if proxy >= 0:
            self.model.geom_pos[proxy] = initial_com
        mujoco.mj_forward(self.model, self.data)

        # 控制内部顺序 [1上,3下,2右,4左]；按名称绑定，避免依赖 XML 下标。
        self.thrusters = [self.model.actuator(name).id for name in
                          ('thruster1', 'thruster3', 'thruster2', 'thruster4')]
        self.servos = [self.model.actuator(name).id for name in ('servo13', 'servo24')]
        self.gears = self.model.actuator_gear[self.thrusters, 0].copy()
        if np.any(np.abs(self.gears) < 1e-9):
            raise ValueError('Thruster gear cannot be zero')
        self.linear_drag = np.asarray(linear_drag, dtype=float)
        self.angular_drag = np.asarray(angular_drag, dtype=float)
        for drag in (self.linear_drag, self.angular_drag):
            if drag.shape != (3,) or not np.all(np.isfinite(drag)) or np.any(drag < 0):
                raise ValueError('Water drag requires three nonnegative finite values')

    def command(self, forces, angles, enabled=True):
        f, a = np.asarray(forces, dtype=float), np.asarray(angles, dtype=float)
        if (f.shape != (4,) or a.shape != (2,) or not np.all(np.isfinite(f))
                or not np.all(np.isfinite(a)) or np.any(f < -24.5)
                or np.any(f > 33) or np.any(np.abs(a) > math.pi / 2)):
            self.stop()
            return False
        if not enabled:
            self.stop()
            return True
        if self.actuator_model is not None:
            self.actuator_model.command(self.data.time, f, a)
            return True
        # actuator gear 将控制量变为 N；舵机 position 执行器直接接收 rad。
        self.data.ctrl[self.thrusters] = f / self.gears
        self.data.ctrl[self.servos] = a
        return True

    def stop(self):
        if self.actuator_model is not None:
            self.actuator_model.stop()
        self.data.ctrl[:] = 0.0

    def _world_velocity(self):
        """MuJoCo 世界系速度排列为 [角速度 xyz, 线速度 xyz]。"""
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                 self.body, velocity, 0)
        return velocity

    def water_wrench(self):
        """世界系外力/力矩，力矩参考点为 base 的惯性质心（xfrc_applied 约定）。"""
        velocity = self._world_velocity()
        rotation = self.data.xmat[self.body].reshape(3, 3)
        # 附加线性阻尼使用相对水流速度；XML 椭球模型另提供速度相关流体阻力。
        drag = rotation @ (-self.linear_drag * (rotation.T @ (velocity[3:] - self.water_current)))
        angular_drag = rotation @ (-self.angular_drag * (rotation.T @ velocity[:3]))
        body_com = self.data.xipos[self.body]
        center_world = self.data.xpos[self.body] + rotation @ self.buoyancy_center
        buoyancy_torque = np.cross(center_world - body_com, self.buoyancy_force)
        drag_torque = np.cross(self.data.subtree_com[self.body] - body_com, drag)
        return np.r_[drag + self.buoyancy_force, angular_drag + drag_torque + buoyancy_torque]

    def step(self, count=1):
        for _ in range(count):
            if self.actuator_model is not None:
                forces, angles = self.actuator_model.step(self.data.time, self.model.opt.timestep)
                self.data.ctrl[self.thrusters] = forces / self.gears
                self.data.ctrl[self.servos] = angles
            # stop() 只关闭执行器，水的浮力、阻力和重力一直存在。
            self.data.xfrc_applied[self.body] = self.water_wrench()
            mujoco.mj_step(self.model, self.data)

    def state(self):
        """返回副本；深度以初始高度为零、向下为正，不是绝对水面深度。"""
        velocity = self._world_velocity()
        return dict(quaternion=self.data.sensor('body_quat').data.copy(),
                    gyro=self.data.sensor('body_gyro').data.copy(),
                    acceleration=self.data.sensor('body_linacc').data.copy(),
                    position=self.data.xpos[self.body].copy(),
                    depth=self.origin_z-float(self.data.xpos[self.body, 2]),
                    depth_rate=-float(velocity[5]))

