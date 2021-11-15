import numpy as np
import robosuite as suite


def relative2absolute_joint_pos_commands(goal_joint_pos, robot, kp, kd):
    assert len(goal_joint_pos) == robot.dof

    action = [0 for _ in range(robot.dof)]
    curr_joint_pos = robot._joint_positions
    curr_joint_vel = robot._joint_velocities

    for i in range(robot.dof):
        action[i] = (goal_joint_pos[i] - curr_joint_pos[i]) * kp - curr_joint_vel[
            i
        ] * kd

    return action

result_qpos = np.array([np.pi / 2, 0, 0, 0, 0, 0])

def robosuite_simulation_controller_test(env, sim_time):
    # Reset the env
    env.reset()

    robot = env.robots[0]

    kp = 2
    kd = 1.2

    cnt = 0
    is_reached = False
    while True:
        # if env.done:
        #     break
        env.render()

        action = relative2absolute_joint_pos_commands(
            result_qpos, robot, kp, kd
        )
        
        pose_error = np.array([abs(result_qpos[i] - robot._joint_positions[i]) for i in range(robot.dof)])

        # print(pose_error)
        if np.all(pose_error< 1e-2):
            cnt += 1
            print("is_reached")
            is_reached = True
        
        if is_reached:
            if cnt%2 == 1:
                result_qpos[0] = 0
            elif cnt%2 == 0:
                result_qpos[0] = np.pi/2

        observation, reward, done, info = env.step(action)

    # close window
    env.close()


env = suite.make(
    "Lift",
    robots="UR5e",
    gripper_types=None,
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    use_object_obs=False,
    control_freq=50,
    render_camera=None,
    horizon=2000,
)

robosuite_simulation_controller_test(env, env.horizon)
